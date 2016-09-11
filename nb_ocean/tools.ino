// Prints a Flash string to the Serial Monitor
inline void print(const __FlashStringHelper* text) {
  printText(text);
  finishSending(); // wait until finished
}
inline void print(const __FlashStringHelper* text, const long value) {
  printText(text);
  printValue(value);
  finishSending(); // wait until finished
}
inline void println(const __FlashStringHelper* text) {
  printText(text);
  endLn();
}
inline void println(const __FlashStringHelper* text, const long value) {
  printText(text);
  printValue(value);
  endLn();
}
inline void println(const __FlashStringHelper* text, const byte* array, const byte arraySize) {
  printText(text);
  printArray(' ', array, arraySize);
  endLn();
}
inline void printText(const __FlashStringHelper* text) {
  if (debug)
    Serial.print(text);
}
inline void printChar(const char c) {
  if (debug)
    Serial.write(c);
}
inline void printValue(const long value) {
  if (debug)
    Serial.print(value);
}
inline void printArray(const char separator, const byte* array, const byte arraySize) {
  for (byte i = 0; i < arraySize; i++) {
    printChar(separator);
    printValue(array[i]);
  }
}
inline void endLn() {
  if (debug) {
    Serial.write(0x0A); // send new line byte
    finishSending(); // wait until finished
    commCount = 0; // reset the comm counter
  }
}

// Convenience functions for printing common debug information
void printMemoryUsed(const __FlashStringHelper* function, const int memory) {
  if (!debug)
    return;
  printText(F("Memory used by "));
  printText(function);
  PRINTLN(": ", memory);
}
inline void printOutputStatus(const __FlashStringHelper* text, const byte chain, const byte output) {
  if (debug && verbose > true) {
    printText(text);
    printTab();
    printValue(time);
    printTab();
    printC(chain);
    printD(output);
    PRINTLN("\tKey Time: ", keyTime[chain][output]);
  }
}
inline void printC(const byte chain) {
  if (debug) {
    printChar('C');
    printValue(chain + 1);
  }
}
inline void printD(const byte output) {
  if (debug) {
    printChar('D');
    printValue(output + 1);
  }
}
inline void printTab() {
  if (debug)
    printChar('\t');
}
inline void printStar() {
  if (debug)
    printChar('*');
}

// This should never happen if code and DB are fine.
// Will flash the error number on the board.
void majorError(const Error error, const byte type, const __FlashStringHelper* message) {
  // Flash the LEDs until a status request is received
  byte flash = 150, num;
  byte data = makePacket(myAddress, STATUS);
  while(true) {
    while(Serial.read() != data) {
      if (debug) {
        printText(F("Major error in "));
        println(message);
      }
      flashValue((error*10) + type);
      delay(flash * 3);
    }
  
    // Status request has been received, send back error number/type
    send(myAddress, STATUS);
    send(myAddress, B111);
    send(myAddress, error);
    send(myAddress, type);
  }
}

// Convenience functions for accessing flash memory
inline const void* copyFromFlash(PGM_VOID_P src, const size_t n) {
  return copyFromFlash(malloc(n), src, n);
}
const void* copyFromFlash(void* dst, PGM_VOID_P src, const size_t n) {
  if (!memcmp_P(memcpy_P(dst, src, n), src, n))
    return dst;
  free(dst);
  return NULL;
}

// This function reads the node type from flash memory
// It should use 16 + maxChainsPerNode bytes
void readNodeType(const byte type) {
  // Allocate memory for the node type. Uses 14 bytes
  NodeType *nodeType = (NodeType*) copyFromFlash(&allNodeTypes[type], sizeof(NodeType));
    
  // Check that the data was copied properly
  if (nodeType == NULL || nodeType->ID != (type + 1))
    MAJORERROR(programError, 3, "readNodeType");
  
  // Allocate memory for the chain type IDs. Uses 2 + maxChainsPerNode bytes
  nodeType->chainTypeIDs = (byte*) copyFromFlash(nodeType->chainTypeIDs, maxChainsPerNode*sizeof(byte));
  
  // Keep a reference to the chain type IDs
  myChainTypeIDs = nodeType->chainTypeIDs;
  
  // Keep a globally accessible pointer to the node type
  myNodeType = nodeType;
}

// This function copies details of the used chain types from flash memory into RAM
void readChainTypes() {
  if (debug && verbose)
    PRINTLN("My Chains:");
  
  // Allocate a temporary array for holding output type IDs. Uses 4 + numChainTypes*2 bytes
  byte **usedChainOutputTypeIDs = (byte**) calloc(numChainTypes + 1, sizeof(byte*));
  
  // Iterate through the chains, copying information to the RAM as needed
  ChainType* chainType;
  byte* chainOutputTypeIDs;
  for (byte chain = 0; chain < maxChainsPerNode; chain++) {
    // Check that this is a valid chain
    byte chainTypeID = myChainTypeIDs[chain];
    if (invalidID(chain, chainTypeID, F("Chain"), true))
      continue;
    
    // If chain type is new, allocate memory for output IDs. Uses 2 + maxOutputsPerChain each time
    if (myUsedChainTypes[chainTypeID] == NULL)
      usedChainOutputTypeIDs[chainTypeID] = (byte*) calloc(maxOutputsPerChain, sizeof(byte));
    
    // Valid chain type, read it from the RAM and copy from flash if needed
    chainType = readChainType(chainTypeID);
  
    // Keep globally accessible pointers to some specific details for each chain
    myInputTypeIDs[chain] = chainType->inputTypeID;
    myRegisterTypeIDs[chain] = chainType->registerTypeIDs;
    myOutputTypeIDs[chain] = usedChainOutputTypeIDs[chainTypeID];
    
    // Read this chain's input type
    readInputType(chain);
    
    // Iterate through this chain's registers, copying information to the RAM as needed
    chainOutputTypeIDs = myOutputTypeIDs[chain];
    for (byte reg = 0; reg < maxRegistersPerChain; reg++) {
      // Check that this is a valid register
      byte registerTypeID = myRegisterTypeIDs[chain][reg];
      if (invalidID(reg, registerTypeID, F("Register"), false))
        continue;
      
      // Valid register, check if the information has been copied to the RAM already
      readRegisterType(registerTypeID, &chainOutputTypeIDs[reg*maxOutputsPerRegister]);
    }
    
    // Iterate through this chain's outputs, copying information to the RAM as needed
    for (byte output = 0; output < maxOutputsPerChain; output++) {
      // Check that this is a valid output
      byte outputTypeID = myOutputTypeIDs[chain][output];
      if (invalidID(output, outputTypeID, F("Output"), false))
        continue;
      
      // Valid output type
      readOutputType(outputTypeID);
    }
  }
  
  // Deallocate the temporary array for holding output type IDs. Frees 4 + numChainTypes*2 bytes
  free(usedChainOutputTypeIDs);
}

// Reads a chain type from the RAM and prints verbose details about it
// If the chain type is not in the RAM, it copies it from the Flash memory first
// Uses (14 + maxRegistersPerChain) bytes of RAM for each unique chain type
ChainType* readChainType(const byte chainTypeID) {
  // Check if this chain type has been copied to the RAM already
  ChainType* chainType = myUsedChainTypes[chainTypeID];
  if (chainType == NULL) {
    // The information is not in the RAM
    printStar();
    
    // Allocate memory for this specific chain type. Uses 12 bytes
    chainType = (ChainType*)
      copyFromFlash(&allChainTypes[chainTypeID - 1], sizeof(ChainType));
    
    // Check that the data was copied properly
    if (chainType == NULL || chainType->ID != chainTypeID)
      MAJORERROR(programError, 4, "readChainTypes");
    
    // Allocate memory for the register type IDs. Uses 2 + maxRegistersPerChain bytes
    chainType->registerTypeIDs = (byte*)
      copyFromFlash(chainType->registerTypeIDs, maxRegistersPerChain*sizeof(byte));
    
    // Store a reference to the RAM copy of the chain type information
    myUsedChainTypes[chainTypeID] = chainType;
  }
  
  // Provide verbose debug information for this chain type
  if (debug && verbose) {
    PRINT("\n\tOverlays: ");
    chainType->doOverlay ? PRINT(" on"): PRINT("off");
    PRINT(", Input Type: ", chainType->inputTypeID);
    PRINTLN(", Register Types:", chainType->registerTypeIDs, maxRegistersPerChain);
  }
  
  // Return a reference to the chain type
  return chainType;
}

// Reads an input type from the RAM and prints verbose details about it
// If the input type is not in the RAM, it copies it from the Flash memory first
// Uses 4 bytes of RAM for each unique input type
void readInputType(const byte chain) {
  // Check that this is a valid input type
  byte inputTypeID = myInputTypeIDs[chain];
  if (invalidID(chain, inputTypeID, F("Input"), false))
    return;
  
  // Check if the input type details have been copied to the RAM already
  InputType* inputType = myUsedInputTypes[inputTypeID];
  if (inputType == NULL) {
    // The information is not in the RAM
    printStar();
    
    // Allocate memory for this specific input type. Uses 4 bytes
    inputType = (InputType*) copyFromFlash(&allInputTypes[inputTypeID - 1], sizeof(InputType));
    
    // Check that the data was copied properly
    if (inputType == NULL || inputType->ID != inputTypeID)
      MAJORERROR(programError, 5, "readInputTypes");
    
    myUsedInputTypes[inputTypeID] = inputType;
  }
  
  // Provide verbose debug information on what information has been saved to RAM
  if (debug && verbose) {
    PRINT(" \tActive ");
    inputType->activeHigh? PRINTLN("High"): PRINTLN("Low");
  }
}

// Reads a register type from the RAM and prints verbose details about it
// If the register type is not in the RAM, it copies it from the Flash memory first
// Uses 5 bytes of RAM for each unique register type
RegisterType* readRegisterType(const byte registerTypeID, byte* outputTypeIDs) {
  // Valid register, check if the information has been copied to the RAM already
  RegisterType* registerType = myUsedRegisterTypes[registerTypeID];
  if (registerType == NULL) {
    // The information is not in the RAM
    printStar();
    
    // Allocate memory for this register type. Uses 5 bytes
    registerType = (RegisterType*)
      copyFromFlash(&allRegisterTypes[registerTypeID - 1], sizeof(RegisterType));
    
    // Check that the data was copied properly
    if (registerType == NULL || registerType->ID != registerTypeID)
      MAJORERROR(programError, 6, "readRegisterTypes");
    
    // Copy the output type IDs to the RAM. Uses 0 bytes, copying into existing array
    registerType->outputTypeIDs = (byte*) copyFromFlash(outputTypeIDs,
                                                        registerType->outputTypeIDs,
                                                        maxOutputsPerRegister*sizeof(byte));
    
    // Store a reference to the RAM copy of the register type information
    myUsedRegisterTypes[registerTypeID] = registerType;
  } else {
    // Copy the output type IDs to the RAM. Uses 0 bytes, copying into existing array
    memcpy(outputTypeIDs, registerType->outputTypeIDs, maxOutputsPerRegister*sizeof(byte));
  }
  
  // Provide verbose debug information on what information has been saved to RAM
  if (debug && verbose)
    PRINTLN("\tOutput Types:", registerType->outputTypeIDs, maxOutputsPerRegister);
  
  return registerType;
}

// Reads an output type from the RAM and prints verbose details about it
// If the output type is not in the RAM, it copies it from the Flash memory first
// Uses 13 bytes of RAM for each unique output type
OutputType* readOutputType(const byte outputTypeID) {
  // Check if the information has been copied to the RAM already
  OutputType* outputType = myUsedOutputTypes[outputTypeID];
  if (outputType == NULL) {
    // The information is not in the RAM
    printStar();
    
    // Allocate memory for this output type. Uses 13 bytes
    outputType = (OutputType*)
      copyFromFlash(&allOutputTypes[outputTypeID - 1], sizeof(OutputType));
    
    // Check that the data was copied properly
    if (outputType == NULL || outputType->ID != outputTypeID)
      MAJORERROR(programError, 7, "readOutputTypes");
  
    // Calculate the actual maxPWM for the outputType using the maxPWMBrightness and holdLevel
    outputType->maxPWM = (maxPWMBrightness*outputType->holdLevel)/0xFF;
    
    // Store a reference to the RAM copy of the output type information
    myUsedOutputTypes[outputTypeID] = outputType;
  }
  
  // Provide verbose debug information on what information has been saved to RAM
  if (debug && verbose) {
    printText(F("\tRise: "));
    printValue(outputType->riseFor);
    printText(F("ms, Hold: "));
    printValue(outputType->holdFor);
    printText(F("ms, Fall: "));
    printValue(outputType->fallFor);
    printText(F("\tPeak: "));
    printValue(100*outputType->maxPWM/maxPWMBrightness);
    printText(F("%\tSensor Start/Offset: "));
    printValue(outputType->sensorStartTime);
    printChar('/');
    printValue(outputType->sensorOffsetTime);
    PRINTLN("ms");
  }
  
  // Return a reference to the output type
  return outputType;
}

// Checks whether a given ID is valid or not, and prints details
boolean invalidID(const byte number, const byte ID, const __FlashStringHelper* name, const boolean chain) {
  // Print the index number and ID
  if (debug && verbose) {
    chain? PRINT("    "): PRINT("\t");
    print(name);
    PRINT(" ", number + 1);
    PRINT(", Type ", ID);
  }
  
  // Check if this is a valid ID
  if (ID == 0) {
    // This is an invalid ID
    if (debug && verbose) {
      if (chain)
        ENDLN();
      PRINT("\tNo ");
      println(name);
    }
    return true;
  }
  
  // This is a valid ID
  return false;
}

// Copies this node's sensor threshold from the flash memory to RAM
inline void readSensorThreshold() {
  mySensorThreshold = (int)pgm_read_word(&sensorThreshold[myAddress]);
}

// Convenience functions to send data on the bus
inline void send(const byte address, const byte msg) {
  send(makePacket(address, msg));
}
void send(const byte data) {
  digitalWrite(DE, HIGH); // enable chip
  Serial.write(data); // send msg
  finishSending(); // wait until msg is sent
  digitalWrite(DE, LOW); // disable chip
  if (++commCount >= commLineLength) // check counter
    endLn(); // whether debugging or not, clean output
}
//void send(const byte a, const byte b, const byte c) {
//  digitalWrite(DE, HIGH); // enable chip
//  Serial.write(a); // send first msg
//  Serial.write(b); // send second msg
//  Serial.write(c); // send third msg
//  finishSending(); // wait until data is sent
//  digitalWrite(DE, LOW); // disable chip
//  if ((commCount += 3) >= commLineLength) // check counter
//    endLn(); // whether debugging or not, clean output
//}
inline void finishSending() {
  bitSet(UCSR0A, TXC0); // clear transmission flag
  loop_until_bit_is_set(UCSR0A, TXC0); // wait until sent
}
inline byte makePacket(const byte address, const byte msg) {
  return (msg << 5) + address;
}
inline byte getAddress(const byte data) {
  return data & B11111;
}
inline byte getMessage(const byte data) {
  return data >> 5;
}

// Convenience functions for digital inputs and outputs
inline void turnOn(const byte chain, const byte output) {
  setPWM(chain, output, maxPWMBrightness);
}
inline void turnOff(const byte chain, const byte output) {
  setPWM(chain, output, 0);
}
inline void setPWM(const byte chain, const byte output, const byte pwm) {
  ShiftMuxPWM.SetOne(chain, output, pwm);
  if (debug && verbose > true) {
    printText(F("PWM ("));
    printValue(pwm);
    printOutputStatus(F("):"), chain, output);
  }
}
byte shiftIn() {
  byte data = 0;
  for (char i = 7; i >= 0; i--) {
    data += digitalRead(addressPin) << i; // read a bit
    digitalWrite(CLK, HIGH); // set CLK to 5V
    digitalWrite(CLK, LOW); // set CLK to GND
  }
  return data;
}

// Convenience functions for inputs
unsigned int readInput(const byte chain) {
  const byte aPin = inPin(chain);
  unsigned int value = 0;
  const byte bufferSize = 64;
  for (byte i = 0; i < bufferSize; i++)
    value += analogRead(aPin);
  value /= bufferSize;
  return value;
}

// Convenience functions for LEDs
inline void LEDOn(const byte LED) {
  digitalWrite(LEDPin(LED), HIGH);
}
inline void allLEDsOn() {
  for (byte led = 0; led < numLEDs; led++)
    LEDOn(led);
}
inline void LEDOff(const byte LED) {
  digitalWrite(LEDPin(LED), LOW);
}
inline void allLEDsOff() {
  for (byte led = 0; led < numLEDs; led++)
    LEDOff(led);
}
void flashValue(const byte value) {
  byte ms = 200;
  flashLED(0, value/10, ms, true); // Double flash 10s
  delay(ms);
  flashLED(0, value%10, ms, false); // Single flash 1s
  delay(ms * 5); // Wait to separate this value from the next
}
void flashLED(const byte LED, const byte times, const byte wait, const boolean twice) {
  byte pin = LEDPin(LED);
  for (byte num = 0; num < times; num++) {
    if (twice) {
      pulse(pin, wait/2);
      pulse(pin, wait/2);
      delay(wait);
    } else {
      pulse(pin, wait);
    }
  }
}

// Pulses a pin high then low with delays
inline void pulse(const byte pin, const byte wait) {
  digitalWrite(pin, HIGH);
  delay(wait);
  digitalWrite(pin, LOW);
  delay(wait);
}

// Sets pin to output and grounds it
inline void ground(const byte pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

/**
 * Used to address arduino pins using DB LED index numbers.
 *
 * Use this everywhere so that there is only one place that
 * translates between DB LEDs and arduino pins.
 *
 * The input should always be in the set [0:numLEDs - 1].
 */
inline byte LEDPin(const byte LED) {
  if (LED >= numLEDs)
    MAJORERROR(programError, 1, "LEDPin");
  return arduinoLEDPins[LED];
}

/**
 * Used to address arduino pins using chain index numbers.
 *
 * Use this everywhere so that there is only one place that
 * translates between chains and arduino analog pins.
 *
 * The chain should always be in the set [0:maxChainsPerNode - 1].
 */
inline byte inPin(const byte chain) {
  if (chain >= maxChainsPerNode)
    MAJORERROR(programError, 2, "inPin");
  return A0 + chain;
}

void printMyAddress(){
  Serial.println("My Address: ");
  Serial.println(myAddress);
}
