#include <MemoryFree.h> // This library uses 110 bytes of Flash

inline void initCommon() {
  randomSeed(analogRead(0)); // Get a random number
  initComm(); // sets up the communication
  printAvailableMemory(); // prints the initial amount of free memory
  if (!isMaster) {
    initLEDPins(); // sets up the LED pins
    initShiftPins(); // sets up the shift register pins
    emptyRegisters(); // empties the registers
    readAddress(); // reads the address
    checkAddress(); // checks the address and prints/flashes it
    findNodeType(); // finds the node type based on myAddress
    initShiftPWM(); // sets up the shift PWM library
  } else {
    initMasterPins(); // sets up the master pins
  }
  readFromFlash(); // reads data from Flash based on myNodeType
  initInputPins(); // sets up the inputs based on myInputTypes
}

inline void finishInit() {
  printAvailableMemory(); // prints the amount of free memory after initialization
  if (!isMaster && codeVersion&1)
    LEDOn(0); // flags the lowest bit of the version
}

inline void printAvailableMemory() {
  if (debug)
    PRINTLN("Memory available: ", freeMemory());
}

// Sets up the communication
inline void initComm() {
  isMaster = digitalRead(DE); // Checks if node is the Master
  ground(DE); // keeps debug statements off the bus: HIGH to enable
  if (isMaster)
    ground(RE); // listens to the bus, ignores messages sent by a connected PC: LOW to enable
  else if (usingTestBed && !commEnabled)
    digitalWrite(DE, HIGH); // make the slaves send debug info over comm
  Serial.begin(115200); // starts serial comm
  ENDLN(); // finishes initializing comm and moves to a new line
  if (debug) {
    PRINTLN("Initializing...");
    if (isMaster)
      PRINTLN("I am the Master");
  }
}

// Initializes the shift register pins
inline void initShiftPins() {
  ShiftMuxPWM_initialize(); // initializes LATCH, CLK, SOUT, CLK_LO, CLK_MD, CLK_HI as outputs
  ground(SIN_LO); // sets SIN_LO to an output and grounds it
  ground(SIN_HI); // sets SIN_HI to an output and grounds it
}

// Initializes the master pins
inline void initMasterPins() {
  ground(CLK);    // sets CLK to an output and grounds it
  ground(SIN_LO); // sets SIN_LO to an output and grounds it
  ground(SIN_HI); // sets SIN_HI to an output and grounds it
}

inline void emptyRegisters() {
  byte reg;
  for (byte chain = 0; chain < maxChainsPerNode; chain++) {
    digitalWrite(CLK_LO, bitRead(chain, 0));
    digitalWrite(CLK_MD, bitRead(chain, 1));
    digitalWrite(CLK_HI, bitRead(chain, 2));
    for (reg = 0; reg < maxRegistersPerChain; reg++)
      shiftOut(SOUT, CLK, MSBFIRST, 0);
  }
  digitalWrite(LATCH, HIGH);
  digitalWrite(LATCH, LOW);
}

// Initializes the LED pins as outputs and turns them off
inline void initLEDPins() {
  for (byte i = 0; i < numLEDs; i++)
    ground(LEDPin(i));
}

inline void readAddress() {
  digitalWrite(SIN_HI, HIGH); // disable serial output
  digitalWrite(SIN_LO, LOW); // read parallel inputs
  digitalWrite(SIN_LO, HIGH); // stop parallel inputs
  digitalWrite(SIN_HI, LOW); // enable serial output
  myAddress = shiftIn(); // read the address
  digitalWrite(SIN_HI, HIGH); // disable serial
}

void checkAddress() {
  if (debug) {
    PRINTLN("My Addr: ", myAddress);
    delay(500); // separates address from boot LED
//    while(1) {flashValue(myAddress);delay(500);}
    if (verbose)
      flashValue(myAddress);
  }
  if (myAddress > maxNodeAddress[numNodeTypes - 1])
    MAJORERROR(switchError, 1, "readAddress: Address too high.");
  if (myAddress == masterAddress)
    MAJORERROR(switchError, 2, "readAddress: Address set to Master Address.");
}

// Finds which type this node is
// Will have an addressing error if type doesn't exist
inline void findNodeType() {
  int memory = freeMemory();
  myNodeType = NULL;
  for (byte type = 0; type < numNodeTypes; type++) {
    if (myAddress <= maxNodeAddress[type]) {
      readNodeType(type);
      break;
    }
  }
  
  if (myNodeType == NULL) {
    MAJORERROR(switchError, 3, "findNodeType");
  } else if (debug) {
    PRINTLN("My Node Type: ", myNodeType->ID);
    PRINT("\tNode Reactor: ");
    myNodeType->doNodeReactor?
      PRINTLN(" on"): PRINTLN("off");
    PRINTLN("\tDefault Timeout: ", myNodeType->defaultTimeout);
    PRINTLN("\tDefault Time Delta: ", myNodeType->defaultTimeDelta);
    PRINT("\tChain Types:");
    for (byte chain = 0; chain < maxChainsPerNode; chain++)
      PRINT(" ", myNodeType->chainTypeIDs[chain]);
    ENDLN();
    flashValue(myNodeType->ID);
    if (verbose)
      printMemoryUsed(F("findNodeType"), memory - freeMemory());
  }
}

inline void initShiftPWM() {
  int memory = freeMemory();
  
  // Initialize the SPI hardware
  SPI.setBitOrder(LSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.begin();
  
  // Initialize the ShiftMuxPWM library
  ShiftMuxPWM.SetAmountOfChains(maxChainsPerNode, maxRegistersPerChain);
  ShiftMuxPWM.Start(pwmFrequency, maxPWMBrightness);
  ShiftMuxPWM.SetAll(0);
  if (debug && verbose) {
    ShiftMuxPWM.PrintInterruptLoad();
    printMemoryUsed(F("initShiftPWM"), memory - freeMemory());
  }
  
  while(runPWMTests) {
    if (debug)
      PRINTLN("Testing PWM...");
    testShiftPWM_chainers();
    delay(100);
  }
}

inline void testShiftPWM_chainers(){
  int chain;
  int brightness;
  int chainOut;
  for (chain = 1; chain <= 6; chain++){
    for (brightness = 0;brightness < maxPWMBrightness; brightness++){
      for (chainOut = 0; chainOut < maxOutputsPerRegister; chainOut ++){
        
        ShiftMuxPWM.SetOne(chain, chainOut, brightness);
        ShiftMuxPWM.SetOne(chain-1, chainOut, maxPWMBrightness - brightness);
      }
      delay(20);
    }
  }
      
}
  

inline void testShiftPWM(){
  int chainlen;
  int chainPat;
  boolean lightOn;
  int stage;
  int chain;
  int chainOut;
  for (stage = 0;stage < patternLength; stage ++){
    ShiftMuxPWM.SetAll(0);
    
    for (chain = 0;chain < 6; chain ++){
      chainlen = chainLengths[chain];
      chainPat = pattern[stage][chain];
      
      for (chainOut = 0; chainOut < chainlen * maxOutputsPerRegister; chainOut ++){
        lightOn = (chainPat >> chainOut) & 1; //bit shift over and read first byte, either 0 or 1, starts with rightmost bit, shifts them to the right
        if (lightOn) {
          ShiftMuxPWM.SetOne(chain, chainOut, maxPWMBrightness);
          Serial.print(" stage: ");
          Serial.print(stage);
          Serial.print(" chain: ");
          Serial.print(chain);
          Serial.print(" out: ");
          Serial.println(chainOut);
        }
      }
    }
    delay(100);
  }
  ShiftMuxPWM.SetAll(0);
  delay(100);
} 

inline void testShiftPWM_fadeover(){
  int chainlen;
  int chainPat;
  boolean lightOn;
  int stage;
  int chain;
  int chainOut;
  for (stage = 0;stage < patternLength; stage ++){
    ShiftMuxPWM.SetAll(0);
    
    for (chain = 0;chain < 6; chain ++){
      chainlen = chainLengths[chain];
      chainPat = pattern[stage][chain];
      
      for (chainOut = 0; chainOut < chainlen * maxOutputsPerRegister; chainOut ++){
        lightOn = (chainPat >> chainOut) & 1; //bit shift over and read first byte, either 0 or 1, starts with rightmost bit, shifts them to the right
        if (lightOn) {
          ShiftMuxPWM.SetOne(chain, chainOut, maxPWMBrightness);
          //Serial.print(" stage: ");
          //Serial.print(stage);
          //Serial.print(" chain: ");
          //Serial.print(chain);
          //Serial.print(" out: ");
          //Serial.println(chainOut);
        }
      }
    }
    delay(100);
  }
  ShiftMuxPWM.SetAll(0);
  delay(100);
} 

inline void testShiftPWM_OLD2(){
    // Fade in all outputs
    int brightness;
  for(brightness = 0; brightness < maxPWMBrightness; brightness++) {
    ShiftMuxPWM.SetAll(brightness);
    delay(20);
  }
  
  //fade out
  for(brightness = 0; brightness < maxPWMBrightness; brightness++) {
    ShiftMuxPWM.SetAll(maxPWMBrightness - brightness);
    delay(20);
  }
  //fade in
    for(brightness = 0; brightness < maxPWMBrightness; brightness++) {
    ShiftMuxPWM.SetAll(brightness);
    delay(20);
  }
  
  //fade out
  for(brightness = 0; brightness < maxPWMBrightness; brightness++) {
    ShiftMuxPWM.SetAll(maxPWMBrightness - brightness);
    delay(20);
  }
//  
//  
//  // test on the SQUARE, uses the pattern defined in definitions.h
//
  int chainlen;
  int chainPat;
  boolean lightOn;
  int stage;
  int chain;
  int chainOut;
  for (stage = 0;stage < patternLength; stage ++){
    ShiftMuxPWM.SetAll(0);
    
    for (chain = 0;chain < 6; chain ++){
      chainlen = chainLengths[chain];
      chainPat = pattern[stage][chain];
      
      for (chainOut = 0; chainOut < chainlen * maxOutputsPerRegister; chainOut ++){
        lightOn = (chainPat >> chainOut) & 1; //bit shift over and read first byte, either 0 or 1, starts with rightmost bit, shifts them to the right
        if (lightOn) {
          ShiftMuxPWM.SetOne(chain, chainOut, maxPWMBrightness);
          Serial.print(" stage: ");
          Serial.print(stage);
          Serial.print(" chain: ");
          Serial.print(chain);
          Serial.print(" out: ");
          Serial.println(chainOut);
        }
      }
    }
    delay(100);
  }
  ShiftMuxPWM.SetAll(0);
  delay(100);
}

inline void testShiftPWM_OLD() {
  byte chain, reg, led;
  int output, brightness;
  
  // Fade in and fade out all outputs one by one fast. Useful for testing your circuit
//  ShiftMuxPWM.OneByOneFast();

  // Fade in all outputs
  for(brightness = 0; brightness < maxPWMBrightness; brightness++) {
    ShiftMuxPWM.SetAll(brightness);
    delay(20);
  }
  
//  // Hold all outputs
//  delay(20*maxPWMBrightness);
//  // Fade out all outputs
//  for(brightness = maxPWMBrightness; brightness >= 0; brightness--) {
//    ShiftMuxPWM.SetAll(brightness);
//    delay(20);
//  }


//  // Fade in and out all chains together, from the top, one output at a time
//  for (output = 0; output < maxOutputsPerChain; output++) {
//    for (brightness = 0; brightness < maxPWMBrightness; brightness++) {
//      for (chain = 0; chain < maxChainsPerNode; chain++) {
//        ShiftMuxPWM.SetOne(chain, output, brightness);
//      }
//      delay(10);
//    }
//    delay(10*maxPWMBrightness);
//    for (brightness = maxPWMBrightness; brightness >= 0; brightness--) {
//      for (chain = 0; chain < maxChainsPerNode; chain++) {
//        ShiftMuxPWM.SetOne(chain, output, brightness);
//      }
//      delay(10);
//    }
//  }

  // Fade in and out 2 outputs at a time
//  for (chain = 0; chain < maxChainsPerNode; chain++) {
//    for(output = 0; output < maxOutputsPerChain + 1; output++) {
//      for(brightness = 0; brightness <= maxPWMBrightness; brightness++) {
//        if (output < maxOutputsPerChain)
//          ShiftMuxPWM.SetOne(chain, output, brightness);
//        if (output > 0)
//          ShiftMuxPWM.SetOne(chain, output - 1, maxPWMBrightness - brightness);
//        delay(10);
//      }
//    }
//  }

//  // Fade in, hold, and fade out 3 outputs at a time
//  for (chain = 0; chain < maxChainsPerNode; chain++) {
//    for(output = 0; output < maxOutputsPerChain + 2; output++) {
//      for(brightness = 0; brightness <= maxPWMBrightness; brightness++) {
//        if (output < maxOutputsPerChain)
//          ShiftMuxPWM.SetOne(chain, output, brightness);
//        if (output > 1)
//          ShiftMuxPWM.SetOne(chain, output - 2, maxPWMBrightness - brightness);
//        delay(10);
//      }
//    }
//  }

  // Fade chain and register index in and out on each register
//  for (brightness = 0; brightness < maxPWMBrightness; brightness++) {
//    for (chain = 0; chain < maxChainsPerNode; chain++) {
//      for (reg = 0; reg < maxRegistersPerChain; reg++) {
//        for (led = 0; led <= chain; led++) {
//          ShiftMuxPWM.SetOne(chain, reg*maxOutputsPerRegister + led, brightness);
//        }
//        for (int led = 7; led >= 7 - reg; led--) {
//          ShiftMuxPWM.SetOne(chain, reg*maxOutputsPerRegister + led, maxPWMBrightness - brightness);
//        }
//      }
//    }
//    delay(20);
//  }
//  for (brightness = maxPWMBrightness; brightness >= 0; brightness--) {
//    for (chain = 0; chain < maxChainsPerNode; chain++) {
//      for (reg = 0; reg < maxRegistersPerChain; reg++) {
//        for (led = 0; led <= chain; led++) {
//          ShiftMuxPWM.SetOne(chain, reg*maxOutputsPerRegister + led, brightness);
//        }
//        for (led = 7; led >= 7 - reg; led--) {
//          ShiftMuxPWM.SetOne(chain, reg*maxOutputsPerRegister + led, maxPWMBrightness - brightness);
//        }
//      }
//    }
//    delay(20);
//  }

  // Fade in and fade out all outputs slowly. Useful for testing your circuit
//  ShiftMuxPWM.OneByOneSlow();
}

// Copies any NodeType specific data from Flash
inline void readFromFlash() {
  if (isMaster) {
    maxNumSlaves = maxSlaveAddress + 1;
    commLineLength = maxNumSlaves;
    return;
  }
  
  int memory = freeMemory();
  readChainTypes();
  if (debug && verbose)
    printMemoryUsed(F("readChainTypes"), memory - freeMemory());
  readSensorThreshold();
  
  if (debug) {
    PRINT("My Chain Types:");
    for (byte chain = 0; chain < maxChainsPerNode; chain++)
      printChainDetails(chain);
  }
}

// Prints the details of a given chain
void printChainDetails(const byte chain) {
  if (!debug)
    return;
  
  // Print chain number and type
  byte typeID = myChainTypeIDs[chain];
  PRINT("\nChain ", chain + 1);
  PRINT(" (Type ", typeID);
  PRINT("):\t ");
  if (typeID == NoChain) {
    PRINTLN("No Chain");
    return;
  }
  
  // Print details of the overlay settings
  PRINT("Overlays ");
  myUsedChainTypes[typeID]->doOverlay ?
    PRINT("on"): PRINT("off");
  
  // Print details of the input type and threshold
  typeID = myInputTypeIDs[chain];
  PRINT("\tInput (Type ", typeID);
  PRINT(") Threshold: ");
  if (typeID == NoInput)
    PRINT("No Input");
  else {
    myUsedInputTypes[typeID]->activeHigh? 
      PRINT(">"): PRINT("<");
    PRINT(" ", mySensorThreshold);
  }
  
  // Print the register and output types
  PRINTLN("\n\tRegister Types:\t", myRegisterTypeIDs[chain], maxRegistersPerChain);
  PRINTLN("\tOutput Types:\t", myOutputTypeIDs[chain], maxOutputsPerChain);
}

inline void initInputPins() {
  // Set the pull-up resistors on all inputs
  for (byte pin = A0; pin <= A5; pin++) {
    pinMode(pin, INPUT);
    digitalWrite(pin, HIGH);
  }
}

