// Slave specific neighbourhood variables
byte lastTiming[maxChainsPerNode]; // the last timing to be run, for overlays
byte levelScale[maxChainsPerNode]; // the scale (out of 255) of the output levels

// Sensor reading variables
unsigned int averageSensorValue[maxChainsPerNode]; // the average sensor values
byte numActiveSensors = 0; // how many sensors are active
byte activeSensors = 0; // which sensors are active

// Cycle variables, reset in endCycle()
unsigned long cycleStart[maxChainsPerNode]; // start of each chain cycle in ms
long cycleTime[maxChainsPerNode]; // length of each chain cycle in ms
byte cycleOn = 0; // 0 if no cycle is going
const byte numCycleTypes = 3;
int keyTime[maxChainsPerNode][maxOutputsPerChain]; // when to switch states, in ms, after cycleStart
unsigned long starting[maxChainsPerNode]; // which outputs are currently waiting to start rising
unsigned long rising[maxChainsPerNode]; // which outputs are currently rising to their maxPWM from off
unsigned long holding[maxChainsPerNode]; // which outputs are currently holding at their maxPWM
unsigned long falling[maxChainsPerNode]; // which outputs are currently falling from their maxPWM to off
unsigned long done[maxChainsPerNode]; // which outputs are done and can be started again

// Default behaviour variables, reset in startDefault()
unsigned long defaultStart; // starts of default timing in ms
unsigned long defaultTime; // time before default starts in ms
byte defaultOn; // if default behaviour is ongoing

// RAM copies of this NodeType's specific Flash data
NodeType* myNodeType;
ChainType* myUsedChainTypes[numChainTypes + 1];                // Indexed using [chainTypeID]
InputType* myUsedInputTypes[numInputTypes + 1];                // Indexed using [inputTypeID]
RegisterType* myUsedRegisterTypes[numRegisterTypes + 1];       // Indexed using [registerTypeID]
OutputType* myUsedOutputTypes[numOutputTypes + 1];             // Indexed using [outputTypeID]
int mySensorThreshold;

// Lists of the various ID values for the above data
const byte* myChainTypeIDs;                                  // Indexed using [chain]
static byte myInputTypeIDs[maxChainsPerNode];                       // Indexed using [chain]
const byte* myRegisterTypeIDs[maxChainsPerNode];                    // Indexed using [chain][register]
byte* myOutputTypeIDs[maxChainsPerNode];                            // Indexed using [chain][output]

void initSlave() {
  for (byte chain = 0; chain < maxChainsPerNode; chain++) {
    done[chain] = allOutputs;
    levelScale[chain] = 0xFF;
    lastTiming[chain] = 0xFF;
    averageSensorValue[chain] = readInput(chain);
    for (byte out = 0; out < maxOutputsPerChain; out++)
      keyTime[chain][out] = -1;
  }
  startDefaultTimer();
}

void slave() {
  checkComm();
  //checkSensors();
  //cycleOn?
    //runCycle():
    //defaultBehaviour();
}

byte mainCommand = NODATA;
byte commandChains = NODATA;
void checkComm() {
  // Check if comm is enabled
  if (!commEnabled)
    return;
  
  // Check if there is data available
  const byte data = Serial.read();
  if (data == NODATA)
    return;
    
      
  // Check if the message is for this node
  if (getAddress(data) != myAddress){
    //Serial.println("NO MESSAGE");
   // Serial.println(data);
    //Serial.println(getAddress(data), DEC);
    //Serial.println(myAddress, DEC);
     //Serial.println("NO MESSAGE");
    return;
  }
  
  
  // Store the message
  const byte message = getMessage(data);
  
  //Serial.print("MESSAGE: ");
  //Serial.println(message);
  
    //byte byte30 = 30;
    if (message == 40){
        LEDOff(0);
        delay(1000);
        
        LEDOn(0);
        delay(100);
        LEDOff(0);
        delay(100);
        LEDOn(0);
        delay(100);
        LEDOff(0);
        delay(100);
        LEDOn(0);
        delay(100);
        LEDOff(0);
        delay(100);
        LEDOn(0);
        delay(100);
        LEDOff(0);
        delay(100);
   } else {
        LEDOff(0);
        delay(1000);
        
        LEDOn(0);
        delay(500);
        LEDOff(0);
        delay(500);
        LEDOn(0);
        delay(500);
        LEDOff(0);
   }
  
  // Check if this is the first byte of a command
  if (mainCommand == NODATA) {
    if (message == STATUS) {
      
      //Serial.println("STATUS MESSAGE");
      // NOTE: The major error response to a status request is in the majorError function
      if (!sensorReporting || numActiveSensors == 0) {
        send(myAddress, 0);
      } else {
//        send(makePacket(myAddress, numActiveSensors),
//             makePacket(myAddress, activeSensors & B111),
//             makePacket(myAddress, activeSensors >> 3));
        send(myAddress, STATUS);
        send(myAddress, numActiveSensors);
        send(myAddress, activeSensors & B111);
        send(myAddress, activeSensors >> 3);
      }
      if (debug && numActiveSensors > 0) {
        ENDLN();
        if (sensorReporting)
          PRINT("Reported ");
        PRINT("Sensors Active: ", numActiveSensors);
        PRINT(", On Chains:");
        for (byte index = 0; index < maxChainsPerNode;) {
          if (bitRead(activeSensors, index++))
            PRINT(" ", index);
        }
        ENDLN();
      }
      numActiveSensors = 0;
      activeSensors = 0;
      return;
    }
    if (debug)
      PRINTLN("\nMain Command: ", message);
    
    switch(message) {
      // All commands other than status are multibyte commands
      case SENSOR: // Sensor command
      case REACTR: // Reactor command
      case NGHBOR: // Neighbour command
      case GLOBAL: // Global behaviour command
        send(myAddress, message);
        mainCommand = message;
        break;
      default:
        MAJORERROR(commError, 1, "checkComm: Unsupported Main Command");
    }
    return;
  }
  
  // Check if this is the second byte of a multibyte command
  if (commandChains == NODATA) {
    // Check if this is a global command (only current 2 byte command set)
    if (mainCommand == GLOBAL) {
      if (debug)
        PRINTLN("\nGlobal command: ", message);
      switch (message) {
        case GBASIC:
        case GNIGHT:
        case GPARTY:
          if (debug)
            PRINTLN("Entering new mode: ", message);
          myMode = message;
          break;
        case GOVRLD:
          if (debug)
            PRINTLN("Global overload!");
          overload(allChains);
          break;
        case GNODEF:
          if (debug)
            PRINTLN("Default activity disabled.");
          defaultFlag = NONE;
          break;
        case GHIDEF:
        case GLODEF:
          defaultFlag++;
          defaultLevelHigh = (message == GHIDEF);
          switch(defaultFlag) {
            case NORMAL:
              if (debug) {
                defaultLevelHigh? PRINT("\nHigh"): PRINT("\nLow");
                PRINTLN(" default activity enabled.");
              }
              if (!cycleOn)
                startDefaultTimer();
              break;
            case EVENING:
              if (debug)
                PRINTLN("Evening mode enabled SLAVE.");
              break;
            case DORMANT:
              if (debug)
                PRINTLN("Dormant mode enabled.");
              break;
            default:
              MAJORERROR(commError, 2, "checkComm: Unsupported default mode");
          }
          break;
        default:
          MAJORERROR(commError, 3, "checkComm: Unsupported Global Command");
      }
      send(myAddress, message);
      mainCommand = NODATA;
      return;
    }
    switch(mainCommand) {
      // All of these commands are 3 byte commands
      case SENSOR: // Sensor command
      case REACTR: // Reactor command
      case NGHBOR: // Neighbour command
        send(myAddress, message);
        commandChains = message;
        if (debug)
          PRINTLN("\nFirst set of command chains: ", commandChains);
        break;
      default:
        if (debug)
          PRINTLN("\nSecond Message: ", message);
        MAJORERROR(commError, 4, "checkComm: Unsupported second message");
    }
    return;
  }
  
  // This is the 3rd byte of a 3 byte command
  if (debug)
    PRINTLN("\nSecond set of command chains: ", message);
  commandChains += message << 3;
  if (commandChains == 0)
    MAJORERROR(commError, 5, "checkComm: No command chains");
  
  // There are command chains, check the main command
  if (mainCommand == REACTR) {
    if (debug) {
      PRINT("Reactor chase! Chains:");
      for (byte i = 0; i < maxChainsPerNode;) {
        if (bitRead(commandChains, i++))
          PRINT(" ", i);
      }
      ENDLN();
    }
    reactorChase(commandChains);
  } else {
    // The main command is based on function-defined behaviours
    switch(mainCommand) {
      case NGHBOR: // Neighbour command
        commNeighbourBehaviour(bitRead(commandChains, 0), commandChains >> 1);
        break;
      case SENSOR: // Sensor command
        for (byte chain = 0; chain < maxChainsPerNode; chain++) {
          if (bitRead(commandChains, chain)) {
            sensorBehaviour(chain);
          }
        }
        break;
      default: // Pretty sure there is no possible way to get here, but just in case...
        MAJORERROR(commError, 6, "checkComm: Unsupported third message");
    }
  }
  send(myAddress, message);
  mainCommand = NODATA;
  commandChains = NODATA;
}

void overload(const byte chains) {
  // last timing index = 0
  for (byte j = 0; j < 10; j++)
    chase(50, chains, allOutputs);
}
  
void reactorChase(const byte chains) {
  // last timing index = 1
  byte hold = 20;
  for (byte iterations = 0; iterations < 3; iterations++) {
    chase(hold, chains, allOutputs);
    hold *= 3;
  }
}

void chase(const int hold, const byte chains, const unsigned long outputs) {
  if (hold <= 0)
    return;
  byte chain;
  for (byte out = 0; out < maxOutputsPerChain; out++) {
    if (!bitRead(outputs, out))
      continue;
    for (chain = 0; chain < maxChainsPerNode; chain++) {
      if (!bitRead(chains, chain) || myOutputTypeIDs[chain][out] == NoOutput)
        continue;
      turnOn(chain, out);
    }
    delay(hold);
    for (chain = 0; chain < maxChainsPerNode; chain++) {
      if (!bitRead(chains, chain) || myOutputTypeIDs[chain][out] == NoOutput)
        continue;
      turnOff(chain, out);
    }
  }
}

void neighbourhoodStuff() {
  // last timing index = 2
}

// This should be called when a neighbour comm message is received
void commNeighbourBehaviour(const boolean startAtFirstChain, byte distance) {
  if (debug) {
    PRINT("Doing comm neighbour behaviour. Starting at C");
    startAtFirstChain? PRINT("1"): PRINT("6");
    PRINTLN(", with distance of ", distance);
  }
  
  if (!startAtFirstChain)
    distance += maxChainsPerNode - 1;
  for (byte chain = 0; chain < maxChainsPerNode; chain++) {
    neighbourChase(chain, distance);
    distance += (startAtFirstChain)? 1: -1;
  }
}

// This should be called to start a neighbour chase on a given
// chain at a given distance away from the active chain.
void neighbourChase(const byte chain, const byte distance) {
  // Check that this is a valid chain and distance
  if (chain >= maxChainsPerNode || distance == 0) {
    if (debug) {
      printText(F("Neighbour chase on "));
      printC(chain);
      PRINTLN(" @ Distance: ", distance);
    }
    MAJORERROR(neighbourError, 1, "neighbourChase*");
  }
  
  // Check that the chain exists
  const byte chainTypeID = myChainTypeIDs[chain];
  if (chainTypeID == NoChain)
    return;
  
  // Check if there is already a cycle going on this chain that's more important
  const boolean chainCycleOn = bitRead(cycleOn, chain);
  if (chainCycleOn && lastTiming[chain] < distance + numCycleTypes)
    return;
  
  // Check if the wave has died from too much dimming
  const unsigned int dimmingFactor = distance*neighbourDimmingFactor;
  if (dimmingFactor >= levelScale[chain])
    return;
  
  // Check if the wave has died from too much output reduction
  const unsigned int outputFactor = distance*neighbourOutputFactor;
  if (outputFactor >= 0xFF)
    return;
  
  // Find the last output
  byte lastOutput;
  byte* chainOutputTypeIDs = myOutputTypeIDs[chain];
  for (lastOutput = 0; lastOutput < maxOutputsPerChain; lastOutput++) {
    if (chainOutputTypeIDs[lastOutput] == backgroundOutputTypeID)
      break;
  }
  
  // Check that there are background enabled outputs on this chain
  if (lastOutput >= maxOutputsPerChain)
    return;
  
  // Find the first output
  byte firstOutput;
  for (firstOutput = maxOutputsPerChain - 1; firstOutput > lastOutput; firstOutput--) {
    if (chainOutputTypeIDs[firstOutput] == backgroundOutputTypeID)
      break;
  }
  
  // Find how many outputs are used
  byte numOutputsUsed = firstOutput - lastOutput + 1;
  if (numOutputsUsed > 2) {
    for (byte output = lastOutput + 1; output < firstOutput; output++) {
      if (chainOutputTypeIDs[output] != backgroundOutputTypeID)
        numOutputsUsed--;
    }
  }
  
  // Check if the wave has died from removing too many outputs
  byte numOutputsToRemove = (numOutputsUsed*outputFactor)/0xFF;
  if (numOutputsToRemove >= numOutputsUsed)
    return;
  
  // Remove outputs as needed
  while (numOutputsToRemove > 0) {
    while(chainOutputTypeIDs[++lastOutput] != backgroundOutputTypeID);
    numOutputsToRemove--;
  }
  
  // Print details of the neighbour chase
  if (debug) {
    printText(F("Neighbour chase on "));
    printC(chain);
    printText(F(" @ Distance: "));
	printValue(distance);
    if (verbose) {
      printText(F(", on "));
	  printD(firstOutput);
      printText(F(" to "));
	  printD(lastOutput);
    }
    endLn();
  }
  
  // This neighbour chase is happening, get rid of the current cycle
  if (chainCycleOn)
    forceChainCycleEnd(chain);
  
  // Setup the keyTime array
  byte outputTypeID;
  unsigned int currentTime = neighbourDelayStep*distance;
  const unsigned int offsetTime = neighbourOffsetTimes[chainTypeID];
  for (char output = firstOutput; output >= lastOutput; output--) {
    // Check that there is actually an output
    outputTypeID = myOutputTypeIDs[chain][output];
    if (outputTypeID == NoOutput)
      continue;
    
    // Set the on time and increment the offset
    keyTime[chain][output] = currentTime;
    currentTime += offsetTime;
    
    // If a cycle is ongoing, setup the output time as necessary
    if (chainCycleOn)
      setupOutputTime(chain, output, outputTypeID);
  }
  
  // Initialize the cycle variables
  levelScale[chain] -= dimmingFactor;
  lastTiming[chain] = distance + numCycleTypes;
  if (!cycleOn) {
    startCycle();
  } else if (starting[chain]) {
    cycleStart[chain] = millis();
    bitSet(cycleOn, chain);
    printCycleStatus(chain);
  }
}

void sensorBehaviour(const byte activeChain) {
  if (activeChain >= maxChainsPerNode)
    MAJORERROR(sensorError, 1, "sensorBehaviour*");
  
  // Check that the chain exists
  const byte chainTypeID = myChainTypeIDs[activeChain];
  if (chainTypeID == NoChain)
    MAJORERROR(sensorError, 2, "sensorBehaviour**");
  
  // Check if there is a cycle on this chain already
  if (bitRead(cycleOn, activeChain)) {
    // Check if sensor behaviour is less important than the current event
    if (lastTiming[activeChain] < numCycleTypes)
      return;
    
    // Check if sensor behaviour is more important than the current event
    if (lastTiming[activeChain] > numCycleTypes)
      forceChainCycleEnd(activeChain);
  }
  
  // Check if overlays are enabled for this chain
  const boolean chainCycleOn = bitRead(cycleOn, activeChain);
  const unsigned long overlay = chainCycleOn? myUsedChainTypes[chainTypeID]->doOverlay: allOutputs;
  if (!overlay) {
    if (debug)
      PRINTLN("Sensor behaviour cancelled, overlays disabled on C", activeChain + 1);
    return;
  }
  
  if (debug)
    PRINTLN("Doing sensor behaviour on C", activeChain + 1);
  
  // Check if there is a cycle on this chain still
  if (chainCycleOn) {
    // Overlays are enabled and the chain has a cycle ongoing
    time = millis() - cycleStart[activeChain];
    if (debug) {
      PRINT("Doing overlay at ", time);
      PRINTLN(" after last cycleStart.");
      printCycleStatus(activeChain);
    }
    cycleStart[activeChain] += time;
    cycleTime[activeChain] = (time >= cycleTime[activeChain])? 0: cycleTime[activeChain] - time;
  }
  
  // Setup the keyTime array
  byte outputTypeID;
  unsigned int currentTime[numOutputTypes + 1];
  for (outputTypeID = numOutputTypes; outputTypeID > NoOutput; outputTypeID--) {
	currentTime[outputTypeID] = myUsedOutputTypes[outputTypeID]->sensorStartTime;
  }
  for (char output = maxOutputsPerChain - 1; output >= 0; output--) {
    // Check that there is actually an output
    outputTypeID = myOutputTypeIDs[activeChain][output];
    if (outputTypeID == NoOutput)
      continue;
    
    // Check if the output is done any previous events
    if (!bitRead(done[activeChain], output)) {
      keyTime[activeChain][output] = (time >= keyTime[activeChain][output])? 0: keyTime[activeChain][output] - time;
      if (debug && verbose)
        printOutputTimes(activeChain, output, outputTypeID);
      continue;
    }
    
    // Check if overlays are enabled for this output
    if (chainCycleOn && !bitRead(overlay, output))
      continue;
    
    // Output exists, is not in use, and has overlays enabled
    keyTime[activeChain][output] = currentTime[outputTypeID];
    
    // If a cycle is ongoing, setup the output time as necessary
    if (chainCycleOn) 
      setupOutputTime(activeChain, output, outputTypeID);
      
    currentTime[outputTypeID] += myUsedOutputTypes[outputTypeID]->sensorOffsetTime;
  }
  
  if (chainCycleOn) {
    printCycleStatus(activeChain);
    if (cycleTime[activeChain] >= 0x7FFF)
      MAJORERROR(sensorError, 3, "sensorBehaviour***");
  }
  
  // Initialize the cycle variables
  levelScale[activeChain] = 0xFF;
  lastTiming[activeChain] = numCycleTypes;
  if (!cycleOn) {
    startCycle();
  } else if (starting[activeChain]) {
    bitSet(cycleOn, activeChain);
  }
  
  // Do peer neighbour behaviour ONLY if sensor behaviour was successful
  if (debug)
    PRINTLN("Doing peer neighbour behaviour.");
  byte distance = activeChain;
  for (byte chain = 0; chain < maxChainsPerNode; chain++) {
    if (chain != activeChain)
      neighbourChase(chain, distance);
    distance += (chain < activeChain)? -1: 1;
  }
}

void checkSensors() {
  if (!sensorsEnabled)
    return;
  
  unsigned int input;
  int diffFromAverage;
  boolean activated;
  byte typeID;
  InputType *type;
  for (byte chain = 0; chain < maxChainsPerNode; chain++) {
    typeID = myInputTypeIDs[chain];
    if (typeID == NoInput)
      continue; // no sensor attached to this chain
    
    input = readInput(chain);
    if (input >= 1000)
      continue; // no signal from this sensor
    
    diffFromAverage = input - averageSensorValue[chain];
    averageSensorValue[chain] = (64*input - 8*(diffFromAverage))/64;
    if (abs(diffFromAverage) < 50)
      continue; // reading is not significantly different from average
    
    // Sensor exists, has a signal, and the signal is different from the average
    activated = myUsedInputTypes[typeID]->activeHigh ?
                  input >= mySensorThreshold: 
                  input <= mySensorThreshold;
    
    // Check if sensor has been activated
    if (!activated)
      continue;
    
    // Sensor is active
    if (debug) {
      PRINT("\nS", chain + 1);
      PRINT(" (Type ", typeID);
      PRINTLN(") on: ", input);
    }
    
    // Record the sensor as active
    if (!bitRead(activeSensors, chain)) {
      bitSet(activeSensors, chain);
      numActiveSensors++;
    }
    if (commEnabled)
      defaultFlag = NONE;
    sensorBehaviour(chain);
  }
  
  // Check if node reactor is enabled and all sensors active
  if (myNodeType->doNodeReactor && numActiveSensors >= maxChainsPerNode) {
    activeSensors = 0;
    numActiveSensors = 0;
    nodeReactor();
  }
}

void nodeReactor() {
  int chains, hold = 10;
  for (byte times = 0; times < 3; times++) {
    for (chains = 1; chains <= 0x80; chains <<= 1)
      chase(hold, chains, allOutputs);
    hold *= 5;
  }
}

/**
 * Starts the default countdown.
 */
void startDefaultTimer() {
  if (debug)
    printText(F("\nFinished cycle."));
  defaultOn = false;
  if (!defaultFlag) {
    endLn();
    return;
  }
  defaultStart = millis();
  defaultTime = myNodeType->defaultTimeout +
    random(-myNodeType->defaultTimeDelta, myNodeType->defaultTimeDelta);
  if (defaultLevelHigh)
    defaultTime = defaultTime/highDefaultLevelRatio;
  if (debug)
    PRINTLN(" Next default: ", defaultTime);
  if (defaultTime < 0)
    MAJORERROR(defaultError, 2, "startDefaultTimer");
}

// Default behaviour is currently a random actuator going off
// at a random time based on defaultTiming and defaultTimeDelta.
void defaultBehaviour() {
  if (!defaultFlag || defaultTime == 0 || millis() - defaultStart < defaultTime)
    return;
  
  if (debug)
    PRINT("\nRunning default ");
  
  if (backgroundRollLikelihood == 0xFF || random(0xFF) < backgroundRollLikelihood) {
    if (debug)
      PRINTLN("rolling glow...");
    createDefaultRoll();
  } else {
    if (debug)
      PRINTLN("random output...");
    // Set a random output to turn on
    byte chain;
    do {
      chain = random(maxChainsPerNode);
    } while (myChainTypeIDs[chain] == NoChain);
    byte out;
    do {
      out = random(maxOutputsPerChain);
    } while (myOutputTypeIDs[chain][out] != backgroundOutputTypeID);
    keyTime[chain][out] = 0;
  }
  
  defaultOn = true;
  startCycle();
}

void createDefaultRoll() {
  // Expanding glow from the middle of the second chain outwards across all chains
  const unsigned int offsetTime = 500;
  for (byte chain = 0; chain < maxChainsPerNode; chain++)
    glowFromMiddle(chain, abs((chain - 1)), offsetTime);
//  glowFromMiddle(0, 1, offsetTime);
//  glowFromMiddle(1, 0, offsetTime);
//  glowFromMiddle(2, 1, offsetTime);
//  glowFromMiddle(3, 2, offsetTime);
//  glowFromMiddle(4, 3, offsetTime);
//  glowFromMiddle(5, 4, offsetTime);
}

void glowFromMiddle(const byte chain, const byte distance, const unsigned int offsetTime) {
  const unsigned int startTime = distance*2*offsetTime;
  if (debug && verbose) {
    printText(F("Glowing "));
    printC(chain);
    printText(F(" from middle: Start @ "));
    printValue(startTime);
    PRINTLN(", offsets of ", offsetTime);
  }
  
  // Check that this is a valid glow command
  if (chain > maxChainsPerNode)
    MAJORERROR(defaultError, 3, "glowFromMiddle*");
  
  // Check that there is a chain
  const byte chainTypeID = myChainTypeIDs[chain];
  if (chainTypeID == NoChain)
    return;
  
  // Find top and bottom background outputs
  byte top = 0;
  char bottom = maxOutputsPerChain - 1;
  while(myOutputTypeIDs[chain][bottom] != backgroundOutputTypeID) {
    if (bottom-- < 0)
      MAJORERROR(defaultError, 4, "glowFromMiddle**");
  }
  while(myOutputTypeIDs[chain][top] != backgroundOutputTypeID) {
    if (top++ > bottom)
      MAJORERROR(defaultError, 5, "glowFromMiddle***");
  }
  
  // Definitely doing a glow
  if (debug && verbose) {
    printText(F("\tOutputs "));
    printD(top);
    printText(F(" to "));
    printD(bottom);
    endLn();
  }
  
  // Insert the chases into the keyTime array
  unsigned int currentTime = startTime;
  for (char output = (bottom-top)/2; output >= 0; output--) {
    keyTime[chain][bottom - output] = currentTime;
    keyTime[chain][top + output] = currentTime;
    currentTime += offsetTime;
  }
}

