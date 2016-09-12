// Master specific communication variables
unsigned int timeout;
byte maxNumSlaves;

// Live node list variables
boolean *alive;
byte checkIndex;
byte numLiveNodes;

// Unused node list variables
boolean *unused;
byte numUnusedAddresses;

// Background behaviour variables
unsigned long lastActiveTime;
// Lengths of time to wait for events in minutes   (* 60000 to convert to ms)
const unsigned long waitBeforeDefault = 60000*     1/4;
const unsigned long waitBeforeEvening = 60000*     1/4;
const unsigned long waitBeforeDormant = 60000*     1/4;

// Reactor variables
byte **reactors;
byte *sensorCount;
unsigned long *reactorTime;

// Global overload variables
byte globalSensorCount = 0;
unsigned long globalOverloadTime;

void initMaster() {
  // Only initialize if comm is enabled
  if (!commEnabled) {
    if (usingTestBed && debug)
      PRINTLN("Passing along serial data from Slave");
    return;
  }
  
  // Initialize all master-specific variables.
  buildUnusedList();
  buildReactors();
  buildLiveList();
  timeout = 200;
  globalOverloadTime = millis();
  lastActiveTime = millis();
}

// Creates an array of unused node addresses.
void buildUnusedList() {
  numUnusedAddresses = sizeof(unusedSlaveAddresses);
  commLineLength -= numUnusedAddresses;
  commLineLength *= 64/commLineLength;
  unused = (boolean*) calloc(maxNumSlaves, sizeof(boolean));
  for (byte index = 0; index < numUnusedAddresses; index++) {
    unused[unusedSlaveAddresses[index]] = true;
  }
  if (debug) {
    PRINT("Unused Addresses:");
    for (byte index = 0; index < numUnusedAddresses; index++)
      PRINT(" ", unusedSlaveAddresses[index]);
    ENDLN();
  }
}

// Creates a multidimensional array of reactor chain addresses.
void buildReactors() {
  byte reactor, address, chain;
  sensorCount = (byte*) calloc(numReactors, sizeof(byte));
  reactorTime = (unsigned long*) calloc(numReactors, sizeof(long));
  reactors = (byte**) calloc(numReactors, sizeof(byte*));
  for (reactor = 0; reactor < numReactors; reactor++) {
    reactorTime[reactor] = millis();
    reactors[reactor] = (byte*) calloc(maxNumSlaves, sizeof(byte));
    for (byte j = 0; j < reactorChainSize[reactor]; j++) {
      address = reactorChains[reactor][j] & 0x1F;
      if (reactorChains[reactor][j] == address) {
        reactors[reactor][address] = allChains;
        continue;
      }
      chain = (reactorChains[reactor][j] >> 5) - 1;
      bitSet(reactors[reactor][address], chain);
    }
  }
  if (debug) {
    PRINTLN("Reactor Addresses:");
    for (byte reactor = 0; reactor < numReactors; reactor++) {
      PRINT("\tReactor ", reactor);
      printChar(':');
      for (address = 0; address <= maxSlaveAddress; address++) {
        if (!reactors[reactor][address])
          continue;
        PRINT(" ", address);
        printChar('C');
        if (reactors[reactor][address] == allChains) {
          printChar('X');
          continue;
        }
        for (chain = 0; chain < maxChainsPerNode;) {
          if (bitRead(reactors[reactor][address], chain++))
            printValue(chain);
        }
      }
      ENDLN();
    }
  }
}

// Creates an array of live nodes
void buildLiveList() {
  numLiveNodes = 0;
  alive = (boolean*) calloc(maxNumSlaves + 1, 1);
  for (byte addr = 0; addr < maxNumSlaves; addr++) {
    // Check if node address is unused
    if (unused[addr])
      continue;
    
    // Add the address to the live list
    alive[addr] = true;
    numLiveNodes++;
  }
  checkIndex = maxNumSlaves;
}

elapsedMillis testMessageTimer = 0;
void doTimedTestMessage(){
    if( testMessageTimer > 4000 ){
        PRINTLN("Timed Message Sending...");
        testMessageTimer = 0;
        sendCommand(B0010, BLINK);
    }
}

void master() {
  // If comm is disabled, the master's role is nonexistant
  if (!commEnabled) {
    if (debug) {
      if (usingTestBed) {
        while (Serial.available())
          Serial.write(Serial.read());
      } else {
        PRINTLN("Comm is DISABLED! Master is not being used.");
        delay(1000);
      }
    }
    return;
  } else { // Communication is enabled. Send test message.
        
        /** Partially working communication test.
        
        The code for reading serial below works, but only conditionally. A few
        things to note:
        
        1. The red LED is connected to the switch that turns on and off reading
        from computer serial on the master. Therefore LEDOn and LEDOff interfere
        with serial read. They cannot be used during serial operations on the
        Master.
        
        2. The delay before the read is necessary in this case because otherwise 
        serial messages from the computer end up getting thrown out. This will
        be an issue moving forward. Currently, only messages sent during the
        delay (when the light is on) show up in the serial buffer.
        */
        
        // Delay and flash the light.
        LEDOn(0);
        delay(500);
        LEDOff(0);
        
        // Switch modes and read serial input from computer.
        digitalWrite(RE, HIGH);
        while (Serial.available()){
            char incoming = Serial.read();
            if( incoming == 'a' ){
                Serial.println('A');
            } else if (incoming == 'b'){
                Serial.println('B');
                digitalWrite(RE, LOW);
                delay(1);
                sendCommand(B0010, BLINK);
                delay(1);
                digitalWrite(RE, HIGH);
            } else if (incoming == 'c'){
                Serial.println('C');
            } else {
                Serial.print(incoming);
            }
        }
        digitalWrite(RE, LOW);
        //doTimedTestMessage();
  }
  
  byte times;
  const byte timesLimit = 5;
  boolean checkingThisAddress = false;
  for (byte addr = 0; addr <= maxNumSlaves; addr++) {
    // Check if node is alive
    if (!alive[addr])
      continue;

    // Check the node's status
    times = 0;
    if (checkIndex == addr) {
      checkingThisAddress = true;
      if (debug && checkIndex != maxNumSlaves)
        PRINTLN("\nChecking node: ", checkIndex);
    }
    while (!checkStatus(addr) && times++ < timesLimit) {
      if (checkingThisAddress) {
        alive[checkIndex] = false;
        break;
      }
    }
    
    // See if we're the checked node
    if (checkingThisAddress) {
      // Check if the node is newly alive
      if (alive[checkIndex]) {
        numLiveNodes++; // increment count
        if (debug) {
          PRINT("\nFound live node: ", checkIndex);
          PRINTLN(", ms: ", millis() - time);
        }
      }
      
      // Increment the checkIndex
      if (numLiveNodes + numUnusedAddresses == maxNumSlaves)
        checkIndex = maxNumSlaves;
      else {
        do
          checkIndex = (checkIndex + 1) % maxNumSlaves;
        while (alive[checkIndex] || unused[checkIndex]);
      }
      
      // Reset the checking flag
      checkingThisAddress = false;
    }
    
    // Check if node is unresponsive
    if (times >= timesLimit) {
      alive[addr] = false;
      numLiveNodes--; // decrement count
      if (debug) {
        PRINT("\nNode died: ", addr);
        PRINTLN(", ms: ", millis() - time);
      }
    }    
    
    // Activate checking dead nodes
    if (numLiveNodes == 0 ||
        commCount == commLineLength - 1)
      alive[checkIndex] = true;
  }
  
  // Check the timers associated with default behaviours
  time = millis() - lastActiveTime;
  if ((defaultFlag == NONE && time >= waitBeforeDefault) ||
      (defaultFlag == NORMAL && time >= waitBeforeDefault + waitBeforeEvening) ||
      (defaultFlag == EVENING && time >= waitBeforeDefault + waitBeforeEvening + waitBeforeDormant)) {
    setGlobalDefaultBehaviour(defaultFlag + 1);
  }
  
  // Check if the mode has been changed
  updateGlobalMode();
}

inline byte read(const byte address) {
  byte data, msgAddr, times = 0;
  do {
    time = millis();
    do
      msgAddr = getAddress((data = Serial.read()));
    while (msgAddr != address && millis() - time < timeout);
  } while (msgAddr != address && times++ < 3);
  
  // Check if we timed out
  if (msgAddr != address)
    return NODATA;
  
  // Return the message
  return getMessage(data);
}

// Check the status of a given node
boolean checkStatus(const byte address) {
  
  
  if (address == maxNumSlaves)
    return false;
  
  // Request the status
  send(address, STATUS);
  const byte response = read(address);
  if (response != STATUS)
    return response == 0;
  
  // Check if node is checked node
  if (checkIndex == address){
    return true;
  }
  
  // Read how many sensors are active
  const byte numActive = read(address);
  if (numActive == NODATA)
    return false;
  
  // Check if major error has occurred
  // TODO: Make this check based on the specific node's number of chains.
  if (numActive > maxChainsPerNode) {
    // Read in the error number and type from the node.
    const byte error = read(address);
    const byte type = read(address);
    if (debug) {
      PRINT("\nNode ", address);
      PRINT(" had a major error:\n\tError #", error);
      PRINTLN(", Type ", type);
    }
    // Return false, as the node will reset itself after sending this message
    return false;
  }
  
  // Find out which chains had active sensors
  const byte lowChains = read(address);
  const byte highChains = read(address);
  const byte activeChains = (highChains << 3) + lowChains;
  
  // Print the contents of the messages
  if (debug) {
    PRINT("\nNode ", address);
    PRINT(" has ", numActive);
    PRINT(" chains active:");
    for (byte chain = 0; chain < maxChainsPerNode;) {
      if (bitRead(activeChains, chain++))
        printValue(chain);
    }
    ENDLN();
  }
  
  // Make sure the messages make sense
  if (numActive <= maxChainsPerNode) {
    byte check = numActive;
    for (byte chain = 0; chain < maxChainsPerNode; chain++) {
      if (bitRead(activeChains, chain))
        check--;
    }
    if (check != 0) {
      if (debug)
        PRINTLN("Active sensors mismatch.");
      return false;
    }
  }
  
  // Sensors have definitely gone off
  lastActiveTime = millis();
  globalSensorCount += numActive;
  if (debug) {
    PRINT("Number of global reactor sensor hits: ", globalSensorCount);
    PRINTLN("/", globalOverloadThreshold);
  }
  
  // Check if the global reactor has been set off
  if (globalSensorCount >= globalOverloadThreshold) {
    activateGlobalOverload();
    return true;
  }
  
  // Global reactor was not set off, check for global sensor count memory loss
  if (millis() - globalOverloadTime >= globalOverloadTimeout) {
    globalOverloadTime = millis();
    globalSensorCount /= 2;
  }
  
  // Check if any local reactors have been set off
  byte activeReactorChains;
  for (byte reactor = 0; reactor < numReactors; reactor++) {
    // Check if this node address is part of this reactor.
    if (!reactors[reactor][address])
      continue;
    
    // Check if any of the active chains are part of this reactor.
    activeReactorChains = reactors[reactor][address] & activeChains;
    if (!activeReactorChains)
      continue;
    
    // Increment the sensor count for each active chain in this reactor.
    for (byte chain = 0; chain < maxChainsPerNode; chain++)
      sensorCount[reactor] += bitRead(activeReactorChains, chain);
    if (debug) {
      PRINT("Number of Reactor ", reactor);
      PRINT(" sensor hits: ", sensorCount[reactor]);
      PRINTLN("/", reactorChainSize[reactor]*reactorMultiplier);
    }
      
    // Check if allotted number of sensors have been triggered.
    if (sensorCount[reactor] >= reactorChainSize[reactor]*reactorMultiplier) {
      activateReactor(reactor);
      reactorTime[reactor] = millis();
      sensorCount[reactor] = 0;
      if (debug) {
        PRINT("\nReactor ", reactor);
        PRINTLN(" was set off!");
      }
      continue;
    }
    
    // Local reactor was not set off, check for sensor count memory loss
    if (millis() - reactorTime[reactor] >= reactorTimeout) {
      sensorCount[reactor] /= 2;
      reactorTime[reactor] = millis();
    }
  }
  /*
  // Disable default behaviour for a while
  setGlobalDefaultBehaviour(NONE);
  
  // Notify the neighbours of the active chains
  notifyNeighbours(address, activeChains);*/
  
  return true;
}

void setGlobalDefaultBehaviour(const byte allow) {
  // Check if default is already set correctly
  if (defaultFlag == allow)
    return;
  
  // Change the local flag
  defaultFlag = allow;
  
  // Check if we are disabling default
  if (!defaultFlag) {
    // Reset all of the sensor counts and times
    globalSensorCount = 0;
    globalOverloadTime = millis();
    for (byte reactor = 0; reactor < numReactors; reactor++) {
      sensorCount[reactor] = 0;
      reactorTime[reactor] = millis();
    }
  }
  if (debug) {
    switch(defaultFlag) {
      case DORMANT:
        PRINT("\nDormant mode");
        break;
      case EVENING:
        PRINT("\nEvening mode MASTER");
        break;
      case NORMAL:
      case NONE:
        PRINT("\nGlobal ");
        defaultLevelHigh? PRINT("high"): PRINT("low");
        PRINT(" default behaviour");
        break;
      default:
        MAJORERROR(defaultError, 1, "Invalid default flag.");
    }
    PRINT(" will now be ");
    defaultFlag? PRINT("en"): PRINT("dis");
    PRINTLN("abled.");
  }
  // Determine which global default command to send
  const byte command = (!allow)? GNODEF:
                                 defaultLevelHigh?
                                   GHIDEF: GLODEF;
  
  // Send the global command to all nodes
  sendGlobalCommand(command);
}

void activateGlobalOverload() {
  // Send the global overload command to all nodes
  sendGlobalCommand(GOVRLD);
  
  // Wait until the global overload is finished
  if (debug)
    PRINT("\nGlobal overload: ");
  overload(0);
  if (debug)
    PRINTLN("Done.");
  
  // Reset the global overload variables
  globalSensorCount = 0;
  globalOverloadTime = millis();
}

void updateGlobalMode() {
  // Check if the switches have changed
  readAddress();
  if (myAddress == myMode)
    return;
  
  // Check if the new value is a valid mode
  if (myAddress < GBASIC || myAddress > GPARTY)
    return;
  
  // The switches are different and a valid mode, store the new mode
  myMode = myAddress;
  
  // Send the new global mode to all nodes
  sendGlobalCommand(myMode);
  
  if (debug)
    PRINTLN("\nNew global mode: ", myMode);
}

void sendGlobalCommand(const byte command) {
  for (byte address = 0; address < maxNumSlaves; address++) {
    // Make sure the node is alive and not the checked node
    if (!alive[address] || checkIndex == address)
      continue;
    
    // Make sure global commands are enabled
    if (!sendCommand(address, GLOBAL))
      continue;
    
    // Send specific global command
    sendCommand(address, command);
  }
}

inline void activateReactor(const byte reactor) {
  byte chains = 0;
  for (byte address = 0; address < maxNumSlaves; address++) {
    // Make sure the node is alive and has chains in this reactor
    chains = reactors[reactor][address];
    if (!alive[address] || chains == 0)
      continue;
    alive[address] = false;
    numLiveNodes--;
    // Check if node can do reactor commands
    if (!sendCommand(address, REACTR))
      continue;
    // Send commands to run reactionChase function on reactor's chains
    if (!sendCommand(address, chains & B111))
      continue;
    sendCommand(address, chains >> 3);
  }
  if (numLiveNodes == 0) {
    do
      checkIndex = (checkIndex + 1) % maxNumSlaves;
    while (alive[checkIndex] || unused[checkIndex]);
    alive[checkIndex] = true;
  }
}

// Call this function to notify neighbours of an active neighbouring node
void notifyNeighbours(const byte address, const byte activeChains) {
  // Need a list of the nodes in a neighbourhood, in order
  char node, hood;
  boolean foundNeighbourhood = false;
  for (hood = 0; hood < numNeighbourhoods; hood++) {
    for (node = neighbourhoodSize[hood] - 1; node >= 0; node--) {
      if (allNeighbourhoods[hood][node] == address) {
        foundNeighbourhood = true;
        break;
      }
    }
    if (foundNeighbourhood)
      break;
  }
  
  // Check that a neighbourhood was found for the active node
  if (!foundNeighbourhood)
    return;
  
  // Create a local reference
  const byte* neighbourhood = allNeighbourhoods[hood];
  
  // Print debug information after checking if there are neighbours
  if (debug) {
    PRINT("Notifying neighbours of node ", address);
    PRINT("'s active chains: ");
    for (byte chain = 0; chain < maxChainsPerNode;) {
      if (bitRead(activeChains, chain++))
        printValue(chain);
    }
    PRINTLN("\n\tNeighbourhood:", allNeighbourhoods[hood], neighbourhoodSize[hood]);
  }
  
  // The active node is located at index [node] of the neighbourhood array
  // Determine the distance to the preceding neighbours and notify them
  byte distance = 0;
  while (!bitRead(activeChains, distance++));
  for (char n = node - 1; n >= 0; n--, distance += maxChainsPerNode)
    sendCommNeighbourCommand(neighbourhood[n], distance, false);
  
  // Determine the distance to the following neighbours and notify them
  distance = maxChainsPerNode - 1;
  while (!bitRead(activeChains, distance--));
  distance = maxChainsPerNode - distance - 1;
  for (char n = node + 1; n < neighbourhoodSize[hood]; n++, distance += maxChainsPerNode)
    sendCommNeighbourCommand(neighbourhood[n], distance, true);
}

boolean sendCommNeighbourCommand(const byte address, const byte distance, const boolean startWithFirstChain) {
  // Check if node is alive
  if (!alive[address])
    return false;
  
  // Print debug information
  if (debug) {
    PRINT("\tNode ", address);
    PRINT(", Distance: ", distance);
    PRINT(", Starting with C");
    startWithFirstChain? PRINTLN("1"): PRINTLN("6");
  }
  
  // Check if node can do neighbourhood commands
  if (!sendCommand(address, NGHBOR))
    return false;
  
  // Send commands to run neighbourhood behaviour
  if (!sendCommand(address, ((distance << 1) + startWithFirstChain) & B111))
    return false;
  if (!sendCommand(address, distance >> 2))
    return false;
  
  // Command was successfully sent
  return true;
}

// Send a command and wait for an echo
boolean sendCommand(const byte address, const byte message) {
  return sendCommand(makePacket(address, message));
}
boolean sendCommand(const byte data) {
//  byte times = 0;
//  while (times++ < 3) {   //  RGaug3 -- is this necessary?
    send(data); // Send the data
    time = millis(); // Check the time
    while (millis() - time < timeout) { // Continue checking until timeout has elapsed
      if (Serial.read() == data) // Check response against the sent data
        return true; // Echo was there
    }
//  }
  return false; // Echo was missing
}

