/**
 * Sets up the cycle variables for a new cycle.
 *
 * NOTE: Some variables are required for this function:
 * - keyTime must be filled with times in ms.
 * - cycleOn, cycleTimes, and cycleStarts must be 0.
 * - entries in starting, rising, holding, and falling arrays must be 0.
 * - entries in done array must be AllOutputs.
 */
void startCycle() {
  if (debug)
    PRINTLN("\nStart cycle...");
  if (cycleOn)
    MAJORERROR(cycleError, 1, "startCycle: cycleOn");
  
  time = millis();
  char output;
  byte outputTypeID;
  for (byte chain = 0; chain < maxChainsPerNode; chain++) {
    if (cycleStart[chain] != 0 || cycleTime[chain] != 0) {
      printCycleStatus(chain);
      MAJORERROR(cycleError, 2, "startCycle: cycleStart/Time");
    }
    
    if (starting[chain] || rising[chain] || holding[chain] ||
         falling[chain] || done[chain] != allOutputs)
      MAJORERROR(cycleError, 3, "startCycle: flags");
    
    for (output = maxOutputsPerChain - 1; output >= 0; output--) {
      outputTypeID = myOutputTypeIDs[chain][output];
      if (outputTypeID == NoOutput || keyTime[chain][output] < 0)
        continue;
      
      setupOutputTime(chain, output, outputTypeID);
    }
    
    if (!starting[chain])
      continue;
    
    if (cycleTime[chain] >= 0x7FFF)
      MAJORERROR(cycleError, 15, "startCycle: cycleTime");
    
    bitSet(cycleOn, chain);
    cycleStart[chain] = time;
    printCycleStatus(chain);
  }
}

void setupOutputTime(const byte chain, const byte output, const byte outputTypeID) {
  const unsigned int* times = myUsedOutputTypes[outputTypeID]->times;
  unsigned int doneBy = keyTime[chain][output];
  for (byte i = 0; i < numOutputTimes; i++)
    doneBy += times[i];
  cycleTime[chain] = max(cycleTime[chain], doneBy);
  bitClear(done[chain], output);
  bitSet(starting[chain], output);
  
  if (debug && verbose)
    printOutputTimes(chain, output, outputTypeID);
}

void printOutputTimes(const byte chain, const byte output, const byte outputTypeID) {
  if (!debug || !verbose)
    return;
  
  printC(chain);
  printD(output);
  PRINT(", (Type ", outputTypeID);
  printChar(')');
  printTab();
  
  int localTime = keyTime[chain][output];
  boolean printText = bitRead(starting[chain], output);
  const unsigned int* times = myUsedOutputTypes[outputTypeID]->times;
  if (printText) {
    PRINT("riseAt: ", localTime);
    localTime += times[0];
    PRINT(",");
  } else {
    printTab();
  }
  printTab();
  printText |= bitRead(rising[chain], output);
  if (printText) {
    PRINT("holdAt: ", localTime);
    localTime += times[1];
    PRINT(",");
  } else {
    printTab();
  }
  printTab();
  printText |= bitRead(holding[chain], output);
  if (printText) {
    PRINT("fallAt: ", localTime);
    localTime += times[2];
    PRINT(",");
  } else {
    printTab();
  }
  printTab();
  printText |= bitRead(falling[chain], output);
  if (printText) {
    PRINT("doneAt: ", localTime);
  }
  ENDLN();
}

/**
 * Performs the main output, PWM and timing logic.
 *
 * NOTE: Some variables are required for this function:
 * - keyTime arrays must be set
 * - cycleStart array must be set
 * - cycleTime array must be set
 * - cycleOn must not be 0
 */
void runCycle() {
  if (!cycleOn)
    MAJORERROR(cycleError, 4, "runCycle");
  
  time = millis();
  for (byte chain = 0; chain < maxChainsPerNode; chain++) {
    if (bitRead(cycleOn, chain))
      runChainCycle(chain);
  }
  
  if (!cycleOn)
    startDefaultTimer();
}

/**
 * Performs the main output, PWM and timing logic for a given chain.
 *
 * NOTE: Some variables are required for this function:
 * - keyTime[chain] array must be set
 * - cycleStart[chain] must be set
 * - cycleTime[chain] must be set
 * - cycleOn must have the chain's bit set
 */
void runChainCycle(const byte chain) {
  // Check that the chain is cycling and the cycle times are set
  if (!bitRead(cycleOn, chain) || cycleStart[chain] <= 0 || cycleTime[chain] <= 0) {
    printCycleStatus(chain);
    MAJORERROR(cycleError, 5, "runChainCycle");
  }
  
  // Check if this chain's cycle has started
  if (time < cycleStart[chain])
    return;
  time -= cycleStart[chain];
  
  // Go through the outputs to see which ones need to change
  int pwm;
  OutputType *type;
  for (byte out = 0; out < maxOutputsPerChain; out++) {
    // Check if this output is already done
    if (bitRead(done[chain], out))
      continue;
    
    // Save a reference to this output's outputType
    type = myUsedOutputTypes[myOutputTypeIDs[chain][out]];
    
    // Check if the output is waiting to start rising
    if (bitRead(starting[chain], out)) {
      // Check if the time has arrived
      if (keyTime[chain][out] > time)
        continue;
      
      // Time to start rising
      printOutputStatus(F("RISE: "), chain, out);
      bitClear(starting[chain], out);
      bitSet(rising[chain], out);
      keyTime[chain][out] += type->riseFor;
    }
    
    // Check if the output is currently rising
    if (bitRead(rising[chain], out)) {
      // Check if the time has arrived
      if (keyTime[chain][out] > time) {
        // Do rising math
        pwm = map(keyTime[chain][out] - time, 0, type->riseFor, type->maxPWM, 0);
        pwm = (pwm*levelScale[chain])/0xFF;
        setPWM(chain, out, pwm);
        continue;
      }
      
      // Time to start holding
      setPWM(chain, out, (type->maxPWM*levelScale[chain])/0xFF);
      printOutputStatus(F("HOLD: "), chain, out);
      bitClear(rising[chain], out);
      bitSet(holding[chain], out);
      keyTime[chain][out] += type->holdFor;
    }
    
    // Check if the output is currently holding
    if (bitRead(holding[chain], out)) {
      // Check if the time has arrived
      if (keyTime[chain][out] > time)
        continue;
      
      // Time to start falling
      printOutputStatus(F("FALL: "), chain, out);
      bitClear(holding[chain], out);
      bitSet(falling[chain], out);
      keyTime[chain][out] += type->fallFor;
    }
    
    // Check if the output is currently falling
    if (bitRead(falling[chain], out)) {
      // Check if the time has arrived
      if (keyTime[chain][out] > time) {
        // Do falling math
        pwm = map(keyTime[chain][out] - time, 0, type->fallFor, 0, type->maxPWM);
        pwm = (pwm*levelScale[chain])/0xFF;
        setPWM(chain, out, pwm);
        continue;
      }
      
      // Time has arrived, flag the output as done and setup for next cycle
      turnOff(chain, out);
      printOutputStatus(F("DONE: "), chain, out);
      bitClear(falling[chain], out);
      bitSet(done[chain], out);
      keyTime[chain][out] = -1;
    }
  }
  
  // Reset the time variable
  time += cycleStart[chain];
  
  // Check if all of this chain's outputs are done
  if (done[chain] == allOutputs) {
    if (debug) {
      printText(F("\nCycle on "));
      printC(chain);
      printText(F(" has finished at "));
      printValue(time - cycleStart[chain]);
      printText(F(" ms after cycle start"));
    }
    bitClear(cycleOn, chain);
    lastTiming[chain] = 0xFF;
    levelScale[chain] = 0xFF;
    cycleStart[chain] = 0;
    cycleTime[chain] = 0;
  }
}

/**
 * Forcibly ends a cycle.
 * Clears keyTime and cycleOn if successful.
 * Resets all cycle flags if successful.
 *
 * NOTE: Some variables are required for this function:
 * - cycleOn must be true
 * - cycle flags (starting, rising, holding, falling, done) must be correct
 */
void forceCycleEnd() {
  if (!cycleOn)
    MAJORERROR(cycleError, 6, "forceCycleEnd");
  
  if (debug)
    PRINTLN("\nForced cycle end");
  for (byte chain = 0; chain < maxChainsPerNode; chain++)
    forceChainCycleEnd(chain);
}

/**
 * Forcibly ends the cycle on a given chain.
 * Clears keyTime and cycleOn flags for the chain if successful.
 * Resets all cycle flags for the chain if successful.
 *
 * NOTE: Some variables are required for this function:
 * - cycleOn must be set for the given chain
 * - cycle flags (starting, rising, holding, falling, done) must be correct for the given chain
 */
void forceChainCycleEnd(const byte chain) {
  // Check if cycle is on for this chain
  if (!bitRead(cycleOn, chain)) {
    printCycleStatus(chain);
    MAJORERROR(cycleError, 7, "forceChainCycleEnd");
  }
  
  // Run through the outputs for this chain
  for (byte out = 0; out < maxOutputsPerChain; out++) {
    // Check if output is doing something
    if (bitRead(done[chain], out))
      continue;
    
    // Output is definitely doing something
    if (bitRead(rising[chain], out) || bitRead(holding[chain], out) || bitRead(falling[chain], out))
      turnOff(chain, out);
    keyTime[chain][out] = -1;
    bitSet(done[chain], out);
  }
  if (done[chain] != allOutputs)
    MAJORERROR(cycleError, 8, "forceChainCycleEnd");
  starting[chain] = 0;
  rising[chain] = 0;
  holding[chain] = 0;
  falling[chain] = 0;
  lastTiming[chain] = 0xFF;
  levelScale[chain] = 0xFF;
  cycleStart[chain] = 0;
  cycleTime[chain] = 0;
  bitClear(cycleOn, chain);
  
  if (!cycleOn)
    startDefaultTimer();
}

// Prints the cycle status
void printCycleStatus(const byte chain) {
  if (!debug)
    return;
  printC(chain);
  PRINT("\tCycleOn: ", bitRead(cycleOn, chain));
  PRINT(", Time: ", cycleTime[chain]);
  PRINTLN(", Start: ", cycleStart[chain]);
}
