// Include the arduino library to use their definitions in this file
#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

// Include the PGM Space library to allow direct use of the flash memory
#include <avr/pgmspace.h>

// Pin definitions for Simons Stack
enum DigitalPin {
  RX = 0, //  0
  TX,     //  1
  DE,     //  2
  LEDR,   //  3
  LEDY,   //  4
  SIN_LO, //  5
  SIN_HI, //  6
  CLK_HI, //  7
  CLK_MD, //  8
  CLK_LO, //  9
  LATCH,  // 10
  SOUT,   // 11
  SIN,    // 12
  CLK,    // 13
};
enum AnalogPin {
  SNS1 = 0, // 0
  SNS2,     // 1
  SNS3,     // 2
  SNS4,     // 3
  SNS5,     // 4
  SNS6      // 5
};
const byte addressPin = LEDY;
const byte RE = LEDR;

const byte numLEDs = 1; // number of debugging LEDs
const byte arduinoLEDPins[numLEDs] = {
  /*LEDY,*/ LEDR}; // arduino LED pins

// If there is an error with the board of some kind then the
// error number will flash the LEDs on the controller.
enum Error {
  programError = 1,
  switchError,
  sensorError,
  cycleError,
  defaultError,
  commError,
  neighbourError
};
void majorError(const Error error, const byte type, const __FlashStringHelper* msg);

// Definitions for ShiftMuxPWM library initialization
const byte maxPWMBrightness = 63;
const byte pwmFrequency = 60;
const byte maxRegistersPerChain = 4; // current max: 4 registers
const byte maxChainsPerNode = 6; // current max: 6 chains

// Definitions based on current architecture
// NOTE: DO NOT CHANGE NAMES OR VALUES!
// NOTE: New values are allowed if a new controller is designed and used.
const byte maxOutputsPerRegister = 8; // unless we move to bigger registers, this is constant
const byte maxOutputsPerChain = maxRegistersPerChain*maxOutputsPerRegister; // number of outputs per chain
const byte allChains = (1 << maxChainsPerNode) - 1;
const unsigned long allOutputs = 0xFFFFFFFF;
const byte masterAddress = 0;

const byte numOutputTimes = 3;
struct OutputType {
  const byte ID;
  union {
    const byte holdLevel;
    byte maxPWM;
  };
  union {
    struct {
      const unsigned int riseFor;
      const unsigned int holdFor;
      const unsigned int fallFor;
    };
    const unsigned int times[numOutputTimes];
  };
  const unsigned int sensorStartTime;
  const unsigned int sensorOffsetTime;
};

struct RegisterType {
  const byte ID;
  const byte *outputTypeIDs;
};

struct InputType {
  const byte ID;
  const bool activeHigh;
};

struct ChainType {
  const byte ID;
  const long doOverlay;
  const byte inputTypeID;
  const byte *registerTypeIDs;
};

struct NodeType {
  const byte ID;
  const bool doNodeReactor;
  const long defaultTimeout;
  const long defaultTimeDelta;
  const byte *chainTypeIDs;
};

