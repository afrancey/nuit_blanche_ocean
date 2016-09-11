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


// pattern definitions on the SQUARE
// there is only ONE node here, easy peasy
// each row has 6 columns each representing one chain
// pattern_ij = n-bit number representing the state of 
// the j'th chain in the i'th stage of the pattern
// where n is the number of cables in the chain
//const int pattern[6][6] = {
//         {1,1,0,0,0,0},
//         {2,2,0,0,0,0},
//         {1,4,0,0,0,0},
//         {8,8,0,0,0,0},
//         {1,16,0,0,0,0},
//         {50,50,0,0,0,0}};


//bounceout
const int pattern_bounce[50][6] = {
{ 0 , 0 , 0 , 0 , 32 , 0 },
{ 0 , 0 , 0 , 0 , 32 , 0 },
{ 0 , 0 , 0 , 0 , 32 , 0 },
{ 0 , 256 , 0 , 0 , 32 , 0 },
{ 0 , 384 , 0 , 0 , 32 , 0 },
{ 0 , 384 , 0 , 0 , 32 , 0 },
{ 0 , 384 , 0 , 0 , 32 , 0 },
{ 0 , 384 , 0 , 0 , 32 , 0 },
{ 0 , 384 , 0 , 0 , 32 , 0 },
{ 0 , 384 , 0 , 0 , 32 , 0 },
{ 0 , 960 , 0 , 0 , 112 , 0 },
{ 0 , 960 , 0 , 0 , 112 , 0 },
{ 0 , 960 , 0 , 0 , 112 , 0 },
{ 0 , 960 , 0 , 0 , 112 , 0 },
{ 0 , 960 , 0 , 0 , 112 , 0 },
{ 0 , 960 , 0 , 0 , 112 , 0 },
{ 0 , 1984 , 0 , 0 , 112 , 0 },
{ 0 , 2016 , 0 , 0 , 112 , 0 },
{ 0 , 2016 , 0 , 0 , 112 , 0 },
{ 0 , 2016 , 0 , 0 , 112 , 0 },
{ 1 , 2016 , 1 , 0 , 248 , 0 },
{ 1 , 2016 , 1 , 0 , 248 , 0 },
{ 1 , 2016 , 1 , 0 , 248 , 0 },
{ 1 , 4064 , 1 , 0 , 248 , 0 },
{ 1 , 4080 , 3 , 0 , 248 , 0 },
{ 3 , 4080 , 3 , 0 , 248 , 0 },
{ 3 , 4080 , 3 , 0 , 248 , 0 },
{ 3 , 4080 , 3 , 0 , 248 , 0 },
{ 3 , 4080 , 7 , 0 , 248 , 0 },
{ 7 , 4080 , 7 , 0 , 248 , 0 },
{ 7 , 8184 , 7 , 0 , 252 , 0 },
{ 7 , 8184 , 7 , 0 , 252 , 0 },
{ 7 , 8184 , 15 , 0 , 252 , 0 },
{ 15 , 8184 , 15 , 0 , 252 , 0 },
{ 15 , 8184 , 15 , 0 , 252 , 0 },
{ 15 , 8184 , 15 , 0 , 252 , 0 },
{ 15 , 16376 , 15 , 0 , 252 , 0 },
{ 15 , 16380 , 31 , 0 , 252 , 0 },
{ 31 , 16380 , 31 , 0 , 252 , 0 },
{ 31 , 16380 , 31 , 0 , 252 , 0 },
{ 31 , 16380 , 31 , 0 , 254 , 0 },
{ 31 , 16380 , 63 , 0 , 254 , 0 },
{ 63 , 16380 , 63 , 0 , 254 , 0 },
{ 63 , 32764 , 63 , 0 , 254 , 0 },
{ 63 , 32766 , 63 , 0 , 254 , 0 },
{ 63 , 32766 , 127 , 0 , 254 , 0 },
{ 127 , 32766 , 127 , 0 , 254 , 0 },
{ 127 , 32766 , 127 , 0 , 254 , 0 },
{ 127 , 32766 , 127 , 0 , 254 , 0 },
{ 127 , 32766 , 127 , 0 , 254 , 0 }};


// lineacross?
const int pattern[51][6] = {
{ 128 , 1 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 64 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 2 , 0 , 0 , 0 , 0 },
{ 32 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 16 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 8 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 16 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 1 , 0 , 0 , 0 , 128 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 64 , 0 , 0 , 64 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 128 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 32 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 512 , 0 , 0 , 16 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 1024 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 1 , 0 , 8 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 2 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 4 , 0 , 0 , 0 },
{ 0 , 4096 , 0 , 0 , 4 , 0 },
{ 0 , 0 , 8 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 8192 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 2 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 0 , 0 , 0 , 0 },
{ 0 , 0 , 128 , 0 , 0 , 255 }};


// pattern with Brightness: brightness of each LED
// pattern definitions on the SQUARE
// there is only ONE node here, easy peasy
// each row has 6 columns each representing one chain
// patternWithBrightness_ij = string representing the state of 
// the j'th chain in the i'th stage of the pattern
// string format: '4lights-4lights-....'
// 4-lights: 24-bit number decribing the brightness of 4 lights
// USES STRINGS --- BAAAD, LOTS OF MEMORY
// USED TO AVOID INITIALIZING HUGE ARRAY WITH MAX NUMBER OF CHAINS
// SOLUTION: DYNAMICALLY ALLOCATE MEMORY TO HAVE UNEVEN MULTIDIMENIONAL ARRAY
  
         
//const int pattern[50][6] = {
//{ 128 , 1 , 0 , 0 , 0 , 0 },
//{ 0 , 8 , 0 , 0 , 0 , 0 },
//{ 0 , 64 , 0 , 0 , 64 , 0 },
//{ 0 , 512 , 0 , 0 , 16 , 0 },
//{ 0 , 4096 , 0 , 0 , 4 , 0 },
//{ 0 , 0 , 128 , 0 , 0 , 255 }};

//const int pattern[101][6] = { //too much memory
//{ 128 , 1 , 0 , 0 , 0 , 0 }};
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 64 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 2 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 32 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 16 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 4 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 8 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 8 , 0 , 0 , 0 , 0 },
//{ 4 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 2 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 16 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 1 , 0 , 0 , 0 , 128 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 32 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 64 , 0 , 0 , 64 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 128 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 32 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 256 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 512 , 0 , 0 , 16 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 1024 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 1 , 0 , 8 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 2048 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 2 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 4 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 4096 , 0 , 0 , 4 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 8 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 8192 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 16 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 2 , 0 },
//{ 0 , 0 , 32 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 16384 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 64 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 0 , 0 , 0 , 0 },
//{ 0 , 0 , 128 , 0 , 0 , 255 }};

//const int pattern
         
const int chainLengths[6] = {1,1,1,1,1,1};

const int patternLength = 51;
