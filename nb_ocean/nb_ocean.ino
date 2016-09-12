// Code for Shift-Based Control Boards
// Primary author: Brandon J DeHart
// (C) 2012 Philip Beesley Architect Inc.
// Last Major Update: September 28, 2012

// NOTE: DO NOT CHANGE OR REMOVE THIS FOR ANY REASON!
#include "definitions.h"
#include <elapsedMillis.h>

// Change this value to identify a new code version: increment by 1
// Even values turn off the red LED, odd values turn it on
// NOTE: DO NOT CHANGE NAME!
const byte codeVersion = 1;

// Setting this to true will print the initialization statements
// NOTE: DO NOT CHANGE NAME!
//const boolean debug = false;
const boolean debug = true; // required for initial debug

// Setting this to false will cause the slave to not perform default behaviour.
// NOTE: DO NOT CHANGE NAME!
boolean defaultFlag = true;
//boolean defaultFlag = false; // handy for initial debug

// Setting this to false will cause the slave to lie about its sensor
// status, never reporting triggered sensors.  This has the effect of
// inhibiting neighbour response initiation from that specific node,
// since the Master will never know that the node's sensors were triggered.
// NOTE: DO NOT CHANGE NAME!
//boolean sensorReporting = true;
boolean sensorReporting = false; // handy for initial debug

// Variable to control the likelihood of background rolls vs randomized outputs
// Various settings will change the background behaviour:
// * A value of 0 will mean the background is always randomized outputs (starry night)
// * A value of 255 will mean the background is always node-localized rolls
// * A value between these two extremes will lead to behaviour between the extremes
// NOTE: DO NOT CHANGE NAME!
const byte backgroundRollLikelihood = 63; // Must be in range of 0 to 255

// **********************************************************************************
// Definitions for NodeTypes
// NOTE: If types or order is changed, must change arrays below.
enum NodeTypeID {
  Master = 0, // NOTE: DO NOT CHANGE NAME OR VALUE!
  NodeOne,    // Node 1
  NodeTwo,    // Node 2
  NodeThree,  // Node 3
  NodeFour,   // Node 4
  NodeFive,   // Node 5
  NodeSix,    // Node 6
  NodeSeven,  // Node 7
  NodeEight,  // Node 8
  EndOfNodes  // NOTE: DO NOT CHANGE NAME!
};
const byte numNodeTypes = EndOfNodes - 1;
  
// The highest used slave address
// NOTE: DO NOT CHANGE NAME!
const byte maxSlaveAddress = 8;

// This array is based on the order of NodeTypeID.
// This array is indexed using [NodeTypeID - 1]
// The address is defined by the rotary HEX switches.
// NOTE: DO NOT CHANGE NAME!
const byte maxNodeAddress [numNodeTypes] = {
  1, 2, 3, 4, 5, 6, 7, maxSlaveAddress}; // maximum address for each node type

// Any node addresses that are unused.
// NOTE: DO NOT CHANGE NAME!
const byte unusedSlaveAddresses[] = {
  1, 3, 4, 5, 6, 7, 8,
  masterAddress}; // The master address + any unused slave addresses

// **********************************************************************************
// Definitions of OutputTypes.
// NOTE: If this enumeration is changed, allOutputTypes must be changed to suit.
// NOTE: DO NOT CHANGE NAME!
enum OutputTypeID {
  NoOutput = 0, // NOTE: DO NOT CHANGE NAME OR VALUE!
  FlaskLED,     // These are the standard, in-column LEDs (all chains)
  ReflexLEDA,   // These are the reflex, below-column LEDs (all chains)
  ReflexLEDB,   // These are the reflex, below-column LEDs (all chains)
  RingLEDA,     // These are the special in-column LEDs (only short chains)
  RingLEDB,     // These are the special in-column LEDs (only short chains)
  EndOfOutputs  // NOTE: DO NOT CHANGE NAME!
};

// This value stores the number of unique OutputTypes.
// NOTE: DO NOT CHANGE NAME OR VALUE!
const byte numOutputTypes = EndOfOutputs - 1;
 
// This array is based on the order of OutputTypeID above.
// This array is indexed using [OutputTypeID - 1].
// NOTE: DO NOT CHANGE NAME!
PROGMEM const OutputType allOutputTypes[numOutputTypes] = {
//         ID, holdLevel, riseFor, holdFor, fallFor, sensorStartTime, sensorOffsetTime
  {  FlaskLED,       255,     600,     500,     400,             500,              500},
  {ReflexLEDA,       255,     600,       0,     400,             250,                0},
  {ReflexLEDB,       255,     600,       0,     400,               0,                0},
  {  RingLEDA,       255,     600,    1500,     400,            1000,                0},
  {  RingLEDB,       255,     600,    1500,     400,             500,                0},
};

// All PWM envelopes are built using the holdLevel and various xxxxFor times (in ms).
// The holdLevel variable dictates the peak power of the OutputType, in a range from 0 to 255.
// The maximum value of the holdLevel is capped at 255 (0xFF), and the minimum useful
// value is 1. If holdLevel is 0 for any OutputType, the outputs will never turn on.
//
// The output will take the following steps (in order) when its starting time is reached:
// - Linearly rise from off to the holdLevel over riseFor milliseconds,
// - Stay at the holdLevel for holdFor milliseconds,
// - Linearly fall from the holdLevel to off over fallFor milliseconds.
//
// Some special conditions can change the shape of this envelope:
// - If riseFor is 0, the output will instantly go to holdLevel when turned on.
// - If holdFor is 0, the output will not plateau, and will immediately move falling.
// - If fallFor is 0, the output will instantly turn off at the end of the holding step.
//
// Using these variables and special conditions, a number of envelopes can be created:
// - Trapezoidal: set riseFor, holdFor, and fallFor as desired to dictate slopes and plateau
// - Triangular: set holdFor to 0, and set riseFor and fallFor as desired to dictate slopes
// - Rectangular: set riseFor and fallFor to 0, and set holdFor to the full desired on time

// This will be the only OutputTypeID which can do background behaviour
const byte backgroundOutputTypeID = FlaskLED;

// **********************************************************************************
// Definitions of RegisterTypes.
// NOTE: If this enumeration is changed, allRegistertTypes must be changed to suit.
// NOTE: DO NOT CHANGE NAME!
enum RegisterTypeID {
  NoRegister = 0, // NOTE: DO NOT CHANGE NAME OR VALUE!
  AllFlasks,      // This is a standard register with 8 FlaskLEDs
  AllReflex,      // This is a sensor register with 8 ReflexLEDs
  ShortTop,       // This is the special register at the top of a ShortColumn
  ShortBottom,    // This is the special register at the bottom of a ShortColumn
  EndOfRegisters  // NOTE: DO NOT CHANGE NAME!
};

// This value stores the number of unique RegisterTypes.
// NOTE: DO NOT CHANGE NAME OR VALUE!
const byte numRegisterTypes = EndOfRegisters - 1;

// **********************************************************************************
// Assignments of output types to specific outputs for each register type.

// There must be one of these arrays for each RegisterType.
// The values must each be an OutputType.
PROGMEM const byte allFlasksOutputTypeIDs[maxOutputsPerRegister] = {
   FlaskLED,  FlaskLED,  FlaskLED,  FlaskLED,  FlaskLED,  FlaskLED,  FlaskLED,  FlaskLED,
};
PROGMEM const byte allReflexOutputTypeIDs[maxOutputsPerRegister] = {
  ReflexLEDA, ReflexLEDA, ReflexLEDA, ReflexLEDA, ReflexLEDB, ReflexLEDB, ReflexLEDB, ReflexLEDB,
};
PROGMEM const byte shortTopOutputTypeIDs[maxOutputsPerRegister] = {
    RingLEDA,   RingLEDA,   RingLEDA,   RingLEDA,   RingLEDA,   RingLEDA,  FlaskLED,  FlaskLED,
};
PROGMEM const byte shortBottomOutputTypeIDs[maxOutputsPerRegister] = {
   FlaskLED,  FlaskLED,   RingLEDB,   RingLEDB,   RingLEDB,   RingLEDB,   RingLEDB,   RingLEDB,
};

// This array is based on the order of RegisterTypeID above.
// This array is indexed using [RegisterTypeID - 1].
// NOTE: DO NOT CHANGE NAME!
PROGMEM const RegisterType allRegisterTypes[numRegisterTypes] = {
//          ID,            outputTypeIDs
  {  AllFlasks,   allFlasksOutputTypeIDs},
  {  AllReflex,   allReflexOutputTypeIDs},
  {   ShortTop,    shortTopOutputTypeIDs},
  {ShortBottom, shortBottomOutputTypeIDs},
};

// **********************************************************************************
// Definitions of InputTypes.
// NOTE: If this enumeration is changed, allInputTypes must be changed to suit.
// NOTE: DO NOT CHANGE NAME!
enum InputTypeID {
  NoInput = 0,    // NOTE: DO NOT CHANGE NAME OR VALUE!
  SharpProximity, // Sharp IR Distance Sensors
  EndOfInputs     // NOTE: DO NOT CHANGE NAME!
};

// This value stores the number of unique InputTypes.
// NOTE: DO NOT CHANGE NAME OR VALUE!
const byte numInputTypes = EndOfInputs - 1;

// This array is based on the order of InputTypeID above.
// This array is indexed using [InputTypeID - 1].
// NOTE: DO NOT CHANGE NAME!
PROGMEM const InputType allInputTypes[numInputTypes] = {
//             ID, activeHigh
  {SharpProximity,       true},
};

// If activeHigh is true, the InputType will activate when above the threshold.
// If activeHigh is false, then the InputType activates below the threshold.

// NOTE: In some installations, there are different types of SHARP sensors used.
// However, the only difference between these sensor types is the thresholds,
// and even within a given type the thresholds can be individually-specified
// on a per-node basis through the sensorThresholds array.
// For this reason, there is only one SharpProximity InputType defined.

// **********************************************************************************
// Definitions of ChainTypes.
// NOTE: If this enumeration is changed, allChainTypes must be changed to suit.
// NOTE: DO NOT CHANGE NAME!
enum ChainTypeID {
  NoChain = 0,   // NOTE: DO NOT CHANGE NAME OR VALUE!
  StandardColumn, // Standard chain
  LongColumn,     // Long chain
  ShortColumn,    // Short chain
  EndOfChains    // NOTE: DO NOT CHANGE NAME!
};

// This value stores the number of unique ChainTypes.
// NOTE: DO NOT CHANGE NAME OR VALUE!
const byte numChainTypes = EndOfChains - 1;

// **********************************************************************************
// Assignments of register types to specific registers for each chain type.

// There must be one of these arrays for each ChainType.
// The values must each be a RegisterType.
PROGMEM const byte standardColumnRegisterTypeIDs[maxRegistersPerChain] = {
  AllFlasks, AllFlasks, AllReflex,
};
PROGMEM const byte longColumnRegisterTypeIDs[maxRegistersPerChain] = {
  AllFlasks, AllFlasks, AllFlasks, AllReflex,
};
PROGMEM const byte shortColumnRegisterTypeIDs[maxRegistersPerChain] = {
  ShortTop, AllFlasks, ShortBottom, AllReflex,
};

// This array is based on the order of ChainTypeID above.
// This array is indexed using [ChainTypeID - 1].
// NOTE: DO NOT CHANGE NAME!
PROGMEM const ChainType allChainTypes[numChainTypes] = {
//             ID,  doOverlay,      inputType,               registerTypeIDs
  {StandardColumn,   0xFF0000, SharpProximity, standardColumnRegisterTypeIDs},
  {    LongColumn, 0xFF000000, SharpProximity,     longColumnRegisterTypeIDs},
  {   ShortColumn, 0xFFFC003F, SharpProximity,    shortColumnRegisterTypeIDs},
};

// The doOverlay variable is defined in hex format, where each character controls 4 outputs.
// In hex format, the letters A to F are used to represent the values 10 to 15.
// The binary format of each of these values uses 4 bits, each of which controls overlay for an output.
// If doOverlay is 0x00000000, then the ChainType does not do overlays.
// If doOverlay is 0xFFFFFFFF, then the ChainType does overlays for all outputs.
// If doOverlay is another value, then the ChainType does overlays on specific outputs.
// Which specific outputs have overlays enabled is dictated by which bits are set.
// Any bit in doOverlay which is set will enable overlays for that output, with the
// lowest bit (right) controlling output 1 and the highest bit (left) controlling output N (of N).
// For example, if doOverlay is 0x00300C01, then overlays are enabled on outputs 1, 11, 12, 21 and 22.

// **********************************************************************************
// Assignments of chain types to specific control channels for each node type.

// There must be one of these arrays for each slave NodeType.
// The values must each be a ChainTypeID.
PROGMEM const byte nodeOneChainTypeIDs[maxChainsPerNode] = {
  ShortColumn, StandardColumn, LongColumn, StandardColumn, StandardColumn, StandardColumn
};
PROGMEM const byte nodeTwoChainTypeIDs[maxChainsPerNode] = {
  ShortColumn, StandardColumn, LongColumn, StandardColumn, LongColumn, StandardColumn
};
PROGMEM const byte nodeThreeChainTypeIDs[maxChainsPerNode] = {
  StandardColumn, StandardColumn, ShortColumn, StandardColumn, StandardColumn, NoChain
};
PROGMEM const byte nodeFourChainTypeIDs[maxChainsPerNode] = {
  LongColumn, ShortColumn, StandardColumn, LongColumn, StandardColumn, StandardColumn
};
PROGMEM const byte nodeFiveChainTypeIDs[maxChainsPerNode] = {
  LongColumn, LongColumn, ShortColumn, StandardColumn, StandardColumn, NoChain
};
PROGMEM const byte nodeSixChainTypeIDs[maxChainsPerNode] = {
  ShortColumn, LongColumn, StandardColumn, StandardColumn, StandardColumn, NoChain
};
PROGMEM const byte nodeSevenChainTypeIDs[maxChainsPerNode] = {
  ShortColumn, StandardColumn, StandardColumn, StandardColumn, NoChain, NoChain
};
PROGMEM const byte nodeEightChainTypeIDs[maxChainsPerNode] = {
  ShortColumn, StandardColumn, StandardColumn, ShortColumn, NoChain, NoChain
};

// This array is based on the order of NodeTypeID above.
// This array is indexed using [NodeTypeID - 1].
// NOTE: DO NOT CHANGE NAME!
PROGMEM const NodeType allNodeTypes[numNodeTypes] = {
//        ID, doNodeReactor, defaultTimeout, defaultTimeDelta,          chainTypeIDs
  {  NodeOne,         false,          40000,            10000,   nodeOneChainTypeIDs},
  {  NodeTwo,         false,          40000,            10000,   nodeTwoChainTypeIDs},
  {NodeThree,         false,          40000,            10000, nodeThreeChainTypeIDs},
  { NodeFour,         false,          40000,            10000,  nodeFourChainTypeIDs},
  { NodeFive,         false,          40000,            10000,  nodeFiveChainTypeIDs},
  {  NodeSix,         false,          40000,            10000,   nodeSixChainTypeIDs},
  {NodeSeven,         false,          40000,            10000, nodeSevenChainTypeIDs},
  {NodeEight,         false,          40000,            10000, nodeEightChainTypeIDs},
};

// The neighbourDelay value sets how many ms of delay there are between neighbourhood layers.
// The defaultTimeout value sets how many ms of idle time there is before default behaviour.
// The defaultTimeDelta value sets how many ms of randomness can affect the defaultTimeout.
// The two default* values define how frequently and with how much variation in period
// each node type performs its default behaviour, respectively.
// If no default behaviour is desired for a given node type, use 0 for both default* values.

// **********************************************************************************
// Definitions for sensor thresholds, based on address
//
// For now, this is hardcoded into an array of set length.
// This is really clumsy but it seems to work for now.
//
// The array is prepopulated with a default value defined below ahead of the array.
// Individual nodes can have their thresholds set between 0 and 1024.
//
// The noise level is typically around 100 to 200, so setting the threshold
// below that for below-threshold sensors like the QProx, or above 800 or 900
// in the case of above-threshold sensors like the Sharps, will act up due to noise.
//
// Whether an input type triggers on values higher or lower than the 
// threshold is deftermined by the activeHigh boolean flag.

// NOTE: DO NOT CHANGE NAMES IN THIS SECTION!
#define SHARP 100

PROGMEM const int sensorThreshold[maxSlaveAddress + 1] = {
    0, SHARP, SHARP, SHARP, SHARP, SHARP, SHARP, SHARP, SHARP, // addresses 0 to 8
};

// **********************************************************************************
// Definitions for neighbour behaviour

// This variable determines how much dimmer one chain is from the next.
// This value can be anywhere from 0 (no dimming) to 255 (no neighbour behaviour).
// A good range is from 10 to 50, leading to a dimming factor of 4-20% per chain.
// NOTE: DO NOT CHANGE NAME!
const byte neighbourDimmingFactor = 64;

// This variable determines how many fewer outputs are used from one chain to the next.
// This value can be anywhere from 0 (all outputs used) to 255 (no neighbour behaviour).
// A good range is from 10 to 50, leading to an output reduction of 4-20% per chain.
// NOTE: DO NOT CHANGE NAME!
const byte neighbourOutputFactor = 32;

// This variable determines how long to delay neighbour behaviour between one chain and the next.
// The value is in ms, and can range from 0 (all neighbours at once) to 65535 (65.5s delay).
// A good range of values are from 100 to 500, or 1/10 of a second up to 1/2 a second delay.
// A delay of 0 (or close to 0) can potentially look bad, as the start of behaviour on
//   neighbouring chains that are operated by other nodes may experience a comm delay.
// NOTE: DO NOT CHANGE NAME!
const unsigned int neighbourDelayStep = 500;

// This array of values determines the offset time between outputs during neighbour behaviour.
// It is indexed by the ChainTypeID of the chain to which the output belongs.
// NOTE: DO NOT CHANGE NAME!
const unsigned int neighbourOffsetTimes[numChainTypes + 1] = {0, // Leave this 0 here!
  500, 500, 500
};

// Neighbourhood definitions
const byte curlOne[] = {
  1, 2, 3,
};
const byte curlTwo[] = {
  4, 5, 6,
};
const byte curlThree[] = {
  7, 8,
};
const byte numNeighbourhoods = 3;
const byte *allNeighbourhoods[numNeighbourhoods] = {
  curlOne,
  curlTwo,
  curlThree,
};
const byte neighbourhoodSize[numNeighbourhoods] = {
  sizeof(curlOne),
  sizeof(curlTwo),
  sizeof(curlThree),
};

// **********************************************************************************
// Definitions of Reactors, based on node address and chain.

// Constants to define how reactors and global  function
// NOTE: DO NOT CHANGE NAMES!
const byte reactorMultiplier = 4; // num nodes * this value = threshold
const int  reactorTimeout = 3000; // A value of 0 effectively disables reactors
const byte globalOverloadThreshold = 100;
const int  globalOverloadTimeout = 4000; // A value of 0 effectively disables overload

// Required for reactor definitions
// NOTE: DO NOT CHANGE THESE LINES!
#define C(a,b) (a + (b << 5))

// Reactor definitions, using node and chain addresses
const byte testReactor[] = {
  C(3,1), C(3,5), C(8,1)
};

const byte numReactors = 1;
const byte *reactorChains[numReactors] = {
  testReactor,
};
const byte reactorChainSize[numReactors] = {
  sizeof(testReactor),
};

