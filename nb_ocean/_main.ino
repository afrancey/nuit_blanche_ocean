// Setting this true will enable extensive debug statements
const boolean verbose = false;

// Setting this true will enable comm
const boolean commEnabled = true;
//const boolean commEnabled = false; // handy for initial debug

// Setting this true will enable sensors
const boolean sensorsEnabled = false;
//const boolean sensorsEnabled = false; // handy for initial debug

// Setting this true to run the PWM tests
const boolean runPWMTests = false;

// Setting this true will mean all slaves send debug over comm
const boolean usingTestBed = false;
//const boolean usingTestBed = true; // only used in the test bed

// Convenience functions for Flash based Serial debug
#define ENDLN() endLn()
#define NOVALUE 0x7FFFFFFF
#define PRINT(str, x...) print(F(str), ##x)
#define PRINTLN(str, x...) println(F(str), ##x)
#define MAJORERROR(error, type, str) majorError(error, type, F(str))
void print(const __FlashStringHelper* data);
void print(const __FlashStringHelper* data, const long value);
void println(const __FlashStringHelper* data);
void println(const __FlashStringHelper* data, const long value);

// Definitions for the ShiftMuxPWM library
const int ShiftMuxPWM_latchPin = LATCH;
const int ShiftMuxPWM_clockLoPin = CLK_LO;
const int ShiftMuxPWM_clockMdPin = CLK_MD;
const int ShiftMuxPWM_clockHiPin = CLK_HI;
const bool ShiftMuxPWM_invertOutputs = false;
#include <SPI.h> // Include SPI.h before including ShiftMuxPWM.h!
#include <ShiftMuxPWM.h> // Include ShiftMuxPWM.h after setting the pins!

// Common communication variables
byte commCount;
byte commLineLength = 50;
const byte NODATA =   -1;

// Common background behaviour variables
boolean defaultLevelHigh = false;
const byte highDefaultLevelRatio = 4;
enum BackgroundLevel {
  NONE = false,
  NORMAL,
  EVENING,
  DORMANT,
};

// Base level commands
const byte STATUS = B001; // Request for a status update (including sensor states and major errors)
const byte SENSOR = B010; // Command to fake a sensor activation (for testing mostly)
const byte REACTR = B011; // Command to perform reactor behaviour
const byte NGHBOR = B100; // Command to perform neighbour behaviour
const byte GLOBAL = B111; // Notification to expect a global subcommand
const byte BLINK  = B101; // Notification to expect a global subcommand

// Global subcommands
const byte GBASIC = B001; // Command to go into basic mode
const byte GNIGHT = B010; // Command to go into night mode
const byte GPARTY = B011; // Command to go into party mode
const byte GOVRLD = B100; // Command to perform a global overload behaviour
const byte GNODEF = B101; // Command to disable default (background) behaviour
const byte GLODEF = B110; // Command to set default behaviour to a low level of activity
const byte GHIDEF = B111; // Command to set default behaviour to a high level of activity

// Common variables
byte myAddress; // the address of this node, read from hardware switches during init
byte myMode = GBASIC; // the current mode of this node
static boolean isMaster; // whether or not this node is the master node
long time;

void setup() {
  initCommon();
  isMaster? initMaster(): initSlave();
  finishInit();
}

void loop() {

  //digitalWrite(DE, HIGH); // enable chip
  //Serial.println("hi"); // send msg
  //finishSending(); // wait until msg is sent
  //digitalWrite(DE, LOW); // disable chip
  isMaster? master(): slave();
  

}

