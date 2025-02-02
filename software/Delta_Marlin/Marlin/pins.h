#ifndef PINS_H
#define PINS_H

#define X_MS1_PIN -1
#define X_MS2_PIN -1
#define Y_MS1_PIN -1
#define Y_MS2_PIN -1
#define Z_MS1_PIN -1
#define Z_MS2_PIN -1
#define E0_MS1_PIN -1
#define E0_MS2_PIN -1
#define E1_MS1_PIN -1
#define E1_MS2_PIN -1
#define DIGIPOTSS_PIN -1

//****************************************************************************************
// RAMPS 1.4
//****************************************************************************************
#if MOTHERBOARD == 14 
  #define KNOWN_BOARD 1

  #define KNOWN_BOARD 1
  
  #define LARGE_FLASH true
  
  #define X_STEP_PIN         54
  #define X_DIR_PIN          55
  #define X_ENABLE_PIN       38
  #define X_MIN_PIN           3
  #define X_MAX_PIN          -1 //PIN 2 is used
  
  #define Y_STEP_PIN         60
  #define Y_DIR_PIN          61
  #define Y_ENABLE_PIN       56
  #define Y_MIN_PIN          14
  #define Y_MAX_PIN          -1 //PIN 15 is used
  
  #define Z_STEP_PIN         46
  #define Z_DIR_PIN          48
  #define Z_ENABLE_PIN       62
  #define Z_MIN_PIN          18
  #define Z_MAX_PIN          -1 //PIN 19 is used
  
  //Modular Tool Common ***********
  #define TOOL_STEP_PIN      30
  #define TOOL_DIR_PIN       32
  #define E0_STEP_PIN        30
  #define E0_DIR_PIN         32
   
  //Modular Tool #1 *********** //Heated bed
  #define TOOL1_ENABLE_PIN   44
  #define TOOL1_DOUT_PIN     42 
  #define TOOL1_AIN_PIN      A12

  #define HEATER_BED_PIN     42 // Heated bed
  #define TEMP_BED_PIN       12 // A12 //ANALOG NUMBERING!!!
  
  //Modular Tool #2 *********** //3D printing extruder / hotend
  #define TOOL2_ENABLE_PIN   52
  #define TOOL2_DOUT_PIN     50
  #define TOOL2_AIN_PIN      A13

  //Modular Tool #3 *********** //SMT paste extruder
  #define TOOL3_ENABLE_PIN   24
  #define TOOL3_DOUT_PIN     22
  #define TOOL3_AIN_PIN      A15
  #define PASTE_PIN          22

  //Modular Tool #4 *********** //SMT nozzle
  #define TOOL4_ENABLE_PIN   34
  #define TOOL4_DOUT_PIN     26
  #define TOOL4_AIN_PIN      A14
  #define VACUUM_PIN         32 //// connected to the pump or soleinoid valve (activate <ith M4(pick) and M5(place))

  #define HEATER_1_PIN       46 //Spare / un-connected pin!  This is a hack until I've added the logic to handle non-hotend E axis... Right now this would error out if set to -1
  #define TEMP_1_PIN         14 // A14 // ANALOG NUMBERING!! Extruder 1

  //Modular Tool Mapping ***********
#ifdef FPD_3DPRINTING
  #define HEATER_0_PIN       TOOL2_DOUT_PIN // Extruder 3 
  #define TEMP_0_PIN         13 // A13 //ANALOG NUMBERING!!!
  #define E0_ENABLE_PIN      TOOL2_ENABLE_PIN
#else
  #define HEATER_0_PIN       48 //Spare / un-connected pin!  This is a hack until I've added the logic to handle non-hotend E axis... Right now this would error out if set to -1
  #define TEMP_0_PIN         13 // A13 //ANALOG NUMBERING!!!
  #define E0_ENABLE_PIN      TOOL4_ENABLE_PIN
#endif

  #define HEATER_2_PIN       -1
  #define TEMP_2_PIN         -1 // Extruder 2

  #define LED_PIN            13
  #define FAN_PIN            10 //PLA fan
  #define PS_ON_PIN          28 //ATX power supply with BSS138 MOSFET (Set HIGH to GROUND the pin)
  #define ESTOP_PIN          12

  //Lighting
  #define LEDRING_UP_PIN     4 // SERVO1
  #define LEDRING_DN_PIN     5 // SERVO2
  
  //#define SERVO0_PIN         4 //X [0]
  //#define SERVO1_PIN         5 //Y [1]
  //#define SERVO2_PIN         6 //Z [2]
  //#define SERVO3_PIN         7 //  [3]

#endif

//****************************************************************************************
// FirePick Delta EMC02 (Rev B) pin assignment (custom shield for Arduino MEGA 2560)
//****************************************************************************************
#if MOTHERBOARD == 642 //EMC02 Rev B
  #define KNOWN_BOARD 1
  
  #define LARGE_FLASH true
  
  //Delta Motor "X" ***********
  #define X_STEP_PIN         38  // X is the BACK motor!
  #define X_DIR_PIN          36
  #define X_ENABLE_PIN       40
  #define X_MIN_PIN          63 //A9
  #define X_MAX_PIN          -1
  
  //Delta Motor "Y" ***********
  #define Y_STEP_PIN         56 // Y is the FRONT LEFT motor! (NOTE: Analog pin A2 w/digital numbering)
  #define Y_DIR_PIN          57 // (NOTE: Analog pin A7 w/digital numbering)
  #define Y_ENABLE_PIN       54 // (NOTE: Analog pin A0 w/digital numbering)
  #define Y_MIN_PIN          64 // (NOTE: Analog pin A10 w/digital numbering)
  #define Y_MAX_PIN          -1
  
  //Delta Motor "Z" ***********
  #define Z_STEP_PIN         9 //Z is the FRONT RIGHT motor!
  #define Z_DIR_PIN          8
  #define Z_ENABLE_PIN       2
  #define Z_MIN_PIN          65 // (NOTE: Analog pin w/digital numbering)
  #define Z_MAX_PIN          -1
  
  //Modular Tool Common ***********
  #define TOOL_STEP_PIN      30
  #define TOOL_DIR_PIN       32
  #define E0_STEP_PIN        30
  #define E0_DIR_PIN         32
   
  //Modular Tool #1 *********** //Heated bed
  #define TOOL1_ENABLE_PIN   44
  #define TOOL1_DOUT_PIN     42 
  #define TOOL1_AIN_PIN      A12

  #define HEATER_BED_PIN     42 // Heated bed
  #define TEMP_BED_PIN       12 // A12 //ANALOG NUMBERING!!!
  
  //Modular Tool #2 *********** //3D printing extruder / hotend
  #define TOOL2_ENABLE_PIN   52
  #define TOOL2_DOUT_PIN     50
  #define TOOL2_AIN_PIN      A13

  //Modular Tool #3 *********** //SMT paste extruder
  #define TOOL3_ENABLE_PIN   24
  #define TOOL3_DOUT_PIN     22
  #define TOOL3_AIN_PIN      A15
  #define PASTE_PIN          22

  //Modular Tool #4 *********** //SMT nozzle
  #define TOOL4_ENABLE_PIN   34
  #define TOOL4_DOUT_PIN     26
  #define TOOL4_AIN_PIN      A14
  #define VACUUM_PIN         26 //

  #define HEATER_1_PIN       46 //Spare / un-connected pin!  This is a hack until I've added the logic to handle non-hotend E axis... Right now this would error out if set to -1
  #define TEMP_1_PIN         14 // A14 // ANALOG NUMBERING!! Extruder 1

  //Modular Tool Mapping ***********
#ifdef FPD_3DPRINTING
  #define HEATER_0_PIN       TOOL2_DOUT_PIN // Extruder 3 
  #define TEMP_0_PIN         13 // A13 //ANALOG NUMBERING!!!
  #define E0_ENABLE_PIN      TOOL2_ENABLE_PIN
#else
  #define HEATER_0_PIN       48 //Spare / un-connected pin!  This is a hack until I've added the logic to handle non-hotend E axis... Right now this would error out if set to -1
  #define TEMP_0_PIN         13 // A13 //ANALOG NUMBERING!!!
  #define E0_ENABLE_PIN      TOOL4_ENABLE_PIN
#endif

  #define HEATER_2_PIN       -1
  #define TEMP_2_PIN         -1 // Extruder 2

  #define LED_PIN            13
  #define FAN_PIN            10 //PLA fan
  #define PS_ON_PIN          28 //ATX power supply with BSS138 MOSFET (Set HIGH to GROUND the pin)
  #define ESTOP_PIN          12

  //Lighting
  #define LEDRING_UP_PIN     4 // SERVO1
  #define LEDRING_DN_PIN     5 // SERVO2
  
  //#define SERVO0_PIN         4 //X [0]
  //#define SERVO1_PIN         5 //Y [1]
  //#define SERVO2_PIN         6 //Z [2]
  //#define SERVO3_PIN         7 //  [3]

#endif //EMC02 pinout




//List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN,
#if EXTRUDERS > 1
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN,
#else
  #define _E1_PINS
#endif
#if EXTRUDERS > 2
  #define _E2_PINS E2_STEP_PIN, E2_DIR_PIN, E2_ENABLE_PIN, HEATER_2_PIN,
#else
  #define _E2_PINS
#endif

#ifdef X_STOP_PIN
  #if X_HOME_DIR < 0
    #define X_MIN_PIN X_STOP_PIN
    #define X_MAX_PIN -1
  #else
    #define X_MIN_PIN -1
    #define X_MAX_PIN X_STOP_PIN
  #endif
#endif

#ifdef Y_STOP_PIN
  #if Y_HOME_DIR < 0
    #define Y_MIN_PIN Y_STOP_PIN
    #define Y_MAX_PIN -1
  #else
    #define Y_MIN_PIN -1
    #define Y_MAX_PIN Y_STOP_PIN
  #endif
#endif

#ifdef Z_STOP_PIN
  #if Z_HOME_DIR < 0
    #define Z_MIN_PIN Z_STOP_PIN
    #define Z_MAX_PIN -1
  #else
    #define Z_MIN_PIN -1
    #define Z_MAX_PIN Z_STOP_PIN
  #endif
#endif

#ifdef DISABLE_MAX_ENDSTOPS
#define X_MAX_PIN          -1
#define Y_MAX_PIN          -1
#define Z_MAX_PIN          -1
#endif

#ifdef DISABLE_MIN_ENDSTOPS
#define X_MIN_PIN          -1
#define Y_MIN_PIN          -1
#define Z_MIN_PIN          -1
#endif

#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, PS_ON_PIN, \
                        HEATER_BED_PIN, FAN_PIN,                  \
                        _E0_PINS _E1_PINS _E2_PINS             \
                        analogInputToDigitalPin(TEMP_0_PIN), analogInputToDigitalPin(TEMP_1_PIN), analogInputToDigitalPin(TEMP_2_PIN), analogInputToDigitalPin(TEMP_BED_PIN) }
#endif
