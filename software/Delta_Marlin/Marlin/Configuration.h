#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// User-specified version info of this build to display in [Pronterface, etc] terminal window during
// startup. Implementation of an idea by Prof Braino to inform user that any changes made to this
// build by the user have been successfully uploaded into firmware.
#define STRING_VERSION_CONFIG_H __DATE__ " " __TIME__ // build date and time
#define STRING_CONFIG_H_AUTHOR "FirePick Delta" // Who made the changes.

// SERIAL_PORT selects which serial port should be used for communication with the host.
// This allows the connection of wireless adapters (for instance) to non-default port pins.
// Serial port 0 is still used by the Arduino bootloader regardless of this setting.
#define SERIAL_PORT 0

// This determines the communication speed of the printer
#define BAUDRATE 250000

//#define MOTHERBOARD 642 //EMC02 rev B
#define MOTHERBOARD 14 //RAMPS 1.4

// Define this to set a custom name for your generic Mendel,
 #define CUSTOM_MENDEL_NAME "FirePick Delta"

// Define this to set a unique identifier for this printer, (Used by some programs to differentiate between machines)
// You can use an online service to generate a random UUID. (eg http://www.uuidgenerator.net/version4)
 #define MACHINE_UUID "22b35943-6c03-4683-ad3d-baddf20d9120"

// This defines the number of extruders
#define EXTRUDERS 1

#define POWER_SUPPLY 2 //ATX (with inverted logic)

// Define this to have the electronics keep the power supply off on startup. If you don't know what this is leave it.
// #define PS_DEFAULT_OFF

//#define FPD_3DPRINTING

#define BARICUDA //for the valve 


//===========================================================================
//=============================Thermal Settings  ============================
//===========================================================================
//
//--NORMAL IS 4.7kohm PULLUP!-- 1kohm pullup can be used on hotend sensor, using correct resistor and table
//
//// Temperature sensor settings:
// -2 is thermocouple with MAX6675 (only for sensor 0)
// -1 is thermocouple with AD595
// 0 is not used
// 1 is 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
// 2 is 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
// 3 is Mendel-parts thermistor (4.7k pullup)
// 4 is 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
// 5 is 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
// 6 is 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
// 7 is 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
// 71 is 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
// 8 is 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
// 9 is 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
// 10 is 100k RS thermistor 198-961 (4.7k pullup)
// 20 is the PT100 circuit found in the Ultimainboard V2.x
// 50 is the NTC 100K Thermistor (http://www.robotdigg.com/product/76/100K-Glass-sealed-Thermistor)
// 60 is 100k Maker's Tool Works Kapton Bed Thermistor
//
//    1k ohm pullup tables - This is not normal, you would have to have changed out your 4.7k for 1k
//                          (but gives greater accuracy and more stable PID)
// 51 is 100k thermistor - EPCOS (1k pullup)
// 52 is 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
// 55 is 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)
//
// 1047 is Pt1000 with 4k7 pullup
// 1010 is Pt1000 with 1k pullup (non standard)
// 147 is Pt100 with 4k7 pullup
// 110 is Pt100 with 1k pullup (non standard)

#ifdef FPD_3DPRINTING
#define TEMP_SENSOR_0 5
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_BED 50
#else
#define TEMP_SENSOR_0 0
#define TEMP_SENSOR_1 0
#define TEMP_SENSOR_2 0
#define TEMP_SENSOR_BED 0
#endif

// This makes temp sensor 1 a redundant sensor for sensor 0. If the temperatures difference between these sensors is to high the print will be aborted.
//#define TEMP_SENSOR_1_AS_REDUNDANT
#define MAX_REDUNDANT_TEMP_SENSOR_DIFF 10

// Actual temperature must be close to target for this long before M109 returns success
#define TEMP_RESIDENCY_TIME 10  // (seconds)
#define TEMP_HYSTERESIS 3       // (degC) range of +/- temperatures considered "close" to the target one
#define TEMP_WINDOW     1       // (degC) Window around target to start the residency timer x degC early.

// The minimal temperature defines the temperature below which the heater will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define HEATER_0_MINTEMP 5
#define HEATER_1_MINTEMP 5
#define HEATER_2_MINTEMP 5
#define BED_MINTEMP 5

// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define HEATER_0_MAXTEMP 275
#define HEATER_1_MAXTEMP 275
#define HEATER_2_MAXTEMP 275
#define BED_MAXTEMP 150

// If your bed has low resistance e.g. .6 ohm and throws the fuse you can duty cycle it to reduce the
// average current. The value should be an integer and the heat bed will be turned on for 1 interval of
// HEATER_BED_DUTY_CYCLE_DIVIDER intervals.
//#define HEATER_BED_DUTY_CYCLE_DIVIDER 4

// If you want the M105 heater power reported in watts, define the BED_WATTS, and (shared for all extruders) EXTRUDER_WATTS
//#define EXTRUDER_WATTS (12.0*12.0/6.7) //  P=I^2/R
//#define BED_WATTS (12.0*12.0/1.1)      // P=I^2/R

// PID settings:
// Comment the following line to disable PID and enable bang-bang.
#define PIDTEMP
#define BANG_MAX 255 // limits current to nozzle while in bang-bang mode; 255=full current
#define PID_MAX 255 // limits current to nozzle while PID is active (see PID_FUNCTIONAL_RANGE below); 255=full current
#ifdef PIDTEMP
  //#define PID_DEBUG // Sends debug data to the serial port.
  //#define PID_OPENLOOP 1 // Puts PID in open loop. M104/M140 sets the output power from 0 to PID_MAX
  #define PID_FUNCTIONAL_RANGE 10 // If the temperature difference between the target temperature and the actual temperature
                                  // is more then PID_FUNCTIONAL_RANGE then the PID will be shut off and the heater will be set to min/max.
  #define PID_INTEGRAL_DRIVE_MAX 255  //limit for the integral term
  #define K1 0.95 //smoothing factor within the PID
  #define PID_dT ((OVERSAMPLENR * 8.0)/(F_CPU / 64.0 / 256.0)) //sampling period of the temperature routine

// If you are using a pre-configured hotend then you can use one of the value sets by uncommenting it
// Ultimaker
    #define  DEFAULT_Kp 22.2
    #define  DEFAULT_Ki 1.08
    #define  DEFAULT_Kd 114

// MakerGear
//    #define  DEFAULT_Kp 7.0
//    #define  DEFAULT_Ki 0.1
//    #define  DEFAULT_Kd 12

// Mendel Parts V9 on 12V
//    #define  DEFAULT_Kp 63.0
//    #define  DEFAULT_Ki 2.25
//    #define  DEFAULT_Kd 440
#endif // PIDTEMP

// Bed Temperature Control
// Select PID or bang-bang with PIDTEMPBED. If bang-bang, BED_LIMIT_SWITCHING will enable hysteresis
//
// Uncomment this to enable PID on the bed. It uses the same frequency PWM as the extruder.
// If your PID_dT above is the default, and correct for your hardware/configuration, that means 7.689Hz,
// which is fine for driving a square wave into a resistive load and does not significantly impact you FET heating.
// This also works fine on a Fotek SSR-10DA Solid State Relay into a 250W heater.
// If your configuration is significantly different than this and you don't understand the issues involved, you probably
// shouldn't use bed PID until someone else verifies your hardware works.
// If this is enabled, find your own PID constants below.
//#define PIDTEMPBED
//
//#define BED_LIMIT_SWITCHING

// This sets the max power delivered to the bed, and replaces the HEATER_BED_DUTY_CYCLE_DIVIDER option.
// all forms of bed control obey this (PID, bang-bang, bang-bang with hysteresis)
// setting this to anything other than 255 enables a form of PWM to the bed just like HEATER_BED_DUTY_CYCLE_DIVIDER did,
// so you shouldn't use it unless you are OK with PWM on your bed.  (see the comment on enabling PIDTEMPBED)
#define MAX_BED_POWER 255 // limits duty cycle to bed; 255=full current

#ifdef PIDTEMPBED
//120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
//from FOPDT model - kp=.39 Tp=405 Tdead=66, Tc set to 79.2, aggressive factor of .15 (vs .1, 1, 10)
    #define  DEFAULT_bedKp 10.00
    #define  DEFAULT_bedKi .023
    #define  DEFAULT_bedKd 305.4

//120v 250W silicone heater into 4mm borosilicate (MendelMax 1.5+)
//from pidautotune
//    #define  DEFAULT_bedKp 97.1
//    #define  DEFAULT_bedKi 1.41
//    #define  DEFAULT_bedKd 1675.16

// FIND YOUR OWN: "M303 E-1 C8 S90" to run autotune on the bed at 90 degreesC for 8 cycles.
#endif // PIDTEMPBED



//this prevents dangerous Extruder moves, i.e. if the temperature is under the limit
//can be software-disabled for whatever purposes by
//#define PREVENT_DANGEROUS_EXTRUDE
//if PREVENT_DANGEROUS_EXTRUDE is on, you can still disable (uncomment) very long bits of extrusion separately.
#define PREVENT_LENGTHY_EXTRUDE

#define EXTRUDE_MINTEMP 170
#define EXTRUDE_MAXLENGTH (X_MAX_LENGTH+Y_MAX_LENGTH) //prevent extrusion of very large distances.

//===========================================================================
//============================== Delta Settings =============================
//===========================================================================
// Enable DELTA kinematics
#define DELTA

// Make delta curves from many straight lines (linear interpolation).
// This is a trade-off between visible corners (not enough segments)
// and processor overload (too many expensive sqrt calls).
#define DELTA_SEGMENTS_PER_SECOND 50

#define DELTA_E         155.885 // End effector length
#define DELTA_F         514.200 // Base length == to calculate
#define DELTA_RE        550.000 // Carbon rod length
#define DELTA_RF         280.000 // Servo horn length
//#define DELTA_Z_OFFSET  293.000 // Distance from delta 8mm rod/pulley to table/bed.

//NOTE: For OpenPnP, set the zero to be about 25mm above the bed...
#define DELTA_Z_OFFSET  530.000 // Distance from delta 8mm rod/pulley to table/bed.


#define DELTA_EE_OFFS    15.000 // Ball joint plane to bottom of end effector surface
//#define TOOL_OFFSET       0.000 // No offset
//#define TOOL_OFFSET      40.000 // Distance between end effector ball joint plane and tip of tool (Z probe)
#define TOOL_OFFSET      30.500 // Distance between end effector ball joint plane and tip of tool (PnP)
#define Z_CALC_OFFSET  ((DELTA_Z_OFFSET - TOOL_OFFSET - DELTA_EE_OFFS) * -1)

#define Z_HOME_ANGLE    -30 // This is the angle where the arms hit the endstop sensor
#define Z_HOME_OFFS    (((DELTA_Z_OFFSET - TOOL_OFFSET - DELTA_EE_OFFS) - 182.002) - 0.5)
                                // This is calculated from the above angle, after applying forward 
                                // kinematics, and adding the Z calc offset to it.


// Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
#define DELTA_PRINTABLE_RADIUS 500



//===========================================================================
//=============================Mechanical Settings===========================
//===========================================================================

// coarse Endstop Settings
//#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

#ifndef ENDSTOPPULLUPS
  // fine endstop settings: Individual pullups. will be ignored if ENDSTOPPULLUPS is defined
  // #define ENDSTOPPULLUP_XMAX
  // #define ENDSTOPPULLUP_YMAX
   #define ENDSTOPPULLUP_ZMAX //for Z touch probe
  // #define ENDSTOPPULLUP_XMIN
  // #define ENDSTOPPULLUP_YMIN
  // #define ENDSTOPPULLUP_ZMIN
#endif

#ifdef ENDSTOPPULLUPS
  #define ENDSTOPPULLUP_XMAX
  #define ENDSTOPPULLUP_YMAX
  #define ENDSTOPPULLUP_ZMAX
  #define ENDSTOPPULLUP_XMIN
  #define ENDSTOPPULLUP_YMIN
  #define ENDSTOPPULLUP_ZMIN
#endif

// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
const bool X_MIN_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
const bool Y_MIN_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
const bool Z_MIN_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
const bool X_MAX_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
const bool Y_MAX_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.
const bool Z_MAX_ENDSTOP_INVERTING = false; // set to true to invert the logic of the endstop.

//#define DISABLE_MAX_ENDSTOPS
//#define DISABLE_MIN_ENDSTOPS

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0 // For all extruders

// Disables axis when it's not being used.
#define DISABLE_X false
#define DISABLE_Y false
#define DISABLE_Z false
#define DISABLE_E false // For all extruders

#define INVERT_X_DIR true    // for Mendel set to false, for Orca set to true
#define INVERT_Y_DIR true    // for Mendel set to true, for Orca set to false
#define INVERT_Z_DIR true    // for Mendel set to false, for Orca set to true
#define INVERT_E0_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E1_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false
#define INVERT_E2_DIR false   // for direct drive extruder v9 set to true, for geared extruder set to false

// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

#define min_software_endstops false // If true, axis won't move to coordinates less than HOME_POS.
#define max_software_endstops false  // If true, axis won't move to coordinates greater than the defined lengths below.

// Travel limits after homing
#define X_MAX_POS DELTA_PRINTABLE_RADIUS
#define X_MIN_POS -DELTA_PRINTABLE_RADIUS
#define Y_MAX_POS DELTA_PRINTABLE_RADIUS
#define Y_MIN_POS -DELTA_PRINTABLE_RADIUS
#define Z_MAX_POS MANUAL_Z_HOME_POS
#define Z_MIN_POS 0

#define X_MAX_LENGTH (X_MAX_POS - X_MIN_POS)
#define Y_MAX_LENGTH (Y_MAX_POS - Y_MIN_POS)
#define Z_MAX_LENGTH (Z_MAX_POS - Z_MIN_POS)
//============================= Bed Auto Leveling ===========================

//#define ENABLE_AUTO_BED_LEVELING // Delete the comment to enable (remove // at the start of the line)

#ifdef ENABLE_AUTO_BED_LEVELING
    
      // these are the positions on the bed to do the probing
      //#define DELTA_PROBABLE_RADIUS (DELTA_PRINTABLE_RADIUS-10)
      #define DELTA_PROBABLE_RADIUS 50.0 //214mm / 2 -10
      #define LEFT_PROBE_BED_POSITION -DELTA_PROBABLE_RADIUS
      #define RIGHT_PROBE_BED_POSITION DELTA_PROBABLE_RADIUS
      #define BACK_PROBE_BED_POSITION DELTA_PROBABLE_RADIUS
      #define FRONT_PROBE_BED_POSITION -DELTA_PROBABLE_RADIUS
    
      // these are the offsets to the probe relative to the extruder tip (Hotend - Probe)
      #define X_PROBE_OFFSET_FROM_EXTRUDER -47.0 //FirePick Delta v1
      #define Y_PROBE_OFFSET_FROM_EXTRUDER  30.0 //FirePick Delta v1
      #define Z_PROBE_OFFSET_FROM_EXTRUDER  -0.0 //FirePick Delta v1
    
      #define Z_RAISE_BEFORE_HOMING 4       // (in mm) Raise Z before homing (G28) for Probe Clearance.
                                            // Be sure you have this distance over your Z_MAX_POS in case
    
      #define XY_TRAVEL_SPEED 8000         // X and Y axis travel speed between probes, in mm/min
    
      #define Z_RAISE_BEFORE_PROBING 100  //How much the extruder will be raised before traveling to the first probing point.
      #define Z_RAISE_BETWEEN_PROBINGS 10  //How much the extruder will be raised when traveling from between next probing points
    
    
      //If defined, the Probe servo will be turned on only during movement and then turned off to avoid jerk
      //The value is the delay to turn the servo off after powered on - depends on the servo speed; 300ms is good value, but you can try lower it.
      // You MUST HAVE the SERVO_ENDSTOPS defined to use here a value higher than zero otherwise your code will not compile.
    
      #define PROBE_SERVO_DEACTIVATION_DELAY 600
    
    
      //If you have enabled the Bed Auto Leveling and are using the same Z Probe for Z Homing,
      //it is highly recommended you let this Z_SAFE_HOMING enabled!!!
    
      #define Z_SAFE_HOMING   // This feature is meant to avoid Z homing with probe outside the bed area.
                              // When defined, it will:
                              // - Allow Z homing only after X and Y homing AND stepper drivers still enabled
                              // - If stepper drivers timeout, it will need X and Y homing again before Z homing
                              // - Position the probe in a defined XY point before Z Homing when homing all axis (G28)
                              // - Block Z homing only when the probe is outside bed area.
    
      #ifdef Z_SAFE_HOMING
    
        #define Z_SAFE_HOMING_X_POINT (X_MAX_LENGTH/2)    // X point for Z homing when homing all axis (G28)
        #define Z_SAFE_HOMING_Y_POINT (Y_MAX_LENGTH/2)    // Y point for Z homing when homing all axis (G28)
    
      #endif
    
      // with accurate bed leveling, the bed is sampled in a ACCURATE_BED_LEVELING_POINTSxACCURATE_BED_LEVELING_POINTS grid and least squares solution is calculated
      // Note: this feature occupies 10'206 byte
      #define ACCURATE_BED_LEVELING
    
      #ifdef ACCURATE_BED_LEVELING
        #define ACCURATE_BED_LEVELING_POINTS 9
        #define ACCURATE_BED_LEVELING_GRID_X ((RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (ACCURATE_BED_LEVELING_POINTS - 1))
        #define ACCURATE_BED_LEVELING_GRID_Y ((BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (ACCURATE_BED_LEVELING_POINTS - 1))
    
        // NONLINEAR_BED_LEVELING means: don't try to calculate linear coefficients but instead
        // compensate by interpolating between the nearest four Z probe values for each point.
        // Useful for deltabots where the print surface may appear like a bowl or dome shape.
        // Works best with ACCURATE_BED_LEVELING_POINTS 5 or higher.
        #define NONLINEAR_BED_LEVELING
      #endif

#endif //auto-bed leveling


// The position of the homing switches
#define MANUAL_HOME_POSITIONS  // If defined, MANUAL_*_HOME_POS below will be used
#define BED_CENTER_AT_0_0  // If defined, the center of the bed is at (X=0, Y=0)

//Manual homing switch locations:
// For deltabots this means top and center of the Cartesian print volume.
#define MANUAL_X_HOME_POS 0
#define MANUAL_Y_HOME_POS 0
#define MANUAL_Z_HOME_POS (Z_HOME_OFFS*0.5) //HALFT Top of the work area
//#define MANUAL_Z_HOME_POS Z_HOME_OFFS //Top of the work area

//// MOVEMENT SETTINGS
#define NUM_AXIS 4 // The axis order in all axis related arrays is X, Y, Z, E
#define HOMING_FEEDRATE {75*60, 75*60, 75*60, 0}  // set the homing speeds (mm/min)

// default settings

//#define DEGREES_PER_STEP 360.0/(STEPS_PER_REV/MICROSTEPPING)/PULLEY_REDUCTION
//#define XYZ_FULL_STEPS_PER_ROTATION 200.0
//#define XYZ_MICROSTEPS 16.0
//#define XYZ_STEPS (XYZ_FULL_STEPS_PER_ROTATION*XYZ_MICROSTEPS*PULLEY_REDUCTION)/360.0
//#define SMALL_PULLEY_TEETH 16.0
//#define BIG_PULLEY_TEETH 150.0
//#define PULLEY_REDUCTION BIG_PULLEY_TEETH/SMALL_PULLEY_TEETH

// FOR DM860I (manoah camporini edit)
#define PULSE_b_REV 51200
#define XYZ_STEPS (PULSE_b_REV)/360.0 

#ifdef FPD_3DPRINTING
  #define E_STEPS_FPD 903.0
  #define E_FEEDRATE_FPD 15
#else
  #define E_STEPS_FPD 8.88888
  #define E_FEEDRATE_FPD 10000
#endif

//For Delta configuration: Units are degrees! That is, steps per degree
#define DEFAULT_AXIS_STEPS_PER_UNIT   {XYZ_STEPS, XYZ_STEPS, XYZ_STEPS, E_STEPS_FPD}
#define DEFAULT_MAX_FEEDRATE          {5000, 5000, 5000, E_FEEDRATE_FPD}    // (mm/sec)
#define DEFAULT_MAX_ACCELERATION      {2500,2500,2500,100}    // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.

#define DEFAULT_ACCELERATION          10000    // X, Y, Z and E max acceleration in mm/s^2 for printing moves
#define DEFAULT_RETRACT_ACCELERATION  3000   // X, Y, Z and E max acceleration in mm/s^2 for retracts

// Offset of the extruders (uncomment if using more than one and relying on firmware to position when changing).
// The offset has to be X=0, Y=0 for the extruder 0 hotend (default extruder).
// For the other hotends it is their distance from the extruder 0 hotend.
// #define EXTRUDER_OFFSET_X {0.0, 20.00} // (in mm) for each extruder, offset of the hotend on the X axis
// #define EXTRUDER_OFFSET_Y {0.0, 5.00}  // (in mm) for each extruder, offset of the hotend on the Y axis

// The speed change that does not require acceleration (i.e. the software might assume it can be done instantaneously)
#define DEFAULT_XYJERK                1//20.0    // (mm/sec)
#define DEFAULT_ZJERK                 1//20.0    // (mm/sec)
#define DEFAULT_EJERK                 20.0    // (mm/sec)

//===========================================================================
//=============================Additional Features===========================
//===========================================================================

// EEPROM
// The microcontroller can store settings in the EEPROM, e.g. max velocity...
// M500 - stores parameters in EEPROM
// M501 - reads parameters from EEPROM (if you need reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings".  You still need to store them in EEPROM afterwards if you want to.
//define this to enable EEPROM support
//#define EEPROM_SETTINGS
//to disable EEPROM Serial responses and decrease program space by ~1700 byte: comment this out:
// please keep turned on if you can.
//#define EEPROM_CHITCHAT

// Preheat Constants
#define PLA_PREHEAT_HOTEND_TEMP 180
#define PLA_PREHEAT_HPB_TEMP 70
#define PLA_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255

#define ABS_PREHEAT_HOTEND_TEMP 240
#define ABS_PREHEAT_HPB_TEMP 100
#define ABS_PREHEAT_FAN_SPEED 255   // Insert Value between 0 and 255


// Increase the FAN pwm frequency. Removes the PWM noise but increases heating in the FET/Arduino
//#define FAST_PWM_FAN

// Temperature status LEDs that display the hotend and bet temperature.
// If all hotends and bed temperature and temperature setpoint are < 54C then the BLUE led is on.
// Otherwise the RED led is on. There is 1C hysteresis.
//#define TEMP_STAT_LEDS

// Use software PWM to drive the fan, as for the heaters. This uses a very low frequency
// which is not ass annoying as with the hardware PWM. On the other hand, if this frequency
// is too low, you should also increment SOFT_PWM_SCALE.
//#define FAN_SOFT_PWM

// Incrementing this by 1 will double the software PWM frequency,
// affecting heaters, and the fan if FAN_SOFT_PWM is enabled.
// However, control resolution will be halved for each increment;
// at zero value, there are 128 effective control positions.
#define SOFT_PWM_SCALE 0

// M240  Triggers a camera by emulating a Canon RC-1 Remote
// Data from: http://www.doc-diy.net/photo/rc-1_hacked/
// #define PHOTOGRAPH_PIN     23

// SF send wrong arc g-codes when using Arc Point as fillet procedure
//#define SF_ARC_FIX


/*********************************************************************\
* R/C SERVO support
* Sponsored by TrinityLabs, Reworked by codexmas
**********************************************************************/

// Number of servos
//
// If you select a configuration below, this will receive a default value and does not need to be set manually
// set it manually if you have more servos than extruders and wish to manually control some
// leaving it undefined or defining as 0 will disable the servo subsystem
// If unsure, leave commented / disabled
//
#define NUM_SERVOS 3 // Servo index starts with 0 for M280 command

// Servo Endstops
//
// This allows for servo actuated endstops, primary usage is for the Z Axis to eliminate calibration or bed height changes.
// Use M206 command to correct for switch height offset to actual nozzle height. Store that setting with M500.
//
#define SERVO_ENDSTOPS {-1, -1, 0} // Servo index for X, Y, Z. Disable with -1
#define SERVO_ENDSTOP_ANGLES {0,0, 0,0, 174,90} // X,Y,Z Axis Extend and Retract angles

#include "Configuration_adv.h"
#include "thermistortables.h"

#endif //__CONFIGURATION_H
