 /*
  *  Config constants used in the Arduino ASPire sketch
  */
 
//Values are taken so the maestro output match the behavior of the radio controller in the motor controllers 
const int RUDDER_MAESTRO_MAX_TARGET = 1900;
const int RUDDER_MAESTRO_MIN_TARGET = 1150;
const int WINGSAIL_MAESTRO_MAX_TARGET = 1950;
const int WINGSAIL_MAESTRO_MIN_TARGET = 1110;

// A signal of e.g. 1200 matches the target 4800 in the
// Maestro Configurations
const int MAESTRO_SIGNAL_MULTIPLIER = 4;

// Rudder should go from -30 to +30 degrees
// which gives an effective range of 60.
const int MAX_RUDDER_ANGLE = 30;

//Windsail should go from -13 to 13 degrees
//range is 26
const int MAX_WINGSAIL_ANGLE = 13;

const double INT16_SIZE = 65535;

const int RUDDER_MIN_FEEDBACK = 278;
const int RUDDER_MAX_FEEDBACK = 358;
const int WINGSAIL_MIN_FEEDBACK =  360;
const int WINGSAIL_MAX_FEEDBACK = 980;

const int RUDDER_MAESTRO_CHANNEL = 0;
const int WINGSAIL_MAESTRO_CHANNEL = 2;

const int RUDDER_FEEDBACK_PIN = A6;
const int WINGSAIL_FEEDBACK_PIN = A4;
const int RADIO_CONTROLL_OFF_PIN = A8;

/////////////////// TODO set analog pins! ///////////////////
// Voltage and Current sensors constants
const int SAMPLES_N = 10;
const int V_REF = 5; // in V
const int ACS712_MV_PER_AMP = 66; // use 100 for 20A Module and 66 for 30A Module

const int SAILDRIVE_VOLTAGE_PIN = A0;
const int SAILDRIVE_CURRENT_PIN = A1;
const int ACTUATOR_UNIT_VOLTAGE_PIN = A2;
const int ACTUATOR_UNIT_CURRRENT_PIN = A3;
const int WINDVANE_SWITCH_VOLTAGE_PIN = A14;
const int WINDVANE_ANGLE_VOLTAGE_PIN = A15;

