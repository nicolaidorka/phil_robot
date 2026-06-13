// def_phil.h -- octopi_firmware_v2 config for the PHIL arm robot.
//
// Phil is an articulated 5-bar arm (X,Y are ROTARY joints; Z is vertical), NOT
// a Cartesian lead-screw stage. The SCREW_PITCH / *_mm numbers below are
// therefore NOTIONAL -- Phil's python layer (phil/hardware/v2_mc.py) commands
// MOVETO in raw MICROSTEPS and never uses the firmware's mm<->step conversion
// for X/Y. They only need to be self-consistent enough to give safe ramps.
//
// SAFETY (first bring-up): currents START LOW and velocity/accel are reduced.
// Raise current toward the motor rating only after confirming motion + that the
// motors stay cool. Motors: Shinano Kenshi SST43D1125 (NEMA-17 frame).
// Board: Squid/octopi V4, Teensy 4.1, TMC4361A + TMC2660 (confirmed 2026-06-12).

// LED matrix
#define DOTSTAR_NUM_LEDS 128

// Joystick -- DISABLED for Phil (no joystick wired; avoids Serial5 handling).
static const bool ENABLE_JOYSTICK = false;
constexpr int joystickSensitivity = 75;

// Motorized stage
static const int FULLSTEPS_PER_REV_X = 200;
static const int FULLSTEPS_PER_REV_Y = 200;
static const int FULLSTEPS_PER_REV_Z = 200;
static const int FULLSTEPS_PER_REV_THETA = 200;

// Sense resistors for the TMC2660 current calc (.ino uses R_sense_xy / R_sense_z).
// Standard Squid/octopi V4 values (from def_octopi.h). VERIFY against this board's
// actual R_sense before flashing -- if different, the real motor current scales
// inversely (smaller R_sense => higher current).
static const float R_sense_xy = 0.22;
static const float R_sense_z = 0.43;

// Axis index -> TMC channel. These decide WHICH motor a MOVE_X/Y/Z command drives.
// Values copied from def_octopi.h (this same V4 board). NOTE x and y are swapped
// (x=1, y=0) per the octopi board layout. *** BRING-UP: first jog X a few steps and
// confirm the intended (left) arm moves; if the wrong arm moves, swap x/y here. ***
static const uint8_t x = 1;
static const uint8_t y = 0;
static const uint8_t z = 2;

// Limit-switch polarity flags (homing is not used on Phil at bring-up; values from def_octopi.h).
static const bool flip_limit_switch_x = true;
static const bool flip_limit_switch_y = false;

// Notional pitch (see header note). Kept = platereader so step math is sane.
float SCREW_PITCH_X_MM = 4;
float SCREW_PITCH_Y_MM = 4;
float SCREW_PITCH_Z_MM = 0.012*25.4;

// 256 microstepping: matches the motors' known-good behavior under the previous
// custom firmware AND gives the finest command grid (this is the whole point of
// the reflash -- the legacy firmware only accepted whole full-step commands,
// which moved the arm tip 3-5 mm per step).
int MICROSTEPPING_X = 256;
int MICROSTEPPING_Y = 256;
int MICROSTEPPING_Z = 256;

static const float HOMING_VELOCITY_X = 1;
static const float HOMING_VELOCITY_Y = 1;
static const float HOMING_VELOCITY_Z = 0.5;

long steps_per_mm_X = FULLSTEPS_PER_REV_X*MICROSTEPPING_X/SCREW_PITCH_X_MM;
long steps_per_mm_Y = FULLSTEPS_PER_REV_Y*MICROSTEPPING_Y/SCREW_PITCH_Y_MM;
long steps_per_mm_Z = FULLSTEPS_PER_REV_Z*MICROSTEPPING_Z/SCREW_PITCH_Z_MM;

// Reduced for safe first bring-up (raise later once motion is verified).
float MAX_VELOCITY_X_mm = 4;
float MAX_VELOCITY_Y_mm = 4;
float MAX_VELOCITY_Z_mm = 1;
float MAX_ACCELERATION_X_mm = 50;
float MAX_ACCELERATION_Y_mm = 50;
float MAX_ACCELERATION_Z_mm = 10;

// Generous soft limits (notional mm). Phil's real bounds are enforced in python.
static const long X_NEG_LIMIT_MM = -130;
static const long X_POS_LIMIT_MM = 130;
static const long Y_NEG_LIMIT_MM = -130;
static const long Y_POS_LIMIT_MM = 130;
static const long Z_NEG_LIMIT_MM = -20;
static const long Z_POS_LIMIT_MM = 20;

// Motor RMS current -- START LOW. SST43D1125 (NEMA-17) is rated well above this;
// the arm is light so low current is plenty. Raise only if it stalls/loses steps,
// watching motor temperature. (platereader used 1000 mA for smaller NEMA-11.)
float X_MOTOR_RMS_CURRENT_mA = 500;
float Y_MOTOR_RMS_CURRENT_mA = 500;
float Z_MOTOR_RMS_CURRENT_mA = 300;

float X_MOTOR_I_HOLD = 0.25;
float Y_MOTOR_I_HOLD = 0.25;
float Z_MOTOR_I_HOLD = 0.5;

// encoder -- Phil is open-loop (no encoders)
bool X_use_encoder = false;
bool Y_use_encoder = false;
bool Z_use_encoder = false;

// signs (resolve actual direction during bring-up; python can also invert)
int MOVEMENT_SIGN_X = 1;
int MOVEMENT_SIGN_Y = 1;
int MOVEMENT_SIGN_Z = 1;
int ENCODER_SIGN_X = 1;
int ENCODER_SIGN_Y = 1;
int ENCODER_SIGN_Z = 1;
int JOYSTICK_SIGN_X = 1;
int JOYSTICK_SIGN_Y = -1;
int JOYSTICK_SIGN_Z = 1;

// limit switch polarity (homing is NOT used on Phil during bring-up)
bool LIM_SWITCH_X_ACTIVE_LOW = false;
bool LIM_SWITCH_Y_ACTIVE_LOW = false;
bool LIM_SWITCH_Z_ACTIVE_LOW = false;

// offset velocity enable/disable
bool enable_offset_velocity = false;
