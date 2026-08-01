//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: SCHED — INav cooperative scheduler + SerialRx failsafe
//
//STM32 Port: Adds scheduler and 4-layer failsafe to BETA 1.3
//Target: STM32F4/F7/H7 (5 boards supported - see README.md)
 
//========================================================================================================================//

//CREDITS + SPECIAL THANKS
/*
Some elements inspired by:
http://www.brokking.net/ymfc-32_main.html

Madgwick filter function adapted from:
https://github.com/arduino-libraries/MadgwickAHRS

MPU9250 implementation based on MPU9250 library by:
brian.taylor@bolderflight.com
http://www.bolderflight.com

Thank you to:
RcGroups 'jihlein' - IMU implementation overhaul + SBUS implementation.
Everyone that sends me pictures and videos of your flying creations! -Nick

*/



//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//

//Uncomment only one receiver type
//STM32: Only serial RX supported (IBus/SBUS/CRSF via SerialRx library)
// #define USE_IBUS_RX  //Uncomment for IBus protocol
// #define USE_SBUS_RX  //Uncomment for SBUS protocol
// #define USE_CRSF_RX  //Uncomment for CRSF/ELRS protocol

// Auto-select default protocol when none defined manually above.
// CRSF for boards with an ELRS/ER6 receiver wired in (HIL-008, HIL-007, BEFH flight target);
// other boards default to SBUS.
#if !defined(USE_IBUS_RX) && !defined(USE_SBUS_RX) && !defined(USE_CRSF_RX)
  #if defined(ARDUINO_NUCLEO_G474RE) || \
      defined(ARDUINO_WEACT_G474CE) || \
      defined(ARDUINO_BEFH_BETAFPVG473)
    #define USE_CRSF_RX
  #else
    #define USE_SBUS_RX
  #endif
#endif

//Uncomment only one IMU
#define USE_ICM42688P
//#define USE_MPU9250_SPI

//Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //Default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //Default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G



//========================================================================================================================//



//REQUIRED LIBRARIES

#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication

//STM32 libraries
// BoardConfig: Auto-detect from Arduino board selection
#if defined(ARDUINO_BLACKPILL_F411CE)
  #include "targets/BLACKPILL_F411CE.h"  //BLACKPILL F411CE (MPU-9250 9-DOF)
#elif defined(ARDUINO_NUCLEO_F411RE)
  #include "targets/NUCLEO_F411RE_JHEF411.h"  //NUCLEO F411RE (ICM42688P 6-DOF)
#elif defined(ARDUINO_BKMN_NERO)
  #include "targets/BKMN-NERO.h"  //NERO F7 flight controller (ICM-20602 6-DOF)
#elif defined(ARDUINO_MATEK_H743VI)
  #include "targets/MTKS-MATEKH743.h"  //MATEK H743 flight controller (ICM42688P 6-DOF)
#elif defined(ARDUINO_OPEN_REVO)
  #include "targets/OPEN-REVO.h"  //OpenPilot Revolution F405 (MPU-6000 6-DOF)
#elif defined(ARDUINO_NUCLEO_G474RE)
  #include "targets/NUCLEO_G474RE_HIL008.h"  //NUCLEO G474RE HIL-008 (ICM-42688P, DSHOT600, CRSF)
#elif defined(ARDUINO_WEACT_G474CE)
  #include "targets/WEACT_G474_HIL007.h"  //WeAct G474 HIL-007 (BEFH G473 surrogate: DSHOT600, CRSF on USART1)
#elif defined(ARDUINO_BEFH_BETAFPVG473)
  #include "targets/BEFH-BETAFPVG473.h"  //BetaFPV G473 flight controller (DSHOT600, CRSF/ELRS)
#else
  #error "Unsupported board! Use BLACKPILL_F411CE, NUCLEO_F411RE, BKMN_NERO, MATEK_H743VI, OPEN_REVO, NUCLEO_G474RE, WEACT_G474CE, or BEFH_BETAFPVG473"
#endif
#include <IMU.h>           //IMU library for ICM42688P
#include <SerialRx.h>      //Serial RX library for IBus/SBUS
#include <PWMOutputBank.h> //TimerPWM for OneShot125 motor output
#include <BoardAlignment.h> //Chip+board rotation matrix (FLU vehicle frame)

//Scheduler (INav cooperative scheduler)
#include "task_list.h"     //Task enum - MUST be before scheduler.h
#include <scheduler.h>     //INav scheduler library

//Task configuration table (defined in tasks.ino)
extern cfTask_t cfTasks[TASK_COUNT];



//========================================================================================================================//



//Setup gyro and accel scale factors for raw-to-physical conversion
//Note: FSR (Full Scale Range) is now set via IMU library's SetGyroFSR_Ex/SetAccelFSR_Ex in IMUinit()

#if defined GYRO_250DPS
  #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
  #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
  #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
  #define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
  #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
  #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
  #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
  #define ACCEL_SCALE_FACTOR 2048.0
#endif



//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

//Radio failsafe values applied when rx.isSignalLost() detects signal loss. Recommended defaults:
unsigned long channel_1_fs = 1000; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1500; //elev
unsigned long channel_4_fs = 1500; //rudd
// CH5 polarity: CRSF builds use Betaflight convention (HIGH=armed, LOW=cut/safe);
// IBus/SBUS builds keep dRehmFlight convention (HIGH=cut/safe, LOW=arm-allowed).
// Failsafe value snaps to the "safe" end so TX loss disarms.
#if defined USE_CRSF_RX
unsigned long channel_5_fs = 1000; //arm switch, Betaflight: LOW = disarmed (safe)
#else
unsigned long channel_5_fs = 2000; //gear, greater than 1500 = throttle cut
#endif
unsigned long channel_6_fs = 2000; //aux1

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  //Madgwick filter parameter
float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0;        //Magnetometer LP filter parameter

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
float MagErrorX = 0.0;
float MagErrorY = 0.0; 
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY= 0.0;
float GyroErrorZ = 0.0;

//Board alignment parameters (Betaflight 3-2-1 Euler, FLU vehicle frame, degrees).
//Defaults 0/0/0 = FC mounted flat with arrow forward. Combined at setup() with
//the per-target chip-to-board alignment (BoardConfig::imu.alignment) into
//boardAlignMatrix, applied to gyro/accel/mag at the end of getIMUdata().
float board_align_yaw_degrees   = 0.0f;
float board_align_pitch_degrees = 0.0f;
float board_align_roll_degrees  = 0.0f;
BoardAlignment::Mat3f boardAlignMatrix;  //R_sensor_to_vehicle

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;     //Max yaw rate in deg/sec

float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.3;           //Yaw P-gain
float Ki_yaw = 0.05;          //Yaw I-gain
float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)



//========================================================================================================================//
//                                                     DECLARE PINS                                                       //
//========================================================================================================================//

//STM32: Motor configuration via MotorManager (BoardConfig motor array)
//PWM servo outputs (TODO: Define using available timer channels):
constexpr Pin servo1Pin = PB10;  // TIM2_CH3 (example)
constexpr Pin servo2Pin = NC_PIN;    // Not yet assigned
constexpr Pin servo3Pin = NC_PIN;    // Not yet assigned
constexpr Pin servo4Pin = NC_PIN;    // Not yet assigned
constexpr Pin servo5Pin = NC_PIN;    // Not yet assigned
constexpr Pin servo6Pin = NC_PIN;    // Not yet assigned
constexpr Pin servo7Pin = NC_PIN;    // Not yet assigned
//LED:
constexpr Pin ledPin = BoardConfig::status_leds.led1_pin;  // PC13



//========================================================================================================================//



//DECLARE GLOBAL VARIABLES

//General stuff
float dt;  //Loop delta time (calculated by scheduler via getTaskDeltaTime)
unsigned long current_time, prev_time;  //Used by calibration functions

//Radio communication:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

//Flight status
bool armedFly = false;

// Create SPI instance using BoardConfig (software CS control)
SPIClass spi_imu(BoardConfig::imu.spi.instance,
                 BoardConfig::imu.spi.mosi_pin,
                 BoardConfig::imu.spi.miso_pin,
                 BoardConfig::imu.spi.sclk_pin,
                 BoardConfig::imu.spi.get_ssel_pin());

// Create IMU instance
IMU imu;

//STM32: Motor outputs via MotorManager (OneShot125)
MotorManager motors;

//STM32: Servo outputs via ServoManager (50 Hz PWM)
ServoManager servos;

//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup() {
  Serial.begin(115200); //USB serial
  while (!Serial && millis() < 3000);

  Serial.println("dRehmFlight STM32 BETA 1.3");

  //Initialize all pins
  pinMode(ledPin, OUTPUT); //LED blinker
  //STM32: Motor pins initialized by MotorManager.Init()
  //TODO: Servos not yet implemented
  //servo1.attach(servo1Pin, 900, 2100);

  //Set built in LED to turn on to signal startup
  digitalWrite(ledPin, HIGH);

  delay(5);

  //Initialize radio communication
  radioSetup();

  //Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;

  //Initialize IMU communication
  IMUinit();

  delay(5);

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  //calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.

  //STM32: TODO - Servos not yet implemented
  //servo1.write(0);

  delay(5);

  //calibrateESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
  //Code will not proceed past here if this function is uncommented!

  //STM32: Initialize motors via MotorManager (auto-discovers timer banks from BoardConfig)
  motors.Init(BoardConfig::Motor::motors, BoardConfig::Motor::num_motors,
              BoardConfig::Motor::frequency_hz, BoardConfig::Motor::protocol);

  //STM32: Initialize servos via ServoManager (if present on this board)
  if (BoardConfig::Servo::num_servos > 0) {
    servos.Init(BoardConfig::Servo::servos, BoardConfig::Servo::num_servos, BoardConfig::Servo::frequency_hz);
    servos.SetAllServos(1500);  // Center all servos
  }

  //Motors are armed at the very END of setup() (see note there) so the DShot
  //frame stream runs unbroken into the scheduler — no silent no-frame gap.
  
  //Indicate entering main loop with 3 quick blinks
  setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)

  //If using 9-DOF IMU (MPU-9250/9255), uncomment for one-time magnetometer calibration (may need to repeat for new locations)
  //calibrateMagnetometer(); //Generates magnetometer error and scale factors to be pasted in user-specified variables section

  //Initialize INav scheduler
  if (!schedulerInit(cfTasks, TASK_COUNT)) {
    Serial.println("ERROR: Scheduler init failed");
    while (1);
  }

  //Enable all tasks
  setTaskEnabled(TASK_FLIGHT, true);
  setTaskEnabled(TASK_RC, true);
  setTaskEnabled(TASK_TELEMETRY, true);
  setTaskEnabled(TASK_BLINK, true);

  Serial.println("Scheduler initialized");

  //Arm motors LAST: stream zero-throttle frames right up to the scheduler handoff
  //so motor output is continuous (loop() -> TASK_FLIGHT -> commandMotors()). DShot
  //ESCs arm on a *continuous* frame stream; a silent no-frame gap is a PWM-era
  //holdover that can delay/disrupt arming (Betaflight streams every loop for all
  //protocols). m*_command_scaled = 0.0f -> OneShot125 min (125 us) / DSHOT600 0 (disarm).
  armMotors();
}



//========================================================================================================================//
//                                                       MAIN LOOP                                                        //
//========================================================================================================================//

void loop() {
  //Run the INav cooperative scheduler
  //All flight tasks are now managed by scheduler - see tasks.ino for task implementations
  scheduler();
}



//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
//========================================================================================================================//



void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands() 
   * in preparation to be sent to the motor ESCs and servos.
   * 
   *Relevant variables:
   *thro_des - direct thottle control
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command passthrough
   *channel_6_pwm - free auxillary channel, can be used to toggle things with an 'if' statement
   */
   
  //Quad mixing - EXAMPLE
  m1_command_scaled = thro_des - pitch_PID + roll_PID + yaw_PID; //Front Left
  m2_command_scaled = thro_des - pitch_PID - roll_PID - yaw_PID; //Front Right
  m3_command_scaled = thro_des + pitch_PID - roll_PID + yaw_PID; //Back Right
  m4_command_scaled = thro_des + pitch_PID + roll_PID - yaw_PID; //Back Left
  m5_command_scaled = 0;
  m6_command_scaled = 0;

  //0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
  s1_command_scaled = 0;
  s2_command_scaled = 0;
  s3_command_scaled = 0;
  s4_command_scaled = 0;
  s5_command_scaled = 0;
  s6_command_scaled = 0;
  s7_command_scaled = 0;
 
}

void armedStatus() {
  //DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.
#if defined USE_CRSF_RX
  // Betaflight convention: CH5 HIGH = armed-allowed.
  if ((channel_5_pwm > 1500) && (channel_1_pwm < 1050)) {
#else
  if ((channel_5_pwm < 1500) && (channel_1_pwm < 1050)) {
#endif
    armedFly = true;
  }
}

void IMUinit() {
  //DESCRIPTION: Initialize IMU
  /*
   * STM32: Uses IMU library with ApplyPreset() for validated config.
   * Supports ICM42688P, MPU-6000, MPU-9250 auto-detection.
   * Honors user-defined GYRO_SCALE and ACCEL_SCALE settings.
   */

  // Init + chip detection (Init() auto-detects chip type)
  if (imu.Init(spi_imu, BoardConfig::imu.spi.cs_pin, BoardConfig::imu.spi.freq_hz) != IMU::Result::OK) {
    Serial.println("IMU initialization failed");
    while(1) {}
  }

  // Apply BALANCED preset (4kHz gyro, 1kHz accel, validated filters)
  // This matches dRehmFlight's 2kHz loop with optimal filtering
  if (imu.ApplyPreset(IMU::Preset::BALANCED) != IMU::Result::OK) {
    Serial.println("IMU preset configuration failed");
    while(1) {}
  }

  // R_sensor_to_vehicle = R_board_to_vehicle * R_sensor_to_board, composed in the
  // library so the multiplication order lives in one unit-tested place.
  //   chip alignment: per-target chip mounting (compile-time, BoardConfig::imu.alignment).
  //   board_align_*:  per-airframe FC mounting (runtime, board_align_*_degrees globals).
  boardAlignMatrix = BoardAlignment::makeSensorToVehicleMatrix(
      BoardConfig::imu.alignment,
      board_align_yaw_degrees, board_align_pitch_degrees, board_align_roll_degrees);

  // Override FSR with user-defined values (like Teensy setFullScaleGyroRange/setFullScaleAccelRange)
  #if defined GYRO_250DPS
    imu.SetGyroFSR_Ex(GyroFSR::DPS_250);
  #elif defined GYRO_500DPS
    imu.SetGyroFSR_Ex(GyroFSR::DPS_500);
  #elif defined GYRO_1000DPS
    imu.SetGyroFSR_Ex(GyroFSR::DPS_1000);
  #elif defined GYRO_2000DPS
    imu.SetGyroFSR_Ex(GyroFSR::DPS_2000);
  #endif

  #if defined ACCEL_2G
    imu.SetAccelFSR_Ex(AccelFSR::G_2);
  #elif defined ACCEL_4G
    imu.SetAccelFSR_Ex(AccelFSR::G_4);
  #elif defined ACCEL_8G
    imu.SetAccelFSR_Ex(AccelFSR::G_8);
  #elif defined ACCEL_16G
    imu.SetAccelFSR_Ex(AccelFSR::G_16);
  #endif

  Serial.println("IMU initialized successfully");

  #if defined USE_MPU9250_SPI
    if (imu.InitMagnetometer() == IMU::Result::OK) {
      Serial.println("Magnetometer initialized (9-DOF mode)");
    }
  #endif
}

void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
  /*
   * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ.
   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
   * the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;

  #if defined USE_MPU9250_SPI
    imu.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
  #else
    imu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
  #endif

 //Accelerometer
  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  //Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  //LP filter gyro data
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

  //Magnetometer
  MagX = MgX/6.0; //uT
  MagY = MgY/6.0;
  MagZ = MgZ/6.0;
  //Correct the outputs with the calculated error values
  MagX = (MagX - MagErrorX)*MagScaleX;
  MagY = (MagY - MagErrorY)*MagScaleY;
  MagZ = (MagZ - MagErrorZ)*MagScaleZ;
  //LP filter magnetometer data
  MagX = (1.0 - B_mag)*MagX_prev + B_mag*MagX;
  MagY = (1.0 - B_mag)*MagY_prev + B_mag*MagY;
  MagZ = (1.0 - B_mag)*MagZ_prev + B_mag*MagZ;
  MagX_prev = MagX;
  MagY_prev = MagY;
  MagZ_prev = MagZ;

  //Apply chip-to-vehicle alignment in place. Inline 3x3*3x1 expand so the
  //scalar globals stay scalar. LP-filter state above stays in chip frame
  //(consistent across iterations); only the published values land in
  //the FLU vehicle frame for Madgwick + controllers downstream.
  {
    const auto& R = boardAlignMatrix.m;
    const float ax = R[0][0]*AccX + R[0][1]*AccY + R[0][2]*AccZ;
    const float ay = R[1][0]*AccX + R[1][1]*AccY + R[1][2]*AccZ;
    const float az = R[2][0]*AccX + R[2][1]*AccY + R[2][2]*AccZ;
    AccX = ax; AccY = ay; AccZ = az;

    const float gx = R[0][0]*GyroX + R[0][1]*GyroY + R[0][2]*GyroZ;
    const float gy = R[1][0]*GyroX + R[1][1]*GyroY + R[1][2]*GyroZ;
    const float gz = R[2][0]*GyroX + R[2][1]*GyroY + R[2][2]*GyroZ;
    GyroX = gx; GyroY = gy; GyroZ = gz;

    const float mx = R[0][0]*MagX + R[0][1]*MagY + R[0][2]*MagZ;
    const float my = R[1][0]*MagX + R[1][1]*MagY + R[1][2]*MagZ;
    const float mz = R[2][0]*MagX + R[2][1]*MagY + R[2][2]*MagZ;
    MagX = mx; MagY = my; MagZ = mz;
  }
}

void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY= 0.0;
  GyroErrorZ = 0.0;
  
  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    imu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    
    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");
  
  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}

void calibrateAttitude() {
  //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  //Assuming vehicle is powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
   * to boot. 
   */
  //Warm up IMU and madgwick filter in simulated main loop
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0; 
    getIMUdata();
    Madgwick(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, MagX, MagY, MagZ, dt);
    loopRate(2000); //do not exceed 2000Hz
  }
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
  /*
   * This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.
   * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
   * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
   * pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  //Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  }

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    //Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    //Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalize quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  
  //compute angles - NWU
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void getDesState() {
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
   */
  thro_des = (channel_1_pwm - 1000.0)/1000.0; //Between 0 and 1
  roll_des = (channel_2_pwm - 1500.0)/500.0; //Between -1 and 1
  pitch_des = (channel_3_pwm - 1500.0)/500.0; //Between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0)/500.0; //Between -1 and 1
  roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
  pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
  yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5
  
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

void controlANGLE() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */
  
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlANGLE2() {
  //DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
  /*
   * Gives better performance than controlANGLE() but requires much more tuning. Not reccommended for first-time setup.
   * See the documentation for tuning this controller.
   */
  //Outer loop - PID on angle
  float roll_des_ol, pitch_des_ol;
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll_ol = integral_roll_prev_ol + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll_ol = 0;
  }
  integral_roll_ol = constrain(integral_roll_ol, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (roll_IMU - roll_IMU_prev)/dt; 
  roll_des_ol = Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll_ol;// - Kd_roll_angle*derivative_roll;

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch_ol = 0;
  }
  integral_pitch_ol = constrain(integral_pitch_ol, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = (pitch_IMU - pitch_IMU_prev)/dt;
  pitch_des_ol = Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch_ol;// - Kd_pitch_angle*derivative_pitch;

  //Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  roll_des_ol = Kl*roll_des_ol;
  pitch_des_ol = Kl*pitch_des_ol;
  roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
  pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
  roll_des_ol = (1.0 - B_loop_roll)*roll_des_prev + B_loop_roll*roll_des_ol;
  pitch_des_ol = (1.0 - B_loop_pitch)*pitch_des_prev + B_loop_pitch*pitch_des_ol;

  //Inner loop - PID on rate
  //Roll
  error_roll = roll_des_ol - GyroX;
  integral_roll_il = integral_roll_prev_il + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll_il = 0;
  }
  integral_roll_il = constrain(integral_roll_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll_il + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch_il = 0;
  }
  integral_pitch_il = constrain(integral_pitch_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch_il + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range
  
  //Yaw
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range
  
  //Update roll variables
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  //Update pitch variables
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;

}

void controlRATE() {
  //DESCRIPTION: Computes control commands based on state error (rate)
  /*
   * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
   */
  //Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = GyroX;
  //Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = GyroY;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void scaleCommands() {
  //DESCRIPTION: Scale normalized servo commands to standard PWM range.
  //Motors take m*_command_scaled (0..1) directly via MotorManager::SetMotor — no per-protocol scaling here.
  //STM32: Scaled to 1000-2000µs for ServoManager (standard PWM servo protocol)
  s1_command_PWM = s1_command_scaled*1000 + 1000;
  s2_command_PWM = s2_command_scaled*1000 + 1000;
  s3_command_PWM = s3_command_scaled*1000 + 1000;
  s4_command_PWM = s4_command_scaled*1000 + 1000;
  s5_command_PWM = s5_command_scaled*1000 + 1000;
  s6_command_PWM = s6_command_scaled*1000 + 1000;
  s7_command_PWM = s7_command_scaled*1000 + 1000;
  //Constrain commands to servos within standard PWM bounds
  s1_command_PWM = constrain(s1_command_PWM, 1000, 2000);
  s2_command_PWM = constrain(s2_command_PWM, 1000, 2000);
  s3_command_PWM = constrain(s3_command_PWM, 1000, 2000);
  s4_command_PWM = constrain(s4_command_PWM, 1000, 2000);
  s5_command_PWM = constrain(s5_command_PWM, 1000, 2000);
  s6_command_PWM = constrain(s6_command_PWM, 1000, 2000);
  s7_command_PWM = constrain(s7_command_PWM, 1000, 2000);

}

void getCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  //STM32: Using SerialRx library adapter (IBus/SBUS only)

  // Update channels from SerialRx adapter
  updateRadioChannels();

  // Get channel values (already in PWM format from SerialRx).
  #if defined USE_CRSF_RX
    // CRSF targets Betaflight AETR (TX ch1=A, ch2=E, ch3=T, ch4=R); remap into dRehmFlight TAER variables.
    channel_1_pwm = getRadioPWM(3);  // throttle
    channel_2_pwm = getRadioPWM(1);  // roll
    channel_3_pwm = getRadioPWM(2);  // pitch
    channel_4_pwm = getRadioPWM(4);  // yaw
    channel_5_pwm = getRadioPWM(5);  // aux1 / throttle-cut switch
    channel_6_pwm = getRadioPWM(6);  // aux2
  #else
    channel_1_pwm = getRadioPWM(1);
    channel_2_pwm = getRadioPWM(2);
    channel_3_pwm = getRadioPWM(3);
    channel_4_pwm = getRadioPWM(4);
    channel_5_pwm = getRadioPWM(5);
    channel_6_pwm = getRadioPWM(6);
  #endif
  
  //Low-pass the critical commands and update previous values
  float b = 0.7; //Lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
  channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
  channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
  channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void commandMotors() {
  //DESCRIPTION: Send command to motors via MotorManager (protocol-aware).
  //STM32: Pass normalized 0..1 scaled commands; MotorManager converts per protocol
  //(OneShot125: min_us..max_us; DSHOT600: 48..2047 with 0 = disarm).
  int num_motors = BoardConfig::Motor::num_motors;

  if (num_motors > 0) motors.SetMotor(0, m1_command_scaled);  // Motor 1
  if (num_motors > 1) motors.SetMotor(1, m2_command_scaled);  // Motor 2
  if (num_motors > 2) motors.SetMotor(2, m3_command_scaled);  // Motor 3
  if (num_motors > 3) motors.SetMotor(3, m4_command_scaled);  // Motor 4
  if (num_motors > 4) motors.SetMotor(4, m5_command_scaled);  // Motor 5
  if (num_motors > 5) motors.SetMotor(5, m6_command_scaled);  // Motor 6 (BLACKPILL, NERO)
  motors.Update();  // No-op for OneShot125; pushes DShot frame for DSHOT600
  //Motor 7-8 would need m7/m8_command_scaled variables (not yet in sketch)
}

void commandServos() {
  //DESCRIPTION: Send pulses to servo pins, standard PWM protocol (50 Hz, 1000-2000µs)
  //STM32: Using ServoManager for standard servo PWM
  //Dynamically command all servos available on this board (defined in BoardConfig)
  int num_servos = BoardConfig::Servo::num_servos;

  if (num_servos > 0) servos.SetServo(0, s1_command_PWM);  // Servo 1
  if (num_servos > 1) servos.SetServo(1, s2_command_PWM);  // Servo 2
  if (num_servos > 2) servos.SetServo(2, s3_command_PWM);  // Servo 3 (BLACKPILL)
  if (num_servos > 3) servos.SetServo(3, s4_command_PWM);  // Servo 4
  if (num_servos > 4) servos.SetServo(4, s5_command_PWM);  // Servo 5
  if (num_servos > 5) servos.SetServo(5, s6_command_PWM);  // Servo 6
  if (num_servos > 6) servos.SetServo(6, s7_command_PWM);  // Servo 7
}

void armMotors() {
  //DESCRIPTION: Sends many command pulses to the motors, to be used to arm motors in the void setup()
  /*  
   *  Loops over the commandMotors() function 50 times with a delay in between, simulating how the commandMotors()
   *  function is used in the main loop. Ensures motors arm within the void setup() where there are some delays
   *  for other processes that sometimes prevent motors from arming.
   */
  for (int i = 0; i <= 50; i++) {
    commandMotors();
    delay(2);
  }
}

void calibrateESCs() {
  //DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
  /*  
   *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
   *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be
   *  uncommented when performing an ESC calibration.
   */
   while (true) {
      prev_time = current_time;      
      current_time = micros();      
      dt = (current_time - prev_time)/1000000.0;
    
      digitalWrite(ledPin, HIGH); //LED on to indicate we are not in main loop

      getCommands(); //Pulls current available radio commands
      //SerialRx failsafe: set defaults on signal loss
      if (radioSignalLost()) {
        channel_1_pwm = channel_1_fs;
        channel_2_pwm = channel_2_fs;
        channel_3_pwm = channel_3_fs;
        channel_4_pwm = channel_4_fs;
        channel_5_pwm = channel_5_fs;
        channel_6_pwm = channel_6_fs;
      }
      getDesState(); //Convert raw commands to normalized values based on saturated control limits
      getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
      Madgwick(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, MagX, MagY, MagZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
      getDesState(); //Convert raw commands to normalized values based on saturated control limits
      
      m1_command_scaled = thro_des;
      m2_command_scaled = thro_des;
      m3_command_scaled = thro_des;
      m4_command_scaled = thro_des;
      m5_command_scaled = thro_des;
      m6_command_scaled = thro_des;
      s1_command_scaled = thro_des;
      s2_command_scaled = thro_des;
      s3_command_scaled = thro_des;
      s4_command_scaled = thro_des;
      s5_command_scaled = thro_des;
      s6_command_scaled = thro_des;
      s7_command_scaled = thro_des;
      scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)
    
      //throttleCut(); //Directly sets motor commands to low based on state of ch5

      //STM32: Command servos (if present on this board)
      commandServos();
      commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol
      
      //printRadioData(); //Radio pwm values (expected: 1000 to 2000)
      
      loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
   }
}

float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time
  /*  
   *  Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency 
   *  and linearly interpolates that param variable between the maximum and minimum bounds. This function can be called in controlMixer()
   *  and high/low states can be determined by monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being 
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
   *  statements in order to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used
   *  to designate the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.
   *  
   */
  float diffParam = (param_max - param_min)/(fadeTime*loopFreq); //Difference to add or subtract from param for each loop iteration for desired fadeTime

  if (state == 1) { //Maximum param bound desired, increase param by diffParam for each loop iteration
    param = param + diffParam;
  }
  else if (state == 0) { //Minimum param bound desired, decrease param by diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain(param, param_min, param_max); //Constrain param within max bounds
  
  return param;
}

float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable from its current value to the desired value, up or down
  /*  
   *  Takes in a float variable to be modified, desired new position, upper value, lower value, fade time, and the loop frequency 
   *  and linearly fades that param variable up or down to the desired value. This function can be called in controlMixer()
   *  to fade up or down between flight modes monitored by an auxillary radio channel. For example, if channel_6_pwm is being 
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
   *  statements in order to fade controller gains, for example between the two dynamic configurations. 
   *  
   */
  if (param > param_des) { //Need to fade down to get to desired
    float diffParam = (param_upper - param_des)/(fadeTime_down*loopFreq);
    param = param - diffParam;
  }
  else if (param < param_des) { //Need to fade up to get to desired
    float diffParam = (param_des - param_lower)/(fadeTime_up*loopFreq);
    param = param + diffParam;
  }

  param = constrain(param, param_lower, param_upper); //Constrain param within max bounds
  
  return param;
}

void switchRollYaw(int reverseRoll, int reverseYaw) {
  //DESCRIPTION: Switches roll_des and yaw_des variables for tailsitter-type configurations
  /*
   * Takes in two integers (either 1 or -1) corresponding to the desired reversing of the roll axis and yaw axis, respectively.
   * Reversing of the roll or yaw axis may be needed when switching between the two for some dynamic configurations. Inputs of 1, 1 does not 
   * reverse either of them, while -1, 1 will reverse the output corresponding to the new roll axis. 
   * This function may be replaced in the future by a function that switches the IMU data instead (so that angle can also be estimated with the 
   * IMU tilted 90 degrees from default level).
   */
  float switch_holder;

  switch_holder = yaw_des;
  yaw_des = reverseYaw*roll_des;
  roll_des = reverseRoll*switch_holder;
}

void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
      Monitors the state of radio command channel_5_pwm and directly sets the mx_command_scaled values to 0.0 when the cut
      is active (OneShot125 then emits its 125 us minimum pulse; DShot emits the throttle-0 disarm token). This is the last function
      called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
      the motors to anything other than minimum value. Safety first.

      channel_5_pwm is LOW then throttle cut is OFF and throttle value can change. (ThrottleCut is DEACTIVATED)
      channel_5_pwm is HIGH then throttle cut is ON and motor output = 0.0 only. (ThrottleCut is ACTIVATED), (drone is DISARMED)
  */
#if defined USE_CRSF_RX
  // Betaflight convention: CH5 LOW = cut active.
  if ((channel_5_pwm < 1500) || (armedFly == false)) {
#else
  if ((channel_5_pwm > 1500) || (armedFly == false)) {
#endif
    armedFly = false;
    m1_command_scaled = 0.0f;
    m2_command_scaled = 0.0f;
    m3_command_scaled = 0.0f;
    m4_command_scaled = 0.0f;
    m5_command_scaled = 0.0f;
    m6_command_scaled = 0.0f;

    //Uncomment if using servo PWM variables to control motor ESCs
    //s1_command_PWM = 0;
    //s2_command_PWM = 0;
    //s3_command_PWM = 0;
    //s4_command_PWM = 0;
    //s5_command_PWM = 0;
    //s6_command_PWM = 0;
    //s7_command_PWM = 0;
  }
}

void calibrateMagnetometer() {
  #if defined USE_MPU9250_SPI
    float success;
    Serial.println("Beginning magnetometer calibration in");
    Serial.println("3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("Rotate the IMU about all axes until complete.");
    Serial.println(" ");
    success = imu.CalibrateMagnetometer() == IMU::Result::OK;
    if(success) {
      Serial.println("Calibration Successful!");
      Serial.println("Please comment out the calibrateMagnetometer() function and copy these values into the code:");
      float bias_x, bias_y, bias_z, scale_x, scale_y, scale_z;
      imu.GetMagCalibration(bias_x, bias_y, bias_z, scale_x, scale_y, scale_z);
      Serial.print("float MagErrorX = ");
      Serial.print(bias_x);
      Serial.println(";");
      Serial.print("float MagErrorY = ");
      Serial.print(bias_y);
      Serial.println(";");
      Serial.print("float MagErrorZ = ");
      Serial.print(bias_z);
      Serial.println(";");
      Serial.print("float MagScaleX = ");
      Serial.print(scale_x);
      Serial.println(";");
      Serial.print("float MagScaleY = ");
      Serial.print(scale_y);
      Serial.println(";");
      Serial.print("float MagScaleZ = ");
      Serial.print(scale_z);
      Serial.println(";");
      Serial.println(" ");
      Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
    }
    else {
      Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
    }

    while(1); //Halt code so it won't enter main loop until this function commented out
  #endif
  Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent magnetometer.");
  while(1); //Halt code so it won't enter main loop until this function commented out
}

//loopBlink() removed - timing now handled by INav scheduler's TASK_BLINK

void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  //NOTE: Still used by calibration functions (calibrateAttitude, calibrateESCs)
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void setupBlink(int numBlinks,int upTime, int downTime) {
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(ledPin, LOW);
    delay(downTime);
    digitalWrite(ledPin, HIGH);
    delay(upTime);
  }
}

void printRadioData() {
  //Rate now controlled by TASK_TELEMETRY (100 Hz)
  Serial.print(" CH1:");
  Serial.print(channel_1_pwm);
  Serial.print(" CH2:");
  Serial.print(channel_2_pwm);
  Serial.print(" CH3:");
  Serial.print(channel_3_pwm);
  Serial.print(" CH4:");
  Serial.print(channel_4_pwm);
  Serial.print(" CH5:");
  Serial.print(channel_5_pwm);
  Serial.print(" CH6:");
  Serial.print(channel_6_pwm);
  Serial.print(" [");
  Serial.print(radioSignalStatus());
  Serial.println("]");
}

void printDesiredState() {
  Serial.print("thro_des:");
  Serial.print(thro_des);
  Serial.print(" roll_des:");
  Serial.print(roll_des);
  Serial.print(" pitch_des:");
  Serial.print(pitch_des);
  Serial.print(" yaw_des:");
  Serial.println(yaw_des);
}

void printGyroData() {
  Serial.print("GyroX:");
  Serial.print(GyroX);
  Serial.print(" GyroY:");
  Serial.print(GyroY);
  Serial.print(" GyroZ:");
  Serial.println(GyroZ);
}

void printAccelData() {
  Serial.print("AccX:");
  Serial.print(AccX);
  Serial.print(" AccY:");
  Serial.print(AccY);
  Serial.print(" AccZ:");
  Serial.println(AccZ);
}

void printMagData() {
  Serial.print("MagX:");
  Serial.print(MagX);
  Serial.print(" MagY:");
  Serial.print(MagY);
  Serial.print(" MagZ:");
  Serial.println(MagZ);
}

void printRollPitchYaw() {
  Serial.print("roll:");
  Serial.print(roll_IMU);
  Serial.print(" pitch:");
  Serial.print(pitch_IMU);
  Serial.print(" yaw:");
  Serial.println(yaw_IMU);
}

void printPIDoutput() {
  Serial.print("roll_PID:");
  Serial.print(roll_PID);
  Serial.print(" pitch_PID:");
  Serial.print(pitch_PID);
  Serial.print(" yaw_PID:");
  Serial.println(yaw_PID);
}

void printMotorCommands() {
  Serial.print("m1_command:");
  Serial.print(m1_command_scaled, 3);
  Serial.print(" m2_command:");
  Serial.print(m2_command_scaled, 3);
  Serial.print(" m3_command:");
  Serial.print(m3_command_scaled, 3);
  Serial.print(" m4_command:");
  Serial.print(m4_command_scaled, 3);
  Serial.print(" m5_command:");
  Serial.print(m5_command_scaled, 3);
  Serial.print(" m6_command:");
  Serial.println(m6_command_scaled, 3);
}

void printServoCommands() {
  Serial.print("s1_command:");
  Serial.print(s1_command_PWM);
  Serial.print(" s2_command:");
  Serial.print(s2_command_PWM);
  Serial.print(" s3_command:");
  Serial.print(s3_command_PWM);
  Serial.print(" s4_command:");
  Serial.print(s4_command_PWM);
  Serial.print(" s5_command:");
  Serial.print(s5_command_PWM);
  Serial.print(" s6_command:");
  Serial.print(s6_command_PWM);
  Serial.print(" s7_command:");
  Serial.println(s7_command_PWM);
}

void printLoopRate() {
  //Print flight loop timing (dt from scheduler)
  int loopHz = (dt > 0) ? (int)(1.0f / dt) : 0;
  Serial.print("FLIGHT:");
  Serial.print(loopHz);
  Serial.print("Hz dt:");
  Serial.print(dt * 1000000.0f, 0);
  Serial.println("us");
}

//=========================================================================================//

//HELPER FUNCTIONS

float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}
