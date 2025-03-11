#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct config_s config_t;

// our config struct contains all data that has to be stored in persistant storage
struct config_s {

  // Version signature
  uint8_t   version; // Byte stamp ID to identify version and already setup
  // (1)[1]

//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //
//========================================================================================================================//

  // Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
  unsigned long channel_1_fs; // thro
  unsigned long channel_2_fs; // ail
  unsigned long channel_3_fs; // elev
  unsigned long channel_4_fs; // rudd
  unsigned long channel_5_fs; // gear, greater than 1500 = throttle cut
  unsigned long channel_6_fs; // aux1
  // (6 * 4 = 24)[25]

  // Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
  float B_madgwick;       // Madgwick filter parameter
  float B_accel;          // Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
  float B_gyro;           // Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
  float B_mag;            // Magnetometer LP filter parameter
  // (4 * 4 = 16)[41]

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
  float MagErrorX;
  float MagErrorY;
  float MagErrorZ;
  float MagScaleX;
  float MagScaleY;
  float MagScaleZ;
  // (6 * 4 = 24)[65]

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
  float AccErrorX;
  float AccErrorY;
  float AccErrorZ;
  float GyroErrorX;
  float GyroErrorY;
  float GyroErrorZ;
  // (6 * 4 = 24)[89]

  // Controller parameters (take note of defaults before modifying!):
  float i_limit;          // Integrator saturation level, mostly for safety (default 25.0)
  float maxRoll;          // Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
  float maxPitch;         // Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
  float maxYaw ;          // Max yaw rate in deg/sec
  // (4 * 4 = 16)[105]

  float Kp_roll_angle;    // Roll P-gain - angle mode
  float Ki_roll_angle;    // Roll I-gain - angle mode
  float Kd_roll_angle;    // Roll D-gain - angle mode (if using controlANGLE2(), set to 0.0)
  float B_loop_roll;      // Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
  float Kp_pitch_angle;   // Pitch P-gain - angle mode
  float Ki_pitch_angle;   // Pitch I-gain - angle mode
  float Kd_pitch_angle;   // Pitch D-gain - angle mode (if using controlANGLE2(), set to 0.0)
  float B_loop_pitch;     // Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
  // (8 * 4 = 32)[137]

  float Kp_roll_rate;     // Roll P-gain - rate mode
  float Ki_roll_rate;     // Roll I-gain - rate mode
  float Kd_roll_rate;     // Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
  float Kp_pitch_rate;    // Pitch P-gain - rate mode
  float Ki_pitch_rate;    // Pitch I-gain - rate mode
  float Kd_pitch_rate;    // Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
  // (6 * 4 = 24)[161]

  float Kp_yaw;           // Yaw P-gain
  float Ki_yaw;           // Yaw I-gain
  float Kd_yaw;           // Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
  // (3 * 4 = 12)[173]

  uint8_t   z_Save;       // Flag to save setup (default to 0, set to 1 to save)
  // (1)[174]

};

// rounded up
// #define SIZEOF_STORAGE_IN_16BIT   ( ( sizeof( cfg ) + 1 ) / 2 )
#define SIZEOF_STORAGE_IN_BYTES   ( sizeof( cfg ) )

extern config_t cfg;

#ifdef __cplusplus
}
#endif
