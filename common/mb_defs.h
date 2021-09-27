/*******************************************************************************
* mb_defs.h
*
*   defines for your bot
*   You will need to fill this in based on the data sheets, schematics, etc. 
*      and your specific configuration...
* 
*******************************************************************************/
#ifndef MB_DEFS_H
#define MB_DEFS_H

//#define EXT_CAPE  //for use with the MOBILE ROB CAPE VERSIONS BELOW
//#define MRC_VERSION_1v3
//#define MRC_VERSION_2v1
#define BEAGLEBONE_BLUE
#define DEFAULT_PWM_FREQ        25000 // period of motor drive pwm
#define LEFT_MOTOR              1     // id of left motor
#define RIGHT_MOTOR             2    // id of right motor

// TODO: Add convienient defines to define things like motor and encoder polarity here
#define LEFT_POLARITY 1  // Left polarity should be one to move forward
#define RIGHT_POLARITY -1
#define LEFT_ENCODER_POLARITY -1
#define RIGHT_ENCODER_POLARITY 1

// TODO: Fill in physical propeties of robot
#define GEAR_RATIO              78.0  // gear ratio of motor
#define ENCODER_RES             20  // encoder counts per motor shaft revolution
#define WHEEL_DIAMETER          0.083 // diameter of wheel in meters
#define WHEEL_BASE              0.15684  // wheel separation distance in meters
#define MAX_FWD_VEL             0.8   // maximum forwad speed (m/s)
#define MAX_TURN_VEL            2.5   // maximum turning speed (rad/s)
// #define CFG_PATH                "../pid_params.yaml"


// These rates are set to defaults - can be changed
#define SAMPLE_RATE_HZ          50   // main filter and control loop speed
#define DT                      0.02  // 1/sample_rate
#define PRINTF_HZ               10    // rate of print loop
#define RC_CTL_HZ               25   // main filter and control loop speed
#define LCM_HZ                  100    // rate of LCM subscribe
#define LCM_PRIORITY            60    // priority of LCM thread (lower is less critical)
#define SETPOINT_PRIORITY       30    // priority of setpoint thread (lower is less critical)
#define CONTROLLER_PRIORITY     90    // priority of controller (lower is less critical)

// LCM Channel Names - should not be changed
#define TRUE_POSE_CHANNEL           "TRUE_POSE"
#define ODOMETRY_CHANNEL            "ODOMETRY"
#define RESET_ODOMETRY_CHANNEL      "RESET_ODOMETRY"
#define CONTROLLER_PATH_CHANNEL     "CONTROLLER_PATH"
#define MBOT_IMU_CHANNEL            "MBOT_IMU"
#define MBOT_ENCODER_CHANNEL        "MBOT_ENCODERS"
#define MBOT_MOTOR_COMMAND_CHANNEL  "MBOT_MOTOR_COMMAND"
#define MBOT_TIMESYNC_CHANNEL       "MBOT_TIMESYNC"
#define LCM_ADDRESS                 "udpm://239.255.76.67:7667?ttl=1"

#define LED_OFF                 1
#define LED_ON                  0

#endif
