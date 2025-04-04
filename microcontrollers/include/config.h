#ifndef CONFIG_H
#define CONFIG_H

// MATCH SPECIFIC SETTINGS
// attaack goal
#define YELLOW_ATTACK
// #define BLUE_ATTACK

// role
#define GOALIE
// #define STRIKER

// location
#define HOME
// #define COMP

// DRIBBLER SETTINGS - NEED TO CHANGE BECAUSE THESE ARE IVANS

#define DRIBBLER_LOWER_LIMIT   128
#define DRIBBLER_UPPER_LIMIT   198
#define DRIBBLER_DEFAULT_SPEED 220

// DRIVE SETTINGS - NEED TO CHANGE COS IVANS
#define DRIVE_STALL_SPEED 30
#define DRIVE_MIN_SPEED   100
#define DRIVE_MAX_SPEED   600
#define SIGMOID_MAX_DIFF  DRIVE_MAX_SPEED
// PID controller for robot heading
#define HEADING_KU 75
#define HEADING_TU 0.264

// ZN (no overshoot)  : kP=0.2  kI=0.4  kD=0.066
// ZN (some overshoot): kP=0.33 kI=0.66 kD=0.11
// ZN (classic)       : kP=0.6  kI=1.2  kD=0.075
#define HEADING_KP                  15
#define HEADING_KI                  113.64
#define HEADING_KD                  1.7
#define HEADING_MIN_DT              4000 // in µs, minimum value for kD to have effect
#define HEADING_MAXI                3.5e4
#define HEADING_MAX_SETPOINT_CHANGE 0.1
#define HEADING_STATIONARY_SCALER   2.0
// PID controller for line tracking (lateral position on line)
#define LINE_TRACK_KU     4.0
#define LINE_TRACK_KP     0.6 * LINE_TRACK_KU
#define LINE_TRACK_KI     1.2
#define LINE_TRACK_KD     5
#define LINE_TRACK_MIN_DT 10000 // in µs, minimum value for kD to have effect
#define LINE_TRACK_MAXI   1000
// PID controller for moving to a point (distance from point)
#define STOP_AT_POINT_KU        10.0
#define STOP_AT_POINT_KP        0.2 * STOP_AT_POINT_KU
#define STOP_AT_POINT_KI        0.4
#define STOP_AT_POINT_KD        0.066
#define STOP_AT_POINT_MIN_DT    4000 // in µs, minimum value for kD to have effect
#define STOP_AT_POINT_MAXI      1000000
#define STOP_AT_POINT_MAX_SPEED 300

#define MOTOR_FL_REVERSED true
#define MOTOR_FR_REVERSED false
#define MOTOR_BL_REVERSED true
#define MOTOR_BR_REVERSED false

#define MOTOR_FL_MULTIPLIER 1.0
#define MOTOR_FR_MULTIPLIER 1.0
#define MOTOR_BL_MULTIPLIER 1.0
#define MOTOR_BR_MULTIPLIER 1.0
// Parameters (for solenoid)
#define SOLENOID_ACTIVATION_PERIOD 150  // in ms
#define SOLENOID_COOLDOWN_PERIOD   2000 // in ms

// Parameters for Localisation
#define BLUETOOTH_RELIABILITY_THRESHOLD 20
#define HALF_GOAL_SEPARATION            103.6
#define HALF_SHORT_SEPARATION           73.1

#define LIGHT_GATE_THRESHOLD            1000
#define WALL_AVOIDANCE_THRESHOLD        0.2
#define WALL_AVOIDANCE_SPEED_MULTIPLIER 1023

#endif