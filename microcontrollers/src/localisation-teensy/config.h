#ifndef CONFIG_H
#define CONFIG_H

#include "shared.h"

#define DEBUG

// Mode
enum class TeensyMode {
    Default,
    // Debug
    // PrintTeensy,
    PrintLightRing,
    PrintIRRing,
    PrintL1ESPLoopTime,
    PrintL2ESPLoopTime,
    // Calibration
    CalibrateLightRing,
    CalibrateIRRing,
};
#define TEENSY_MODE TeensyMode::Default
#define FACE_YELLOW_GOAL                                                       \
    false // will be overriden by DIP switch on NEW_BOT (YES = face yellow)
#define IS_GOALIE                                                              \
    false // will be overriden by DIP switch on NEW_BOT (YES = goalie)
#define SUPERTEAM

// Pins
#define PIN_LED_BUILTIN 13
#ifdef NEW_BOT
    #define PIN_LIGHTGATE      23
    #define PIN_MOTOR_FL_PWM   2
    #define PIN_MOTOR_FL_DIR   3
    #define PIN_MOTOR_FR_PWM   4
    #define PIN_MOTOR_FR_DIR   5
    #define PIN_MOTOR_BR_PWM   8
    #define PIN_MOTOR_BR_DIR   9
    #define PIN_MOTOR_BL_PWM   6
    #define PIN_MOTOR_BL_DIR   7
    #define PIN_DRIBBLER_PWM   10
    #define PIN_SOFTWARE_RESET 22
    #define PIN_DIP_GOAL       12
    #define PIN_DIP_PLAYSTYLE  11
#else
    #define PIN_LIGHTGATE      23
    #define PIN_MOTOR_FL_PWM   3
    #define PIN_MOTOR_FL_DIR   4
    #define PIN_MOTOR_FR_PWM   5
    #define PIN_MOTOR_FR_DIR   6
    #define PIN_MOTOR_BR_PWM   9
    #define PIN_MOTOR_BR_DIR   10
    #define PIN_MOTOR_BL_PWM   11
    #define PIN_MOTOR_BL_DIR   12
    #define PIN_DRIBBLER_PWM   19
    #define PIN_SOFTWARE_RESET 22
#endif

// Motors and sensors
#ifdef NEW_BOT
    #define MOTOR_FL_REVERSED   false
    #define MOTOR_FR_REVERSED   false
    #define MOTOR_BL_REVERSED   false
    #define MOTOR_BR_REVERSED   false
    #define MOTOR_FL_MULTIPLIER 1.0
    #define MOTOR_FR_MULTIPLIER 1.0
    #define MOTOR_BL_MULTIPLIER 1.0
    #define MOTOR_BR_MULTIPLIER 1.0
    // #define MOTOR_FL_MULTIPLIER            1.25
    // #define MOTOR_FR_MULTIPLIER            1.70
    // #define MOTOR_BL_MULTIPLIER            1.35
    // #define MOTOR_BR_MULTIPLIER            1.0
    #define LIGHTGATE_THRESHOLD            600
    #define LIGHTGATE_THRESHOLD_COMPARATOR >
#else
    #define MOTOR_FL_REVERSED              false
    #define MOTOR_FR_REVERSED              true
    #define MOTOR_BL_REVERSED              false
    #define MOTOR_BR_REVERSED              true
    #define MOTOR_FL_MULTIPLIER            1.0
    #define MOTOR_FR_MULTIPLIER            1.50
    #define MOTOR_BL_MULTIPLIER            1.40
    #define MOTOR_BR_MULTIPLIER            1.0
    #define LIGHTGATE_THRESHOLD            800
    #define LIGHTGATE_THRESHOLD_COMPARATOR <
#endif

// Serial devices
#ifdef NEW_BOT
    #define L1ESPSerial     Serial1
    #define L2ESPSerial     Serial4
    #define CameraSerial    Serial3
    #define BluetoothSerial Serial5
#else
    #define L1ESPSerial  Serial4
    #define L2ESPSerial  Serial2
    #define CameraSerial Serial3
    #define IMUSerial    Serial5
#endif

// EEPROM Addresses
#define EEPROM_HAS_OFFSETS 0x000
#define EEPROM_OFFSETS     0x000 + sizeof(bool)

// Parameters (for field)
#ifndef SUPERTEAM
    #define HALF_GOAL_SEPARATION 107.5 // in cm
    #define FIELD_LENGTH         243.0 // in cm
    #define FIELD_WIDTH          182.0 // in cm
    // clang-format off
    #define HOME                (Point){0, -40}
    #define NEUTRAL_SPOT_CENTER (Point){0, 0}
    #define NEUTRAL_SPOT_FL     (Point){-11.5, 45}
    #define NEUTRAL_SPOT_FR     (Point){11.5, 45}
    #define NEUTRAL_SPOT_BL     (Point){-11.5, -45}
    #define NEUTRAL_SPOT_BR     (Point){11.5, -45}
// clang-format on
#else
    #define HALF_GOAL_SEPARATION 270.0 // in cm
    #define FIELD_LENGTH         600.0 // in cm
    #define FIELD_WIDTH          400.0 // in cm
    // clang-format off
    #define HOME                (Point){0, -40}
    #define NEUTRAL_SPOT_CENTER (Point){0, 0}
    #define NEUTRAL_SPOT_FL     (Point){-80, 135}
    #define NEUTRAL_SPOT_FR     (Point){80, 135}
    #define NEUTRAL_SPOT_BL     (Point){-80, -135}
    #define NEUTRAL_SPOT_BR     (Point){80, 135}
// clang-format on
#endif

// Parameters (for solenoid)
#define SOLENOID_ACTIVATION_PERIOD 150  // in ms
#define SOLENOID_COOLDOWN_PERIOD   2000 // in ms

#ifdef NEW_BOT
    // Parameters (for drive)
    #define DRIVE_STALL_SPEED 30
    #define DRIVE_MIN_SPEED   100
    #define DRIVE_MAX_SPEED   600
    #define SIGMOID_MAX_DIFF  DRIVE_MAX_SPEED
    // PID controller for robot heading
    #define HEADING_KU 6.0e1 // tuned to ±0.5e1
    // ZN (no overshoot)  : kP=0.2  kI=0.4  kD=0.066
    // ZN (some overshoot): kP=0.33 kI=0.66 kD=0.11
    // ZN (classic)       : kP=0.6  kI=1.2  kD=0.075
    #define HEADING_KP                  0.5 * HEADING_KU
    #define HEADING_KI                  1.2
    #define HEADING_KD                  60
    #define HEADING_MIN_DT              4000 // in µs, minimum value for kD to have effect
    #define HEADING_MAXI                3.5e4
    #define HEADING_MAX_SETPOINT_CHANGE 0.1
    #define HEADING_STATIONARY_SCALER   2.0
    // PID controller for line tracking (lateral position on line)
    #define LINE_TRACK_KU 4.0
    #define LINE_TRACK_KP 0.6 * LINE_TRACK_KU
    #define LINE_TRACK_KI 1.2
    #define LINE_TRACK_KD 5
    #define LINE_TRACK_MIN_DT                                                  \
        10000 // in µs, minimum value for kD to have effect
    #define LINE_TRACK_MAXI 1000
    // PID controller for moving to a point (distance from point)
    #define STOP_AT_POINT_KU 10.0
    #define STOP_AT_POINT_KP 0.2 * STOP_AT_POINT_KU
    #define STOP_AT_POINT_KI 0.4
    #define STOP_AT_POINT_KD 0.066
    #define STOP_AT_POINT_MIN_DT                                               \
        4000 // in µs, minimum value for kD to have effect
    #define STOP_AT_POINT_MAXI      1000000
    #define STOP_AT_POINT_MAX_SPEED 300

    // Parameters (for ball track)
    #ifndef SUPERTEAM
        // Multiplier = e^(DECAY * (START - Ball Distance))
        // https://www.desmos.com/calculator/ixwhywbd5i
        // This is the distance where we start curving maximally as the ball
        // gets closer
        #define BALL_MOVEMENT_MAX_CURVE 28 // in cm, tuned to ±1.0
        // The larger the value, the faster the decay of the angle offset
        #define BALL_MOVEMENT_DECAY          0.06
        #define BALL_MOVEMENT_MAX_MULTIPLIER 1.5
        // Decelerate as we approach the ball
        #define BALL_MOVEMENT_START_DECELERATING 100 // in cm, from ball
        #define BALL_MOVEMENT_STOP_DECELERATING  0   // in cm, from ball
        #define BALL_MOVEMENT_START_SPEED        220
        #define BALL_MOVEMENT_END_SPEED          170
    #else
        // Multiplier = e^(DECAY * (START - Ball Distance))
        // https://www.desmos.com/calculator/ixwhywbd5i
        // This is the distance where we start curving maximally as the ball
        // gets closer
        #define BALL_MOVEMENT_MAX_CURVE          30 // in cm, tuned to ±1.0
        // The larger the value, the faster the decay of the angle offset
        #define BALL_MOVEMENT_DECAY              0.06
        #define BALL_MOVEMENT_MAX_MULTIPLIER     1.2
        // Decelerate as we approach the ball
        #define BALL_MOVEMENT_START_DECELERATING 400 // in cm, from ball
        #define BALL_MOVEMENT_STOP_DECELERATING  30  // in cm, from ball
        #define BALL_MOVEMENT_START_SPEED        500
        #define BALL_MOVEMENT_END_SPEED          200
    #endif
    // Face the ball as we approach the ball
    #define BALL_MOVEMENT_FACE_BALL_DISTANCE 60 // in cm
    #define BALL_MOVEMENT_MAX_HEADING        70 // in degrees
    #define BALL_ANGLE_OFFSET                5  // in degrees

    // Parameters (for goal track)
    #ifndef SUPERTEAM
        #define GOAL_MOVEMENT_START_DECELERATING 200 // in cm, from goal
        #define GOAL_MOVEMENT_START_SPEED        350
        #define GOAL_MOVEMENT_END_SPEED          200
    #else
        #define GOAL_MOVEMENT_START_DECELERATING 400 // in cm, from goal
        #define GOAL_MOVEMENT_START_SPEED        600
        #define GOAL_MOVEMENT_END_SPEED          150
    #endif
    #define KICKER_WITH_BALL_THRESHOLD_DISTANCE 80
    #define KICKER_THRESHOLD_DISTANCE           70
    #define ROBOT_BALL_ANGLE_OFFSET             0
    #if IS_GOALIE
        #define ROBOT_BALL_MAX_ANGLE    10.0
        #define ROBOT_BALL_MIN_DISTANCE 10.0
    #else
        #define ROBOT_BALL_MAX_ANGLE    8.0
        #define ROBOT_BALL_MIN_DISTANCE 20.0
    #endif
    #define BALL_GOAL_ANGLE_THRESHOLD 60

    // Parameters (for line avoidance)
    // Speed limiting (no line)
    #define SPEED_LIMIT_START       80.0
    #define SPEED_LIMIT_END         20.0
    #define SPEED_LIMIT_START_SPEED 250
    #define SPEED_LIMIT_END_SPEED   200
    // Staying away from the wall (line depth >=
    // WALL_AVOIDANCE_THRESHOLD)
    #define WALL_AVOIDANCE_THRESHOLD 0.2
    #ifndef SUPERTEAM
        #define WALL_AVOIDANCE_SPEED_MULTIPLIER 200 / 0.2
    #else
        #define WALL_AVOIDANCE_SPEED_MULTIPLIER 1023
    #endif
    // Travelling on the line (0 <= line depth <
    // WALL_AVOIDANCE_THRESHOLD)
    #define BALL_LINE_TRACK_TARGET         0.1
    #define BALL_LINE_TRACK_MAX_HEADING    40.0 // in degrees
    #define BALL_LINE_TRACK_BEHIND_BALL_BY 4.0  // in cm
    #define BALL_LINE_TRACK_MIN_SPEED      150
    #define BALL_LINE_TRACK_MAX_SPEED      230
#else
    // Parameters (for drive)
    #define DRIVE_STALL_SPEED           20
    #define DRIVE_MIN_SPEED             100
    #define DRIVE_MAX_SPEED             600
    #define SIGMOID_MAX_DIFF            DRIVE_MAX_SPEED
    // PID controller for robot heading
    #define HEADING_KU                  5.0e1 // tuned to ±0.5e1
    // ZN (no overshoot)  : kP=0.2  kI=0.4  kD=0.066
    // ZN (some overshoot): kP=0.33 kI=0.66 kD=0.11
    // ZN (classic)       : kP=0.6  kI=1.2  kD=0.075
    #define HEADING_KP                  0.6 * HEADING_KU
    #define HEADING_KI                  1.2
    #define HEADING_KD                  15
    #define HEADING_MIN_DT              4000 // in µs, minimum value for kD to have effect
    #define HEADING_MAXI                3.5e4
    #define HEADING_MAX_SETPOINT_CHANGE 1.0
    #define HEADING_STATIONARY_SCALER   1.0
    // PID controller for line tracking (lateral position on line)
    #define LINE_TRACK_KU               4.0
    #define LINE_TRACK_KP               0.6 * LINE_TRACK_KU
    #define LINE_TRACK_KI               1.2
    #define LINE_TRACK_KD               5
    #define LINE_TRACK_MIN_DT                                                  \
        10000 // in µs, minimum value for kD to have effect
    #define LINE_TRACK_MAXI  1000
    // PID controller for moving to a point (distance from point)
    #define STOP_AT_POINT_KU 10.0
    #define STOP_AT_POINT_KP 0.2 * STOP_AT_POINT_KU
    #define STOP_AT_POINT_KI 0.4
    #define STOP_AT_POINT_KD 0.066
    #define STOP_AT_POINT_MIN_DT                                               \
        4000 // in µs, minimum value for kD to have effect
    #define STOP_AT_POINT_MAXI      1000000
    #define STOP_AT_POINT_MAX_SPEED 300

    // Parameters (for ball track)
    #ifndef SUPERTEAM
        // Multiplier = e^(DECAY * (START - Ball Distance))
        // https://www.desmos.com/calculator/ixwhywbd5i
        // This is the distance where we start curving maximally as the ball
        // gets closer
        #define BALL_MOVEMENT_MAX_CURVE          47 // in cm, tuned to ±1.0
        // The larger the value, the faster the decay of the angle offset
        #define BALL_MOVEMENT_DECAY              0.02
        #define BALL_MOVEMENT_MAX_MULTIPLIER     2.0
        // Decelerate as we approach the ball
        #define BALL_MOVEMENT_START_DECELERATING 150 // in cm, from ball
        #define BALL_MOVEMENT_STOP_DECELERATING  40  // in cm, from ball
        #define BALL_MOVEMENT_START_SPEED        230
        #define BALL_MOVEMENT_END_SPEED          190
    #else
        // Multiplier = e^(DECAY * (START - Ball Distance))
        // https://www.desmos.com/calculator/ixwhywbd5i
        // This is the distance where we start curving maximally as the ball
        // gets closer
        #define BALL_MOVEMENT_MAX_CURVE          80 // in cm, tuned to ±1.0
        // The larger the value, the faster the decay of the angle offset
        #define BALL_MOVEMENT_DECAY              0.01
        #define BALL_MOVEMENT_MAX_MULTIPLIER     2.0
        // Decelerate as we approach the ball
        #define BALL_MOVEMENT_START_DECELERATING 300 // in cm, from ball
        #define BALL_MOVEMENT_STOP_DECELERATING  0   // in cm, from ball
        #define BALL_MOVEMENT_START_SPEED        500
        #define BALL_MOVEMENT_END_SPEED          230
    #endif
    // Face the ball as we approach the ball
    #define BALL_MOVEMENT_FACE_BALL_DISTANCE 80 // in cm
    #define BALL_MOVEMENT_MAX_HEADING        70 // in degrees
    #define BALL_ANGLE_OFFSET                0  // in degrees

    // Parameters (for goal track)
    #ifndef SUPERTEAM
        #define GOAL_MOVEMENT_START_DECELERATING 150 // in cm, from goal
        #define GOAL_MOVEMENT_START_SPEED        300
        #define GOAL_MOVEMENT_END_SPEED          200
    #else

        #define GOAL_MOVEMENT_START_DECELERATING 200 // in cm, from goal
        #define GOAL_MOVEMENT_START_SPEED        400
        #define GOAL_MOVEMENT_END_SPEED          150
    #endif
    #define KICKER_WITH_BALL_THRESHOLD_DISTANCE 0
    #define KICKER_THRESHOLD_DISTANCE           0
    #define ROBOT_BALL_ANGLE_OFFSET             0
    #if IS_GOALIE
        #define ROBOT_BALL_MAX_ANGLE    15.0
        #define ROBOT_BALL_MIN_DISTANCE 35
    #else
        #define ROBOT_BALL_MAX_ANGLE    15.0
        #define ROBOT_BALL_MIN_DISTANCE 35
    #endif

    #define BALL_GOAL_ANGLE_THRESHOLD 80

    // Parameters (for line avoidance)
    // Speed limiting (no line)
    #define SPEED_LIMIT_START         90.0
    #define SPEED_LIMIT_END           30.0
    #define SPEED_LIMIT_START_SPEED   400
    #define SPEED_LIMIT_END_SPEED     200
    // Staying away from the wall (line depth >=
    // WALL_AVOIDANCE_THRESHOLD)
    #define WALL_AVOIDANCE_THRESHOLD  0.2
    #ifndef SUPERTEAM
        #define WALL_AVOIDANCE_SPEED_MULTIPLIER 200 / 0.2
    #else
        #define WALL_AVOIDANCE_SPEED_MULTIPLIER 1023
    #endif
    // Travelling on the line (0 <= line depth <
    // WALL_AVOIDANCE_THRESHOLD)
    #define BALL_LINE_TRACK_TARGET         0.1
    #define BALL_LINE_TRACK_MAX_HEADING    40.0 // in degrees
    #define BALL_LINE_TRACK_BEHIND_BALL_BY 4.0  // in cm
    #define BALL_LINE_TRACK_MIN_SPEED      150
    #define BALL_LINE_TRACK_MAX_SPEED      230
#endif

#endif // CONFIG_H
