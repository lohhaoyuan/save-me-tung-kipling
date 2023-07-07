#ifndef MAIN_H
#define MAIN_H

#include "shared.h"

#include <Arduino.h>
#ifdef NEW_BOT
    #include <SoftwareSerial.h>
#endif
#include <algorithm>
#include <array>
#include <stm32f103c_variant_generic.h>
#include <utility>

// Pins
#define PIN_LED PB15

// Serial devices
#ifdef NEW_BOT
    #define TeensySerial Serial1
#else
    #define TeensySerial Serial2
#endif

// IR ring information
#ifdef NEW_BOT
    #define IR_COUNT 24
#else
    #define IR_COUNT 24
#endif
#ifdef NEW_BOT
constexpr std::array<uint8_t, IR_COUNT> IR_PINS = {
    PA5,
    PA4,
    PA3,
    PA2,
    PA1,
    PA0,
    PB9,
    PB8,
    PB7,
    PB6,
    PB5,
    PB4,
    PB3,
    PA15,
    PA12,
    PA11,
    PA8,
    PB14,
    PB14 /* B15 isn't connected */,
    PB2,
    PB1,
    PB0,
    PA7,
    PA6,
};
constexpr std::array<double, IR_COUNT> IR_BEARINGS = {
    0.00,   15.00,  30.00,  45.00,  60.00,  75.00,  90.00,  105.00,
    120.00, 135.00, 150.00, 165.00, 180.00, 195.00, 210.00, 225.00,
    240.00, 255.00, 270.00, 285.00, 300.00, 315.00, 330.00, 345.00,
};
// Scaling actually doesn't work very well for some reason
// #define SCALE_IR
// #define IR_SCALER_BASELINE 0.58
// constexpr std::array<double, IR_COUNT> IR_SCALER = {
//     0.58, 0.56, 0.58, 0.58, 0.55, 0.52, // 0º to 90º
//     0.58, 0.39, 0.48, 0.45, 0.40, 0.41, // 90º to 180º
//     0.50, 0.52, 0.48, 0.49, 0.61, 0.60, // 180º to 270º
//     0.60, 0.60, 0.60, 0.64, 0.64, 0.63, // 270º to 360º
// };
#else
constexpr std::array<uint8_t, IR_COUNT> IR_PINS = {
    PA5, PA4,  PA1,  PA0,  PC15, PC14, PB9,  PB8, PB7, PB6, PB5, PB4,
    PB3, PA15, PB14, PB13, PB12, PB11, PB10, PB2, PB1, PB0, PA7, PA6,
};
constexpr std::array<double, IR_COUNT> IR_BEARINGS = {
    0.00,   15.00,  30.00,  45.00,  60.00,  75.00,  90.00,  105.00,
    120.00, 135.00, 150.00, 165.00, 180.00, 195.00, 210.00, 225.00,
    240.00, 255.00, 270.00, 285.00, 300.00, 315.00, 330.00, 345.00,
};
#endif
const std::array<std::pair<double, double>, IR_COUNT> IR_UNIT_VECTORS = []() {
    std::array<std::pair<double, double>, IR_COUNT> v{};
    for (size_t i = 0; i < IR_COUNT; ++i) {
        const auto bearing_rad = radians(IR_BEARINGS[i]);
        v[i] = std::make_pair(cos(bearing_rad), sin(bearing_rad));
    }
    return v;
}();

// Parameters (for IR ring)
// We sample this quantity of cycles of pulses emitted by the IR ball
#define IR_SAMPLE_CYCLE_COUNT 4 // results in mean loop time of 4.2 to 4.3 ms
// We sample this quantity of IR reading vectors
#define IR_ANGLE_SAMPLE_COUNT    24
#define IR_STRENGTH_SAMPLE_COUNT 24
// We sample this quantity of ball data to perform a moving average
#ifdef NEW_BOT
    #define IR_SAMPLE_BALL_ANGLE_COUNT    24
    #define IR_SAMPLE_BALL_STRENGTH_COUNT 24
#else
    #define IR_SAMPLE_BALL_ANGLE_COUNT    24
    #define IR_SAMPLE_BALL_STRENGTH_COUNT 24
#endif
#ifdef NEW_BOT
    /*
    Polynomial regression was performed on the following data points:
    |     |      0º     |      90º    |     180º    |     270º    |
    | cm  | min  | max  | min  | max  | min  | max  | min  | max  |
    | 0   | 2.50 | 3.26 | 1.84 | 2.11 |
    | 5   | 2.46 | 3.21 | 1.66 | 1.95 |
    | 10  | 2.21 | 2.79 | 1.59 | 1.83 |
    | 15  | 1.97 | 2.63 | 1.57 | 1.91 |
    | 20  | 1.74 | 2.44 | 1.52 | 1.88 |
    | 25  | 1.70 | 2.37 | 1.51 | 1.71 |
    | 30  | 1.64 | 2.07 | 1.53 | 1.70 |
    | 40  | 1.60 | 2.01 | 1.44 | 1.63 |
    | 50  | 1.51 | 1.61 | 1.35 | 1.45 |
    | 70  | 1.36 | 1.57 | 1.09 | 1.42 |
    | 90  | 0.83 | 1.37 | 0.67 | 1.32 |
    | 120 | 0.63 | 0.81 | 0.53 | 0.81 |
    | 150 | 0.48 | 0.58 | 0.44 | 0.59 |
    | 190 | NA   | 0.53 | 0.49 | 0.50 |
    */
    #define BALL_DISTANCE_A 268.16175769
    #define BALL_DISTANCE_B -230.55140268
    #define BALL_DISTANCE_C 71.42444777
    #define BALL_DISTANCE_D -8.18186262
    #define BALL_DISTANCE_E 0
    #define BALL_DISTANCE_F 0
    // this was when tape was there
    // #define BALL_DISTANCE_90_A  251.50336907
    // #define BALL_DISTANCE_90_B  -189.72881678
    // #define BALL_DISTANCE_90_C  31.0799153
    // #define BALL_DISTANCE_90_D  0
    // #define BALL_DISTANCE_90_E  0
#else
    /*
    Polynomial regression was performed on the following data points:
    cm / strength
    0 1.98
    5 2.10
    10 2.40
    15 2.34
    20 2.25
    25 2.13
    30 2.12
    40 2.04
    50 1.97
    60 1.89
    70 1.75
    80 1.53
    90 1.39
    100 1.27
    120 0.92
    150 0.85
    190 0.74
    */
    #define BALL_DISTANCE_A 1749.54169074
    #define BALL_DISTANCE_B -4696.44567174
    #define BALL_DISTANCE_C 5175.07061571
    #define BALL_DISTANCE_D -2733.77159153
    #define BALL_DISTANCE_E 681.7518904
    #define BALL_DISTANCE_F -64.08559721
    /*
    0 2.79
    5 2.22
    10 1.96
    15 1.80
    20 1.68
    25 1.48
    30 1.36
    35 1.19
    40 1.10
    45 1.04
    50 1.01
    60 0.87
    70 0.83
    80 0.78
    90 0.62
    100 0.53
    110 0.52
    120 0.53
    130 0.52
    140 0.52
    150 0.35
    160 0.24
    170 0.37
    180 0.09
    190 0.17
    200 0.16
    */
    // #define BALL_DISTANCE_A 277.83664157
    // #define BALL_DISTANCE_B -265.02296079
    // #define BALL_DISTANCE_C 106.82200809
    // #define BALL_DISTANCE_D -14.65762874
    // #define BALL_DISTANCE_E 0
    // #define BALL_DISTANCE_F 0
#endif

// Subroutines
void setupIRRing();
std::tuple<double, double, double> readIRRing(bool debugPrint = false);
void printIRRingCalibration();

#endif // MAIN_H
