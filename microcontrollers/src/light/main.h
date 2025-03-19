#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <array>
#include <deque>
#include <stm32f103c_variant_generic.h>

#include "mux.h"
#include "shared.h"

// Pins
#define PIN_LED      PB15

// Serial devices
#define TeensySerial Serial2
// pins
#define m1s0 PB2
#define m1s1 PB10
#define m1s2 PB0
#define m1s3 PA7
#define m1 PB1
#define m2s0 PB6
#define m2s1 PB7
#define m2s2 PB5
#define m2s3 PB4
#define m2 PA6



// muxes
const auto muxs = std::array<Mux, 2>{
    // S0, S1, S2, S3, SIGNAL
    Mux(m1s0, m1s1, m1s2, m1s3, m1),
    Mux(m2s0, m2s1, m2s2, m2s3, m2),
};

// Light ring information

#define PHOTODIODE_COUNT 30
constexpr std::array<uint8_t, PHOTODIODE_COUNT> PHOTODIODE_MAP = {
    0x10 + 0, 0x00 + 14, 0x00 + 13,0x00 + 12, 0x00+11, 0x00 + 10,
    0x00 + 9, 0x00 + 8, 0x00 + 7, 0x00 + 6, 0x00 + 5, 0x00 + 4,
    0x00 + 3, 0x00 + 2, 0x00 + 1, 0x00 + 0, 0x10 + 14, 0x10 + 13,
    0x10 + 12, 0x10 + 11, 0x10 + 10, 0x10 + 9, 0x10 + 8, 0x10 + 7,
    0x10 + 6, 0x10 + 5, 0x10 + 4, 0x10 + 3, 0x10 + 2, 0x10 + 1,
};
constexpr std::array<double, PHOTODIODE_COUNT> PHOTODIODE_BEARINGS = {
    0.00,   12.00,  24.00,  36.00,  48.00,  60.00,  72.00,  84.00,
    96.00,  108.00, 120.00, 132.00, 144.00, 156.00, 168.00, 180.00,
    192.00, 204.00, 216.00, 228.00, 240.00, 252.00, 264.00, 276.00,
    288.00, 300.00, 312.00, 324.00, 336.00, 348.00,
};
// Parameters (for computing whether the photodiode is on the line)
#define MUX_READ_DELAY                  0 // delay between channel switch and read in µs
#define PHOTODIODE_ACTIVATION_THRESHOLD 2 // number of consecutive reads on line

// Parameters (for photodiode calibration)
#define LIGHT_RING_CALIBRATION_DURATION 1000 // in ms

constexpr std::array<uint16_t, PHOTODIODE_COUNT> PHOTODIODE_THRESHOLDS = {
    1967, 731, 343,        134,         67,  47, 40, 10,41, 32, 37,
    6,  15,  21,         33,         7, 2152, 2576,  1942,  1760,  3143,
    3308,  2028,  1860, 3305,  3323,  1948,  1980,  2620,  1959, 
};

// Parameters (for computing which side of the line is the robot on)
// To have switched sides of the line, the line angle must have jumped this much
#define LINE_ANGLE_SWITCH_ANGLE 90.0
// for this number of times
#define LINE_ANGLE_SWITCH_COUNT 2
// when compared to this number of previous line angles
#define LINE_ANGLE_HISTORY 2

// Subroutines
uint16_t readPhotodiode(uint8_t i);
std::pair<double, double> findLine(std::array<bool, PHOTODIODE_COUNT> onLine);
std::pair<double, double> adjustForLineSide(std::pair<double, double> line);
void printLightRingCalibration();
void printLightRing();

#endif // MAIN_H
