#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <array>
#include <deque>
#include <stm32f103c_variant_generic.h>
#include <PacketSerial.h>

#include "config.h"
#include "mux.h"
#include "shared.h"

#define TeensySerial Serial2
extern PacketSerial TeensyPacketSerial;

#define LED_PIN PB15
// Remap pins with new names
#define M1    PB1    // Analog input from MUX1
#define M1S0  PB2    // MUX1 control bit 0
#define M1S1  PB10   // MUX1 control bit 1
#define M1S2  PB0    // MUX1 control bit 2
#define M1S3  PA7    // MUX1 control bit 3

#define M2    PA6    // Analog input from MUX2
#define M2S0  PB6    // MUX2 control bit 0
#define M2S1  PB7    // MUX2 control bit 1
#define M2S2  PB5    // MUX2 control bit 2
#define M2S3  PB4    // MUX2 control bit 3

#define FLAG_FRONT PA5
#define FLAG_BACK PA9

//muxes
const auto muxs = std::array<Mux, 2>{
    Mux(M1S0,M1S1,M1S2,M1S3,M1),
    Mux(M2S0,M2S1,M2S2,M2S3,M2),
};

#define PHOTODIODE_COUNT 30
constexpr std::array<uint8_t, PHOTODIODE_COUNT> PHOTODIODE_MAP = {
    0x10 + 0,  0x00 + 14, 0x00 + 13, 0x00 + 12, 0x00 + 11, 0x00 + 10,
    0x00 + 9,  0x00 + 8,  0x00 + 7,  0x00 + 6,  0x00 + 5,  0x00 + 4,
    0x00 + 3,  0x00 + 2,  0x00 + 1,  0x00 + 0,  0x10 + 14, 0x10 + 13,
    0x10 + 12, 0x10 + 11, 0x10 + 10, 0x10 + 9,  0x10 + 8,  0x10 + 7,
    0x10 + 6,  0x10 + 5,  0x10 + 4,  0x10 + 3,  0x10 + 2,  0x10 + 1,
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
    1342, 1188, 1182,        928,         887,  1057, 1157, 1238, 1281, 1203,
    767,  843,  828,         980,         1022, 1062, 986,  616,  679,  799,
    568,  833,  100 /*ded*/, 100 /*ded*/, 782,  794,  978,  861,  1145, 1284,
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
void updateLightGates();


#endif