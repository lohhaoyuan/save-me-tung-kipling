#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <array>
#include <deque>
#include <stm32f103c_variant_generic.h>

#include "mux.h"
#include "shared.h"

// Pins
#define PIN_LED      PC13
#define PIN_SOLENOID PA8

// Serial devices
#define TeensySerial Serial2

// Multiplexers
const auto muxs = std::array<Mux, 2>{
#ifdef NEW_BOT
    // S0, S1, S2, S3, SIGNAL
    Mux(PB6, PB7, PB5, PB4, PA6),
    Mux(PB2, PB10, PB0, PA7, PB1),
#else
    // S0, S1, S2, S3, SIGNAL
    Mux(PB1, PB2, PA6, PA5, PB0),
    Mux(PB8, PB9, PB6, PB5, PA4),
#endif
};

// Light ring information
#ifdef NEW_BOT
    #define PHOTODIODE_COUNT 30
constexpr std::array<uint8_t, PHOTODIODE_COUNT> PHOTODIODE_MAP = {
    0x00 + 0,  0x10 + 14, 0x10 + 13, 0x10 + 12, 0x10 + 11, 0x10 + 10,
    0x10 + 9,  0x10 + 8,  0x10 + 7,  0x10 + 6,  0x10 + 5,  0x10 + 4,
    0x10 + 3,  0x10 + 2,  0x10 + 1,  0x10 + 0,  0x00 + 14, 0x00 + 13,
    0x00 + 12, 0x00 + 11, 0x00 + 10, 0x00 + 9,  0x00 + 8,  0x00 + 7,
    0x00 + 6,  0x00 + 5,  0x00 + 4,  0x00 + 3,  0x00 + 2,  0x00 + 1,
};
constexpr std::array<double, PHOTODIODE_COUNT> PHOTODIODE_BEARINGS = {
    0.00,   12.00,  24.00,  36.00,  48.00,  60.00,  72.00,  84.00,
    96.00,  108.00, 120.00, 132.00, 144.00, 156.00, 168.00, 180.00,
    192.00, 204.00, 216.00, 228.00, 240.00, 252.00, 264.00, 276.00,
    288.00, 300.00, 312.00, 324.00, 336.00, 348.00,
};
#else
    #define PHOTODIODE_COUNT 30
constexpr std::array<uint8_t, PHOTODIODE_COUNT> PHOTODIODE_MAP = {
    0x00 + 15, 0x00 + 14, 0x00 + 13, 0x00 + 12, 0x00 + 11, 0x00 + 10,
    0x00 + 9,  0x00 + 8,  0x00 + 7,  0x00 + 6,  0x00 + 5,  0x00 + 4,
    0x00 + 3,  0x00 + 2,  0x00 + 1,  0x00 + 0,  0x10 + 13, 0x10 + 12,
    0x10 + 11, 0x10 + 10, 0x10 + 9,  0x10 + 8,  0x10 + 7,  0x10 + 6,
    0x10 + 5,  0x10 + 4,  0x10 + 3,  0x10 + 2,  0x10 + 1,  0x10 + 0,
};
constexpr std::array<double, PHOTODIODE_COUNT> PHOTODIODE_BEARINGS = {
    0.00,   12.00,  24.00,  36.00,  48.00,  60.00,  72.00,  84.00,
    96.00,  108.00, 120.00, 132.00, 144.00, 156.00, 168.00, 180.00,
    192.00, 204.00, 216.00, 228.00, 240.00, 252.00, 264.00, 276.00,
    288.00, 300.00, 312.00, 324.00, 336.00, 348.00,
};
#endif

// Parameters (for computing whether the photodiode is on the line)
#define MUX_READ_DELAY                  0 // delay between channel switch and read in µs
#define PHOTODIODE_ACTIVATION_THRESHOLD 2 // number of consecutive reads on line

// Parameters (for photodiode calibration)
#define LIGHT_RING_CALIBRATION_DURATION 1000 // in ms
#ifdef NEW_BOT
// old
// constexpr std::array<uint16_t, PHOTODIODE_COUNT> PHOTODIODE_THRESHOLDS =
// {
//     2185, 1885, 1386, 1288, 1305, 1213, 1329, 2070, 2031, 1880,
//     964,  1257, 1174, 1693, 2301, 2322, 1673, 1152, 1192, 1270,
//     1135, 1368, 18,   15,   1376, 1280, 1410, 1239, 1564, 1803,
// };
// at bordeaux
// constexpr std::array<uint16_t, PHOTODIODE_COUNT> PHOTODIODE_THRESHOLDS =
// {
//     2248, 2085, 1549,        1455,        1594, 1509, 1825, 2181, 2176,
//     2185, 1592, 1234, 1493,        2096,        2196, 2259, 1883, 1297,
//     1279, 1337, 1318, 1772, 100 /*ded*/, 100 /*ded*/, 1607, 1539, 1756,
//     1775, 2070, 2234,
// };
// lifted up at bordeaux
// constexpr std::array<uint16_t, PHOTODIODE_COUNT> PHOTODIODE_THRESHOLDS =
// {
//     842, 671, 755,         627,         588, 806, 882, 932, 1043, 1020,
//     662, 776, 732,         783,         812, 807, 828, 589, 679,  799,
//     568, 833, 100 /*ded*/, 100 /*ded*/, 782, 614, 712, 586, 861,  910,
// };
// best at bordeaux (USE THIS)
constexpr std::array<uint16_t, PHOTODIODE_COUNT> PHOTODIODE_THRESHOLDS = {
    1342, 1188, 1182,        928,         887,  1057, 1157, 1238, 1281, 1203,
    767,  843,  828,         980,         1022, 1062, 986,  616,  679,  799,
    568,  833,  100 /*ded*/, 100 /*ded*/, 782,  794,  978,  861,  1145, 1284,
};
    // superteam
    // constexpr std::array<uint16_t, PHOTODIODE_COUNT> PHOTODIODE_THRESHOLDS =
    // {
    //     1720, 1370, 1178,        968,         1078, 1102, 1205, 1514, 1552,
    //     1428, 955,  797,  947,         1213,        1340, 1502, 1192, 830,
    //     980,  1027, 790,  1108, 100 /*ded*/, 100 /*ded*/, 785,  783,  931,
    //     910,  1327, 1603,
    // };

#else
// old
// constexpr std::array<uint16_t, PHOTODIODE_COUNT> PHOTODIODE_THRESHOLDS = {
//     2335, 2465, 2134, 2082, 2235, 2299, 2320, 2218, 2365, 2340,
//     2444, 2387, 2378, 2282, 2317, 2470, 2589, 2550, 2589, 2527,
//     2418, 2446, 2655, 2406, 2491, 2373, 2440, 2415, 2482, 2537,
// };
// at bordeaux
constexpr std::array<uint16_t, PHOTODIODE_COUNT> PHOTODIODE_THRESHOLDS = {
    2321, 2483, 2160, 2072, 2242, 2308, 2334, 2248, 2399, 2369,
    2464, 2409, 2379, 2335, 2410, 2557, 2615, 2596, 2635, 2570,
    2459, 2474, 2657, 2430, 2538, 2391, 2413, 2472, 2471, 2574,
};
#endif

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
