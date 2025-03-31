#ifndef SHARED_H
#define SHARED_H

#include <Arduino.h>
#include <cmath>

#include "vector.h"

// If it's the new robot, uncomment this
#define NEW_BOT

#define SHARED_BAUD_RATE  1000000 // this is used between microcontrollers
#define MONITOR_BAUD_RATE 115200

// // ESP modes

// enum class L1ESPMode {
//     Default,
//     // Debug
//     PrintLightRing,
//     PrintLoopTime,
//     // Calibration
//     CalibrateLightRing,
// };

// enum class L2ESPMode {
//     Default,
//     // Debug
//     PrintIRRing,
//     PrintLoopTime,
//     // Calibration
//     CalibrateIRRing,
// };

// Sensor reading objects

struct Line {
    double angleBisector = NAN;
    double depth = NAN;

    bool exists() { return !std::isnan(angleBisector) && !std::isnan(depth); }
};

// Serial payloads

struct LightTxPayload {
    Line line;
};
struct CameraPayload {
    int data[6];
};

struct SesbianLexPayload {
    Vector ball;
    Vector blue;
    Vector yellow;
    int lidarDist[4];
    double lidarConfidence[4];
    double yaw;
    Point robot_position;
}; // teensy payload

struct EncoderTxPayload {
    int16_t motorSpeed[4];
};

struct EncoderRxPayload{
    uint8_t rpm[4];
};


// union L2ESPTxPayloadUnion {
//     L2ESPTxPayload data;
//     byte bytes[sizeof(L2ESPTxPayload)];

//     L2ESPTxPayloadUnion() : data() {}
// };

// union L2ESPRxPayloadUnion {
//     L2ESPRxPayload data;
//     byte bytes[sizeof(L2ESPRxPayload)];

//     L2ESPRxPayloadUnion() : data() {}
// };

// union CameraTxPayloadUnion {
//     CameraTxPayload data;
//     byte bytes[sizeof(CameraTxPayload)];

//     CameraTxPayloadUnion() : data() {}
// };

#endif // SHARED_H
