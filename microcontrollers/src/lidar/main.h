#pragma once

#include "PacketSerial.h"
#include "util.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <TFLI2C.h>
#include <Wire.h>

// Uncomment to enable debug output
// #define DEBUGTOF

extern HardwareSerial MySerial0;
extern PacketSerial L3LIDARSerial;
extern TFLI2C tflI2C[4];
extern int16_t tfDist[4];
extern int tfAddress[4];

// Structs
struct lidardata {
    int distance[4];
};

extern lidardata esp32lidardata;

typedef struct lidarTxPayload {
    lidardata esp32lidardata;
} lidarTxPayload;

// Function prototypes
void setupLidar();
void readLidar();
void sendLidarData();

