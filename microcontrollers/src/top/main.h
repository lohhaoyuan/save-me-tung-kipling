#ifndef MAIN_H
#define MAIN_H

#include "PacketSerial.h"
#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <SPI.h>

#include "config.h"
#include "shared.h"
#include "util.h"
#include "vector.h"
// Pins and Serial
#define IMU_CS  10
#define IMU_INT 14
#define IMU_RST 15

#define BluetoothSerial Serial1
#define SubSerial       Serial2
#define LidarSerial     Serial4
#define CameraSerial    Serial5

struct Sensors {
    Vector cam_ball;
    Vector bt_ball;
    Vector attack;
    Vector defense;
    Vector ball;
    int lidarDist[4];
    double yaw;
    Vector robot_position;
    bool TeammateLive;
};

extern Sensors sensors;
extern PacketSerial BluetoothPacketSerial;
extern PacketSerial SubPacketSerial;
extern PacketSerial LidarPacketSerial;
extern PacketSerial CameraPacketSerial;

// functions

// Packet Handlers
void BTPacketHandler(const uint8_t *buf, size_t size);
void CameraPacketHandler(const uint8_t *buf, size_t size);
void LidarPacketHandler(const uint8_t *buf, size_t size);

// IMU
void setupIMU();
double readIMUHeading();

// localisation
double ballMapping(double distance);
double goalMapping(double distance);
Vector localise();

void ballFusion();
void positionFusion();

#endif