#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include "PacketSerial.h"
#include "config.h"
#include "shared.h"
#include "vector.h"
#include "util.h"
#include <Servo.h>
// SERIAL OBJECTS
extern PacketSerial LightPacketSerial; 
extern PacketSerial TopPacketSerial; // teensy teensy
extern PacketSerial EncoderPacketSerial;

#define LightSerial Serial3
#define TopSerial Serial4 // teensy teensy
#define EncoderSerial Serial5

void TopPacketHandler(const byte *buf, size_t size); // teensy teensy
void LightPacketHandler(const byte *buf, size_t size);
void EncoderPacketHandler(const byte *buf, size_t size);

// PINS
#define KickerSignal 23
#define FrontESCpin 9
#define BackESCpin 6
#define FLAG1 13
#define FLAG2 22

struct Processed{
    Vector ball;
    Vector blue;
    Vector yellow;
    int lidarDist[4];
    double lidarConfidence[4];
    double yaw;
    Point robot_position;
};


// dribbler kicker 
extern Servo FrontESC;
extern Servo BackESC;
void setupDribblers();
void runDribblers(int speed);


#endif