#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <PacketSerial.h>

#include "movement.h"
#include "shared.h"
#include "vector.h"

// Objects
struct Sensors {
    Line line;
    Vector ball;
    Vector offensiveGoal;
    Vector defensiveGoal;
    double robotAngle = NAN;
    bool robotHasBall = false;
    Vector robotPosition;
};

// Global variables
extern PacketSerial L1ESPPacketSerial;
extern PacketSerial L2ESPPacketSerial;
extern PacketSerial CameraPacketSerial;
extern Sensors sensors;
extern Movement movement;
extern bool limitSpeed;

// Subroutines
void checkTeensyReset();
void setMode();
void setupIMU();
double readIMUHeading();

void moveBehindBall();
void avoidLine();

#endif // MAIN_H
