#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <PacketSerial.h>

#include "shared.h"

#define LightSerial Serial3


struct Sensors {
    Line line;
};

extern PacketSerial LightPacketSerial;
extern Sensors sensors;

#endif