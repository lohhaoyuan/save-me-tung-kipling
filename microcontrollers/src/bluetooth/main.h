#pragma once

#include <Arduino.h>
#include <shared.h>
#include <config.h>
#include "PacketSerial.h"
#define TeensySerial Serial0

extern PacketSerial TeensyPacketSerial;
void TeensyPacketHandler(const uint8_t *buf, size_t length);

