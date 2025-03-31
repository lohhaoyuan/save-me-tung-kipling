#ifndef MAIN_H
#define MAIN_H

#include "shared.h"
#include "config.h"

#include <Arduino.h>
#include <PacketSerial.h>

#define TeensySerial Serial2
extern PacketSerial TeensyPacketSerial;

#define LED_PIN PA15

#define M1_PWM PA6
#define BR_PWM PA6
#define M1_DIR PA7
#define BR_DIR PA7
#define M1_ENCA PB13
#define BR_ENCA PB13
#define M1_ENCB PB12
#define BR_ENCB PB12

#define M2_PWM PB6
#define FR_PWM PB6
#define M2_DIR PB5
#define FR_DIR PB5
#define M2_ENCA PA9
#define FR_ENCA PA9
#define M2_ENCB PA8
#define FR_ENCB PA8

#define M3_PWM PB8
#define FL_PWM PB8
#define M3_DIR PB9
#define FL_DIR PB9
#define M3_ENCA PA10
#define FL_ENCA PA10
#define M3_ENCB PA11
#define FL_ENCB PA11

#define M4_PWM PA5
#define BL_PWM PA5
#define M4_DIR PA4
#define BL_DIR PA4
#define M4_ENCA PB1
#define BL_ENCA PB1
#define M4_ENCB PB0
#define BL_ENCB PB0

extern int16_t speeds[4];

    // Constrain motor speed with "hardware-imposed" limits
void TeensyPacketHandler(const uint8_t *buf, size_t size);
void drive(int16_t FL_SPEED,
           int16_t FR_SPEED,
           int16_t BL_SPEED,
           int16_t BR_SPEED);

#endif
