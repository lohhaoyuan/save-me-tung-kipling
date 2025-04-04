#ifndef MAIN_H
#define MAIN_H


#include "shared.h"
#include "stm32f4.h"

#include "config.h"

#include <Arduino.h>
#include <PacketSerial.h>

#define TeensySerial Serial2
extern PacketSerial TeensyPacketSerial;

#define LED_PIN PA15

#define M1_PWM PA6
#define FL_PWM PA6
#define M1_DIR PA7
#define FL_DIR PA7
#define M1_ENCA PB13
#define FL_ENCA PB13
#define M1_ENCB PB12
#define FL_ENCB PB12

#define M2_PWM PB6
#define BL_PWM PB6
#define M2_DIR PB5
#define BL_DIR PB5
#define M2_ENCA PA9
#define BL_ENCA PA9
#define M2_ENCB PA8
#define BL_ENCB PA8

#define M3_PWM PB8
#define BR_PWM PB8
#define M3_DIR PB9
#define BR_DIR PB9
#define M3_ENCA PA10
#define BR_ENCA PA10
#define M3_ENCB PA11
#define BR_ENCB PA11

#define M4_PWM PA5
#define FR_PWM PA5
#define M4_DIR PA4
#define FR_DIR PA4
#define M4_ENCA PB1
#define FR_ENCA PB1
#define M4_ENCB PB0
#define FR_ENCB PB0

extern int16_t speeds[4];
    // Constrain motor speed with "hardware-imposed" limits
void TeensyPacketHandler(const uint8_t *buf, size_t size);
void drive(int16_t FL_SPEED,
           int16_t FR_SPEED,
           int16_t BL_SPEED,
           int16_t BR_SPEED);


#endif