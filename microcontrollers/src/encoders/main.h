#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <PacketSerial.h>

#define TeensySerial Serial2

#define LED_PIN PA15

#define M1_PWM PA6
#define M1_DIR PA7
#define M1_ENCA PB13
#define M1_ENCB PB12

#define M2_PWM PB6
#define M2_DIR PB5
#define M2_ENCA PA9
#define M2_ENCB PA8

#define M3_PWM PB8
#define M3_DIR PB9
#define M3_ENCA PA10
#define M3_ENCB PA11

#define M4_PWM PA5
#define M4_DIR PA4
#define M4_ENCA PB1
#define M4_ENCB PB0

#endif
