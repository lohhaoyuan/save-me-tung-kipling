#ifndef MAIN_H
#define MAIN_H

#include "Adafruit_BNO055.h"
#include "Adafruit_Sensor.h"
#include "Arduino.h"
#include "EEPROM.h"
#include "SPI.h"
#include "Wire.h"
#include "utility/imumaths.h"

#define SYNC_BYTE 0b11010110

// Pin Definitions
#define FL_PWM       3
#define FL_DIR       4
#define FR_PWM       5
#define FR_DIR       6
#define BR_PWM       9
#define BR_DIR       10
#define BL_PWM       11
#define BL_DIR       12
#define BALL_DETECT  20
#define DRIBBLER_PWM 19
#define BUILTIN_LED  13
// Serial Definitions
#define DEBUG         Serial
#define IR_SERIAL     Serial2
#define CAMERA_SERIAL Serial3
#define LAYER1_SERIAL Serial4
// Global Variables
int16_t bearingToAngle(float bearing);
int16_t robotAngle = 0;
int16_t lastError = 0;
int ballAngle, ballStrength;
int correction = 0;
bool ballCaptured, ballFound, kick = false;
float offsetAngle;

int correctionKP = 0;
int correctionKI = 0;
int correctionKD = 0;
// Movement Config

#define COS45 0.70710678118654752440084436210485F
#define SIN45 0.70710678118654752440084436210485F

// Should ideally be <=1.0F to preserve resolution

#define DRIVE_STALL_SPEED (int16_t)20
#define DRIVE_MIN_SPEED   (int16_t)40
#define DRIVE_MAX_SPEED   (int16_t)100

#define IMUKP  1.2
#define IMUKI  0.2
#define IMUKD  15
#define MATH_E 2.7182818284590452353602874713527

int FLSpeed, FRSpeed, BLSpeed, BRSpeed = 0;

#define BNO055_SAMPLERATE_DELAY_MS (100)

#endif