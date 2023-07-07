#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>
#include <Wire.h>

// Functions to handle angles

#define SIN45 0.70710678F
#define COS45 0.70710678F

double clipAngleTo180(double angle);
double clipAngleTo360(double angle);
double angleDifference(double leftAngle, double rightAngle);
double smallerAngleDifference(double leftAngle, double rightAngle);
double angleBisector(double leftAngle, double rightAngle);

double sind(double angle);
double cosd(double angle);
double acosd(double angle);
double atan2d(double y, double x);

// Functions to deal with interfaces

void scanI2C(Stream &serial, TwoWire wire);

void wipeEEPROM();

// Functions to print debug output

uint32_t printLoopTime(Stream &serial, uint16_t sampleCount);
void printDouble(Stream &serial, double value, uint8_t integerPlaces = 0,
                uint8_t decimalPlaces = 2);

#endif // UTIL_H
