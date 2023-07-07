#include "util.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>

// Clips an angle to the range (-180, 180]
double clipAngleTo180(double angle) {
    angle = fmod(angle, 360);
    return angle > 180 ? angle - 360 : angle;
}

// Clips an angle to the range [0, 360)
double clipAngleTo360(double angle) {
    angle = fmod(angle, 360);
    return angle < 0 ? angle + 360 : angle;
}

// Finds the difference between two angles, in the range [0, 360)
double angleDifference(double leftAngle, double rightAngle) {
    return clipAngleTo360(rightAngle - leftAngle);
}

// Finds the smaller difference between two angles, in the range [0, 180]
double smallerAngleDifference(double leftAngle, double rightAngle) {
    const auto angle = angleDifference(leftAngle, rightAngle);
    return fmin(angle, 360 - angle);
}

// Finds the angle bisector between two angles, in the range [0, 360)
double angleBisector(double leftAngle, double rightAngle) {
    return clipAngleTo360(leftAngle +
                          smallerAngleDifference(leftAngle, rightAngle) / 2);
}

// Trigonometric functions that work in degrees clipped to the range (-180, 180]
double sind(double angle) { return sin(angle * DEG_TO_RAD); }
double cosd(double angle) { return cos(angle * DEG_TO_RAD); }
double acosd(double angle) { return acos(angle) * RAD_TO_DEG; }
double atan2d(double y, double x) {
    return clipAngleTo180(atan2(y, x) * RAD_TO_DEG);
}

// Scans the I2C bus for devices.
void scanI2C(Stream &serial, TwoWire wire) {
    // Adapted from
    // https://github.com/stm32duino/Arduino_Core_STM32/blob/main/libraries/Wire/examples/i2c_scanner/i2c_scanner.ino
    byte error, address;
    uint8_t deviceCount = 0;

    serial.println("Scanning...");

    for (address = 1; address < 127; address++) {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.

        wire.beginTransmission(address);
        error = wire.endTransmission();

        if (error == 0) {
            serial.printf("I2C device found at address 0x%02x\n", address);
            ++deviceCount;
        } else if (error == 4) {
            serial.printf("Unknown error at address 0x%02x\n", address);
        }
    }

    if (deviceCount == 0)
        serial.println("No I2C devices found\n");
    else
        serial.printf("Found %d I2C devices\n", deviceCount);
}

// Wipes the EEPROM.
void wipeEEPROM() {
    Serial.println("Wiping EEPROM...");
    for (int i = 0; i < EEPROM.length(); i++) {
        EEPROM.write(i, 0);
        Serial.printf("%4d / %d\n", i, EEPROM.length());
    }
    Serial.println("Done");
}

// Global variables for printLoopTime()
uint32_t _time = 0;
uint32_t _min_delta = UINT32_MAX;
uint32_t _max_delta = 0;
uint64_t _sum_delta = 0;
uint16_t _iter = 0;

// Get loop time in µs.
uint32_t printLoopTime(Stream &serial, uint16_t sampleCount) {
    const auto now = micros();
    const auto delta = micros() - _time;
    _time = now;
    _min_delta = min(_min_delta, delta);
    _max_delta = max(_max_delta, delta);
    _sum_delta += delta;
    ++_iter;

    if (_iter == sampleCount) {
        serial.printf(
            "Loop time (µs): mean=%5u (min=%5u, max=%5u), samples=%4u\n",
            (uint32_t)roundf((double)_sum_delta / (double)_iter), _min_delta,
            _max_delta, _iter);
        _time = micros(); // Update time as Serial.print() took some time
        _min_delta = UINT32_MAX;
        _max_delta = 0;
        _sum_delta = 0;
        _iter = 0;
    }

    return delta;
}

// Prints a double to serial with a specified number of integer and decimal
// places. We need this because the Arduino library's Serial.printf() doesn't
// support floats.
void printDouble(Stream &serial, double value, uint8_t integerPlaces,
                 uint8_t decimalPlaces) {
    const auto integerComponent = (int)value;
    const auto decimalComponent =
        (int)round(abs(value * pow(10, decimalPlaces))) %
        (int)pow(10, decimalPlaces);

    if (integerPlaces == 0)
        serial.printf("%d", integerComponent);
    else
        serial.printf("%*d", integerPlaces, integerComponent);
    serial.print(".");
    if (decimalPlaces == 0)
        serial.printf("%d", decimalComponent);
    else
        serial.printf("%0*d", decimalPlaces, decimalComponent);
}
