#include "main.h"

#include <Arduino.h>
#include <PacketSerial.h>
#include <array>

#include "shared.h"
#include "util.h"

// Global variables
PacketSerial TeensyPacketSerial;
L1ESPMode mode;
std::array<uint8_t, PHOTODIODE_COUNT> photodiodeActivationCount; // default to 0



void setup() {
    // Turn the LED on until setup is complete
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // Initialise serial
    TeensySerial.begin(SHARED_BAUD_RATE);
    TeensyPacketSerial.setStream(&TeensySerial);

    // Initialise the light ring
    analogReadResolution(12);
    for (auto mux : muxs) mux.init();



    // Turn the LED off
    digitalWrite(PIN_LED, LOW);
}

// This is the loop function in the default mode
void defaultLoop(bool sendPacket = true, bool debugPrint = false) {
    // Figure out which photodiodes are above the line
    std::array<bool, PHOTODIODE_COUNT> photodiodeOnLine;
    for (uint8_t i = 0; i < PHOTODIODE_COUNT; ++i) {
        const auto value = readPhotodiode(i);
        if (value > PHOTODIODE_THRESHOLDS[i])
            ++photodiodeActivationCount[i];
        else
            photodiodeActivationCount[i] = 0;

        photodiodeOnLine[i] =
            photodiodeActivationCount[i] > PHOTODIODE_ACTIVATION_THRESHOLD;
    }

    // Compute the position of the line relative to the robot
    auto line = findLine(photodiodeOnLine);
    // Account for which side of the line the robot is on
    line = adjustForLineSide(line);

    if (debugPrint) {
        printDouble(TeensySerial, line.first, 4, 2);
        TeensySerial.print("º ");
        printDouble(TeensySerial, line.second, 1, 2);
        TeensySerial.print(" ");
        printLightRing();
    }

    if (sendPacket) {
        // Package data into a payload object and send it out to the Teensy
        LightTxPayloadUnion txPayload;
        txPayload.data.line.angleBisector = line.first;
        txPayload.data.line.depth = line.second;
        TeensyPacketSerial.send(txPayload.bytes, sizeof(txPayload.bytes));
    }
}

void loop() {
    mode = L1ESPMode::CalibrateLightRing;

    // The Teensy can set this ESP to different modes
    switch (mode) {
    case L1ESPMode::Default:
        defaultLoop();
        break;
    case L1ESPMode::PrintLightRing:
        defaultLoop(false, true);
        break;
    case L1ESPMode::PrintLoopTime:
        defaultLoop(false, false);
        printLoopTime(TeensySerial, 50);
        break;
    case L1ESPMode::CalibrateLightRing:
        printLightRingCalibration();
        break;
    }
    TeensySerial.println(readPhotodiode(0));
    delay(1000);    
}
