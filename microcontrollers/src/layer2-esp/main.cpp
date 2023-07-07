#include "main.h"

#include <Arduino.h>
#include <PacketSerial.h>
#ifdef NEW_BOT
    #include <SoftwareSerial.h>
#endif
#include <deque>
#include <numeric>
#include <tuple>

#include "shared.h"
#include "util.h"

// Global variables
PacketSerial TeensyPacketSerial;
L2ESPMode mode;

void onTeensyPacket(const byte *buf, size_t size) {
    // Read the payload
    L2ESPRxPayloadUnion payload;
    // Don't continue if the payload is invalid
    if (size != sizeof(payload)) return;
    std::copy(buf, buf + size, std::begin(payload.bytes));

    // Handle the payload
    mode = payload.data.mode;
}

void setup() {
    // Turn the LED on until setup is complete
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // Initialise serial
    TeensySerial.begin(SHARED_BAUD_RATE);
    TeensyPacketSerial.setStream(&TeensySerial);
    TeensyPacketSerial.setPacketHandler(&onTeensyPacket);

    // Initialise IR sensing
    setupIRRing();

    // Turn the LED off
    digitalWrite(PIN_LED, LOW);
}

// This is the loop function in the default mode
void defaultLoop(bool sendPacket = true) {
    // Read the ball angle and strength (note: this is blocking)
    double ballAngle, ballDistance, ballStrength;
    std::tie(ballAngle, ballDistance, ballStrength) = readIRRing();

    // Package data into a payload object and send it out to the Teensy
    L2ESPTxPayloadUnion txPayload;
    txPayload.data.ball.angle = ballAngle;
    txPayload.data.ball.distance = ballDistance;
    TeensyPacketSerial.send(txPayload.bytes, sizeof(txPayload.bytes));
}

void loop() {
    // Read serial packets
    TeensyPacketSerial.update();

    // The Teensy can set this ESP to different modes
    switch (mode) {
    case L2ESPMode::Default:
        defaultLoop();
        break;
    case L2ESPMode::PrintIRRing:
        readIRRing(true);
        break;
    case L2ESPMode::PrintLoopTime:
        defaultLoop(false);
        printLoopTime(TeensySerial, 50);
        break;
    case L2ESPMode::CalibrateIRRing:
        printIRRingCalibration();
        break;
    }
}
