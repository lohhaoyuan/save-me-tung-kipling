#include <Arduino.h>
#include "stm32f103c_variant_generic.h"
#include "main.h"

PacketSerial TeensyPacketSerial;
std::array<uint8_t, PHOTODIODE_COUNT> photodiodeActivationCount; // default to 0
bool frontLG = false;
bool backLG = false;

void setup() {
    TeensySerial.begin(115200);
    // pinMode(FLAG_BACK, OUTPUT);
    // pinMode(FLAG_FRONT, OUTPUT);
    TeensyPacketSerial.setStream(&TeensySerial);
    analogReadResolution(12);
    for (auto mux : muxs) mux.init();
}

void loop(){
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

    #ifdef DEBUG
        printDouble(TeensySerial, line.first, 4, 2);
        TeensySerial.print("º ");
        printDouble(TeensySerial, line.second, 1, 2);
        TeensySerial.print(" ");
        printLightRing();
    #endif


    // Package data into a payload object and send it out to the Teensy
    LightTxPayloadUnion txPayload;
    txPayload.data.line.angleBisector = line.first;
    txPayload.data.line.depth = line.second;
    txPayload.data.frontLG = frontLG;
    txPayload.data.backLG = backLG;
    TeensyPacketSerial.send(txPayload.bytes, sizeof(txPayload.bytes));
    // for (int i = 256; i >0 ; i--){
    //     analogWrite(LED_PIN, i);
    //     delay(5);
    // }

}