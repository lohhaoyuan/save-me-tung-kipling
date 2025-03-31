#include "main.h"
#include "stm32f4.h"
#include "enc.h"
PacketSerial TeensyPacketSerial;

void setup(){
    pinMode(LED_PIN, OUTPUT);
    pinMode(FL_DIR, OUTPUT);
    pinMode(FR_DIR, OUTPUT);
    pinMode(BL_DIR, OUTPUT);
    pinMode(BR_DIR, OUTPUT);

    pinMode(FL_PWM, OUTPUT);
    pinMode(FR_PWM, OUTPUT);
    pinMode(BL_PWM, OUTPUT);
    pinMode(BR_PWM, OUTPUT);

    TeensySerial.begin(SHARED_BAUD_RATE);
    TeensyPacketSerial.setStream(&TeensySerial);
    TeensyPacketSerial.setPacketHandler(&TeensyPacketHandler);


    for (uint8_t i = 0; i < 4; i++) {
    pinMode(encoderPins[i][0], INPUT_PULLUP);
    pinMode(encoderPins[i][1], INPUT_PULLUP); 
    }

    // Attach interrupts for each encoder pin
    attachInterrupt(digitalPinToInterrupt(encoderPins[0][0]), encoder1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPins[0][1]), encoder1ISR, CHANGE);

    attachInterrupt(digitalPinToInterrupt(encoderPins[1][0]), encoder2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPins[1][1]), encoder2ISR, CHANGE);

    attachInterrupt(digitalPinToInterrupt(encoderPins[2][0]), encoder3ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPins[2][1]), encoder3ISR, CHANGE);

    attachInterrupt(digitalPinToInterrupt(encoderPins[3][0]), encoder4ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPins[3][1]), encoder4ISR, CHANGE);


}

void loop(){
    // updateRPM();
    // delay(TIMER_INTERVAL);
    // TeensyPacketSerial.update();
    // drive(200,200,200,200);
    for (int i = 0; i < 256; i++) {
        analogWrite(LED_PIN, i);
        delay(5);
    }
}