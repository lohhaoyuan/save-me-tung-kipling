#include "main.h"
#include "stm32f4.h"
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
    digitalWrite(LED_PIN, HIGH);
  

}

void loop(){

    TeensyPacketSerial.update();
    // drive(speeds[0],speeds[1],speeds[2],speeds[3]);
    // drive(200,200,200,200);
    for (int i = 0; i < 256; i++) {
        analogWrite(LED_PIN, i);
        delay(5);
    }
}