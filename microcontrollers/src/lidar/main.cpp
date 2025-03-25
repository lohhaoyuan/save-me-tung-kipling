#include "main.h"

void setup(){
    Serial.begin(MONITOR_BAUD_RATE);
    TeensySerial.begin(115200);
}

void loop(){
    TeensySerial.print("FUCK");
}