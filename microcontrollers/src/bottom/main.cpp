#include <Arduino.h>
#include "main.h"
#include "config.h"
void setup(){
    Serial.begin(MONITOR_BAUD_RATE);

    // // LightSerial.begin(SHARED_BAUD_RATE);
    // // LightPacketSerial.setStream(&LightSerial);
    // // LightPacketSerial.setPacketHandler(&LightPacketHandler);

    // // TopSerial.begin(SHARED_BAUD_RATE);
    // // TopPacketSerial.setStream(&TopSerial);
    // // TopPacketSerial.setPacketHandler(&TopPacketHandler);

    // LightSerial.begin(115200);
    // // EncoderPacketSerial.setStream(&LightSerial);
    // // EncoderPacketSerial.setPacketHandler(&EncoderPacketHandler);

    
    setupDribblers();
}

void loop(){
    TopSerial.print("ping from bottom");
    if (TopSerial.available()>0){
        Serial.write(TopSerial.read());
    }
}