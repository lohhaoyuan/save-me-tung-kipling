#include <Arduino.h>
#include "main.h"
#include "config.h"
#include "movement.h"

PacketSerial EncoderPacketSerial;
PacketSerial LightPacketSerial;
PacketSerial TopPacketSerial;
Movement seggs;
Sensors sensors;

void setup(){
    Serial.begin(115200);


    // LightSerial.begin(115200);
    // LightPacketSerial.setStream(&LightSerial);
    // LightPacketSerial.setPacketHandler(&LightPacketHandler);

    TopSerial.begin(SHARED_BAUD_RATE);
    TopPacketSerial.setStream(&TopSerial);
    TopPacketSerial.setPacketHandler(&TopPacketHandler);

    EncoderSerial.begin(SHARED_BAUD_RATE);
    EncoderPacketSerial.setStream(&EncoderSerial);
    
    seggs.init();
    // setupDribblers();
}

void loop(){
    TopPacketSerial.update();
    seggs.updateHeading(sensors.yaw);
    seggs.setHeading((Heading::Constant){0});
    seggs.setVelocity((Velocity::Constant){200});
    seggs.setDirection((Direction::Constant){0});    
    seggs.drive();
    Serial.println(sensors.yaw);
    // if (LightSerial.available()>0){
    //     Serial.write(LightSerial.read());
    // }

}