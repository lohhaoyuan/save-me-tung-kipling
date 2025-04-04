#include "main.h"
#include "config.h"
#include "movement.h"
#include <Arduino.h>

PacketSerial EncoderPacketSerial;
PacketSerial LightPacketSerial;
PacketSerial TopPacketSerial;

Movement seggs;
Sensors sensors;

#define HEADING_CALIBRATION
void setup() {
    Serial.begin(115200);

    LightSerial.begin(115200);
    LightPacketSerial.setStream(&LightSerial);
    LightPacketSerial.setPacketHandler(&LightPacketHandler);

    TopSerial.begin(SHARED_BAUD_RATE);
    TopPacketSerial.setStream(&TopSerial);
    TopPacketSerial.setPacketHandler(&TopPacketHandler);

    EncoderSerial.begin(SHARED_BAUD_RATE);
    EncoderPacketSerial.setStream(&EncoderSerial);

    seggs.init();
    // setupDribblers();
}

void loop() {
    LightPacketSerial.update();
    TopPacketSerial.update();
    Serial.print("Line angle bisector");
    Serial.print(sensors.line.angleBisector);
    Serial.print(" Depth ");
    Serial.println(sensors.line.depth);
    delay(1000);
    //     seggs.updateHeading(sensors.yaw);
    // #ifdef HEADING_CALIBRATION
    //     seggs.setHeading((Heading::Constant){0});
    //     seggs.setVelocity((Velocity::Constant){0});
    //     seggs.setDirection((Direction::Constant){0});
    // #endif
    //     seggs.drive();
    // Serial.println(sensors.yaw);
    // if (LightSerial.available()>0){
    //     Serial.write(LightSerial.read());
    // }
}