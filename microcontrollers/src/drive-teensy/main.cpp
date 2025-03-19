#include "main.h"
#include "shared.h"
#include "util.h"

PacketSerial LightPacketSerial;
Sensors sensors;
void onLightPacket(const byte *buf, size_t size) {
    // Read the payload
    LightTxPayloadUnion payload;
    // Don't continue if the payload is invalid
    if (size != sizeof(payload)) return;
    std::copy(buf, buf + size, std::begin(payload.bytes));

    // Handle the payload
    sensors.line = payload.data.line;
}

void setup(){
    Serial.begin(MONITOR_BAUD_RATE);    
    LightSerial.begin(SHARED_BAUD_RATE);
    LightPacketSerial.setStream(&LightSerial);
    LightPacketSerial.setPacketHandler(&onLightPacket);
}

void loop(){
    // LightPacketSerial.update();
    // if (sensors.line.exists()) {
    //     Serial.printf(" | Line %4dº ", (int)sensors.line.angleBisector);
    //     printDouble(Serial, sensors.line.depth, 1, 2);
    // }
    if(LightSerial.available()>0){
        Serial.write(LightSerial.read());
    }
}


