#include "main.h"

BluetoothPayloadUnion outgoingBluetoothPayload;
BluetoothPayloadUnion incomingBluetoothPayload;
void TeensyPacketHandler(const uint8_t *buf , size_t size){
    BluetoothPayloadUnion incomingTeensyPayload;
    if (size != sizeof(incomingTeensyPayload)){
        return;
    }
    memcpy(incomingTeensyPayload.bytes, buf, sizeof(incomingTeensyPayload));
    outgoingBluetoothPayload.data.ball = incomingTeensyPayload.data.ball;
    outgoingBluetoothPayload.data.ballExists = incomingTeensyPayload.data.ballExists;
    outgoingBluetoothPayload.data.robotLive = incomingTeensyPayload.data.robotLive;
}

void setup(){
    Serial.begin(MONITOR_BAUD_RATE);
    TeensySerial.begin(SHARED_BAUD_RATE);
    TeensyPacketSerial.setStream(&TeensySerial);
    #ifdef STRIKER
        TeensyPacketSerial.setPacketHandler(TeensyPacketHandler);
    #endif
}

