#include "main.h"

Sensors sensors;
PacketSerial BluetoothPacketSerial;
PacketSerial SubPacketSerial;
PacketSerial LidarPacketSerial;
PacketSerial CameraPacketSerial;



void setup(){
    Serial.begin(MONITOR_BAUD_RATE);
    BluetoothSerial.begin(SHARED_BAUD_RATE);
    SubSerial.begin(SHARED_BAUD_RATE);
    LidarSerial.begin(SHARED_BAUD_RATE);
    CameraSerial.begin(SHARED_BAUD_RATE);

    // BluetoothPacketSerial.setStream(&BluetoothSerial);
    // SubPacketSerial.setStream(&SubSerial);
    // LidarPacketSerial.setStream(&LidarSerial);
    CameraPacketSerial.setStream(&CameraSerial);
    CameraPacketSerial.setPacketHandler(&CameraPacketHandler);
    setupIMU();
}

void loop(){
    delay(100);
}