#include "main.h"

void setup(){
    Serial.begin(MONITOR_BAUD_RATE);
    BluetoothSerial.begin(SHARED_BAUD_RATE);
    SubSerial.begin(SHARED_BAUD_RATE);
    LidarSerial.begin(SHARED_BAUD_RATE);
    CameraSerial.begin(SHARED_BAUD_RATE);

    // BluetoothPacketSerial.setStream(&BluetoothSerial);
    // SubPacketSerial.setStream(&SubSerial);
    // LidarPacketSerial.setStream(&LidarSerial);
    // // CameraPacketSerial.setStream(&CameraSerial);
    // CameraPacketSerial.setPacketHandler(&CameraPacketHandler);

    setupIMU();
}

void loop(){
    if(LidarSerial.available()>0){
        Serial.print(LidarSerial.read());
    }    
}