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
    SubSerial.print("ping from top serial");
    if(SubSerial.available()>0){
        Serial.write(SubSerial.read());
    }    
}