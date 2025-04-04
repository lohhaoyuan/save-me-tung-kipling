#include "main.h"

// Sensors sensors;
// PacketSerial BluetoothPacketSerial;
// PacketSerial LidarPacketSerial;
// PacketSerial CameraPacketSerial;
// PacketSerial SubPacketSerial;



void setup(){
    Serial.begin(MONITOR_BAUD_RATE);
    // BluetoothSerial.begin(SHARED_BAUD_RATE);
    // SubSerial.begin(SHARED_BAUD_RATE);
    // LidarSerial.begin(SHARED_BAUD_RATE);
    // CameraSerial.begin(SHARED_BAUD_RATE);

    // BluetoothPacketSerial.setStream(&BluetoothSerial);
    // BluetoothPacketSerial.setPacketHandler(&BTPacketHandler);

    // LidarPacketSerial.setStream(&LidarSerial);
    // LidarPacketSerial.setPacketHandler(&LidarPacketHandler);

    // CameraPacketSerial.setStream(&CameraSerial);
    // CameraPacketSerial.setPacketHandler(&CameraPacketHandler);
    
    // SubPacketSerial.setStream(&SubSerial);
    setupIMU();
}

void loop(){
    Serial.println(readIMUHeading());
}