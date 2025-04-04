#include "main.h"

Sensors sensors;
// PacketSerial BluetoothPacketSerial;
// PacketSerial LidarPacketSerial;
PacketSerial CameraPacketSerial;
PacketSerial SubPacketSerial;

void setup() {
    Serial.begin(115200);
    // BluetoothSerial.begin(SHARED_BAUD_RATE);
    SubSerial.begin(SHARED_BAUD_RATE);
    // LidarSerial.begin(SHARED_BAUD_RATE);
    CameraSerial.begin(115200);

    // BluetoothPacketSerial.setStream(&BluetoothSerial);
    // BluetoothPacketSerial.setPacketHandler(&BTPacketHandler);

    // LidarPacketSerial.setStream(&LidarSerial);
    // LidarPacketSerial.setPacketHandler(&LidarPacketHandler);

    CameraPacketSerial.setStream(&CameraSerial);
    CameraPacketSerial.setPacketHandler(&CameraPacketHandler);

    SubPacketSerial.setStream(&SubSerial);
    setupIMU();
    Serial.println(sizeof(CameraPayloadUnion));
}

void loop() {
    CameraPacketSerial.update();
    // SesbianLexPayloadUnion payload;
    // payload.data.yaw = readIMUHeading();
    // Serial.println(payload.data.yaw);
    // SubPacketSerial.send(payload.bytes, sizeof(SesbianLexPayload));
    Serial.print("x: ");
    Serial.println(sensors.robot_position.x());
    Serial.print("y: ");
    Serial.println(sensors.robot_position.y());
    delay(1000);
}