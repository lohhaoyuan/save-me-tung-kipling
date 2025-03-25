#include "main.h"

// Packet Handlers
// void CameraPacketHandler(const byte *buf, size_t size){
//     CameraPayload payload;
//     memcpy(&payload, buf, sizeof(payload));
//     #ifdef BLUE
//     sensors.blue.angle =
//         payload.data[0];
//     sensors.blue.distance =
//         payload.data[1];
//     sensors.yellow.angle =
//         payload.data[2];
//     sensors.yellow.distance =
//         payload.data[3];
//     #endif
//     #ifdef YELLOW
//     sensors.yellow.angle =
//         payload.data[0];
//     sensors.yellow.distance =
//         payload.data[1];
//     sensors.blue.angle =
//         payload.data[2];
//     sensors.blue.distance =
//         payload.data[3];
//     #endif
//     sensors.cam_ball.angle = payload.data[4];
//     sensors.cam_ball.distance = payload.data[5];

// }

// IMU 
Adafruit_BNO08x bno(IMU_RST);
Eigen::Quaterniond initialRotationOffset = Eigen::Quaterniond::Identity();

void setupIMU(){

    if (!bno.begin_SPI(IMU_CS, IMU_INT)) {
        Serial.println("Failed to find BNO08x chip");
        while(1){delay(10);}
    }
    Serial.println("BNO found");

    if (! bno.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        Serial.println("Could not enable game vector");
        while(1){delay(10);}
    }   
    Serial.println("Game vector enabled");
    Serial.println("Reports set");
}
void setIMUReports(){

    Serial.println("Setting desired reports");
    

      
}

double readIMUHeading() {
    sh2_SensorValue_t bnoValue;
    if (bno.getSensorEvent(&bnoValue)) {
        switch (bnoValue.sensorId) {
        case SH2_GAME_ROTATION_VECTOR: {
            const Eigen::Quaterniond rotation = {
                bnoValue.un.gameRotationVector.real,
                bnoValue.un.gameRotationVector.i,
                bnoValue.un.gameRotationVector.j,
                bnoValue.un.gameRotationVector.k,
            };
            
            // Set the initial offset if it hasn't been set
            if (initialRotationOffset == Eigen::Quaterniond::Identity())
                initialRotationOffset = rotation.inverse();

            // Compute the robot angle
            const auto correctedRotation = initialRotationOffset * rotation;
            const auto rotationMatrix = correctedRotation.toRotationMatrix();
            const auto yaw = -atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
            return degrees(yaw);
        }
        default:
            break;
        }
    }
    return NAN;
}