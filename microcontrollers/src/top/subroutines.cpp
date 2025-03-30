#include "main.h"

// Localisaation
double ballMirrorRegress(double distance){
    return (1.91653705 * powf(10, -7) * powf(distance, 5) -
    6.07403517 * powf(10, -5) * powf(distance, 4) +
    7.34613337 * powf(10, -3) * powf(distance, 3) -
    4.15046723 * powf(10, -1) * powf(distance, 2) +
    1.12502900 * powf(10, 1) * distance -
    1.01032228 * powf(10, 2));
}

void CameraPacketHandler(const byte *buf, size_t size){
    CameraPayload payload;
    memcpy(&payload, buf, sizeof(payload));
    #ifdef YELLOW_ATTACK
    sensors.blue.angle =
        payload.data[0];
    sensors.blue.distance =
        payload.data[1];
    sensors.yellow.angle =
        payload.data[2];
    sensors.yellow.distance =
        payload.data[3];
    #endif
    #ifdef BLUE_ATTACK
    sensors.yellow.angle =
        payload.data[0];
    sensors.yellow.distance =
        payload.data[1];
    sensors.blue.angle =
        payload.data[2];
    sensors.blue.distance =
        payload.data[3];
    #endif
    sensors.cam_ball.angle = payload.data[4];
    sensors.cam_ball.distance = payload.data[5];
    sensors.cam_ball.distance =
        ballMirrorRegress(sensors.cam_ball.distance);
    
}

Vector centreVectorAtk(){
    const Vector realGoalToCenter = {0, 113.5};
    const Vector yellow_goalactualposition = {
        sensors.yellow.angle -
            sensors.yaw,
        sensors.yellow.distance};
    const auto fakeCenter = - yellow_goalactualposition + realGoalToCenter;
    Vector actualCenter = {clipAngleTo180(fakeCenter.angle),
                           fakeCenter.distance};
    return actualCenter;
}
Vector centreVectorDef(){
    const Vector realGoalToCenter = {0, 113.5};
    const Vector yellow_goalactualposition = {
        sensors.yellow.angle -
            sensors.yaw,
        sensors.yellow.distance};
    const auto fakeCenter = - yellow_goalactualposition + realGoalToCenter;
    Vector actualCenter = {clipAngleTo180(fakeCenter.angle),
                           fakeCenter.distance};
    return actualCenter;
}

Vector centreVectorBoth(){
    return (centreVectorAtk() + centreVectorDef()) / 2;
}
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

