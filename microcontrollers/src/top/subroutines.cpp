#include "main.h"

// Localisaation
ddouble ballMapping(double distance) {
    if (distance != 500) {
        return (3.41295982 * powf(10, -7) * powf(distance, 5) -
                1.68660902 * powf(10, -4) * powf(distance, 4) +
                3.30063050 * powf(10, -2) * powf(distance, 3) -
                3.18336643 * powf(10, 0) * powf(distance, 2) +
                1.51464368 * powf(10, 2) * distance -
                2.83727898 * powf(10, 3));
    } else
        return 0;
}

double goalMapping(double distance) {
    if (distance != 500) {
        return (5.85589344 * powf(10, -7) * powf(distance, 5) -
                3.43829641 * powf(10, -4) * powf(distance, 4) +
                8.05109171 * powf(10, -2) * powf(distance, 3) -
                9.37413014 * powf(10, 0) * powf(distance, 2) +
                5.42735448 * powf(10, 2) * distance -
                1.24927556 * powf(10, 4));
    } else
        return 0;
}


void BTPacketHandler(const uint8_t* buf, size_t size) {
    if (size != sizeof(BluetoothPayloadUnion)) return;
    BluetoothPayloadUnion payload;
    memcpy(payload.bytes, buf, sizeof(BluetoothPayloadUnion));
    sensors.bt_ball = payload.data.ball;
    sensors.TeammateLive = payload.data.robotLive;
}

void CameraPacketHandler(const uint8_t* buf, size_t size){
    if (size != sizeof(CameraPayloadUnion)) return;
    CameraPayloadUnion payload;
    memcpy(payload.bytes, buf, sizeof(payload));
    #ifdef YELLOW_ATTACK
    sensors.blue.angle =
        payload.data.data[0];
    sensors.blue.distance =
        payload.data.data[1];
    sensors.yellow.angle =
        payload.data.data[2];
    sensors.yellow.distance =
        payload.data.data[3];
    #endif
    #ifdef BLUE_ATTACK
    sensors.yellow.angle =
        payload.data.data[0];
    sensors.yellow.distance =
        payload.data.data[1];
    sensors.blue.angle =
        payload.data.data[2];
    sensors.blue.distance =
        payload.data.data[3];
    #endif
    sensors.cam_ball.angle = payload.data.data[4];
    sensors.cam_ball.distance = payload.data.data[5];
    sensors.cam_ball.distance =
        ballMirrorRegress(sensors.cam_ball.distance);
    sensors.blue.distance = goalMirrorRegress(sensors.blue.distance);
    sensors.yellow.distance = goalMirrorRegress(sensors.yellow.distance);
    
    sensors.blue.angle = clipAngleTo180(sensors.blue.angle);
    sensors.yellow.angle = clipAngleTo180(sensors.yellow.angle);
    auto localizeWithBothGoals = []() {
        // We use an algorithm that allows us to minimise error propagated by
        // goal distance and rely more on goal angle

        // Computer a "real" center vector from the two goal vectors
        const auto fakeCenter =
            (sensors.blue + sensors.yellow) / 2;
        const auto scalingFactor =
            (((sensors.blue - fakeCenter).distance + (sensors.yellow = fakeCenter).distance)/2) /
            HALF_GOAL_SEPARATION;
        const auto realCenter = fakeCenter * scalingFactor;

        // Update robot position
        sensors.robot_position = -realCenter;
    };
    auto localizeWithYellowGoal = []() {
        // Compute a "fake" center vector from the goal vector
        const Vector realGoalToCenter = {180 - sensors.yaw,
                                         HALF_GOAL_SEPARATION};
        const auto fakeCenter = sensors.yellow + realGoalToCenter;
        // Update robot position
        sensors.robot_position = -fakeCenter;
    };
    auto localizeWithBlueGoal = []() {
        // Compute a "fake" center vector from the goal vector
        const Vector realGoalToCenter = {-sensors.yaw,
                                         HALF_GOAL_SEPARATION};
        const auto fakeCenter = sensors.blue + realGoalToCenter;

        // Update robot position
        sensors.robot_position = -fakeCenter;
    };

    // Try to determine the robot position
    if (sensors.yellow.exists() && sensors.blue.exists()) {
        if (sensors.yellow.distance <= 80 ||
            sensors.blue.distance <= 80) {
            // If a goal is close, the other goal is quite far away, it's
            // probably more accurate to just localise with the closer one
            if (sensors.yellow.distance <=
                sensors.blue.distance)
                localizeWithYellowGoal();
            else
                localizeWithBlueGoal();

        } else {
            // We can see both goals, so we can localise with that :D
            localizeWithBothGoals();
        }
    } else if (sensors.yellow.exists()) {
        // Compute a "fake" center vector from the offensive goal vector
        localizeWithYellowGoal();
    } else if (sensors.blue.exists()) {
        // Compute a "fake" center vector from the defensive goal vector
        localizeWithBlueGoal();
    } else {
        // We can't see any goals :(
        sensors.robot_position = {NAN, NAN};
    }

}

void LidarPacketHandler(const uint8_t* buf, size_t size){
    if (size != sizeof(LidarPayloadUnion)) return;
    LidarPayloadUnion payload;
    sensors.lidarDist[0] = payload.data.lidarDist[0];
    sensors.lidarDist[1] = payload.data.lidarDist[1];
    sensors.lidarDist[2] = payload.data.lidarDist[2];
    sensors.lidarDist[3] = payload.data.lidarDist[3];

    if (sensors.lidarDist[1] + sensors.lidarDist[3] > 110){
        return;
    }
}

Adafruit_BNO08x bno(IMU_RST);
Eigen::Quaterniond initialRotationOffset = Eigen::Quaterniond::Identity();

void setupIMU(){

    if (!bno.begin_SPI(IMU_CS, IMU_INT)) {
        Serial.println("Failed to find BNO08x chip");
        while(1){delay(10);}
    }
    Serial.println("BNO found");

    if (!bno.enableReport(SH2_GAME_ROTATION_VECTOR)) {
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