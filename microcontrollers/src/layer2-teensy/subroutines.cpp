#include "main.h"

#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <ArduinoEigenDense.h>
#include <SPI.h> // necessary for Adafruit_BNO055.h dependencies to install

#include "config.h"
#include "shared.h"

void checkTeensyReset() {
    // Check if the reset button is pressed
    if (!digitalRead(PIN_SOFTWARE_RESET)) {
        // Reset the Teensy
        Serial.println("Resetting Teensy");
        SCB_AIRCR = 0x05FA0004;
    }
}

void setMode() {
    // Prepare payload to set ESP modes
    L1ESPRxPayloadUnion l1Payload;
    L2ESPRxPayloadUnion l2Payload;
    switch (TEENSY_MODE) {
    case TeensyMode::Default:
        l1Payload.data.mode = L1ESPMode::Default;
        l2Payload.data.mode = L2ESPMode::Default;
        break;
    case TeensyMode::PrintLightRing:
        l1Payload.data.mode = L1ESPMode::PrintLightRing;
        break;
    case TeensyMode::PrintIRRing:
        l2Payload.data.mode = L2ESPMode::PrintIRRing;
        break;
    case TeensyMode::PrintL1ESPLoopTime:
        l1Payload.data.mode = L1ESPMode::PrintLoopTime;
        break;
    case TeensyMode::PrintL2ESPLoopTime:
        l2Payload.data.mode = L2ESPMode::PrintLoopTime;
        break;
    case TeensyMode::CalibrateLightRing:
        l1Payload.data.mode = L1ESPMode::CalibrateLightRing;
        break;
    case TeensyMode::CalibrateIRRing:
        l2Payload.data.mode = L2ESPMode::CalibrateIRRing;
        break;
    }
    // Send payload to ESPs
    L1ESPPacketSerial.send(l1Payload.bytes, sizeof(l1Payload.bytes));
    L2ESPPacketSerial.send(l2Payload.bytes, sizeof(l2Payload.bytes));

    // If we are debugging or calibrating, enter another loop instead of the
    // default loop
    switch (TEENSY_MODE) {
    case TeensyMode::PrintLightRing:
    case TeensyMode::PrintL1ESPLoopTime:
    case TeensyMode::CalibrateLightRing:
        // Redirect serial output from L1 ESP
        Serial.println("Redirecting serial output from L1 ESP");
        while (1) {
            checkTeensyReset();
            // TODO: This line crashes the L1 ESP?
            // L1ESPPacketSerial.send(l1Payload.bytes, sizeof(l1Payload.bytes));
            while (L1ESPSerial.available()) Serial.write(L1ESPSerial.read());
        }
    case TeensyMode::PrintIRRing:
    case TeensyMode::PrintL2ESPLoopTime:
    case TeensyMode::CalibrateIRRing:
        // Redirect serial output from L2 ESP
        Serial.println("Redirecting serial output from L2 ESP");
        while (1) {
            checkTeensyReset();
            L2ESPPacketSerial.send(l2Payload.bytes, sizeof(l2Payload.bytes));
            while (L2ESPSerial.available()) Serial.write(L2ESPSerial.read());
        }
    default:
        break;
    }
}

// Global variables for IMU
Adafruit_BNO08x bno;
Eigen::Quaterniond initialRotationOffset = Eigen::Quaterniond::Identity();

void setupIMU() {
    // Initialise IMU
#ifdef NEW_BOT
    if (!bno.begin_I2C()) {
#else
    IMUSerial.begin(115200);
    if (!bno.begin_UART(&IMUSerial)) {
#endif
        Serial.println("Failed to find BNO085!!");
        // Blink the debug LED on failure
        while (1) {
            checkTeensyReset();
            digitalWrite(PIN_LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(PIN_LED_BUILTIN, LOW);
            delay(100);
        }
    }
    // TODO: try SH2_ROTATION_VECTOR, SH2_GYRO_INTEGRATED_RV
    // The SH2_GAME_ROTATION_VECTOR mode uses fusion of gyroscope and
    // accelerometer data to determine a rotation vector in quaternions
    if (!bno.enableReport(SH2_GAME_ROTATION_VECTOR)) {
        Serial.println("Could not enable game rotation vector!!");
        // Blink the debug LED on failure
        while (1) {
            checkTeensyReset();
            digitalWrite(PIN_LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(PIN_LED_BUILTIN, LOW);
            delay(100);
        }
    }
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

            break;
        }
        default:
            break;
        }
    }

    return NAN;
}

void moveBehindBall() {
    // We can't just move straight at the ball. We need to move
    // behind it in a curve. Here, we calculate the angle offset we
    // need to achieve this.
    auto angleOffset =
        // The angle offset is directly related to the angle of the
        // ball, but we constrain it to 90º because the robot should
        // never move in a range greater than 90º away from the
        // ball, as it would be moving away from the ball.
        constrain(sensors.ball.angle, -90, 90) *
        // The angle offset undergoes exponential decay. As the ball
        // gets closer to the robot, the robot moves more directly
        // at the ball.
        fmin(powf(exp(1), BALL_MOVEMENT_DECAY * (BALL_MOVEMENT_MAX_CURVE -
                                                 sensors.ball.distance)),
             BALL_MOVEMENT_MAX_MULTIPLIER);

    // Then, we pack it into instructions for our update function.
    movement.setDirection(
        (Direction::Constant){sensors.ball.angle + angleOffset});
    movement.setHeading((Heading::Constant){
        // Try to keep straight as much as possible to ensure the robot has to
        // leave the field the least if the ball is near the boundary.
        sensors.ball.distance <= BALL_MOVEMENT_FACE_BALL_DISTANCE
            ? constrain(sensors.ball.angle, -BALL_MOVEMENT_MAX_HEADING,
                        BALL_MOVEMENT_MAX_HEADING)
            : 0});
    movement.setVelocity((Velocity::Constant){Movement::applySigmoid(
        BALL_MOVEMENT_START_SPEED, BALL_MOVEMENT_END_SPEED,
        (sensors.ball.distance - BALL_MOVEMENT_STOP_DECELERATING) /
            (BALL_MOVEMENT_START_DECELERATING -
             BALL_MOVEMENT_STOP_DECELERATING))});
}

void avoidLine() {
    const auto x = sensors.robotPosition.x();
    const auto y = sensors.robotPosition.y();
    const auto leftMargin = max(FIELD_WIDTH / 2 + x, 0);
    const auto rightMargin = max(FIELD_WIDTH / 2 - x, 0);
    const auto topMargin = max(FIELD_LENGTH / 2 + y, 0);
    const auto bottomMargin = max(FIELD_LENGTH / 2 - y, 0);
    const auto xMargin = min(leftMargin, rightMargin);
    const auto yMargin = min(topMargin, bottomMargin);

    if (sensors.line.exists()) {
        if (sensors.line.depth > WALL_AVOIDANCE_THRESHOLD) {
            // We're too far into the line, move away quickly
            movement.setDirection(
                (Direction::Constant){sensors.line.angleBisector});
            movement.setVelocity((Velocity::Constant){
                sensors.line.depth * WALL_AVOIDANCE_SPEED_MULTIPLIER});
#ifdef DEBUG
            Serial.print(" AVOID LINE");
#endif
            return;
        } else if (sensors.ball.exists() && !sensors.robotHasBall) {
            // Do another check to see if the ball has rolled out
            bool ballRolledOut;
            if (yMargin < xMargin) {
                ballRolledOut = topMargin < bottomMargin
                                    ? abs(sensors.ball.angle) <= 90
                                    : abs(sensors.ball.angle) >= 90;
            } else {
                ballRolledOut = leftMargin < rightMargin
                                    ? sensors.ball.angle <= 0
                                    : sensors.ball.angle >= 0;
            }

            if (!ballRolledOut) {
                // We're reasonably within the line, so let's try to line track
                // towards the ball
                const auto ballError =
                    xMargin < yMargin ? sensors.ball.y() : sensors.ball.x();
                const auto trackLeftwards = (x < 0) ^ (ballError < 0);
                movement.setDirection((Direction::LineTrack){
                    sensors.line, BALL_LINE_TRACK_TARGET, trackLeftwards});

                // Always face the ball
                const auto absoluteBallAngle =
                    sensors.ball.angle + sensors.robotAngle;
                movement.setHeading((Heading::Constant){
                    constrain(absoluteBallAngle, -BALL_LINE_TRACK_MAX_HEADING,
                              BALL_LINE_TRACK_MAX_HEADING)});

                // Slow down as we approach the ball
                if (xMargin < yMargin) {
                    // Ball is to the side of the robot, stop slightly behind
                    // ball
                    movement.setVelocity((Velocity::StopAtPoint){
                        -(sensors.ball.y() + BALL_LINE_TRACK_BEHIND_BALL_BY),
                        150, 300});
                } else {
                    // Ball is in front of the robot, stop directly behind ball
                    movement.setVelocity(
                        (Velocity::StopAtPoint){sensors.ball.x(), 150, 300});
                }
#ifdef DEBUG
                Serial.print(" BALL IN");
#endif
                // return; // we can skip the speed limit :D
#ifdef DEBUG
            } else {
                Serial.print(" BALL OUT");
#endif
            }
        }
    }

#ifndef SUPERTEAM
    if (limitSpeed && min(xMargin, yMargin) <= SPEED_LIMIT_START) {
        // We're near the line, so let's slow down regardless
        movement.setVelocity((Velocity::Limit){
            Movement::applySigmoid(SPEED_LIMIT_START_SPEED,
                                   SPEED_LIMIT_END_SPEED,
                                   (xMargin - SPEED_LIMIT_END) /
                                       (SPEED_LIMIT_START - SPEED_LIMIT_END)),
            Movement::applySigmoid(SPEED_LIMIT_START_SPEED,
                                   SPEED_LIMIT_END_SPEED,
                                   (yMargin - SPEED_LIMIT_END) /
                                       (SPEED_LIMIT_START - SPEED_LIMIT_END)),
        });
    #ifdef DEBUG
        Serial.print(" L");
    } else {
        Serial.print("  ");
    #endif
    }
#endif
}
