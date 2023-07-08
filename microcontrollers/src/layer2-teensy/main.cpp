#include "main.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <PacketSerial.h>
#include <algorithm>
#include <cmath>
#include <iterator>

#include "config.h"
#include "movement.h"
#include "shared.h"
#include "util.h"

// Global variables
PacketSerial L1ESPPacketSerial;
PacketSerial L2ESPPacketSerial;
PacketSerial CameraPacketSerial;
bool limitSpeed = true;
bool faceYellowGoal = FACE_YELLOW_GOAL;
bool isGoalie = IS_GOALIE;
// Sensor values
struct Sensors sensors;
// Movement output
Movement movement;

void onL1ESPPacket(const byte *buf, size_t size) {
    // Read the payload
    L1ESPTxPayloadUnion payload;
    // Don't continue if the payload is invalid
    if (size != sizeof(payload)) return;
    std::copy(buf, buf + size, std::begin(payload.bytes));

    // Handle the payload
    sensors.line = payload.data.line;
}

void onL2ESPPacket(const byte *buf, size_t size) {
    // Read the payload
    L2ESPTxPayloadUnion payload;
    // Don't continue if the payload is invalid
    if (size != sizeof(payload)) return;
    std::copy(buf, buf + size, std::begin(payload.bytes));

    // Handle the payload
    sensors.ball = payload.data.ball;
#ifdef BALL_ANGLE_OFFSET
    sensors.ball.angle += BALL_ANGLE_OFFSET;
#endif
}

void onCameraPacket(const byte *buf, size_t size) {
    // Read the payload
    CameraTxPayloadUnion payload;
    // Don't continue if the payload is invalid
    if (size != sizeof(payload)) return;
    std::copy(buf, buf + size, std::begin(payload.bytes));

    // Handle the payload
    if (faceYellowGoal) {
        sensors.offensiveGoal = payload.data.yellowGoal;
        sensors.defensiveGoal = payload.data.blueGoal;
    } else {
        sensors.offensiveGoal = payload.data.blueGoal;
        sensors.defensiveGoal = payload.data.yellowGoal;
    }

    sensors.offensiveGoal.angle = clipAngleTo180(sensors.offensiveGoal.angle);
    sensors.defensiveGoal.angle = clipAngleTo180(sensors.defensiveGoal.angle);

    auto localizeWithBothGoals = []() {
        // We use an algorithm that allows us to minimise error propagated by
        // goal distance and rely more on goal angle

        // Computer a "real" center vector from the two goal vectors
        const auto fakeCenter =
            (sensors.offensiveGoal + sensors.defensiveGoal) / 2;
        const auto scalingFactor =
            (sensors.offensiveGoal - fakeCenter).distance /
            HALF_GOAL_SEPARATION;
        const auto realCenter = fakeCenter * scalingFactor;

        // Update robot position
        sensors.robotPosition = -realCenter;
    };
    auto localizeWithOffensiveGoal = []() {
        // Compute a "fake" center vector from the goal vector
        const Vector realGoalToCenter = {180 - sensors.robotAngle,
                                         HALF_GOAL_SEPARATION};
        const auto fakeCenter = sensors.offensiveGoal + realGoalToCenter;

        // Update robot position
        sensors.robotPosition = -fakeCenter;
    };
    auto localizeWithDefensiveGoal = []() {
        // Compute a "fake" center vector from the goal vector
        const Vector realGoalToCenter = {-sensors.robotAngle,
                                         HALF_GOAL_SEPARATION};
        const auto fakeCenter = sensors.defensiveGoal + realGoalToCenter;

        // Update robot position
        sensors.robotPosition = -fakeCenter;
    };

    // Try to determine the robot position
    if (sensors.offensiveGoal.exists() && sensors.defensiveGoal.exists()) {
        if (sensors.offensiveGoal.distance <= 80 ||
            sensors.defensiveGoal.distance <= 80) {
            // If a goal is close, the other goal is quite far away, it's
            // probably more accurate to just localise with the closer one
            if (sensors.offensiveGoal.distance <=
                sensors.defensiveGoal.distance)
                localizeWithOffensiveGoal();
            else
                localizeWithDefensiveGoal();

        } else {
            // We can see both goals, so we can localise with that :D
            localizeWithBothGoals();
        }
    } else if (sensors.offensiveGoal.exists()) {
        // Compute a "fake" center vector from the offensive goal vector
        localizeWithOffensiveGoal();
    } else if (sensors.defensiveGoal.exists()) {
        // Compute a "fake" center vector from the defensive goal vector
        localizeWithDefensiveGoal();
    } else {
        // We can't see any goals :(
        sensors.robotPosition = {NAN, NAN};
    }
}

void setup() {
    // Initialise USB serial at the beginning
    Serial.begin(115200);

    // Turn the LED on until setup is complete
    pinMode(PIN_LED_BUILTIN, OUTPUT);
    digitalWrite(PIN_LED_BUILTIN, HIGH);

    // Initialise reset buton
    pinMode(PIN_SOFTWARE_RESET, INPUT);
#ifdef NEW_BOT
    pinMode(PIN_DIP_GOAL, INPUT_PULLUP);
    pinMode(PIN_DIP_PLAYSTYLE, INPUT_PULLUP);
#endif

    // Initialise serial
    L1ESPSerial.begin(SHARED_BAUD_RATE);
    L2ESPSerial.begin(SHARED_BAUD_RATE);
    CameraSerial.begin(SHARED_BAUD_RATE);
    L1ESPPacketSerial.setStream(&L1ESPSerial);
    L2ESPPacketSerial.setStream(&L2ESPSerial);
    CameraPacketSerial.setStream(&CameraSerial);
    if (TEENSY_MODE == TeensyMode::Default) {
        L1ESPPacketSerial.setPacketHandler(&onL1ESPPacket);
        L2ESPPacketSerial.setPacketHandler(&onL2ESPPacket);
        CameraPacketSerial.setPacketHandler(&onCameraPacket);
    }
    Serial.println("Initialised serial");

    // Configure pin resolutions
    analogReadResolution(12);  // Used by lightgate
    analogReadAveraging(64);   // Used by lightgate
    analogWriteResolution(10); // Used by motor driver

    // Initialise movement devices
    movement.init();
    Serial.println("Initialised movement devices");

    // while (1) {
    //     digitalWrite(PIN_MOTOR_FL_DIR, LOW);
    //     digitalWrite(PIN_MOTOR_FR_DIR, LOW);
    //     digitalWrite(PIN_MOTOR_BL_DIR, LOW);
    //     digitalWrite(PIN_MOTOR_BR_DIR, LOW);
    //     analogWrite(PIN_MOTOR_FL_PWM, 200);
    //     analogWrite(PIN_MOTOR_FR_PWM, 200);
    //     analogWrite(PIN_MOTOR_BL_PWM, 200);
    //     analogWrite(PIN_MOTOR_BR_PWM, 200);
    //     delay(1000);
    //     digitalWrite(PIN_MOTOR_FL_DIR, HIGH);
    //     digitalWrite(PIN_MOTOR_FR_DIR, HIGH);
    //     digitalWrite(PIN_MOTOR_BL_DIR, HIGH);
    //     digitalWrite(PIN_MOTOR_BR_DIR, HIGH);
    //     analogWrite(PIN_MOTOR_FL_PWM, 200);
    //     analogWrite(PIN_MOTOR_FR_PWM, 200);
    //     analogWrite(PIN_MOTOR_BL_PWM, 200);
    //     analogWrite(PIN_MOTOR_BR_PWM, 200);
    //     delay(1000);
    // }

    // Initialise the IMU
    setupIMU();
    Serial.println("Initialised IMU");

    // Set the ESP and Teensy modes
    setMode();

    // Turn the LED off to indicate setup is complete
    digitalWrite(PIN_LED_BUILTIN, LOW);
    Serial.println("Completed default setup");
}

void loop() {
    checkTeensyReset();
#ifdef NEW_BOT
    faceYellowGoal = !digitalRead(PIN_DIP_GOAL);
    isGoalie = !digitalRead(PIN_DIP_PLAYSTYLE);
#endif

    // Read Teensy sensor inputs
    const auto heading = readIMUHeading();
    if (!std::isnan(heading)) sensors.robotAngle = heading;
    const auto lightgateReading = analogRead(PIN_LIGHTGATE);
    sensors.robotHasBall =
        lightgateReading LIGHTGATE_THRESHOLD_COMPARATOR LIGHTGATE_THRESHOLD;

    // Test motion
    // Serial.println(sensors.robotAngle);
    // // movement.updateHeading(sensors.robotAngle);
    // movement.headingController.debugPrint();
    // movement.setDirection((Direction::Constant){90});
    // movement.setHeading((Heading::Constant){0});
    // movement.setVelocity((Velocity::Constant){200});
    // movement.drive();
    // return;

    // Read serial packets
    L1ESPPacketSerial.update();
    L2ESPPacketSerial.update();
    CameraPacketSerial.update();

    // Override for robotHasBall because the ball bounces out
    if (abs(sensors.ball.angle + ROBOT_BALL_ANGLE_OFFSET) <=
            ROBOT_BALL_MAX_ANGLE &&
        sensors.ball.distance <= ROBOT_BALL_MIN_DISTANCE
#ifndef SUPERTEAM
        && smallerAngleDifference(sensors.ball.angle + ROBOT_BALL_ANGLE_OFFSET,
                                  sensors.offensiveGoal.angle) <=
               BALL_GOAL_ANGLE_THRESHOLD
#endif
    )
        sensors.robotHasBall = true;

    // Update movement controllers
    movement.updateHeading(sensors.robotAngle);

#ifdef DEBUG
    // Print debug information
    const auto x = sensors.robotPosition.x();
    const auto y = sensors.robotPosition.y();
    const auto leftMargin = constrain(FIELD_WIDTH / 2 + x, 0, FIELD_WIDTH);
    const auto rightMargin = constrain(FIELD_WIDTH / 2 - x, 0, FIELD_WIDTH);
    const auto frontMargin = constrain(FIELD_LENGTH / 2 - y, 0, FIELD_LENGTH);
    const auto backMargin = constrain(FIELD_LENGTH / 2 + y, 0, FIELD_LENGTH);
    // Print goal info
    Serial.print("Goal O ");
    if (sensors.offensiveGoal.exists()) {
        Serial.printf("%4dº %3d cm ", (int)sensors.offensiveGoal.angle,
                      (int)sensors.offensiveGoal.distance);
    } else {
        Serial.print("             ");
    }
    Serial.print(" D ");
    if (sensors.defensiveGoal.exists()) {
        Serial.printf("%4dº %3d cm ", (int)sensors.defensiveGoal.angle,
                      (int)sensors.defensiveGoal.distance);
    } else {
        Serial.print("             ");
    }
    // Print ball info
    if (sensors.ball.exists()) {
        Serial.print(" | Ball ");
        printDouble(Serial, sensors.ball.angle, 4, 1);
        Serial.printf("º %3d cm", (int)sensors.ball.distance);
    } else {
        Serial.print(" | Ball               ");
    }
    // Print robot info
    Serial.printf(" | Robot X %3d Y %3d ", (int)x, (int)y);
    printDouble(Serial, sensors.robotAngle, 4, 1);
    Serial.printf("º Ball %c %4d ", sensors.robotHasBall ? 'Y' : 'N',
                  lightgateReading);
    // Print bounds info
    Serial.printf(" | Bounds L %3d R %3d F %3d B %4d", (int)leftMargin,
                  (int)rightMargin, (int)frontMargin, (int)backMargin);
    // Print line info
    if (sensors.line.exists()) {
        Serial.printf(" | Line %4dº ", (int)sensors.line.angleBisector);
        printDouble(Serial, sensors.line.depth, 1, 2);
    } else {
        Serial.print(" | Line           ");
    }
    Serial.print(" | ");
#endif

    limitSpeed = true;

    // Check if we can find the ball
    if (sensors.robotHasBall && sensors.offensiveGoal.exists()) {
        // If we have the ball, move to the goal
        movement.setDirection(
            (Direction::Constant){sensors.offensiveGoal.angle});
        movement.setHeading(
            (Heading::Constant){sensors.offensiveGoal.angle * 1.4});
        movement.setVelocity((Velocity::Constant){Movement::applySigmoid(
            GOAL_MOVEMENT_START_SPEED, GOAL_MOVEMENT_END_SPEED,
            sensors.offensiveGoal.distance /
                GOAL_MOVEMENT_START_DECELERATING)});
        limitSpeed = false;

#ifdef DEBUG
        Serial.print("TRACK GOAL");
#endif
    } else if (isGoalie) {
        if (sensors.line.exists()) {
            // If we are on the line, track the ball, or center with the
            // goal if we can't see the ball

            double heading, x;
            if (sensors.ball.exists()) {
                heading = sensors.ball.angle;
                x = sensors.ball.angle + sensors.robotAngle;

                // Don't chase the ball if it's too far
                if (sensors.robotAngle + sensors.line.angleBisector < -30)
                    x = max(x, 0);
                if (sensors.robotAngle + sensors.line.angleBisector > 30)
                    x = min(x, 0);
            } else {
                heading = 0;
                x = -clipAngleTo180(sensors.defensiveGoal.angle - 180);
            }

            bool trackLeftwards = x < 0;
            Serial.print(sensors.robotAngle + sensors.line.angleBisector);

            movement.setDirection(
                (Direction::LineTrack){sensors.line, 0.3, trackLeftwards});
            movement.setHeading((Heading::Constant){heading});
            if (x != 0)
                movement.setVelocity((Velocity::StopAtPoint){x, 0, 250});
            else
                movement.setVelocity((Velocity::Constant){0});
        } else {
            // If we aren't on the line, return to it
            if (sensors.robotPosition.y() > -100) {
                movement.setHeading((Heading::Constant){sensors.ball.angle});
                movement.setDirection(
                    (Direction::Constant){sensors.defensiveGoal.angle});
                movement.setVelocity(
                    (Velocity::Constant){Movement::applySigmoid(
                        300, 200, sensors.defensiveGoal.distance / 180)});
            } else {
                // It'll just continue the way it was moving right before it
                // exited the line, which would probably result in it
                // returning to the line lol
            }
        }
    } else {
        if (sensors.ball.exists()) {
            if (sensors.robotAngle + sensors.offensiveGoal.angle > 55 &&
                sensors.robotAngle + sensors.ball.angle <= 0 &&
                sensors.robotAngle + sensors.ball.angle >= -120) {
                movement.setDirection((Direction::MoveToPoint){
                    sensors.robotPosition, NEUTRAL_SPOT_FL});
                movement.setHeading((Heading::MoveToPoint){
                    sensors.robotPosition, NEUTRAL_SPOT_FL, 0});
                movement.setVelocity((Velocity::StopAtPoint){
                    (Vector::fromPoint(NEUTRAL_SPOT_FL) - sensors.robotPosition)
                        .distance,
                    0, 300});
            } else if (sensors.robotAngle + sensors.offensiveGoal.angle < -55 &&
                       sensors.robotAngle + sensors.ball.angle >= 0 &&
                       sensors.robotAngle + sensors.ball.angle <= 120) {
                movement.setDirection((Direction::MoveToPoint){
                    sensors.robotPosition, NEUTRAL_SPOT_FR});
                movement.setHeading((Heading::MoveToPoint){
                    sensors.robotPosition, NEUTRAL_SPOT_FR, 0});
                movement.setVelocity((Velocity::StopAtPoint){
                    (Vector::fromPoint(NEUTRAL_SPOT_FR) - sensors.robotPosition)
                        .distance,
                    0, 300});
            } else {
                // If we can find the ball, move behind it
                moveBehindBall();
            }
#ifdef DEBUG
            Serial.print("TRACK BALL");
#endif
        } else if (sensors.robotPosition.exists()) {
            // If we can't find the ball, return to the home position
            movement.setDirection(
                (Direction::MoveToPoint){sensors.robotPosition, HOME});
            movement.setHeading(
                (Heading::MoveToPoint){sensors.robotPosition, HOME, 0});
            movement.setVelocity((Velocity::StopAtPoint){
                (Vector::fromPoint(HOME) - sensors.robotPosition).distance, 0,
                300});
#ifdef DEBUG
            Serial.print("GO TO HOME");
#endif
        } else {
            // We can't find the ball and we don't know where we are ono
        }
    }

    // Regardless of the logic above, there are some subroutines we MUST
    // run Avoid the line if we're near it
    if (!isGoalie) avoidLine();
    // Cap speed if we're not facing our goal
    if (abs(sensors.offensiveGoal.angle) > 135 || abs(sensors.robotAngle) > 150)
        movement.setVelocity((Velocity::Limit){200, 200});
    // Activate solenoid if we're near the offensive goal
    if (sensors.offensiveGoal.exists() &&
        (sensors.offensiveGoal.distance <= KICKER_THRESHOLD_DISTANCE ||
         (sensors.robotHasBall && sensors.offensiveGoal.distance <=
                                      KICKER_WITH_BALL_THRESHOLD_DISTANCE)))
        movement.setSolenoidActive();

    // Actuate output
    movement.drive();
}
