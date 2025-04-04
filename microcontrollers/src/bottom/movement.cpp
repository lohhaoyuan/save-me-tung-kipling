#include "movement.h"
#include "main.h"

#include "util.h"

Movement::Movement() {}

void Movement::init() {
    analogWriteResolution(10);
    // Values from https://www.pjrc.com/teensy/td_pulse.html
    // (based on F_CPU_ACTUAL = 600 MHz)
    // analogWriteFrequency(20000);
  
}

void Movement::updateHeading(const double heading) {
    if (!std::isnan(heading)) actualHeading = heading;
}

void Movement::setDirection(const Direction::Constant params) {
    direction = params.value;
}

void Movement::setVelocity(const Velocity::Constant params) {
    velocity = params.value;
}

void Movement::setHeading(const Heading::Constant params) {
    heading = params.value;
}

void Movement::setDirection(const Direction::LineTrack params) {
    // Reset the controller if we're just starting to track a line
    if (!isLineTracking) {
        lineTrackController.reset();
        isLineTracking = true;
    }

    // Compute the correction
    const auto error = params.targetLineDepth - params.line.depth;
    const auto correction = lineTrackController.advance(error);

    // Compute the direction
    direction =
        params.line.angleBisector +
        (params.trackLeftwards ? -acosd(correction) : acosd(correction));
}

void Movement::setDirection(const Direction::MoveToPoint params) {
    // Reset the parameters if we're just starting to move to a point
    if (!isMovingToPoint || moveToPointDestination != params.destination) {
        moveToPointDestination = params.destination;
        moveToPointStart = params.robot;
        moveToPointInitialHeading = actualHeading;
        isMovingToPoint = true;
    }

    // Compute the correction
    direction = (Vector::fromPoint(params.destination) - params.robot).angle;
}

void Movement::setHeading(const Heading::MoveToPoint params) {
    // Compute progress
    const auto progress =
        (Vector::fromPoint(params.destination) - params.robot).distance /
        (Vector::fromPoint(params.destination) - moveToPointStart).distance;

    // Compute the heading
    heading = moveToPointInitialHeading +
              clipAngleTo180(params.targetHeading - moveToPointInitialHeading) *
                  progress;
}

void Movement::setVelocity(const Velocity::StopAtPoint params) {
    // Reset the controller if we're just starting to stop at a point
    if (!isStoppingAtPoint) {
        stopAtPointController.reset();
        isStoppingAtPoint = true;
    }

    // Compute the correction
    const auto correction = stopAtPointController.advance(params.error);

    // Compute the velocity
    velocity = abs(correction); // i probably should not abs() a PID output
    velocity = constrain(velocity, params.minSpeed, params.maxSpeed);
}

void Movement::setVelocity(const Velocity::Limit params) {
    const auto x = sind(direction) * velocity;
    const auto y = cosd(direction) * velocity;

    if ((params.maxX >= 0 && x > params.maxX) ||
        (params.maxX < 0 && x < params.maxX) ||
        (params.maxY >= 0 && y > params.maxY) ||
        (params.maxY < 0 && y < params.maxY)) {
        // Either of the velocity components are too high, so scale it down
        const auto scale = min(params.maxX / abs(x), params.maxY / abs(y));
        velocity *= scale;
    }
}

void Movement::setSolenoidActive() {
    if (millis() - solenoidLastActivated >=
        SOLENOID_ACTIVATION_PERIOD + SOLENOID_COOLDOWN_PERIOD) {
        solenoidLastActivated = millis();
    }
}

// Writes the current movement data.
void Movement::drive() {
    // Convert polar to cartesian
    const auto x = sind(direction);
    const auto y = cosd(direction);

    // Compute the speeds of the individual motors
    const auto transformSpeed = [this](double velocity_,
                                       double angularComponent) {
        return (int16_t)roundf(velocity_ * velocity + angularComponent);
    };
    // Find angular component
    headingController.updateSetpoint(heading);
    // Scale the controller output linearly to velocity with a reference
    // velocity of 300 (we tune it at that)
    // auto scaler = velocity == 0 ? HEADING_STATIONARY_SCALER : velocity / 300;
    auto scaler = 1.0;
    const auto angularVelocity =
        headingController.advance(actualHeading, scaler, true);
    const auto angular = 0.25 * angularVelocity;
    // Compute speeds
    const int16_t FLSpeed = transformSpeed(x * COS45 + y * SIN45, angular);
    const int16_t FRSpeed = transformSpeed(x * -COS45 + y * SIN45, -angular);
    const int16_t BLSpeed = transformSpeed(x * -COS45 + y * SIN45, +angular);
    const int16_t BRSpeed = transformSpeed(x * COS45 + y * SIN45, -angular);

    EncoderTxPayloadUnion payload;
    payload.data.motorSpeed[0] = FLSpeed;
    payload.data.motorSpeed[1] = FRSpeed;
    payload.data.motorSpeed[2] = BLSpeed;
    payload.data.motorSpeed[3] = BRSpeed;
    // payload.data.motorSpeed[0] = 200;
    // payload.data.motorSpeed[1] = 200;
    // payload.data.motorSpeed[2] = 200;
    // payload.data.motorSpeed[3] = 200;



    EncoderPacketSerial.send(payload.bytes, sizeof(payload.bytes));
    // // Constrain motor speed with "hardware-imposed" limits
    // auto constrainSpeed = [](int16_t speed) {
    //     // If the speed is below the stall speed, don't bother moving
    //     if (abs(speed) < DRIVE_STALL_SPEED) return 0;
    //     return min(abs(speed), DRIVE_MAX_SPEED);
    // };

    // // Set the motor directions and speeds
    // digitalWriteFast(PIN_MOTOR_FL_DIR,
    //                  FLSpeed > 0 ? MOTOR_FL_REVERSED : !MOTOR_FL_REVERSED);
    // digitalWriteFast(PIN_MOTOR_FR_DIR,
    //                  FRSpeed > 0 ? MOTOR_FR_REVERSED : !MOTOR_FR_REVERSED);
    // digitalWriteFast(PIN_MOTOR_BL_DIR,
    //                  BLSpeed > 0 ? MOTOR_BL_REVERSED : !MOTOR_BL_REVERSED);
    // digitalWriteFast(PIN_MOTOR_BR_DIR,
    //                  BRSpeed > 0 ? MOTOR_BR_REVERSED : !MOTOR_BR_REVERSED);
    // analogWrite(PIN_MOTOR_FL_PWM,
    //             constrainSpeed(FLSpeed) * MOTOR_FL_MULTIPLIER);
    // analogWrite(PIN_MOTOR_FR_PWM,
    //             constrainSpeed(FRSpeed) * MOTOR_FR_MULTIPLIER);
    // analogWrite(PIN_MOTOR_BL_PWM,
    //             constrainSpeed(BLSpeed) * MOTOR_BL_MULTIPLIER);
    // analogWrite(PIN_MOTOR_BR_PWM,
    //             constrainSpeed(BRSpeed) * MOTOR_BR_MULTIPLIER);

// #ifdef NEW_BOT
//     // Update solenoid state
//     L1ESPRxPayloadUnion l1Payload;
//     l1Payload.data.activateSolenoid =
//         millis() <= solenoidLastActivated + SOLENOID_ACTIVATION_PERIOD;
//     L1ESPPacketSerial.send(l1Payload.bytes, sizeof(l1Payload.bytes));
// #endif

#ifdef DEBUG
    Serial.print(" | Drive ");
    printDouble(Serial, direction, 3, 1);
    Serial.printf("º %4d facing ", (int)velocity);
    printDouble(Serial, heading, 3, 1);
    Serial.print("º");
    Serial.printf(" FL %4d FR %4d BL %4d BR %4d", FLSpeed, FRSpeed, BLSpeed,
                  BRSpeed);
    Serial.println();
#endif
}

double Movement::applySigmoid(const double startSpeed, const double endSpeed,
                              const double progress) {
    // We use a transformed sigmoid here
    const auto multiplier =
        1 / (1 + pow(2 * SIGMOID_MAX_DIFF - 1, 2 * progress - 1));
    return startSpeed + (endSpeed - startSpeed) * multiplier;
}