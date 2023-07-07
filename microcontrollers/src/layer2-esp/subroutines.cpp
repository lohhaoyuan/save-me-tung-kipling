#include "main.h"

#include <Arduino.h>
#include <numeric>
#include <tuple>
#include <utility>

#include "sensor_value.h"
#include "util.h"

// Setup IR sensing
void setupIRRing() {
    for (auto pin : IR_PINS) pinMode(pin, INPUT);
}

// Global variables for readIRRing()
SensorValue ballAngleValue(IR_SAMPLE_BALL_ANGLE_COUNT);
SensorValue ballStrengthValue(IR_SAMPLE_BALL_STRENGTH_COUNT);

// Determine the IR ball angle and distance from the IR ring (this is blocking)
std::tuple<double, double, double> readIRRing(bool debugPrint) {
    /*
    Note: All numbers provided outside the brackets are as in the rule document,
    though they seem to be inconsistent to some degree of precision, but that
    doesn't matter because we just use the 833 µs cycle period in our code.

    Each cycle at ~1.2 kHz emits pulses at ~40 kHz.

    For ever ~833 µs cycle:
    - 8 pulses at 1x intensity (~194 µs)
    - 4 pulses at 1/4x intensity (~97 µs)
    - 4 pulses at 1/16x intensity (~97 µs)
    - 4 pulses at 1/64x intensity (~97 µs)
    - ~346 µs of space

    The number of IR presence readings correlates with distance because at
    different distances, the robot will detect a differently subset of pulses as
    they are at different intensities.
    */

    // Collect IR presence readings over a period of a few cycles
    std::array<double, IR_COUNT> irReadings = {0};
    uint8_t readCount = 0;
    const auto startTime = micros();
    while (micros() - startTime < 833 * IR_SAMPLE_CYCLE_COUNT) {
        for (uint8_t i = 0; i < IR_COUNT; ++i)
            // Note that the IR sensor is LOW when it detects presence
            if (!digitalRead(IR_PINS[i])) irReadings[i] += 1;
        ++readCount;
    }
    // Normalise the readings
    for (uint8_t i = 0; i < IR_COUNT; ++i) {
        irReadings[i] /= readCount;
#ifdef SCALE_IR
        irReadings[i] *= IR_SCALER_BASELINE / IR_SCALER[i];
        irReadings[i] = min(irReadings[i], 1.0);
#endif
    }

    std::array<size_t, IR_COUNT> sortedByStrengthIndices;
    std::iota(sortedByStrengthIndices.begin(), sortedByStrengthIndices.end(),
              0);
    std::sort(sortedByStrengthIndices.begin(), sortedByStrengthIndices.end(),
              [irReadings](size_t i, size_t j) {
                  return irReadings[i] > irReadings[j];
              });
    std::array<size_t, IR_ANGLE_SAMPLE_COUNT> angleIndices;
    std::copy_n(sortedByStrengthIndices.begin(), IR_ANGLE_SAMPLE_COUNT,
                angleIndices.begin());

    // Take each IR reading as a vector, and aggregate them into a single vector
    double angleX = 0, angleY = 0;
    for (auto i : angleIndices) {
        angleX += irReadings[i] * IR_UNIT_VECTORS[i].first;
        angleY += irReadings[i] * IR_UNIT_VECTORS[i].second;
    }

    // Take the ball angle and strength directly from the aggregate vector
    // atan2(0, 0) is implementation-defined behaviour, so to avoid calling it,
    // we assume x = 0 and y = 0 to mean there is no ball
    if (angleX == 0 && angleY == 0) {
        if (debugPrint) TeensySerial.println("No ball");
        ballAngleValue.update(NAN);
        ballStrengthValue.update(NAN);
        return {NAN, NAN, NAN};
    }
    const auto ballAngle = clipAngleTo180(degrees(atan2(angleY, angleX)));

    // For the ball strength, we only take readings nearest to the ball angle
    // Determine the indices of the readings to take
    std::array<size_t, IR_COUNT> sortedByAngleIndices;
    std::iota(sortedByAngleIndices.begin(), sortedByAngleIndices.end(), 0);
    std::sort(sortedByAngleIndices.begin(), sortedByAngleIndices.end(),
              [ballAngle](size_t i, size_t j) {
                  return fmod(abs(IR_BEARINGS[i] - ballAngle), 360) <
                         fmod(abs(IR_BEARINGS[j] - ballAngle), 360);
              });
    std::array<size_t, IR_STRENGTH_SAMPLE_COUNT> strengthIndices;
    std::copy_n(sortedByAngleIndices.begin(), IR_STRENGTH_SAMPLE_COUNT,
                strengthIndices.begin());

    // Find the aggregate vector formed by these readings
    double strengthX = 0, strengthY = 0;
    for (auto i : strengthIndices) {
        strengthX += irReadings[i] * IR_UNIT_VECTORS[i].first;
        strengthY += irReadings[i] * IR_UNIT_VECTORS[i].second;
    }
    const auto ballStrength =
        sqrt(strengthX * strengthX + strengthY * strengthY);

    // Update moving average
    ballAngleValue.update(ballAngle);
    ballStrengthValue.update(ballStrength);

    // Take the average of the ball data over the last few readings
    auto averageBallAngle = ballAngleValue.getAverage();
    if (std::isnan(averageBallAngle))
        // If the average is invalid, let's just use the single sample we have
        averageBallAngle = ballAngleValue.value;
    auto averageBallStrength = ballStrengthValue.getAverage();
    if (std::isnan(averageBallStrength))
        // If the average is invalid, let's just use the single sample we have
        averageBallStrength = ballStrengthValue.value;

    // Map the ball strength to a distance in cm
    double ballDistance;
    ballDistance = BALL_DISTANCE_F * pow(averageBallStrength, 5) +
                   BALL_DISTANCE_E * pow(averageBallStrength, 4) +
                   BALL_DISTANCE_D * pow(averageBallStrength, 3) +
                   BALL_DISTANCE_C * pow(averageBallStrength, 2) +
                   BALL_DISTANCE_B * averageBallStrength + BALL_DISTANCE_A;
    ballDistance = max(ballDistance, 0.0);

    // Print the readings for debugging if indicated
    if (debugPrint) {
        // Print the ball parameters
        printDouble(TeensySerial, ballAngle, 4, 2);
        TeensySerial.print("º ");
        printDouble(TeensySerial, ballDistance, 3, 2);
        TeensySerial.print(" cm (");
        printDouble(TeensySerial, averageBallStrength, 1, 2);
        TeensySerial.print(":");
        printDouble(TeensySerial, angleX, 2, 2);
        TeensySerial.print(",");
        printDouble(TeensySerial, angleY, 2, 2);
        TeensySerial.print(") ");
        // Print the positions of the IR readings which have been sampled
        for (uint8_t quadrant = 0; quadrant < 4; ++quadrant) {
            TeensySerial.print("|");
            for (uint8_t i = quadrant * IR_COUNT / 4;
                 i < (quadrant + 1) * IR_COUNT / 4; ++i) {
                const auto it =
                    std::find(angleIndices.begin(), angleIndices.end(), i);
                TeensySerial.print(it != angleIndices.end() ? "*" : "_");
            }
        }
        // Print the IR readings
        for (uint8_t quadrant = 0; quadrant < 4; ++quadrant) {
            TeensySerial.print("| ");
            for (uint8_t i = quadrant * IR_COUNT / 4;
                 i < (quadrant + 1) * IR_COUNT / 4; ++i) {
                printDouble(TeensySerial, irReadings[i], 1, 2);
                TeensySerial.print(" ");
            }
        }
        // Print the number of samples taken
        TeensySerial.printf("| %d\n", readCount);

        // Add a time buffer between prints
        delay(0.833 * 100);
    }

    return {averageBallAngle, ballDistance, averageBallStrength};
}

// Global variables for printIRRingCalibration()
SensorValue ballDistanceValue(200);
SensorValue calibrationBallStrengthValue(300); // this is more stable

void printIRRingCalibration() {
    // Read the ball angle and strength
    double ballAngle, ballDistance, ballStrength;
    std::tie(ballAngle, ballDistance, ballStrength) = readIRRing();

    // Compute the moving average
    ballDistanceValue.update(ballDistance);
    const auto averageBallDistance = ballDistanceValue.getAverage();
    calibrationBallStrengthValue.update(ballStrength);
    const auto averageBallStrength = calibrationBallStrengthValue.getAverage();

    // Print the ball parameters
    if (std::isnan(ballAngle) || std::isnan(ballDistance) ||
        std::isnan(averageBallDistance) || std::isnan(averageBallStrength))
        TeensySerial.println("No ball");
    else {
        TeensySerial.print("Ball: ");
        printDouble(TeensySerial, ballAngle, 4, 2);
        TeensySerial.print("º ");
        printDouble(TeensySerial, ballDistance, 3, 2);
        TeensySerial.print(" (");
        printDouble(TeensySerial, averageBallDistance, 3, 2);
        TeensySerial.print(") cm | ");
        printDouble(TeensySerial, ballStrength, 1, 2);
        TeensySerial.print(" (");
        printDouble(TeensySerial, averageBallStrength, 1, 3);
        TeensySerial.println(")");
    }
}
