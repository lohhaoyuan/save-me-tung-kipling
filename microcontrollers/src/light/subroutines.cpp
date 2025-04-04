#include "main.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

#include "util.h"
#include "config.h"

extern bool frontLG;
extern bool backLG;
// Read the value of a photodiode
uint16_t readPhotodiode(uint8_t i) {
    return muxs[PHOTODIODE_MAP[i] >> 4].readChannel(PHOTODIODE_MAP[i] & 0x0F,
                                                    MUX_READ_DELAY);
}

// Find line
std::pair<double, double> findLine(std::array<bool, PHOTODIODE_COUNT> onLine) {
    // Compile the matches (to the line)
    std::vector<uint8_t> matches;
    matches.reserve(PHOTODIODE_COUNT);
    for (uint8_t i = 0; i < PHOTODIODE_COUNT; ++i)
        if (onLine[i]) matches.push_back(i);

    // If we don't find at least 2 matches (to the line), we're not on the line
    if (matches.size() <= 1) return {NAN, NAN};

    // Let's try to find the scope of the line
    double lineStartAngle = NAN, lineEndAngle = NAN;
    // Iterate through all combinations of matching indices and find the pair
    // of indices furthest apart
    double maxAngleDifference = 0;
    for (uint8_t i = 0; i < matches.size() - 1; ++i)
        for (uint8_t j = i + 1; j < matches.size(); ++j) {
            const auto angleI = PHOTODIODE_BEARINGS[matches[i]];
            const auto angleJ = PHOTODIODE_BEARINGS[matches[j]];
            const auto angleDifference = smallerAngleDifference(angleI, angleJ);
            if (angleDifference > maxAngleDifference) {
                maxAngleDifference = angleDifference;
                lineStartAngle = angleI;
                lineEndAngle = angleJ;
            }
        }

    // This should not happen, but just in case
    if (std::isnan(lineStartAngle) || std::isnan(lineEndAngle))
        return {NAN, NAN};

    // Calculate the line angle bisector and size
    if (lineEndAngle - lineStartAngle > 180)
        std::swap(lineStartAngle, lineEndAngle);
    const auto lineAngleBisector = angleBisector(lineStartAngle, lineEndAngle);
    // Let line size be the ratio of the cluster size to 180°, so it's in [0, 1]
    const auto lineSize = maxAngleDifference / 180.0;

    return {lineAngleBisector, lineSize};
}

// Global variables for adjustForLineSide()
bool isInside = true;    // Which side of the line is the robot on?
uint8_t switchCount = 0; // Has the angle jumped consistently enough to
                         // consider the robot to have "switched sides"?
std::deque<double> lineAngleBisectorHistory; // Past angles for comparison

// Adjust the values for which side of the line the robot is on
std::pair<double, double> adjustForLineSide(std::pair<double, double> line) {
    auto angleBisector = line.first;
    auto size = line.second;

    // Compute line depth
    if (!std::isnan(size)) {
        // The robot is on the line, proceed to compute line parameters

        // This part records jumps in line angle bisectors by referencing the
        // history by updating switchCount
        const auto hasJump = [angleBisector](double lastAngleBisector) {
            return smallerAngleDifference(angleBisector, lastAngleBisector) >
                   LINE_ANGLE_SWITCH_ANGLE;
        };
        if (!lineAngleBisectorHistory.empty() &&
            std::all_of(lineAngleBisectorHistory.begin(),
                        lineAngleBisectorHistory.end(), hasJump)) {
            // There's a huge jump in the line angle bisector while in the line
            // so we have probbaly switched sides, increment the counter.
            ++switchCount;
        } else {
            // Only add to the line angle history if we haven't switched sides
            // as we want it to reflect the last line angle at the last side.
            // This also filters out noise in the line angle history.
            lineAngleBisectorHistory.push_back(angleBisector);
            while (lineAngleBisectorHistory.size() > LINE_ANGLE_HISTORY)
                lineAngleBisectorHistory.pop_front();
        }

        // Now, we can use switchCount to determine if the robot switched sides
        if (switchCount >= LINE_ANGLE_SWITCH_COUNT) {
            Serial.println("SWITCH");
            // Register the switch
            switchCount = 0;
            lineAngleBisectorHistory.clear();
            lineAngleBisectorHistory.push_back(angleBisector);

            // The robot has probably switched sides
            if (isInside) {
                // Robot moved from the inner half to the outer half of the line
                size = 1 - size / 2;
                angleBisector = clipAngleTo180(angleBisector);
            } else {
                // Robot moved from the outer half to the inner half of the line
                size = size / 2;
                angleBisector = clipAngleTo180(angleBisector + 180);
            }
            isInside = !isInside;
        } else if (isInside) {
            // The robot didn't switch sides, on the inner half of the line
            size = size / 2;
            angleBisector = clipAngleTo180(angleBisector + 180);
        } else {
            // The robot didn't switch sides, on the outer half of the line
            size = 1 - size / 2;
            angleBisector = clipAngleTo180(angleBisector);
        }
    } else {
        // The robot is not on the line

        // // Snap the depth to 0 or 1
        // if (isInside) {
        //     size = 0; // It exited on the inside of the line
        // } else {
        //     size = 1; // It exited on the outside of the line
        // }

        // Snap the depth to 0 as it's more likely to be inside
        isInside = true;
        size = 0;

        // Reset the line angle bisector history
        lineAngleBisectorHistory.clear();
    }

    return std::make_pair(angleBisector, size);
}

// Run the calibration routine
void printLightRingCalibration() {
    TeensySerial.println("Penis");
    // min = green (field), max = white (line)
    uint16_t min[PHOTODIODE_COUNT], max[PHOTODIODE_COUNT];
    for (int i = 0; i < PHOTODIODE_COUNT; ++i) {
        min[i] = 0xFFFF;
        max[i] = 0x0000;
    }

    const auto endTime = millis() + LIGHT_RING_CALIBRATION_DURATION;
    while (millis() < endTime) {
        for (uint8_t i = 0; i < PHOTODIODE_COUNT; ++i) {
            const auto value = readPhotodiode(i);
            if (value < min[i]) min[i] = value;
            if (value > max[i]) max[i] = value;
        }
    }

    // Print the thresholds (averages of min and max)
    TeensySerial.printf("Thresholds: {");
    for (uint8_t i = 0; i < PHOTODIODE_COUNT; ++i) {
        // TODO: Find a more accurate way to calculate the threshold
        const auto threshold = (max[i] + min[i]) / 2;
        TeensySerial.printf("%d, ", (uint16_t)threshold);
    }
    TeensySerial.printf("}\n");
}

// Print the values of the light ring
void printLightRing() {
    // Read raw values from each photodiode
    std::array<uint16_t, PHOTODIODE_COUNT> values;
    for (uint8_t i = 0; i < PHOTODIODE_COUNT; ++i)
        values[i] = readPhotodiode(i);

    // Print photodiodes fulfilling threshold
    for (uint8_t quadrant = 0; quadrant < 4; ++quadrant) {
        TeensySerial.print("|");
        for (uint8_t i = quadrant * PHOTODIODE_COUNT / 4;
             i < (quadrant + 1) * PHOTODIODE_COUNT / 4; ++i)
            TeensySerial.printf(
                "%c", values[i] > PHOTODIODE_THRESHOLDS[i] ? 'X' : '_');
    }
    // Print photodiode values
    for (uint8_t quadrant = 0; quadrant < 4; ++quadrant) {
        TeensySerial.print("| ");
        for (uint8_t i = quadrant * PHOTODIODE_COUNT / 4;
             i < (quadrant + 1) * PHOTODIODE_COUNT / 4; ++i)
            TeensySerial.printf("%4d ", values[i]);
    }
    TeensySerial.println("|");
}

void updateLightGates() {
    // Update the light gates
    digitalWrite(FLAG_FRONT, readPhotodiode(16) > LIGHT_GATE_THRESHOLD ? HIGH : LOW); frontLG = readPhotodiode(16) > LIGHT_GATE_THRESHOLD ? true : false;
    digitalWrite(FLAG_BACK, readPhotodiode(32) > LIGHT_GATE_THRESHOLD ? HIGH : LOW); backLG = readPhotodiode(32) > LIGHT_GATE_THRESHOLD ? true : false;
    
}