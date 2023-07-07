#include "sensor_value.h"

SensorValue::SensorValue(uint16_t historySize) : maxHistorySize(historySize) {}

void SensorValue::update(double newValue) {
    // Update value
    value = newValue;

    // Add value to history
    history.push_back(newValue);

    if (!std::isnan(newValue)) {
        // Update rolling sum
        rollingSum += newValue;
        ++validHistorySize;
    }

    // Remove old values from history
    while (history.size() > maxHistorySize) {
        if (!std::isnan(history.front())) {
            // Update rolling sum
            rollingSum -= history.front();
            --validHistorySize;
        }
        history.pop_front();
    }
}

double SensorValue::getAverage() {
    // At least half the history must be valid to return a valid average
    if (validHistorySize / history.size() >= 0.5)
        return rollingSum / validHistorySize;
    else
        return NAN;
}
