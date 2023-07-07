#ifndef SENSOR_VALUE_H
#define SENSOR_VALUE_H

#include <Arduino.h>
#include <deque>

class SensorValue {
  public:
    SensorValue(uint16_t maxHistorySize = 1);
    void update(double newValue);
    double getAverage();

    double value;
    std::deque<double> history;
    uint16_t maxHistorySize;

  private:
    double rollingSum = 0;
    uint16_t validHistorySize = 0; // this only counts non-NaN values
};

#endif // SENSOR_VALUE_H
