#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PIDController {
  public:
    PIDController(const double setpoint, const double minOut, const double maxOut,
                  const double kp, const double ki, const double kd,
                  const uint32_t minDt = 0, const double maxi = infinity(),
                  const double maxSetpointChange = infinity());

    // Update controller
    double advance(const double input, const double scaler = 1.0,
                   const bool isAngle = false);
    void reset();

    // Update parameters
    void updateSetpoint(const double value);
    void updateLimits(const double minOut, const double maxOut);
    void updateGains(const double kp, const double ki, const double kd);

    void debugPrint(const char *name = nullptr, Stream &serial = Serial);

    double currentSetpoint() const { return _setpoint; }

  private:
    // Parameters
    double _targetSetpoint;
    double _setpoint;
    double _minOut;
    double _maxOut;
    double _kp;
    double _ki;
    double _kd;
    uint32_t _minDt;
    double _maxi;
    double _maxSetpointChange;
    // Internal values
    double _integral = 0.0;
    double _lastError = 0.0;
    uint32_t _lastTime = 0;
    double _lastOutput = 0.0;
    bool _justStarted = true;
    // For debugging
    double _lastInput = 0.0;
    double _lastP = 0.0;
    double _lastI = 0.0;
    double _lastD = 0.0;
    uint32_t _lastDt = 0;
};

#endif // PID_H