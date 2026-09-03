#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <Arduino.h>

struct PIDGains {
    float Kp;
    float Ki;
    float Kd;
};

class ControlLoop {
public:
    ControlLoop(PIDGains anglePID, PIDGains speedPID);
    float computeCascade(float driveCommand, long leftPosition, long rightPosition,
                         float currentAngle, float gyroRate, bool joystickActive, float dt);
    void setAngleGains(PIDGains gains);
    void setSpeedGains(PIDGains gains);
    void reset();
    static void handleSerialTuning(ControlLoop& controller);
private:
    PIDGains angleGains;
    PIDGains speedGains;
    float speedIntegral;
    float angleIntegral;
    float lastAngleError;
    float lastPositionError;
};

#endif