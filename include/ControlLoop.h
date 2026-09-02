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
    float computeCascade(float targetSpeed, float currentSpeed, float currentAngle, float dt);
    void setAngleGains(PIDGains gains);

private:
    PIDGains angleGains;
    PIDGains speedGains;
    
    float speedIntegral;
    float angleIntegral;
    float lastAngleError;
};

#endif