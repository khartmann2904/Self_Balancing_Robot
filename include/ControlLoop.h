#pragma once
#include <Arduino.h>

struct PIDGains {
    float Kp;
    float Ki;
    float Kd;
};

class ControlLoop {
public:
    // angleGains = inner loop (balance), positionGains = outer loop (hold position)
    ControlLoop(PIDGains angleGains, PIDGains positionGains);

    // targetAngle: desired lean in degrees (from the joystick)
    // returns the wheel speed command in steps/s
    float computeCascade(float targetAngle, long leftPosition, long rightPosition,
                         float currentAngle, float gyroRate, bool joystickActive, float dt);

    void setAngleGains(PIDGains gains);
    void setPositionGains(PIDGains gains);
    void reset();

    // Non-blocking: call every loop() pass.
    void handleSerialTuning();

private:
    void processTuningLine(char* line);

    PIDGains angleGains;
    PIDGains positionGains;
    float positionIntegral;
    float angleIntegral;
    float lastPositionError;
    bool  positionInitialized;
    char    serialBuf[32];
    uint8_t serialLen;
};
