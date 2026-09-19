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

    // Set by the serial commands "stop" / "start". Each call returns true once, then clears.
    bool takeStopRequest();
    bool takeStartRequest();

private:
    void processTuningLine(char* line);

    PIDGains angleGains;
    PIDGains positionGains;
    float positionIntegral;
    float angleIntegral;
    float lastPositionError;
    bool  positionInitialized;
    bool  stopRequested;
    bool  startRequested;
    char    serialBuf[32];
    uint8_t serialLen;
};