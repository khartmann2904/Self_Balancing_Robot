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

    enum SerialRequest : uint8_t {
        SERIAL_NONE = 0,
        SERIAL_STOP = 1,
        SERIAL_START = 2
    };

    // Non-blocking: call every loop() pass. Returns stop/start request flags.
    uint8_t handleSerialTuning();

private:
    PIDGains angleGains;
    PIDGains positionGains;
    float positionIntegral;
    float angleIntegral;
    float lastPositionError;
    bool  positionInitialized;
    char    serialBuf[32];
    uint8_t serialLen;
};