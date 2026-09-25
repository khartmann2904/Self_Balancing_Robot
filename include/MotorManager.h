#pragma once
#include <Arduino.h>
#include <AccelStepper.h>

class MotorManager {
public:
    MotorManager(uint8_t stepPinL, uint8_t dirPinL, uint8_t enPinL,
                 uint8_t stepPinR, uint8_t dirPinR, uint8_t enPinR);

    void begin();                       // leaves the drivers DISABLED
    void enableMotors(bool en);         // only touches the pins when the state changes
    void resetPositions();
    void setSpeeds(float leftSpeed, float rightSpeed);   // steps/s, clamped
    void run();                         // emits due steps: call on EVERY loop() pass
    long getLeftPosition();
    long getRightPosition();

private:
    uint8_t stepL, dirL, enL, stepR, dirR, enR;
    AccelStepper leftMotor;
    AccelStepper rightMotor;
    bool enabled;
};
