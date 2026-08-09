#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <Arduino.h>

class MotorManager {
public:
    MotorManager(uint8_t stepPinL, uint8_t dirPinL, uint8_t stepPinR, uint8_t dirPinR, uint8_t enPin);
    void begin();
    void setSpeeds(float leftSpeed, float rightSpeed);
    void enableMotors(bool enable);

private:
    uint8_t stepL, dirL;
    uint8_t stepR, dirR;
    uint8_t enable;
};

#endif