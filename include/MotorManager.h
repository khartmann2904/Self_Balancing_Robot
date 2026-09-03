#ifndef MOTOR_MANAGER_H
#define MOTOR_MANAGER_H

#include <Arduino.h>
#include <AccelStepper.h>

class MotorManager {
public:
    MotorManager(uint8_t stepPinL, uint8_t dirPinL, uint8_t enPinL, uint8_t stepPinR, uint8_t dirPinR, uint8_t enPinR);
    void begin();
    void setSpeeds(float leftSpeed, float rightSpeed);
    void enableMotors(bool enable);
    void resetPositions();
    long getLeftPosition();
    long getRightPosition();
    
    
private:
    uint8_t stepL, dirL, enL;
    uint8_t stepR, dirR, enR;
    AccelStepper leftMotor;
    AccelStepper rightMotor;
};

#endif