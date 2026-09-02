#include "MotorManager.h"
#include "AccelStepper.h"

MotorManager::MotorManager(uint8_t stepPinL, uint8_t dirPinL, uint8_t stepPinR, uint8_t dirPinR, uint8_t enPin)
        : stepL(stepPinL), dirL(dirPinL), stepR(stepPinR), dirR(dirPinR), enable(enPin),
            leftMotor(AccelStepper::DRIVER, stepPinL, dirPinL),
            rightMotor(AccelStepper::DRIVER, stepPinR, dirPinR) {}  // Constructor that accepts the pin numbers for the motors
    // The member initialization list assigns the pin numbers to the private variables. For example, stepL(stepPinL) assigns the value of stepPinL to the private variable stepL. This is an efficient way to initialize variables before the constructor body is executed.                                                                                    
void MotorManager::begin() {
    pinMode(stepL, OUTPUT);
    pinMode(dirL, OUTPUT);
    pinMode(stepR, OUTPUT);
    pinMode(dirR, OUTPUT);
    pinMode(enable, OUTPUT);
    enableMotors(true);
}

void MotorManager::enableMotors(bool en) {
    digitalWrite(enable, en ? LOW : HIGH); // The motors are enabled when LOW and disabled when HIGH
}  

void MotorManager::setSpeeds(float leftSpeed, float rightSpeed) {
    // Set direction
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
    leftMotor.runSpeed();
    rightMotor.runSpeed();

    // TODO: Pass speeds to the timer / stepper pulses
}