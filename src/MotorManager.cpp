#include "MotorManager.h"
#include "AccelStepper.h"

MotorManager::MotorManager(uint8_t stepPinL, uint8_t dirPinL, uint8_t enPinL, uint8_t stepPinR, uint8_t dirPinR, uint8_t enPinR)
        : stepL(stepPinL), dirL(dirPinL), enL(enPinL), stepR(stepPinR), dirR(dirPinR), enR(enPinR),
            leftMotor(AccelStepper::DRIVER, stepPinL, dirPinL),
            rightMotor(AccelStepper::DRIVER, stepPinR, dirPinR) {}  // Constructor that accepts the pin numbers for the motors
    // The member initialization list assigns the pin numbers to the private variables. For example, stepL(stepPinL) assigns the value of stepPinL to the private variable stepL. This is an efficient way to initialize variables before the constructor body is executed.                                                                                    
void MotorManager::begin() {
    pinMode(stepL, OUTPUT);
    pinMode(dirL, OUTPUT);
    pinMode(enL, OUTPUT);
    pinMode(stepR, OUTPUT);
    pinMode(dirR, OUTPUT);
    pinMode(enR, OUTPUT);
    enableMotors(true);
}

void MotorManager::enableMotors(bool en) {
    digitalWrite(enL, en ? LOW : HIGH);
    digitalWrite(enR, en ? LOW : HIGH);
}

void MotorManager::resetPositions() {
    leftMotor.setCurrentPosition(0);
    rightMotor.setCurrentPosition(0);
}

void MotorManager::setSpeeds(float leftSpeed, float rightSpeed) {
    // Set direction
    leftMotor.setSpeed(leftSpeed);
    rightMotor.setSpeed(rightSpeed);
    leftMotor.runSpeed();
    rightMotor.runSpeed();

}

long MotorManager::getLeftPosition() {
    return leftMotor.currentPosition();
}

long MotorManager::getRightPosition() {
    return rightMotor.currentPosition();
}