#include "MotorManager.h"
#include "Config.h"

MotorManager::MotorManager(uint8_t stepPinL, uint8_t dirPinL, uint8_t enPinL,
                           uint8_t stepPinR, uint8_t dirPinR, uint8_t enPinR)
    : stepL(stepPinL), dirL(dirPinL), enL(enPinL),
      stepR(stepPinR), dirR(dirPinR), enR(enPinR),
      leftMotor(AccelStepper::DRIVER, stepPinL, dirPinL),
      rightMotor(AccelStepper::DRIVER, stepPinR, dirPinR),
      enabled(false) {}

void MotorManager::begin() {
    pinMode(stepL, OUTPUT);
    pinMode(dirL, OUTPUT);
    pinMode(enL, OUTPUT);
    pinMode(stepR, OUTPUT);
    pinMode(dirR, OUTPUT);
    pinMode(enR, OUTPUT);

    digitalWrite(enL, HIGH);   // start disabled (EN is active low); main() arms the robot
    digitalWrite(enR, HIGH);
    enabled = false;

    leftMotor.setMaxSpeed(MAX_STEP_RATE);
    rightMotor.setMaxSpeed(MAX_STEP_RATE);
    leftMotor.setCurrentPosition(0);
    rightMotor.setCurrentPosition(0);
}

void MotorManager::enableMotors(bool en) {
    if (en == enabled) return;
    enabled = en;
    if (!en) {
        // Stop the step generator too, otherwise it keeps counting steps while disabled.
        leftMotor.setSpeed(0.0f);
        rightMotor.setSpeed(0.0f);
    }
    digitalWrite(enL, en ? LOW : HIGH);
    digitalWrite(enR, en ? LOW : HIGH);
}

void MotorManager::resetPositions() {
    leftMotor.setCurrentPosition(0);
    rightMotor.setCurrentPosition(0);
}

void MotorManager::setSpeeds(float leftSpeed, float rightSpeed) {
    if (!enabled) {
        leftSpeed = 0.0f;
        rightSpeed = 0.0f;
    }
    leftMotor.setSpeed(constrain(leftSpeed, -MAX_STEP_RATE, MAX_STEP_RATE));
    rightMotor.setSpeed(constrain(rightSpeed, -MAX_STEP_RATE, MAX_STEP_RATE));
}

void MotorManager::run() {
    leftMotor.runSpeed();
    rightMotor.runSpeed();
}

long MotorManager::getLeftPosition() {
    return leftMotor.currentPosition();
}

long MotorManager::getRightPosition() {
    return rightMotor.currentPosition();
}
