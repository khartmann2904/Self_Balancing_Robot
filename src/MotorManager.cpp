#include "MotorManager.h"

MotorManager::MotorManager(uint8_t stepPinL, uint8_t dirPinL, uint8_t stepPinR, uint8_t dirPinR, uint8_t enPin)
    : stepL(stepPinL), dirL(dirPinL), stepR(stepPinR), dirR(dirPinR), enable(enPin) {}

void MotorManager::begin() {
    pinMode(stepL, OUTPUT);
    pinMode(dirL, OUTPUT);
    pinMode(stepR, OUTPUT);
    pinMode(dirR, OUTPUT);
    pinMode(enable, OUTPUT);
    
    enableMotors(true);
}

void MotorManager::enableMotors(bool en) {
    digitalWrite(enable, en ? LOW : HIGH); // LOW ist bei TMC2209 aktiv
}

void MotorManager::setSpeeds(float leftSpeed, float rightSpeed) {
    // Richtung setzen
    digitalWrite(dirL, leftSpeed >= 0 ? HIGH : LOW);
    digitalWrite(dirR, rightSpeed >= 0 ? HIGH : LOW);

    // TODO: Geschwindigkeiten an Timer / Stepper-Pulse übergeben
}