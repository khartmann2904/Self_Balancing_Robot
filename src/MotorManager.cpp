#include "MotorManager.h"
#include <FastAccelStepper.h>   

MotorManager::MotorManager(uint8_t stepPinL, uint8_t dirPinL, uint8_t stepPinR, uint8_t dirPinR, uint8_t enPin)
    : stepL(stepPinL), dirL(dirPinL), stepR(stepPinR), dirR(dirPinR), enable(enPin) {}  // Konstruktor, der die Pin-Nummern für die Motoren entgegennimmt
    // Member initialization list verwendet, um die Pin-Nummern den privaten Variablen zuzuweisen, zb stepL(stepPinL) weist den Wert von stepPinL der privaten Variable stepL zu. Dies ist eine effiziente Möglichkeit, die Variablen zu initialisieren, bevor der Konstruktorkörper ausgeführt wird.                                                                                    
void MotorManager::begin() {
    pinMode(stepL, OUTPUT);
    pinMode(dirL, OUTPUT);
    pinMode(stepR, OUTPUT);
    pinMode(dirR, OUTPUT);
    pinMode(enable, OUTPUT);

    enableMotors(true);
}

void MotorManager::enableMotors(bool en) {
    digitalWrite(enable, en ? LOW : HIGH); // bei LOW sind die Motoren aktiviert, bei HIGH deaktiviert
}  

void MotorManager::setSpeeds(float leftSpeed, float rightSpeed) {
    // Richtung setzen
    digitalWrite(dirL, leftSpeed >= 0 ? HIGH : LOW);
    digitalWrite(dirR, rightSpeed >= 0 ? HIGH : LOW);

    // TODO: Geschwindigkeiten an Timer / Stepper-Pulse übergeben
}