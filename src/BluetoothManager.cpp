#include "BluetoothManager.h"
#include <Bluepad32.h>
// Definition der statischen Member-Variable
BluetoothManager* BluetoothManager::instance = nullptr;

BluetoothManager::BluetoothManager() 
    : activeController(nullptr), currentDriveCommand(0.0f), joystickActive(false) {
    instance = this;
}

void BluetoothManager::begin() {
    BP32.setup(&BluetoothManager::staticOnConnected, &BluetoothManager::staticOnDisconnected);
    BP32.forgetBluetoothKeys(); // Optional: Löscht vorherige Kopplungen
}

void BluetoothManager::update() {
    BP32.update();

    if (activeController && activeController->isConnected()) {
        int stickY = activeController->axisY(); // negative = vorwärts, positive = rückwärts

        if (stickY < -STICK_DEADZONE) {
            currentDriveCommand = map(stickY, -STICK_DEADZONE, -512, 0, 8);
            joystickActive = true;
        } 
        else if (stickY > STICK_DEADZONE) {
            currentDriveCommand = map(stickY, STICK_DEADZONE, 512, 0, -8);
            joystickActive = true;
        } 
        else {
            currentDriveCommand = 0.0f;
            joystickActive = false;
        }
    } else {
        currentDriveCommand = 0.0f;
        joystickActive = false;
    }
}

float BluetoothManager::getDriveCommand() const {
    return currentDriveCommand;
}

bool BluetoothManager::isJoystickActive() const {
    return joystickActive;
}

bool BluetoothManager::isConnected() const {
    return (activeController != nullptr && activeController->isConnected());
}

bool BluetoothManager::isEmergencyStopPressed() const {
    if (activeController && activeController->isConnected()) {
        return activeController->a(); // 'A' bzw. 'Cross' auf dem Gamepad
    }
    return false;
}
// Static Callbacks delegieren an die Klasseninstanz
void BluetoothManager::staticOnConnected(ControllerPtr ctl) {
    if (instance) {
        instance->handleConnected(ctl);
    }
}

void BluetoothManager::staticOnDisconnected(ControllerPtr ctl) {
    if (instance) {
        instance->handleDisconnected(ctl);
    }
}

void BluetoothManager::handleConnected(ControllerPtr ctl) {
    if (activeController == nullptr) {
        Serial.println("Bluetooth Controller verbunden!");
        activeController = ctl;
        ctl->setColorLED(0, 255, 0); // Grüne LED als Feedback
    }
}

void BluetoothManager::handleDisconnected(ControllerPtr ctl) {
    if (activeController == ctl) {
        Serial.println("Bluetooth Controller getrennt!");
        activeController = nullptr;
    }
}