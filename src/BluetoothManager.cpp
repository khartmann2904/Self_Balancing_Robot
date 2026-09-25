#include "BluetoothManager.h"
#include "Config.h"

// Definition of the static member variable
BluetoothManager* BluetoothManager::instance = nullptr;

static float mapf(float x, float inMin, float inMax, float outMin, float outMax) {
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

BluetoothManager::BluetoothManager()
    : activeController(nullptr), currentDriveCommand(0.0f), joystickActive(false), lastUpdateUs(0) {
    instance = this;
}

void BluetoothManager::begin() {
    BP32.setup(&BluetoothManager::staticOnConnected, &BluetoothManager::staticOnDisconnected);
    if (BT_FORGET_KEYS_ON_BOOT) {
        BP32.forgetBluetoothKeys();   // deletes previous pairings
    }
    lastUpdateUs = micros();
}

void BluetoothManager::update() {
    BP32.update();

    // Raw target lean from the stick (0 if there is no controller)
    float targetLean = 0.0f;
    bool stickActive = false;

    if (activeController && activeController->isConnected()) {
        const int stickY = activeController->axisY();   // negative = forward, positive = backward
        if (stickY < -STICK_DEADZONE) {
            targetLean = mapf(static_cast<float>(max(stickY, -STICK_MAX)), -STICK_DEADZONE, -STICK_MAX, 0.0f, MAX_LEAN_DEG);
            stickActive = true;
        } else if (stickY > STICK_DEADZONE) {
            targetLean = mapf(static_cast<float>(min(stickY, STICK_MAX)), STICK_DEADZONE, STICK_MAX, 0.0f, -MAX_LEAN_DEG);
            stickActive = true;
        }
    }

    // Slew-limit the lean command so a stick snap or release does not tip the robot.
    const unsigned long now = micros();
    float dt = (now - lastUpdateUs) / 1000000.0f;
    lastUpdateUs = now;
    if (dt < 0.0f || dt > 0.1f) dt = 0.1f;

    const float maxStep = DRIVE_SLEW_DEG_PER_S * dt;
    const float diff = targetLean - currentDriveCommand;
    currentDriveCommand += constrain(diff, -maxStep, maxStep);

    // Stay in "driving" mode until the lean has ramped back to zero, so the
    // position hold starts from a clean state.
    joystickActive = stickActive || fabsf(currentDriveCommand) > 0.01f;
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
        return activeController->a();   // 'A' or 'Cross' on the gamepad
    }
    return false;
}

// Static callbacks delegate to the class instance
void BluetoothManager::staticOnConnected(ControllerPtr ctl) {
    if (instance) instance->handleConnected(ctl);
}

void BluetoothManager::staticOnDisconnected(ControllerPtr ctl) {
    if (instance) instance->handleDisconnected(ctl);
}

void BluetoothManager::handleConnected(ControllerPtr ctl) {
    if (activeController == nullptr) {
        Serial.println("Bluetooth Controller verbunden!");
        activeController = ctl;
        ctl->setColorLED(0, 255, 0);   // green LED as feedback
    }
}

void BluetoothManager::handleDisconnected(ControllerPtr ctl) {
    if (activeController == ctl) {
        Serial.println("Bluetooth Controller getrennt!");
        activeController = nullptr;
    }
}
