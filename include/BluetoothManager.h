#ifndef BLUETOOTHMANAGER_H
#define BLUETOOTHMANAGER_H

#include <Arduino.h>
#include <Bluepad32.h>

class BluetoothManager {
public:
    BluetoothManager();

    void begin();
    void update();

    // Note: const must be identical here and in the .cpp file!
    float getDriveCommand() const;
    bool isJoystickActive() const;
    bool isConnected() const;
    bool isEmergencyStopPressed() const;

private:
    static void staticOnConnected(ControllerPtr ctl);
    static void staticOnDisconnected(ControllerPtr ctl);

    void handleConnected(ControllerPtr ctl);
    void handleDisconnected(ControllerPtr ctl);

    static BluetoothManager* instance;

    ControllerPtr activeController;
    float currentDriveCommand;
    bool joystickActive;

    const int STICK_DEADZONE = 50;
};

#endif