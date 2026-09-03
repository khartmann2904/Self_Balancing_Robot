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
    float getDriveCommand() const;  // Returns the current drive command based on joystick input
    bool isJoystickActive() const;  // Returns true if the joystick is actively being used (i.e., outside the deadzone)
    bool isConnected() const;   // Returns true if a controller is connected
    bool isEmergencyStopPressed() const;    // Returns true if the emergency stop button (e.g., 'A' button) is pressed on the controller

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