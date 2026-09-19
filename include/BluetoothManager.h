#pragma once
#include <Arduino.h>
#include <Bluepad32.h>

class BluetoothManager {
public:
    BluetoothManager();

    void  begin();
    void  update();                        // call every loop() pass
    float getDriveCommand() const;         // desired lean angle in degrees (slew-limited)
    bool  isJoystickActive() const;        // true while the stick is deflected or the lean is still ramping down
    bool  isConnected() const;
    bool  isEmergencyStopPressed() const;  // A / Cross button

private:
    static BluetoothManager* instance;
    static void staticOnConnected(ControllerPtr ctl);
    static void staticOnDisconnected(ControllerPtr ctl);
    void handleConnected(ControllerPtr ctl);
    void handleDisconnected(ControllerPtr ctl);

    ControllerPtr activeController;
    float currentDriveCommand;
    bool  joystickActive;
    unsigned long lastUpdateUs;
};
