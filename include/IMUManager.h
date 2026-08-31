#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <Arduino.h>
#include <Wire.h>

class IMUManager {
public:
    IMUManager();
    bool begin();
    void update();
    float getPitch();

private:
    float pitch;
    unsigned long lastUpdate;
};

#endif