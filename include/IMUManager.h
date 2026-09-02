#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

class IMUManager {
public:
    IMUManager();
    bool begin();
    void update();
    float getPitch();

private:
    float pitch;
    unsigned long lastUpdate;
    MPU6050 mpu6050;
};

#endif