#pragma once
#include <Arduino.h>
#include <MPU6050_tockn.h>

class IMUManager {
public:
    explicit IMUManager(float pitchOffset);

    bool  begin();            // returns false if the MPU6050 does not answer
    void  update();           // call once per control tick
    float getPitch();         // filtered pitch angle in degrees (incl. mounting offset)
    float getGyroX();         // pitch-axis gyro rate in deg/s
    void  printSensorData();

private:
    MPU6050 mpu6050;
    float pitch;              // angle + offset
    float angle;              // filtered angle without offset
    float gyroRate;
    float pitchOffset;
    unsigned long lastUpdate; // micros()
};
