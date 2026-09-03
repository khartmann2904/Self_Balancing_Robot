#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_tockn.h>

class IMUManager {
public:
    IMUManager();
    bool begin();   // Initializes the MPU6050 sensor and performs calibration
    void update();  // Updates the IMU readings and calculates the pitch angle
    float getPitch();   // Returns the current pitch angle of the robot
    float getGyroX();    // Returns the pitch-axis gyro rate in degrees per second
    void printSensorData();  // New method to print sensor data for debugging

private:
    float pitch;
    unsigned long lastUpdate;
    MPU6050 mpu6050;
};

#endif