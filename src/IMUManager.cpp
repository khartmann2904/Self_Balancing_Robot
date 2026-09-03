#include "IMUManager.h"
#include <Wire.h>

IMUManager::IMUManager() : pitch(0.0), lastUpdate(0), mpu6050(Wire) {}

bool IMUManager::begin() {
    Wire.begin();
    mpu6050.begin();
    Serial.println("MPU6050 calibrating...");
    mpu6050.calcGyroOffsets(true); // Calculates offset in order to know what real 'zero' is
    Serial.println("MPU6050 initialized and calibrated.");
    // Perform MPU6050 / sensor initialization here
    lastUpdate = millis();
    return true;
}

void IMUManager::update() {
    // Read sensor values and calculate the angle (e.g. using a complementary or Kalman filter)
    // Example placeholder for angle update:
    unsigned long now = millis();
    float dt = (now - lastUpdate) / 1000.0f;
    lastUpdate = now;
    mpu6050.update();
    pitch = mpu6050.getAngleX();
}

float IMUManager::getPitch() {
    return pitch;
}

float IMUManager::getGyroX() {
    return mpu6050.getGyroX();
}

void IMUManager::printSensorData() {
    Serial.print("AccX: "); Serial.print(mpu6050.getAccX());
    Serial.print(" AccY: "); Serial.print(mpu6050.getAccY());
    Serial.print(" AccZ: "); Serial.print(mpu6050.getAccZ());
    Serial.print(" GyroX: "); Serial.print(mpu6050.getGyroX());
    Serial.print(" GyroY: "); Serial.print(mpu6050.getGyroY());
    Serial.print(" GyroZ: "); Serial.println(mpu6050.getGyroZ());
}