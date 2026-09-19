#include "IMUManager.h"
#include "Config.h"
#include <Wire.h>

IMUManager::IMUManager(float pitchOffset)
    : mpu6050(Wire), pitch(0.0f), angle(0.0f), gyroRate(0.0f),
      pitchOffset(pitchOffset), lastUpdate(0) {}

bool IMUManager::begin() {
    Wire.begin();
    Wire.setClock(400000);   // 400 kHz: much shorter read time than the 100 kHz default

    Wire.beginTransmission(MPU6050_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("MPU6050 not found on I2C!");
        return false;
    }

    mpu6050.begin();
    Serial.println("MPU6050 calibrating - keep the robot still...");
    mpu6050.calcGyroOffsets(true);   // determines the gyro zero point
    Serial.println("MPU6050 initialized and calibrated.");

    // Start the filter from the accelerometer angle.
    mpu6050.update();
    angle = mpu6050.getAccAngleX();
    gyroRate = mpu6050.getGyroX();
    pitch = angle + pitchOffset;
    lastUpdate = micros();
    return true;
}

void IMUManager::update() {
    mpu6050.update();

    // Complementary filter with a micros()-based dt. The library's own angle uses
    // millis(), whose 1 ms resolution adds noise at a 5 ms tick.
    const unsigned long now = micros();
    float dt = (now - lastUpdate) / 1000000.0f;
    lastUpdate = now;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.05f;

    gyroRate = mpu6050.getGyroX();
    const float accAngle = mpu6050.getAccAngleX();
    angle = IMU_GYRO_WEIGHT * (angle + gyroRate * dt) + (1.0f - IMU_GYRO_WEIGHT) * accAngle;
    pitch = angle + pitchOffset;
}

float IMUManager::getPitch() {
    return pitch;
}

float IMUManager::getGyroX() {
    return gyroRate;
}

void IMUManager::printSensorData() {
    Serial.print("AccX: ");  Serial.print(mpu6050.getAccX());
    Serial.print(" AccY: "); Serial.print(mpu6050.getAccY());
    Serial.print(" AccZ: "); Serial.print(mpu6050.getAccZ());
    Serial.print(" GyroX: "); Serial.print(mpu6050.getGyroX());
    Serial.print(" GyroY: "); Serial.print(mpu6050.getGyroY());
    Serial.print(" GyroZ: "); Serial.println(mpu6050.getGyroZ());
}
