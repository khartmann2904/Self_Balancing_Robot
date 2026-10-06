#include "IMUManager.h"
#include "Config.h"
#include <Wire.h>

IMUManager::IMUManager(float pitchOffset)
    : mpu6050(Wire), pitch(0.0f), angle(0.0f), gyroRate(0.0f),
      pitchOffset(pitchOffset), lastUpdate(0) {}

bool IMUManager::begin() {
    Wire.begin();
    Wire.setClock(400000);   // 400 kHz: much shorter read time than the 100 kHz default

    Wire.beginTransmission(MPU6050_ADDR);   // check if the MPU6050 is present on the I2C bus
    if (Wire.endTransmission() != 0) {  // non-zero means no ACK from the device
        Serial.println("MPU6050 not found on I2C!");
        return false;
    }

    mpu6050.begin();    // initializes the MPU6050 and sets the default configuration (gyro range ±250 deg/s, accel range ±2 g, 1 kHz sample rate)
    Serial.println("MPU6050 calibrating - keep the robot still...");
    mpu6050.calcGyroOffsets(true);   // determines the gyro zero point
    Serial.println("MPU6050 initialized and calibrated.");

    // Start the filter from the accelerometer angle.
    mpu6050.update();   // reads the sensor and updates the internal angle and gyro values
    angle = mpu6050.getAccAngleX(); // initial angle from the accelerometer
    gyroRate = mpu6050.getGyroX();  // initial gyro rate
    pitch = angle + pitchOffset;    // initial pitch angle including the mounting offset
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
    if (dt <= 0.0f || dt > 0.05f) dt = 0.05f;   // limits dt to a reasonable range to avoid spikes after a long delay (e.g., during debugging)

    gyroRate = mpu6050.getGyroX();
    const float accAngle = mpu6050.getAccAngleX();
    angle = IMU_GYRO_WEIGHT * (angle + gyroRate * dt) + (1.0f - IMU_GYRO_WEIGHT) * accAngle;    // complementary filter: gyro + accel
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
