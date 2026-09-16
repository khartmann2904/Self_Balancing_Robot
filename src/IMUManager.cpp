#include "IMUManager.h"
#include <Wire.h>

IMUManager::IMUManager(float pitchOffset) : pitch(0.0), lastUpdate(0), mpu6050(Wire), pitchOffset(pitchOffset) {}

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
    pitch = mpu6050.getAngleX() + pitchOffset;
    //Serial.print("Pitch: ");
    //Serial.println(pitch);

}

float IMUManager::getPitch() {
    return pitch;
}