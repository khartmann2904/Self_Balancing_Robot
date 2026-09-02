#include "IMUManager.h"

IMUManager::IMUManager() : pitch(0.0), lastUpdate(0) {}

bool IMUManager::begin() {
    Wire.begin();
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

    // TODO: Calculate the actual pitch update from the gyroscope/accelerometer
}

float IMUManager::getPitch() {
    return pitch;
}