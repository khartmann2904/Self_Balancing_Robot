#include "IMUManager.h"

IMUManager::IMUManager() : pitch(0.0), lastUpdate(0) {}

bool IMUManager::begin() {
    Wire.begin();
    // Hier MPU6050 / Sensor-Initialisierung durchführen
    lastUpdate = millis();
    return true;
}

void IMUManager::update() {
    // Sensorwerte auslesen und Winkelberechnung (z. B. Komplementär- oder Kalman-Filter)
    // Beispielplatzhalter für Winkel-Update:
    unsigned long now = millis();
    float dt = (now - lastUpdate) / 1000.0f;
    lastUpdate = now;

    // TODO: Echtes Pitch-Update aus Gyro/Accel berechnen
}

float IMUManager::getPitch() {
    return pitch;
}