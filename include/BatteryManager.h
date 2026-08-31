#ifndef BATTERY_MANAGER_H
#define BATTERY_MANAGER_H

#include <Arduino.h>

class BatteryManager {
public:
    BatteryManager(uint8_t batteryPIN, float R1, float R2, float lowThreshold);
    float getVoltage();
    bool isBatteryLow();
    void printBatteryStatus();

private:
    uint8_t batteryPIN;
    float batteryVoltage;
    float R1;
    float R2;
    float lowThreshold;
    unsigned long lastUpdate;
};

#endif
