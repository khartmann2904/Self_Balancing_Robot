#pragma once
#include <Arduino.h>

class BatteryManager {
public:
    BatteryManager(uint8_t batteryPIN, float R1, float R2, float lowThreshold);

    float getVoltage();        // averaged, ADC-calibrated reading in volts
    bool  isBatteryLow();      // call at ~10 Hz; latches once the battery was low long enough
    void  printBatteryStatus();

private:
    uint8_t batteryPIN;
    float   R1;
    float   R2;
    float   lowThreshold;
    float   batteryVoltage;
    uint8_t lowCount;
    bool    latched;
};
