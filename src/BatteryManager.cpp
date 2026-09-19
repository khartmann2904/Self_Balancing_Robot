#include "BatteryManager.h"
#include "Config.h"

BatteryManager::BatteryManager(uint8_t batteryPIN, float R1, float R2, float lowThreshold)
    : batteryPIN(batteryPIN), R1(R1), R2(R2), lowThreshold(lowThreshold),
      batteryVoltage(0.0f), lowCount(0), latched(false) {}

float BatteryManager::getVoltage() {
    // analogReadMilliVolts() applies the ESP32's factory ADC calibration.
    uint32_t mvSum = 0;
    for (uint8_t i = 0; i < 16; i++) {
        mvSum += analogReadMilliVolts(batteryPIN);
    }
    const float pinVolts = (mvSum / 16.0f) / 1000.0f;
    batteryVoltage = pinVolts * ((R1 + R2) / R2);   // undo the voltage divider
    return batteryVoltage;
}

bool BatteryManager::isBatteryLow() {
    if (latched) return true;

    if (getVoltage() < lowThreshold) {
        if (++lowCount >= BATTERY_LOW_SAMPLES) latched = true;   // sag under load must not trip it
    } else {
        lowCount = 0;
    }
    return latched;
}

void BatteryManager::printBatteryStatus() {
    Serial.print("Battery Voltage: ");
    Serial.print(getVoltage());
    Serial.println(" V");
}
