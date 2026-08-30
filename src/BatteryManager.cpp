#include "BatteryManager.h"

BatteryManager::BatteryManager(uint8_t batteryPIN, float R1, float R2, float lowThreshold) : batteryPIN(batteryPIN), R1(R1), R2(R2), lowThreshold(lowThreshold), batteryVoltage(0.0) {}

float BatteryManager::getVoltage() {
    batteryVoltage = analogRead(batteryPIN) * (3.3 / 4095.0) * ((R1 + R2) / R2);
    return batteryVoltage;
    //returns the battery voltage in volts, calculated using the voltage divider formula
}

bool BatteryManager::isBatteryLow() {
    return getVoltage() < lowThreshold;
    //returns true if the battery voltage is below the specified threshold
}

void BatteryManager::printBatteryStatus() {
    Serial.print("Battery Voltage: ");
    Serial.print(getVoltage());
    Serial.println(" V");
    //prints the current battery voltage to the serial monitor
}