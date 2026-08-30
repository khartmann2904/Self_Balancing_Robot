#include <Arduino.h>
#include "IMUManager.h"
#include "MotorManager.h"
#include "ControlLoop.h"
#include "BatteryManager.h"
#include "BluetoothManager.h"   //Still need to implement the BluetoothManager class and its methods

// Pin-Definitionen für ESP32
#define LEFT_STEP_PIN  16
#define LEFT_DIR_PIN   17
#define LEFT_EN_PIN 25
#define LEFT_MS1_PIN 33
#define LEFT_MS2_PIN 32
#define RIGHT_STEP_PIN 27
#define RIGHT_DIR_PIN  26
#define RIGHT_EN_PIN 2
#define RIGHT_MS1_PIN 12
#define RIGHT_MS2_PIN 14
#define BATTERY_VOLTAGE_PIN 15

//Widerstandswerte für Spannungsteiler
#define R1 32000.0f // Widerstand R1 in Ohm
#define R2 10000.0f // Widerstand R2 in Ohm

//Threshold-Spannung für Batterie-Überwachung
#define BATTERY_LOW_THRESHOLD 9.0f // Spannung in Volt, bei der sich die Motoren abschalten sollen

// Objekte instanziieren
IMUManager imu;
MotorManager motors(LEFT_STEP_PIN, LEFT_DIR_PIN, RIGHT_STEP_PIN, RIGHT_DIR_PIN, RIGHT_EN_PIN);  //passes the values for both motors to the motor manager
BatteryManager battery(BATTERY_VOLTAGE_PIN, R1, R2, BATTERY_LOW_THRESHOLD);  //passes the values for the voltage divider to the battery manager

// Regler-Parameter (Kp, Ki, Kd)
PIDGains anglePID = {20.0f, 0.5f, 1.2f};        //Outer Loop Values
PIDGains speedPID = {0.1f, 0.01f, 0.0f};        //Inner Loop Values
ControlLoop controller(anglePID, speedPID);

unsigned long lastLoopTime = 0;

void setup() {
    Serial.begin(115200);       //Needs to be checked if it lowers performance
    
    if (!imu.begin()) {
        Serial.println("Fehler beim Starten der IMU!");
    }else{
        Serial.println("IMU erfolgreich gestartet.");
    }

    motors.begin();
    Serial.println("System erfolgreich gestartet.");
}

void loop() {
    unsigned long now = micros();
    float dt = (now - lastLoopTime) / 1000000.0f;

    //Battery voltage check and motor enable/disable based on battery status
    if (dt >= 0.1f) {  // Check battery status every 100 ms
        // Update last loop time for different time intervals to avoid blocking the main loop
        lastLoopTime = now;
        //Prints the battery voltage to the serial monitor
        battery.printBatteryStatus();
        // Check if battery is too low and disable motors if necessary
        if (battery.isBatteryLow()) {
            Serial.println("Warnung: Batteriespannung niedrig! Motoren werden deaktiviert.");
            motors.enableMotors(false);
        } else {
            motors.enableMotors(true);
        }
    }

    // Regelkreis mit fester Frequenz ausführen (z. B. 200 Hz = 5 ms)
    if (dt >= 0.005f) {
        lastLoopTime = now;

        imu.update();
        float currentAngle = imu.getPitch();

        // Safety-Cutoff bei Sturz (> 45 Grad)
        if (abs(currentAngle) > 45.0f) {
            motors.enableMotors(false);
            return;
        } else {
            motors.enableMotors(true);
        }

        float motorCommand = controller.computeCascade(0.0f, 0.0f, currentAngle, dt);
        motors.setSpeeds(motorCommand, motorCommand);
    }
}