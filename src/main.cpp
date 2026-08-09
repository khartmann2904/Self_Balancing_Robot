#include <Arduino.h>
#include "IMUManager.h"
#include "MotorManager.h"
#include "ControlLoop.h"

// Pin-Definitionen für ESP32
#define STEP_L 25
#define DIR_L  26
#define STEP_R 27
#define DIR_R  14
#define EN_PIN 12

// Objekte instanziieren
IMUManager imu;
MotorManager motors(STEP_L, DIR_L, STEP_R, DIR_R, EN_PIN);

// Regler-Parameter (Kp, Ki, Kd)
PIDGains anglePID = {20.0f, 0.5f, 1.2f};
PIDGains speedPID = {0.1f, 0.01f, 0.0f};
ControlLoop controller(anglePID, speedPID);

unsigned long lastLoopTime = 0;

void setup() {
    Serial.begin(115200);
    
    if (!imu.begin()) {
        Serial.println("Fehler beim Starten der IMU!");
    }

    motors.begin();
    Serial.println("System erfolgreich gestartet.");
}

void loop() {
    unsigned long now = micros();
    float dt = (now - lastLoopTime) / 1000000.0f;

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