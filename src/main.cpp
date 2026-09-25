#include <Arduino.h>
#include "Config.h"
#include "IMUManager.h"
#include "MotorManager.h"
#include "ControlLoop.h"
#include "BatteryManager.h"
#include "BluetoothManager.h"

// Instantiate objects
IMUManager       imu(IMU_PITCH_OFFSET_DEG);
MotorManager     motors(LEFT_STEP_PIN, LEFT_DIR_PIN, LEFT_EN_PIN,
                        RIGHT_STEP_PIN, RIGHT_DIR_PIN, RIGHT_EN_PIN);
BatteryManager   battery(BATTERY_VOLTAGE_PIN, BATTERY_R1_OHMS, BATTERY_R2_OHMS, BATTERY_LOW_THRESHOLD_V);
BluetoothManager bluetooth;

// Controller parameters (Kp, Ki, Kd)
PIDGains anglePID    = {200.0f, 0.0f, 25.0f};    // inner loop: balance angle
PIDGains positionPID = {0.1f, 0.01f, 0.0f};    // outer loop: position hold
ControlLoop controller(anglePID, positionPID);

unsigned long lastControlTime  = 0;
unsigned long lastBatteryCheck = 0;
bool batteryLow = false;
bool armed      = false;   // motors only run after the robot was held upright
bool serialStop = false;   // set by the serial command "stop", cleared by "start"

void setup() {
    Serial.begin(115200);

    // MS pins: LOW/LOW = 8x microstepping on the TMC2209 (must match MICROSTEPS in Config.h)
    pinMode(LEFT_MS1_PIN, OUTPUT);
    pinMode(LEFT_MS2_PIN, OUTPUT);
    pinMode(RIGHT_MS1_PIN, OUTPUT);
    pinMode(RIGHT_MS2_PIN, OUTPUT);
    digitalWrite(LEFT_MS1_PIN, LOW);
    digitalWrite(RIGHT_MS1_PIN, LOW);
    digitalWrite(LEFT_MS2_PIN, LOW);
    digitalWrite(RIGHT_MS2_PIN, LOW);

    motors.begin();     // first, so the drivers are disabled during IMU calibration

    if (!imu.begin()) {
        while (true) {
            Serial.println("IMU nicht gefunden - bitte Verkabelung pruefen.");
            delay(1000);
        }
    }

    bluetooth.begin();

    lastControlTime  = micros();
    lastBatteryCheck = micros();
    Serial.println("System erfolgreich gestartet.");
}

void loop() {
    motors.run();   // emit step pulses as often as possible, independent of the control tick

    const unsigned long now = micros();

    // Bluepad32 must be updated continuously to process controller input.
    bluetooth.update();
    controller.handleSerialTuning();

    // Serial "stop" takes effect immediately, not only at the next control tick.
    if (controller.takeStopRequest()) {
        serialStop = true;
        motors.enableMotors(false);
        controller.reset();
        armed = false;
    }
    if (controller.takeStartRequest()) {
        serialStop = false;
    }

    // Battery check at 10 Hz; the result is latched inside BatteryManager.
    if ((now - lastBatteryCheck) >= BATTERY_CHECK_PERIOD_US) {
        lastBatteryCheck = now;
        const bool wasLow = batteryLow;
        batteryLow = battery.isBatteryLow();
        if (batteryLow && !wasLow) {
            Serial.println("Warnung: Batteriespannung niedrig! Motoren werden deaktiviert.");
        }
    }
    if (batteryLow) {
        motors.enableMotors(false);
        controller.reset();
        armed = false;
        return;
    }

    // Control loop at a fixed period
    if ((now - lastControlTime) < CONTROL_PERIOD_US) return;

    const float dt = (now - lastControlTime) / 1000000.0f;
    lastControlTime = now;

    imu.update();
    const float currentAngle = imu.getPitch();
    const float gyroRate     = imu.getGyroX();

    // Safety cutoff: serial stop, emergency stop button or a fall
    if (serialStop || bluetooth.isEmergencyStopPressed() || fabsf(currentAngle) > FALL_ANGLE_DEG) {
        motors.enableMotors(false);
        controller.reset();
        armed = false;
        return;
    }

    // (Re-)arm only when the robot is held close to upright. Clears stale positions.
    if (!armed) {
        if (fabsf(currentAngle) > ARM_ANGLE_DEG) return;
        motors.resetPositions();
        controller.reset();
        motors.enableMotors(true);
        armed = true;
    }

    const bool joystickActive = bluetooth.isJoystickActive();
    if (joystickActive) {
        motors.resetPositions();
    }

    const float motorCommand = controller.computeCascade(
        bluetooth.getDriveCommand(),      // desired lean angle in degrees
        motors.getLeftPosition(),
        motors.getRightPosition(),
        currentAngle,
        gyroRate,
        joystickActive,
        dt);

    // The sign depends on how the motors are wired. If the direction is wrong, swap it here.
    motors.setSpeeds(-motorCommand, -motorCommand);
}