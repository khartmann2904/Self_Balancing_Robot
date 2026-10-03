#include <Arduino.h>
#include "Config.h"
#include "IMUManager.h"
#include "MotorManager.h"
#include "ControlLoop.h"
#include "BatteryManager.h"
#include "BluetoothManager.h"
#include "Telemetry.h"

// Instantiate objects
IMUManager       imu(IMU_PITCH_OFFSET_DEG);
MotorManager     motors(LEFT_STEP_PIN, LEFT_DIR_PIN, LEFT_EN_PIN,
                        RIGHT_STEP_PIN, RIGHT_DIR_PIN, RIGHT_EN_PIN);
BatteryManager   battery(BATTERY_VOLTAGE_PIN, BATTERY_R1_OHMS, BATTERY_R2_OHMS, BATTERY_LOW_THRESHOLD_V);
BluetoothManager bluetooth;
Telemetry telemetry;

// Controller parameters (Kp, Ki, Kd)
PIDGains anglePID    = {0.0f, 0.0f, 0.0f};    // inner loop: balance angle
PIDGains positionPID = {0.0f, 0.0f, 0.0f};    // outer loop: position hold
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

    telemetry.begin();
    telemetry.setParamCallback([](const String& name, float value) {
    if      (name == "kp")    controller.setAngleGains({value, anglePID.Ki, anglePID.Kd});
    else if (name == "ki")    controller.setAngleGains({anglePID.Kp, value, anglePID.Kd});
    else if (name == "kd")    controller.setAngleGains({anglePID.Kp, anglePID.Ki, value});
    else if (name == "posKp") controller.setPositionGains({value, positionPID.Ki, positionPID.Kd});
    else if (name == "posKi") controller.setPositionGains({positionPID.Kp, value, positionPID.Kd});
    else if (name == "posKd") controller.setPositionGains({positionPID.Kp, positionPID.Ki, value});
    });


    motors.begin();     // first, so the drivers are disabled during IMU calibration

    if (!imu.begin()) {
        while (true) {
            Serial.println("IMU nicht gefunden - bitte Verkabelung pruefen.");
            delay(1000);
        }
    }

    bluetooth.begin();

    lastControlTime  = micros();    // time for control loop at 200 Hz
    lastBatteryCheck = micros();    // battery check at 10 Hz, independent of the control tick
    Serial.println("System erfolgreich gestartet.");
}

void loop() {
    motors.run();   // emit step pulses as often as possible, independent of the control tick

    const unsigned long now = micros();

    // Bluepad32 must be updated continuously to process controller input.
    bluetooth.update();
    const uint8_t serialRequests = controller.handleSerialTuning();

    // Serial "stop" takes effect immediately, not only at the next control tick.
    if (serialRequests & ControlLoop::SERIAL_STOP) {
        serialStop = true;
        motors.enableMotors(false);
        controller.reset();
        armed = false;
    }
    if (serialRequests & ControlLoop::SERIAL_START) {
        serialStop = false;
    }

    // Temporarily disabled for testing; re-enable by removing #if 0 and #endif.
#if 0
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
#endif

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

    const bool joystickActive = bluetooth.isJoystickActive();   // true while the stick is deflected or the lean is still ramping down
    if (joystickActive) {
        motors.resetPositions();    // reset the position hold when the joystick is used, so the robot does not "jump" back to the old position
    }

    const float motorCommand = controller.computeCascade(   // returns wheel speed command in steps/s
        bluetooth.getDriveCommand(),      // desired lean angle in degrees
        motors.getLeftPosition(),
        motors.getRightPosition(),
        currentAngle,
        gyroRate,
        joystickActive,
        dt);

    telemetry.send(currentAngle, bluetooth.getDriveCommand(), motorCommand);
    motors.setSpeeds(-motorCommand, -motorCommand); // depends on the motor wiring; negative = forward
    
}