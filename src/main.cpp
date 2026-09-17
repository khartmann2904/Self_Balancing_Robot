#include <Arduino.h>
#include "IMUManager.h"
#include "MotorManager.h"
#include "ControlLoop.h"
#include "BatteryManager.h"
#include "BluetoothManager.h"   

// Pin definitions for ESP32
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

// Resistor values for voltage divider
#define R1 32000.0f // Resistor R1 in ohms
#define R2 10000.0f // Resistor R2 in ohms

//Offset for the IMU pitch angle to account for sensor mounting orientation
#define IMU_PITCH_OFFSET 1.0f // Adjust this value based on the actual

// Battery monitoring voltage threshold
#define BATTERY_LOW_THRESHOLD 9.0f // Voltage in volts at which the motors should be disabled

// Instantiate objects
IMUManager imu(IMU_PITCH_OFFSET);  // Pass the pitch offset to the IMUManager constructor
MotorManager motors(LEFT_STEP_PIN, LEFT_DIR_PIN, LEFT_EN_PIN, RIGHT_STEP_PIN, RIGHT_DIR_PIN, RIGHT_EN_PIN);  //passes the values for both motors to the motor manager
BatteryManager battery(BATTERY_VOLTAGE_PIN, R1, R2, BATTERY_LOW_THRESHOLD);  //passes the values for the voltage divider to the battery manager
BluetoothManager bluetooth;  // Instance of the BluetoothManager class to handle Bluetooth communication and joystick input

// Controller parameters (Kp, Ki, Kd)
PIDGains anglePID = {20.0f, 0.5f, 1.2f};        //Outer Loop Values
PIDGains speedPID = {0.1f, 0.01f, 0.0f};        //Inner Loop Values
ControlLoop controller(anglePID, speedPID);     // Instance of the ControlLoop object with the PID parameters

unsigned long lastControlTime = 0;
unsigned long lastBatteryCheck = 0;
bool batteryLow = false;

void setup() {
    Serial.begin(115200);       //Needs to be checked if it lowers performance

    //MS PINS
    pinMode(LEFT_MS1_PIN, OUTPUT);
    pinMode(LEFT_MS2_PIN, OUTPUT);
    pinMode(RIGHT_MS1_PIN, OUTPUT);
    pinMode(RIGHT_MS2_PIN, OUTPUT);
    digitalWrite(LEFT_MS1_PIN, LOW);  // The combination of HIGH and LOW MS_PINS decides on the step size of the motors
    digitalWrite(RIGHT_MS1_PIN, LOW);
    digitalWrite(LEFT_MS2_PIN, LOW);
    digitalWrite(RIGHT_MS2_PIN, LOW);

    imu.begin();    // Initialize IMUManager
    motors.begin(); // Initialize MotorManager
    bluetooth.begin();  // Initialize BluetoothManager
    lastControlTime = micros();  // Initialize lastControlTime to the current time
    Serial.println("System erfolgreich gestartet.");
    
}

void loop() {
    unsigned long now = micros();

    // Bluepad32 must be updated continuously to process controller input.
    bluetooth.update();
    ControlLoop::handleSerialTuning(controller);  // Check for PID parameter updates from the serial interface

    //Battery voltage check and motor enable/disable based on battery status
    if ((now - lastBatteryCheck) >= 100000UL) {  // Check battery status every 100 ms
        lastBatteryCheck = now;
        //Prints the battery voltage to the serial monitor
        //battery.printBatteryStatus();
        // Check if battery is too low and disable motors if necessary
        batteryLow = battery.isBatteryLow();
        if (batteryLow) {
            Serial.println("Warnung: Batteriespannung niedrig! Motoren werden deaktiviert.");
            motors.enableMotors(false);
            return; // Skips the rest of the loop and goes back to the beginning of the loop
        }
    }

    // Run the control loop at a fixed frequency (e.g. 200 Hz = 5 ms)
    if ((now - lastControlTime) >= 1000UL) {
        float dt = (now - lastControlTime) / 1000000.0f;
        lastControlTime = now;

        imu.update();
        float currentAngle = imu.getPitch();
        //Serial.print("Current Angle: "); Serial.println(currentAngle);  // Debugging output for current angle
        float gyroRate = imu.getGyroX();

        // Safety cutoff in case of a fall (> 45 degrees)
        if (bluetooth.isEmergencyStopPressed() || abs(currentAngle) > 45.0f) {
            motors.enableMotors(false);
            controller.reset(); // Reset the controller to prevent integral windup
            Serial.println("Emergency stop activated or robot fell over. Motors disabled.");
            return; // Skips the rest of the loop and goes back to the beginning of the loop
        } else {
            motors.enableMotors(true);
        }

        float targetSpeed = bluetooth.getDriveCommand();
        if (bluetooth.isJoystickActive()) {
            motors.resetPositions();
        }

        static unsigned long lastTargetSpeedPrint = 0;
        if (millis() - lastTargetSpeedPrint >= 500) {
            lastTargetSpeedPrint = millis();
            Serial.print("Target Speed: ");
            Serial.println(targetSpeed);
        }


        float motorCommand = controller.computeCascade(
            targetSpeed,
            motors.getLeftPosition(),
            motors.getRightPosition(),
            currentAngle,
            gyroRate,
            bluetooth.isJoystickActive(),
            dt);
        motors.setSpeeds(motorCommand, motorCommand);  // The sign must be checked depending on how the motors are connected. If the direction is incorrect, simply swap the pins
    }
}