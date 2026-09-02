#include <Arduino.h>
#include "IMUManager.h"
#include "MotorManager.h"
#include "ControlLoop.h"
#include "BatteryManager.h"
#include "BluetoothManager.h"   //Still need to implement the BluetoothManager class and its methods

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

// Battery monitoring voltage threshold
#define BATTERY_LOW_THRESHOLD 9.0f // Voltage in volts at which the motors should be disabled

// Instantiate objects
IMUManager imu;
MotorManager motors(LEFT_STEP_PIN, LEFT_DIR_PIN, RIGHT_STEP_PIN, RIGHT_DIR_PIN, RIGHT_EN_PIN);  //passes the values for both motors to the motor manager
BatteryManager battery(BATTERY_VOLTAGE_PIN, R1, R2, BATTERY_LOW_THRESHOLD);  //passes the values for the voltage divider to the battery manager
BluetoothManager bluetooth;  // Instance of the BluetoothManager class to handle Bluetooth communication and joystick input

// Controller parameters (Kp, Ki, Kd)
PIDGains anglePID = {20.0f, 0.5f, 1.2f};        //Outer Loop Values
PIDGains speedPID = {0.1f, 0.01f, 0.0f};        //Inner Loop Values
ControlLoop controller(anglePID, speedPID);     // Instance of the ControlLoop object with the PID parameters

unsigned long lastLoopTime = 0;

void setup() {
    Serial.begin(115200);       //Needs to be checked if it lowers performance

    //MS PINS
    digitalWrite(LEFT_MS1_PIN, LOW);  // The combination of HIGH and LOW MS_PINS decides on the step size of the motors
    digitalWrite(RIGHT_MS1_PIN, LOW);
    digitalWrite(LEFT_MS2_PIN, LOW);
    digitalWrite(RIGHT_MS2_PIN, LOW);
    //The combination of HIGH and LOW MS_PINS decides on the step size of the motors -> TMC2209-Datasheet

    imu.begin();    // Initialize IMUManager
    motors.begin(); // Initialize MotorManager
    bluetooth.begin();  // Initialize BluetoothManager

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

    // Run the control loop at a fixed frequency (e.g. 200 Hz = 5 ms)
    if (dt >= 0.005f) {
        lastLoopTime = now;

        imu.update();
        float currentAngle = imu.getPitch();

        // Safety cutoff in case of a fall (> 45 degrees)
        if (abs(currentAngle) > 45.0f) {
            motors.enableMotors(false);
            return;
        } else {
            motors.enableMotors(true);
        }

        float motorCommand = controller.computeCascade(0.0f, 0.0f, currentAngle, dt); //needs to be worked on to get the speed from the bluetooth controller and the current speed from the encoders
        motors.setSpeeds(-motorCommand, motorCommand);  // The sign must be checked depending on how the motors are connected. If the direction is incorrect, simply swap the pins
    }
}