#include <AccelStepper.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Bluepad32.h>
//For my esp32 select DOIT ESP32 DEVKIT V1 board

ControllerPtr myControllers[BP32_MAX_GAMEPADS]; //PS4-Controller object
// Pin Definitions
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
//Stepper interface type (1 = Driver with Step/Dir pins)
#define MOTOR_INTERFACE_TYPE 1
#define BATTERY_VOLTAGE_PIN 15  //reads battery voltage in order to prevent undercharging

//Constants for battery voltage calculation
const float R1 = 32000.0; 
const float R2 = 10000.0;
const float RATIO = (R1 + R2) / R2;  // = 4.3
const float ADC_REF = 3.3;
const float ADC_RES = 4095.0;

// Initialize the steppers
AccelStepper leftMotor(MOTOR_INTERFACE_TYPE, LEFT_STEP_PIN, LEFT_DIR_PIN);
AccelStepper rightMotor(MOTOR_INTERFACE_TYPE, RIGHT_STEP_PIN, RIGHT_DIR_PIN);
MPU6050 mpu6050(Wire);

//PID Constants (400.0, 1.0, 180.0 works best so far)
float kp = 400.0;
float ki = 1.0;   
float kd = 180.0;
//Variables for error calculations
float targetAngle = 0.0; //PID controller tries to control the robot to stay at 0 degrees -> upright position
float error, lastError, integratedError, derivative;  
float motorSpeed;

unsigned long lastTime;

//Battery reading funtion
float readBatteryVoltage() {
    long sum = 0;
    for (int i = 0; i < 64; i++) {
        sum += analogRead(BATTERY_VOLTAGE_PIN);
        delay(5);
    }
    float vout = ((sum / 64.0) / ADC_RES) * ADC_REF;
    float voltage = vout * RATIO;
    return voltage * (10.5 / 12.8); // correction factor = 0.777
}

//PS4-Controller functions
void onConnectedController(ControllerPtr ctl) {
    if (myControllers[0] == nullptr) {
        Serial.println("Controller connected!");
        myControllers[0] = ctl;
        ctl->setColorLED(0, 255, 0); // Green LED on PS4 controller
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.println("Controller disconnected");
            myControllers[i] = nullptr;
        }
    }
}

float processGamepad(ControllerPtr ctl) {
    // 1. Check emergency button FIRST. 
    // Otherwise, returning early from joysticks bypasses it!
    if (ctl->a()) { 
        digitalWrite(LEFT_EN_PIN, HIGH);  
        digitalWrite(RIGHT_EN_PIN, HIGH);
    }

    int r2 = ctl->throttle();   // Forward trigger
    int l2 = ctl->brake();      // Reverse trigger
    
    // CHANGE THIS TO FLOAT so decimals aren't chopped off!
    float angleCorrection = 0.0; 

    // 2. Drive Controls
    if (r2 > 50) { // Added a small deadzone of 50
        // If it falls flat on its face when moving forward, change the '-' to '+'
        angleCorrection = -(map(r2, 0, 1023, 0, 100) / 100.0);  // Smooth 0.0 to -4.0 degrees
    } 
    else if (l2 > 50) {
        // If it falls flat on its back when moving backward, change the '+' to '-'
        angleCorrection = (map(l2, 0, 1023, 0, 100) / 100.0);   // Smooth 0.0 to 4.0 degrees
    }
    else {
        angleCorrection = 0.0; 
    }

    return angleCorrection;
}

void setup() {
  Serial.begin(115200);
  //PS4_CONTROLLER SETUP
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();  // Clears previous bluetooth pairings
  Wire.begin();                // Starts I2C communication for Gy-86
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true); // Calculates offset in order to know what real 'zero' is; Robot should be upright for this adjustment
  Serial.println("Done!");

  //Max speed for the stepper motors
  leftMotor.setMaxSpeed(4000);  
  rightMotor.setMaxSpeed(4000);

  //PINMODES
  pinMode(LEFT_EN_PIN, OUTPUT); 
  pinMode(RIGHT_EN_PIN, OUTPUT);
  pinMode(LEFT_MS1_PIN, OUTPUT);  
  pinMode(RIGHT_MS1_PIN, OUTPUT); 
  pinMode(LEFT_MS2_PIN, OUTPUT);
  pinMode(RIGHT_MS2_PIN, OUTPUT);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  //EN PINS
  digitalWrite(LEFT_EN_PIN, LOW); // Turns on the motors if set on LOW
  digitalWrite(RIGHT_EN_PIN, LOW);
  //MS PINS
  digitalWrite(LEFT_MS1_PIN, LOW);  // The combination of HIGH and LOW MS_PINS decides on the step size of the motors
  digitalWrite(RIGHT_MS1_PIN, LOW);
  digitalWrite(LEFT_MS2_PIN, LOW);
  digitalWrite(RIGHT_MS2_PIN, LOW);
  //The combination of HIGH and LOW MS_PINS decides on the step size of the motors -> TMC2209-Datasheet

}
void loop() {
  mpu6050.update(); //Updates gyroscope
  unsigned long currentTime = millis(); //Tracks time in ms
  BP32.update();  //Updates PS4-controller inputs
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
      ControllerPtr ctl = myControllers[i];
      if (ctl && ctl->isConnected() && ctl->hasData()) {
          targetAngle = processGamepad(ctl);
      }
  }
  //PID calculation at 500Hz
  if (currentTime - lastTime >= 2) {
    float currentAngle = mpu6050.getAngleX(); // Gets X-angle from the gy-86
    // Error calculation
    error = currentAngle - targetAngle;
    // Partial term
    float P = kp * error;
    // Integral term
    integratedError += error;
    integratedError = constraint(integratedError, -200, 200);
    // Optional: slowly decay the integral when error is tiny
    if (abs(error) < 0.5) {
        integratedError *= 0.95; 
    }
    float I = ki * integratedError;
    // Derivative term
    derivative = error - lastError;
    float D = kd * derivative;
    // PID-Output
    motorSpeed = P + I + D;
    
    // Kill motors if tipping angle goes over 40 degrees
    if (abs(currentAngle) > 40) {
      digitalWrite(LEFT_EN_PIN, HIGH);  // Kills motors by setting the enable pins on the motor drivers to HIGH to turn them off
      digitalWrite(RIGHT_EN_PIN, HIGH);
      motorSpeed = 0;                   // In case that turn off doesnt work, the motorspeed is set to 0
    } else {
      digitalWrite(LEFT_EN_PIN, LOW);
      digitalWrite(RIGHT_EN_PIN, LOW);
    }

    //Apply speed to motors
    leftMotor.setSpeed(-motorSpeed);
    rightMotor.setSpeed(motorSpeed);
    lastError = error;
    lastTime = currentTime;
  }
  leftMotor.runSpeed(); //Starts to move the motors with setSpeed
  rightMotor.runSpeed();
  
  // Battery check every second
  static unsigned long lastBatteryCheck = 0;
  if (currentTime - lastBatteryCheck >= 1000) {
    static int sampleCount = 0;
    static long voltageSum = 0;
    //One sample per loop -> tracks the voltage over time
    if (sampleCount < 64) {
      voltageSum += analogRead(BATTERY_VOLTAGE_PIN);
      sampleCount++;  //increase sample count
    } else {
      // Calculate voltage from accumulated samples
      float vout = ((voltageSum / 64.0) / ADC_RES) * ADC_REF; //gets the average battery voltage over 64 samples 
      float voltage = vout * RATIO * (10.5 / 12.8);
      Serial.print("Battery: "); Serial.println(voltage);
      if(voltage <= 9.0) {    //turns off the motors if battery voltage is under 9V
        digitalWrite(LEFT_EN_PIN, HIGH);
        digitalWrite(RIGHT_EN_PIN, HIGH);
      }
      // Reset for next reading
      sampleCount = 0;
      voltageSum = 0;
      lastBatteryCheck = currentTime;
    }
  }
}

// Simple helper to keep numbers in range
float constraint(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}