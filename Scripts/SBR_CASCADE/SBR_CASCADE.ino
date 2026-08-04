#include <AccelStepper.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Bluepad32.h>
// For my esp32 select DOIT ESP32 DEVKIT V1 board

ControllerPtr myControllers[BP32_MAX_GAMEPADS]; // PS4-Controller object

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
// Stepper interface type (1 = Driver with Step/Dir pins)
#define MOTOR_INTERFACE_TYPE 1
#define BATTERY_VOLTAGE_PIN 15  // Reads battery voltage in order to prevent undercharging

// Constants for battery voltage calculation
const float R1 = 32000.0; //Ohm
const float R2 = 10000.0;
const float RATIO = (R1 + R2) / R2;
const float ADC_REF = 3.3;  // Maximal voltage reading for esp32 pin
const float ADC_RES = 4095.0; // Resolution for the voltage reading

// Initialize the stepper motors
AccelStepper leftMotor(MOTOR_INTERFACE_TYPE, LEFT_STEP_PIN, LEFT_DIR_PIN);
AccelStepper rightMotor(MOTOR_INTERFACE_TYPE, RIGHT_STEP_PIN, RIGHT_DIR_PIN);
MPU6050 mpu6050(Wire);

// Inner Loop
float kp = 400.0; //100
float ki = 5.0; //0
float kd = 100.0; //50 200
// Variables for error calculations
float targetAngle = 0.0; //Angle which the cascade controller tries to hold
float error, lastError, integratedError, derivative;

float motorSpeed;
unsigned long lastTime;

// Constants for outer loop PID-Controller
float posKp = 0.05;
float posKi = 0.0;
float posKd = 0.0;
float posIntegral = 0.0;
float posLastError = 0.0;
float angleBias = 0.0;

// Microstepping, depending on MS1 and MS2 configuration
const float stepsPerRev = 200.0 * 8.0;
const float wheelDiameterMM = 65.0; // Wheel diameter
const float mmPerStep = (PI * wheelDiameterMM) / stepsPerRev;
unsigned long lastOuterTime = 0;
const unsigned long outerInterval = 20; // Value in ms -> 1/T -> 50Hz for 20 ms

// Angle bias safety clamp in degrees, safety-critical limit
const float MAX_ANGLE_BIAS = 3.0;

// Battery reading funtion
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

// PS4-Controller functions
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
    int stickY = ctl->axisY();  // left stick: negative = up, positive = down 
    int angleCorrection = 0;
    Serial.println(stickY);
    if (stickY < -50) {
        // Forward (-50 for avoiding stick drift of ps4 controller joystick)
        angleCorrection = map(stickY, -50, -512, 0, 8);
        return angleCorrection;
    }
    else if (stickY > 50) {
        // Reverse (50 for avoiding stick drift of ps4 controller joystick)
        angleCorrection = map(stickY, 50, 512, 0, -8);
        return angleCorrection;
    }
    else {
        angleCorrection = 0;
    }

    if (ctl->a()) {
        digitalWrite(LEFT_EN_PIN, HIGH);
        digitalWrite(RIGHT_EN_PIN, HIGH);
    }
    return angleCorrection;
}

// Outer Position Loop
// Produces a small angle bias that gets added to the drive command to form targetAngle.
void updateOuterLoop(bool joystickActive) {
  if (millis() - lastOuterTime < outerInterval) return; //returns nothing if outerInterval is not reached yet -> outer loop of cascade controller should be slower than inner loop
  float dt = (millis() - lastOuterTime) / 1000.0; 
  lastOuterTime = millis();

  if (joystickActive) {
    posIntegral = 0;
    posLastError = 0;
    angleBias = 0;
    leftMotor.setCurrentPosition(0);
    rightMotor.setCurrentPosition(0);
    return;
  }

  // Average of both wheels' traveled distance since last idle reset.
  float avgPos = (-leftMotor.currentPosition() + rightMotor.currentPosition()) / 2.0;
  float posMM = avgPos * mmPerStep;

  float posError = 0.0 - posMM; // setpoint = position where idle mode started
  posIntegral += posError * dt;
  posIntegral = constraint(posIntegral, -50, 50);
  float posDerivative = (dt > 0) ? (posError - posLastError) / dt : 0;

  angleBias = posKp * posError + posKi * posIntegral + posKd * posDerivative;
  angleBias = constraint(angleBias, -MAX_ANGLE_BIAS, MAX_ANGLE_BIAS);
  posLastError = posError;
}

void setup() {
  Serial.begin(115200);
  //PS4_CONTROLLER SETUP
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);
  BP32.forgetBluetoothKeys();  // Clears previous bluetooth pairings
  Wire.begin();                // Starts I2C communication for Gy-86 
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true); // Calculates offset in order to know what real 'zero' is
  Serial.println("Done!");

  //Max speed for the stepper motors
  leftMotor.setMaxSpeed(10000);
  rightMotor.setMaxSpeed(10000);
  leftMotor.setCurrentPosition(0);
  rightMotor.setCurrentPosition(0);

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

  lastOuterTime = millis();
}

void loop() {
  mpu6050.update(); //Updates gyroscope
  unsigned long currentTime = millis(); //Tracks time in ms
  BP32.update();  //Updates PS4-controller inputs
  handleSerialTuning();
  // Make these persist across loop iterations instead of resetting each time
  static float driveCommand = 0;
  static bool joystickActive = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
      ControllerPtr ctl = myControllers[i];
      if (ctl && ctl->isConnected() && ctl->hasData()) {
          driveCommand = processGamepad(ctl);
          joystickActive = (driveCommand != 0);
      }
  }
  // Outer loop runs at its own slower rate and only contributes a bias when idle
  updateOuterLoop(joystickActive);
  targetAngle = driveCommand + angleBias;

  //PID calculation at 500Hz
  if (currentTime - lastTime >= 2) {
    float currentAngle = mpu6050.getAngleX(); // Gets X-angle from the gy-86
    float gyroRateX = mpu6050.getGyroX();     // deg/s - raw rate, used for D-term instead of differenced error
    //Serial.println(currentAngle);
    // Error calculation
    error = currentAngle - targetAngle;
    // Partial term
    float P = kp * error;
    // Integral term
    integratedError += error;
    integratedError = constraint(integratedError, -500, 500); //prevents integratedError from rising too high while tipped over for example
    float I = ki * integratedError;
    derivative = gyroRateX;
    float D = kd * derivative;
    // PID-Output
    motorSpeed = P + I + D;

    // Kill motors if tipping angle goes over 40 degrees
    if (abs(currentAngle) > 40) {
      digitalWrite(LEFT_EN_PIN, HIGH);  // Kills motors by setting the enable pins on the motor drivers to HIGH to turn them off
      digitalWrite(RIGHT_EN_PIN, HIGH);
      motorSpeed = 0;                   // In case that turn off doesnt work, the motorspeed is set to 0
      // Reset both loops so recovery doesn't start from a wound-up integral
      integratedError = 0;
      posIntegral = 0;
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

  if (currentTime - lastTime >= 100) {
    Serial.print("motorSpeed: "); Serial.println(motorSpeed);
  }
  
}

// Function for keeping values in a defined range
float constraint(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;
}


// Function is used for tuning the robot over the serial monitor, so the code doesnt need to uploaded all the time
void handleSerialTuning() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return; 

    if (line.equalsIgnoreCase("show")) {  //Command "show" prints the current values of the controller
      Serial.println("---- Current gains ----");
      Serial.print("kp="); Serial.print(kp);
      Serial.print("  ki="); Serial.print(ki);
      Serial.print("  kd="); Serial.println(kd);
      Serial.print("posKp="); Serial.print(posKp);
      Serial.print("  posKi="); Serial.print(posKi);
      Serial.print("  posKd="); Serial.println(posKd);
      return;
    }

    int sepIdx = line.indexOf(' ');
    if (sepIdx == -1) sepIdx = line.indexOf('=');
    if (sepIdx == -1) {
      Serial.println("Format: <name> <value>   e.g. kp 450   or  type 'show'");
      return;
    }

    String name = line.substring(0, sepIdx);
    float val = line.substring(sepIdx + 1).toFloat();
    name.toLowerCase();

    if (name == "kp") kp = val; // Sets values for the changed values
    else if (name == "ki") ki = val;
    else if (name == "kd") kd = val;
    else if (name == "poskp") posKp = val;
    else if (name == "poski") posKi = val;
    else if (name == "poskd") posKd = val;
    else { Serial.println("Unknown parameter"); return; }

    Serial.print(name); Serial.print(" set to "); Serial.println(val);
  }
}
