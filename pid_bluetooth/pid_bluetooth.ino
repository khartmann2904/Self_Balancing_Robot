#include <AccelStepper.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS]; //for the ps4 controller
// Pin Definitions (Matches your KiCad Schematic)
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
// Define the stepper interface type (1 = Driver with Step/Dir pins)
#define MOTOR_INTERFACE_TYPE 1
#define BATTERY_VOLTAGE_PIN 15  //reads battery voltage in order to prevent undercharging

const float R1 = 32000.0;
const float R2 = 10000.0;
const float RATIO = (R1 + R2) / R2;  // = 4.3
const float ADC_REF = 3.3;
const float ADC_RES = 4095.0;

// Initialize the steppers
AccelStepper leftMotor(MOTOR_INTERFACE_TYPE, LEFT_STEP_PIN, LEFT_DIR_PIN);
AccelStepper rightMotor(MOTOR_INTERFACE_TYPE, RIGHT_STEP_PIN, RIGHT_DIR_PIN);
MPU6050 mpu6050(Wire);

//PID Constants
float kp = 30.0;   // Start small (try 10-50)
float ki = 0.1;    // Start very small (try 0.1-1.0)
float kd = 2.5;    // Start small (try 1.0-5.0)

float targetAngle = 0.0; // The "Perfectly Level" goal
float error, lastError, integratedError, derivative;
float motorSpeed;

unsigned long lastTime;

//##########BatteryVoltage################
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
//#############PS4_CONTROLLER######################
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
    // Get r2 and l2 button values (0-1023)
    int r2 = ctl->throttle();   // Left stick Y-axis (forward/backward)
    int l2 = ctl->brake();   // Left stick X-axis (left/right)
    int angleCorrection = 0;
    // Simple tank drive control
    if (r2 > 0) {
        // Forward
        int angleCorrection = map(abs(r2), 0, 1023, 0, 5);  //this translates the joystick to a 5 degree angle
        Serial.println(angleCorrection);
        return angleCorrection;
    } 
    else if (l2 > 0) {
        // Reverse
        int angleCorrection = map(abs(l2), 0, 1023, 0, -5);  //this translates the joystick to a 5 degree angle
        Serial.println(angleCorrection);
        return angleCorrection;
    }
    else{
      targetAngle= 0.0;
      angleCorrection = 0; 
    }
    // Button example - emergency stop
    if (ctl->a()) { 
        // Stop all motors, if x button on ps4-controller gets pressed
        digitalWrite(LEFT_EN_PIN, HIGH);  //EN_PINS get activated to shut off motors
        digitalWrite(RIGHT_EN_PIN, HIGH);
    }
    return angleCorrection;
}



void setup() {
 // Set a constant speed (Steps per second)
  // 800 steps/sec = 1/4 turn per second at 1/16 microstepping
  Serial.begin(115200);
  //PS4_CONTROLLER SETUP
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();  // Optional: clear previous pairings

  Wire.begin(); // Starts I2C communication
  
  mpu6050.begin();
  // This calculates the 'offset' so your robot knows what 'zero' is.
  // Make sure the GY-87 is perfectly level during this!
  mpu6050.calcGyroOffsets(true); 
  
  Serial.println("Done!");


  leftMotor.setMaxSpeed(10000);  // Higher ceiling

  rightMotor.setMaxSpeed(10000);

  pinMode(LEFT_EN_PIN, OUTPUT); 
  pinMode(RIGHT_EN_PIN, OUTPUT);
  pinMode(LEFT_MS1_PIN, OUTPUT);  //The combination of HIGH and LOW MS_PINS decides on the step size of the motors
  pinMode(RIGHT_MS1_PIN, OUTPUT);
  pinMode(LEFT_MS2_PIN, OUTPUT);
  pinMode(RIGHT_MS2_PIN, OUTPUT);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);

  digitalWrite(LEFT_EN_PIN, LOW); //Turns on the motors
  digitalWrite(RIGHT_EN_PIN, LOW);

  digitalWrite(LEFT_MS1_PIN, LOW);  //The combination of HIGH and LOW MS_PINS decides on the step size of the motors
  digitalWrite(RIGHT_MS1_PIN, LOW);
  digitalWrite(LEFT_MS2_PIN, LOW);
  digitalWrite(RIGHT_MS2_PIN, LOW);

}
void loop() {
  mpu6050.update(); //updates gyroscope
  unsigned long currentTime = millis(); //tracks time

  BP32.update();  //updates ps4-controller
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
      ControllerPtr ctl = myControllers[i];
      if (ctl && ctl->isConnected() && ctl->hasData()) {
          targetAngle = processGamepad(ctl);
      }
  }
  
  // PID runs every 5ms
  if (currentTime - lastTime >= 5) {
    float currentAngle = mpu6050.getAngleX();
    
    // 1. Calculate Error
    error = currentAngle - targetAngle;
    
    // 2. Proportional Term
    float P = kp * error;
    
    // 3. Integral Term
    integratedError += error;
    integratedError = constraint(integratedError, -500, 500); 
    float I = ki * integratedError;
    
    // 4. Derivative Term
    derivative = error - lastError;
    float D = kd * derivative;
    
    // 5. Total Output
    motorSpeed = P + I + D;
    
    // Safety: Kill motors if tipped too far
    if (abs(currentAngle) > 40) {
      digitalWrite(LEFT_EN_PIN, HIGH);
      digitalWrite(RIGHT_EN_PIN, HIGH);
      motorSpeed = 0;
    } else {
      digitalWrite(LEFT_EN_PIN, LOW);
      digitalWrite(RIGHT_EN_PIN, LOW);
    }

    // Apply to motors
    leftMotor.setSpeed(-motorSpeed);
    rightMotor.setSpeed(motorSpeed);
    
    lastError = error;
    lastTime = currentTime;
    
    // Debug output (optional - can slow things down)
    //Serial.print("Angle: "); Serial.print(currentAngle);
    //Serial.print(" | Speed: "); Serial.println(motorSpeed);
  }

  // CRITICAL: These must run every loop iteration
  leftMotor.runSpeed();
  rightMotor.runSpeed();
  
  // Battery check - only once per second, NON-BLOCKING
  static unsigned long lastBatteryCheck = 0;
  if (currentTime - lastBatteryCheck >= 1000) {
    static int sampleCount = 0;
    static long voltageSum = 0;
    
    // Take one sample per loop, accumulate over time
    if (sampleCount < 64) {
      voltageSum += analogRead(BATTERY_VOLTAGE_PIN);
      sampleCount++;
    } else {
      // Calculate voltage from accumulated samples
      float vout = ((voltageSum / 64.0) / ADC_RES) * ADC_REF;
      float voltage = vout * RATIO * (10.5 / 12.8);
      
      Serial.print("Battery: "); Serial.println(voltage);
      
      if(voltage <= 9.0) {
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