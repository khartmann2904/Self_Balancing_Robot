#include <AccelStepper.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
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
float kp = 50.0;   // Start small (try 10-50)
float ki = 0.3;    // Start very small (try 0.1-1.0)
float kd = 2.5;    // Start small (try 1.0-5.0)

float targetAngle = 0.0; // The "Perfectly Level" goal
float error, lastError, integratedError, derivative;
float motorSpeed;

unsigned long lastTime;

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

void setup() {
 // Set a constant speed (Steps per second)
  // 800 steps/sec = 1/4 turn per second at 1/16 microstepping
  Serial.begin(115200);
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
  mpu6050.update(); //gets new angle data
  unsigned long currentTime = millis(); //tracks time every loop
  
  // Run the PID math every 5 milliseconds
  if (currentTime - lastTime >= 5) {
    float currentAngle = mpu6050.getAngleX();
    // 1. Calculate Error
    error = currentAngle - targetAngle;
    //Serial.println(error);
    // 2. Proportional Term
    float P = kp * error;
    
    // 3. Integral Term (Accumulates over time)
    integratedError += error;
    // "Windup" Protection: prevent the I-term from getting too huge
    integratedError = constraint(integratedError, -500, 500); 
    float I = ki * integratedError;
    
    // 4. Derivative Term (Change in error / time)
    derivative = error - lastError;
    float D = kd * derivative;
    
    // 5. Total Output
    motorSpeed = P + I + D;
    //Serial.println(motorSpeed);
    
    // Safety: If it falls over 45 degrees, kill the motors
    if (abs(currentAngle) > 40) {
      digitalWrite(LEFT_EN_PIN, HIGH);
      digitalWrite(RIGHT_EN_PIN, HIGH);
      motorSpeed = 0;
    } else {
      digitalWrite(LEFT_EN_PIN, LOW);
      digitalWrite(RIGHT_EN_PIN, LOW);
    }

    // Apply to motors
    Serial.print("MotorSpeed: ");
    Serial.println(motorSpeed);
    Serial.print("CurrentAngle: ");
    Serial.println(currentAngle);
    leftMotor.setSpeed(motorSpeed);
    rightMotor.setSpeed(-motorSpeed); // One motor is reversed, but depends on the geometry of the robot
    
    lastError = error;
    lastTime = currentTime;
    Serial.print("Battery: ");
    Serial.println(readBatteryVoltage()); //displays battery voltage
    if(readBatteryVoltage() <= 9){
      digitalWrite(LEFT_EN_PIN, HIGH); //Turns off the motors if battery voltage is below 9V
      digitalWrite(RIGHT_EN_PIN, HIGH);

    }
  }

  leftMotor.runSpeed();
  rightMotor.runSpeed();

}

// Simple helper to keep numbers in range
float constraint(float val, float minVal, float maxVal) {
  if (val < minVal) return minVal;
  if (val > maxVal) return maxVal;
  return val;



}