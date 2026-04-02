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

// Initialize the steppers
AccelStepper leftMotor(MOTOR_INTERFACE_TYPE, LEFT_STEP_PIN, LEFT_DIR_PIN);
AccelStepper rightMotor(MOTOR_INTERFACE_TYPE, RIGHT_STEP_PIN, RIGHT_DIR_PIN);
MPU6050 mpu6050(Wire);

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
  leftMotor.setAcceleration(2000); // How fast it reaches top speed
  leftMotor.setSpeed(8000);      // The actual target speed

  rightMotor.setMaxSpeed(4000);
  rightMotor.setSpeed(400); 
  //Hier müssen noch beide en auf masse gelegt werden
  //MS1 und MS2 PINS auf HIGH
  pinMode(LEFT_EN_PIN, OUTPUT);
  pinMode(RIGHT_EN_PIN, OUTPUT);
  pinMode(LEFT_MS1_PIN, OUTPUT);
  pinMode(RIGHT_MS1_PIN, OUTPUT);
  pinMode(LEFT_MS2_PIN, OUTPUT);
  pinMode(RIGHT_MS2_PIN, OUTPUT);
  
  digitalWrite(LEFT_EN_PIN, LOW);
  digitalWrite(RIGHT_EN_PIN, LOW);

  digitalWrite(LEFT_MS1_PIN, LOW);
  digitalWrite(RIGHT_MS1_PIN, LOW);
  digitalWrite(LEFT_MS2_PIN, LOW);
  digitalWrite(RIGHT_MS2_PIN, LOW);
}

void loop() {
  mpu6050.update();

  // We care about the 'Angle Y' or 'Angle X' depending on 
  // how you mounted the sensor on your robot frame.
  float tiltAngle = mpu6050.getAngleY(); 

  Serial.print("Tilt Angle: ");
  Serial.println(tiltAngle);
  // runSpeed() tells the motor to spin at the setSpeed() indefinitely
  leftMotor.runSpeed();
  rightMotor.runSpeed();
}