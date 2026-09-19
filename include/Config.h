#pragma once    // include file only once in a single compilation
#include <Arduino.h>

// ---------------------------------------------------------------- Pins (ESP32)
#define LEFT_STEP_PIN        16
#define LEFT_DIR_PIN         17
#define LEFT_EN_PIN          25
#define LEFT_MS1_PIN         33
#define LEFT_MS2_PIN         32
#define RIGHT_STEP_PIN       27
#define RIGHT_DIR_PIN        26
#define RIGHT_EN_PIN          2   // strapping pin: if boot problems occur, move it
#define RIGHT_MS1_PIN        12   // strapping pin: if boot problems occur, move it
#define RIGHT_MS2_PIN        14
#define BATTERY_VOLTAGE_PIN  15   // strapping pin, but fine for ADC use

// ---------------------------------------------------------------- Battery
constexpr float         BATTERY_R1_OHMS          = 32000.0f;   // divider: top resistor
constexpr float         BATTERY_R2_OHMS          = 10000.0f;   // divider: bottom resistor,
constexpr float         BATTERY_LOW_THRESHOLD_V  = 9.6f;       // set for your pack (3S LiPo: ~9.6 V is safer)
constexpr uint8_t       BATTERY_LOW_SAMPLES      = 5;          // consecutive low readings before latching
constexpr unsigned long BATTERY_CHECK_PERIOD_US  = 100000UL;   // 10 Hz, UL -> unsigned long

// ---------------------------------------------------------------- IMU
constexpr float IMU_PITCH_OFFSET_DEG = 1.0f;    // mounting offset so 0 deg = balance point
constexpr float IMU_GYRO_WEIGHT      = 0.98f;   // complementary filter: gyro share

// ---------------------------------------------------------------- Mechanics
// The MS pins are set LOW/LOW in setup(), which is 8x microstepping on a TMC2209.
constexpr float STEPS_PER_REV    = 200.0f;
constexpr float MICROSTEPS       = 8.0f;
constexpr float WHEEL_DIAMETER_MM = 116.0f;
constexpr float MM_PER_STEP      = (PI * WHEEL_DIAMETER_MM) / (STEPS_PER_REV * MICROSTEPS);

// ---------------------------------------------------------------- Motors
// Upper limit for the step rate in steps/s. runSpeed() emits at most one step per
// call, so the real limit is also how often loop() runs.
constexpr float MAX_STEP_RATE = 4000.0f;

// ---------------------------------------------------------------- Control
constexpr unsigned long CONTROL_PERIOD_US = 5000;   // 200 Hz
constexpr float FALL_ANGLE_DEG            = 45.0f;  // motors off beyond this
constexpr float ARM_ANGLE_DEG             = 3.0f;   // must be within this to (re-)arm

constexpr float POSITION_BIAS_LIMIT_DEG   = 3.0f;   // max lean the position loop may request
constexpr float POSITION_INTEGRAL_LIMIT   = 50.0f;  // mm*s
constexpr float ANGLE_I_MAX_OUT           = 100.0f; // max contribution of the angle I-term (steps/s)

// Flip to -1.0f if the robot runs away instead of returning when pushed.
constexpr float POSITION_SIGN             = 1.0f;

// ---------------------------------------------------------------- Bluetooth / joystick
constexpr int   STICK_DEADZONE         = 60;      // Bluepad32 axes: about -511..512
constexpr int   STICK_MAX              = 512;
constexpr float MAX_LEAN_DEG           = 8.0f;    // lean angle at full stick
constexpr float DRIVE_SLEW_DEG_PER_S   = 30.0f;   // how fast the lean command may change
constexpr bool  BT_FORGET_KEYS_ON_BOOT = false;   // true = re-pair the controller every boot
