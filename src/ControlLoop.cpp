#include "ControlLoop.h"

ControlLoop::ControlLoop(PIDGains anglePID, PIDGains speedPID)
    : angleGains(anglePID), speedGains(speedPID), speedIntegral(0), angleIntegral(0), lastAngleError(0) {}

float ControlLoop::computeCascade(float targetSpeed, float currentSpeed, float currentAngle, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // 1. Outer controller: speed -> target angle
    float speedError = targetSpeed - currentSpeed;
    speedIntegral += speedError * dt;
    float targetAngle = (speedGains.Kp * speedError) + (speedGains.Ki * speedIntegral);

    // 2. Inner controller: angle -> motor control (torque/acceleration)
    float angleError = targetAngle - currentAngle;
    angleIntegral += angleError * dt;
    float angleDerivative = (angleError - lastAngleError) / dt;
    lastAngleError = angleError;

    float output = (angleGains.Kp * angleError) + (angleGains.Ki * angleIntegral) + (angleGains.Kd * angleDerivative);

    return output;
}

void ControlLoop::setAngleGains(PIDGains gains) {
    angleGains = gains;
}

void handleSerialTuning(ControlLoop& controller) {  // Function for receiving PID parameters through the serial interface
    static String inputString = "";
    static bool stringComplete = false;

    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            stringComplete = true;
            break;
        } else {
            inputString += inChar;
        }
    }

    if (stringComplete) {
        // Parse the input string for PID parameters
        float Kp, Ki, Kd;
        int parsed = sscanf(inputString.c_str(), "Kp:%f Ki:%f Kd:%f", &Kp, &Ki, &Kd);
        if (parsed == 3) {
            controller.setAngleGains({Kp, Ki, Kd});
            Serial.println("PID parameters updated.");
        } else {
            Serial.println("Invalid input format. Use: Kp:<value> Ki:<value> Kd:<value>");
        }
        // Clear the input string and reset the flag
        inputString = "";
        stringComplete = false;
    }
}