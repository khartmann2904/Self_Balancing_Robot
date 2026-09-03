#include "ControlLoop.h"

ControlLoop::ControlLoop(PIDGains anglePID, PIDGains speedPID)
        : angleGains(anglePID), speedGains(speedPID), speedIntegral(0), angleIntegral(0),
            lastAngleError(0), lastPositionError(0) {}

float ControlLoop::computeCascade(float driveCommand, long leftPosition, long rightPosition,
                                  float currentAngle, float gyroRate, bool joystickActive, float dt) {
    if (dt <= 0.0f) {
        return 0.0f;
    }

    // While driving, the command is the desired lean angle. Position hold resumes from zero when idle.
    float angleBias = 0.0f;
    if (joystickActive) {
        speedIntegral = 0.0f;
        lastPositionError = 0.0f;
    } else {
        const float stepsPerRevolution = 200.0f * 8.0f;
        const float wheelDiameterMM = 116.0f;
        const float mmPerStep = (PI * wheelDiameterMM) / stepsPerRevolution;
        const float averagePosition = (-leftPosition + rightPosition) / 2.0f;
        const float positionError = -(averagePosition * mmPerStep);

        speedIntegral += positionError * dt;
        speedIntegral = constrain(speedIntegral, -50.0f, 50.0f);
        const float positionDerivative = (positionError - lastPositionError) / dt;
        angleBias = (speedGains.Kp * positionError)
                  + (speedGains.Ki * speedIntegral)
                  + (speedGains.Kd * positionDerivative);
        angleBias = constrain(angleBias, -3.0f, 3.0f);
        lastPositionError = positionError;
    }

    const float targetAngle = driveCommand + angleBias;

    // The gyro rate is used directly for the derivative term, as in the original controller.
    const float angleError = currentAngle - targetAngle;
    angleIntegral += angleError * dt;
    angleIntegral = constrain(angleIntegral, -500.0f, 500.0f);

    return (angleGains.Kp * angleError)
         + (angleGains.Ki * angleIntegral)
         + (angleGains.Kd * gyroRate);
}

void ControlLoop::setAngleGains(PIDGains gains) {
    angleGains = gains;
}

void ControlLoop::setSpeedGains(PIDGains gains) {
    speedGains = gains;
}

void ControlLoop::reset() {
    speedIntegral = 0.0f;
    angleIntegral = 0.0f;
    lastAngleError = 0.0f;
    lastPositionError = 0.0f;
}

void ControlLoop::handleSerialTuning(ControlLoop& controller) {  // Function for receiving PID parameters through the serial interface
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