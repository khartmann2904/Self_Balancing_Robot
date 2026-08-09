#include "ControlLoop.h"

ControlLoop::ControlLoop(PIDGains anglePID, PIDGains speedPID)
    : angleGains(anglePID), speedGains(speedPID), speedIntegral(0), angleIntegral(0), lastAngleError(0) {}

float ControlLoop::computeCascade(float targetSpeed, float currentSpeed, float currentAngle, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // 1. Äußerer Regler: Geschwindigkeit -> Soll-Winkel
    float speedError = targetSpeed - currentSpeed;
    speedIntegral += speedError * dt;
    float targetAngle = (speedGains.Kp * speedError) + (speedGains.Ki * speedIntegral);

    // 2. Innerer Regler: Winkel -> Motoransteuerung (Drehmoment/Beschleunigung)
    float angleError = targetAngle - currentAngle;
    angleIntegral += angleError * dt;
    float angleDerivative = (angleError - lastAngleError) / dt;
    lastAngleError = angleError;

    float output = (angleGains.Kp * angleError) + (angleGains.Ki * angleIntegral) + (angleGains.Kd * angleDerivative);

    return output;
}