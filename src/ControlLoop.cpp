#include "ControlLoop.h"
#include "Config.h"
#include <string.h> // Libraries for handleSerialTuning
#include <stdlib.h>
#include <ctype.h>

ControlLoop::ControlLoop(PIDGains anglePID, PIDGains positionPID)
    : angleGains(anglePID), positionGains(positionPID),
      positionIntegral(0.0f), angleIntegral(0.0f), lastPositionError(0.0f),
      positionInitialized(false), stopRequested(false), startRequested(false), serialLen(0) {}

float ControlLoop::computeCascade(float targetAngleCmd, long leftPosition, long rightPosition,
                                  float currentAngle, float gyroRate, bool joystickActive, float dt) {
    if (dt <= 0.0f) {   // avoid division by zero or negative time, but this should never happen if the control loop is called at a fixed period
        return 0.0f;
    }

    // Outer loop: position hold. While driving, it is switched off and restarts from zero.
    float angleBias = 0.0f;
    if (joystickActive) {
        positionIntegral = 0.0f;
        positionInitialized = false;
    } else {
        // Both motors get the same signed speed, so both positions count in the same
        // direction when driving straight -> average is (L + R) / 2.
        const float averagePosition = (static_cast<float>(leftPosition) + static_cast<float>(rightPosition)) / 2.0f; // average position in steps
        const float positionError = -POSITION_SIGN * (averagePosition * MM_PER_STEP);   // convert to mm and flip sign if needed

        if (!positionInitialized) {          // avoid a derivative spike on the first sample
            lastPositionError = positionError;
            positionInitialized = true;
        }

        positionIntegral += positionError * dt; // integrate the position error over time
        positionIntegral = constrain(positionIntegral, -POSITION_INTEGRAL_LIMIT, POSITION_INTEGRAL_LIMIT); // limit the integral to avoid windup
        const float positionDerivative = (positionError - lastPositionError) / dt;  // derivative of the position error
        lastPositionError = positionError;  // save for the next iteration

        angleBias = (positionGains.Kp * positionError)  
                  + (positionGains.Ki * positionIntegral)
                  + (positionGains.Kd * positionDerivative);    // convert position error to a lean angle command
        angleBias = constrain(angleBias, -POSITION_BIAS_LIMIT_DEG, POSITION_BIAS_LIMIT_DEG);
    }

    const float targetAngle = targetAngleCmd + angleBias;   // the target lean angle is the joystick command plus the position hold bias

    // Inner loop: the gyro rate is used directly as the derivative term.
    const float angleError = currentAngle - targetAngle;
    angleIntegral += angleError * dt;
    if (angleGains.Ki > 0.0f) {
        // Limit the I-term's contribution to the output, not the raw integral.
        const float limit = ANGLE_I_MAX_OUT / angleGains.Ki;
        angleIntegral = constrain(angleIntegral, -limit, limit);
    } else {
        angleIntegral = 0.0f;
    }

    return (angleGains.Kp * angleError)
         + (angleGains.Ki * angleIntegral)
         + (angleGains.Kd * gyroRate);
}

void ControlLoop::setAngleGains(PIDGains gains)    { angleGains = gains; }
void ControlLoop::setPositionGains(PIDGains gains) { positionGains = gains; }

void ControlLoop::reset() { // clears the integrals and position error of the cascade controller
    positionIntegral = 0.0f;
    angleIntegral = 0.0f;
    lastPositionError = 0.0f;
    positionInitialized = false;
}


void ControlLoop::handleSerialTuning() {    // Handles
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
            if (serialLen > 0) {
                serialBuf[serialLen] = '\0';
                serialLen = 0;
                processTuningLine(serialBuf);
            }
        } else if (serialLen < sizeof(serialBuf) - 1) {
            serialBuf[serialLen++] = c;
        } else {
            serialLen = 0;   // line too long: discard
            Serial.println("Line too long, discarded");
        }
    }
}

bool ControlLoop::takeStopRequest() {
    const bool r = stopRequested;
    stopRequested = false;
    return r;
}

bool ControlLoop::takeStartRequest() {
    const bool r = startRequested;
    startRequested = false;
    return r;
}
