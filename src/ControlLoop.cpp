#include "ControlLoop.h"
#include "Config.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

ControlLoop::ControlLoop(PIDGains anglePID, PIDGains positionPID)
    : angleGains(anglePID), positionGains(positionPID),
      positionIntegral(0.0f), angleIntegral(0.0f), lastPositionError(0.0f),
      positionInitialized(false), stopRequested(false), startRequested(false), serialLen(0) {}

float ControlLoop::computeCascade(float targetAngleCmd, long leftPosition, long rightPosition,
                                  float currentAngle, float gyroRate, bool joystickActive, float dt) {
    if (dt <= 0.0f) {
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
        const float averagePosition = (static_cast<float>(leftPosition) + static_cast<float>(rightPosition)) / 2.0f;
        const float positionError = -POSITION_SIGN * (averagePosition * MM_PER_STEP);

        if (!positionInitialized) {          // avoid a derivative spike on the first sample
            lastPositionError = positionError;
            positionInitialized = true;
        }

        positionIntegral += positionError * dt;
        positionIntegral = constrain(positionIntegral, -POSITION_INTEGRAL_LIMIT, POSITION_INTEGRAL_LIMIT);
        const float positionDerivative = (positionError - lastPositionError) / dt;
        lastPositionError = positionError;

        angleBias = (positionGains.Kp * positionError)
                  + (positionGains.Ki * positionIntegral)
                  + (positionGains.Kd * positionDerivative);
        angleBias = constrain(angleBias, -POSITION_BIAS_LIMIT_DEG, POSITION_BIAS_LIMIT_DEG);
    }

    const float targetAngle = targetAngleCmd + angleBias;

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

void ControlLoop::reset() {
    positionIntegral = 0.0f;
    angleIntegral = 0.0f;
    lastPositionError = 0.0f;
    positionInitialized = false;
}

// ---------------------------------------------------------------- Serial tuning
void ControlLoop::handleSerialTuning() {
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

void ControlLoop::processTuningLine(char* line) {
    while (*line == ' ' || *line == '\t') line++;
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\t')) line[--len] = '\0';
    if (*line == '\0') return;
    for (char* p = line; *p; ++p) *p = static_cast<char>(tolower(static_cast<unsigned char>(*p)));

    if (strcmp(line, "show") == 0) {
        Serial.println("---- Current gains ----");
        Serial.print("kp=");    Serial.print(angleGains.Kp);
        Serial.print("  ki=");  Serial.print(angleGains.Ki);
        Serial.print("  kd=");  Serial.println(angleGains.Kd);
        Serial.print("posKp="); Serial.print(positionGains.Kp);
        Serial.print("  posKi="); Serial.print(positionGains.Ki);
        Serial.print("  posKd="); Serial.println(positionGains.Kd);
        return;
    }

    if (strcmp(line, "stop") == 0) {
        stopRequested = true;
        Serial.println("STOP: motors disabled. Type 'start' to allow re-arming.");
        return;
    }
    if (strcmp(line, "start") == 0) {
        startRequested = true;
        Serial.println("START: re-arming allowed (robot must be held upright).");
        return;
    }

    char* sep = strpbrk(line, " =");
    if (sep == nullptr) {
        Serial.println("Format: <name> <value>   e.g. kp 450   or  type 'show'");
        return;
    }
    *sep = '\0';
    char* valueStr = sep + 1;
    while (*valueStr == ' ' || *valueStr == '=' || *valueStr == '\t') valueStr++;

    const char* name = line;
    const float val = static_cast<float>(atof(valueStr));

    if      (strcmp(name, "kp") == 0)    angleGains.Kp = val;
    else if (strcmp(name, "ki") == 0)  { angleGains.Ki = val;    angleIntegral = 0.0f; }
    else if (strcmp(name, "kd") == 0)    angleGains.Kd = val;
    else if (strcmp(name, "poskp") == 0) positionGains.Kp = val;
    else if (strcmp(name, "poski") == 0) { positionGains.Ki = val; positionIntegral = 0.0f; }
    else if (strcmp(name, "poskd") == 0) positionGains.Kd = val;
    else { Serial.println("Unknown parameter"); return; }

    Serial.print(name); Serial.print(" set to "); Serial.println(val);
}