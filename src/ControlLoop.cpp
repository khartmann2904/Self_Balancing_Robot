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
        const float stepsPerRevolution = 200.0f * 8.0f; // 200 steps per revolution with 8x microstepping
        const float wheelDiameterMM = 116.0f;
        const float mmPerStep = (PI * wheelDiameterMM) / stepsPerRevolution; // Calculate the distance in millimeters per step
        const float averagePosition = (-leftPosition + rightPosition) / 2.0f; //
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


void ControlLoop::handleSerialTuning() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return; 

    if (line.equalsIgnoreCase("show")) {  //Command "show" prints the current values of the controller
      Serial.println("---- Current gains ----");
      Serial.print("kp="); Serial.print(angleGains.Kp);
      Serial.print("  ki="); Serial.print(angleGains.Ki);
      Serial.print("  kd="); Serial.println(angleGains.Kd);
      Serial.print("posKp="); Serial.print(speedGains.Kp);
      Serial.print("  posKi="); Serial.print(speedGains.Ki);
      Serial.print("  posKd="); Serial.println(speedGains.Kd);
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

    if (name == "kp") angleGains.Kp = val; // Sets values for the changed values
    else if (name == "ki") angleGains.Ki = val;
    else if (name == "kd") angleGains.Kd = val;
    else if (name == "poskp") speedGains.Kp = val;
    else if (name == "poski") speedGains.Ki = val;
    else if (name == "poskd") speedGains.Kd = val;
    else { Serial.println("Unknown parameter"); return; }

    Serial.print(name); Serial.print(" set to "); Serial.println(val);
  }
}