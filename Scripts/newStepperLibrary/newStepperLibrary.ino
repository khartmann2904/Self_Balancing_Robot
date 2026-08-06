#include <FastAccelStepper.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Pin Definitions
#define LEFT_STEP_PIN   16
#define LEFT_DIR_PIN    17
#define LEFT_EN_PIN     25
#define LEFT_MS1_PIN    33
#define LEFT_MS2_PIN    32

#define RIGHT_STEP_PIN  27
#define RIGHT_DIR_PIN   26
#define RIGHT_EN_PIN    2
#define RIGHT_MS1_PIN   12
#define RIGHT_MS2_PIN   14

#define BATTERY_VOLTAGE_PIN 15  

// Akku-Konstanten
const float R1 = 32000.0;
const float R2 = 10000.0;
const float RATIO = (R1 + R2) / R2;
const float ADC_REF = 3.3;  
const float ADC_RES = 4095.0; 

// FastAccelStepper Instanzen
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *leftMotor = NULL;
FastAccelStepper *rightMotor = NULL;

MPU6050 mpu6050(Wire);

// Inner Loop
float kp = 10.0;
float ki = 0.0;
float kd = 0.0;

float targetAngle = 0.0;
float error, lastError, integratedError, derivative;
float motorSpeed;
unsigned long lastTime;

// Outer Loop (Position)
float posKp = 0.0;
float posKi = 0.0;
float posKd = 0.0;
float posIntegral = 0.0;
float posLastError = 0.0;
float angleBias = 0.0;

// Geometrie & Schrittauflösung
const float microSteps = 8.0; 
const float stepsPerRev = 200.0 * microSteps;
const float wheelDiameterMM = 116.0;
const float mmPerStep = (PI * wheelDiameterMM) / stepsPerRev;
unsigned long lastOuterTime = 0;
const unsigned long outerInterval = 20; 

const float MAX_ANGLE_BIAS = 3.0;

// Bluetooth Controller Callbacks
void onConnectedController(ControllerPtr ctl) {
    if (myControllers[0] == nullptr) {
        Serial.println("Controller connected!");
        myControllers[0] = ctl;
        ctl->setColorLED(0, 255, 0);
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
    int stickY = ctl->axisY();  
    int angleCorrection = 0;

    if (stickY < -50) {
        angleCorrection = map(stickY, -50, -512, 0, 8);
        return angleCorrection;
    }
    else if (stickY > 50) {
        angleCorrection = map(stickY, 50, 512, 0, -8);
        return angleCorrection;
    }

    if (ctl->a()) {
        if (leftMotor) leftMotor->disableOutputs();
        if (rightMotor) rightMotor->disableOutputs();
    }
    return 0;
}

// Outer Position Loop
void updateOuterLoop(bool joystickActive) {
    if (millis() - lastOuterTime < outerInterval) return; 
    float dt = (millis() - lastOuterTime) / 1000.0; 
    lastOuterTime = millis();

    if (joystickActive || !leftMotor || !rightMotor) {
        posIntegral = 0;
        posLastError = 0;
        angleBias = 0;
        if (leftMotor) leftMotor->setCurrentPosition(0);
        if (rightMotor) rightMotor->setCurrentPosition(0);
        return;
    }

    float avgPos = (-leftMotor->getCurrentPosition() + rightMotor->getCurrentPosition()) / 2.0;
    float posMM = avgPos * mmPerStep;

    float posError = 0.0 - posMM; 
    posIntegral += posError * dt;
    posIntegral = constraint(posIntegral, -50, 50);
    float posDerivative = (dt > 0) ? (posError - posLastError) / dt : 0;

    angleBias = posKp * posError + posKi * posIntegral + posKd * posDerivative;
    angleBias = constraint(angleBias, -MAX_ANGLE_BIAS, MAX_ANGLE_BIAS);
    posLastError = posError;
}

void setup() {
    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(400000);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.enableVirtualDevice(false);
    BP32.forgetBluetoothKeys();  

    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);

    // Microstepping & Pinmodes
    pinMode(LEFT_MS1_PIN, OUTPUT);
    pinMode(RIGHT_MS1_PIN, OUTPUT);
    pinMode(LEFT_MS2_PIN, OUTPUT);
    pinMode(RIGHT_MS2_PIN, OUTPUT);
    pinMode(BATTERY_VOLTAGE_PIN, INPUT);

    digitalWrite(LEFT_MS1_PIN, LOW);
    digitalWrite(RIGHT_MS1_PIN, LOW);
    digitalWrite(LEFT_MS2_PIN, LOW);
    digitalWrite(RIGHT_MS2_PIN, LOW);

    // FastAccelStepper Engine starten
    engine.init();

    // Linker Motor
    leftMotor = engine.stepperConnectToPin(LEFT_STEP_PIN);
    if (leftMotor) {
        leftMotor->setDirectionPin(LEFT_DIR_PIN);
        leftMotor->setEnablePin(LEFT_EN_PIN);
        leftMotor->setAutoEnable(true);
        leftMotor->setSpeedInHz(50000);  
        leftMotor->setAcceleration(100000);
    } else {
        Serial.println("Fehler beim Initialisieren von leftMotor!");
    }

    // Rechter Motor
    rightMotor = engine.stepperConnectToPin(RIGHT_STEP_PIN);
    if (rightMotor) {
        rightMotor->setDirectionPin(RIGHT_DIR_PIN);
        rightMotor->setEnablePin(RIGHT_EN_PIN);
        rightMotor->setAutoEnable(true);
        rightMotor->setSpeedInHz(50000);
        rightMotor->setAcceleration(100000);
    } else {
        Serial.println("Fehler beim Initialisieren von rightMotor!");
    }

    lastOuterTime = millis();
    lastTime = millis();
}

void loop() {
    mpu6050.update(); 
    unsigned long currentTime = millis(); 
    BP32.update();  
    handleSerialTuning();

    static float driveCommand = 0;
    static bool joystickActive = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        ControllerPtr ctl = myControllers[i];
        if (ctl && ctl->isConnected() && ctl->hasData()) {
            driveCommand = processGamepad(ctl);
            joystickActive = (driveCommand != 0);
        }
    }

    updateOuterLoop(joystickActive);
    targetAngle = driveCommand + angleBias;

    // PID Berechnung bei 500Hz (alle 2ms)
    // PID Berechnung bei 500Hz (alle 2ms)
    // PID Berechnung bei 500Hz (alle 2ms)
    if (currentTime - lastTime >= 2) {
        float currentAngle = mpu6050.getAngleX(); 
        float gyroRateX = mpu6050.getGyroX();     

        // =========================================================================
        // CUTOFF-FUNKTION: Winkelprüfer (> 40 Grad in beide Richtungen)
        // =========================================================================
        if (abs(currentAngle) > 40.0) {
            // 1. Motoren sofort per Hardware-Stopp anhalten und Spulen stromlos schalten
            if (leftMotor) {
                leftMotor->forceStopAndNewPosition(0); // Hardware-Pulse sofort abbrechen
                leftMotor->disableOutputs();           // Treiber stromlos schalten (LOW-aktiv Enable Pin)
            }
            if (rightMotor) {
                rightMotor->forceStopAndNewPosition(0);
                rightMotor->disableOutputs();
            }

            // 2. Alle internen Reglerwerte & Sollgrößen auf 0 zurücksetzen
            motorSpeed = 0; 
            integratedError = 0;
            posIntegral = 0;
            posLastError = 0;
            angleBias = 0;
            
        } else {
            // Normaler Betrieb: Motoren aktivieren
            if (leftMotor) leftMotor->enableOutputs();
            if (rightMotor) rightMotor->enableOutputs();

            // Error & PID Berechnung
            error = currentAngle - targetAngle;

            float P = kp * error;

            integratedError += error;
            integratedError = constraint(integratedError, -200, 200); 
            float I = ki * integratedError;

            derivative = gyroRateX;
            float D = kd * derivative;

            motorSpeed = P + I + D;

            // Maximale Drehzahl begrenzen (z. B. auf 3000 Hz / Steps pro Sekunde)
            motorSpeed = constraint(motorSpeed, -3000.0, 3000.0);

            // Ansteuerung der Motoren über FastAccelStepper
            if (leftMotor && rightMotor) {
                // Linker Motor
                float speedLeft = -motorSpeed;
                if (abs(speedLeft) < 10) { 
                    // Toter Bereich nahe 0 Hz, um Mikroruckeln im Stand zu vermeiden
                    leftMotor->stopMove();
                } else {
                    leftMotor->setSpeedInHz((uint32_t)abs(speedLeft));
                    if (speedLeft >= 0) {
                        leftMotor->runForward();
                    } else {
                        leftMotor->runBackward();
                    }
                }

                // Rechter Motor
                float speedRight = motorSpeed;
                if (abs(speedRight) < 10) {
                    rightMotor->stopMove();
                } else {
                    rightMotor->setSpeedInHz((uint32_t)abs(speedRight));
                    if (speedRight >= 0) {
                        rightMotor->runForward();
                    } else {
                        rightMotor->runBackward();
                    }
                }
            }
        }

        lastError = error;
        lastTime = currentTime;
    }
    checkBatteryNonBlocking(currentTime);
}

void checkBatteryNonBlocking(unsigned long currentTime) {
    static unsigned long lastBatteryCheck = 0;
    static int sampleCount = 0;
    static long voltageSum = 0;

    if (currentTime - lastBatteryCheck >= 15) { 
        lastBatteryCheck = currentTime;
        voltageSum += analogRead(BATTERY_VOLTAGE_PIN);
        sampleCount++;

        if (sampleCount >= 64) {
            float vout = ((voltageSum / 64.0) / ADC_RES) * ADC_REF;
            float voltage = vout * RATIO * (10.5 / 12.8);

            if (voltage <= 9.0) {
                if (leftMotor) leftMotor->disableOutputs();
                if (rightMotor) rightMotor->disableOutputs();
            }

            sampleCount = 0;
            voltageSum = 0;
        }
    }
}

float constraint(float val, float minVal, float maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

void handleSerialTuning() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) return; 

        if (line.equalsIgnoreCase("show")) {
            Serial.println("---- Current gains ----");
            Serial.print("kp="); Serial.print(kp);
            Serial.print("  ki="); Serial.print(ki);
            Serial.print("  kd="); Serial.println(kd);
            Serial.print("posKp="); Serial.print(posKp);
            Serial.print("  posKi="); Serial.print(posKi);
            Serial.print("  posKd="); Serial.println(posKd);
            return;
        }

        int sepIdx = line.indexOf(' ');
        if (sepIdx == -1) sepIdx = line.indexOf('=');
        if (sepIdx == -1) return;

        String name = line.substring(0, sepIdx);
        float val = line.substring(sepIdx + 1).toFloat();
        name.toLowerCase();

        if (name == "kp") kp = val;
        else if (name == "ki") ki = val;
        else if (name == "kd") kd = val;
        else if (name == "poskp") posKp = val;
        else if (name == "poski") posKi = val;
        else if (name == "poskd") posKd = val;
    }
}