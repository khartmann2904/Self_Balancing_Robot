# Self-Balancing Robot

A two-wheeled, self-balancing robot (inverted pendulum) built around an **ESP32**, driven by an IMU-based cascade PID control loop, with stepper-motor actuation and PS4 controller input for manual driving.

<img width="1610" height="1170" alt="robot" src="https://github.com/user-attachments/assets/ebe64cd1-d09e-4216-9bcd-508d340d6657" />

## Hardware

- **Microcontroller:** ESP32
- **IMU:** GY-86 / MPU6050 (accelerometer + gyroscope)
- **Actuation:** NEMA17 stepper motors with TMC2209 stepper drivers
- **Input:** PS4 controller via Bluepad32 (Bluetooth)
- **Chassis:** 3D-printed frame (see `3D_Printing/`)

## Electrical Circuit

<img width="1824" height="1053" alt="image" src="https://github.com/user-attachments/assets/2754bd74-80f2-4fb8-b6df-a43a7ef6d9d1" />


## Control Architecture

The robot uses a **cascade PID control** scheme:

- **Inner loop (balance):** stabilizes the tilt angle using gyro rate as the derivative term (rather than differentiating the angle itself), reacting quickly to keep the robot upright.
- **Outer loop (position hold):** holds the robot's position by feeding a target lean angle into the inner balance loop, correcting for drift.
- **Drive layer:** converts driving commands into a target velocity (mm/s), which is layered on top of the balance/position control so the robot can be driven around with the PS4 controller while still balancing itself.

A `balanceTrim` value is used to compensate for small mechanical/sensor offsets so the robot can rest at true vertical.

## Repository Structure

```
Self_Balancing_Robot/
├── docs/
│   └── Datasheets/            # Component datasheets (IMU, drivers, motors)
├── hardware/
│   ├── 3D_Printing/           # STL/CAD files for the frame and mounts
│   └── Schematics/            # KiCad circuit schematics and PCB files
├── include/                   # Header files
│   ├── BatteryManager.h
│   ├── BluetoothManager.h
│   ├── ControlLoop.h
│   ├── IMUManager.h
│   └── MotorManager.h
├── lib/                       # PlatformIO project-specific libraries
├── src/                       # Firmware source code
│   ├── BatteryManager.cpp
│   ├── BluetoothManager.cpp
│   ├── ControlLoop.cpp
│   ├── IMUManager.cpp
│   ├── MotorManager.cpp
│   └── main.cpp                # Entry point: setup(), loop(), ties everything together
├── platformio.ini             # PlatformIO project configuration
└── .gitignore
```

## Software Architecture

The firmware is structured around small, single-responsibility classes rather than one monolithic sketch:

- **`ControlLoop`** — runs the cascaded PID control: the outer loop converts a target speed into a target lean angle, and the inner loop converts that angle error into motor output. Also exposes serial-based live gain tuning.
- **`IMUManager`** — wraps IMU initialization and orientation/angle readout, keeping sensor-specific code out of the control logic.
- **`MotorManager`** — wraps stepper motor control (step generation, direction, enable/disable), keeping hardware I/O out of the control logic.
- **`main.cpp`** — wires the above together: reads sensors, runs the control loop, drives the motors, and handles Bluetooth input.

This separation means the control math can be tested/tuned independently of the IMU or motor driver implementation, and either can be swapped out without touching `ControlLoop`.

## Getting Started

1. Flash `src/main.cpp` to the ESP32 (Arduino IDE or PlatformIO).
2. Wire up the IMU, motor drivers, and motors per [`hardware/Schematics/`](./hardware/Schematics).
3. Power on and hold the robot upright to let it settle into balance.
4. Connect via the serial monitor to tune `Kp`/`Ki`/`Kd` live if needed.
5. Pair a PS4 controller via Bluepad32 for manual drive control.

## TODO

This is an ongoing build-and-learn platform. Recent work has focused on:
- MotorManager still needs a proper FastAccel library integration
 

## Notes

This project doubles as a hands-on platform for exploring control theory concepts (PID tuning, cascade control, sensor fusion) alongside embedded systems work (ESP32 firmware, stepper motor control, Bluetooth HID input).
