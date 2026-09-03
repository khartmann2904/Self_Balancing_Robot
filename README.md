# Self-Balancing Robot

A two-wheeled, self-balancing robot (inverted pendulum) built around an **ESP32**, driven by an IMU-based cascade PID control loop, with stepper-motor actuation and PS4 controller input for manual driving.

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

| Folder | Contents |
|---|---|
| `Scripts` | Firmware/control code for the robot (ESP32) |
| `Schaltplan` | Circuit schematics / wiring diagrams |
| `Datasheets` | Component datasheets (MPU6050, TMC2209, ESP32, etc.) |
| `3D_Printing` | 3D-printable chassis/mounting parts |

## Getting Started

1. **Print the chassis** using the files in `3D_Printing/`.
2. **Wire the electronics** according to the schematics in `Schaltplan/`, referencing component specs in `Datasheets/` as needed.
3. **Flash the firmware** in `Scripts/` to the ESP32 (Arduino IDE or PlatformIO, depending on the project setup).
4. **Calibrate** the IMU and tune the PID gains for the balance loop until the robot can stand upright and recover from small pushes.
5. **Pair a PS4 controller** via Bluepad32 to drive the robot around once it's balancing reliably.

## TODO

This is an ongoing build-and-learn platform. Recent work has focused on:
- MotorManager still needs a proper FastAccel library integration
 

## Notes

This project doubles as a hands-on platform for exploring control theory concepts (PID tuning, cascade control, sensor fusion) alongside embedded systems work (ESP32 firmware, stepper motor control, Bluetooth HID input).
