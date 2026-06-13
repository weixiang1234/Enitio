# ESP32 Joystick Servo Maze Controller

This project uses an ESP32, joystick inputs, and a PCA9685 servo driver to control servo motors for a physical maze board. The joystick does not directly set the servo angle. Instead, the joystick push strength controls how fast the servo moves toward its limit, giving smoother and more stable control.

## Project Overview

The goal of this project is to build a joystick-controlled servo maze system where the player can tilt the maze board by moving joysticks. The firmware reads analog joystick values, applies deadzone and smoothing logic, then moves the servos gradually within calibrated angle limits.

## Features

* ESP32-based control system
* PlatformIO project structure
* PCA9685 servo driver support
* Joystick-controlled servo movement
* Servo angle limits for safer movement
* Auto-return to center when joystick is released
* Deadzone filtering to reduce joystick noise
* Adjustable speed and response tuning
* Designed for maze board control

## Hardware Used

* ESP32 development board
* PCA9685 16-channel PWM servo driver
* Analog joystick module
* Servo motors, such as MG90S
* External 5V servo power supply
* Jumper wires
* Common ground connection between ESP32 and servo power supply

## Wiring Overview

| Component           | ESP32 / PCA9685 Connection            |
| ------------------- | ------------------------------------- |
| PCA9685 SDA         | ESP32 GPIO 21                         |
| PCA9685 SCL         | ESP32 GPIO 22                         |
| PCA9685 VCC         | ESP32 3.3V                            |
| PCA9685 GND         | ESP32 GND                             |
| PCA9685 V+          | External 5V servo power               |
| Servo signal        | PCA9685 servo channel                 |
| Joystick X/Y output | ESP32 analog input pins               |
| Joystick GND        | ESP32 GND                             |
| Joystick VCC        | ESP32 3.3V or 5V, depending on module |

> Important: The ESP32 ground, PCA9685 ground, joystick ground, and external servo power ground must be connected together.

## Project Structure

```text
esp32_joystick_servo/
├── platformio.ini
├── src/
│   └── main.cpp
├── include/
├── lib/
├── test/
├── README.md
└── .gitignore
```

## Software Requirements

* Visual Studio Code
* PlatformIO extension
* ESP32 board package
* Required libraries listed in `platformio.ini`

## How to Build

Open the project folder in Visual Studio Code.

Make sure the folder contains:

```text
platformio.ini
```

Then build the project using PlatformIO:

```powershell
pio run
```

Or use the PlatformIO sidebar:

```text
PlatformIO → Project Tasks → Build
```

## How to Upload

Connect the ESP32 board to your computer using USB.

Then upload the firmware:

```powershell
pio run --target upload
```

Or use the PlatformIO sidebar:

```text
PlatformIO → Project Tasks → Upload
```

## How to Open Serial Monitor

Use:

```powershell
pio device monitor
```

This is useful for checking joystick readings, servo angles, and debugging messages.

## Control Concept

The joystick controls the speed and direction of servo movement.

When the joystick is near the center:

```text
Servo stays near center
```

When the joystick is pushed slightly:

```text
Servo moves slowly
```

When the joystick is pushed further:

```text
Servo moves faster toward the angle limit
```

When the joystick is released:

```text
Servo gradually returns to center
```

This gives better control compared to directly mapping joystick position to servo angle.

## Important Parameters to Tune

These values may need to be adjusted based on your actual joystick and maze board.

| Parameter                  | Purpose                                    |
| -------------------------- | ------------------------------------------ |
| Servo center angle         | Sets the neutral board position            |
| Servo minimum angle        | Limits maximum tilt in one direction       |
| Servo maximum angle        | Limits maximum tilt in the other direction |
| Joystick minimum raw value | Calibrates joystick low end                |
| Joystick maximum raw value | Calibrates joystick high end               |
| Deadzone                   | Ignores small joystick noise near center   |
| Maximum speed              | Controls how fast servo can move           |
| Response exponent          | Controls how sensitive the joystick feels  |

## Calibration Notes

Before final use, manually check:

1. The joystick center value when untouched.
2. The minimum and maximum joystick readings.
3. The servo center angle where the maze board is level.
4. The safe minimum and maximum servo angles.
5. Whether the servo movement is too fast or too slow.
6. Whether the maze board shakes due to unstable servo movement.

## Safety Notes

* Do not power multiple servos directly from the ESP32 5V pin.
* Use an external 5V power supply for the servos.
* Always connect all grounds together.
* Start with small servo angle limits first.
* Increase servo limits only after confirming the maze mechanism moves safely.
* Avoid forcing the servo beyond the mechanical limit of the maze board.

## Common Git Commands

After editing the code:

```powershell
git status
git add .
git commit -m "Update servo joystick control"
git push
```

To download the latest GitHub version:

```powershell
git pull
```

## Future Improvements

* Add support for more joysticks and servos
* Add automatic joystick calibration
* Add OLED display for servo status
* Add button-controlled reset to center
* Add smoother motion filtering
* Add different difficulty modes for the maze game
