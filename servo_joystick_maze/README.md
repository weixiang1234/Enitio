# ESP32 Joystick Servo Maze Code Explanation

This document explains the code in a simple way so you can quickly understand how the maze control system works.

## 1. Project Purpose

The code controls a maze board using:

- ESP32
- PCA9685 servo driver
- 8 joystick inputs
- 8 servo outputs

Each joystick channel controls one servo channel.  
The joystick does **not directly set the servo angle**. Instead, the joystick controls the **speed of movement**.

This makes the board movement smoother and easier to control.

---

## 2. Main Control Idea

The important idea is:

```text
Joystick push amount → movement speed → target servo angle → servo moves
```

So the servo angle changes gradually over time.

This is different from direct control:

```text
Joystick position → servo angle
```

Direct control can be more shaky because small joystick noise directly changes the servo position.

---

## 3. Hardware Pins

### I2C Pins for PCA9685

```cpp
static constexpr int I2C_SDA_PIN = 21;
static constexpr int I2C_SCL_PIN = 22;
```

These pins connect the ESP32 to the PCA9685 servo driver.

### Joystick and Servo Channel Setup

Each row in `channelCfg` represents one joystick-servo pair.

Example:

```cpp
{25, 0, 1, 100, 4000, 100, 90.0f, DEFAULT_SERVO_LIMIT_DEG}
```

Meaning:

| Field | Meaning |
|---|---|
| `25` | ESP32 ADC pin for joystick |
| `0` | PCA9685 servo channel |
| `1` | Servo direction |
| `100` | Minimum expected ADC value |
| `4000` | Maximum expected ADC value |
| `100` | Deadzone around joystick center |
| `90.0f` | Servo center angle |
| `DEFAULT_SERVO_LIMIT_DEG` | Maximum angle away from center |

---

## 4. Important Tuning Parameters

These values affect how smooth and responsive the maze feels.

### Control Interval

```cpp
static constexpr unsigned long CONTROL_INTERVAL_MS = 10;
```

The control loop runs every 10 ms, or about 100 times per second.

A fixed control interval helps the servo movement stay consistent.

---

### Maximum Speed

```cpp
static constexpr float MAX_SPEED_DPS = 30.0f;
```

This controls how fast the servo target angle changes when the joystick is pushed.

`DPS` means degrees per second.

Higher value:
- faster response
- more possible shaking

Lower value:
- smoother movement
- slower response

---

### Return Speed

```cpp
static constexpr float RETURN_SPEED_DPS = 30.0f;
```

This controls how fast the servo returns to center when the joystick is released.

If the board shakes when returning to center, reduce this value.

Example:

```cpp
static constexpr float RETURN_SPEED_DPS = 15.0f;
```

---

### Joystick Smoothing

```cpp
static constexpr float COMMAND_SMOOTH_ALPHA = 0.04f;
```

This smooths the joystick input.

Lower value:
- smoother
- slower response

Higher value:
- faster response
- more sensitive to noise

Recommended range:

```text
0.02 to 0.08
```

---

### Exponential Response

```cpp
static constexpr float RESPONSE_EXPONENT = 1.3f;
```

This makes small joystick movement less sensitive.

Effect:

```text
Small push → slow movement
Large push → faster movement
```

This helps reduce shaking near the joystick center.

---

### Target Threshold

```cpp
static constexpr float TARGET_THRESHOLD_DEG = 0.05f;
```

The servo is updated only if the target angle changes by at least this amount.

This reduces unnecessary servo commands.

If there is shaking caused by too many small updates, increase it slightly:

```cpp
static constexpr float TARGET_THRESHOLD_DEG = 0.1f;
```

---

### Servo Limit

```cpp
static constexpr float DEFAULT_SERVO_LIMIT_DEG = 30.0f;
```

This limits how far each servo can move away from its center angle.

Example:

If the center is `90°` and limit is `30°`, the servo can move from:

```text
60° to 120°
```

For a physical maze board, too large a limit can cause strong tilting and shaking.

For stability, you can try:

```cpp
static constexpr float DEFAULT_SERVO_LIMIT_DEG = 20.0f;
```

or:

```cpp
static constexpr float DEFAULT_SERVO_LIMIT_DEG = 15.0f;
```

---

## 5. Code Structure

The code is organized into these main sections:

```text
1. System configuration
2. Control tuning
3. Per-channel configuration
4. Servo objects
5. Runtime state
6. Helper functions
7. Joystick calibration
8. Joystick reading
9. Servo output
10. Core update logic
11. Debug printing
12. setup()
13. loop()
```

---

## 6. Runtime State

Each channel has a `ChannelState`.

```cpp
struct ChannelState {
    int centerRaw;
    int raw;
    float filteredCmd;
    float targetDeg;
    float lastSentDeg;
    float speedDps;
    bool inDeadzone;
};
```

### What each value means

| Variable | Meaning |
|---|---|
| `centerRaw` | Joystick center value after calibration |
| `raw` | Latest joystick ADC value |
| `filteredCmd` | Smoothed joystick command from -1 to +1 |
| `targetDeg` | Current target servo angle |
| `lastSentDeg` | Last angle sent to servo |
| `speedDps` | Current movement speed |
| `inDeadzone` | Whether joystick is near center |

---

## 7. Joystick Calibration

Function:

```cpp
calibrateAllJoysticks();
```

During startup, the ESP32 reads each joystick many times and calculates its center value.

Important:

```text
Do not touch the joystick during calibration.
```

This value is stored as:

```cpp
channelState[ch].centerRaw
```

The code uses this value to know where the joystick center is.

---

## 8. Joystick Reading

Function:

```cpp
readJoystickCommand(int ch)
```

This function converts raw ADC values into a normalized command:

```text
-1.0 to +1.0
```

Example:

| Joystick Position | Command |
|---|---:|
| Full left / down | -1.0 |
| Center | 0.0 |
| Full right / up | +1.0 |

The function also applies:

1. Deadzone
2. Normalization
3. Exponential response
4. Direction correction

---

## 9. Deadzone

The deadzone prevents small joystick noise from moving the servo.

Example:

```cpp
deadzone = 100
```

If the joystick is only slightly away from center, the command becomes:

```cpp
0.0f
```

If the board moves when the joystick is released, increase the deadzone.

Example:

```cpp
deadzone = 150
```

---

## 10. Exponential Control

The code uses:

```cpp
normalized = applyExpo(normalized, RESPONSE_EXPONENT);
```

This makes the joystick less sensitive near the center.

For example:

```text
Small joystick push → very small servo speed
Large joystick push → much faster servo speed
```

This is useful for maze control because you need fine control near center.

---

## 11. Servo Output

Function:

```cpp
sendServoIfNeeded(int ch)
```

This function only writes to the servo if the target angle has changed enough.

```cpp
if (fabsf(st.targetDeg - st.lastSentDeg) < TARGET_THRESHOLD_DEG) return;
```

This helps reduce unnecessary servo updates.

---

## 12. Main Movement Logic

Function:

```cpp
updateChannel(int ch, float dt)
```

This is the most important function.

It does these steps:

```text
1. Read joystick command
2. Smooth the command
3. Calculate servo speed
4. Update target angle
5. Return to center if joystick is released
6. Send servo command if needed
```

---

## 13. Active Movement

When the joystick is pushed:

```cpp
st.speedDps = st.filteredCmd * MAX_SPEED_DPS;
st.targetDeg += st.speedDps * dt;
```

This means:

```text
Joystick controls speed, not direct angle.
```

The longer you push the joystick, the more the board tilts.

---

## 14. Return to Center

When the joystick is released, the code returns the servo to its center angle:

```cpp
st.targetDeg = moveToward(st.targetDeg, cfg.centerDeg, maxStep);
```

This makes the board slowly return to level.

---

## 15. setup()

The `setup()` function does the startup process:

```text
1. Start Serial Monitor
2. Start I2C
3. Set ADC resolution
4. Calibrate joystick centers
5. Attach all servos
6. Move all servos to center
7. Start the control loop
```

---

## 16. loop()

The `loop()` function runs repeatedly.

It has two main jobs:

```text
1. Update servo control at fixed interval
2. Print debug information at fixed interval
```

The control update happens every:

```cpp
CONTROL_INTERVAL_MS
```

The debug print happens every:

```cpp
PRINT_INTERVAL_MS
```

---

## 17. Debug Output

The Serial Monitor prints this table:

```text
CH RAW  CTR  DELTA  CMD     SPD    TGT
```

Meaning:

| Column | Meaning |
|---|---|
| `CH` | Channel number |
| `RAW` | Current joystick ADC value |
| `CTR` | Calibrated joystick center |
| `DELTA` | Difference from center |
| `CMD` | Smoothed command |
| `SPD` | Servo speed in degrees/sec |
| `TGT` | Target servo angle |

This helps you tune the system.

---

## 18. Tuning Guide

### Problem: Board shakes when joystick is released

Try reducing:

```cpp
RETURN_SPEED_DPS
```

Example:

```cpp
static constexpr float RETURN_SPEED_DPS = 15.0f;
```

---

### Problem: Board shakes while pushing joystick

Try reducing:

```cpp
MAX_SPEED_DPS
```

Example:

```cpp
static constexpr float MAX_SPEED_DPS = 20.0f;
```

---

### Problem: Board moves by itself near center

Increase deadzone:

```cpp
deadzone = 150
```

or:

```cpp
deadzone = 200
```

---

### Problem: Movement is too slow

Increase:

```cpp
MAX_SPEED_DPS
```

Example:

```cpp
static constexpr float MAX_SPEED_DPS = 35.0f;
```

---

### Problem: Board tilts too much

Reduce:

```cpp
DEFAULT_SERVO_LIMIT_DEG
```

Example:

```cpp
static constexpr float DEFAULT_SERVO_LIMIT_DEG = 20.0f;
```

---

### Problem: Small joystick push is too sensitive

Increase:

```cpp
RESPONSE_EXPONENT
```

Example:

```cpp
static constexpr float RESPONSE_EXPONENT = 1.5f;
```

---

## 19. Recommended Stable Starting Values

For a stable maze board, try:

```cpp
static constexpr float MAX_SPEED_DPS = 20.0f;
static constexpr float RETURN_SPEED_DPS = 15.0f;
static constexpr float COMMAND_SMOOTH_ALPHA = 0.04f;
static constexpr float RESPONSE_EXPONENT = 1.3f;
static constexpr float DEFAULT_SERVO_LIMIT_DEG = 20.0f;
```

For each channel, try:

```cpp
deadzone = 150
```

---

## 20. Simple Summary

The code works like this:

```text
Joystick moves
→ raw ADC value is read
→ joystick value is calibrated around center
→ deadzone removes small noise
→ exponential curve makes small push gentle
→ command is smoothed
→ command becomes servo speed
→ servo target angle changes gradually
→ servo moves the maze board
→ when joystick is released, servo returns to center
```

The most important concept is:

```text
The joystick controls how fast the servo moves, not the exact servo angle.
```

This is why the board movement feels smoother and more controllable for a maze game.
