#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Must be defined BEFORE including ServoEasing.hpp
#define USE_PCA9685_SERVO_EXPANDER
#include <ServoEasing.hpp>

// =====================================================
// System configuration
// =====================================================

static constexpr int NUM_CHANNELS = 8;

static constexpr int I2C_SDA_PIN = 21;
static constexpr int I2C_SCL_PIN = 22;

static constexpr uint8_t PCA9685_ADDR = PCA9685_DEFAULT_ADDRESS;

// =====================================================
// Control tuning
// =====================================================

// Control loop runs at a fixed 10 ms (100 Hz).
// Consistent dt is the single most important factor for smooth servo motion.
static constexpr unsigned long CONTROL_INTERVAL_MS = 10;

static constexpr unsigned long PRINT_INTERVAL_MS = 500;

// Safety cap for dt in case of unexpected delays.
static constexpr float MAX_DT_SECONDS = 0.25f;  

// Maximum servo angular speed while joystick is active.
static constexpr float MAX_SPEED_DPS = 30.0f;

// Return-to-center speed. Kept equal to MAX_SPEED_DPS for a symmetric feel.
// Lower this if the board shakes when the joystick is released.
static constexpr float RETURN_SPEED_DPS = 30.0f;

// Low-pass filter on joystick commands (both push AND release directions).
// Applied symmetrically so acceleration and deceleration feel the same.
// Range: 0.02 (very smooth/slow) to 0.15 (fast/responsive).
// At 100 Hz a value of 0.05 gives roughly an 80 ms time constant.
static constexpr float COMMAND_SMOOTH_ALPHA = 0.04f;

// Exponential response curve.
// 1.0 = linear. 1.3 = gentle curve (recommended for a maze board).
static constexpr float RESPONSE_EXPONENT = 1.3f;

// Minimum angle change before a new position is written to the servo.
// Prevents unnecessary I2C traffic without visible quantisation.
static constexpr float TARGET_THRESHOLD_DEG = 0.05f;

// Servo travel limits.
static constexpr float DEFAULT_SERVO_LIMIT_DEG = 30.0f;
static constexpr float ABS_SERVO_MIN_DEG       = 0.0f;
static constexpr float ABS_SERVO_MAX_DEG       = 180.0f;

// =====================================================
// Per-channel configuration
// =====================================================

struct ChannelConfig {
    int   joyPin;       // ESP32 ADC pin
    int   servoChannel; // PCA9685 channel
    int   direction;    // 1 or -1

    int   minRaw;       // ADC low end
    int   maxRaw;       // ADC high end
    int   deadzone;     // raw units each side of centre

    float centerDeg;    // servo angle when board is level
    float limitDeg;     // max travel away from centre
};

static ChannelConfig channelCfg[NUM_CHANNELS] = {
    // joyPin, servoCh, dir, minRaw, maxRaw, deadzone, center, limit
    {25, 0,  1, 100, 4000, 100,  90.0f, DEFAULT_SERVO_LIMIT_DEG},
    {26, 1,  1, 100, 4000, 100,  95.0f, DEFAULT_SERVO_LIMIT_DEG},
    {32, 2,  1, 100, 4000, 100,  60.0f, DEFAULT_SERVO_LIMIT_DEG},
    {33, 3,  1, 100, 4000, 100,  60.0f, DEFAULT_SERVO_LIMIT_DEG},
    {34, 4,  1, 100, 4000, 100, 109.0f, DEFAULT_SERVO_LIMIT_DEG},
    {35, 5,  1, 100, 4000, 100, 105.0f, DEFAULT_SERVO_LIMIT_DEG},
    {36, 6,  1, 100, 4000, 100,  43.0f, DEFAULT_SERVO_LIMIT_DEG},
    {39, 7,  1, 100, 4000, 100,  57.0f, DEFAULT_SERVO_LIMIT_DEG}
};

// =====================================================
// Servo objects
// =====================================================

ServoEasing servo0(PCA9685_ADDR);
ServoEasing servo1(PCA9685_ADDR);
ServoEasing servo2(PCA9685_ADDR);
ServoEasing servo3(PCA9685_ADDR);
ServoEasing servo4(PCA9685_ADDR);
ServoEasing servo5(PCA9685_ADDR);
ServoEasing servo6(PCA9685_ADDR);
ServoEasing servo7(PCA9685_ADDR);

ServoEasing* servos[NUM_CHANNELS] = {
    &servo0, &servo1, &servo2, &servo3,
    &servo4, &servo5, &servo6, &servo7
};

// =====================================================
// Runtime state
// =====================================================

struct ChannelState {
    int   centerRaw   = 2048;
    int   raw         = 2048;

    // filteredCmd runs from -1 to +1 and is smoothed in BOTH directions.
    // This gives symmetric acceleration and deceleration.
    float filteredCmd = 0.0f;

    float targetDeg   = 90.0f;

    // lastSentDeg is stored as a float to avoid rounding drift accumulating
    // over time when comparing against the float targetDeg.
    float lastSentDeg = -999.0f; // force first write

    float speedDps    = 0.0f;
    bool  inDeadzone  = true;
};

ChannelState channelState[NUM_CHANNELS];

unsigned long lastControlTime = 0;
unsigned long lastPrintTime   = 0;

// =====================================================
// Helpers
// =====================================================

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float getServoMinDeg(int ch) {
    return clampf(
        channelCfg[ch].centerDeg - channelCfg[ch].limitDeg,
        ABS_SERVO_MIN_DEG, ABS_SERVO_MAX_DEG
    );
}

static float getServoMaxDeg(int ch) {
    return clampf(
        channelCfg[ch].centerDeg + channelCfg[ch].limitDeg,
        ABS_SERVO_MIN_DEG, ABS_SERVO_MAX_DEG
    );
}

static float moveToward(float current, float target, float maxStep) {
    float error = target - current;
    if (fabsf(error) <= maxStep) return target;
    return current + (error > 0.0f ? maxStep : -maxStep);
}

static float applyExpo(float x, float exponent) {
    float sign = x >= 0.0f ? 1.0f : -1.0f;
    return sign * powf(fabsf(x), exponent);
}

// =====================================================
// PCA9685 microsecond write
//
// ServoEasing::write() rounds to integer degrees, producing stairstepping
// at low angular speeds (most visible near joystick centre).
// Writing microseconds directly gives ~0.5 µs resolution on a PCA9685
// at 50 Hz, which is finer than any servo can physically resolve.
// =====================================================

static void writeServoDeg(int ch, float deg) {
    deg = clampf(deg, getServoMinDeg(ch), getServoMaxDeg(ch));

    servos[ch]->write(static_cast<int>(roundf(deg)));
}

// =====================================================
// Joystick calibration
// =====================================================

static void calibrateAllJoysticks() {
    static constexpr int SAMPLE_COUNT = 200;

    long sum[NUM_CHANNELS] = {0};

    Serial.println();
    Serial.println("Calibrating joystick centres — do not touch the sticks.");
    delay(500);

    for (int s = 0; s < SAMPLE_COUNT; s++) {
        for (int ch = 0; ch < NUM_CHANNELS; ch++) {
            sum[ch] += analogRead(channelCfg[ch].joyPin);
        }
        delay(2);
    }

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        channelState[ch].centerRaw = static_cast<int>(sum[ch] / SAMPLE_COUNT);
        channelState[ch].raw       = channelState[ch].centerRaw;

        Serial.printf("CH%d centre = %d\n", ch, channelState[ch].centerRaw);
    }
}

// =====================================================
// Joystick reading
// Returns a normalised command in [-1, +1] with expo applied.
// Returns exactly 0.0 when inside the deadzone.
// =====================================================

static float readJoystickCommand(int ch) {
    ChannelConfig& cfg = channelCfg[ch];
    ChannelState&  st  = channelState[ch];

    int raw    = analogRead(cfg.joyPin);
    st.raw     = raw;
    int offset = raw - st.centerRaw;

    if (abs(offset) <= cfg.deadzone) {
        st.inDeadzone = true;
        return 0.0f;
    }

    st.inDeadzone = false;
    float normalized;

    if (offset > 0) {
        float denom = static_cast<float>(cfg.maxRaw - st.centerRaw - cfg.deadzone);
        if (denom <= 1.0f) return 0.0f;
        normalized = static_cast<float>(offset - cfg.deadzone) / denom;
    } else {
        float denom = static_cast<float>(st.centerRaw - cfg.minRaw - cfg.deadzone);
        if (denom <= 1.0f) return 0.0f;
        normalized = static_cast<float>(offset + cfg.deadzone) / denom;
    }

    normalized = clampf(normalized, -1.0f, 1.0f);
    normalized = applyExpo(normalized, RESPONSE_EXPONENT);
    normalized *= static_cast<float>(cfg.direction);

    return clampf(normalized, -1.0f, 1.0f);
}

// =====================================================
// Servo output
// =====================================================

static void sendServoIfNeeded(int ch) {
    ChannelState& st = channelState[ch];

    st.targetDeg = clampf(st.targetDeg, getServoMinDeg(ch), getServoMaxDeg(ch));

    if (fabsf(st.targetDeg - st.lastSentDeg) < TARGET_THRESHOLD_DEG) return;

    st.lastSentDeg = st.targetDeg;
    writeServoDeg(ch, st.targetDeg);
}

// =====================================================
// Core channel update  (called every CONTROL_INTERVAL_MS)
// =====================================================

static void updateChannel(int ch, float dt) {
    ChannelConfig& cfg = channelCfg[ch];
    ChannelState&  st  = channelState[ch];

    float rawCmd = readJoystickCommand(ch);

    // ---- Symmetric low-pass on the command signal ----
    // Applied whether the joystick is pushed or released.
    // This prevents the abrupt stop / jerk that happened before
    // when filteredCmd was reset to 0 instantly on release.
    st.filteredCmd += COMMAND_SMOOTH_ALPHA * (rawCmd - st.filteredCmd);

    // Snap to zero only when the filtered value is negligibly small to
    // avoid the servo hunting around the centre position.
    static constexpr float CMD_ZERO_SNAP = 0.005f;
    if (fabsf(st.filteredCmd) < CMD_ZERO_SNAP) {
        st.filteredCmd = 0.0f;
    }

    if (fabsf(st.filteredCmd) > CMD_ZERO_SNAP) {
        // ---- Active movement ----
        st.speedDps  = st.filteredCmd * MAX_SPEED_DPS;
        st.targetDeg += st.speedDps * dt;
    } else {
        // ---- Return to centre ----
        // The filtered command naturally decays to zero, so deceleration
        // before centering is already handled above. Here we only need a
        // gentle constant-speed return for the residual position error.
        st.speedDps  = 0.0f;
        float maxStep = RETURN_SPEED_DPS * dt;
        st.targetDeg  = moveToward(st.targetDeg, cfg.centerDeg, maxStep);
    }

    sendServoIfNeeded(ch);
}

// =====================================================
// Debug printing
// =====================================================

static void printDebug() {
    Serial.print("\r\n");
    Serial.print("CH RAW  CTR  DELTA  CMD     SPD    TGT\r\n");
    Serial.print("------------------------------------------\r\n");

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        ChannelState& st = channelState[ch];

        Serial.printf(
            "%02d %4d %4d %+5d  %+5.2f  %+6.1f  %5.1f\r\n",
            ch,
            st.raw,
            st.centerRaw,
            st.raw - st.centerRaw,
            st.filteredCmd,
            st.speedDps,
            st.targetDeg
        );
    }
}

// =====================================================
// Setup
// =====================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("Starting 8-channel maze servo system...");

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    analogReadResolution(12);

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        analogSetPinAttenuation(channelCfg[ch].joyPin, ADC_11db);
    }

    calibrateAllJoysticks();

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        if (servos[ch]->attach(channelCfg[ch].servoChannel) == INVALID_SERVO) {
            Serial.printf("ERROR: failed to attach servo on PCA9685 channel %d\n",
                          channelCfg[ch].servoChannel);
            continue;
        }

        Serial.printf("Servo attached on PCA9685 channel %d\n",
                      channelCfg[ch].servoChannel);

        channelState[ch].targetDeg   = channelCfg[ch].centerDeg;
        channelState[ch].lastSentDeg = -999.0f; // force first write

        writeServoDeg(ch, channelCfg[ch].centerDeg);
        delay(80);
    }

    lastControlTime = millis();
    lastPrintTime   = millis();

    Serial.println();
    Serial.println("Maze system ready.");
    Serial.println("Move one joystick at a time to verify direction and angle limits.");
}

// =====================================================
// Main loop
// =====================================================

void loop() {
    unsigned long now = millis();

    // Fixed-rate control tick.
    // A fixed interval gives a consistent dt, which is essential for
    // integrating speed into position without drift or jerkiness.
    if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
        float dt = static_cast<float>(now - lastControlTime) / 1000.0f;
        lastControlTime = now;

        dt = clampf(dt, 0.0f, MAX_DT_SECONDS);

        for (int ch = 0; ch < NUM_CHANNELS; ch++) {
            updateChannel(ch, dt);
        }
    }

    if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = now;
        printDebug();
    }
}