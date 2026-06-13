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

// PCA9685 default I2C address is usually 0x40
static constexpr uint8_t PCA9685_ADDR = PCA9685_DEFAULT_ADDRESS;

// =====================================================
// Control tuning
// =====================================================

// How fast the target angle changes when joystick is fully pushed.
// This controls the "maze tilt rate".
static constexpr float MAX_SPEED_DPS = 20.0f;

// How fast the target angle returns to center when joystick is released.
static constexpr float RETURN_SPEED_DPS = 35.0f;

// Easing speed passed to ServoEasing.
// This should be higher than MAX_SPEED_DPS so the servo can track the target smoothly.
static constexpr float EASING_SPEED_DPS = 210.0f;

// Minimum target angle change before sending a new command.
// Prevents spamming ServoEasing due to ADC noise.
static constexpr float TARGET_THRESHOLD_DEG = 0.2f;

// Joystick smoothing.
// 0.05 = very smooth but slower response.
// 0.15 to 0.25 = good starting range.
static constexpr float COMMAND_SMOOTH_ALPHA = 0.10f;

// Optional exponential response.
// 1.0 = linear.
// 1.5 to 2.5 = slower near center, stronger near full push.
static constexpr float RESPONSE_EXPONENT = 2.0f;

// Timing
static constexpr unsigned long CONTROL_INTERVAL_MS = 20;
static constexpr unsigned long PRINT_INTERVAL_MS   = 300;

// Safety cap for dt in case Serial printing or other code delays the loop.
static constexpr float MAX_DT_SECONDS = 0.05f;

static constexpr float DEFAULT_SERVO_LIMIT_DEG = 20.0f;
static constexpr float ABS_SERVO_MIN_DEG = 0.0f;
static constexpr float ABS_SERVO_MAX_DEG = 180.0f;

// =====================================================
// Per-channel configuration
// =====================================================
//
// IMPORTANT:
// - joyPin: ESP32 ADC pin connected to the joystick signal.
// - servoChannel: PCA9685 output channel.
// - direction: set to -1 if the servo moves opposite to what you want.
// - minRaw/maxRaw: tune after checking Serial Monitor.
// - deadzone: ADC counts around center where joystick is treated as released.
// - centerDeg/minDeg/maxDeg: tune based on your maze mechanism.
//
// Current pin example keeps your existing GPIO25 and GPIO26 as first two channels.
// GPIO25 and GPIO26 are ADC2 pins, so avoid Wi-Fi while using them.
// If possible, use ADC1 pins for more stable analog reading.

struct ChannelConfig {
    int joyPin;
    int servoChannel;
    int direction;

    int minRaw;
    int maxRaw;
    int deadzone;

    float centerDeg;
    float limitDeg;
};

static ChannelConfig channelCfg[NUM_CHANNELS] = {
    // joyPin, servoCh, dir, minRaw, maxRaw, deadzone, center, limit
    {25, 0,  1, 100, 4000, 180,  90.0f, DEFAULT_SERVO_LIMIT_DEG},
    {26, 1,  1, 100, 4000, 180,  95.0f, DEFAULT_SERVO_LIMIT_DEG},
    {32, 2,  1, 100, 4000, 180,  85.0f, DEFAULT_SERVO_LIMIT_DEG},
    {33, 3,  1, 100, 4000, 180,  95.0f, DEFAULT_SERVO_LIMIT_DEG},
    {34, 4,  1, 100, 4000, 180, 109.0f, DEFAULT_SERVO_LIMIT_DEG},
    {35, 5,  1, 100, 4000, 180, 105.0f, DEFAULT_SERVO_LIMIT_DEG},
    {36, 6,  1, 100, 4000, 180,  43.0f, DEFAULT_SERVO_LIMIT_DEG},
    {39, 7,  1, 100, 4000, 180,  53.0f, DEFAULT_SERVO_LIMIT_DEG}
};

// =====================================================
// Servo objects
// =====================================================
//
// Using individual objects + pointer array is safer with Arduino libraries
// than trying to dynamically allocate or copy ServoEasing objects.

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
    int centerRaw = 2048;
    int raw       = 2048;

    float filteredCmd = 0.0f;
    float targetDeg   = 90.0f;
    float lastSentDeg = 90.0f;
    float speedDps    = 0.0f;

    bool inDeadzone = true;
};

ChannelState channelState[NUM_CHANNELS];

unsigned long lastControlTime = 0;
unsigned long lastPrintTime   = 0;

// =====================================================
// Helper functions
// =====================================================

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static float getServoMinDeg(int ch) {
    return clampf(
        channelCfg[ch].centerDeg - channelCfg[ch].limitDeg,
        ABS_SERVO_MIN_DEG,
        ABS_SERVO_MAX_DEG
    );
}

static float getServoMaxDeg(int ch) {
    return clampf(
        channelCfg[ch].centerDeg + channelCfg[ch].limitDeg,
        ABS_SERVO_MIN_DEG,
        ABS_SERVO_MAX_DEG
    );
}

static float moveToward(float current, float target, float maxStep) {
    float error = target - current;

    if (fabsf(error) <= maxStep) {
        return target;
    }

    return current + ((error > 0.0f) ? maxStep : -maxStep);
}

static float applyExpo(float x, float exponent) {
    float signValue = (x >= 0.0f) ? 1.0f : -1.0f;
    float magnitude = powf(fabsf(x), exponent);
    return signValue * magnitude;
}

static bool attachServo(ServoEasing* servo, int channel) {
    if (servo->attach(channel) == INVALID_SERVO) {
        Serial.printf("ERROR: failed to attach servo on PCA9685 channel %d\n", channel);
        return false;
    }

    Serial.printf("Servo attached on PCA9685 channel %d\n", channel);
    return true;
}

// Calibrate all joystick centers at startup.
// Keep all joysticks released during this process.
static void calibrateAllJoysticks() {
    static constexpr int SAMPLE_COUNT = 100;

    long sum[NUM_CHANNELS] = {0};

    Serial.println("Calibrating joystick centers...");
    Serial.println("Do not touch the joysticks.");

    delay(500);

    for (int sample = 0; sample < SAMPLE_COUNT; sample++) {
        for (int ch = 0; ch < NUM_CHANNELS; ch++) {
            sum[ch] += analogRead(channelCfg[ch].joyPin);
        }
        delay(2);
    }

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        channelState[ch].centerRaw = static_cast<int>(sum[ch] / SAMPLE_COUNT);
        channelState[ch].raw = channelState[ch].centerRaw;

        Serial.printf(
            "CH%d joystick center = %d\n",
            ch,
            channelState[ch].centerRaw
        );
    }
}

// Returns normalized joystick command from -1.0 to +1.0.
// 0.0 means joystick is inside the deadzone.
static float readJoystickCommand(int ch) {
    ChannelConfig& cfg = channelCfg[ch];
    ChannelState&  st  = channelState[ch];

    int raw = analogRead(cfg.joyPin);
    st.raw = raw;

    int center = st.centerRaw;
    int offset = raw - center;

    if (abs(offset) <= cfg.deadzone) {
        st.inDeadzone = true;
        return 0.0f;
    }

    st.inDeadzone = false;

    float normalized = 0.0f;

    if (offset > 0) {
        float denom = static_cast<float>(cfg.maxRaw - center - cfg.deadzone);

        if (denom <= 1.0f) {
            return 0.0f;
        }

        normalized = static_cast<float>(offset - cfg.deadzone) / denom;
    } else {
        float denom = static_cast<float>(center - cfg.minRaw - cfg.deadzone);

        if (denom <= 1.0f) {
            return 0.0f;
        }

        normalized = static_cast<float>(offset + cfg.deadzone) / denom;
    }

    normalized = clampf(normalized, -1.0f, 1.0f);

    // Exponential response gives finer control near joystick center.
    normalized = applyExpo(normalized, RESPONSE_EXPONENT);

    // Apply servo direction.
    normalized *= static_cast<float>(cfg.direction);

    return clampf(normalized, -1.0f, 1.0f);
}

static void sendServoTargetIfNeeded(int ch) {
    ChannelConfig& cfg = channelCfg[ch];
    ChannelState&  st  = channelState[ch];

    st.targetDeg = clampf(st.targetDeg, getServoMinDeg(ch), getServoMaxDeg(ch));

    if (fabsf(st.targetDeg - st.lastSentDeg) < TARGET_THRESHOLD_DEG) {
        return;
    }

    st.lastSentDeg = st.targetDeg;

    servos[ch]->startEaseTo(st.targetDeg, EASING_SPEED_DPS);
}

// =====================================================
// Core channel update
// =====================================================

static void updateChannel(int ch, float dtSeconds) {
    ChannelConfig& cfg = channelCfg[ch];
    ChannelState&  st  = channelState[ch];

    float cmd = readJoystickCommand(ch);

    if (cmd == 0.0f) {
        // Joystick released.
        // Reset filtered command immediately to avoid "speed tail" after release.
        st.filteredCmd = 0.0f;
        st.speedDps = 0.0f;

        // Move target angle back toward center gradually.
        float maxReturnStep = RETURN_SPEED_DPS * dtSeconds;

        st.targetDeg = moveToward(
            st.targetDeg,
            cfg.centerDeg,
            maxReturnStep
        );
    } else {
        // Joystick active.
        // Smooth only active joystick commands.
        st.filteredCmd += COMMAND_SMOOTH_ALPHA * (cmd - st.filteredCmd);

        st.speedDps = st.filteredCmd * MAX_SPEED_DPS;

        st.targetDeg += st.speedDps * dtSeconds;

        st.targetDeg = clampf(st.targetDeg, getServoMinDeg(ch), getServoMaxDeg(ch));
    }

    sendServoTargetIfNeeded(ch);
}

// =====================================================
// Debug printing
// =====================================================

static void printDebug() {
    Serial.print("\r\n");
    Serial.print("CH RAW  CTR  DELTA CMD    SPD    TGT\r\n");
    Serial.print("-----------------------------------------\r\n");

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        ChannelState& st = channelState[ch];

        int delta = st.raw - st.centerRaw;

        Serial.printf(
            "%02d %4d %4d %+5d %+5.2f %+6.2f %6.1f\r\n",
            ch,
            st.raw,
            st.centerRaw,
            delta,
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
    Serial.println("Starting 8-channel maze game system...");

    // I2C setup for PCA9685
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    // ESP32 ADC setup
    analogReadResolution(12);

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        analogSetPinAttenuation(channelCfg[ch].joyPin, ADC_11db);
    }

    calibrateAllJoysticks();

    // Attach and initialize all servos
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        bool ok = attachServo(servos[ch], channelCfg[ch].servoChannel);

        if (!ok) {
            Serial.printf("CH%d disabled due to servo attach error.\n", ch);
            continue;
        }

        servos[ch]->setEasingType(EASE_LINEAR);

        channelState[ch].targetDeg   = channelCfg[ch].centerDeg;
        channelState[ch].lastSentDeg = channelCfg[ch].centerDeg;

        servos[ch]->write(channelCfg[ch].centerDeg);

        delay(50);
    }

    lastControlTime = millis();
    lastPrintTime   = millis();

    Serial.println();
    Serial.println("Maze game ready.");
    Serial.println("Move one joystick at a time first to verify direction and angle limits.");
}

// =====================================================
// Main loop
// =====================================================

void loop() {
    unsigned long now = millis();

    // Required by ServoEasing for non-blocking movement.
    updateAllServos();

    if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
        float dt = static_cast<float>(now - lastControlTime) / 1000.0f;
        lastControlTime = now;

        // Prevent a large jump if loop timing is interrupted.
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