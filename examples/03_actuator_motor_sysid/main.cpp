#include <Arduino.h>
#include <Wire.h>
#include "PinMap.h"
#include "RobotConfig.h"

// ==============================================================================
// 1. HARDWARE CONSTANTS & ACTUATOR POLARITIES
// ==============================================================================
constexpr uint8_t TCA9548A_ADDR       = 0x70;
constexpr uint8_t AS5600_ADDR         = 0x36;
constexpr uint8_t AS5600_ANGLE_REG    = 0x0E;
constexpr uint32_t I2C_CLOCK_FREQ_HZ  = 400000;
constexpr uint32_t PWM_FREQ_HZ        = 20000;  // 20 kHz ultrasonic PWM
constexpr uint8_t  PWM_RES_BITS       = 10;     // 10-bit resolution (0 - 1023)

// Universal Left-Mirror Inversion for Encoders
constexpr bool INVERT_ENC_LEFT  = true;  // Left is Inverted on ALL robots
constexpr bool INVERT_ENC_RIGHT = false; // Right is Natural on ALL robots

// Experimentally Discovered Motor Polarities (Forward Command = Forward Roll)
#if ROBOT_ID == 1
    constexpr bool INVERT_MOTOR_L = false;
    constexpr bool INVERT_MOTOR_R = true;
#elif ROBOT_ID == 2
    constexpr bool INVERT_MOTOR_L = true;
    constexpr bool INVERT_MOTOR_R = true;
#elif ROBOT_ID == 3
    constexpr bool INVERT_MOTOR_L = false;
    constexpr bool INVERT_MOTOR_R = true; // Inverted as discovered
#elif ROBOT_ID == 4
    constexpr bool INVERT_MOTOR_L = true;  // Inverted as discovered
    constexpr bool INVERT_MOTOR_R = false;
#endif

// Motor Drive Modes
enum MotorMode : uint8_t {
    MODE_BRAKE     = 0,
    MODE_COAST     = 1,
    MODE_DRIVE_FWD = 2,
    MODE_DRIVE_REV = 3
};

// Raw Encoder State
struct EncoderChannel {
    int32_t cumulativeSteps = 0;
    int16_t lastRawAngle = 0;
    int16_t lastDelta = 0;
    uint16_t currentRawAngle = 0;
    bool isFirstRead = true;
};

EncoderChannel encL;
EncoderChannel encR;

// ==============================================================================
// 2. ENCODER HAL ROUTINES
// ==============================================================================
#if ROBOT_HAS_TCA9548A
bool selectTCAChannel(uint8_t channel) {
    if (channel > 7) return false;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);
    return (Wire.endTransmission() == 0);
}

uint16_t readAS5600Angle(uint8_t channel) {
    if (!selectTCAChannel(channel)) return 0xFFFF;
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_ANGLE_REG);
    if (Wire.endTransmission() != 0) return 0xFFFF;

    Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
    if (Wire.available() >= 2) {
        uint8_t msb = Wire.read();
        uint8_t lsb = Wire.read();
        return ((uint16_t)(msb & 0x0F) << 8) | lsb;
    }
    return 0xFFFF;
}

void pollEncoders() {
    // 1. Left Encoder (Ch0)
    uint16_t rawL = readAS5600Angle(0);
    if (rawL != 0xFFFF) {
        encL.currentRawAngle = rawL;
        int16_t current = (int16_t)rawL;
        if (encL.isFirstRead) {
            encL.lastRawAngle = current;
            encL.isFirstRead = false;
        } else {
            int16_t delta = current - encL.lastRawAngle;
            if (delta > 2048)  delta -= 4096;
            if (delta < -2048) delta += 4096;
            
            encL.lastDelta = -delta; // Invert Left
            encL.cumulativeSteps += encL.lastDelta;
            encL.lastRawAngle = current;
        }
    }

    // 2. Right Encoder (Ch1)
    uint16_t rawR = readAS5600Angle(1);
    if (rawR != 0xFFFF) {
        encR.currentRawAngle = rawR;
        int16_t current = (int16_t)rawR;
        if (encR.isFirstRead) {
            encR.lastRawAngle = current;
            encR.isFirstRead = false;
        } else {
            int16_t delta = current - encR.lastRawAngle;
            if (delta > 2048)  delta -= 4096;
            if (delta < -2048) delta += 4096;

            encR.lastDelta = delta; // Natural Right
            encR.cumulativeSteps += encR.lastDelta;
            encR.lastRawAngle = current;
        }
    }
}
#else
// Direct Dual-I2C on Robot 2
uint16_t readAS5600Direct(TwoWire &bus) {
    bus.beginTransmission(AS5600_ADDR);
    bus.write(AS5600_ANGLE_REG);
    if (bus.endTransmission() != 0) return 0xFFFF;

    bus.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
    if (bus.available() >= 2) {
        uint8_t msb = bus.read();
        uint8_t lsb = bus.read();
        return ((uint16_t)(msb & 0x0F) << 8) | lsb;
    }
    return 0xFFFF;
}

void pollEncoders() {
    // Left Encoder on Wire (GPIO 7, 8)
    uint16_t rawL = readAS5600Direct(Wire);
    if (rawL != 0xFFFF) {
        encL.currentRawAngle = rawL;
        int16_t current = (int16_t)rawL;
        if (encL.isFirstRead) {
            encL.lastRawAngle = current;
            encL.isFirstRead = false;
        } else {
            int16_t delta = current - encL.lastRawAngle;
            if (delta > 2048)  delta -= 4096;
            if (delta < -2048) delta += 4096;

            encL.lastDelta = -delta; // Invert Left
            encL.cumulativeSteps += encL.lastDelta;
            encL.lastRawAngle = current;
        }
    }

    // Right Encoder on Wire1 (GPIO 4, 6)
    uint16_t rawR = readAS5600Direct(Wire1);
    if (rawR != 0xFFFF) {
        encR.currentRawAngle = rawR;
        int16_t current = (int16_t)rawR;
        if (encR.isFirstRead) {
            encR.lastRawAngle = current;
            encR.isFirstRead = false;
        } else {
            int16_t delta = current - encR.lastRawAngle;
            if (delta > 2048)  delta -= 4096;
            if (delta < -2048) delta += 4096;

            encR.lastDelta = delta; // Natural Right
            encR.cumulativeSteps += encR.lastDelta;
            encR.lastRawAngle = current;
        }
    }
}
#endif

// ==============================================================================
// 3. MOTOR CONTROLLER (DRV8833 WITH BRAKE, COAST, AND DIRECTION POLARITIES)
// ==============================================================================
void setupMotors() {
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT);
    pinMode(PIN_MOTOR_R_IN2, OUTPUT);

    ledcAttach(PIN_MOTOR_L_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_MOTOR_L_IN2, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_MOTOR_R_IN1, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_MOTOR_R_IN2, PWM_FREQ_HZ, PWM_RES_BITS);

    // Initial Brake
    ledcWrite(PIN_MOTOR_L_IN1, 1023);
    ledcWrite(PIN_MOTOR_L_IN2, 1023);
    ledcWrite(PIN_MOTOR_R_IN1, 1023);
    ledcWrite(PIN_MOTOR_R_IN2, 1023);
}

void commandMotor(uint8_t pin1, uint8_t pin2, float duty, bool invert, MotorMode &outMode) {
    if (invert) duty = -duty;
    duty = constrain(duty, -1.0f, 1.0f);

    uint32_t val = (uint32_t)(fabs(duty) * 1023.0f);

    if (duty > 0.01f) {
        ledcWrite(pin1, val);
        ledcWrite(pin2, 0);
        outMode = MODE_DRIVE_FWD;
    } else if (duty < -0.01f) {
        ledcWrite(pin1, 0);
        ledcWrite(pin2, val);
        outMode = MODE_DRIVE_REV;
    } else {
        // High-Z Coast (Freewheeling)
        ledcWrite(pin1, 0);
        ledcWrite(pin2, 0);
        outMode = MODE_COAST;
    }
}

void applyBrake() {
    ledcWrite(PIN_MOTOR_L_IN1, 1023);
    ledcWrite(PIN_MOTOR_L_IN2, 1023);
    ledcWrite(PIN_MOTOR_R_IN1, 1023);
    ledcWrite(PIN_MOTOR_R_IN2, 1023);
}

// ==============================================================================
// 4. COMPREHENSIVE 165-SECOND ACTUATOR SYSID FSM
// ==============================================================================
void runActuatorSysIDSequence(uint32_t elapsedMs, float &cmdL, float &cmdR, uint8_t &phase, bool &forceBrake) {
    forceBrake = false;

    // -------------------------------------------------------------------------
    // Phase 0: REST (0 to 5s) -> Baseline Static Sensor Noise
    // -------------------------------------------------------------------------
    if (elapsedMs < 5000) {
        phase = 0;
        cmdL = 0.0f; cmdR = 0.0f;
        forceBrake = true;
    }
    // -------------------------------------------------------------------------
    // Phase 1: LEFT MOTOR STATIC STAIRCASE FORWARD (5 to 23s) -> Deadband & Gain
    // 9 levels: 0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.60, 0.80, 1.00 (2s each)
    // -------------------------------------------------------------------------
    else if (elapsedMs < 23000) {
        phase = 1;
        cmdR = 0.0f;
        uint32_t stepIdx = (elapsedMs - 5000) / 2000;
        constexpr float levels[9] = {0.10f, 0.15f, 0.20f, 0.25f, 0.30f, 0.40f, 0.60f, 0.80f, 1.00f};
        cmdL = (stepIdx < 9) ? levels[stepIdx] : 1.00f;
    }
    // -------------------------------------------------------------------------
    // Phase 2: LEFT MOTOR STATIC STAIRCASE REVERSE (23 to 41s) -> Reverse Gain
    // 9 levels: -0.10 to -1.00 (2s each)
    // -------------------------------------------------------------------------
    else if (elapsedMs < 41000) {
        phase = 2;
        cmdR = 0.0f;
        uint32_t stepIdx = (elapsedMs - 23000) / 2000;
        constexpr float levels[9] = {-0.10f, -0.15f, -0.20f, -0.25f, -0.30f, -0.40f, -0.60f, -0.80f, -1.00f};
        cmdL = (stepIdx < 9) ? levels[stepIdx] : -1.00f;
    }
    // -------------------------------------------------------------------------
    // Phase 3: LEFT MOTOR STEP RESPONSES (41 to 59s) -> Time Constant (tau)
    // 0 -> 0.40 (3s) -> 0 (3s) -> 0.70 (3s) -> 0 (3s) -> 1.00 (3s) -> 0 (3s)
    // -------------------------------------------------------------------------
    else if (elapsedMs < 59000) {
        phase = 3;
        cmdR = 0.0f;
        uint32_t subMs = elapsedMs - 41000;
        if (subMs < 3000)       cmdL = 0.40f;
        else if (subMs < 6000)  cmdL = 0.0f;
        else if (subMs < 9000)  cmdL = 0.70f;
        else if (subMs < 12000) cmdL = 0.0f;
        else if (subMs < 15000) cmdL = 1.00f;
        else                    cmdL = 0.0f;
    }
    // -------------------------------------------------------------------------
    // Phase 4: RIGHT MOTOR STATIC STAIRCASE FORWARD (59 to 77s)
    // -------------------------------------------------------------------------
    else if (elapsedMs < 77000) {
        phase = 4;
        cmdL = 0.0f;
        uint32_t stepIdx = (elapsedMs - 59000) / 2000;
        constexpr float levels[9] = {0.10f, 0.15f, 0.20f, 0.25f, 0.30f, 0.40f, 0.60f, 0.80f, 1.00f};
        cmdR = (stepIdx < 9) ? levels[stepIdx] : 1.00f;
    }
    // -------------------------------------------------------------------------
    // Phase 5: RIGHT MOTOR STATIC STAIRCASE REVERSE (77 to 95s)
    // -------------------------------------------------------------------------
    else if (elapsedMs < 95000) {
        phase = 5;
        cmdL = 0.0f;
        uint32_t stepIdx = (elapsedMs - 77000) / 2000;
        constexpr float levels[9] = {-0.10f, -0.15f, -0.20f, -0.25f, -0.30f, -0.40f, -0.60f, -0.80f, -1.00f};
        cmdR = (stepIdx < 9) ? levels[stepIdx] : -1.00f;
    }
    // -------------------------------------------------------------------------
    // Phase 6: RIGHT MOTOR STEP RESPONSES (95 to 113s) -> Time Constant (tau)
    // -------------------------------------------------------------------------
    else if (elapsedMs < 113000) {
        phase = 6;
        cmdL = 0.0f;
        uint32_t subMs = elapsedMs - 95000;
        if (subMs < 3000)       cmdR = 0.40f;
        else if (subMs < 6000)  cmdR = 0.0f;
        else if (subMs < 9000)  cmdR = 0.70f;
        else if (subMs < 12000) cmdR = 0.0f;
        else if (subMs < 15000) cmdR = 1.00f;
        else                    cmdR = 0.0f;
    }
    // -------------------------------------------------------------------------
    // Phase 7: DUAL MOTOR SYNCHRONOUS STEPS (113 to 137s) -> Asymmetry Ratio
    // +0.3 (6s) -> +0.5 (6s) -> +0.7 (6s) -> +0.9 (6s)
    // -------------------------------------------------------------------------
    else if (elapsedMs < 137000) {
        phase = 7;
        uint32_t subMs = elapsedMs - 113000;
        float val = (subMs < 6000) ? 0.30f : (subMs < 12000) ? 0.50f : (subMs < 18000) ? 0.70f : 0.90f;
        cmdL = val; cmdR = val;
    }
    // -------------------------------------------------------------------------
    // Phase 8: COAST-DOWN FRICTION TEST (137 to 157s) -> Mechanical Damping
    // Drive 80% (5s) -> COAST (5s) -> Drive -80% (5s) -> COAST (5s)
    // -------------------------------------------------------------------------
    else if (elapsedMs < 157000) {
        phase = 8;
        uint32_t subMs = elapsedMs - 137000;
        if (subMs < 5000)       { cmdL = 0.80f;  cmdR = 0.80f; }
        else if (subMs < 10000) { cmdL = 0.00f;  cmdR = 0.00f; } // Coasting!
        else if (subMs < 15000) { cmdL = -0.80f; cmdR = -0.80f; }
        else                    { cmdL = 0.00f;  cmdR = 0.00f; } // Coasting!
    }
    // -------------------------------------------------------------------------
    // Phase 9: TERMINATION & BRAKE (157s+)
    // -------------------------------------------------------------------------
    else {
        phase = 9;
        cmdL = 0.0f; cmdR = 0.0f;
        forceBrake = true;
    }
}

// ==============================================================================
// 5. SETUP & 100 HZ REAL-TIME EXECUTION
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1000);

    pinMode(PIN_STATUS_RGB, OUTPUT);
    digitalWrite(PIN_STATUS_RGB, LOW);

#if ROBOT_HAS_TCA9548A
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_CLOCK_FREQ_HZ);
#else
    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_CLOCK_FREQ_HZ);   // GPIO 7, 8
    Wire1.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, I2C_CLOCK_FREQ_HZ); // GPIO 4, 6
#endif
    delay(20);

    setupMotors();

    // 12-Column High-Resolution SysID Header
    Serial.println("TimeUs,Phase,PwmCmdL,PwmCmdR,ModeL,ModeR,RawAngL,RawAngR,DeltaL,DeltaR,StepsL,StepsR");
}

void loop() {
    static uint32_t startTestTimeMs = millis();
    static bool testCompleted = false;

    if (testCompleted) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    uint32_t nowMs = millis();
    uint32_t elapsedMs = nowMs - startTestTimeMs;
    uint64_t nowUs = micros();

    // 1. Read Encoders
    pollEncoders();

    // 2. Compute Excitation Profile
    float cmdL = 0.0f;
    float cmdR = 0.0f;
    uint8_t phase = 0;
    bool forceBrake = false;
    runActuatorSysIDSequence(elapsedMs, cmdL, cmdR, phase, forceBrake);

    // 3. Command Motors
    MotorMode modeL = MODE_BRAKE;
    MotorMode modeR = MODE_BRAKE;

    if (forceBrake) {
        applyBrake();
        modeL = MODE_BRAKE;
        modeR = MODE_BRAKE;
    } else {
        commandMotor(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, cmdL, INVERT_MOTOR_L, modeL);
        commandMotor(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, cmdR, INVERT_MOTOR_R, modeR);
    }

    // 4. Stream 12-Column Pure CSV Data Row (100 Hz)
    Serial.printf("%llu,%u,%.3f,%.3f,%u,%u,%u,%u,%d,%d,%ld,%ld\n",
                  (unsigned long long)nowUs,
                  phase,
                  cmdL,
                  cmdR,
                  (uint8_t)modeL,
                  (uint8_t)modeR,
                  encL.currentRawAngle,
                  encR.currentRawAngle,
                  encL.lastDelta,
                  encR.lastDelta,
                  (long)encL.cumulativeSteps,
                  (long)encR.cumulativeSteps);

    // 5. Automatic Shutdown at 160 Seconds
    if (elapsedMs >= 160000) {
        testCompleted = true;
        applyBrake();

        Serial.println("\n======================================================================");
        Serial.printf("[ACTUATOR SYSID COMPLETE] Robot %d Benchmark Finished.\n", Config::ID);
        Serial.printf("Final Displacements: Left=%ld | Right=%ld (Forward Positive Verified)\n", 
                      (long)encL.cumulativeSteps, (long)encR.cumulativeSteps);
        Serial.println("======================================================================");
    }

    delay(10); // 100 Hz Loop Rate
}
