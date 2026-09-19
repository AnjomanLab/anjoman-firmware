#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "PinMap.h"
#include "RobotConfig.h"
#include "MagneticEncoder.h"
#include "MotorController.h"
#include "BMI160_Custom.h"
#include "HeadingKalmanFilter.h"

// ==============================================================================
// 1. HARDWARE & KINEMATIC CONSTANTS
// ==============================================================================
constexpr uint8_t  TCA9548A_ADDR       = 0x70;
constexpr uint8_t  INA226_ADDR         = 0x40;
constexpr uint32_t I2C_CLOCK_FREQ_HZ  = 400000;
constexpr uint32_t CONTROL_RATE_HZ     = 100;
constexpr uint32_t SAMPLE_PERIOD_US   = 1000000 / CONTROL_RATE_HZ; // 10 ms
constexpr float    CONTROL_PERIOD_S    = 0.010f;

// Calibrated Physical Constants
constexpr float    R_EFF_M             = Config::WHEEL_RADIUS_M; // 0.02685 m
constexpr float    W_EFF_M             = Config::TRACK_WIDTH_M;  // Decoupled effective track width

// 1.000m Straight Leg Target Ticks:
constexpr int32_t  LEG_TARGET_TICKS    = (int32_t)((1.000f / (2.0f * PI * R_EFF_M)) * Config::ENCODER_CPR); // 24278 ticks
constexpr float    CRUISE_SPEED_MPS    = 0.15f;
constexpr float    CRUISE_RPM          = (CRUISE_SPEED_MPS / (2.0f * PI * R_EFF_M)) * 60.0f; // 53.30 RPM
constexpr float    TURN_RPM            = 30.0f; // Stable 90-degree in-place turn speed

constexpr float    SHUNT_RESISTOR_OHM  = Config::SHUNT_RESISTOR_OHM;

// Hardware Interfaces
SPIClass SPI_SD(HSPI);
File logFile;
bool sdCardReady = false;
bool ina226Ready = false;

MagneticEncoder encL(Wire, TCA9548A_ADDR, 0, Config::INVERT_ENCODER_LEFT);
MagneticEncoder encR(Wire, TCA9548A_ADDR, 1, Config::INVERT_ENCODER_RIGHT);
MotorController motorL(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, Config::INVERT_MOTOR_LEFT);
MotorController motorR(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, Config::INVERT_MOTOR_RIGHT);
BMI160_Custom   imu(Wire, 0x69, 2);
HeadingKalmanFilter kf;

// 2D Dead Reckoning Position State
float posX_m = 0.0f;
float posY_m = 0.0f;

#pragma pack(push, 1)
struct SquareBenchmarkRecord {
    uint64_t timestamp_us;
    uint32_t elapsed_ms;
    uint8_t  state;              // 0: ZUPT, 1..4: Straight Legs, 11..14: 90-deg Turns, 99: Done
    float    pos_x_m;
    float    pos_y_m;
    float    filtered_yaw_deg;
    float    filtered_bias_dps;
    float    raw_gyro_z_dps;
    float    meas_rpm_l;
    float    meas_rpm_r;
    float    pwm_duty_l;
    float    pwm_duty_r;
    int32_t  steps_l;
    int32_t  steps_r;
    float    vbus_v;
    float    current_ma;
    uint8_t  slip_detected;
};
#pragma pack(pop)

QueueHandle_t sdLogQueue = nullptr;
TaskHandle_t  core0TaskHandle = nullptr;

bool selectTCAChannel(uint8_t ch) {
    if (ch > 7) return false;
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << ch);
    return (Wire.endTransmission() == 0);
}

bool initINA226() {
    if (!selectTCAChannel(3)) return false;
    Wire.beginTransmission(INA226_ADDR);
    if (Wire.endTransmission() != 0) return false;

    Wire.beginTransmission(INA226_ADDR);
    Wire.write(0x00); Wire.write(0x80); Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(INA226_ADDR);
    Wire.write(0x00); Wire.write(0x42); Wire.write(0x27);
    return (Wire.endTransmission() == 0);
}

bool readINA226(float &vbus_V, float &current_mA) {
    vbus_V = current_mA = 0.0f;
    if (!selectTCAChannel(3)) return false;

    Wire.beginTransmission(INA226_ADDR);
    Wire.write(0x02);
    if (Wire.endTransmission() != 0) return false;
    Wire.requestFrom((uint8_t)INA226_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return false;
    uint16_t rawBus = (Wire.read() << 8) | Wire.read();
    vbus_V = (float)rawBus * 0.00125f;

    Wire.beginTransmission(INA226_ADDR);
    Wire.write(0x01);
    if (Wire.endTransmission() != 0) return false;
    Wire.requestFrom((uint8_t)INA226_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return false;
    int16_t rawShunt = (int16_t)((Wire.read() << 8) | Wire.read());
    float vshunt_mV = (float)rawShunt * 0.0025f;

    current_mA = vshunt_mV / SHUNT_RESISTOR_OHM;
    return true;
}

void core0LoggerTask(void *pvParameters) {
    SquareBenchmarkRecord rec;
    uint32_t recordsLogged = 0;

    while (true) {
        if (xQueueReceive(sdLogQueue, &rec, portMAX_DELAY) == pdTRUE) {
            if (sdCardReady && logFile) {
                logFile.printf("%llu,%lu,%u,%.4f,%.4f,%.2f,%.3f,%.2f,%.2f,%.2f,%.3f,%.3f,%ld,%ld,%.2f,%.1f,%u\n",
                               (unsigned long long)rec.timestamp_us,
                               (unsigned long)rec.elapsed_ms,
                               rec.state,
                               rec.pos_x_m, rec.pos_y_m,
                               rec.filtered_yaw_deg, rec.filtered_bias_dps, rec.raw_gyro_z_dps,
                               rec.meas_rpm_l, rec.meas_rpm_r,
                               rec.pwm_duty_l, rec.pwm_duty_r,
                               (long)rec.steps_l, (long)rec.steps_r,
                               rec.vbus_v, rec.current_ma,
                               rec.slip_detected);

                recordsLogged++;
                if (recordsLogged % 50 == 0) {
                    logFile.flush();
                }
            }
        }
    }
}

void setup() {
    Serial.begin(460800);
    delay(1000);

    rgbLedWrite(PIN_STATUS_RGB, 20, 10, 0); // Amber

    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_CLOCK_FREQ_HZ);
    delay(50);

    encL.begin();
    encR.begin();

    motorL.begin(20000, 10);
    motorR.begin(20000, 10);

    MotorSysIDParams paramsL = {
        Config::DEADBAND_FWD_L, Config::DEADBAND_REV_L,
        Config::GAIN_RPM_FWD_L, Config::GAIN_RPM_REV_L,
        7.40f
    };
    MotorSysIDParams paramsR = {
        Config::DEADBAND_FWD_R, Config::DEADBAND_REV_R,
        Config::GAIN_RPM_FWD_R, Config::GAIN_RPM_REV_R,
        7.40f
    };
    motorL.setCalibration(paramsL);
    motorR.setCalibration(paramsR);

    PIDGains gainsL = {0.0030f, 0.0250f, 0.25f};
    PIDGains gainsR = {0.0030f, 0.0250f, 0.25f};
    motorL.setPIDGains(gainsL);
    motorR.setPIDGains(gainsR);

    imu.begin();
    ina226Ready = initINA226();

    // Initialize Heading Kalman Filter with identified covariances
    kf.init(0.0f, Config::GYRO_BIAS_Z_RAD_S, Config::Q_YAW_DISCRETE, Config::Q_GYRO_BIAS_WALK, Config::R_YAW_ENCODER);

    SPI_SD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);
    delay(50);

    if (SD.begin(PIN_SD_CS, SPI_SD, 10000000)) {
        sdCardReady = true;
        char filename[32];
        snprintf(filename, sizeof(filename), "/square_1m_r%d.csv", Config::ID);
        logFile = SD.open(filename, FILE_WRITE);
        if (logFile) {
            logFile.println("timestamp_us,elapsed_ms,state,pos_x_m,pos_y_m,yaw_deg,bias_dps,raw_gz_dps,rpm_l,rpm_r,pwm_l,pwm_r,steps_l,steps_r,vbus_v,current_ma,slip");
            logFile.flush();
        }
    }

    sdLogQueue = xQueueCreate(256, sizeof(SquareBenchmarkRecord));
    xTaskCreatePinnedToCore(core0LoggerTask, "SDLogger", 4096, nullptr, 1, &core0TaskHandle, 0);

    Serial.println("======================================================================");
    Serial.printf("   ANJOMAN 1x1 METER SQUARE MANEUVER BENCHMARK (ROBOT %d)\n", Config::ID);
    Serial.printf("   Calibrated R_eff: %.4f m | Decoupled W_eff: %.4f m\n", R_EFF_M, W_EFF_M);
    Serial.printf("   Target: 4 Legs of 1.000m (%d ticks) + 4 In-Place 90-deg CCW Turns\n", LEG_TARGET_TICKS);
    Serial.printf("   MicroSD File: /square_1m_r%d.csv | Status: %s\n", Config::ID, sdCardReady ? "READY" : "FAILED");
    Serial.println("======================================================================");
}

void loop() {
    static uint32_t startTestTimeMs = millis();
    static uint64_t nextSampleUs = micros();
    static bool testComplete = false;

    // Maneuver State Machine:
    // 0: ZUPT (3s)
    // 1: Leg 1 (Straight +Y, 1m)   | 11: Turn 1 (Spin to 90 deg)
    // 2: Leg 2 (Straight -X, 1m)   | 12: Turn 2 (Spin to 180 deg)
    // 3: Leg 3 (Straight -Y, 1m)   | 13: Turn 3 (Spin to 270 deg)
    // 4: Leg 4 (Straight +X, 1m)   | 14: Turn 4 (Spin to 360/0 deg)
    // 99: Complete
    static uint8_t  state = 0;
    static uint32_t stateStartTimeMs = 0;
    static int32_t  legStartStepsL = 0;
    static int32_t  legStartStepsR = 0;
    static float    targetHeadingDeg = 0.0f;

    static float gyroBiasSum = 0.0f;
    static uint32_t zuptCount = 0;

    static int32_t prevStepsL = 0;
    static int32_t prevStepsR = 0;

    if (testComplete) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    uint64_t nowUs = micros();
    uint32_t elapsedMs = millis() - startTestTimeMs;

    if (nowUs >= nextSampleUs) {
        nextSampleUs = nowUs + SAMPLE_PERIOD_US;

        // 1. Hardware Sensing
        encL.update(CONTROL_PERIOD_S);
        encR.update(CONTROL_PERIOD_S);

        imu.readSensorData();
        float gz_raw_dps = imu.getGyroZ();
        float gz_raw_rad_s = gz_raw_dps * (PI / 180.0f);

        float vbus_V = 7.40f;
        float current_mA = 0.0f;
        readINA226(vbus_V, current_mA);
        if (vbus_V < 6.0f) vbus_V = 7.40f;

        int32_t stepsL = encL.getCumulativeSteps();
        int32_t stepsR = encR.getCumulativeSteps();

        int32_t dStepsL = stepsL - prevStepsL;
        int32_t dStepsR = stepsR - prevStepsR;
        prevStepsL = stepsL;
        prevStepsR = stepsR;

        float ds_l = ((float)dStepsL / Config::ENCODER_CPR) * (2.0f * PI * R_EFF_M);
        float ds_r = ((float)dStepsR / Config::ENCODER_CPR) * (2.0f * PI * R_EFF_M);
        float ds_mid = (ds_r + ds_l) * 0.5f;
        float delta_theta_wheel = (ds_r - ds_l) / W_EFF_M;

        // 2. Kalman Filter Heading Estimation (100 Hz)
        kf.predict(gz_raw_rad_s, CONTROL_PERIOD_S);
        bool slip = !kf.updateEncoder(delta_theta_wheel, gz_raw_rad_s, CONTROL_PERIOD_S);

        float currentHeadingRad = kf.getHeadingRad();
        float currentHeadingDeg = kf.getHeadingDeg();

        // 3. Cartesian Dead Reckoning Position Update (Forward = +Y, Right = +X, Left = -X)
        // With standard CCW rotation (+theta turns left towards -X):
        posX_m += ds_mid * (-sinf(currentHeadingRad));
        posY_m += ds_mid * cosf(currentHeadingRad);

        float targetRpmL = 0.0f;
        float targetRpmR = 0.0f;
        float dutyL = 0.0f;
        float dutyR = 0.0f;

        // 4. Square Maneuver State Machine
        if (state == 0) {
            // ZUPT: Pre-mission static calibration (0 to 3000 ms)
            motorL.brake();
            motorR.brake();
            gyroBiasSum += gz_raw_rad_s;
            zuptCount++;

            if (elapsedMs >= 3000) {
                if (zuptCount > 100) {
                    float measuredBias = gyroBiasSum / (float)zuptCount;
                    kf.init(0.0f, measuredBias, Config::Q_YAW_DISCRETE, Config::Q_GYRO_BIAS_WALK, Config::R_YAW_ENCODER);
                }
                state = 1; // Transition to Leg 1
                stateStartTimeMs = millis();
                legStartStepsL = stepsL;
                legStartStepsR = stepsR;
                rgbLedWrite(PIN_STATUS_RGB, 0, 0, 50); // Blue (Driving Straight)
            }
        }
        // Straight Driving Legs (1, 2, 3, 4)
        else if (state >= 1 && state <= 4) {
            targetRpmL = CRUISE_RPM;
            targetRpmR = CRUISE_RPM;

            int32_t legStepsL = stepsL - legStartStepsL;
            int32_t legStepsR = stepsR - legStartStepsR;
            int32_t avgLegTicks = (legStepsL + legStepsR) / 2;

            // Cross-Coupled Tick Sync to maintain strict straightness
            int32_t tickError = legStepsL - legStepsR;
            float syncCorrection = constrain((float)tickError * 0.00015f, -0.08f, 0.08f);

            dutyL = motorL.computeVelocityControl(targetRpmL, encL.getRPM(), vbus_V, CONTROL_PERIOD_S, -syncCorrection);
            dutyR = motorR.computeVelocityControl(targetRpmR, encR.getRPM(), vbus_V, CONTROL_PERIOD_S, +syncCorrection);

            // Transition: Exactly 1.000m reached
            if (avgLegTicks >= LEG_TARGET_TICKS || (millis() - stateStartTimeMs) >= 12000) {
                state += 10; // State 1 -> 11, 2 -> 12, etc.
                targetHeadingDeg += 90.0f; // Target next corner
                stateStartTimeMs = millis();
                motorL.brake();
                motorR.brake();
                rgbLedWrite(PIN_STATUS_RGB, 40, 0, 40); // Magenta (In-Place Turn)
            }
        }
        // In-Place 90-Degree Closed-Loop Turns (11, 12, 13, 14)
        else if (state >= 11 && state <= 14) {
            // Turn CCW (+yaw)
            targetRpmL = -TURN_RPM;
            targetRpmR = +TURN_RPM;

            dutyL = motorL.computeVelocityControl(targetRpmL, encL.getRPM(), vbus_V, CONTROL_PERIOD_S, 0.0f);
            dutyR = motorR.computeVelocityControl(targetRpmR, encR.getRPM(), vbus_V, CONTROL_PERIOD_S, 0.0f);

            // Threshold: Stop precisely when Kalman Filter Heading reaches target
            if (currentHeadingDeg >= (targetHeadingDeg - 1.0f) || (millis() - stateStartTimeMs) >= 8000) {
                motorL.brake();
                motorR.brake();

                if (state == 14) {
                    // All 4 legs and 4 turns complete!
                    state = 99;
                    testComplete = true;

                    vTaskDelay(pdMS_TO_TICKS(150));
                    if (logFile) {
                        logFile.flush();
                        logFile.close();
                    }

                    rgbLedWrite(PIN_STATUS_RGB, 0, 60, 0); // Solid Green (Mission Complete)

                    Serial.println("\n======================================================================");
                    Serial.printf("[MISSION COMPLETE] 1x1m Square Finished for Robot %d\n", Config::ID);
                    Serial.printf("   Calculated Final Position: X = %.4f m, Y = %.4f m\n", posX_m, posY_m);
                    Serial.printf("   Filtered Final Yaw: %.2f deg (Nominal: 360.00 deg)\n", currentHeadingDeg);
                    Serial.printf("   Estimated Final Gyro Bias: %.4f deg/s\n", kf.getBiasDegS());
                    Serial.printf("   Return-to-Origin Distance Error: %.1f mm\n", sqrtf(posX_m*posX_m + posY_m*posY_m) * 1000.0f);
                    Serial.println("   --> Measure actual physical position from the start line on the floor!");
                    Serial.println("======================================================================");
                } else {
                    // Transition to next straight leg
                    state = (state - 10) + 1;
                    stateStartTimeMs = millis();
                    legStartStepsL = stepsL;
                    legStartStepsR = stepsR;
                    rgbLedWrite(PIN_STATUS_RGB, 0, 0, 50); // Blue
                }
            }
        }

        // 5. Stream Real-Time Telemetry to Core 0 Async Queue
        SquareBenchmarkRecord rec = {};
        rec.timestamp_us = nowUs;
        rec.elapsed_ms = elapsedMs;
        rec.state = state;
        rec.pos_x_m = posX_m;
        rec.pos_y_m = posY_m;
        rec.filtered_yaw_deg = currentHeadingDeg;
        rec.filtered_bias_dps = kf.getBiasDegS();
        rec.raw_gyro_z_dps = gz_raw_dps;
        rec.meas_rpm_l = encL.getRPM();
        rec.meas_rpm_r = encR.getRPM();
        rec.pwm_duty_l = dutyL;
        rec.pwm_duty_r = dutyR;
        rec.steps_l = stepsL;
        rec.steps_r = stepsR;
        rec.vbus_v = vbus_V;
        rec.current_ma = current_mA;
        rec.slip_detected = slip ? 1 : 0;

        if (sdLogQueue != nullptr) {
            xQueueSend(sdLogQueue, &rec, 0);
        }
    }
    yield();
}


