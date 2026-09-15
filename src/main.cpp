#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "PinMap.h"
#include "RobotConfig.h"
#include "MagneticEncoder.h"
#include "MotorController.h"
#include "BMI160_Custom.h"

// ==============================================================================
// 1. HARDWARE DEFINITIONS & 10-TURN BIDIRECTIONAL METROLOGY CONSTANTS
// ==============================================================================
constexpr uint8_t  TCA9548A_ADDR       = 0x70;
constexpr uint8_t  INA226_ADDR         = 0x40;
constexpr uint32_t I2C_CLOCK_FREQ_HZ  = 400000;
constexpr uint32_t CONTROL_RATE_HZ     = 100;
constexpr uint32_t SAMPLE_PERIOD_US   = 1000000 / CONTROL_RATE_HZ; // 10 ms
constexpr float    CONTROL_PERIOD_S    = 0.010f;

constexpr float    R_EFF_M             = Config::WHEEL_RADIUS_M; // 0.02685 m
constexpr float    W_NOM_M             = Config::TRACK_WIDTH_M;  // 0.1350m (R1) or 0.1250m (R2-R4)

// 10 Full Turns (3600 deg = 20*PI rad) nominal wheel displacement and ticks:
// S_wheel = 10 * PI * W_NOM_M
constexpr int32_t  TARGET_10TURN_TICKS = (int32_t)((10.0f * PI * W_NOM_M) / (2.0f * PI * R_EFF_M) * Config::ENCODER_CPR);
constexpr float    SPIN_CRUISE_RPM     = 35.0f; // Soft, stable angular rate (~90 deg/s body yaw)

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

#pragma pack(push, 1)
struct BidirectionalSpinRecord {
    uint64_t timestamp_us;
    uint32_t elapsed_ms;
    uint8_t  phase;              // 0: ZUPT, 1: CCW (10-turn), 2: Pause, 3: CW (10-turn), 4: Done
    float    target_rpm_l;
    float    target_rpm_r;
    float    meas_rpm_l;
    float    meas_rpm_r;
    float    pwm_duty_l;
    float    pwm_duty_r;
    int32_t  steps_l;
    int32_t  steps_r;
    float    vbus_v;
    float    current_ma;
    float    gyro_z_dps;
    float    integrated_yaw_deg;
    uint8_t  enc_l_valid;
    uint8_t  enc_r_valid;
    uint8_t  ina_valid;
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
    BidirectionalSpinRecord rec;
    uint32_t recordsLogged = 0;

    while (true) {
        if (xQueueReceive(sdLogQueue, &rec, portMAX_DELAY) == pdTRUE) {
            if (sdCardReady && logFile) {
                logFile.printf("%llu,%lu,%u,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%ld,%ld,%.2f,%.1f,%.3f,%.3f,%u,%u,%u\n",
                               (unsigned long long)rec.timestamp_us,
                               (unsigned long)rec.elapsed_ms,
                               rec.phase,
                               rec.target_rpm_l, rec.target_rpm_r,
                               rec.meas_rpm_l, rec.meas_rpm_r,
                               rec.pwm_duty_l, rec.pwm_duty_r,
                               (long)rec.steps_l, (long)rec.steps_r,
                               rec.vbus_v, rec.current_ma,
                               rec.gyro_z_dps, rec.integrated_yaw_deg,
                               rec.enc_l_valid, rec.enc_r_valid, rec.ina_valid);

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

    SPI_SD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);
    delay(50);

    if (SD.begin(PIN_SD_CS, SPI_SD, 10000000)) {
        sdCardReady = true;
        char filename[32];
        snprintf(filename, sizeof(filename), "/spin_bi_10turn_r%d.csv", Config::ID);
        logFile = SD.open(filename, FILE_WRITE);
        if (logFile) {
            logFile.println("timestamp_us,elapsed_ms,phase,target_rpm_l,target_rpm_r,meas_rpm_l,meas_rpm_r,pwm_duty_l,pwm_duty_r,steps_l,steps_r,vbus_v,current_ma,gyro_z_dps,yaw_deg,enc_l_valid,enc_r_valid,ina_valid");
            logFile.flush();
        }
    }

    sdLogQueue = xQueueCreate(256, sizeof(BidirectionalSpinRecord));
    xTaskCreatePinnedToCore(core0LoggerTask, "SDLogger", 4096, nullptr, 1, &core0TaskHandle, 0);

    Serial.println("======================================================================");
    Serial.printf("   ANJOMAN 10-TURN BIDIRECTIONAL SPIN (CCW + CW) BENCHMARK (ROBOT %d)\n", Config::ID);
    Serial.printf("   Target: %d ticks/direction (3600 deg each) @ %.1f RPM\n", TARGET_10TURN_TICKS, SPIN_CRUISE_RPM);
    Serial.printf("   MicroSD File: /spin_bi_10turn_r%d.csv | Status: %s\n", Config::ID, sdCardReady ? "READY" : "FAILED");
    Serial.println("======================================================================");
}

void loop() {
    static uint32_t startTestTimeMs = millis();
    static uint64_t nextSampleUs = micros();
    static bool testComplete = false;

    // Phase: 0 = ZUPT (0-3s), 1 = CCW 10 Turns, 2 = Dwell Pause (3s), 3 = CW 10 Turns, 4 = Complete
    static uint8_t testPhase = 0;
    static uint32_t phaseStartTimeMs = 0;

    static int32_t baseStepsL = 0;
    static int32_t baseStepsR = 0;

    static float gyroBiasZ = Config::GYRO_BIAS_Z_DPS;
    static float gyroBiasSum = 0.0f;
    static uint32_t biasSampleCount = 0;
    static float integratedYawDeg = 0.0f;

    if (testComplete) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        return;
    }

    uint64_t nowUs = micros();
    uint32_t elapsedMs = millis() - startTestTimeMs;

    if (nowUs >= nextSampleUs) {
        nextSampleUs = nowUs + SAMPLE_PERIOD_US;

        bool encLValid = encL.update(CONTROL_PERIOD_S);
        bool encRValid = encR.update(CONTROL_PERIOD_S);

        imu.readSensorData();
        float gz_raw = imu.getGyroZ();

        float vbus_V = 7.40f;
        float current_mA = 0.0f;
        bool inaValid = readINA226(vbus_V, current_mA);
        if (!inaValid || vbus_V < 6.0f) vbus_V = 7.40f;

        int32_t stepsL = encL.getCumulativeSteps();
        int32_t stepsR = encR.getCumulativeSteps();

        float targetRpmL = 0.0f;
        float targetRpmR = 0.0f;
        float dutyL = 0.0f;
        float dutyR = 0.0f;

        // Phase 0: Stationary Pre-Spin ZUPT (0 to 3000 ms)
        if (testPhase == 0) {
            motorL.brake();
            motorR.brake();

            gyroBiasSum += gz_raw;
            biasSampleCount++;

            if (elapsedMs >= 3000) {
                if (biasSampleCount > 100) {
                    float measuredBias = gyroBiasSum / (float)biasSampleCount;
                    if (fabs(measuredBias - Config::GYRO_BIAS_Z_DPS) < 0.35f) {
                        gyroBiasZ = measuredBias;
                    }
                }
                testPhase = 1;
                phaseStartTimeMs = millis();
                baseStepsL = stepsL;
                baseStepsR = stepsR;
                rgbLedWrite(PIN_STATUS_RGB, 0, 0, 50); // Blue (CCW)
            }
        }
        // Phase 1: 10 Full Turns CCW (Left REV, Right FWD)
        else if (testPhase == 1) {
            targetRpmL = -SPIN_CRUISE_RPM;
            targetRpmR = +SPIN_CRUISE_RPM;

            int32_t dStepsL = stepsL - baseStepsL;
            int32_t dStepsR = stepsR - baseStepsR;
            int32_t avgSpinTicks = (abs(dStepsL) + abs(dStepsR)) / 2;

            // Tick sync balance for CCW: dStepsR + dStepsL == 0
            int32_t syncError = dStepsR + dStepsL;
            float syncCorrection = constrain((float)syncError * 0.00015f, -0.08f, 0.08f);

            dutyL = motorL.computeVelocityControl(targetRpmL, encL.getRPM(), vbus_V, CONTROL_PERIOD_S, -syncCorrection);
            dutyR = motorR.computeVelocityControl(targetRpmR, encR.getRPM(), vbus_V, CONTROL_PERIOD_S, -syncCorrection);

            float correctedGz = gz_raw - gyroBiasZ;
            integratedYawDeg += correctedGz * CONTROL_PERIOD_S;

            if (avgSpinTicks >= TARGET_10TURN_TICKS || (millis() - phaseStartTimeMs) >= 55000) {
                testPhase = 2;
                phaseStartTimeMs = millis();
                motorL.brake();
                motorR.brake();
                rgbLedWrite(PIN_STATUS_RGB, 30, 20, 0); // Yellow (Pause)
            }
        }
        // Phase 2: Dwell Pause (3 seconds)
        else if (testPhase == 2) {
            motorL.brake();
            motorR.brake();

            float correctedGz = gz_raw - gyroBiasZ;
            integratedYawDeg += correctedGz * CONTROL_PERIOD_S;

            if ((millis() - phaseStartTimeMs) >= 3000) {
                testPhase = 3;
                phaseStartTimeMs = millis();
                baseStepsL = stepsL;
                baseStepsR = stepsR;
                rgbLedWrite(PIN_STATUS_RGB, 50, 0, 50); // Purple (CW)
            }
        }
        // Phase 3: 10 Full Turns CW (Left FWD, Right REV)
        else if (testPhase == 3) {
            targetRpmL = +SPIN_CRUISE_RPM;
            targetRpmR = -SPIN_CRUISE_RPM;

            int32_t dStepsL = stepsL - baseStepsL;
            int32_t dStepsR = stepsR - baseStepsR;
            int32_t avgSpinTicks = (abs(dStepsL) + abs(dStepsR)) / 2;

            // Tick sync balance for CW: dStepsL + dStepsR == 0
            int32_t syncError = dStepsL + dStepsR;
            float syncCorrection = constrain((float)syncError * 0.00015f, -0.08f, 0.08f);

            dutyL = motorL.computeVelocityControl(targetRpmL, encL.getRPM(), vbus_V, CONTROL_PERIOD_S, -syncCorrection);
            dutyR = motorR.computeVelocityControl(targetRpmR, encR.getRPM(), vbus_V, CONTROL_PERIOD_S, -syncCorrection);

            float correctedGz = gz_raw - gyroBiasZ;
            integratedYawDeg += correctedGz * CONTROL_PERIOD_S;

            if (avgSpinTicks >= TARGET_10TURN_TICKS || (millis() - phaseStartTimeMs) >= 55000) {
                testPhase = 4;
                testComplete = true;

                motorL.brake();
                motorR.brake();

                vTaskDelay(pdMS_TO_TICKS(150));
                if (logFile) {
                    logFile.flush();
                    logFile.close();
                }

                rgbLedWrite(PIN_STATUS_RGB, 0, 60, 0); // Solid Green (Done)

                Serial.println("\n======================================================================");
                Serial.printf("[TEST FINISHED] 10-Turn Bidirectional Spin Completed for Robot %d\n", Config::ID);
                Serial.printf("   Net Integrated Gyro Yaw: %.2f deg (Nominal Return: 0.00 deg)\n", integratedYawDeg);
                Serial.printf("   Final Measured INA226 Voltage: %.2f V\n", vbus_V);
                Serial.println("======================================================================");
            }
        }

        BidirectionalSpinRecord rec = {};
        rec.timestamp_us = nowUs;
        rec.elapsed_ms = elapsedMs;
        rec.phase = testPhase;
        rec.target_rpm_l = targetRpmL;
        rec.target_rpm_r = targetRpmR;
        rec.meas_rpm_l = encL.getRPM();
        rec.meas_rpm_r = encR.getRPM();
        rec.pwm_duty_l = dutyL;
        rec.pwm_duty_r = dutyR;
        rec.steps_l = stepsL;
        rec.steps_r = stepsR;
        rec.vbus_v = vbus_V;
        rec.current_ma = current_mA;
        rec.gyro_z_dps = gz_raw;
        rec.integrated_yaw_deg = integratedYawDeg;
        rec.enc_l_valid = encLValid ? 1 : 0;
        rec.enc_r_valid = encRValid ? 1 : 0;
        rec.ina_valid = inaValid ? 1 : 0;

        if (sdLogQueue != nullptr) {
            xQueueSend(sdLogQueue, &rec, 0);
        }
    }
    yield();
}
