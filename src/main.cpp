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
// 1. HARDWARE DEFINITIONS & BUS CONFIGURATION
// ==============================================================================
constexpr uint8_t  TCA9548A_ADDR       = 0x70;
constexpr uint8_t  INA226_ADDR         = 0x40;
constexpr uint32_t I2C_CLOCK_FREQ_HZ  = 400000;
constexpr uint32_t CONTROL_RATE_HZ     = 100;
constexpr uint32_t SAMPLE_PERIOD_US   = 1000000 / CONTROL_RATE_HZ; // 10000 us (10 ms)
constexpr float    CONTROL_PERIOD_S    = 0.010f;

// 2.000 meters calibrated target ticks: (2.000 / (2 * PI * 0.02685)) * 4096 = 48556 ticks
constexpr int32_t TARGET_DISTANCE_TICKS = 48556;
constexpr float    TARGET_CRUISE_RPM     = (Config::SWARM_CRUISE_VEL_M_S / (Config::WHEEL_RADIUS_M * 2.0f * PI)) * 60.0f; // 57.296 RPM
constexpr float    SHUNT_RESISTOR_OHM    = Config::SHUNT_RESISTOR_OHM;

// Hardware Interfaces
SPIClass SPI_SD(HSPI);
File logFile;
bool sdCardReady = false;
bool ina226Ready = false;

// Custom Modular Drivers
MagneticEncoder encL(Wire, TCA9548A_ADDR, 0, Config::INVERT_ENCODER_LEFT);
MagneticEncoder encR(Wire, TCA9548A_ADDR, 1, Config::INVERT_ENCODER_RIGHT);
MotorController motorL(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, Config::INVERT_MOTOR_LEFT);
MotorController motorR(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, Config::INVERT_MOTOR_RIGHT);
BMI160_Custom   imu(Wire, 0x69, 2);

// ==============================================================================
// 2. TELEMETRY LOG RECORD (100 Hz Async SD Logging)
// ==============================================================================
#pragma pack(push, 1)
struct VelocityBenchmarkRecord {
    uint64_t timestamp_us;
    uint32_t elapsed_ms;
    uint8_t  state;
    float    cmd_vel_mps;
    float    target_rpm_l;
    float    target_rpm_r;
    float    meas_rpm_l;
    float    meas_rpm_r;
    float    pwm_duty_l;
    float    pwm_duty_r;
    int32_t  steps_l;
    int32_t  steps_r;
    float    distance_m_l;
    float    distance_m_r;
    float    vbus_v;
    float    current_ma;
    float    gyro_z_dps;
    float    estimated_yaw_deg;
    uint8_t  enc_l_valid;
    uint8_t  enc_r_valid;
    uint8_t  ina_valid;
};
#pragma pack(pop)

QueueHandle_t sdLogQueue = nullptr;
TaskHandle_t  core0TaskHandle = nullptr;

// ==============================================================================
// 3. HARDWARE I2C INA226 POWER DRIVER
// ==============================================================================
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

    // Reset INA226
    Wire.beginTransmission(INA226_ADDR);
    Wire.write(0x00); Wire.write(0x80); Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    // Continuous Shunt and Bus, 1.1ms conversion, 4 averages (0x4227)
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

// ==============================================================================
// 4. CORE 0 ASYNCHRONOUS LOGGER TASK
// ==============================================================================
void core0LoggerTask(void *pvParameters) {
    VelocityBenchmarkRecord rec;
    uint32_t recordsLogged = 0;

    while (true) {
        if (xQueueReceive(sdLogQueue, &rec, portMAX_DELAY) == pdTRUE) {
            if (sdCardReady && logFile) {
                logFile.printf("%llu,%lu,%u,%.3f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%ld,%ld,%.4f,%.4f,%.2f,%.1f,%.3f,%.3f,%u,%u,%u\n",
                               (unsigned long long)rec.timestamp_us,
                               (unsigned long)rec.elapsed_ms,
                               rec.state,
                               rec.cmd_vel_mps,
                               rec.target_rpm_l, rec.target_rpm_r,
                               rec.meas_rpm_l, rec.meas_rpm_r,
                               rec.pwm_duty_l, rec.pwm_duty_r,
                               (long)rec.steps_l, (long)rec.steps_r,
                               rec.distance_m_l, rec.distance_m_r,
                               rec.vbus_v, rec.current_ma,
                               rec.gyro_z_dps, rec.estimated_yaw_deg,
                               rec.enc_l_valid, rec.enc_r_valid, rec.ina_valid);

                recordsLogged++;
                if (recordsLogged % 50 == 0) {
                    logFile.flush();
                }
            }
        }
    }
}

// ==============================================================================
// 5. SETUP & INITIALIZATION
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1000);

    rgbLedWrite(PIN_STATUS_RGB, 20, 10, 0); // Dim Amber (Initializing)

    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, I2C_CLOCK_FREQ_HZ);
    delay(50);

    // Initialize Modular Encoders
    encL.begin();
    encR.begin();

    // Initialize Motor Drivers and Load SysID Calibration
    motorL.begin(20000, 10);
    motorR.begin(20000, 10);

    MotorSysIDParams paramsL = {
        Config::DEADBAND_FWD_L,
        Config::DEADBAND_REV_L,
        Config::GAIN_RPM_FWD_L,
        Config::GAIN_RPM_REV_L,
        7.40f
    };
    MotorSysIDParams paramsR = {
        Config::DEADBAND_FWD_R,
        Config::DEADBAND_REV_R,
        Config::GAIN_RPM_FWD_R,
        Config::GAIN_RPM_REV_R,
        7.40f
    };
    motorL.setCalibration(paramsL);
    motorR.setCalibration(paramsR);

    // Conservative PI Gains to guarantee stability with Feedforward
    // Responsive PI Gains to eliminate steady-state RPM asymmetry
    PIDGains gainsL = {0.0030f, 0.0250f, 0.25f};
    PIDGains gainsR = {0.0030f, 0.0250f, 0.25f};
    motorL.setPIDGains(gainsL);
    motorR.setPIDGains(gainsR);

    // Initialize IMU & INA226
    imu.begin();
    ina226Ready = initINA226();

    // Initialize MicroSD on Dedicated SPI3
    SPI_SD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);
    delay(50);

    if (SD.begin(PIN_SD_CS, SPI_SD, 10000000)) {
        sdCardReady = true;
        char filename[32];
        snprintf(filename, sizeof(filename), "/straight_2m_r%d.csv", Config::ID);
        logFile = SD.open(filename, FILE_WRITE);
        if (logFile) {
            logFile.println("timestamp_us,elapsed_ms,state,cmd_vel_mps,target_rpm_l,target_rpm_r,meas_rpm_l,meas_rpm_r,pwm_duty_l,pwm_duty_r,steps_l,steps_r,dist_m_l,dist_m_r,vbus_v,current_ma,gyro_z_dps,yaw_deg,enc_l_valid,enc_r_valid,ina_valid");
            logFile.flush();
        }
    }

    // Start Core 0 SD Logger Task
    sdLogQueue = xQueueCreate(256, sizeof(VelocityBenchmarkRecord));
    xTaskCreatePinnedToCore(core0LoggerTask, "SDLogger", 4096, nullptr, 1, &core0TaskHandle, 0);

    Serial.println("======================================================================");
    Serial.printf("   ANJOMAN 2-METER STRAIGHT-LINE KINEMATICS & VELOCITY TEST (ROBOT %d)\n", Config::ID);
    Serial.printf("   Target: 2.000m (%d ticks) @ %.2f RPM (%.2f m/s)\n", TARGET_DISTANCE_TICKS, TARGET_CRUISE_RPM, Config::SWARM_CRUISE_VEL_M_S);
    Serial.printf("   MicroSD File: /straight_2m_r%d.csv | Status: %s\n", Config::ID, sdCardReady ? "READY" : "FAILED");
    Serial.println("======================================================================");
}

// ==============================================================================
// 6. 100 HZ HARD REAL-TIME CONTROL LOOP (CORE 1)
// ==============================================================================
void loop() {
    static uint32_t startTestTimeMs = millis();
    static uint64_t nextSampleUs = micros();
    static bool testComplete = false;

    // Test States: 0 = Stationary ZUPT (0-2.5s), 1 = Cruising (2.5s to 2.0m), 2 = Complete
    static uint8_t testState = 0;

    // In-run Gyroscope Alignment
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

        // 1. Read Motion Sensors
        bool encLValid = encL.update(CONTROL_PERIOD_S);
        bool encRValid = encR.update(CONTROL_PERIOD_S);

        imu.readSensorData();
        float gz_raw = imu.getGyroZ();

        float vbus_V = 7.40f;
        float current_mA = 0.0f;
        bool inaValid = readINA226(vbus_V, current_mA);
        if (!inaValid || vbus_V < 6.0f) {
            vbus_V = 7.40f; // Safe fallback
        }

        // 2. Kinematic Metrics from Encoders
        int32_t stepsL = encL.getCumulativeSteps();
        int32_t stepsR = encR.getCumulativeSteps();
        float distL = ((float)stepsL / Config::ENCODER_CPR) * (Config::WHEEL_RADIUS_M * 2.0f * PI);
        float distR = ((float)stepsR / Config::ENCODER_CPR) * (Config::WHEEL_RADIUS_M * 2.0f * PI);
        int32_t avgSteps = (stepsL + stepsR) / 2;

        float targetRpmL = 0.0f;
        float targetRpmR = 0.0f;
        float cmdVel = 0.0f;
        float dutyL = 0.0f;
        float dutyR = 0.0f;

        // 3. State Machine Sequence
        // Phase 0: Stationary Pre-Run ZUPT (0 to 2500 ms)
        if (testState == 0) {
            motorL.brake();
            motorR.brake();

            gyroBiasSum += gz_raw;
            biasSampleCount++;

            if (elapsedMs >= 2500) {
                if (biasSampleCount > 100) {
                    float measuredBias = gyroBiasSum / (float)biasSampleCount;
                    // Check sanity against RobotConfig
                    if (fabs(measuredBias - Config::GYRO_BIAS_Z_DPS) < 0.35f) {
                        gyroBiasZ = measuredBias;
                    }
                }
                testState = 1; // Transition to Active Cruise
                rgbLedWrite(PIN_STATUS_RGB, 0, 0, 50); // Solid Blue (Cruising)
            }
        }
        // Phase 1: 100 Hz Velocity Controlled Cruise (Target: 52152 ticks = 2.000m)
        else if (testState == 1) {
            cmdVel = Config::SWARM_CRUISE_VEL_M_S;
            targetRpmL = TARGET_CRUISE_RPM;
            targetRpmR = TARGET_CRUISE_RPM;

            // Cross-Coupled Tick Synchronization (forces straight trajectory)
            // If Left has more ticks, syncTerm is positive -> slow Left, speed up Right
            int32_t tickError = stepsL - stepsR;
            float syncCorrection = constrain((float)tickError * 0.00015f, -0.08f, 0.08f);

            dutyL = motorL.computeVelocityControl(targetRpmL, encL.getRPM(), vbus_V, CONTROL_PERIOD_S, -syncCorrection);
            dutyR = motorR.computeVelocityControl(targetRpmR, encR.getRPM(), vbus_V, CONTROL_PERIOD_S, +syncCorrection);

            // Integrate Heading Drift
            float correctedGz = gz_raw - gyroBiasZ;
            integratedYawDeg += correctedGz * CONTROL_PERIOD_S;

            // Termination Condition: 2.000 meters reached or 18 seconds timeout
            if (avgSteps >= TARGET_DISTANCE_TICKS || elapsedMs >= 18000) {
                testState = 2;
                testComplete = true;

                motorL.brake();
                motorR.brake();

                vTaskDelay(pdMS_TO_TICKS(150));
                if (logFile) {
                    logFile.flush();
                    logFile.close();
                }

                rgbLedWrite(PIN_STATUS_RGB, 0, 60, 0); // Solid Bright Green (Test Complete)

                Serial.println("\n======================================================================");
                Serial.printf("[TEST FINISHED] 2-Meter Benchmark Completed for Robot %d\n", Config::ID);
                Serial.printf("   Total Ticks L: %ld | Total Ticks R: %ld | Average Ticks: %ld\n", (long)stepsL, (long)stepsR, (long)avgSteps);
                Serial.printf("   Nominal Distance L: %.4f m | Nominal Distance R: %.4f m\n", distL, distR);
                Serial.printf("   Integrated Heading Deviation: %.2f deg\n", integratedYawDeg);
                Serial.printf("   Final Measured INA226 Voltage: %.2f V\n", vbus_V);
                Serial.println("   --> Measure actual ground distance with a tape measure to get r_eff!");
                Serial.println("======================================================================");
            }
        }

        // 4. Send Telemetry Record to Core 0 Async SD Queue
        VelocityBenchmarkRecord rec = {};
        rec.timestamp_us = nowUs;
        rec.elapsed_ms = elapsedMs;
        rec.state = testState;
        rec.cmd_vel_mps = cmdVel;
        rec.target_rpm_l = targetRpmL;
        rec.target_rpm_r = targetRpmR;
        rec.meas_rpm_l = encL.getRPM();
        rec.meas_rpm_r = encR.getRPM();
        rec.pwm_duty_l = dutyL;
        rec.pwm_duty_r = dutyR;
        rec.steps_l = stepsL;
        rec.steps_r = stepsR;
        rec.distance_m_l = distL;
        rec.distance_m_r = distR;
        rec.vbus_v = vbus_V;
        rec.current_ma = current_mA;
        rec.gyro_z_dps = gz_raw;
        rec.estimated_yaw_deg = integratedYawDeg;
        rec.enc_l_valid = encLValid ? 1 : 0;
        rec.enc_r_valid = encRValid ? 1 : 0;
        rec.ina_valid = inaValid ? 1 : 0;

        if (sdLogQueue != nullptr) {
            xQueueSend(sdLogQueue, &rec, 0);
        }
    }
    yield();
}
