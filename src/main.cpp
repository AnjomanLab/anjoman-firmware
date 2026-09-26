#include <Arduino.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>

// ---- Anjoman modules ----
#include "PinMap.h"
#include "RobotConfig.h"
#include "Types.h"
#include "AnjomanI2C.h"
#include "MagneticEncoder.h"
#include "MotorController.h"
#include "BMI160_Custom.h"
#include "INA226.h"
#include "HeadingKalmanFilter.h"
#include "UWBPreprocessor.h"
#include "FormationReference.h"
#include "FormationController.h"
#include "ManeuverLogger.h"
#include "TDMAEngine.h"

// ==============================================================================
// 1. TIME / PHYSICS CONSTANTS
// ==============================================================================
constexpr double SPEED_OF_LIGHT = 299792458.0;
constexpr double TIME_UNIT_SEC  = 0.000000000015650040064103;
constexpr uint64_t SCHEDULED_REPLY_DELAY = 159744000ULL;

constexpr double FREQ_OFFSET_MULTIPLIER         = 998.4e6 / (2.0 * 1024.0 * 131072.0);
constexpr double HERTZ_TO_PPM_MULTIPLIER_CHAN_5 = -1.0e6 / 6489.6e6;

uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

device_configuration_t UWB_CONFIG = {
    false, true, true, true, false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

// ==============================================================================
// 2. HARDWARE SINGLETONS
// ==============================================================================
MotorController motorL(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, Config::INVERT_MOTOR_LEFT);
MotorController motorR(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, Config::INVERT_MOTOR_RIGHT);

MagneticEncoder encL(Wire, 0x70, 0, Config::INVERT_ENCODER_LEFT);
MagneticEncoder encR(Wire, 0x70, 1, Config::INVERT_ENCODER_RIGHT);

BMI160_Custom   imu(Wire, 0x69, 2);
INA226          power_monitor(Wire, 0x40, 3, Config::SHUNT_RESISTOR_OHM);

HeadingKalmanFilter g_headingFilter;
ManeuverLogger      logger;
TDMAEngine          tdma;

// ==============================================================================
// 3. SHARED STATE BETWEEN CORES
// ==============================================================================
struct SharedState {
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    float posX;
    float posY;
    float headingRad;
    float vCommand;
    float omegaCommand;
    float rpmL;
    float rpmR;
    float gyroZ;
    bool  maneuverRunning;
    bool  maneuverFinished;
    uint32_t maneuverStartMs;
} g_shared;

struct UWBMetrics {
    float    rawDist;
    float    cleanDist;
    float    rssi;
    float    fpPower;
    uint16_t stdNoise;
    uint8_t  peerId;
    uint8_t  ldeErr;
};
UWBMetrics g_uwbMetrics;

// Temperature cached for UWB preprocessor
float g_cachedTempUwb = 25.0f;

// Latest sync packet's maneuver info (slave side)
volatile uint8_t  g_remoteManeuverActive = 0;
volatile uint32_t g_remoteManeuverStartMs = 0;

// ==============================================================================
// 4. UTILITIES
// ==============================================================================
inline void write40BitTime(uint8_t *dest, uint64_t val) {
    dest[0] = (uint8_t)(val & 0xFF);
    dest[1] = (uint8_t)((val >> 8) & 0xFF);
    dest[2] = (uint8_t)((val >> 16) & 0xFF);
    dest[3] = (uint8_t)((val >> 24) & 0xFF);
    dest[4] = (uint8_t)((val >> 32) & 0xFF);
}

inline uint64_t read40BitTime(const uint8_t *src) {
    return ((uint64_t)src[0]) |
           (((uint64_t)src[1]) << 8) |
           (((uint64_t)src[2]) << 16) |
           (((uint64_t)src[3]) << 24) |
           (((uint64_t)src[4]) << 32);
}

// ==============================================================================
// 5. CORE 1 — 100 Hz HARD REAL-TIME LOOP (motor + odometry + heading)
// ==============================================================================
void Core1_ControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(Config::CONTROL_PERIOD_MS);

    float initRx, initRy;
    FormationReference::getUnitOffset(Config::ID, initRx, initRy);
    float odomX = FormationReference::L_INITIAL * initRx;
    float odomY = FormationReference::L_INITIAL * initRy;
    float odomTheta = 0.0f;

    g_headingFilter.init(0.0f, Config::GYRO_BIAS_Z_RAD_S,
                         Config::Q_YAW_DISCRETE,
                         Config::Q_GYRO_BIAS_WALK,
                         Config::R_YAW_ENCODER);

    constexpr float RAD_S_TO_RPM = 60.0f / (2.0f * 3.1415926535f);

    while (true) {
        const float dt = Config::CONTROL_PERIOD_S;

        encL.update(dt);
        encR.update(dt);
        imu.readSensorData();

        const float measRpmL = encL.getRPM();
        const float measRpmR = encR.getRPM();
        const float measRadL = encL.getRadPerSec();
        const float measRadR = encR.getRadPerSec();

        const float gyroZRadS = imu.getGyroZ() * 0.01745329251f;

        // Heading filter
        g_headingFilter.predict(gyroZRadS, dt);
        const float deltaThetaEnc =
            (measRadR - measRadL) * (Config::WHEEL_RADIUS_M / Config::TRACK_WIDTH_M) * dt;
        g_headingFilter.updateEncoder(deltaThetaEnc, gyroZRadS, dt);
        odomTheta = g_headingFilter.getHeadingRad();

        // Position (dead-reckoning)
        const float vActual = 0.5f * (measRadL + measRadR) * Config::WHEEL_RADIUS_M;
        odomX += vActual * cosf(odomTheta) * dt;
        odomY += vActual * sinf(odomTheta) * dt;

        // Publish state
        portENTER_CRITICAL(&g_shared.mux);
        g_shared.posX       = odomX;
        g_shared.posY       = odomY;
        g_shared.headingRad = odomTheta;
        g_shared.rpmL       = measRpmL;
        g_shared.rpmR       = measRpmR;
        g_shared.gyroZ      = gyroZRadS;
        const float targetV     = g_shared.vCommand;
        const float targetOmega = g_shared.omegaCommand;
        const bool  isActive    = g_shared.maneuverRunning;
        const bool  isFinished  = g_shared.maneuverFinished;
        portEXIT_CRITICAL(&g_shared.mux);

        // Motor control
        if (!isActive || isFinished) {
            motorL.brake();
            motorR.brake();
        } else {
            const float vTargetL = targetV - 0.5f * Config::TRACK_WIDTH_M * targetOmega;
            const float vTargetR = targetV + 0.5f * Config::TRACK_WIDTH_M * targetOmega;

            const float targetRpmL = (vTargetL / Config::WHEEL_RADIUS_M) * RAD_S_TO_RPM;
            const float targetRpmR = (vTargetR / Config::WHEEL_RADIUS_M) * RAD_S_TO_RPM;

            motorL.computeVelocityControl(targetRpmL, measRpmL, 7.4f, dt);
            motorR.computeVelocityControl(targetRpmR, measRpmR, 7.4f, dt);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==============================================================================
// 6. ESP-NOW RECEIVER
// ==============================================================================
void onDataRecv(const uint8_t *mac, const uint8_t *data, int data_len) {
    if (data_len != sizeof(SyncBeaconPacket)) return;

    SyncBeaconPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (tdma.isMaster()) return;

    tdma.onSyncReceived(pkt.frameId, pkt.timestampMs, micros());

    if (pkt.maneuverActive && !g_shared.maneuverRunning) {
        portENTER_CRITICAL(&g_shared.mux);
        g_shared.maneuverRunning = true;
        g_shared.maneuverStartMs = pkt.maneuverStartMs;
        portEXIT_CRITICAL(&g_shared.mux);
    }
}


void setupESPNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERROR] ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BROADCAST_MAC, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    peerInfo.ifidx   = WIFI_IF_STA;
    esp_now_add_peer(&peerInfo);
}

// ==============================================================================
// 7. UWB RANGING
// ==============================================================================
void setupUWB() {
    pinMode(PIN_UWB_RST, OUTPUT);
    digitalWrite(PIN_UWB_RST, LOW);
    delay(10);
    pinMode(PIN_UWB_RST, INPUT);
    delay(25);

    SPI.begin(PIN_UWB_SCK, PIN_UWB_MISO, PIN_UWB_MOSI, PIN_UWB_CS);
    DW1000Ng::initializeNoInterrupt(PIN_UWB_CS, PIN_UWB_RST);
    DW1000Ng::applyConfiguration(UWB_CONFIG);
    DW1000Ng::setDeviceAddress(Config::ID);
    DW1000Ng::setNetworkId(0xDECA);
    DW1000Ng::setAntennaDelay(16436);
    DW1000Ng::clearReceiveStatus();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
}

bool performRangingPoll(uint8_t targetPeerId) {
    UWBPollPacket pollPkt = {};
    memcpy(pollPkt.header, "POLL", 4);
    pollPkt.initiatorId = Config::ID;
    pollPkt.targetId    = targetPeerId;
    pollPkt.sequence    = tdma.getFrameId();

    DW1000Ng::forceTRxOff();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::clearReceiveStatus();
    DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&pollPkt), sizeof(pollPkt));
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    uint32_t txStart = millis();
    while (!DW1000Ng::isTransmitDone()) {
        if (millis() - txStart > 6) return false;
        yield();
    }
    DW1000Ng::clearTransmitStatus();
    const uint64_t tTx1 = DW1000Ng::getTransmitTimestamp();

    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
    uint32_t waitRx = millis();
    bool success = false;

    while (millis() - waitRx < 8) {
        if (DW1000Ng::isReceiveDone()) {
            DW1000Ng::clearReceiveStatus();
            const size_t len = DW1000Ng::getReceivedDataLength();

            if (len >= sizeof(UWBResponsePacket)) {
                UWBResponsePacket respPkt;
                DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&respPkt), sizeof(respPkt));

                if (memcmp(respPkt.header, "RESP", 4) == 0 &&
                    respPkt.responderId == targetPeerId &&
                    respPkt.targetId == Config::ID) {

                    const uint64_t tRx1 = DW1000Ng::getReceiveTimestamp();
                    const uint64_t tRx2 = read40BitTime(respPkt.rxTimestamp);
                    const uint64_t tTx2 = read40BitTime(respPkt.txTimestamp);

                    const int64_t tRound = (int64_t)((tRx1 - tTx1) & 0xFFFFFFFFFFULL);
                    const int64_t tReply = (int64_t)((tTx2 - tRx2) & 0xFFFFFFFFFFULL);
                    const int64_t tofRawTicks = (tRound - tReply) / 2;
                    const float distRawM = (float)(tofRawTicks * TIME_UNIT_SEC * SPEED_OF_LIGHT);

                    // Apply deterministic bias + thermal correction
                    const float cleanM = UWBPreprocessor::correctRawDistance(
                        Config::ID, targetPeerId, distRawM,
                        g_cachedTempUwb, respPkt.tempUwb);

                    const auto diag = DW1000Ng::getChannelDiagnostics();

                    g_uwbMetrics.rawDist   = distRawM;
                    g_uwbMetrics.cleanDist = cleanM;
                    g_uwbMetrics.rssi      = (float)DW1000Ng::getReceivePower();
                    g_uwbMetrics.fpPower   = (float)DW1000Ng::getFirstPathPower();
                    g_uwbMetrics.stdNoise  = diag.stdNoise;
                    g_uwbMetrics.peerId    = targetPeerId;
                    g_uwbMetrics.ldeErr    = diag.ldeError;
                    success = true;
                }
            }
            break;
        }
        yield();
    }
    DW1000Ng::forceTRxOff();
    return success;
}

void performRangingListenSafe() {
    static uint32_t rxArmedTime = 0;

    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);

    if (DW1000Ng::isReceiveFailed() || (millis() - rxArmedTime > 15)) {
        DW1000Ng::forceTRxOff();
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
        rxArmedTime = millis();
        return;
    }

    if (DW1000Ng::isReceiveDone()) {
        DW1000Ng::clearReceiveStatus();
        const size_t len = DW1000Ng::getReceivedDataLength();

        if (len >= sizeof(UWBPollPacket)) {
            UWBPollPacket pollPkt;
            DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&pollPkt), sizeof(pollPkt));

            if (memcmp(pollPkt.header, "POLL", 4) == 0 && pollPkt.targetId == Config::ID) {
                const uint64_t tRx2 = DW1000Ng::getReceiveTimestamp();
                const uint64_t tTx2 = (tRx2 + SCHEDULED_REPLY_DELAY) & 0xFFFFFFFE00ULL;

                UWBResponsePacket respPkt = {};
                memcpy(respPkt.header, "RESP", 4);
                respPkt.responderId = Config::ID;
                respPkt.targetId    = pollPkt.initiatorId;
                respPkt.sequence    = pollPkt.sequence;
                write40BitTime(respPkt.rxTimestamp, tRx2);
                write40BitTime(respPkt.txTimestamp, tTx2);
                respPkt.tempUwb     = g_cachedTempUwb;
                respPkt.tempEsp     = 25.0f;

                DW1000Ng::forceTRxOff();
                DW1000Ng::clearTransmitStatus();
                DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&respPkt), sizeof(respPkt));
                DW1000Ng::setDelayedTRX(respPkt.txTimestamp);
                DW1000Ng::startTransmit(TransmitMode::DELAYED);

                const uint32_t txWait = millis();
                while (!DW1000Ng::isTransmitDone()) {
                    if (millis() - txWait > 8) break;
                    yield();
                }
                DW1000Ng::clearTransmitStatus();
            }
        }
        DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
        rxArmedTime = millis();
    }
}

// ==============================================================================
// 8. SETUP
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1000);

    // ---- I2C bus (must be first) ----
    AnjomanI2C::init(PIN_I2C0_SDA, PIN_I2C0_SCL, 400000);

    // ---- Motors ----
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

    PIDGains gains = { 0.0050f, 0.035f, 0.40f };
    motorL.setPIDGains(gains);
    motorR.setPIDGains(gains);

    // ---- Sensors ----
    encL.begin();
    encR.begin();
    imu.begin();
    power_monitor.begin();

    // ---- Network & UWB ----
    setupESPNow();
    setupUWB();

    // ---- Logger ----
    logger.init();

    // ---- TDMA ----
    tdma.init(Config::ID, /*isMaster=*/ (Config::ID == 1));

    // ---- Status LED ----
    neopixelWrite(PIN_STATUS_RGB, 50, 40, 0);

    // ---- Core 1 control task ----
    xTaskCreatePinnedToCore(
        Core1_ControlTask,
        "Core1_Control",
        8192,
        NULL,
        3,
        NULL,
        1
    );
}

// ==============================================================================
// 9. CORE 0 LOOP
// ==============================================================================
void loop() {
    const uint32_t nowMs = millis();
    tdma.tick(nowMs, micros());

    // --------------------------------------------------------------------------
    // MASTER: broadcast sync beacon each frame
    // --------------------------------------------------------------------------
    if (tdma.shouldBroadcastSync()) {
        static bool autoTriggerArmed = false;
        static uint32_t mStartMs = 0;

        if (!autoTriggerArmed && nowMs >= TDMAConfig::AUTO_START_DELAY_MS) {
            autoTriggerArmed = true;
            mStartMs = nowMs + 1000;

            portENTER_CRITICAL(&g_shared.mux);
            g_shared.maneuverRunning = true;
            g_shared.maneuverStartMs = mStartMs;
            portEXIT_CRITICAL(&g_shared.mux);
        }

        SyncBeaconPacket sync = {};
        sync.frameId          = tdma.getFrameId();
        sync.timestampMs      = nowMs;
        sync.maneuverActive   = (uint8_t)(autoTriggerArmed ? 1 : 0);
        sync.maneuverStartMs  = mStartMs;
        esp_now_send(BROADCAST_MAC, (uint8_t*)&sync, sizeof(sync));
    }

    // --------------------------------------------------------------------------
    // 10 Hz FORMATION CONTROLLER
    // --------------------------------------------------------------------------
    static uint32_t lastCtrlMs = 0;
    if (nowMs - lastCtrlMs >= 100) {
        lastCtrlMs = nowMs;

        portENTER_CRITICAL(&g_shared.mux);
        const bool  isRunning = g_shared.maneuverRunning;
        const bool  isDone    = g_shared.maneuverFinished;
        const uint32_t startMs = g_shared.maneuverStartMs;
        const float curX  = g_shared.posX;
        const float curY  = g_shared.posY;
        const float curTh = g_shared.headingRad;
        portEXIT_CRITICAL(&g_shared.mux);

        if (isRunning && !isDone && nowMs >= startMs) {
            const float tElapsedSec = (float)(nowMs - startMs) / 1000.0f;

            neopixelWrite(PIN_STATUS_RGB, 0, 50, 0);

            if (tElapsedSec >= FormationReference::T_TOTAL_SEC) {
                portENTER_CRITICAL(&g_shared.mux);
                g_shared.maneuverFinished = true;
                g_shared.vCommand = 0.0f;
                g_shared.omegaCommand = 0.0f;
                portEXIT_CRITICAL(&g_shared.mux);

                logger.commitToFlash(1);
                logger.startDownloadServer();
                neopixelWrite(PIN_STATUS_RGB, 0, 0, 50);
            } else {
                const FormationState2D target =
                    FormationReference::evaluate(Config::ID, tElapsedSec);

                const WheelVelocityCommand cmd =
                    FormationController::compute(Config::ID, target,
                                                 curX, curY, curTh);

                portENTER_CRITICAL(&g_shared.mux);
                g_shared.vCommand     = cmd.vLinear;
                g_shared.omegaCommand = cmd.omegaRadS;
                portEXIT_CRITICAL(&g_shared.mux);
            }
        }
    }

    if (g_shared.maneuverFinished) {
        logger.handleClient();
    }

    // --------------------------------------------------------------------------
    // 5 Hz RAM LOGGING
    // --------------------------------------------------------------------------
    static uint32_t lastLogMs = 0;
    if (nowMs - lastLogMs >= Config::TELEMETRY_PERIOD_MS) {
        lastLogMs = nowMs;

        if (g_shared.maneuverRunning && !g_shared.maneuverFinished) {
            portENTER_CRITICAL(&g_shared.mux);
            const float curX  = g_shared.posX;
            const float curY  = g_shared.posY;
            const float curTh = g_shared.headingRad;
            const float rL    = g_shared.rpmL;
            const float rR    = g_shared.rpmR;
            const float gZ    = g_shared.gyroZ;
            const float vCmd  = g_shared.vCommand;
            const float wCmd  = g_shared.omegaCommand;
            portEXIT_CRITICAL(&g_shared.mux);

            float vBat = 7.4f, iBat = 0.0f, pBat = 0.0f;
            power_monitor.read(vBat, iBat, pBat);

            // Refresh UWB temperature every ~500 ms
            static uint32_t lastTempMs = 0;
            if (nowMs - lastTempMs >= 500) {
                lastTempMs = nowMs;
                const float t = DW1000Ng::getTemperature();
                if (t > -30.0f && t < 90.0f) {
                    g_cachedTempUwb = t;
                }
            }

            LogRecord rec = {};
            rec.t_ms          = nowMs;
            rec.x             = curX;
            rec.y             = curY;
            rec.heading       = curTh;
            rec.v_cmd         = vCmd;
            rec.omega_cmd     = wCmd;
            rec.rpm_l         = rL;
            rec.rpm_r         = rR;
            rec.gyro_z        = gZ;
            rec.vbat          = vBat;
            rec.current_a     = iBat;
            rec.uwb_raw       = g_uwbMetrics.rawDist;
            rec.uwb_clean     = g_uwbMetrics.cleanDist;
            rec.uwb_rssi      = g_uwbMetrics.rssi;
            rec.uwb_fp_power  = g_uwbMetrics.fpPower;
            rec.uwb_std_noise = g_uwbMetrics.stdNoise;
            rec.uwb_peer_id   = g_uwbMetrics.peerId;
            rec.uwb_lde_err   = g_uwbMetrics.ldeErr;
            rec.uwb_temp      = g_cachedTempUwb;
            logger.record(rec);
        }
    }

    // --------------------------------------------------------------------------
    // TDMA SLOT EXECUTION
    // --------------------------------------------------------------------------
    const uint32_t slot = tdma.getCurrentSlotIndex();

    switch (slot) {
        // ---- R1 slots ----
        case 1: if (Config::ID == 1) { performRangingPoll(2); } else performRangingListenSafe(); break;
        case 2: if (Config::ID == 1) { performRangingPoll(3); } else performRangingListenSafe(); break;
        case 3: if (Config::ID == 1) { performRangingPoll(4); } else performRangingListenSafe(); break;

        // ---- R2 slots ----
        case 4: if (Config::ID == 2) { performRangingPoll(1); } else performRangingListenSafe(); break;

        // ---- R3 slots ----
        case 7: if (Config::ID == 3) { performRangingPoll(1); } else performRangingListenSafe(); break;

        // ---- R4 slots ----
        case 10: if (Config::ID == 4) { performRangingPoll(1); } else performRangingListenSafe(); break;

        default: performRangingListenSafe(); break;
    }

    yield();
}
