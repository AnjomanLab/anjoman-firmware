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
#include "ESKF.h"
#include "UWBPreprocessor.h"
#include "FormationReference.h"
#include "FormationController.h"
#include "ManeuverLogger.h"
#include "TDMAEngine.h"

// ==============================================================================
// 1. CONSTANTS
// ==============================================================================
constexpr double SPEED_OF_LIGHT = 299792458.0;
constexpr double TIME_UNIT_SEC  = 0.000000000015650040064103;
constexpr uint64_t SCHEDULED_REPLY_DELAY = 159744000ULL;

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

ESKF            g_eskf;
ManeuverLogger  logger;
TDMAEngine      tdma;

// ==============================================================================
// 3. DECENTRALIZED TDMA SLOT TABLE
// ==============================================================================
struct SlotAction {
    uint8_t actor;      // 1..4 (transmitting robot), 0 = idle
    uint8_t target;     // 0 for beacon, else responder ID
    bool    isBeacon;
};

static const SlotAction SLOT_TABLE[16] = {
    {1, 0, true},  {1, 2, false}, {1, 3, false}, {1, 4, false},
    {2, 0, true},  {2, 1, false}, {2, 3, false}, {2, 4, false},
    {3, 0, true},  {3, 1, false}, {3, 2, false}, {3, 4, false},
    {4, 0, true},  {4, 1, false}, {4, 2, false}, {4, 3, false},
};

// ==============================================================================
// 4. SHARED STATE
// ==============================================================================
struct SharedState {
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

    float posX, posY, headingRad;
    float vCommand, omegaCommand;
    float rpmL, rpmR;
    float gyroZ;
    float imuTempC;
    float accelX;

    float eskfVarX, eskfVarY, eskfVarTheta, eskfVarBias;

    float dutyL, dutyR;
    float targetRpmL, targetRpmR;

    bool     maneuverRunning;
    bool     maneuverFinished;
    uint32_t maneuverStartMs;
    uint8_t  robotState;
} g_shared;

// ==============================================================================
// 5. UWB METRICS PER PEER
// ==============================================================================
struct UWBMetrics {
    bool     valid;
    uint8_t  peerId;
    uint8_t  ldeErr;
    uint16_t stdNoise;
    uint16_t fpAmpl1;
    uint16_t fpAmpl2;
    uint16_t cirPwr;
    uint16_t rxpacc;
    float    rawDist;
    float    cleanDist;
    float    rssi;
    float    fpPower;
    float    respTemp;
};
UWBMetrics g_uwbMetrics[5];

uint16_t g_lastRxpacc = 0;
float    g_cachedTempUwb = 25.0f;

// ==============================================================================
// 6. CONSENSUS STATE
// ==============================================================================
static uint8_t  g_peersSeenMask       = 0;
static uint32_t g_candidateStartFrame = 0;
static bool     g_maneuverTriggered   = false;

// ==============================================================================
// 7. UTILITIES
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
// 8. CORE 1 — 100 Hz real-time loop
// ==============================================================================
void Core1_ControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(Config::CONTROL_PERIOD_MS);

    float initRx, initRy;
    FormationReference::getUnitOffset(Config::ID, initRx, initRy);
    float odomX = FormationReference::L_INITIAL * initRx;
    float odomY = FormationReference::L_INITIAL * initRy;

    g_eskf.init(odomX, odomY, 0.0f, Config::GYRO_BIAS_Z_RAD_S);

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

        const float vActual = 0.5f * (measRadL + measRadR) * Config::WHEEL_RADIUS_M;
        const float deltaThetaEnc =
            (measRadR - measRadL) * (Config::WHEEL_RADIUS_M / Config::TRACK_WIDTH_M) * dt;

        g_eskf.predict(vActual, gyroZRadS, dt,
                       Config::Q_POS_ESKF,
                       Config::Q_THETA_ESKF,
                       Config::Q_BIAS_ESKF);
        g_eskf.updateEncoder(deltaThetaEnc, gyroZRadS, dt,
                             Config::R_THETA_ESKF);

        odomX = g_eskf.getX();
        odomY = g_eskf.getY();
        const float odomTheta = g_eskf.getTheta();

        portENTER_CRITICAL(&g_shared.mux);
        const float targetV     = g_shared.vCommand;
        const float targetOmega = g_shared.omegaCommand;
        const bool  isActive    = g_shared.maneuverRunning;
        const bool  isFinished  = g_shared.maneuverFinished;
        portEXIT_CRITICAL(&g_shared.mux);

        float dutyL = 0.0f, dutyR = 0.0f;
        float targetRpmL = 0.0f, targetRpmR = 0.0f;

        if (!isActive || isFinished) {
            motorL.brake();
            motorR.brake();
        } else {
            const float vTargetL = targetV - 0.5f * Config::TRACK_WIDTH_M * targetOmega;
            const float vTargetR = targetV + 0.5f * Config::TRACK_WIDTH_M * targetOmega;

            targetRpmL = (vTargetL / Config::WHEEL_RADIUS_M) * RAD_S_TO_RPM;
            targetRpmR = (vTargetR / Config::WHEEL_RADIUS_M) * RAD_S_TO_RPM;

            dutyL = motorL.computeVelocityControl(targetRpmL, measRpmL, 7.4f, dt);
            dutyR = motorR.computeVelocityControl(targetRpmR, measRpmR, 7.4f, dt);
        }

        portENTER_CRITICAL(&g_shared.mux);
        g_shared.posX          = odomX;
        g_shared.posY          = odomY;
        g_shared.headingRad    = odomTheta;
        g_shared.rpmL          = measRpmL;
        g_shared.rpmR          = measRpmR;
        g_shared.gyroZ         = gyroZRadS;
        g_shared.imuTempC      = imu.getTemperature();
        g_shared.accelX        = imu.getAccX();
        g_shared.eskfVarX      = g_eskf.getVarX();
        g_shared.eskfVarY      = g_eskf.getVarY();
        g_shared.eskfVarTheta  = g_eskf.getVarTheta();
        g_shared.eskfVarBias   = g_eskf.getVarBias();
        g_shared.dutyL         = dutyL;
        g_shared.dutyR         = dutyR;
        g_shared.targetRpmL    = targetRpmL;
        g_shared.targetRpmR    = targetRpmR;
        portEXIT_CRITICAL(&g_shared.mux);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==============================================================================
// 9. ESP-NOW
// ==============================================================================
void onDataRecv(const uint8_t *mac, const uint8_t *data, int data_len) {
    if (data_len != sizeof(SyncBeaconPacket)) return;

    SyncBeaconPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (pkt.senderId == Config::ID) return;

    tdma.onSyncReceived(pkt.senderId,
                        pkt.frameId,
                        pkt.senderUptimeMs,
                        pkt.senderSlot,
                        millis(),
                        micros());

    // Track peers and converge on maneuver start frame
    g_peersSeenMask |= (1u << pkt.senderId);
    if (pkt.maneuverStartFrame > g_candidateStartFrame) {
        g_candidateStartFrame = pkt.maneuverStartFrame;
    }
}

void setupESPNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) return;

    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BROADCAST_MAC, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    peerInfo.ifidx   = WIFI_IF_STA;
    esp_now_add_peer(&peerInfo);
}

// ==============================================================================
// 10. UWB
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
    if (targetPeerId < 1 || targetPeerId > 4 || targetPeerId == Config::ID) {
        return false;
    }

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

    const uint32_t txStart = millis();
    while (!DW1000Ng::isTransmitDone()) {
        if (millis() - txStart > 4) return false;
        yield();
    }
    DW1000Ng::clearTransmitStatus();
    const uint64_t tTx1 = DW1000Ng::getTransmitTimestamp();

    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
    const uint32_t waitRx = millis();
    bool success = false;

    while (millis() - waitRx < 8) {
        if (DW1000Ng::isReceiveDone()) {
            DW1000Ng::clearReceiveStatus();
            const size_t len = DW1000Ng::getReceivedDataLength();

            if (len >= sizeof(UWBResponsePacket)) {
                UWBResponsePacket respPkt;
                DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&respPkt),
                                          sizeof(respPkt));

                if (memcmp(respPkt.header, "RESP", 4) == 0 &&
                    respPkt.responderId == targetPeerId &&
                    respPkt.targetId == Config::ID) {

                    const uint64_t tRx1 = DW1000Ng::getReceiveTimestamp();
                    const uint64_t tRx2 = read40BitTime(respPkt.rxTimestamp);
                    const uint64_t tTx2 = read40BitTime(respPkt.txTimestamp);

                    const int64_t tRound = (int64_t)((tRx1 - tTx1) & 0xFFFFFFFFFFULL);
                    const int64_t tReply = (int64_t)((tTx2 - tRx2) & 0xFFFFFFFFFFULL);
                    const int64_t tofRawTicks = (tRound - tReply) / 2;
                    const float distRawM = (float)(tofRawTicks * TIME_UNIT_SEC *
                                                   SPEED_OF_LIGHT);

                    const float cleanM = UWBPreprocessor::correctRawDistance(
                        Config::ID, targetPeerId, distRawM,
                        g_cachedTempUwb, respPkt.tempUwb);

                    const auto diag = DW1000Ng::getChannelDiagnostics();

                    UWBMetrics &m = g_uwbMetrics[targetPeerId];
                    m.valid      = true;
                    m.peerId     = targetPeerId;
                    m.ldeErr     = diag.ldeError;
                    m.stdNoise   = diag.stdNoise;
                    m.fpAmpl1    = diag.fpAmpl1;
                    m.fpAmpl2    = diag.fpAmpl2;
                    m.cirPwr     = diag.cirPwr;
                    m.rxpacc     = diag.rxpacc;
                    m.rawDist    = distRawM;
                    m.cleanDist  = cleanM;
                    m.rssi       = (float)DW1000Ng::getReceivePower();
                    m.fpPower    = (float)DW1000Ng::getFirstPathPower();
                    m.respTemp   = respPkt.tempUwb;

                    g_lastRxpacc = diag.rxpacc;
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

// Blocking responder: listen for the entire slot window (up to 13 ms),
// so we are still receiving when the initiator's poll arrives mid-slot.
bool performRangingListenBlocking(uint32_t timeoutMs) {
    DW1000Ng::forceTRxOff();
    DW1000Ng::clearReceiveStatus();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);

    const uint32_t t_start = millis();
    bool success = false;

    while (millis() - t_start < timeoutMs) {
        if (DW1000Ng::isReceiveDone()) {
            DW1000Ng::clearReceiveStatus();
            const size_t len = DW1000Ng::getReceivedDataLength();

            if (len >= sizeof(UWBPollPacket)) {
                UWBPollPacket pollPkt;
                DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&pollPkt),
                                          sizeof(pollPkt));

                if (memcmp(pollPkt.header, "POLL", 4) == 0 &&
                    pollPkt.targetId == Config::ID) {

                    const uint64_t tRx2 = DW1000Ng::getReceiveTimestamp();
                    const uint64_t tTx2 = (tRx2 + SCHEDULED_REPLY_DELAY) &
                                          0xFFFFFFFE00ULL;

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
                    DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&respPkt),
                                              sizeof(respPkt));
                    DW1000Ng::setDelayedTRX(respPkt.txTimestamp);
                    DW1000Ng::startTransmit(TransmitMode::DELAYED);

                    const uint32_t txWait = millis();
                    while (!DW1000Ng::isTransmitDone()) {
                        if (millis() - txWait > 6) break;
                        yield();
                    }
                    DW1000Ng::clearTransmitStatus();
                    success = true;
                    break;
                }
            }
            // Not our poll; keep listening
        }

        if (DW1000Ng::isReceiveFailed() || DW1000Ng::isReceiveTimeout()) {
            DW1000Ng::clearReceiveStatus();
        }

        yield();
    }

    DW1000Ng::forceTRxOff();
    return success;
}

// ==============================================================================
// 11. SLOT HANDLER AND BEACON
// ==============================================================================
static void sendBeacon(uint8_t slot) {
    SyncBeaconPacket pkt = {};
    pkt.senderId           = Config::ID;
    pkt.senderSlot         = slot;
    pkt.frameId            = tdma.getFrameId();
    pkt.senderUptimeMs     = millis();
    pkt.maneuverStartFrame = g_candidateStartFrame;
    esp_now_send(BROADCAST_MAC, (uint8_t*)&pkt, sizeof(pkt));
}

static void handleSlot(uint32_t slot) {
    if (slot >= 16) return;
    const SlotAction &a = SLOT_TABLE[slot];

    if (a.isBeacon) {
        if (a.actor == Config::ID) {
            sendBeacon((uint8_t)slot);
        }
        return;
    }

    if (a.actor == Config::ID) {
        performRangingPoll(a.target);
    } else if (a.target == Config::ID) {
        performRangingListenBlocking(13);
    }
    // else: idle for this slot
}

// ==============================================================================
// 12. SETUP
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1000);

    AnjomanI2C::init(PIN_I2C0_SDA, PIN_I2C0_SCL, 400000);

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

    encL.begin();
    encR.begin();
    imu.begin();
    power_monitor.begin();

    setupESPNow();
    setupUWB();

    logger.init();
    tdma.init(Config::ID);

    for (int i = 0; i < 5; i++) g_uwbMetrics[i] = UWBMetrics{};

    neopixelWrite(PIN_STATUS_RGB, 50, 40, 0);

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
// 13. CORE 0 LOOP
// ==============================================================================
void loop() {
    const uint32_t nowMs = millis();
    tdma.tick(nowMs, micros());

    // ---- UWB temperature refresh ----
    static uint32_t lastTempMs = 0;
    if (nowMs - lastTempMs >= 500) {
        lastTempMs = nowMs;
        const float t = DW1000Ng::getTemperature();
        if (t > -30.0f && t < 90.0f) {
            g_cachedTempUwb = t;
        }
    }

    // ---- Consensus: propose a start frame once all peers have been seen ----
    if (g_candidateStartFrame == 0) {
        const uint8_t selfBit    = (1u << Config::ID);
        const uint8_t allPeers   = 0b11110u & ~selfBit;   // bits for 3 peers
        if ((g_peersSeenMask & allPeers) == allPeers) {
            g_candidateStartFrame = tdma.getFrameId() + 100;
        }
    }

    // ---- Maneuver trigger ----
    if (!g_maneuverTriggered &&
        g_candidateStartFrame > 0 &&
        tdma.getFrameId() >= g_candidateStartFrame) {

        g_maneuverTriggered = true;

        portENTER_CRITICAL(&g_shared.mux);
        g_shared.maneuverRunning = true;
        g_shared.maneuverStartMs = nowMs;
        g_shared.robotState      = 1;
        portEXIT_CRITICAL(&g_shared.mux);
    }

    // ---- Formation controller (10 Hz sub-rate) ----
    static uint32_t lastCtrlMs = 0;
    if (nowMs - lastCtrlMs >= 100) {
        lastCtrlMs = nowMs;

        portENTER_CRITICAL(&g_shared.mux);
        const bool     isRunning = g_shared.maneuverRunning;
        const bool     isDone    = g_shared.maneuverFinished;
        const uint32_t startMs   = g_shared.maneuverStartMs;
        const float    curX      = g_shared.posX;
        const float    curY      = g_shared.posY;
        const float    curTh     = g_shared.headingRad;
        portEXIT_CRITICAL(&g_shared.mux);

        if (isRunning && !isDone && nowMs >= startMs) {
            const float tElapsedSec = (float)(nowMs - startMs) / 1000.0f;

            neopixelWrite(PIN_STATUS_RGB, 0, 50, 0);

            if (tElapsedSec >= FormationReference::T_TOTAL_SEC) {
                portENTER_CRITICAL(&g_shared.mux);
                g_shared.maneuverFinished = true;
                g_shared.vCommand         = 0.0f;
                g_shared.omegaCommand     = 0.0f;
                g_shared.robotState       = 2;
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

    // ---- Slot execution ----
    const uint32_t frame = tdma.getFrameId();
    const uint32_t slot  = tdma.getCurrentSlotIndex();

    static uint32_t lastActedFrame = UINT32_MAX;
    static uint32_t lastActedSlot  = UINT32_MAX;

    // ---- Log check (before the slot's blocking operations) ----
    static uint32_t lastLoggedFrame = UINT32_MAX;
    const bool logArmed = g_shared.maneuverRunning && !g_shared.maneuverFinished;
    const uint32_t logTargetSlot = frame % 16;

    if (logArmed && frame != lastLoggedFrame && slot == logTargetSlot) {
        lastLoggedFrame = frame;

        LogRecord rec = {};

        portENTER_CRITICAL(&g_shared.mux);
        rec.t_ms            = nowMs;
        rec.x               = g_shared.posX;
        rec.y               = g_shared.posY;
        rec.heading         = g_shared.headingRad;
        rec.v_cmd           = g_shared.vCommand;
        rec.omega_cmd       = g_shared.omegaCommand;
        rec.rpm_l           = g_shared.rpmL;
        rec.rpm_r           = g_shared.rpmR;
        rec.gyro_z          = g_shared.gyroZ;
        rec.imu_temp_c      = g_shared.imuTempC;
        rec.imu_accel_x     = g_shared.accelX;
        rec.eskf_var_x      = g_shared.eskfVarX;
        rec.eskf_var_y      = g_shared.eskfVarY;
        rec.eskf_var_theta  = g_shared.eskfVarTheta;
        rec.eskf_var_bias   = g_shared.eskfVarBias;
        rec.duty_l          = g_shared.dutyL;
        rec.duty_r          = g_shared.dutyR;
        rec.target_rpm_l    = g_shared.targetRpmL;
        rec.target_rpm_r    = g_shared.targetRpmR;
        rec.robot_state     = g_shared.robotState;
        portEXIT_CRITICAL(&g_shared.mux);

        float vBat = 7.4f, iBat = 0.0f, pBat = 0.0f;
        power_monitor.read(vBat, iBat, pBat);
        rec.vbat      = vBat;
        rec.current_a = iBat;

        // UWB peer blocks
        uint8_t peers[3];
        uint8_t pidx = 0;
        for (uint8_t p = 1; p <= 4; p++) {
            if (p == Config::ID) continue;
            peers[pidx++] = p;
        }

        for (int i = 0; i < 3; i++) {
            const UWBMetrics &m = g_uwbMetrics[peers[i]];
            UWBPeerBlock &blk = rec.uwb[i];
            blk.peerId    = peers[i];
            blk.ldeErr    = m.valid ? m.ldeErr    : 0;
            blk.stdNoise  = m.valid ? m.stdNoise  : 0;
            blk.fpAmpl1   = m.valid ? m.fpAmpl1   : 0;
            blk.fpAmpl2   = m.valid ? m.fpAmpl2   : 0;
            blk.cirPwr    = m.valid ? m.cirPwr    : 0;
            blk.rawDist   = m.valid ? m.rawDist   : 0.0f;
            blk.cleanDist = m.valid ? m.cleanDist : 0.0f;
            blk.rssi      = m.valid ? m.rssi      : 0.0f;
            blk.fpPower   = m.valid ? m.fpPower   : 0.0f;
            blk.respTemp  = m.valid ? m.respTemp  : 0.0f;
        }
        rec.rxpacc = g_lastRxpacc;

        rec.tdma_frame_id   = frame;
        rec.tdma_slot_index = (uint8_t)slot;
        rec.tdma_sync_lost  = tdma.isSyncLost() ? 1 : 0;
        rec.maneuver_id     = 1;

        logger.record(rec);
    }

    // ---- Slot execution (once per slot change) ----
    if (frame != lastActedFrame || slot != lastActedSlot) {
        lastActedFrame = frame;
        lastActedSlot  = slot;
        handleSlot(slot);
    }

    yield();
}
