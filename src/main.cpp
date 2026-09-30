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
// 2. ACTIVE FLEET
// ==============================================================================
constexpr uint8_t ACTIVE_ROBOTS[] = {2, 3, 4};
constexpr uint8_t N_ACTIVE        = 3;

static bool isActive(uint8_t id) {
    for (uint8_t i = 0; i < N_ACTIVE; i++) {
        if (ACTIVE_ROBOTS[i] == id) return true;
    }
    return false;
}

static uint8_t allPeersMaskForMe() {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < N_ACTIVE; i++) {
        if (ACTIVE_ROBOTS[i] != Config::ID) {
            mask |= (1u << ACTIVE_ROBOTS[i]);
        }
    }
    return mask;
}

// ==============================================================================
// 3. HARDWARE SINGLETONS
// ==============================================================================
MotorController motorL(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, Config::INVERT_MOTOR_LEFT);
MotorController motorR(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, Config::INVERT_MOTOR_RIGHT);

MagneticEncoder encL(Wire, 0x70, 0, Config::INVERT_ENCODER_LEFT);
MagneticEncoder encR(Wire, 0x70, 1, Config::INVERT_ENCODER_RIGHT);

BMI160_Custom   imu(Wire, 0x69, 2);
INA226          power_monitor(Wire, 0x40, 3, Config::SHUNT_RESISTOR_OHM);

ESKF            g_eskf;
TDMAEngine      tdma;

// ==============================================================================
// 4. TDMA SLOT TABLE — 3-robot equilateral triangle
// ==============================================================================
struct SlotAction {
    uint8_t actor;
    uint8_t target;
    bool    isBeacon;
};

static const SlotAction SLOT_TABLE[16] = {
    // R2 owns slots 0-3
    {2, 0, true},  {2, 3, false}, {2, 4, false}, {0, 0, false},
    // R3 owns slots 4-7
    {3, 0, true},  {3, 2, false}, {3, 4, false}, {0, 0, false},
    // R4 owns slots 8-11
    {4, 0, true},  {4, 2, false}, {4, 3, false}, {0, 0, false},
    // Listen windows (margin slots)
    {0, 0, false}, {0, 0, false}, {0, 0, false}, {0, 0, false},
};

// ==============================================================================
// 5. SHARED STATE
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
// 6. UWB METRICS PER PEER
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

// ==============================================================================
// 7. CONSENSUS STATE
// ==============================================================================
static uint8_t  g_peersSeenMask       = 0;
static uint32_t g_candidateStartFrame = 0;
static bool     g_maneuverTriggered   = false;

// ==============================================================================
// 8. UTILITIES
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
// 9. CORE 1 — 100 Hz REAL-TIME LOOP
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
        const bool  isActive_   = g_shared.maneuverRunning;
        const bool  isFinished_ = g_shared.maneuverFinished;
        portEXIT_CRITICAL(&g_shared.mux);

        float dutyL = 0.0f, dutyR = 0.0f;
        float targetRpmL = 0.0f, targetRpmR = 0.0f;

        if (!isActive_ || isFinished_) {
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
// 10. ESP-NOW
// ==============================================================================
void onDataRecv(const uint8_t *mac, const uint8_t *data, int data_len) {
    if (data_len != sizeof(SyncBeaconPacket)) return;

    SyncBeaconPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (pkt.senderId == Config::ID) return;
    if (!isActive(pkt.senderId))     return;

    tdma.onSyncReceived(pkt.senderId,
                        pkt.frameId,
                        pkt.senderUptimeMs,
                        pkt.senderSlot,
                        millis(),
                        micros());

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
// 11. UWB
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
    if (!isActive(targetPeerId)) return false;

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
                    respPkt.targetId    == Config::ID) {

                    const uint64_t tRx1 = DW1000Ng::getReceiveTimestamp();
                    const uint64_t tRx2 = read40BitTime(respPkt.rxTimestamp);
                    const uint64_t tTx2 = read40BitTime(respPkt.txTimestamp);

                    const int64_t tRound = (int64_t)((tRx1 - tTx1) & 0xFFFFFFFFFFULL);
                    const int64_t tReply = (int64_t)((tTx2 - tRx2) & 0xFFFFFFFFFFULL);

                    if (tReply >= tRound) {
                        break;
                    }

                    const int64_t tofRawTicks = (tRound - tReply) / 2;
                    const float distRawM = (float)(tofRawTicks * TIME_UNIT_SEC *
                                                   SPEED_OF_LIGHT);

                    const float cleanM = UWBPreprocessor::correctRawDistance(
                        Config::ID, targetPeerId, distRawM, 0.0f, 0.0f);

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
                    m.respTemp   = 25.0f;

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
                    respPkt.tempUwb     = 25.0f;
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
// 12. SLOT HANDLER AND BEACON
// ==============================================================================
static void sendBeacon(uint8_t slot) {
    SyncBeaconPacket pkt = {};
    pkt.senderId           = Config::ID;
    pkt.senderSlot         = slot;
    pkt.frameId            = tdma.getFrameId();
    pkt.senderUptimeMs     = millis();
    pkt.maneuverStartFrame = g_candidateStartFrame;

    uint8_t idx = 0;
    for (uint8_t i = 0; i < N_ACTIVE && idx < 2; i++) {
        const uint8_t p = ACTIVE_ROBOTS[i];
        if (p == Config::ID) continue;

        const UWBMetrics &m = g_uwbMetrics[p];
        if (!m.valid) continue;

        UWBBeaconEntry &e = pkt.peers[idx];
        e.peerId    = p;
        e.ldeErr    = m.ldeErr;
        e.rawDist   = m.rawDist;
        e.cleanDist = m.cleanDist;
        e.rssi      = m.rssi;
        e.fpPower   = m.fpPower;
        e.respTemp  = m.respTemp;
        idx++;
    }
    pkt.nPeers = idx;

    static float    lastVbat   = 7.4f;
    static uint32_t lastVbatMs = 0;
    if (millis() - lastVbatMs >= 1000) {
        lastVbatMs = millis();
        float v, i, pwr;
        if (power_monitor.read(v, i, pwr)) lastVbat = v;
    }
    pkt.vbat = lastVbat;

    esp_now_send(BROADCAST_MAC, (uint8_t*)&pkt, sizeof(pkt));
}

static void handleSlot(uint32_t slot) {
    if (slot >= 16) return;
    const SlotAction &a = SLOT_TABLE[slot];

    if (!isActive(a.actor)) return;

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
}

// ==============================================================================
// 13. SETUP
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
// 14. CORE 0 LOOP
// ==============================================================================
void loop() {
    const uint32_t nowMs = millis();
    tdma.tick(nowMs, micros());

    // ---- Consensus: propose a start frame once all peers have been seen ----
    if (g_candidateStartFrame == 0) {
        const uint8_t requiredPeers = allPeersMaskForMe();
        if ((g_peersSeenMask & requiredPeers) == requiredPeers) {
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

    // ---- Slot execution ----
    const uint32_t frame = tdma.getFrameId();
    const uint32_t slot  = tdma.getCurrentSlotIndex();

    static uint32_t lastActedFrame = UINT32_MAX;
    static uint32_t lastActedSlot  = UINT32_MAX;

    if (frame != lastActedFrame || slot != lastActedSlot) {
        lastActedFrame = frame;
        lastActedSlot  = slot;
        handleSlot(slot);
    }

    yield();
}
