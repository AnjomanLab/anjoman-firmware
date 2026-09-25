#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>

#include "PinMap.h"
#include "RobotConfig.h"
#include "MotorController.h"
#include "MagneticEncoder.h"
#include "BMI160_Custom.h"
#include "HeadingKalmanFilter.h"
#include "UWBPreprocessor.h"
#include "FormationReference.h"
#include "FormationController.h"
#include "ManeuverLogger.h"

// ==============================================================================
// 1. HARDWARE TIME CONSTANTS & TIMING
// ==============================================================================
constexpr double   SPEED_OF_LIGHT         = 299792458.0;
constexpr double   TIME_UNIT_SEC          = 0.000000000015650040064103;
constexpr uint64_t SCHEDULED_REPLY_DELAY  = 159744000ULL; 
constexpr uint32_t TDMA_FRAME_US          = 200000; 

constexpr double   FREQ_OFFSET_MULTIPLIER         = 998.4e6 / (2.0 * 1024.0 * 131072.0);
constexpr double   HERTZ_TO_PPM_MULTIPLIER_CHAN_5 = -1.0e6 / 6489.6e6;

constexpr uint32_t AUTO_START_DELAY_MS    = 10000; // 10-second automatic countdown

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

#pragma pack(push, 1)
struct UWBPollPacket {
    char     header[4];
    uint8_t  initiatorId;    
    uint8_t  targetId;       
    uint32_t sequence;
};

struct UWBResponsePacket {
    char     header[4];
    uint8_t  responderId;    
    uint8_t  targetId;       
    uint32_t sequence;
    uint8_t  rxTimestamp[5];
    uint8_t  txTimestamp[5];
    float    tempUwb;
    float    tempEsp;
    float    vbatUwb;
};

struct SyncBeaconPacket {
    uint32_t frameId;
    uint32_t timestampMs;
    uint8_t  maneuverActive;
    uint32_t maneuverStartMs;
};
#pragma pack(pop)

// Singletons
MotorController motorL(PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, Config::INVERT_MOTOR_LEFT);
MotorController motorR(PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, Config::INVERT_MOTOR_RIGHT);
MagneticEncoder encL(Wire, 0x70, 0, Config::INVERT_ENCODER_LEFT);
MagneticEncoder encR(Wire, 0x70, 1, Config::INVERT_ENCODER_RIGHT);
BMI160_Custom   imu(Wire, 0x69, 2);
HeadingKalmanFilter g_headingFilter;
ManeuverLogger  logger;

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

struct LatestUWBMetrics {
    float    rawDist;
    float    cleanDist;
    float    rssi;
    float    fpPower;
    uint16_t stdNoise;
    uint8_t  peerId;
    uint8_t  ldeErr;
} g_uwbMetrics;

volatile uint32_t globalFrameId = 0;
volatile uint64_t frameStartUs   = 0;
bool isFrameSynced = false;
bool inRxMode = false;
float cachedTempUwb = 25.0f;
float cachedTempEsp = 25.0f;

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

void readINA226(float &vBus, float &currentA) {
    Wire.beginTransmission(0x70);
    Wire.write(1 << 3); // TCA Channel 3
    if (Wire.endTransmission() != 0) { vBus = 7.4f; currentA = 0.0f; return; }

    Wire.beginTransmission(0x40);
    Wire.write(0x02); // Bus Voltage Register
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)0x40, (uint8_t)2) >= 2) {
        uint16_t rawV = (Wire.read() << 8) | Wire.read();
        vBus = (float)rawV * 0.00125f;
    }

    Wire.beginTransmission(0x40);
    Wire.write(0x01); // Shunt Register
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)0x40, (uint8_t)2) >= 2) {
        int16_t rawI = (int16_t)((Wire.read() << 8) | Wire.read());
        currentA = ((float)rawI * 2.5e-6f) / Config::SHUNT_RESISTOR_OHM;
    }
}

// ==============================================================================
// 2. CORE 1: 100 Hz HARD REAL-TIME MOTOR PI & ODOMETRY LOOP
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
                         Config::Q_YAW_DISCRETE, Config::Q_GYRO_BIAS_WALK, Config::R_YAW_ENCODER);

    constexpr float RAD_S_TO_RPM = 60.0f / (2.0f * 3.1415926535f);

    while (true) {
        float dt = Config::CONTROL_PERIOD_S;

        encL.update(dt);
        encR.update(dt);
        imu.readSensorData();

        float measRpmL = encL.getRPM();
        float measRpmR = encR.getRPM();
        float measRadL = encL.getRadPerSec();
        float measRadR = encR.getRadPerSec();

        float gyroZRadS = imu.getGyroZ() * 0.01745329251f;

        g_headingFilter.predict(gyroZRadS, dt);
        float deltaThetaEnc = (measRadR - measRadL) * (Config::WHEEL_RADIUS_M / Config::TRACK_WIDTH_M) * dt;
        g_headingFilter.updateEncoder(deltaThetaEnc, gyroZRadS, dt);
        odomTheta = g_headingFilter.getHeadingRad();

        float vActual = 0.5f * (measRadL + measRadR) * Config::WHEEL_RADIUS_M;
        odomX += vActual * cosf(odomTheta) * dt;
        odomY += vActual * sinf(odomTheta) * dt;

        portENTER_CRITICAL(&g_shared.mux);
        g_shared.posX       = odomX;
        g_shared.posY       = odomY;
        g_shared.headingRad = odomTheta;
        g_shared.rpmL       = measRpmL;
        g_shared.rpmR       = measRpmR;
        g_shared.gyroZ      = gyroZRadS;
        float targetV       = g_shared.vCommand;
        float targetOmega   = g_shared.omegaCommand;
        bool  isActive      = g_shared.maneuverRunning;
        bool  isFinished    = g_shared.maneuverFinished;
        portEXIT_CRITICAL(&g_shared.mux);

        if (!isActive || isFinished) {
            motorL.brake();
            motorR.brake();
        } else {
            float vTargetL = targetV - 0.5f * Config::TRACK_WIDTH_M * targetOmega;
            float vTargetR = targetV + 0.5f * Config::TRACK_WIDTH_M * targetOmega;

            float targetRpmL = (vTargetL / Config::WHEEL_RADIUS_M) * RAD_S_TO_RPM;
            float targetRpmR = (vTargetR / Config::WHEEL_RADIUS_M) * RAD_S_TO_RPM;

            motorL.computeVelocityControl(targetRpmL, measRpmL, 7.4f, dt);
            motorR.computeVelocityControl(targetRpmR, measRpmR, 7.4f, dt);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==============================================================================
// 3. ESP-NOW TDMA SYNC (BROADCAST MISSION START)
// ==============================================================================
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
    if (data_len == sizeof(SyncBeaconPacket)) {
        SyncBeaconPacket pkt;
        memcpy(&pkt, data, sizeof(SyncBeaconPacket));
        globalFrameId = pkt.frameId;
        frameStartUs = micros();
        isFrameSynced = true;

        if (pkt.maneuverActive && !g_shared.maneuverRunning) {
            portENTER_CRITICAL(&g_shared.mux);
            g_shared.maneuverRunning = true;
            g_shared.maneuverStartMs = pkt.maneuverStartMs;
            portEXIT_CRITICAL(&g_shared.mux);
        }
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
    peerInfo.ifidx = WIFI_IF_STA;
    esp_now_add_peer(&peerInfo);
}

// ==============================================================================
// 4. DW1000 UWB HARDWARE & TDMA RANGING
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
    inRxMode = true;
}

bool performRangingPoll(uint8_t targetPeerId) {
    UWBPollPacket pollPkt = {};
    memcpy(pollPkt.header, "POLL", 4);
    pollPkt.initiatorId = Config::ID;
    pollPkt.targetId    = targetPeerId;
    pollPkt.sequence    = globalFrameId;

    DW1000Ng::forceTRxOff();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::clearReceiveStatus();
    DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&pollPkt), sizeof(pollPkt));
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    uint32_t txStart = millis();
    while (!DW1000Ng::isTransmitDone()) {
        if (millis() - txStart > 6) { inRxMode = false; return false; }
        yield();
    }
    DW1000Ng::clearTransmitStatus();
    uint64_t tTx1 = DW1000Ng::getTransmitTimestamp();

    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
    inRxMode = true;
    uint32_t waitRx = millis();
    bool success = false;

    while (millis() - waitRx < 8) {
        if (DW1000Ng::isReceiveDone()) {
            DW1000Ng::clearReceiveStatus();
            size_t len = DW1000Ng::getReceivedDataLength();
            if (len >= sizeof(UWBResponsePacket)) {
                UWBResponsePacket respPkt;
                DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&respPkt), sizeof(respPkt));

                if (memcmp(respPkt.header, "RESP", 4) == 0 &&
                    respPkt.responderId == targetPeerId &&
                    respPkt.targetId == Config::ID) {

                    uint64_t tRx1 = DW1000Ng::getReceiveTimestamp();
                    uint64_t tRx2 = read40BitTime(respPkt.rxTimestamp);
                    uint64_t tTx2 = read40BitTime(respPkt.txTimestamp);

                    auto diag = DW1000Ng::getChannelDiagnostics();

                    int32_t ci = DW1000Ng::getCarrierIntegrator();
                    double cfoPpm = (double)ci * FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5;
                    double clockOffsetRatio = cfoPpm * 1.0e-6;

                    int64_t tRound = (int64_t)((tRx1 - tTx1) & 0xFFFFFFFFFFULL);
                    int64_t tReply = (int64_t)((tTx2 - tRx2) & 0xFFFFFFFFFFULL);
                    double tReplyComp = (double)tReply * (1.0 + clockOffsetRatio);
                    double tofCompTicks = ((double)tRound - tReplyComp) / 2.0;
                    float distCfoM = (float)(tofCompTicks * TIME_UNIT_SEC * SPEED_OF_LIGHT);

                    float cleanM = UWBPreprocessor::correctDistance(
                        Config::ID, targetPeerId, distCfoM, cachedTempUwb, respPkt.tempUwb
                    );

                    g_uwbMetrics.rawDist   = distCfoM;
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
    inRxMode = false;
    return success;
}

void performRangingListenSafe() {
    static uint32_t rxArmedTime = 0;
    if (!inRxMode) {
        DW1000Ng::forceTRxOff();
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
        inRxMode = true;
        rxArmedTime = millis();
    }

    if (DW1000Ng::isReceiveFailed() || (millis() - rxArmedTime > 15)) {
        DW1000Ng::forceTRxOff();
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
        inRxMode = true;
        rxArmedTime = millis();
        return;
    }

    if (DW1000Ng::isReceiveDone()) {
        DW1000Ng::clearReceiveStatus();
        size_t len = DW1000Ng::getReceivedDataLength();
        if (len >= sizeof(UWBPollPacket)) {
            UWBPollPacket pollPkt;
            DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&pollPkt), sizeof(pollPkt));

            if (memcmp(pollPkt.header, "POLL", 4) == 0 && pollPkt.targetId == Config::ID) {
                uint64_t tRx2 = DW1000Ng::getReceiveTimestamp();
                uint64_t tTx2 = (tRx2 + SCHEDULED_REPLY_DELAY) & 0xFFFFFFFE00ULL;

                UWBResponsePacket respPkt = {};
                memcpy(respPkt.header, "RESP", 4);
                respPkt.responderId = Config::ID;
                respPkt.targetId    = pollPkt.initiatorId;
                respPkt.sequence    = pollPkt.sequence;
                write40BitTime(respPkt.rxTimestamp, tRx2);
                write40BitTime(respPkt.txTimestamp, tTx2);
                respPkt.tempUwb     = cachedTempUwb;
                respPkt.tempEsp     = cachedTempEsp;

                DW1000Ng::forceTRxOff();
                DW1000Ng::clearTransmitStatus();
                DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&respPkt), sizeof(respPkt));
                DW1000Ng::setDelayedTRX(respPkt.txTimestamp);
                DW1000Ng::startTransmit(TransmitMode::DELAYED);

                uint32_t txWait = millis();
                while (!DW1000Ng::isTransmitDone()) {
                    if (millis() - txWait > 8) break;
                    yield();
                }
                DW1000Ng::clearTransmitStatus();
            }
        }
        DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
        inRxMode = true;
        rxArmedTime = millis();
    }
}

// ==============================================================================
// 5. SETUP
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1000);

    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, 400000);

    motorL.begin(20000, 10);
    motorR.begin(20000, 10);

    MotorSysIDParams paramsL = { Config::DEADBAND_FWD_L, Config::DEADBAND_REV_L, Config::GAIN_RPM_FWD_L, Config::GAIN_RPM_REV_L, 7.40f };
    MotorSysIDParams paramsR = { Config::DEADBAND_FWD_R, Config::DEADBAND_REV_R, Config::GAIN_RPM_FWD_R, Config::GAIN_RPM_REV_R, 7.40f };
    motorL.setCalibration(paramsL);
    motorR.setCalibration(paramsR);

    PIDGains gains = { 0.0050f, 0.035f, 0.40f };
    motorL.setPIDGains(gains);
    motorR.setPIDGains(gains);

    encL.begin();
    encR.begin();
    imu.begin();

    setupESPNow();
    setupUWB();

    logger.init();

    // Solid Yellow = 10s Countdown Armed
    rgbLedWrite(PIN_STATUS_RGB, 50, 40, 0);

    // Core 1 Hard Real-Time Motor PI Task (100 Hz, Priority 3)
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
// 6. CORE 0 LOOP: 10s COUNTDOWN, FORMATION CONTROLLER & 5 Hz RAM LOGGER
// ==============================================================================
void loop() {
    uint32_t nowMs = millis();

    // --------------------------------------------------------------------------
    // ROBOT 1: AUTOMATIC 10.0-SECOND SYNCHRONIZED COUNTDOWN TRIGGER
    // --------------------------------------------------------------------------
    if (Config::ID == 1) {
        static bool autoTriggerArmed = false;
        static uint32_t mStartMs = 0;

        if (!autoTriggerArmed && nowMs >= AUTO_START_DELAY_MS) {
            autoTriggerArmed = true;
            mStartMs = nowMs + 1000;

            portENTER_CRITICAL(&g_shared.mux);
            g_shared.maneuverRunning = true;
            g_shared.maneuverStartMs = mStartMs;
            portEXIT_CRITICAL(&g_shared.mux);
        }

        uint64_t curUs = micros();
        if (curUs - frameStartUs >= TDMA_FRAME_US) {
            frameStartUs = curUs;
            globalFrameId = globalFrameId + 1;

            SyncBeaconPacket sync = {
                globalFrameId, nowMs,
                (uint8_t)(autoTriggerArmed ? 1 : 0),
                mStartMs
            };
            esp_now_send(BROADCAST_MAC, (uint8_t*)&sync, sizeof(SyncBeaconPacket));
            isFrameSynced = true;
        }
    }

    if (!isFrameSynced && Config::ID != 1) {
        performRangingListenSafe();
        yield();
        return;
    }

    // --------------------------------------------------------------------------
    // 10 Hz FORMATION CONTROLLER & 5 Hz RAM LOGGING
    // --------------------------------------------------------------------------
    static uint32_t lastCtrlMs = 0;
    static uint32_t lastLogMs  = 0;

    if (nowMs - lastCtrlMs >= 100) {
        lastCtrlMs = nowMs;

        portENTER_CRITICAL(&g_shared.mux);
        bool isRunning   = g_shared.maneuverRunning;
        bool isDone      = g_shared.maneuverFinished;
        uint32_t startMs = g_shared.maneuverStartMs;
        float curX       = g_shared.posX;
        float curY       = g_shared.posY;
        float curTh      = g_shared.headingRad;
        float rL         = g_shared.rpmL;
        float rR         = g_shared.rpmR;
        float gZ         = g_shared.gyroZ;
        portEXIT_CRITICAL(&g_shared.mux);

        if (isRunning && !isDone && nowMs >= startMs) {
            float tElapsedSec = (float)(nowMs - startMs) / 1000.0f;

            // Solid Green = Actively Moving
            rgbLedWrite(PIN_STATUS_RGB, 0, 50, 0);

            if (tElapsedSec >= FormationReference::T_TOTAL_SEC) {
                // MANEUVER COMPLETE: Brake Motors & Commit RAM to Flash
                portENTER_CRITICAL(&g_shared.mux);
                g_shared.maneuverFinished = true;
                g_shared.vCommand = 0.0f;
                g_shared.omegaCommand = 0.0f;
                portEXIT_CRITICAL(&g_shared.mux);

                logger.commitToFlash(1); // Maneuver ID = 1
                logger.startDownloadServer();

                // Solid Blue = Ready for curl download!
                rgbLedWrite(PIN_STATUS_RGB, 0, 0, 50);
            } else {
                // 1. Formation Reference Trajectory
                FormationState2D target = FormationReference::evaluate(Config::ID, tElapsedSec);

                // 2. Virtual Point Feedback Linearization
                WheelVelocityCommand cmd = FormationController::compute(
                    Config::ID, target, curX, curY, curTh
                );

                portENTER_CRITICAL(&g_shared.mux);
                g_shared.vCommand     = cmd.vLinear;
                g_shared.omegaCommand = cmd.omegaRadS;
                portEXIT_CRITICAL(&g_shared.mux);

                // 3. 5 Hz Snapshot Logging to RAM Buffer
                if (nowMs - lastLogMs >= 200) {
                    lastLogMs = nowMs;

                    float vBat = 7.4f, iBat = 0.0f;
                    readINA226(vBat, iBat);

                    LogRecord rec = {};
                    rec.t_ms          = nowMs;
                    rec.x             = curX;
                    rec.y             = curY;
                    rec.heading       = curTh;
                    rec.v_cmd         = cmd.vLinear;
                    rec.omega_cmd     = cmd.omegaRadS;
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
                    rec.uwb_temp      = cachedTempUwb;

                    logger.record(rec);
                }
            }
        }
    }

    if (g_shared.maneuverFinished) {
        logger.handleClient();
    }

    // TDMA Execution Slots
    uint32_t slotUs = (uint32_t)(micros() - frameStartUs);

    if (slotUs >= 15000 && slotUs < 30000) {
        if (Config::ID == 1) {
            static uint32_t lp = 0;
            if (lp != globalFrameId) { lp = globalFrameId; performRangingPoll(2); }
        } else performRangingListenSafe();
    }
    else if (slotUs >= 60000 && slotUs < 75000) {
        if (Config::ID == 2) {
            static uint32_t lp = 0;
            if (lp != globalFrameId) { lp = globalFrameId; performRangingPoll(1); }
        } else performRangingListenSafe();
    }
    else if (slotUs >= 105000 && slotUs < 120000) {
        if (Config::ID == 3) {
            static uint32_t lp = 0;
            if (lp != globalFrameId) { lp = globalFrameId; performRangingPoll(1); }
        } else performRangingListenSafe();
    }
    else if (slotUs >= 150000 && slotUs < 165000) {
        if (Config::ID == 4) {
            static uint32_t lp = 0;
            if (lp != globalFrameId) { lp = globalFrameId; performRangingPoll(1); }
        } else performRangingListenSafe();
    } else {
        performRangingListenSafe();
    }
    yield();
}
