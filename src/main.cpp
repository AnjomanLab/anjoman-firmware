#include <Arduino.h>
#include <SPI.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>

#include "PinMap.h"
#include "RobotConfig.h"

// ==============================================================================
// 1. HARDWARE TIME CONSTANTS & PHYSICAL EQUATIONS
// ==============================================================================
constexpr double   SPEED_OF_LIGHT         = 299792458.0;
constexpr double   TIME_UNIT_SEC          = 0.000000000015650040064103;

// Verified hardware reply delay for all robots: exactly 2.5 ms (159,744,000 ticks)
constexpr uint64_t SCHEDULED_REPLY_DELAY  = 159744000ULL; 

// Deterministic 200 ms TDMA Frame (5 Hz full mesh update rate)
constexpr uint32_t TDMA_FRAME_US          = 200000; 
constexpr uint32_t PREFLIGHT_TIMEOUT_MS   = 4000;   // 4-second initial link check

// Official Decawave Carrier Integrator conversion constants (Channel 5, N=1024)
constexpr double   FREQ_OFFSET_MULTIPLIER         = 998.4e6 / (2.0 * 1024.0 * 131072.0); // ~3.71933 Hz/count
constexpr double   HERTZ_TO_PPM_MULTIPLIER_CHAN_5 = -1.0e6 / 6489.6e6;                   // ~-1.5409e-4 ppm/Hz

uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

device_configuration_t UWB_CONFIG = {
    false,
    true,
    true,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

#pragma pack(push, 1)
struct UWBPollPacket {
    char     header[4];      // "POLL"
    uint8_t  initiatorId;    
    uint8_t  targetId;       
    uint32_t sequence;
};

struct UWBResponsePacket {
    char     header[4];      // "RESP"
    uint8_t  responderId;    
    uint8_t  targetId;       
    uint32_t sequence;
    uint8_t  rxTimestamp[5]; // tRx2 (40-bit)
    uint8_t  txTimestamp[5]; // tTx2 (40-bit)
    float    tempUwb;        // Cached DW1000 internal temp
    float    tempEsp;        // Cached ESP32 temp
};

struct SyncBeaconPacket {
    uint32_t frameId;
    uint32_t timestampMs;
};

// 22-Parameter Pure Telemetry Record
struct FullEdgeTelemetry {
    uint32_t timestampMs;
    uint32_t frameId;
    uint8_t  initiatorId;
    uint8_t  responderId;
    uint8_t  status;         // 1 = Success, 0 = Timeout/Loss
    int32_t  carrierIntegrator;
    float    cfoPpm;
    int64_t  tofUncompTicks;
    int64_t  tofCompTicks;
    float    distUncompM;
    float    distClockCompM;
    float    distRssiCompM;
    float    distCalibM;
    float    rssi_dbm;
    uint64_t tTx1;
    uint64_t tRx1;
    uint64_t tRx2;
    uint64_t tTx2;
    float    tempUwbInit;
    float    tempUwbResp;
    float    tempEspInit;
    float    tempEspResp;
};
#pragma pack(pop)

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

volatile uint32_t globalFrameId = 0;
volatile uint64_t frameStartUs   = 0;
bool isFrameSynced = false;
bool inRxMode = false;

float cachedTempUwb = 25.0f;
float cachedTempEsp = 25.0f;

FullEdgeTelemetry fleetTelemetry[6]; // Storage for all 6 edges: (1,2), (1,3), (1,4), (2,3), (2,4), (3,4)

// ==============================================================================
// 2. ESP-NOW MESH NETWORKING
// ==============================================================================
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
    if (data_len == sizeof(SyncBeaconPacket)) {
        SyncBeaconPacket pkt;
        memcpy(&pkt, data, sizeof(SyncBeaconPacket));
        globalFrameId = pkt.frameId;
        frameStartUs = micros();
        isFrameSynced = true;
    } else if (data_len == sizeof(FullEdgeTelemetry)) {
        FullEdgeTelemetry edge;
        memcpy(&edge, data, sizeof(FullEdgeTelemetry));
        
        uint8_t idx = 0xFF;
        if (edge.initiatorId == 1 && edge.responderId == 2) idx = 0;
        else if (edge.initiatorId == 1 && edge.responderId == 3) idx = 1;
        else if (edge.initiatorId == 1 && edge.responderId == 4) idx = 2;
        else if (edge.initiatorId == 2 && edge.responderId == 3) idx = 3;
        else if (edge.initiatorId == 2 && edge.responderId == 4) idx = 4;
        else if (edge.initiatorId == 3 && edge.responderId == 4) idx = 5;

        if (idx != 0xFF) {
            fleetTelemetry[idx] = edge;
        }
    }
}

void setupESPNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERROR] ESP-NOW Init Failed!");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, BROADCAST_MAC, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;
    esp_now_add_peer(&peerInfo);
}

// ==============================================================================
// 3. DW1000 HARDWARE INITIALIZATION
// ==============================================================================
void setupUWB() {
    pinMode(PIN_UWB_RST, OUTPUT);
    digitalWrite(PIN_UWB_RST, LOW);
    delay(10);
    pinMode(PIN_UWB_RST, INPUT);
    delay(25);

    SPI.begin(PIN_UWB_SCK, PIN_UWB_MISO, PIN_UWB_MOSI, PIN_UWB_CS);
    delay(10);

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

// ==============================================================================
// 4. SS-TWR ENGINE WITH CARRIER-INTEGRATOR COMPENSATION
// ==============================================================================
bool performRangingPoll(uint8_t targetPeerId, FullEdgeTelemetry &rec) {
    rec.timestampMs       = millis();
    rec.frameId           = globalFrameId;
    rec.initiatorId       = Config::ID;
    rec.responderId       = targetPeerId;
    rec.status            = 0; // Failed by default
    rec.carrierIntegrator = 0;
    rec.cfoPpm            = 0.0f;
    rec.tofUncompTicks    = 0;
    rec.tofCompTicks      = 0;
    rec.distUncompM       = 0.0f;
    rec.distClockCompM    = 0.0f;
    rec.distRssiCompM     = 0.0f;
    rec.distCalibM        = 0.0f;
    rec.rssi_dbm          = 0.0f;
    rec.tTx1 = rec.tRx1 = rec.tRx2 = rec.tTx2 = 0;
    rec.tempUwbInit       = cachedTempUwb;
    rec.tempUwbResp       = 0.0f;
    rec.tempEspInit       = cachedTempEsp;
    rec.tempEspResp       = 0.0f;

    UWBPollPacket pollPkt = {};
    memcpy(pollPkt.header, "POLL", 4);
    pollPkt.initiatorId   = Config::ID;
    pollPkt.targetId      = targetPeerId;
    pollPkt.sequence      = globalFrameId;

    DW1000Ng::forceTRxOff();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::clearReceiveStatus();

    DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&pollPkt), sizeof(pollPkt));
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    uint32_t txStart = millis();
    while (!DW1000Ng::isTransmitDone()) {
        if (millis() - txStart > 6) {
            inRxMode = false;
            return false;
        }
        yield();
    }
    DW1000Ng::clearTransmitStatus();
    uint64_t tTx1 = DW1000Ng::getTransmitTimestamp();

    // Await Response with generous 10 ms window
    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
    inRxMode = true;
    uint32_t waitRx = millis();
    bool success = false;

    while (millis() - waitRx < 10) {
        if (DW1000Ng::isReceiveDone()) {
            DW1000Ng::clearReceiveStatus();

            size_t len = DW1000Ng::getReceivedDataLength();
            if (len >= sizeof(UWBResponsePacket)) {
                UWBResponsePacket respPkt;
                DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&respPkt), sizeof(respPkt));

                if (memcmp(respPkt.header, "RESP", 4) == 0 &&
                    respPkt.responderId == targetPeerId &&
                    respPkt.targetId == Config::ID) {

                    // 1. Capture Timestamps
                    uint64_t tRx1 = DW1000Ng::getReceiveTimestamp();
                    uint64_t tRx2 = read40BitTime(respPkt.rxTimestamp);
                    uint64_t tTx2 = read40BitTime(respPkt.txTimestamp);

                    // 2. Read Raw Carrier Integrator ONCE
                    int32_t ci = DW1000Ng::getCarrierIntegrator();

                    // 3. Compute Official Decawave CFO & Clock Offset Ratio
                    double cfoPpm = (double)ci * FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5;
                    double clockOffsetRatio = cfoPpm * 1.0e-6;

                    // 4. Raw Durations
                    int64_t tRound = (int64_t)((tRx1 - tTx1) & 0xFFFFFFFFFFULL);
                    int64_t tReply = (int64_t)((tTx2 - tRx2) & 0xFFFFFFFFFFULL);

                    // A) Uncompensated Raw SS-TWR (Displays the legacy 20-40m bias)
                    int64_t tofUncomp = (tRound - tReply) / 2;
                    double distUncomp = (double)tofUncomp * TIME_UNIT_SEC * SPEED_OF_LIGHT;

                    // B) Clock-Offset Compensated SS-TWR (Official Decawave Formulation)
                    double tReplyCorrected = (double)tReply * (1.0 + clockOffsetRatio);
                    double tofComp = ((double)tRound - tReplyCorrected) / 2.0;
                    double distClockComp = tofComp * TIME_UNIT_SEC * SPEED_OF_LIGHT;

                    // C) Decawave Internal RSSI Power Bias Correction Curve
                    double distRssiComp = (double)DW1000NgRanging::correctRange((float)distClockComp);

                    // D) Final Zero-Offset Calibration
                    double distCalib = Config::getCalibratedDistance(Config::ID, targetPeerId, (float)distRssiComp);

                    // Pack into Telemetry
                    rec.status            = 1;
                    rec.carrierIntegrator = ci;
                    rec.cfoPpm            = (float)cfoPpm;
                    rec.tofUncompTicks    = tofUncomp;
                    rec.tofCompTicks      = (int64_t)tofComp;
                    rec.distUncompM       = (float)distUncomp;
                    rec.distClockCompM    = (float)distClockComp;
                    rec.distRssiCompM     = (float)distRssiComp;
                    rec.distCalibM        = (float)distCalib;
                    rec.rssi_dbm          = (float)DW1000Ng::getReceivePower();
                    rec.tTx1              = tTx1;
                    rec.tRx1              = tRx1;
                    rec.tRx2              = tRx2;
                    rec.tTx2              = tTx2;
                    rec.tempUwbResp       = respPkt.tempUwb;
                    rec.tempEspResp       = respPkt.tempEsp;

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
        DW1000Ng::clearReceiveFailedStatus();
        DW1000Ng::clearReceiveTimeoutStatus();
        DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
        inRxMode = true;
        rxArmedTime = millis();
    }

    // Auto-recovery Watchdog: if receiver is stuck or errored for > 15ms, clear and re-arm
    if (DW1000Ng::isReceiveFailed() || (millis() - rxArmedTime > 15)) {
        DW1000Ng::forceTRxOff();
        DW1000Ng::clearReceiveStatus();
        DW1000Ng::clearReceiveFailedStatus();
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
// 5. PRE-FLIGHT SANITY CHECK (PST) ROUTINE
// ==============================================================================
bool runPreflightSanityCheck() {
    Serial.println("# [PST] Running 4-Second Pre-Flight Link Sanity Check...");
    uint32_t pstStart = millis();
    bool peerOk[4] = {false, false, false, false}; // Indices 1..3 for R2, R3, R4

    while (millis() - pstStart < PREFLIGHT_TIMEOUT_MS) {
        if (Config::ID == 1) {
            for (uint8_t target = 2; target <= 4; target++) {
                FullEdgeTelemetry dummy;
                if (performRangingPoll(target, dummy)) {
                    peerOk[target - 1] = true;
                }
                delay(15);
            }
            if (peerOk[1] && peerOk[2] && peerOk[3]) break;
        } else {
            performRangingListenSafe();
        }
        yield();
    }

    if (Config::ID == 1) {
        bool allOk = (peerOk[1] && peerOk[2] && peerOk[3]);
        if (allOk) {
            Serial.println("# [PST SUCCESS] All 3 peer robots (R2, R3, R4) responded cleanly!");
            return true;
        } else {
            Serial.printf("# [PST WARNING] Unresponsive peers: %s %s %s\n",
                          peerOk[1] ? "" : "ROBOT_2_DEAD!",
                          peerOk[2] ? "" : "ROBOT_3_DEAD!",
                          peerOk[3] ? "" : "ROBOT_4_DEAD!");
            return false;
        }
    }
    return true;
}

// ==============================================================================
// 6. SETUP
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1000);

    // Lock motors in safe electrical brake mode
    pinMode(PIN_MOTOR_L_IN1, OUTPUT); pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT); pinMode(PIN_MOTOR_R_IN2, OUTPUT);
    digitalWrite(PIN_MOTOR_L_IN1, HIGH); digitalWrite(PIN_MOTOR_L_IN2, HIGH);
    digitalWrite(PIN_MOTOR_R_IN1, HIGH); digitalWrite(PIN_MOTOR_R_IN2, HIGH);

    setupESPNow();
    setupUWB();

    // Initial temperature capture with validity guard
    float t_raw = DW1000Ng::getTemperature();
    cachedTempEsp = temperatureRead();
    if (t_raw > -30.0f && t_raw < 90.0f) {
        cachedTempUwb = t_raw;
    } else {
        cachedTempUwb = cachedTempEsp - 1.5f; // Safe empirical fallback for unburned OTP
    }

    bool pstPassed = runPreflightSanityCheck();
    if (!pstPassed && Config::ID == 1) {
        rgbLedWrite(PIN_STATUS_RGB, 60, 0, 0); // Blink Red on missing peer
    } else {
        rgbLedWrite(PIN_STATUS_RGB, 0, 0, 40); // Solid Blue (Active Measurement)
    }

    // 22-Column Full Metrology Telemetry Header
    if (Config::ID == 1) {
        Serial.println("timestamp_ms,frame_id,init_id,resp_id,status,carrier_int,cfo_ppm,tof_uncomp,tof_comp,dist_uncomp_m,dist_clock_m,dist_rssi_m,dist_calib_m,rssi_dbm,tTx1,tRx1,tRx2,tTx2,temp_uwb_init,temp_uwb_resp,temp_esp_init,temp_esp_resp");
    }
}

// ==============================================================================
// 7. DETERMINISTIC 200 ms TDMA LOOP (20 ms DEDICATED SLOTS)
// ==============================================================================
void loop() {
    uint32_t nowMs = millis();

    // Master Clock Synchronization (Robot 1)
    if (Config::ID == 1) {
        uint64_t curUs = micros();
        if (curUs - frameStartUs >= TDMA_FRAME_US) {
            frameStartUs = curUs;
            globalFrameId = globalFrameId + 1;

            SyncBeaconPacket sync = {globalFrameId, nowMs};
            esp_now_send(BROADCAST_MAC, (uint8_t*)&sync, sizeof(SyncBeaconPacket));
            isFrameSynced = true;
        }
    }

    if (!isFrameSynced) {
        performRangingListenSafe();
        yield();
        return;
    }

    uint32_t slotUs = (uint32_t)(micros() - frameStartUs);

    // --------------------------------------------------------------------------
    // SLOT 1 (10 to 30 ms): Link (1 -> 2)
    // --------------------------------------------------------------------------
    if (slotUs >= 15000 && slotUs < 30000) {
        if (Config::ID == 1) {
            static uint32_t lastPoll1 = 0;
            if (lastPoll1 != globalFrameId) {
                lastPoll1 = globalFrameId;
                FullEdgeTelemetry rec;
                performRangingPoll(2, rec);
                fleetTelemetry[0] = rec;
            }
        } else {
            performRangingListenSafe();
        }
    }
    // --------------------------------------------------------------------------
    // SLOT 2 (30 to 50 ms): Link (1 -> 3)
    // --------------------------------------------------------------------------
    else if (slotUs >= 30000 && slotUs < 50000) {
        if (Config::ID == 1) {
            static uint32_t lastPoll2 = 0;
            if (lastPoll2 != globalFrameId) {
                lastPoll2 = globalFrameId;
                FullEdgeTelemetry rec;
                performRangingPoll(3, rec);
                fleetTelemetry[1] = rec;
            }
        } else {
            performRangingListenSafe();
        }
    }
    // --------------------------------------------------------------------------
    // SLOT 3 (50 to 70 ms): Link (1 -> 4)
    // --------------------------------------------------------------------------
    else if (slotUs >= 50000 && slotUs < 70000) {
        if (Config::ID == 1) {
            static uint32_t lastPoll3 = 0;
            if (lastPoll3 != globalFrameId) {
                lastPoll3 = globalFrameId;
                FullEdgeTelemetry rec;
                performRangingPoll(4, rec);
                fleetTelemetry[2] = rec;
            }
        } else {
            performRangingListenSafe();
        }
    }
    // --------------------------------------------------------------------------
    // SLOT 4 (70 to 90 ms): Link (2 -> 3)
    // --------------------------------------------------------------------------
    else if (slotUs >= 70000 && slotUs < 90000) {
        if (Config::ID == 2) {
            static uint32_t lastPoll4 = 0;
            if (lastPoll4 != globalFrameId) {
                lastPoll4 = globalFrameId;
                FullEdgeTelemetry rec;
                performRangingPoll(3, rec);
                fleetTelemetry[3] = rec;
                esp_now_send(BROADCAST_MAC, (uint8_t*)&rec, sizeof(FullEdgeTelemetry));
            }
        } else {
            performRangingListenSafe();
        }
    }
    // --------------------------------------------------------------------------
    // SLOT 5 (90 to 110 ms): Link (2 -> 4)
    // --------------------------------------------------------------------------
    else if (slotUs >= 90000 && slotUs < 110000) {
        if (Config::ID == 2) {
            static uint32_t lastPoll5 = 0;
            if (lastPoll5 != globalFrameId) {
                lastPoll5 = globalFrameId;
                FullEdgeTelemetry rec;
                performRangingPoll(4, rec);
                fleetTelemetry[4] = rec;
                esp_now_send(BROADCAST_MAC, (uint8_t*)&rec, sizeof(FullEdgeTelemetry));
            }
        } else {
            performRangingListenSafe();
        }
    }
    // --------------------------------------------------------------------------
    // SLOT 6 (110 to 130 ms): Link (3 -> 4)
    // --------------------------------------------------------------------------
    else if (slotUs >= 110000 && slotUs < 130000) {
        if (Config::ID == 3) {
            static uint32_t lastPoll6 = 0;
            if (lastPoll6 != globalFrameId) {
                lastPoll6 = globalFrameId;
                FullEdgeTelemetry rec;
                performRangingPoll(4, rec);
                fleetTelemetry[5] = rec;
                esp_now_send(BROADCAST_MAC, (uint8_t*)&rec, sizeof(FullEdgeTelemetry));
            }
        } else {
            performRangingListenSafe();
        }
    }
    // --------------------------------------------------------------------------
    // SLOT 7 (140 to 180 ms): Gateway Streaming (Robot 1)
    // --------------------------------------------------------------------------
    else if (slotUs >= 140000 && slotUs < 180000) {
        if (Config::ID == 1) {
            static uint32_t lastPrintedFrame = 0;
            if (globalFrameId != lastPrintedFrame) {
                lastPrintedFrame = globalFrameId;

                for (uint8_t i = 0; i < 6; i++) {
                    FullEdgeTelemetry &e = fleetTelemetry[i];
                    Serial.printf("%lu,%lu,%u,%u,%u,%ld,%.2f,%lld,%lld,%.4f,%.4f,%.4f,%.4f,%.2f,%llu,%llu,%llu,%llu,%.1f,%.1f,%.1f,%.1f\n",
                                  (unsigned long)nowMs, (unsigned long)globalFrameId,
                                  e.initiatorId, e.responderId,
                                  e.status,
                                  (long)e.carrierIntegrator,
                                  e.cfoPpm,
                                  (long long)e.tofUncompTicks,
                                  (long long)e.tofCompTicks,
                                  e.distUncompM,
                                  e.distClockCompM,
                                  e.distRssiCompM,
                                  e.distCalibM,
                                  e.rssi_dbm,
                                  (unsigned long long)e.tTx1, (unsigned long long)e.tRx1,
                                  (unsigned long long)e.tRx2, (unsigned long long)e.tTx2,
                                  e.tempUwbInit, e.tempUwbResp,
                                  e.tempEspInit, e.tempEspResp);
                }
            }
        }
        inRxMode = false;
    }
    // --------------------------------------------------------------------------
    // SLOT 8 (185 to 195 ms): Periodic Temperature Sampling (Outside RF Windows)
    // --------------------------------------------------------------------------
    else if (slotUs >= 185000 && slotUs < 195000) {
        static uint32_t lastTempUpdateMs = 0;
        if (nowMs - lastTempUpdateMs >= 500) {
            lastTempUpdateMs = nowMs;
            float t_raw = DW1000Ng::getTemperature();
            cachedTempEsp = temperatureRead();
            if (t_raw > -30.0f && t_raw < 90.0f) {
                cachedTempUwb = t_raw;
            } else {
                cachedTempUwb = cachedTempEsp - 1.5f;
            }
        }
        if (Config::ID != 1) performRangingListenSafe();
    } else {
        if (Config::ID != 1) performRangingListenSafe();
    }
    yield();
}
