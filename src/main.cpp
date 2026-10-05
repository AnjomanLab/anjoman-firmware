// ==============================================================================
// Anjoman Firmware - Robot R2/R3/R4
// TDMA-based UWB ranging + ESP-NOW telemetry to gateway
// ==============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <SPI.h>

#include "PinMap.h"
#include "RobotConfig.h"
#include "DW1000Ng.hpp"
#include "DW1000NgConstants.hpp"
#include "DW1000NgRegisters.hpp"

// ==============================================================================
// 1. Constants
// ==============================================================================
static constexpr double   SPEED_OF_LIGHT        = 299792458.0;
static constexpr double   TIME_UNIT_SEC         = 1.0 / (499.2e6 * 128.0);
static constexpr uint64_t SCHEDULED_REPLY_DELAY = 159744000ULL; // 2.5 ms
static constexpr uint32_t TDMA_FRAME_US         = 200000;       // 200 ms
static constexpr uint32_t SLOT_US               = 15000;        // 15 ms

static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ==============================================================================
// 2. Packet structures (must match gateway.cpp exactly)
// ==============================================================================
#pragma pack(push, 1)

struct TDMASyncPacket {
    uint32_t frameId;
    uint32_t _pad;
};

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
    uint8_t  rxTimestamp[5];
    uint8_t  txTimestamp[5];
};

struct UWBBeaconEntry {
    uint8_t  peerId;
    uint8_t  ldeErr;
    uint16_t rawDistMm;
    uint16_t cleanDistMm;
    int8_t   rssi;
    int8_t   fpPower;
    uint16_t _pad;
};

struct SyncBeaconPacket {
    uint32_t frameId;
    uint32_t senderUptimeMs;
    uint8_t  senderId;
    uint8_t  senderSlot;
    uint8_t  nPeers;
    uint8_t  _pad;
    UWBBeaconEntry peers[2];
};

#pragma pack(pop)

// ==============================================================================
// 3. Global state
// ==============================================================================
static volatile uint32_t g_frameId      = 0;
static volatile uint64_t g_frameStartUs = 0;
static volatile bool     g_isSynced     = false;

struct PeerMetric {
    bool     valid;
    uint8_t  peerId;
    uint8_t  ldeErr;
    float    rawDist;
    float    cleanDist;
    float    rssi;
    float    fpPower;
};

static PeerMetric g_metrics[5] = {};

// ==============================================================================
// 4. ESP-NOW
// ==============================================================================
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    (void)mac;
    if (len == sizeof(TDMASyncPacket)) {
        TDMASyncPacket pkt;
        memcpy(&pkt, data, sizeof(pkt));
        g_frameId      = pkt.frameId;
        g_frameStartUs = micros();
        g_isSynced     = true;
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

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = 1;
    peer.encrypt = false;
    peer.ifidx   = WIFI_IF_STA;
    esp_now_add_peer(&peer);
}

// ==============================================================================
// 5. Time helpers
// ==============================================================================
static inline void write40(uint8_t *dest, uint64_t v) {
    for (int i = 0; i < 5; i++) dest[i] = (uint8_t)((v >> (8*i)) & 0xFF);
}

static inline uint64_t read40(const uint8_t *src) {
    uint64_t v = 0;
    for (int i = 0; i < 5; i++) v |= ((uint64_t)src[i]) << (8*i);
    return v;
}

// ==============================================================================
// 6. UWB setup - OLD PROVEN SPI CONFIGURATION
// ==============================================================================
static device_configuration_t UWB_CONFIG = {
    false,                      // extendedFrameLength
    true,                       // receiverAutoReenable
    true,                       // smartPower
    true,                       // frameCheck
    false,                      // nlos
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

void setupUWB() {
    pinMode(PIN_UWB_RST, OUTPUT);
    digitalWrite(PIN_UWB_RST, LOW);
    delay(10);
    pinMode(PIN_UWB_RST, INPUT);
    delay(25);

    // *** OLD SPI SETUP from your working code ***
    SPI.begin(PIN_UWB_SCK, PIN_UWB_MISO, PIN_UWB_MOSI, PIN_UWB_CS);
    delay(10);

    DW1000Ng::initializeNoInterrupt(PIN_UWB_CS, PIN_UWB_RST);
    DW1000Ng::applyConfiguration(UWB_CONFIG);
    DW1000Ng::setDeviceAddress(Config::ID);
    DW1000Ng::setNetworkId(0xDECA);
    DW1000Ng::setAntennaDelay(ANTENNA_DELAY_VAL);
    DW1000Ng::forceTRxOff();

    // DEV_ID diagnostic - THIS IS THE KEY TEST
    char buf[64];
    DW1000Ng::getPrintableDeviceIdentifier(buf);
    Serial.printf("[UWB] DEV_ID: %s\n", buf);
}

// ==============================================================================
// 7. Ranging
// ==============================================================================
static bool performRangingPoll(uint8_t targetId) {
    UWBPollPacket poll = {};
    memcpy(poll.header, "POLL", 4);
    poll.initiatorId = Config::ID;
    poll.targetId    = targetId;
    poll.sequence    = g_frameId;

    DW1000Ng::forceTRxOff();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::clearReceiveStatus();
    DW1000Ng::setTransmitData((byte*)&poll, sizeof(poll));
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    uint32_t t0 = millis();
    while (!DW1000Ng::isTransmitDone()) {
        if (millis() - t0 > 6) { DW1000Ng::forceTRxOff(); return false; }
        yield();
    }
    DW1000Ng::clearTransmitStatus();
    uint64_t tTx1 = DW1000Ng::getTransmitTimestamp() & 0xFFFFFFFFFFULL;

    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
    t0 = millis();
    bool ok = false;

    while (millis() - t0 < 10) {
        if (DW1000Ng::isReceiveDone()) {
            DW1000Ng::clearReceiveStatus();
            size_t len = DW1000Ng::getReceivedDataLength();
            if (len >= sizeof(UWBResponsePacket)) {
                UWBResponsePacket resp;
                DW1000Ng::getReceivedData((byte*)&resp, sizeof(resp));

                if (memcmp(resp.header, "RESP", 4) == 0 &&
                    resp.responderId == targetId &&
                    resp.targetId == Config::ID) {

                    uint64_t tRx1 = DW1000Ng::getReceiveTimestamp() & 0xFFFFFFFFFFULL;
                    uint64_t tRx2 = read40(resp.rxTimestamp);
                    uint64_t tTx2 = read40(resp.txTimestamp);

                    int64_t tRound = (int64_t)((tRx1 - tTx1) & 0xFFFFFFFFFFULL);
                    int64_t tReply = (int64_t)((tTx2 - tRx2) & 0xFFFFFFFFFFULL);

                    if (tReply < tRound) {
                        int64_t tof  = (tRound - tReply) / 2;
                        float dist   = (float)((double)tof * TIME_UNIT_SEC * SPEED_OF_LIGHT);

                        auto diag = DW1000Ng::getChannelDiagnostics();

                        PeerMetric &m = g_metrics[targetId];
                        m.valid     = true;
                        m.peerId    = targetId;
                        m.ldeErr    = diag.ldeError;
                        m.rawDist   = dist;
                        m.cleanDist = dist;
                        m.rssi      = (float)DW1000Ng::getReceivePower();
                        m.fpPower   = (float)DW1000Ng::getFirstPathPower();
                        ok = true;
                    }
                }
            }
            break;
        }
        if (DW1000Ng::isReceiveFailed() || DW1000Ng::isReceiveTimeout()) {
            DW1000Ng::clearReceiveStatus();
            break;
        }
        yield();
    }

    DW1000Ng::forceTRxOff();
    return ok;
}

static bool performRangingListen(uint32_t timeoutMs) {
    DW1000Ng::forceTRxOff();
    DW1000Ng::clearReceiveStatus();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(0);
    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);

    uint32_t t0 = millis();
    bool ok = false;

    while (millis() - t0 < timeoutMs) {
        if (DW1000Ng::isReceiveDone()) {
            DW1000Ng::clearReceiveStatus();
            size_t len = DW1000Ng::getReceivedDataLength();
            if (len >= sizeof(UWBPollPacket)) {
                UWBPollPacket poll;
                DW1000Ng::getReceivedData((byte*)&poll, sizeof(poll));

                if (memcmp(poll.header, "POLL", 4) == 0 && poll.targetId == Config::ID) {
                    uint64_t tRx2 = DW1000Ng::getReceiveTimestamp() & 0xFFFFFFFFFFULL;
                    uint64_t tTx2 = (tRx2 + SCHEDULED_REPLY_DELAY) & 0xFFFFFFFE00ULL;

                    UWBResponsePacket resp = {};
                    memcpy(resp.header, "RESP", 4);
                    resp.responderId = Config::ID;
                    resp.targetId    = poll.initiatorId;
                    resp.sequence    = poll.sequence;
                    write40(resp.rxTimestamp, tRx2);
                    write40(resp.txTimestamp, tTx2);

                    DW1000Ng::forceTRxOff();
                    DW1000Ng::clearTransmitStatus();
                    DW1000Ng::setTransmitData((byte*)&resp, sizeof(resp));
                    DW1000Ng::setDelayedTRX(resp.txTimestamp);
                    DW1000Ng::startTransmit(TransmitMode::DELAYED);

                    uint32_t tw = millis();
                    while (!DW1000Ng::isTransmitDone()) {
                        if (millis() - tw > 8) break;
                        yield();
                    }
                    DW1000Ng::clearTransmitStatus();
                    ok = true;
                }
            }
            break;
        }
        if (DW1000Ng::isReceiveFailed()) {
            DW1000Ng::clearReceiveStatus();
        }
        yield();
    }

    DW1000Ng::forceTRxOff();
    return ok;
}

// ==============================================================================
// 8. Beacon to gateway
// ==============================================================================
static void sendBeacon(uint8_t slot) {
    SyncBeaconPacket pkt = {};
    pkt.frameId        = g_frameId;
    pkt.senderUptimeMs = millis();
    pkt.senderId       = Config::ID;
    pkt.senderSlot     = slot;
    pkt.nPeers         = 0;

    for (uint8_t p = 2; p <= 4; p++) {
        if (p == Config::ID) continue;
        if (pkt.nPeers >= 2) break;
        const PeerMetric &m = g_metrics[p];
        if (m.valid) {
            UWBBeaconEntry &e = pkt.peers[pkt.nPeers++];
            e.peerId      = m.peerId;
            e.ldeErr      = m.ldeErr;
            e.rawDistMm   = (uint16_t)constrain(m.rawDist   * 1000.0f, 0.0f, 65535.0f);
            e.cleanDistMm = (uint16_t)constrain(m.cleanDist * 1000.0f, 0.0f, 65535.0f);
            e.rssi        = (int8_t)constrain(m.rssi, -128.0f, 127.0f);
            e.fpPower     = (int8_t)constrain(m.fpPower, -128.0f, 127.0f);
        }
    }

    esp_now_send(BROADCAST_MAC, (uint8_t*)&pkt, sizeof(pkt));
}

// ==============================================================================
// 9. Slot table for 3-robot fleet
// ==============================================================================
enum class SlotType : uint8_t {
    IDLE,
    POLL,
    BEACON
};

struct SlotAction {
    uint8_t  actor;
    uint8_t  target;
    SlotType type;
};

static const SlotAction SLOT_TABLE[12] = {
    {2, 3, SlotType::POLL},     // 0: R2 -> R3
    {2, 4, SlotType::POLL},     // 1: R2 -> R4
    {2, 0, SlotType::BEACON},   // 2: R2 -> gateway (with fresh data)
    {3, 2, SlotType::POLL},     // 3: R3 -> R2
    {3, 4, SlotType::POLL},     // 4: R3 -> R4
    {3, 0, SlotType::BEACON},   // 5: R3 -> gateway
    {4, 2, SlotType::POLL},     // 6: R4 -> R2
    {4, 3, SlotType::POLL},     // 7: R4 -> R3
    {4, 0, SlotType::BEACON},   // 8: R4 -> gateway
    {0, 0, SlotType::IDLE},     // 9
    {0, 0, SlotType::IDLE},     // 10
    {0, 0, SlotType::IDLE},     // 11
};

static void handleSlot(uint8_t slotIdx) {
    if (slotIdx >= 12) slotIdx = 11;
    const SlotAction &a = SLOT_TABLE[slotIdx];

    switch (a.type) {
        case SlotType::IDLE:
            performRangingListen(14);
            break;

        case SlotType::POLL:
            if (Config::ID == a.actor) {
                performRangingPoll(a.target);
            } else {
                // target or third robot - both listen
                performRangingListen(14);
            }
            break;

        case SlotType::BEACON:
            if (Config::ID == a.actor) {
                sendBeacon(slotIdx);
            }
            break;
    }
}

// ==============================================================================
// 10. Setup / Loop
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1000);

    Serial.printf("\n[BOOT] Robot %u\n", Config::ID);

    // Motors coast (disabled)
    pinMode(PIN_MOTOR_L_IN1, OUTPUT); pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_IN1, OUTPUT); pinMode(PIN_MOTOR_R_IN2, OUTPUT);
    digitalWrite(PIN_MOTOR_L_IN1, LOW); digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN1, LOW); digitalWrite(PIN_MOTOR_R_IN2, LOW);

    setupESPNow();
    setupUWB();

    Serial.printf("[READY] Robot %u\n", Config::ID);
}

void loop() {
    if (!g_isSynced) {
        performRangingListen(14);
        yield();
        return;
    }

    uint32_t slotUs = (uint32_t)(micros() - g_frameStartUs);
    uint8_t  slotIdx = slotUs / SLOT_US;

    if (slotIdx >= 12) slotIdx = 11;

    static uint32_t lastFrame = UINT32_MAX;
    static uint8_t  lastSlot  = 0xFF;

    if (g_frameId != lastFrame || slotIdx != lastSlot) {
        lastFrame = g_frameId;
        lastSlot  = slotIdx;
        handleSlot(slotIdx);
    }

    yield();
}
