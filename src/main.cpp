#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <SPI.h>
#include <Wire.h>

#include "PinMap.h"
#include "RobotConfig.h"
#include "Types.h"
#include "DW1000Ng.hpp"
#include "DW1000NgConstants.hpp"
#include "DW1000NgRegisters.hpp"
#include "TDMAEngine.h"
#include "TDMAConfig.h"
#include "UWBPreprocessor.h"

// ==============================================================================
// 1. PHYSICAL CONSTANTS AND TIME CONVERSIONS
// ==============================================================================
static constexpr double   TIME_UNIT_SEC      = 1.0 / (499.2e6 * 128.0);
static constexpr double   SPEED_OF_LIGHT     = 299792458.0;
static constexpr uint64_t REPLY_DELAY_TICKS  = 159744000ULL; // ~2.5 ms

static inline uint64_t read40BitTime(const uint8_t *b) {
    uint64_t t = 0;
    for (int i = 0; i < 5; ++i) {
        t |= (static_cast<uint64_t>(b[i]) << (i * 8));
    }
    return t;
}

static inline void write40BitTime(uint8_t *b, uint64_t t) {
    for (int i = 0; i < 5; ++i) {
        b[i] = static_cast<uint8_t>((t >> (i * 8)) & 0xFF);
    }
}

static inline void setDelayedTime40(uint64_t t) {
    uint8_t buf[5];
    write40BitTime(buf, t & 0xFFFFFFFFFFULL);
    DW1000Ng::setDelayedTRX(buf);
}

// ==============================================================================
// 2. STATE AND INSTANCES
// ==============================================================================
static SPIClass uwbSPI(FSPI);
TDMAEngine tdma;

static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static UWBRangeMetric g_uwbMetrics[Config::FLEET_SIZE + 2] = {};
static float g_chipTempUwb = 23.0f;
static float g_chipVbatUwb = 3.3f;
static uint32_t g_lastUwbMonitorMs = 0;

static device_configuration_t UWB_CONFIG = {
    false,
    true,
    false,
    true,
    false,
    SFDMode::STANDARD_SFD,
    Channel::CHANNEL_5,
    DataRate::RATE_850KBPS,
    PulseFrequency::FREQ_16MHZ,
    PreambleLength::LEN_256,
    PreambleCode::CODE_3
};

struct SlotAction {
    uint8_t actor;
    uint8_t target;
    bool    isBeacon;
};

static const SlotAction SLOT_TABLE[16] = {
    {2, 0, true},  {2, 3, false}, {2, 4, false}, {0, 0, false},
    {3, 0, true},  {3, 2, false}, {3, 4, false}, {0, 0, false},
    {4, 0, true},  {4, 2, false}, {4, 3, false}, {0, 0, false},
    {0, 0, false}, {0, 0, false}, {0, 0, false}, {0, 0, false}
};

bool isActive(uint8_t id) {
    return (id >= 2 && id <= 4);
}

// ==============================================================================
// 3. ROBUST RANGING ENGINE
// ==============================================================================
bool performRangingPoll(uint8_t targetPeerId) {
    if (targetPeerId < 1 || targetPeerId > 4 || targetPeerId == Config::ID) return false;
    if (!isActive(targetPeerId)) return false;

    UWBPollPacket pollPkt = {};
    pollPkt.initiatorId = Config::ID;
    pollPkt.targetId    = targetPeerId;
    pollPkt.sequence    = tdma.getFrameId();

    DW1000Ng::forceTRxOff();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::clearReceiveStatus();
    DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&pollPkt), sizeof(pollPkt));

    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(15000);
    DW1000Ng::setWait4Response(500);

    delayMicroseconds(2000);
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    const uint32_t t_tx = millis();
    while (!DW1000Ng::isTransmitDone()) {
        if (millis() - t_tx > 6) {
            DW1000Ng::forceTRxOff();
            return false;
        }
        yield();
    }
    DW1000Ng::clearTransmitStatus();
    const uint64_t tTx1 = DW1000Ng::getTransmitTimestamp() & 0xFFFFFFFFFFULL;

    const uint32_t t_rx = millis();
    bool success = false;

    while (millis() - t_rx < 12) {
        if (DW1000Ng::isReceiveDone()) {
            DW1000Ng::clearReceiveStatus();
            const size_t len = DW1000Ng::getReceivedDataLength();

            if (len >= sizeof(UWBResponsePacket)) {
                UWBResponsePacket respPkt;
                DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&respPkt), sizeof(respPkt));

                if (respPkt.responderId == targetPeerId && respPkt.targetId == Config::ID) {
                    const uint64_t tRx1 = DW1000Ng::getReceiveTimestamp() & 0xFFFFFFFFFFULL;
                    const uint64_t tRx2 = read40BitTime(respPkt.rxTimestamp);
                    const uint64_t tTx2 = read40BitTime(respPkt.txTimestamp);

                    const int64_t tRound = static_cast<int64_t>((tRx1 - tTx1) & 0xFFFFFFFFFFULL);
                    const int64_t tReply = static_cast<int64_t>((tTx2 - tRx2) & 0xFFFFFFFFFFULL);

                    if (tReply < tRound) {
                        float cfoRatio = DW1000Ng::getClockOffsetRatio();
                        if (cfoRatio > 0.0001f)  cfoRatio = 0.0001f;
                        if (cfoRatio < -0.0001f) cfoRatio = -0.0001f;

                        const double tReplyCorr = static_cast<double>(tReply) * (1.0 - static_cast<double>(cfoRatio));
                        const int64_t tofTicks  = static_cast<int64_t>((static_cast<double>(tRound) - tReplyCorr) / 2.0);

                        const float distRawM = static_cast<float>(tofTicks * TIME_UNIT_SEC * SPEED_OF_LIGHT);
                        const float cleanM   = UWBPreprocessor::correctRawDistance(
                            Config::ID, targetPeerId, distRawM, 0.0f, 0.0f);

                        const auto diag = DW1000Ng::getChannelDiagnostics();

                        UWBRangeMetric &m = g_uwbMetrics[targetPeerId];
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
                        m.rssi       = static_cast<float>(DW1000Ng::getReceivePower());
                        m.fpPower    = static_cast<float>(DW1000Ng::getFirstPathPower());
                        m.respTemp   = respPkt.tempUwb;

                        success = true;
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
    return success;
}

bool performRangingListenBlocking(uint32_t timeoutMs) {
    DW1000Ng::forceTRxOff();
    DW1000Ng::clearReceiveStatus();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::setReceiveFrameWaitTimeoutPeriod(0);
    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);

    const uint32_t t_start = millis();
    bool success = false;

    while (millis() - t_start < timeoutMs) {
        if (DW1000Ng::isReceiveDone()) {
            DW1000Ng::clearReceiveStatus();
            const size_t len = DW1000Ng::getReceivedDataLength();

            if (len >= sizeof(UWBPollPacket)) {
                UWBPollPacket pollPkt;
                DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&pollPkt), sizeof(pollPkt));

                if (pollPkt.targetId == Config::ID) {
                    const uint64_t tRx2 = DW1000Ng::getReceiveTimestamp() & 0xFFFFFFFFFFULL;
                    const uint64_t tTx2 = (tRx2 + REPLY_DELAY_TICKS) & 0xFFFFFFFE00ULL;

                    UWBResponsePacket respPkt = {};
                    respPkt.responderId = Config::ID;
                    respPkt.targetId    = pollPkt.initiatorId;
                    respPkt.tempUwb     = g_chipTempUwb;

                    write40BitTime(respPkt.rxTimestamp, tRx2);
                    write40BitTime(respPkt.txTimestamp, tTx2);

                    DW1000Ng::forceTRxOff();
                    DW1000Ng::clearTransmitStatus();
                    DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&respPkt), sizeof(respPkt));

                    setDelayedTime40(tTx2);
                    DW1000Ng::startTransmit(TransmitMode::DELAYED);

                    const uint32_t t_wait_tx = millis();
                    while (!DW1000Ng::isTransmitDone()) {
                        if (millis() - t_wait_tx > 6) {
                            DW1000Ng::forceTRxOff();
                            break;
                        }
                        yield();
                    }

                    if (DW1000Ng::isTransmitDone()) {
                        DW1000Ng::clearTransmitStatus();
                        success = true;
                    }
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
    return success;
}

// ==============================================================================
// 4. BEACONING TO GATEWAY (100% MATCHING TYPES.H)
// ==============================================================================
static void sendBeacon(uint8_t slot) {
    SyncBeaconPacket pkt = {};
    pkt.senderId            = Config::ID;
    pkt.senderSlot          = slot;
    pkt.frameId             = tdma.getFrameId();
    pkt.senderUptimeMs      = millis();
    pkt.maneuverStartFrame  = 0;
    pkt.vbat                = g_chipVbatUwb;
    pkt.senderTempUwb       = g_chipTempUwb;
    pkt.posX                = 0.0f;
    pkt.posY                = 0.0f;
    pkt.headingRad          = 0.0f;

    uint8_t count = 0;
    for (uint8_t p = 1; p <= 4; ++p) {
        if (p == Config::ID) continue;
        if (count >= 2) break; // Maximum 2 peers in SyncBeaconPacket::peers[2]
        const UWBRangeMetric &m = g_uwbMetrics[p];
        if (m.valid) {
            UWBBeaconEntry &e = pkt.peers[count++];
            e.peerId      = m.peerId;
            e.ldeErr      = m.ldeErr;
            e.rawDistMm   = static_cast<uint16_t>(constrain(m.rawDist   * 1000.0f, 0.0f, 65535.0f));
            e.cleanDistMm = static_cast<uint16_t>(constrain(m.cleanDist * 1000.0f, 0.0f, 65535.0f));
            e.rssi        = static_cast<int8_t>(constrain(m.rssi, -128.0f, 127.0f));
            e.fpPower     = static_cast<int8_t>(constrain(m.fpPower, -128.0f, 127.0f));
            e.respTemp    = m.respTemp;
            e.cfoPpm      = 0.0f;
        }
    }
    pkt.nPeers = count;
    esp_now_send(BROADCAST_MAC, reinterpret_cast<uint8_t*>(&pkt), sizeof(pkt));
}

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    (void)mac;
    if (len < static_cast<int>(sizeof(SyncBeaconPacket))) return;

    SyncBeaconPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    tdma.onSyncReceived(pkt.senderId, pkt.frameId, pkt.senderUptimeMs,
                        pkt.senderSlot, millis(), micros());
}

static void handleSlot(uint32_t slot) {
    if (slot >= 16) return;
    const SlotAction &a = SLOT_TABLE[slot];
    if (a.isBeacon) {
        if (a.actor == Config::ID) {
            sendBeacon(static_cast<uint8_t>(slot));
        }
    } else {
        if (a.actor == Config::ID) {
            performRangingPoll(a.target);
        } else if (a.target == Config::ID) {
            performRangingListenBlocking(13);
        }
    }
}

// ==============================================================================
// 5. SYSTEM INITIALIZATION
// ==============================================================================
void setupUWB() {
    pinMode(PIN_UWB_CS, OUTPUT);
    digitalWrite(PIN_UWB_CS, HIGH);

    if (PIN_UWB_RST != 0xFF) {
        pinMode(PIN_UWB_RST, OUTPUT);
        digitalWrite(PIN_UWB_RST, LOW);
        delay(5);
        pinMode(PIN_UWB_RST, INPUT);
        delay(25);
    }

    uwbSPI.begin(PIN_UWB_SCK, PIN_UWB_MISO, PIN_UWB_MOSI, -1);
    DW1000Ng::initialize(PIN_UWB_CS, PIN_UWB_IRQ, PIN_UWB_RST, uwbSPI);
    DW1000Ng::applyConfiguration(UWB_CONFIG);
    DW1000Ng::setDeviceAddress(Config::ID);
    DW1000Ng::setNetworkId(0xDECA);
    DW1000Ng::setAntennaDelay(Config::ANTENNA_DELAY_VAL);
    DW1000Ng::forceTRxOff();
}

void setup() {
    Serial.begin(460800);
    delay(1000);

    setupUWB();

    WiFi.mode(WIFI_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(onDataRecv);
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, BROADCAST_MAC, 6);
        peer.channel = 1;
        peer.encrypt = false;
        esp_now_add_peer(&peer);
    }

    Serial.printf("\n[STATIC TEST] Robot %u Ready. UWB + Telemetry Active.\n", Config::ID);
}

void loop() {
    const uint32_t nowMs = millis();
    tdma.tick(nowMs, micros());

    if (nowMs - g_lastUwbMonitorMs >= 1000) {
        g_lastUwbMonitorMs = nowMs;
        float t = 0.0f, v = 0.0f;
        DW1000Ng::getTemperatureAndBatteryVoltage(t, v);
        if (t > 0.0f && t < 100.0f) g_chipTempUwb = t;
        if (v > 2.0f && v < 4.5f)   g_chipVbatUwb = v;
    }

    const uint32_t frame = tdma.getFrameId();
    const uint32_t slot  = tdma.getCurrentSlotIndex();

    static uint32_t lastActedFrame = UINT32_MAX;
    static uint32_t lastActedSlot  = UINT32_MAX;

    if (frame != lastActedFrame || slot != lastActedSlot) {
        lastActedFrame = frame;
        lastActedSlot  = slot;
        handleSlot(slot);
    }
}
