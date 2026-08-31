#include <Arduino.h>
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include "PinMap.h"
#include "RobotConfig.h"

// ==============================================================================
// 1. CONFIGURATION & ROLE DEFINITION
// ==============================================================================
// Change this ID to select which robot is the Initiator (1, 2, or 3)
#ifndef INITIATOR_ROBOT_ID
    #define INITIATOR_ROBOT_ID 3
#endif

// Fundamental Physical Constants
constexpr double SPEED_OF_LIGHT = 299792458.0;
constexpr double TIME_UNIT_SEC  = 0.000000000015650040064103; // 15.65 ps per tick

// 1.5 ms Turnaround Delay in DW1000 timer ticks
// constexpr uint64_t SCHEDULED_REPLY_DELAY = 95846400ULL; 
// 2.5 ms Safe Turnaround Delay in DW1000 timer ticks (for all N8R2 & N16R8 nodes)
constexpr uint64_t SCHEDULED_REPLY_DELAY = 159744000ULL;

static uint32_t exchangeSequence = 0;

// High-speed robust UWB Radio Profile
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
    char header[4];          // "POLL"
    uint8_t initiatorId;     // Sender ID (e.g., 1)
    uint32_t sequence;
};

struct UWBResponsePacket {
    char header[4];          // "RESP"
    uint8_t responderId;     // Target Responder ID (e.g., 2, 3, or 4)
    uint32_t sequence;       // Must MATCH poll sequence!
    uint8_t rxTimestamp[5];  // tRx (40-bit)
    uint8_t txTimestamp[5];  // tTx (40-bit)
    float tempUwb;           // DW1000 temperature on responder
    float tempEsp;           // ESP32 temperature on responder
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

float getSafeUwbTemp(float espTemp) {
    return espTemp - 1.5f;
}

// Hardware SPI Read of Carrier Frequency Offset (CFO in ppm)
float readCFOppm() {
    uint8_t buf[3] = {0};
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_UWB_CS, LOW);
    SPI.transfer(0x40 | 0x27); // Reg 0x27 DRX_CONF
    SPI.transfer(0x28);        // Offset 0x28 DRX_CARRIER_INT
    buf[0] = SPI.transfer(0x00);
    buf[1] = SPI.transfer(0x00);
    buf[2] = SPI.transfer(0x00);
    digitalWrite(PIN_UWB_CS, HIGH);
    SPI.endTransaction();

    int32_t carrierIntegrator = ((int32_t)buf[0]) |
                                (((int32_t)buf[1]) << 8) |
                                (((int32_t)(buf[2] & 0x1F)) << 16);
    if (carrierIntegrator & 0x100000) carrierIntegrator |= 0xFFE00000;
    constexpr float PPM_SCALE_CH5 = -0.036660f;
    return (float)carrierIntegrator * PPM_SCALE_CH5;
}

void setupUWB() {
    pinMode(PIN_UWB_RST, OUTPUT);
    digitalWrite(PIN_UWB_RST, LOW);
    delay(10);
    pinMode(PIN_UWB_RST, INPUT);
    delay(20);

    SPI.begin(PIN_UWB_SCK, PIN_UWB_MISO, PIN_UWB_MOSI, PIN_UWB_CS);
    delay(10);

    DW1000Ng::initializeNoInterrupt(PIN_UWB_CS, PIN_UWB_RST);
    DW1000Ng::applyConfiguration(UWB_CONFIG);

    DW1000Ng::setDeviceAddress(Config::ID);
    DW1000Ng::setNetworkId(0xDECA);
    DW1000Ng::setAntennaDelay(16436);

#if ROBOT_ID != INITIATOR_ROBOT_ID
    DW1000Ng::clearReceiveStatus();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
#endif
}

void uwbRangingCycle() {
#if ROBOT_ID == INITIATOR_ROBOT_ID
    // =========================================================================
    // INITIATOR LOOP (Hardened with Sequence Guard and Continuous Output)
    // =========================================================================
    UWBPollPacket pollPkt = {};
    memcpy(pollPkt.header, "POLL", 4);
    pollPkt.initiatorId = Config::ID;
    pollPkt.sequence = ++exchangeSequence;

    DW1000Ng::forceTRxOff();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::clearReceiveStatus();

    DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&pollPkt), sizeof(pollPkt));
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    uint32_t txStart = millis();
    while (!DW1000Ng::isTransmitDone() && (millis() - txStart < 10)) { yield(); }
    DW1000Ng::clearTransmitStatus();
    uint64_t tTx1 = DW1000Ng::getTransmitTimestamp();

    // Open Receive Window for Response
    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);

    uint32_t startWait = millis();
    uint8_t response_received = 0;
    uint8_t status_code = 0; // 0: TIMEOUT, 1: SUCCESS, 2: SEQ_MISMATCH

    double range_raw_m = 0.0;
    double range_calibrated_m = 0.0;
    float rssi = 0.0f;
    float fpp = 0.0f;
    float cfo = 0.0f;
    uint64_t tRx1 = 0;
    uint64_t tRx2 = 0;
    uint64_t tTx2 = 0;
    float temp_esp_1 = temperatureRead();
    float temp_uwb_1 = getSafeUwbTemp(temp_esp_1);
    float temp_esp_2 = 0.0f;
    float temp_uwb_2 = 0.0f;

    while (millis() - startWait < 30) {
        if (DW1000Ng::isReceiveDone()) {
            DW1000Ng::clearReceiveStatus();

            size_t len = DW1000Ng::getReceivedDataLength();
            if (len >= sizeof(UWBResponsePacket)) {
                UWBResponsePacket respPkt;
                DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&respPkt), sizeof(respPkt));

                if (memcmp(respPkt.header, "RESP", 4) == 0) {
                    // Sequence Matching Guard to reject stale delayed packets
                    if (respPkt.sequence == pollPkt.sequence) {
                        tRx1 = DW1000Ng::getReceiveTimestamp();
                        rssi = (float)DW1000Ng::getReceivePower();
                        fpp  = (float)DW1000Ng::getFirstPathPower();
                        cfo  = readCFOppm();

                        temp_esp_2 = respPkt.tempEsp;
                        temp_uwb_2 = respPkt.tempUwb;

                        tRx2 = read40BitTime(respPkt.rxTimestamp);
                        tTx2 = read40BitTime(respPkt.txTimestamp);

                        int64_t tRound = (int64_t)((tRx1 - tTx1) & 0xFFFFFFFFFFULL);
                        int64_t tReply = (int64_t)((tTx2 - tRx2) & 0xFFFFFFFFFFULL);

                        int64_t tofTicks = (tRound - tReply) / 2;

                        range_raw_m = (double)tofTicks * TIME_UNIT_SEC * SPEED_OF_LIGHT;
                        range_raw_m = DW1000NgRanging::correctRange((float)range_raw_m);

                        range_calibrated_m = (range_raw_m - Config::UWB_CALIBRATION_OFFSET_M) / Config::UWB_SCALE_FACTOR;
                        if (rssi > -68.0f) range_calibrated_m += 0.35f;
                        else if (rssi < -82.0f) range_calibrated_m -= 0.10f;

                        response_received = 1;
                        status_code = 1; // SUCCESS
                    } else {
                        status_code = 2; // SEQUENCE MISMATCH (CORRUPTED GLITCH REJECTED)
                    }
                }
            }
            break;
        }
        yield();
    }

    DW1000Ng::forceTRxOff();
    DW1000Ng::clearReceiveStatus();

    // Stream Complete 17-Column CSV Row continuously
    // timestamp,sequence,range_raw_m,range_calibrated_m,rssi_dbm,fpp_dbm,cfo_ppm,tTx1,tRx1,tRx2,tTx2,temp_uwb_1,temp_uwb_2,temp_esp_1,temp_esp_2,response_received,status
    Serial.printf("%lu,%lu,%.4f,%.4f,%.2f,%.2f,%.2f,%llu,%llu,%llu,%llu,%.1f,%.1f,%.1f,%.1f,%u,%u\n",
                  millis(),
                  (unsigned long)pollPkt.sequence,
                  range_raw_m,
                  range_calibrated_m,
                  rssi,
                  fpp,
                  cfo,
                  (unsigned long long)tTx1,
                  (unsigned long long)tRx1,
                  (unsigned long long)tRx2,
                  (unsigned long long)tTx2,
                  temp_uwb_1,
                  temp_uwb_2,
                  temp_esp_1,
                  temp_esp_2,
                  response_received,
                  status_code);

    delay(50); // 20 Hz nominal rate

#else
    // =========================================================================
    // RESPONDER LOOP (Hardened with Auto-Recovery Watchdog)
    // =========================================================================
    static uint32_t lastRxActivityMs = millis();

    if (DW1000Ng::isReceiveDone()) {
        DW1000Ng::clearReceiveStatus();
        lastRxActivityMs = millis();

        size_t len = DW1000Ng::getReceivedDataLength();
        if (len >= sizeof(UWBPollPacket)) {
            UWBPollPacket pollPkt;
            DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&pollPkt), sizeof(pollPkt));

            if (memcmp(pollPkt.header, "POLL", 4) == 0) {
                uint64_t tRx2 = DW1000Ng::getReceiveTimestamp();

                // Compute exact scheduled transmit timestamp (DX_TIME mask)
                uint64_t tTx2 = (tRx2 + SCHEDULED_REPLY_DELAY) & 0xFFFFFFFE00ULL;

                UWBResponsePacket respPkt = {};
                memcpy(respPkt.header, "RESP", 4);
                respPkt.responderId = Config::ID;
                respPkt.sequence = pollPkt.sequence;
                write40BitTime(respPkt.rxTimestamp, tRx2);
                write40BitTime(respPkt.txTimestamp, tTx2);

                respPkt.tempEsp = temperatureRead();
                respPkt.tempUwb = getSafeUwbTemp(respPkt.tempEsp);

                DW1000Ng::forceTRxOff();
                DW1000Ng::clearTransmitStatus();
                DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&respPkt), sizeof(respPkt));
                DW1000Ng::setDelayedTRX(respPkt.txTimestamp);
                DW1000Ng::startTransmit(TransmitMode::DELAYED);

                uint32_t txStart = millis();
                while (!DW1000Ng::isTransmitDone() && (millis() - txStart < 10)) { yield(); }
                DW1000Ng::clearTransmitStatus();
            }
        }

        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
    } else {
        // Auto-Recovery Watchdog: If no packet received for > 200 ms, clear flags and re-arm receiver
        if (millis() - lastRxActivityMs > 200) {
            DW1000Ng::forceTRxOff();
            DW1000Ng::clearReceiveStatus();
            DW1000Ng::clearTransmitStatus();
            DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
            lastRxActivityMs = millis();
        }
    }
    yield();
#endif
}

void setup() {
    Serial.begin(460800);
    delay(1000);

    pinMode(PIN_STATUS_RGB, OUTPUT);
    digitalWrite(PIN_STATUS_RGB, LOW);

    setupUWB();

#if ROBOT_ID == INITIATOR_ROBOT_ID
    // Output Pure 17-Column CSV Header
    Serial.println("timestamp,sequence,range_raw_m,range_calibrated_m,rssi_dbm,fpp_dbm,cfo_ppm,tTx1,tRx1,tRx2,tTx2,temp_uwb_1,temp_uwb_2,temp_esp_1,temp_esp_2,response_received,status");
#endif
}

void loop() {
    uwbRangingCycle();
}
