#include <Arduino.h>
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include "PinMap.h"
#include "RobotConfig.h"
#include "Types.h"

// Fundamental Physical Constants
constexpr double SPEED_OF_LIGHT = 299792458.0;
constexpr double TIME_UNIT_SEC  = 0.000000000015650040064103; // 15.65 ps per DW1000 tick

// 1.5 ms Turnaround Delay in DW1000 timer ticks
constexpr uint64_t SCHEDULED_REPLY_DELAY = 95846400ULL; 

static uint32_t exchangeSequence = 0;
volatile float latestCalibratedDistanceM = 0.0f;
volatile float latestRssi = 0.0f;
volatile uint32_t successfulExchanges = 0;

// Rolling Median Filter (Size 5) for ultra-stable millimeter output
class MedianFilter5 {
public:
    MedianFilter5() : _idx(0), _filled(false) {
        for (int i = 0; i < 5; i++) _buf[i] = 0.0f;
    }

    float update(float val) {
        _buf[_idx] = val;
        _idx = (_idx + 1) % 5;
        if (_idx == 0) _filled = true;

        uint8_t count = _filled ? 5 : _idx;
        if (count == 0) return val;

        float sorted[5];
        for (uint8_t i = 0; i < count; i++) sorted[i] = _buf[i];

        for (uint8_t i = 0; i < count - 1; i++) {
            for (uint8_t j = 0; j < count - i - 1; j++) {
                if (sorted[j] > sorted[j + 1]) {
                    float tmp = sorted[j];
                    sorted[j] = sorted[j + 1];
                    sorted[j + 1] = tmp;
                }
            }
        }
        return sorted[count / 2];
    }

private:
    float _buf[5];
    uint8_t _idx;
    bool _filled;
};

MedianFilter5 distanceFilter;

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
struct UWBPacket {
    char header[4];          // "POLL" or "RESP"
    uint32_t sequence;
    uint8_t rxTimestamp[5];  // 40-bit hardware Rx timestamp
    uint8_t txTimestamp[5];  // 40-bit hardware Tx timestamp
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

#if ROBOT_ID == 2
    DW1000Ng::clearReceiveStatus();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
#endif
}

void uwbRangingCycle() {
#if ROBOT_ID == 1
    // =========================================================================
    // Robot 1: Precision Initiator (POLL -> Await RESP -> Full Calibration Model)
    // =========================================================================
    UWBPacket pollPkt = {};
    memcpy(pollPkt.header, "POLL", 4);
    pollPkt.sequence = ++exchangeSequence;

    DW1000Ng::forceTRxOff();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::clearReceiveStatus();

    DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&pollPkt), sizeof(pollPkt));
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    while (!DW1000Ng::isTransmitDone()) { yield(); }
    DW1000Ng::clearTransmitStatus();
    uint64_t tTx1 = DW1000Ng::getTransmitTimestamp();

    // Open Receive Window for Response
    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);

    uint32_t startWait = millis();
    bool gotResp = false;

    while (millis() - startWait < 25) {
        if (DW1000Ng::isReceiveDone()) {
            DW1000Ng::clearReceiveStatus();

            size_t len = DW1000Ng::getReceivedDataLength();
            if (len >= sizeof(UWBPacket)) {
                UWBPacket respPkt;
                DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&respPkt), sizeof(respPkt));

                if (memcmp(respPkt.header, "RESP", 4) == 0) {
                    uint64_t tRx1 = DW1000Ng::getReceiveTimestamp();
                    latestRssi = (float)DW1000Ng::getReceivePower();

                    uint64_t tRx2 = read40BitTime(respPkt.rxTimestamp);
                    uint64_t tTx2 = read40BitTime(respPkt.txTimestamp);

                    int64_t tRound = (int64_t)((tRx1 - tTx1) & 0xFFFFFFFFFFULL);
                    int64_t tReply = (int64_t)((tTx2 - tRx2) & 0xFFFFFFFFFFULL);

                    int64_t tofTicks = (tRound - tReply) / 2;

                    if (tofTicks > 0 && tofTicks < 50000000ULL) {
                        double rawDist = (double)tofTicks * TIME_UNIT_SEC * SPEED_OF_LIGHT;
                        rawDist = DW1000NgRanging::correctRange((float)rawDist);

                        // 1. Invert Systematic Scale Factor & Hardware Offset
                        double calibratedDist = (rawDist - Config::UWB_CALIBRATION_OFFSET_M) / Config::UWB_SCALE_FACTOR;

                        // 2. Apply Piecewise RSSI Power Bias Compensation (Decawave APS011)
                        if (latestRssi > -68.0f) {
                            calibratedDist += 0.35f; // Near-field LNA saturation compensation
                        } else if (latestRssi < -82.0f) {
                            calibratedDist -= 0.10f; // Far-field attenuation compensation
                        }

                        // 3. Filter and Store Final Precision Distance
                        if (calibratedDist > 0.05 && calibratedDist < 30.0) {
                            latestCalibratedDistanceM = distanceFilter.update((float)calibratedDist);
                            successfulExchanges++;
                            gotResp = true;

                            // Stream Live Calibrated Telemetry
                            Serial.printf("[UWB] Calibrated Distance: %.2f m (%.1f cm) | RSSI: %.1f dBm | Ex: %u\n",
                                          latestCalibratedDistanceM,
                                          latestCalibratedDistanceM * 100.0f,
                                          latestRssi,
                                          successfulExchanges);
                        }
                    }
                }
            }
            break;
        }
        yield();
    }

    DW1000Ng::forceTRxOff();
    DW1000Ng::clearReceiveStatus();
    delay(50); // 20 Hz loop rate

#elif ROBOT_ID == 2
    // =========================================================================
    // Robot 2: Hardware-Scheduled Precision Responder
    // =========================================================================
    if (DW1000Ng::isReceiveDone()) {
        DW1000Ng::clearReceiveStatus();

        size_t len = DW1000Ng::getReceivedDataLength();
        if (len >= sizeof(UWBPacket)) {
            UWBPacket recvPkt;
            DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&recvPkt), sizeof(recvPkt));

            if (memcmp(recvPkt.header, "POLL", 4) == 0) {
                uint64_t tRx2 = DW1000Ng::getReceiveTimestamp();

                uint64_t tTx2 = (tRx2 + SCHEDULED_REPLY_DELAY) & 0xFFFFFFFE00ULL;

                UWBPacket respPkt = {};
                memcpy(respPkt.header, "RESP", 4);
                respPkt.sequence = recvPkt.sequence;
                write40BitTime(respPkt.rxTimestamp, tRx2);
                write40BitTime(respPkt.txTimestamp, tTx2);

                DW1000Ng::forceTRxOff();
                DW1000Ng::clearTransmitStatus();
                DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&respPkt), sizeof(respPkt));
                DW1000Ng::setDelayedTRX(respPkt.txTimestamp);
                DW1000Ng::startTransmit(TransmitMode::DELAYED);

                while (!DW1000Ng::isTransmitDone()) { yield(); }
                DW1000Ng::clearTransmitStatus();
                
                successfulExchanges++;
            }
        }

        DW1000Ng::clearReceiveStatus();
        DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
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

#if ROBOT_ID == 1
    Serial.println("==================================================");
    Serial.println("   ANJOMAN SWARM - PRECISION CALIBRATED UWB RANGING");
    Serial.printf("   Scale: %.6f | Offset: %.4f m\n", Config::UWB_SCALE_FACTOR, Config::UWB_CALIBRATION_OFFSET_M);
    Serial.println("==================================================");
#endif
}

void loop() {
    uwbRangingCycle();
}
