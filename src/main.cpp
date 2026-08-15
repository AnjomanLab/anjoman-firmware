#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include "PinMap.h"
#include "RobotConfig.h"
#include "Types.h"
#include "ESPNowBroadcast.h"

WiFiServer netcatServer(Config::NETCAT_PORT);
WiFiClient netcatClient;

ESPNowBroadcast comms;
TaskHandle_t commsTaskHandle = nullptr;
static uint32_t telemetrySeq = 0;

volatile float latestDistanceM = 0.0f;
volatile float latestRssi = 0.0f;
volatile uint32_t successfulExchanges = 0;

constexpr double SPEED_OF_LIGHT = 299792458.0;
constexpr double TIME_UNIT_SEC  = 0.000000000015650040064103;

// 1.5 ms Turnaround Delay in DW1000 timer ticks
constexpr uint64_t SCHEDULED_REPLY_DELAY = 95846400ULL; 

// Rolling Median Filter (Size 5)
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

void setupWiFi() {
    Serial.printf("[WiFi] Connecting to SSID: %s with Static IP: %s\n", 
                  Config::WIFI_SSID, Config::STATIC_IP.toString().c_str());

    WiFi.mode(WIFI_AP_STA);
    if (!WiFi.config(Config::STATIC_IP, Config::GATEWAY, Config::SUBNET)) {
        Serial.println("[WiFi] Static IP configuration failed!");
    }

    WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);

    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] Connected! IP: %s | RSSI: %d dBm\n", 
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
        netcatServer.begin();
        Serial.printf("[Netcat] Stream listening on port %d. Connect using: nc %s %d\n",
                      Config::NETCAT_PORT, WiFi.localIP().toString().c_str(), Config::NETCAT_PORT);
    } else {
        Serial.println("\n[WiFi] Operating in standalone mesh mode.");
    }
}

void setupUWB() {
    Serial.println("\n[UWB] Initializing DW1000 on Dedicated SPI2 Bus...");

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

    char devId[128];
    DW1000Ng::getPrintableDeviceIdentifier(devId);
    Serial.printf("[UWB] Hardware Ready: %s | Node ID: %d\n", devId, Config::ID);

#if ROBOT_ID == 2
    DW1000Ng::clearReceiveStatus();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
#endif
}

void handlePeerTelemetry(const TelemetryPacket &pkt) {
    if (pkt.senderId == Config::ID) return;

    if (pkt.measuredDistanceM > 0.0f) {
        latestDistanceM = pkt.measuredDistanceM;
        latestRssi = pkt.signalRssi;
    }
}

void ssTwrLoop() {
#if ROBOT_ID == 1
    // =========================================================================
    // Robot 1: SS-TWR Initiator (Sends POLL -> Awaits RESP)
    // =========================================================================
    UWBPacket pollPkt = {};
    memcpy(pollPkt.header, "POLL", 4);
    pollPkt.sequence = telemetrySeq;

    DW1000Ng::forceTRxOff();
    DW1000Ng::clearTransmitStatus();
    DW1000Ng::clearReceiveStatus();

    DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&pollPkt), sizeof(pollPkt));
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    while (!DW1000Ng::isTransmitDone()) { yield(); }
    DW1000Ng::clearTransmitStatus();
    uint64_t tTx1 = DW1000Ng::getTransmitTimestamp();

    // Open receive window for response
    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);

    uint32_t startWait = millis();
    bool gotResp = false;

    while (millis() - startWait < 30) {
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

                        // Subtract identified systematic antenna offset
                        double calibratedDist = rawDist - Config::UWB_CALIBRATION_OFFSET_M;

                        if (calibratedDist > 0.02 && calibratedDist < 30.0) {
                            latestDistanceM = distanceFilter.update((float)calibratedDist);
                            successfulExchanges++;
                            gotResp = true;
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
    delay(50); // 20 Hz loop

#elif ROBOT_ID == 2
    // =========================================================================
    // Robot 2: SS-TWR Hardware-Scheduled Responder
    // =========================================================================
    if (DW1000Ng::isReceiveDone()) {
        DW1000Ng::clearReceiveStatus();

        size_t len = DW1000Ng::getReceivedDataLength();
        if (len >= sizeof(UWBPacket)) {
            UWBPacket recvPkt;
            DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&recvPkt), sizeof(recvPkt));

            if (memcmp(recvPkt.header, "POLL", 4) == 0) {
                uint64_t tRx2 = DW1000Ng::getReceiveTimestamp();
                latestRssi = (float)DW1000Ng::getReceivePower();

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

void commsTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);

    uint32_t lastNetcatLogMs = 0;

    while (true) {
        if (!netcatClient || !netcatClient.connected()) {
            netcatClient = netcatServer.accept();
        }

        TelemetryPacket packet = {};
        packet.senderId = Config::ID;
        packet.timestampMs = millis();
        packet.sequenceId = telemetrySeq++;
        packet.measuredDistanceM = latestDistanceM;
        packet.signalRssi = latestRssi;

        // 1. Broadcast over ESP-NOW to peer
        comms.sendTelemetry(packet);

        // 2. Stream calibrated distance over Netcat (2 Hz)
        if (millis() - lastNetcatLogMs >= 500) {
            lastNetcatLogMs = millis();

            if (netcatClient && netcatClient.connected()) {
                if (latestDistanceM > 0.0f) {
                    netcatClient.printf("[R%d] Calibrated Distance: %.2f m (%.1f cm) | RSSI: %.1f dBm | Ex: %u\n",
                                        packet.senderId, packet.measuredDistanceM,
                                        packet.measuredDistanceM * 100.0f,
                                        packet.signalRssi, successfulExchanges);
                } else {
                    netcatClient.printf("[R%d] UWB Ranging Initializing... (Exchanges: %u)\n",
                                        packet.senderId, successfulExchanges);
                }
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("==================================================");
    Serial.printf("        ANJOMAN FIRMWARE - CALIBRATED UWB (ROBOT %d)\n", Config::ID);
    Serial.println("==================================================");

    pinMode(PIN_STATUS_RGB, OUTPUT);
    digitalWrite(PIN_STATUS_RGB, LOW);

    setupWiFi();

    if (!comms.begin(Config::ID)) {
        Serial.println("[ERROR] ESP-NOW Init Failed!");
    }
    comms.registerTelemetryCallback(handlePeerTelemetry);

    setupUWB();

    xTaskCreatePinnedToCore(
        commsTask,
        "CommsTask",
        4096,
        nullptr,
        1,
        &commsTaskHandle,
        0
    );

    Serial.println("[System] Calibrated UWB Ranging Engine Ready.");
}

void loop() {
    ssTwrLoop();
}
