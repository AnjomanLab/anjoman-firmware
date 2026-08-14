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
volatile uint32_t pingPongCount = 0;

// High-speed, robust UWB Radio Profile
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
    char header[4];          // "PING" or "PONG"
    uint32_t sequence;
    uint32_t replyDelayTicks;// Turnaround time on responder
};
#pragma pack(pop)

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
        Serial.println("\n[WiFi] Connection timed out. Operating in standalone mode.");
    }
}

void setupUWB() {
    Serial.println("\n[UWB] Initializing DW1000...");

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
    Serial.printf("[UWB] Ready: %s | Node ID: %d\n", devId, Config::ID);

#if ROBOT_ID == 2
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

void uwbLoop() {
#if ROBOT_ID == 1
    // =========================================================================
    // Robot 1: TWR Initiator
    // =========================================================================
    UWBPacket pingPkt;
    memcpy(pingPkt.header, "PING", 4);
    pingPkt.sequence = ++pingPongCount;
    pingPkt.replyDelayTicks = 0;

    DW1000Ng::forceTRxOff();
    DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&pingPkt), sizeof(pingPkt));
    DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

    while (!DW1000Ng::isTransmitDone()) { yield(); }
    DW1000Ng::clearTransmitStatus();
    uint64_t txPollTime = DW1000Ng::getTransmitTimestamp();

    // Listen for PONG response
    DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);

    uint32_t startWait = millis();
    bool gotPong = false;

    while (millis() - startWait < 30) {
        if (DW1000Ng::isReceiveDone()) {
            DW1000Ng::clearReceiveStatus();

            size_t dataLen = DW1000Ng::getReceivedDataLength();
            if (dataLen >= sizeof(UWBPacket)) {
                UWBPacket pongPkt;
                DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&pongPkt), sizeof(pongPkt));

                if (memcmp(pongPkt.header, "PONG", 4) == 0) {
                    uint64_t rxPongTime = DW1000Ng::getReceiveTimestamp();
                    latestRssi = (float)DW1000Ng::getReceivePower();

                    // Calculate Time of Flight eliminating responder delay
                    uint32_t roundTripTicks = (uint32_t)(rxPongTime - txPollTime);
                    uint32_t replyTicks = pongPkt.replyDelayTicks;

                    int32_t tofTicks = ((int32_t)roundTripTicks - (int32_t)replyTicks) / 2;
                    
                    if (tofTicks > 0) {
                        double tofSec = (double)tofTicks * 0.000000000015650040064103;
                        double dist = tofSec * 299792458.0;

                        // Antenna delay offset correction
                        dist = dist - 0.75; // Baseline antenna calibration offset
                        if (dist < 0.05) dist = 0.05;

                        latestDistanceM = (float)dist;
                        gotPong = true;
                    }
                }
            }
            break;
        }
        yield();
    }

    if (!gotPong) {
        DW1000Ng::forceTRxOff();
    }

    delay(60); // ~15 Hz

#elif ROBOT_ID == 2
    // =========================================================================
    // Robot 2: TWR Responder
    // =========================================================================
    if (DW1000Ng::isReceiveDone()) {
        DW1000Ng::clearReceiveStatus();

        size_t dataLen = DW1000Ng::getReceivedDataLength();
        if (dataLen >= sizeof(UWBPacket)) {
            UWBPacket recvPkt;
            DW1000Ng::getReceivedData(reinterpret_cast<byte*>(&recvPkt), sizeof(recvPkt));

            if (memcmp(recvPkt.header, "PING", 4) == 0) {
                uint64_t rxPingTime = DW1000Ng::getReceiveTimestamp();
                latestRssi = (float)DW1000Ng::getReceivePower();
                pingPongCount++;

                // Prepare PONG
                UWBPacket pongPkt;
                memcpy(pongPkt.header, "PONG", 4);
                pongPkt.sequence = recvPkt.sequence;

                // Send immediate response
                DW1000Ng::forceTRxOff();
                DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&pongPkt), sizeof(pongPkt));
                DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);

                while (!DW1000Ng::isTransmitDone()) { yield(); }
                DW1000Ng::clearTransmitStatus();
                uint64_t txPongTime = DW1000Ng::getTransmitTimestamp();

                // Compute exact turnaround time on Robot 2
                uint32_t replyDelay = (uint32_t)(txPongTime - rxPingTime);

                // Re-embed delay for next frame or fast feedback
                pongPkt.replyDelayTicks = replyDelay;
                DW1000Ng::setTransmitData(reinterpret_cast<byte*>(&pongPkt), sizeof(pongPkt));
            }
        }

        // Return immediately to continuous listening
        DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
    }
    yield();
#endif
}

void commsTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20 Hz internal tick

    uint32_t lastNetcatLogMs = 0;

    while (true) {
        // Accept incoming Netcat connection
        if (!netcatClient || !netcatClient.connected()) {
            netcatClient = netcatServer.accept();
        }

        TelemetryPacket packet = {};
        packet.senderId = Config::ID;
        packet.timestampMs = millis();
        packet.sequenceId = telemetrySeq++;
        packet.measuredDistanceM = latestDistanceM;
        packet.signalRssi = latestRssi;

        // 1. Broadcast state over ESP-NOW to peer
        comms.sendTelemetry(packet);

        // 2. Stream to Netcat strictly once every 500ms (2 Hz)
        if (millis() - lastNetcatLogMs >= 500) {
            lastNetcatLogMs = millis();

            if (netcatClient && netcatClient.connected()) {
                if (latestDistanceM > 0.0f) {
                    netcatClient.printf("[R%d] Distance: %.2f m | RSSI: %.1f dBm | Exchanges: %u\n",
                                        packet.senderId, packet.measuredDistanceM,
                                        packet.signalRssi, pingPongCount);
                } else {
                    netcatClient.printf("[R%d] UWB Ranging searching peer... (Packets: %u)\n",
                                        packet.senderId, pingPongCount);
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
    Serial.printf("        ANJOMAN FIRMWARE - REAL UWB RANGING (ROBOT %d)\n", Config::ID);
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

    Serial.println("[System] Dynamic UWB Ranging Active.");
}

void loop() {
    uwbLoop();
}
