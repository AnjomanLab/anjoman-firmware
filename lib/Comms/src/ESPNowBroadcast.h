#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Types.h"

class ESPNowBroadcast {
public:
    ESPNowBroadcast();

    bool begin(uint8_t robotId = 1);
    bool sendTelemetry(const TelemetryPacket &packet);

    typedef void (*TelemetryCallback)(const TelemetryPacket &packet);
    void registerTelemetryCallback(TelemetryCallback cb);

    static ESPNowBroadcast* getInstance();

private:
    static ESPNowBroadcast* _instance;
    static uint8_t _broadcastMac[6];
    uint8_t _robotId;

    TelemetryCallback _onTelemetryReceived;

    static void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len);
};
