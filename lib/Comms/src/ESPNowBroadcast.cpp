#include "ESPNowBroadcast.h"

ESPNowBroadcast* ESPNowBroadcast::_instance = nullptr;
uint8_t ESPNowBroadcast::_broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

ESPNowBroadcast::ESPNowBroadcast()
    : _robotId(1), _onTelemetryReceived(nullptr) {
    _instance = this;
}

ESPNowBroadcast* ESPNowBroadcast::getInstance() {
    return _instance;
}

bool ESPNowBroadcast::begin(uint8_t robotId) {
    _robotId = robotId;

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init Failed!");
        return false;
    }

    esp_now_register_recv_cb(onDataRecv);

    // Register peer for broadcast
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, _broadcastMac, 6);
    peerInfo.channel = 0; // Current Wi-Fi channel
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[ESP-NOW] Failed to add broadcast peer!");
        return false;
    }

    Serial.printf("[ESP-NOW] Initialized successfully (Robot ID: %d)\n", _robotId);
    return true;
}

bool ESPNowBroadcast::sendTelemetry(const TelemetryPacket &packet) {
    esp_err_t result = esp_now_send(_broadcastMac, reinterpret_cast<const uint8_t*>(&packet), sizeof(TelemetryPacket));
    return (result == ESP_OK);
}

void ESPNowBroadcast::registerTelemetryCallback(TelemetryCallback cb) {
    _onTelemetryReceived = cb;
}

void ESPNowBroadcast::onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
    if (!_instance) return;

    if (data_len == sizeof(TelemetryPacket) && _instance->_onTelemetryReceived) {
        TelemetryPacket packet;
        memcpy(&packet, data, sizeof(TelemetryPacket));
        _instance->_onTelemetryReceived(packet);
    }
}
