#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "Types.h"

// ==============================================================================
// TELEMETRY RECEPTION AND SERIAL CSV STREAMER
// ==============================================================================
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    (void)mac;
    if (len < static_cast<int>(sizeof(SyncBeaconPacket))) {
        return;
    }

    SyncBeaconPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    const uint32_t nowMs = millis();

    // Stream base robot telemetry
    Serial.printf("%lu,%u,%u,%lu,%lu,%lu,%u,%.3f,%.2f,%.4f,%.4f,%.4f,",
                  static_cast<unsigned long>(nowMs),
                  pkt.senderId,
                  pkt.senderSlot,
                  static_cast<unsigned long>(pkt.frameId),
                  static_cast<unsigned long>(pkt.senderUptimeMs),
                  static_cast<unsigned long>(pkt.maneuverStartFrame),
                  pkt.nPeers,
                  pkt.vbat,
                  pkt.senderTempUwb,
                  pkt.posX,
                  pkt.posY,
                  pkt.headingRad);

    // Stream peer range metrics (Exactly 2 peer slots in CSV schema)
    for (uint8_t i = 0; i < 2; ++i) {
        if (i < pkt.nPeers) {
            const UWBBeaconEntry &e = pkt.peers[i];
            const float rawM   = static_cast<float>(e.rawDistMm) / 1000.0f;
            const float cleanM = static_cast<float>(e.cleanDistMm) / 1000.0f;

            Serial.printf("%u,%u,%.4f,%.4f,%d,%d,%.2f,%.2f%s",
                          e.peerId,
                          e.ldeErr,
                          rawM,
                          cleanM,
                          e.rssi,
                          e.fpPower,
                          e.respTemp,
                          e.cfoPpm,
                          (i == 0) ? "," : "");
        } else {
            Serial.printf("0,0,0,0,0,0,0,0%s", (i == 0) ? "," : "");
        }
    }
    Serial.println();
}

// ==============================================================================
// INITIALIZATION
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(1000);

    WiFi.mode(WIFI_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[GATEWAY ERROR] ESP-NOW initialization failed!");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);

    // Print CSV Header matching the schema
    Serial.println("rx_ms,senderId,senderSlot,frameId,uptimeMs,manStartFrame,nPeers,vbat,senderTempUwb,posX,posY,headingRad,p1_id,p1_lde,p1_raw,p1_clean,p1_rssi,p1_fp,p1_temp,p1_cfo,p2_id,p2_lde,p2_raw,p2_clean,p2_rssi,p2_fp,p2_temp,p2_cfo");
}

void loop() {
    delay(1000);
}
