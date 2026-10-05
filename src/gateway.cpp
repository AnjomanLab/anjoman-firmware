#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "Types.h"

static const char* CSV_HEADER =
    "rx_ms,senderId,senderSlot,frameId,uptimeMs,manStartFrame,nPeers,"
    "vbat,senderTempUwb,posX,posY,headingRad,"
    "p1_id,p1_lde,p1_raw,p1_clean,p1_rssi,p1_fp,p1_temp,p1_cfo,"
    "p2_id,p2_lde,p2_raw,p2_clean,p2_rssi,p2_fp,p2_temp,p2_cfo";

static void printCSV(const SyncBeaconPacket &pkt) {
    char buf[384];

    int n = snprintf(buf, sizeof(buf),
        "%lu,%u,%u,%lu,%lu,%lu,%u,%.3f,%.2f,%.4f,%.4f,%.4f",
        static_cast<unsigned long>(millis()),
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

    for (int i = 0; i < 2; i++) {
        if (i < pkt.nPeers) {
            const UWBBeaconEntry &e = pkt.peers[i];
            n += snprintf(buf + n, sizeof(buf) - n,
                ",%u,%u,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f",
                e.peerId,
                e.ldeErr,
                e.rawDist,
                e.cleanDist,
                e.rssi,
                e.fpPower,
                e.respTemp,
                e.cfoPpm);
        } else {
            n += snprintf(buf + n, sizeof(buf) - n,
                ",0,0,0,0,0,0,0,0");
        }
    }

    Serial.println(buf);
}

static volatile uint32_t g_pktCount = 0;

void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (len != sizeof(SyncBeaconPacket)) return;

    SyncBeaconPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (pkt.senderId < 1 || pkt.senderId > 4) return;

    printCSV(pkt);
    g_pktCount++;
}

void setup() {
    Serial.begin(460800);
    delay(2000);

    Serial.println("# Anjoman Swarm Gateway - Comprehensive Telemetry Logger");
    Serial.printf("# Packet Size: %u bytes\n", static_cast<unsigned>(sizeof(SyncBeaconPacket)));
    Serial.println(CSV_HEADER);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("# Error: esp_now_init failed");
        while (1) delay(1000);
    }

    esp_now_register_recv_cb(onRecv);
}

void loop() {
    delay(100);
}
