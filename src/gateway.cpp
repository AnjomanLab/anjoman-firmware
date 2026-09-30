// ==============================================================================
// ESP-NOW Gateway for Anjoman Swarm (3-robot fleet)
// ==============================================================================
// Listens for SyncBeaconPacket broadcasts on channel 1 and prints each
// received packet as a CSV row on USB Serial.
//
// Usage:
//   pio run -e gateway -t upload
//   pio device monitor -e gateway -b 460800 > gateway_log.csv
//
// Header row is printed once at boot.
// ==============================================================================

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "Types.h"

// ==============================================================================
// CSV header — matches the SyncBeaconPacket layout in Types.h
// ==============================================================================
static const char* CSV_HEADER =
    "rx_ms,senderId,senderSlot,frameId,uptimeMs,manStartFrame,nPeers,"
    "vbat,"
    "p1_id,p1_lde,p1_raw,p1_clean,p1_rssi,p1_fp,p1_temp,"
    "p2_id,p2_lde,p2_raw,p2_clean,p2_rssi,p2_fp,p2_temp";

// ==============================================================================
// CSV printer — one row per beacon
// ==============================================================================
static void printCSV(const SyncBeaconPacket &pkt) {
    char buf[320];

    int n = snprintf(buf, sizeof(buf),
        "%lu,%u,%u,%lu,%lu,%lu,%u,%.3f",
        (unsigned long)millis(),
        pkt.senderId,
        pkt.senderSlot,
        (unsigned long)pkt.frameId,
        (unsigned long)pkt.senderUptimeMs,
        (unsigned long)pkt.maneuverStartFrame,
        pkt.nPeers,
        pkt.vbat);

    for (int i = 0; i < 2; i++) {
        if (i < pkt.nPeers) {
            const UWBBeaconEntry &e = pkt.peers[i];
            n += snprintf(buf + n, sizeof(buf) - n,
                ",%u,%u,%.4f,%.4f,%.2f,%.2f,%.2f",
                e.peerId,
                e.ldeErr,
                e.rawDist,
                e.cleanDist,
                e.rssi,
                e.fpPower,
                e.respTemp);
        } else {
            // Empty slot — fill with zeros to keep column count consistent
            n += snprintf(buf + n, sizeof(buf) - n,
                ",0,0,0,0,0,0,0");
        }
    }

    Serial.println(buf);
}

// ==============================================================================
// ESP-NOW receive callback
// ==============================================================================
static volatile uint32_t g_pktCount = 0;

void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (len != sizeof(SyncBeaconPacket)) return;

    SyncBeaconPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    // Sanity: senderId must be 1..4
    if (pkt.senderId < 1 || pkt.senderId > 4) return;

    printCSV(pkt);
    g_pktCount++;
}

// ==============================================================================
// Setup
// ==============================================================================
void setup() {
    Serial.begin(460800);
    delay(2000);

    // Banner (as comment lines, ignored by CSV parsers)
    Serial.println("# Anjoman ESP-NOW Gateway");
    Serial.printf("# sizeof(SyncBeaconPacket) = %u bytes\n",
                  (unsigned)sizeof(SyncBeaconPacket));
    Serial.println(CSV_HEADER);

    // WiFi / ESP-NOW — same channel as the swarm
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("# ERROR: esp_now_init failed");
        while (1) delay(1000);
    }

    esp_now_register_recv_cb(onRecv);

    // NOTE: no esp_now_add_peer() needed. We only receive broadcast.
}

// ==============================================================================
// Loop — idle
// ==============================================================================
void loop() {
    delay(100);
}
