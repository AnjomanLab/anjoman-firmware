// ==============================================================================
// Anjoman Gateway - R1
// Sends TDMA sync beacons + receives robot telemetry + prints CSV
// ==============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static constexpr uint32_t TDMA_FRAME_MS = 200;

// Must match main.cpp exactly
#pragma pack(push, 1)

struct TDMASyncPacket {
    uint32_t frameId;
    uint32_t _pad;
};

struct UWBBeaconEntry {
    uint8_t  peerId;
    uint8_t  ldeErr;
    uint16_t rawDistMm;
    uint16_t cleanDistMm;
    int8_t   rssi;
    int8_t   fpPower;
    uint16_t _pad;
};

struct SyncBeaconPacket {
    uint32_t frameId;
    uint32_t senderUptimeMs;
    uint8_t  senderId;
    uint8_t  senderSlot;
    uint8_t  nPeers;
    uint8_t  _pad;
    UWBBeaconEntry peers[2];
};

#pragma pack(pop)

static volatile uint32_t g_frameId = 0;

// Per-robot gap detection
static uint32_t lastFrame[5]  = {0};
static uint32_t lostFrame[5]  = {0};
static uint32_t recvCount[5]  = {0};

void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    (void)mac;
    if (len != sizeof(SyncBeaconPacket)) return;

    SyncBeaconPacket pkt;
    memcpy(&pkt, data, sizeof(pkt));

    uint8_t id = pkt.senderId;
    if (id < 2 || id > 4) return;

    // Gap detection
    if (lastFrame[id] != 0) {
        uint32_t expected = lastFrame[id] + 1;
        if (pkt.frameId > expected) {
            lostFrame[id] += (pkt.frameId - expected);
        }
    }
    lastFrame[id] = pkt.frameId;
    recvCount[id]++;

    // CSV line: rx_ms,senderId,senderSlot,frameId,nPeers,senderUptimeMs,
    //           p1_id,p1_lde,p1_raw,p1_clean,p1_rssi,p1_fp,
    //           p2_id,p2_lde,p2_raw,p2_clean,p2_rssi,p2_fp
    Serial.printf("%lu,%u,%u,%lu,%u,%lu",
                  (unsigned long)millis(),
                  pkt.senderId, pkt.senderSlot,
                  (unsigned long)pkt.frameId,
                  pkt.nPeers,
                  (unsigned long)pkt.senderUptimeMs);

    for (int i = 0; i < 2; i++) {
        if (i < pkt.nPeers) {
            const UWBBeaconEntry &e = pkt.peers[i];
            Serial.printf(",%u,%u,%u,%u,%d,%d",
                          e.peerId, e.ldeErr,
                          e.rawDistMm, e.cleanDistMm,
                          (int)e.rssi, (int)e.fpPower);
        } else {
            Serial.print(",0,0,0,0,0,0");
        }
    }

    Serial.println();
}

void setup() {
    Serial.begin(460800);
    delay(1000);

    Serial.println("[GATEWAY] R1 starting...");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[GATEWAY] ESP-NOW init failed");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, BROADCAST_MAC, 6);
    peer.channel = 1;
    peer.encrypt = false;
    peer.ifidx   = WIFI_IF_STA;
    esp_now_add_peer(&peer);

    Serial.println("rx_ms,senderId,senderSlot,frameId,nPeers,senderUptimeMs,"
                   "p1_id,p1_lde,p1_raw,p1_clean,p1_rssi,p1_fp,"
                   "p2_id,p2_lde,p2_raw,p2_clean,p2_rssi,p2_fp");
    Serial.println("[GATEWAY] Ready");
}

void loop() {
    static uint32_t lastSync  = 0;
    static uint32_t lastStats = 0;
    uint32_t now = millis();

    if (now - lastSync >= TDMA_FRAME_MS) {
        lastSync = now;
        g_frameId++;

        TDMASyncPacket sync = {};
        sync.frameId = g_frameId;
        esp_now_send(BROADCAST_MAC, (uint8_t*)&sync, sizeof(sync));
    }

    // Stats every 5 seconds
    if (now - lastStats >= 5000) {
        lastStats = now;
        Serial.printf("# STATS frame=%lu | recv r2=%lu r3=%lu r4=%lu | "
                      "lost r2=%lu r3=%lu r4=%lu\n",
                      (unsigned long)g_frameId,
                      (unsigned long)recvCount[2],
                      (unsigned long)recvCount[3],
                      (unsigned long)recvCount[4],
                      (unsigned long)lostFrame[2],
                      (unsigned long)lostFrame[3],
                      (unsigned long)lostFrame[4]);
    }
}
