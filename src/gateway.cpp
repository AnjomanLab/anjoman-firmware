#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "PinMap.h"

#define SERIAL_BAUD             460800
#define RSSI_PRINT_INTERVAL_MS  5000

typedef struct __attribute__((packed)) {
    uint8_t  robot_id;
    uint8_t  n_peers;
    uint8_t  n_ranging_ok;
    uint8_t  n_ranging_fail;
    uint16_t dist_r2_cm;
    uint16_t dist_r3_cm;
    uint16_t dist_r4_cm;
    uint8_t  temperature_c;
    uint16_t voltage_mv;
    uint32_t sync_error_us;
    uint32_t frame_counter;
    uint32_t uptime_ms;
} telemetry_packet_t;

typedef struct {
    uint32_t rx_count;
    uint32_t last_rx_ms;
    int8_t   last_rssi;
    uint32_t last_frame_counter;
    telemetry_packet_t last_packet;
    uint8_t  has_data;
} robot_track_t;

static robot_track_t g_robots[5];
static uint32_t g_last_summary_ms = 0;
static uint32_t g_total_rx = 0;

static const char *robot_name(uint8_t id) {
    switch (id) {
        case 2: return "R2";
        case 3: return "R3";
        case 4: return "R4";
        default: return "??";
    }
}

static void print_banner(void) {
    Serial.println();
    Serial.println("============================================");
    Serial.println("  ANJOMAN GATEWAY (R1)");
    Serial.println("============================================");
    Serial.printf("  Build:  %s %s\n", __DATE__, __TIME__);
    Serial.printf("  Baud:   %d\n", SERIAL_BAUD);
    Serial.println("============================================");
    Serial.println();
}

static void print_packet(const telemetry_packet_t *p, int8_t rssi) {
    const char *name = robot_name(p->robot_id);

    Serial.println("--------------------------------------------");
    Serial.printf("[%s] rx  rssi=%d dBm  frame=%lu  uptime=%lu ms\n",
                  name,
                  (int)rssi,
                  (unsigned long)p->frame_counter,
                  (unsigned long)p->uptime_ms);
    Serial.printf("     n_peers=%u  ok=%u  fail=%u\n",
                  p->n_peers, p->n_ranging_ok, p->n_ranging_fail);
    Serial.printf("     D(2)=%u cm  D(3)=%u cm  D(4)=%u cm\n",
                  p->dist_r2_cm, p->dist_r3_cm, p->dist_r4_cm);
    Serial.printf("     Temp=%u C  Volt=%u mV  SyncErr=%lu us\n",
                  p->temperature_c,
                  p->voltage_mv,
                  (unsigned long)p->sync_error_us);
}

static void print_summary(void) {
    Serial.println("================= SUMMARY ==================");
    Serial.printf("  Total RX: %lu\n", (unsigned long)g_total_rx);
    for (uint8_t id = 2; id <= 4; id++) {
        robot_track_t *t = &g_robots[id];
        if (!t->has_data) {
            Serial.printf("  %s: no data yet\n", robot_name(id));
            continue;
        }
        uint32_t age_ms = millis() - t->last_rx_ms;
        Serial.printf("  %s: rx=%lu  age=%lu ms  rssi=%d dBm  peers=%u  ok=%u\n",
                      robot_name(id),
                      (unsigned long)t->rx_count,
                      (unsigned long)age_ms,
                      (int)t->last_rssi,
                      t->last_packet.n_peers,
                      t->last_packet.n_ranging_ok);
    }
    Serial.println("============================================");
}

static void esp_now_recv_cb(const uint8_t *mac, const uint8_t *data, int len) {
    if (len != (int)sizeof(telemetry_packet_t)) {
        Serial.printf("[RX] Invalid length: %d (expected %d)\n",
                      len, (int)sizeof(telemetry_packet_t));
        return;
    }

    telemetry_packet_t pkt;
    memcpy(&pkt, data, sizeof(telemetry_packet_t));

    if (pkt.robot_id < 2 || pkt.robot_id > 4) {
        Serial.printf("[RX] Invalid robot_id: %u\n", pkt.robot_id);
        return;
    }

    robot_track_t *t = &g_robots[pkt.robot_id];
    t->rx_count++;
    t->last_rx_ms = millis();
    t->last_frame_counter = pkt.frame_counter;
    t->last_packet = pkt;
    t->has_data = 1;
    t->last_rssi = 0;

    g_total_rx++;

    print_packet(&pkt, t->last_rssi);
}

static void esp_now_send_cb(const uint8_t *mac, esp_now_send_status_t status) {
    (void)mac;
    (void)status;
}

static void init_esp_now(void) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init FAILED");
        return;
    }

    esp_now_register_recv_cb(esp_now_recv_cb);
    esp_now_register_send_cb(esp_now_send_cb);

    uint8_t local_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, local_mac);
    Serial.printf("[ESP-NOW] Local MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  local_mac[0], local_mac[1], local_mac[2],
                  local_mac[3], local_mac[4], local_mac[5]);
    Serial.println("[ESP-NOW] Listening for telemetry from R2, R3, R4...");
    Serial.println();
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);

    memset(g_robots, 0, sizeof(g_robots));

    print_banner();
    init_esp_now();
}

void loop() {
    uint32_t now = millis();
    if (now - g_last_summary_ms >= RSSI_PRINT_INTERVAL_MS) {
        g_last_summary_ms = now;
        if (g_total_rx > 0) {
            print_summary();
        }
    }

    delay(50);
}
