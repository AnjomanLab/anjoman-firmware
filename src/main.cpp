#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "PinMap.h"

#include "dw1000_osal.h"
#include "dw1000_core.h"
#include "dw1000_regs.h"
#include "dw1000_tuning.h"
#include "ss_twr.h"
#include "tdma_scheduler.h"

#define DW1000_PIN_CS       PIN_UWB_CS
#define DW1000_PIN_IRQ      PIN_UWB_IRQ
#define DW1000_PIN_RST      -1
#define DW1000_PIN_WAKEUP   PIN_UWB_WAKEUP

#define GATEWAY_MAC_0       0xFF
#define GATEWAY_MAC_1       0xFF
#define GATEWAY_MAC_2       0xFF
#define GATEWAY_MAC_3       0xFF
#define GATEWAY_MAC_4       0xFF
#define GATEWAY_MAC_5       0xFF

#define TELEMETRY_INTERVAL_MS   500
#define SERIAL_BAUD             460800

extern const dw1000_spi_ops_t   esp32_spi_ops;
extern const dw1000_gpio_ops_t  esp32_gpio_ops;
extern const dw1000_delay_ops_t esp32_delay_ops;

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

static dw1000_port_ops_t   g_port_runtime;
static telemetry_packet_t  g_telemetry;
static uint8_t g_gateway_mac[6] = {
    GATEWAY_MAC_0, GATEWAY_MAC_1, GATEWAY_MAC_2,
    GATEWAY_MAC_3, GATEWAY_MAC_4, GATEWAY_MAC_5
};
static uint32_t g_last_telemetry_ms = 0;
static uint32_t g_frame_counter = 0;

static void build_port_ops(void) {
    g_port_runtime.spi   = esp32_spi_ops;
    g_port_runtime.gpio  = esp32_gpio_ops;
    g_port_runtime.delay = esp32_delay_ops;
}

static int get_robot_index(uint8_t robot_id) {
    if (robot_id == 2) return 0;
    if (robot_id == 3) return 1;
    if (robot_id == 4) return 2;
    return 0;
}

static void print_banner(void) {
    Serial.println();
    Serial.println("============================================");
    Serial.printf("  ANJOMAN UWB v2 - ROBOT R%d\n", ROBOT_ID);
    Serial.println("============================================");
    Serial.printf("  Build:    %s %s\n", __DATE__, __TIME__);
    Serial.printf("  CS pin:   %d\n", DW1000_PIN_CS);
    Serial.printf("  IRQ pin:  %d\n", DW1000_PIN_IRQ);
    Serial.printf("  RST pin:  %d\n", DW1000_PIN_RST);
    Serial.printf("  WAKEUP:   %d\n", DW1000_PIN_WAKEUP);
    Serial.println("============================================");
    Serial.println();
}

static int init_dw1000(void) {
    build_port_ops();
    dw1000_port_init(&g_port_runtime);

    dw1000_port_set_pins(DW1000_PIN_CS, DW1000_PIN_IRQ,
                         DW1000_PIN_RST, DW1000_PIN_WAKEUP);

    dw1000_config_t cfg;
    cfg.channel       = DW1000_CHANNEL_5;
    cfg.prf           = DW1000_PRF_16MHZ;
    cfg.preamble_len  = DW1000_PREAMBLE_256;
    cfg.datarate      = DW1000_DATARATE_850K;
    cfg.pac_size      = DW1000_PAC_8;
    cfg.xtal_trim     = DW1000_XTAL_TRIM_OTP_DEFAULT;
    cfg.spi_speed_hz  = DW1000_SPI_SPEED_HZ;

    int idx = get_robot_index(ROBOT_ID);
    cfg.tx_antd = dw1000_get_tx_antd(idx);
    cfg.rx_antd = dw1000_get_rx_antd(idx);

    Serial.println("[DW1000] Initializing...");
    Serial.printf("[DW1000] Channel:    %d\n", cfg.channel);
    Serial.printf("[DW1000] PRF:        %d MHz\n", cfg.prf == DW1000_PRF_16MHZ ? 16 : 64);
    Serial.printf("[DW1000] Preamble:   %d\n", cfg.preamble_len == DW1000_PREAMBLE_256 ? 256 : 128);
    Serial.printf("[DW1000] TX_ANTD:    %u\n", cfg.tx_antd);
    Serial.printf("[DW1000] RX_ANTD:    %u\n", cfg.rx_antd);
    Serial.printf("[DW1000] XTAL_TRIM:  0x%02X\n", cfg.xtal_trim);
    Serial.printf("[DW1000] SPI Mode:   %d\n", DW1000_SPI_MODE_0);
    pinMode(13, INPUT);
delay(10);
Serial.printf("[DEBUG] GPIO 13 (MISO) idle state: %d\n", digitalRead(13));

    int ret = dw1000_init(&cfg);
    if (ret != DW1000_OK) {
        Serial.printf("[DW1000] Init FAILED: %d\n", ret);
        return ret;
    }

    uint32_t dev_id = dw1000_get_device_id();
    Serial.printf("[DW1000] DEV_ID:     0x%08X (expected 0x%08X)\n",
                  dev_id, DW1000_DEV_ID_VAL);

    if (dev_id != DW1000_DEV_ID_VAL) {
        Serial.println("[DW1000] DEV_ID mismatch!");
        return DW1000_ERR_NO_DEVICE;
    }

    Serial.println("[DW1000] Init OK");
    return DW1000_OK;
}

static void esp_now_recv_cb(const uint8_t *mac, const uint8_t *data, int len) {
    (void)mac;
    (void)data;
    (void)len;
}

static void esp_now_send_cb(const uint8_t *mac, esp_now_send_status_t status) {
    (void)mac;
    (void)status;
}

static int init_esp_now(void) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESP-NOW] Init FAILED");
        return -1;
    }

    esp_now_register_recv_cb(esp_now_recv_cb);
    esp_now_register_send_cb(esp_now_send_cb);

    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, g_gateway_mac, 6);
    peer.channel = 0;
    peer.encrypt = false;

    if (!esp_now_is_peer_exist(g_gateway_mac)) {
        if (esp_now_add_peer(&peer) != ESP_OK) {
            Serial.println("[ESP-NOW] Add peer FAILED");
            return -1;
        }
    }

    uint8_t local_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, local_mac);
    Serial.printf("[ESP-NOW] Local MAC:   %02X:%02X:%02X:%02X:%02X:%02X\n",
                  local_mac[0], local_mac[1], local_mac[2],
                  local_mac[3], local_mac[4], local_mac[5]);
    Serial.printf("[ESP-NOW] Gateway MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  g_gateway_mac[0], g_gateway_mac[1], g_gateway_mac[2],
                  g_gateway_mac[3], g_gateway_mac[4], g_gateway_mac[5]);

    return 0;
}

static void send_telemetry(void) {
    g_telemetry.robot_id       = ROBOT_ID;
    g_telemetry.n_peers        = (uint8_t)tdma_scheduler_get_n_peers();
    g_telemetry.n_ranging_ok   = (uint8_t)tdma_scheduler_get_n_ranging_ok();
    g_telemetry.n_ranging_fail = (uint8_t)tdma_scheduler_get_n_ranging_fail();
    g_telemetry.dist_r2_cm     = (uint16_t)tdma_scheduler_get_peer_distance(2);
    g_telemetry.dist_r3_cm     = (uint16_t)tdma_scheduler_get_peer_distance(3);
    g_telemetry.dist_r4_cm     = (uint16_t)tdma_scheduler_get_peer_distance(4);
    g_telemetry.temperature_c  = dw1000_read_temperature();
    g_telemetry.voltage_mv     = dw1000_read_voltage();
    g_telemetry.sync_error_us  = tdma_scheduler_get_sync_error_us();
    g_telemetry.frame_counter  = g_frame_counter;
    g_telemetry.uptime_ms      = millis();

    esp_now_send(g_gateway_mac, (uint8_t *)&g_telemetry, sizeof(g_telemetry));
}

static void print_status(void) {
    Serial.println("--------------------------------------------");
    Serial.printf("[R%d] frame=%lu  n_peers=%lu  ok=%lu  fail=%lu\n",
                  ROBOT_ID,
                  (unsigned long)g_frame_counter,
                  (unsigned long)tdma_scheduler_get_n_peers(),
                  (unsigned long)tdma_scheduler_get_n_ranging_ok(),
                  (unsigned long)tdma_scheduler_get_n_ranging_fail());

    Serial.printf("     D(2)=%u cm  D(3)=%u cm  D(4)=%u cm\n",
                  tdma_scheduler_get_peer_distance(2),
                  tdma_scheduler_get_peer_distance(3),
                  tdma_scheduler_get_peer_distance(4));

    Serial.printf("     Temp=%u C  Volt=%u mV  SyncErr=%lu us\n",
                  dw1000_read_temperature(),
                  dw1000_read_voltage(),
                  (unsigned long)tdma_scheduler_get_sync_error_us());

    Serial.print("     Slot roles: ");
    for (uint8_t i = 0; i < TDMA_FRAME_SLOTS; i++) {
        Serial.printf("%d", (int)tdma_scheduler_get_role(i));
    }
    Serial.println();
    Serial.println("--------------------------------------------");
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);

    print_banner();

    if (init_dw1000() != DW1000_OK) {
        Serial.println("[FATAL] DW1000 init failed - halting");
        while (1) { delay(1000); }
    }

    if (tdma_scheduler_init(ROBOT_ID) != TDMA_OK) {
        Serial.println("[FATAL] TDMA init failed - halting");
        while (1) { delay(1000); }
    }
    Serial.printf("[TDMA] Initialized for R%d\n", ROBOT_ID);
    Serial.printf("[TDMA] Slot duration:  %lu us\n",
                  (unsigned long)tdma_scheduler_slot_duration_us());
    Serial.printf("[TDMA] Frame duration: %lu us\n",
                  (unsigned long)tdma_scheduler_frame_duration_us());
    Serial.printf("[TDMA] Guard time:     %d us\n", TDMA_GUARD_US);
    Serial.printf("[TDMA] TX offset:      %d us\n", TDMA_TX_OFFSET_US);

    if (init_esp_now() != 0) {
        Serial.println("[WARN] ESP-NOW init failed - telemetry disabled");
    }

    Serial.println();
    Serial.println("[READY] Starting TDMA loop...");
    Serial.println();
}

void loop() {
    tdma_scheduler_run_frame();
    g_frame_counter++;

    uint32_t now = millis();
    if (now - g_last_telemetry_ms >= TELEMETRY_INTERVAL_MS) {
        g_last_telemetry_ms = now;
        send_telemetry();
        print_status();
    }
}
