#include "dw1000_osal.h"
#include <Arduino.h>
#include <esp_timer.h>
#include <rom/ets_sys.h>

static void esp32_delay_us(uint32_t us) {
    esp_rom_delay_us(us);
}

static void esp32_delay_ms(uint32_t ms) {
    delay(ms);
}

static uint32_t esp32_get_time_us(void) {
    return (uint32_t)esp_timer_get_time();
}

extern const dw1000_delay_ops_t esp32_delay_ops = {
    .delay_us    = esp32_delay_us,
    .delay_ms    = esp32_delay_ms,
    .get_time_us = esp32_get_time_us,
};
