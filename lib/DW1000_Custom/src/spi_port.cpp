#include "dw1000_osal.h"
#include <Arduino.h>
#include <SPI.h>

#include "PinMap.h"

static SPIClass *g_spi = nullptr;
static int g_cs_pin  = 14;
static int g_irq_pin = 5;
static int g_rst_pin = -1;
static int g_wakeup_pin = -1;

void dw1000_port_set_pins(int cs_pin, int irq_pin, int rst_pin, int wakeup_pin) {
    g_cs_pin     = cs_pin;
    g_irq_pin    = irq_pin;
    g_rst_pin    = rst_pin;
    g_wakeup_pin = wakeup_pin;
}

int dw1000_port_get_cs_pin(void)     { return g_cs_pin; }
int dw1000_port_get_irq_pin(void)    { return g_irq_pin; }
int dw1000_port_get_rst_pin(void)    { return g_rst_pin; }
int dw1000_port_get_wakeup_pin(void) { return g_wakeup_pin; }

static int esp32_spi_init(void) {
    if (g_cs_pin < 0) return -1;

    g_spi = new SPIClass(FSPI);

    g_spi->begin(PIN_UWB_SCK, PIN_UWB_MISO, PIN_UWB_MOSI, -1);

    g_spi->setFrequency(DW1000_SPI_SPEED_HZ);
    g_spi->setDataMode(DW1000_SPI_MODE_2);
    g_spi->setBitOrder(MSBFIRST);

    pinMode(g_cs_pin, OUTPUT);
    digitalWrite(g_cs_pin, HIGH);

    return 0;
}

static int esp32_spi_transfer(const uint8_t *tx, uint8_t *rx, size_t len) {
    if (!g_spi) return -1;
    if (len == 0) return 0;

    if (tx && rx) {
        g_spi->transferBytes(tx, rx, len);
    } else if (tx && !rx) {
        uint8_t dummy[DW1000_SPI_MAX_XFER];
        size_t remaining = len;
        const uint8_t *src = tx;
        while (remaining > 0) {
            size_t chunk = remaining > sizeof(dummy) ? sizeof(dummy) : remaining;
            g_spi->transferBytes(src, dummy, chunk);
            src       += chunk;
            remaining -= chunk;
        }
    } else if (!tx && rx) {
        uint8_t dummy[DW1000_SPI_MAX_XFER] = {0};
        size_t remaining = len;
        uint8_t *dst = rx;
        while (remaining > 0) {
            size_t chunk = remaining > sizeof(dummy) ? sizeof(dummy) : remaining;
            g_spi->transferBytes(dummy, dst, chunk);
            dst       += chunk;
            remaining -= chunk;
        }
    }

    return 0;
}

static void esp32_spi_cs_assert(void) {
    digitalWrite(g_cs_pin, LOW);
}

static void esp32_spi_cs_deassert(void) {
    digitalWrite(g_cs_pin, HIGH);
}

extern const dw1000_spi_ops_t esp32_spi_ops = {
    .spi_init        = esp32_spi_init,
    .spi_transfer    = esp32_spi_transfer,
    .spi_cs_assert   = esp32_spi_cs_assert,
    .spi_cs_deassert = esp32_spi_cs_deassert,
};
