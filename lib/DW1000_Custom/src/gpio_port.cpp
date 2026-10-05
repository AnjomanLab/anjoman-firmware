#include "dw1000_osal.h"
#include <Arduino.h>

extern int dw1000_port_get_cs_pin(void);
extern int dw1000_port_get_irq_pin(void);
extern int dw1000_port_get_rst_pin(void);
extern int dw1000_port_get_wakeup_pin(void);

static int esp32_gpio_init(void) {
    int irq_pin    = dw1000_port_get_irq_pin();
    int rst_pin    = dw1000_port_get_rst_pin();
    int wakeup_pin = dw1000_port_get_wakeup_pin();

    if (irq_pin >= 0) {
        pinMode(irq_pin, INPUT_PULLUP);
    }
    if (rst_pin >= 0) {
        pinMode(rst_pin, OUTPUT);
        digitalWrite(rst_pin, HIGH);
    }
    if (wakeup_pin >= 0) {
        pinMode(wakeup_pin, OUTPUT);
        digitalWrite(wakeup_pin, HIGH);
    }

    return 0;
}

static int esp32_irq_read(void) {
    int irq_pin = dw1000_port_get_irq_pin();
    if (irq_pin < 0) return 0;
    return digitalRead(irq_pin);
}

static void esp32_rst_set(int level) {
    int rst_pin = dw1000_port_get_rst_pin();
    if (rst_pin < 0) return;
    digitalWrite(rst_pin, level ? HIGH : LOW);
}

static void esp32_wakeup_set(int level) {
    int wakeup_pin = dw1000_port_get_wakeup_pin();
    if (wakeup_pin < 0) return;
    digitalWrite(wakeup_pin, level ? HIGH : LOW);
}

static void esp32_irq_clear(void) {
    int irq_pin = dw1000_port_get_irq_pin();
    if (irq_pin < 0) return;
    (void)digitalRead(irq_pin);
}

extern const dw1000_gpio_ops_t esp32_gpio_ops = {
    .gpio_init  = esp32_gpio_init,
    .irq_read   = esp32_irq_read,
    .rst_set    = esp32_rst_set,
    .wakeup_set = esp32_wakeup_set,
    .irq_clear  = esp32_irq_clear,
};
