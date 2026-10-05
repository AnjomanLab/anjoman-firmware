#ifndef DW1000_OSAL_H
#define DW1000_OSAL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DW1000_SPI_MODE_2           2
#define DW1000_SPI_MODE_0           0
#define DW1000_SPI_SPEED_HZ         20000000
#define DW1000_SPI_MAX_XFER         1024

typedef struct {
    int      (*spi_init)(void);
    int      (*spi_transfer)(const uint8_t *tx, uint8_t *rx, size_t len);
    void     (*spi_cs_assert)(void);
    void     (*spi_cs_deassert)(void);
} dw1000_spi_ops_t;

typedef struct {
    int      (*gpio_init)(void);
    int      (*irq_read)(void);
    void     (*rst_set)(int level);
    void     (*wakeup_set)(int level);
    void     (*irq_clear)(void);
} dw1000_gpio_ops_t;

typedef struct {
    void     (*delay_us)(uint32_t us);
    void     (*delay_ms)(uint32_t ms);
    uint32_t (*get_time_us)(void);
} dw1000_delay_ops_t;

typedef struct {
    dw1000_spi_ops_t   spi;
    dw1000_gpio_ops_t  gpio;
    dw1000_delay_ops_t delay;
} dw1000_port_ops_t;

void dw1000_port_set_pins(int cs_pin, int irq_pin, int rst_pin, int wakeup_pin);
const dw1000_port_ops_t *dw1000_port_get(void);

#ifdef __cplusplus
}
#endif

#endif
