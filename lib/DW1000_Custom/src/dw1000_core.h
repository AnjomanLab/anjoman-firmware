#ifndef DW1000_CORE_H
#define DW1000_CORE_H

#include <stdint.h>
#include <stddef.h>
#include "dw1000_osal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DW1000_OK                       0
#define DW1000_ERR_FRAME_SIZE          -1
#define DW1000_ERR_TIMEOUT             -2
#define DW1000_ERR_CRC                 -3
#define DW1000_ERR_NO_DEVICE           -4
#define DW1000_ERR_PLL_LOCK            -5
#define DW1000_ERR_RX_FAIL             -6

#define DW1000_MAX_FRAME_LEN            1023
#define DW1000_TICK_TO_CM               0.00001565f
#define DW1000_TICK_TO_M                1.565e-8f

#define DW1000_TX_FLAG_NONE             0x00
#define DW1000_TX_FLAG_DELAYED          0x01
#define DW1000_TX_FLAG_RANGING          0x02

#define DW1000_RX_FLAG_NONE             0x00
#define DW1000_RX_FLAG_TIMESTAMP        0x01

#define DW1000_CHANNEL_5                5
#define DW1000_PRF_16MHZ                0x01
#define DW1000_PRF_64MHZ                0x02
#define DW1000_PREAMBLE_256             0x04
#define DW1000_DATARATE_850K            0x00
#define DW1000_PAC_8                    0x01

#define DW1000_SAR_CTRL_REG          0x31
#define DW1000_SAR_CTRL_LDO          0x01
#define DW1000_SAR_CTRL_TRIM         0x02
#define DW1000_SAR_CTRL_START        0x80
#define DW1000_SAR_STATUS_REG        0x32
#define DW1000_SAR_STATUS_DONE       0x04
#define DW1000_SAR_DATA_REG          0x32
#define DW1000_SAR_READ_REG          0x33


typedef struct {
    uint8_t  channel;
    uint8_t  prf;
    uint8_t  preamble_len;
    uint8_t  datarate;
    uint8_t  pac_size;
    uint16_t tx_antd;
    uint16_t rx_antd;
    uint8_t  xtal_trim;
    uint32_t spi_speed_hz;
} dw1000_config_t;

int dw1000_init(const dw1000_config_t *config);
int dw1000_is_connected(void);
uint32_t dw1000_get_device_id(void);

int dw1000_tx_send(const uint8_t *frame, uint16_t len, uint32_t flags);
int dw1000_tx_send_at(const uint8_t *frame, uint16_t len, uint64_t tx_time);
int dw1000_tx_wait_done(uint32_t timeout_us);
uint64_t dw1000_get_tx_timestamp(void);

int dw1000_rx_start(uint32_t timeout_us);
int dw1000_rx_wait_done(uint32_t timeout_us);
int dw1000_read_frame(uint8_t *buf, uint16_t max_len, uint16_t *out_len);
uint64_t dw1000_get_rx_timestamp(void);

int dw1000_rx_enable(void);
int dw1000_rx_disable(void);

uint32_t dw1000_read_clock_ratio(void);
uint8_t  dw1000_read_temperature(void);
uint16_t dw1000_read_voltage(void);

int dw1000_set_rx_antenna_delay(uint16_t delay);
int dw1000_set_tx_antenna_delay(uint16_t delay);
int dw1000_load_xtal_trim(uint8_t trim);

void dw1000_reset(void);
uint64_t dw1000_get_system_time(void);
uint32_t dw1000_get_time_us(void);

int dw1000_reg_read(uint16_t reg, uint8_t *buf, uint16_t len);
int dw1000_reg_write(uint16_t reg, const uint8_t *buf, uint16_t len);
uint32_t dw1000_reg_read32(uint16_t reg);
void dw1000_reg_write32(uint16_t reg, uint32_t val);

void dw1000_correct_raw_distance(float raw_distance, float *corrected_distance);

static inline void dw1000_port_delay_us(uint32_t us) {
    const dw1000_port_ops_t *p = dw1000_port_get();
    if (p && p->delay.delay_us) p->delay.delay_us(us);
}

static inline uint32_t dw1000_port_get_time_us(void) {
    const dw1000_port_ops_t *p = dw1000_port_get();
    if (p && p->delay.get_time_us) return p->delay.get_time_us();
    return 0;
}

int dw1000_port_init(const dw1000_port_ops_t *ops);
const dw1000_port_ops_t *dw1000_port_get(void);

#ifdef __cplusplus
}
#endif

#endif
