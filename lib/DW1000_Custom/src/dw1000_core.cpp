#include "dw1000_core.h"
#include "dw1000_regs.h"
#include "dw1000_tuning.h"

#define DW1000_SAR_CHANNEL_TEMP     0x00
#define DW1000_SAR_CHANNEL_VBAT     0x01
#define DW1000_SAR_TIMEOUT_US       5000
#define DW1000_SAR_VREF             1.2f
#define DW1000_SAR_RESOLUTION       4096.0f
#define DW1000_SAR_TEMP_OFFSET      0.5f
#define DW1000_SAR_TEMP_SLOPE       0.0075f
#define DW1000_SAR_TEMP_REF_C       25.0f
#define DW1000_SAR_VBAT_SCALE       1.2f

static dw1000_config_t g_config;
static const dw1000_port_ops_t *g_port = nullptr;
static uint8_t g_tx_scratch[32];

static int dw1000_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len) {
    if (!g_port || !g_port->spi.spi_transfer) return DW1000_ERR_NO_DEVICE;
    return g_port->spi.spi_transfer(tx, rx, len);
}

int dw1000_reg_read(uint16_t reg, uint8_t *buf, uint16_t len) {
    if (!g_port || !buf || len == 0) return DW1000_ERR_NO_DEVICE;

    uint8_t header = (uint8_t)(reg & 0x3F);
    g_port->spi.spi_cs_assert();
    dw1000_spi_transfer(&header, nullptr, 1);

    uint8_t dummy[256];
    uint8_t tx_zero[256] = {0};
    if (len > sizeof(dummy)) len = sizeof(dummy);
    dw1000_spi_transfer(tx_zero, dummy, len);

    for (uint16_t i = 0; i < len; i++) {
        buf[i] = dummy[i];
    }
    g_port->spi.spi_cs_deassert();
    return DW1000_OK;
}

int dw1000_reg_write(uint16_t reg, const uint8_t *buf, uint16_t len) {
    if (!g_port || !buf || len == 0) return DW1000_ERR_NO_DEVICE;

    uint8_t header = (uint8_t)(0x80 | (reg & 0x3F));
    g_port->spi.spi_cs_assert();
    dw1000_spi_transfer(&header, nullptr, 1);
    dw1000_spi_transfer(buf, nullptr, len);
    g_port->spi.spi_cs_deassert();
    return DW1000_OK;
}

uint32_t dw1000_reg_read32(uint16_t reg) {
    uint8_t buf[4] = {0};
    dw1000_reg_read(reg, buf, 4);
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

void dw1000_reg_write32(uint16_t reg, uint32_t val) {
    uint8_t buf[4];
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
    dw1000_reg_write(reg, buf, 4);
}

static int dw1000_read_sub_reg(uint16_t reg, uint16_t subaddr,
                                uint8_t *buf, uint16_t len) {
    if (!g_port || !buf || len == 0) return DW1000_ERR_NO_DEVICE;

    uint8_t header[3];
    header[0] = (uint8_t)(0x40 | (reg & 0x3F));
    header[1] = (uint8_t)((subaddr >> 8) & 0xFF);
    header[2] = (uint8_t)(subaddr & 0xFF);

    g_port->spi.spi_cs_assert();
    dw1000_spi_transfer(header, nullptr, 3);
    uint8_t dummy[16];
    uint8_t tx_zero[16] = {0};
    if (len > sizeof(dummy)) len = sizeof(dummy);
    dw1000_spi_transfer(tx_zero, dummy, len);
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = dummy[i];
    }
    g_port->spi.spi_cs_deassert();
    return DW1000_OK;
}

static int dw1000_write_sub_reg(uint16_t reg, uint16_t subaddr,
                                 const uint8_t *buf, uint16_t len) {
    if (!g_port || !buf || len == 0) return DW1000_ERR_NO_DEVICE;

    uint8_t header[3];
    header[0] = (uint8_t)(0xC0 | (reg & 0x3F));
    header[1] = (uint8_t)((subaddr >> 8) & 0xFF);
    header[2] = (uint8_t)(subaddr & 0xFF);

    g_port->spi.spi_cs_assert();
    dw1000_spi_transfer(header, nullptr, 3);
    dw1000_spi_transfer(buf, nullptr, len);
    g_port->spi.spi_cs_deassert();
    return DW1000_OK;
}

static int dw1000_write_tx_buffer(const uint8_t *frame, uint16_t len) {
    if (len > DW1000_MAX_FRAME_LEN) return DW1000_ERR_FRAME_SIZE;

    uint8_t header = DW1000_SPI_WRITE_HEADER(DW1000_REG_TX_BUFFER);
    g_port->spi.spi_cs_assert();
    dw1000_spi_transfer(&header, nullptr, 1);
    dw1000_spi_transfer(frame, nullptr, len);
    g_port->spi.spi_cs_deassert();
    return DW1000_OK;
}

static int dw1000_read_rx_buffer(uint8_t *frame, uint16_t len) {
    if (len > DW1000_MAX_FRAME_LEN) return DW1000_ERR_FRAME_SIZE;

    uint8_t header = DW1000_SPI_READ_HEADER(DW1000_REG_RX_BUFFER);
    g_port->spi.spi_cs_assert();
    dw1000_spi_transfer(&header, nullptr, 1);

    uint8_t dummy[256];
    uint8_t tx_zero[256] = {0};
    if (len > sizeof(dummy)) len = sizeof(dummy);
    dw1000_spi_transfer(tx_zero, dummy, len);
    for (uint16_t i = 0; i < len; i++) {
        frame[i] = dummy[i];
    }
    g_port->spi.spi_cs_deassert();
    return DW1000_OK;
}

static int dw1000_wait_status(uint32_t mask, uint32_t timeout_us) {
    uint32_t start = dw1000_get_time_us();
    while (1) {
        uint32_t status = dw1000_reg_read32(DW1000_REG_SYS_STATUS);
        if (status & mask) return (int)(status & mask);
        if (dw1000_get_time_us() - start > timeout_us) {
            return DW1000_ERR_TIMEOUT;
        }
    }
}

void dw1000_reset(void) {
    if (!g_port) return;
    g_port->gpio.rst_set(0);
    g_port->delay.delay_ms(2);
    g_port->gpio.rst_set(1);
    g_port->delay.delay_ms(5);
}

int dw1000_is_connected(void) {
    uint32_t dev_id = dw1000_reg_read32(DW1000_REG_DEV_ID);
    return (dev_id == DW1000_DEV_ID_VAL) ? 1 : 0;
}

uint32_t dw1000_get_device_id(void) {
    return dw1000_reg_read32(DW1000_REG_DEV_ID);
}

static int dw1000_load_xtal_trim_internal(uint8_t trim) {
    trim &= DW1000_FS_XTALT_MASK;
    return dw1000_write_sub_reg(DW1000_REG_FS_XTALT,
                                DW1000_FS_XTALT_SUBADDR,
                                &trim, 1);
}

int dw1000_load_xtal_trim(uint8_t trim) {
    return dw1000_load_xtal_trim_internal(trim);
}

static int dw1000_configure_channel(uint8_t channel) {
    uint32_t chan_ctrl = 0;

    switch (channel) {
        case 5:
            chan_ctrl = DW1000_CHAN_CTRL_TX_CHAN_5 |
                        DW1000_CHAN_CTRL_RX_CHAN_5 |
                        DW1000_CHAN_CTRL_TX_PCODE_9 |
                        DW1000_CHAN_CTRL_RX_PCODE_9;
            break;
        default:
            return DW1000_ERR_NO_DEVICE;
    }

    dw1000_reg_write32(DW1000_REG_CHAN_CTRL, chan_ctrl);

    uint8_t rf_tx[3];
    rf_tx[0] = (uint8_t)(DW1000_RF_TXCTRL_CH5 & 0xFF);
    rf_tx[1] = (uint8_t)((DW1000_RF_TXCTRL_CH5 >> 8) & 0xFF);
    rf_tx[2] = (uint8_t)((DW1000_RF_TXCTRL_CH5 >> 16) & 0xFF);
    dw1000_reg_write(DW1000_REG_RF_TXCTRL, rf_tx, 3);

    uint8_t rf_rx = DW1000_RF_RXCTRLH_CH5;
    dw1000_reg_write(DW1000_REG_RF_RXCTRLH, &rf_rx, 1);

    uint8_t agc1[2];
    agc1[0] = (uint8_t)(DW1000_AGC_TUNE1_CH5 & 0xFF);
    agc1[1] = (uint8_t)((DW1000_AGC_TUNE1_CH5 >> 8) & 0xFF);
    dw1000_reg_write(DW1000_REG_AGC_TUNE1, agc1, 2);

    uint8_t agc2[4];
    agc2[0] = (uint8_t)(DW1000_AGC_TUNE2_VAL & 0xFF);
    agc2[1] = (uint8_t)((DW1000_AGC_TUNE2_VAL >> 8) & 0xFF);
    agc2[2] = (uint8_t)((DW1000_AGC_TUNE2_VAL >> 16) & 0xFF);
    agc2[3] = (uint8_t)((DW1000_AGC_TUNE2_VAL >> 24) & 0xFF);
    dw1000_reg_write(DW1000_REG_AGC_TUNE2, agc2, 4);

    uint8_t agc3[2];
    agc3[0] = (uint8_t)(DW1000_AGC_TUNE3_VAL & 0xFF);
    agc3[1] = (uint8_t)((DW1000_AGC_TUNE3_VAL >> 8) & 0xFF);
    dw1000_reg_write(DW1000_REG_AGC_TUNE3, agc3, 2);

    uint8_t ext = DW1000_EXT_SYNC_VAL;
    dw1000_reg_write(DW1000_REG_EXT_SYNC, &ext, 1);

    uint8_t acc = DW1000_ACC_MEM_VAL;
    dw1000_reg_write(DW1000_REG_ACC_MEM, &acc, 1);

    uint8_t lde1[2];
    lde1[0] = (uint8_t)(DW1000_LDE_CFG1_CH5 & 0xFF);
    lde1[1] = (uint8_t)((DW1000_LDE_CFG1_CH5 >> 8) & 0xFF);
    dw1000_reg_write(DW1000_REG_LDE_CFG1, lde1, 2);

    uint8_t lder = DW1000_LDE_REPC_CH5;
    dw1000_reg_write(DW1000_REG_LDE_REPC, &lder, 1);

    return DW1000_OK;
}

static int dw1000_configure_preamble(uint8_t datarate, uint8_t prf) {
    uint32_t sys_cfg = dw1000_reg_read32(DW1000_REG_SYS_CFG);
    sys_cfg &= ~(DW1000_SYS_CFG_RXM110K);
    dw1000_reg_write32(DW1000_REG_SYS_CFG, sys_cfg);

    uint8_t rx_sfd = DW1000_RX_SFD_VAL;
    dw1000_reg_write(DW1000_REG_RX_SFD, &rx_sfd, 1);

    (void)datarate;
    (void)prf;
    return DW1000_OK;
}

int dw1000_set_rx_antenna_delay(uint16_t delay) {
    uint8_t buf[2];
    buf[0] = (uint8_t)(delay & 0xFF);
    buf[1] = (uint8_t)((delay >> 8) & 0xFF);
    return dw1000_reg_write(DW1000_REG_LDE_RXANTD, buf, 2);
}

int dw1000_set_tx_antenna_delay(uint16_t delay) {
    uint8_t buf[2];
    buf[0] = (uint8_t)(delay & 0xFF);
    buf[1] = (uint8_t)((delay >> 8) & 0xFF);
    return dw1000_reg_write(DW1000_REG_TX_ANTD, buf, 2);
}

int dw1000_port_init(const dw1000_port_ops_t *ops) {
    if (!ops) return DW1000_ERR_NO_DEVICE;
    g_port = ops;
    return DW1000_OK;
}

const dw1000_port_ops_t *dw1000_port_get(void) {
    return g_port;
}

int dw1000_init(const dw1000_config_t *config) {
    if (!g_port || !config) return DW1000_ERR_NO_DEVICE;

    g_config = *config;

    if (g_port->spi.spi_init) g_port->spi.spi_init();
    if (g_port->gpio.gpio_init) g_port->gpio.gpio_init();

    dw1000_reset();

    if (!dw1000_is_connected()) {
        return DW1000_ERR_NO_DEVICE;
    }

    dw1000_load_xtal_trim_internal(config->xtal_trim);

    uint32_t pmsc0 = dw1000_reg_read32(DW1000_REG_PMSC_CTRL0);
    pmsc0 |= DW1000_PMSC_CTRL0_FORCE_CLK;
    dw1000_reg_write32(DW1000_REG_PMSC_CTRL0, pmsc0);

    if (dw1000_configure_channel(config->channel) != DW1000_OK) {
        return DW1000_ERR_NO_DEVICE;
    }

    dw1000_configure_preamble(config->datarate, config->prf);

    uint32_t sys_cfg = dw1000_reg_read32(DW1000_REG_SYS_CFG);
    sys_cfg |= DW1000_SYS_CFG_DIS_FC;
    sys_cfg &= ~DW1000_SYS_CFG_DIS_FCE;
    dw1000_reg_write32(DW1000_REG_SYS_CFG, sys_cfg);

    uint8_t rx_fwto[2] = {0x00, 0x00};
    dw1000_reg_write(DW1000_REG_RX_FWTO, rx_fwto, 2);

    dw1000_set_tx_antenna_delay(config->tx_antd);
    dw1000_set_rx_antenna_delay(config->rx_antd);

    dw1000_reg_write32(DW1000_REG_SYS_STATUS, 0xFFFFFFFFUL);

    uint32_t mask = DW1000_SYS_MASK_MTXFRS |
                    DW1000_SYS_MASK_MRXFCG |
                    DW1000_SYS_MASK_MRXFCE |
                    DW1000_SYS_MASK_MRXPHE |
                    DW1000_SYS_MASK_MRXSFDTO |
                    DW1000_SYS_MASK_MRXPTO |
                    DW1000_SYS_MASK_MGPIOIRQ;
    dw1000_reg_write32(DW1000_REG_SYS_MASK, mask);

    return DW1000_OK;
}

int dw1000_tx_send(const uint8_t *frame, uint16_t len, uint32_t flags) {
    if (!frame || len == 0 || len > DW1000_MAX_FRAME_LEN) {
        return DW1000_ERR_FRAME_SIZE;
    }

    if (flags & DW1000_TX_FLAG_DELAYED) {
        uint32_t pmsc0 = dw1000_reg_read32(DW1000_REG_PMSC_CTRL0);
        pmsc0 |= DW1000_PMSC_CTRL0_FORCE_CLK;
        dw1000_reg_write32(DW1000_REG_PMSC_CTRL0, pmsc0);
    }

    dw1000_write_tx_buffer(frame, len);

    uint32_t tx_fctrl = (uint32_t)(len + 2) & DW1000_TX_FCTRL_TFLEN_MASK;
    tx_fctrl |= DW1000_TX_FCTRL_TXBR_850K;
    tx_fctrl |= DW1000_TX_FCTRL_TR_16MHZ;
    tx_fctrl |= DW1000_TX_FCTRL_TXPRF_16MHZ;
    tx_fctrl |= DW1000_TX_FCTRL_TXPSR_256;
    tx_fctrl |= DW1000_TX_FCTRL_PE_16;
    if (flags & DW1000_TX_FLAG_RANGING) {
        tx_fctrl |= DW1000_TX_FCTRL_TXR;
    }

    dw1000_reg_write32(DW1000_REG_TX_FCTRL, tx_fctrl);

    dw1000_reg_write32(DW1000_REG_SYS_STATUS, DW1000_SYS_STATUS_TXFRS);

    uint32_t sys_ctrl = DW1000_SYS_CTRL_TXSTRT;
    if (flags & DW1000_TX_FLAG_DELAYED) {
        sys_ctrl |= DW1000_SYS_CTRL_TXDLYS;
    }
    dw1000_reg_write32(DW1000_REG_SYS_CTRL, sys_ctrl);

    return DW1000_OK;
}

int dw1000_tx_send_at(const uint8_t *frame, uint16_t len, uint64_t tx_time) {
    if (!frame || len == 0 || len > DW1000_MAX_FRAME_LEN) {
        return DW1000_ERR_FRAME_SIZE;
    }

    uint8_t tx_time_buf[5];
    for (int i = 0; i < 5; i++) {
        tx_time_buf[i] = (uint8_t)((tx_time >> (8 * i)) & 0xFF);
    }
    dw1000_reg_write(DW1000_REG_TX_TIME, tx_time_buf, 5);

    uint32_t pmsc0 = dw1000_reg_read32(DW1000_REG_PMSC_CTRL0);
    pmsc0 |= DW1000_PMSC_CTRL0_FORCE_CLK;
    dw1000_reg_write32(DW1000_REG_PMSC_CTRL0, pmsc0);

    dw1000_write_tx_buffer(frame, len);

    uint32_t tx_fctrl = (uint32_t)(len + 2) & DW1000_TX_FCTRL_TFLEN_MASK;
    tx_fctrl |= DW1000_TX_FCTRL_TXBR_850K;
    tx_fctrl |= DW1000_TX_FCTRL_TR_16MHZ;
    tx_fctrl |= DW1000_TX_FCTRL_TXPRF_16MHZ;
    tx_fctrl |= DW1000_TX_FCTRL_TXPSR_256;
    tx_fctrl |= DW1000_TX_FCTRL_PE_16;
    tx_fctrl |= DW1000_TX_FCTRL_TXR;

    dw1000_reg_write32(DW1000_REG_TX_FCTRL, tx_fctrl);
    dw1000_reg_write32(DW1000_REG_SYS_STATUS, DW1000_SYS_STATUS_TXFRS);

    uint32_t sys_ctrl = DW1000_SYS_CTRL_TXSTRT | DW1000_SYS_CTRL_TXDLYS;
    dw1000_reg_write32(DW1000_REG_SYS_CTRL, sys_ctrl);

    return DW1000_OK;
}

int dw1000_tx_wait_done(uint32_t timeout_us) {
    int ret = dw1000_wait_status(DW1000_SYS_STATUS_TXFRS, timeout_us);
    if (ret < 0) return ret;
    dw1000_reg_write32(DW1000_REG_SYS_STATUS, DW1000_SYS_STATUS_TXFRS);
    return DW1000_OK;
}

uint64_t dw1000_get_tx_timestamp(void) {
    uint8_t buf[5] = {0};
    dw1000_reg_read(DW1000_REG_TX_TIME, buf, 5);
    uint64_t ts = 0;
    for (int i = 4; i >= 0; i--) {
        ts = (ts << 8) | buf[i];
    }
    return ts & 0xFFFFFFFFFFULL;
}

int dw1000_rx_start(uint32_t timeout_us) {
    dw1000_reg_write32(DW1000_REG_SYS_STATUS,
                       DW1000_SYS_STATUS_RXFCG |
                       DW1000_SYS_STATUS_RXFCE |
                       DW1000_SYS_STATUS_RXPHE |
                       DW1000_SYS_STATUS_RXRFSL |
                       DW1000_SYS_STATUS_RXRFTO |
                       DW1000_SYS_STATUS_RXPTO |
                       DW1000_SYS_STATUS_RXSFDTO);

    uint8_t fwto[2];
    if (timeout_us > 0) {
        uint32_t fwto_val = timeout_us * 4992 / 10000;
        if (fwto_val > 0xFFFF) fwto_val = 0xFFFF;
        fwto[0] = (uint8_t)(fwto_val & 0xFF);
        fwto[1] = (uint8_t)((fwto_val >> 8) & 0xFF);
    } else {
        fwto[0] = 0;
        fwto[1] = 0;
    }
    dw1000_reg_write(DW1000_REG_RX_FWTO, fwto, 2);

    dw1000_reg_write32(DW1000_REG_SYS_CTRL, DW1000_SYS_CTRL_RXENAB);
    return DW1000_OK;
}

int dw1000_rx_wait_done(uint32_t timeout_us) {
    uint32_t start = dw1000_get_time_us();
    while (1) {
        uint32_t status = dw1000_reg_read32(DW1000_REG_SYS_STATUS);

        if (status & DW1000_SYS_STATUS_RXFCG) {
            dw1000_reg_write32(DW1000_REG_SYS_STATUS, DW1000_SYS_STATUS_RXFCG);
            return DW1000_OK;
        }

        if (status & DW1000_SYS_STATUS_RX_ERROR) {
            dw1000_reg_write32(DW1000_REG_SYS_STATUS, DW1000_SYS_STATUS_RX_ERROR);
            return DW1000_ERR_CRC;
        }

        if (dw1000_get_time_us() - start > timeout_us) {
            dw1000_reg_write32(DW1000_REG_SYS_CTRL, DW1000_SYS_CTRL_TRXOFF);
            return DW1000_ERR_TIMEOUT;
        }
    }
}

int dw1000_read_frame(uint8_t *buf, uint16_t max_len, uint16_t *out_len) {
    if (!buf) return DW1000_ERR_FRAME_SIZE;

    uint32_t rx_finfo = dw1000_reg_read32(DW1000_REG_RX_FINFO);
    uint16_t len = (uint16_t)(rx_finfo & DW1000_RX_FINFO_RFLEN_MASK);
    if (len > max_len) len = max_len;

    if (len > 0) {
        dw1000_read_rx_buffer(buf, len);
    }
    if (out_len) *out_len = len;

    return DW1000_OK;
}

uint64_t dw1000_get_rx_timestamp(void) {
    uint8_t buf[5] = {0};
    dw1000_reg_read(DW1000_REG_RX_TIME, buf, 5);
    uint64_t ts = 0;
    for (int i = 4; i >= 0; i--) {
        ts = (ts << 8) | buf[i];
    }
    return ts & 0xFFFFFFFFFFULL;
}

int dw1000_rx_enable(void) {
    return dw1000_rx_start(0);
}

int dw1000_rx_disable(void) {
    dw1000_reg_write32(DW1000_REG_SYS_CTRL, DW1000_SYS_CTRL_TRXOFF);
    return DW1000_OK;
}

uint32_t dw1000_read_clock_ratio(void) {
    uint32_t val = dw1000_reg_read32(DW1000_REG_DRX_CAR_INT);
    return val & 0x03FFFFFFUL;
}

static int dw1000_sar_trigger(uint8_t channel) {
    uint8_t ctrl = DW1000_SAR_CTRL_START | (channel & 0x07);
    return dw1000_reg_write(DW1000_SAR_CTRL_REG, &ctrl, 1);
}

static int dw1000_sar_wait_done(uint32_t timeout_us) {
    uint32_t start = dw1000_get_time_us();
    while (1) {
        uint8_t status = 0;
        dw1000_reg_read(DW1000_SAR_STATUS_REG, &status, 1);
        if (status & DW1000_SAR_STATUS_DONE) {
            return DW1000_OK;
        }
        if ((uint32_t)(dw1000_get_time_us() - start) > timeout_us) {
            return DW1000_ERR_TIMEOUT;
        }
    }
}

static int dw1000_sar_read_raw(uint16_t *out_raw) {
    if (!out_raw) return DW1000_ERR_FRAME_SIZE;

    uint8_t buf[2] = {0, 0};
    dw1000_reg_read(DW1000_SAR_READ_REG, buf, 2);

    uint16_t raw = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    raw &= 0x0FFF;

    *out_raw = raw;
    return DW1000_OK;
}

static int dw1000_sar_measure(uint8_t channel, uint16_t *out_raw) {
    if (dw1000_sar_trigger(channel) != DW1000_OK) {
        return DW1000_ERR_TIMEOUT;
    }
    if (dw1000_sar_wait_done(DW1000_SAR_TIMEOUT_US) != DW1000_OK) {
        return DW1000_ERR_TIMEOUT;
    }
    return dw1000_sar_read_raw(out_raw);
}

uint8_t dw1000_read_temperature(void) {
    uint16_t raw = 0;
    if (dw1000_sar_measure(DW1000_SAR_CHANNEL_TEMP, &raw) != DW1000_OK) {
        return 0;
    }

    float voltage = (float)raw * DW1000_SAR_VREF / DW1000_SAR_RESOLUTION;
    float temp_c = (voltage - DW1000_SAR_TEMP_OFFSET) / DW1000_SAR_TEMP_SLOPE
                   + DW1000_SAR_TEMP_REF_C;

    if (temp_c < -40.0f) temp_c = -40.0f;
    if (temp_c > 125.0f) temp_c = 125.0f;

    return (uint8_t)temp_c;
}

uint16_t dw1000_read_voltage(void) {
    uint16_t raw = 0;
    if (dw1000_sar_measure(DW1000_SAR_CHANNEL_VBAT, &raw) != DW1000_OK) {
        return 0;
    }

    float voltage = (float)raw * DW1000_SAR_VBAT_SCALE / DW1000_SAR_RESOLUTION;

    if (voltage < 0.0f) voltage = 0.0f;
    if (voltage > 5.0f) voltage = 5.0f;

    return (uint16_t)(voltage * 1000.0f);
}

uint64_t dw1000_get_system_time(void) {
    uint8_t buf[5] = {0};
    dw1000_reg_read(DW1000_REG_SYS_TIME, buf, 5);
    uint64_t ts = 0;
    for (int i = 4; i >= 0; i--) {
        ts = (ts << 8) | buf[i];
    }
    return ts & 0xFFFFFFFFFFULL;
}

uint32_t dw1000_get_time_us(void) {
    if (g_port && g_port->delay.delay_us) {
        static uint32_t counter = 0;
        counter += 1;
        return counter;
    }
    return 0;
}

void dw1000_correct_raw_distance(float raw_distance, float *corrected_distance) {
    if (!corrected_distance) return;

    uint8_t temp = dw1000_read_temperature();
    float delta_t = (float)temp - DW1000_THERMAL_REF_C;
    float correction = (DW1000_THERMAL_SLOPE_MM_PER_C * delta_t) / 10.0f;

    *corrected_distance = raw_distance - correction;
    if (*corrected_distance < 0.0f) *corrected_distance = 0.0f;
}
