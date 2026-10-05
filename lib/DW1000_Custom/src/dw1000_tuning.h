#ifndef DW1000_TUNING_H
#define DW1000_TUNING_H

#include <stdint.h>

#define DW1000_TX_ANTD_R2           24626
#define DW1000_TX_ANTD_R3           24689
#define DW1000_TX_ANTD_R4           24786

#define DW1000_LDE_RXANTD_R2        24626
#define DW1000_LDE_RXANTD_R3        24689
#define DW1000_LDE_RXANTD_R4        24786

#define DW1000_TX_ANTD_DEFAULT      16436
#define DW1000_LDE_RXANTD_DEFAULT   16436

#define DW1000_XTAL_TRIM_OTP_DEFAULT 0x1F
#define DW1000_XTAL_TRIM_OTP_ADDR    0x01
#define DW1000_XTAL_TRIM_SUBADDR     0x0E

#define DW1000_THERMAL_SLOPE_MM_PER_C   2.15f
#define DW1000_THERMAL_REF_C            25.0f
#define DW1000_THERMAL_MAX_DELTA_C      20.0f

#define DW1000_CLOCK_RATIO_NOMINAL      1.0f
#define DW1000_CLOCK_RATIO_MIN          0.9f
#define DW1000_CLOCK_RATIO_MAX          1.1f
#define DW1000_CLOCK_RATIO_SHIFT        26

#define DW1000_LDE_CFG1_CH5             0x0607
#define DW1000_LDE_REPC_CH5             0x2E
#define DW1000_LDE_CFG2_CH5             0x1607
#define DW1000_LDE_RXANTD_CH5           0x1637

#define DW1000_AGC_TUNE1_CH5            0x8870
#define DW1000_AGC_TUNE2_CH5            0x2502A907UL
#define DW1000_AGC_TUNE3_CH5            0x0035

#define DW1000_EXT_SYNC_CH5             0x0010
#define DW1000_ACC_MEM_CH5              0x0F

#define DW1000_RF_TXCTRL_CH5            0x1E3FE3UL
#define DW1000_RF_RXCTRLH_CH5           0xD8

#define DW1000_FS_PLLCFG_CH5            0x0E
#define DW1000_FS_PLLTUNE_CH5           0x00
#define DW1000_FS_XTALT_CH5             0x00

#define DW1000_RANGING_DELAY_US         2500
#define DW1000_RANGING_MAX_DISTANCE_M   200
#define DW1000_RANGING_MIN_DISTANCE_M   0
#define DW1000_RANGING_TIMEOUT_US       8000

#define DW1000_CFO_CORRECTION_ENABLE    1
#define DW1000_CFO_CORRECTION_DISABLE   0

#define DW1000_UWB_RMSE_M               0.1383f
#define DW1000_UWB_VARIANCE_M2          (0.1383f * 0.1383f)

#define DW1000_ANTD_R2                  0
#define DW1000_ANTD_R3                  1
#define DW1000_ANTD_R4                  2

static const uint16_t dw1000_tx_antd_table[3] = {
    DW1000_TX_ANTD_R2,
    DW1000_TX_ANTD_R3,
    DW1000_TX_ANTD_R4
};

static const uint16_t dw1000_rx_antd_table[3] = {
    DW1000_LDE_RXANTD_R2,
    DW1000_LDE_RXANTD_R3,
    DW1000_LDE_RXANTD_R4
};

static inline uint16_t dw1000_get_tx_antd(uint8_t robot_index) {
    if (robot_index > 2) return DW1000_TX_ANTD_DEFAULT;
    return dw1000_tx_antd_table[robot_index];
}

static inline uint16_t dw1000_get_rx_antd(uint8_t robot_index) {
    if (robot_index > 2) return DW1000_LDE_RXANTD_DEFAULT;
    return dw1000_rx_antd_table[robot_index];
}

#endif
