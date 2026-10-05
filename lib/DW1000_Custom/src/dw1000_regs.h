#ifndef DW1000_REGS_H
#define DW1000_REGS_H

#include <stdint.h>

#define DW1000_REG_DEV_ID           0x00
#define DW1000_REG_EUI              0x01
#define DW1000_REG_PANADR           0x03
#define DW1000_REG_SYS_CFG          0x04
#define DW1000_REG_SYS_TIME         0x06
#define DW1000_REG_TX_FCTRL         0x08
#define DW1000_REG_TX_BUFFER        0x09
#define DW1000_REG_RX_FWTO          0x0C
#define DW1000_REG_SYS_CTRL         0x0D
#define DW1000_REG_SYS_MASK         0x0E
#define DW1000_REG_SYS_STATUS       0x0F
#define DW1000_REG_RX_FINFO         0x10
#define DW1000_REG_RX_BUFFER        0x11
#define DW1000_REG_RX_FQUAL         0x13
#define DW1000_REG_RX_TIME          0x14
#define DW1000_REG_TX_TIME          0x15
#define DW1000_REG_TX_RAW_STAMP     0x16
#define DW1000_REG_RX_RAW_STAMP     0x17
#define DW1000_REG_TX_ANTD          0x18
#define DW1000_REG_LDE_RXANTD       0x18
#define DW1000_REG_RX_SFD           0x1D
#define DW1000_REG_CHAN_CTRL        0x1F
#define DW1000_REG_AGC_TUNE1        0x23
#define DW1000_REG_AGC_TUNE2        0x24
#define DW1000_REG_AGC_TUNE3        0x24
#define DW1000_REG_EXT_SYNC         0x24
#define DW1000_REG_ACC_MEM          0x25
#define DW1000_REG_GPIO_CTRL        0x26
#define DW1000_REG_DRX_CAR_INT      0x27
#define DW1000_REG_LDE_REPC         0x28
#define DW1000_REG_RF_RXCTRLH       0x2B
#define DW1000_REG_RF_TXCTRL        0x2C
#define DW1000_REG_OTP_IF           0x2D
#define DW1000_REG_LDE_CFG1         0x2E
#define DW1000_REG_LDE_CFG2         0x2E
#define DW1000_REG_LDE_IF           0x2E
#define DW1000_REG_FS_XTALT         0x2E
#define DW1000_REG_FS_CTRL          0x35
#define DW1000_REG_PMSC_CTRL0       0x36
#define DW1000_REG_PMSC_CTRL1       0x37
#define DW1000_REG_PMSC_LEDC        0x28

#define DW1000_DEV_ID_VAL           0xDECA0130UL
#define DW1000_DEV_ID_MASK          0xFFFFFFFFUL

#define DW1000_SYS_CFG_FFEN         (1UL << 0)
#define DW1000_SYS_CFG_FFBC         (1UL << 1)
#define DW1000_SYS_CFG_FFAB         (1UL << 2)
#define DW1000_SYS_CFG_FFAD         (1UL << 3)
#define DW1000_SYS_CFG_FFAA         (1UL << 4)
#define DW1000_SYS_CFG_FFAM         (1UL << 5)
#define DW1000_SYS_CFG_FFAR         (1UL << 6)
#define DW1000_SYS_CFG_FFA4         (1UL << 7)
#define DW1000_SYS_CFG_FFA5         (1UL << 8)
#define DW1000_SYS_CFG_HIRQ_POL     (1UL << 9)
#define DW1000_SYS_CFG_DIS_DRXB     (1UL << 12)
#define DW1000_SYS_CFG_DIS_PHE      (1UL << 16)
#define DW1000_SYS_CFG_DIS_RSDE     (1UL << 17)
#define DW1000_SYS_CFG_DIS_FCE      (1UL << 18)
#define DW1000_SYS_CFG_DIS_FC       (1UL << 19)
#define DW1000_SYS_CFG_DIS_STXP     (1UL << 20)
#define DW1000_SYS_CFG_DIS_PHY      (1UL << 22)
#define DW1000_SYS_CFG_RXM110K      (1UL << 24)
#define DW1000_SYS_CFG_RX_SFD_MT    (1UL << 25)

#define DW1000_SYS_CTRL_TXSTRT      (1UL << 1)
#define DW1000_SYS_CTRL_TXDLYS      (1UL << 2)
#define DW1000_SYS_CTRL_TRXOFF      (1UL << 6)
#define DW1000_SYS_CTRL_WAIT4RESP   (1UL << 7)
#define DW1000_SYS_CTRL_RXENAB      (1UL << 0)
#define DW1000_SYS_CTRL_RXDLYE      (1UL << 3)
#define DW1000_SYS_CTRL_HRBST       (1UL << 4)
#define DW1000_SYS_CTRL_TXRXOFF     (1UL << 5)

#define DW1000_SYS_STATUS_TXFRS     (1UL << 0)
#define DW1000_SYS_STATUS_RXPHE     (1UL << 1)
#define DW1000_SYS_STATUS_RXDFR     (1UL << 2)
#define DW1000_SYS_STATUS_RXFCG     (1UL << 3)
#define DW1000_SYS_STATUS_RXFCE     (1UL << 4)
#define DW1000_SYS_STATUS_RXRFSL    (1UL << 5)
#define DW1000_SYS_STATUS_RXRFTO    (1UL << 6)
#define DW1000_SYS_STATUS_LDEERR    (1UL << 7)
#define DW1000_SYS_STATUS_RXPTO     (1UL << 8)
#define DW1000_SYS_STATUS_RXSFDTO   (1UL << 9)
#define DW1000_SYS_STATUS_GPIOIRQ   (1UL << 10)
#define DW1000_SYS_STATUS_SLPDBG    (1UL << 11)
#define DW1000_SYS_STATUS_SPIERR    (1UL << 12)
#define DW1000_SYS_STATUS_TXFHS     (1UL << 14)
#define DW1000_SYS_STATUS_AAT       (1UL << 16)

#define DW1000_SYS_STATUS_RX_ERROR  (DW1000_SYS_STATUS_RXPHE | \
                                     DW1000_SYS_STATUS_RXFCE | \
                                     DW1000_SYS_STATUS_RXRFSL | \
                                     DW1000_SYS_STATUS_RXRFTO | \
                                     DW1000_SYS_STATUS_RXPTO | \
                                     DW1000_SYS_STATUS_RXSFDTO)

#define DW1000_SYS_MASK_MTXFRS      (1UL << 1)
#define DW1000_SYS_MASK_MRXPHE      (1UL << 2)
#define DW1000_SYS_MASK_MRXDFR      (1UL << 3)
#define DW1000_SYS_MASK_MRXFCG      (1UL << 4)
#define DW1000_SYS_MASK_MRXFCE      (1UL << 5)
#define DW1000_SYS_MASK_MLDEERR     (1UL << 8)
#define DW1000_SYS_MASK_MRXPTO      (1UL << 9)
#define DW1000_SYS_MASK_MRXSFDTO    (1UL << 10)
#define DW1000_SYS_MASK_MGPIOIRQ    (1UL << 11)
#define DW1000_SYS_MASK_MCPLOCK     (1UL << 15)

#define DW1000_TX_FCTRL_TFLEN_MASK  0x000007FFUL
#define DW1000_TX_FCTRL_TFLE        (1UL << 13)
#define DW1000_TX_FCTRL_TXBR_110K   (0x00UL << 15)
#define DW1000_TX_FCTRL_TXBR_850K   (0x01UL << 15)
#define DW1000_TX_FCTRL_TXBR_6M8    (0x02UL << 15)
#define DW1000_TX_FCTRL_TR_16MHZ    (0x01UL << 17)
#define DW1000_TX_FCTRL_TR_64MHZ    (0x02UL << 17)
#define DW1000_TX_FCTRL_TXPRF_16MHZ (0x01UL << 20)
#define DW1000_TX_FCTRL_TXPRF_64MHZ (0x02UL << 20)
#define DW1000_TX_FCTRL_TXPSR_16    (0x01UL << 22)
#define DW1000_TX_FCTRL_TXPSR_32    (0x02UL << 22)
#define DW1000_TX_FCTRL_TXPSR_64    (0x03UL << 22)
#define DW1000_TX_FCTRL_TXPSR_128   (0x04UL << 22)
#define DW1000_TX_FCTRL_TXPSR_256   (0x05UL << 22)
#define DW1000_TX_FCTRL_TXPSR_512   (0x06UL << 22)
#define DW1000_TX_FCTRL_TXPSR_1024  (0x07UL << 22)
#define DW1000_TX_FCTRL_PE_16       (0x00UL << 24)
#define DW1000_TX_FCTRL_PE_32       (0x01UL << 24)
#define DW1000_TX_FCTRL_PE_64       (0x02UL << 24)
#define DW1000_TX_FCTRL_PE_128      (0x03UL << 24)
#define DW1000_TX_FCTRL_TXBOFFS     (0x00UL << 26)
#define DW1000_TX_FCTRL_TXR         (1UL << 30)

#define DW1000_RX_FINFO_RFLEN_MASK  0x000003FFUL
#define DW1000_RX_FINFO_RXBR_110K   (0x00UL << 11)
#define DW1000_RX_FINFO_RXBR_850K   (0x01UL << 11)
#define DW1000_RX_FINFO_RXBR_6M8    (0x02UL << 11)
#define DW1000_RX_FINFO_RPSR_16     (0x01UL << 16)
#define DW1000_RX_FINFO_RPSR_256    (0x05UL << 16)
#define DW1000_RX_FINFO_RXPRF_16    (0x01UL << 18)
#define DW1000_RX_FINFO_RXPRF_64    (0x02UL << 18)
#define DW1000_RX_FINFO_RNG         (1UL << 20)
#define DW1000_RX_FINFO_RXPACC_MASK (0x1FUL << 21)

#define DW1000_PMSC_CTRL0_SYS_CLK_EN (1UL << 0)
#define DW1000_PMSC_CTRL0_RX_CLK_EN  (1UL << 1)
#define DW1000_PMSC_CTRL0_TX_CLK_EN  (1UL << 2)
#define DW1000_PMSC_CTRL0_FORCE_CLK  (DW1000_PMSC_CTRL0_SYS_CLK_EN | \
                                      DW1000_PMSC_CTRL0_TX_CLK_EN)

#define DW1000_CHAN_CTRL_TX_CHAN_5   (0x05UL << 0)
#define DW1000_CHAN_CTRL_RX_CHAN_5   (0x05UL << 4)
#define DW1000_CHAN_CTRL_DWSFD       (1UL << 17)
#define DW1000_CHAN_CTRL_TNSSFD      (1UL << 18)
#define DW1000_CHAN_CTRL_RNSSFD      (1UL << 19)
#define DW1000_CHAN_CTRL_TX_PCODE_9  (0x09UL << 22)
#define DW1000_CHAN_CTRL_RX_PCODE_9  (0x09UL << 27)

#define DW1000_FS_XTALT_MASK         0x1FUL
#define DW1000_FS_XTALT_MSB_MASK     0x60UL
#define DW1000_FS_XTALT_SUBADDR      0x0E

#define DW1000_OTP_IF_ADDR           0x04
#define DW1000_OTP_IF_CTRL           0x06
#define DW1000_OTP_IF_RDAT           0x08

#define DW1000_SAR_CTRL_REG          0x31
#define DW1000_SAR_CTRL_LDO          0x01
#define DW1000_SAR_CTRL_TRIM         0x02
#define DW1000_SAR_CTRL_START        0x80
#define DW1000_SAR_STATUS_REG        0x32
#define DW1000_SAR_STATUS_DONE       0x04
#define DW1000_SAR_DATA_REG          0x32
#define DW1000_SAR_READ_REG          0x33

#define DW1000_RF_TXCTRL_CH5         0x1E3FE3UL
#define DW1000_RF_RXCTRLH_CH5        0xD8
#define DW1000_RF_RXCTRLH_LEN        2
#define DW1000_AGC_TUNE1_CH5         0x8870
#define DW1000_AGC_TUNE2_VAL         0x2502A907UL
#define DW1000_AGC_TUNE3_VAL         0x0035
#define DW1000_EXT_SYNC_VAL          0x0010
#define DW1000_ACC_MEM_VAL           0x0F
#define DW1000_LDE_CFG1_CH5          0x0607
#define DW1000_LDE_REPC_CH5          0x2E
#define DW1000_FS_PLLCFG_CH5         0x0E
#define DW1000_FS_PLLTUNE_CH5        0x00
#define DW1000_RX_SFD_VAL            0x00

#define DW1000_SPI_READ_HEADER(addr)  ((uint8_t)((addr) & 0x3F))
#define DW1000_SPI_WRITE_HEADER(addr) ((uint8_t)(0x80 | ((addr) & 0x3F)))

#endif
