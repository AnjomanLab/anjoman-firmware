#include "ss_twr.h"
#include "dw1000_core.h"
#include <math.h>

#define SS_TWR_POLL_LEN             12
#define SS_TWR_RESP_LEN             16
#define SS_TWR_FINAL_LEN            12

#define SS_TWR_RESP_DELAY_US        2500
#define SS_TWR_RX_TIMEOUT_US        8000

#define SS_TWR_MSG_POLL             0x41
#define SS_TWR_MSG_RESP             0x42
#define SS_TWR_MSG_FINAL            0x43

static uint16_t g_local_addr = 0;
static uint16_t g_peer_addr  = 0;

static void write_u16(uint8_t *buf, uint16_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

static uint16_t read_u16(const uint8_t *buf) {
    return (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
}

static void write_u32(uint8_t *buf, uint32_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

static uint32_t read_u32(const uint8_t *buf) {
    return (uint32_t)(buf[0] | ((uint32_t)buf[1] << 8) |
                      ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24));
}

static void write_u40(uint8_t *buf, uint64_t val) {
    for (int i = 0; i < 5; i++) {
        buf[i] = (uint8_t)((val >> (8 * i)) & 0xFF);
    }
}

static uint64_t read_u40(const uint8_t *buf) {
    uint64_t val = 0;
    for (int i = 4; i >= 0; i--) {
        val = (val << 8) | buf[i];
    }
    return val;
}

void ss_twr_init(uint16_t local_addr) {
    g_local_addr = local_addr;
}

int ss_twr_measure_initiator(uint16_t peer_addr, uint32_t *out_distance_cm,
                              float *out_clock_ratio) {
    uint8_t poll_frame[SS_TWR_POLL_LEN] = {0};
    uint8_t resp_frame[SS_TWR_RESP_LEN] = {0};
    int ret;

    poll_frame[0] = SS_TWR_MSG_POLL;
    poll_frame[1] = 0x88;
    write_u16(&poll_frame[2], peer_addr);
    write_u16(&poll_frame[4], g_local_addr);

    ret = dw1000_tx_send(poll_frame, SS_TWR_POLL_LEN, DW1000_TX_FLAG_RANGING);
    if (ret != DW1000_OK) return ret;

    ret = dw1000_tx_wait_done(SS_TWR_RX_TIMEOUT_US);
    if (ret != DW1000_OK) return ret;

    uint64_t t_poll_tx = dw1000_get_tx_timestamp();

    ret = dw1000_rx_start(SS_TWR_RX_TIMEOUT_US);
    if (ret != DW1000_OK) return ret;

    ret = dw1000_rx_wait_done(SS_TWR_RX_TIMEOUT_US);
    if (ret != DW1000_OK) return ret;

    uint64_t t_resp_rx = dw1000_get_rx_timestamp();

    uint16_t rx_len = 0;
    ret = dw1000_read_frame(resp_frame, SS_TWR_RESP_LEN, &rx_len);
    if (ret != DW1000_OK) return ret;

    if (resp_frame[0] != SS_TWR_MSG_RESP) return DW1000_ERR_RX_FAIL;

    uint64_t t_reply_40 = read_u40(&resp_frame[8]);

    uint64_t t_round = (t_resp_rx - t_poll_tx) & 0xFFFFFFFFFFULL;
    uint64_t t_reply = t_reply_40 & 0xFFFFFFFFFFULL;

    if (t_reply >= t_round) {
        return DW1000_ERR_RX_FAIL;
    }

    uint32_t raw_ratio = dw1000_read_clock_ratio();
    float clock_ratio = (float)raw_ratio / (float)(1UL << 26);
    if (clock_ratio < 0.9f || clock_ratio > 1.1f) {
        clock_ratio = 1.0f;
    }

    float t_round_f = (float)t_round;
    float t_reply_f = (float)t_reply;
    float t_prop = (t_round_f * clock_ratio - t_reply_f) / 2.0f;

    float distance_cm = t_prop * DW1000_TICK_TO_CM;

    if (distance_cm < 0.0f) distance_cm = 0.0f;

    *out_distance_cm = (uint32_t)distance_cm;
    if (out_clock_ratio) *out_clock_ratio = clock_ratio;

    return DW1000_OK;
}

int ss_twr_responder(uint32_t timeout_us) {
    uint8_t poll_frame[SS_TWR_POLL_LEN] = {0};
    uint8_t resp_frame[SS_TWR_RESP_LEN] = {0};
    int ret;

    ret = dw1000_rx_start(timeout_us);
    if (ret != DW1000_OK) return ret;

    ret = dw1000_rx_wait_done(timeout_us);
    if (ret != DW1000_OK) return ret;

    uint64_t t_poll_rx = dw1000_get_rx_timestamp();

    uint16_t rx_len = 0;
    ret = dw1000_read_frame(poll_frame, SS_TWR_POLL_LEN, &rx_len);
    if (ret != DW1000_OK) return ret;

    if (poll_frame[0] != SS_TWR_MSG_POLL) return DW1000_ERR_RX_FAIL;

    uint16_t initiator_addr = read_u16(&poll_frame[4]);

    resp_frame[0] = SS_TWR_MSG_RESP;
    resp_frame[1] = 0x88;
    write_u16(&resp_frame[2], initiator_addr);
    write_u16(&resp_frame[4], g_local_addr);

    uint64_t t_resp_tx_target = t_poll_rx + (uint64_t)(SS_TWR_RESP_DELAY_US * 499.2f);

    write_u40(&resp_frame[8], 0);

    ret = dw1000_tx_send_at(resp_frame, SS_TWR_RESP_LEN, t_resp_tx_target);
    if (ret != DW1000_OK) return ret;

    ret = dw1000_tx_wait_done(timeout_us);
    if (ret != DW1000_OK) return ret;

    uint64_t t_resp_tx = dw1000_get_tx_timestamp();
    uint64_t t_reply = (t_resp_tx - t_poll_rx) & 0xFFFFFFFFFFULL;

    write_u40(&resp_frame[8], t_reply);

    return DW1000_OK;
}

int ss_twr_measure_roundtrip(uint16_t peer_addr, uint32_t *out_distance_cm,
                              float *out_clock_ratio) {
    int ret;

    ret = ss_twr_responder(SS_TWR_RX_TIMEOUT_US);
    if (ret != DW1000_OK) return ret;

    return ss_twr_measure_initiator(peer_addr, out_distance_cm, out_clock_ratio);
}

void ss_twr_get_stats(float *out_last_ratio, uint32_t *out_last_distance) {
    if (out_last_ratio) *out_last_ratio = 1.0f;
    if (out_last_distance) *out_last_distance = 0;
}
