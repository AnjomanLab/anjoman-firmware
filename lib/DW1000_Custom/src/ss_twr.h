#ifndef SS_TWR_H
#define SS_TWR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ss_twr_init(uint16_t local_addr);
int ss_twr_measure_initiator(uint16_t peer_addr, uint32_t *out_distance_cm,
                              float *out_clock_ratio);
int ss_twr_responder(uint32_t timeout_us);
int ss_twr_measure_roundtrip(uint16_t peer_addr, uint32_t *out_distance_cm,
                              float *out_clock_ratio);
void ss_twr_get_stats(float *out_last_ratio, uint32_t *out_last_distance);

#ifdef __cplusplus
}
#endif

#endif
