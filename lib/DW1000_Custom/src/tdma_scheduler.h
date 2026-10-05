#ifndef TDMA_SCHEDULER_H
#define TDMA_SCHEDULER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TDMA_SLOT_US                15000
#define TDMA_FRAME_SLOTS            16
#define TDMA_FRAME_US               (TDMA_SLOT_US * TDMA_FRAME_SLOTS)
#define TDMA_GUARD_US               2000
#define TDMA_TX_OFFSET_US           2000
#define TDMA_RX_TIMEOUT_US          8000
#define TDMA_RESP_DELAY_US          2500
#define TDMA_MAX_PEERS              2
#define TDMA_BEACON_SLOT_BASE       0
#define TDMA_RANGING_SLOT_BASE      3
#define TDMA_LISTEN_SLOT_BASE       12

typedef enum {
    TDMA_ROLE_IDLE = 0,
    TDMA_ROLE_BEACON_TX,
    TDMA_ROLE_RANGING_TX,
    TDMA_ROLE_RANGING_RX,
    TDMA_ROLE_LISTEN,
    TDMA_ROLE_MARGIN
} tdma_role_t;

typedef enum {
    TDMA_OK = 0,
    TDMA_ERR_NOT_INIT = -1,
    TDMA_ERR_INVALID_SLOT = -2,
    TDMA_ERR_INVALID_ROBOT = -3,
    TDMA_ERR_RANGING_FAIL = -4,
    TDMA_ERR_SYNC_LOST = -5
} tdma_status_t;

typedef struct {
    uint8_t robot_id;
    uint8_t peer_id;
    uint8_t slot_index;
    tdma_role_t role;
    uint8_t active;
} tdma_slot_entry_t;

typedef struct {
    uint8_t  robot_id;
    uint8_t  initialized;
    uint8_t  slot_owner[TDMA_FRAME_SLOTS];
    uint8_t  slot_peer[TDMA_FRAME_SLOTS];
    uint8_t  slot_role[TDMA_FRAME_SLOTS];
    uint8_t  peer_count;
    uint8_t  peers[TDMA_MAX_PEERS];
    uint32_t slot_start_us;
    uint32_t frame_start_us;
    uint32_t last_beacon_us;
    uint32_t sync_error_us;
    uint32_t n_peers;
    uint32_t n_ranging_ok;
    uint32_t n_ranging_fail;
} tdma_state_t;

int      tdma_scheduler_init(uint8_t robot_id);
void     tdma_scheduler_reset(void);
int      tdma_scheduler_set_slot(uint8_t slot, tdma_role_t role,
                                 uint8_t owner, uint8_t peer);
tdma_role_t tdma_scheduler_get_role(uint8_t slot);
uint8_t  tdma_scheduler_get_peer(uint8_t slot);
uint8_t  tdma_scheduler_get_owner(uint8_t slot);

uint32_t tdma_scheduler_slot_duration_us(void);
uint32_t tdma_scheduler_frame_duration_us(void);
uint8_t  tdma_scheduler_get_current_slot(uint64_t now_us);

int      tdma_scheduler_sync_beacon(uint64_t rx_timestamp_us);
uint32_t tdma_scheduler_get_sync_error_us(void);

int      tdma_scheduler_run_slot(uint8_t slot);
int      tdma_scheduler_run_frame(void);

uint32_t tdma_scheduler_get_peer_distance(uint8_t peer_id);
uint32_t tdma_scheduler_get_n_peers(void);
uint32_t tdma_scheduler_get_n_ranging_ok(void);
uint32_t tdma_scheduler_get_n_ranging_fail(void);

const tdma_state_t *tdma_scheduler_get_state(void);

#ifdef __cplusplus
}
#endif

#endif
