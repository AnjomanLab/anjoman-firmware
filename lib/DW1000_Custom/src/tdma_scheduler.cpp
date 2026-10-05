#include "tdma_scheduler.h"
#include "ss_twr.h"
#include "dw1000_core.h"
#include <string.h>

static tdma_state_t g_tdma;
static uint32_t g_peer_distance[TDMA_MAX_PEERS + 1];
static float    g_peer_clock_ratio[TDMA_MAX_PEERS + 1];

static int tdma_is_owner(uint8_t slot, uint8_t robot_id) {
    return g_tdma.slot_owner[slot] == robot_id;
}

static int tdma_is_peer(uint8_t slot, uint8_t robot_id) {
    return g_tdma.slot_peer[slot] == robot_id;
}

static int tdma_is_beacon_slot(uint8_t slot) {
    return g_tdma.slot_role[slot] == TDMA_ROLE_BEACON_TX;
}

static void tdma_assign_slot(uint8_t slot, tdma_role_t role,
                              uint8_t owner, uint8_t peer) {
    g_tdma.slot_owner[slot] = owner;
    g_tdma.slot_peer[slot]  = peer;
    g_tdma.slot_role[slot]  = (uint8_t)role;
}

int tdma_scheduler_init(uint8_t robot_id) {
    if (robot_id < 2 || robot_id > 4) return TDMA_ERR_INVALID_ROBOT;

    memset(&g_tdma, 0, sizeof(g_tdma));
    memset(g_peer_distance, 0, sizeof(g_peer_distance));
    memset(g_peer_clock_ratio, 0, sizeof(g_peer_clock_ratio));

    g_tdma.robot_id    = robot_id;
    g_tdma.initialized = 1;

    for (uint8_t i = 0; i < TDMA_FRAME_SLOTS; i++) {
        g_tdma.slot_owner[i] = 0;
        g_tdma.slot_peer[i]  = 0;
        g_tdma.slot_role[i]  = (uint8_t)TDMA_ROLE_IDLE;
    }

    tdma_assign_slot(0, TDMA_ROLE_BEACON_TX, 2, 0);
    tdma_assign_slot(1, TDMA_ROLE_BEACON_TX, 3, 0);
    tdma_assign_slot(2, TDMA_ROLE_BEACON_TX, 4, 0);

    tdma_assign_slot(3, TDMA_ROLE_RANGING_TX, 2, 3);
    tdma_assign_slot(4, TDMA_ROLE_RANGING_TX, 3, 2);
    tdma_assign_slot(5, TDMA_ROLE_RANGING_TX, 2, 4);
    tdma_assign_slot(6, TDMA_ROLE_RANGING_TX, 4, 2);
    tdma_assign_slot(7, TDMA_ROLE_RANGING_TX, 3, 4);
    tdma_assign_slot(8, TDMA_ROLE_RANGING_TX, 4, 3);

    for (uint8_t i = 9; i < TDMA_FRAME_SLOTS; i++) {
        g_tdma.slot_role[i] = (uint8_t)TDMA_ROLE_MARGIN;
    }

    g_tdma.peer_count = 0;
    if (robot_id == 2) {
        g_tdma.peers[0] = 3;
        g_tdma.peers[1] = 4;
        g_tdma.peer_count = 2;
    } else if (robot_id == 3) {
        g_tdma.peers[0] = 2;
        g_tdma.peers[1] = 4;
        g_tdma.peer_count = 2;
    } else if (robot_id == 4) {
        g_tdma.peers[0] = 2;
        g_tdma.peers[1] = 3;
        g_tdma.peer_count = 2;
    }

    ss_twr_init(robot_id);

    return TDMA_OK;
}

void tdma_scheduler_reset(void) {
    g_tdma.n_peers          = 0;
    g_tdma.n_ranging_ok     = 0;
    g_tdma.n_ranging_fail   = 0;
    g_tdma.sync_error_us    = 0;
    g_tdma.last_beacon_us   = 0;
}

int tdma_scheduler_set_slot(uint8_t slot, tdma_role_t role,
                             uint8_t owner, uint8_t peer) {
    if (slot >= TDMA_FRAME_SLOTS) return TDMA_ERR_INVALID_SLOT;
    tdma_assign_slot(slot, role, owner, peer);
    return TDMA_OK;
}

tdma_role_t tdma_scheduler_get_role(uint8_t slot) {
    if (slot >= TDMA_FRAME_SLOTS) return TDMA_ROLE_IDLE;
    return (tdma_role_t)g_tdma.slot_role[slot];
}

uint8_t tdma_scheduler_get_peer(uint8_t slot) {
    if (slot >= TDMA_FRAME_SLOTS) return 0;
    return g_tdma.slot_peer[slot];
}

uint8_t tdma_scheduler_get_owner(uint8_t slot) {
    if (slot >= TDMA_FRAME_SLOTS) return 0;
    return g_tdma.slot_owner[slot];
}

uint32_t tdma_scheduler_slot_duration_us(void) {
    return TDMA_SLOT_US;
}

uint32_t tdma_scheduler_frame_duration_us(void) {
    return TDMA_FRAME_US;
}

uint8_t tdma_scheduler_get_current_slot(uint64_t now_us) {
    if (g_tdma.frame_start_us == 0) {
        g_tdma.frame_start_us = (uint32_t)now_us;
    }
    uint32_t elapsed = (uint32_t)now_us - g_tdma.frame_start_us;
    uint32_t slot_idx = elapsed / TDMA_SLOT_US;
    if (slot_idx >= TDMA_FRAME_SLOTS) {
        g_tdma.frame_start_us += TDMA_FRAME_US;
        slot_idx = 0;
    }
    return (uint8_t)slot_idx;
}

int tdma_scheduler_sync_beacon(uint64_t rx_timestamp_us) {
    uint32_t rx_us = (uint32_t)rx_timestamp_us;
    if (g_tdma.last_beacon_us == 0) {
        g_tdma.last_beacon_us = rx_us;
        g_tdma.frame_start_us = rx_us;
        g_tdma.sync_error_us  = 0;
        return TDMA_OK;
    }

    uint32_t expected = g_tdma.frame_start_us + TDMA_FRAME_US;
    int32_t error = (int32_t)(rx_us - expected);
    if (error < 0) error = -error;

    g_tdma.sync_error_us = (uint32_t)error;
    g_tdma.last_beacon_us = rx_us;

    if (error > (int32_t)(TDMA_SLOT_US / 2)) {
        g_tdma.frame_start_us = rx_us;
    }

    return TDMA_OK;
}

uint32_t tdma_scheduler_get_sync_error_us(void) {
    return g_tdma.sync_error_us;
}

int tdma_scheduler_run_slot(uint8_t slot) {
    if (!g_tdma.initialized) return TDMA_ERR_NOT_INIT;
    if (slot >= TDMA_FRAME_SLOTS) return TDMA_ERR_INVALID_SLOT;

    uint8_t robot_id = g_tdma.robot_id;
    tdma_role_t role = (tdma_role_t)g_tdma.slot_role[slot];

    if (role == TDMA_ROLE_MARGIN || role == TDMA_ROLE_IDLE) {
        return TDMA_OK;
    }

    if (role == TDMA_ROLE_BEACON_TX) {
        if (g_tdma.slot_owner[slot] == robot_id) {
            return TDMA_OK;
        }
        return TDMA_OK;
    }

    if (role == TDMA_ROLE_RANGING_TX) {
        uint8_t owner = g_tdma.slot_owner[slot];
        uint8_t peer  = g_tdma.slot_peer[slot];

        if (owner == robot_id) {
            dw1000_port_delay_us(TDMA_GUARD_US);
            dw1000_port_delay_us(TDMA_TX_OFFSET_US);

            uint32_t distance_cm = 0;
            float ratio = 1.0f;

            int ret = ss_twr_measure_initiator((uint16_t)peer,
                                                &distance_cm, &ratio);
            if (ret == DW1000_OK && distance_cm > 0) {
                g_peer_distance[peer] = distance_cm;
                g_peer_clock_ratio[peer] = ratio;
                g_tdma.n_ranging_ok++;
                g_tdma.n_peers = g_tdma.peer_count;
            } else {
                g_tdma.n_ranging_fail++;
            }
            return TDMA_OK;
        }

        if (peer == robot_id) {
            uint32_t timeout = TDMA_SLOT_US - TDMA_GUARD_US;
            int ret = ss_twr_responder(timeout);
            if (ret != DW1000_OK) {
                g_tdma.n_ranging_fail++;
            }
            return TDMA_OK;
        }

        return TDMA_OK;
    }

    return TDMA_OK;
}

int tdma_scheduler_run_frame(void) {
    if (!g_tdma.initialized) return TDMA_ERR_NOT_INIT;

    uint64_t frame_start = dw1000_port_get_time_us();
    g_tdma.frame_start_us = (uint32_t)frame_start;

    for (uint8_t slot = 0; slot < TDMA_FRAME_SLOTS; slot++) {
        uint64_t slot_start = frame_start + ((uint64_t)slot * TDMA_SLOT_US);

        while (dw1000_port_get_time_us() < slot_start) {
        }

        tdma_scheduler_run_slot(slot);
    }

    return TDMA_OK;
}

uint32_t tdma_scheduler_get_peer_distance(uint8_t peer_id) {
    if (peer_id > TDMA_MAX_PEERS) return 0;
    return g_peer_distance[peer_id];
}

uint32_t tdma_scheduler_get_n_peers(void) {
    return g_tdma.n_peers;
}

uint32_t tdma_scheduler_get_n_ranging_ok(void) {
    return g_tdma.n_ranging_ok;
}

uint32_t tdma_scheduler_get_n_ranging_fail(void) {
    return g_tdma.n_ranging_fail;
}

const tdma_state_t *tdma_scheduler_get_state(void) {
    return &g_tdma;
}
