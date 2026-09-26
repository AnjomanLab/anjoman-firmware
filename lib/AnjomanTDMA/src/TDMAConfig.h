#pragma once

#include <Arduino.h>

// ==============================================================================
// TDMA Configuration
// ==============================================================================
// Frame layout: 200 ms, divided into equal-length slots.
// Currently 13 slots × 15 ms = 195 ms; last 5 ms reserved for margin.
// Slot allocation (slot index → owner robot + target peer):
//
//   slot 0 : (idle)          — reserved for future sync beacon on-air margin
//   slot 1 : R1 → R2
//   slot 2 : R1 → R3
//   slot 3 : R1 → R4
//   slot 4 : R2 → R1
//   slot 5 : R2 → R3
//   slot 6 : R2 → R4
//   slot 7 : R3 → R1
//   slot 8 : R3 → R2
//   slot 9 : R3 → R4
//   slot 10: R4 → R1
//   slot 11: R4 → R2
//   slot 12: R4 → R3
//
// For the current hub-and-spoke firmware (only R1↔R2, R1↔R3, R1↔R4),
// only slots 1-3 and 4, 7, 10 are active. Others are reserved for
// full-mesh mode (step 7+).
// ==============================================================================

namespace TDMAConfig {

    constexpr uint32_t FRAME_PERIOD_US = 200000;      // 200 ms frame
    constexpr uint32_t SLOT_PERIOD_US  = 15000;       // 15 ms per slot
    constexpr uint8_t  N_SLOTS         = 13;          // 13 × 15 ms = 195 ms

    // ---- Watchdog thresholds ----
    // Master sends a sync beacon every FRAME_PERIOD_US.
    // If a slave sees no sync for SYNC_TIMEOUT_MS, it starts the local fallback.
    constexpr uint32_t SYNC_TIMEOUT_MS = 300;         // 1.5× frame period

    // After this many consecutive missed syncs, raise the `syncLost` flag.
    constexpr uint32_t SYNC_LOST_THRESHOLD = 5;

    // ---- Master behavior ----
    // Master (robot 1) broadcasts sync at each frame boundary.
    // AUTO_START_DELAY_MS is the countdown before the maneuver begins.
    constexpr uint32_t AUTO_START_DELAY_MS = 10000;
}
