#pragma once

#include <Arduino.h>

// ==============================================================================
// TDMA Configuration — 3-robot fleet (R2, R3, R4)
// ==============================================================================
// Frame: 240 ms, 16 slots of 15 ms each.
//
// Slot allocation (each robot owns 4 consecutive slots):
//   slot 0  : R2 beacon
//   slot 1  : R2 → R3
//   slot 2  : R2 → R4
//   slot 3  : (idle / margin)
//   slot 4  : R3 beacon
//   slot 5  : R3 → R2
//   slot 6  : R3 → R4
//   slot 7  : (idle / margin)
//   slot 8  : R4 beacon
//   slot 9  : R4 → R2
//   slot 10 : R4 → R3
//   slot 11 : (idle / margin)
//   slot 12 : R2 listen window for R3/R4 polls
//   slot 13 : R3 listen window
//   slot 14 : R4 listen window
//   slot 15 : (idle / margin)
//
// The four margin slots (3, 7, 11, 15) absorb clock drift and jitter,
// preventing slot-boundary collisions between consecutive transmitters.
// ==============================================================================

namespace TDMAConfig {

    constexpr uint32_t FRAME_PERIOD_US = 240000;   // 240 ms
    constexpr uint32_t SLOT_PERIOD_US  = 15000;    // 15 ms
    constexpr uint8_t  N_SLOTS         = 16;

    constexpr uint32_t SYNC_TIMEOUT_MS     = 500;
    constexpr uint32_t SYNC_LOST_THRESHOLD = 3;

    constexpr uint32_t MANEUVER_START_FRAME = 42;
}
