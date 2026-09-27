#pragma once

#include <Arduino.h>

namespace TDMAConfig {

    constexpr uint32_t FRAME_PERIOD_US = 240000;    // 240 ms
    constexpr uint32_t SLOT_PERIOD_US  = 15000;     // 15 ms
    constexpr uint8_t  N_SLOTS         = 16;

    constexpr uint32_t SYNC_TIMEOUT_MS     = 500;
    constexpr uint32_t SYNC_LOST_THRESHOLD = 3;

    constexpr uint32_t MANEUVER_START_FRAME = 42;   // ~10.1 s after boot/sync
}
