#pragma once

#include <Arduino.h>
#include "TDMAConfig.h"

// ==============================================================================
// TDMAEngine — frame counter + slot timing + sync watchdog
// ==============================================================================
// Responsibilities:
//   1. Maintain a frame counter (`frameId`) and per-frame start timestamp.
//   2. In master mode: signal when a new sync beacon must be broadcast.
//   3. In slave mode: accept sync beacons and re-align the local clock.
//   4. In both modes: if sync is lost, continue counting locally so the
//      rest of the system stays functional (degraded but not stuck).
//
// Usage (master):
//     engine.init(robotId, /*isMaster=*/true);
//     loop() {
//         engine.tick(millis(), micros());
//         if (engine.shouldBroadcastSync()) {
//             SyncBeaconPacket pkt = ...;
//             esp_now_send(..., &pkt, ...);
//         }
//         uint32_t slot = engine.getCurrentSlotIndex();
//         ...
//     }
//
// Usage (slave):
//     engine.init(robotId, /*isMaster=*/false);
//     onDataRecv(...) {
//         if (isSyncBeacon) {
//             engine.onSyncReceived(pkt.frameId, pkt.timestampMs, micros());
//         }
//     }
//     loop() {
//         engine.tick(millis(), micros());
//         uint32_t slot = engine.getCurrentSlotIndex();
//         ...
//     }
// ==============================================================================

class TDMAEngine {
public:
    TDMAEngine();

    // ---- Initialization ----
    void init(uint8_t robotId, bool isMaster);

    // ---- Called from loop() at high rate ----
    // @param nowMs   Current millis()
    // @param nowUs   Current micros()
    void tick(uint32_t nowMs, uint64_t nowUs);

    // ---- Master side ----
    // Returns true ONCE per frame, immediately after the frame boundary.
    // The caller should then broadcast a SyncBeaconPacket.
    bool shouldBroadcastSync();

    // ---- Slave side ----
    // Called when a sync beacon arrives.
    void onSyncReceived(uint32_t remoteFrameId,
                        uint32_t remoteTimestampMs,
                        uint64_t nowUs);

    // ---- Query API (safe to call from any context) ----
    uint32_t getFrameId() const;
    uint32_t getCurrentSlotIndex() const;
    uint32_t getFrameElapsedUs() const;
    uint32_t getSlotElapsedUs() const;

    bool isSynced() const;             // true → receiving sync normally
    bool isSyncLost() const;           // true → fallback mode active
    bool isMaster() const;
    bool hasFrameAdvanced();           // consumer flag

    // ---- Diagnostics ----
    uint32_t getFramesSinceLastSync() const;
    uint32_t getTotalFrames() const;
    uint32_t getTotalSyncLosses() const;
    uint32_t getSyncLossDurationMs() const;

private:
    uint8_t  _robotId;
    bool     _isMaster;
    bool     _initialized;

    // ---- Frame state ----
    uint32_t _frameId;
    uint64_t _frameStartUs;
    bool     _frameAdvancedFlag;
    bool     _broadcastPending;

    // ---- Sync watchdog ----
    uint32_t _lastSyncMs;              // millis() at last sync beacon
    uint32_t _framesSinceLastSync;
    uint32_t _totalFrames;
    uint32_t _totalSyncLosses;
    uint32_t _syncLossStartMs;
    bool     _syncLost;

    // internal helper
    void _beginNewFrame(uint64_t nowUs);
};
