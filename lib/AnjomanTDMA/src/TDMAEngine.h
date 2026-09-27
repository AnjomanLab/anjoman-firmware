#pragma once

#include <Arduino.h>
#include "TDMAConfig.h"

class TDMAEngine {
public:
    TDMAEngine();

    void init(uint8_t robotId);
    void tick(uint32_t nowMs, uint64_t nowUs);

    bool shouldBroadcastBeacon();

    void onSyncReceived(uint8_t senderId,
                        uint32_t remoteFrameId,
                        uint32_t remoteUptimeMs,
                        uint8_t senderSlot,
                        uint32_t localMs,
                        uint64_t localUs);

    uint32_t getFrameId() const;
    uint32_t getCurrentSlotIndex() const;
    uint32_t getFrameElapsedUs() const;

    bool isSynced() const;
    bool isSyncLost() const;

    uint32_t getFramesSinceLastSync() const;
    uint32_t getTotalFrames() const;
    uint32_t getTotalSyncLosses() const;

private:
    uint8_t  _robotId;
    bool     _initialized;

    uint32_t _frameId;
    uint64_t _frameStartUs;
    bool     _beaconPending;

    uint32_t _lastSyncLocalMs;
    uint32_t _framesSinceLastSync;
    uint32_t _totalFrames;
    uint32_t _totalSyncLosses;
    bool     _syncLost;

    uint32_t _lastAlignedFrameId;

    void _beginNewFrame(uint64_t nowUs);
};
