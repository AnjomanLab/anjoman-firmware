#include "TDMAEngine.h"

TDMAEngine::TDMAEngine()
    : _robotId(0), _initialized(false),
      _frameId(0), _frameStartUs(0),
      _beaconPending(false),
      _lastSyncLocalMs(0), _framesSinceLastSync(0),
      _totalFrames(0), _totalSyncLosses(0), _syncLost(false),
      _lastAlignedFrameId(UINT32_MAX) {}

void TDMAEngine::init(uint8_t robotId) {
    _robotId = robotId;
    _frameId = 0;
    _frameStartUs = micros();
    _beaconPending = true;
    _lastSyncLocalMs = millis();
    _framesSinceLastSync = 0;
    _totalFrames = 0;
    _totalSyncLosses = 0;
    _syncLost = false;
    _lastAlignedFrameId = UINT32_MAX;
    _initialized = true;
}

void TDMAEngine::_beginNewFrame(uint64_t nowUs) {
    _frameStartUs = nowUs;
    _frameId++;
    _totalFrames++;
    _beaconPending = true;
}

void TDMAEngine::tick(uint32_t nowMs, uint64_t nowUs) {
    if (!_initialized) return;

    if ((nowUs - _frameStartUs) >= TDMAConfig::FRAME_PERIOD_US) {
        _beginNewFrame(nowUs);
    }

    if ((nowMs - _lastSyncLocalMs) > TDMAConfig::SYNC_TIMEOUT_MS) {
        _framesSinceLastSync++;
        if (!_syncLost &&
            _framesSinceLastSync >= TDMAConfig::SYNC_LOST_THRESHOLD) {
            _syncLost = true;
            _totalSyncLosses++;
        }
    }
}

bool TDMAEngine::shouldBroadcastBeacon() {
    if (!_initialized) return false;
    if (!_beaconPending) return false;
    _beaconPending = false;
    return true;
}

void TDMAEngine::onSyncReceived(uint8_t senderId,
                                uint32_t remoteFrameId,
                                uint32_t remoteUptimeMs,
                                uint8_t senderSlot,
                                uint32_t localMs,
                                uint64_t localUs) {
    (void)senderId;
    (void)remoteUptimeMs;

    // Watchdog: any received beacon clears the sync-lost condition
    _lastSyncLocalMs = localMs;
    _framesSinceLastSync = 0;
    if (_syncLost) _syncLost = false;

    // Frame counter: only jump forward
    if (remoteFrameId > _frameId) {
        _frameId = remoteFrameId;
    }

    // Phase alignment: once per frame, on the first beacon with a new frameId.
    // Because beacons are sent in ascending slot order (R1 at slot 0,
    // R2 at slot 4, R3 at slot 8, R4 at slot 12), the first beacon we
    // receive in a frame is R1's, and it carries senderSlot=0.
    if (remoteFrameId != _lastAlignedFrameId) {
        _lastAlignedFrameId = remoteFrameId;
        const uint64_t slotOffsetUs =
            (uint64_t)senderSlot * TDMAConfig::SLOT_PERIOD_US;
        if (localUs >= slotOffsetUs) {
            _frameStartUs = localUs - slotOffsetUs;
        } else {
            _frameStartUs = localUs;
        }
    }
}

uint32_t TDMAEngine::getFrameId() const { return _frameId; }

uint32_t TDMAEngine::getCurrentSlotIndex() const {
    uint64_t elapsed = micros() - _frameStartUs;
    uint32_t slot = (uint32_t)(elapsed / TDMAConfig::SLOT_PERIOD_US);
    if (slot >= TDMAConfig::N_SLOTS) slot = TDMAConfig::N_SLOTS - 1;
    return slot;
}

uint32_t TDMAEngine::getFrameElapsedUs() const {
    return (uint32_t)(micros() - _frameStartUs);
}

bool TDMAEngine::isSynced()   const { return !_syncLost; }
bool TDMAEngine::isSyncLost() const { return _syncLost; }

uint32_t TDMAEngine::getFramesSinceLastSync() const { return _framesSinceLastSync; }
uint32_t TDMAEngine::getTotalFrames()         const { return _totalFrames; }
uint32_t TDMAEngine::getTotalSyncLosses()     const { return _totalSyncLosses; }
