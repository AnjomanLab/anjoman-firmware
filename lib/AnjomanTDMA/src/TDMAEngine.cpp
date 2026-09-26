#include "TDMAEngine.h"

TDMAEngine::TDMAEngine()
    : _robotId(0),
      _isMaster(false),
      _initialized(false),
      _frameId(0),
      _frameStartUs(0),
      _frameAdvancedFlag(false),
      _broadcastPending(false),
      _lastSyncMs(0),
      _framesSinceLastSync(0),
      _totalFrames(0),
      _totalSyncLosses(0),
      _syncLossStartMs(0),
      _syncLost(false) {}

void TDMAEngine::init(uint8_t robotId, bool isMaster) {
    _robotId = robotId;
    _isMaster = isMaster;
    _frameId = 0;
    _frameStartUs = micros();
    _frameAdvancedFlag = false;
    _broadcastPending = false;
    _lastSyncMs = millis();
    _framesSinceLastSync = 0;
    _totalFrames = 0;
    _totalSyncLosses = 0;
    _syncLossStartMs = 0;
    _syncLost = false;
    _initialized = true;

    if (_isMaster) {
        // Master's first frame boundary is the initialization instant.
        _broadcastPending = true;
    }
}

void TDMAEngine::_beginNewFrame(uint64_t nowUs) {
    _frameStartUs = nowUs;
    _frameId++;
    _frameAdvancedFlag = true;
    _totalFrames++;

    if (_isMaster) {
        _broadcastPending = true;
    }
}

void TDMAEngine::tick(uint32_t nowMs, uint64_t nowUs) {
    if (!_initialized) return;

    // ---- Advance frame if period elapsed ----
    if ((nowUs - _frameStartUs) >= TDMAConfig::FRAME_PERIOD_US) {
        _beginNewFrame(nowUs);
    }

    // ---- Sync watchdog (only for slaves) ----
    if (!_isMaster) {
        if ((nowMs - _lastSyncMs) > TDMAConfig::SYNC_TIMEOUT_MS) {
            // Track consecutive missed syncs
            _framesSinceLastSync++;

            if (!_syncLost &&
                _framesSinceLastSync >= TDMAConfig::SYNC_LOST_THRESHOLD) {
                _syncLost = true;
                _syncLossStartMs = nowMs;
                _totalSyncLosses++;
            }
        }
    }
}

bool TDMAEngine::shouldBroadcastSync() {
    if (!_isMaster || !_initialized) return false;
    if (!_broadcastPending) return false;
    _broadcastPending = false;
    return true;
}

void TDMAEngine::onSyncReceived(uint32_t remoteFrameId,
                                uint32_t remoteTimestampMs,
                                uint64_t nowUs) {
    if (_isMaster) return;   // master ignores incoming syncs

    // ---- Realign local clock to the master's frame ----
    // We trust the master's frameId and reset our local frame start to now.
    _frameId = remoteFrameId;
    _frameStartUs = nowUs;
    _lastSyncMs = remoteTimestampMs;
    _framesSinceLastSync = 0;

    // ---- Clear the "sync lost" condition ----
    if (_syncLost) {
        _syncLost = false;
        _syncLossStartMs = 0;
    }
}

uint32_t TDMAEngine::getFrameId() const {
    return _frameId;
}

uint32_t TDMAEngine::getCurrentSlotIndex() const {
    uint64_t elapsed = micros() - _frameStartUs;
    uint32_t slot = (uint32_t)(elapsed / TDMAConfig::SLOT_PERIOD_US);
    if (slot >= TDMAConfig::N_SLOTS) slot = TDMAConfig::N_SLOTS - 1;
    return slot;
}

uint32_t TDMAEngine::getFrameElapsedUs() const {
    return (uint32_t)(micros() - _frameStartUs);
}

uint32_t TDMAEngine::getSlotElapsedUs() const {
    uint64_t elapsed = micros() - _frameStartUs;
    return (uint32_t)(elapsed % TDMAConfig::SLOT_PERIOD_US);
}

bool TDMAEngine::isSynced() const {
    return !_syncLost;
}

bool TDMAEngine::isSyncLost() const {
    return _syncLost;
}

bool TDMAEngine::isMaster() const {
    return _isMaster;
}

bool TDMAEngine::hasFrameAdvanced() {
    bool flag = _frameAdvancedFlag;
    _frameAdvancedFlag = false;
    return flag;
}

uint32_t TDMAEngine::getFramesSinceLastSync() const {
    return _framesSinceLastSync;
}

uint32_t TDMAEngine::getTotalFrames() const {
    return _totalFrames;
}

uint32_t TDMAEngine::getTotalSyncLosses() const {
    return _totalSyncLosses;
}

uint32_t TDMAEngine::getSyncLossDurationMs() const {
    if (!_syncLost) return 0;
    return millis() - _syncLossStartMs;
}
