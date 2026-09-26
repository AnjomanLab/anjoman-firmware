#include "AnjomanI2C.h"

namespace AnjomanI2C {

    static std::mutex        _i2cMutex;
    static bool              _initialized = false;
    static volatile uint32_t _acquireCount = 0;
    static volatile uint32_t _timeoutCount = 0;

    void init(int sdaPin, int sclPin, uint32_t frequencyHz) {
        if (_initialized) return;
        Wire.begin(sdaPin, sclPin, frequencyHz);
        _initialized = true;
    }

    bool lock(uint32_t timeoutMs) {
        // std::mutex::try_lock_for is not available in std::mutex;
        // we use try_lock in a small polling loop with a deadline.
        if (_i2cMutex.try_lock()) {
            _acquireCount++;
            return true;
        }

        const uint32_t t_start = millis();
        while ((millis() - t_start) < timeoutMs) {
            if (_i2cMutex.try_lock()) {
                _acquireCount++;
                return true;
            }
            delay(1);
        }

        _timeoutCount++;
        return false;
    }

    void unlock() {
        _i2cMutex.unlock();
    }

    Guard::Guard(uint32_t timeoutMs) : _locked(false) {
        _locked = AnjomanI2C::lock(timeoutMs);
    }

    Guard::~Guard() {
        if (_locked) {
            AnjomanI2C::unlock();
        }
    }

    bool isInitialized() { return _initialized; }
    uint32_t getAcquireCount() { return _acquireCount; }
    uint32_t getTimeoutCount() { return _timeoutCount; }

} // namespace AnjomanI2C
