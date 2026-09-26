#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <mutex>

// ==============================================================================
// AnjomanI2C — Thread-safe I2C bus access
// ==============================================================================
// Single global mutex protecting ALL I2C transactions on the Wire bus.
// This includes: BMI160 IMU, AS5600 encoders (via TCA9548A), INA226.
//
// Usage:
//   AnjomanI2C::init(PIN_SDA, PIN_SCL, 400000);   // once in setup()
//
//   {
//       AnjomanI2C::Guard g;
//       if (!g.isValid()) return false;
//       Wire.beginTransmission(...);
//       ...
//   }
//
// All modules that touch Wire MUST use this guard.
// ==============================================================================

namespace AnjomanI2C {

    // Initialize bus and mutex. Call once before any I2C transaction.
    void init(int sdaPin, int sclPin, uint32_t frequencyHz);

    // Manual lock/unlock (prefer RAII Guard).
    bool lock(uint32_t timeoutMs = 100);
    void unlock();

    // RAII guard.
    class Guard {
    public:
        explicit Guard(uint32_t timeoutMs = 100);
        ~Guard();
        Guard(const Guard&) = delete;
        Guard& operator=(const Guard&) = delete;
        bool isValid() const { return _locked; }
    private:
        bool _locked;
    };

    bool isInitialized();
    uint32_t getAcquireCount();
    uint32_t getTimeoutCount();
}
