#pragma once

#include <Arduino.h>
#include <Wire.h>

// ==============================================================================
// INA226 bidirectional current/power monitor
// ==============================================================================
// Connected via TCA9548A on the specified mux channel.
// Configured for continuous shunt + bus voltage measurement with 1.1 ms
// conversion time and 4-sample averaging.
// ==============================================================================

class INA226 {
public:
    INA226(TwoWire &wire, uint8_t address, uint8_t muxChannel,
           float shuntResistorOhm = 0.010f);

    bool begin();
    bool read(float &vBus, float &currentA, float &powerW);

    // Convenience accessors
    float getVoltage() const { return _vbus; }
    float getCurrent() const { return _current; }
    float getPower()   const { return _power; }

    bool isConnected() const { return _connected; }

private:
    static constexpr uint8_t REG_CONFIG       = 0x00;
    static constexpr uint8_t REG_SHUNT_VOLT   = 0x01;
    static constexpr uint8_t REG_BUS_VOLT     = 0x02;

    bool selectMuxChannel();

    TwoWire &_wire;
    uint8_t _addr;
    uint8_t _muxChannel;
    float   _shuntOhm;
    bool    _connected;

    float _vbus;
    float _current;
    float _power;
};
