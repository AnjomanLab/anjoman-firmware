#include "INA226.h"
#include "AnjomanI2C.h"

INA226::INA226(TwoWire &wire, uint8_t address, uint8_t muxChannel,
               float shuntResistorOhm)
    : _wire(wire),
      _addr(address),
      _muxChannel(muxChannel),
      _shuntOhm(shuntResistorOhm),
      _connected(false),
      _vbus(0.0f), _current(0.0f), _power(0.0f) {}

bool INA226::selectMuxChannel() {
    if (_muxChannel > 7) return false;
    _wire.beginTransmission(0x70);
    _wire.write(1 << _muxChannel);
    return (_wire.endTransmission() == 0);
}

bool INA226::begin() {
    AnjomanI2C::Guard g;
    if (!g.isValid()) return false;

    if (!selectMuxChannel()) return false;

    // Check presence
    _wire.beginTransmission(_addr);
    if (_wire.endTransmission() != 0) {
        _connected = false;
        return false;
    }

    // Software reset
    _wire.beginTransmission(_addr);
    _wire.write(REG_CONFIG);
    _wire.write(0x80); _wire.write(0x00);
    _wire.endTransmission();
    delay(10);

    // Continuous shunt + bus, 1.1 ms, 4 averages → 0x4227
    _wire.beginTransmission(_addr);
    _wire.write(REG_CONFIG);
    _wire.write(0x42); _wire.write(0x27);
    bool ok = (_wire.endTransmission() == 0);

    _connected = ok;
    return ok;
}

bool INA226::read(float &vBus, float &currentA, float &powerW) {
    AnjomanI2C::Guard g;
    if (!g.isValid() || !_connected) {
        vBus = 0.0f; currentA = 0.0f; powerW = 0.0f;
        return false;
    }

    if (!selectMuxChannel()) {
        vBus = 0.0f; currentA = 0.0f; powerW = 0.0f;
        return false;
    }

    // ---- Bus voltage ----
    _wire.beginTransmission(_addr);
    _wire.write(REG_BUS_VOLT);
    if (_wire.endTransmission(false) != 0 ||
        _wire.requestFrom(_addr, (uint8_t)2) < 2) {
        return false;
    }
    uint16_t rawBus = (uint16_t)((_wire.read() << 8) | _wire.read());
    _vbus = (float)rawBus * 0.00125f;

    // ---- Shunt voltage ----
    _wire.beginTransmission(_addr);
    _wire.write(REG_SHUNT_VOLT);
    if (_wire.endTransmission(false) != 0 ||
        _wire.requestFrom(_addr, (uint8_t)2) < 2) {
        return false;
    }
    int16_t rawShunt = (int16_t)((_wire.read() << 8) | _wire.read());
    float vShuntV = (float)rawShunt * 2.5e-6f;

    _current = vShuntV / _shuntOhm;
    _power   = _vbus * _current;

    vBus     = _vbus;
    currentA = _current;
    powerW   = _power;
    return true;
}
