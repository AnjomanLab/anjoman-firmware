#include <Arduino.h>
#include <SPI.h>
#include "SPIporting.hpp"
#include "DW1000NgConstants.hpp"
#include "DW1000NgRegisters.hpp"

static SPIClass *_spi;

namespace SPIporting {
    
    namespace {
        constexpr uint32_t EspSPImaximumSpeed = 16000000;
        constexpr uint32_t SPIminimumSpeed    = 2000000;

        const SPISettings _fastSPI = SPISettings(EspSPImaximumSpeed, MSBFIRST, SPI_MODE0);
        const SPISettings _slowSPI = SPISettings(SPIminimumSpeed, MSBFIRST, SPI_MODE0);
        const SPISettings* _currentSPI = &_fastSPI;

        inline void _openSPI(uint8_t slaveSelectPIN) {
            _spi->beginTransaction(*_currentSPI);
            digitalWrite(slaveSelectPIN, LOW);
            delayMicroseconds(1);
        }

        inline void _closeSPI(uint8_t slaveSelectPIN) {
            delayMicroseconds(1);
            digitalWrite(slaveSelectPIN, HIGH);
            _spi->endTransaction();
            delayMicroseconds(1);
        }
    }

    void SPIinit(SPIClass &spi) {
        _spi = &spi;
    }

    void SPIend() {
        _spi->end();
    }

    void SPIselect(uint8_t slaveSelectPIN, uint8_t irq) {
        #if !defined(ESP32) && !defined(ESP8266)
            if(irq != 0xff)
                _spi->usingInterrupt(digitalPinToInterrupt(irq));
        #endif
        pinMode(slaveSelectPIN, OUTPUT);
        digitalWrite(slaveSelectPIN, HIGH);
    }

    void writeToSPI(uint8_t slaveSelectPIN, uint8_t headerLen, byte header[], uint16_t dataLen, byte data[]) {
        _openSPI(slaveSelectPIN);
        for(auto i = 0; i < headerLen; ++i) {
            _spi->transfer(header[i]);
        }
        for(auto i = 0; i < dataLen; ++i) {
            _spi->transfer(data[i]);
        }
        delayMicroseconds(1);
        _closeSPI(slaveSelectPIN);
    }

void readFromSPI(uint8_t slaveSelectPIN, uint8_t headerLen, byte header[], uint16_t dataLen, byte data[]) {
        _openSPI(slaveSelectPIN);
        byte prev = 0;
        for(auto i = 0; i < headerLen; ++i) {
            prev = _spi->transfer(header[i]);
        }
        for(auto i = 0; i < dataLen; ++i) {
            byte curr = _spi->transfer(0x00);
            data[i] = static_cast<byte>((prev << 7) | (curr >> 1));
            prev = curr;
        }
        delayMicroseconds(1);
        _closeSPI(slaveSelectPIN);
    }

    void setSPIspeed(SPIClock speed) {
        if(speed == SPIClock::FAST) {
            _currentSPI = &_fastSPI;
        } else if(speed == SPIClock::SLOW) {
            _currentSPI = &_slowSPI;
        }
    }

}
