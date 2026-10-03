#include <Arduino.h>
#include <SPI.h>
#include "PinMap.h"

static SPIClass uwbSPI(FSPI);

void readAligned(uint8_t reg, uint16_t sub, uint8_t *outBuf, uint16_t len) {
    uint8_t headerLen = 0;
    uint8_t header[3];

    if (sub == 0xFFFF) {
        header[0] = 0x00 | (reg & 0x3F);
        headerLen = 1;
    } else if (sub < 128) {
        header[0] = 0x40 | (reg & 0x3F);
        header[1] = static_cast<uint8_t>(sub & 0x7F);
        headerLen = 2;
    } else {
        header[0] = 0x40 | (reg & 0x3F);
        header[1] = 0x80 | static_cast<uint8_t>(sub & 0x7F);
        header[2] = static_cast<uint8_t>((sub >> 7) & 0xFF);
        headerLen = 3;
    }

    uint16_t rawLen = headerLen + len + 1;
    uint8_t rawBuf[64];
    if (rawLen > sizeof(rawBuf)) return;

    digitalWrite(PIN_UWB_CS, LOW);

    // Send header and store incoming full-duplex bytes
    for (uint8_t i = 0; i < headerLen; ++i) {
        rawBuf[i] = uwbSPI.transfer(header[i]);
    }
    // Read payload bytes plus one trailing byte for bit alignment
    for (uint16_t i = 0; i <= len; ++i) {
        rawBuf[headerLen + i] = uwbSPI.transfer(0x00);
    }

    digitalWrite(PIN_UWB_CS, HIGH);

    // Reconstruct bits across byte boundaries
    for (uint16_t i = 0; i < len; ++i) {
        uint8_t prevByte = rawBuf[headerLen + i - 1];
        uint8_t currByte = rawBuf[headerLen + i];
        outBuf[i] = static_cast<uint8_t>((prevByte << 7) | (currByte >> 1));
    }
}

uint8_t readAligned8(uint8_t reg, uint16_t sub) {
    uint8_t val = 0;
    readAligned(reg, sub, &val, 1);
    return val;
}

void writeReg8(uint8_t reg, uint16_t sub, uint8_t val) {
    digitalWrite(PIN_UWB_CS, LOW);
    if (sub == 0xFFFF) {
        uwbSPI.transfer(0x80 | (reg & 0x3F));
    } else if (sub < 128) {
        uwbSPI.transfer(0xC0 | (reg & 0x3F));
        uwbSPI.transfer(static_cast<uint8_t>(sub & 0x7F));
    } else {
        uwbSPI.transfer(0xC0 | (reg & 0x3F));
        uwbSPI.transfer(0x80 | static_cast<uint8_t>(sub & 0x7F));
        uwbSPI.transfer(static_cast<uint8_t>((sub >> 7) & 0xFF));
    }
    uwbSPI.transfer(val);
    digitalWrite(PIN_UWB_CS, HIGH);
}

uint8_t readOtpByte(uint16_t address) {
    writeReg8(0x2D, 0x04, address & 0xFF);
    writeReg8(0x2D, 0x05, (address >> 8) & 0xFF);
    writeReg8(0x2D, 0x06, 0x03);
    writeReg8(0x2D, 0x06, 0x01);
    delayMicroseconds(20);
    uint8_t val = readAligned8(0x2D, 0x0A);
    writeReg8(0x2D, 0x06, 0x00);
    return val;
}

uint32_t readDevId() {
    uint8_t buf[4] = {0};
    readAligned(0x00, 0xFFFF, buf, 4);
    return (static_cast<uint32_t>(buf[3]) << 24) |
           (static_cast<uint32_t>(buf[2]) << 16) |
           (static_cast<uint32_t>(buf[1]) << 8)  |
            static_cast<uint32_t>(buf[0]);
}

void setup() {
    Serial.begin(460800);
    delay(2000);

    pinMode(PIN_UWB_CS, OUTPUT);
    digitalWrite(PIN_UWB_CS, HIGH);

    if (PIN_UWB_RST != 0xFF) {
        pinMode(PIN_UWB_RST, OUTPUT);
        digitalWrite(PIN_UWB_RST, LOW);
        delay(5);
        pinMode(PIN_UWB_RST, INPUT);
        delay(25);
    }

    uwbSPI.begin(PIN_UWB_SCK, PIN_UWB_MISO, PIN_UWB_MOSI, -1);
    uwbSPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

    Serial.println("\n=== DW1000 BIT-ALIGNED REGISTER DIAGNOSTIC ===");

    uint32_t id = readDevId();
    Serial.printf("DEV_ID: 0x%08X ", id);
    if (id == 0xDECA0130) {
        Serial.println("[EXACT MATCH! 0xDECA0130]");
    } else {
        Serial.printf("[STILL 0x%08X]\n", id);
    }

    uint8_t vbatRef = readOtpByte(0x008);
    uint8_t tempRef = readOtpByte(0x009);
    Serial.printf("OTP -> VbatRef: 0x%02X (%u), TempRef: 0x%02X (%u)\n", 
                  vbatRef, vbatRef, tempRef, tempRef);

    // SAR A/D conversion sequence
    writeReg8(0x28, 0x11, 0x80);
    writeReg8(0x28, 0x12, 0x0A);
    writeReg8(0x28, 0x12, 0x0F);
    delayMicroseconds(50);

    writeReg8(0x2A, 0x00, 0x01);
    delayMicroseconds(50);
    writeReg8(0x2A, 0x00, 0x00);
    delayMicroseconds(50);

    uint8_t rawVbat = readAligned8(0x2A, 0x03);
    uint8_t rawTemp = readAligned8(0x2A, 0x04);
    Serial.printf("SAR -> LVBAT: 0x%02X (%u), LTEMP: 0x%02X (%u)\n", 
                  rawVbat, rawVbat, rawTemp, rawTemp);

    if (tempRef > 0 && rawTemp > 0) {
        float tempC = (static_cast<float>(rawTemp) - static_cast<float>(tempRef)) * 1.14f + 23.0f;
        float vbatV = (static_cast<float>(rawVbat) - static_cast<float>(vbatRef)) / 173.0f + 3.3f;
        Serial.printf("Physical -> Temp: %.2f C, Vbat: %.2f V\n", tempC, vbatV);
    }
}

void loop() {
    delay(2000);
}
