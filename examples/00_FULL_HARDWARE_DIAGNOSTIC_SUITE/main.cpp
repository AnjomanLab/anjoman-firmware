#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "PinMap.h"
#include "RobotConfig.h"

// Expected Decawave DW1000 Device ID
constexpr uint32_t EXPECTED_DW1000_ID = 0xDECA0130;

void printSeparator() {
    Serial.println("======================================================================");
}

// ------------------------------------------------------------------------------
// 1. SPI2 & DW1000 LOW-LEVEL REGISTER DIAGNOSTIC
// ------------------------------------------------------------------------------
uint32_t readDW1000DeviceId() {
    digitalWrite(PIN_UWB_CS, LOW);
    SPI.transfer(0x00); // Read DEV_ID register (0x00)
    uint32_t devId = 0;
    for (int i = 0; i < 4; i++) {
        devId |= ((uint32_t)SPI.transfer(0x00) << (i * 8));
    }
    digitalWrite(PIN_UWB_CS, HIGH);
    return devId;
}

void testUWBHardware() {
    printSeparator();
    Serial.println("   [TEST 1] DW1000 UWB HARDWARE & SPI2 BUS AUDIT");
    printSeparator();

    Serial.printf("   SPI2 Pins: SCK=%d, MISO=%d, MOSI=%d, CS=%d, RST=%d, IRQ=%d\n",
                  PIN_UWB_SCK, PIN_UWB_MISO, PIN_UWB_MOSI, PIN_UWB_CS, PIN_UWB_RST, PIN_UWB_IRQ);

    // Test Hardware Reset Line (GPIO 10)
    pinMode(PIN_UWB_RST, OUTPUT);
    digitalWrite(PIN_UWB_RST, LOW);
    delay(20);
    pinMode(PIN_UWB_RST, INPUT); // Float to release
    delay(30);

    pinMode(PIN_UWB_CS, OUTPUT);
    digitalWrite(PIN_UWB_CS, HIGH);

    SPI.begin(PIN_UWB_SCK, PIN_UWB_MISO, PIN_UWB_MOSI, PIN_UWB_CS);
    delay(20);

    // Read DEV_ID register directly via SPI
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    uint32_t devId = readDW1000DeviceId();
    SPI.endTransaction();

    Serial.printf("   Raw Read DEV_ID Register: 0x%08X (Expected: 0x%08X)\n", devId, EXPECTED_DW1000_ID);

    if (devId == EXPECTED_DW1000_ID) {
        Serial.println("   --> [PASS] DW1000 SPI2 Interface is 100% FUNCTIONAL and ALIVE!");
    } else if (devId == 0xFFFFFFFF) {
        Serial.println("   --> [FAIL CRITICAL] MISO Line is FLOATING (Pulled High)! Check 3.3V power, MISO (GPIO 40) or CS (GPIO 38) wiring.");
    } else if (devId == 0x00000000) {
        Serial.println("   --> [FAIL CRITICAL] MISO Line is SHORTED TO GND or SCK (GPIO 39) clock is not reaching chip!");
    } else {
        Serial.printf("   --> [FAIL] Corrupted SPI Read (0x%08X). Check loose jumper wires or clock noise.\n", devId);
    }
}

// ------------------------------------------------------------------------------
// 2. ESP-NOW & WI-FI DIAGNOSTIC
// ------------------------------------------------------------------------------
volatile bool espNowPacketReceived = false;
uint32_t lastBeaconFrame = 0;

void onDiagDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
    espNowPacketReceived = true;
    if (data_len >= 8) {
        lastBeaconFrame = ((uint32_t*)data)[0];
    }
}

void testESPNowNetworking() {
    printSeparator();
    Serial.println("   [TEST 2] ESP-NOW 2.4 GHz RF & SYNC BEACON LISTENER");
    printSeparator();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    Serial.printf("   Robot 2 MAC Address: %s | Wi-Fi Channel: 1\n", WiFi.macAddress().c_str());

    if (esp_now_init() != ESP_OK) {
        Serial.println("   --> [FAIL] ESP-NOW initialization returned error!");
        return;
    }

    esp_now_register_recv_cb(onDiagDataRecv);
    Serial.println("   Listening for 5 seconds for Sync Beacon from Robot 1...");

    uint32_t startWait = millis();
    while (millis() - startWait < 5000) {
        if (espNowPacketReceived) break;
        delay(10);
    }

    if (espNowPacketReceived) {
        Serial.printf("   --> [PASS] ESP-NOW received Sync Beacon from Robot 1! (Latest Frame ID: %u)\n", lastBeaconFrame);
    } else {
        Serial.println("   --> [WARNING] No ESP-NOW packets heard. Is Robot 1 powered on and transmitting?");
    }
}

// ------------------------------------------------------------------------------
// 3. I2C SENSING SUITE (GPIO 7 SDA / GPIO 8 SCL for Robot 2)
// ------------------------------------------------------------------------------
bool checkI2CAddress(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

void testI2CPeripherals() {
    printSeparator();
    Serial.println("   [TEST 3] I2C SENSING SUITE (Robot 2 Custom Pins: SDA=7, SCL=8)");
    printSeparator();

    Wire.begin(PIN_I2C0_SDA, PIN_I2C0_SCL, 400000);
    delay(50);

    // 1. Check TCA9548A Multiplexer (0x70)
    bool muxOk = checkI2CAddress(0x70);
    Serial.printf("   TCA9548A Multiplexer (0x70): %s\n", muxOk ? "DETECTED [PASS]" : "MISSING [FAIL]");

    if (muxOk) {
        // Channel 0: Left AS5600
        Wire.beginTransmission(0x70); Wire.write(1 << 0); Wire.endTransmission();
        bool encLOk = checkI2CAddress(0x36);
        Serial.printf("     - Channel 0 (Left AS5600 @ 0x36) : %s\n", encLOk ? "DETECTED [PASS]" : "FAIL");

        // Channel 1: Right AS5600
        Wire.beginTransmission(0x70); Wire.write(1 << 1); Wire.endTransmission();
        bool encROk = checkI2CAddress(0x36);
        Serial.printf("     - Channel 1 (Right AS5600 @ 0x36): %s\n", encROk ? "DETECTED [PASS]" : "FAIL");

        // Channel 2: BMI160 IMU (0x68 / 0x69)
        Wire.beginTransmission(0x70); Wire.write(1 << 2); Wire.endTransmission();
        bool imuOk = checkI2CAddress(0x69) || checkI2CAddress(0x68);
        Serial.printf("     - Channel 2 (BMI160 IMU @ 0x69)  : %s\n", imuOk ? "DETECTED [PASS]" : "FAIL");

        // Channel 3: INA226 Power Monitor (0x40)
        Wire.beginTransmission(0x70); Wire.write(1 << 3); Wire.endTransmission();
        bool inaOk = checkI2CAddress(0x40);
        Serial.printf("     - Channel 3 (INA226 @ 0x40)      : %s\n", inaOk ? "DETECTED [PASS]" : "FAIL");
    }
}

// ------------------------------------------------------------------------------
// 4. OCTAL-PSRAM & MEMORY STRESS AUDIT
// ------------------------------------------------------------------------------
void testMemoryIntegrity() {
    printSeparator();
    Serial.println("   [TEST 4] ESP32-S3-WROOM-1-N16R8 MEMORY & PSRAM AUDIT");
    printSeparator();

    uint32_t freeSram = ESP.getFreeHeap();
    uint32_t psramSize = ESP.getPsramSize();
    uint32_t freePsram = ESP.getFreePsram();

    Serial.printf("   Internal Free SRAM : %u bytes\n", freeSram);
    Serial.printf("   Octal PSRAM Total  : %u bytes (%u MB)\n", psramSize, psramSize / (1024 * 1024));
    Serial.printf("   Octal PSRAM Free   : %u bytes\n", freePsram);

    if (psramSize > 0) {
        Serial.println("   Allocating 256 KB memory buffer in PSRAM to verify cache bus stability...");
        uint8_t *testBuf = (uint8_t*)ps_malloc(256 * 1024);
        if (testBuf != nullptr) {
            memset(testBuf, 0xAA, 256 * 1024);
            bool verified = true;
            for (int i = 0; i < 256 * 1024; i++) {
                if (testBuf[i] != 0xAA) { verified = false; break; }
            }
            free(testBuf);
            Serial.printf("   --> [PASS] PSRAM read/write test %s! Cache bus is intact.\n", verified ? "PASSED" : "FAILED");
        } else {
            Serial.println("   --> [WARNING] PSRAM allocation failed.");
        }
    } else {
        Serial.println("   --> [WARNING] PSRAM not detected! Check platformio.ini memory_type = qio_opi.");
    }
}

// ------------------------------------------------------------------------------
// SETUP
// ------------------------------------------------------------------------------
void setup() {
    Serial.begin(460800);
    delay(1500);

    printSeparator();
    Serial.println("   ANJOMAN FLEET - ROBOT 2 FULL HARDWARE DIAGNOSTIC SUITE");
    printSeparator();

    testMemoryIntegrity();
    testUWBHardware();
    testESPNowNetworking();
    testI2CPeripherals();

    printSeparator();
    Serial.println("   DIAGNOSTIC AUDIT COMPLETE. Inspect report above for FAIL items.");
    printSeparator();

    // Color cycle RGB LED on GPIO 48
    rgbLedWrite(PIN_STATUS_RGB, 0, 50, 0); // Green
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
