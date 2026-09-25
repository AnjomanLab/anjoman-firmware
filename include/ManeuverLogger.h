#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <rom/crc.h>

#include "RobotConfig.h"

// ==============================================================================
// 1. DETERMINISTIC BINARY DATA SPECIFICATION (68 BYTES / RECORD)
// ==============================================================================
#pragma pack(push, 1)
struct LogHeader {
    char     magic[4];       // "ANJM"
    uint16_t version;        // 1
    uint8_t  robotId;        // 1..4
    uint8_t  maneuverId;     // e.g. 1
    uint32_t recordCount;    // Number of valid records
    uint16_t recordSize;     // sizeof(LogRecord) = 68
};

struct LogRecord {
    uint32_t t_ms;           // Timestamp (ms)

    // Motion & Odometry (Core 1)
    float x;                 // Odometry X (m)
    float y;                 // Odometry Y (m)
    float heading;           // Heading theta (rad)
    float v_cmd;             // Command linear velocity (m/s)
    float omega_cmd;         // Command angular velocity (rad/s)
    float rpm_l;             // Measured Left RPM
    float rpm_r;             // Measured Right RPM
    float gyro_z;            // Gyroscope rate (rad/s)

    // Electrical / Power (INA226)
    float vbat;              // Battery Voltage (V)
    float current_a;         // Motor Current (A)

    // UWB Metrology & RF Channel (Core 0)
    float    uwb_raw;        // Raw UWB range (m)
    float    uwb_clean;      // Calibrated UWB range (m)
    float    uwb_rssi;       // RSSI (dBm)
    float    uwb_fp_power;   // First Path Power (dBm)
    uint16_t uwb_std_noise;  // Noise floor (Direct Motor EMI metric)
    uint8_t  uwb_peer_id;    // Target peer (1..4)
    uint8_t  uwb_lde_err;    // Leading Edge error flag
    float    uwb_temp;       // Transceiver temperature (C)
};

struct LogFooter {
    uint32_t crc32;          // CRC32 of all payload records
    char     endMagic[4];    // "MJNA"
};
#pragma pack(pop)

// ==============================================================================
// 2. MODULAR LOGGER & DOWNLOAD CLASS
// ==============================================================================
class ManeuverLogger {
public:
    static constexpr size_t MAX_RECORDS = 300; // 60 seconds @ 5 Hz = 20.4 KB in RAM
    static constexpr const char* LOG_FILE_PATH = "/maneuver_log.bin";

    ManeuverLogger() : _recordCount(0), _server(80), _isServerRunning(false) {}

    bool init() {
        // Auto-format LittleFS on first mount if partition not formatted
        if (!LittleFS.begin(true)) {
            Serial.println("[LOGGER] LittleFS Mount Failed!");
            return false;
        }
        Serial.printf("[LOGGER] LittleFS Mounted (Total: %u KB, Used: %u KB)\n",
                      (unsigned int)(LittleFS.totalBytes() / 1024),
                      (unsigned int)(LittleFS.usedBytes() / 1024));
        _recordCount = 0;
        return true;
    }

    // Ultra-fast O(1) RAM write (takes ~0.1 microseconds)
    inline bool record(const LogRecord &rec) {
        if (_recordCount >= MAX_RECORDS) return false;
        _ramBuffer[_recordCount++] = rec;
        return true;
    }

    size_t getCount() const { return _recordCount; }

    // Commit RAM buffer to Flash with Header, CRC32, and Footer
    bool commitToFlash(uint8_t maneuverId) {
        if (_recordCount == 0) return false;

        File f = LittleFS.open(LOG_FILE_PATH, FILE_WRITE);
        if (!f) {
            Serial.println("[LOGGER] Failed to create log file on Flash!");
            return false;
        }

        // 1. Write Header
        LogHeader header = {
            {'A', 'N', 'J', 'M'},
            1,
            Config::ID,
            maneuverId,
            (uint32_t)_recordCount,
            (uint16_t)sizeof(LogRecord)
        };
        f.write((const uint8_t*)&header, sizeof(LogHeader));

        // 2. Write Data Payload & Calculate Hardware CRC32
        uint32_t payloadBytes = _recordCount * sizeof(LogRecord);
        f.write((const uint8_t*)_ramBuffer, payloadBytes);

        uint32_t crc = crc32_le(0, (const uint8_t*)_ramBuffer, payloadBytes);

        // 3. Write Footer
        LogFooter footer = {
            crc,
            {'M', 'J', 'N', 'A'}
        };
        f.write((const uint8_t*)&footer, sizeof(LogFooter));
        f.close();

        Serial.printf("[LOGGER] Successfully saved %u records (%u bytes) to Flash with CRC32=0x%08X\n",
                      (unsigned int)_recordCount, (unsigned int)(sizeof(LogHeader) + payloadBytes + sizeof(LogFooter)),
                      (unsigned int)crc);
        return true;
    }

    // Launch WiFi and HTTP Server for 'curl' download
    void startDownloadServer() {
        Serial.println("\n[LOGGER] Starting Wi-Fi for log download...");

        WiFi.mode(WIFI_STA);
        WiFi.config(Config::STATIC_IP, Config::GATEWAY, Config::SUBNET);
        WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);

        uint32_t tStart = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - tStart < 8000) {
            delay(200);
            Serial.print(".");
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("\n[LOGGER] Connected to Wi-Fi! IP: %s\n", WiFi.localIP().toString().c_str());
        } else {
            // Fallback to SoftAP if home router is unavailable
            Serial.println("\n[LOGGER] Router not found. Launching SoftAP...");
            WiFi.mode(WIFI_AP);
            char apName[32];
            snprintf(apName, sizeof(apName), "Anjoman_Robot_%u", Config::ID);
            WiFi.softAP(apName, "12345678");
            Serial.printf("[LOGGER] SoftAP IP: %s (SSID: %s)\n", WiFi.softAPIP().toString().c_str(), apName);
        }

        // Endpoint: Download raw binary
        _server.on("/download", HTTP_GET, [this]() {
            if (!LittleFS.exists(LOG_FILE_PATH)) {
                _server.send(404, "text/plain", "Log file not found!");
                return;
            }
            File f = LittleFS.open(LOG_FILE_PATH, FILE_READ);
            _server.sendHeader("Content-Disposition", "attachment; filename=\"robot_log.bin\"");
            _server.streamFile(f, "application/octet-stream");
            f.close();
        });

        // Endpoint: Status
        _server.on("/status", HTTP_GET, [this]() {
            char msg[128];
            snprintf(msg, sizeof(msg), "Robot %u: %u records stored on Flash.\n", Config::ID, (unsigned int)_recordCount);
            _server.send(200, "text/plain", msg);
        });

        _server.begin();
        _isServerRunning = true;
        Serial.println("[LOGGER] HTTP Download Server active! Use 'curl' to retrieve log.");
    }

    void handleClient() {
        if (_isServerRunning) {
            _server.handleClient();
        }
    }

private:
    LogRecord    _ramBuffer[MAX_RECORDS];
    size_t       _recordCount;
    WebServer    _server;
    bool         _isServerRunning;
};
