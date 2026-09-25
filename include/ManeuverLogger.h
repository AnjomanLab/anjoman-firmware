#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <rom/crc.h>

#include "RobotConfig.h"

#pragma pack(push, 1)
struct LogHeader {
    char     magic[4];       // "ANJM"
    uint16_t version;        // 1
    uint8_t  robotId;        // Config::ID
    uint8_t  maneuverId;     // 1
    uint32_t recordCount;    // N
    uint16_t recordSize;     // 68 bytes
};

struct LogRecord {
    uint32_t t_ms;           // Timestamp (ms)
    float x;                 // Odometry X (m)
    float y;                 // Odometry Y (m)
    float heading;           // Heading theta (rad)
    float v_cmd;             // Command linear velocity (m/s)
    float omega_cmd;         // Command angular velocity (rad/s)
    float rpm_l;             // Measured Left RPM
    float rpm_r;             // Measured Right RPM
    float gyro_z;            // Gyroscope rate (rad/s)
    float vbat;              // Battery Voltage (V)
    float current_a;         // Motor Current (A)
    float    uwb_raw;        // Raw UWB range (m)
    float    uwb_clean;      // Calibrated UWB range (m)
    float    uwb_rssi;       // RSSI (dBm)
    float    uwb_fp_power;   // First Path Power (dBm)
    uint16_t uwb_std_noise;  // Noise floor (Motor EMI metric)
    uint8_t  uwb_peer_id;    // Target peer (1..4)
    uint8_t  uwb_lde_err;    // Leading Edge error flag
    float    uwb_temp;       // Transceiver temperature (C)
};

struct LogFooter {
    uint32_t crc32;          // CRC32 of payload
    char     endMagic[4];    // "MJNA"
};
#pragma pack(pop)

class ManeuverLogger {
public:
    static constexpr size_t MAX_RECORDS = 300; // 60s @ 5 Hz = 20.4 KB in RAM
    static constexpr const char* LOG_FILE_PATH = "/maneuver_log.bin";

    ManeuverLogger() : _recordCount(0), _server(80), _isServerRunning(false) {}

    bool init() {
        if (!LittleFS.begin(true)) {
            return false;
        }
        _recordCount = 0;
        return true;
    }

    inline bool record(const LogRecord &rec) {
        if (_recordCount >= MAX_RECORDS) return false;
        _ramBuffer[_recordCount++] = rec;
        return true;
    }

    size_t getCount() const { return _recordCount; }

    bool commitToFlash(uint8_t maneuverId) {
        if (_recordCount == 0) return false;

        File f = LittleFS.open(LOG_FILE_PATH, FILE_WRITE);
        if (!f) return false;

        LogHeader header = {
            {'A', 'N', 'J', 'M'},
            1,
            Config::ID,
            maneuverId,
            (uint32_t)_recordCount,
            (uint16_t)sizeof(LogRecord)
        };
        f.write((const uint8_t*)&header, sizeof(LogHeader));

        uint32_t payloadBytes = _recordCount * sizeof(LogRecord);
        f.write((const uint8_t*)_ramBuffer, payloadBytes);

        uint32_t crc = crc32_le(0, (const uint8_t*)_ramBuffer, payloadBytes);

        LogFooter footer = { crc, {'M', 'J', 'N', 'A'} };
        f.write((const uint8_t*)&footer, sizeof(LogFooter));
        f.close();
        return true;
    }

    void startDownloadServer() {
        WiFi.mode(WIFI_STA);
        WiFi.config(Config::STATIC_IP, Config::GATEWAY, Config::SUBNET);
        WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);

        uint32_t tStart = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - tStart < 6000) {
            delay(200);
        }

        if (WiFi.status() != WL_CONNECTED) {
            WiFi.mode(WIFI_AP);
            char apName[32];
            snprintf(apName, sizeof(apName), "Anjoman_Robot_%u", Config::ID);
            WiFi.softAP(apName, "12345678");
        }

        _server.on("/download", HTTP_GET, [this]() {
            if (!LittleFS.exists(LOG_FILE_PATH)) {
                _server.send(404, "text/plain", "Log not found");
                return;
            }
            File f = LittleFS.open(LOG_FILE_PATH, FILE_READ);
            _server.sendHeader("Content-Disposition", "attachment; filename=\"maneuver_log.bin\"");
            _server.streamFile(f, "application/octet-stream");
            f.close();
        });

        _server.begin();
        _isServerRunning = true;
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
