#include "ManeuverLogger.h"
#include "RobotConfig.h"
#include <WiFi.h>
#include <rom/crc.h>

ManeuverLogger::ManeuverLogger()
    : _recordCount(0), _server(80), _isServerRunning(false) {}

bool ManeuverLogger::init() {
    if (!LittleFS.begin(true)) {
        return false;
    }
    _recordCount = 0;
    return true;
}

bool ManeuverLogger::record(const LogRecord &rec) {
    if (_recordCount >= MAX_RECORDS) {
        return false;
    }
    _ramBuffer[_recordCount++] = rec;
    return true;
}

bool ManeuverLogger::commitToFlash(uint8_t maneuverId) {
    if (_recordCount == 0) {
        return false;
    }

    File f = LittleFS.open(LOG_FILE_PATH, FILE_WRITE);
    if (!f) {
        return false;
    }

    // 1. Header
    LogHeader header = {};
    memcpy(header.magic, "ANJM", 4);
    header.version     = 1;
    header.robotId     = Config::ID;
    header.maneuverId  = maneuverId;
    header.recordCount = (uint32_t)_recordCount;
    header.recordSize  = (uint16_t)sizeof(LogRecord);
    f.write((const uint8_t*)&header, sizeof(LogHeader));

    // 2. Payload
    uint32_t payloadBytes = (uint32_t)_recordCount * sizeof(LogRecord);
    f.write((const uint8_t*)_ramBuffer, payloadBytes);

    // 3. Footer with CRC32
    uint32_t crc = crc32_le(0, (const uint8_t*)_ramBuffer, payloadBytes);
    LogFooter footer = {};
    footer.crc32 = crc;
    memcpy(footer.endMagic, "MJNA", 4);
    f.write((const uint8_t*)&footer, sizeof(LogFooter));

    f.close();
    return true;
}

void ManeuverLogger::_configureDownloadRoute() {
    _server.on("/download", HTTP_GET, [this]() {
        if (!LittleFS.exists(LOG_FILE_PATH)) {
            _server.send(404, "text/plain", "Log not found");
            return;
        }
        File f = LittleFS.open(LOG_FILE_PATH, FILE_READ);
        _server.sendHeader(
            "Content-Disposition",
            "attachment; filename=\"maneuver_log.bin\"");
        _server.streamFile(f, "application/octet-stream");
        f.close();
    });

    _server.on("/status", HTTP_GET, [this]() {
        String body = "{\"robot\":";
        body += String(Config::ID);
        body += ",\"records\":";
        body += String(_recordCount);
        body += ",\"logSize\":";
        body += String((uint32_t)_recordCount * sizeof(LogRecord));
        body += "}";
        _server.send(200, "application/json", body);
    });
}

void ManeuverLogger::startDownloadServer() {
    // Try station mode first
    WiFi.mode(WIFI_STA);
    WiFi.config(Config::STATIC_IP, Config::GATEWAY, Config::SUBNET);
    WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);

    uint32_t tStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - tStart < 6000) {
        delay(200);
    }

    // Fallback: AP mode
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.mode(WIFI_AP);
        char apName[32];
        snprintf(apName, sizeof(apName), "Anjoman_Robot_%u", Config::ID);
        WiFi.softAP(apName, "12345678");
    }

    _configureDownloadRoute();
    _server.begin();
    _isServerRunning = true;
}

void ManeuverLogger::handleClient() {
    if (_isServerRunning) {
        _server.handleClient();
    }
}
