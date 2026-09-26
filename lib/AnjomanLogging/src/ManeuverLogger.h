#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <WebServer.h>
#include "LogSchema.h"

class ManeuverLogger {
public:
    // RAM buffer capacity — 300 records @ 68 bytes = 20.4 KB
    static constexpr size_t MAX_RECORDS       = 300;
    static constexpr const char* LOG_FILE_PATH = "/maneuver_log.bin";

    ManeuverLogger();

    // ---- Lifecycle ----
    bool init();

    // ---- Recording (called from control loop) ----
    bool record(const LogRecord &rec);
    size_t getCount() const { return _recordCount; }
    bool isFull() const { return _recordCount >= MAX_RECORDS; }

    // ---- Persistence ----
    bool commitToFlash(uint8_t maneuverId);

    // ---- Download server ----
    void startDownloadServer();
    void handleClient();
    bool isServerRunning() const { return _isServerRunning; }

    // ---- Diagnostics ----
    size_t getRamUsageBytes() const { return _recordCount * sizeof(LogRecord); }

private:
    LogRecord    _ramBuffer[MAX_RECORDS];
    size_t       _recordCount;
    WebServer    _server;
    bool         _isServerRunning;

    void _configureDownloadRoute();
};
