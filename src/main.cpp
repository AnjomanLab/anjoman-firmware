#include <Arduino.h>
#include "PinMap.h"
#include "RobotConfig.h"
#include "ManeuverLogger.h"

ManeuverLogger logger;

void setup() {
    Serial.begin(460800);
    delay(1500);

    Serial.println("\n==================================================");
    Serial.println("STANDALONE LOGGER & FLASH DOWNLOAD TEST SUITE");
    Serial.println("==================================================");

    // 1. Initialize Logger & LittleFS
    if (!logger.init()) {
        Serial.println("[FATAL] Cannot initialize LittleFS! Stopping.");
        while (true) delay(1000);
    }

    // 2. Simulate a 5-Second Maneuver Logging at 5 Hz (25 Records) into RAM
    Serial.println("\n>>> [STEP 1] Simulating 5-Second Maneuver @ 5 Hz into RAM...");
    for (uint32_t i = 0; i < 25; i++) {
        LogRecord rec = {};
        rec.t_ms          = i * 200;
        rec.x             = 1.0f + 0.02f * i;
        rec.y             = -1.0f + 0.01f * i;
        rec.heading       = 0.05f * i;
        rec.v_cmd         = 0.12f;
        rec.omega_cmd     = 0.02f;
        rec.rpm_l         = 42.5f + i * 0.2f;
        rec.rpm_r         = 43.1f + i * 0.1f;
        rec.gyro_z        = 0.02f;
        rec.vbat          = 7.85f - 0.005f * i;
        rec.current_a     = 0.45f + 0.02f * (i % 5);
        rec.uwb_raw       = 2.05f + 0.001f * i;
        rec.uwb_clean     = 2.00f;
        rec.uwb_rssi      = -65.4f;
        rec.uwb_fp_power  = -78.2f;
        rec.uwb_std_noise = 16 + (i % 3); // Emulated low noise floor
        rec.uwb_peer_id   = 2;
        rec.uwb_lde_err   = 0;
        rec.uwb_temp      = 35.4f + 0.1f * i;

        logger.record(rec);
        Serial.printf("Buffered record %2u / 25 in RAM (t = %u ms)\n", i + 1, (unsigned int)rec.t_ms);
        delay(200); // 5 Hz rate
    }

    // 3. Commit RAM to Flash (End of Maneuver)
    Serial.println("\n>>> [STEP 2] Maneuver Ended. Committing RAM Buffer to LittleFS Flash...");
    uint32_t tStart = millis();
    bool saved = logger.commitToFlash(1); // Maneuver ID = 1
    uint32_t tDuration = millis() - tStart;
    Serial.printf("Flash Commit Duration: %lu ms (Result: %s)\n", (unsigned long)tDuration, saved ? "SUCCESS" : "FAILED");

    // 4. Launch Web Server for curl Retrieval
    Serial.println("\n>>> [STEP 3] Launching Web Download Server...");
    logger.startDownloadServer();

    Serial.println("\n==================================================");
    Serial.printf("READY! Download via terminal using:\n");
    Serial.printf("curl http://%s/download -o robot_%u_test.bin\n", 
                  WiFi.localIP().toString().c_str(), Config::ID);
    Serial.println("==================================================");
}

void loop() {
    // Keep HTTP server responsive for downloads
    logger.handleClient();
    delay(2);
}
