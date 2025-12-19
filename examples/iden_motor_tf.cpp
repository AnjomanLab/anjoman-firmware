#include <Arduino.h>
#include <Wire.h>
#include "MotorDriver.h"
#include "MagneticEncoder.h"

// --- Hardware Pins ---
#define PIN_AIN1 2
#define PIN_AIN2 3
#define PIN_STBY 4

#define PIN_SDA 0
#define PIN_SCL 1

// --- Settings ---
const int TEST_PWM = 110;
// I chose a test duration of 5 seconds to make sure it reached its final speed, although even 2 seconds would be enough.
const int TEST_DURATION_MS = 5000; 
const int SAMPLE_INTERVAL_MS = 20; // 50Hz

// --- Objects ---
MotorDriver motor(PIN_AIN1, PIN_AIN2, PIN_STBY);
MagneticEncoder encoder;

// --- Helper: Run Step Test ---
// pwm: Input speed
// dir_name: Direction name for printing on output
void run_step_response(int pwm, const char* dir_name) {
    Serial.println(""); 
    Serial.print("--- START TEST: "); Serial.println(dir_name);
    Serial.println("Time_ms,Input_PWM,Speed_RPM");

    // Status variables
    unsigned long startTime = millis();
    unsigned long lastSampleTime = 0;
    
    // Initial state of the encoder
    float lastRevs = encoder.getRevolutions();
    float currentRevs = 0;
    unsigned long sampleT0 = micros(); // For high accuracy of speed calculation

    // 1. Apply Step (Step Input)
    motor.drive(pwm);
    while (millis() - startTime < TEST_DURATION_MS) {
        // Continuous encoder reading
        encoder.update();
        unsigned long now = millis();
        
        // Sampling at specified time intervals (without using delay)
        if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
            
            // a) Read the current state
            currentRevs = encoder.getRevolutions();
            unsigned long sampleT1 = micros();

            // b) Calculate the velocity (discrete derivative)
            // dAngle / dt
            float dt_seconds = (sampleT1 - sampleT0) / 1000000.0;
            float dRevs = currentRevs - lastRevs;
            float rpm = (dRevs / dt_seconds) * 60.0;

            // c) Print data
            // Time: Time elapsed since the start of the test (Time constant axis)
            Serial.print(now - startTime); 
            Serial.print(",");
            Serial.print(pwm);
            Serial.print(",");
            Serial.println(rpm, 2);
            
            // d) Update for next round
            lastSampleTime = now;
            lastRevs = currentRevs;
            sampleT0 = sampleT1;
        }
    }

    // End of test
    motor.drive(0);
    motor.brake(); 
    Serial.println("--- END TEST ---");
    Serial.println(""); 
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    // 1. Hardware Init
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();
    Wire.setClock(400000); 

    motor.begin();
    motor.enable();
    encoder.begin();

    // 2. Health Check
    if (!encoder.isMagnetDetected()) {
        Serial.println("CRITICAL: Magnet Missing!");
        while(1);
    }

    Serial.println("=== SYSTEM IDENTIFICATION MODE ===");
    Serial.println("Protocol: Forward Step -> Stop -> Reverse Step -> Stop");
    Serial.println("Wait 3 seconds...");
    delay(3000);

    // --- TEST 1: Forward ---
    run_step_response(TEST_PWM, "FORWARD");

    // Cool down / Settling time
    Serial.println("Waiting for full stop...");
    delay(2000); 

    // --- TEST 2: Reverse ---
    run_step_response(-TEST_PWM, "REVERSE");
    Serial.println("=== IDENTIFICATION DONE ===");
}

void loop() {
    // Nothing. Restart board to run again.
}
