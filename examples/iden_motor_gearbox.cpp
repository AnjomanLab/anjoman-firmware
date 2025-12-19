#include <Arduino.h>
#include <Wire.h>
#include "MotorDriver.h"
#include "MagneticEncoder.h"

// Hardware Configuration
#define PIN_AIN1 2
#define PIN_AIN2 3
#define PIN_STBY 4

#define PIN_SDA 0
#define PIN_SCL 1

// Test Configuration
const int TEST_PWM = 110;       // Same PWM as motor shaft test
const int WARMUP_TIME = 2000;   // 2 seconds to stabilize
const int MEASURE_TIME = 10000; // 10 seconds for averaging

// Objects
MotorDriver motor(PIN_AIN1, PIN_AIN2, PIN_STBY);
MagneticEncoder encoder;

// Helper to run test in one direction
float measure_rpm(int pwm, const char* direction_label) {
    Serial.println("");
    Serial.print(">>> TESTING "); 
    Serial.print(direction_label);
    Serial.println(" <<<");

    // 1. Start Motor & Warmup
    motor.drive(pwm);
    Serial.println("Status: Accelerating/Stabilizing...");
    
    unsigned long startTime = millis();
    while (millis() - startTime < WARMUP_TIME) {
        encoder.update(); 
    }

    // 2. Start Measurement
    Serial.println("Status: Recording Data...");
    
    float start_revs = encoder.getRevolutions();
    unsigned long measure_start = millis();
    
    while (millis() - measure_start < MEASURE_TIME) {
        encoder.update(); 
    }
    
    // 3. End Measurement
    float end_revs = encoder.getRevolutions();
    unsigned long duration = millis() - measure_start;
    
    // Stop Motor
    motor.drive(0);
    motor.brake();

    // 4. Calculate
    float total_revs = abs(end_revs - start_revs);
    float duration_min = duration / 60000.0;
    float rpm = total_revs / duration_min;

    Serial.print("Time Elapsed: "); Serial.print(duration / 1000.0); Serial.println(" s");
    Serial.print("Total Revolutions: "); Serial.println(total_revs, 4);
    Serial.print("Measured RPM: "); Serial.println(rpm, 2);
    
    return rpm;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();
    Wire.setClock(400000); 

    motor.begin();
    motor.enable();
    encoder.begin();

    if (!encoder.isMagnetDetected()) {
        Serial.println("ERROR: Magnet not detected on Output Shaft!");
        while(1);
    }

    delay(2000);
    Serial.println("=== GEARBOX OUTPUT SPEED TEST ===");
    Serial.print("PWM: "); Serial.println(TEST_PWM);

    // Run Forward Test
    float fwd_rpm = measure_rpm(TEST_PWM, "FORWARD");
    
    delay(2000); // Cool down

    // Run Reverse Test
    float rev_rpm = measure_rpm(-TEST_PWM, "REVERSE");

    Serial.println("\n=== FINAL RESULTS ===");
    float avg_output_rpm = (fwd_rpm + rev_rpm) / 2.0;
    Serial.print("Average Output RPM: "); Serial.println(avg_output_rpm, 2);
    
    Serial.println("\nUse this value to calculate ratio:");
    Serial.println("Gear Ratio = (Motor Raw RPM) / (Average Output RPM)");
}

void loop() {
    // End of test
}
