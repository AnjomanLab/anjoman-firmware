# Anjoman Firmware

Firmware for the "Anjoman" project: A decentralized, leaderless formation control framework for differential drive robots without GPS.

## Hardware Architecture
- **MCU:** RP2040 (Motor control, Sensors) + ESP32-S3 (UWB, High-level logic)
- **Sensors:** AS5600 (Encoders), VL53L7CX (ToF), DW1000 (UWB), BMI160 (IMU)
- **Actuators:** DC Motors with DRV8833 Driver

## Dependencies
- PlatformIO
- Arduino Framework for RP2040
