#pragma once

#include <Arduino.h>

#ifdef PIN_RGB_LED
#undef PIN_RGB_LED
#endif

// ==============================================================================
// 1. I2C1 Bus (BMI160 IMU + TCA9548A Multiplexer for Dual AS5600 Encoders)
// ==============================================================================
constexpr uint8_t PIN_I2C1_SDA       = 1;
constexpr uint8_t PIN_I2C1_SCL       = 2;

// ==============================================================================
// 2. Dedicated SPI2 Bus (Decawave DW1000 UWB Transceiver ONLY)
// ==============================================================================
constexpr uint8_t PIN_UWB_MOSI       = 41;
constexpr uint8_t PIN_UWB_MISO       = 40;
constexpr uint8_t PIN_UWB_SCK        = 39;
constexpr uint8_t PIN_UWB_CS         = 38;
constexpr uint8_t PIN_UWB_RST        = 10;
constexpr uint8_t PIN_UWB_IRQ        = 5;

// ==============================================================================
// 3. Dedicated SPI3 Bus (MicroSD Card Module ONLY)
// ==============================================================================
constexpr uint8_t PIN_SD_MOSI        = 11;
constexpr uint8_t PIN_SD_MISO        = 12;
constexpr uint8_t PIN_SD_SCK         = 14;
constexpr uint8_t PIN_SD_CS          = 9;

// ==============================================================================
// 4. Motor Drive (DRV8833 Dual H-Bridge)
// ==============================================================================
constexpr uint8_t PIN_MOTOR_L_IN1    = 15;
constexpr uint8_t PIN_MOTOR_L_IN2    = 16;
constexpr uint8_t PIN_MOTOR_R_IN1    = 17;
constexpr uint8_t PIN_MOTOR_R_IN2    = 18;

// ==============================================================================
// 5. System Interfaces & Diagnostics
// ==============================================================================
constexpr uint8_t PIN_STATUS_RGB     = 48;
constexpr uint8_t PIN_BOOT_BTN       = 0;
