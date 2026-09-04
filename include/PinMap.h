#pragma once

#include <Arduino.h>

#ifdef PIN_RGB_LED
    #undef PIN_RGB_LED
#endif

#ifndef ROBOT_ID
    #define ROBOT_ID 1
#endif

// ==============================================================================
// 1. I2C SENSING BUS ARCHITECTURE (ROBOT 2 DUAL-BUS vs ROBOTS 1, 3, 4 MULTIPLEXED)
// ==============================================================================
#if ROBOT_ID == 2
    // Robot 2: Direct Dual Hardware I2C Architecture (No Multiplexer)
    #define ROBOT_HAS_TCA9548A           false

    // Primary I2C (Wire) -> Left AS5600 Encoder (0x36) + Bosch BMI160 IMU (0x69)
    constexpr uint8_t PIN_I2C0_SDA       = 7;
    constexpr uint8_t PIN_I2C0_SCL       = 8;

    // Secondary I2C (Wire1) -> Right AS5600 Encoder (0x36) ONLY
    constexpr uint8_t PIN_I2C1_SDA       = 4;
    constexpr uint8_t PIN_I2C1_SCL       = 6;

#else
    // Robots 1, 3, 4: Single Multiplexed I2C Architecture via TCA9548A
    #define ROBOT_HAS_TCA9548A           true

    // Shared I2C (Wire) -> TCA9548A Multiplexer (0x70)
    // Ch0: Left AS5600 (0x36) | Ch1: Right AS5600 (0x36) | Ch2: BMI160 IMU (0x69)
    constexpr uint8_t PIN_I2C0_SDA       = 1;
    constexpr uint8_t PIN_I2C0_SCL       = 2;

    // Alias for backwards compatibility
    constexpr uint8_t PIN_I2C1_SDA       = PIN_I2C0_SDA;
    constexpr uint8_t PIN_I2C1_SCL       = PIN_I2C0_SCL;
#endif

// ==============================================================================
// 2. DEDICATED SPI2 BUS (Decawave DW1000 UWB Transceiver ONLY - ALL ROBOTS)
// ==============================================================================
constexpr uint8_t PIN_UWB_MOSI       = 41;
constexpr uint8_t PIN_UWB_MISO       = 40;
constexpr uint8_t PIN_UWB_SCK        = 39;
constexpr uint8_t PIN_UWB_CS         = 38;
constexpr uint8_t PIN_UWB_RST        = 10;
constexpr uint8_t PIN_UWB_IRQ        = 5;

// ==============================================================================
// 3. DEDICATED SPI3 BUS (MicroSD Card Module ONLY - ALL ROBOTS)
// ==============================================================================
constexpr uint8_t PIN_SD_MOSI        = 11;
constexpr uint8_t PIN_SD_MISO        = 12;
constexpr uint8_t PIN_SD_SCK         = 14;
constexpr uint8_t PIN_SD_CS          = 9;

// ==============================================================================
// 4. MOTOR DRIVE (DRV8833 Dual H-Bridge PWM - ALL ROBOTS)
// ==============================================================================
constexpr uint8_t PIN_MOTOR_L_IN1    = 15;
constexpr uint8_t PIN_MOTOR_L_IN2    = 16;
constexpr uint8_t PIN_MOTOR_R_IN1    = 17;
constexpr uint8_t PIN_MOTOR_R_IN2    = 18;

// ==============================================================================
// 5. SYSTEM INTERFACES & DIAGNOSTICS
// ==============================================================================
constexpr uint8_t PIN_STATUS_RGB     = 48;
constexpr uint8_t PIN_BOOT_BTN       = 0;

// Reserved / Unused GPIOs:
// Robot 1, 3, 4: GPIO 4, 6, 7, 8, 42
// Robot 2      : GPIO 1, 2, 42
