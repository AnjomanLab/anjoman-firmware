#pragma once

#include <Arduino.h>

#ifdef PIN_RGB_LED
    #undef PIN_RGB_LED
#endif

#ifndef ROBOT_ID
    #define ROBOT_ID 1
#endif

// ==============================================================================
// 1. I2C SENSING BUS ARCHITECTURE
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
    // Robots 1, 3, 4: Multiplexed I2C Architecture via TCA9548A (0x70)
    #define ROBOT_HAS_TCA9548A           true

    constexpr uint8_t PIN_I2C0_SDA       = 1;
    constexpr uint8_t PIN_I2C0_SCL       = 2;
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
// 3. DEDICATED SPI3 BUS (MicroSD Card Module - VARIANT SPECIFIC)
// ==============================================================================
#if ROBOT_ID == 1
    // Robot 1 Original SPI3 Pins
    constexpr uint8_t PIN_SD_MOSI    = 11;
    constexpr uint8_t PIN_SD_MISO    = 12;
    constexpr uint8_t PIN_SD_SCK     = 14;
    constexpr uint8_t PIN_SD_CS      = 9;
#else
    // Robots 2, 3, 4 Conflict-Free Dedicated SPI3 Pins
    constexpr uint8_t PIN_SD_MOSI    = 42;
    constexpr uint8_t PIN_SD_MISO    = 47;
    constexpr uint8_t PIN_SD_SCK     = 21;
    constexpr uint8_t PIN_SD_CS      = 13;
#endif

// ==============================================================================
// 4. BATTERY VOLTAGE MONITORING (ADC1 PIN)
// ==============================================================================
#if ROBOT_ID == 2
    constexpr uint8_t PIN_VBAT_SENSE = 3; // ADC1_CH0 on Robot 2
#else
    constexpr uint8_t PIN_VBAT_SENSE = 4; // ADC1_CH3 on Robots 1, 3, 4
#endif

// ==============================================================================
// 5. MOTOR DRIVE (DRV8833 Dual H-Bridge PWM - ALL ROBOTS)
// ==============================================================================
constexpr uint8_t PIN_MOTOR_L_IN1    = 15;
constexpr uint8_t PIN_MOTOR_L_IN2    = 16;
constexpr uint8_t PIN_MOTOR_R_IN1    = 17;
constexpr uint8_t PIN_MOTOR_R_IN2    = 18;

// ==============================================================================
// 6. SYSTEM INTERFACES & DIAGNOSTICS
// ==============================================================================
constexpr uint8_t PIN_STATUS_RGB     = 48;
constexpr uint8_t PIN_BOOT_BTN       = 0;
