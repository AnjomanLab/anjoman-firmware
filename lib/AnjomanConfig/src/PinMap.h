#pragma once

#include <Arduino.h>

#ifdef PIN_RGB_LED
    #undef PIN_RGB_LED
#endif

#ifndef ROBOT_ID
    #define ROBOT_ID 1
#endif

// ==============================================================================
// 1. I2C SENSING BUS (TCA9548A multiplexer for encoders + IMU + INA226)
// ==============================================================================
#define ROBOT_HAS_TCA9548A               true

#if ROBOT_ID == 2
    constexpr uint8_t PIN_I2C0_SDA       = 7;
    constexpr uint8_t PIN_I2C0_SCL       = 8;
#else
    constexpr uint8_t PIN_I2C0_SDA       = 1;
    constexpr uint8_t PIN_I2C0_SCL       = 2;
#endif

constexpr uint8_t PIN_I2C1_SDA           = PIN_I2C0_SDA;
constexpr uint8_t PIN_I2C1_SCL           = PIN_I2C0_SCL;

// ==============================================================================
// 2. DW1000 UWB — dedicated SPI bus
// ==============================================================================
#if ROBOT_ID == 2
    constexpr uint8_t PIN_UWB_MOSI       = 11;
    constexpr uint8_t PIN_UWB_SCK        = 12;
    constexpr uint8_t PIN_UWB_MISO       = 13;
    constexpr uint8_t PIN_UWB_CS         = 14;
#else
    constexpr uint8_t PIN_UWB_MOSI       = 41;
    constexpr uint8_t PIN_UWB_SCK        = 39;
    constexpr uint8_t PIN_UWB_MISO       = 40;
    constexpr uint8_t PIN_UWB_CS         = 38;
#endif

constexpr uint8_t PIN_UWB_RST            = 10;
constexpr uint8_t PIN_UWB_IRQ            = 5;

// ==============================================================================
// 3. MicroSD — dedicated SPI bus
// ==============================================================================
#if ROBOT_ID == 1
    constexpr uint8_t PIN_SD_MOSI        = 11;
    constexpr uint8_t PIN_SD_MISO        = 12;
    constexpr uint8_t PIN_SD_SCK         = 14;
    constexpr uint8_t PIN_SD_CS          = 9;
#else
    constexpr uint8_t PIN_SD_MOSI        = 42;
    constexpr uint8_t PIN_SD_MISO        = 47;
    constexpr uint8_t PIN_SD_SCK         = 21;
    constexpr uint8_t PIN_SD_CS          = 13;
#endif

// ==============================================================================
// 4. Battery voltage monitoring
// ==============================================================================
constexpr uint8_t PIN_VBAT_SENSE         = 4;

// ==============================================================================
// 5. Motor drive (DRV8833)
// ==============================================================================
#if ROBOT_ID == 1
    constexpr uint8_t PIN_MOTOR_L_IN1    = 6;
    constexpr uint8_t PIN_MOTOR_L_IN2    = 7;
#else
    constexpr uint8_t PIN_MOTOR_L_IN1    = 15;
    constexpr uint8_t PIN_MOTOR_L_IN2    = 16;
#endif

constexpr uint8_t PIN_MOTOR_R_IN1        = 17;
constexpr uint8_t PIN_MOTOR_R_IN2        = 18;

// ==============================================================================
// 6. System interfaces
// ==============================================================================
constexpr uint8_t PIN_STATUS_RGB         = 48;
constexpr uint8_t PIN_BOOT_BTN           = 0;
