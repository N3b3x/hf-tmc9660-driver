/**
 * @file esp32_tmc9660_test_config.hpp
 * @brief Hardware configuration for TMC9660 driver on ESP32-C6
 *
 * This file contains the actual hardware configuration that is used by the HAL
 * and example applications. Modify these values to match your hardware setup.
 *
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 */

#pragma once

#include <cstdint>

//==============================================================================
// COMPILE-TIME CONFIGURATION FLAGS
//==============================================================================

/**
 * @brief Enable detailed SPI/UART transaction logging
 *
 * @details
 * When enabled (set to 1), the Esp32Tmc9660SpiBus/Esp32Tmc9660UartBus will log
 * detailed information about each transaction including:
 * - TX/RX frame bytes
 * - TMCL command/response parsing
 * - CRC verification results
 *
 * When disabled (set to 0), only basic error logging is performed.
 *
 * Default: 0 (disabled) - Set to 1 to enable for debugging
 */
#ifndef ESP32_TMC9660_ENABLE_DETAILED_LOGGING
#define ESP32_TMC9660_ENABLE_DETAILED_LOGGING 0
#endif

namespace TMC9660_TestConfig {

/**
 * @brief SPI Pin Configuration for ESP32-C6
 *
 * These pins are used for SPI communication with the TMC9660.
 * Ensure your hardware matches these pin assignments or modify accordingly.
 */
struct SPIPins {
    static constexpr uint8_t MISO = 2;          ///< GPIO2 - SPI MISO (Master In Slave Out)
    static constexpr uint8_t MOSI = 7;          ///< GPIO7 - SPI MOSI (Master Out Slave In)
    static constexpr uint8_t SCLK = 6;          ///< GPIO6 - SPI Clock
    static constexpr uint8_t CS   = 18;         ///< GPIO18 - Chip Select (active low)
};

/**
 * @brief UART Pin Configuration for ESP32-C6
 *
 * These pins are used for UART (single-wire) communication with the TMC9660.
 */
struct UARTPins {
    static constexpr uint8_t TX = 5;            ///< GPIO5 - UART TX
    static constexpr uint8_t RX = 4;            ///< GPIO4 - UART RX
};

/**
 * @brief Control GPIO Pins for TMC9660
 *
 * These pins control device operation and status monitoring.
 */
struct ControlPins {
    static constexpr uint8_t RST     = 22;      ///< GPIO22 - Reset pin (active HIGH)
    static constexpr uint8_t DRV_EN  = 20;      ///< GPIO20 - Driver enable (active HIGH)
    static constexpr uint8_t FAULTN  = 19;      ///< GPIO19 - Fault indicator (active LOW, open-drain)
    static constexpr uint8_t WAKE    = 21;      ///< GPIO21 - Wake control (active LOW)
};

/**
 * @brief SPI Communication Parameters
 *
 * The TMC9660 supports SPI frequencies up to 10MHz.
 *
 * ⚠️ CRITICAL: TMC9660 requires SPI Mode 3 (CPOL=1, CPHA=1).
 * Using wrong SPI mode will result in communication failure.
 *
 * Data format: 64-bit (8-byte) SPI transfers for parameter mode.
 */
struct SPIParams {
    static constexpr uint32_t FREQUENCY = 1000000;    ///< 1MHz SPI frequency
    static constexpr uint8_t MODE = 3;                ///< SPI Mode 3 (CPOL=1, CPHA=1) ⚠️ CRITICAL
    static constexpr uint8_t QUEUE_SIZE = 1;          ///< Transaction queue size
    static constexpr uint8_t SPI_HOST_ID = 2;         ///< SPI2_HOST
};

/**
 * @brief UART Communication Parameters
 *
 * Configuration for UART (TMCL single-wire) communication.
 */
struct UARTParams {
    static constexpr uint32_t BAUD_RATE = 115200;    ///< UART baud rate
    static constexpr uint8_t ADDRESS = 0;            ///< TMCL module address
    static constexpr uint8_t UART_PORT = 1;          ///< UART_NUM_1
};

/**
 * @brief Motor Controller Specifications
 *
 * TMC9660 is an integrated motor controller with FOC support.
 */
struct ControllerSpecs {
    static constexpr float MAX_SUPPLY_VOLTAGE = 60.0f;   ///< Maximum supply voltage (V)
    static constexpr float MAX_CURRENT_A = 3.0f;         ///< Maximum phase current (A)
    static constexpr uint32_t PWM_FREQUENCY_HZ = 25000;  ///< Default PWM frequency (Hz)
};

/**
 * @brief Supply Voltage Specifications (volts)
 *
 * VS: Motor supply voltage
 * VDD: Logic supply
 */
struct SupplyVoltage {
    static constexpr float VS_MIN = 8.0f;      ///< Minimum motor supply voltage (V)
    static constexpr float VS_NOM = 24.0f;     ///< Nominal motor supply voltage (V)
    static constexpr float VS_MAX = 60.0f;     ///< Maximum motor supply voltage (V)
    static constexpr float VDD_NOM = 3.3f;     ///< Logic supply voltage (V)
};

/**
 * @brief Temperature Specifications (celsius)
 *
 * Operating temperature range from TMC9660 datasheet.
 */
struct Temperature {
    static constexpr int16_t OPERATING_MIN = -40;    ///< Minimum operating temperature (°C)
    static constexpr int16_t OPERATING_MAX = 125;    ///< Maximum operating temperature (°C)
    static constexpr int16_t JUNCTION_MAX = 150;     ///< Maximum junction temperature (°C)
    static constexpr int16_t WARNING_THRESHOLD = 110; ///< Temperature warning threshold (°C)
};

/**
 * @brief Timing Parameters
 *
 * Timing requirements from the TMC9660 datasheet.
 */
struct Timing {
    static constexpr uint16_t POWER_ON_DELAY_MS = 100;      ///< Power-on initialization delay (ms)
    static constexpr uint16_t RESET_DELAY_MS = 50;          ///< Reset recovery delay (ms)
    static constexpr uint16_t BOOT_DELAY_MS = 500;          ///< Bootloader startup delay (ms)
    static constexpr uint16_t INTER_FRAME_US = 10;          ///< Minimum time between frames (μs)
};

/**
 * @brief Diagnostic Thresholds
 *
 * Thresholds for motor controller fault detection.
 */
struct Diagnostics {
    static constexpr uint16_t POLL_INTERVAL_MS = 100;      ///< Diagnostic polling interval (ms)
    static constexpr uint8_t MAX_RETRY_COUNT = 3;          ///< Maximum communication retries
    static constexpr uint8_t MAX_FAULT_COUNT = 5;          ///< Faults before failsafe
};

/**
 * @brief Test Configuration
 *
 * Default parameters for testing.
 */
struct TestConfig {
    static constexpr uint16_t TEST_DURATION_MS = 5000;       ///< Test duration (ms)
    static constexpr uint16_t RAMP_STEP_DELAY_MS = 100;     ///< Velocity ramp step delay (ms)
    static constexpr int32_t DEFAULT_VELOCITY = 1000;        ///< Default test velocity (RPM)
    static constexpr uint16_t REGISTER_READ_COUNT = 100;     ///< Register reads per test
};

/**
 * @brief Application-specific Configuration
 *
 * Configuration values that can be adjusted per application.
 */
struct AppConfig {
    // Logging
    static constexpr bool ENABLE_DEBUG_LOGGING = true;     ///< Enable detailed debug logs
    static constexpr bool ENABLE_BUS_LOGGING = false;      ///< Enable SPI/UART transaction logs

    // Performance
    static constexpr bool ENABLE_PERFORMANCE_MONITORING = true;  ///< Enable performance metrics
    static constexpr uint16_t STATS_REPORT_INTERVAL_MS = 10000;  ///< Statistics reporting interval

    // Error handling
    static constexpr bool ENABLE_AUTO_RECOVERY = true;     ///< Enable automatic error recovery
    static constexpr uint8_t MAX_ERROR_COUNT = 10;         ///< Maximum errors before failsafe
};

} // namespace TMC9660_TestConfig

/**
 * @brief Hardware configuration validation
 *
 * Compile-time checks to ensure configuration is valid.
 */
static_assert(TMC9660_TestConfig::SPIParams::FREQUENCY <= 10000000,
              "SPI frequency exceeds TMC9660 maximum of 10MHz");

static_assert(TMC9660_TestConfig::SPIParams::MODE == 3,
              "TMC9660 requires SPI Mode 3 (CPOL=1, CPHA=1)");

static_assert(TMC9660_TestConfig::UARTParams::BAUD_RATE >= 9600 &&
              TMC9660_TestConfig::UARTParams::BAUD_RATE <= 921600,
              "UART baud rate must be between 9600 and 921600");

/**
 * @brief Helper macro for compile-time GPIO pin validation
 */
#define TMC9660_VALIDATE_GPIO(pin) \
    static_assert((pin) >= 0 && (pin) < 30, "Invalid GPIO pin number for ESP32-C6")
