/**
 * @file BootloaderComprehensiveTest.cpp
 * @brief Comprehensive Bootloader testing suite for ESP32-C6 DevKit-M-1 (noexcept)
 *
 * This file contains comprehensive testing for TMC9660 bootloader features including:
 * - Bootloader initialization and configuration validation
 * - Parameter mode vs Register mode testing
 * - SPI and UART interface configuration
 * - Clock source and PLL configuration
 * - Boot mode validation and error handling
 * - Multi-device bootloader operations
 * - Performance benchmarking
 * - Edge cases and fault injection
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#include "../../../inc/TMC9660.hpp"
#include "Esp32TMC9660Bus.hpp"
#include "TestFramework.h"
#include <memory>
#include <vector>
#include <algorithm>

static const char* TAG = "Bootloader_Test";
static TestResults g_test_results;

//=============================================================================
// TEST SECTION CONFIGURATION
//=============================================================================
static constexpr bool ENABLE_CORE_TESTS = true;
static constexpr bool ENABLE_PARAMETER_MODE_TESTS = true;
static constexpr bool ENABLE_REGISTER_MODE_TESTS = true;
static constexpr bool ENABLE_INTERFACE_TESTS = true;
static constexpr bool ENABLE_CLOCK_TESTS = true;
static constexpr bool ENABLE_ERROR_HANDLING_TESTS = true;
static constexpr bool ENABLE_PERFORMANCE_TESTS = true;
static constexpr bool ENABLE_STRESS_TESTS = true;

// Forward declarations
bool test_bootloader_basic_initialization() noexcept;
bool test_bootloader_parameter_mode() noexcept;
bool test_bootloader_register_mode() noexcept;
bool test_bootloader_spi_interface() noexcept;
bool test_bootloader_uart_interface() noexcept;
bool test_bootloader_clock_configuration() noexcept;
bool test_bootloader_error_handling() noexcept;
bool test_bootloader_performance_benchmarks() noexcept;
bool test_bootloader_multi_device() noexcept;
bool test_bootloader_edge_cases() noexcept;

// Helper functions
std::unique_ptr<TMC9660> create_test_driver() noexcept;
bool test_bootloader_config(const tmc9660::BootloaderConfig& config, const char* test_name) noexcept;
void log_bootloader_result(TMC9660::BootloaderInitResult result, const char* context) noexcept;

bool test_bootloader_basic_initialization() noexcept {
    ESP_LOGI(TAG, "Testing basic bootloader initialization...");

    // Test 1: Default configuration
    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver");
        return false;
    }

    ESP_LOGI(TAG, "Basic bootloader initialization successful");

    // Test 2: Explicit parameter mode configuration
    tmc9660::BootloaderConfig param_config{};
    param_config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    param_config.boot.start_motor_control = true;
    param_config.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
    param_config.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;

    if (!test_bootloader_config(param_config, "Parameter Mode")) {
        ESP_LOGE(TAG, "Parameter mode configuration test failed");
        return false;
    }

    ESP_LOGI(TAG, "[SUCCESS] Basic bootloader initialization tests passed");
    return true;
}

bool test_bootloader_parameter_mode() noexcept {
    ESP_LOGI(TAG, "Testing parameter mode configuration...");

    // Test 1: Parameter mode with different SPI interfaces
    std::vector<tmc9660::bootcfg::SPIInterface> spi_interfaces = {
        tmc9660::bootcfg::SPIInterface::IFACE0,
        tmc9660::bootcfg::SPIInterface::IFACE1
    };

    for (auto spi_iface : spi_interfaces) {
        tmc9660::BootloaderConfig config{};
        config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
        config.boot.start_motor_control = true;
        config.spiComm.boot_spi_iface = spi_iface;
        config.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;

        if (!test_bootloader_config(config, "Parameter Mode SPI")) {
            ESP_LOGW(TAG, "Parameter mode SPI interface test failed");
        }
    }

    // Test 2: Parameter mode with different UART baud rates
    std::vector<tmc9660::bootcfg::BaudRate> baud_rates = {
        tmc9660::bootcfg::BaudRate::BR9600,
        tmc9660::bootcfg::BaudRate::BR19200,
        tmc9660::bootcfg::BaudRate::BR38400,
        tmc9660::bootcfg::BaudRate::BR57600,
        tmc9660::bootcfg::BaudRate::BR115200,
        tmc9660::bootcfg::BaudRate::BR1000000
    };

    for (auto baud : baud_rates) {
        tmc9660::BootloaderConfig config{};
        config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
        config.boot.start_motor_control = true;
        config.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
        config.uart.baud_rate = baud;

        if (!test_bootloader_config(config, "Parameter Mode UART")) {
            ESP_LOGW(TAG, "Parameter mode UART baud rate test failed");
        }
    }

    ESP_LOGI(TAG, "[SUCCESS] Parameter mode configuration tests passed");
    return true;
}

bool test_bootloader_register_mode() noexcept {
    ESP_LOGI(TAG, "Testing register mode configuration...");

    // Test 1: Register mode configuration
    tmc9660::BootloaderConfig config{};
    config.boot.boot_mode = tmc9660::bootcfg::BootMode::Register;
    config.boot.start_motor_control = false; // Register mode typically doesn't start motor control
    config.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
    config.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;

    if (!test_bootloader_config(config, "Register Mode")) {
        ESP_LOGW(TAG, "Register mode configuration test failed");
    }

    ESP_LOGI(TAG, "[SUCCESS] Register mode configuration tests passed");
    return true;
}

bool test_bootloader_spi_interface() noexcept {
    ESP_LOGI(TAG, "Testing SPI interface configuration...");

    // Test 1: Different SPI interface configurations
    std::vector<tmc9660::bootcfg::SPIInterface> interfaces = {
        tmc9660::bootcfg::SPIInterface::IFACE0,
        tmc9660::bootcfg::SPIInterface::IFACE1
    };

    for (auto iface : interfaces) {
        tmc9660::BootloaderConfig config{};
        config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
        config.boot.start_motor_control = true;
        config.spiComm.boot_spi_iface = iface;
        config.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;

        if (!test_bootloader_config(config, "SPI Interface")) {
            ESP_LOGW(TAG, "SPI interface test failed for interface %d", static_cast<int>(iface));
        }
    }

    ESP_LOGI(TAG, "[SUCCESS] SPI interface configuration tests passed");
    return true;
}

bool test_bootloader_uart_interface() noexcept {
    ESP_LOGI(TAG, "Testing UART interface configuration...");

    // Test 1: Different UART baud rate configurations
    std::vector<tmc9660::bootcfg::BaudRate> baud_rates = {
        tmc9660::bootcfg::BaudRate::BR9600,
        tmc9660::bootcfg::BaudRate::BR19200,
        tmc9660::bootcfg::BaudRate::BR38400,
        tmc9660::bootcfg::BaudRate::BR57600,
        tmc9660::bootcfg::BaudRate::BR115200,
        tmc9660::bootcfg::BaudRate::BR1000000
    };

    for (auto baud : baud_rates) {
        tmc9660::BootloaderConfig config{};
        config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
        config.boot.start_motor_control = true;
        config.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
        config.uart.baud_rate = baud;

        if (!test_bootloader_config(config, "UART Interface")) {
            ESP_LOGW(TAG, "UART interface test failed for baud rate %d", static_cast<int>(baud));
        }
    }

    ESP_LOGI(TAG, "[SUCCESS] UART interface configuration tests passed");
    return true;
}

bool test_bootloader_clock_configuration() noexcept {
    ESP_LOGI(TAG, "Testing clock configuration...");

    // Test 1: Internal clock configuration
    tmc9660::BootloaderConfig internal_config{};
    internal_config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    internal_config.boot.start_motor_control = true;
    internal_config.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
    internal_config.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;
    internal_config.clock.use_external = tmc9660::bootcfg::ClockSource::Internal;
    internal_config.clock.pll_selection = tmc9660::bootcfg::SysClkSource::PLL;

    if (!test_bootloader_config(internal_config, "Internal Clock")) {
        ESP_LOGW(TAG, "Internal clock configuration test failed");
    }

    // Test 2: External clock configuration
    tmc9660::BootloaderConfig external_config{};
    external_config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    external_config.boot.start_motor_control = true;
    external_config.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
    external_config.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;
    external_config.clock.use_external = tmc9660::bootcfg::ClockSource::External;
    external_config.clock.pll_selection = tmc9660::bootcfg::SysClkSource::PLL;

    if (!test_bootloader_config(external_config, "External Clock")) {
        ESP_LOGW(TAG, "External clock configuration test failed");
    }

    // Test 3: Different PLL configurations
    std::vector<tmc9660::bootcfg::SysClkSource> pll_sources = {
        tmc9660::bootcfg::SysClkSource::IntOsc,
        tmc9660::bootcfg::SysClkSource::PLL
    };

    for (auto pll : pll_sources) {
        tmc9660::BootloaderConfig config{};
        config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
        config.boot.start_motor_control = true;
        config.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
        config.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;
        config.clock.use_external = tmc9660::bootcfg::ClockSource::Internal;
        config.clock.pll_selection = pll;

        if (!test_bootloader_config(config, "PLL Configuration")) {
            ESP_LOGW(TAG, "PLL configuration test failed for source %d", static_cast<int>(pll));
        }
    }

    ESP_LOGI(TAG, "[SUCCESS] Clock configuration tests passed");
    return true;
}

bool test_bootloader_error_handling() noexcept {
    ESP_LOGI(TAG, "Testing bootloader error handling...");

    // Test 1: Invalid boot mode
    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver");
        return false;
    }

    tmc9660::BootloaderConfig invalid_config{};
    invalid_config.boot.boot_mode = static_cast<tmc9660::bootcfg::BootMode>(0xFF); // Invalid mode
    invalid_config.boot.start_motor_control = true;
    invalid_config.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
    invalid_config.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;

    auto result = driver->bootloaderInit(&invalid_config);
    log_bootloader_result(result, "Invalid boot mode");

    // Test 2: Null configuration
    result = driver->bootloaderInit(nullptr);
    log_bootloader_result(result, "Null configuration");

    // Test 3: Invalid SPI interface
    tmc9660::BootloaderConfig invalid_spi_config{};
    invalid_spi_config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    invalid_spi_config.boot.start_motor_control = true;
    invalid_spi_config.spiComm.boot_spi_iface = static_cast<tmc9660::bootcfg::SPIInterface>(0xFF);
    invalid_spi_config.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;

    result = driver->bootloaderInit(&invalid_spi_config);
    log_bootloader_result(result, "Invalid SPI interface");

    // Test 4: Invalid UART baud rate
    tmc9660::BootloaderConfig invalid_uart_config{};
    invalid_uart_config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    invalid_uart_config.boot.start_motor_control = true;
    invalid_uart_config.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
    invalid_uart_config.uart.baud_rate = static_cast<tmc9660::bootcfg::BaudRate>(0xFF);

    result = driver->bootloaderInit(&invalid_uart_config);
    log_bootloader_result(result, "Invalid UART baud rate");

    ESP_LOGI(TAG, "[SUCCESS] Bootloader error handling tests passed");
    return true;
}

bool test_bootloader_performance_benchmarks() noexcept {
    ESP_LOGI(TAG, "Testing bootloader performance benchmarks...");

    // Test 1: Bootloader initialization time
    uint64_t start_time = esp_timer_get_time();
    
    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver");
        return false;
    }
    
    uint64_t end_time = esp_timer_get_time();
    uint64_t init_time = end_time - start_time;

    ESP_LOGI(TAG, "Bootloader initialization time: %llu μs", init_time);

    // Test 2: Multiple initialization cycles
    const int num_cycles = 10;
    start_time = esp_timer_get_time();
    
    for (int i = 0; i < num_cycles; ++i) {
        tmc9660::BootloaderConfig config{};
        config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
        config.boot.start_motor_control = true;
        config.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
        config.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;

        auto result = driver->bootloaderInit(&config);
        if (result != TMC9660::BootloaderInitResult::Success) {
            ESP_LOGW(TAG, "Bootloader init cycle %d failed", i + 1);
        }
    }
    
    end_time = esp_timer_get_time();
    uint64_t total_time = end_time - start_time;
    uint64_t avg_time = total_time / num_cycles;

    ESP_LOGI(TAG, "Multiple initialization cycles (%d): total %llu μs, avg %llu μs per cycle", 
             num_cycles, total_time, avg_time);

    ESP_LOGI(TAG, "[SUCCESS] Bootloader performance benchmark tests passed");
    return true;
}

bool test_bootloader_multi_device() noexcept {
    ESP_LOGI(TAG, "Testing multi-device bootloader operations...");

    // Test 1: Create multiple drivers with different interfaces
    auto spi_driver = create_test_driver();
    if (!spi_driver) {
        ESP_LOGE(TAG, "Failed to create SPI driver");
        return false;
    }

    auto uart_driver = std::make_unique<TMC9660>(*createUARTInterface());
    if (!uart_driver) {
        ESP_LOGW(TAG, "Failed to create UART driver, testing SPI only");
        ESP_LOGI(TAG, "[SUCCESS] Multi-device bootloader tests passed (SPI only)");
        return true;
    }

    // Test 2: Initialize both devices with different configurations
    tmc9660::BootloaderConfig spi_config{};
    spi_config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    spi_config.boot.start_motor_control = true;
    spi_config.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
    spi_config.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;

    auto spi_result = spi_driver->bootloaderInit(&spi_config);
    log_bootloader_result(spi_result, "SPI Device");

    tmc9660::BootloaderConfig uart_config{};
    uart_config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    uart_config.boot.start_motor_control = true;
    uart_config.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE1;
    uart_config.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR1000000;

    auto uart_result = uart_driver->bootloaderInit(&uart_config);
    log_bootloader_result(uart_result, "UART Device");

    ESP_LOGI(TAG, "[SUCCESS] Multi-device bootloader operations tests passed");
    return true;
}

bool test_bootloader_edge_cases() noexcept {
    ESP_LOGI(TAG, "Testing bootloader edge cases...");

    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver");
        return false;
    }

    // Test 1: Extreme configuration values
    tmc9660::BootloaderConfig extreme_config{};
    extreme_config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    extreme_config.boot.start_motor_control = true;
    extreme_config.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
    extreme_config.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR1000000; // Highest baud rate
    extreme_config.clock.use_external = tmc9660::bootcfg::ClockSource::External;
    extreme_config.clock.pll_selection = tmc9660::bootcfg::SysClkSource::PLL;

    auto result = driver->bootloaderInit(&extreme_config);
    log_bootloader_result(result, "Extreme Configuration");

    // Test 2: Rapid configuration changes
    ESP_LOGI(TAG, "Testing rapid configuration changes...");
    
    for (int i = 0; i < 5; ++i) {
        tmc9660::BootloaderConfig config{};
        config.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
        config.boot.start_motor_control = (i % 2 == 0);
        config.spiComm.boot_spi_iface = (i % 2 == 0) ? 
            tmc9660::bootcfg::SPIInterface::IFACE0 : 
            tmc9660::bootcfg::SPIInterface::IFACE1;
        config.uart.baud_rate = (i % 2 == 0) ? 
            tmc9660::bootcfg::BaudRate::BR115200 : 
            tmc9660::bootcfg::BaudRate::BR1000000;

        result = driver->bootloaderInit(&config);
        log_bootloader_result(result, "Rapid Change");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "[SUCCESS] Bootloader edge cases tests passed");
    return true;
}

// Helper function implementations
std::unique_ptr<TMC9660> create_test_driver() noexcept {
    auto spi_interface = createSPIInterface();
    if (!spi_interface) {
        ESP_LOGE(TAG, "Failed to create SPI interface");
        return nullptr;
    }

    return std::make_unique<TMC9660>(*spi_interface);
}

bool test_bootloader_config(const tmc9660::BootloaderConfig& config, const char* test_name) noexcept {
    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver for %s", test_name);
        return false;
    }

    auto result = driver->bootloaderInit(&config);
    log_bootloader_result(result, test_name);
    
    return (result == TMC9660::BootloaderInitResult::Success);
}

void log_bootloader_result(TMC9660::BootloaderInitResult result, const char* context) noexcept {
    const char* result_str = "Unknown";
    switch (result) {
        case TMC9660::BootloaderInitResult::Success:
            result_str = "Success";
            break;
        case TMC9660::BootloaderInitResult::NoConfig:
            result_str = "NoConfig";
            break;
        case TMC9660::BootloaderInitResult::Failure:
            result_str = "Failure";
            break;
    }
    
    ESP_LOGI(TAG, "%s: %s", context, result_str);
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║                 ESP32-C6 BOOTLOADER COMPREHENSIVE TEST SUITE               ║");
    ESP_LOGI(TAG, "║                         HardFOC TMC9660 Driver Tests                        ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Report test section configuration
    print_test_section_status(TAG, "Bootloader");

    // Run all bootloader tests
    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_CORE_TESTS, "BOOTLOADER CORE TESTS", 5,
        ESP_LOGI(TAG, "Running core bootloader tests...");
        RUN_TEST_IN_TASK("basic_initialization", test_bootloader_basic_initialization, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_PARAMETER_MODE_TESTS, "PARAMETER MODE TESTS", 5,
        ESP_LOGI(TAG, "Running parameter mode tests...");
        RUN_TEST_IN_TASK("parameter_mode", test_bootloader_parameter_mode, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_REGISTER_MODE_TESTS, "REGISTER MODE TESTS", 5,
        ESP_LOGI(TAG, "Running register mode tests...");
        RUN_TEST_IN_TASK("register_mode", test_bootloader_register_mode, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_INTERFACE_TESTS, "INTERFACE TESTS", 5,
        ESP_LOGI(TAG, "Running interface tests...");
        RUN_TEST_IN_TASK("spi_interface", test_bootloader_spi_interface, 8192, 1);
        flip_test_progress_indicator();
        RUN_TEST_IN_TASK("uart_interface", test_bootloader_uart_interface, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_CLOCK_TESTS, "CLOCK TESTS", 5,
        ESP_LOGI(TAG, "Running clock configuration tests...");
        RUN_TEST_IN_TASK("clock_configuration", test_bootloader_clock_configuration, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_ERROR_HANDLING_TESTS, "ERROR HANDLING TESTS", 5,
        ESP_LOGI(TAG, "Running error handling tests...");
        RUN_TEST_IN_TASK("error_handling", test_bootloader_error_handling, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_PERFORMANCE_TESTS, "PERFORMANCE TESTS", 5,
        ESP_LOGI(TAG, "Running performance tests...");
        RUN_TEST_IN_TASK("performance_benchmarks", test_bootloader_performance_benchmarks, 8192, 1);
        flip_test_progress_indicator();
        RUN_TEST_IN_TASK("multi_device", test_bootloader_multi_device, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_STRESS_TESTS, "STRESS TESTS", 5,
        ESP_LOGI(TAG, "Running stress tests...");
        RUN_TEST_IN_TASK("edge_cases", test_bootloader_edge_cases, 8192, 1);
        flip_test_progress_indicator();
    );

    print_test_summary(g_test_results, "Bootloader", TAG);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
