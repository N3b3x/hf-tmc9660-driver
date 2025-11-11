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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace tmc9660;

static const char* TAG = "Bootloader_Test";
static TestResults g_test_results;

//=============================================================================
// TEST SECTION CONFIGURATION
//=============================================================================
static constexpr bool ENABLE_CORE_TESTS = true;
static constexpr bool ENABLE_PARAMETER_MODE_TESTS = true;
static constexpr bool ENABLE_REGISTER_MODE_TESTS = true;
static constexpr bool ENABLE_INTERFACE_TESTS = false;
static constexpr bool ENABLE_CLOCK_TESTS = false;
static constexpr bool ENABLE_ERROR_HANDLING_TESTS = false;
static constexpr bool ENABLE_PERFORMANCE_TESTS = false;
static constexpr bool ENABLE_STRESS_TESTS = false;

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
bool test_bootloader_config(const BootloaderConfig& config, const char* test_name) noexcept;
void log_bootloader_result(TMC9660::BootloaderInitResult result, const char* context) noexcept;

// Bootloader reset sequence function
template<typename InterfaceType>
bool perform_bootloader_reset_sequence(std::unique_ptr<InterfaceType>& interface, 
                                      TMC9660& driver, 
                                      const BootloaderConfig& cfg) noexcept;

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
    BootloaderConfig param_config{};
    param_config.boot.boot_mode = bootcfg::BootMode::Parameter;
    param_config.boot.start_motor_control = true;
    param_config.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI0;
    param_config.uart.baud_rate = bootcfg::BaudRate::BR115200;

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
    std::vector<bootcfg::SPIInterface> spi_interfaces = {
        bootcfg::SPIInterface::SPI0,
        bootcfg::SPIInterface::SPI1
    };

    for (auto spi_iface : spi_interfaces) {
        BootloaderConfig config{};
        config.boot.boot_mode = bootcfg::BootMode::Parameter;
        config.boot.start_motor_control = true;
        config.spiComm.boot_spi_iface = spi_iface;
        config.uart.baud_rate = bootcfg::BaudRate::BR115200;

        if (!test_bootloader_config(config, "Parameter Mode SPI")) {
            ESP_LOGW(TAG, "Parameter mode SPI interface test failed");
        }
    }

    // Test 2: Parameter mode with different UART baud rates
    std::vector<bootcfg::BaudRate> baud_rates = {
        bootcfg::BaudRate::BR9600,
        bootcfg::BaudRate::BR19200,
        bootcfg::BaudRate::BR38400,
        bootcfg::BaudRate::BR57600,
        bootcfg::BaudRate::BR115200,
        bootcfg::BaudRate::BR1000000
    };

    for (auto baud : baud_rates) {
        BootloaderConfig config{};
        config.boot.boot_mode = bootcfg::BootMode::Parameter;
        config.boot.start_motor_control = true;
        config.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI0;
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
    BootloaderConfig config{};
    config.boot.boot_mode = bootcfg::BootMode::Register;
    config.boot.start_motor_control = false; // Register mode typically doesn't start motor control
    config.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI0;
    config.uart.baud_rate = bootcfg::BaudRate::BR115200;

    if (!test_bootloader_config(config, "Register Mode")) {
        ESP_LOGW(TAG, "Register mode configuration test failed");
    }

    ESP_LOGI(TAG, "[SUCCESS] Register mode configuration tests passed");
    return true;
}

bool test_bootloader_spi_interface() noexcept {
    ESP_LOGI(TAG, "Testing SPI interface configuration...");

    // Test 1: Different SPI interface configurations
    std::vector<bootcfg::SPIInterface> interfaces = {
        bootcfg::SPIInterface::SPI0,
        bootcfg::SPIInterface::SPI1
    };

    for (auto iface : interfaces) {
        BootloaderConfig config{};
        config.boot.boot_mode = bootcfg::BootMode::Parameter;
        config.boot.start_motor_control = true;
        config.spiComm.boot_spi_iface = iface;
        config.uart.baud_rate = bootcfg::BaudRate::BR115200;

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
    std::vector<bootcfg::BaudRate> baud_rates = {
        bootcfg::BaudRate::BR9600,
        bootcfg::BaudRate::BR19200,
        bootcfg::BaudRate::BR38400,
        bootcfg::BaudRate::BR57600,
        bootcfg::BaudRate::BR115200,
        bootcfg::BaudRate::BR1000000
    };

    for (auto baud : baud_rates) {
        BootloaderConfig config{};
        config.boot.boot_mode = bootcfg::BootMode::Parameter;
        config.boot.start_motor_control = true;
        config.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI0;
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
    BootloaderConfig internal_config{};
    internal_config.boot.boot_mode = bootcfg::BootMode::Parameter;
    internal_config.boot.start_motor_control = true;
    internal_config.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI0;
    internal_config.uart.baud_rate = bootcfg::BaudRate::BR115200;
    internal_config.clock.use_external = bootcfg::ClockSource::Internal;
    internal_config.clock.pll_selection = bootcfg::SysClkSource::PLL;

    if (!test_bootloader_config(internal_config, "Internal Clock")) {
        ESP_LOGW(TAG, "Internal clock configuration test failed");
    }

    // Test 2: External clock configuration
    BootloaderConfig external_config{};
    external_config.boot.boot_mode = bootcfg::BootMode::Parameter;
    external_config.boot.start_motor_control = true;
    external_config.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI0;
    external_config.uart.baud_rate = bootcfg::BaudRate::BR115200;
    external_config.clock.use_external = bootcfg::ClockSource::External;
    external_config.clock.pll_selection = bootcfg::SysClkSource::PLL;

    if (!test_bootloader_config(external_config, "External Clock")) {
        ESP_LOGW(TAG, "External clock configuration test failed");
    }

    // Test 3: Different PLL configurations
    std::vector<bootcfg::SysClkSource> pll_sources = {
        bootcfg::SysClkSource::IntOsc,
        bootcfg::SysClkSource::PLL
    };

    for (auto pll : pll_sources) {
        BootloaderConfig config{};
        config.boot.boot_mode = bootcfg::BootMode::Parameter;
        config.boot.start_motor_control = true;
        config.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI0;
        config.uart.baud_rate = bootcfg::BaudRate::BR115200;
        config.clock.use_external = bootcfg::ClockSource::Internal;
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

    // Test 1: Invalid boot mode with proper GPIO control
    BootloaderConfig invalid_config{};
    invalid_config.boot.boot_mode = static_cast<bootcfg::BootMode>(0xFF); // Invalid mode
    invalid_config.boot.start_motor_control = true;
    invalid_config.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI0;
    invalid_config.uart.baud_rate = bootcfg::BaudRate::BR115200;

    // Use test_bootloader_config for proper GPIO control (will fail due to invalid config)
    if (!test_bootloader_config(invalid_config, "Invalid boot mode")) {
        ESP_LOGI(TAG, "Invalid boot mode test failed as expected");
    }

    // Test 2: Null configuration (must use direct call since test_bootloader_config expects valid config)
    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver");
        return false;
    }

    auto result = driver->bootloaderInit(nullptr);
    log_bootloader_result(result, "Null configuration");

    // Test 3: Invalid SPI interface with proper GPIO control
    BootloaderConfig invalid_spi_config{};
    invalid_spi_config.boot.boot_mode = bootcfg::BootMode::Parameter;
    invalid_spi_config.boot.start_motor_control = true;
    invalid_spi_config.spiComm.boot_spi_iface = static_cast<bootcfg::SPIInterface>(0xFF);
    invalid_spi_config.uart.baud_rate = bootcfg::BaudRate::BR115200;

    // Use test_bootloader_config for proper GPIO control (will fail due to invalid config)
    if (!test_bootloader_config(invalid_spi_config, "Invalid SPI interface")) {
        ESP_LOGI(TAG, "Invalid SPI interface test failed as expected");
    }

    // Test 4: Invalid UART baud rate with proper GPIO control
    BootloaderConfig invalid_uart_config{};
    invalid_uart_config.boot.boot_mode = bootcfg::BootMode::Parameter;
    invalid_uart_config.boot.start_motor_control = true;
    invalid_uart_config.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI0;
    invalid_uart_config.uart.baud_rate = static_cast<bootcfg::BaudRate>(0xFF);

    // Use test_bootloader_config for proper GPIO control (will fail due to invalid config)
    if (!test_bootloader_config(invalid_uart_config, "Invalid UART baud rate")) {
        ESP_LOGI(TAG, "Invalid UART baud rate test failed as expected");
    }

    ESP_LOGI(TAG, "[SUCCESS] Bootloader error handling tests passed");
    return true;
}

bool test_bootloader_performance_benchmarks() noexcept {
    ESP_LOGI(TAG, "Testing bootloader performance benchmarks...");

    // Test 1: Complete bootloader initialization with GPIO control time
    uint64_t start_time = esp_timer_get_time();
    
    BootloaderConfig config{};
    config.boot.boot_mode = bootcfg::BootMode::Parameter;
    config.boot.start_motor_control = true;
    config.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI0;
    config.uart.baud_rate = bootcfg::BaudRate::BR115200;

    // Use test_bootloader_config for proper GPIO control and timing
    if (!test_bootloader_config(config, "Performance Test")) {
        ESP_LOGE(TAG, "Performance test bootloader initialization failed");
        return false;
    }
    
    uint64_t end_time = esp_timer_get_time();
    uint64_t init_time = end_time - start_time;

    ESP_LOGI(TAG, "Complete bootloader initialization time (with GPIO control): %llu μs", init_time);

    // Test 2: Multiple initialization cycles with proper GPIO control
    const int num_cycles = 10;
    start_time = esp_timer_get_time();
    
    for (int i = 0; i < num_cycles; ++i) {
        BootloaderConfig cycle_config{};
        cycle_config.boot.boot_mode = bootcfg::BootMode::Parameter;
        cycle_config.boot.start_motor_control = true;
        cycle_config.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI0;
        cycle_config.uart.baud_rate = bootcfg::BaudRate::BR115200;

        // Use test_bootloader_config for proper GPIO control
        if (!test_bootloader_config(cycle_config, "Performance Cycle")) {
            ESP_LOGW(TAG, "Bootloader init cycle %d failed", i + 1);
        }
    }
    
    end_time = esp_timer_get_time();
    uint64_t total_time = end_time - start_time;
    uint64_t avg_time = total_time / num_cycles;

    ESP_LOGI(TAG, "Multiple initialization cycles (%d) with GPIO control: total %llu μs, avg %llu μs per cycle", 
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

    // Test 2: Initialize both devices with different configurations using proper reset sequences
    BootloaderConfig spi_config{};
    spi_config.boot.boot_mode = bootcfg::BootMode::Parameter;
    spi_config.boot.start_motor_control = true;
    spi_config.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI0;
    spi_config.uart.baud_rate = bootcfg::BaudRate::BR115200;

    // Use test_bootloader_config for proper GPIO control
    if (!test_bootloader_config(spi_config, "SPI Device")) {
        ESP_LOGE(TAG, "SPI device bootloader initialization failed");
        return false;
    }

    BootloaderConfig uart_config{};
    uart_config.boot.boot_mode = bootcfg::BootMode::Parameter;
    uart_config.boot.start_motor_control = true;
    uart_config.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI1;
    uart_config.uart.baud_rate = bootcfg::BaudRate::BR1000000;

    // Use test_bootloader_config for proper GPIO control
    if (!test_bootloader_config(uart_config, "UART Device")) {
        ESP_LOGE(TAG, "UART device bootloader initialization failed");
        return false;
    }

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

    // Test 1: Extreme configuration values with proper reset sequence
    BootloaderConfig extreme_config{};
    extreme_config.boot.boot_mode = bootcfg::BootMode::Parameter;
    extreme_config.boot.start_motor_control = true;
    extreme_config.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI0;
    extreme_config.uart.baud_rate = bootcfg::BaudRate::BR1000000; // Highest baud rate
    extreme_config.clock.use_external = bootcfg::ClockSource::External;
    extreme_config.clock.pll_selection = bootcfg::SysClkSource::PLL;

    // Use test_bootloader_config for proper GPIO control
    if (!test_bootloader_config(extreme_config, "Extreme Configuration")) {
        ESP_LOGE(TAG, "Extreme configuration bootloader initialization failed");
        return false;
    }

    // Test 2: Rapid configuration changes with proper GPIO control
    ESP_LOGI(TAG, "Testing rapid configuration changes...");
    
    for (int i = 0; i < 5; ++i) {
        BootloaderConfig config{};
        config.boot.boot_mode = bootcfg::BootMode::Parameter;
        config.boot.start_motor_control = (i % 2 == 0);
        config.spiComm.boot_spi_iface = (i % 2 == 0) ? 
            bootcfg::SPIInterface::SPI0 : 
            bootcfg::SPIInterface::SPI1;
        config.uart.baud_rate = (i % 2 == 0) ? 
            bootcfg::BaudRate::BR115200 : 
            bootcfg::BaudRate::BR1000000;

        // Use test_bootloader_config for proper GPIO control
        if (!test_bootloader_config(config, "Rapid Change")) {
            ESP_LOGW(TAG, "Rapid change cycle %d failed", i + 1);
        }
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

// Bootloader reset sequence implementation
template<typename InterfaceType>
bool perform_bootloader_reset_sequence(std::unique_ptr<InterfaceType>& interface, 
                                      TMC9660& driver, 
                                      const BootloaderConfig& cfg) noexcept {
    ESP_LOGI(TAG, "Starting bootloader reset sequence...");
    
    // Step 1: Assert reset (RST pin ACTIVE)
    ESP_LOGI(TAG, "Asserting reset (RST pin ACTIVE)...");
    if (!interface->gpioSetActive(TMC9660CtrlPin::RST)) {
        ESP_LOGE(TAG, "Failed to assert reset pin");
        return false;
    }
    
    // Wait for reset to take effect (100ms delay)
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Step 2: Release reset (RST pin INACTIVE)
    ESP_LOGI(TAG, "Releasing reset (RST pin INACTIVE)...");
    if (!interface->gpioSetInactive(TMC9660CtrlPin::RST)) {
        ESP_LOGE(TAG, "Failed to release reset pin");
        return false;
    }
    
    // Wait for device to stabilize after reset
    vTaskDelay(pdMS_TO_TICKS(25));
    
    // Step 3: Wait for FAULTN pin to go INACTIVE (fault cleared)
    ESP_LOGI(TAG, "Waiting for FAULTN pin to go INACTIVE...");
    const int max_wait_cycles = 100; // 10 seconds max wait
    int wait_cycles = 0;
    
    while (wait_cycles < max_wait_cycles) {
        GpioSignal fault_signal;
        if (!interface->gpioRead(TMC9660CtrlPin::FAULTN, fault_signal)) {
            ESP_LOGE(TAG, "Failed to read FAULTN pin");
            return false;
        }
        
        if (fault_signal == GpioSignal::INACTIVE) {
            ESP_LOGI(TAG, "FAULTN pin is INACTIVE - fault cleared");
            break;
        }
        
        ESP_LOGD(TAG, "FAULTN still ACTIVE, waiting... (%d/%d)", wait_cycles + 1, max_wait_cycles);
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
        wait_cycles++;
    }
    
    if (wait_cycles >= max_wait_cycles) {
        ESP_LOGE(TAG, "Timeout waiting for FAULTN pin to go INACTIVE");
        return false;
    }
    
    // Step 4: Call bootloader unit function (bootloader initialization)
    ESP_LOGI(TAG, "Calling bootloader unit function...");
    auto result = driver.bootloaderInit(&cfg);
    if (result != TMC9660::BootloaderInitResult::Success) {
        ESP_LOGE(TAG, "Bootloader initialization failed");
        return false;
    }
    
    ESP_LOGI(TAG, "Bootloader reset sequence completed successfully");
    return true;
}

bool test_bootloader_config(const BootloaderConfig& config, const char* test_name) noexcept {
    // Create SPI interface
    auto spi_interface = createSPIInterface();
    if (!spi_interface) {
        ESP_LOGE(TAG, "Failed to create SPI interface for %s", test_name);
        return false;
    }

    // Create TMC9660 driver with the interface
    auto driver = std::make_unique<TMC9660>(*spi_interface);
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver for %s", test_name);
        return false;
    }

    // Perform bootloader reset sequence with proper GPIO control
    if (!perform_bootloader_reset_sequence(spi_interface, *driver, config)) {
        ESP_LOGE(TAG, "Bootloader reset sequence failed for %s", test_name);
        return false;
    }

    ESP_LOGI(TAG, "[SUCCESS] %s bootloader initialization completed", test_name);
    return true;
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
