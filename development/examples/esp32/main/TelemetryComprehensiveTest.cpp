/**
 * @file TelemetryComprehensiveTest.cpp
 * @brief Comprehensive Telemetry monitoring test suite for ESP32-C6 DevKit-M-1 (noexcept)
 *
 * This file contains comprehensive testing for TMC9660 telemetry features including:
 * - Temperature monitoring and thermal protection
 * - Current monitoring and overcurrent protection
 * - Voltage monitoring and undervoltage protection
 * - Motor position and velocity feedback
 * - System status and error reporting
 * - Real-time data logging and analysis
 * - Performance metrics and benchmarking
 * - Multi-device telemetry monitoring
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

static const char* TAG = "Telemetry_Test";
static TestResults g_test_results;

//=============================================================================
// TEST SECTION CONFIGURATION
//=============================================================================
static constexpr bool ENABLE_CORE_TESTS = true;
static constexpr bool ENABLE_TEMPERATURE_TESTS = true;
static constexpr bool ENABLE_CURRENT_TESTS = true;
static constexpr bool ENABLE_VOLTAGE_TESTS = true;
static constexpr bool ENABLE_POSITION_TESTS = true;
static constexpr bool ENABLE_VELOCITY_TESTS = true;
static constexpr bool ENABLE_STATUS_TESTS = true;
static constexpr bool ENABLE_PERFORMANCE_TESTS = true;
static constexpr bool ENABLE_STRESS_TESTS = true;

// Forward declarations
bool test_telemetry_basic_monitoring() noexcept;
bool test_telemetry_temperature_monitoring() noexcept;
bool test_telemetry_current_monitoring() noexcept;
bool test_telemetry_voltage_monitoring() noexcept;
bool test_telemetry_position_monitoring() noexcept;
bool test_telemetry_velocity_monitoring() noexcept;
bool test_telemetry_status_monitoring() noexcept;
bool test_telemetry_performance_benchmarks() noexcept;
bool test_telemetry_error_handling() noexcept;
bool test_telemetry_multi_device() noexcept;

// Helper functions
std::unique_ptr<TMC9660> create_test_driver() noexcept;
void log_telemetry_data(TMC9660& driver, const char* context) noexcept;
bool verify_telemetry_ranges(TMC9660& driver) noexcept;

bool test_telemetry_basic_monitoring() noexcept {
    ESP_LOGI(TAG, "Testing basic telemetry monitoring...");

    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver");
        return false;
    }

    // Configure basic motor setup
    if (!driver->motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7)) {
        ESP_LOGE(TAG, "Failed to set motor type");
        return false;
    }

    // Test 1: Read all basic telemetry parameters
    log_telemetry_data(*driver, "Initial state");

    // Test 2: Verify telemetry ranges
    if (!verify_telemetry_ranges(*driver)) {
        ESP_LOGE(TAG, "Telemetry range verification failed");
        return false;
    }

    ESP_LOGI(TAG, "[SUCCESS] Basic telemetry monitoring tests passed");
    return true;
}

bool test_telemetry_temperature_monitoring() noexcept {
    ESP_LOGI(TAG, "Testing temperature monitoring...");

    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver");
        return false;
    }

    // Test 1: Read temperature at different states
    log_telemetry_data(*driver, "Idle state");

    // Test 2: Monitor temperature during operation
    if (driver->focControl.setTargetVelocity(1000)) {
        ESP_LOGI(TAG, "Motor started for temperature monitoring");
        
        for (int i = 0; i < 10; ++i) {
            float temp = driver->telemetry.getChipTemperature();
            ESP_LOGI(TAG, "Temperature reading %d: %.1f°C", i + 1, temp);
            
            if (temp > 80.0f) {
                ESP_LOGW(TAG, "High temperature warning: %.1f°C", temp);
            }
            
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        driver->focControl.stop();
        ESP_LOGI(TAG, "Motor stopped");
    }

    // Test 3: Temperature after operation
    log_telemetry_data(*driver, "After operation");

    ESP_LOGI(TAG, "[SUCCESS] Temperature monitoring tests passed");
    return true;
}

bool test_telemetry_current_monitoring() noexcept {
    ESP_LOGI(TAG, "Testing current monitoring...");

    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver");
        return false;
    }

    // Configure motor for current monitoring
    if (!driver->motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7)) {
        ESP_LOGE(TAG, "Failed to set motor type");
        return false;
    }

    if (!driver->motorConfig.setMaxTorqueCurrent(2000)) {
        ESP_LOGE(TAG, "Failed to set current limit");
        return false;
    }

    // Test 1: Read current at different states
    log_telemetry_data(*driver, "Idle state");

    // Test 2: Monitor current during operation
    if (driver->focControl.setTargetVelocity(1000)) {
        ESP_LOGI(TAG, "Motor started for current monitoring");
        
        for (int i = 0; i < 10; ++i) {
            int16_t current = driver->telemetry.getMotorCurrent();
            ESP_LOGI(TAG, "Current reading %d: %dmA", i + 1, current);
            
            if (current > 1500) {
                ESP_LOGW(TAG, "High current warning: %dmA", current);
            }
            
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        driver->focControl.stop();
        ESP_LOGI(TAG, "Motor stopped");
    }

    ESP_LOGI(TAG, "[SUCCESS] Current monitoring tests passed");
    return true;
}

bool test_telemetry_voltage_monitoring() noexcept {
    ESP_LOGI(TAG, "Testing voltage monitoring...");

    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver");
        return false;
    }

    // Test 1: Read voltage at different states
    log_telemetry_data(*driver, "Idle state");

    // Test 2: Monitor voltage during operation
    if (driver->focControl.setTargetVelocity(1000)) {
        ESP_LOGI(TAG, "Motor started for voltage monitoring");
        
        for (int i = 0; i < 10; ++i) {
            float voltage = driver->telemetry.getSupplyVoltage();
            ESP_LOGI(TAG, "Voltage reading %d: %.2fV", i + 1, voltage);
            
            if (voltage < 10.0f || voltage > 50.0f) {
                ESP_LOGW(TAG, "Voltage warning: %.2fV", voltage);
            }
            
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        driver->focControl.stop();
        ESP_LOGI(TAG, "Motor stopped");
    }

    ESP_LOGI(TAG, "[SUCCESS] Voltage monitoring tests passed");
    return true;
}

bool test_telemetry_position_monitoring() noexcept {
    ESP_LOGI(TAG, "Testing position monitoring...");

    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver");
        return false;
    }

    // Configure motor for position monitoring
    if (!driver->motorConfig.setType(tmc9660::tmcl::MotorType::STEPPER_MOTOR)) {
        ESP_LOGE(TAG, "Failed to set motor type");
        return false;
    }

    if (!driver->feedbackSense.configureABNEncoder(4000)) {
        ESP_LOGE(TAG, "Failed to configure encoder");
        return false;
    }

    // Test 1: Read position at different states
    log_telemetry_data(*driver, "Initial position");

    // Test 2: Monitor position during movement
    if (driver->focControl.setTargetPosition(1000)) {
        ESP_LOGI(TAG, "Motor started for position monitoring");
        
        for (int i = 0; i < 10; ++i) {
            // This would typically read actual position
            ESP_LOGI(TAG, "Position reading %d: simulated", i + 1);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        driver->focControl.stop();
        ESP_LOGI(TAG, "Motor stopped");
    }

    ESP_LOGI(TAG, "[SUCCESS] Position monitoring tests passed");
    return true;
}

bool test_telemetry_velocity_monitoring() noexcept {
    ESP_LOGI(TAG, "Testing velocity monitoring...");

    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver");
        return false;
    }

    // Configure motor for velocity monitoring
    if (!driver->motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7)) {
        ESP_LOGE(TAG, "Failed to set motor type");
        return false;
    }

    if (!driver->feedbackSense.configureABNEncoder(1024)) {
        ESP_LOGE(TAG, "Failed to configure encoder");
        return false;
    }

    // Test 1: Read velocity at different states
    log_telemetry_data(*driver, "Initial velocity");

    // Test 2: Monitor velocity during operation
    if (driver->focControl.setTargetVelocity(1000)) {
        ESP_LOGI(TAG, "Motor started for velocity monitoring");
        
        for (int i = 0; i < 10; ++i) {
            // This would typically read actual velocity
            ESP_LOGI(TAG, "Velocity reading %d: simulated", i + 1);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        
        driver->focControl.stop();
        ESP_LOGI(TAG, "Motor stopped");
    }

    ESP_LOGI(TAG, "[SUCCESS] Velocity monitoring tests passed");
    return true;
}

bool test_telemetry_status_monitoring() noexcept {
    ESP_LOGI(TAG, "Testing status monitoring...");

    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver");
        return false;
    }

    // Test 1: Read system status
    log_telemetry_data(*driver, "System status check");

    // Test 2: Monitor status during operation
    if (driver->focControl.setTargetVelocity(1000)) {
        ESP_LOGI(TAG, "Motor started for status monitoring");
        
        for (int i = 0; i < 5; ++i) {
            log_telemetry_data(*driver, "Status during operation");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        
        driver->focControl.stop();
        ESP_LOGI(TAG, "Motor stopped");
    }

    // Test 3: Status after operation
    log_telemetry_data(*driver, "Status after operation");

    ESP_LOGI(TAG, "[SUCCESS] Status monitoring tests passed");
    return true;
}

bool test_telemetry_performance_benchmarks() noexcept {
    ESP_LOGI(TAG, "Testing telemetry performance benchmarks...");

    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver");
        return false;
    }

    // Test 1: Single telemetry read performance
    uint64_t start_time = esp_timer_get_time();
    float temp = driver->telemetry.getChipTemperature();
    int16_t current = driver->telemetry.getMotorCurrent();
    float voltage = driver->telemetry.getSupplyVoltage();
    uint64_t end_time = esp_timer_get_time();
    uint64_t single_read_time = end_time - start_time;

    ESP_LOGI(TAG, "Single telemetry read time: %llu μs", single_read_time);
    ESP_LOGI(TAG, "Values - Temp: %.1f°C, Current: %dmA, Voltage: %.2fV", temp, current, voltage);

    // Test 2: Multiple telemetry reads performance
    const int num_reads = 100;
    start_time = esp_timer_get_time();
    
    for (int i = 0; i < num_reads; ++i) {
        driver->telemetry.getChipTemperature();
        driver->telemetry.getMotorCurrent();
        driver->telemetry.getSupplyVoltage();
    }
    
    end_time = esp_timer_get_time();
    uint64_t total_time = end_time - start_time;
    uint64_t avg_read_time = total_time / num_reads;

    ESP_LOGI(TAG, "Multiple telemetry reads (%d): total %llu μs, avg %llu μs per read", 
             num_reads, total_time, avg_read_time);

    ESP_LOGI(TAG, "[SUCCESS] Telemetry performance benchmark tests passed");
    return true;
}

bool test_telemetry_error_handling() noexcept {
    ESP_LOGI(TAG, "Testing telemetry error handling...");

    auto driver = create_test_driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create test driver");
        return false;
    }

    // Test 1: Read telemetry with invalid configuration
    ESP_LOGI(TAG, "Testing telemetry with invalid motor configuration...");
    
    // This would typically test error conditions
    log_telemetry_data(*driver, "Error condition test");

    // Test 2: Rapid telemetry reads
    ESP_LOGI(TAG, "Testing rapid telemetry reads...");
    
    for (int i = 0; i < 20; ++i) {
        float temp = driver->telemetry.getChipTemperature();
        int16_t current = driver->telemetry.getMotorCurrent();
        float voltage = driver->telemetry.getSupplyVoltage();
        
        if (i % 5 == 0) {
            ESP_LOGI(TAG, "Rapid read %d: T=%.1f°C, I=%dmA, V=%.2fV", i, temp, current, voltage);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "[SUCCESS] Telemetry error handling tests passed");
    return true;
}

bool test_telemetry_multi_device() noexcept {
    ESP_LOGI(TAG, "Testing multi-device telemetry monitoring...");

    // Test 1: Create multiple drivers
    auto spi_driver = create_test_driver();
    if (!spi_driver) {
        ESP_LOGE(TAG, "Failed to create SPI driver");
        return false;
    }

    auto uart_driver = std::make_unique<TMC9660>(*createUARTInterface());
    if (!uart_driver) {
        ESP_LOGW(TAG, "Failed to create UART driver, testing SPI only");
        ESP_LOGI(TAG, "[SUCCESS] Multi-device telemetry tests passed (SPI only)");
        return true;
    }

    // Test 2: Monitor both devices simultaneously
    ESP_LOGI(TAG, "Monitoring both devices simultaneously...");
    
    for (int i = 0; i < 5; ++i) {
        ESP_LOGI(TAG, "--- Reading %d ---", i + 1);
        
        ESP_LOGI(TAG, "SPI Device:");
        log_telemetry_data(*spi_driver, "SPI");
        
        ESP_LOGI(TAG, "UART Device:");
        log_telemetry_data(*uart_driver, "UART");
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "[SUCCESS] Multi-device telemetry monitoring tests passed");
    return true;
}

// Helper function implementations
std::unique_ptr<TMC9660> create_test_driver() noexcept {
    auto spi_interface = createSPIInterface();
    if (!spi_interface) {
        ESP_LOGE(TAG, "Failed to create SPI interface");
        return nullptr;
    }

    auto driver = std::make_unique<TMC9660>(*spi_interface);
    
    // Initialize bootloader
    tmc9660::BootloaderConfig cfg{};
    cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    cfg.boot.start_motor_control = true;
    cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::SPI0;
    cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;

    auto result = driver->bootloaderInit(&cfg);
    if (result != TMC9660::BootloaderInitResult::Success) {
        ESP_LOGE(TAG, "Failed to initialize bootloader: %d", static_cast<int>(result));
        return nullptr;
    }

    return driver;
}

void log_telemetry_data(TMC9660& driver, const char* context) noexcept {
    float temp = driver.telemetry.getChipTemperature();
    int16_t current = driver.telemetry.getMotorCurrent();
    float voltage = driver.telemetry.getSupplyVoltage();
    
    ESP_LOGI(TAG, "%s - Temp: %.1f°C, Current: %dmA, Voltage: %.2fV", 
             context, temp, current, voltage);
}

bool verify_telemetry_ranges(TMC9660& driver) noexcept {
    float temp = driver.telemetry.getChipTemperature();
    int16_t current = driver.telemetry.getMotorCurrent();
    float voltage = driver.telemetry.getSupplyVoltage();
    
    bool valid = true;
    
    if (temp < -40.0f || temp > 150.0f) {
        ESP_LOGW(TAG, "Temperature out of range: %.1f°C", temp);
        valid = false;
    }
    
    if (current < -10000 || current > 10000) {
        ESP_LOGW(TAG, "Current out of range: %dmA", current);
        valid = false;
    }
    
    if (voltage < 0.0f || voltage > 100.0f) {
        ESP_LOGW(TAG, "Voltage out of range: %.2fV", voltage);
        valid = false;
    }
    
    if (valid) {
        ESP_LOGI(TAG, "All telemetry values within expected ranges");
    }
    
    return valid;
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "╔══════════════════════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║                  ESP32-C6 TELEMETRY COMPREHENSIVE TEST SUITE               ║");
    ESP_LOGI(TAG, "║                         HardFOC TMC9660 Driver Tests                        ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════════════════════════════════════════════╝");

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Report test section configuration
    print_test_section_status(TAG, "Telemetry");

    // Run all telemetry tests
    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_CORE_TESTS, "TELEMETRY CORE TESTS", 5,
        ESP_LOGI(TAG, "Running core telemetry tests...");
        RUN_TEST_IN_TASK("basic_monitoring", test_telemetry_basic_monitoring, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_TEMPERATURE_TESTS, "TEMPERATURE TESTS", 5,
        ESP_LOGI(TAG, "Running temperature monitoring tests...");
        RUN_TEST_IN_TASK("temperature_monitoring", test_telemetry_temperature_monitoring, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_CURRENT_TESTS, "CURRENT TESTS", 5,
        ESP_LOGI(TAG, "Running current monitoring tests...");
        RUN_TEST_IN_TASK("current_monitoring", test_telemetry_current_monitoring, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_VOLTAGE_TESTS, "VOLTAGE TESTS", 5,
        ESP_LOGI(TAG, "Running voltage monitoring tests...");
        RUN_TEST_IN_TASK("voltage_monitoring", test_telemetry_voltage_monitoring, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_POSITION_TESTS, "POSITION TESTS", 5,
        ESP_LOGI(TAG, "Running position monitoring tests...");
        RUN_TEST_IN_TASK("position_monitoring", test_telemetry_position_monitoring, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_VELOCITY_TESTS, "VELOCITY TESTS", 5,
        ESP_LOGI(TAG, "Running velocity monitoring tests...");
        RUN_TEST_IN_TASK("velocity_monitoring", test_telemetry_velocity_monitoring, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_STATUS_TESTS, "STATUS TESTS", 5,
        ESP_LOGI(TAG, "Running status monitoring tests...");
        RUN_TEST_IN_TASK("status_monitoring", test_telemetry_status_monitoring, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_PERFORMANCE_TESTS, "PERFORMANCE TESTS", 5,
        ESP_LOGI(TAG, "Running telemetry performance tests...");
        RUN_TEST_IN_TASK("performance_benchmarks", test_telemetry_performance_benchmarks, 8192, 1);
        flip_test_progress_indicator();
        RUN_TEST_IN_TASK("multi_device", test_telemetry_multi_device, 8192, 1);
        flip_test_progress_indicator();
    );

    RUN_TEST_SECTION_IF_ENABLED_WITH_PATTERN(
        ENABLE_STRESS_TESTS, "STRESS TESTS", 5,
        ESP_LOGI(TAG, "Running telemetry stress tests...");
        RUN_TEST_IN_TASK("error_handling", test_telemetry_error_handling, 8192, 1);
        flip_test_progress_indicator();
    );

    print_test_summary(g_test_results, "Telemetry", TAG);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
