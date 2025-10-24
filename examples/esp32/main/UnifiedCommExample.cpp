/**
 * @file UnifiedCommExample.cpp
 * @brief Example demonstrating real-time SPI/UART switching with TMC9660
 *
 * This example shows how to use the unified communication interface to
 * switch between SPI and UART communication modes in real-time. It
 * demonstrates various scenarios including:
 * - Basic mode switching
 * - Fallback scenarios
 * - Performance comparison
 * - Error handling
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#include "TMC9660.hpp"
#include "Esp32TMC9660UnifiedBus.hpp"
#include "TestFramework.h"
#include <memory>
#include <chrono>

static const char* TAG = "UnifiedComm_Example";

/**
 * @brief Test basic mode switching functionality
 * @param unified_interface Reference to unified communication interface
 * @return true if all tests passed
 */
bool test_basic_mode_switching(TMC9660UnifiedCommInterface& unified_interface) noexcept {
    ESP_LOGI(TAG, "Testing basic mode switching...");

    // Test 1: Switch to SPI mode
    if (!unified_interface.switchToSPI()) {
        ESP_LOGE(TAG, "Failed to switch to SPI mode");
        return false;
    }
    ESP_LOGI(TAG, "✅ Successfully switched to SPI mode");

    // Test 2: Switch to UART mode
    if (!unified_interface.switchToUART()) {
        ESP_LOGE(TAG, "Failed to switch to UART mode");
        return false;
    }
    ESP_LOGI(TAG, "✅ Successfully switched to UART mode");

    // Test 3: Switch back to SPI
    if (!unified_interface.switchToSPI()) {
        ESP_LOGE(TAG, "Failed to switch back to SPI mode");
        return false;
    }
    ESP_LOGI(TAG, "✅ Successfully switched back to SPI mode");

    ESP_LOGI(TAG, "✅ Basic mode switching tests passed");
    return true;
}

/**
 * @brief Test communication with both modes
 * @param unified_interface Reference to unified communication interface
 * @return true if all tests passed
 */
bool test_communication_both_modes(TMC9660UnifiedCommInterface& unified_interface) noexcept {
    ESP_LOGI(TAG, "Testing communication with both modes...");

    // Create a TMC9660 driver using the unified interface
    TMC9660 driver(unified_interface, 0);

    // Test SPI mode
    ESP_LOGI(TAG, "Testing SPI mode communication...");
    if (!unified_interface.switchToSPI()) {
        ESP_LOGE(TAG, "Failed to switch to SPI mode");
        return false;
    }

    // Test basic communication in SPI mode
    uint32_t version = 0;
    if (!driver.getVersion(version)) {
        ESP_LOGW(TAG, "SPI mode: Failed to get version (device may not be connected)");
    } else {
        ESP_LOGI(TAG, "SPI mode: Version = 0x%08X", version);
    }

    // Test UART mode
    ESP_LOGI(TAG, "Testing UART mode communication...");
    if (!unified_interface.switchToUART()) {
        ESP_LOGE(TAG, "Failed to switch to UART mode");
        return false;
    }

    // Test basic communication in UART mode
    if (!driver.getVersion(version)) {
        ESP_LOGW(TAG, "UART mode: Failed to get version (device may not be connected)");
    } else {
        ESP_LOGI(TAG, "UART mode: Version = 0x%08X", version);
    }

    ESP_LOGI(TAG, "✅ Communication tests completed");
    return true;
}

/**
 * @brief Test performance comparison between modes
 * @param unified_interface Reference to unified communication interface
 * @return true if all tests passed
 */
bool test_performance_comparison(TMC9660UnifiedCommInterface& unified_interface) noexcept {
    ESP_LOGI(TAG, "Testing performance comparison between modes...");

    TMC9660 driver(unified_interface, 0);
    const int num_iterations = 100;
    
    // Test SPI performance
    ESP_LOGI(TAG, "Testing SPI performance...");
    if (!unified_interface.switchToSPI()) {
        ESP_LOGE(TAG, "Failed to switch to SPI mode");
        return false;
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    int spi_success_count = 0;
    
    for (int i = 0; i < num_iterations; ++i) {
        uint32_t version = 0;
        if (driver.getVersion(version)) {
            spi_success_count++;
        }
    }
    
    auto spi_end_time = std::chrono::high_resolution_clock::now();
    auto spi_duration = std::chrono::duration_cast<std::chrono::milliseconds>(spi_end_time - start_time);

    // Test UART performance
    ESP_LOGI(TAG, "Testing UART performance...");
    if (!unified_interface.switchToUART()) {
        ESP_LOGE(TAG, "Failed to switch to UART mode");
        return false;
    }

    start_time = std::chrono::high_resolution_clock::now();
    int uart_success_count = 0;
    
    for (int i = 0; i < num_iterations; ++i) {
        uint32_t version = 0;
        if (driver.getVersion(version)) {
            uart_success_count++;
        }
    }
    
    auto uart_end_time = std::chrono::high_resolution_clock::now();
    auto uart_duration = std::chrono::duration_cast<std::chrono::milliseconds>(uart_end_time - start_time);

    // Report results
    ESP_LOGI(TAG, "Performance Results:");
    ESP_LOGI(TAG, "  SPI:  %d/%d successful, %ld ms", spi_success_count, num_iterations, spi_duration.count());
    ESP_LOGI(TAG, "  UART: %d/%d successful, %ld ms", uart_success_count, num_iterations, uart_duration.count());
    
    if (spi_duration.count() > 0 && uart_duration.count() > 0) {
        float spi_rate = (float)spi_success_count / spi_duration.count() * 1000.0f;
        float uart_rate = (float)uart_success_count / uart_duration.count() * 1000.0f;
        ESP_LOGI(TAG, "  SPI Rate:  %.2f operations/second", spi_rate);
        ESP_LOGI(TAG, "  UART Rate: %.2f operations/second", uart_rate);
    }

    ESP_LOGI(TAG, "✅ Performance comparison completed");
    return true;
}

/**
 * @brief Test fallback scenarios
 * @param unified_interface Reference to unified communication interface
 * @return true if all tests passed
 */
bool test_fallback_scenarios(TMC9660UnifiedCommInterface& unified_interface) noexcept {
    ESP_LOGI(TAG, "Testing fallback scenarios...");

    TMC9660CommModeManager mode_manager(unified_interface);

    // Test 1: Check available modes
    ESP_LOGI(TAG, "Available modes: %s", mode_manager.getAvailableModes().c_str());

    // Test 2: Try switching to other mode
    if (mode_manager.hasBothModes()) {
        ESP_LOGI(TAG, "Both modes available, testing switch to other mode...");
        if (mode_manager.switchToOtherMode()) {
            ESP_LOGI(TAG, "✅ Successfully switched to other mode");
            
            // Switch back
            if (mode_manager.switchToLastMode()) {
                ESP_LOGI(TAG, "✅ Successfully switched back to last mode");
            } else {
                ESP_LOGE(TAG, "❌ Failed to switch back to last mode");
                return false;
            }
        } else {
            ESP_LOGE(TAG, "❌ Failed to switch to other mode");
            return false;
        }
    } else {
        ESP_LOGW(TAG, "Only one mode available, skipping fallback test");
    }

    // Test 3: Test error handling with invalid operations
    ESP_LOGI(TAG, "Testing error handling...");
    
    // This should work fine
    if (mode_manager.getCurrentMode() == CommMode::SPI) {
        if (!mode_manager.switchToSPI()) {
            ESP_LOGE(TAG, "❌ Failed to switch to current mode (SPI)");
            return false;
        }
    } else {
        if (!mode_manager.switchToUART()) {
            ESP_LOGE(TAG, "❌ Failed to switch to current mode (UART)");
            return false;
        }
    }

    ESP_LOGI(TAG, "✅ Fallback scenarios completed");
    return true;
}

/**
 * @brief Test real-time switching during operation
 * @param unified_interface Reference to unified communication interface
 * @return true if all tests passed
 */
bool test_realtime_switching(TMC9660UnifiedCommInterface& unified_interface) noexcept {
    ESP_LOGI(TAG, "Testing real-time switching during operation...");

    TMC9660 driver(unified_interface, 0);
    TMC9660CommModeManager mode_manager(unified_interface);

    if (!mode_manager.hasBothModes()) {
        ESP_LOGW(TAG, "Both modes not available, skipping real-time switching test");
        return true;
    }

    // Perform operations while switching modes
    for (int cycle = 0; cycle < 5; ++cycle) {
        ESP_LOGI(TAG, "Cycle %d: Switching modes during operation...", cycle + 1);

        // Start with SPI
        if (!mode_manager.switchToSPI()) {
            ESP_LOGE(TAG, "Failed to switch to SPI in cycle %d", cycle + 1);
            return false;
        }

        // Perform some operations
        for (int i = 0; i < 3; ++i) {
            uint32_t version = 0;
            driver.getVersion(version); // May fail if device not connected
        }

        // Switch to UART
        if (!mode_manager.switchToUART()) {
            ESP_LOGE(TAG, "Failed to switch to UART in cycle %d", cycle + 1);
            return false;
        }

        // Perform some operations
        for (int i = 0; i < 3; ++i) {
            uint32_t version = 0;
            driver.getVersion(version); // May fail if device not connected
        }
    }

    ESP_LOGI(TAG, "✅ Real-time switching test completed");
    return true;
}

/**
 * @brief Main example function
 * @return true if all tests passed
 */
bool run_unified_comm_example() noexcept {
    ESP_LOGI(TAG, "Starting Unified Communication Interface Example");
    ESP_LOGI(TAG, "================================================");

    // Create unified interface
    auto unified_interface = createUnifiedInterface();
    if (!unified_interface) {
        ESP_LOGE(TAG, "Failed to create unified interface");
        return false;
    }

    ESP_LOGI(TAG, "Unified interface created successfully");
    ESP_LOGI(TAG, "Status: %s", unified_interface->getDetailedStatus().c_str());

    // Run tests
    bool all_passed = true;

    // Test 1: Basic mode switching
    if (!test_basic_mode_switching(*unified_interface)) {
        ESP_LOGE(TAG, "❌ Basic mode switching test failed");
        all_passed = false;
    }

    // Test 2: Communication with both modes
    if (!test_communication_both_modes(*unified_interface)) {
        ESP_LOGE(TAG, "❌ Communication test failed");
        all_passed = false;
    }

    // Test 3: Performance comparison
    if (!test_performance_comparison(*unified_interface)) {
        ESP_LOGE(TAG, "❌ Performance comparison test failed");
        all_passed = false;
    }

    // Test 4: Fallback scenarios
    if (!test_fallback_scenarios(*unified_interface)) {
        ESP_LOGE(TAG, "❌ Fallback scenarios test failed");
        all_passed = false;
    }

    // Test 5: Real-time switching
    if (!test_realtime_switching(*unified_interface)) {
        ESP_LOGE(TAG, "❌ Real-time switching test failed");
        all_passed = false;
    }

    // Final status
    if (all_passed) {
        ESP_LOGI(TAG, "✅ All unified communication tests passed!");
    } else {
        ESP_LOGE(TAG, "❌ Some unified communication tests failed");
    }

    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "Unified Communication Interface Example Complete");
    
    return all_passed;
}

/**
 * @brief Example showing how to use the unified interface in a real application
 * @return true if example completed successfully
 */
bool run_application_example() noexcept {
    ESP_LOGI(TAG, "Running Application Example with Unified Interface");
    ESP_LOGI(TAG, "=================================================");

    // Create TMC9660 driver with unified interface
    auto driver = createUnifiedTMC9660Driver();
    if (!driver) {
        ESP_LOGE(TAG, "Failed to create TMC9660 driver with unified interface");
        return false;
    }

    // Get the unified interface for mode switching
    auto unified_interface = std::make_unique<Esp32TMC9660UnifiedCommInterface>(
        Esp32TMC9660BusConfig{});
    if (!unified_interface || !unified_interface->initialize()) {
        ESP_LOGE(TAG, "Failed to create unified interface for application example");
        return false;
    }

    TMC9660CommModeManager mode_manager(*unified_interface);

    ESP_LOGI(TAG, "Application started with unified communication interface");
    ESP_LOGI(TAG, "Available modes: %s", mode_manager.getAvailableModes().c_str());

    // Simulate application logic with mode switching
    for (int phase = 0; phase < 3; ++phase) {
        ESP_LOGI(TAG, "Phase %d: Application logic with mode switching", phase + 1);

        // Phase 1: Use SPI for high-speed operations
        if (mode_manager.switchToSPI()) {
            ESP_LOGI(TAG, "Phase %d: Using SPI for high-speed operations", phase + 1);
            
            // Simulate high-speed operations
            for (int i = 0; i < 10; ++i) {
                uint32_t version = 0;
                driver->getVersion(version);
                vTaskDelay(pdMS_TO_TICKS(10)); // Small delay
            }
        }

        // Phase 2: Switch to UART for debugging
        if (mode_manager.switchToUART()) {
            ESP_LOGI(TAG, "Phase %d: Using UART for debugging operations", phase + 1);
            
            // Simulate debugging operations
            for (int i = 0; i < 5; ++i) {
                uint32_t version = 0;
                driver->getVersion(version);
                vTaskDelay(pdMS_TO_TICKS(50)); // Longer delay for debugging
            }
        }

        // Phase 3: Switch back to SPI for final operations
        if (mode_manager.switchToSPI()) {
            ESP_LOGI(TAG, "Phase %d: Using SPI for final operations", phase + 1);
            
            // Simulate final operations
            for (int i = 0; i < 5; ++i) {
                uint32_t version = 0;
                driver->getVersion(version);
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }

    ESP_LOGI(TAG, "✅ Application example completed successfully");
    return true;
}

// Export functions for use in main.cpp
extern "C" {
    bool run_unified_comm_example_c() {
        return run_unified_comm_example();
    }
    
    bool run_application_example_c() {
        return run_application_example();
    }
}