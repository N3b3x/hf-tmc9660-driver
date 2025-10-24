/**
 * @file UnifiedCommMain.cpp
 * @brief Main application demonstrating unified communication interface
 *
 * This is the main application file that demonstrates the unified communication
 * interface for TMC9660. It shows how to switch between SPI and UART modes
 * in real-time and perform various operations.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#include "TMC9660.hpp"
#include "Esp32TMC9660UnifiedBus.hpp"
#include "UnifiedCommExample.cpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "UnifiedComm_Main";

extern "C" void app_main() {
    ESP_LOGI(TAG, "Starting TMC9660 Unified Communication Interface Demo");
    ESP_LOGI(TAG, "=====================================================");

    // Initialize NVS (required for ESP-IDF)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Wait a bit for system to stabilize
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Run the unified communication example
    bool success = run_unified_comm_example();
    
    if (success) {
        ESP_LOGI(TAG, "✅ Unified communication example completed successfully");
        
        // Run the application example
        ESP_LOGI(TAG, "Running application example...");
        if (run_application_example()) {
            ESP_LOGI(TAG, "✅ Application example completed successfully");
        } else {
            ESP_LOGE(TAG, "❌ Application example failed");
        }
    } else {
        ESP_LOGE(TAG, "❌ Unified communication example failed");
    }

    ESP_LOGI(TAG, "=====================================================");
    ESP_LOGI(TAG, "Demo completed. System will continue running...");
    
    // Keep the system running
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "System running... (unified comm interface available)");
    }
}