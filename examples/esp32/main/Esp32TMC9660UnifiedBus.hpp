/**
 * @file Esp32TMC9660UnifiedBus.hpp
 * @brief ESP32-specific unified communication interface for TMC9660 with real-time SPI/UART switching
 *
 * This file provides ESP32-specific implementations of the unified communication interface
 * that can dynamically switch between SPI and UART communication modes in real-time.
 * It extends the base unified interface with ESP32-specific configuration and management.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#pragma once

#include "TMC9660UnifiedCommInterface.hpp"
#include "Esp32TMC9660Bus.hpp"
#include <memory>

/**
 * @brief ESP32-specific unified communication interface
 * 
 * This class provides ESP32-specific implementation of the unified communication
 * interface with real-time switching between SPI and UART modes. It manages
 * ESP32-specific resources and provides convenient factory methods.
 */
class Esp32TMC9660UnifiedCommInterface : public TMC9660UnifiedCommInterface {
public:
    /**
     * @brief Construct ESP32 unified communication interface
     * @param config Bus configuration for both SPI and UART
     */
    Esp32TMC9660UnifiedCommInterface(const Esp32TMC9660BusConfig& config) noexcept
        : TMC9660UnifiedCommInterface(true, true, false, false), // RST: HIGH, DRV_EN: HIGH, WAKE: LOW, FAULTN: LOW
          config_(config), initialized_(false) {
    }

    /**
     * @brief Destructor - cleans up all resources
     */
    ~Esp32TMC9660UnifiedCommInterface() noexcept override {
        deinitializeAll();
    }

    /**
     * @brief Initialize the unified interface with ESP32-specific configuration
     * @return true if at least one interface was successfully initialized
     */
    bool initialize() noexcept {
        if (initialized_) {
            return true;
        }

        // Create SPI interface
        auto spi_interface = std::make_unique<Esp32SPITMC9660CommInterface>(
            config_.spi.host,
            config_.spi.mosi_pin,
            config_.spi.miso_pin,
            config_.spi.sclk_pin,
            config_.spi.cs_pin,
            config_.gpio.rst_pin,
            config_.gpio.drv_en_pin,
            config_.gpio.faultn_pin,
            config_.gpio.wake_pin,
            config_.spi.clock_speed_hz,
            config_.spi.mode
        );

        // Create UART interface
        auto uart_interface = std::make_unique<Esp32UARTTMC9660CommInterface>(
            config_.uart.uart_num,
            config_.uart.tx_pin,
            config_.uart.rx_pin,
            config_.gpio.rst_pin,
            config_.gpio.drv_en_pin,
            config_.gpio.faultn_pin,
            config_.gpio.wake_pin,
            config_.uart.baud_rate,
            config_.uart.address
        );

        // Set the interfaces
        if (!setSPIInterface(std::move(spi_interface))) {
            ESP_LOGE(BUS_TAG, "Failed to set SPI interface");
            return false;
        }

        if (!setUARTInterface(std::move(uart_interface))) {
            ESP_LOGE(BUS_TAG, "Failed to set UART interface");
            return false;
        }

        // Initialize all available interfaces
        if (!initializeAll()) {
            ESP_LOGE(BUS_TAG, "Failed to initialize any communication interface");
            return false;
        }

        initialized_ = true;
        ESP_LOGI(BUS_TAG, "Unified communication interface initialized successfully");
        ESP_LOGI(BUS_TAG, "Status: %s", getStatus().c_str());
        return true;
    }

    /**
     * @brief Deinitialize the unified interface
     * @return true if successful
     */
    bool deinitialize() noexcept {
        if (!initialized_) {
            return true;
        }

        deinitializeAll();
        initialized_ = false;
        ESP_LOGI(BUS_TAG, "Unified communication interface deinitialized");
        return true;
    }

    /**
     * @brief Switch to SPI mode with ESP32-specific logging
     * @return true if successful
     */
    bool switchToSPI() noexcept override {
        bool result = TMC9660UnifiedCommInterface::switchToSPI();
        if (result) {
            ESP_LOGI(BUS_TAG, "Switched to SPI mode");
        } else {
            ESP_LOGE(BUS_TAG, "Failed to switch to SPI mode");
        }
        return result;
    }

    /**
     * @brief Switch to UART mode with ESP32-specific logging
     * @return true if successful
     */
    bool switchToUART() noexcept override {
        bool result = TMC9660UnifiedCommInterface::switchToUART();
        if (result) {
            ESP_LOGI(BUS_TAG, "Switched to UART mode");
        } else {
            ESP_LOGE(BUS_TAG, "Failed to switch to UART mode");
        }
        return result;
    }

    /**
     * @brief Get ESP32-specific status information
     * @return String describing current interface status with ESP32 details
     */
    std::string getDetailedStatus() const noexcept {
        std::string status = getStatus();
        status += "\nESP32 Details: ";
        status += "SPI_Host=" + std::to_string(static_cast<int>(config_.spi.host));
        status += ", UART_Port=" + std::to_string(static_cast<int>(config_.uart.uart_num));
        status += ", Baud=" + std::to_string(config_.uart.baud_rate);
        status += ", Clock=" + std::to_string(config_.spi.clock_speed_hz) + "Hz";
        return status;
    }

    /**
     * @brief Check if the interface is initialized
     * @return true if initialized
     */
    bool isInitialized() const noexcept {
        return initialized_;
    }

    /**
     * @brief Get the current configuration
     * @return Reference to the current configuration
     */
    const Esp32TMC9660BusConfig& getConfig() const noexcept {
        return config_;
    }

    /**
     * @brief Update the configuration (requires reinitialization)
     * @param new_config New configuration
     * @return true if configuration was updated successfully
     */
    bool updateConfig(const Esp32TMC9660BusConfig& new_config) noexcept {
        if (initialized_) {
            ESP_LOGW(BUS_TAG, "Configuration update requires deinitialization first");
            return false;
        }
        
        config_ = new_config;
        ESP_LOGI(BUS_TAG, "Configuration updated");
        return true;
    }

private:
    Esp32TMC9660BusConfig config_;
    bool initialized_;
};

/**
 * @brief Create and initialize ESP32 unified communication interface
 * @param config Bus configuration (optional, uses defaults if not provided)
 * @return Unique pointer to unified interface, or nullptr on failure
 */
inline std::unique_ptr<Esp32TMC9660UnifiedCommInterface> createUnifiedInterface(
    const Esp32TMC9660BusConfig& config = Esp32TMC9660BusConfig{}) noexcept {
    
    auto interface = std::make_unique<Esp32TMC9660UnifiedCommInterface>(config);
    
    if (!interface) {
        ESP_LOGE(BUS_TAG, "Failed to create unified interface");
        return nullptr;
    }
    
    if (!interface->initialize()) {
        ESP_LOGE(BUS_TAG, "Failed to initialize unified interface");
        return nullptr;
    }
    
    return interface;
}

/**
 * @brief Create a TMC9660 driver with unified communication interface
 * @param config Bus configuration (optional, uses defaults if not provided)
 * @param address TMC9660 module address
 * @param bootCfg Bootloader configuration (optional)
 * @return Unique pointer to TMC9660 driver, or nullptr on failure
 */
inline std::unique_ptr<TMC9660> createUnifiedTMC9660Driver(
    const Esp32TMC9660BusConfig& config = Esp32TMC9660BusConfig{},
    uint8_t address = 0,
    const tmc9660::BootloaderConfig* bootCfg = nullptr) noexcept {
    
    auto interface = createUnifiedInterface(config);
    if (!interface) {
        ESP_LOGE(BUS_TAG, "Failed to create unified interface for TMC9660 driver");
        return nullptr;
    }
    
    auto driver = std::make_unique<TMC9660>(*interface, address, bootCfg);
    if (!driver) {
        ESP_LOGE(BUS_TAG, "Failed to create TMC9660 driver");
        return nullptr;
    }
    
    ESP_LOGI(BUS_TAG, "Created TMC9660 driver with unified communication interface");
    return driver;
}

/**
 * @brief Utility class for managing communication mode switching
 * 
 * This class provides convenient methods for switching between communication
 * modes and managing the unified interface lifecycle.
 */
class TMC9660CommModeManager {
public:
    /**
     * @brief Construct communication mode manager
     * @param unified_interface Reference to unified communication interface
     */
    TMC9660CommModeManager(TMC9660UnifiedCommInterface& unified_interface) noexcept
        : interface_(unified_interface), last_mode_(interface_.mode()) {}

    /**
     * @brief Switch to SPI mode if available
     * @return true if successful, false if SPI not available or switch failed
     */
    bool switchToSPI() noexcept {
        if (!interface_.isSPIAvailable()) {
            ESP_LOGW(BUS_TAG, "SPI interface not available");
            return false;
        }
        
        last_mode_ = interface_.mode();
        return interface_.switchToSPI();
    }

    /**
     * @brief Switch to UART mode if available
     * @return true if successful, false if UART not available or switch failed
     */
    bool switchToUART() noexcept {
        if (!interface_.isUARTAvailable()) {
            ESP_LOGW(BUS_TAG, "UART interface not available");
            return false;
        }
        
        last_mode_ = interface_.mode();
        return interface_.switchToUART();
    }

    /**
     * @brief Switch to the other available mode
     * @return true if successful, false if no other mode available
     */
    bool switchToOtherMode() noexcept {
        if (interface_.mode() == CommMode::SPI) {
            return switchToUART();
        } else {
            return switchToSPI();
        }
    }

    /**
     * @brief Switch back to the last used mode
     * @return true if successful, false if last mode not available
     */
    bool switchToLastMode() noexcept {
        if (last_mode_ == CommMode::SPI) {
            return switchToSPI();
        } else {
            return switchToUART();
        }
    }

    /**
     * @brief Get current mode
     * @return Current communication mode
     */
    CommMode getCurrentMode() const noexcept {
        return interface_.mode();
    }

    /**
     * @brief Get last used mode
     * @return Last used communication mode
     */
    CommMode getLastMode() const noexcept {
        return last_mode_;
    }

    /**
     * @brief Check if both modes are available
     * @return true if both SPI and UART are available
     */
    bool hasBothModes() const noexcept {
        return interface_.isSPIAvailable() && interface_.isUARTAvailable();
    }

    /**
     * @brief Get available modes as a string
     * @return String describing available modes
     */
    std::string getAvailableModes() const noexcept {
        std::string modes = "Available modes: ";
        if (interface_.isSPIAvailable()) {
            modes += "SPI";
        }
        if (interface_.isSPIAvailable() && interface_.isUARTAvailable()) {
            modes += ", ";
        }
        if (interface_.isUARTAvailable()) {
            modes += "UART";
        }
        return modes;
    }

private:
    TMC9660UnifiedCommInterface& interface_;
    CommMode last_mode_;
};