/**
 * @file Esp32TMC9660Bus.hpp
 * @brief ESP32-specific communication interfaces for TMC9660 using SPI and UART
 *
 * This file provides ESP32-specific implementations of the TMC9660CommInterface
 * using ESP-IDF SPI and UART drivers. It serves as a common bus interface for
 * all comprehensive test applications.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#pragma once

#include "../../../inc/TMC9660CommInterface.hpp"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <array>
#include <cstdarg>
#include <cstring>
#include <cstdint>
#include <memory>

static const char* BUS_TAG = "TMC9660_Bus";

/**
 * @brief ESP32 SPI implementation of TMC9660CommInterface
 * 
 * This class provides SPI communication for the TMC9660 using ESP-IDF SPI driver.
 * It handles the 8-byte SPI transfers required by the TMC9660 parameter mode.
 */
class Esp32SPITMC9660CommInterface : public SPITMC9660CommInterface {
public:
    /**
     * @brief Construct ESP32 SPI communication interface
     * @param host SPI host device (e.g., SPI2_HOST)
     * @param mosi_pin MOSI GPIO pin
     * @param miso_pin MISO GPIO pin  
     * @param sclk_pin SCLK GPIO pin
     * @param cs_pin CS GPIO pin
     * @param rstn_pin RSTN control pin
     * @param drv_en_pin DRV_EN control pin
     * @param faultn_pin FAULTN status pin
     * @param wake_pin WAKE control pin
     * @param clock_speed_hz SPI clock speed in Hz
     * @param mode SPI mode (0-3)
     */
    Esp32SPITMC9660CommInterface(spi_host_device_t host, 
                                 gpio_num_t mosi_pin, 
                                 gpio_num_t miso_pin,
                                 gpio_num_t sclk_pin, 
                                 gpio_num_t cs_pin,
                                 gpio_num_t rstn_pin,
                                 gpio_num_t drv_en_pin,
                                 gpio_num_t faultn_pin,
                                 gpio_num_t wake_pin,
                                 uint32_t clock_speed_hz = 10000000,
                                 uint8_t mode = 0) noexcept
        : host_(host), mosi_pin_(mosi_pin), miso_pin_(miso_pin), 
          sclk_pin_(sclk_pin), cs_pin_(cs_pin), rstn_pin_(rstn_pin),
          drv_en_pin_(drv_en_pin), faultn_pin_(faultn_pin), wake_pin_(wake_pin),
          clock_speed_hz_(clock_speed_hz), mode_(mode), device_handle_(nullptr), 
          initialized_(false) {
    }

    /**
     * @brief Destructor - cleans up SPI resources
     */
    ~Esp32SPITMC9660CommInterface() noexcept override {
        deinitialize();
    }

    /**
     * @brief Initialize the SPI interface
     * @return true if successful, false otherwise
     */
    bool initialize() noexcept {
        if (initialized_) {
            return true;
        }

        // Configure GPIO pins
        if (!configureGpioPins()) {
            ESP_LOGE(BUS_TAG, "Failed to configure GPIO pins");
            return false;
        }

        // Configure SPI bus
        spi_bus_config_t bus_config = {};
        bus_config.mosi_io_num = mosi_pin_;
        bus_config.miso_io_num = miso_pin_;
        bus_config.sclk_io_num = sclk_pin_;
        bus_config.quadwp_io_num = -1;
        bus_config.quadhd_io_num = -1;
        bus_config.max_transfer_sz = 8; // TMC9660 uses 8-byte transfers
        bus_config.flags = SPICOMMON_BUSFLAG_MASTER;

        esp_err_t ret = spi_bus_initialize(host_, &bus_config, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK) {
            ESP_LOGE(BUS_TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
            return false;
        }

        // Configure SPI device
        spi_device_interface_config_t dev_config = {};
        dev_config.clock_speed_hz = clock_speed_hz_;
        dev_config.mode = mode_;
        dev_config.spics_io_num = cs_pin_;
        dev_config.queue_size = 7;
        dev_config.command_bits = 0;
        dev_config.address_bits = 0;
        dev_config.dummy_bits = 0;
        dev_config.duty_cycle_pos = 128;
        dev_config.cs_ena_pretrans = 2;
        dev_config.cs_ena_posttrans = 2;
        dev_config.flags = 0;
        dev_config.input_delay_ns = 0;
        dev_config.pre_cb = nullptr;
        dev_config.post_cb = nullptr;

        ret = spi_bus_add_device(host_, &dev_config, &device_handle_);
        if (ret != ESP_OK) {
            ESP_LOGE(BUS_TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
            spi_bus_free(host_);
            return false;
        }

        initialized_ = true;
        ESP_LOGI(BUS_TAG, "SPI interface initialized successfully");
        return true;
    }

    /**
     * @brief Deinitialize the SPI interface
     * @return true if successful, false otherwise
     */
    bool deinitialize() noexcept {
        if (!initialized_) {
            return true;
        }

        if (device_handle_) {
            spi_bus_remove_device(device_handle_);
            device_handle_ = nullptr;
        }

        spi_bus_free(host_);
        initialized_ = false;
        ESP_LOGI(BUS_TAG, "SPI interface deinitialized");
        return true;
    }

    /**
     * @brief Perform SPI transfer for TMC9660
     * @param tx Transmit buffer (8 bytes)
     * @param rx Receive buffer (8 bytes) 
     * @return true if successful, false otherwise
     */
    bool spiTransfer(std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept override {
        if (!initialized_ || !device_handle_) {
            ESP_LOGE(BUS_TAG, "SPI interface not initialized");
            return false;
        }

        spi_transaction_t trans = {};
        trans.length = 64; // 8 bytes * 8 bits
        trans.tx_buffer = tx.data();
        trans.rx_buffer = rx.data();

        esp_err_t ret = spi_device_transmit(device_handle_, &trans);
        if (ret != ESP_OK) {
            ESP_LOGE(BUS_TAG, "SPI transfer failed: %s", esp_err_to_name(ret));
            return false;
        }

        return true;
    }

    /**
     * @brief Get communication mode
     * @return CommMode::SPI
     */
    CommMode mode() const noexcept override {
        return CommMode::SPI;
    }

    /**
     * @brief Set GPIO pin level for TMC9660 control pins
     * @param pin The TMC9660 control pin to control
     * @param level The desired GPIO level
     * @return true if the GPIO was set successfully
     */
    bool gpioSet(TMC9660CtrlPin pin, GpioLevel level) noexcept override {
        gpio_num_t gpio_pin = getGpioPin(pin);
        if (gpio_pin == GPIO_NUM_NC) {
            ESP_LOGE(BUS_TAG, "Invalid GPIO pin for TMC9660 control pin");
            return false;
        }

        // Handle pins with inverting transistor logic
        uint32_t gpio_level = static_cast<uint32_t>(level);
        if (pin == TMC9660CtrlPin::WAKE || pin == TMC9660CtrlPin::RSTN) {
            // WAKE and RSTN pins are driven through inverting transistors
            // To make pin HIGH (active), we need to drive GPIO LOW
            // To make pin LOW (inactive), we need to drive GPIO HIGH
            gpio_level = (level == GpioLevel::HIGH) ? 0 : 1;
        }

        esp_err_t ret = gpio_set_level(gpio_pin, gpio_level);
        if (ret != ESP_OK) {
            ESP_LOGE(BUS_TAG, "Failed to set GPIO level: %s", esp_err_to_name(ret));
            return false;
        }
        return true;
    }

    /**
     * @brief Read GPIO pin level for TMC9660 status pins
     * @param pin The TMC9660 control pin to read
     * @param level Reference to store the current GPIO level
     * @return true if the GPIO was read successfully
     */
    bool gpioRead(TMC9660CtrlPin pin, GpioLevel &level) noexcept override {
        gpio_num_t gpio_pin = getGpioPin(pin);
        if (gpio_pin == GPIO_NUM_NC) {
            ESP_LOGE(BUS_TAG, "Invalid GPIO pin for TMC9660 control pin");
            return false;
        }

        int gpio_level = gpio_get_level(gpio_pin);
        if (gpio_level < 0) {
            ESP_LOGE(BUS_TAG, "Failed to read GPIO level");
            return false;
        }
        level = static_cast<GpioLevel>(gpio_level);
        return true;
    }

    /**
     * @brief Debug logging function that routes logs through ESP-IDF logging system
     * @param level Log level (0=Error, 1=Warning, 2=Info, 3=Debug, 4=Verbose)
     * @param tag Log tag for categorization
     * @param format printf-style format string
     * @param ... Variable arguments for format string
     */
    void debugLog(int level, const char* tag, const char* format, ...) noexcept override {
        va_list args;
        va_start(args, format);
        
        // Route to appropriate ESP-IDF log level
        switch (level) {
            case 0: // Error
                esp_log_writev(ESP_LOG_ERROR, tag, format, args);
                break;
            case 1: // Warning
                esp_log_writev(ESP_LOG_WARN, tag, format, args);
                break;
            case 2: // Info
                esp_log_writev(ESP_LOG_INFO, tag, format, args);
                break;
            case 3: // Debug
                esp_log_writev(ESP_LOG_DEBUG, tag, format, args);
                break;
            case 4: // Verbose
                esp_log_writev(ESP_LOG_VERBOSE, tag, format, args);
                break;
            default:
                esp_log_writev(ESP_LOG_INFO, tag, format, args);
                break;
        }
        
        va_end(args);
    }

private:
    /**
     * @brief Configure GPIO pins for TMC9660 control and status
     * @return true if successful, false otherwise
     */
    bool configureGpioPins() noexcept {
        // Configure control pins as outputs
        gpio_config_t output_config = {};
        output_config.intr_type = GPIO_INTR_DISABLE;
        output_config.mode = GPIO_MODE_OUTPUT;
        output_config.pin_bit_mask = (1ULL << rstn_pin_) | (1ULL << drv_en_pin_) | (1ULL << wake_pin_);
        output_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        output_config.pull_up_en = GPIO_PULLUP_DISABLE;

        esp_err_t ret = gpio_config(&output_config);
        if (ret != ESP_OK) {
            ESP_LOGE(BUS_TAG, "Failed to configure output GPIO pins: %s", esp_err_to_name(ret));
            return false;
        }

        // Configure status pin as input
        gpio_config_t input_config = {};
        input_config.intr_type = GPIO_INTR_DISABLE;
        input_config.mode = GPIO_MODE_INPUT;
        input_config.pin_bit_mask = (1ULL << faultn_pin_);
        input_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        input_config.pull_up_en = GPIO_PULLUP_ENABLE; // Enable pull-up for open-drain FAULTN

        ret = gpio_config(&input_config);
        if (ret != ESP_OK) {
            ESP_LOGE(BUS_TAG, "Failed to configure input GPIO pin: %s", esp_err_to_name(ret));
            return false;
        }

        // Set initial states
        gpio_set_level(rstn_pin_, 0);  // RSTN active low, start with inactive (GPIO low = RSTN high due to inverting transistor)
        gpio_set_level(drv_en_pin_, 0); // DRV_EN active high, start with disabled (low)
        gpio_set_level(wake_pin_, 1);   // WAKE active high, start with inactive (GPIO high = WAKE low due to inverting transistor)

        return true;
    }

    /**
     * @brief Map TMC9660 control pins to ESP32 GPIO pins
     * @param pin TMC9660 control pin
     * @return Corresponding ESP32 GPIO pin number
     */
    gpio_num_t getGpioPin(TMC9660CtrlPin pin) const noexcept {
        switch (pin) {
            case TMC9660CtrlPin::RSTN:
                return rstn_pin_;
            case TMC9660CtrlPin::DRV_EN:
                return drv_en_pin_;
            case TMC9660CtrlPin::FAULTN:
                return faultn_pin_;
            case TMC9660CtrlPin::WAKE:
                return wake_pin_;
            default:
                return GPIO_NUM_NC;
        }
    }

    spi_host_device_t host_;
    gpio_num_t mosi_pin_;
    gpio_num_t miso_pin_;
    gpio_num_t sclk_pin_;
    gpio_num_t cs_pin_;
    gpio_num_t rstn_pin_;
    gpio_num_t drv_en_pin_;
    gpio_num_t faultn_pin_;
    gpio_num_t wake_pin_;
    uint32_t clock_speed_hz_;
    uint8_t mode_;
    spi_device_handle_t device_handle_;
    bool initialized_;
};

/**
 * @brief ESP32 UART implementation of TMC9660CommInterface
 * 
 * This class provides UART communication for the TMC9660 using ESP-IDF UART driver.
 * It handles the TMCL protocol over UART as specified in the TMC9660 documentation.
 */
class Esp32UARTTMC9660CommInterface : public UARTTMC9660CommInterface {
public:
    /**
     * @brief Construct ESP32 UART communication interface
     * @param uart_num UART port number (e.g., UART_NUM_1)
     * @param tx_pin TX GPIO pin
     * @param rx_pin RX GPIO pin
     * @param rstn_pin RSTN control pin
     * @param drv_en_pin DRV_EN control pin
     * @param faultn_pin FAULTN status pin
     * @param wake_pin WAKE control pin
     * @param baud_rate UART baud rate
     * @param address TMC9660 module address
     */
    Esp32UARTTMC9660CommInterface(uart_port_t uart_num,
                                  gpio_num_t tx_pin,
                                  gpio_num_t rx_pin,
                                  gpio_num_t rstn_pin,
                                  gpio_num_t drv_en_pin,
                                  gpio_num_t faultn_pin,
                                  gpio_num_t wake_pin,
                                  uint32_t baud_rate = 115200,
                                  uint8_t address = 0) noexcept
        : uart_num_(uart_num), tx_pin_(tx_pin), rx_pin_(rx_pin), 
          rstn_pin_(rstn_pin), drv_en_pin_(drv_en_pin), faultn_pin_(faultn_pin), wake_pin_(wake_pin),
          baud_rate_(baud_rate), address_(address), initialized_(false) {
    }

    /**
     * @brief Destructor - cleans up UART resources
     */
    ~Esp32UARTTMC9660CommInterface() noexcept override {
        deinitialize();
    }

    /**
     * @brief Initialize the UART interface
     * @return true if successful, false otherwise
     */
    bool initialize() noexcept {
        if (initialized_) {
            return true;
        }

        // Configure GPIO pins
        if (!configureGpioPins()) {
            ESP_LOGE(BUS_TAG, "Failed to configure GPIO pins");
            return false;
        }

        // Configure UART
        uart_config_t uart_config = {};
        uart_config.baud_rate = baud_rate_;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config.source_clk = UART_SCLK_DEFAULT;

        esp_err_t ret = uart_driver_install(uart_num_, 1024, 1024, 0, nullptr, 0);
        if (ret != ESP_OK) {
            ESP_LOGE(BUS_TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
            return false;
        }

        ret = uart_param_config(uart_num_, &uart_config);
        if (ret != ESP_OK) {
            ESP_LOGE(BUS_TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
            uart_driver_delete(uart_num_);
            return false;
        }

        ret = uart_set_pin(uart_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (ret != ESP_OK) {
            ESP_LOGE(BUS_TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
            uart_driver_delete(uart_num_);
            return false;
        }

        initialized_ = true;
        ESP_LOGI(BUS_TAG, "UART interface initialized successfully");
        return true;
    }

    /**
     * @brief Deinitialize the UART interface
     * @return true if successful, false otherwise
     */
    bool deinitialize() noexcept {
        if (!initialized_) {
            return true;
        }

        uart_driver_delete(uart_num_);
        initialized_ = false;
        ESP_LOGI(BUS_TAG, "UART interface deinitialized");
        return true;
    }

    /**
     * @brief Send raw 9-byte UART TMCL datagram
     * @param data Array of 9 bytes including sync, fields, and checksum
     * @return true if transmission succeeded
     */
    bool sendUartDatagram(const std::array<uint8_t, 9> &data) noexcept override {
        if (!initialized_) {
            ESP_LOGE(BUS_TAG, "UART interface not initialized");
            return false;
        }

        int bytes_written = uart_write_bytes(uart_num_, data.data(), data.size());
        if (bytes_written != static_cast<int>(data.size())) {
            ESP_LOGE(BUS_TAG, "UART write failed: expected %zu, wrote %d", data.size(), bytes_written);
            return false;
        }

        // Wait for transmission to complete
        uart_wait_tx_done(uart_num_, portMAX_DELAY);
        return true;
    }

    /**
     * @brief Receive raw 9-byte UART TMCL datagram
     * @param data Array to store 9 received bytes
     * @return true if reception succeeded
     */
    bool receiveUartDatagram(std::array<uint8_t, 9> &data) noexcept override {
        if (!initialized_) {
            ESP_LOGE(BUS_TAG, "UART interface not initialized");
            return false;
        }

        int bytes_read = uart_read_bytes(uart_num_, data.data(), data.size(), pdMS_TO_TICKS(1000));
        if (bytes_read != static_cast<int>(data.size())) {
            ESP_LOGE(BUS_TAG, "UART read failed: expected %zu, read %d", data.size(), bytes_read);
            return false;
        }

        return true;
    }

    /**
     * @brief Get communication mode
     * @return CommMode::UART
     */
    CommMode mode() const noexcept override {
        return CommMode::UART;
    }

    /**
     * @brief Set GPIO pin level for TMC9660 control pins
     * @param pin The TMC9660 control pin to control
     * @param level The desired GPIO level
     * @return true if the GPIO was set successfully
     */
    bool gpioSet(TMC9660CtrlPin pin, GpioLevel level) noexcept override {
        gpio_num_t gpio_pin = getGpioPin(pin);
        if (gpio_pin == GPIO_NUM_NC) {
            ESP_LOGE(BUS_TAG, "Invalid GPIO pin for TMC9660 control pin");
            return false;
        }

        // Handle pins with inverting transistor logic
        uint32_t gpio_level = static_cast<uint32_t>(level);
        if (pin == TMC9660CtrlPin::WAKE || pin == TMC9660CtrlPin::RSTN) {
            // WAKE and RSTN pins are driven through inverting transistors
            // To make pin HIGH (active), we need to drive GPIO LOW
            // To make pin LOW (inactive), we need to drive GPIO HIGH
            gpio_level = (level == GpioLevel::HIGH) ? 0 : 1;
        }

        esp_err_t ret = gpio_set_level(gpio_pin, gpio_level);
        if (ret != ESP_OK) {
            ESP_LOGE(BUS_TAG, "Failed to set GPIO level: %s", esp_err_to_name(ret));
            return false;
        }
        return true;
    }

    /**
     * @brief Read GPIO pin level for TMC9660 status pins
     * @param pin The TMC9660 control pin to read
     * @param level Reference to store the current GPIO level
     * @return true if the GPIO was read successfully
     */
    bool gpioRead(TMC9660CtrlPin pin, GpioLevel &level) noexcept override {
        gpio_num_t gpio_pin = getGpioPin(pin);
        if (gpio_pin == GPIO_NUM_NC) {
            ESP_LOGE(BUS_TAG, "Invalid GPIO pin for TMC9660 control pin");
            return false;
        }

        int gpio_level = gpio_get_level(gpio_pin);
        if (gpio_level < 0) {
            ESP_LOGE(BUS_TAG, "Failed to read GPIO level");
            return false;
        }
        level = static_cast<GpioLevel>(gpio_level);
        return true;
    }

    /**
     * @brief Debug logging function that routes logs through ESP-IDF logging system
     * @param level Log level (0=Error, 1=Warning, 2=Info, 3=Debug, 4=Verbose)
     * @param tag Log tag for categorization
     * @param format printf-style format string
     * @param ... Variable arguments for format string
     */
    void debugLog(int level, const char* tag, const char* format, ...) noexcept override {
        va_list args;
        va_start(args, format);
        
        // Route to appropriate ESP-IDF log level
        switch (level) {
            case 0: // Error
                esp_log_writev(ESP_LOG_ERROR, tag, format, args);
                break;
            case 1: // Warning
                esp_log_writev(ESP_LOG_WARN, tag, format, args);
                break;
            case 2: // Info
                esp_log_writev(ESP_LOG_INFO, tag, format, args);
                break;
            case 3: // Debug
                esp_log_writev(ESP_LOG_DEBUG, tag, format, args);
                break;
            case 4: // Verbose
                esp_log_writev(ESP_LOG_VERBOSE, tag, format, args);
                break;
            default:
                esp_log_writev(ESP_LOG_INFO, tag, format, args);
                break;
        }
        
        va_end(args);
    }

private:
    /**
     * @brief Configure GPIO pins for TMC9660 control and status
     * @return true if successful, false otherwise
     */
    bool configureGpioPins() noexcept {
        // Configure control pins as outputs
        gpio_config_t output_config = {};
        output_config.intr_type = GPIO_INTR_DISABLE;
        output_config.mode = GPIO_MODE_OUTPUT;
        output_config.pin_bit_mask = (1ULL << rstn_pin_) | (1ULL << drv_en_pin_) | (1ULL << wake_pin_);
        output_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        output_config.pull_up_en = GPIO_PULLUP_DISABLE;

        esp_err_t ret = gpio_config(&output_config);
        if (ret != ESP_OK) {
            ESP_LOGE(BUS_TAG, "Failed to configure output GPIO pins: %s", esp_err_to_name(ret));
            return false;
        }

        // Configure status pin as input
        gpio_config_t input_config = {};
        input_config.intr_type = GPIO_INTR_DISABLE;
        input_config.mode = GPIO_MODE_INPUT;
        input_config.pin_bit_mask = (1ULL << faultn_pin_);
        input_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        input_config.pull_up_en = GPIO_PULLUP_ENABLE; // Enable pull-up for open-drain FAULTN

        ret = gpio_config(&input_config);
        if (ret != ESP_OK) {
            ESP_LOGE(BUS_TAG, "Failed to configure input GPIO pin: %s", esp_err_to_name(ret));
            return false;
        }

        // Set initial states
        gpio_set_level(rstn_pin_, 0);  // RSTN active low, start with inactive (GPIO low = RSTN high due to inverting transistor)
        gpio_set_level(drv_en_pin_, 0); // DRV_EN active high, start with disabled (low)
        gpio_set_level(wake_pin_, 1);   // WAKE active high, start with inactive (GPIO high = WAKE low due to inverting transistor)

        return true;
    }

    /**
     * @brief Map TMC9660 control pins to ESP32 GPIO pins
     * @param pin TMC9660 control pin
     * @return Corresponding ESP32 GPIO pin number
     */
    gpio_num_t getGpioPin(TMC9660CtrlPin pin) const noexcept {
        switch (pin) {
            case TMC9660CtrlPin::RSTN:
                return rstn_pin_;
            case TMC9660CtrlPin::DRV_EN:
                return drv_en_pin_;
            case TMC9660CtrlPin::FAULTN:
                return faultn_pin_;
            case TMC9660CtrlPin::WAKE:
                return wake_pin_;
            default:
                return GPIO_NUM_NC;
        }
    }

    uart_port_t uart_num_;
    gpio_num_t tx_pin_;
    gpio_num_t rx_pin_;
    gpio_num_t rstn_pin_;
    gpio_num_t drv_en_pin_;
    gpio_num_t faultn_pin_;
    gpio_num_t wake_pin_;
    uint32_t baud_rate_;
    uint8_t address_;
    bool initialized_;
};

/**
 * @brief Common bus configuration for ESP32 TMC9660 tests
 * 
 * This structure contains the standard pin assignments and configuration
 * used across all comprehensive test applications.
 */
struct Esp32TMC9660BusConfig {
    // SPI Configuration
    struct {
        spi_host_device_t host = SPI2_HOST;
        gpio_num_t mosi_pin = GPIO_NUM_7;
        gpio_num_t miso_pin = GPIO_NUM_2;
        gpio_num_t sclk_pin = GPIO_NUM_6;
        gpio_num_t cs_pin = GPIO_NUM_18;
        uint32_t clock_speed_hz = 5000000; // 5 MHz
        uint8_t mode = 0;
    } spi;

    // UART Configuration  
    struct {
        uart_port_t uart_num = UART_NUM_1;
        gpio_num_t tx_pin = GPIO_NUM_4;
        gpio_num_t rx_pin = GPIO_NUM_5;
        uint32_t baud_rate = 115200;
        uint8_t address = 0;
    } uart;

    // GPIO Configuration for TMC9660 control pins
    struct {
        gpio_num_t rstn_pin = GPIO_NUM_22;    // RSTN control pin (active low, driven through inverting transistor)
        gpio_num_t drv_en_pin = GPIO_NUM_20;  // DRV_EN control pin (active high)
        gpio_num_t faultn_pin = GPIO_NUM_19;  // FAULTN status pin (open drain)
        gpio_num_t wake_pin = GPIO_NUM_21;    // WAKE control pin (active high, driven through inverting transistor)
    } gpio;
};

/**
 * @brief Create and initialize SPI communication interface
 * @param config Bus configuration
 * @return Unique pointer to SPI interface, or nullptr on failure
 */
inline std::unique_ptr<Esp32SPITMC9660CommInterface> createSPIInterface(
    const Esp32TMC9660BusConfig& config = Esp32TMC9660BusConfig{}) noexcept {
    
    auto interface = std::make_unique<Esp32SPITMC9660CommInterface>(
        config.spi.host,
        config.spi.mosi_pin,
        config.spi.miso_pin,
        config.spi.sclk_pin,
        config.spi.cs_pin,
        config.gpio.rstn_pin,
        config.gpio.drv_en_pin,
        config.gpio.faultn_pin,
        config.gpio.wake_pin,
        config.spi.clock_speed_hz,
        config.spi.mode
    );

    if (!interface->initialize()) {
        ESP_LOGE(BUS_TAG, "Failed to initialize SPI interface");
        return nullptr;
    }

    return interface;
}

/**
 * @brief Create and initialize UART communication interface
 * @param config Bus configuration
 * @return Unique pointer to UART interface, or nullptr on failure
 */
inline std::unique_ptr<Esp32UARTTMC9660CommInterface> createUARTInterface(
    const Esp32TMC9660BusConfig& config = Esp32TMC9660BusConfig{}) noexcept {
    
    auto interface = std::make_unique<Esp32UARTTMC9660CommInterface>(
        config.uart.uart_num,
        config.uart.tx_pin,
        config.uart.rx_pin,
        config.gpio.rstn_pin,
        config.gpio.drv_en_pin,
        config.gpio.faultn_pin,
        config.gpio.wake_pin,
        config.uart.baud_rate,
        config.uart.address
    );

    if (!interface->initialize()) {
        ESP_LOGE(BUS_TAG, "Failed to initialize UART interface");
        return nullptr;
    }

    return interface;
}
