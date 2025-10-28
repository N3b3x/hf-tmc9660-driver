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
     * @param rst_pin RST control pin
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
                                 gpio_num_t rst_pin,
                                 gpio_num_t drv_en_pin,
                                 gpio_num_t faultn_pin,
                                 gpio_num_t wake_pin,
                                 uint32_t clock_speed_hz = 10000000,
                                 uint8_t mode = 0) noexcept
        : SPITMC9660CommInterface(true, true, false, false), // RST: HIGH, DRV_EN: HIGH, WAKE: LOW, FAULTN: LOW
          host_(host), mosi_pin_(mosi_pin), miso_pin_(miso_pin), 
          sclk_pin_(sclk_pin), cs_pin_(cs_pin), rst_pin_(rst_pin),
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
     * @brief Perform SPI transfer for TMC9660 TMCL parameter mode communication
     * @param tx Transmit buffer (8 bytes, TMCL format)
     * @param rx Receive buffer (8 bytes, TMCL format) 
     * @return true if successful, false otherwise
     */
    bool spiTransferTMCL(std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept override {
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

    bool spiTransferBootloader(std::array<uint8_t, 5> &tx, std::array<uint8_t, 5> &rx) noexcept override {
        if (!initialized_ || !device_handle_) {
            ESP_LOGE(BUS_TAG, "SPI interface not initialized");
            return false;
        }

        spi_transaction_t trans = {};
        trans.length = 40; // 5 bytes * 8 bits
        trans.tx_buffer = tx.data();
        trans.rx_buffer = rx.data();

        esp_err_t ret = spi_device_transmit(device_handle_, &trans);
        if (ret != ESP_OK) {
            ESP_LOGE(BUS_TAG, "SPI 5-byte transfer failed: %s", esp_err_to_name(ret));
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
     * @brief Set GPIO pin signal state for TMC9660 control pins
     * @param pin The TMC9660 control pin to control
     * @param signal The desired signal state
     * @return true if the GPIO was set successfully
     */
    bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept override {
        gpio_num_t gpio_pin = getGpioPin(pin);
        if (gpio_pin == GPIO_NUM_NC) {
            ESP_LOGE(BUS_TAG, "Invalid GPIO pin for TMC9660 control pin");
            return false;
        }

        // Convert signal state to physical GPIO level using base class helper
        uint32_t gpio_level = signalToGpioLevel(pin, signal) ? 1 : 0;

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
    bool gpioRead(TMC9660CtrlPin pin, GpioSignal &signal) noexcept override {
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
        // Convert physical GPIO level to signal state using base class helper
        signal = gpioLevelToSignal(pin, gpio_level == 1);
        return true;
    }

    /**
     * @brief Debug logging function that routes logs through ESP-IDF logging system
     * @param level Log level (0=Error, 1=Warning, 2=Info, 3=Debug, 4=Verbose)
     * @param tag Log tag for categorization
     * @param format printf-style format string
     * @param ... Variable arguments for format string
     */
    void debugLog(int level, const char* tag, const char* format, va_list args) noexcept override {
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
    }
    
    void delayMs(uint32_t ms) noexcept override {
        vTaskDelay(pdMS_TO_TICKS(ms));
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
        output_config.pin_bit_mask = (1ULL << rst_pin_) | (1ULL << drv_en_pin_) | (1ULL << wake_pin_);
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
        gpio_set_level(rst_pin_, 0);    // RST active HIGH, start with inactive (low)
        gpio_set_level(drv_en_pin_, 0); // DRV_EN active HIGH, start with disabled (low)
        gpio_set_level(wake_pin_, 1);   // WAKE active LOW, start with inactive (high)

        return true;
    }

    /**
     * @brief Map TMC9660 control pins to ESP32 GPIO pins
     * @param pin TMC9660 control pin
     * @return Corresponding ESP32 GPIO pin number
     */
    gpio_num_t getGpioPin(TMC9660CtrlPin pin) const noexcept {
        switch (pin) {
            case TMC9660CtrlPin::RST:
                return rst_pin_;
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
    gpio_num_t rst_pin_;
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
     * @param rst_pin RST control pin
     * @param drv_en_pin DRV_EN control pin
     * @param faultn_pin FAULTN status pin
     * @param wake_pin WAKE control pin
     * @param baud_rate UART baud rate
     * @param address TMC9660 module address
     */
    Esp32UARTTMC9660CommInterface(uart_port_t uart_num,
                                  gpio_num_t tx_pin,
                                  gpio_num_t rx_pin,
                                  gpio_num_t rst_pin,
                                  gpio_num_t drv_en_pin,
                                  gpio_num_t faultn_pin,
                                  gpio_num_t wake_pin,
                                  uint32_t baud_rate = 115200,
                                  uint8_t address = 0) noexcept
        : UARTTMC9660CommInterface(true, true, false, false), // RST: HIGH, DRV_EN: HIGH, WAKE: LOW, FAULTN: LOW
          uart_num_(uart_num), tx_pin_(tx_pin), rx_pin_(rx_pin), 
          rst_pin_(rst_pin), drv_en_pin_(drv_en_pin), faultn_pin_(faultn_pin), wake_pin_(wake_pin),
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
     * @brief Send raw 9-byte UART TMCL datagram for parameter mode communication
     * @param data Array of 9 bytes including sync, fields, and checksum (TMCL format)
     * @return true if transmission succeeded
     */
    bool uartSendTMCL(const std::array<uint8_t, 9> &data) noexcept override {
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
     * @brief Receive raw 9-byte UART TMCL datagram for parameter mode communication
     * @param data Array to store 9 received bytes (TMCL format)
     * @return true if reception succeeded
     */
    bool uartReceiveTMCL(std::array<uint8_t, 9> &data) noexcept override {
        if (!initialized_) {
            ESP_LOGE(BUS_TAG, "UART interface not initialized");
            return false;
        }

        int bytes_read = uart_read_bytes(uart_num_, data.data(), data.size(), pdMS_TO_TICKS(10));
        if (bytes_read != static_cast<int>(data.size())) {
            ESP_LOGE(BUS_TAG, "UART read failed: expected %zu, read %d", data.size(), bytes_read);
            return false;
        }

        return true;
    }

    /**
     * @brief Transfer TMCL frame over UART for parameter mode communication
     * @param tx TMCL command frame to transmit
     * @param reply TMCL reply frame to receive
     * @param address TMC9660 module address
     * @param firstReply Optional pointer to capture first reply (ignored for UART)
     * @param secondCommand Optional second command (ignored for UART)
     * @return true if transfer succeeded
     */
    bool transferTMCL(const TMCLFrame &tx, TMCLReply &reply, uint8_t address,
                     TMCLReply *firstReply, const TMCLFrame *secondCommand) noexcept override {
        // UART doesn't use the two-transaction pattern, so firstReply and secondCommand are ignored
        (void)firstReply;      // Suppress unused parameter warning
        (void)secondCommand;   // Suppress unused parameter warning
        if (!initialized_) {
            ESP_LOGE(BUS_TAG, "UART interface not initialized");
            return false;
        }

        // Clear RX buffer before sending
        uart_flush_input(uart_num_);

        // ✅ FIX: Use TMCLFrame::toUart() for correct frame encoding
        // Previous manual packing had multiple bugs:
        // - Wrong sync bit position (should be bit 0, not 0x55 in byte 0)
        // - Only wrote lower 8 bits of 16-bit type field
        // - Array indexing off by one (overwrote checksum with value LSB)
        std::array<uint8_t, 9> uart_frame;
        tx.toUart(address, uart_frame);  // Correctly encodes all fields + checksum

        // Send 9-byte UART frame
        int bytes_written = uart_write_bytes(uart_num_, uart_frame.data(), uart_frame.size());
        if (bytes_written != static_cast<int>(uart_frame.size())) {
            ESP_LOGE(BUS_TAG, "UART write failed: expected %zu, wrote %d", uart_frame.size(), bytes_written);
            return false;
        }

        // Log transmitted bytes
        logDebug(2, "UART_TMCL", "[UART TX] %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                 uart_frame[0], uart_frame[1], uart_frame[2], uart_frame[3],
                 uart_frame[4], uart_frame[5], uart_frame[6], uart_frame[7], uart_frame[8]);

        // Wait for transmission to complete
        uart_wait_tx_done(uart_num_, portMAX_DELAY);

        // Receive 9-byte UART reply
        std::array<uint8_t, 9> uart_reply;
        int bytes_read = uart_read_bytes(uart_num_, uart_reply.data(), uart_reply.size(), pdMS_TO_TICKS(10));
        if (bytes_read != static_cast<int>(uart_reply.size())) {
            ESP_LOGE(BUS_TAG, "UART read failed: expected %zu, read %d", uart_reply.size(), bytes_read);
            // Log partial data if any was received
            if (bytes_read > 0) {
                logDebug(2, "UART_TMCL", "[UART RX] (partial %d bytes) %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                         bytes_read, 
                         bytes_read > 0 ? uart_reply[0] : 0, bytes_read > 1 ? uart_reply[1] : 0,
                         bytes_read > 2 ? uart_reply[2] : 0, bytes_read > 3 ? uart_reply[3] : 0,
                         bytes_read > 4 ? uart_reply[4] : 0, bytes_read > 5 ? uart_reply[5] : 0,
                         bytes_read > 6 ? uart_reply[6] : 0, bytes_read > 7 ? uart_reply[7] : 0,
                         bytes_read > 8 ? uart_reply[8] : 0);
            }
            return false;
        }

        // Log received bytes
        logDebug(2, "UART_TMCL", "[UART RX] %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                 uart_reply[0], uart_reply[1], uart_reply[2], uart_reply[3],
                 uart_reply[4], uart_reply[5], uart_reply[6], uart_reply[7], uart_reply[8]);

        // Use TMCLReply::fromUart() for correct decoding with command context
        // This handles checksum verification and proper field extraction
        // Pass command context (opcode, type) for handling special reply formats like GetVersion string
        if (!TMCLReply::fromUart(uart_reply, address, reply, tx.opcode, tx.type)) {
            ESP_LOGE(BUS_TAG, "Failed to parse UART reply (checksum or address mismatch)");
            return false;
        }

        return true;
    }

    /**
     * @brief Transfer 8-byte UART bootloader datagram (send and receive)
     * @param tx Buffer containing 8 bytes to transmit (bootloader format)
     * @param rx Buffer to receive 8 bytes from device (bootloader format)
     * @return true if transfer succeeded
     */
    bool uartTransferBootloader(const std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept override {
        if (!initialized_) {
            ESP_LOGE(BUS_TAG, "UART interface not initialized");
            return false;
        }

        // Clear RX buffer before sending
        uart_flush_input(uart_num_);

        // Send 8-byte bootloader command
        int bytes_written = uart_write_bytes(uart_num_, tx.data(), tx.size());
        if (bytes_written != static_cast<int>(tx.size())) {
            ESP_LOGE(BUS_TAG, "UART bootloader write failed: expected %zu, wrote %d", tx.size(), bytes_written);
            return false;
        }

#ifndef TMC9660_DISABLE_COMM_DEBUG
        // Log transmitted bytes
        logDebug(2, "UART_BL", "[UART BL TX] %02X %02X %02X %02X %02X %02X %02X %02X",
                 tx[0], tx[1], tx[2], tx[3], tx[4], tx[5], tx[6], tx[7]);
#endif

        // Wait for transmission to complete
        uart_wait_tx_done(uart_num_, portMAX_DELAY);

        // Receive 8-byte bootloader reply
        int bytes_read = uart_read_bytes(uart_num_, rx.data(), rx.size(), pdMS_TO_TICKS(10));
        if (bytes_read != static_cast<int>(rx.size())) {
            ESP_LOGE(BUS_TAG, "UART bootloader read failed: expected %zu, read %d", rx.size(), bytes_read);
#ifndef TMC9660_DISABLE_COMM_DEBUG
            // Log partial data if any was received
            if (bytes_read > 0) {
                logDebug(2, "UART_BL", "[UART BL RX] (partial %d bytes) %02X %02X %02X %02X %02X %02X %02X %02X",
                         bytes_read,
                         bytes_read > 0 ? rx[0] : 0, bytes_read > 1 ? rx[1] : 0,
                         bytes_read > 2 ? rx[2] : 0, bytes_read > 3 ? rx[3] : 0,
                         bytes_read > 4 ? rx[4] : 0, bytes_read > 5 ? rx[5] : 0,
                         bytes_read > 6 ? rx[6] : 0, bytes_read > 7 ? rx[7] : 0);
            }
#endif
            return false;
        }

#ifndef TMC9660_DISABLE_COMM_DEBUG
        // Log received bytes
        logDebug(2, "UART_BL", "[UART BL RX] %02X %02X %02X %02X %02X %02X %02X %02X",
                 rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7]);
#endif

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
     * @brief Set GPIO pin signal state for TMC9660 control pins
     * @param pin The TMC9660 control pin to control
     * @param signal The desired signal state
     * @return true if the GPIO was set successfully
     */
     bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept override {
        gpio_num_t gpio_pin = getGpioPin(pin);
        if (gpio_pin == GPIO_NUM_NC) {
            ESP_LOGE(BUS_TAG, "Invalid GPIO pin for TMC9660 control pin");
            return false;
        }

        // Convert signal state to physical GPIO level using base class helper
        uint32_t gpio_level = signalToGpioLevel(pin, signal) ? 1 : 0;

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
    bool gpioRead(TMC9660CtrlPin pin, GpioSignal &signal) noexcept override {
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
        // Convert physical GPIO level to signal state using base class helper
        signal = gpioLevelToSignal(pin, gpio_level == 1);
        return true;
    }

    /**
     * @brief Debug logging function that routes logs through ESP-IDF logging system
     * @param level Log level (0=Error, 1=Warning, 2=Info, 3=Debug, 4=Verbose)
     * @param tag Log tag for categorization
     * @param format printf-style format string
     * @param ... Variable arguments for format string
     */
    void debugLog(int level, const char* tag, const char* format, va_list args) noexcept override {
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
    }
    
    void delayMs(uint32_t ms) noexcept override {
        vTaskDelay(pdMS_TO_TICKS(ms));
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
        output_config.pin_bit_mask = (1ULL << rst_pin_) | (1ULL << drv_en_pin_) | (1ULL << wake_pin_);
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
        gpio_set_level(rst_pin_, 0);    // RST active HIGH, start with inactive (low)
        gpio_set_level(drv_en_pin_, 0); // DRV_EN active HIGH, start with disabled (low)
        gpio_set_level(wake_pin_, 1);   // WAKE active LOW, start with inactive (high)

        return true;
    }

    /**
     * @brief Map TMC9660 control pins to ESP32 GPIO pins
     * @param pin TMC9660 control pin
     * @return Corresponding ESP32 GPIO pin number
     */
    gpio_num_t getGpioPin(TMC9660CtrlPin pin) const noexcept {
        switch (pin) {
            case TMC9660CtrlPin::RST:
                return rst_pin_;
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
    gpio_num_t rst_pin_;
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
        uint32_t clock_speed_hz = 1000000; // 1 MHz
        uint8_t mode = 3;  // ⚠️ CRITICAL: TMC9660 requires SPI MODE 3 (CPOL=1, CPHA=1)
    } spi;

    // UART Configuration  
    struct {
        uart_port_t uart_num = UART_NUM_1;
        gpio_num_t tx_pin = GPIO_NUM_5;
        gpio_num_t rx_pin = GPIO_NUM_4;
        uint32_t baud_rate = 115200;
        uint8_t address = 0;
    } uart;

    // GPIO Configuration for TMC9660 control pins
    struct {
        gpio_num_t rst_pin = GPIO_NUM_22;     // RST control pin (active HIGH)
        gpio_num_t drv_en_pin = GPIO_NUM_20;  // DRV_EN control pin (active HIGH)
        gpio_num_t faultn_pin = GPIO_NUM_19;  // FAULTN status pin (active LOW, open drain)
        gpio_num_t wake_pin = GPIO_NUM_21;    // WAKE control pin (active LOW)
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
        config.gpio.rst_pin,
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
        config.gpio.rst_pin,
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
