/**
 * @file TMC9660UnifiedCommInterface.hpp
 * @brief Unified communication interface for TMC9660 supporting real-time SPI/UART switching
 *
 * This file provides a unified communication interface that can dynamically switch
 * between SPI and UART communication modes in real-time. This is useful for:
 * - Testing both communication interfaces with the same hardware setup
 * - Fallback scenarios where one interface fails
 * - Development and debugging workflows
 * - Applications requiring different communication modes for different operations
 *
 * The interface maintains the same API as the base TMC9660CommInterface but
 * allows switching between underlying SPI and UART implementations at runtime.
 *
 * @author Nebiyu Tadesse
 * @date 2025
 * @copyright HardFOC
 */

#pragma once

#include "TMC9660CommInterface.hpp"
#include <memory>
#include <functional>

/**
 * @brief Unified communication interface supporting real-time SPI/UART switching
 *
 * This class provides a unified interface that can dynamically switch between
 * SPI and UART communication modes. It maintains the same API as the base
 * TMC9660CommInterface but allows runtime switching between underlying implementations.
 *
 * Key features:
 * - Real-time switching between SPI and UART modes
 * - Automatic initialization/deinitialization of underlying interfaces
 * - Shared GPIO control across both modes
 * - Thread-safe operation (when properly synchronized)
 * - Fallback support for failed interface initialization
 */
class TMC9660UnifiedCommInterface : public TMC9660CommInterface {
public:
    /**
     * @brief Construct unified communication interface
     * @param rstActiveLevel Physical GPIO level for RST pin when ACTIVE (true=HIGH, false=LOW)
     * @param drvEnActiveLevel Physical GPIO level for DRV_EN pin when ACTIVE (true=HIGH, false=LOW)
     * @param wakeActiveLevel Physical GPIO level for WAKE pin when ACTIVE (true=HIGH, false=LOW)
     * @param faultnActiveLevel Physical GPIO level for FAULTN pin when ACTIVE (true=HIGH, false=LOW)
     */
    TMC9660UnifiedCommInterface(bool rstActiveLevel, bool drvEnActiveLevel, 
                                bool wakeActiveLevel, bool faultnActiveLevel) noexcept
        : TMC9660CommInterface(rstActiveLevel, drvEnActiveLevel, wakeActiveLevel, faultnActiveLevel),
          current_mode_(CommMode::SPI), spi_interface_(nullptr), uart_interface_(nullptr) {}

    /**
     * @brief Destructor - cleans up all interfaces
     */
    ~TMC9660UnifiedCommInterface() noexcept override {
        deinitializeAll();
    }

    /**
     * @brief Set the SPI interface implementation
     * @param spi_interface Unique pointer to SPI interface (will be moved)
     * @return true if successful, false otherwise
     */
    bool setSPIInterface(std::unique_ptr<SPITMC9660CommInterface> spi_interface) noexcept {
        if (!spi_interface) {
            return false;
        }
        
        // Deinitialize current SPI interface if switching
        if (spi_interface_ && current_mode_ == CommMode::SPI) {
            spi_interface_->deinitialize();
        }
        
        spi_interface_ = std::move(spi_interface);
        
        // If we're currently in SPI mode, initialize the new interface
        if (current_mode_ == CommMode::SPI) {
            return spi_interface_->initialize();
        }
        
        return true;
    }

    /**
     * @brief Set the UART interface implementation
     * @param uart_interface Unique pointer to UART interface (will be moved)
     * @return true if successful, false otherwise
     */
    bool setUARTInterface(std::unique_ptr<UARTTMC9660CommInterface> uart_interface) noexcept {
        if (!uart_interface) {
            return false;
        }
        
        // Deinitialize current UART interface if switching
        if (uart_interface_ && current_mode_ == CommMode::UART) {
            uart_interface_->deinitialize();
        }
        
        uart_interface_ = std::move(uart_interface);
        
        // If we're currently in UART mode, initialize the new interface
        if (current_mode_ == CommMode::UART) {
            return uart_interface_->initialize();
        }
        
        return true;
    }

    /**
     * @brief Switch to SPI communication mode
     * @return true if successful, false otherwise
     */
    bool switchToSPI() noexcept {
        if (!spi_interface_) {
            logDebug(0, "UnifiedComm", "SPI interface not set");
            return false;
        }

        // Deinitialize current interface if different
        if (current_mode_ == CommMode::UART && uart_interface_) {
            uart_interface_->deinitialize();
        }

        // Initialize SPI interface
        if (!spi_interface_->initialize()) {
            logDebug(0, "UnifiedComm", "Failed to initialize SPI interface");
            return false;
        }

        current_mode_ = CommMode::SPI;
        logDebug(2, "UnifiedComm", "Switched to SPI mode");
        return true;
    }

    /**
     * @brief Switch to UART communication mode
     * @return true if successful, false otherwise
     */
    bool switchToUART() noexcept {
        if (!uart_interface_) {
            logDebug(0, "UnifiedComm", "UART interface not set");
            return false;
        }

        // Deinitialize current interface if different
        if (current_mode_ == CommMode::SPI && spi_interface_) {
            spi_interface_->deinitialize();
        }

        // Initialize UART interface
        if (!uart_interface_->initialize()) {
            logDebug(0, "UnifiedComm", "Failed to initialize UART interface");
            return false;
        }

        current_mode_ = CommMode::UART;
        logDebug(2, "UnifiedComm", "Switched to UART mode");
        return true;
    }

    /**
     * @brief Get current communication mode
     * @return Current communication mode
     */
    CommMode mode() const noexcept override {
        return current_mode_;
    }

    /**
     * @brief Check if SPI interface is available
     * @return true if SPI interface is set and ready
     */
    bool isSPIAvailable() const noexcept {
        return spi_interface_ != nullptr;
    }

    /**
     * @brief Check if UART interface is available
     * @return true if UART interface is set and ready
     */
    bool isUARTAvailable() const noexcept {
        return uart_interface_ != nullptr;
    }

    /**
     * @brief Get current active interface (for advanced usage)
     * @return Pointer to current active interface, or nullptr if none active
     */
    TMC9660CommInterface* getActiveInterface() noexcept {
        switch (current_mode_) {
            case CommMode::SPI:
                return spi_interface_.get();
            case CommMode::UART:
                return uart_interface_.get();
            default:
                return nullptr;
        }
    }

    /**
     * @brief Perform TMCL transfer using current active interface
     */
    bool transferTMCL(const TMCLFrame &tx, TMCLReply &reply, uint8_t address,
                     TMCLReply *firstReply, const TMCLFrame *secondCommand) noexcept override {
        switch (current_mode_) {
            case CommMode::SPI:
                if (!spi_interface_) {
                    logDebug(0, "UnifiedComm", "SPI interface not available");
                    return false;
                }
                return spi_interface_->transferTMCL(tx, reply, address, firstReply, secondCommand);
                
            case CommMode::UART:
                if (!uart_interface_) {
                    logDebug(0, "UnifiedComm", "UART interface not available");
                    return false;
                }
                return uart_interface_->transferTMCL(tx, reply, address, firstReply, secondCommand);
                
            default:
                logDebug(0, "UnifiedComm", "No active communication mode");
                return false;
        }
    }

    /**
     * @brief Set GPIO pin signal state using current active interface
     */
    bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept override {
        // Use the active interface for GPIO control
        TMC9660CommInterface* active = getActiveInterface();
        if (!active) {
            logDebug(0, "UnifiedComm", "No active interface for GPIO control");
            return false;
        }
        return active->gpioSet(pin, signal);
    }

    /**
     * @brief Read GPIO pin signal state using current active interface
     */
    bool gpioRead(TMC9660CtrlPin pin, GpioSignal &signal) noexcept override {
        // Use the active interface for GPIO control
        TMC9660CommInterface* active = getActiveInterface();
        if (!active) {
            logDebug(0, "UnifiedComm", "No active interface for GPIO control");
            return false;
        }
        return active->gpioRead(pin, signal);
    }

    /**
     * @brief Debug logging function
     */
    void debugLog(int level, const char* tag, const char* format, va_list args) noexcept override {
        // Use the active interface for logging, or fall back to base implementation
        TMC9660CommInterface* active = getActiveInterface();
        if (active) {
            active->debugLog(level, tag, format, args);
        } else {
            // Fallback to base implementation (no-op)
            (void)level;
            (void)tag;
            (void)format;
            (void)args;
        }
    }

    /**
     * @brief Delay execution using current active interface
     */
    void delayMs(uint32_t ms) noexcept override {
        // Use the active interface for delay, or fall back to base implementation
        TMC9660CommInterface* active = getActiveInterface();
        if (active) {
            active->delayMs(ms);
        } else {
            // Fallback implementation - this should be overridden by platform-specific implementations
            // For now, we'll use a simple busy wait (not recommended for production)
            volatile uint32_t count = ms * 1000; // Approximate for 1MHz CPU
            while (count--) {
                __asm__ __volatile__("nop");
            }
        }
    }

    /**
     * @brief Initialize all available interfaces
     * @return true if at least one interface was successfully initialized
     */
    bool initializeAll() noexcept {
        bool spi_ok = false;
        bool uart_ok = false;

        if (spi_interface_) {
            spi_ok = spi_interface_->initialize();
            if (spi_ok) {
                logDebug(2, "UnifiedComm", "SPI interface initialized successfully");
            } else {
                logDebug(1, "UnifiedComm", "Failed to initialize SPI interface");
            }
        }

        if (uart_interface_) {
            uart_ok = uart_interface_->initialize();
            if (uart_ok) {
                logDebug(2, "UnifiedComm", "UART interface initialized successfully");
            } else {
                logDebug(1, "UnifiedComm", "Failed to initialize UART interface");
            }
        }

        // Set default mode to the first successfully initialized interface
        if (spi_ok && !uart_ok) {
            current_mode_ = CommMode::SPI;
        } else if (uart_ok && !spi_ok) {
            current_mode_ = CommMode::UART;
        } else if (spi_ok && uart_ok) {
            // Both available, default to SPI
            current_mode_ = CommMode::SPI;
        }

        return spi_ok || uart_ok;
    }

    /**
     * @brief Deinitialize all interfaces
     */
    void deinitializeAll() noexcept {
        if (spi_interface_) {
            spi_interface_->deinitialize();
        }
        if (uart_interface_) {
            uart_interface_->deinitialize();
        }
        current_mode_ = CommMode::SPI; // Reset to default
    }

    /**
     * @brief Get interface status information
     * @return String describing current interface status
     */
    std::string getStatus() const noexcept {
        std::string status = "UnifiedComm Status: ";
        status += "Mode=" + std::string(current_mode_ == CommMode::SPI ? "SPI" : "UART");
        status += ", SPI=" + std::string(spi_interface_ ? "Available" : "Not Set");
        status += ", UART=" + std::string(uart_interface_ ? "Available" : "Not Set");
        return status;
    }

private:
    CommMode current_mode_;
    std::unique_ptr<SPITMC9660CommInterface> spi_interface_;
    std::unique_ptr<UARTTMC9660CommInterface> uart_interface_;
};

/**
 * @brief Factory function to create a unified communication interface
 * @param spi_interface SPI interface implementation
 * @param uart_interface UART interface implementation
 * @param rstActiveLevel Physical GPIO level for RST pin when ACTIVE
 * @param drvEnActiveLevel Physical GPIO level for DRV_EN pin when ACTIVE
 * @param wakeActiveLevel Physical GPIO level for WAKE pin when ACTIVE
 * @param faultnActiveLevel Physical GPIO level for FAULTN pin when ACTIVE
 * @return Unique pointer to unified interface, or nullptr on failure
 */
inline std::unique_ptr<TMC9660UnifiedCommInterface> createUnifiedCommInterface(
    std::unique_ptr<SPITMC9660CommInterface> spi_interface,
    std::unique_ptr<UARTTMC9660CommInterface> uart_interface,
    bool rstActiveLevel = true,
    bool drvEnActiveLevel = true,
    bool wakeActiveLevel = false,
    bool faultnActiveLevel = false) noexcept {
    
    auto unified = std::make_unique<TMC9660UnifiedCommInterface>(
        rstActiveLevel, drvEnActiveLevel, wakeActiveLevel, faultnActiveLevel);
    
    if (!unified) {
        return nullptr;
    }
    
    // Set the interfaces
    if (spi_interface && !unified->setSPIInterface(std::move(spi_interface))) {
        return nullptr;
    }
    
    if (uart_interface && !unified->setUARTInterface(std::move(uart_interface))) {
        return nullptr;
    }
    
    // Initialize all available interfaces
    if (!unified->initializeAll()) {
        return nullptr;
    }
    
    return unified;
}