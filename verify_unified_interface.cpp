/**
 * @file verify_unified_interface.cpp
 * @brief Verification script for unified communication interface
 * 
 * This file contains a simple verification that the unified interface
 * compiles correctly and has the expected API.
 */

#include "inc/TMC9660UnifiedCommInterface.hpp"
#include "examples/esp32/main/Esp32TMC9660UnifiedBus.hpp"
#include <memory>
#include <iostream>

// Mock implementations for testing
class MockSPIInterface : public SPITMC9660CommInterface {
public:
    MockSPIInterface() : SPITMC9660CommInterface(true, true, false, false) {}
    CommMode mode() const noexcept override { return CommMode::SPI; }
    bool spiTransferTMCL(std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept override { return true; }
    bool spiTransferBootloader(std::array<uint8_t, 5> &tx, std::array<uint8_t, 5> &rx) noexcept override { return true; }
    bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept override { return true; }
    bool gpioRead(TMC9660CtrlPin pin, GpioSignal &signal) noexcept override { signal = GpioSignal::INACTIVE; return true; }
    void debugLog(int level, const char* tag, const char* format, va_list args) noexcept override {}
    void delayMs(uint32_t ms) noexcept override {}
};

class MockUARTInterface : public UARTTMC9660CommInterface {
public:
    MockUARTInterface() : UARTTMC9660CommInterface(true, true, false, false) {}
    CommMode mode() const noexcept override { return CommMode::UART; }
    bool uartSendTMCL(const std::array<uint8_t, 9> &data) noexcept override { return true; }
    bool uartReceiveTMCL(std::array<uint8_t, 9> &data) noexcept override { return true; }
    bool uartTransferBootloader(const std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept override { return true; }
    bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept override { return true; }
    bool gpioRead(TMC9660CtrlPin pin, GpioSignal &signal) noexcept override { signal = GpioSignal::INACTIVE; return true; }
    void debugLog(int level, const char* tag, const char* format, va_list args) noexcept override {}
    void delayMs(uint32_t ms) noexcept override {}
};

int main() {
    std::cout << "Testing TMC9660 Unified Communication Interface..." << std::endl;
    
    try {
        // Test 1: Create unified interface
        std::cout << "Test 1: Creating unified interface..." << std::endl;
        auto unified = std::make_unique<TMC9660UnifiedCommInterface>(true, true, false, false);
        if (!unified) {
            std::cout << "❌ Failed to create unified interface" << std::endl;
            return 1;
        }
        std::cout << "✅ Unified interface created successfully" << std::endl;
        
        // Test 2: Set interfaces
        std::cout << "Test 2: Setting interfaces..." << std::endl;
        auto spi_interface = std::make_unique<MockSPIInterface>();
        auto uart_interface = std::make_unique<MockUARTInterface>();
        
        if (!unified->setSPIInterface(std::move(spi_interface))) {
            std::cout << "❌ Failed to set SPI interface" << std::endl;
            return 1;
        }
        
        if (!unified->setUARTInterface(std::move(uart_interface))) {
            std::cout << "❌ Failed to set UART interface" << std::endl;
            return 1;
        }
        std::cout << "✅ Interfaces set successfully" << std::endl;
        
        // Test 3: Mode switching
        std::cout << "Test 3: Testing mode switching..." << std::endl;
        
        if (!unified->switchToSPI()) {
            std::cout << "❌ Failed to switch to SPI mode" << std::endl;
            return 1;
        }
        std::cout << "✅ Switched to SPI mode" << std::endl;
        
        if (unified->mode() != CommMode::SPI) {
            std::cout << "❌ Mode not set to SPI" << std::endl;
            return 1;
        }
        std::cout << "✅ Mode correctly set to SPI" << std::endl;
        
        if (!unified->switchToUART()) {
            std::cout << "❌ Failed to switch to UART mode" << std::endl;
            return 1;
        }
        std::cout << "✅ Switched to UART mode" << std::endl;
        
        if (unified->mode() != CommMode::UART) {
            std::cout << "❌ Mode not set to UART" << std::endl;
            return 1;
        }
        std::cout << "✅ Mode correctly set to UART" << std::endl;
        
        // Test 4: Interface availability
        std::cout << "Test 4: Testing interface availability..." << std::endl;
        
        if (!unified->isSPIAvailable()) {
            std::cout << "❌ SPI interface not available" << std::endl;
            return 1;
        }
        std::cout << "✅ SPI interface available" << std::endl;
        
        if (!unified->isUARTAvailable()) {
            std::cout << "❌ UART interface not available" << std::endl;
            return 1;
        }
        std::cout << "✅ UART interface available" << std::endl;
        
        // Test 5: Status information
        std::cout << "Test 5: Testing status information..." << std::endl;
        std::string status = unified->getStatus();
        std::cout << "Status: " << status << std::endl;
        
        if (status.empty()) {
            std::cout << "❌ Status information is empty" << std::endl;
            return 1;
        }
        std::cout << "✅ Status information available" << std::endl;
        
        // Test 6: TMCL transfer (mock)
        std::cout << "Test 6: Testing TMCL transfer..." << std::endl;
        TMCLFrame frame;
        frame.opcode = 1;
        frame.type = 2;
        frame.motor = 0;
        frame.value = 0x12345678;
        
        TMCLReply reply;
        if (!unified->transferTMCL(frame, reply, 0)) {
            std::cout << "❌ TMCL transfer failed" << std::endl;
            return 1;
        }
        std::cout << "✅ TMCL transfer successful" << std::endl;
        
        // Test 7: GPIO operations
        std::cout << "Test 7: Testing GPIO operations..." << std::endl;
        
        if (!unified->gpioSet(TMC9660CtrlPin::RST, GpioSignal::ACTIVE)) {
            std::cout << "❌ GPIO set failed" << std::endl;
            return 1;
        }
        std::cout << "✅ GPIO set successful" << std::endl;
        
        GpioSignal signal;
        if (!unified->gpioRead(TMC9660CtrlPin::FAULTN, signal)) {
            std::cout << "❌ GPIO read failed" << std::endl;
            return 1;
        }
        std::cout << "✅ GPIO read successful" << std::endl;
        
        std::cout << std::endl;
        std::cout << "🎉 All tests passed! Unified interface is working correctly." << std::endl;
        std::cout << "The implementation supports real-time switching between SPI and UART modes." << std::endl;
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cout << "❌ Exception caught: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cout << "❌ Unknown exception caught" << std::endl;
        return 1;
    }
}