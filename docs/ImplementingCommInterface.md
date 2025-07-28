---
layout: default
title: Implementing a Custom Communication Interface
---

# 🔌 Implementing a Custom Communication Interface

The HF-TMC9660 driver is designed to be **hardware-agnostic**. This means it doesn't contain any platform-specific communication code. Instead, you provide a custom class that implements the low-level SPI or UART communication for your specific hardware platform.

This guide shows you how to create that communication interface with practical examples for different platforms.

---

## 🎯 What You'll Learn

- ✅ Understanding the `TMC9660CommInterface` architecture
- ✅ Implementing SPI communication interfaces
- ✅ Implementing UART communication interfaces  
- ✅ Platform-specific examples (Arduino, STM32, Linux, etc.)
- ✅ Testing and debugging your implementation
- ✅ Best practices for reliable communication

---

## 🏗️ Interface Architecture

The communication system uses an abstract base class that you inherit from:

```cpp
TMC9660CommInterface (Abstract Base)
├── SPITMC9660CommInterface (SPI Implementation)
└── UARTTMC9660CommInterface (UART Implementation)
```

### Core Requirements

Your implementation must:
1. **Exchange 8-byte frames** with the TMC9660
2. **Handle communication errors** gracefully
3. **Return success/failure status** for each transfer
4. **Be thread-safe** if used in multi-threaded environments

---

## 📡 SPI Communication Interface

Most applications use SPI communication due to its simplicity and speed.

### Step 1: Basic SPI Interface

```cpp
#include "TMC9660CommInterface.hpp"

class MySPIInterface : public SPITMC9660CommInterface {
private:
    // Your hardware-specific members
    SPIClass* spi_;
    int cs_pin_;
    
public:
    MySPIInterface(SPIClass* spi, int cs_pin) 
        : spi_(spi), cs_pin_(cs_pin) {
        // Initialize CS pin
        pinMode(cs_pin_, OUTPUT);
        digitalWrite(cs_pin_, HIGH);
    }
    
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        // Perform the 8-byte SPI transaction
        digitalWrite(cs_pin_, LOW);        // Assert CS
        
        for (int i = 0; i < 8; i++) {
            rx[i] = spi_->transfer(tx[i]); // Simultaneous TX/RX
        }
        
        digitalWrite(cs_pin_, HIGH);       // Release CS
        return true; // Return false on communication error
    }
};
```

### Step 2: Platform-Specific Examples

#### Arduino/ESP32 Implementation
```cpp
#include <SPI.h>

class ArduinoSPIInterface : public SPITMC9660CommInterface {
private:
    int cs_pin_;
    SPISettings spi_settings_;
    
public:
    ArduinoSPIInterface(int cs_pin, uint32_t frequency = 1000000) 
        : cs_pin_(cs_pin), spi_settings_(frequency, MSBFIRST, SPI_MODE3) {
        pinMode(cs_pin_, OUTPUT);
        digitalWrite(cs_pin_, HIGH);
        SPI.begin();
    }
    
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        SPI.beginTransaction(spi_settings_);
        digitalWrite(cs_pin_, LOW);
        
        // Transfer all 8 bytes
        for (size_t i = 0; i < 8; i++) {
            rx[i] = SPI.transfer(tx[i]);
        }
        
        digitalWrite(cs_pin_, HIGH);
        SPI.endTransaction();
        return true;
    }
};

// Usage
ArduinoSPIInterface spi_bus(10);  // CS on pin 10
TMC9660 driver(spi_bus);
```

#### STM32 HAL Implementation
```cpp
class STM32SPIInterface : public SPITMC9660CommInterface {
private:
    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;
    
public:
    STM32SPIInterface(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
        : hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin) {
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    }
    
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
        
        HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(
            hspi_, tx.data(), rx.data(), 8, 100);
        
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
        
        return (result == HAL_OK);
    }
};

// Usage
STM32SPIInterface spi_bus(&hspi1, GPIOA, GPIO_PIN_4);
TMC9660 driver(spi_bus);
```

#### Linux userspace SPI Implementation
```cpp
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

class LinuxSPIInterface : public SPITMC9660CommInterface {
private:
    int spi_fd_;
    
public:
    LinuxSPIInterface(const char* device = "/dev/spidev0.0") {
        spi_fd_ = open(device, O_RDWR);
        if (spi_fd_ < 0) {
            // Handle error without exceptions - return error code or use error callback
            spi_fd_ = -1;  // Mark as invalid
            return;
        }
        
        // Configure SPI mode and speed
        uint8_t mode = SPI_MODE_3;
        uint32_t speed = 1000000; // 1 MHz
        
        ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode);
        ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    }
    
    ~LinuxSPIInterface() {
        if (spi_fd_ >= 0) close(spi_fd_);
    }
    
    bool isValid() const noexcept {
        return spi_fd_ >= 0;
    }
    
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        if (!isValid()) {
            return false;  // Interface not properly initialized
        }
        
        struct spi_ioc_transfer transfer = {};
        transfer.tx_buf = reinterpret_cast<uintptr_t>(tx.data());
        transfer.rx_buf = reinterpret_cast<uintptr_t>(rx.data());
        transfer.len = 8;
        transfer.speed_hz = 1000000;
        transfer.bits_per_word = 8;
        
        int result = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &transfer);
        return (result >= 0);
    }
};

// Usage
LinuxSPIInterface spi_bus("/dev/spidev0.0");
TMC9660 driver(spi_bus);
```

---

## 📺 UART Communication Interface

For applications requiring UART communication (RS485 networks, longer distances):

### Basic UART Interface

```cpp
class MyUARTInterface : public UARTTMC9660CommInterface {
private:
    HardwareSerial* uart_;
    int txen_pin_;  // RS485 TX enable pin (optional)
    
public:
    MyUARTInterface(HardwareSerial* uart, int txen_pin = -1) 
        : uart_(uart), txen_pin_(txen_pin) {
        if (txen_pin_ >= 0) {
            pinMode(txen_pin_, OUTPUT);
            digitalWrite(txen_pin_, LOW); // RX mode
        }
    }
    
    bool uartTransfer(const TMCLFrame& tx, TMCLReply& reply, 
                      uint8_t address) noexcept override {
        // Enable transmission for RS485
        if (txen_pin_ >= 0) digitalWrite(txen_pin_, HIGH);
        
        // Send 8-byte frame
        uint8_t frame[8];
        tx.serialize(frame, address);
        uart_->write(frame, 8);
        uart_->flush();
        
        // Switch to receive mode
        if (txen_pin_ >= 0) digitalWrite(txen_pin_, LOW);
        
        // Wait for reply (8 bytes + checksum)
        uint8_t reply_data[9];
        size_t bytes_read = 0;
        unsigned long start_time = millis();
        
        while (bytes_read < 9 && (millis() - start_time) < 100) {
            if (uart_->available()) {
                reply_data[bytes_read++] = uart_->read();
            }
        }
        
        if (bytes_read == 9) {
            return reply.deserialize(reply_data);
        }
        return false; // Timeout or incomplete frame
    }
};
```

---

## 🧪 Testing Your Implementation

### Basic Communication Test

```cpp
bool testCommunication(TMC9660& driver) {
    // Try to read a parameter to verify communication
    uint32_t motor_type = 0;
    
    if (driver.readParameter(tmc9660::tmcl::Parameters::MOTOR_TYPE, motor_type)) {
        std::cout << "✅ Communication working! Motor type: " << motor_type << std::endl;
        return true;
    } else {
        std::cout << "❌ Communication failed!" << std::endl;
        return false;
    }
}

// Usage
MySPIInterface my_interface(/* your parameters */);
TMC9660 driver(my_interface);

// Configure for parameter mode
tmc9660::BootloaderConfig cfg{};
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
driver.bootloaderInit(&cfg);

// Test communication
testCommunication(driver);
```

### Advanced Testing with Loopback

```cpp
bool testLoopback(TMC9660& driver) {
    // Write a known value and read it back
    const uint32_t test_value = 0x12345678;
    
    if (!driver.writeParameter(tmc9660::tmcl::Parameters::USER_VARIABLE_0, test_value)) {
        std::cout << "❌ Write failed" << std::endl;
        return false;
    }
    
    uint32_t read_value = 0;
    if (!driver.readParameter(tmc9660::tmcl::Parameters::USER_VARIABLE_0, read_value)) {
        std::cout << "❌ Read failed" << std::endl;
        return false;
    }
    
    if (read_value == test_value) {
        std::cout << "✅ Loopback test passed!" << std::endl;
        return true;
    } else {
        std::cout << "❌ Loopback test failed: wrote 0x" << std::hex << test_value 
                  << ", read 0x" << read_value << std::endl;
        return false;
    }
}
```

---

## 🔧 Best Practices & Tips

### Performance Optimization

```cpp
class OptimizedSPIInterface : public SPITMC9660CommInterface {
private:
    // Use DMA for faster transfers (platform-specific)
    bool use_dma_;
    
public:
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        if (use_dma_) {
            return dmaTransfer(tx, rx);  // Your DMA implementation
        } else {
            return pollingTransfer(tx, rx);  // Fallback polling method
        }
    }
};
```

### Error Handling & Retry Logic

```cpp
class RobustSPIInterface : public SPITMC9660CommInterface {
private:
    static constexpr int MAX_RETRIES = 3;
    
public:
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        for (int attempt = 0; attempt < MAX_RETRIES; attempt++) {
            if (attemptTransfer(tx, rx)) {
                return true;
            }
            
            // Brief delay before retry
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        return false; // All retries failed
    }
    
private:
    bool attemptTransfer(std::array<uint8_t,8>& tx,
                        std::array<uint8_t,8>& rx) noexcept {
        // Your actual transfer implementation
        // Return false on any error condition
    }
};
```

### Thread Safety

```cpp
class ThreadSafeSPIInterface : public SPITMC9660CommInterface {
private:
    std::mutex spi_mutex_;
    
public:
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        std::lock_guard<std::mutex> lock(spi_mutex_);
        
        // Your transfer implementation here
        // Mutex ensures only one transfer at a time
        
        return performTransfer(tx, rx);
    }
};
```

---

## 🚨 Troubleshooting Common Issues

### SPI Communication Problems

**Problem:** All transfers return success but parameter reads/writes fail
**Solution:** Check SPI mode (TMC9660 uses Mode 3), bit order (MSB first), and timing

**Problem:** Intermittent communication failures  
**Solution:** Add delays between CS assertion and first clock, verify signal integrity

**Problem:** Wrong data received
**Solution:** Verify CS polarity, check for electrical noise, ensure proper grounding

### UART Communication Problems

**Problem:** No response from device
**Solution:** Verify baud rate, check RX/TX pin connections, ensure proper address configuration

**Problem:** Corrupted data
**Solution:** Check for timing issues, verify start/stop bits, ensure RS485 timing if used

---

## 📋 Implementation Checklist

Before integrating your communication interface:

- [ ] **Correct inheritance** from `SPITMC9660CommInterface` or `UARTTMC9660CommInterface`
- [ ] **8-byte frame handling** implemented correctly
- [ ] **Error conditions handled** and return appropriate boolean status
- [ ] **Hardware initialization** performed in constructor
- [ ] **Resource cleanup** implemented in destructor (if needed)
- [ ] **Basic communication test** passes
- [ ] **Loopback test** with known values works
- [ ] **Thread safety** considered if applicable

---

## 🎯 Next Steps

With your communication interface working, you're ready to explore motor control:

**👉 [Building Examples](BuildingExamples.html)** - Compile and test the provided examples

**👉 [Hardware-Agnostic Examples](HardwareAgnosticExamples.html)** - Complete motor control scenarios

---

[⬅️ Setup Guide](SetupGuide.html) | [⬆️ Back to Index](index.html) | [Next ➡️ Building Examples](BuildingExamples.html)

---

*Need specific platform examples? Check the examples directory or contribute your implementation to help others!*
