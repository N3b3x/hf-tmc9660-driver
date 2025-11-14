# Platform Integration

The TMC9660 driver uses a hardware-agnostic communication interface pattern (CRTP - Curiously Recurring Template Pattern) to abstract platform-specific SPI or UART implementations. This allows the same driver code to work across different MCU platforms.

## Understanding the Interface Pattern

The driver uses **CRTP (Curiously Recurring Template Pattern)** for compile-time polymorphism. This provides:

- ✅ **Zero Runtime Overhead**: No virtual function calls, all resolved at compile time
- ✅ **Compile-Time Optimization**: Inlining and dead code elimination
- ✅ **Memory Efficiency**: No vtable overhead
- ✅ **Type Safety**: Compile-time interface checking

### Interface Architecture

```cpp
CommInterface (Abstract Base)
├── SpiCommInterface (SPI Implementation)
└── UartCommInterface (UART Implementation)
```

## SPI Interface Implementation

### Base Interface

```cpp
#include "inc/tmc9660_comm_interface.hpp"

class MySPI : public tmc9660::SpiCommInterface {
public:
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        // Your SPI transfer implementation
        // Exchange 8 bytes: tx -> TMC9660, rx <- TMC9660
        return true; // Return false on error
    }
};
```

### ESP32 Example

```cpp
#include "driver/spi_master.h"
#include "inc/tmc9660_comm_interface.hpp"

class ESP32SPI : public tmc9660::SpiCommInterface {
private:
    spi_device_handle_t spi_;
    
public:
    ESP32SPI(spi_host_device_t host, gpio_num_t mosi, gpio_num_t miso, 
              gpio_num_t sclk, gpio_num_t cs) {
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = mosi,
            .miso_io_num = miso,
            .sclk_io_num = sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        };
        spi_bus_initialize(host, &bus_cfg, SPI_DMA_CH_AUTO);
        
        spi_device_interface_config_t dev_cfg = {
            .clock_speed_hz = 4 * 1000 * 1000, // 4 MHz
            .mode = 0,
            .spics_io_num = cs,
            .queue_size = 1,
        };
        spi_bus_add_device(host, &dev_cfg, &spi_);
    }
    
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        spi_transaction_t trans = {
            .length = 64, // 8 bytes * 8 bits
            .tx_buffer = tx.data(),
            .rx_buffer = rx.data(),
        };
        
        esp_err_t ret = spi_device_transmit(spi_, &trans);
        return (ret == ESP_OK);
    }
};

// Usage
ESP32SPI spi(VSPI_HOST, GPIO_NUM_7, GPIO_NUM_2, GPIO_NUM_6, GPIO_NUM_18);
tmc9660::TMC9660 driver(spi);
```

### STM32 HAL Example

```cpp
#include "stm32f4xx_hal.h"
#include "inc/tmc9660_comm_interface.hpp"

class STM32SPI : public tmc9660::SpiCommInterface {
private:
    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;
    
public:
    STM32SPI(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
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
STM32SPI spi(&hspi1, GPIOA, GPIO_PIN_4);
tmc9660::TMC9660 driver(spi);
```

### Arduino Example

```cpp
#include <SPI.h>
#include "inc/tmc9660_comm_interface.hpp"

class ArduinoSPI : public tmc9660::SpiCommInterface {
private:
    int cs_pin_;
    SPISettings spi_settings_;
    
public:
    ArduinoSPI(int cs_pin, uint32_t frequency = 1000000) 
        : cs_pin_(cs_pin), spi_settings_(frequency, MSBFIRST, SPI_MODE0) {
        pinMode(cs_pin_, OUTPUT);
        digitalWrite(cs_pin_, HIGH);
        SPI.begin();
    }
    
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        SPI.beginTransaction(spi_settings_);
        digitalWrite(cs_pin_, LOW);
        
        for (size_t i = 0; i < 8; i++) {
            rx[i] = SPI.transfer(tx[i]);
        }
        
        digitalWrite(cs_pin_, HIGH);
        SPI.endTransaction();
        return true;
    }
};

// Usage
ArduinoSPI spi(10); // CS on pin 10
tmc9660::TMC9660 driver(spi);
```

## UART Interface Implementation

### Base Interface

```cpp
class MyUART : public tmc9660::UartCommInterface {
public:
    bool uartTransfer(const tmc9660::TMCLFrame& tx, 
                      tmc9660::TMCLReply& reply,
                      uint8_t address) noexcept override {
        // Your UART transfer implementation
        // Send 8-byte frame, receive 9-byte reply
        return true; // Return false on error
    }
};
```

### ESP32 UART Example

```cpp
#include "driver/uart.h"
#include "inc/tmc9660_comm_interface.hpp"

class ESP32UART : public tmc9660::UartCommInterface {
private:
    uart_port_t uart_num_;
    
public:
    ESP32UART(uart_port_t uart_num, gpio_num_t tx, gpio_num_t rx, 
               int baud_rate = 115200) : uart_num_(uart_num) {
        uart_config_t uart_config = {
            .baud_rate = baud_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        };
        uart_param_config(uart_num_, &uart_config);
        uart_set_pin(uart_num_, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_driver_install(uart_num_, 1024, 1024, 0, NULL, 0);
    }
    
    bool uartTransfer(const tmc9660::TMCLFrame& tx,
                     tmc9660::TMCLReply& reply,
                     uint8_t address) noexcept override {
        // Serialize and send frame
        uint8_t frame[9];
        tx.serialize(frame, address);
        uart_write_bytes(uart_num_, frame, 9);
        
        // Wait for and read reply
        uint8_t reply_data[9];
        int len = uart_read_bytes(uart_num_, reply_data, 9, 
                                   pdMS_TO_TICKS(100));
        if (len == 9) {
            return reply.deserialize(reply_data);
        }
        return false;
    }
};

// Usage
ESP32UART uart(UART_NUM_1, GPIO_NUM_5, GPIO_NUM_4);
tmc9660::TMC9660 driver(uart);
```

## GPIO Control Interface

The TMC9660 also requires GPIO control for RST, DRV_EN, WAKE, and FAULTN pins. Implement these methods if your interface class inherits from `CommInterface`:

```cpp
class MySPI : public tmc9660::SpiCommInterface {
public:
    // ... spiTransfer implementation ...
    
    // GPIO control (optional but recommended)
    void gpioSetActive(tmc9660::TMC9660CtrlPin pin) noexcept override {
        // Set pin to active state
    }
    
    void gpioSetInactive(tmc9660::TMC9660CtrlPin pin) noexcept override {
        // Set pin to inactive state
    }
    
    bool gpioRead(tmc9660::TMC9660CtrlPin pin, bool& signal) noexcept override {
        // Read pin state
        return true;
    }
};
```

## Testing Your Implementation

### Basic Communication Test

```cpp
bool testCommunication(tmc9660::TMC9660& driver) {
    // Try to read a known register
    auto result = driver.bootloaderGetVersion();
    return (result.has_value());
}
```

## Best Practices

1. **Error Handling**: Always return `false` from transfer methods on communication errors
2. **CS Management**: Properly assert/deassert CS for SPI
3. **Timing**: Respect SPI/UART timing requirements
4. **Thread Safety**: Make implementations thread-safe if used in multi-threaded environments
5. **No Exceptions**: Use `noexcept` and return error codes instead of throwing

## Next Steps

- **[Quick Start](quickstart.md)** - Get a minimal example running
- **[Hardware Setup](hardware_setup.md)** - Complete wiring guide

