# TMC9660 Unified Communication Interface

This example demonstrates the unified communication interface for TMC9660 that allows real-time switching between SPI and UART communication modes.

## Overview

The unified communication interface provides a single API that can dynamically switch between SPI and UART communication modes at runtime. This is particularly useful for:

- **Development and Testing**: Test both communication interfaces with the same hardware setup
- **Fallback Scenarios**: Switch to an alternative communication method if one fails
- **Debugging**: Use UART for debugging while maintaining SPI for high-speed operations
- **Flexible Applications**: Different operations may benefit from different communication modes

## Key Features

- **Real-time Switching**: Switch between SPI and UART modes without restarting the application
- **Unified API**: Same interface for both communication modes
- **Automatic Management**: Handles initialization/deinitialization of underlying interfaces
- **Thread-safe**: Safe to use in multi-threaded environments (with proper synchronization)
- **Fallback Support**: Graceful handling when interfaces are not available
- **Performance Monitoring**: Built-in performance comparison between modes

## Architecture

### Core Components

1. **TMC9660UnifiedCommInterface**: Base unified interface class
2. **Esp32TMC9660UnifiedCommInterface**: ESP32-specific implementation
3. **TMC9660CommModeManager**: Utility class for mode management
4. **Factory Functions**: Convenient creation functions

### Class Hierarchy

```
TMC9660CommInterface (base)
    ├── SPITMC9660CommInterface
    ├── UARTTMC9660CommInterface
    └── TMC9660UnifiedCommInterface
            └── Esp32TMC9660UnifiedCommInterface
```

## Usage Examples

### Basic Usage

```cpp
#include "Esp32TMC9660UnifiedBus.hpp"

// Create unified interface
auto unified_interface = createUnifiedInterface();
if (!unified_interface) {
    ESP_LOGE(TAG, "Failed to create unified interface");
    return;
}

// Create TMC9660 driver
TMC9660 driver(*unified_interface, 0);

// Switch to SPI mode
if (unified_interface->switchToSPI()) {
    ESP_LOGI(TAG, "Switched to SPI mode");
    
    // Perform SPI operations
    uint32_t version = 0;
    driver.getVersion(version);
}

// Switch to UART mode
if (unified_interface->switchToUART()) {
    ESP_LOGI(TAG, "Switched to UART mode");
    
    // Perform UART operations
    uint32_t version = 0;
    driver.getVersion(version);
}
```

### Using Mode Manager

```cpp
#include "Esp32TMC9660UnifiedBus.hpp"

// Create unified interface
auto unified_interface = createUnifiedInterface();
TMC9660CommModeManager mode_manager(*unified_interface);

// Check available modes
ESP_LOGI(TAG, "Available modes: %s", mode_manager.getAvailableModes().c_str());

// Switch to other mode
if (mode_manager.hasBothModes()) {
    mode_manager.switchToOtherMode();
    
    // Switch back to last mode
    mode_manager.switchToLastMode();
}
```

### Application Example

```cpp
// Create TMC9660 driver with unified interface
auto driver = createUnifiedTMC9660Driver();
TMC9660CommModeManager mode_manager(*driver->getCommInterface());

// Application logic with mode switching
for (int phase = 0; phase < 3; ++phase) {
    // Phase 1: Use SPI for high-speed operations
    if (mode_manager.switchToSPI()) {
        // High-speed operations
        for (int i = 0; i < 10; ++i) {
            uint32_t version = 0;
            driver->getVersion(version);
        }
    }
    
    // Phase 2: Use UART for debugging
    if (mode_manager.switchToUART()) {
        // Debugging operations
        for (int i = 0; i < 5; ++i) {
            uint32_t version = 0;
            driver->getVersion(version);
        }
    }
}
```

## Building and Running

### Prerequisites

- ESP-IDF v5.0 or later
- ESP32-C6 DevKit-M-1 (or compatible board)
- TMC9660 evaluation board

### Build Commands

```bash
# Navigate to the examples directory
cd examples/esp32

# Build the unified communication example
./build_unified_comm.sh

# Or build with specific build type
./build_unified_comm.sh Release
```

### Flash and Monitor

```bash
# Flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor

# Or use the build script
./build_unified_comm.sh && idf.py -p /dev/ttyUSB0 flash monitor
```

## Configuration

### Pin Configuration

The unified interface uses the same pin configuration as the individual interfaces:

```cpp
Esp32TMC9660BusConfig config;

// SPI Configuration
config.spi.host = SPI2_HOST;
config.spi.mosi_pin = GPIO_NUM_7;
config.spi.miso_pin = GPIO_NUM_2;
config.spi.sclk_pin = GPIO_NUM_6;
config.spi.cs_pin = GPIO_NUM_18;
config.spi.clock_speed_hz = 1000000; // 1 MHz
config.spi.mode = 3; // SPI Mode 3

// UART Configuration
config.uart.uart_num = UART_NUM_1;
config.uart.tx_pin = GPIO_NUM_4;
config.uart.rx_pin = GPIO_NUM_5;
config.uart.baud_rate = 115200;
config.uart.address = 0;

// GPIO Configuration
config.gpio.rst_pin = GPIO_NUM_22;
config.gpio.drv_en_pin = GPIO_NUM_20;
config.gpio.faultn_pin = GPIO_NUM_19;
config.gpio.wake_pin = GPIO_NUM_21;
```

### Custom Configuration

```cpp
// Create with custom configuration
Esp32TMC9660BusConfig custom_config;
custom_config.spi.clock_speed_hz = 2000000; // 2 MHz
custom_config.uart.baud_rate = 230400;

auto unified_interface = createUnifiedInterface(custom_config);
```

## API Reference

### TMC9660UnifiedCommInterface

#### Core Methods

- `bool switchToSPI()` - Switch to SPI communication mode
- `bool switchToUART()` - Switch to UART communication mode
- `CommMode mode()` - Get current communication mode
- `bool isSPIAvailable()` - Check if SPI interface is available
- `bool isUARTAvailable()` - Check if UART interface is available

#### Interface Management

- `bool setSPIInterface(std::unique_ptr<SPITMC9660CommInterface>)` - Set SPI interface
- `bool setUARTInterface(std::unique_ptr<UARTTMC9660CommInterface>)` - Set UART interface
- `bool initializeAll()` - Initialize all available interfaces
- `void deinitializeAll()` - Deinitialize all interfaces

#### Status and Debugging

- `std::string getStatus()` - Get interface status
- `TMC9660CommInterface* getActiveInterface()` - Get current active interface

### TMC9660CommModeManager

#### Mode Switching

- `bool switchToSPI()` - Switch to SPI mode
- `bool switchToUART()` - Switch to UART mode
- `bool switchToOtherMode()` - Switch to the other available mode
- `bool switchToLastMode()` - Switch back to last used mode

#### Status Queries

- `CommMode getCurrentMode()` - Get current mode
- `CommMode getLastMode()` - Get last used mode
- `bool hasBothModes()` - Check if both modes are available
- `std::string getAvailableModes()` - Get available modes as string

## Performance Considerations

### SPI vs UART Performance

- **SPI**: Generally faster, lower latency, better for high-frequency operations
- **UART**: Slower but more reliable, better for debugging and long-distance communication

### Switching Overhead

- Mode switching involves deinitializing one interface and initializing another
- Typical switching time: 10-50ms depending on interface complexity
- Consider caching frequently used data to minimize switching

### Memory Usage

- Both interfaces are kept in memory simultaneously
- Additional overhead: ~2-4KB for interface management
- GPIO control is shared between interfaces

## Error Handling

### Common Error Scenarios

1. **Interface Not Available**: Check `isSPIAvailable()` or `isUARTAvailable()`
2. **Switch Failed**: Check interface initialization status
3. **Communication Failed**: Verify hardware connections and configuration

### Error Recovery

```cpp
// Robust error handling example
if (!unified_interface->switchToSPI()) {
    ESP_LOGW(TAG, "SPI switch failed, trying UART");
    if (!unified_interface->switchToUART()) {
        ESP_LOGE(TAG, "Both interfaces failed");
        return false;
    }
}
```

## Troubleshooting

### Common Issues

1. **Build Errors**: Ensure ESP-IDF v5.0+ and proper component configuration
2. **Interface Not Found**: Check pin configuration and hardware connections
3. **Switch Failures**: Verify interface initialization and resource availability
4. **Communication Errors**: Check TMC9660 power and signal integrity

### Debug Output

Enable debug logging to see detailed interface switching information:

```cpp
// Enable debug logging
esp_log_level_set("UnifiedComm", ESP_LOG_DEBUG);
esp_log_level_set("TMC9660_Bus", ESP_LOG_DEBUG);
```

## Examples

The example includes several test scenarios:

1. **Basic Mode Switching**: Test fundamental switching functionality
2. **Communication Tests**: Verify communication works in both modes
3. **Performance Comparison**: Compare SPI vs UART performance
4. **Fallback Scenarios**: Test error handling and recovery
5. **Real-time Switching**: Test switching during active operations

## Future Enhancements

- **Automatic Fallback**: Automatic switching on communication failures
- **Load Balancing**: Distribute operations across interfaces
- **Hot-swapping**: Support for runtime interface configuration changes
- **Performance Metrics**: Detailed performance monitoring and optimization

## License

This code is part of the TMC9660 driver library and follows the same license terms.