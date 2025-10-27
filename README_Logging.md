# TMC9660 Logging Configuration System

This document explains how to use the TMC9660 logging configuration system to control whether logging statements are compiled in or optimized out.

## Quick Start

### Enable All Logging (Debug Build)
```cpp
#define TMC9660_LOG_LEVEL_ALL
```

### Enable Specific Log Levels
```cpp
#define TMC9660_LOG_LEVEL_ERROR
#define TMC9660_LOG_LEVEL_WARNING
#define TMC9660_LOG_LEVEL_INFO
```

### Disable All Logging (Release Build)
```cpp
#define TMC9660_LOG_DISABLE_ALL
```

## What's New

The TMC9660 library now includes a comprehensive logging configuration system that allows you to:

- **Control logging at compile time**: Enable or disable logging statements completely
- **Select specific log levels**: Choose which logging levels are active
- **Optimize for production**: Remove unwanted logging in release builds
- **Easy detection**: Determine if logging is compiled in or optimized out

## Files Added

- `inc/TMC9660LoggingConfig.hpp` - Main logging configuration header
- `examples/esp32/logging_configs.h` - Example configurations
- `examples/esp32/CMakeLists_logging.txt` - CMake configuration examples
- `examples/esp32/main/LoggingTest.cpp` - Test program
- `examples/esp32/scripts/test_logging.sh` - Test script
- `docs/LoggingConfigurationGuide.md` - Complete documentation

## Files Modified

- `inc/TMC9660CommInterface.hpp` - Updated to use new logging configuration
- `inc/TMC9660Bootloader.hpp` - Added logging configuration include
- `src/TMC9660Bootloader.cpp` - Updated all logging calls to use new macros
- `src/TMC9660.cpp` - Updated all logging calls to use new macros
- `examples/esp32/CMakeLists.txt` - Added logging configuration options

## Usage Examples

### Basic Logging
```cpp
#include "TMC9660LoggingConfig.hpp"

void exampleFunction() {
    TMC9660_LOG_ERROR("MyTag", "This is an error message: %d", 42);
    TMC9660_LOG_WARNING("MyTag", "This is a warning message");
    TMC9660_LOG_INFO("MyTag", "This is an info message: %s", "hello");
    TMC9660_LOG_DEBUG("MyTag", "This debug message may be optimized out");
    TMC9660_LOG_VERBOSE("MyTag", "This verbose message may be optimized out");
}
```

### Compile-Time Detection
```cpp
#include "TMC9660LoggingConfig.hpp"

void conditionalLogging() {
    #if TMC9660_LOG_ENABLED
        printf("Logging is enabled in this build\n");
    #endif
    
    #if TMC9660_LOG_LEVEL_AT_LEAST(2)  // INFO level or higher
        printf("Info level logging is available\n");
    #endif
}
```

### CMake Configuration
```cmake
# Enable all logging
set(TMC9660_LOG_LEVEL_ALL ON)

# Or enable specific levels
set(TMC9660_LOG_LEVEL_ERROR ON)
set(TMC9660_LOG_LEVEL_WARNING ON)
set(TMC9660_LOG_LEVEL_INFO ON)
```

## Testing

To test the logging configuration system:

1. **Run the test program**:
   ```bash
   cd examples/esp32
   ./scripts/test_logging.sh
   ```

2. **Test specific configurations**:
   ```bash
   # Test with all logging enabled
   idf.py build -DTMC9660_LOG_LEVEL_ALL=ON
   
   # Test with all logging disabled
   idf.py build -DTMC9660_LOG_DISABLE_ALL=ON
   
   # Test with specific levels
   idf.py build -DTMC9660_LOG_LEVEL_ERROR=ON -DTMC9660_LOG_LEVEL_WARNING=ON
   ```

## Performance Impact

### When Logging is Disabled
- **Zero runtime overhead**: No function calls, no string formatting, no memory allocation
- **Complete optimization**: Logging statements are completely removed by the preprocessor
- **Minimum binary size**: No logging code is included in the final binary

### When Logging is Enabled
- **Runtime overhead**: Function calls, string formatting, and memory allocation occur
- **Binary size increase**: Logging code is included in the final binary
- **Configurable impact**: You can control which levels are enabled to balance debugging vs. performance

## Common Configurations

### Development Build
```cpp
#define TMC9660_LOG_LEVEL_ALL
```
- All logging enabled
- Maximum debugging information
- Higher memory usage and runtime overhead

### Production Build
```cpp
#define TMC9660_LOG_DISABLE_ALL
```
- No logging enabled
- Zero logging overhead
- Minimum memory usage

### Standard Production Build
```cpp
#define TMC9660_LOG_LEVEL_ERROR
#define TMC9660_LOG_LEVEL_WARNING
#define TMC9660_LOG_LEVEL_INFO
```
- Error, warning, and info logging enabled
- Debug and verbose logging disabled
- Balanced approach for production use

## Integration

The logging configuration system integrates seamlessly with:

- **ESP-IDF logging**: Routes to ESP-IDF logging system on ESP32
- **Platform-specific logging**: Works with any logging implementation
- **CMake builds**: Full CMake integration with options
- **Preprocessor definitions**: Works with any build system

## Troubleshooting

### Logging Not Working
1. Check that the logging configuration is properly defined
2. Verify that the logging level is enabled for the messages you're trying to log
3. Ensure that the logging implementation is properly connected

### Compilation Errors
1. Make sure you're including `TMC9660LoggingConfig.hpp`
2. Check that the logging configuration macros are properly defined
3. Verify that you're using the correct logging macro names

### Performance Issues
1. Consider disabling debug and verbose logging in production builds
2. Use `TMC9660_LOG_DISABLE_ALL` for maximum performance
3. Check that logging is actually being optimized out

## Documentation

For complete documentation, see:
- `docs/LoggingConfigurationGuide.md` - Comprehensive guide
- `inc/TMC9660LoggingConfig.hpp` - API reference
- `examples/esp32/logging_configs.h` - Configuration examples

## Support

If you have questions or issues with the logging configuration system, please:
1. Check the documentation
2. Run the test program to verify your configuration
3. Check the troubleshooting section
4. Create an issue with your specific configuration and error details