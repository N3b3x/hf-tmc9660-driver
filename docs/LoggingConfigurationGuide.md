# TMC9660 Logging Configuration Guide

This guide explains how to configure logging for the TMC9660 library to control whether logging statements are compiled in or optimized out.

## Overview

The TMC9660 library includes a comprehensive logging system that can be configured at compile time. This allows you to:

- Enable or disable logging completely
- Control which log levels are active
- Optimize out unwanted logging in production builds
- Easily determine if logging is compiled in or optimized out

## Log Levels

The TMC9660 library supports five log levels:

| Level | Name | Description | Typical Use |
|-------|------|-------------|-------------|
| 0 | ERROR | Critical errors that prevent operation | Always enabled in production |
| 1 | WARNING | Warnings about potential issues | Usually enabled in production |
| 2 | INFO | General information about operation | Often enabled in production |
| 3 | DEBUG | Detailed debugging information | Usually disabled in production |
| 4 | VERBOSE | Very detailed debugging information | Usually disabled in production |

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

## Configuration Methods

### Method 1: Preprocessor Definitions

Add the desired logging configuration to your source files or compiler flags:

```cpp
// In your source file or header
#define TMC9660_LOG_LEVEL_ERROR
#define TMC9660_LOG_LEVEL_WARNING
#define TMC9660_LOG_LEVEL_INFO
```

### Method 2: Compiler Flags

Pass the configuration through compiler flags:

```bash
# GCC/Clang
gcc -DTMC9660_LOG_LEVEL_ERROR -DTMC9660_LOG_LEVEL_WARNING -DTMC9660_LOG_LEVEL_INFO

# Visual Studio
# Add to preprocessor definitions: TMC9660_LOG_LEVEL_ERROR;TMC9660_LOG_LEVEL_WARNING;TMC9660_LOG_LEVEL_INFO
```

### Method 3: CMake Configuration

Use CMake options to configure logging:

```cmake
# Enable all logging
set(TMC9660_LOG_LEVEL_ALL ON)

# Or enable specific levels
set(TMC9660_LOG_LEVEL_ERROR ON)
set(TMC9660_LOG_LEVEL_WARNING ON)
set(TMC9660_LOG_LEVEL_INFO ON)

# Apply to target
if(TMC9660_LOG_LEVEL_ALL)
    target_compile_definitions(your_target PRIVATE TMC9660_LOG_LEVEL_ALL)
elseif(TMC9660_LOG_DISABLE_ALL)
    target_compile_definitions(your_target PRIVATE TMC9660_LOG_DISABLE_ALL)
else()
    # Apply individual log level definitions
    if(TMC9660_LOG_LEVEL_ERROR)
        target_compile_definitions(your_target PRIVATE TMC9660_LOG_LEVEL_ERROR)
    endif()
    # ... other levels
endif()
```

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

### Conditional Compilation
```cpp
#include "TMC9660LoggingConfig.hpp"

void conditionalLogging() {
    #if TMC9660_LOG_ENABLED
        printf("Logging is enabled in this build\n");
    #endif
    
    #if TMC9660_LOG_LEVEL_AT_LEAST(2)  // INFO level or higher
        printf("Info level logging is available\n");
    #endif
    
    #if TMC9660_LOG_LEVEL_ENABLED(3)  // DEBUG level
        printf("Debug logging is specifically enabled\n");
    #endif
}
```

### Runtime Configuration Check
```cpp
#include "TMC9660LoggingConfig.hpp"

void runtimeCheck() {
    auto config = tmc9660::logging::LoggingConfig::getCurrent();
    
    if (config.enabled) {
        printf("Logging is enabled\n");
        printf("Error logging: %s\n", config.errorEnabled ? "enabled" : "disabled");
        printf("Warning logging: %s\n", config.warningEnabled ? "enabled" : "disabled");
        printf("Info logging: %s\n", config.infoEnabled ? "enabled" : "disabled");
        printf("Debug logging: %s\n", config.debugEnabled ? "enabled" : "disabled");
        printf("Verbose logging: %s\n", config.verboseEnabled ? "enabled" : "disabled");
        
        int highestLevel = config.getHighestEnabledLevel();
        printf("Highest enabled level: %d\n", highestLevel);
    } else {
        printf("Logging is completely disabled\n");
    }
}
```

## Compile-Time Detection

The logging system provides several macros for compile-time detection:

### Check if Logging is Enabled
```cpp
#if TMC9660_LOG_ENABLED
    // This code is only compiled when logging is enabled
    printf("Logging is enabled!\n");
#endif
```

### Check if Specific Level is Enabled
```cpp
#if TMC9660_LOG_LEVEL_ENABLED(2)  // INFO level
    // This code is only compiled when INFO logging is enabled
    printf("Info logging is enabled!\n");
#endif
```

### Check if Level or Higher is Enabled
```cpp
#if TMC9660_LOG_LEVEL_AT_LEAST(2)  // INFO level or higher
    // This code is only compiled for INFO, WARNING, ERROR levels
    printf("Info level or higher logging is enabled!\n");
#endif
```

## Performance Impact

### When Logging is Disabled
- No function calls are made
- No string formatting occurs
- No memory allocation happens
- Zero runtime overhead
- Logging statements are completely removed by the preprocessor

### When Logging is Enabled
- Function calls are made to the logging system
- String formatting occurs
- Memory may be allocated for formatted strings
- Runtime overhead depends on the logging implementation
- Logging statements are compiled into the binary

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

### Debug Build
```cpp
#define TMC9660_LOG_LEVEL_ERROR
#define TMC9660_LOG_LEVEL_WARNING
#define TMC9660_LOG_LEVEL_INFO
#define TMC9660_LOG_LEVEL_DEBUG
```
- All logging except verbose enabled
- Good balance of debugging information and performance

## Troubleshooting

### Logging Not Working
1. Check that the logging configuration is properly defined
2. Verify that the logging level is enabled for the messages you're trying to log
3. Ensure that the logging implementation is properly connected to your platform's logging system

### Compilation Errors
1. Make sure you're including `TMC9660LoggingConfig.hpp` in your source files
2. Check that the logging configuration macros are properly defined
3. Verify that you're using the correct logging macro names

### Performance Issues
1. Consider disabling debug and verbose logging in production builds
2. Use `TMC9660_LOG_DISABLE_ALL` for maximum performance
3. Check that logging is actually being optimized out by examining the compiled binary

## Integration with Platform Logging

The TMC9660 library is designed to integrate with platform-specific logging systems. For example, on ESP32, logging is routed to the ESP-IDF logging system. The logging configuration system works with any platform-specific implementation.

## Best Practices

1. **Use appropriate log levels**: Reserve ERROR for critical issues, WARNING for potential problems, INFO for general information, DEBUG for detailed debugging, and VERBOSE for very detailed debugging.

2. **Configure for your use case**: Use minimal logging in production builds and comprehensive logging in development builds.

3. **Test logging configuration**: Verify that logging is properly enabled/disabled in your builds.

4. **Use conditional compilation**: Take advantage of compile-time detection to optimize your code based on logging availability.

5. **Document your configuration**: Make it clear which logging configuration is used in different builds.

## Examples

See the following files for complete examples:
- `examples/esp32/logging_configs.h` - Example configurations
- `examples/esp32/CMakeLists_logging.txt` - CMake configuration examples
- `inc/TMC9660LoggingConfig.hpp` - Complete API reference