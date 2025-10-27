/**
 * @file logging_configs.h
 * @brief Example logging configurations for TMC9660 library
 * 
 * This file demonstrates different ways to configure logging for the TMC9660 library.
 * Copy the relevant configuration to your project or use as a reference.
 */

#pragma once

// ============================================================================
// CONFIGURATION 1: DEBUG BUILD (All Logging Enabled)
// ============================================================================
// Uncomment this section for maximum logging during development
/*
#define TMC9660_LOG_LEVEL_ALL
#define TMC9660_ENABLE_COMM_LOGGING  // Enable raw SPI/UART data logging
*/

// ============================================================================
// CONFIGURATION 2: RELEASE BUILD (No Logging)
// ============================================================================
// Uncomment this section for production builds with no logging overhead
/*
#define TMC9660_LOG_DISABLE_ALL
*/

// ============================================================================
// CONFIGURATION 3: MINIMAL LOGGING (Errors Only)
// ============================================================================
// Uncomment this section for minimal logging (errors only)
/*
#define TMC9660_LOG_LEVEL_ERROR
*/

// ============================================================================
// CONFIGURATION 4: STANDARD LOGGING (Errors + Warnings + Info)
// ============================================================================
// Uncomment this section for standard logging (recommended for most applications)
/*
#define TMC9660_LOG_LEVEL_ERROR
#define TMC9660_LOG_LEVEL_WARNING
#define TMC9660_LOG_LEVEL_INFO
// Communication logging disabled by default (too verbose for production)
*/

// ============================================================================
// CONFIGURATION 5: DETAILED LOGGING (All except Verbose)
// ============================================================================
// Uncomment this section for detailed logging without verbose output
/*
#define TMC9660_LOG_LEVEL_ERROR
#define TMC9660_LOG_LEVEL_WARNING
#define TMC9660_LOG_LEVEL_INFO
#define TMC9660_LOG_LEVEL_DEBUG
*/

// ============================================================================
// CONFIGURATION 6: MAXIMUM LOGGING (All Levels)
// ============================================================================
// Uncomment this section for maximum logging including verbose output
/*
#define TMC9660_LOG_LEVEL_ERROR
#define TMC9660_LOG_LEVEL_WARNING
#define TMC9660_LOG_LEVEL_INFO
#define TMC9660_LOG_LEVEL_DEBUG
#define TMC9660_LOG_LEVEL_VERBOSE
*/

// ============================================================================
// CONFIGURATION 7: CUSTOM LOGGING (Selective Levels)
// ============================================================================
// Uncomment and modify this section for custom logging configuration
/*
#define TMC9660_LOG_LEVEL_ERROR      // Always enable errors
#define TMC9660_LOG_LEVEL_WARNING    // Enable warnings
// #define TMC9660_LOG_LEVEL_INFO    // Disable info messages
#define TMC9660_LOG_LEVEL_DEBUG      // Enable debug messages
// #define TMC9660_LOG_LEVEL_VERBOSE // Disable verbose messages
*/

// ============================================================================
// CONFIGURATION 8: COMMUNICATION DEBUGGING
// ============================================================================
// Uncomment this section for debugging SPI/UART communication issues
/*
#define TMC9660_LOG_LEVEL_ERROR
#define TMC9660_LOG_LEVEL_WARNING
#define TMC9660_LOG_LEVEL_INFO
#define TMC9660_ENABLE_COMM_LOGGING  // Enable raw SPI/UART data logging
*/

// ============================================================================
// USAGE EXAMPLES
// ============================================================================

// Example 1: Basic logging usage
/*
#include "TMC9660LoggingConfig.hpp"

void exampleFunction() {
    TMC9660_LOG_ERROR("MyTag", "This is an error message: %d", 42);
    TMC9660_LOG_WARNING("MyTag", "This is a warning message");
    TMC9660_LOG_INFO("MyTag", "This is an info message: %s", "hello");
    TMC9660_LOG_DEBUG("MyTag", "This debug message may be optimized out");
    TMC9660_LOG_VERBOSE("MyTag", "This verbose message may be optimized out");
}
*/

// Example 2: Conditional compilation based on logging availability
/*
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
*/

// Example 3: Runtime configuration check
/*
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
*/

// ============================================================================
// COMPILER-SPECIFIC CONFIGURATIONS
// ============================================================================

// For GCC/Clang with -D flags:
// gcc -DTMC9660_LOG_LEVEL_ERROR -DTMC9660_LOG_LEVEL_WARNING -DTMC9660_LOG_LEVEL_INFO

// For Visual Studio with preprocessor definitions:
// TMC9660_LOG_LEVEL_ERROR;TMC9660_LOG_LEVEL_WARNING;TMC9660_LOG_LEVEL_INFO

// For Arduino IDE, add to platform.txt or boards.txt:
// compiler.cpp.extra_flags=-DTMC9660_LOG_LEVEL_ERROR -DTMC9660_LOG_LEVEL_WARNING -DTMC9660_LOG_LEVEL_INFO

// ============================================================================
// PERFORMANCE IMPACT
// ============================================================================

// When logging is disabled at compile time:
// - No function calls are made
// - No string formatting occurs
// - No memory allocation happens
// - Zero runtime overhead
// - Logging statements are completely removed by the preprocessor

// When logging is enabled:
// - Function calls are made to the logging system
// - String formatting occurs
// - Memory may be allocated for formatted strings
// - Runtime overhead depends on the logging implementation
// - Logging statements are compiled into the binary