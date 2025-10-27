/**
 * @file TMC9660LoggingConfig.hpp
 * @brief Compile-time logging configuration for TMC9660 library
 * 
 * This file provides a comprehensive logging configuration system that allows
 * easy determination of whether logging statements are compiled in or optimized out.
 * The system supports different logging levels and can be configured at compile time
 * through preprocessor definitions or CMake options.
 * 
 * @defgroup TMC9660_Logging Logging Configuration
 * @brief Compile-time logging control and configuration
 * 
 * ## Quick Start
 * 
 * ### Enable All Logging (Debug Build)
 * ```cpp
 * #define TMC9660_LOG_LEVEL_ALL
 * ```
 * 
 * ### Enable Specific Log Levels
 * ```cpp
 * #define TMC9660_LOG_LEVEL_ERROR
 * #define TMC9660_LOG_LEVEL_WARNING
 * #define TMC9660_LOG_LEVEL_INFO
 * #define TMC9660_LOG_LEVEL_DEBUG
 * #define TMC9660_LOG_LEVEL_VERBOSE
 * ```
 * 
 * ### Disable All Logging (Release Build)
 * ```cpp
 * #define TMC9660_LOG_DISABLE_ALL
 * ```
 * 
 * ## CMake Integration
 * 
 * ```cmake
 * # Enable all logging
 * set(TMC9660_LOG_LEVEL_ALL ON)
 * 
 * # Or enable specific levels
 * set(TMC9660_LOG_LEVEL_ERROR ON)
 * set(TMC9660_LOG_LEVEL_WARNING ON)
 * set(TMC9660_LOG_LEVEL_INFO ON)
 * 
 * # Or disable all logging
 * set(TMC9660_LOG_DISABLE_ALL ON)
 * ```
 * 
 * ## Compile-Time Detection
 * 
 * The system provides macros to detect at compile time whether logging is enabled:
 * 
 * ```cpp
 * #if TMC9660_LOG_ENABLED
 *     // This code is only compiled when logging is enabled
 *     printf("Logging is enabled!\n");
 * #endif
 * 
 * #if TMC9660_LOG_LEVEL_AT_LEAST(2)  // INFO level or higher
 *     // This code is only compiled for INFO, WARNING, ERROR levels
 *     printf("Info level logging is enabled!\n");
 * #endif
 * ```
 */

#pragma once

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace tmc9660 {
namespace logging {

/**
 * @brief Log levels for TMC9660 library
 * 
 * These levels correspond to the ESP-IDF logging levels and provide
 * a consistent interface across different platforms.
 */
enum class LogLevel : int {
    ERROR = 0,      ///< Error messages only
    WARNING = 1,    ///< Warning and error messages
    INFO = 2,       ///< Info, warning, and error messages
    DEBUG = 3,      ///< Debug, info, warning, and error messages
    VERBOSE = 4     ///< All messages including verbose debug
};

} // namespace logging
} // namespace tmc9660

// ============================================================================
// COMPILE-TIME LOGGING CONFIGURATION
// ============================================================================

// Check for CMake-defined logging configuration
#ifdef TMC9660_LOG_LEVEL_ALL
    #define TMC9660_LOG_ENABLED 1
    #define TMC9660_LOG_LEVEL_ERROR 1
    #define TMC9660_LOG_LEVEL_WARNING 1
    #define TMC9660_LOG_LEVEL_INFO 1
    #define TMC9660_LOG_LEVEL_DEBUG 1
    #define TMC9660_LOG_LEVEL_VERBOSE 1
#endif

#ifdef TMC9660_LOG_DISABLE_ALL
    #define TMC9660_LOG_ENABLED 0
    #define TMC9660_LOG_LEVEL_ERROR 0
    #define TMC9660_LOG_LEVEL_WARNING 0
    #define TMC9660_LOG_LEVEL_INFO 0
    #define TMC9660_LOG_LEVEL_DEBUG 0
    #define TMC9660_LOG_LEVEL_VERBOSE 0
#endif

// Individual log level definitions (if not already defined)
#ifndef TMC9660_LOG_LEVEL_ERROR
    #define TMC9660_LOG_LEVEL_ERROR 1
#endif

#ifndef TMC9660_LOG_LEVEL_WARNING
    #define TMC9660_LOG_LEVEL_WARNING 1
#endif

#ifndef TMC9660_LOG_LEVEL_INFO
    #define TMC9660_LOG_LEVEL_INFO 1
#endif

#ifndef TMC9660_LOG_LEVEL_DEBUG
    #define TMC9660_LOG_LEVEL_DEBUG 0
#endif

#ifndef TMC9660_LOG_LEVEL_VERBOSE
    #define TMC9660_LOG_LEVEL_VERBOSE 0
#endif

// Overall logging enable/disable
#ifndef TMC9660_LOG_ENABLED
    #define TMC9660_LOG_ENABLED 1
#endif

// ============================================================================
// LOGGING MACROS
// ============================================================================

/**
 * @brief Check if logging is enabled at compile time
 * 
 * This macro evaluates to 1 if logging is enabled, 0 if disabled.
 * Useful for conditional compilation based on logging availability.
 */
#define TMC9660_LOG_ENABLED TMC9660_LOG_ENABLED

/**
 * @brief Check if a specific log level is enabled at compile time
 * 
 * @param level Log level to check (0=ERROR, 1=WARNING, 2=INFO, 3=DEBUG, 4=VERBOSE)
 * @return 1 if the level is enabled, 0 if disabled
 * 
 * Example:
 * ```cpp
 * #if TMC9660_LOG_LEVEL_AT_LEAST(2)  // INFO level or higher
 *     printf("Info logging is enabled!\n");
 * #endif
 * ```
 */
#define TMC9660_LOG_LEVEL_AT_LEAST(level) \
    (TMC9660_LOG_ENABLED && ( \
        (level) == 0 ? TMC9660_LOG_LEVEL_ERROR : \
        (level) == 1 ? TMC9660_LOG_LEVEL_WARNING : \
        (level) == 2 ? TMC9660_LOG_LEVEL_INFO : \
        (level) == 3 ? TMC9660_LOG_LEVEL_DEBUG : \
        (level) == 4 ? TMC9660_LOG_LEVEL_VERBOSE : 0))

/**
 * @brief Check if a specific log level is enabled
 * 
 * @param level Log level to check (0=ERROR, 1=WARNING, 2=INFO, 3=DEBUG, 4=VERBOSE)
 * @return 1 if the level is enabled, 0 if disabled
 */
#define TMC9660_LOG_LEVEL_ENABLED(level) \
    (TMC9660_LOG_ENABLED && ( \
        (level) == 0 ? TMC9660_LOG_LEVEL_ERROR : \
        (level) == 1 ? TMC9660_LOG_LEVEL_WARNING : \
        (level) == 2 ? TMC9660_LOG_LEVEL_INFO : \
        (level) == 3 ? TMC9660_LOG_LEVEL_DEBUG : \
        (level) == 4 ? TMC9660_LOG_LEVEL_VERBOSE : 0))

// ============================================================================
// CONDITIONAL LOGGING MACROS
// ============================================================================

/**
 * @brief Log an error message (level 0)
 * 
 * This macro is completely removed by the preprocessor when logging is disabled
 * or when error logging is disabled, resulting in zero runtime overhead.
 * 
 * @param tag Log tag for categorization
 * @param format printf-style format string
 * @param ... Variable arguments for format string
 */
#if TMC9660_LOG_LEVEL_ENABLED(0)
    #define TMC9660_LOG_ERROR(tag, format, ...) \
        tmc9660::logging::logMessage(0, tag, format, ##__VA_ARGS__)
#else
    #define TMC9660_LOG_ERROR(tag, format, ...) do {} while(0)
#endif

/**
 * @brief Log a warning message (level 1)
 */
#if TMC9660_LOG_LEVEL_ENABLED(1)
    #define TMC9660_LOG_WARNING(tag, format, ...) \
        tmc9660::logging::logMessage(1, tag, format, ##__VA_ARGS__)
#else
    #define TMC9660_LOG_WARNING(tag, format, ...) do {} while(0)
#endif

/**
 * @brief Log an info message (level 2)
 */
#if TMC9660_LOG_LEVEL_ENABLED(2)
    #define TMC9660_LOG_INFO(tag, format, ...) \
        tmc9660::logging::logMessage(2, tag, format, ##__VA_ARGS__)
#else
    #define TMC9660_LOG_INFO(tag, format, ...) do {} while(0)
#endif

/**
 * @brief Log a debug message (level 3)
 */
#if TMC9660_LOG_LEVEL_ENABLED(3)
    #define TMC9660_LOG_DEBUG(tag, format, ...) \
        tmc9660::logging::logMessage(3, tag, format, ##__VA_ARGS__)
#else
    #define TMC9660_LOG_DEBUG(tag, format, ...) do {} while(0)
#endif

/**
 * @brief Log a verbose message (level 4)
 */
#if TMC9660_LOG_LEVEL_ENABLED(4)
    #define TMC9660_LOG_VERBOSE(tag, format, ...) \
        tmc9660::logging::logMessage(4, tag, format, ##__VA_ARGS__)
#else
    #define TMC9660_LOG_VERBOSE(tag, format, ...) do {} while(0)
#endif

// ============================================================================
// COMPATIBILITY MACROS
// ============================================================================

/**
 * @brief Legacy logDebug macro for backward compatibility
 * 
 * This macro maintains compatibility with the existing logDebug function
 * while using the new logging system.
 * 
 * @param level Log level (0=Error, 1=Warning, 2=Info, 3=Debug, 4=Verbose)
 * @param tag Log tag for categorization
 * @param format printf-style format string
 * @param ... Variable arguments for format string
 */
#if TMC9660_LOG_ENABLED
    #define TMC9660_LOG_DEBUG_LEGACY(level, tag, format, ...) \
        tmc9660::logging::logMessage(level, tag, format, ##__VA_ARGS__)
#else
    #define TMC9660_LOG_DEBUG_LEGACY(level, tag, format, ...) do {} while(0)
#endif

// ============================================================================
// LOGGING IMPLEMENTATION
// ============================================================================

namespace tmc9660 {
namespace logging {

/**
 * @brief Default logging implementation
 * 
 * This function provides a default logging implementation that can be
 * overridden by platform-specific implementations. The default implementation
 * uses printf for output.
 * 
 * @param level Log level (0=Error, 1=Warning, 2=Info, 3=Debug, 4=Verbose)
 * @param tag Log tag for categorization
 * @param format printf-style format string
 * @param ... Variable arguments for format string
 */
inline void logMessage(int level, const char* tag, const char* format, ...) noexcept {
    // Only log if the level is enabled
    if (!TMC9660_LOG_LEVEL_AT_LEAST(level)) {
        return;
    }
    
    // Format the log message
    va_list args;
    va_start(args, format);
    
    // Get level name
    const char* levelName = "UNKNOWN";
    switch (level) {
        case 0: levelName = "ERROR"; break;
        case 1: levelName = "WARNING"; break;
        case 2: levelName = "INFO"; break;
        case 3: levelName = "DEBUG"; break;
        case 4: levelName = "VERBOSE"; break;
    }
    
    // Print timestamp, level, tag, and message
    printf("[TMC9660] %s (%s): ", levelName, tag);
    vprintf(format, args);
    
    // Ensure newline
    if (format && strlen(format) > 0 && format[strlen(format) - 1] != '\n') {
        printf("\n");
    }
    
    va_end(args);
}

/**
 * @brief Logging configuration information
 * 
 * This structure contains information about the current logging configuration
 * and can be used to determine what logging features are available at runtime.
 */
struct LoggingConfig {
    bool enabled;           ///< Whether logging is enabled
    bool errorEnabled;      ///< Whether error logging is enabled
    bool warningEnabled;    ///< Whether warning logging is enabled
    bool infoEnabled;       ///< Whether info logging is enabled
    bool debugEnabled;      ///< Whether debug logging is enabled
    bool verboseEnabled;    ///< Whether verbose logging is enabled
    
    /**
     * @brief Get the current logging configuration
     * @return LoggingConfig structure with current settings
     */
    static constexpr LoggingConfig getCurrent() noexcept {
        return LoggingConfig{
            .enabled = TMC9660_LOG_ENABLED,
            .errorEnabled = TMC9660_LOG_LEVEL_ERROR,
            .warningEnabled = TMC9660_LOG_LEVEL_WARNING,
            .infoEnabled = TMC9660_LOG_LEVEL_INFO,
            .debugEnabled = TMC9660_LOG_LEVEL_DEBUG,
            .verboseEnabled = TMC9660_LOG_LEVEL_VERBOSE
        };
    }
    
    /**
     * @brief Check if a specific log level is enabled
     * @param level Log level to check
     * @return true if the level is enabled
     */
    constexpr bool isLevelEnabled(int level) const noexcept {
        switch (level) {
            case 0: return errorEnabled;
            case 1: return warningEnabled;
            case 2: return infoEnabled;
            case 3: return debugEnabled;
            case 4: return verboseEnabled;
            default: return false;
        }
    }
    
    /**
     * @brief Get the highest enabled log level
     * @return Highest enabled log level (0-4), or -1 if logging is disabled
     */
    constexpr int getHighestEnabledLevel() const noexcept {
        if (!enabled) return -1;
        if (verboseEnabled) return 4;
        if (debugEnabled) return 3;
        if (infoEnabled) return 2;
        if (warningEnabled) return 1;
        if (errorEnabled) return 0;
        return -1;
    }
};

} // namespace logging
} // namespace tmc9660

// ============================================================================
// COMPILE-TIME ASSERTIONS
// ============================================================================

// Ensure that at least one logging level is defined
#if !TMC9660_LOG_LEVEL_ERROR && !TMC9660_LOG_LEVEL_WARNING && !TMC9660_LOG_LEVEL_INFO && !TMC9660_LOG_LEVEL_DEBUG && !TMC9660_LOG_LEVEL_VERBOSE
    #error "At least one logging level must be enabled. Define TMC9660_LOG_LEVEL_ERROR or higher."
#endif

// ============================================================================
// USAGE EXAMPLES
// ============================================================================

/*
 * Example 1: Basic usage
 * ```cpp
 * #include "TMC9660LoggingConfig.hpp"
 * 
 * void exampleFunction() {
 *     TMC9660_LOG_INFO("MyTag", "This is an info message: %d", 42);
 *     TMC9660_LOG_DEBUG("MyTag", "This debug message may be optimized out");
 * }
 * ```
 * 
 * Example 2: Conditional compilation
 * ```cpp
 * #if TMC9660_LOG_ENABLED
 *     printf("Logging is enabled in this build\n");
 * #endif
 * 
 * #if TMC9660_LOG_LEVEL_AT_LEAST(2)
 *     printf("Info level logging is available\n");
 * #endif
 * ```
 * 
 * Example 3: Runtime configuration check
 * ```cpp
 * auto config = tmc9660::logging::LoggingConfig::getCurrent();
 * if (config.debugEnabled) {
 *     printf("Debug logging is enabled\n");
 * }
 * ```
 * 
 * Example 4: CMake configuration
 * ```cmake
 * # In CMakeLists.txt
 * option(TMC9660_LOG_LEVEL_ALL "Enable all logging levels" OFF)
 * option(TMC9660_LOG_LEVEL_DEBUG "Enable debug logging" OFF)
 * 
 * if(TMC9660_LOG_LEVEL_ALL)
 *     target_compile_definitions(my_target PRIVATE TMC9660_LOG_LEVEL_ALL)
 * elseif(TMC9660_LOG_LEVEL_DEBUG)
 *     target_compile_definitions(my_target PRIVATE TMC9660_LOG_LEVEL_DEBUG)
 * endif()
 * ```
 */