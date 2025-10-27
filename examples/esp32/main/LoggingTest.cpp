/**
 * @file LoggingTest.cpp
 * @brief Test program to verify TMC9660 logging configuration
 * 
 * This program demonstrates the TMC9660 logging configuration system
 * and verifies that logging statements are properly compiled in or
 * optimized out based on the configuration.
 */

#include "TMC9660LoggingConfig.hpp"
#include <cstdio>

/**
 * @brief Test function to demonstrate logging configuration
 */
void testLoggingConfiguration() {
    printf("=== TMC9660 Logging Configuration Test ===\n\n");
    
    // Test basic logging macros
    printf("Testing basic logging macros:\n");
    TMC9660_LOG_ERROR("TEST", "This is an error message: %d", 42);
    TMC9660_LOG_WARNING("TEST", "This is a warning message");
    TMC9660_LOG_INFO("TEST", "This is an info message: %s", "hello");
    TMC9660_LOG_DEBUG("TEST", "This debug message may be optimized out");
    TMC9660_LOG_VERBOSE("TEST", "This verbose message may be optimized out");
    printf("\n");
    
    // Test compile-time detection
    printf("Testing compile-time detection:\n");
    #if TMC9660_LOG_ENABLED
        printf("✅ Logging is enabled in this build\n");
    #else
        printf("❌ Logging is disabled in this build\n");
    #endif
    
    #if TMC9660_LOG_LEVEL_AT_LEAST(0)  // ERROR level or higher
        printf("✅ Error level or higher logging is available\n");
    #else
        printf("❌ Error level or higher logging is not available\n");
    #endif
    
    #if TMC9660_LOG_LEVEL_AT_LEAST(1)  // WARNING level or higher
        printf("✅ Warning level or higher logging is available\n");
    #else
        printf("❌ Warning level or higher logging is not available\n");
    #endif
    
    #if TMC9660_LOG_LEVEL_AT_LEAST(2)  // INFO level or higher
        printf("✅ Info level or higher logging is available\n");
    #else
        printf("❌ Info level or higher logging is not available\n");
    #endif
    
    #if TMC9660_LOG_LEVEL_AT_LEAST(3)  // DEBUG level or higher
        printf("✅ Debug level or higher logging is available\n");
    #else
        printf("❌ Debug level or higher logging is not available\n");
    #endif
    
    #if TMC9660_LOG_LEVEL_AT_LEAST(4)  // VERBOSE level or higher
        printf("✅ Verbose level or higher logging is available\n");
    #else
        printf("❌ Verbose level or higher logging is not available\n");
    #endif
    
    printf("\n");
    
    // Test specific level detection
    printf("Testing specific level detection:\n");
    #if TMC9660_LOG_LEVEL_ENABLED(0)  // ERROR level
        printf("✅ Error level logging is specifically enabled\n");
    #else
        printf("❌ Error level logging is not specifically enabled\n");
    #endif
    
    #if TMC9660_LOG_LEVEL_ENABLED(1)  // WARNING level
        printf("✅ Warning level logging is specifically enabled\n");
    #else
        printf("❌ Warning level logging is not specifically enabled\n");
    #endif
    
    #if TMC9660_LOG_LEVEL_ENABLED(2)  // INFO level
        printf("✅ Info level logging is specifically enabled\n");
    #else
        printf("❌ Info level logging is not specifically enabled\n");
    #endif
    
    #if TMC9660_LOG_LEVEL_ENABLED(3)  // DEBUG level
        printf("✅ Debug level logging is specifically enabled\n");
    #else
        printf("❌ Debug level logging is not specifically enabled\n");
    #endif
    
    #if TMC9660_LOG_LEVEL_ENABLED(4)  // VERBOSE level
        printf("✅ Verbose level logging is specifically enabled\n");
    #else
        printf("❌ Verbose level logging is not specifically enabled\n");
    #endif
    
    printf("\n");
    
    // Test runtime configuration
    printf("Testing runtime configuration:\n");
    auto config = tmc9660::logging::LoggingConfig::getCurrent();
    
    printf("Logging enabled: %s\n", config.enabled ? "yes" : "no");
    printf("Error logging: %s\n", config.errorEnabled ? "enabled" : "disabled");
    printf("Warning logging: %s\n", config.warningEnabled ? "enabled" : "disabled");
    printf("Info logging: %s\n", config.infoEnabled ? "enabled" : "disabled");
    printf("Debug logging: %s\n", config.debugEnabled ? "enabled" : "disabled");
    printf("Verbose logging: %s\n", config.verboseEnabled ? "enabled" : "disabled");
    
    int highestLevel = config.getHighestEnabledLevel();
    if (highestLevel >= 0) {
        printf("Highest enabled level: %d\n", highestLevel);
    } else {
        printf("No logging levels are enabled\n");
    }
    
    printf("\n");
    
    // Test conditional compilation
    printf("Testing conditional compilation:\n");
    #if TMC9660_LOG_ENABLED
        printf("This code block is only compiled when logging is enabled\n");
        #if TMC9660_LOG_LEVEL_AT_LEAST(2)
            printf("This code block is only compiled for INFO level or higher\n");
        #endif
    #else
        printf("This code block is only compiled when logging is disabled\n");
    #endif
    
    printf("\n=== Test Complete ===\n");
}

/**
 * @brief Main function
 */
extern "C" void app_main() {
    testLoggingConfiguration();
}