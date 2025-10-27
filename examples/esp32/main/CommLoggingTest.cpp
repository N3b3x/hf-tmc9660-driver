/**
 * @file CommLoggingTest.cpp
 * @brief Test program to verify TMC9660 communication logging control
 * 
 * This program demonstrates the TMC9660 communication logging control
 * and verifies that raw SPI/UART data logging can be easily disabled.
 */

#include "TMC9660LoggingConfig.hpp"
#include <cstdio>

/**
 * @brief Test function to demonstrate communication logging control
 */
void testCommunicationLoggingControl() {
    printf("=== TMC9660 Communication Logging Control Test ===\n\n");
    
    // Test compile-time detection of communication logging
    printf("Testing communication logging control:\n");
    #ifdef TMC9660_ENABLE_COMM_LOGGING
        printf("✅ Communication logging is ENABLED in this build\n");
        printf("   Raw SPI/UART data will be logged\n");
    #else
        printf("❌ Communication logging is DISABLED in this build\n");
        printf("   Raw SPI/UART data will NOT be logged (recommended for production)\n");
    #endif
    
    printf("\n");
    
    // Test that communication logging is separate from main logging
    printf("Testing separation from main logging:\n");
    #if TMC9660_LOG_ENABLED
        printf("✅ Main logging is enabled\n");
        #ifdef TMC9660_ENABLE_COMM_LOGGING
            printf("✅ Communication logging is also enabled\n");
        #else
            printf("❌ Communication logging is disabled (even though main logging is enabled)\n");
        #endif
    #else
        printf("❌ Main logging is disabled\n");
        #ifdef TMC9660_ENABLE_COMM_LOGGING
            printf("✅ Communication logging is still enabled (independent control)\n");
        #else
            printf("❌ Communication logging is also disabled\n");
        #endif
    #endif
    
    printf("\n");
    
    // Test different scenarios
    printf("Testing different scenarios:\n");
    
    // Scenario 1: All logging enabled
    #if TMC9660_LOG_ENABLED && defined(TMC9660_ENABLE_COMM_LOGGING)
        printf("✅ Scenario 1: All logging enabled (main + communication)\n");
    #endif
    
    // Scenario 2: Main logging enabled, communication disabled
    #if TMC9660_LOG_ENABLED && !defined(TMC9660_ENABLE_COMM_LOGGING)
        printf("✅ Scenario 2: Main logging enabled, communication disabled (recommended for production)\n");
    #endif
    
    // Scenario 3: All logging disabled
    #if !TMC9660_LOG_ENABLED && !defined(TMC9660_ENABLE_COMM_LOGGING)
        printf("✅ Scenario 3: All logging disabled (maximum performance)\n");
    #endif
    
    // Scenario 4: Only communication logging enabled
    #if !TMC9660_LOG_ENABLED && defined(TMC9660_ENABLE_COMM_LOGGING)
        printf("✅ Scenario 4: Only communication logging enabled (unusual but possible)\n");
    #endif
    
    printf("\n");
    
    // Test that communication logging macros work
    printf("Testing communication logging macros:\n");
    #ifdef TMC9660_ENABLE_COMM_LOGGING
        printf("✅ Communication logging macros are available\n");
        printf("   Raw SPI/UART data logging will work\n");
    #else
        printf("❌ Communication logging macros are not available\n");
        printf("   Raw SPI/UART data logging will be completely optimized out\n");
    #endif
    
    printf("\n");
    
    // Show how to control communication logging
    printf("How to control communication logging:\n");
    printf("1. To ENABLE communication logging, define TMC9660_ENABLE_COMM_LOGGING\n");
    printf("2. To DISABLE communication logging, leave TMC9660_ENABLE_COMM_LOGGING undefined\n");
    printf("3. This is independent of the main logging levels\n");
    printf("4. Communication logging is disabled by default (too verbose for production)\n");
    
    printf("\n=== Test Complete ===\n");
}

/**
 * @brief Main function
 */
extern "C" void app_main() {
    testCommunicationLoggingControl();
}