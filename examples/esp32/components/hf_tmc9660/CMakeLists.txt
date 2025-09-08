# =============================================================================
# TMC9660 Driver - ESP-IDF Component CMakeLists.txt
# =============================================================================
# This CMakeLists.txt configures the TMC9660 driver as an ESP-IDF component.
# It provides a hardware-agnostic driver for the TMC9660 motor controller IC
# with support for both parameter mode and register mode operations.
#
# Features:
# - Hardware-agnostic design (no target-specific definitions)
# - C++20 support for modern features like std::span
# - Comprehensive file discovery and validation
# - Optimized build settings for embedded systems
# =============================================================================

cmake_minimum_required(VERSION 3.22)

# =============================================================================
# Path Configuration
# =============================================================================
# Set the source root path relative to this component directory
set(SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../")
set(INC_ROOT "${SRC_ROOT}")

message(STATUS "TMC9660 Driver - ESP32 Component Configuration")
message(STATUS "Source Root: ${SRC_ROOT}")

# =============================================================================
# Source File Discovery and Validation
# =============================================================================
# List all potential driver source files
set(DRIVER_SRCS
    "${SRC_ROOT}/src/TMC9660.cpp"
    "${SRC_ROOT}/src/TMC9660Bootloader.cpp"
)

# Check which source files actually exist and add them
set(EXISTING_SRCS "")
foreach(src_file ${DRIVER_SRCS})
    if(EXISTS "${src_file}")
        list(APPEND EXISTING_SRCS "${src_file}")
        message(STATUS "Found source: ${src_file}")
    else()
        message(WARNING "Source file not found: ${src_file}")
    endif()
endforeach()

# =============================================================================
# Include Directories Configuration
# =============================================================================
# Define include directories for the driver component
set(DRIVER_INCLUDE_DIRS
    "${INC_ROOT}/inc"                       # Main driver headers
    "${INC_ROOT}/inc/register_mode"         # Register mode specific headers
    "${INC_ROOT}/inc/parameter_mode"        # Parameter mode specific headers
)

# =============================================================================
# Component Dependencies
# =============================================================================
# Define required ESP-IDF components for the driver
set(DRIVER_REQUIRES
    # ESP-IDF core components
    driver              # GPIO, I2C, SPI, UART drivers
    esp_timer           # Timer support
    freertos            # RTOS support
    esp_event           # Event system
    esp_system          # System utilities
    esp_hw_support      # Hardware support
    hal                 # Hardware abstraction
    soc                 # SoC definitions
    log                 # Logging
    
    # Driver components
    esp_driver_gpio
    esp_driver_i2c
    esp_driver_spi
    esp_driver_uart
    
    # Additional components for full functionality
    esp_common          # Common ESP32 utilities
    bootloader_support  # Bootloader support utilities
    app_update          # Application update support
    spi_flash           # SPI flash support
    partition_table     # Partition table support
)

# =============================================================================
# Component Registration
# =============================================================================
# Register the driver component with ESP-IDF build system
idf_component_register(
    SRCS ${EXISTING_SRCS}                 # Source files (validated above)
    INCLUDE_DIRS ${DRIVER_INCLUDE_DIRS}   # Include directories defined above
    REQUIRES ${DRIVER_REQUIRES}           # Component dependencies defined above
    PRIV_REQUIRES
        # Private requirements (internal use only)
        esp_pm              # Power management (private)
        esp_phy
)

# =============================================================================
# Compiler Configuration
# =============================================================================
# Set C++ standard to C++20 (required for std::span and modern C++ features)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)

# =============================================================================
# Compiler Flags and Optimization
# =============================================================================
# General compiler flags for optimization and debugging
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall                    # Enable all warnings
    -Wextra                  # Enable extra warnings
    -O2                      # Optimize for performance
    -g                       # Include debug information
    -ffunction-sections      # Place each function in its own section
    -fdata-sections          # Place each data item in its own section
    -Wno-pedantic           # Disable pedantic warnings
)

# Add specific flags for C++ files optimized for embedded systems
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++20>      # C++20 standard with GNU extensions
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>         # Disable RTTI for smaller binary size
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>   # Disable exceptions for embedded systems
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-error=pedantic> # Allow pedantic warnings without errors
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-pedantic>     # Disable pedantic warnings
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-variadic-macros> # Disable variadic macro warnings
)

# =============================================================================
# Link Time Optimization (LTO)
# =============================================================================
# Enable LTO for better performance when optimization is enabled
if(CONFIG_COMPILER_OPTIMIZATION_SIZE OR CONFIG_COMPILER_OPTIMIZATION_PERF)
    target_compile_options(${COMPONENT_LIB} PRIVATE -flto)
    target_link_options(${COMPONENT_LIB} PRIVATE -flto)
endif()

# =============================================================================
# Build Summary and Status Messages
# =============================================================================
# Display final configuration summary
list(LENGTH EXISTING_SRCS NUM_SOURCES)
message(STATUS "=== TMC9660 Driver Component Summary ===")
message(STATUS "Component configured successfully")
message(STATUS "Included ${NUM_SOURCES} source files")
message(STATUS "C++ Standard: C++20")
message(STATUS "Hardware-agnostic driver")
message(STATUS "Optimization: LTO enabled (if configured)")
message(STATUS "===============================================")
