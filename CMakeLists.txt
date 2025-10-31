cmake_minimum_required(VERSION 3.16)

# TMC9660 Driver Component
# This component provides a hardware-agnostic driver for the TMC9660 motor controller IC
# Supports both parameter mode (TMCL) and register mode operations

set(COMPONENT_SRCDIR "src")
set(COMPONENT_INCLUDE_DIRS "inc" "inc/register_mode" "inc/parameter_mode")

# Source files
set(COMPONENT_SRCS
    src/TMC9660.cpp
    src/TMC9660Bootloader.cpp
)

# Component dependencies
set(COMPONENT_REQUIRES
    freertos
    esp_common
    esp_hw_support
    esp_system
    log
    soc
    hal
    esp_driver_gpio
    esp_driver_i2c
    esp_driver_spi
    esp_driver_uart
    bootloader_support
    app_update
    spi_flash
    partition_table
)

# Optional dependencies
set(COMPONENT_PRIV_REQUIRES
    esp_netif
    esp_event
    esp_timer
    lwip
)

# Register the component
idf_component_register(
    SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS ${COMPONENT_INCLUDE_DIRS}
    REQUIRES ${COMPONENT_REQUIRES}
    PRIV_REQUIRES ${COMPONENT_PRIV_REQUIRES}
)

# Set C++ standard (TMC9660 driver uses C++20 for std::span)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)

# Add compiler flags for better C++ support
target_compile_options(${COMPONENT_LIB} PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
    $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++20>
    # Suppress ESP-IDF specific warnings in C++ builds
    $<$<COMPILE_LANGUAGE:CXX>:-Wno-pedantic>
)

# Define component version (optional)
set_property(TARGET ${COMPONENT_LIB} PROPERTY VERSION "1.0.0")

