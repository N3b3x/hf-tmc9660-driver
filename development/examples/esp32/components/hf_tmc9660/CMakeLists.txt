# =============================================================================
# TMC9660 Driver Component - CMakeLists.txt
# =============================================================================
# This component wraps the hardware-agnostic TMC9660 driver for ESP-IDF.
# The TMC9660 has compiled source files (bootloader, core driver).
# =============================================================================

cmake_minimum_required(VERSION 3.16)

# =============================================================================
# Path Configuration
# =============================================================================
# Set the source root path relative to this component directory
set(SRC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../src")
set(INC_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../../inc")

message(STATUS "TMC9660 Driver - ESP32 Component Configuration")
message(STATUS "Source Root: ${SRC_ROOT}")

# =============================================================================
# Source File Discovery and Validation
# =============================================================================
# TMC9660 has compiled source files
set(DRIVER_SRCS
    "${SRC_ROOT}/tmc9660.cpp"
    "${SRC_ROOT}/bootloader/tmc9660_bootloader.cpp"
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
    "${INC_ROOT}"                           # Main driver headers
    "${INC_ROOT}/register_mode"             # Register mode specific headers
    "${INC_ROOT}/parameter_mode"            # Parameter mode specific headers
)

# =============================================================================
# Component Dependencies
# =============================================================================
# Define required ESP-IDF components for the driver
set(DRIVER_REQUIRES
    driver              # ESP-IDF driver framework
    freertos            # FreeRTOS support
)

# =============================================================================
# Component Registration
# =============================================================================
# Register the driver component with ESP-IDF build system
idf_component_register(
    SRCS ${EXISTING_SRCS}                 # Source files (validated above)
    INCLUDE_DIRS ${DRIVER_INCLUDE_DIRS}   # Include directories defined above
    REQUIRES ${DRIVER_REQUIRES}           # Component dependencies defined above
)

# =============================================================================
# Compiler Configuration
# =============================================================================
# Set C++ standard to C++20 (required for std::span and modern C++ features)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)
