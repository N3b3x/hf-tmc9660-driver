#===============================================================================
# TMC9660 Driver - Build Settings
# Shared variables for target name, includes, sources, and dependencies.
# This file is the SINGLE SOURCE OF TRUTH for the driver version.
#===============================================================================

include_guard(GLOBAL)

set(HF_TMC9660_TARGET_NAME "hf_tmc9660")

#===============================================================================
# Versioning (single source of truth)
#===============================================================================
set(HF_TMC9660_VERSION_MAJOR 1)
set(HF_TMC9660_VERSION_MINOR 0)
set(HF_TMC9660_VERSION_PATCH 0)
set(HF_TMC9660_VERSION "${HF_TMC9660_VERSION_MAJOR}.${HF_TMC9660_VERSION_MINOR}.${HF_TMC9660_VERSION_PATCH}")

#===============================================================================
# Generate version header from template (into build directory)
#===============================================================================
set(HF_TMC9660_VERSION_TEMPLATE "${CMAKE_CURRENT_LIST_DIR}/../inc/tmc9660_version.h.in")
set(HF_TMC9660_VERSION_HEADER_DIR "${CMAKE_CURRENT_BINARY_DIR}/hf_tmc9660_generated")
set(HF_TMC9660_VERSION_HEADER     "${HF_TMC9660_VERSION_HEADER_DIR}/tmc9660_version.h")

file(MAKE_DIRECTORY "${HF_TMC9660_VERSION_HEADER_DIR}")

if(EXISTS "${HF_TMC9660_VERSION_TEMPLATE}")
    configure_file(
        "${HF_TMC9660_VERSION_TEMPLATE}"
        "${HF_TMC9660_VERSION_HEADER}"
        @ONLY
    )
    message(STATUS "TMC9660 driver v${HF_TMC9660_VERSION} — generated tmc9660_version.h in ${HF_TMC9660_VERSION_HEADER_DIR}")
else()
    message(WARNING "tmc9660_version.h.in not found at ${HF_TMC9660_VERSION_TEMPLATE}")
endif()

#===============================================================================
# Public include directories
#===============================================================================
set(HF_TMC9660_PUBLIC_INCLUDE_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../inc"
    "${CMAKE_CURRENT_LIST_DIR}/../inc/register_mode"
    "${CMAKE_CURRENT_LIST_DIR}/../inc/parameter_mode"
    "${HF_TMC9660_VERSION_HEADER_DIR}"
)

#===============================================================================
# Source files (bootloader logic)
#===============================================================================
set(HF_TMC9660_SOURCE_FILES
    "${CMAKE_CURRENT_LIST_DIR}/../src/bootloader/tmc9660_bootloader.cpp"
)

#===============================================================================
# ESP-IDF component dependencies
#===============================================================================
set(HF_TMC9660_IDF_REQUIRES driver freertos)
