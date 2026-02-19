# =============================================================================
# hf_tmc9660_build_settings.cmake — Single source of truth
# =============================================================================
# This file defines ALL build settings for the TMC9660 driver.
# It is consumed by:
#   1. The root CMakeLists.txt   (desktop / non-IDF builds)
#   2. The ESP-IDF component wrapper (examples/esp32/components/…/CMakeLists.txt)
#
# Prerequisites:
#   HF_TMC9660_ROOT must be set to the driver's repository root before
#   including this file.
# =============================================================================
cmake_minimum_required(VERSION 3.16)

# ── Guard ────────────────────────────────────────────────────────────────────
if(_HF_TMC9660_BUILD_SETTINGS_INCLUDED)
  return()
endif()
set(_HF_TMC9660_BUILD_SETTINGS_INCLUDED TRUE)

# ── Root validation ──────────────────────────────────────────────────────────
if(NOT DEFINED HF_TMC9660_ROOT)
  message(FATAL_ERROR "HF_TMC9660_ROOT must be set before including "
                      "hf_tmc9660_build_settings.cmake")
endif()

# ── Version ──────────────────────────────────────────────────────────────────
set(HF_TMC9660_VERSION_MAJOR 1)
set(HF_TMC9660_VERSION_MINOR 0)
set(HF_TMC9660_VERSION_PATCH 0)
set(HF_TMC9660_VERSION_STRING
    "${HF_TMC9660_VERSION_MAJOR}.${HF_TMC9660_VERSION_MINOR}.${HF_TMC9660_VERSION_PATCH}")

# ── Generate version header ─────────────────────────────────────────────────
configure_file(
  "${HF_TMC9660_ROOT}/inc/tmc9660_version.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/generated/tmc9660_version.h"
  @ONLY
)

# ── Source files (compiled) ──────────────────────────────────────────────────
set(HF_TMC9660_SOURCES
    "${HF_TMC9660_ROOT}/src/bootloader/tmc9660_bootloader.cpp"
)

# ── Public include directories ───────────────────────────────────────────────
set(HF_TMC9660_INCLUDE_DIRS
    "${HF_TMC9660_ROOT}/inc"
    "${HF_TMC9660_ROOT}/inc/register_mode"
    "${HF_TMC9660_ROOT}/inc/parameter_mode"
    "${CMAKE_CURRENT_BINARY_DIR}/generated"
)

# ── ESP-IDF component dependencies ──────────────────────────────────────────
set(HF_TMC9660_IDF_REQUIRES
    driver
    freertos
)

message(STATUS "[hf_tmc9660] v${HF_TMC9660_VERSION_STRING} — "
               "${HF_TMC9660_ROOT}")
