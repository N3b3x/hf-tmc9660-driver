# =============================================================================
# hf-tmc9660-driver — Root CMakeLists.txt
# =============================================================================
# Platform-agnostic build for the HardFOC TMC9660 driver.
# For ESP-IDF builds use the component wrapper instead.
# =============================================================================
cmake_minimum_required(VERSION 3.16)

project(hf_tmc9660
  VERSION   1.0.0
  LANGUAGES CXX
)

# ── Import shared build settings ─────────────────────────────────────────────
set(HF_TMC9660_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE PATH
    "Root of the TMC9660 driver repository")

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/hf_tmc9660_build_settings.cmake")

# ── Library target ───────────────────────────────────────────────────────────
add_library(hf_tmc9660 STATIC ${HF_TMC9660_SOURCES})

target_include_directories(hf_tmc9660 PUBLIC ${HF_TMC9660_INCLUDE_DIRS})

target_compile_features(hf_tmc9660 PUBLIC cxx_std_20)

# ── Optional warnings (off by default, enable with -DHF_TMC9660_ENABLE_WARNINGS=ON)
option(HF_TMC9660_ENABLE_WARNINGS "Enable compiler warnings" OFF)
if(HF_TMC9660_ENABLE_WARNINGS)
  target_compile_options(hf_tmc9660 PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ── CMake package export ─────────────────────────────────────────────────────
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/hf_tmc9660ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/hf_tmc9660Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/hf_tmc9660Config.cmake"
  INSTALL_DESTINATION lib/cmake/hf_tmc9660
)
