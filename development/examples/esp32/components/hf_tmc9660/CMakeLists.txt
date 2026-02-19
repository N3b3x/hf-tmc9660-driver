# =============================================================================
# hf_tmc9660 — ESP-IDF component wrapper
# =============================================================================
# This thin wrapper imports the shared build-settings file and registers the
# component for ESP-IDF.  All sources, includes, and version info are defined
# in cmake/hf_tmc9660_build_settings.cmake (single source of truth).
# =============================================================================

# ── Locate driver root ───────────────────────────────────────────────────────
set(HF_TMC9660_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." CACHE PATH
    "Root of the hf-tmc9660-driver repository")

if(NOT EXISTS "${HF_TMC9660_ROOT}/cmake/hf_tmc9660_build_settings.cmake")
  message(FATAL_ERROR
    "Cannot find hf_tmc9660_build_settings.cmake.\n"
    "  HF_TMC9660_ROOT = ${HF_TMC9660_ROOT}")
endif()

# ── Import build settings ────────────────────────────────────────────────────
include("${HF_TMC9660_ROOT}/cmake/hf_tmc9660_build_settings.cmake")

# ── Register ESP-IDF component ───────────────────────────────────────────────
idf_component_register(
    SRCS        ${HF_TMC9660_SOURCES}
    INCLUDE_DIRS ${HF_TMC9660_INCLUDE_DIRS}
    REQUIRES     ${HF_TMC9660_IDF_REQUIRES}
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)
