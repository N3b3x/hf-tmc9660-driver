# =============================================================================
# hf_tmc9660 — ESP-IDF component wrapper
# =============================================================================
# This thin wrapper imports the shared build-settings file and registers the
# component for ESP-IDF.  All sources, includes, and version info are defined
# in cmake/hf_tmc9660_build_settings.cmake (single source of truth).
# =============================================================================

# ── Locate driver root ───────────────────────────────────────────────────────
set(HF_TMC9660_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../../.." CACHE PATH
    "Path to root of hf-tmc9660-driver (auto-detected from component location)")

get_filename_component(HF_TMC9660_ROOT "${HF_TMC9660_ROOT}" ABSOLUTE)
if(NOT EXISTS "${HF_TMC9660_ROOT}/cmake/hf_tmc9660_build_settings.cmake")
    message(FATAL_ERROR
        "Cannot find TMC9660 driver root.\n"
        "  Resolved path: ${HF_TMC9660_ROOT}\n"
        "  Expected file: ${HF_TMC9660_ROOT}/cmake/hf_tmc9660_build_settings.cmake\n"
        "  Fix: set HF_TMC9660_ROOT to the correct driver checkout path.")
endif()

# ── Import build settings ────────────────────────────────────────────────────
include("${HF_TMC9660_ROOT}/cmake/hf_tmc9660_build_settings.cmake")

# ── Register ESP-IDF component ───────────────────────────────────────────────
idf_component_register(
    SRCS         ${HF_TMC9660_SOURCE_FILES}
    INCLUDE_DIRS ${HF_TMC9660_PUBLIC_INCLUDE_DIRS}
    REQUIRES     ${HF_TMC9660_IDF_REQUIRES}
)

target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_20)
