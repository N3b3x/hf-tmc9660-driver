#pragma once
#include <cstdint>

/**
 * @file tmc9660_adc.hpp
 * @brief TMC9660 ADC and analog measurement registers (Register Mode).
 *
 * Contains definitions for ADC readings (phase currents, voltages, temperature) and ADC/CSA configuration registers.
 */

namespace TMC9660 {
namespace ADC {

/// ADC Sources Configuration (Address 0x001, Block 1).
/**
 * Configures the ADC input sources and connections.
 * This includes routing of internal measurement points to ADC MUXes.
 * - Bit 31: **ADC3_MUX2_DETOUR** (RW, default 0x1) – If enabled, skip second MUX2 measurement and repeat MUX1 (AIN3) measurement.
 * - Other bits configure which sources are connected to ADC channels.
 */
struct SRC_CONFIG {
    static constexpr uint8_t ADDRESS = 0x01;  ///< Register address (Block 1)
    union {
        uint32_t value;
        struct {
            uint32_t ADC3_MUX2_DETOUR : 1;  ///< If set, the ADC channel 3 second MUX measurement is detoured to repeat channel 1 measurement (AIN3).
            uint32_t : 31;
        } bits;
    };
};

/// ADC Setup Register (Address 0x002, Block 1).
/**
 * General ADC setup parameters such as timing, mode (single-ended vs differential), etc.
 */
struct SETUP {
    static constexpr uint8_t ADDRESS = 0x02;
    union {
        uint32_t value;
        struct {
            uint32_t ADC_REF_SEL : 2;    ///< ADC reference voltage selection.
            uint32_t ADC_MODE    : 2;    ///< ADC operation mode.
            uint32_t : 28;
        } bits;
    };
};

/// ADC Status Flags (Address 0x005, Block 1).
/**
 * Status bits for the ADC block (Block1).
 * Provides flags like calibration done, error flags, etc.
 */
struct STATUS_FLAGS {
    static constexpr uint8_t ADDRESS = 0x05;
    union {
        uint32_t value;
        struct {
            uint32_t CALIB_DONE : 1;   ///< Calibration sequence completed.
            uint32_t ADC_ERROR  : 1;   ///< ADC error flag.
            uint32_t : 30;
        } bits;
    };
};

/// Current Sense Amplifier (CSA) Setup Register (Address 0x007, Block 1).
/**
 * Configures the on-chip current sense amplifiers (CSAs) used for phase current measurement.
 * This includes gain range, amplifier mode (high bandwidth vs filtered), and optional offset cancellation settings.
 */
struct CSA_SETUP {
    static constexpr uint8_t ADDRESS = 0x07;
    union {
        uint32_t value;
        struct {
            uint32_t CSA_GAIN     : 2;  ///< CSA gain setting (00: lowest, 11: highest gain).
            uint32_t CSA_BYPASS   : 1;  ///< Bypass filter on CSA if set.
            uint32_t CSA_OFFSET_TRIM : 5;  ///< Trim value for CSA offset calibration.
            uint32_t : 24;
        } bits;
    };
};

} // namespace ADC
} // namespace TMC9660
