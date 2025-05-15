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

/// ADC Sources Configuration Register (Address 0x001, Block 1).
/**
 * @brief Configures the ADC input sources and connections.
 * @details Block 1, Address: 0x001
 *
 * Register Map:
 * Bits    | Name                  | Access | Description
 * --------|-----------------------|--------|-------------
 * 31      | ADC3_MUX2_DETOUR      | RW     | Skip second MUX2 measurement and repeat MUX1 (AIN3) measurement.
 * 29:28   | ADC3_MUX2_CFG         | RW     | Measurement position of MUX2 input in ADC3 sequence.
 * 27:26   | ADC3_MUX1_CFG         | RW     | Measurement position of MUX1 input in ADC3 sequence.
 * 25:24   | ADC3_MUX0_CFG         | RW     | Measurement position of MUX0 input in ADC3 sequence.
 * 23      | ADC2_MUX2_DETOUR      | RW     | Skip second MUX2 measurement and repeat MUX1 (AIN2) measurement.
 * 22      | ADC2_MUX3_DIS         | RW     | Disable measurement of MUX3 input (junction temperature VTJ).
 * 21:20   | ADC2_MUX2_CFG         | RW     | Measurement position of MUX2 input in ADC2 sequence.
 * 19:18   | ADC2_MUX1_CFG         | RW     | Measurement position of MUX1 input in ADC2 sequence.
 * 17:16   | ADC2_MUX0_CFG         | RW     | Measurement position of MUX0 input in ADC2 sequence.
 * 15      | ADC1_MUX2_DETOUR      | RW     | Skip second MUX2 measurement and repeat MUX1 (AIN1) measurement.
 * 13:12   | ADC1_MUX2_CFG         | RW     | Measurement position of MUX2 input in ADC1 sequence.
 * 11:10   | ADC1_MUX1_CFG         | RW     | Measurement position of MUX1 input in ADC1 sequence.
 * 9:8     | ADC1_MUX0_CFG         | RW     | Measurement position of MUX0 input in ADC1 sequence.
 * 7       | ADC0_MUX2_DETOUR      | RW     | Skip second MUX2 measurement and repeat MUX1 (AIN0) measurement.
 * 6       | ADC0_MUX3_DIS         | RW     | Disable measurement of MUX3 input (supply voltage).
 * 5:4     | ADC0_MUX2_CFG         | RW     | Measurement position of MUX2 input in ADC0 sequence.
 * 3:2     | ADC0_MUX1_CFG         | RW     | Measurement position of MUX1 input in ADC0 sequence.
 * 1:0     | ADC0_MUX0_CFG         | RW     | Measurement position of MUX0 input in ADC0 sequence.
 *
 * @note This register configures the routing of internal measurement points to ADC MUXes.
 */
struct SRC_CONFIG {
    static constexpr uint8_t ADDRESS = 0x01;  ///< Register address (Block 1)

    /// MUX detour configuration.
    enum class Mux2Detour : uint8_t {
        NO_CHANGE = 0, ///< No changes to the measurement sequence.
        DETOUR    = 1  ///< Skip MUX2 measurement for a second MUX1 measurement.
    };

    /// MUX configuration options.
    enum class MuxConfig : uint8_t {
        OFF   = 0, ///< Skip MUX input.
        FIRST = 1, ///< Sample MUX input first after trigger.
        SECOND= 2, ///< Sample MUX input second after trigger.
        THIRD = 3  ///< Sample MUX input third after trigger.
    };

    union {
        uint32_t value;
        struct {
            // ADC3
            Mux2Detour ADC3_MUX2_DETOUR : 1;
            uint32_t : 1;
            MuxConfig ADC3_MUX2_CFG : 2;
            MuxConfig ADC3_MUX1_CFG : 2;
            MuxConfig ADC3_MUX0_CFG : 2;
            // ADC2
            Mux2Detour ADC2_MUX2_DETOUR : 1;
            uint32_t ADC2_MUX3_DIS : 1;
            MuxConfig ADC2_MUX2_CFG : 2;
            MuxConfig ADC2_MUX1_CFG : 2;
            MuxConfig ADC2_MUX0_CFG : 2;
            // ADC1
            Mux2Detour ADC1_MUX2_DETOUR : 1;
            uint32_t : 1;
            MuxConfig ADC1_MUX2_CFG : 2;
            MuxConfig ADC1_MUX1_CFG : 2;
            MuxConfig ADC1_MUX0_CFG : 2;
            // ADC0
            Mux2Detour ADC0_MUX2_DETOUR : 1;
            uint32_t ADC0_MUX3_DIS : 1;
            MuxConfig ADC0_MUX2_CFG : 2;
            MuxConfig ADC0_MUX1_CFG : 2;
            MuxConfig ADC0_MUX0_CFG : 2;
        } bits;
    };
};

/// ADC Setup Register (Address 0x002, Block 1).
/**
 * @brief General ADC setup parameters such as timing and mode.
 * @details Block 1, Address: 0x002
 *
 * Register Map:
 * Bits    | Name                  | Access | Description
 * --------|-----------------------|--------|-------------
 * 19:16   | ADC_SHIFT_SAMPLE      | RW     | Shift ADC sample time in steps of 100ns, base = 500ns.
 *
 * @note This register configures ADC timing and operational modes.
 */
struct SETUP {
    static constexpr uint8_t ADDRESS = 0x02; ///< Register address (Block 1)

    /// ADC sample time shift options.
    enum class ADCShiftSample : uint8_t {
        SHIFT_500NS  = 0,  ///< 500ns
        SHIFT_600NS  = 1,  ///< 600ns
        SHIFT_700NS  = 2,  ///< 700ns
        SHIFT_800NS  = 3,  ///< 800ns
        SHIFT_900NS  = 4,  ///< 900ns
        SHIFT_1000NS = 5,  ///< 1000ns
        SHIFT_1100NS = 6,  ///< 1100ns
        SHIFT_1200NS = 7,  ///< 1200ns
        SHIFT_1300NS = 8,  ///< 1300ns
        SHIFT_1400NS = 9,  ///< 1400ns
        SHIFT_1500NS = 10, ///< 1500ns
        SHIFT_1600NS = 11, ///< 1600ns
        SHIFT_1700NS = 12, ///< 1700ns
        SHIFT_1800NS = 13, ///< 1800ns
        SHIFT_1900NS = 14, ///< 1900ns
        SHIFT_2000NS = 15  ///< 2000ns
    };

    union {
        uint32_t value;
        struct {
            uint32_t : 16;
            ADCShiftSample ADC_SHIFT_SAMPLE : 4; ///< ADC sample time shift.
            uint32_t : 12;
        } bits;
    };
};

/// ADC Status Flags Register (Address 0x005, Block 1).
/**
 * @brief Status flags for the ADC block.
 * @details Block 1, Address: 0x005
 *
 * Register Map:
 * Bits    | Name                  | Access | Description
 * --------|-----------------------|--------|-------------
 * 15      | ADC3_MUXSEQ_FAIL      | R      | ADC3 sequence configuration error.
 * 14      | ADC2_MUXSEQ_FAIL      | R      | ADC2 sequence configuration error.
 * 13      | ADC1_MUXSEQ_FAIL      | R      | ADC1 sequence configuration error.
 * 12      | ADC0_MUXSEQ_FAIL      | R      | ADC0 sequence configuration error.
 * 11      | ADC3_WTCHDG_FAIL      | R      | ADC3 watchdog fail.
 * 10      | ADC2_WTCHDG_FAIL      | R      | ADC2 watchdog fail.
 * 9       | ADC1_WTCHDG_FAIL      | R      | ADC1 watchdog fail.
 * 8       | ADC0_WTCHDG_FAIL      | R      | ADC0 watchdog fail.
 * 3       | RDY_ADC_3             | R      | ADC3 ready.
 * 2       | RDY_ADC_2             | R      | ADC2 ready.
 * 1       | RDY_ADC_1             | R      | ADC1 ready.
 * 0       | RDY_ADC_0             | R      | ADC0 ready.
 *
 * @note This register provides status flags for ADC calibration and errors.
 */
struct STATUS_FLAGS {
    static constexpr uint8_t ADDRESS = 0x05; ///< Register address (Block 1)

    union {
        uint32_t value;
        struct {
            uint32_t RDY_ADC_0         : 1; ///< ADC0 ready.
            uint32_t RDY_ADC_1         : 1; ///< ADC1 ready.
            uint32_t RDY_ADC_2         : 1; ///< ADC2 ready.
            uint32_t RDY_ADC_3         : 1; ///< ADC3 ready.
            uint32_t : 4;
            uint32_t ADC0_WTCHDG_FAIL  : 1; ///< ADC0 watchdog fail.
            uint32_t ADC1_WTCHDG_FAIL  : 1; ///< ADC1 watchdog fail.
            uint32_t ADC2_WTCHDG_FAIL  : 1; ///< ADC2 watchdog fail.
            uint32_t ADC3_WTCHDG_FAIL  : 1; ///< ADC3 watchdog fail.
            uint32_t ADC0_MUXSEQ_FAIL  : 1; ///< ADC0 sequence configuration error.
            uint32_t ADC1_MUXSEQ_FAIL  : 1; ///< ADC1 sequence configuration error.
            uint32_t ADC2_MUXSEQ_FAIL  : 1; ///< ADC2 sequence configuration error.
            uint32_t ADC3_MUXSEQ_FAIL  : 1; ///< ADC3 sequence configuration error.
            uint32_t : 16;
        } bits;
    };
};

/// Current Sense Amplifier (CSA) Setup Register (Address 0x007, Block 1).
/**
 * @brief Configures the on-chip current sense amplifiers (CSAs).
 * @details Block 1, Address: 0x007
 *
 * Register Map:
 * Bits    | Name                  | Access | Description
 * --------|-----------------------|--------|-------------
 * 19:16   | CSA_AZ_FLTLNGTH_EXP   | RW     | Filter length exponent for AZ values.
 * 15:14   | CSA3_FILT             | RW     | Bandwidth filter settings for CSA3.
 * 13:12   | CSA012_FILT           | RW     | Bandwidth filter settings for CSA0...2.
 * 10      | CSA3_BYPASS           | RW     | Bypass of CSA3.
 * 9:8     | CSA3_GAIN             | RW     | Gain for CSA3.
 * 6       | CSA012_BYPASS         | RW     | Bypass of CSA0...2.
 * 5:4     | CSA012_GAIN           | RW     | Gain for CSA0...2.
 * 3       | CSA3_EN               | RW     | CSA3 enable.
 * 2       | CSA2_EN               | RW     | CSA2 enable.
 * 1       | CSA1_EN               | RW     | CSA1 enable.
 * 0       | CSA0_EN               | RW     | CSA0 enable.
 *
 * @note This register configures CSA gain, filter, and bypass settings.
 */
struct CSA_SETUP {
    static constexpr uint8_t ADDRESS = 0x07; ///< Register address (Block 1)

    /// CSA AZ filter length exponent.
    enum class CSAFilterLength : uint8_t {
        OFF     = 0, ///< No filter (length = 1)
        LENGTH_2= 1, ///< Filter over 2 values
        LENGTH_4= 2, ///< Filter over 4 values
        LENGTH_8= 3  ///< Filter over 8 values
    };

    /// CSA bandwidth filter settings.
    enum class CSAFilterBW : uint8_t {
        BW_0U55 = 0, ///< 0.55us
        BW_0U75 = 1, ///< 0.75us
        BW_1U00 = 2, ///< 1.00us
        BW_1U35 = 3  ///< 1.35us
    };

    /// CSA gain settings.
    enum class CSAGain : uint8_t {
        X5  = 0, ///< x5
        X10 = 1, ///< x10
        X20 = 2, ///< x20
        X40 = 3  ///< x40
    };

    union {
        uint32_t value;
        struct {
            uint32_t CSA0_EN              : 1; ///< CSA0 enable
            uint32_t CSA1_EN              : 1; ///< CSA1 enable
            uint32_t CSA2_EN              : 1; ///< CSA2 enable
            uint32_t CSA3_EN              : 1; ///< CSA3 enable
            CSAGain  CSA012_GAIN          : 2; ///< Gain for CSA0...2
            uint32_t CSA012_BYPASS        : 1; ///< Bypass for CSA0...2
            uint32_t : 1;
            CSAGain  CSA3_GAIN            : 2; ///< Gain for CSA3
            uint32_t CSA3_BYPASS          : 1; ///< Bypass for CSA3
            uint32_t : 1;
            CSAFilterBW CSA012_FILT       : 2; ///< BW filter for CSA0...2
            CSAFilterBW CSA3_FILT         : 2; ///< BW filter for CSA3
            uint32_t : 0;
            CSAFilterLength CSA_AZ_FLTLNGTH_EXP : 4; ///< Filter length exponent for AZ values
            uint32_t : 12;
        } bits;
    };
};

} // namespace ADC
} // namespace TMC9660
