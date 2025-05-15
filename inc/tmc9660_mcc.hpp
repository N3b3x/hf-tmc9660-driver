#pragma once
#include <cstdint>
#include "tmc9660_pwm.hpp"   // Include PWM definitions (if cross-referenced)
#include "tmc9660_foc.hpp"   // Include FOC definitions (for completeness)

/**
 * @file tmc9660_mcc.hpp
 * @brief TMC9660 Motor Control Core (MCC) registers (Register Mode).
 *
 * Contains definitions for registers in the Motor Control Core, including motion control configurations,
 * encoder and hall sensor feedback, ramp generator, PI controllers, and status flags.
 */

namespace TMC9660 {
namespace MCC {

/// Phase Current Raw Readings (Address 0x020, Block 0: MCC_ADC_I1_I0_RAW)
/**
 * Simultaneous raw ADC readings of phase currents I1 and I0.
 * These are signed 16-bit values representing the instantaneous phase currents.
 * - **I1**: Phase current I1 (bits 31:16, signed).
 * - **I0**: Phase current I0 (bits 15:0, signed).
 */
struct I1_I0_RAW {
    static constexpr uint16_t ADDRESS = 0x020;
    union {
        uint32_t value;
        struct {
            int16_t I1 : 16;  ///< Raw phase current I1 (signed value).
            int16_t I0 : 16;  ///< Raw phase current I0 (signed value).
        } bits;
    };
};

/// Phase Current Raw Readings (Address 0x021, Block 0: MCC_ADC_I3_I2_RAW)
/**
 * Raw ADC readings of phase currents I3 and I2.
 * - **I3**: Phase current I3 (bits 31:16, signed).
 * - **I2**: Phase current I2 (bits 15:0, signed).
 */
struct I3_I2_RAW {
    static constexpr uint16_t ADDRESS = 0x021;
    union {
        uint32_t value;
        struct {
            int16_t I3 : 16;  ///< Raw phase current I3 (signed).
            int16_t I2 : 16;  ///< Raw phase current I2 (signed).
        } bits;
    };
};

/// Phase Voltage Raw Readings (Address 0x022, Block 0: MCC_ADC_U1_U0_RAW)
/**
 * Raw ADC readings of phase voltages U1 and U0.
 * - **U1**: Phase voltage U1 (bits 31:16, signed).
 * - **U0**: Phase voltage U0 (bits 15:0, signed).
 */
struct U1_U0_RAW {
    static constexpr uint16_t ADDRESS = 0x022;
    union {
        uint32_t value;
        struct {
            int16_t U1 : 16;  ///< Raw phase voltage U1 (signed).
            int16_t U0 : 16;  ///< Raw phase voltage U0 (signed).
        } bits;
    };
};

/// Phase Voltage Raw Readings (Address 0x023, Block 0: MCC_ADC_U3_U2_RAW)
/**
 * Raw ADC readings of phase voltages U3 and U2.
 * - **U3**: Phase voltage U3 (bits 31:16, signed).
 * - **U2**: Phase voltage U2 (bits 15:0, signed).
 */
struct U3_U2_RAW {
    static constexpr uint16_t ADDRESS = 0x023;
    union {
        uint32_t value;
        struct {
            int16_t U3 : 16;  ///< Raw phase voltage U3 (signed).
            int16_t U2 : 16;  ///< Raw phase voltage U2 (signed).
        } bits;
    };
};

/// Temperature and Supply Voltage Raw Readings (Address 0x024, Block 0: MCC_ADC_TEMP_VM_RAW)
/**
 * Raw ADC readings of on-chip temperature sensor and supply (V<sub>SA</sub>) voltage monitor.
 * - **TEMP**: Die temperature sensor raw value (bits 31:16, signed).
 * - **VM**: Supply voltage monitor raw value (bits 15:0, unsigned).
 */
struct TEMP_VM_RAW {
    static constexpr uint16_t ADDRESS = 0x024;
    union {
        uint32_t value;
        struct {
            int16_t TEMP : 16;  ///< Raw die temperature sensor value (signed).
            uint16_t VM   : 16;  ///< Raw supply voltage (V<sub>SA</sub>) monitor reading.
        } bits;
    };
};

/// Auxiliary Analog Inputs Raw Readings (Address 0x025, Block 0: MCC_ADC_AIN1_AIN0_RAW)
/**
 * Raw ADC readings of auxiliary analog inputs AIN1 and AIN0.
 * - **AIN1**: Analog input 1 raw value (bits 31:16, unsigned or signed as configured).
 * - **AIN0**: Analog input 0 raw value (bits 15:0).
 */
struct AIN1_AIN0_RAW {
    static constexpr uint16_t ADDRESS = 0x025;
    union {
        uint32_t value;
        struct {
            uint16_t AIN1 : 16;  ///< Raw analog input 1.
            uint16_t AIN0 : 16;  ///< Raw analog input 0.
        } bits;
    };
};

/// Auxiliary Analog Inputs Raw Readings (Address 0x026, Block 0: MCC_ADC_AIN3_AIN2_RAW)
/**
 * Raw ADC readings of auxiliary analog inputs AIN3 and AIN2.
 */
struct AIN3_AIN2_RAW {
    static constexpr uint16_t ADDRESS = 0x026;
    union {
        uint32_t value;
        struct {
            uint16_t AIN3 : 16;  ///< Raw analog input 3.
            uint16_t AIN2 : 16;  ///< Raw analog input 2.
        } bits;
    };
};

/// ADC General Configuration Register (Address 0x040, Block 0).
/**
 * @brief General configuration setup of the current ADCs (MCC_ADC_I_GEN_CONFIG).
 * 
 * This structure represents the configuration register for the current ADCs, 
 * allowing control over various calibration and measurement settings.
 * 
 * @details
 * Address: 0x040, Block 0
 * 
 * BITS & NAME          | TYPE & RESET | DESCRIPTION
 * ---------------------|--------------|-------------------------------------------------
 * [31:16] TRIGGER_POS  | RW, unsigned | 0x0000 Relative position of ADC trigger event 
 *                     |              | in PWM cycle. Percentage of maxcnt 
 *                     |              | (0 -> PWM_Z, 32768 -> PWM_C).
 * [12] TRIGGER_SELECT  | RW           | 0x1 Select trigger point to start process of 
 *                     |              | new ADC samples and start next FOC calculation 
 *                     |              | afterwards. 
 *                     |              | 0: INLINE Trigger on start of each PWM cycle (PWM_Z).
 *                     |              | 1: SYNC_TRIGGER Trigger when ADC current 
 *                     |              | measurement is finished (default).
 * [11:9] MEASUREMENT_MODE | RW        | 0x0 Configuration of measurement mode:
 *                     |              | 0: INLINE 3 channel BLDC/2 channel Stepper Inline 
 *                     |              |    Shunt Measurement.
 *                     |              | 1: INLINE_VW 2 channels with I_V and I_WY measured (BLDC).
 *                     |              | 2: INLINE_UW 2 channels with I_UX and I_WY measured (BLDC).
 *                     |              | 3: INLINE_UV 2 channels with I_UX and I_V measured (BLDC).
 *                     |              | 4: BOTTOM 3/4 phase bottom shunt with automatic 
 *                     |              |    switching (BLDC and Stepper).
 * [7:6] Y2_SELECT      | RW           | 0x3 Input selection of raw current ADC_I_Y2:
 *                     |              | 0: ADC_I0
 *                     |              | 1: ADC_I1
 *                     |              | 2: ADC_I2
 *                     |              | 3: ADC_I3
 * [5:4] WY1_SELECT     | RW           | 0x2 Input selection of raw current ADC_I_WY1:
 *                     |              | 0: ADC_I0
 *                     |              | 1: ADC_I1
 *                     |              | 2: ADC_I2
 *                     |              | 3: ADC_I3
 * [3:2] VX2_SELECT     | RW           | 0x1 Input selection of raw current ADC_I_VX2:
 *                     |              | 0: ADC_I0
 *                     |              | 1: ADC_I1
 *                     |              | 2: ADC_I2
 *                     |              | 3: ADC_I3
 * [1:0] UX1_SELECT     | RW           | 0x0 Input selection of raw current ADC_I_UX:
 *                     |              | 0: ADC_I0
 *                     |              | 1: ADC_I1
 *                     |              | 2: ADC_I2
 *                     |              | 3: ADC_I3
 * 
 * Fields:
 * - OFFSET_EN: Enable automatic offset calibration for current ADCs.
 * - CALIB_EN: Enable current sense calibration.
 */
struct I_GEN_CONFIG {
    static constexpr uint16_t ADDRESS = 0x040;
    union {
        uint32_t value;
        struct {
            uint32_t UX1_SELECT : 2;      ///< Input selection of raw current ADC_I_UX.
            uint32_t VX2_SELECT : 2;      ///< Input selection of raw current ADC_I_VX2.
            uint32_t WY1_SELECT : 2;      ///< Input selection of raw current ADC_I_WY1.
            uint32_t Y2_SELECT  : 2;      ///< Input selection of raw current ADC_I_Y2.
            uint32_t : 1;                 ///< Reserved.
            uint32_t MEASUREMENT_MODE : 3;///< Configuration of measurement mode.
            uint32_t : 3;                 ///< Reserved.
            uint32_t TRIGGER_SELECT : 1;  ///< Select trigger point to start process of new ADC samples.
            uint32_t : 3;                 ///< Reserved.
            uint32_t TRIGGER_POS : 16;    ///< Relative position of ADC trigger event in PWM cycle.
        } bits;
    };
};

/// Current ADC Channel 0 Configuration (Address 0x041, Block 0: MCC_ADC_I0_CONFIG)
/**
 * Current ADC channel 0 offset and scaling values.
 * - **SCALE** (bits 31:16): Current ADC channel 0 scaling value (RW, signed, default 0xFC00).
 * - **OFFSET** (bits 15:0): Current ADC channel 0 offset value (RW, signed, default 0x0000).
 */
struct I0_CONFIG {
    static constexpr uint16_t ADDRESS = 0x041;
    union {
        uint32_t value;
        struct {
            int16_t OFFSET; ///< Current ADC channel 0 offset value.
            int16_t SCALE;  ///< Current ADC channel 0 scaling value.
        } bits;
    };
};

/// Current ADC Channel 1 Configuration (Address 0x042, Block 0: MCC_ADC_I1_CONFIG)
/**
 * Current ADC channel 1 offset and scaling values.
 * - **SCALE** (bits 31:16): Current ADC channel 1 scaling value (RW, signed, default 0xFC00).
 * - **OFFSET** (bits 15:0): Current ADC channel 1 offset value (RW, signed, default 0x0000).
 */
struct I1_CONFIG {
    static constexpr uint16_t ADDRESS = 0x042;
    union {
        uint32_t value;
        struct {
            int16_t OFFSET; ///< Current ADC channel 1 offset value.
            int16_t SCALE;  ///< Current ADC channel 1 scaling value.
        } bits;
    };
};

/// Current ADC Channel 2 Configuration (Address 0x043, Block 0: MCC_ADC_I2_CONFIG)
/**
 * Current ADC channel 2 offset and scaling values.
 * - **SCALE** (bits 31:16): Current ADC channel 2 scaling value (RW, signed, default 0xFC00).
 * - **OFFSET** (bits 15:0): Current ADC channel 2 offset value (RW, signed, default 0x0000).
 */
struct I2_CONFIG {
    static constexpr uint16_t ADDRESS = 0x043;
    union {
        uint32_t value;
        struct {
            int16_t OFFSET; ///< Current ADC channel 2 offset value.
            int16_t SCALE;  ///< Current ADC channel 2 scaling value.
        } bits;
    };
};

/// Current ADC Channel 3 Configuration (Address 0x044, Block 0: MCC_ADC_I3_CONFIG)
/**
 * Current ADC channel 3 offset and scaling values.
 * - **SCALE** (bits 31:16): Current ADC channel 3 scaling value (RW, signed, default 0xFC00).
 * - **OFFSET** (bits 15:0): Current ADC channel 3 offset value (RW, signed, default 0x0000).
 */
struct I3_CONFIG {
    static constexpr uint16_t ADDRESS = 0x044;
    union {
        uint32_t value;
        struct {
            int16_t OFFSET; ///< Current ADC channel 3 offset value.
            int16_t SCALE;  ///< Current ADC channel 3 scaling value.
        } bits;
    };
};

/// Scaled Current Readings I1/I0 (Address 0x045, Block 0: MCC_ADC_I1_I0_SCALED)
/**
 * Phase current I1, I0; after applying scaling and offset.
 * - **I1** (bits 31:16): Calculated Phase Current I1 after applying scaling and offset for further processing.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 * - **I0** (bits 15:0): Calculated Phase Current I0 after applying scaling and offset for further processing.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct I1_I0_SCALED {
    static constexpr uint16_t ADDRESS = 0x045;
    union {
        uint32_t value;
        struct {
            int16_t I1 : 16; ///< Calculated Phase Current I1.
            int16_t I0 : 16; ///< Calculated Phase Current I0.
        } bits;
    };
};

/// Scaled Current Readings I3/I2 (Address 0x046, Block 0: MCC_ADC_I3_I2_SCALED)
/**
 * Phase current I3, I2; after applying scaling and offset.
 * - **I3** (bits 31:16): Calculated Phase Current I3 after applying scaling and offset for further processing.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 * - **I2** (bits 15:0): Calculated Phase Current I2 after applying scaling and offset for further processing.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct I3_I2_SCALED {
    static constexpr uint16_t ADDRESS = 0x046;
    union {
        uint32_t value;
        struct {
            int16_t I3 : 16; ///< Calculated Phase Current I3.
            int16_t I2 : 16; ///< Calculated Phase Current I2.
        } bits;
    };
};

/// Scaled Current ADC Value (Address 0x047, Block 0: MCC_ADC_IWY_IUX)
/**
 * Scaled current ADC value including signed added offset as input for the FOC.
 * - **IWY** (bits 31:16): Scaled current ADC value including signed added offset as input for the FOC phase W/Y.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 * - **IUX** (bits 15:0): Scaled current ADC value including signed added offset as input for the FOC phase U/X.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct IWY_IUX {
    static constexpr uint16_t ADDRESS = 0x047;
    union {
        uint32_t value;
        struct {
            int16_t IWY : 16; ///< Scaled current ADC value for phase W/Y.
            int16_t IUX : 16; ///< Scaled current ADC value for phase U/X.
        } bits;
    };
};

/// Scaled Current ADC Value (Address 0x048, Block 0: MCC_ADC_IV)
/**
 * Scaled current ADC value including signed added offset as input for the FOC.
 * - **IV** (bits 15:0): Scaled current ADC value including signed added offset as input for the FOC phase V.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct IV_RAW {
    static constexpr uint16_t ADDRESS = 0x048;
    union {
        uint32_t value;
        struct {
            int16_t IV : 16; ///< Scaled current ADC value for phase V.
            uint16_t : 16;
        } bits;
    };
};

/// ADC Status Register (Address 0x049, Block 0: MCC_ADC_STATUS)
/**
 * @struct STATUS
 * @brief Represents the status register of the TMC9660 driver.
 *
 * This structure encapsulates the status register of the TMC9660 driver, 
 * providing detailed information about the state of various ADC measurements 
 * and their completion status. The register is represented as a 32-bit value 
 * with individual bits indicating specific conditions or events.
 *
 * The bits in the status register are organized as follows:
 * 
 * - **Clipping Flags (Bits 0-13):**
 *   These flags indicate whether the corresponding ADC measurement value 
 *   has been clipped (i.e., exceeded the measurable range).
 *   - `I0_CLIPPED` (Bit 0): ADC current measurement I0 value clipped.
 *   - `I1_CLIPPED` (Bit 1): ADC current measurement I1 value clipped.
 *   - `I2_CLIPPED` (Bit 2): ADC current measurement I2 value clipped.
 *   - `I3_CLIPPED` (Bit 3): ADC current measurement I3 value clipped.
 *   - `U0_CLIPPED` (Bit 4): ADC voltage measurement U0 value clipped.
 *   - `U1_CLIPPED` (Bit 5): ADC voltage measurement U1 value clipped.
 *   - `U2_CLIPPED` (Bit 6): ADC voltage measurement U2 value clipped.
 *   - `U3_CLIPPED` (Bit 7): ADC voltage measurement U3 value clipped.
 *   - `AIN0_CLIPPED` (Bit 8): ADC measurement AIN0 value clipped.
 *   - `AIN1_CLIPPED` (Bit 9): ADC measurement AIN1 value clipped.
 *   - `AIN2_CLIPPED` (Bit 10): ADC measurement AIN2 value clipped.
 *   - `AIN3_CLIPPED` (Bit 11): ADC measurement AIN3 value clipped.
 *   - `VM_CLIPPED` (Bit 12): ADC supply voltage measurement VM value clipped.
 *   - `TEMP_CLIPPED` (Bit 13): ADC temperature voltage measurement value clipped.
 * 
 * - **Reserved Bits (Bits 14-15):**
 *   These bits are reserved for future use and should be ignored.
 * 
 * - **Completion Flags (Bits 16-31):**
 *   These flags indicate whether the corresponding ADC measurement has been 
 *   completed successfully.
 *   - `I0_DONE` (Bit 16): ADC current measurement I0 finished.
 *   - `I1_DONE` (Bit 17): ADC current measurement I1 finished.
 *   - `I2_DONE` (Bit 18): ADC current measurement I2 finished.
 *   - `I3_DONE` (Bit 19): ADC current measurement I3 finished.
 *   - `U0_DONE` (Bit 20): ADC voltage measurement U0 finished.
 *   - `U1_DONE` (Bit 21): ADC voltage measurement U1 finished.
 *   - `U2_DONE` (Bit 22): ADC voltage measurement U2 finished.
 *   - `U3_DONE` (Bit 23): ADC voltage measurement U3 finished.
 *   - `AIN0_DONE` (Bit 24): ADC measurement AIN0 finished.
 *   - `AIN1_DONE` (Bit 25): ADC measurement AIN1 finished.
 *   - `AIN2_DONE` (Bit 26): ADC measurement AIN2 finished.
 *   - `AIN3_DONE` (Bit 27): ADC measurement AIN3 finished.
 *   - `VM_DONE` (Bit 28): ADC supply voltage measurement VM finished.
 *   - `TEMP_DONE` (Bit 29): ADC temperature voltage measurement finished.
 */
struct STATUS {
    static constexpr uint16_t ADDRESS = 0x049;
    union {
        uint32_t value;
        struct {
            uint32_t I0_CLIPPED : 1;    ///< ADC current measurement I0 value clipped.
            uint32_t I1_CLIPPED : 1;    ///< ADC current measurement I1 value clipped.
            uint32_t I2_CLIPPED : 1;    ///< ADC current measurement I2 value clipped.
            uint32_t I3_CLIPPED : 1;    ///< ADC current measurement I3 value clipped.
            uint32_t U0_CLIPPED : 1;    ///< ADC voltage measurement U0 value clipped.
            uint32_t U1_CLIPPED : 1;    ///< ADC voltage measurement U1 value clipped.
            uint32_t U2_CLIPPED : 1;    ///< ADC voltage measurement U2 value clipped.
            uint32_t U3_CLIPPED : 1;    ///< ADC voltage measurement U3 value clipped.
            uint32_t AIN0_CLIPPED : 1;  ///< ADC measurement AIN0 value clipped.
            uint32_t AIN1_CLIPPED : 1;  ///< ADC measurement AIN1 value clipped.
            uint32_t AIN2_CLIPPED : 1;  ///< ADC measurement AIN2 value clipped.
            uint32_t AIN3_CLIPPED : 1;  ///< ADC measurement AIN3 value clipped.
            uint32_t VM_CLIPPED : 1;    ///< ADC supply voltage measurement VM value clipped.
            uint32_t TEMP_CLIPPED : 1;  ///< ADC temperature voltage measurement value clipped.
            uint32_t : 2;
            uint32_t I0_DONE : 1;       ///< ADC current measurement I0 finished.
            uint32_t I1_DONE : 1;       ///< ADC current measurement I1 finished.
            uint32_t I2_DONE : 1;       ///< ADC current measurement I2 finished.
            uint32_t I3_DONE : 1;       ///< ADC current measurement I3 finished.
            uint32_t U0_DONE : 1;       ///< ADC voltage measurement U0 finished.
            uint32_t U1_DONE : 1;       ///< ADC voltage measurement U1 finished.
            uint32_t U2_DONE : 1;       ///< ADC voltage measurement U2 finished.
            uint32_t U3_DONE : 1;       ///< ADC voltage measurement U3 finished.
            uint32_t AIN0_DONE : 1;     ///< ADC measurement AIN0 finished.
            uint32_t AIN1_DONE : 1;     ///< ADC measurement AIN1 finished.
            uint32_t AIN2_DONE : 1;     ///< ADC measurement AIN2 finished.
            uint32_t AIN3_DONE : 1;     ///< ADC measurement AIN3 finished.
            uint32_t VM_DONE : 1;       ///< ADC supply voltage measurement VM finished.
            uint32_t TEMP_DONE : 1;     ///< ADC temperature voltage measurement finished.
        } bits;
    };
};

/// Motor Configuration Register (Address 0x060, Block 0).
/**
 * Configures fundamental motor parameters and operating modes.
 * - Bits may select motor type (NONE, DC, STEPPER, BLDC), and number of pole pairs.
 * - Example fields:
 *   - **TYPE** (bits 17:16): 0 = NONE, 1 = DC, 2 = STEPPER, 3 = BLDC.
 *   - **N_POLE_PAIRS** (bits 6:0): Number of pole pairs (minimum 1).
 */
struct MOTOR_CONFIG {
    static constexpr uint16_t ADDRESS = 0x060;
    union {
        uint32_t value;
        struct {
            uint32_t N_POLE_PAIRS : 7;  ///< Number of pole pairs (minimum 1).
            uint32_t : 9;
            uint32_t TYPE         : 2;  ///< Motor type (0: NONE, 1: DC, 2: STEPPER, 3: BLDC).
            uint32_t : 14;
        } bits;
    };
};

/// Motion Control Configuration Register (Address 0x061, Block 0).
/**
 * Configures the motion control loop behaviors (velocity and position control settings).
 * - This includes enabling/disabling position or velocity control loops, setting default controller modes, and feedforward configurations.
 * - **FEEDFORWARD** (bits 7:6): Control of the feedforward structure.
 *   - 0: Disabled.
 *   - 1: MCC_RAMPER_V_ACTUAL used as feedforward input for velocity controller.
 *   - 2: MCC_RAMPER_A_ACTUAL used as feedforward input for torque controller.
 *   - 3: Both feedforward inputs for velocity and torque are used.
 * - **RAMP_MODE** (bit 5): Selection of ramp mode.
 *   - 0: Position mode.
 *   - 1: Velocity mode.
 * - **RAMP_ENABLE** (bit 4): Enable ramp generator.
 * - **MOTION_MODE** (bits 3:0): Configuration of motion mode.
 *   - 0: Stopped mode.
 *   - 1: Torque mode.
 *   - 2: Velocity mode.
 *   - 3: Position mode.
 *   - 4: PRBS flux mode.
 *   - 5: PRBS torque mode.
 *   - 6: PRBS velocity mode.
 *   - 7: PRBS position mode.
 *   - 8: Voltage external mode.
 *   - 9: PRBS UD mode.
 */
struct MOTION_CONFIG {
    static constexpr uint16_t ADDRESS = 0x061;
    union {
        uint32_t value;
        struct {
            uint32_t MOTION_MODE : 4;    ///< Motion mode configuration.
            uint32_t RAMP_ENABLE : 1;   ///< Enable ramp generator.
            uint32_t RAMP_MODE   : 1;   ///< Ramp mode selection.
            uint32_t FEEDFORWARD : 2;   ///< Feedforward control structure.
            uint32_t : 24;
        } bits;
    };
};

/// Electrical Angle Selection Register (Address 0x062, Block 0).
/**
 * Selects the source for the electrical rotor angle (phi_e) used in FOC commutation.
 * - Possible sources: External phi, ramp-generated phi, encoder-based phi, etc.
 * - Bits define a multiplexer setting for phi_e:
 *   - 0: RESERVED (not used).
 *   - 1: Use external phi (PHI_E_EXT).
 *   - 2: Use ramp-generated phi (PHI_E_RAMP).
 *   - 3: Use encoder-based phi (PHI_E_ABN).
 *   - 4: Use ramp position (RAMP_X_ACTUAL).
 *   - 5: Use hall sensor with extrapolation (PHI_E_HAL).
 */
struct PHI_E_SELECTION {
    static constexpr uint16_t ADDRESS = 0x062;
    union {
        uint32_t value;
        struct {
            uint32_t PHI_E_SEL : 4;  ///< Rotor angle source selection.
            uint32_t : 28;
        } bits;
    };
};

/// Electrical Angle Value Register (Address 0x063, Block 0).
/**
 * Actual electrical angle of the motor (phi_e), as used by the FOC.
 * Reading gives the current angle used for the inner FOC loop.
 */
struct PHI_E {
    static constexpr uint16_t ADDRESS = 0x063;
    union {
        uint32_t value;
        struct {
            int16_t PHI_E : 16;  ///< Angle used for the inner FOC loop (signed 16-bit).
            uint16_t : 16;
        } bits;
    };
};

/// PWM Configuration Register (Address 0x080, Block 0).
/**
 * Configures the PWM unit (frequency, mode, center-aligned vs edge, space vector modulation, etc.).
 * - **SV_MODE** (bit field): Space vector modulation mode selection (0 = off, >0 = enabled with specific scheme).
 * - **POLARITY** (bit): Inverts PWM output polarity if set.
 * - Additional fields configure PWM operating mode.
 */
struct CONFIG {
    static constexpr uint16_t ADDRESS = 0x080;
    union {
        uint32_t value;
        struct {
            uint32_t SV_MODE     : 2;   ///< Space Vector PWM mode (0: off, 1: 3rd harmonic injection, etc.).
            uint32_t POLARITY    : 1;   ///< Invert PWM output polarity (1 = inverted).
            uint32_t CENTER_ALIGNED : 1;///< Center-aligned PWM if set, edge-aligned if clear.
            uint32_t PWM_FREQ_DIV : 4;  ///< PWM frequency divider (sets base PWM frequency).
            uint32_t : 24;
        } bits;
    };
};

/// PWM Max Counter Register (Address 0x081, Block 0).
/**
 * Sets the maximum PWM timer count (period). Determines the PWM frequency together with the base clock.
 * Default is chip-specific (e.g., 0x00FF for 8-bit resolution).
 */
struct MAXCNT {
    static constexpr uint16_t ADDRESS = 0x081;
    union {
        uint32_t value;
        struct {
            uint32_t MAXCNT : 16;  ///< PWM period count (PWM resolution).
            uint32_t : 16;
        } bits;
    };
};

/// PWM Switch Frequency Limit Register (Address 0x083, Block 0).
/**
 * Defines the velocity threshold above which the PWM frequency is increased (for FOC control).
 * When motor velocity exceeds this limit, the system may switch to a higher PWM frequency or different modulation to optimize performance.
 */
struct SWITCH_LIMIT {
    static constexpr uint16_t ADDRESS = 0x083;
    union {
        uint32_t value;
        struct {
            uint32_t SWITCH_VEL_LIMIT : 16;  ///< Velocity threshold for PWM mode switch (in encoder counts per cycle or another unit).
            uint32_t : 16;
        } bits;
    };
};

/// ABN Encoder Angle and Mechanical Angle Register (Address 0x0A0, Block 0).
/**
 * Contains the current electrical angle (phi_e) and mechanical angle (phi_m) from the ABN incremental encoder interface.
 * - **PHI_E_ABN**: Electrical angle from encoder (bits 31:16, signed).
 *   - ABN_PHI_E = (ABN_PHI_M × N_POLE_PAIRS) + ABN_PHI_E_OFFSET.
 * - **PHI_M_ABN**: Mechanical angle (multi-turn) from encoder (bits 15:0, signed).
 *   - ABN_PHI_M = ABN_COUNT × 2^16 / ABN_CPR.
 */
struct ABN_PHI_E_PHI_M {
    static constexpr uint16_t ADDRESS = 0x0A0;
    union {
        uint32_t value;
        struct {
            int16_t PHI_E_ABN : 16;  ///< Encoder-derived electrical angle (signed).
            int16_t PHI_M_ABN : 16;  ///< Encoder mechanical position (absolute count, signed).
        } bits;
    };
};

/// ABN Encoder Mode Register (Address 0x0A1, Block 0).
/**
 * Configuration of the ABN encoder interface.
 * - Bits define mode such as incremental vs absolute, index (N) pulse usage, filter enabling, etc.
 * - For example:
 *   - **DIRECTION** (bit 12): Decoder count direction.
 *     - 0: POS (positive).
 *     - 1: NEG (negative).
 *   - **CLN** (bit 8): N channel event writes ABN_COUNT_N into ABN_COUNT at N pulse instead of 0.
 *     - 0: OFF (ABN_COUNT is written to 0).
 *     - 1: ON (ABN_COUNT is written to ABN_COUNT_N).
 *   - **DISABLE_FILTER** (bit 5): Disable digital noise filter on encoder signals.
 *     - 0: FILTERED (Filter enabled: pulses longer than 3 system clock cycles are evaluated).
 *     - 1: UNFILTERED (Filter disabled).
 *   - **CLEAR_COUNT_ON_N** (bit 4): Set ABN_COUNT to 0 on Null signal.
 *     - 0: DISABLED.
 *     - 1: ENABLED.
 *   - **COMBINED_N** (bit 3): Use AND of all three signals A, B, N to determine the Null signal.
 *     - 0: ONLY_N (Ignore A and B, just use N pulse as Null signal).
 *     - 1: ALL (Use all three signals as Null signal).
 *   - **N_POL** (bit 2): Polarity of N pulse at Null.
 *     - 0: HIGH_ACT (High active).
 *     - 1: LOW_ACT (Low active).
 *   - **B_POL** (bit 1): Polarity of B pulse at N (used if COMBINED_N is on).
 *     - 0: HIGH_ACT (High active).
 *     - 1: LOW_ACT (Low active).
 *   - **A_POL** (bit 0): Polarity of A pulse at N (used if COMBINED_N is on).
 *     - 0: HIGH_ACT (High active).
 *     - 1: LOW_ACT (Low active).
 */
struct ABN_MODE {
    static constexpr uint16_t ADDRESS = 0x0A1;
    union {
        uint32_t value;
        struct {
            uint32_t A_POL            : 1;   ///< Polarity of A pulse at N.
            uint32_t B_POL            : 1;   ///< Polarity of B pulse at N.
            uint32_t N_POL            : 1;   ///< Polarity of N pulse at Null.
            uint32_t COMBINED_N       : 1;   ///< Use AND of A, B, N to determine Null signal.
            uint32_t CLEAR_COUNT_ON_N : 1;   ///< Set ABN_COUNT to 0 on Null signal.
            uint32_t DISABLE_FILTER   : 1;   ///< Disable digital noise filter on encoder signals.
            uint32_t : 2;
            uint32_t CLN              : 1;   ///< N channel event writes ABN_COUNT_N into ABN_COUNT.
            uint32_t : 3;
            uint32_t DIRECTION        : 1;   ///< Decoder count direction.
            uint32_t : 19;
        } bits;
    };
};

/// ABN Encoder Counts per Revolution Register (Address 0x0A2, Block 0).
/**
 * Sets the number of counts per mechanical revolution for the ABN encoder.
 * This is used for scaling the encoder feedback:
 * - **ABN_CPR**: Encoder counts per revolution (24-bit, bits 23:0).
 *   - Must also set ABN_CPR_INV accordingly (ABN_CPR_INV = 2^32 / ABN_CPR).
 * - Default value: 0x010000 (65536 counts per revolution).
 */
struct ABN_CPR {
    static constexpr uint16_t ADDRESS = 0x0A2;
    union {
        uint32_t value;
        struct {
            uint32_t CPR : 24;  ///< Encoder counts-per-revolution (CPR).
            uint32_t : 8;
        } bits;
    };

    /**
     * @brief Calculates the inverse CPR value for ABN_CPR_INV register.
     * @return The calculated ABN_CPR_INV value (2^32 / CPR).
     */
    uint32_t calculateInverseCPR() const {
        return (CPR != 0) ? (static_cast<uint64_t>(1) << 32) / CPR : 0;
    }
};

/// ABN Encoder Inverse CPR Register (Address 0x0A3, Block 0).
/**
 * Holds 2^32 divided by the encoder CPR (for internal use in angle calculations).
 * - **ABN_CPR_INV**: 32-bit value such that `ABN_CPR * ABN_CPR_INV ≈ 2^32`.
 * - Default reset value: 0x00010000.
 */
struct ABN_CPR_INV {
    static constexpr uint16_t ADDRESS = 0x0A3;
    union {
        uint32_t value;
        struct {
            uint32_t ABN_CPR_INV : 32; ///< 2^32 divided by encoder CPR.
        } bits;
    };
};

/// ABN Encoder Counter Register (Address 0x0A4, Block 0).
/**
 * Raw decoder count. The digital decoder engine counts modulo ABN_CPR.
 * - **ABN_COUNT** (bits 23:0): Raw decoder count.
 *   - Type: RW, unsigned.
 *   - Reset: 0x000000.
 */
struct ABN_COUNT {
    static constexpr uint16_t ADDRESS = 0x0A4;
    union {
        uint32_t value;
        struct {
            uint32_t ABN_COUNT : 24;  ///< Raw decoder count (modulo ABN_CPR).
            uint32_t : 8;
        } bits;
    };
};

/// ABN Encoder N Pulse Count Register (Address 0x0A5, Block 0).
/**
 * ABN_COUNT latched on N pulse. When N pulse clears ABN_COUNT, ABN_COUNT_N is also set to 0.
 */
struct ABN_COUNT_N {
    static constexpr uint16_t ADDRESS = 0x0A5;
    union {
        uint32_t value;
        struct {
            uint32_t ABN_COUNT_N : 24;  ///< ABN_COUNT latched on N pulse.
            uint32_t : 8;
        } bits;
    };
};

/// ABN Encoder Offset Angle Register (Address 0x0A6, Block 0).
/**
 * Offset for ABN_PHI_E. This offset is added to align the encoder angle with the motor's electrical zero.
 */
struct ABN_PHI_E_OFFSET {
    static constexpr uint16_t ADDRESS = 0x0A6;
    union {
        uint32_t value;
        struct {
            int16_t ABN_PHI_E_OFFSET : 16;  ///< Offset for ABN_PHI_E.
            uint16_t : 16;
        } bits;
    };
};

/// Hall Sensor Mode Register (Address 0x0C0, Block 0).
/**
 * Configuration for the digital Hall sensor interface.
 * - **FILTER** (bits 15:8): Define filter length for hall signals.
 * - **ORDER** (bits 6:4): Ordering of the hall signals.
 *   - 0: UVW Hall Signal Order U/V/W.
 *   - 1: VWU Hall Signal Order V/W/U.
 *   - 2: WUV Hall Signal Order W/U/V.
 *   - 4: UWV Hall Signal Order U/W/V.
 *   - 5: VUW Hall Signal Order V/U/W.
 *   - 6: WVU Hall Signal Order W/V/U.
 * - **EXTRAPOLATION** (bit 1): Enable extrapolation for PHI_E.
 *   - 0: DISABLED (Raw signal is used for HALL_PHI_E).
 *   - 1: ENABLED (HALL_PHI_E_EXTRAPOLATED is used for HALL_PHI_E).
 * - **POLARITY** (bit 0): Polarity of the hall signals.
 *   - 0: NORMAL.
 *   - 1: INVERSED.
 */
struct HALL_MODE {
    static constexpr uint16_t ADDRESS = 0x0C0;
    union {
        uint32_t value;
        struct {
            uint32_t POLARITY      : 1;   ///< Polarity of the hall signals.
            uint32_t EXTRAPOLATION : 1;   ///< Enable extrapolation for PHI_E.
            uint32_t : 2;
            uint32_t ORDER         : 3;   ///< Ordering of the hall signals.
            uint32_t : 1;
            uint32_t FILTER        : 8;   ///< Filter length for hall signals.
            uint32_t : 16;
        } bits;
    };
};

/// Hall Sensor Digital Filter Register (Address 0x0C1, Block 0).
/**
 * Maximum delta in hall angle per timestep (to filter noise).
 * - **HALL_DPHI_MAX** (bits 15:0): Maximum allowable hall angle change per sample.
 *   - Type: RW, unsigned.
 *   - Reset: 0x2AAA (default for digital hall: (2^16)/6).
 *   - Description: Extrapolation of phi_e stops after HALL_DPHI_MAX if no new hall position is detected.
 */
struct HALL_DPHI_MAX {
    static constexpr uint16_t ADDRESS = 0x0C1;
    union {
        uint32_t value;
        struct {
            uint16_t HALL_DPHI_MAX; ///< Maximum phi_e change for extrapolation.
            uint16_t _reserved;
        } bits;
    };
};

/// Hall Sensor Angle Offset Register (Address 0x0C2, Block 0).
/**
 * Electrical angle offset added to hall-sensor-derived angle (to align with motor FOC angle).
 * - **HALL_PHI_E_OFFSET** (bits 15:0): Offset for electrical angle hall_phi_e of hall decoder.
 *   - Type: RW, signed.
 *   - Reset: 0x0000.
 */
struct HALL_PHI_E_OFFSET {
    static constexpr uint16_t ADDRESS = 0x0C2;
    union {
        uint32_t value;
        struct {
            int16_t HALL_PHI_E_OFFSET; ///< Offset for electrical angle hall_phi_e.
            uint16_t _reserved;
        } bits;
    };
};

/// Hall Sensor Counter Register (Address 0x0C3, Block 0).
/**
 * Counter for hall sensor transitions or integrated hall angle (if extrapolation is used).
 * - **HALL_COUNT** (bits 15:0): Count of passed hall states.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct HALL_COUNT {
    static constexpr uint16_t ADDRESS = 0x0C3;
    union {
        uint32_t value;
        struct {
            int16_t HALL_COUNT; ///< Count of passed hall states.
            uint16_t _reserved;
        } bits;
    };
};

/// Hall Sensor Extrapolated Angle Register (Address 0x0C4, Block 0).
/**
 * Extrapolated electrical angle (phi_e) from hall sensors (if prediction/extrapolation enabled).
 * - **PHI_E_EXTRAPOLATED** (bits 31:16): Extrapolated electrical angle hall_phi_e_extrapolated.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 * - **PHI_E** (bits 15:0): Electrical angle hall_phi_e of hall decoder.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct HALL_PHI_E_EXTRAPOLATED_PHI_E {
    static constexpr uint16_t ADDRESS = 0x0C4;
    union {
        uint32_t value;
        struct {
            int16_t PHI_E_EXTRAPOLATED; ///< Extrapolated electrical angle.
            int16_t PHI_E;              ///< Electrical angle hall_phi_e.
        } bits;
    };
};

/// Hall Sensor Positions 0°/60° Register (Address 0x0C5, Block 0).
/**
 * Exact electrical positions (hall sensor thresholds) for hall sensor states corresponding to 0° and 60° electrical angles.
 * - **POSITION_060**: Hall sensor position for 60° (bits 31:16, signed).
 * - **POSITION_000**: Hall sensor position for 0° (bits 15:0, signed).
 */
struct HALL_POSITION_060_000 {
    static constexpr uint16_t ADDRESS = 0x0C5;
    union {
        uint32_t value;
        struct {
            int16_t POSITION_060 : 16;  ///< Hall sensor angle at 60° electrical.
            int16_t POSITION_000 : 16;  ///< Hall sensor angle at 0° electrical.
        } bits;
    };
    static constexpr int16_t RESET_POSITION_060 = 0x2AAA; ///< Default reset value for POSITION_060.
    static constexpr int16_t RESET_POSITION_000 = 0x0000; ///< Default reset value for POSITION_000.
};

/// Hall Sensor Positions 180°/120° Register (Address 0x0C6, Block 0).
/**
 * Exact positions for hall states at 180° and 120°.
 * - **POSITION_180**: Hall sensor position for 180° (bits 31:16, signed).
 * - **POSITION_120**: Hall sensor position for 120° (bits 15:0, signed).
 */
struct HALL_POSITION_180_120 {
    static constexpr uint16_t ADDRESS = 0x0C6;
    union {
        uint32_t value;
        struct {
            int16_t POSITION_180 : 16; ///< Hall sensor angle at 180° electrical.
            int16_t POSITION_120 : 16; ///< Hall sensor angle at 120° electrical.
        } bits;
    };
    static constexpr int16_t RESET_POSITION_180 = 0x8000; ///< Default reset value for POSITION_180.
    static constexpr int16_t RESET_POSITION_120 = 0x5555; ///< Default reset value for POSITION_120.
};

/// Hall Sensor Positions 300°/240° Register (Address 0x0C7, Block 0).
/**
 * Exact positions for hall states at 300° and 240°.
 * - **POSITION_300**: Hall sensor position for 300° (bits 31:16, signed).
 * - **POSITION_240**: Hall sensor position for 240° (bits 15:0, signed).
 */
struct HALL_POSITION_300_240 {
    static constexpr uint16_t ADDRESS = 0x0C7;
    union {
        uint32_t value;
        struct {
            int16_t POSITION_300 : 16; ///< Hall sensor angle at 300° electrical.
            int16_t POSITION_240 : 16; ///< Hall sensor angle at 240° electrical.
        } bits;
    };
    static constexpr int16_t RESET_POSITION_300 = 0xD555; ///< Default reset value for POSITION_300.
    static constexpr int16_t RESET_POSITION_240 = 0xAAAA; ///< Default reset value for POSITION_240.
};

/// Velocity Biquad Filter Coefficient A1 (Address 0x0E0, Block 0).
/**
 * A1 coefficient for the velocity PI controller's biquad (filter).
 * - **BIQUAD_V_A1**: 24-bit signed value.
 */
struct BIQUAD_V_A1 {
    static constexpr uint16_t ADDRESS = 0x0E0;
    int32_t A1 : 24; ///< Biquad velocity filter coefficient A1.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_V_A1 = 0x1C376B; ///< Default reset value for BIQUAD_V_A1.
};

/// Velocity Biquad Filter Coefficient A2 (Address 0x0E1, Block 0).
/**
 * A2 coefficient for velocity filter biquad.
 * - **BIQUAD_V_A_2** (bits 23:0): Biquad velocity filter coefficient A_2.
 *   - Type: RW, signed.
 *   - Reset: 0xF38F52.
 */
struct BIQUAD_V_A2 {
    static constexpr uint16_t ADDRESS = 0x0E1;
    int32_t A2 : 24; ///< Biquad velocity filter coefficient A_2.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_V_A2 = 0xF38F52; ///< Default reset value for A2.
};

/// Velocity Biquad Filter Coefficient B0 (Address 0x0E2, Block 0).
/**
 * B0 coefficient for velocity filter biquad.
 * - **BIQUAD_V_B_0** (bits 23:0): Biquad velocity filter coefficient B_0.
 *   - Type: RW, signed.
 *   - Reset: 0x000E51.
 */
struct BIQUAD_V_B0 {
    static constexpr uint16_t ADDRESS = 0x0E2;
    int32_t B0 : 24; ///< Biquad velocity filter coefficient B_0.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_V_B0 = 0x000E51; ///< Default reset value for B0.
};

/// Velocity Biquad Filter Coefficient B1 (Address 0x0E3, Block 0).
/**
 * B1 coefficient for velocity filter biquad.
 * - **BIQUAD_V_B_1** (bits 23:0): Biquad velocity filter coefficient B_1.
 *   - Type: RW, signed.
 *   - Reset: 0x001CA1.
 */
struct BIQUAD_V_B1 {
    static constexpr uint16_t ADDRESS = 0x0E3;
    int32_t B1 : 24; ///< Biquad velocity filter coefficient B_1.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_V_B1 = 0x001CA1; ///< Default reset value for B1.
};

/// Velocity Biquad Filter Coefficient B2 (Address 0x0E4, Block 0).
/**
 * B2 coefficient for velocity filter biquad.
 * - **BIQUAD_V_B_2** (bits 23:0): Biquad velocity filter coefficient B_2.
 *   - Type: RW, signed.
 *   - Reset: 0x000E51.
 */
struct BIQUAD_V_B2 {
    static constexpr uint16_t ADDRESS = 0x0E4;
    int32_t B2 : 24; ///< Biquad velocity filter coefficient B_2.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_V_B2 = 0x000E51; ///< Default reset value for B2.
};

/// Velocity Biquad Filter Enable Register (Address 0x0E5, Block 0).
/**
 * Enable/disable the velocity biquad filter.
 * - **BIQUAD_V_ENABLE** (bit 0): Enable Biquad Velocity Filter.
 *   - Type: RW.
 *   - Reset: 0x1.
 */
struct BIQUAD_V_ENABLE {
    static constexpr uint16_t ADDRESS = 0x0E5;
    union {
        uint32_t value;
        struct {
            uint32_t ENABLED : 1; ///< 1 to enable velocity filter, 0 to bypass.
            uint32_t : 31;
        } bits;
    };
    static constexpr uint32_t RESET_BIQUAD_V_ENABLE = 0x1; ///< Default reset value for ENABLED.
};

/// Torque (Flux) Biquad Filter Coefficient A1 (Address 0x0E6, Block 0).
/**
 * Biquad torque filter coefficient A_1.
 * - **BIQUAD_T_A_1** (bits 23:0): Coefficient A_1.
 *   - Type: RW, signed.
 *   - Reset: 0x000000.
 */
struct BIQUAD_T_A1 {
    static constexpr uint16_t ADDRESS = 0x0E6;
    int32_t A1 : 24; ///< Biquad torque filter coefficient A_1.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_T_A1 = 0x000000; ///< Default reset value for A1.
};

/// Torque (Flux) Biquad Filter Coefficient A2 (Address 0x0E7, Block 0).
/**
 * Biquad torque filter coefficient A_2.
 * - **BIQUAD_T_A_2** (bits 23:0): Coefficient A_2.
 *   - Type: RW, signed.
 *   - Reset: 0x000000.
 */
struct BIQUAD_T_A2 {
    static constexpr uint16_t ADDRESS = 0x0E7;
    int32_t A2 : 24; ///< Biquad torque filter coefficient A_2.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_T_A2 = 0x000000; ///< Default reset value for A2.
};

/// Torque (Flux) Biquad Filter Coefficient B0 (Address 0x0E8, Block 0).
/**
 * Biquad torque filter coefficient B_0.
 * - **BIQUAD_T_B_0** (bits 23:0): Coefficient B_0.
 *   - Type: RW, signed.
 *   - Reset: 0x100000.
 */
struct BIQUAD_T_B0 {
    static constexpr uint16_t ADDRESS = 0x0E8;
    int32_t B0 : 24; ///< Biquad torque filter coefficient B_0.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_T_B0 = 0x100000; ///< Default reset value for B0.
};

/// Torque (Flux) Biquad Filter Coefficient B1 (Address 0x0E9, Block 0).
/**
 * Biquad torque filter coefficient B_1.
 * - **BIQUAD_T_B_1** (bits 23:0): Coefficient B_1.
 *   - Type: RW, signed.
 *   - Reset: 0x000000.
 */
struct BIQUAD_T_B1 {
    static constexpr uint16_t ADDRESS = 0x0E9;
    int32_t B1 : 24; ///< Biquad torque filter coefficient B_1.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_T_B1 = 0x000000; ///< Default reset value for B1.
};

/// Torque (Flux) Biquad Filter Coefficient B2 (Address 0x0EA, Block 0).
/**
 * Biquad torque filter coefficient B_2.
 * - **BIQUAD_T_B_2** (bits 23:0): Coefficient B_2.
 *   - Type: RW, signed.
 *   - Reset: 0x000000.
 */
struct BIQUAD_T_B2 {
    static constexpr uint16_t ADDRESS = 0x0EA;
    int32_t B2 : 24; ///< Biquad torque filter coefficient B_2.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_T_B2 = 0x000000; ///< Default reset value for B2.
};

/// Torque (Flux) Biquad Filter Enable Register (Address 0x0EB, Block 0).
/**
 * Enable/disable the torque biquad filter.
 * - **BIQUAD_T_ENABLE** (bit 0): Enable Biquad Torque Filter.
 *   - Type: RW.
 *   - Reset: 0x0.
 */
struct BIQUAD_T_ENABLE {
    static constexpr uint16_t ADDRESS = 0x0EB;
    union {
        uint32_t value;
        struct {
            uint32_t ENABLE : 1; ///< 1 to enable torque filter, 0 to bypass.
            uint32_t : 31;
        } bits;
    };
    static constexpr uint32_t RESET_BIQUAD_T_ENABLE = 0x0; ///< Default reset value for ENABLED.
};

/// Velocity PI Controller Configuration (Address 0x100, Block 0).
/**
 * Configuration bits for the velocity PI controller.
 * - Includes velocity meter type, synchronization, filtering options, and position source selection.
 * - **MOVING_AVRG_FILTER_SAMPLES** (bits 14:12): Number of velocity samples for moving average filter.
 *   - 0: No additional filter.
 *   - 1-7: Average over 2-8 samples respectively.
 * - **METER_TYPE** (bits 10:9): Velocity meter type selection.
 *   - 0: VELOCITY_PER (time measurement between position changes).
 *   - 1: VELOCITY_FREQ (difference of angle in one clock cycle).
 *   - 2: VELOCITY_EXT (software-provided velocity).
 * - **METER_SYNC_PULSE** (bit 8): Synchronization pulse for velocity meter.
 *   - 0: PWM_Z (start of each PWM cycle).
 *   - 1: PWM_C (center of each PWM cycle).
 * - **SELECTION** (bits 7:0): Source of rotor position for velocity measurement.
 *   - 0: PHI_E (selected through PHI_E_SELECTION).
 *   - 1: PHI_E_EXT.
 *   - 2: PHI_E_RAMP.
 *   - 3: PHI_E_ABN.
 *   - 4: RAMP_X_ACTUAL.
 *   - 5: PHI_E_HAL.
 *   - 6: PHI_M_EXT.
 *   - 8: ABN_COUNT.
 *   - 9: PHI_M_ABN.
 *   - 12: HALL_COUNT.
 */
struct VELOCITY_CONFIG {
    static constexpr uint16_t ADDRESS = 0x100;
    union {
        uint32_t value;
        struct {
            uint32_t SELECTION : 8;               ///< Source of rotor position for velocity measurement.
            uint32_t METER_SYNC_PULSE : 1;        ///< Synchronization pulse for velocity meter.
            uint32_t METER_TYPE : 2;             ///< Velocity meter type selection.
            uint32_t : 1;
            uint32_t MOVING_AVRG_FILTER_SAMPLES : 3; ///< Moving average filter samples.
            uint32_t : 17;
        } bits;
    };
};

/// Velocity Scaling Register (Address 0x101, Block 0: MCC_VELOCITY_SCALING)
/**
 * Scaling factor for velocity meter output. This value is only used when VELOCITY_FREQ in MCC_VELOCITY_CONFIG - METER_TYPE is selected.
 * - **VELOCITY_SCALING** (bits 15:0): Scaling factor for velocity meter output.
 *   - Type: RW, signed.
 *   - Reset: 0x28F6.
 */
struct VELOCITY_SCALING {
    static constexpr uint16_t ADDRESS = 0x101;
    union {
        uint32_t value;
        struct {
            int16_t VELOCITY_SCALING : 16; ///< Scaling factor for velocity meter output.
            uint16_t : 16;
        } bits;
    };
    static constexpr int16_t RESET_VELOCITY_SCALING = 0x28F6; ///< Default reset value for VELOCITY_SCALING.
};

/// Velocity Meter Minimal Deviation and Counter Limit (Address 0x102, Block 0: MCC_V_MIN_POS_DEV_TIME_COUNTER_LIMIT)
/**
 * Velocity meter configuration. These values are only used when VELOCITY_PER in MCC_VELOCITY_CONFIG - METER_TYPE is selected.
 * - **V_MIN_POS_DEV** (bits 30:16): Minimal position deviation to calculate velocity.
 *   - Type: RW, unsigned.
 *   - Reset: 0x001.
 * - **TIME_COUNTER_LIMIT** (bits 15:0): Counter limit for velocity minimum deviation functionality.
 *   - Type: RW, unsigned.
 *   - Reset: 0xFFF0.
 */
struct V_MIN_POSDEV_TIME {
    static constexpr uint16_t ADDRESS = 0x102;
    union {
        uint32_t value;
        struct {
            uint32_t TIME_COUNTER_LIMIT : 16; ///< Counter limit for velocity minimum deviation functionality.
            uint32_t V_MIN_POS_DEV : 15;   ///< Minimal position deviation to calculate velocity.
            uint32_t : 1;
        } bits;
    };
    static constexpr uint32_t RESET_V_MIN_POS_DEV = 0x001; ///< Default reset value for V_MIN_POS_DEV.
    static constexpr uint32_t RESET_TIME_COUNTER_LIMIT = 0xFFF0; ///< Default reset value for TIME_COUNTER_LIMIT.
};

/// Maximum Velocity Deviation Register (Address 0x103, Block 0: MCC_MAX_VEL_DEVIATION)
/**
 * Velocity deviation to generate tracking error flag.
 * - **MAX_VEL_DEVIATION** (bits 30:0): Maximum allowed absolute velocity deviation/error.
 *   - Type: RW, unsigned.
 *   - Reset: 0x0010000.
 */
struct MAX_VEL_DEVIATION {
    static constexpr uint16_t ADDRESS = 0x103;
    union {
        uint32_t value;
        struct {
            uint32_t MAX_VEL_DEVIATION : 31; ///< Maximum allowed absolute velocity deviation/error.
            uint32_t : 1;
        } bits;
    };
    static constexpr uint32_t RESET_MAX_VEL_DEVIATION = 0x0010000; ///< Default reset value for MAX_VEL_DEVIATION.
};

/// Position Control Configuration (Address 0x120, Block 0).
/**
 * Configuration of position measurement and control settings.
 * - **SELECTION** (bits 7:0): Source of position feedback for position control.
 *   - 0x0: Use PHI_E (electrical angle selected by PHI_E_SELECTION).
 *   - 0x5: Use hall sensor with extrapolation (if hall phi_e with extrapolation is used, instead of 0x0).
 */
struct POSITION_CONFIG {
    static constexpr uint16_t ADDRESS = 0x120;
    union {
        uint32_t value;
        struct {
            uint8_t SELECTION;  ///< Position measurement source selection.
            uint8_t : 24;
        } bits;
    };
};

/// Maximum Position Deviation Register (Address 0x121, Block 0).
/**
 * Maximum allowed position error for tracking before a tracking error is flagged.
 * If the position controller's absolute error exceeds this value, a tracking error (STALL_IN_POS_ERR) is set and can trigger a motor stop.
 */
struct MAX_POS_DEVIATION {
    static constexpr uint16_t ADDRESS = 0x121;
    union {
        uint32_t value;
        struct {
            uint32_t MAX_POS_ERR : 31;  ///< Maximum allowed position error (unsigned).
            uint32_t : 1;
        } bits;
    };
};

/// Ramp Status and Switch Event Register (Address 0x140, Block 0).
/**
 * Contains status flags for the ramp generator and limit switch events.
 * - Bits (R=read-only flags, RW= read/write/clearable):
 *   - **STALL_IN_POS_ERR** (bit 17, R): Position deviation exceeds MAX_POS_DEVIATION (tracking error). If enabled, a ramp stop is activated.
 *   - **STALL_IN_VEL_ERR** (bit 16, R): Velocity deviation exceeds MAX_VEL_DEVIATION. If enabled, a ramp stop is activated.
 *   - **SECOND_MOVE** (bit 15, RW, W1C): Indicates ramp had to reverse direction (second move), e.g., due to on-the-fly parameter change.
 *   - **T_ZEROWAIT_ACTIVE** (bit 14, R): Indicates ramp is in zero-velocity wait phase.
 *   - **V_ZERO** (bit 13, R): Indicates current velocity is zero.
 *   - **POSITION_REACHED** (bit 12, R): Target position reached (X_ACTUAL == X_TARGET).
 *   - **VELOCITY_REACHED** (bit 11, R): Target velocity reached (V_ACTUAL == V_MAX).
 *   - **EVENT_POS_REACHED** (bit 10, RW, W1C): Signals target position reached (cleared by writing '1').
 *   - **EVENT_STOP_SG** (bit 9, RW, W1C): Signals an active stop event (cleared by writing '1').
 *   - **EVENT_STOP_H** (bit 8, R): Signals an active stop home condition due to stop switch.
 *   - **EVENT_STOP_R** (bit 7, R): Signals an active stop right condition due to stop switch.
 *   - **EVENT_STOP_L** (bit 6, R): Signals an active stop left condition due to stop switch.
 *   - **STATUS_LATCH_H** (bit 5, RW, W1C): Latch home ready (cleared by writing '1').
 *   - **STATUS_LATCH_R** (bit 4, RW, W1C): Latch right ready (cleared by writing '1').
 *   - **STATUS_LATCH_L** (bit 3, RW, W1C): Latch left ready (cleared by writing '1').
 *   - **STATUS_STOP_H** (bit 2, R): Home reference switch status.
 *   - **STATUS_STOP_R** (bit 1, R): Right reference switch status.
 *   - **STATUS_STOP_L** (bit 0, R): Left reference switch status.
 */
struct RAMP_STATUS {
    static constexpr uint16_t ADDRESS = 0x140;
    union {
        uint32_t value;
        struct {
            uint32_t STATUS_STOP_L      : 1;  ///< Left reference switch status.
            uint32_t STATUS_STOP_R      : 1;  ///< Right reference switch status.
            uint32_t STATUS_STOP_H      : 1;  ///< Home reference switch status.
            uint32_t STATUS_LATCH_L     : 1;  ///< Latch left ready.
            uint32_t STATUS_LATCH_R     : 1;  ///< Latch right ready.
            uint32_t STATUS_LATCH_H     : 1;  ///< Latch home ready.
            uint32_t EVENT_STOP_L       : 1;  ///< Active stop left condition.
            uint32_t EVENT_STOP_R       : 1;  ///< Active stop right condition.
            uint32_t EVENT_STOP_H       : 1;  ///< Active stop home condition.
            uint32_t EVENT_STOP_SG      : 1;  ///< Active stop event.
            uint32_t EVENT_POS_REACHED  : 1;  ///< Target position reached event.
            uint32_t VELOCITY_REACHED   : 1;  ///< Target velocity reached.
            uint32_t POSITION_REACHED   : 1;  ///< Target position reached.
            uint32_t V_ZERO             : 1;  ///< Velocity is zero.
            uint32_t T_ZEROWAIT_ACTIVE  : 1;  ///< Zero-wait time active after stop.
            uint32_t SECOND_MOVE        : 1;  ///< Second move (reverse) was required.
            uint32_t STALL_IN_VEL_ERR   : 1;  ///< Velocity tracking error (stall).
            uint32_t STALL_IN_POS_ERR   : 1;  ///< Position tracking error (stall).
            uint32_t : 14;
        } bits;
    };
};

/// Ramp Generator Acceleration A1 (Address 0x141, Block 0: MCC_RAMPER_A1)
/**
 * First acceleration value during EigthPoint ramp mode.
 * - **RAMPER_A1** (bits 22:0): Acceleration value if RAMPER_V_START (resp. 0) < abs(RAMPER_V_ACTUAL) < RAMPER_V1.
 *   - Type: RW, unsigned.
 *   - Reset: 0x10000.
 */
struct RAMP_A1 {
    static constexpr uint16_t ADDRESS = 0x141;
    uint32_t RAMPER_A1 : 23; ///< Acceleration value for the first ramp segment.
    uint32_t : 9;
};

/// Ramp Generator Acceleration A2 (Address 0x142, Block 0: MCC_RAMPER_A2)
/**
 * Second acceleration value during EigthPoint ramp mode.
 * - **RAMPER_A2** (bits 22:0): Acceleration value if RAMPER_V1 < abs(RAMPER_V_ACTUAL) < RAMPER_V2.
 *   - Type: RW, unsigned.
 *   - Reset: 0x10000.
 */
struct RAMP_A2 {
    static constexpr uint16_t ADDRESS = 0x142;
    uint32_t RAMPER_A2 : 23; ///< Acceleration value for the second ramp segment.
    uint32_t : 9;
};

/// Ramp Generator Maximum Acceleration (Address 0x143, Block 0: MCC_RAMPER_A_MAX)
/**
 * Maximum acceleration value in the top part of EigthPoint ramp mode.
 * - **RAMPER_A_MAX** (bits 22:0): Acceleration value if RAMPER_V2 < abs(RAMPER_V_ACTUAL) < RAMPER_V_MAX (resp. RAMPER_V_TARGET).
 *   - Type: RW, unsigned.
 *   - Reset: 0x10000.
 */
struct RAMP_A_MAX {
    static constexpr uint16_t ADDRESS = 0x143;
    uint32_t RAMPER_A_MAX : 23; ///< Maximum acceleration value for the top ramp segment.
    uint32_t : 9;
};

/// Ramp Generator Deceleration D1 (Address 0x144, Block 0: MCC_RAMPER_D1)
/**
 * Lower deceleration value during EigthPoint ramp mode.
 * - **RAMPER_D1** (bits 22:0): Last deceleration value if RAMPER_V_STOP (resp. 0) < abs(RAMPER_V_ACTUAL) < RAMPER_V1.
 *   - Type: RW, unsigned.
 *   - Reset: 0x10000.
 */
struct RAMP_D1 {
    static constexpr uint16_t ADDRESS = 0x144;
    uint32_t RAMPER_D1 : 23; ///< Deceleration value for the last ramp segment.
    uint32_t : 9;
};

/// Ramp Generator Deceleration D2 (Address 0x145, Block 0: MCC_RAMPER_D2)
/**
 * Higher deceleration value in EigthPoint ramp mode.
 * - **RAMPER_D2** (bits 22:0): Deceleration value if RAMPER_V1 < abs(RAMPER_V_ACTUAL) < RAMPER_V2.
 *   - Type: RW, unsigned.
 *   - Reset: 0x10000.
 *   - Description: Used during soft-stop or with ramp in position mode, not for regular ramp velocity mode.
 */
struct RAMP_D2 {
    static constexpr uint16_t ADDRESS = 0x145;
    uint32_t RAMPER_D2 : 23; ///< Deceleration value for the higher ramp segment.
    uint32_t : 9;
};

/// Ramp Generator Maximum Deceleration (Address 0x146, Block 0: MCC_RAMPER_D_MAX)
/**
 * Deceleration in the top part of EigthPoint ramp mode.
 * - **RAMPER_D_MAX** (bits 22:0): Deceleration value if RAMPER_V2 < abs(RAMPER_V_ACTUAL) < RAMPER_V_MAX (resp. RAMPER_V_TARGET).
 *   - Type: RW, unsigned.
 *   - Reset: 0x10000.
 *   - Description: Used during soft-stop or with ramp in position mode, not for regular ramp velocity mode.
 */
struct RAMP_D_MAX {
    static constexpr uint16_t ADDRESS = 0x146;
    uint32_t RAMPER_D_MAX : 23; ///< Maximum deceleration value for the top ramp segment.
    uint32_t : 9;
};

/// Ramp Generator Start Velocity (Address 0x147, Block 0: MCC_RAMPER_V_START)
/**
 * First velocity value during EigthPoint ramp mode.
 * - **RAMPER_V_START** (bits 22:0): Start velocity of position ramp mode when V_ACTUAL = 0 or crossing 0 during motion.
 *   - Type: RW, unsigned.
 *   - Reset: 0x00100.
 *   - Description: Not used during ramp velocity mode.
 */
struct RAMP_V_START {
    static constexpr uint16_t ADDRESS = 0x147;
    uint32_t RAMPER_V_START : 23; ///< Start velocity for position ramp mode.
    uint32_t : 9;
};

/// Ramp Generator Threshold Velocity 1 (Address 0x148, Block 0: MCC_RAMPER_V1)
/**
 * First velocity value for ac-/deceleration value switching during EigthPoint ramp mode.
 * - **RAMPER_V1** (bits 26:0): Velocity value to switch from RAMPER_A1 to RAMPER_A2 during acceleration (ramp positioning mode) and deceleration phase (RAMPER_D1 and RAMPER_D2 during ramp velocity mode only).
 *   - Type: RW, unsigned.
 *   - Reset: 0x000000.
 */
struct RAMP_V1 {
    static constexpr uint16_t ADDRESS = 0x148;
    uint32_t RAMPER_V1 : 27; ///< Velocity threshold for switching acceleration/deceleration values.
    uint32_t : 5;
};

/// Ramp Generator Threshold Velocity 2 (Address 0x149, Block 0: MCC_RAMPER_V2)
/**
 * Second velocity value for ac-/deceleration value switching during EigthPoint ramp mode.
 * - **RAMPER_V2** (bits 26:0): Velocity value to switch to from RAMPER_A2 to RAMPER_A_MAX during acceleration (ramp positioning mode) and deceleration phase (RAMPER_D2 and RAMPER_D_MAX during ramp velocity mode only).
 *   - Type: RW, unsigned.
 *   - Reset: 0x000000.
 */
struct RAMP_V2 {
    static constexpr uint16_t ADDRESS = 0x149;
    uint32_t RAMPER_V2 : 27; ///< Second velocity threshold for switching acceleration/deceleration values.
    uint32_t : 5;
};

/// Ramp Generator Stop Velocity (Address 0x14A, Block 0: MCC_RAMPER_V_STOP)
/**
 * Stop velocity in ramp in position ramp mode.
 * - **RAMPER_V_STOP** (bits 22:0): Velocity used before reaching the target position.
 *   - Type: RW, unsigned.
 *   - Reset: 0x00100.
 *   - Description: Not used during ramp velocity mode.
 */
struct RAMP_V_STOP {
    static constexpr uint16_t ADDRESS = 0x14A;
    uint32_t RAMPER_V_STOP : 23; ///< Stop velocity for position ramp mode.
    uint32_t : 9;
};

/// Ramp Generator Maximum Velocity (Address 0x14B, Block 0: MCC_RAMPER_V_MAX)
/**
 * Maximum velocity value for positioning in EigthPoint ramp mode.
 * - **RAMPER_V_MAX** (bits 26:0): Maximum velocity value of EigthPoint Ramp in ramp positioning mode.
 *   - Type: RW, unsigned.
 *   - Reset: 0x7FFFFFF.
 */
struct RAMP_V_MAX {
    static constexpr uint16_t ADDRESS = 0x14B;
    uint32_t RAMPER_V_MAX : 27; ///< Maximum velocity value for ramp positioning mode.
    uint32_t : 5;
};

/// Ramp Generator Target Velocity (Address 0x14C, Block 0: MCC_RAMPER_V_TARGET)
/**
 * Target velocity value in EigthPoint ramp mode.
 * - **RAMPER_V_TARGET** (bits 27:0): Target velocity in ramp velocity mode.
 *   - Type: RW, signed.
 *   - Reset: 0x0000000.
 */
struct RAMP_V_TARGET {
    static constexpr uint16_t ADDRESS = 0x14C;
    int32_t RAMPER_V_TARGET : 28; ///< Target velocity for ramp velocity mode.
    uint32_t : 4;
};

/// Ramp Generator Switch Mode Register (Address 0x14D, Block 0: MCC_RAMPER_SWITCH_MODE)
/**
 * Configures behavior of stop switches (if using endstops or reference switches).
 * - **VELOCITY_OVERWRITE** (bit 19): If enabled, velocity from overwrite input (PID_VELOCITY_TARGET) is written to ramp.
 * - **STOP_ON_VEL_DEVIATION** (bit 18): Enables a hard stop during ramp mode if velocity tracking error emerges.
 * - **STOP_ON_POS_DEVIATION** (bit 17): Enables a hard stop during ramp mode if position tracking error emerges.
 * - **SW_HARD_STOP** (bit 16): Enables a hard stop during ramp mode in case any activated reference switch has been triggered.
 * - **SOFTSTOP_ENABLE** (bit 15): Enables soft stop mode using deceleration ramp settings.
 * - **SG_STOP_ENABLE** (bit 14): Enables stop conditions like SW_HARD_STOP, STOP_ON_POS_DEVIATION, STOP_ON_VEL_DEVIATION.
 * - **LATCH_H_INACTIVE** (bit 12): Activates position latching to RAMPER_X_ACTUAL_LATCH when home reference switch is deactivated.
 * - **LATCH_H_ACTIVE** (bit 11): Activates position latching to RAMPER_X_ACTUAL_LATCH when home reference switch is activated.
 * - **LATCH_R_INACTIVE** (bit 10): Activates position latching to RAMPER_X_ACTUAL_LATCH when right reference switch is deactivated.
 * - **LATCH_R_ACTIVE** (bit 9): Activates position latching to RAMPER_X_ACTUAL_LATCH when right reference switch is activated.
 * - **LATCH_L_INACTIVE** (bit 8): Activates position latching to RAMPER_X_ACTUAL_LATCH when left reference switch is deactivated.
 * - **LATCH_L_ACTIVE** (bit 7): Activates position latching to RAMPER_X_ACTUAL_LATCH when left reference switch is activated.
 * - **SWAP_LR** (bit 6): Swaps left and right reference switch inputs internally.
 * - **STOP_H_POL** (bit 5): Defines active polarity of the home reference switch input.
 * - **STOP_R_POL** (bit 4): Defines active polarity of the right reference switch input.
 * - **STOP_L_POL** (bit 3): Defines active polarity of the left reference switch input.
 * - **STOP_H_ENABLE** (bit 2): Enables automatic motor stop during active home reference switch input.
 * - **STOP_R_ENABLE** (bit 1): Enables automatic motor stop during active right reference switch input.
 * - **STOP_L_ENABLE** (bit 0): Enables automatic motor stop during active left reference switch input.
 */
struct RAMP_SWITCH_MODE {
    static constexpr uint16_t ADDRESS = 0x14D;
    union {
        uint32_t value;
        struct {
            uint32_t STOP_L_ENABLE       : 1;
            uint32_t STOP_R_ENABLE       : 1;
            uint32_t STOP_H_ENABLE       : 1;
            uint32_t STOP_L_POL          : 1;
            uint32_t STOP_R_POL          : 1;
            uint32_t STOP_H_POL          : 1;
            uint32_t SWAP_LR             : 1;
            uint32_t LATCH_L_ACTIVE      : 1;
            uint32_t LATCH_L_INACTIVE    : 1;
            uint32_t LATCH_R_ACTIVE      : 1;
            uint32_t LATCH_R_INACTIVE    : 1;
            uint32_t LATCH_H_ACTIVE      : 1;
            uint32_t LATCH_H_INACTIVE    : 1;
            uint32_t SG_STOP_ENABLE      : 1;
            uint32_t SOFTSTOP_ENABLE     : 1;
            uint32_t SW_HARD_STOP        : 1;
            uint32_t STOP_ON_POS_DEVIATION : 1;
            uint32_t STOP_ON_VEL_DEVIATION : 1;
            uint32_t VELOCITY_OVERWRITE  : 1;
            uint32_t : 13;
        } bits;
    };
};

/// Ramp Generator Timing Configuration Register (Address 0x14E, Block 0: MCC_RAMPER_TIME_CONFIG)
/**
 * Timing settings for the ramp generator jerk reduction.
 * - **T_VMAX** (bits 31:16, unsigned): Minimum time at constant velocity (in 12.8µs units) before deceleration.
 * - **T_ZEROWAIT** (bits 15:0, unsigned): Wait time after reaching zero velocity before next move (in 12.8µs units).
 */
struct RAMP_TIME_CONFIG {
    static constexpr uint16_t ADDRESS = 0x14E;
    union {
        uint32_t value;
        struct {
            uint16_t T_VMAX     : 16;  ///< Minimum time at constant velocity before deceleration (units of 12.8 µs).
            uint16_t T_ZEROWAIT : 16;  ///< Wait time at zero velocity before next movement (units of 12.8 µs).
        } bits;
    };
    static constexpr uint32_t RESET_T_VMAX = 0x0000; ///< Default reset value for T_VMAX.
    static constexpr uint32_t RESET_T_ZEROWAIT = 0x0000; ///< Default reset value for T_ZEROWAIT.
};

/// Actual Ramp Acceleration Register (Address 0x14F, Block 0: MCC_RAMPER_A_ACTUAL)
/**
 * Actual acceleration value currently being applied by the ramp generator.
 * - **RAMPER_A_ACTUAL**: 24-bit signed actual acceleration.
 */
struct RAMP_A_ACTUAL {
    static constexpr uint16_t ADDRESS = 0x14F;
    union {
        uint32_t value;
        struct {
            int32_t A_ACTUAL : 24;  ///< Actual acceleration (signed).
            uint32_t : 8;
        } bits;
    };
    static constexpr int32_t RESET_RAMPER_A_ACTUAL = 0x000000; ///< Default reset value for RAMPER_A_ACTUAL.
};

/// Actual Ramp Position Register (Address 0x150, Block 0: MCC_RAMPER_X_ACTUAL)
/**
 * Multi-turn position output of the ramp generator.
 * This is the target position that the ramp has generated (follows PID_POSITION_ACTUAL when written).
 */
struct RAMP_X_ACTUAL {
    static constexpr uint16_t ADDRESS = 0x150;
    int32_t X_ACTUAL;  ///< Actual ramp position output (signed 32-bit).
    static constexpr int32_t RESET_RAMPER_X_ACTUAL = 0x00000000; ///< Default reset value for RAMPER_X_ACTUAL.
};

/// Actual Ramp Velocity Register (Address 0x151, Block 0: MCC_RAMPER_V_ACTUAL)
/**
 * Current velocity output of the ramp generator.
 * - **RAMPER_V_ACTUAL** (bits 27:0): Actual velocity output value of ramp controller.
 *   - Type: R, signed.
 *   - Reset: 0x0000000.
 */
struct RAMP_V_ACTUAL {
    static constexpr uint16_t ADDRESS = 0x151;
    union {
        uint32_t value;
        struct {
            int32_t RAMPER_V_ACTUAL : 28; ///< Actual ramp velocity (signed).
            uint32_t : 4;
        } bits;
    };
};

/// Ramp Target Position Register (Address 0x152, Block 0: MCC_RAMPER_X_TARGET)
/**
 * Multi-turn target position of the ramp controller.
 * - **RAMPER_X_TARGET** (bits 31:0): Target position of ramp controller.
 *   - Type: RW, signed.
 *   - Reset: 0x00000000.
 *   - Description: When writing PID_POSITION_ACTUAL, this value is overwritten also.
 */
struct RAMP_X_TARGET {
    static constexpr uint16_t ADDRESS = 0x152;
    int32_t RAMPER_X_TARGET;
};

/// Ramp Electrical Angle Register (Address 0x153, Block 0: MCC_RAMPER_PHI_E)
/**
 * PHI_E of the ramp controller.
 * - **RAMPER_PHI_E** (bits 15:0): PHI_E calculated from RAMPER_X_ACTUAL × N_POLE_PAIRS + Offset.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct RAMP_PHI_E {
    static constexpr uint16_t ADDRESS = 0x153;
    union {
        uint32_t value;
        struct {
            int16_t RAMPER_PHI_E; ///< PHI_E calculated from RAMPER_X_ACTUAL × N_POLE_PAIRS + Offset.
            uint16_t : 16;
        } bits;
    };
};

/// Ramp Feedforward Acceleration Register (Address 0x155, Block 0: MCC_RAMPER_ACC_FF)
/**
 * Gain and shift factor for acceleration feedforward.
 * - **GAIN** (bits 15:0): Gain factor for acceleration feedforward.
 *   - Type: RW, unsigned.
 *   - Reset: 0x0000.
 *   - Description: Result is used as offset for PIDIN_TORQUE_TARGET.
 *     Formula: (RAMPER_A_ACTUAL × GAIN) >> (SHIFT × 4).
 * - **SHIFT** (bits 18:16): Shift factor for acceleration feedforward.
 *   - Type: RW.
 *   - Reset: 0x6.
 *   - Description: Result is used as offset for PIDIN_TORQUE_TARGET.
 *     Formula: (RAMPER_A_ACTUAL × GAIN) >> (SHIFT × 4).
 *     - 0: SHIFT_0 (0).
 *     - 1: SHIFT_4 (4).
 *     - 2: SHIFT_8 (8).
 *     - 3: SHIFT_12 (12).
 *     - 4: SHIFT_16 (16).
 *     - 5: SHIFT_20 (20).
 *     - 6: SHIFT_24 (24).
 */
struct RAMP_ACC_FF {
    static constexpr uint16_t ADDRESS = 0x155;
    union {
        uint32_t value;
        struct {
            uint32_t GAIN  : 16; ///< Gain factor for acceleration feedforward.
            uint32_t SHIFT : 3;  ///< Shift factor for acceleration feedforward.
            uint32_t : 13;
        } bits;
    };
};

/// Latched Multi-turn Position Register (Address 0x156, Block 0: MCC_RAMPER_X_ACTUAL_LATCH)
/**
 * Latches RAMPER_X_ACTUAL on left or right switch or encoder trigger.
 * - **RAMPER_X_ACTUAL_LATCH** (bits 31:0): Latched X-Actual value at stop switch event.
 *   - Type: R, signed.
 *   - Reset: 0x00000000.
 */
struct RAMP_X_ACTUAL_LATCH {
    static constexpr uint16_t ADDRESS = 0x156;
    int32_t RAMPER_X_ACTUAL_LATCH;
};

/// Latched Actual Position Register (Address 0x157, Block 0: MCC_POSITION_ACTUAL_LATCH)
/**
 * Latches PID_POSITION_ACTUAL on left or right switch or encoder trigger.
 * - **POSITION_ACTUAL_LATCH** (bits 31:0): Actual feedback position latch at stop switch event.
 *   - Type: R, signed.
 *   - Reset: 0x00000000.
 */
struct POSITION_ACTUAL_LATCH {
    static constexpr uint16_t ADDRESS = 0x157;
    int32_t POSITION_ACTUAL_LATCH;
};
/// PRBS Amplitude Register (Address 0x160, Block 0: MCC_PRBS_AMPLITUDE)
/**
 * Set the amplitude of the PRBS (Pseudo-Random Binary Sequence) signal used by some settings of MCC_MOTION_CONFIG -> MOTION_MODE.
 * The PRBS signal alternates between +PRBS_AMPLITUDE and -PRBS_AMPLITUDE.
 * - **PRBS_AMPLITUDE** (bits 31:0): Amplitude of the PRBS signal.
 *   - Type: RW, signed.
 *   - Reset: 0x00000000.
 *   - Description:
 *     - Setting this value to 0 resets the random sequence.
 *     - After resetting, the random sequence is always the same.
 *     - This register is useful for testing and debugging motion control systems by introducing controlled noise.
 */
struct PRBS_AMPLITUDE {
    static constexpr uint16_t ADDRESS = 0x160;
    union {
        uint32_t value;
        struct {
            int32_t PRBS_AMPLITUDE : 32; ///< Amplitude of the PRBS signal (signed).
        } bits;
    };
    static constexpr int32_t RESET_PRBS_AMPLITUDE = 0x00000000; ///< Default reset value.
};

/// PRBS Down-Sampling Ratio Register (Address 0x161, Block 0: MCC_PRBS_DOWN_SAMPLING_RATIO)
/**
 * Set the downsampling rate of the PWM frequency to trigger new PRBS value generation.
 * - **PRBS_DOWN_SAMPLING_RATIO** (bits 7:0): Downsampling rate for PRBS generation.
 *   - Type: RW, unsigned.
 *   - Reset: 0x00.
 *   - Description:
 *     - Determines how often a new PRBS value is generated relative to the PWM frequency.
 *     - A higher value reduces the frequency of PRBS updates, effectively slowing down the noise signal.
 *     - Useful for controlling the bandwidth of the PRBS signal.
 */
struct PRBS_DOWNSAMPLING_RATIO {
    static constexpr uint16_t ADDRESS = 0x161;
    union {
        uint32_t value;
        struct {
            uint8_t PRBS_DOWN_SAMPLING_RATIO : 8; ///< Downsampling rate for PRBS generation.
            uint32_t : 24;
        } bits;
    };
    static constexpr uint8_t RESET_PRBS_DOWNSAMPLING_RATIO = 0x00; ///< Default reset value.
};

/// PID Controller Configuration Register (Address 0x180, Block 0: MCC_PID_CONFIG)
/**
 * General configuration for on-chip PI(D) controllers (torque/flux, velocity, position).
 * Includes downsampling factors, normalization, and scaling options.
 * - **KEEP_POS_TARGET** (bit 0): Do not overwrite position target on position actual write.
 *   - 0: OVERWRITE - Overwrite the position target.
 *   - 1: KEEP - Keep the position target unchanged.
 * - **CURRENT_NORM_P** (bit 2): Normalization of P Factor of Current Control.
 *   - 0: SHIFT_8 - Shift 8 bits right.
 *   - 1: SHIFT_16 - Shift 16 bits right.
 * - **CURRENT_NORM_I** (bit 3): Normalization of I Factor of Current Control.
 *   - 0: SHIFT_8 - Shift 8 bits right.
 *   - 1: SHIFT_16 - Shift 16 bits right.
 * - **VELOCITY_NORM_P** (bits 5:4): Normalization of P Factor of Velocity Control.
 *   - 0: SHIFT_0 - No shift.
 *   - 1: SHIFT_8 - Shift 8 bits right + VEL_SCALE.
 *   - 2: SHIFT_16 - Shift 16 bits right + VEL_SCALE.
 *   - 3: SHIFT_24 - Shift 24 bits right + VEL_SCALE.
 * - **VELOCITY_NORM_I** (bits 7:6): Normalization of I Factor of Velocity Control.
 *   - 0: SHIFT_8 - Shift 8 bits right + VEL_SCALE.
 *   - 1: SHIFT_16 - Shift 16 bits right + VEL_SCALE.
 *   - 2: SHIFT_24 - Shift 24 bits right + VEL_SCALE.
 *   - 3: SHIFT_32 - Shift 32 bits right + VEL_SCALE.
 * - **POSITION_NORM_P** (bits 9:8): Normalization of P Factor of Position Control.
 *   - 0: SHIFT_0 - No shift.
 *   - 1: SHIFT_8 - Shift 8 bits right.
 *   - 2: SHIFT_16 - Shift 16 bits right.
 *   - 3: SHIFT_24 - Shift 24 bits right.
 * - **POSITION_NORM_I** (bits 11:10): Normalization of I Factor of Position Control.
 *   - 0: SHIFT_8 - Shift 8 bits right.
 *   - 1: SHIFT_16 - Shift 16 bits right.
 *   - 2: SHIFT_24 - Shift 24 bits right.
 *   - 3: SHIFT_32 - Shift 32 bits right.
 * - **VEL_SCALE** (bits 15:12): Output right shift factor of the velocity controller.
 *   - Type: RW, unsigned.
 *   - Reset: 0x8.
 * - **POS_SMPL** (bits 22:16): Downsampling factor for Position controller.
 *   - Type: RW, unsigned.
 *   - Reset: 0x0.
 * - **VEL_SMPL** (bits 30:24): Downsampling factor for Velocity controller.
 *   - Type: RW, unsigned.
 *   - Reset: 0x0.
 */
struct PID_CONFIG {
    static constexpr uint16_t ADDRESS = 0x180;
    union {
        uint32_t value;
        struct {
            uint32_t KEEP_POS_TARGET    : 1;  ///< Do not overwrite position target on position actual write.
            uint32_t CURRENT_NORM_P    : 1;  ///< Normalization of P Factor of Current Control.
            uint32_t CURRENT_NORM_I    : 1;  ///< Normalization of I Factor of Current Control.
            uint32_t VELOCITY_NORM_P   : 2;  ///< Normalization of P Factor of Velocity Control.
            uint32_t VELOCITY_NORM_I   : 2;  ///< Normalization of I Factor of Velocity Control.
            uint32_t POSITION_NORM_P   : 2;  ///< Normalization of P Factor of Position Control.
            uint32_t POSITION_NORM_I   : 2;  ///< Normalization of I Factor of Position Control.
            uint32_t VEL_SCALE         : 4;  ///< Output right shift factor of the velocity controller.
            uint32_t POS_SMPL          : 7;  ///< Downsampling factor for Position controller.
            uint32_t VEL_SMPL          : 7;  ///< Downsampling factor for Velocity controller.
            uint32_t : 5;
        } bits;
    };
    static constexpr uint32_t RESET_PID_CONFIG = 0x00000800; ///< Default reset value.
};

/// Flux PI Controller Coefficients Register (Address 0x181, Block 0: MCC_PID_FLUX_COEFF)
/**
 * Configuration of the PI Flux controller gains.
 * - **P** (bits 31:16, signed): Proportional gain for the PI Flux controller (default 0x0000).
 * - **I** (bits 15:0, signed): Integral gain for the PI Flux controller (default 0x0000).
 */
struct PID_FLUX_COEFF {
    static constexpr uint16_t ADDRESS = 0x181;
    union {
        uint32_t value;
        struct {
            int16_t P;  ///< Proportional gain for the PI Flux controller.
            int16_t I;  ///< Integral gain for the PI Flux controller.
        } bits;
    };
};

/// Torque PI Controller Coefficients Register (Address 0x182, Block 0: MCC_PID_TORQUE_COEFF)
/**
 * Configuration of the PI Torque controller gains.
 * - **P** (bits 31:16, signed): Proportional gain for the PI Torque controller (default 0x0000).
 * - **I** (bits 15:0, signed): Integral gain for the PI Torque controller (default 0x0000).
 */
struct PID_TORQUE_COEFF {
    static constexpr uint16_t ADDRESS = 0x182;
    union {
        uint32_t value;
        struct {
            int16_t P;  ///< Proportional gain for the PI Torque controller.
            int16_t I;  ///< Integral gain for the PI Torque controller.
        } bits;
    };
};

/// Field Weakening PI Controller Coefficients Register (Address 0x183, Block 0: MCC_PID_FIELDWEAK_COEFF)
/**
 * Configuration of the PI Fieldweakening controller gains.
 * - **P** (bits 31:16): Proportional gain for the PI Fieldweakening controller (default 0x0000).
 * - **I** (bits 15:0): Integral gain for the PI Fieldweakening controller (default 0x0000).
 */
struct PID_FIELDWEAK_COEFF {
    static constexpr uint16_t ADDRESS = 0x183;
    union {
        uint32_t value;
        struct {
            int16_t P;  ///< Proportional gain for the PI Fieldweakening controller.
            int16_t I;  ///< Integral gain for the PI Fieldweakening controller.
        } bits;
    };
};

/// Maximum Voltage (U<sub>S</sub>) Register (Address 0x184, Block 0: MCC_PID_U_S_MAX)
/**
 * Maximum voltage allowed for fieldweakening.
 * - **U_S_MAX** (bits 15:0): Maximum voltage allowed for fieldweakening (default 0x7FFF).
 */
struct PID_U_S_MAX {
    static constexpr uint16_t ADDRESS = 0x184;
    uint16_t U_S_MAX;  ///< Maximum voltage allowed for fieldweakening.
    uint16_t _reserved;
};

/// Velocity PI Controller Coefficients Register (Address 0x185, Block 0: MCC_PID_VELOCITY_COEFF)
/**
 * Configuration of the PI Velocity controller gains.
 * - **P** (bits 31:16): Proportional gain for the PI Velocity controller (default 0x0000).
 * - **I** (bits 15:0): Integral gain for the PI Velocity controller (default 0x0000).
 */
struct PID_VELOCITY_COEFF {
    static constexpr uint16_t ADDRESS = 0x185;
    union {
        uint32_t value;
        struct {
            int16_t P;  ///< Proportional gain for the PI Velocity controller.
            int16_t I;  ///< Integral gain for the PI Velocity controller.
        } bits;
    };
};

/// Position PI Controller Coefficients Register (Address 0x186, Block 0: MCC_PID_POSITION_COEFF)
/**
 * Configuration of the PI Position controller gains.
 * - **P** (bits 31:16, signed): Proportional gain for the PI Position controller (default 0x0000).
 * - **I** (bits 15:0, signed): Integral gain for the PI Position controller (default 0x0000).
 */
struct PID_POSITION_COEFF {
    static constexpr uint16_t ADDRESS = 0x186;
    union {
        uint32_t value;
        struct {
            int16_t P;  ///< Proportional gain for the PI Position controller.
            int16_t I;  ///< Integral gain for the PI Position controller.
        } bits;
    };
    static constexpr int16_t RESET_P = 0x0000; ///< Default reset value for P.
    static constexpr int16_t RESET_I = 0x0000; ///< Default reset value for I.
};

/// Position Tolerance Register (Address 0x187, Block 0: MCC_PID_POSITION_TOLERANCE)
/**
 * Position controller ignores position errors smaller than PID_POSITION_TOLERANCE if EVENT_POS_REACHED
 * and after (PID_POSITION_TOLERANCE_DELAY × PWM period).
 * - **PID_POSITION_TOLERANCE** (bits 30:0, unsigned): Position error tolerance (default 0x0000000).
 */
struct PID_POSITION_TOLERANCE {
    static constexpr uint16_t ADDRESS = 0x187;
    union {
        uint32_t value;
        struct {
            uint32_t PID_POSITION_TOLERANCE : 31; ///< Position error tolerance.
            uint32_t : 1;
        } bits;
    };
    static constexpr uint32_t RESET_PID_POSITION_TOLERANCE = 0x0000000; ///< Default reset value.
};

/// Position Tolerance Delay Register (Address 0x188, Block 0: MCC_PID_POSITION_TOLERANCE_DELAY)
/**
 * Number of PWM periods the abs(PID_POSITION_ERROR) must stay within PID_POSITION_TOLERANCE
 * after EVENT_POS_REACHED to disable the controller.
 * - **PID_POSITION_TOLERANCE_DELAY** (bits 15:0, unsigned): PWM periods to hold within tolerance (default 0x0000).
 */
struct PID_POSITION_TOLERANCE_DELAY {
    static constexpr uint16_t ADDRESS = 0x188;
    union {
        uint32_t value;
        struct {
            uint16_t PID_POSITION_TOLERANCE_DELAY; ///< PWM periods to hold within tolerance.
            uint16_t : 16;
        } bits;
    };
    static constexpr uint16_t RESET_PID_POSITION_TOLERANCE_DELAY = 0x0000; ///< Default reset value.
};

/// Voltage Limit Register (Address 0x189, Block 0: MCC_PID_UQ_UD_LIMITS)
/**
 * Set maximum output voltage limit value.
 * - **PID_UQ_UD_LIMITS** (bits 15:0, unsigned): Maximum voltage limit (default 0x5A81).
 *   - Limits U_D to PID_UQ_UD_LIMITS and U_Q to sqrt(PID_UQ_UD_LIMITS^2 - U_D^2).
 *   - If set above 0x3FFF (16383), the limiter uses that value instead.
 *   - If MCC_PWM_CONFIG -> SV_MODE is set to use third harmonic injection (SV_MODE not 0x0),
 *     the internal maximum voltage limit is 18900 (0x49D4).
 */
struct PID_UQ_UD_LIMITS {
    static constexpr uint16_t ADDRESS = 0x189;
    union {
        uint32_t value;
        struct {
            uint16_t PID_UQ_UD_LIMITS; ///< Maximum voltage limit.
            uint16_t : 16;
        } bits;
    };
    static constexpr uint16_t RESET_PID_UQ_UD_LIMITS = 0x5A81; ///< Default reset value.
};

/// Torque/Flux Current Limit Register (Address 0x18A, Block 0: MCC_PID_TORQUE_FLUX_LIMITS)
/**
 * Set maximum target absolute current for torque and flux PI controller.
 * - **PID_TORQUE_LIMIT** (bits 30:16): Max absolute target torque current (unsigned).
 *   - Type: RW, unsigned.
 *   - Reset: 0x7FFF.
 *   - Description: Limits the torque target value from torque target register and velocity controller output.
 * - **PID_FLUX_LIMIT** (bits 14:0): Max absolute target flux current (unsigned).
 *   - Type: RW, unsigned.
 *   - Reset: 0x7FFF.
 *   - Description: Limits the target values from Flux weakening controller and register.
 */
struct PID_TORQUE_FLUX_LIMITS {
    static constexpr uint16_t ADDRESS = 0x18A;
    union {
        uint32_t value;
        struct {
            uint32_t PID_TORQUE_LIMIT : 15;  ///< Maximum torque current target.
            uint32_t : 1;
            uint32_t PID_FLUX_LIMIT   : 15;  ///< Maximum flux current target.
            uint32_t : 1;
        } bits;
    };
};

/// Velocity Limit Register (Address 0x18B, Block 0: MCC_PID_VELOCITY_LIMIT)
/**
 * Set maximum absolute velocity for velocity PI controller.
 * - **PID_VELOCITY_LIMIT** (bits 30:0): Maximum velocity limit.
 *   - Type: RW, unsigned.
 *   - Reset: 0x7FFFFFFF.
 *   - Description: Limits the velocity target value from velocity target register and position controller output.
 */
struct PID_VELOCITY_LIMIT {
    static constexpr uint16_t ADDRESS = 0x18B;
    uint32_t PID_VELOCITY_LIMIT : 31;
    uint32_t : 1;
};

/// Position Limit Low Register (Address 0x18C, Block 0: MCC_PID_POSITION_LIMIT_LOW)
/**
 * Set minimum target position for position PI controller.
 * - **PID_POSITION_LIMIT_LOW** (bits 31:0): Minimum position limit.
 *   - Type: RW, signed.
 *   - Reset: 0x80000001.
 *   - Description: Programmable position barrier.
 */
struct PID_POSITION_LIMIT_LOW {
    static constexpr uint16_t ADDRESS = 0x18C;
    int32_t PID_POSITION_LIMIT_LOW;
};

/// Position Limit High Register (Address 0x18D, Block 0: MCC_PID_POSITION_LIMIT_HIGH)
/**
 * Set maximum target position for position PI controller.
 * - **PID_POSITION_LIMIT_HIGH** (bits 31:0): Maximum position limit.
 *   - Type: RW, signed.
 *   - Reset: 0x7FFFFFFF.
 *   - Description: Programmable position barrier.
 */
struct PID_POSITION_LIMIT_HIGH {
    static constexpr uint16_t ADDRESS = 0x18D;
    int32_t PID_POSITION_LIMIT_HIGH;
};

/// Torque/Flux Target Register (Address 0x18E, Block 0: MCC_PID_TORQUE_FLUX_TARGET)
/**
 * PID target torque and target flux (for torque mode).
 * - **PID_TORQUE_TARGET** (bits 31:16): Target torque.
 *   - Type: RW, signed.
 *   - Reset: 0x0000.
 * - **PID_FLUX_TARGET** (bits 15:0): Target flux.
 *   - Type: RW, signed.
 *   - Reset: 0x0000.
 */
struct PID_TORQUE_FLUX_TARGET {
    static constexpr uint16_t ADDRESS = 0x18E;
    union {
        uint32_t value;
        struct {
            int16_t PID_TORQUE_TARGET; ///< Target torque.
            int16_t PID_FLUX_TARGET;   ///< Target flux.
        } bits;
    };
};

/// Torque/Flux Offset Register (Address 0x18F, Block 0: MCC_PID_TORQUE_FLUX_OFFSET)
/**
 * PID torque and flux offset.
 * - **PID_TORQUE_OFFSET** (bits 31:16): Torque offset for feedforward control.
 *   - Type: RW, signed.
 *   - Reset: 0x0000.
 * - **PID_FLUX_OFFSET** (bits 15:0): Flux offset for feedforward control.
 *   - Type: RW, signed.
 *   - Reset: 0x0000.
 */
struct PID_TORQUE_FLUX_OFFSET {
    static constexpr uint16_t ADDRESS = 0x18F;
    union {
        uint32_t value;
        struct {
            int16_t PID_TORQUE_OFFSET; ///< Torque offset.
            int16_t PID_FLUX_OFFSET;   ///< Flux offset.
        } bits;
    };
};

/// Velocity Target Register (Address 0x190, Block 0: MCC_PID_VELOCITY_TARGET)
/**
 * PID Target velocity (for velocity mode).
 * - **PID_VELOCITY_TARGET** (bits 31:0): Target velocity.
 *   - Type: RW, signed.
 *   - Reset: 0x00000000.
 */
struct PID_VELOCITY_TARGET {
    static constexpr uint16_t ADDRESS = 0x190;
    int32_t PID_VELOCITY_TARGET;
};

/// Velocity Offset Register (Address 0x191, Block 0: MCC_PID_VELOCITY_OFFSET)
/**
 * PID velocity offset for feedforward control.
 * - **PID_VELOCITY_OFFSET** (bits 31:0): Velocity offset.
 *   - Type: RW, signed.
 *   - Reset: 0x00000000.
 */
struct PID_VELOCITY_OFFSET {
    static constexpr uint16_t ADDRESS = 0x191;
    int32_t PID_VELOCITY_OFFSET;
};

/// Position Target Register (Address 0x192, Block 0: MCC_PID_POSITION_TARGET)
/**
 * Target position register (for position mode).
 * - **PID_POSITION_TARGET** (bits 31:0): Target position.
 *   - Type: RW, signed.
 *   - Reset: 0x00000000.
 */
struct PID_POSITION_TARGET {
    static constexpr uint16_t ADDRESS = 0x192;
    int32_t PID_POSITION_TARGET;
};

/// Torque/Flux Actual Register (Address 0x193, Block 0: MCC_PID_TORQUE_FLUX_ACTUAL)
/**
 * PID actual torque and flux.
 * - **PID_TORQUE_ACTUAL** (bits 31:16): Actual torque.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 * - **PID_FLUX_ACTUAL** (bits 15:0): Actual flux.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct PID_TORQUE_FLUX_ACTUAL {
    static constexpr uint16_t ADDRESS = 0x193;
    union {
        uint32_t value;
        struct {
            int16_t PID_TORQUE_ACTUAL; ///< Actual torque.
            int16_t PID_FLUX_ACTUAL;   ///< Actual flux.
        } bits;
    };
};

/// Velocity Actual Register (Address 0x194, Block 0: MCC_PID_VELOCITY_ACTUAL)
/**
 * PID actual velocity.
 * - **PID_VELOCITY_ACTUAL** (bits 31:0): Actual velocity.
 *   - Type: R, signed.
 *   - Reset: 0x00000000.
 */
struct PID_VELOCITY_ACTUAL {
    static constexpr uint16_t ADDRESS = 0x194;
    int32_t PID_VELOCITY_ACTUAL;
};

/// Position Actual Register (Address 0x195, Block 0: MCC_PID_POSITION_ACTUAL)
/**
 * PID actual position.
 * - **PID_POSITION_ACTUAL** (bits 31:0): Actual position.
 *   - Type: RW, signed.
 *   - Reset: 0x00000000.
 *   - Description: Writing to this register also writes the same value into PID_POSITION_TARGET to avoid unwanted moves.
 */
struct PID_POSITION_ACTUAL {
    static constexpr uint16_t ADDRESS = 0x195;
    int32_t PID_POSITION_ACTUAL;
};

/// Position Actual Offset Register (Address 0x196, Block 0: MCC_PID_POSITION_ACTUAL_OFFSET)
/**
 * Offset for actual position.
 * - **PID_POSITION_ACTUAL_OFFSET** (bits 31:0): Position offset.
 *   - Type: RW, signed.
 *   - Reset: 0x00000000.
 */
struct PID_POSITION_ACTUAL_OFFSET {
    static constexpr uint16_t ADDRESS = 0x196;
    int32_t PID_POSITION_ACTUAL_OFFSET;
};

/// Torque Error Register (Address 0x197, Block 0: MCC_PID_TORQUE_ERROR)
/**
 * PID torque error.
 * - **PID_TORQUE_ERROR** (bits 15:0): Torque error.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct PID_TORQUE_ERROR {
    static constexpr uint16_t ADDRESS = 0x197;
    int16_t PID_TORQUE_ERROR;
};

/// Flux Error Register (Address 0x198, Block 0: MCC_PID_FLUX_ERROR)
/**
 * PID flux error.
 * - **PID_FLUX_ERROR** (bits 15:0): Flux error.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct PID_FLUX_ERROR {
    static constexpr uint16_t ADDRESS = 0x198;
    int16_t PID_FLUX_ERROR;
};

/// Velocity Error Register (Address 0x199, Block 0: MCC_PID_VELOCITY_ERROR)
/**
 * PID velocity error.
 * - **PID_VELOCITY_ERROR** (bits 31:0): Velocity error.
 *   - Type: R, signed.
 *   - Reset: 0x00000000.
 */
struct PID_VELOCITY_ERROR {
    static constexpr uint16_t ADDRESS = 0x199;
    int32_t PID_VELOCITY_ERROR;
};

/// Position Error Register (Address 0x19A, Block 0: MCC_PID_POSITION_ERROR)
/**
 * PID position error.
 * - **PID_POSITION_ERROR** (bits 31:0): Position error.
 *   - Type: R, signed.
 *   - Reset: 0x00000000.
 */
struct PID_POSITION_ERROR {
    static constexpr uint16_t ADDRESS = 0x19A;
    int32_t PID_POSITION_ERROR;
};

/// Torque Integrator Register (Address 0x19B, Block 0: MCC_PID_TORQUE_INTEGRATOR)
/**
 * PID torque integrator.
 * - **PID_TORQUE_INTEGRATOR** (bits 31:0): Torque integrator.
 *   - Type: RW, signed.
 *   - Reset: 0x00000000.
 */
struct PID_TORQUE_INTEGRATOR {
    static constexpr uint16_t ADDRESS = 0x19B;
    int32_t PID_TORQUE_INTEGRATOR;
};

/// Flux Integrator Register (Address 0x19C, Block 0: MCC_PID_FLUX_INTEGRATOR)
/**
 * PID flux integrator.
 * - **PID_FLUX_INTEGRATOR** (bits 31:0): Flux integrator.
 *   - Type: RW, signed.
 *   - Reset: 0x00000000.
 */
struct PID_FLUX_INTEGRATOR {
    static constexpr uint16_t ADDRESS = 0x19C;
    int32_t PID_FLUX_INTEGRATOR;
};

/// Velocity Integrator Register (Address 0x19D, Block 0: MCC_PID_VELOCITY_INTEGRATOR)
/**
 * PID velocity integrator.
 * - **PID_VELOCITY_INTEGRATOR** (bits 31:0): Velocity integrator.
 *   - Type: RW, signed.
 *   - Reset: 0x00000000.
 */
struct PID_VELOCITY_INTEGRATOR {
    static constexpr uint16_t ADDRESS = 0x19D;
    int32_t PID_VELOCITY_INTEGRATOR;
};

/// Position Integrator Register (Address 0x19E, Block 0: MCC_PID_POSITION_INTEGRATOR)
/**
 * PID position integrator.
 * - **PID_POSITION_INTEGRATOR** (bits 31:0): Position integrator.
 *   - Type: RW, signed.
 *   - Reset: 0x00000000.
 */
struct PID_POSITION_INTEGRATOR {
    static constexpr uint16_t ADDRESS = 0x19E;
    int32_t PID_POSITION_INTEGRATOR;
};

/// PID Input - Torque/Flux Target Register (Address 0x1A0, Block 0: MCC_PIDIN_TORQUE_FLUX_TARGET)
/**
 * PID target torque and target flux for readback.
 * - **PIDIN_TORQUE_TARGET** (bits 31:16): Torque target before any filtering/limiting.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 * - **PIDIN_FLUX_TARGET** (bits 15:0): Flux target before any filtering/limiting.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct PIDIN_TORQUE_FLUX_TARGET {
    static constexpr uint16_t ADDRESS = 0x1A0;
    union {
        uint32_t value;
        struct {
            int16_t PIDIN_TORQUE_TARGET; ///< Torque target before filtering/limiting.
            int16_t PIDIN_FLUX_TARGET;   ///< Flux target before filtering/limiting.
        } bits;
    };
};

/// PID Input - Velocity Target Register (Address 0x1A1, Block 0: MCC_PIDIN_VELOCITY_TARGET)
/**
 * PID target velocity for readback.
 * - **PIDIN_VELOCITY_TARGET** (bits 31:0): Velocity target before any filtering/limiting.
 *   - Type: R, signed.
 *   - Reset: 0x00000000.
 */
struct PIDIN_VELOCITY_TARGET {
    static constexpr uint16_t ADDRESS = 0x1A1;
    int32_t PIDIN_VELOCITY_TARGET; ///< Velocity target before filtering/limiting.
};

/// PID Input - Position Target Register (Address 0x1A2, Block 0: MCC_PIDIN_POSITION_TARGET)
/**
 * PID target position for readback.
 * - **PIDIN_POSITION_TARGET** (bits 31:0): Position target before any filtering/limiting.
 *   - Type: R, signed.
 *   - Reset: 0x00000000.
 */
struct PIDIN_POSITION_TARGET {
    static constexpr uint16_t ADDRESS = 0x1A2;
    int32_t PIDIN_POSITION_TARGET; ///< Position target before filtering/limiting.
};

/// PID Input - Limited Torque/Flux Target Register (Address 0x1A3, Block 0: MCC_PIDIN_TORQUE_FLUX_TARGET_LIMITED)
/**
 * PID target torque and target flux after PID_TORQUE_FLUX_LIMITS applied.
 * - **PIDIN_TORQUE_TARGET_LIMITED** (bits 31:16): Torque target after limiter.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 * - **PIDIN_FLUX_TARGET_LIMITED** (bits 15:0): Flux target after limiter.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct PIDIN_TORQUE_FLUX_TARGET_LIMITED {
    static constexpr uint16_t ADDRESS = 0x1A3;
    union {
        uint32_t value;
        struct {
            int16_t PIDIN_TORQUE_TARGET_LIMITED; ///< Torque target after limiter.
            int16_t PIDIN_FLUX_TARGET_LIMITED;   ///< Flux target after limiter.
        } bits;
    };
};

/// PID Input - Limited Velocity Target Register (Address 0x1A4, Block 0: MCC_PIDIN_VELOCITY_TARGET_LIMITED)
/**
 * PID target velocity after PID_VELOCITY_LIMIT applied.
 * - **PIDIN_VELOCITY_TARGET_LIMITED** (bits 31:0): Velocity target after limiter.
 *   - Type: R, signed.
 *   - Reset: 0x00000000.
 */
struct PIDIN_VELOCITY_TARGET_LIMITED {
    static constexpr uint16_t ADDRESS = 0x1A4;
    int32_t PIDIN_VELOCITY_TARGET_LIMITED; ///< Velocity target after limiter.
};

/// PID Input - Limited Position Target Register (Address 0x1A5, Block 0: MCC_PIDIN_POSITION_TARGET_LIMITED)
/**
 * PID target position after PID_POSITION_LIMIT_LOW and PID_POSITION_LIMIT_HIGH applied.
 * - **PIDIN_POSITION_TARGET_LIMITED** (bits 31:0): Position target after limiter.
 *   - Type: R, signed.
 *   - Reset: 0x00000000.
 */
struct PIDIN_POSITION_TARGET_LIMITED {
    static constexpr uint16_t ADDRESS = 0x1A5;
    int32_t PIDIN_POSITION_TARGET_LIMITED; ///< Limited position target.
};

/// FOC Interim Result - IALPHA and IBETA (Address 0x1A6, Block 0: MCC_FOC_IBETA_IALPHA)
/**
 * Interim result of the FOC, IALPHA, and IBETA term.
 * - **IBETA** (bits 31:16): Interim result of the FOC, IBETA term.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 * - **IALPHA** (bits 15:0): Interim result of the FOC, IALPHA term.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct FOC_IBETA_IALPHA {
    static constexpr uint16_t ADDRESS = 0x1A6;
    union {
        uint32_t value;
        struct {
            int16_t IALPHA; ///< Interim result of the FOC, IALPHA term.
            int16_t IBETA;  ///< Interim result of the FOC, IBETA term.
        } bits;
    };
};

/// FOC Interim Result - ID and IQ (Address 0x1A7, Block 0: MCC_FOC_IQ_ID)
/**
 * Interim result of the FOC, ID, and IQ term.
 * - **IQ** (bits 31:16): Interim result of the FOC, IQ term.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 * - **ID** (bits 15:0): Interim result of the FOC, ID term.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct FOC_IQ_ID {
    static constexpr uint16_t ADDRESS = 0x1A7;
    union {
        uint32_t value;
        struct {
            int16_t ID; ///< Interim result of the FOC, ID term.
            int16_t IQ; ///< Interim result of the FOC, IQ term.
        } bits;
    };
};

/// FOC Interim Result - UD and UQ (Address 0x1A8, Block 0: MCC_FOC_UQ_UD)
/**
 * Interim result of the FOC, UD, and UQ term.
 * - **UQ** (bits 31:16): Interim result of the FOC, UQ term.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 * - **UD** (bits 15:0): Interim result of the FOC, UD term.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct FOC_UQ_UD {
    static constexpr uint16_t ADDRESS = 0x1A8;
    union {
        uint32_t value;
        struct {
            int16_t UD; ///< Interim result of the FOC, UD term.
            int16_t UQ; ///< Interim result of the FOC, UQ term.
        } bits;
    };
};

/// FOC Interim Result - Limited UD and UQ (Address 0x1A9, Block 0: MCC_FOC_UQ_UD_LIMITED)
/**
 * Interim result of the FOC, UD, and UQ term after PID_UQ_UD_LIMITS applied.
 * - **UQ** (bits 31:16): Interim result of the FOC, UQ term limited.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 * - **UD** (bits 15:0): Interim result of the FOC, UD term limited.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct FOC_UQ_UD_LIMITED {
    static constexpr uint16_t ADDRESS = 0x1A9;
    union {
        uint32_t value;
        struct {
            int16_t UD; ///< Interim result of the FOC, UD term limited.
            int16_t UQ; ///< Interim result of the FOC, UQ term limited.
        } bits;
    };
};

/// FOC Interim Result - UALPHA and UBETA (Address 0x1AA, Block 0: MCC_FOC_UBETA_UALPHA)
/**
 * Interim result of the FOC, UALPHA, and UBETA term.
 * - **UBETA** (bits 31:16): Interim result of the FOC, UBETA term.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 * - **UALPHA** (bits 15:0): Interim result of the FOC, UALPHA term.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct FOC_UBETA_UALPHA {
    static constexpr uint16_t ADDRESS = 0x1AA;
    union {
        uint32_t value;
        struct {
            int16_t UALPHA; ///< Interim result of the FOC, UALPHA term.
            int16_t UBETA;  ///< Interim result of the FOC, UBETA term.
        } bits;
    };
};

/// FOC Interim Result - UUX and UWY (Address 0x1AB, Block 0: MCC_FOC_UWY_UUX)
/**
 * Interim result of the FOC, UUX, and UWY term.
 * - **UWY** (bits 31:16): Interim result of the FOC, UWY term.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 * - **UUX** (bits 15:0): Interim result of the FOC, UUX term.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct FOC_UWY_UUX {
    static constexpr uint16_t ADDRESS = 0x1AB;
    union {
        uint32_t value;
        struct {
            int16_t UUX; ///< Interim result of the FOC, UUX term.
            int16_t UWY; ///< Interim result of the FOC, UWY term.
        } bits;
    };
};

/// FOC Interim Result - UV (Address 0x1AC, Block 0: MCC_FOC_UV)
/**
 * Interim result of the FOC, UV term.
 * - **UV** (bits 15:0): Interim result of the FOC, UV term.
 *   - Type: R, signed.
 *   - Reset: 0x0000.
 */
struct FOC_UV {
    static constexpr uint16_t ADDRESS = 0x1AC;
    int16_t UV; ///< Interim result of the FOC, UV term.
};

/// PWM Duty Cycle - UX1 and VX2 (Address 0x1AD, Block 0: MCC_PWM_VX2_UX1)
/**
 * Interim result PWM duty cycle UX1 and VX2.
 * - **VX2** (bits 31:16): Interim result PWM VX2.
 *   - Type: R, unsigned.
 *   - Reset: 0x0000.
 * - **UX1** (bits 15:0): Interim result PWM UX1.
 *   - Type: R, unsigned.
 *   - Reset: 0x0000.
 */
struct PWM_VX2_UX1 {
    static constexpr uint16_t ADDRESS = 0x1AD;
    union {
        uint32_t value;
        struct {
            uint16_t UX1; ///< Interim result PWM UX1.
            uint16_t VX2; ///< Interim result PWM VX2.
        } bits;
    };
};

/// PWM Duty Cycle - WY1 and Y2 (Address 0x1AE, Block 0: MCC_PWM_Y2_WY1)
/**
 * Interim result PWM duty cycle WY1 and Y2.
 * - **Y2** (bits 31:16): Interim result PWM Y2.
 *   - Type: R, unsigned.
 *   - Reset: 0x0000.
 * - **WY1** (bits 15:0): Interim result PWM WY1.
 *   - Type: R, unsigned.
 *   - Reset: 0x0000.
 */
struct PWM_Y2_WY1 {
    static constexpr uint16_t ADDRESS = 0x1AE;
    union {
        uint32_t value;
        struct {
            uint16_t WY1; ///< Interim result PWM WY1.
            uint16_t Y2;  ///< Interim result PWM Y2.
        } bits;
    };
};

/// Velocity (Fixed Frequency) Measurement Register (Address 0x1AF, Block 0: MCC_VELOCITY_FRQ)
/**
 * Actual velocity measured by fixed frequency sampling.
 * - **VELOCITY_FRQ** (bits 31:0): Actual velocity measured by fixed frequency sampling.
 *   - Type: R, signed.
 *   - Reset: 0x00000000.
 */
struct VELOCITY_FRQ {
    static constexpr uint16_t ADDRESS = 0x1AF;
    int32_t VELOCITY_FRQ; ///< Actual velocity measured by fixed frequency sampling.
};

/// Velocity (Period) Measurement Register (Address 0x1B0, Block 0).
/**
 * Actual velocity measured using period measurement (time between increments).
 * - **VELOCITY_PER** (bits 31:0): Actual velocity measured by period measurement.
 *   - Type: R, signed.
 *   - Reset: 0x00000000.
 */
struct VELOCITY_PER {
    static constexpr uint16_t ADDRESS = 0x1B0;
    int32_t VELOCITY_PER_VAL; ///< Actual velocity measured by period measurement.
};

/// Motor Voltage & Current Actual Values Register (Address 0x1C0, Block 0).
/**
 * Real-time values of the magnitude of voltage (U<sub>S</sub>) and current (I<sub>S</sub>) vectors.
 * - **U_S_ACTUAL** (bits 31:16): Actual motor voltage magnitude.
 *   - Type: R, unsigned.
 *   - Reset: 0x0000.
 *   - Description: U_S_ACTUAL = sqrt(UD^2 + UQ^2).
 * - **I_S_ACTUAL** (bits 15:0): Actual motor current magnitude.
 *   - Type: R, unsigned.
 *   - Reset: 0x0000.
 *   - Description: I_S_ACTUAL = sqrt(ID^2 + IQ^2).
 */
struct U_S_ACTUAL_I_S_ACTUAL {
    static constexpr uint16_t ADDRESS = 0x1C0;
    union {
        uint32_t value;
        struct {
            uint16_t U_S_ACTUAL; ///< Actual motor voltage magnitude.
            uint16_t I_S_ACTUAL; ///< Actual motor current magnitude.
        } bits;
    };
};

/// Motor Power (Mechanical) Measurement Register (Address 0x1C1, Block 0).
/**
 * Estimated mechanical output power of the motor.
 * - **P_MOTOR** (bits 31:0): Actual power applied to the motor.
 *   - Type: R, unsigned.
 *   - Reset: 0x00000000.
 *   - Description: P_MOTOR = U_S_ACTUAL × I_S_ACTUAL.
 */
struct P_MOTOR {
    static constexpr uint16_t ADDRESS = 0x1C1;
    uint32_t P_MECH; ///< Actual power applied to the motor.
};

/// Raw Digital Inputs Register (Address 0x1C2, Block 0: MCC_INPUTS_RAW)
/**
 * @brief Represents the raw input signals for the TMC9660 driver.
 * 
 * This structure provides access to raw input signals such as PWM_IN, DIR, STP, 
 * digital Hall inputs, and digital ABN encoder inputs. These signals are directly 
 * read from the pins and are useful for system setup and validation during the 
 * development phase.
 * 
 * Bitfield Description:
 * - [0]  ENC_A       : Encoder signal A directly from pin.
 * - [1]  ENC_B       : Encoder signal B directly from pin.
 * - [2]  ENC_N       : Encoder signal N directly from pin.
 * - [8]  HALL_U      : Hall signal U directly from pin.
 * - [9]  HALL_V      : Hall signal V directly from pin.
 * - [10] HALL_W      : Hall signal W directly from pin.
 * - [12] REF_SW_R    : Right reference switch value directly from pin.
 * - [13] REF_SW_L    : Left reference switch value directly from pin.
 * - [14] REF_SW_H    : Home reference switch value directly from pin.
 * - [15] ENI         : DRV_ENABLE pin value.
 * - [20] HALL_U_FILT : Hall signal U after filter and reordering.
 * - [21] HALL_V_FILT : Hall signal V after filter and reordering.
 * - [22] HALL_W_FILT : Hall signal W after filter and reordering.
 * 
 * Unused bits are reserved and should be ignored.
 */
struct INPUTS_RAW {
    static constexpr uint16_t ADDRESS = 0x1C2;
    union {
        uint32_t value;
        struct {
            uint32_t ENC_A       : 1;  ///< Encoder signal A directly from pin.
            uint32_t ENC_B       : 1;  ///< Encoder signal B directly from pin.
            uint32_t ENC_N       : 1;  ///< Encoder signal N directly from pin.
            uint32_t             : 5;
            uint32_t HALL_U      : 1;  ///< Hall signal U directly from pin.
            uint32_t HALL_V      : 1;  ///< Hall signal V directly from pin.
            uint32_t HALL_W      : 1;  ///< Hall signal W directly from pin.
            uint32_t             : 1;
            uint32_t REF_SW_R    : 1;  ///< Right reference switch value directly from pin.
            uint32_t REF_SW_L    : 1;  ///< Left reference switch value directly from pin.
            uint32_t REF_SW_H    : 1;  ///< Home reference switch value directly from pin.
            uint32_t ENI         : 1;  ///< DRV_ENABLE pin value.
            uint32_t             : 4;
            uint32_t HALL_U_FILT : 1;  ///< Hall signal U after filter and reordering.
            uint32_t HALL_V_FILT : 1;  ///< Hall signal V after filter and reordering.
            uint32_t HALL_W_FILT : 1;  ///< Hall signal W after filter and reordering.
            uint32_t             : 9;
        } bits;
    };
};

/// Raw Digital Outputs Register (Address 0x1C3, Block 0: MCC_OUTPUTS_RAW)
/**
 * @brief Represents the raw output signals for the TMC9660 driver.
 * 
 * This structure provides access to raw output signals for each PWM channel's high- and low-side.
 * These signals are directly related to the PWM phases and are useful for debugging and monitoring
 * the motor control outputs.
 * 
 * Bitfield Description:
 * - [0]  PWM_UX1_L : Value of PWM phase UX1 low side.
 * - [1]  PWM_UX1_H : Value of PWM phase UX1 high side.
 * - [2]  PWM_VX2_L : Value of PWM phase VX2 low side.
 * - [3]  PWM_VX2_H : Value of PWM phase VX2 high side.
 * - [4]  PWM_WY1_L : Value of PWM phase WY1 low side.
 * - [5]  PWM_WY1_H : Value of PWM phase WY1 high side.
 * - [6]  PWM_Y2_L  : Value of PWM phase Y2 low side.
 * - [7]  PWM_Y2_H  : Value of PWM phase Y2 high side.
 * 
 * Unused bits are reserved and should be ignored.
 */
struct OUTPUTS_RAW {
    static constexpr uint16_t ADDRESS = 0x1C3;
    union {
        uint32_t value;
        struct {
            uint32_t PWM_UX1_L : 1; ///< Value of PWM phase UX1 low side.
            uint32_t PWM_UX1_H : 1; ///< Value of PWM phase UX1 high side.
            uint32_t PWM_VX2_L : 1; ///< Value of PWM phase VX2 low side.
            uint32_t PWM_VX2_H : 1; ///< Value of PWM phase VX2 high side.
            uint32_t PWM_WY1_L : 1; ///< Value of PWM phase WY1 low side.
            uint32_t PWM_WY1_H : 1; ///< Value of PWM phase WY1 high side.
            uint32_t PWM_Y2_L  : 1; ///< Value of PWM phase Y2 low side.
            uint32_t PWM_Y2_H  : 1; ///< Value of PWM phase Y2 high side.
            uint32_t : 24;
        } bits;
    };
};

/// General Status Flags Register (Address 0x1C4, Block 0: MCC_STATUS_FLAGS)
/**
 * @brief Represents the general status flags and events for the TMC9660 driver.
 * 
 * This structure provides access to various status flags and events, including error conditions,
 * limiters, and reference switch states. These flags are useful for monitoring the system's state
 * and diagnosing issues during operation.
 * 
 * Bitfield Description:
 * - [0]  PID_X_TARGET_LIMIT      : PI position controller target value limiter active.
 * - [1]  PID_X_OUTPUT_LIMIT      : PI position controller output limiter active.
 * - [2]  PID_V_TARGET_LIMIT      : PI velocity controller target value limiter active.
 * - [3]  PID_V_OUTPUT_LIMIT      : PI velocity controller output limiter active.
 * - [4]  PID_ID_TARGET_LIMIT     : PI flux controller target value limiter active.
 * - [5]  PID_ID_OUTPUT_LIMIT     : PI flux controller output limiter active.
 * - [6]  PID_IQ_TARGET_LIMIT     : PI torque controller target value limiter active.
 * - [7]  PID_IQ_OUTPUT_LIMIT     : PI torque controller output limiter active.
 * - [8]  IPARK_VOLTLIM_LIMIT_U   : Ud or Uq limited by PID_UQ_UD_LIMITS or internal max value.
 * - [9]  PWM_SWITCH_LIMIT_ACTIVE : PWM switch limit active.
 * - [10] HALL_ERROR              : Hall vector error (000 or 111).
 * - [11] POSITION_TRACKING_ERROR : PI position controller error exceeds MAX_POS_DEVIATION.
 * - [12] VELOCITY_TRACKING_ERROR : PI velocity controller error exceeds MAX_VEL_DEVIATION.
 * - [13] PID_FW_OUTPUT_LIMIT     : Field weakening PI controller output limit active.
 * - [16] SHORT                   : HS_FAULT or LS_FAULT triggered.
 * - [20] REF_SW_L                : Left reference switch active.
 * - [21] REF_SW_R                : Right reference switch active.
 * - [22] REF_SW_H                : Home reference switch active.
 * - [23] POSITION_REACHED        : Ramper position reached.
 * - [26] ADC_I_CLIPPED           : ADC current measurement clipped.
 * - [28] ENC_N                   : Filtered encoder signal N active.
 * - [31] ENI                     : Change on DRV_ENABLE pin detected.
 * 
 * Unused bits are reserved and should be ignored.
 */
struct STATUS_FLAGS {
    static constexpr uint16_t ADDRESS = 0x1C4;
    union {
        uint32_t value;
        struct {
            uint32_t PID_X_TARGET_LIMIT         : 1; ///< PI position controller target value limiter active.
            uint32_t PID_X_OUTPUT_LIMIT         : 1; ///< PI position controller output limiter active.
            uint32_t PID_V_TARGET_LIMIT         : 1; ///< PI velocity controller target value limiter active.
            uint32_t PID_V_OUTPUT_LIMIT         : 1; ///< PI velocity controller output limiter active.
            uint32_t PID_ID_TARGET_LIMIT        : 1; ///< PI flux controller target value limiter active.
            uint32_t PID_ID_OUTPUT_LIMIT        : 1; ///< PI flux controller output limiter active.
            uint32_t PID_IQ_TARGET_LIMIT        : 1; ///< PI torque controller target value limiter active.
            uint32_t PID_IQ_OUTPUT_LIMIT        : 1; ///< PI torque controller output limiter active.
            uint32_t IPARK_VOLTLIM_LIMIT_U      : 1; ///< Ud or Uq limited by PID_UQ_UD_LIMITS or internal max value.
            uint32_t PWM_SWITCH_LIMIT_ACTIVE    : 1; ///< PWM switch limit active.
            uint32_t HALL_ERROR                 : 1; ///< Hall vector error (000 or 111).
            uint32_t POSITION_TRACKING_ERROR    : 1; ///< PI position controller error exceeds MAX_POS_DEVIATION.
            uint32_t VELOCITY_TRACKING_ERROR    : 1; ///< PI velocity controller error exceeds MAX_VEL_DEVIATION.
            uint32_t PID_FW_OUTPUT_LIMIT        : 1; ///< Field weakening PI controller output limit active.
            uint32_t                            : 2;
            uint32_t SHORT                      : 1; ///< HS_FAULT or LS_FAULT triggered.
            uint32_t                            : 2;
            uint32_t REF_SW_L                   : 1; ///< Left reference switch active.
            uint32_t REF_SW_R                   : 1; ///< Right reference switch active.
            uint32_t REF_SW_H                   : 1; ///< Home reference switch active.
            uint32_t POSITION_REACHED           : 1; ///< Ramper position reached.
            uint32_t                            : 2;
            uint32_t ADC_I_CLIPPED              : 1; ///< ADC current measurement clipped.
            uint32_t                            : 1;
            uint32_t ENC_N                      : 1; ///< Filtered encoder signal N active.
            uint32_t                            : 2;
            uint32_t ENI                        : 1; ///< Change on DRV_ENABLE pin detected.
        } bits;
    };
};

/// Gate Driver Hardware Configuration Register (Address 0x1E3, Block 0).
/**
 * Controls top-level gate driver enable bits for each half-bridge.
 * - **BRIDGE_ENABLE_U** (bit 0, RW, default 0): Enable bridge U (UX1 low-side & corresponding high-side):contentReference[oaicite:176]{index=176}:contentReference[oaicite:177]{index=177}.
 * - **BRIDGE_ENABLE_V** (bit 1, RW, default 0): Enable bridge V (VX2 half-bridge):contentReference[oaicite:178]{index=178}:contentReference[oaicite:179]{index=179}.
 *   - When disabled, both high and low side FETs of that bridge are off (pulled down).
 */
struct HW_CONFIG {
    static constexpr uint16_t ADDRESS = 0x1E3;
    enum class BridgeEnable : uint8_t { Disabled = 0, Enabled = 1 };  ///< Enumerated state for bridge enable bits.
    union {
        uint32_t value;
        struct {
            BridgeEnable BRIDGE_ENABLE_U : 1;  ///< Enable Bridge U (phase U half-bridge):contentReference[oaicite:180]{index=180}.
            BridgeEnable BRIDGE_ENABLE_V : 1;  ///< Enable Bridge V (phase V half-bridge):contentReference[oaicite:181]{index=181}.
            uint32_t : 30;
        } bits;
    };
};

/// Gate Driver Configuration Register (Address 0x1E4, Block 0: MCC_GDRV_CFG)
/**
 * Configures gate driver behavior and strengths:
 * - **VS_UVLO_LVL** (bits 23:20, RW, default 0x0): Sets the VS undervoltage lockout threshold level.
 *   - Range: 0-15 (4.4V to 8.1V in ~0.2V steps).
 * - **ADAPTIVE_MODE_Y2** (bit 17, RW, default 0x1): Enables adaptive gate drive mode for phase Y2.
 * - **ADAPTIVE_MODE_UVW** (bit 16, RW, default 0x1): Enables adaptive gate drive mode for phases U, V, W.
 * - **IGATE_SOURCE_Y2** (bits 15:12, RW, default 0x0): Gate source current limit for high-side Y2 FET.
 * - **IGATE_SINK_Y2** (bits 11:8, RW, default 0x0): Gate sink current limit for low-side Y2 FET.
 * - **IGATE_SOURCE_UVW** (bits 7:4, RW, default 0x0): Gate source current limit for high-side U/V/W FETs.
 * - **IGATE_SINK_UVW** (bits 3:0, RW, default 0x0): Gate sink current limit for low-side U/V/W FETs.
 */
struct CFG {
    static constexpr uint16_t ADDRESS = 0x1E4;

    /// Undervoltage threshold selection for VS (supply).
    enum class VsUvloLevel : uint8_t {
        VSUVLO_44 = 0,  ///< VS UVLO = 4.4V.
        VSUVLO_46 = 1,  ///< VS UVLO = 4.6V.
        VSUVLO_48 = 2,  ///< VS UVLO = 4.8V.
        VSUVLO_50 = 3,  ///< VS UVLO = 5.0V.
        VSUVLO_52 = 4,  ///< VS UVLO = 5.2V.
        VSUVLO_54 = 5,  ///< VS UVLO = 5.4V.
        VSUVLO_56 = 6,  ///< VS UVLO = 5.6V.
        VSUVLO_58 = 7,  ///< VS UVLO = 5.8V.
        VSUVLO_60 = 8,  ///< VS UVLO = 6.0V.
        VSUVLO_63 = 9,  ///< VS UVLO = 6.3V.
        VSUVLO_66 = 10, ///< VS UVLO = 6.6V.
        VSUVLO_69 = 11, ///< VS UVLO = 6.9V.
        VSUVLO_72 = 12, ///< VS UVLO = 7.2V.
        VSUVLO_75 = 13, ///< VS UVLO = 7.5V.
        VSUVLO_78 = 14, ///< VS UVLO = 7.8V.
        VSUVLO_81 = 15  ///< VS UVLO = 8.1V.
    };

    /// Gate source current options.
    enum class GateSourceCurrent : uint8_t {
        SOURCE_25MA = 0,   ///< 25mA.
        SOURCE_50MA = 1,   ///< 50mA.
        SOURCE_80MA = 2,   ///< 80mA.
        SOURCE_105MA = 3,  ///< 105mA.
        SOURCE_135MA = 4,  ///< 135mA.
        SOURCE_160MA = 5,  ///< 160mA.
        SOURCE_190MA = 6,  ///< 190mA.
        SOURCE_215MA = 7,  ///< 215mA.
        SOURCE_290MA = 8,  ///< 290mA.
        SOURCE_360MA = 9,  ///< 360mA.
        SOURCE_430MA = 10, ///< 430mA.
        SOURCE_500MA = 11, ///< 500mA.
        SOURCE_625MA = 12, ///< 625mA.
        SOURCE_755MA = 13, ///< 755mA.
        SOURCE_885MA = 14, ///< 885mA.
        SOURCE_1000MA = 15 ///< 1000mA.
    };

    /// Gate sink current options.
    enum class GateSinkCurrent : uint8_t {
        SINK_50MA = 0,    ///< 50mA.
        SINK_100MA = 1,   ///< 100mA.
        SINK_160MA = 2,   ///< 160mA.
        SINK_210MA = 3,   ///< 210mA.
        SINK_270MA = 4,   ///< 270mA.
        SINK_320MA = 5,   ///< 320mA.
        SINK_380MA = 6,   ///< 380mA.
        SINK_430MA = 7,   ///< 430mA.
        SINK_580MA = 8,   ///< 580mA.
        SINK_720MA = 9,   ///< 720mA.
        SINK_860MA = 10,  ///< 860mA.
        SINK_1000MA = 11, ///< 1000mA.
        SINK_1250MA = 12, ///< 1250mA.
        SINK_1510MA = 13, ///< 1510mA.
        SINK_1770MA = 14, ///< 1770mA.
        SINK_2000MA = 15  ///< 2000mA.
    };

    union {
        uint32_t value;
        struct {
            GateSinkCurrent IGATE_SINK_UVW   : 4;  ///< Gate sink current setting for UVW.
            GateSourceCurrent IGATE_SOURCE_UVW : 4;  ///< Gate source current setting for UVW.
            GateSinkCurrent IGATE_SINK_Y2    : 4;  ///< Gate sink current setting for Y2.
            GateSourceCurrent IGATE_SOURCE_Y2  : 4;  ///< Gate source current setting for Y2.
            uint32_t ADAPTIVE_MODE_UVW: 1;  ///< Adaptive gate discharge for UVW (1=enabled).
            uint32_t ADAPTIVE_MODE_Y2 : 1;  ///< Adaptive gate discharge for Y2 (1=enabled).
            uint32_t : 2;
            VsUvloLevel VS_UVLO_LVL   : 4;  ///< VS undervoltage lockout threshold setting.
            uint32_t : 12;
        } bits;
    };
};

/// Gate Driver Timing Register (Address 0x1E9, Block 0: MCC_GDRV_TIMING)
/**
 * Sets the T_DRIVE sink and source times for all channels.
 * - **T_DRIVE_SOURCE_Y2** (bits 31:24): Charge time of the MOSFET for Y2 (default 0xFF).
 *   - Description: During this time, the full gate drive current is applied.
 *     The applied time is defined as "(1s / PWM_CLK) × (2 × TDRIVE + 5)".
 * - **T_DRIVE_SINK_Y2** (bits 23:16): Discharge time of the MOSFET for Y2 (default 0xFF).
 *   - Description: During this time, the full gate drive current is applied.
 *     The applied time is defined as "(1s / PWM_CLK) × (2 × TDRIVE + 5)".
 * - **T_DRIVE_SOURCE_UVW** (bits 15:8): Charge time of the MOSFET for UVW (default 0xFF).
 *   - Description: During this time, the full gate drive current is applied.
 *     The applied time is defined as "(1s / PWM_CLK) × (2 × TDRIVE + 5)".
 * - **T_DRIVE_SINK_UVW** (bits 7:0): Discharge time of the MOSFET for UVW (default 0xFF).
 *   - Description: During this time, the full gate drive current is applied.
 *     The applied time is defined as "(1s / PWM_CLK) × (2 × TDRIVE + 5)".
 */
struct TIMING {
    static constexpr uint16_t ADDRESS = 0x1E9;
    union {
        uint32_t value;
        struct {
            uint8_t T_DRIVE_SINK_UVW;    ///< Low-side drive (sink) time for UVW FETs.
            uint8_t T_DRIVE_SOURCE_UVW;  ///< High-side drive (source) time for UVW FETs.
            uint8_t T_DRIVE_SINK_Y2;     ///< Low-side drive time for Y2 FET.
            uint8_t T_DRIVE_SOURCE_Y2;   ///< High-side drive time for Y2 FET.
        } bits;
    };
};

/// Gate Driver Blanking and Deadtime Register (Address 0x1EA, Block 0: MCC_GDRV_BBM)
/**
 * Controls the BBM_L and BBM_H times for all channels.
 * - **BBM_H_Y2** (bits 31:24): Break Before Make time for high-side Y2 (default 0x14).
 *   - Description: "(1s / PWM_CLK) × (BBM + 1)" for high-side MOSFET gate control.
 *     Applies before switching from low to high. Should usually be set to zero in favor of T_DRIVE.
 * - **BBM_L_Y2** (bits 23:16): Break Before Make time for low-side Y2 (default 0x14).
 *   - Description: "(1s / PWM_CLK) × (BBM + 1)" for low-side MOSFET gate control.
 *     Applies before switching from high to low. Should usually be set to zero in favor of T_DRIVE.
 * - **BBM_H_UVW** (bits 15:8): Break Before Make time for high-side UVW (default 0x14).
 *   - Description: "(1s / PWM_CLK) × (BBM + 1)" for high-side MOSFET gate control.
 *     Applies before switching from low to high. Should usually be set to zero in favor of T_DRIVE.
 * - **BBM_L_UVW** (bits 7:0): Break Before Make time for low-side UVW (default 0x14).
 *   - Description: "(1s / PWM_CLK) × (BBM + 1)" for low-side MOSFET gate control.
 *     Applies before switching from high to low. Should usually be set to zero in favor of T_DRIVE.
 */
struct BBM {
    static constexpr uint16_t ADDRESS = 0x1EA;
    union {
        uint32_t value;
        struct {
            uint8_t BBM_L_UVW;  ///< Deadtime (off time) after turning off UVW low-side before high-side can turn on.
            uint8_t BBM_H_UVW;  ///< Deadtime for UVW high-side.
            uint8_t BBM_L_Y2;   ///< Deadtime for Y2 low-side.
            uint8_t BBM_H_Y2;   ///< Deadtime for Y2 high-side.
        } bits;
    };
};

/// Gate Driver Protection Register (Address 0x1EB, Block 0: MCC_GDRV_PROT)
/**
 * @brief Protection settings for MCC_GDRV_PROT (Block 0, Address 0x1EB).
 * 
 * This structure represents the general and VGS-related protection settings.
 * It includes bit fields for various configuration options such as retry counts,
 * VGS blanking and deglitch times, and PWM termination behavior on faults.
 * 
 * Bit Field Description:
 * - [28] TERM_PWM_ON_SHORT: Terminate PWM on other phases if a fault occurs.
 *   - 0: OFF - Keep PWM running for other phases.
 *   - 1: ON - Terminate PWM for other phases.
 * 
 * - [23:22] HS_RETRIES_Y2: High-side Y2 retry count on fault.
 *   - 0: OFF - No retries.
 *   - 1: ONE - 1 retry.
 *   - 2: TWO - 2 retries.
 *   - 3: THREE - 3 retries.
 * 
 * - [21:20] LS_RETRIES_Y2: Low-side Y2 retry count on fault.
 *   - 0: OFF - No retries.
 *   - 1: ONE - 1 retry.
 *   - 2: TWO - 2 retries.
 *   - 3: THREE - 3 retries.
 * 
 * - [19:18] HS_RETRIES_UVW: High-side UVW retry count on fault.
 *   - 0: OFF - No retries.
 *   - 1: ONE - 1 retry.
 *   - 2: TWO - 2 retries.
 *   - 3: THREE - 3 retries.
 * 
 * - [17:16] LS_RETRIES_UVW: Low-side UVW retry count on fault.
 *   - 0: OFF - No retries.
 *   - 1: ONE - 1 retry.
 *   - 2: TWO - 2 retries.
 *   - 3: THREE - 3 retries.
 * 
 * - [13:12] VGS_BLANKING_Y2: VGS short blanking time for Y2.
 *   - 0: BLK_OFF - Off.
 *   - 1: BLK_250NS - 0.25 µs.
 *   - 2: BLK_500NS - 0.5 µs.
 *   - 3: BLK_1000NS - 1 µs.
 * 
 * - [10:8] VGS_DEGLITCH_Y2: VGS short deglitch time for Y2.
 *   - 0: DEG_OFF - Off.
 *   - 1: DEG_250NS - 0.25 µs.
 *   - 2: DEG_500NS - 0.5 µs.
 *   - 3: DEG_1000NS - 1 µs.
 *   - 4: DEG_2000NS - 2 µs.
 *   - 5: DEG_4000NS - 4 µs.
 *   - 6: DEG_6000NS - 6 µs.
 *   - 7: DEG_8000NS - 8 µs.
 * 
 * - [5:4] VGS_BLANKING_UVW: VGS short blanking time for UVW.
 *   - 0: BLK_OFF - Off.
 *   - 1: BLK_250NS - 0.25 µs.
 *   - 2: BLK_500NS - 0.5 µs.
 *   - 3: BLK_1000NS - 1 µs.
 * 
 * - [2:0] VGS_DEGLITCH_UVW: VGS short deglitch time for UVW.
 *   - 0: DEG_OFF - Off.
 *   - 1: DEG_250NS - 0.25 µs.
 *   - 2: DEG_500NS - 0.5 µs.
 *   - 3: DEG_1000NS - 1 µs.
 *   - 4: DEG_2000NS - 2 µs.
 *   - 5: DEG_4000NS - 4 µs.
 *   - 6: DEG_6000NS - 6 µs.
 *   - 7: DEG_8000NS - 8 µs.
 */
struct PROT {
    static constexpr uint16_t ADDRESS = 0x1EB;

    /// Enum for TERM_PWM_ON_SHORT field.
    enum class TermPwmOnShort : uint8_t {
        OFF = 0, ///< Keep PWM running for other phases.
        ON = 1   ///< Terminate PWM for other phases.
    };

    /// Enum for retry counts.
    enum class RetryCount : uint8_t {
        OFF = 0, ///< No retries.
        ONE = 1, ///< 1 retry.
        TWO = 2, ///< 2 retries.
        THREE = 3 ///< 3 retries.
    };

    /// Enum for VGS blanking time.
    enum class VgsBlanking : uint8_t {
        BLK_OFF = 0, ///< Off.
        BLK_250NS = 1, ///< 0.25 µs.
        BLK_500NS = 2, ///< 0.5 µs.
        BLK_1000NS = 3 ///< 1 µs.
    };

    /// Enum for VGS deglitch time.
    enum class VgsDeglitch : uint8_t {
        DEG_OFF = 0, ///< Off.
        DEG_250NS = 1, ///< 0.25 µs.
        DEG_500NS = 2, ///< 0.5 µs.
        DEG_1000NS = 3, ///< 1 µs.
        DEG_2000NS = 4, ///< 2 µs.
        DEG_4000NS = 5, ///< 4 µs.
        DEG_6000NS = 6, ///< 6 µs.
        DEG_8000NS = 7 ///< 8 µs.
    };

    union {
        uint32_t value;
        struct {
            VgsDeglitch VGS_DEGLITCH_UVW : 3; ///< VGS short deglitch time for UVW.
            VgsBlanking VGS_BLANKING_UVW : 2; ///< VGS short blanking time for UVW.
            uint32_t : 1;
            VgsDeglitch VGS_DEGLITCH_Y2 : 3;  ///< VGS short deglitch time for Y2.
            VgsBlanking VGS_BLANKING_Y2 : 2;  ///< VGS short blanking time for Y2.
            uint32_t : 2;
            RetryCount LS_RETRIES_UVW : 2;   ///< Low-side UVW retry count on fault.
            RetryCount HS_RETRIES_UVW : 2;   ///< High-side UVW retry count.
            RetryCount LS_RETRIES_Y2 : 2;    ///< Low-side Y2 retry count.
            RetryCount HS_RETRIES_Y2 : 2;    ///< High-side Y2 retry count.
            uint32_t : 3;
            TermPwmOnShort TERM_PWM_ON_SHORT : 1; ///< Terminate PWM on other phases if fault occurs.
            uint32_t : 3;
        } bits;
    };
};

/// Overcurrent Protection (OCP) UVW Register (Address 0x1EC, Block 0: MCC_GDRV_OCP_UVW)
/**
 * Configures the overcurrent protection for phases U, V, and W.
 * 
 * Bit Field Description:
 * - [27:24] HS_OCP_THRES_UVW: Threshold of the high-side overcurrent protection.
 *   - 0: THRES_63MV (63mV)
 *   - 1: THRES_125MV (125mV)
 *   - 2: THRES_187MV (187mV)
 *   - 3: THRES_248MV (248mV)
 *   - 4: THRES_312MV (312mV)
 *   - 5: THRES_374MV (374mV)
 *   - 6: THRES_434MV (434mV)
 *   - 7: THRES_504MV (504mV)
 *   - 8: THRES_705MV (705mV)
 *   - 9: THRES_940MV (940mV)
 *   - 10: THRES_1180MV (1180mV)
 *   - 11: THRES_1410MV (1410mV)
 *   - 12: THRES_1650MV (1650mV)
 *   - 13: THRES_1880MV (1880mV)
 *   - 14: THRES_2110MV (2110mV)
 *   - 15: THRES_2350MV (2350mV)
 * 
 * - [22:20] HS_OCP_BLANKING_UVW: OCP blanking time for high-side.
 *   - 0: BLK_OFF (Off)
 *   - 1: BLK_250NS (0.25µs)
 *   - 2: BLK_500NS (0.5µs)
 *   - 3: BLK_1000NS (1µs)
 *   - 4: BLK_2000NS (2µs)
 *   - 5: BLK_4000NS (4µs)
 *   - 6: BLK_6000NS (6µs)
 *   - 7: BLK_8000NS (8µs)
 * 
 * - [18:16] HS_OCP_DEGLITCH_UVW: OCP deglitch time for high-side.
 *   - 0: DEG_OFF (Off)
 *   - 1: DEG_250NS (0.25µs)
 *   - 2: DEG_500NS (0.5µs)
 *   - 3: DEG_1000NS (1µs)
 *   - 4: DEG_2000NS (2µs)
 *   - 5: DEG_4000NS (4µs)
 *   - 6: DEG_6000NS (6µs)
 *   - 7: DEG_8000NS (8µs)
 * 
 * - [15] LS_OCP_USE_VDS_UVW: Switches between shunt and RDSon measurement.
 *   - 0: Use shunt measurement.
 *   - 1: Use RDSon measurement.
 * 
 * - [11:8] LS_OCP_THRES_UVW: Threshold of the low-side overcurrent protection.
 *   - 0: THRES_80_63MV (80mV SHUNT, 63mV VDS)
 *   - 1: THRES_165_125MV (165mV SHUNT, 125mV VDS)
 *   - 2: THRES_250_187MV (250mV SHUNT, 187mV VDS)
 *   - 3: THRES_330_248MV (330mV SHUNT, 248mV VDS)
 *   - 4: THRES_415_312MV (415mV SHUNT, 312mV VDS)
 *   - 5: THRES_500_374MV (500mV SHUNT, 374mV VDS)
 *   - 6: THRES_582_434MV (582mV SHUNT, 434mV VDS)
 *   - 7: THRES_660_504MV (660mV SHUNT, 504mV VDS)
 *   - 8: THRES_125_705MV (125mV SHUNT, 705mV VDS)
 *   - 9: THRES_250_940MV (250mV SHUNT, 940mV VDS)
 *   - 10: THRES_375_1180MV (375mV SHUNT, 1180mV VDS)
 *   - 11: THRES_500_1410MV (500mV SHUNT, 1410mV VDS)
 *   - 12: THRES_625_1650MV (625mV SHUNT, 1650mV VDS)
 *   - 13: THRES_750_1880MV (750mV SHUNT, 1880mV VDS)
 *   - 14: THRES_873_2110MV (873mV SHUNT, 2110mV VDS)
 *   - 15: THRES_1000_2350MV (1000mV SHUNT, 2350mV VDS)
 * 
 * - [6:4] LS_OCP_BLANKING_UVW: OCP blanking time for low-side.
 *   - 0: BLK_OFF (Off)
 *   - 1: BLK_250NS (0.25µs)
 *   - 2: BLK_500NS (0.5µs)
 *   - 3: BLK_1000NS (1µs)
 *   - 4: BLK_2000NS (2µs)
 *   - 5: BLK_4000NS (4µs)
 *   - 6: BLK_6000NS (6µs)
 *   - 7: BLK_8000NS (8µs)
 * 
 * - [2:0] LS_OCP_DEGLITCH_UVW: OCP deglitch time for low-side.
 *   - 0: DEG_OFF (Off)
 *   - 1: DEG_250NS (0.25µs)
 *   - 2: DEG_500NS (0.5µs)
 *   - 3: DEG_1000NS (1µs)
 *   - 4: DEG_2000NS (2µs)
 *   - 5: DEG_4000NS (4µs)
 *   - 6: DEG_6000NS (6µs)
 *   - 7: DEG_8000NS (8µs)
 */
struct OCP_UVW {
    static constexpr uint16_t ADDRESS = 0x1EC;

    /// Enum for OCP thresholds.
    enum class OcpThreshold : uint8_t {
        THRES_63MV = 0, THRES_125MV, THRES_187MV, THRES_248MV,
        THRES_312MV, THRES_374MV, THRES_434MV, THRES_504MV,
        THRES_705MV, THRES_940MV, THRES_1180MV, THRES_1410MV,
        THRES_1650MV, THRES_1880MV, THRES_2110MV, THRES_2350MV
    };

    /// Enum for blanking times.
    enum class BlankingTime : uint8_t {
        BLK_OFF = 0, BLK_250NS, BLK_500NS, BLK_1000NS,
        BLK_2000NS, BLK_4000NS, BLK_6000NS, BLK_8000NS
    };

    /// Enum for deglitch times.
    enum class DeglitchTime : uint8_t {
        DEG_OFF = 0, DEG_250NS, DEG_500NS, DEG_1000NS,
        DEG_2000NS, DEG_4000NS, DEG_6000NS, DEG_8000NS
    };

    union {
        uint32_t value;
        struct {
            DeglitchTime LS_OCP_DEGLITCH_UVW : 3; ///< Low-side OCP deglitch time.
            BlankingTime LS_OCP_BLANKING_UVW : 3; ///< Low-side OCP blanking time.
            uint32_t : 2;
            OcpThreshold LS_OCP_THRES_UVW : 4;   ///< Low-side OCP threshold.
            uint32_t : 1;
            uint32_t LS_OCP_USE_VDS_UVW : 1;     ///< Use RDSon measurement for low-side OCP.
            uint32_t : 1;
            DeglitchTime HS_OCP_DEGLITCH_UVW : 3; ///< High-side OCP deglitch time.
            BlankingTime HS_OCP_BLANKING_UVW : 3; ///< High-side OCP blanking time.
            uint32_t : 1;
            OcpThreshold HS_OCP_THRES_UVW : 4;   ///< High-side OCP threshold.
            uint32_t : 4;
        } bits;
    };
};


/// Overcurrent Protection (OCP) Y2 Register (Address 0x1ED, Block 0: MCC_GDRV_OCP_Y2)
/**
 * @struct OCP_Y2
 * @brief Represents the configuration for Overcurrent Protection (OCP) settings.
 * 
 * This structure encapsulates the configuration for both low-side and high-side 
 * OCP settings, including thresholds, blanking times, and deglitch times. The 
 * configuration is represented as a 32-bit value or as individual bit fields.
 * 
 * Bit field details:
 * - **LS_OCP_DEGLITCH_Y2 (3 bits)**: Low-side OCP deglitch time. Configured using the `DeglitchTime` enum.
 * - **LS_OCP_BLANKING_Y2 (3 bits)**: Low-side OCP blanking time. Configured using the `BlankingTime` enum.
 * - **Reserved (2 bits)**: Reserved for future use.
 * - **LS_OCP_THRES_Y2 (4 bits)**: Low-side OCP threshold. Configured using the `OcpThreshold` enum.
 * - **Reserved (1 bit)**: Reserved for future use.
 * - **LS_OCP_USE_VDS_Y2 (1 bit)**: Indicates whether RDSon measurement is used for low-side OCP.
 * - **Reserved (1 bit)**: Reserved for future use.
 * - **HS_OCP_DEGLITCH_Y2 (3 bits)**: High-side OCP deglitch time. Configured using the `DeglitchTime` enum.
 * - **HS_OCP_BLANKING_Y2 (3 bits)**: High-side OCP blanking time. Configured using the `BlankingTime` enum.
 * - **Reserved (1 bit)**: Reserved for future use.
 * - **HS_OCP_THRES_Y2 (4 bits)**: High-side OCP threshold. Configured using the `OcpThreshold` enum.
 * - **Reserved (4 bits)**: Reserved for future use.
 * 
 * The structure provides a union to access the configuration as a raw 32-bit value 
 * or as individual bit fields for fine-grained control.
 */
struct OCP_Y2 {
    /**
     * @brief The register address for the OCP_Y2 configuration.
     */
    static constexpr uint16_t ADDRESS = 0x1ED;

    /**
     * @brief Enum representing the OCP threshold levels in millivolts (mV).
     */
    enum class OcpThreshold : uint8_t {
        THRES_63MV = 0,   ///< Threshold: 63 mV
        THRES_125MV,      ///< Threshold: 125 mV
        THRES_187MV,      ///< Threshold: 187 mV
        THRES_248MV,      ///< Threshold: 248 mV
        THRES_312MV,      ///< Threshold: 312 mV
        THRES_374MV,      ///< Threshold: 374 mV
        THRES_434MV,      ///< Threshold: 434 mV
        THRES_504MV,      ///< Threshold: 504 mV
        THRES_705MV,      ///< Threshold: 705 mV
        THRES_940MV,      ///< Threshold: 940 mV
        THRES_1180MV,     ///< Threshold: 1180 mV
        THRES_1410MV,     ///< Threshold: 1410 mV
        THRES_1650MV,     ///< Threshold: 1650 mV
        THRES_1880MV,     ///< Threshold: 1880 mV
        THRES_2110MV,     ///< Threshold: 2110 mV
        THRES_2350MV      ///< Threshold: 2350 mV
    };

    /**
     * @brief Enum representing the blanking times in nanoseconds (ns).
     */
    enum class BlankingTime : uint8_t {
        BLK_OFF = 0,      ///< Blanking time: Off
        BLK_250NS,        ///< Blanking time: 250 ns
        BLK_500NS,        ///< Blanking time: 500 ns
        BLK_1000NS,       ///< Blanking time: 1000 ns
        BLK_2000NS,       ///< Blanking time: 2000 ns
        BLK_4000NS,       ///< Blanking time: 4000 ns
        BLK_6000NS,       ///< Blanking time: 6000 ns
        BLK_8000NS        ///< Blanking time: 8000 ns
    };

    /**
     * @brief Enum representing the deglitch times in nanoseconds (ns).
     */
    enum class DeglitchTime : uint8_t {
        DEG_OFF = 0,      ///< Deglitch time: Off
        DEG_250NS,        ///< Deglitch time: 250 ns
        DEG_500NS,        ///< Deglitch time: 500 ns
        DEG_1000NS,       ///< Deglitch time: 1000 ns
        DEG_2000NS,       ///< Deglitch time: 2000 ns
        DEG_4000NS,       ///< Deglitch time: 4000 ns
        DEG_6000NS,       ///< Deglitch time: 6000 ns
        DEG_8000NS        ///< Deglitch time: 8000 ns
    };

    /**
     * @union Represents the OCP configuration as a 32-bit value or as individual bit fields.
     */
    union {
        uint32_t value; ///< The raw 32-bit value of the OCP configuration.

        /**
         * @brief Bit field representation of the OCP configuration.
         */
        struct {
            DeglitchTime LS_OCP_DEGLITCH_Y2 : 3; ///< Low-side OCP deglitch time.
            BlankingTime LS_OCP_BLANKING_Y2 : 3; ///< Low-side OCP blanking time.
            uint32_t : 2;                        ///< Reserved bits.
            OcpThreshold LS_OCP_THRES_Y2 : 4;   ///< Low-side OCP threshold.
            uint32_t : 1;                        ///< Reserved bit.
            uint32_t LS_OCP_USE_VDS_Y2 : 1;     ///< Use RDSon measurement for low-side OCP.
            uint32_t : 1;                        ///< Reserved bit.
            DeglitchTime HS_OCP_DEGLITCH_Y2 : 3; ///< High-side OCP deglitch time.
            BlankingTime HS_OCP_BLANKING_Y2 : 3; ///< High-side OCP blanking time.
            uint32_t : 1;                        ///< Reserved bit.
            OcpThreshold HS_OCP_THRES_Y2 : 4;   ///< High-side OCP threshold.
            uint32_t : 4;                        ///< Reserved bits.
        } bits;
    };
};

/// Protection Enable Register (Address 0x1EE, Block 0: MCC_GDRV_PROT_EN)
/**
 * Enables the protection mechanism for various fault events.
 * Note: The respective fault needs to be set in MCC_GDRV_STATUS_EN as well.
 *
 * BITS & NAME          | TYPE & RESET | DESCRIPTION
 * ---------------------|--------------|-------------------------------------------------
 * [31] VS_UVLO_PROT    | RW, 0x0      | Disables the gate driver on VS undervoltage event.
 * [29] VDRV_UVLO_PROT  | RW, 0x0      | Disables the gate driver on VDRV undervoltage event.
 * [27] HS_VGS_ON_SHORT_PROT_Y2 | RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned on (Y2 phase).
 * [26] HS_VGS_ON_SHORT_PROT_W  | RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned on (W phase).
 * [25] HS_VGS_ON_SHORT_PROT_V  | RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned on (V phase).
 * [24] HS_VGS_ON_SHORT_PROT_U  | RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned on (U phase).
 * [23] HS_VGS_OFF_SHORT_PROT_Y2| RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned off (Y2 phase).
 * [22] HS_VGS_OFF_SHORT_PROT_W | RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned off (W phase).
 * [21] HS_VGS_OFF_SHORT_PROT_V | RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned off (V phase).
 * [20] HS_VGS_OFF_SHORT_PROT_U | RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned off (U phase).
 * [19] HS_SHORT_PROT_Y2        | RW, 0x0 | Disables the channel if an overcurrent event occurs (Y2 phase).
 * [18] HS_SHORT_PROT_W         | RW, 0x0 | Disables the channel if an overcurrent event occurs (W phase).
 * [17] HS_SHORT_PROT_V         | RW, 0x0 | Disables the channel if an overcurrent event occurs (V phase).
 * [16] HS_SHORT_PROT_U         | RW, 0x0 | Disables the channel if an overcurrent event occurs (U phase).
 * [15] BST_UVLO_PROT_Y2        | RW, 0x0 | Disables the affected phase if a bootstrap capacitor undervoltage event occurs (Y2 phase).
 * [14] BST_UVLO_PROT_W         | RW, 0x0 | Disables the affected phase if a bootstrap capacitor undervoltage event occurs (W phase).
 * [13] BST_UVLO_PROT_V         | RW, 0x0 | Disables the affected phase if a bootstrap capacitor undervoltage event occurs (V phase).
 * [12] BST_UVLO_PROT_U         | RW, 0x0 | Disables the affected phase if a bootstrap capacitor undervoltage event occurs (U phase).
 * [11] LS_VGS_ON_SHORT_PROT_Y2 | RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned on (Y2 phase).
 * [10] LS_VGS_ON_SHORT_PROT_W  | RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned on (W phase).
 * [9]  LS_VGS_ON_SHORT_PROT_V  | RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned on (V phase).
 * [8]  LS_VGS_ON_SHORT_PROT_U  | RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned on (U phase).
 * [7]  LS_VGS_OFF_SHORT_PROT_Y2| RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned off (Y2 phase).
 * [6]  LS_VGS_OFF_SHORT_PROT_W | RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned off (W phase).
 * [5]  LS_VGS_OFF_SHORT_PROT_V | RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned off (V phase).
 * [4]  LS_VGS_OFF_SHORT_PROT_U | RW, 0x0 | Disables the gate driver if a gate short event occurs while the gate is turned off (U phase).
 * [3]  LS_SHORT_PROT_Y2        | RW, 0x0 | Disables the channel if an overcurrent event occurs (Y2 phase).
 * [2]  LS_SHORT_PROT_W         | RW, 0x0 | Disables the channel if an overcurrent event occurs (W phase).
 * [1]  LS_SHORT_PROT_V         | RW, 0x0 | Disables the channel if an overcurrent event occurs (V phase).
 * [0]  LS_SHORT_PROT_U         | RW, 0x0 | Disables the channel if an overcurrent event occurs (U phase).
 */
struct PROT_ENABLE {
    static constexpr uint16_t ADDRESS = 0x1EE;
    union {
        uint32_t value;
        struct {
            uint32_t LS_SHORT_PROT_U         : 1;  ///< Disables the channel if an overcurrent event occurs (U phase).
            uint32_t LS_SHORT_PROT_V         : 1;  ///< Disables the channel if an overcurrent event occurs (V phase).
            uint32_t LS_SHORT_PROT_W         : 1;  ///< Disables the channel if an overcurrent event occurs (W phase).
            uint32_t LS_SHORT_PROT_Y2        : 1;  ///< Disables the channel if an overcurrent event occurs (Y2 phase).
            uint32_t LS_VGS_OFF_SHORT_PROT_U : 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned off (U phase).
            uint32_t LS_VGS_OFF_SHORT_PROT_V : 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned off (V phase).
            uint32_t LS_VGS_OFF_SHORT_PROT_W : 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned off (W phase).
            uint32_t LS_VGS_OFF_SHORT_PROT_Y2: 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned off (Y2 phase).
            uint32_t LS_VGS_ON_SHORT_PROT_U  : 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned on (U phase).
            uint32_t LS_VGS_ON_SHORT_PROT_V  : 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned on (V phase).
            uint32_t LS_VGS_ON_SHORT_PROT_W  : 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned on (W phase).
            uint32_t LS_VGS_ON_SHORT_PROT_Y2 : 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned on (Y2 phase).
            uint32_t BST_UVLO_PROT_U         : 1;  ///< Disables the affected phase if a bootstrap capacitor undervoltage event occurs (U phase).
            uint32_t BST_UVLO_PROT_V         : 1;  ///< Disables the affected phase if a bootstrap capacitor undervoltage event occurs (V phase).
            uint32_t BST_UVLO_PROT_W         : 1;  ///< Disables the affected phase if a bootstrap capacitor undervoltage event occurs (W phase).
            uint32_t BST_UVLO_PROT_Y2        : 1;  ///< Disables the affected phase if a bootstrap capacitor undervoltage event occurs (Y2 phase).
            uint32_t HS_SHORT_PROT_U         : 1;  ///< Disables the channel if an overcurrent event occurs (U phase).
            uint32_t HS_SHORT_PROT_V         : 1;  ///< Disables the channel if an overcurrent event occurs (V phase).
            uint32_t HS_SHORT_PROT_W         : 1;  ///< Disables the channel if an overcurrent event occurs (W phase).
            uint32_t HS_SHORT_PROT_Y2        : 1;  ///< Disables the channel if an overcurrent event occurs (Y2 phase).
            uint32_t HS_VGS_OFF_SHORT_PROT_U : 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned off (U phase).
            uint32_t HS_VGS_OFF_SHORT_PROT_V : 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned off (V phase).
            uint32_t HS_VGS_OFF_SHORT_PROT_W : 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned off (W phase).
            uint32_t HS_VGS_OFF_SHORT_PROT_Y2: 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned off (Y2 phase).
            uint32_t HS_VGS_ON_SHORT_PROT_U  : 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned on (U phase).
            uint32_t HS_VGS_ON_SHORT_PROT_V  : 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned on (V phase).
            uint32_t HS_VGS_ON_SHORT_PROT_W  : 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned on (W phase).
            uint32_t HS_VGS_ON_SHORT_PROT_Y2 : 1;  ///< Disables the gate driver if a gate short event occurs while the gate is turned on (Y2 phase).
            uint32_t VDRV_UVLO_PROT          : 1;  ///< Disables the gate driver on VDRV undervoltage event.
            uint32_t : 1;
            uint32_t VS_UVLO_PROT            : 1;  ///< Disables the gate driver on VS undervoltage event.
        } bits;
    };
};

/// Gate Driver Status Enable Register (Address 0x1EF, Block 0: MCC_GDRV_STATUS_EN)
/**
 * @struct STATUS_INT_ENABLE
 * @brief Represents the STATUS_INT_ENABLE register configuration for enabling 
 *        reporting or actions for various fault conditions in a motor driver.
 * 
 * This structure defines a 32-bit register with individual bit fields to enable
 * or disable fault reporting and actions for specific conditions such as overcurrent,
 * gate short events, bootstrap undervoltage, and more. Each bit corresponds to a 
 * specific phase or condition.
 * 
 * @details
 * - The `ADDRESS` constant specifies the register address (0x1EF).
 * - The `value` member allows access to the entire 32-bit register as a single unit.
 * - The `bits` member provides access to individual fault enable flags.
 * 
 * Bit field descriptions:
 * - **LS_SHORT_EN_U, LS_SHORT_EN_V, LS_SHORT_EN_W, LS_SHORT_EN_Y2**: 
 *   Enable reporting/action for overcurrent events in the respective low-side phases.
 * - **LS_VGS_OFF_SHORT_EN_U, LS_VGS_OFF_SHORT_EN_V, LS_VGS_OFF_SHORT_EN_W, LS_VGS_OFF_SHORT_EN_Y2**: 
 *   Enable reporting/action for gate short events while off in the respective low-side phases.
 * - **LS_VGS_ON_SHORT_EN_U, LS_VGS_ON_SHORT_EN_V, LS_VGS_ON_SHORT_EN_W, LS_VGS_ON_SHORT_EN_Y2**: 
 *   Enable reporting/action for gate short events while on in the respective low-side phases.
 * - **BST_UVLO_EN_U, BST_UVLO_EN_V, BST_UVLO_EN_W, BST_UVLO_EN_Y2**: 
 *   Enable reporting/action for bootstrap undervoltage in the respective phases.
 * - **HS_SHORT_EN_U, HS_SHORT_EN_V, HS_SHORT_EN_W, HS_SHORT_EN_Y2**: 
 *   Enable reporting/action for overcurrent events in the respective high-side phases.
 * - **HS_VGS_OFF_SHORT_EN_U, HS_VGS_OFF_SHORT_EN_V, HS_VGS_OFF_SHORT_EN_W, HS_VGS_OFF_SHORT_EN_Y2**: 
 *   Enable reporting/action for gate short events while off in the respective high-side phases.
 * - **HS_VGS_ON_SHORT_EN_U, HS_VGS_ON_SHORT_EN_V, HS_VGS_ON_SHORT_EN_W, HS_VGS_ON_SHORT_EN_Y2**: 
 *   Enable reporting/action for gate short events while on in the respective high-side phases.
 * - **VDRV_UVLO_EN**: Enable reporting/action for VDRV undervoltage events.
 * - **VDRV_UVLWRN_EN**: Enable reporting/action for VDRV undervoltage warnings.
 * - **VS_UVLO_EN**: Enable reporting/action for VS undervoltage events.
 */
struct STATUS_INT_ENABLE {
    static constexpr uint16_t ADDRESS = 0x1EF;
    union {
        uint32_t value;
        struct {
            uint32_t LS_SHORT_EN_U         : 1;  ///< Enables reporting/action for overcurrent event (U phase).
            uint32_t LS_SHORT_EN_V         : 1;  ///< Enables reporting/action for overcurrent event (V phase).
            uint32_t LS_SHORT_EN_W         : 1;  ///< Enables reporting/action for overcurrent event (W phase).
            uint32_t LS_SHORT_EN_Y2        : 1;  ///< Enables reporting/action for overcurrent event (Y2 phase).
            uint32_t LS_VGS_OFF_SHORT_EN_U : 1;  ///< Enables reporting/action for gate short event while off (U phase).
            uint32_t LS_VGS_OFF_SHORT_EN_V : 1;  ///< Enables reporting/action for gate short event while off (V phase).
            uint32_t LS_VGS_OFF_SHORT_EN_W : 1;  ///< Enables reporting/action for gate short event while off (W phase).
            uint32_t LS_VGS_OFF_SHORT_EN_Y2: 1;  ///< Enables reporting/action for gate short event while off (Y2 phase).
            uint32_t LS_VGS_ON_SHORT_EN_U  : 1;  ///< Enables reporting/action for gate short event while on (U phase).
            uint32_t LS_VGS_ON_SHORT_EN_V  : 1;  ///< Enables reporting/action for gate short event while on (V phase).
            uint32_t LS_VGS_ON_SHORT_EN_W  : 1;  ///< Enables reporting/action for gate short event while on (W phase).
            uint32_t LS_VGS_ON_SHORT_EN_Y2 : 1;  ///< Enables reporting/action for gate short event while on (Y2 phase).
            uint32_t BST_UVLO_EN_U         : 1;  ///< Enables reporting/action for bootstrap undervoltage (U phase).
            uint32_t BST_UVLO_EN_V         : 1;  ///< Enables reporting/action for bootstrap undervoltage (V phase).
            uint32_t BST_UVLO_EN_W         : 1;  ///< Enables reporting/action for bootstrap undervoltage (W phase).
            uint32_t BST_UVLO_EN_Y2        : 1;  ///< Enables reporting/action for bootstrap undervoltage (Y2 phase).
            uint32_t HS_SHORT_EN_U         : 1;  ///< Enables reporting/action for overcurrent event (U phase).
            uint32_t HS_SHORT_EN_V         : 1;  ///< Enables reporting/action for overcurrent event (V phase).
            uint32_t HS_SHORT_EN_W         : 1;  ///< Enables reporting/action for overcurrent event (W phase).
            uint32_t HS_SHORT_EN_Y2        : 1;  ///< Enables reporting/action for overcurrent event (Y2 phase).
            uint32_t HS_VGS_OFF_SHORT_EN_U : 1;  ///< Enables reporting/action for gate short event while off (U phase).
            uint32_t HS_VGS_OFF_SHORT_EN_V : 1;  ///< Enables reporting/action for gate short event while off (V phase).
            uint32_t HS_VGS_OFF_SHORT_EN_W : 1;  ///< Enables reporting/action for gate short event while off (W phase).
            uint32_t HS_VGS_OFF_SHORT_EN_Y2: 1;  ///< Enables reporting/action for gate short event while off (Y2 phase).
            uint32_t HS_VGS_ON_SHORT_EN_U  : 1;  ///< Enables reporting/action for gate short event while on (U phase).
            uint32_t HS_VGS_ON_SHORT_EN_V  : 1;  ///< Enables reporting/action for gate short event while on (V phase).
            uint32_t HS_VGS_ON_SHORT_EN_W  : 1;  ///< Enables reporting/action for gate short event while on (W phase).
            uint32_t HS_VGS_ON_SHORT_EN_Y2 : 1;  ///< Enables reporting/action for gate short event while on (Y2 phase).
            uint32_t VDRV_UVLO_EN          : 1;  ///< Enables reporting/action for VDRV undervoltage event.
            uint32_t VDRV_UVLWRN_EN        : 1;  ///< Enables reporting/action for VDRV undervoltage warning.
            uint32_t VS_UVLO_EN            : 1;  ///< Enables reporting/action for VS undervoltage event.
        } bits;
    };
};

/// Gate Driver Status Register (Address 0x1F0, Block 0: MCC_GDRV_STATUS)
/**
 * @brief Represents the status register of the MCC_GDRV_STATUS block.
 * 
 * This structure contains the status of various fault events, including
 * overcurrent protection, gate short detection, and undervoltage conditions
 * for different phases and components of the motor driver.
 */
struct STATUS {
    /**
     * @brief Address of the MCC_GDRV_STATUS register.
     */
    static constexpr uint16_t ADDRESS = 0x1F0;

    union {
        uint32_t value; ///< Raw 32-bit value of the status register.

        struct {
            uint32_t LS_SHORT_U         : 1;  ///< Status of the overcurrent protection (low-side, U phase).
            uint32_t LS_SHORT_V         : 1;  ///< Status of the overcurrent protection (low-side, V phase).
            uint32_t LS_SHORT_W         : 1;  ///< Status of the overcurrent protection (low-side, W phase).
            uint32_t LS_SHORT_Y2        : 1;  ///< Status of the overcurrent protection (low-side, Y2 phase).
            uint32_t LS_VGS_OFF_SHORT_U : 1;  ///< Gate short detection while off (low-side, U phase).
            uint32_t LS_VGS_OFF_SHORT_V : 1;  ///< Gate short detection while off (low-side, V phase).
            uint32_t LS_VGS_OFF_SHORT_W : 1;  ///< Gate short detection while off (low-side, W phase).
            uint32_t LS_VGS_OFF_SHORT_Y2: 1;  ///< Gate short detection while off (low-side, Y2 phase).
            uint32_t LS_VGS_ON_SHORT_U  : 1;  ///< Gate short detection while on (low-side, U phase).
            uint32_t LS_VGS_ON_SHORT_V  : 1;  ///< Gate short detection while on (low-side, V phase).
            uint32_t LS_VGS_ON_SHORT_W  : 1;  ///< Gate short detection while on (low-side, W phase).
            uint32_t LS_VGS_ON_SHORT_Y2 : 1;  ///< Gate short detection while on (low-side, Y2 phase).
            uint32_t BST_UVLO_U         : 1;  ///< Undervoltage condition of the bootstrap cap (U phase).
            uint32_t BST_UVLO_V         : 1;  ///< Undervoltage condition of the bootstrap cap (V phase).
            uint32_t BST_UVLO_W         : 1;  ///< Undervoltage condition of the bootstrap cap (W phase).
            uint32_t BST_UVLO_Y2        : 1;  ///< Undervoltage condition of the bootstrap cap (Y2 phase).
            uint32_t HS_SHORT_U         : 1;  ///< Status of the overcurrent protection (high-side, U phase).
            uint32_t HS_SHORT_V         : 1;  ///< Status of the overcurrent protection (high-side, V phase).
            uint32_t HS_SHORT_W         : 1;  ///< Status of the overcurrent protection (high-side, W phase).
            uint32_t HS_SHORT_Y2        : 1;  ///< Status of the overcurrent protection (high-side, Y2 phase).
            uint32_t HS_VGS_OFF_SHORT_U : 1;  ///< Gate short detection while off (high-side, U phase).
            uint32_t HS_VGS_OFF_SHORT_V : 1;  ///< Gate short detection while off (high-side, V phase).
            uint32_t HS_VGS_OFF_SHORT_W : 1;  ///< Gate short detection while off (high-side, W phase).
            uint32_t HS_VGS_OFF_SHORT_Y2: 1;  ///< Gate short detection while off (high-side, Y2 phase).
            uint32_t HS_VGS_ON_SHORT_U  : 1;  ///< Gate short detection while on (high-side, U phase).
            uint32_t HS_VGS_ON_SHORT_V  : 1;  ///< Gate short detection while on (high-side, V phase).
            uint32_t HS_VGS_ON_SHORT_W  : 1;  ///< Gate short detection while on (high-side, W phase).
            uint32_t HS_VGS_ON_SHORT_Y2 : 1;  ///< Gate short detection while on (high-side, Y2 phase).
            uint32_t VDRV_UVLO          : 1;  ///< Undervoltage condition of VDRV (gate drive voltage).
            uint32_t VDRV_UVLWRN        : 1;  ///< Low voltage warning condition of VDRV (gate drive voltage).
            uint32_t VS_UVLO            : 1;  ///< Undervoltage condition of VS (supply voltage).
        } bits; ///< Bitfield representation of the status register.
    };
};

/// Gate Driver Fault Register (Address 0x1F1, Block 0).
/**
 * @brief Represents the MCC_GDRV_FAULT register (0x1F1, Block 0).
 * 
 * This register contains the status of various fault events in the system.
 * 
 * @details
 * - **Address**: 0x1F1
 * - **Block**: MCC_GDRV_FAULT
 * 
 * ### Fault Event Descriptions:
 * 
 * - **[31] VS_UVLO_STS (R, Reset: 0x0)**  
 *   Indicates an undervoltage condition of VS (supply voltage).
 * 
 * - **[30] VDRV_UVLWRN_STS (R, Reset: 0x0)**  
 *   Indicates a low voltage warning condition of VDRV (gate drive voltage).
 * 
 * - **[29] VDRV_UVLO_STS (R, Reset: 0x0)**  
 *   Indicates an undervoltage condition of VDRV (gate drive voltage).
 * 
 * - **[19] HS_FAULT_ACTIVE_Y2 (RW, W1C, Reset: 0x0)**  
 *   Set if any fault occurred on phase Y2 that triggered hardware protection.  
 *   Clear to resume normal operation. Ensure both HS_FAULT_ACTIVE and LS_FAULT_ACTIVE are cleared simultaneously to resume operation.
 * 
 * - **[18] HS_FAULT_ACTIVE_W (RW, W1C, Reset: 0x0)**  
 *   Set if any fault occurred on phase W that triggered hardware protection.  
 *   Clear to resume normal operation. Ensure both HS_FAULT_ACTIVE and LS_FAULT_ACTIVE are cleared simultaneously to resume operation.
 * 
 * - **[17] HS_FAULT_ACTIVE_V (RW, W1C, Reset: 0x0)**  
 *   Set if any fault occurred on phase V that triggered hardware protection.  
 *   Clear to resume normal operation. Ensure both HS_FAULT_ACTIVE and LS_FAULT_ACTIVE are cleared simultaneously to resume operation.
 * 
 * - **[16] HS_FAULT_ACTIVE_U (RW, W1C, Reset: 0x0)**  
 *   Set if any fault occurred on phase U that triggered hardware protection.  
 *   Clear to resume normal operation. Ensure both HS_FAULT_ACTIVE and LS_FAULT_ACTIVE are cleared simultaneously to resume operation.
 * 
 * - **[15] BST_UVLO_STS_Y2 (R, Reset: 0x0)**  
 *   Indicates an undervoltage condition of the bootstrap capacitor on phase Y2.
 * 
 * - **[14] BST_UVLO_STS_W (R, Reset: 0x0)**  
 *   Indicates an undervoltage condition of the bootstrap capacitor on phase W.
 * 
 * - **[13] BST_UVLO_STS_V (R, Reset: 0x0)**  
 *   Indicates an undervoltage condition of the bootstrap capacitor on phase V.
 * 
 * - **[12] BST_UVLO_STS_U (R, Reset: 0x0)**  
 *   Indicates an undervoltage condition of the bootstrap capacitor on phase U.
 * 
 * - **[3] LS_FAULT_ACTIVE_Y2 (RW, W1C, Reset: 0x0)**  
 *   Set if any fault occurred on phase Y2 that triggered hardware protection.  
 *   Clear to resume normal operation. Ensure both HS_FAULT_ACTIVE and LS_FAULT_ACTIVE are cleared simultaneously to resume operation.
 * 
 * - **[2] LS_FAULT_ACTIVE_W (RW, W1C, Reset: 0x0)**  
 *   Set if any fault occurred on phase W that triggered hardware protection.  
 *   Clear to resume normal operation. Ensure both HS_FAULT_ACTIVE and LS_FAULT_ACTIVE are cleared simultaneously to resume operation.
 * 
 * - **[1] LS_FAULT_ACTIVE_V (RW, W1C, Reset: 0x0)**  
 *   Set if any fault occurred on phase V that triggered hardware protection.  
 *   Clear to resume normal operation. Ensure both HS_FAULT_ACTIVE and LS_FAULT_ACTIVE are cleared simultaneously to resume operation.
 * 
 * - **[0] LS_FAULT_ACTIVE_U (RW, W1C, Reset: 0x0)**  
 *   Set if any fault occurred on phase U that triggered hardware protection.  
 *   Clear to resume normal operation. Ensure both HS_FAULT_ACTIVE and LS_FAULT_ACTIVE are cleared simultaneously to resume operation.
 */
struct FAULT {
    static constexpr uint16_t ADDRESS = 0x1F1;
    union {
        uint32_t value;
        struct {
            uint32_t LS_FAULT_ACTIVE_U : 1;  ///< Low-side fault active on phase U.
            uint32_t LS_FAULT_ACTIVE_V : 1;  ///< Low-side fault active on phase V.
            uint32_t LS_FAULT_ACTIVE_W : 1;  ///< Low-side fault active on phase W.
            uint32_t LS_FAULT_ACTIVE_Y2 : 1; ///< Low-side fault active on phase Y2.
            uint32_t : 8;
            uint32_t BST_UVLO_STS_U : 1;     ///< Bootstrap undervoltage on phase U.
            uint32_t BST_UVLO_STS_V : 1;     ///< Bootstrap undervoltage on phase V.
            uint32_t BST_UVLO_STS_W : 1;     ///< Bootstrap undervoltage on phase W.
            uint32_t BST_UVLO_STS_Y2 : 1;    ///< Bootstrap undervoltage on phase Y2.
            uint32_t HS_FAULT_ACTIVE_U : 1;  ///< High-side fault active on phase U.
            uint32_t HS_FAULT_ACTIVE_V : 1;  ///< High-side fault active on phase V.
            uint32_t HS_FAULT_ACTIVE_W : 1;  ///< High-side fault active on phase W.
            uint32_t HS_FAULT_ACTIVE_Y2 : 1; ///< High-side fault active on phase Y2.
            uint32_t : 9;
            uint32_t VDRV_UVLO_STS : 1;      ///< Undervoltage condition of VDRV.
            uint32_t VDRV_UVLWRN_STS : 1;    ///< Low voltage warning condition of VDRV.
            uint32_t VS_UVLO_STS : 1;        ///< Undervoltage condition of VS.
        } bits;
    };
};

/// External Writable ADC Values (Address 0x200, Block 0: MCC_ADC_I1_I0_EXT)
/**
 * External writable ADC values for phase currents I1 and I0.
 * - **I1** (bits 31:16): External writable ADC value for phase I1.
 *   - Type: RW, signed.
 *   - Reset: 0x0000.
 * - **I0** (bits 15:0): External writable ADC value for phase I0.
 *   - Type: RW, signed.
 *   - Reset: 0x0000.
 */
struct ADC_I1_I0_EXT {
    static constexpr uint16_t ADDRESS = 0x200;
    union {
        uint32_t value;
        struct {
            int16_t I0 : 16; ///< External writable ADC value for phase I0 (signed).
            int16_t I1 : 16; ///< External writable ADC value for phase I1 (signed).
        } bits;
    };
};

/// External Current Input (Address 0x201, Block 0: MCC_ADC_I2_EXT)
/**
 * External writable ADC value for phase I2.
 * Note that depending on the selection in register MCC_ADC_I_GEN_CONFIG -> TRIGGER_SELECT,
 * writing to this register may not trigger a new processing of the ADC data.
 * So when updating all three external ADC values, it is recommended to update this register first.
 * - **I2** (bits 15:0): External writable ADC value for phase I2.
 *   - Type: RW, signed.
 *   - Reset: 0x0000.
 */
struct ADC_I2_EXT {
    static constexpr uint16_t ADDRESS = 0x201;
    union {
        uint32_t value;
        struct {
            int16_t I2 : 16;  ///< External writable ADC value for phase I2 (signed).
            uint16_t : 16;
        } bits;
    };
};

/// External PWM Duty Cycle (Address 0x202, Block 0: MCC_PWM_VX2_UX1_EXT)
/**
 * External writable PWM value.
 * - **VX2** (bits 31:16): External writable PWM value for phase VX2.
 *   - Type: RW, unsigned.
 *   - Reset: 0x0000.
 * - **UX1** (bits 15:0): External writable PWM value for phase UX1.
 *   - Type: RW, unsigned.
 *   - Reset: 0x0000.
 */
struct PWM_VX2_UX1_EXT {
    static constexpr uint16_t ADDRESS = 0x202;
    union {
        uint32_t value;
        struct {
            uint16_t UX1 : 16;  ///< External writable PWM value for phase UX1.
            uint16_t VX2 : 16;  ///< External writable PWM value for phase VX2.
        } bits;
    };
};

/// External PWM Duty Cycle (Address 0x203, Block 0: MCC_PWM_Y2_WY1_EXT)
/**
 * External writable PWM value.
 * - **Y2** (bits 31:16): External writable PWM value for phase Y2.
 *   - Type: RW, unsigned.
 *   - Reset: 0x0000.
 * - **WY1** (bits 15:0): External writable PWM value for phase WY1.
 *   - Type: RW, unsigned.
 *   - Reset: 0x0000.
 */
struct PWM_Y2_WY1_EXT {
    static constexpr uint16_t ADDRESS = 0x203;
    union {
        uint32_t value;
        struct {
            uint16_t WY1 : 16;  ///< External writable PWM value for phase WY1.
            uint16_t Y2 : 16;   ///< External writable PWM value for phase Y2.
        } bits;
    };
};

/// External PWM Alternate Channel (Address 0x204, Block 0: MCC_PWM_EXT_Y2_ALT)
/**
 * External writable PWM compare value for phase Y2_ALT.
 * - **PWM_EXT_Y2_ALT** (bits 15:0): Duty cycle to be used in independent Y2 mode.
 *   - Type: RW, unsigned.
 *   - Reset: 0x0000.
 *   - Description: This is not scaled to the PWM_MAXCNT value and therefore needs to be set accordingly.
 *     Duty cycle = PWM_EXT_Y2_ALT / (PWM_MAXCNT + 1).
 */
struct PWM_EXT_Y2_ALT {
    static constexpr uint16_t ADDRESS = 0x204;
    union {
        uint32_t value;
        struct {
            uint16_t Y2_ALT : 16;  ///< Duty cycle for independent Y2 mode.
            uint16_t : 16;
        } bits;
    };
};

/// External Voltage Sense (Address 0x205, Block 0: MCC_VOLTAGE_EXT)
/**
 * External writable parameter for open-loop voltage control mode, useful during system setup.
 * - **UQ** (bits 31:16): U_Q component in Voltage mode.
 *   - Type: RW, signed.
 *   - Reset: 0x0000.
 * - **UD** (bits 15:0): U_D component in Voltage mode.
 *   - Type: RW, signed.
 *   - Reset: 0x0000.
 */
struct VOLTAGE_EXT {
    static constexpr uint16_t ADDRESS = 0x205;
    union {
        uint32_t value;
        struct {
            int16_t UD; ///< U_D component in Voltage mode.
            int16_t UQ; ///< U_Q component in Voltage mode.
        } bits;
    };
};

/// External Angle/Position Input (Address 0x206, Block 0: MCC_PHI_EXT)
/**
 * Angle phi_e_ext and phi_m_ext for external writing into this register.
 * - **PHI_M_EXT** (bits 31:16): Angle phi_m_ext for external input.
 *   - Type: RW, signed.
 *   - Reset: 0x0000.
 * - **PHI_E_EXT** (bits 15:0): Angle phi_e_ext for external input.
 *   - Type: RW, signed.
 *   - Reset: 0x0000.
 */
struct PHI_EXT {
    static constexpr uint16_t ADDRESS = 0x206;
    union {
        uint32_t value;
        struct {
            int16_t PHI_E_EXT; ///< Angle phi_e_ext for external input.
            int16_t PHI_M_EXT; ///< Angle phi_m_ext for external input.
        } bits;
    };
};

/// External Velocity Input (Address 0x208, Block 0: MCC_VELOCITY_EXT)
/**
 * Actual velocity for external override.
 * - **VELOCITY_EXT** (bits 31:0): Actual velocity for SW override.
 *   - Type: RW, signed.
 *   - Reset: 0x00000000.
 */
struct VELOCITY_EXT {
    static constexpr uint16_t ADDRESS = 0x208;
    union {
        uint32_t value;
        struct {
            int32_t VELOCITY_EXT; ///< Actual velocity for SW override.
        } bits;
    };
};

} // namespace MCC
} // namespace TMC9660
