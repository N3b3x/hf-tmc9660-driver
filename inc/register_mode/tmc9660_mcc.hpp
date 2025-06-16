#pragma once
#include <cstdint>

/**
 * @file tmc9660_mcc.hpp
 * @brief TMC9660 Motor Control Core (MCC) Register Definitions
 *
 * This file contains register definitions for the Motor Control Core of the TMC9660,
 * including motor control configurations, feedback systems, and status monitoring.
 *
 * Key Features:
 * - Motor type configuration and pole pair settings
 * - Position, velocity and current control loops
 * - ADC measurements for current and voltage sensing
 * - PWM generation and control
 * - Protection and fault handling
 *
 * @note All registers are 32-bit wide and use little-endian byte ordering
 */

namespace TMC9660 {
namespace MCC {

/**
 * @brief Chip ID Register (MCC_INFO_CHIP)
 * @details Block 0, Address: 0x000
 *
 * Provides the chip identification number. This is a read-only constant register.
 *
 * Register Map:
 * Bits    | Name | Access | Reset      | Description
 * --------|------|--------|------------|-------------------------------
 * 31:0    | ID   | R      | 0x544D0001 | Chip ID, static value, should read 0x544D0001
 *
 * @note All bits are read-only and reflect the device identity.
 */
struct CHIP_ID {
  static constexpr uint16_t ADDRESS = 0x000; ///< Register address (Block 0)
  union {
    uint32_t value;
    struct {
      uint32_t ID : 32; ///< Chip identifier (should read 0x544D0001 for TMC9660).
    } bits;
  };
};

/**
 * @brief Phase Current Raw Readings Register (ADC_I1_I0_RAW)
 * @details Block 0, Address: 0x020
 *
 * Provides raw ADC readings of phase currents I1 and I0 directly from the ADC.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 31:16   | I1        | R      | Raw phase current I1 (signed)
 * 15:0    | I0        | R      | Raw phase current I0 (signed)
 *
 * @note These are uncalibrated values before scaling and offset compensation
 * @see I1_I0_SCALED for calibrated values
 * @see I_GEN_CONFIG for ADC configuration
 */
struct I1_I0_RAW {
  static constexpr uint16_t ADDRESS = 0x020;
  union {
    uint32_t value;
    struct {
      int16_t I1 : 16; ///< Raw phase current I1 (signed value).
      int16_t I0 : 16; ///< Raw phase current I0 (signed value).
    } bits;
  };
};

/**
 * @brief Phase Current Raw Readings Register (ADC_I3_I2_RAW)
 * @details Block 0, Address: 0x021
 *
 * Provides raw ADC readings of phase currents I3 and I2 directly from the ADC.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 31:16   | I3        | R      | Raw phase current I3 (signed)
 * 15:0    | I2        | R      | Raw phase current I2 (signed)
 *
 * @note These values must be processed with calibration data for accurate current measurement
 * @see I3_I2_SCALED for calibrated values
 */
struct I3_I2_RAW {
  static constexpr uint16_t ADDRESS = 0x021;
  union {
    uint32_t value;
    struct {
      int16_t I3 : 16; ///< Raw phase current I3 (signed).
      int16_t I2 : 16; ///< Raw phase current I2 (signed).
    } bits;
  };
};

/**
 * @brief Phase Voltage Raw Readings Register (ADC_U1_U0_RAW)
 * @details Block 0, Address: 0x022
 *
 * Provides raw ADC readings of phase voltages U1 and U0 directly from the ADC.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 31:16   | U1        | R      | Raw phase voltage U1 (signed)
 * 15:0    | U0        | R      | Raw phase voltage U0 (signed)
 *
 * @note These are uncalibrated values before scaling and offset compensation
 */
struct U1_U0_RAW {
  static constexpr uint16_t ADDRESS = 0x022;
  union {
    uint32_t value;
    struct {
      int16_t U1 : 16; ///< Raw phase voltage U1 (signed).
      int16_t U0 : 16; ///< Raw phase voltage U0 (signed).
    } bits;
  };
};

/**
 * @brief Phase Voltage Raw Readings Register (ADC_U3_U2_RAW)
 * @details Block 0, Address: 0x023
 *
 * Provides raw ADC readings of phase voltages U3 and U2 directly from the ADC.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 31:16   | U3        | R      | Raw phase voltage U3 (signed)
 * 15:0    | U2        | R      | Raw phase voltage U2 (signed)
 *
 * @note These values must be processed with calibration data for accurate voltage measurement
 */
struct U3_U2_RAW {
  static constexpr uint16_t ADDRESS = 0x023;
  union {
    uint32_t value;
    struct {
      int16_t U3 : 16; ///< Raw phase voltage U3 (signed).
      int16_t U2 : 16; ///< Raw phase voltage U2 (signed).
    } bits;
  };
};

/**
 * @brief Temperature and Supply Voltage Raw Readings Register (ADC_TEMP_VM_RAW)
 * @details Block 0, Address: 0x024
 *
 * Provides raw ADC readings of on-chip temperature sensor and supply (V<sub>SA</sub>) voltage
 * monitor.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 31:16   | TEMP      | R      | Raw die temperature sensor value (signed)
 * 15:0    | VM        | R      | Raw supply voltage (V<sub>SA</sub>) monitor reading (unsigned)
 *
 * @note These are uncalibrated values before scaling and offset compensation
 */
struct TEMP_VM_RAW {
  static constexpr uint16_t ADDRESS = 0x024;
  union {
    uint32_t value;
    struct {
      int16_t TEMP : 16; ///< Raw die temperature sensor value (signed).
      uint16_t VM : 16;  ///< Raw supply voltage (V<sub>SA</sub>) monitor reading.
    } bits;
  };
};

/**
 * @brief Auxiliary Analog Inputs Raw Readings Register (ADC_AIN1_AIN0_RAW)
 * @details Block 0, Address: 0x025
 *
 * Provides raw ADC readings of auxiliary analog inputs AIN1 and AIN0.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 31:16   | AIN1      | R      | Raw analog input 1
 * 15:0    | AIN0      | R      | Raw analog input 0
 *
 * @note These are uncalibrated values before scaling and offset compensation
 */
struct AIN1_AIN0_RAW {
  static constexpr uint16_t ADDRESS = 0x025;
  union {
    uint32_t value;
    struct {
      uint16_t AIN1 : 16; ///< Raw analog input 1.
      uint16_t AIN0 : 16; ///< Raw analog input 0.
    } bits;
  };
};

/**
 * @brief Auxiliary Analog Inputs Raw Readings Register (ADC_AIN3_AIN2_RAW)
 * @details Block 0, Address: 0x026
 *
 * Provides raw ADC readings of auxiliary analog inputs AIN3 and AIN2.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 31:16   | AIN3      | R      | Raw analog input 3
 * 15:0    | AIN2      | R      | Raw analog input 2
 *
 * @note These are uncalibrated values before scaling and offset compensation
 */
struct AIN3_AIN2_RAW {
  static constexpr uint16_t ADDRESS = 0x026;
  union {
    uint32_t value;
    struct {
      uint16_t AIN3 : 16; ///< Raw analog input 3.
      uint16_t AIN2 : 16; ///< Raw analog input 2.
    } bits;
  };
};

/**
 * @brief ADC General Configuration Register (MCC_ADC_I_GEN_CONFIG)
 * @details Block 0, Address: 0x040
 *
 * General configuration setup of the current ADCs, allowing control over various calibration and
 * measurement settings.
 *
 * Register Map:
 * Bits    | Name            | Access | Description
 * --------|-----------------|--------|-------------
 * 31:16   | TRIGGER_POS     | RW     | Relative position of ADC trigger event in PWM cycle
 *                                      Values: 0-65535 (representing 0-100% of PWM period)
 * 12      | TRIGGER_SELECT  | RW     | Select trigger point for ADC sampling
 *                                      Values:
 *                                      - 0: Sample at PWM zero crossing
 *                                      - 1: Sample at PWM center
 * 11:9    | MEASUREMENT_MODE| RW     | Configuration of measurement mode
 *                                      Values:
 *                                      - 0: Single-shunt current sampling
 *                                      - 1: Dual-shunt current sampling
 *                                      - 2: Triple-shunt current sampling
 *                                      - 3: N-shunt for N phases
 *                                      - 4-7: Reserved
 * 7:6     | Y2_SELECT       | RW     | Input selection for ADC_I_Y2
 *                                      Values:
 *                                      - 0: Phase current measurement
 *                                      - 1: Supply current measurement
 *                                      - 2-3: Reserved
 * 5:4     | WY1_SELECT      | RW     | Input selection of raw current ADC_I_WY1
 * 3:2     | VX2_SELECT      | RW     | Input selection of raw current ADC_I_VX2
 * 1:0     | UX1_SELECT      | RW     | Input selection of raw current ADC_I_UX
 *                                      Values for WY1/VX2/UX1_SELECT:
 *                                      - 0: Phase current measurement
 *                                      - 1: DC link current measurement
 *                                      - 2: Temperature sensor input
 *                                      - 3: External ADC input
 *
 * @note This register allows fine-tuning of ADC sampling and measurement modes
 */
struct I_GEN_CONFIG {
  static constexpr uint16_t ADDRESS = 0x040;
  union {
    uint32_t value;
    struct {
      uint32_t UX1_SELECT : 2;       ///< Input selection of raw current ADC_I_UX.
      uint32_t VX2_SELECT : 2;       ///< Input selection of raw current ADC_I_VX2.
      uint32_t WY1_SELECT : 2;       ///< Input selection of raw current ADC_I_WY1.
      uint32_t Y2_SELECT : 2;        ///< Input selection of raw current ADC_I_Y2.
      uint32_t : 1;                  ///< Reserved.
      uint32_t MEASUREMENT_MODE : 3; ///< Configuration of measurement mode.
      uint32_t : 3;                  ///< Reserved.
      uint32_t TRIGGER_SELECT : 1;   ///< Select trigger point to start process of new ADC samples.
      uint32_t : 3;                  ///< Reserved.
      uint32_t TRIGGER_POS : 16;     ///< Relative position of ADC trigger event in PWM cycle.
    } bits;
  };
};

/**
 * @brief Current ADC Channel 0 Configuration Register (MCC_ADC_I0_CONFIG)
 * @details Block 0, Address: 0x041
 *
 * Configuration of current ADC channel 0 offset and scaling values.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 31:16   | SCALE     | RW     | Current ADC channel 0 scaling value (signed)
 * 15:0    | OFFSET    | RW     | Current ADC channel 0 offset value (signed)
 *
 * @note These values are used to calibrate the raw ADC readings
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

/**
 * @brief Current ADC Channel 1 Configuration Register (MCC_ADC_I1_CONFIG)
 * @details Block 0, Address: 0x042
 *
 * Configuration of current ADC channel 1 offset and scaling values.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 31:16   | SCALE     | RW     | Current ADC channel 1 scaling value (signed)
 * 15:0    | OFFSET    | RW     | Current ADC channel 1 offset value (signed)
 *
 * @note These values are used to calibrate the raw ADC readings
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

/**
 * @brief Current ADC Channel 2 Configuration Register (MCC_ADC_I2_CONFIG)
 * @details Block 0, Address: 0x043
 *
 * Configuration of current ADC channel 2 offset and scaling values.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 31:16   | SCALE     | RW     | Current ADC channel 2 scaling value (signed)
 * 15:0    | OFFSET    | RW     | Current ADC channel 2 offset value (signed)
 *
 * @note These values are used to calibrate the raw ADC readings
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

/**
 * @brief Current ADC Channel 3 Configuration Register (MCC_ADC_I3_CONFIG)
 * @details Block 0, Address: 0x044
 *
 * Configuration of current ADC channel 3 offset and scaling values.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 31:16   | SCALE     | RW     | Current ADC channel 3 scaling value (signed)
 * 15:0    | OFFSET    | RW     | Current ADC channel 3 offset value (signed)
 *
 * @note These values are used to calibrate the raw ADC readings
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

/**
 * @brief Scaled Current Readings Register (MCC_ADC_I1_I0_SCALED)
 * @details Block 0, Address: 0x045
 *
 * Provides phase current I1 and I0 after applying scaling and offset.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 31:16   | I1        | R      | Calculated Phase Current I1 (signed)
 * 15:0    | I0        | R      | Calculated Phase Current I0 (signed)
 *
 * @note These values are used for further processing in the control algorithms
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

/**
 * @brief Scaled Current Readings Register (MCC_ADC_I3_I2_SCALED)
 * @details Block 0, Address: 0x046
 *
 * Provides phase current I3 and I2 after applying scaling and offset.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 31:16   | I3        | R      | Calculated Phase Current I3 (signed)
 * 15:0    | I2        | R      | Calculated Phase Current I2 (signed)
 *
 * @note These values are used for further processing in the control algorithms
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

/**
 * @brief Scaled Current ADC Value Register (MCC_ADC_IWY_IUX)
 * @details Block 0, Address: 0x047
 *
 * Provides scaled current ADC value including signed added offset as input for the FOC.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 31:16   | IWY       | R      | Scaled current ADC value for phase W/Y (signed)
 * 15:0    | IUX       | R      | Scaled current ADC value for phase U/X (signed)
 *
 * @note These values are used as input for the FOC control algorithms
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

/**
 * @brief Scaled Current ADC Value Register (MCC_ADC_IV)
 * @details Block 0, Address: 0x048
 *
 * Provides scaled current ADC value including signed added offset as input for the FOC.
 *
 * Register Map:
 * Bits    | Name      | Access | Description
 * --------|-----------|--------|-------------
 * 15:0    | IV        | R      | Scaled current ADC value for phase V (signed)
 *
 * @note These values are used as input for the FOC control algorithms
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

/**
 * @brief ADC Status Register (MCC_ADC_STATUS)
 * @details Block 0, Address: 0x049
 *
 * Represents the status register of the TMC9660 driver, providing detailed information about the
 * state of various ADC measurements and their completion status.
 *
 * Register Map:
 * Bits    | Name            | Access | Description
 * --------|-----------------|--------|-------------
 * 0       | I0_CLIPPED      | R      | ADC current measurement I0 value clipped
 * 1       | I1_CLIPPED      | R      | ADC current measurement I1 value clipped
 * 2       | I2_CLIPPED      | R      | ADC current measurement I2 value clipped
 * 3       | I3_CLIPPED      | R      | ADC current measurement I3 value clipped
 * 4       | U0_CLIPPED      | R      | ADC voltage measurement U0 value clipped
 * 5       | U1_CLIPPED      | R      | ADC voltage measurement U1 value clipped
 * 6       | U2_CLIPPED      | R      | ADC voltage measurement U2 value clipped
 * 7       | U3_CLIPPED      | R      | ADC voltage measurement U3 value clipped
 * 8       | AIN0_CLIPPED    | R      | ADC measurement AIN0 value clipped
 * 9       | AIN1_CLIPPED    | R      | ADC measurement AIN1 value clipped
 * 10      | AIN2_CLIPPED    | R      | ADC measurement AIN2 value clipped
 * 11      | AIN3_CLIPPED    | R      | ADC measurement AIN3 value clipped
 * 12      | VM_CLIPPED      | R      | ADC supply voltage measurement VM value clipped
 * 13      | TEMP_CLIPPED    | R      | ADC temperature voltage measurement value clipped
 * 16      | I0_DONE         | R      | ADC current measurement I0 finished
 * 17      | I1_DONE         | R      | ADC current measurement I1 finished
 * 18      | I2_DONE         | R      | ADC current measurement I2 finished
 * 19      | I3_DONE         | R      | ADC current measurement I3 finished
 * 20      | U0_DONE         | R      | ADC voltage measurement U0 finished
 * 21      | U1_DONE         | R      | ADC voltage measurement U1 finished
 * 22      | U2_DONE         | R      | ADC voltage measurement U2 finished
 * 23      | U3_DONE         | R      | ADC voltage measurement U3 finished
 * 24      | AIN0_DONE       | R      | ADC measurement AIN0 finished
 * 25      | AIN1_DONE       | R      | ADC measurement AIN1 finished
 * 26      | AIN2_DONE       | R      | ADC measurement AIN2 finished
 * 27      | AIN3_DONE       | R      | ADC measurement AIN3 finished
 * 28      | VM_DONE         | R      | ADC supply voltage measurement VM finished
 * 29      | TEMP_DONE       | R      | ADC temperature voltage measurement finished
 *
 * @note This register provides status flags for ADC measurements and their completion
 */
struct STATUS {
  static constexpr uint16_t ADDRESS = 0x049;
  union {
    uint32_t value;
    struct {
      uint32_t I0_CLIPPED : 1;   ///< ADC current measurement I0 value clipped.
      uint32_t I1_CLIPPED : 1;   ///< ADC current measurement I1 value clipped.
      uint32_t I2_CLIPPED : 1;   ///< ADC current measurement I2 value clipped.
      uint32_t I3_CLIPPED : 1;   ///< ADC current measurement I3 value clipped.
      uint32_t U0_CLIPPED : 1;   ///< ADC voltage measurement U0 value clipped.
      uint32_t U1_CLIPPED : 1;   ///< ADC voltage measurement U1 value clipped.
      uint32_t U2_CLIPPED : 1;   ///< ADC voltage measurement U2 value clipped.
      uint32_t U3_CLIPPED : 1;   ///< ADC voltage measurement U3 value clipped.
      uint32_t AIN0_CLIPPED : 1; ///< ADC measurement AIN0 value clipped.
      uint32_t AIN1_CLIPPED : 1; ///< ADC measurement AIN1 value clipped.
      uint32_t AIN2_CLIPPED : 1; ///< ADC measurement AIN2 value clipped.
      uint32_t AIN3_CLIPPED : 1; ///< ADC measurement AIN3 value clipped.
      uint32_t VM_CLIPPED : 1;   ///< ADC supply voltage measurement VM value clipped.
      uint32_t TEMP_CLIPPED : 1; ///< ADC temperature voltage measurement value clipped.
      uint32_t : 2;
      uint32_t I0_DONE : 1;   ///< ADC current measurement I0 finished.
      uint32_t I1_DONE : 1;   ///< ADC current measurement I1 finished.
      uint32_t I2_DONE : 1;   ///< ADC current measurement I2 finished.
      uint32_t I3_DONE : 1;   ///< ADC current measurement I3 finished.
      uint32_t U0_DONE : 1;   ///< ADC voltage measurement U0 finished.
      uint32_t U1_DONE : 1;   ///< ADC voltage measurement U1 finished.
      uint32_t U2_DONE : 1;   ///< ADC voltage measurement U2 finished.
      uint32_t U3_DONE : 1;   ///< ADC voltage measurement U3 finished.
      uint32_t AIN0_DONE : 1; ///< ADC measurement AIN0 finished.
      uint32_t AIN1_DONE : 1; ///< ADC measurement AIN1 finished.
      uint32_t AIN2_DONE : 1; ///< ADC measurement AIN2 finished.
      uint32_t AIN3_DONE : 1; ///< ADC measurement AIN3 finished.
      uint32_t VM_DONE : 1;   ///< ADC supply voltage measurement VM finished.
      uint32_t TEMP_DONE : 1; ///< ADC temperature voltage measurement finished.
    } bits;
  };
};

/**
 * @brief Motor Configuration Register (MCC_MOTOR_CONFIG)
 * @details Block 0, Address: 0x060
 *
 * Configures fundamental motor parameters and operating modes.
 *
 * Register Map:
 * Bits    | Name          | Access | Description
 * --------|---------------|--------|-------------
 * 6:0     | N_POLE_PAIRS  | RW     | Number of motor pole pairs (minimum 1)
 *                                    Values:
 *                                    - 1: Single pole pair motor
 *                                    - 2-127: Multi-pole pair motor
 *                                    Note: Most BLDC/PMSM motors use 4-8 pole pairs
 * 17:16   | TYPE          | RW     | Motor type selection
 *                                    Values:
 *                                    - 0: NONE (Motor disabled)
 *                                    - 1: DC motor
 *                                    - 2: Stepper motor
 *                                    - 3: BLDC/PMSM motor
 *
 * @note This register allows selection of motor type and number of pole pairs
 */
struct MOTOR_CONFIG {
  static constexpr uint16_t ADDRESS = 0x060;
  union {
    uint32_t value;
    struct {
      uint32_t N_POLE_PAIRS : 7; ///< Number of pole pairs (minimum 1).
      uint32_t : 9;
      uint32_t TYPE : 2; ///< Motor type (0: NONE, 1: DC, 2: STEPPER, 3: BLDC).
      uint32_t : 14;
    } bits;
  };
};

/**
 * @brief Motion Control Configuration Register (MCC_MOTION_CONFIG)
 * @details Block 0, Address: 0x061
 *
 * Configures the motion control loop behaviors (velocity and position control settings).
 *
 * Register Map:
 * Bits    | Name          | Access | Description
 * --------|---------------|--------|-------------
 * 3:0     | MOTION_MODE   | RW     | Motion mode configuration
 *                                    Values:
 *                                    - 0: Stop (motor disabled)
 *                                    - 1: Torque mode (direct current control)
 *                                    - 2: Velocity mode (closed loop velocity)
 *                                    - 3: Position mode (closed loop position)
 *                                    - 4: PWM mode (direct PWM control)
 *                                    - 5-15: Reserved
 * 4       | RAMP_ENABLE   | RW     | Enable ramp generator
 *                                    Values:
 *                                    - 0: Direct target values
 *                                    - 1: Use ramp generator for smooth transitions
 * 5       | RAMP_MODE     | RW     | Ramp mode selection
 *                                    Values:
 *                                    - 0: Position ramps
 *                                    - 1: Velocity ramps
 * 7:6     | FEEDFORWARD   | RW     | Feedforward control structure
 *                                    Values:
 *                                    - 0: No feedforward
 *                                    - 1: Velocity FF
 *                                    - 2: Torque FF
 *                                    - 3: Velocity & Torque FF
 *
 * @note This register allows configuration of motion control modes and feedforward settings
 */
struct MOTION_CONFIG {
  static constexpr uint16_t ADDRESS = 0x061;
  struct PHI_E {
    static constexpr uint16_t ADDRESS = 0x063;
    union {
      uint32_t value;
      struct {
        int16_t PHI_E : 16; ///< Angle used for the inner FOC loop (signed 16-bit).
        uint16_t : 16;
      } bits;
    };
  };

  /**
   * @brief PWM Configuration Register (MCC_PWM_CONFIG)
   * @details Block 0, Address: 0x080
   *
   * Configures the PWM unit (frequency, mode, center-aligned vs edge, space vector modulation,
   * etc.).
   *
   * Register Map:
   * Bits    | Name            | Access | Description
   * --------|-----------------|--------|-------------
   * 1:0     | SV_MODE         | RW     | Space Vector PWM mode (0: off, 1: 3rd harmonic injection,
   * etc.) 2       | POLARITY        | RW     | Invert PWM output polarity (1 = inverted) 3       |
   * CENTER_ALIGNED  | RW     | Center-aligned PWM if set, edge-aligned if clear 7:4     |
   * PWM_FREQ_DIV    | RW     | PWM frequency divider (sets base PWM frequency)
   *
   * @note This register allows configuration of PWM operating modes and frequency
   */
  struct CONFIG {
    static constexpr uint16_t ADDRESS = 0x080;
    union {
      uint32_t value;
      struct {
        uint32_t SV_MODE : 2;  ///< Space Vector PWM mode (0: off, 1: 3rd harmonic injection, etc.).
        uint32_t POLARITY : 1; ///< Invert PWM output polarity (1 = inverted).
        uint32_t CENTER_ALIGNED : 1; ///< Center-aligned PWM if set, edge-aligned if clear.
        uint32_t PWM_FREQ_DIV : 4;   ///< PWM frequency divider (sets base PWM frequency).
        uint32_t : 24;
      } bits;
    };
  };

  /**
   * @brief PWM Max Counter Register (MCC_PWM_MAXCNT)
   * @details Block 0, Address: 0x081
   *
   * Sets the maximum PWM timer count (period). Determines the PWM frequency together with the base
   * clock.
   *
   * Register Map:
   * Bits    | Name      | Access | Description
   * --------|-----------|--------|-------------
   * 15:0    | MAXCNT    | RW     | PWM period count (PWM resolution)
   *
   * @note This register allows setting the PWM period and resolution
   */
  struct MAXCNT {
    static constexpr uint16_t ADDRESS = 0x081;
    union {
      uint32_t value;
      struct {
        uint32_t MAXCNT : 16; ///< PWM period count (PWM resolution).
        uint32_t : 16;
      } bits;
    };
  };

  /**
   * @brief PWM Switch Frequency Limit Register (MCC_PWM_SWITCH_LIMIT)
   * @details Block 0, Address: 0x083
   *
   * Defines the velocity threshold above which the PWM frequency is increased (for FOC control).
   *
   * Register Map:
   * Bits    | Name              | Access | Description
   * --------|-------------------|--------|-------------
   * 15:0    | SWITCH_VEL_LIMIT  | RW     | Velocity threshold for PWM mode switch
   *
   * @note This register allows setting the velocity threshold for PWM frequency switching
   */
  struct SWITCH_LIMIT {
    static constexpr uint16_t ADDRESS = 0x083;
    union {
      uint32_t value;
      struct {
        uint32_t SWITCH_VEL_LIMIT : 16; ///< Velocity threshold for PWM mode switch (in encoder
                                        ///< counts per cycle or another unit).
        uint32_t : 16;
      } bits;
    };
  };

  /**
   * @brief ABN Encoder Angle and Mechanical Angle Register (MCC_ABN_PHI_E_PHI_M)
   * @details Block 0, Address: 0x0A0
   *
   * Contains the current electrical angle (phi_e) and mechanical angle (phi_m) from the ABN
   * incremental encoder interface.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | PHI_E_ABN     | R      | Encoder-derived electrical angle (signed)
   * 15:0    | PHI_M_ABN     | R      | Encoder mechanical position (absolute count, signed)
   *
   * @note This register provides the current electrical and mechanical angles from the ABN encoder
   */
  struct ABN_PHI_E_PHI_M {
    static constexpr uint16_t ADDRESS = 0x0A0;
    union {
      uint32_t value;
      struct {
        int16_t PHI_E_ABN : 16; ///< Encoder-derived electrical angle (signed).
        int16_t PHI_M_ABN : 16; ///< Encoder mechanical position (absolute count, signed).
      } bits;
    };
  };

  /**
   * @brief ABN Encoder Mode Register (MCC_ABN_MODE)
   * @details Block 0, Address: 0x0A1
   *
   * Configuration of the ABN encoder interface.
   *
   * Register Map:
   * Bits    | Name                | Access | Description
   * --------|---------------------|--------|-------------
   * 0       | A_POL               | RW     | Polarity of A pulse at N
   * 1       | B_POL               | RW     | Polarity of B pulse at N
   * 2       | N_POL               | RW     | Polarity of N pulse at Null
   * 3       | COMBINED_N          | RW     | Use AND of A, B, N to determine Null signal
   * 4       | CLEAR_COUNT_ON_N    | RW     | Set ABN_COUNT to 0 on Null signal
   * 5       | DISABLE_FILTER      | RW     | Disable digital noise filter on encoder signals
   * 8       | CLN                 | RW     | N channel event writes ABN_COUNT_N into ABN_COUNT
   * 12      | DIRECTION           | RW     | Decoder count direction
   *
   * @note This register allows configuration of the ABN encoder interface
   */
  struct ABN_MODE {
    static constexpr uint16_t ADDRESS = 0x0A1;
    union {
      uint32_t value;
      struct {
        uint32_t A_POL : 1;            ///< Polarity of A pulse at N.
        uint32_t B_POL : 1;            ///< Polarity of B pulse at N.
        uint32_t N_POL : 1;            ///< Polarity of N pulse at Null.
        uint32_t COMBINED_N : 1;       ///< Use AND of A, B, N to determine Null signal.
        uint32_t CLEAR_COUNT_ON_N : 1; ///< Set ABN_COUNT to 0 on Null signal.
        uint32_t DISABLE_FILTER : 1;   ///< Disable digital noise filter on encoder signals.
        uint32_t : 2;
        uint32_t CLN : 1; ///< N channel event writes ABN_COUNT_N into ABN_COUNT.
        uint32_t : 3;
        uint32_t DIRECTION : 1; ///< Decoder count direction.
        uint32_t : 19;
      } bits;
    };
  };

  /**
   * @brief ABN Encoder Counts per Revolution Register (MCC_ABN_CPR)
   * @details Block 0, Address: 0x0A2
   *
   * Sets the number of counts per mechanical revolution for the ABN encoder.
   *
   * Register Layout (32-bit RW):
   *   Bits    | Name | Access | Description
   *   --------|------|--------|------------------------------------------
   *   [23:0]  | CPR  | RW     | Encoder counts-per-revolution (CPR)
   *   [31:24] | —    | R0     | Reserved
   */
  struct ABN_CPR {
    /// Combined block<<9 | offset address
    static constexpr uint16_t ADDRESS = (0 << 9) | 0xA2;

    union {
      uint32_t value;
      struct {
        uint32_t CPR : 24; ///< Encoder counts-per-revolution (CPR)
        uint32_t : 8;      ///< Reserved
      } bits;
    };

    /**
     * @brief Compute the inverse Counts-Per-Revolution for use in the MCC_ABN_CPR_INV register.
     * @return 2^32 / CPR, or 0 if CPR==0
     */
    uint32_t calculateInverseCPR() const {
      auto cpr = bits.CPR;
      return cpr ? static_cast<uint32_t>((uint64_t{1} << 32) / cpr) : 0u;
    }
  };

  /**
   * @brief ABN Encoder Inverse CPR Register (MCC_ABN_CPR_INV)
   * @details Block 0, Address: 0x0A3
   *
   * Holds 2^32 divided by the encoder CPR (for internal use in angle calculations).
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:0    | ABN_CPR_INV   | RW     | 2^32 divided by encoder CPR
   *
   * @note This register provides the inverse CPR value for internal calculations
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

  /**
   * @brief ABN Encoder Counter Register (MCC_ABN_COUNT)
   * @details Block 0, Address: 0x0A4
   *
   * Raw decoder count. The digital decoder engine counts modulo ABN_CPR.
   *
   * Register Map:
   * Bits    | Name      | Access | Description
   * --------|-----------|--------|-------------
   * 23:0    | ABN_COUNT | RW     | Raw decoder count (modulo ABN_CPR)
   *
   * @note This register provides the raw decoder count
   */
  struct ABN_COUNT {
    static constexpr uint16_t ADDRESS = 0x0A4;
    union {
      uint32_t value;
      struct {
        uint32_t ABN_COUNT : 24; ///< Raw decoder count (modulo ABN_CPR).
        uint32_t : 8;
      } bits;
    };
  };

  /**
   * @brief ABN Encoder N Pulse Count Register (MCC_ABN_COUNT_N)
   * @details Block 0, Address: 0x0A5
   *
   * ABN_COUNT latched on N pulse. When N pulse clears ABN_COUNT, ABN_COUNT_N is also set to 0.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 23:0    | ABN_COUNT_N   | RW     | ABN_COUNT latched on N pulse
   *
   * @note This register provides the latched ABN_COUNT value on N pulse
   */
  struct ABN_COUNT_N {
    static constexpr uint16_t ADDRESS = 0x0A5;
    union {
      uint32_t value;
      struct {
        uint32_t ABN_COUNT_N : 24; ///< ABN_COUNT latched on N pulse.
        uint32_t : 8;
      } bits;
    };
  };

  /**
   * @brief ABN Encoder Offset Angle Register (MCC_ABN_PHI_E_OFFSET)
   * @details Block 0, Address: 0x0A6
   *
   * Offset for ABN_PHI_E. This offset is added to align the encoder angle with the motor's
   * electrical zero.
   *
   * Register Map:
   * Bits    | Name              | Access | Description
   * --------|-------------------|--------|-------------
   * 15:0    | ABN_PHI_E_OFFSET  | RW     | Offset for ABN_PHI_E
   *
   * @note This register provides the offset for aligning the encoder angle
   */
  struct ABN_PHI_E_OFFSET {
    static constexpr uint16_t ADDRESS = 0x0A6;
    union {
      uint32_t value;
      struct {
        int16_t ABN_PHI_E_OFFSET : 16; ///< Offset for ABN_PHI_E.
        uint16_t : 16;
      } bits;
    };
  };

  /**
   * @brief Hall Sensor Mode Register (MCC_HALL_MODE)
   * @details Block 0, Address: 0x0C0
   *
   * Configuration for the digital Hall sensor interface.
   *
   * Register Map:
   * Bits    | Name            | Access | Description
   * --------|-----------------|--------|-------------
   * 0       | POLARITY        | RW     | Polarity of the hall signals
   * 1       | EXTRAPOLATION   | RW     | Enable extrapolation for PHI_E
   * 6:4     | ORDER           | RW     | Ordering of the hall signals
   * 15:8    | FILTER          | RW     | Filter length for hall signals
   *
   * @note This register allows configuration of the Hall sensor interface
   */
  struct HALL_MODE {
    static constexpr uint16_t ADDRESS = 0x0C0;
    union {
      uint32_t value;
      struct {
        uint32_t POLARITY : 1;      ///< Polarity of the hall signals.
        uint32_t EXTRAPOLATION : 1; ///< Enable extrapolation for PHI_E.
        uint32_t : 2;
        uint32_t ORDER : 3; ///< Ordering of the hall signals.
        uint32_t : 1;
        uint32_t FILTER : 8; ///< Filter length for hall signals.
        uint32_t : 16;
      } bits;
    };
  };

  /**
   * @brief Hall Sensor Digital Filter Register (MCC_HALL_DPHI_MAX)
   * @details Block 0, Address: 0x0C1
   *
   * Maximum delta in hall angle per timestep (to filter noise).
   *
   * Register Map:
   * Bits    | Name            | Access | Description
   * --------|-----------------|--------|-------------
   * 15:0    | HALL_DPHI_MAX   | RW     | Maximum phi_e change for extrapolation
   *
   * @note This register provides the maximum allowable hall angle change per sample
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

  /**
   * @brief Hall Sensor Angle Offset Register (MCC_HALL_PHI_E_OFFSET)
   * @details Block 0, Address: 0x0C2
   *
   * Electrical angle offset added to hall-sensor-derived angle (to align with motor FOC angle).
   *
   * Register Map:
   * Bits    | Name                | Access | Description
   * --------|---------------------|--------|-------------
   * 15:0    | HALL_PHI_E_OFFSET   | RW     | Offset for electrical angle hall_phi_e
   *
   * @note This register provides the offset for aligning the hall sensor angle
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

  /**
   * @brief Hall Sensor Counter Register (MCC_HALL_COUNT)
   * @details Block 0, Address: 0x0C3
   *
   * Counter for hall sensor transitions or integrated hall angle (if extrapolation is used).
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 15:0    | HALL_COUNT    | R      | Count of passed hall states
   *
   * @note This register provides the count of passed hall states
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

  /**
   * @brief Hall Sensor Extrapolated Angle Register (MCC_HALL_PHI_E_EXTRAPOLATED_PHI_E)
   * @details Block 0, Address: 0x0C4
   *
   * Extrapolated electrical angle (phi_e) from hall sensors (if prediction/extrapolation enabled).
   *
   * Register Map:
   * Bits    | Name                    | Access | Description
   * --------|-------------------------|--------|-------------
   * 31:16   | PHI_E_EXTRAPOLATED      | R      | Extrapolated electrical angle
   * hall_phi_e_extrapolated 15:0    | PHI_E                   | R      | Electrical angle
   * hall_phi_e
   *
   * @note This register provides the extrapolated and raw electrical angles from the hall sensors
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

  /**
   * @brief Hall Sensor Positions 0°/60° Register (MCC_HALL_POSITION_060_000)
   * @details Block 0, Address: 0x0C5
   *
   * Exact electrical positions (hall sensor thresholds) for hall sensor states corresponding to 0°
   * and 60° electrical angles.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | POSITION_060  | RW     | Hall sensor angle at 60° electrical
   * 15:0    | POSITION_000  | RW     | Hall sensor angle at 0° electrical
   *
   * @note This register provides the hall sensor positions for 0° and 60° electrical angles
   */
  struct HALL_POSITION_060_000 {
    static constexpr uint16_t ADDRESS = 0x0C5;
    union {
      uint32_t value;
      struct {
        int16_t POSITION_060 : 16; ///< Hall sensor angle at 60° electrical.
        int16_t POSITION_000 : 16; ///< Hall sensor angle at 0° electrical.
      } bits;
    };
    static constexpr int16_t RESET_POSITION_060 = 0x2AAA; ///< Default reset value for POSITION_060.
    static constexpr int16_t RESET_POSITION_000 = 0x0000; ///< Default reset value for POSITION_000.
  };

  /**
   * @brief Hall Sensor Positions 180°/120° Register (MCC_HALL_POSITION_180_120)
   * @details Block 0, Address: 0x0C6
   *
   * Exact positions for hall states at 180° and 120°.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | POSITION_180  | RW     | Hall sensor angle at 180° electrical
   * 15:0    | POSITION_120  | RW     | Hall sensor angle at 120° electrical
   *
   * @note This register provides the hall sensor positions for 180° and 120° electrical angles
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

  /**
   * @brief Hall Sensor Positions 300°/240° Register (MCC_HALL_POSITION_300_240)
   * @details Block 0, Address: 0x0C7
   *
   * Exact positions for hall states at 300° and 240°.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | POSITION_300  | RW     | Hall sensor angle at 300° electrical
   * 15:0    | POSITION_240  | RW     | Hall sensor angle at 240° electrical
   *
   * @note This register provides the hall sensor positions for 300° and 240° electrical angles
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

  /**
   * @brief Velocity Biquad Filter Coefficient A1 Register (MCC_BIQUAD_V_A1)
   * @details Block 0, Address: 0x0E0
   *
   * A1 coefficient for the velocity PI controller's biquad (filter).
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 23:0    | BIQUAD_V_A1   | RW     | Biquad velocity filter coefficient A1 (signed)
   *
   * @note This register provides the A1 coefficient for the velocity filter biquad
   */
  struct BIQUAD_V_A1 {
    static constexpr uint16_t ADDRESS = 0x0E0;
    int32_t A1 : 24; ///< Biquad velocity filter coefficient A1.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_V_A1 = 0x1C376B; ///< Default reset value for BIQUAD_V_A1.
  };

  /**
   * @brief Velocity Biquad Filter Coefficient A2 Register (MCC_BIQUAD_V_A2)
   * @details Block 0, Address: 0x0E1
   *
   * A2 coefficient for velocity filter biquad.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 23:0    | BIQUAD_V_A2   | RW     | Biquad velocity filter coefficient A2 (signed)
   *
   * @note This register provides the A2 coefficient for the velocity filter biquad
   */
  struct BIQUAD_V_A2 {
    static constexpr uint16_t ADDRESS = 0x0E1;
    int32_t A2 : 24; ///< Biquad velocity filter coefficient A2.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_V_A2 = 0xF38F52; ///< Default reset value for A2.
  };

  /**
   * @brief Velocity Biquad Filter Coefficient B0 Register (MCC_BIQUAD_V_B0)
   * @details Block 0, Address: 0x0E2
   *
   * B0 coefficient for velocity filter biquad.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 23:0    | BIQUAD_V_B0   | RW     | Biquad velocity filter coefficient B0 (signed)
   *
   * @note This register provides the B0 coefficient for the velocity filter biquad
   */
  struct BIQUAD_V_B0 {
    static constexpr uint16_t ADDRESS = 0x0E2;
    int32_t B0 : 24; ///< Biquad velocity filter coefficient B0.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_V_B0 = 0x000E51; ///< Default reset value for B0.
  };

  /**
   * @brief Velocity Biquad Filter Coefficient B1 Register (MCC_BIQUAD_V_B1)
   * @details Block 0, Address: 0x0E3
   *
   * B1 coefficient for velocity filter biquad.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 23:0    | BIQUAD_V_B1   | RW     | Biquad velocity filter coefficient B1 (signed)
   *
   * @note This register provides the B1 coefficient for the velocity filter biquad
   */
  struct BIQUAD_V_B1 {
    static constexpr uint16_t ADDRESS = 0x0E3;
    int32_t B1 : 24; ///< Biquad velocity filter coefficient B1.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_V_B1 = 0x001CA1; ///< Default reset value for B1.
  };

  /**
   * @brief Velocity Biquad Filter Coefficient B2 Register (MCC_BIQUAD_V_B2)
   * @details Block 0, Address: 0x0E4
   *
   * B2 coefficient for velocity filter biquad.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 23:0    | BIQUAD_V_B2   | RW     | Biquad velocity filter coefficient B2 (signed)
   *
   * @note This register provides the B2 coefficient for the velocity filter biquad
   */
  struct BIQUAD_V_B2 {
    static constexpr uint16_t ADDRESS = 0x0E4;
    int32_t B2 : 24; ///< Biquad velocity filter coefficient B2.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_V_B2 = 0x000E51; ///< Default reset value for B2.
  };

  /**
   * @brief Velocity Biquad Filter Enable Register (MCC_BIQUAD_V_ENABLE)
   * @details Block 0, Address: 0x0E5
   *
   * Enable/disable the velocity biquad filter.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 0       | ENABLED       | RW     | 1 to enable velocity filter, 0 to bypass
   *
   * @note This register allows enabling or disabling the velocity biquad filter
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

  /**
   * @brief Torque (Flux) Biquad Filter Coefficient A1 Register (MCC_BIQUAD_T_A1)
   * @details Block 0, Address: 0x0E6
   *
   * Biquad torque filter coefficient A1.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 23:0    | BIQUAD_T_A1   | RW     | Biquad torque filter coefficient A1 (signed)
   *
   * @note This register provides the A1 coefficient for the torque filter biquad
   */
  struct BIQUAD_T_A1 {
    static constexpr uint16_t ADDRESS = 0x0E6;
    int32_t A1 : 24; ///< Biquad torque filter coefficient A1.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_T_A1 = 0x000000; ///< Default reset value for A1.
  };

  /**
   * @brief Torque (Flux) Biquad Filter Coefficient A2 Register (MCC_BIQUAD_T_A2)
   * @details Block 0, Address: 0x0E7
   *
   * Biquad torque filter coefficient A2.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 23:0    | BIQUAD_T_A2   | RW     | Biquad torque filter coefficient A2 (signed)
   *
   * @note This register provides the A2 coefficient for the torque filter biquad
   */
  struct BIQUAD_T_A2 {
    static constexpr uint16_t ADDRESS = 0x0E7;
    int32_t A2 : 24; ///< Biquad torque filter coefficient A2.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_T_A2 = 0x000000; ///< Default reset value for A2.
  };

  /**
   * @brief Torque (Flux) Biquad Filter Coefficient B0 Register (MCC_BIQUAD_T_B0)
   * @details Block 0, Address: 0x0E8
   *
   * Biquad torque filter coefficient B0.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 23:0    | BIQUAD_T_B0   | RW     | Biquad torque filter coefficient B0 (signed)
   *
   * @note This register provides the B0 coefficient for the torque filter biquad
   */
  struct BIQUAD_T_B0 {
    static constexpr uint16_t ADDRESS = 0x0E8;
    int32_t B0 : 24; ///< Biquad torque filter coefficient B0.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_T_B0 = 0x100000; ///< Default reset value for B0.
  };

  /**
   * @brief Torque (Flux) Biquad Filter Coefficient B1 Register (MCC_BIQUAD_T_B1)
   * @details Block 0, Address: 0x0E9
   *
   * Biquad torque filter coefficient B1.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 23:0    | BIQUAD_T_B1   | RW     | Biquad torque filter coefficient B1 (signed)
   *
   * @note This register provides the B1 coefficient for the torque filter biquad
   */
  struct BIQUAD_T_B1 {
    static constexpr uint16_t ADDRESS = 0x0E9;
    int32_t B1 : 24; ///< Biquad torque filter coefficient B1.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_T_B1 = 0x000000; ///< Default reset value for B1.
  };

  /**
   * @brief Torque (Flux) Biquad Filter Coefficient B2 Register (MCC_BIQUAD_T_B2)
   * @details Block 0, Address: 0x0EA
   *
   * Biquad torque filter coefficient B2.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 23:0    | BIQUAD_T_B2   | RW     | Biquad torque filter coefficient B2 (signed)
   *
   * @note This register provides the B2 coefficient for the torque filter biquad
   */
  struct BIQUAD_T_B2 {
    static constexpr uint16_t ADDRESS = 0x0EA;
    int32_t B2 : 24; ///< Biquad torque filter coefficient B2.
    int32_t : 8;
    static constexpr int32_t RESET_BIQUAD_T_B2 = 0x000000; ///< Default reset value for B2.
  };

  /**
   * @brief Torque (Flux) Biquad Filter Enable Register (MCC_BIQUAD_T_ENABLE)
   * @details Block 0, Address: 0x0EB
   *
   * Enable/disable the torque biquad filter.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 0       | ENABLE        | RW     | 1 to enable torque filter, 0 to bypass
   *
   * @note This register allows enabling or disabling the torque biquad filter
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

  /**
   * @brief Velocity PI Controller Configuration Register (MCC_VELOCITY_CONFIG)
   * @details Block 0, Address: 0x100
   *
   * Configuration bits for the velocity PI controller.
   *
   * Register Map:
   * Bits    | Name                        | Access | Description
   * --------|-----------------------------|--------|-------------
   * 7:0     | SELECTION                   | RW     | Source of rotor position for velocity
   * measurement 8       | METER_SYNC_PULSE            | RW     | Synchronization pulse for velocity
   * meter 10:9    | METER_TYPE                  | RW     | Velocity meter type selection 14:12   |
   * MOVING_AVRG_FILTER_SAMPLES  | RW     | Moving average filter samples
   *
   * @note This register allows configuration of the velocity PI controller
   */
  struct VELOCITY_CONFIG {
    static constexpr uint16_t ADDRESS = 0x100;
    union {
      uint32_t value;
      struct {
        uint32_t SELECTION : 8;        ///< Source of rotor position for velocity measurement.
        uint32_t METER_SYNC_PULSE : 1; ///< Synchronization pulse for velocity meter.
        uint32_t METER_TYPE : 2;       ///< Velocity meter type selection.
        uint32_t : 1;
        uint32_t MOVING_AVRG_FILTER_SAMPLES : 3; ///< Moving average filter samples.
        uint32_t : 17;
      } bits;
    };
  };

  /**
   * @brief Velocity Scaling Register (MCC_VELOCITY_SCALING)
   * @details Block 0, Address: 0x101
   *
   * Scaling factor for velocity meter output. This value is only used when VELOCITY_FREQ in
   * MCC_VELOCITY_CONFIG - METER_TYPE is selected.
   *
   * Register Map:
   * Bits    | Name              | Access | Description
   * --------|-------------------|--------|-------------
   * 15:0    | VELOCITY_SCALING  | RW     | Scaling factor for velocity meter output
   *
   * @note This register provides the scaling factor for the velocity meter output
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
    static constexpr int16_t RESET_VELOCITY_SCALING =
        0x28F6; ///< Default reset value for VELOCITY_SCALING.
  };

  /**
   * @brief Velocity Meter Minimal Deviation and Counter Limit Register
   * (MCC_V_MIN_POS_DEV_TIME_COUNTER_LIMIT)
   * @details Block 0, Address: 0x102
   *
   * Velocity meter configuration. These values are only used when VELOCITY_PER in
   * MCC_VELOCITY_CONFIG - METER_TYPE is selected.
   *
   * Register Map:
   * Bits    | Name                | Access | Description
   * --------|---------------------|--------|-------------
   * 30:16   | V_MIN_POS_DEV       | RW     | Minimal position deviation to calculate velocity
   * 15:0    | TIME_COUNTER_LIMIT  | RW     | Counter limit for velocity minimum deviation
   * functionality
   *
   * @note This register provides the configuration for the velocity meter
   */
  struct V_MIN_POSDEV_TIME {
    static constexpr uint16_t ADDRESS = 0x102;
    union {
      uint32_t value;
      struct {
        uint32_t TIME_COUNTER_LIMIT : 16; ///< Counter limit for velocity minimum deviation
                                          ///< functionality.
        uint32_t V_MIN_POS_DEV : 15;      ///< Minimal position deviation to calculate velocity.
        uint32_t : 1;
      } bits;
    };
    static constexpr uint32_t RESET_V_MIN_POS_DEV =
        0x001; ///< Default reset value for V_MIN_POS_DEV.
    static constexpr uint32_t RESET_TIME_COUNTER_LIMIT =
        0xFFF0; ///< Default reset value for TIME_COUNTER_LIMIT.
  };

  /**
   * @brief Maximum Velocity Deviation Register (MCC_MAX_VEL_DEVIATION)
   * @details Block 0, Address: 0x103
   *
   * Velocity deviation to generate tracking error flag.
   *
   * Register Map:
   * Bits    | Name              | Access | Description
   * --------|-------------------|--------|-------------
   * 30:0    | MAX_VEL_DEVIATION | RW     | Maximum allowed absolute velocity deviation/error
   *
   * @note This register provides the maximum allowed absolute velocity deviation/error
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
    static constexpr uint32_t RESET_MAX_VEL_DEVIATION =
        0x0010000; ///< Default reset value for MAX_VEL_DEVIATION.
  };

  /**
   * @brief Position Control Configuration Register (MCC_POSITION_CONFIG)
   * @details Block 0, Address: 0x120
   *
   * Configuration of position measurement and control settings.
   *
   * Register Map:
   * Bits    | Name        | Access | Description
   * --------|-------------|--------|-------------
   * 7:0     | SELECTION   | RW     | Position measurement source selection
   *
   * @note This register provides the configuration for position measurement and control
   */
  struct POSITION_CONFIG {
    static constexpr uint16_t ADDRESS = 0x120;
    union {
      uint32_t value;
      struct {
        uint8_t SELECTION; ///< Position measurement source selection.
        uint8_t : 24;
      } bits;
    };
  };

  /**
   * @brief Maximum Position Deviation Register (MCC_MAX_POS_DEVIATION)
   * @details Block 0, Address: 0x121
   *
   * Maximum allowed position error for tracking before a tracking error is flagged.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 30:0    | MAX_POS_ERR   | RW     | Maximum allowed position error
   *
   * @note This register provides the maximum allowed position error for tracking
   */
  struct MAX_POS_DEVIATION {
    static constexpr uint16_t ADDRESS = 0x121;
    union {
      uint32_t value;
      struct {
        uint32_t MAX_POS_ERR : 31; ///< Maximum allowed position error (unsigned).
        uint32_t : 1;
      } bits;
    };
  };

  /**
   * @brief Ramp Status and Switch Event Register (MCC_RAMP_STATUS)
   * @details Block 0, Address: 0x140
   *
   * Contains status flags for the ramp generator and limit switch events.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 0       | STATUS_STOP_L         | R      | Left reference switch status
   * 1       | STATUS_STOP_R         | R      | Right reference switch status
   * 2       | STATUS_STOP_H         | R      | Home reference switch status
   * 3       | STATUS_LATCH_L        | RW     | Latch left ready
   * 4       | STATUS_LATCH_R        | RW     | Latch right ready
   * 5       | STATUS_LATCH_H        | RW     | Latch home ready
   * 6       | EVENT_STOP_L          | R      | Active stop left condition
   * 7       | EVENT_STOP_R          | R      | Active stop right condition
   * 8       | EVENT_STOP_H          | R      | Active stop home condition
   * 9       | EVENT_STOP_SG         | RW     | Active stop event
   * 10      | EVENT_POS_REACHED     | RW     | Target position reached event
   * 11      | VELOCITY_REACHED      | R      | Target velocity reached
   * 12      | POSITION_REACHED      | R      | Target position reached
   * 13      | V_ZERO                | R      | Velocity is zero
   * 14      | T_ZEROWAIT_ACTIVE     | R      | Zero-wait time active after stop
   * 15      | SECOND_MOVE           | RW     | Second move (reverse) was required
   * 16      | STALL_IN_VEL_ERR      | R      | Velocity tracking error (stall)
   * 17      | STALL_IN_POS_ERR      | R      | Position tracking error (stall)
   *
   * @note This register provides status flags for the ramp generator and limit switch events
   */
  struct RAMP_STATUS {
    static constexpr uint16_t ADDRESS = 0x140;
    union {
      uint32_t value;
      struct {
        uint32_t STATUS_STOP_L : 1;     ///< Left reference switch status.
        uint32_t STATUS_STOP_R : 1;     ///< Right reference switch status.
        uint32_t STATUS_STOP_H : 1;     ///< Home reference switch status.
        uint32_t STATUS_LATCH_L : 1;    ///< Latch left ready.
        uint32_t STATUS_LATCH_R : 1;    ///< Latch right ready.
        uint32_t STATUS_LATCH_H : 1;    ///< Latch home ready.
        uint32_t EVENT_STOP_L : 1;      ///< Active stop left condition.
        uint32_t EVENT_STOP_R : 1;      ///< Active stop right condition.
        uint32_t EVENT_STOP_H : 1;      ///< Active stop home condition.
        uint32_t EVENT_STOP_SG : 1;     ///< Active stop event.
        uint32_t EVENT_POS_REACHED : 1; ///< Target position reached event.
        uint32_t VELOCITY_REACHED : 1;  ///< Target velocity reached.
        uint32_t POSITION_REACHED : 1;  ///< Target position reached.
        uint32_t V_ZERO : 1;            ///< Velocity is zero.
        uint32_t T_ZEROWAIT_ACTIVE : 1; ///< Zero-wait time active after stop.
        uint32_t SECOND_MOVE : 1;       ///< Second move (reverse) was required.
        uint32_t STALL_IN_VEL_ERR : 1;  ///< Velocity tracking error (stall).
        uint32_t STALL_IN_POS_ERR : 1;  ///< Position tracking error (stall).
        uint32_t : 14;
      } bits;
    };
  };

  /**
   * @brief Ramp Generator Acceleration A1 Register (MCC_RAMPER_A1)
   * @details Block 0, Address: 0x141
   *
   * First acceleration value during EigthPoint ramp mode.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 22:0    | RAMPER_A1     | RW     | Acceleration value for the first ramp segment
   *
   * @note This register provides the first acceleration value for the ramp generator
   */
  struct RAMP_A1 {
    static constexpr uint16_t ADDRESS = 0x141;
    uint32_t RAMPER_A1 : 23; ///< Acceleration value for the first ramp segment.
    uint32_t : 9;
  };

  /**
   * @brief Ramp Generator Acceleration A2 Register (MCC_RAMPER_A2)
   * @details Block 0, Address: 0x142
   *
   * Second acceleration value during EigthPoint ramp mode.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 22:0    | RAMPER_A2     | RW     | Acceleration value for the second ramp segment
   *
   * @note This register provides the second acceleration value for the ramp generator
   */
  struct RAMP_A2 {
    static constexpr uint16_t ADDRESS = 0x142;
    uint32_t RAMPER_A2 : 23; ///< Acceleration value for the second ramp segment.
    uint32_t : 9;
  };

  /**
   * @brief Ramp Generator Maximum Acceleration Register (MCC_RAMPER_A_MAX)
   * @details Block 0, Address: 0x143
   *
   * Maximum acceleration value in the top part of EigthPoint ramp mode.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 22:0    | RAMPER_A_MAX  | RW     | Maximum acceleration value for the top ramp segment
   *
   * @note This register provides the maximum acceleration value for the ramp generator
   */
  struct RAMP_A_MAX {
    static constexpr uint16_t ADDRESS = 0x143;
    uint32_t RAMPER_A_MAX : 23; ///< Maximum acceleration value for the top ramp segment.
    uint32_t : 9;
  };

  /**
   * @brief Ramp Generator Deceleration D1 Register (MCC_RAMPER_D1)
   * @details Block 0, Address: 0x144
   *
   * Lower deceleration value during EigthPoint ramp mode.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 22:0    | RAMPER_D1     | RW     | Deceleration value for the last ramp segment
   *
   * @note This register provides the lower deceleration value for the ramp generator
   */
  struct RAMP_D1 {
    static constexpr uint16_t ADDRESS = 0x144;
    uint32_t RAMPER_D1 : 23; ///< Deceleration value for the last ramp segment.
    uint32_t : 9;
  };

  /**
   * @brief Ramp Generator Deceleration D2 Register (MCC_RAMPER_D2)
   * @details Block 0, Address: 0x145
   *
   * Higher deceleration value in EigthPoint ramp mode.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 22:0    | RAMPER_D2     | RW     | Deceleration value for the higher ramp segment
   *
   * @note This register provides the higher deceleration value for the ramp generator
   */
  struct RAMP_D2 {
    static constexpr uint16_t ADDRESS = 0x145;
    uint32_t RAMPER_D2 : 23; ///< Deceleration value for the higher ramp segment.
    uint32_t : 9;
  };

  /**
   * @brief Ramp Generator Maximum Deceleration Register (MCC_RAMPER_D_MAX)
   * @details Block 0, Address: 0x146
   *
   * Deceleration in the top part of EigthPoint ramp mode.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 22:0    | RAMPER_D_MAX  | RW     | Maximum deceleration value for the top ramp segment
   *
   * @note This register provides the maximum deceleration value for the ramp generator
   */
  struct RAMP_D_MAX {
    static constexpr uint16_t ADDRESS = 0x146;
    uint32_t RAMPER_D_MAX : 23; ///< Maximum deceleration value for the top ramp segment.
    uint32_t : 9;
  };

  /**
   * @brief Ramp Generator Start Velocity Register (MCC_RAMPER_V_START)
   * @details Block 0, Address: 0x147
   *
   * First velocity value during EigthPoint ramp mode.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 22:0    | RAMPER_V_START| RW     | Start velocity for position ramp mode
   *
   * @note This register provides the start velocity for the ramp generator
   */
  struct RAMP_V_START {
    static constexpr uint16_t ADDRESS = 0x147;
    uint32_t RAMPER_V_START : 23; ///< Start velocity for position ramp mode.
    uint32_t : 9;
  };

  /**
   * @brief Ramp Generator Threshold Velocity 1 Register (MCC_RAMPER_V1)
   * @details Block 0, Address: 0x148
   *
   * First velocity value for ac-/deceleration value switching during EigthPoint ramp mode.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 26:0    | RAMPER_V1     | RW     | Velocity threshold for switching acceleration/deceleration
   * values
   *
   * @note This register provides the first velocity threshold for the ramp generator
   */
  struct RAMP_V1 {
    static constexpr uint16_t ADDRESS = 0x148;
    uint32_t RAMPER_V1 : 27; ///< Velocity threshold for switching acceleration/deceleration values.
    uint32_t : 5;
  };

  /**
   * @brief Ramp Generator Threshold Velocity 2 Register (MCC_RAMPER_V2)
   * @details Block 0, Address: 0x149
   *
   * Second velocity value for ac-/deceleration value switching during EigthPoint ramp mode.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 26:0    | RAMPER_V2     | RW     | Second velocity threshold for switching
   * acceleration/deceleration values
   *
   * @note This register provides the second velocity threshold for the ramp generator
   */
  struct RAMP_V2 {
    static constexpr uint16_t ADDRESS = 0x149;
    uint32_t RAMPER_V2 : 27; ///< Second velocity threshold for switching acceleration/deceleration
                             ///< values.
    uint32_t : 5;
  };

  /**
   * @brief Ramp Generator Stop Velocity Register (MCC_RAMPER_V_STOP)
   * @details Block 0, Address: 0x14A
   *
   * Stop velocity in ramp in position ramp mode.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 22:0    | RAMPER_V_STOP | RW     | Stop velocity for position ramp mode
   *
   * @note This register provides the stop velocity for the ramp generator
   */
  struct RAMP_V_STOP {
    static constexpr uint16_t ADDRESS = 0x14A;
    uint32_t RAMPER_V_STOP : 23; ///< Stop velocity for position ramp mode.
    uint32_t : 9;
  };

  /**
   * @brief Ramp Generator Maximum Velocity Register (MCC_RAMPER_V_MAX)
   * @details Block 0, Address: 0x14B
   *
   * Maximum velocity value for positioning in EigthPoint ramp mode.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 26:0    | RAMPER_V_MAX  | RW     | Maximum velocity value for ramp positioning mode
   *
   * @note This register provides the maximum velocity value for the ramp generator
   */
  struct RAMP_V_MAX {
    static constexpr uint16_t ADDRESS = 0x14B;
    uint32_t RAMPER_V_MAX : 27; ///< Maximum velocity value for ramp positioning mode.
    uint32_t : 5;
  };

  /**
   * @brief Ramp Generator Target Velocity Register (MCC_RAMPER_V_TARGET)
   * @details Block 0, Address: 0x14C
   *
   * Target velocity value in EigthPoint ramp mode.
   *
   * Register Map:
   * Bits    | Name              | Access | Description
   * --------|-------------------|--------|-------------
   * 27:0    | RAMPER_V_TARGET   | RW     | Target velocity for ramp velocity mode
   *
   * @note This register provides the target velocity for the ramp generator
   */
  struct RAMP_V_TARGET {
    static constexpr uint16_t ADDRESS = 0x14C;
    int32_t RAMPER_V_TARGET : 28; ///< Target velocity for ramp velocity mode.
    uint32_t : 4;
  };

  /**
   * @brief Ramp Generator Switch Mode Register (MCC_RAMPER_SWITCH_MODE)
   * @details Block 0, Address: 0x14D
   *
   * Configures behavior of stop switches (if using endstops or reference switches).
   *
   * Register Map:
   * Bits    | Name                    | Access | Description
   * --------|-------------------------|--------|-------------
   * 0       | STOP_L_ENABLE           | RW     | Enables automatic motor stop during active left
   * reference switch input 1       | STOP_R_ENABLE           | RW     | Enables automatic motor
   * stop during active right reference switch input 2       | STOP_H_ENABLE           | RW     |
   * Enables automatic motor stop during active home reference switch input 3       | STOP_L_POL |
   * RW     | Defines active polarity of the left reference switch input 4       | STOP_R_POL | RW
   * | Defines active polarity of the right reference switch input 5       | STOP_H_POL | RW     |
   * Defines active polarity of the home reference switch input 6       | SWAP_LR                 |
   * RW     | Swaps left and right reference switch inputs internally 7       | LATCH_L_ACTIVE | RW
   * | Activates position latching to RAMPER_X_ACTUAL_LATCH when left reference switch is activated
   * 8       | LATCH_L_INACTIVE        | RW     | Activates position latching to
   * RAMPER_X_ACTUAL_LATCH when left reference switch is deactivated 9       | LATCH_R_ACTIVE | RW
   * | Activates position latching to RAMPER_X_ACTUAL_LATCH when right reference switch is activated
   * 10      | LATCH_R_INACTIVE        | RW     | Activates position latching to
   * RAMPER_X_ACTUAL_LATCH when right reference switch is deactivated 11      | LATCH_H_ACTIVE | RW
   * | Activates position latching to RAMPER_X_ACTUAL_LATCH when home reference switch is activated
   * 12      | LATCH_H_INACTIVE        | RW     | Activates position latching to
   * RAMPER_X_ACTUAL_LATCH when home reference switch is deactivated 14      | SG_STOP_ENABLE | RW
   * | Enables stop conditions like SW_HARD_STOP, STOP_ON_POS_DEVIATION, STOP_ON_VEL_DEVIATION 15 |
   * SOFTSTOP_ENABLE         | RW     | Enables soft stop mode using deceleration ramp settings 16
   * | SW_HARD_STOP            | RW     | Enables a hard stop during ramp mode in case any activated
   * reference switch has been triggered 17      | STOP_ON_POS_DEVIATION   | RW     | Enables a hard
   * stop during ramp mode if position tracking error emerges 18      | STOP_ON_VEL_DEVIATION   | RW
   * | Enables a hard stop during ramp mode if velocity tracking error emerges 19      |
   * VELOCITY_OVERWRITE      | RW     | If enabled, velocity from overwrite input
   * (PID_VELOCITY_TARGET) is written to ramp
   *
   * @note This register provides the configuration for stop switches and ramp behavior
   */
  struct RAMP_SWITCH_MODE {
    static constexpr uint16_t ADDRESS = 0x14D;
    union {
      uint32_t value;
      struct {
        uint32_t STOP_L_ENABLE : 1;
        uint32_t STOP_R_ENABLE : 1;
        uint32_t STOP_H_ENABLE : 1;
        uint32_t STOP_L_POL : 1;
        uint32_t STOP_R_POL : 1;
        uint32_t STOP_H_POL : 1;
        uint32_t SWAP_LR : 1;
        uint32_t LATCH_L_ACTIVE : 1;
        uint32_t LATCH_L_INACTIVE : 1;
        uint32_t LATCH_R_ACTIVE : 1;
        uint32_t LATCH_R_INACTIVE : 1;
        uint32_t LATCH_H_ACTIVE : 1;
        uint32_t LATCH_H_INACTIVE : 1;
        uint32_t SG_STOP_ENABLE : 1;
        uint32_t SOFTSTOP_ENABLE : 1;
        uint32_t SW_HARD_STOP : 1;
        uint32_t STOP_ON_POS_DEVIATION : 1;
        uint32_t STOP_ON_VEL_DEVIATION : 1;
        uint32_t VELOCITY_OVERWRITE : 1;
        uint32_t : 13;
      } bits;
    };
  };

  /**
   * @brief Ramp Generator Timing Configuration Register (MCC_RAMPER_TIME_CONFIG)
   * @details Block 0, Address: 0x14E
   *
   * Timing settings for the ramp generator jerk reduction.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | T_VMAX        | RW     | Minimum time at constant velocity before deceleration
   * 15:0    | T_ZEROWAIT    | RW     | Wait time at zero velocity before next movement
   *
   * @note This register provides the timing settings for the ramp generator
   */
  struct RAMP_TIME_CONFIG {
    static constexpr uint16_t ADDRESS = 0x14E;
    union {
      uint32_t value;
      struct {
        uint16_t T_VMAX : 16;     ///< Minimum time at constant velocity before deceleration (units
                                  ///< of 12.8 µs).
        uint16_t T_ZEROWAIT : 16; ///< Wait time at zero velocity before next movement (units
                                  ///< of 12.8 µs).
      } bits;
    };
    static constexpr uint32_t RESET_T_VMAX = 0x0000;     ///< Default reset value for T_VMAX.
    static constexpr uint32_t RESET_T_ZEROWAIT = 0x0000; ///< Default reset value for T_ZEROWAIT.
  };

  /**
   * @brief Actual Ramp Acceleration Register (MCC_RAMPER_A_ACTUAL)
   * @details Block 0, Address: 0x14F
   *
   * Actual acceleration value currently being applied by the ramp generator.
   *
   * Register Map:
   * Bits    | Name              | Access | Description
   * --------|-------------------|--------|-------------
   * 23:0    | RAMPER_A_ACTUAL   | R      | Actual acceleration (signed)
   *
   * @note This register provides the actual acceleration value for the ramp generator
   */
  struct RAMP_A_ACTUAL {
    static constexpr uint16_t ADDRESS = 0x14F;
    union {
      uint32_t value;
      struct {
        int32_t A_ACTUAL : 24; ///< Actual acceleration (signed).
        uint32_t : 8;
      } bits;
    };
    static constexpr int32_t RESET_RAMPER_A_ACTUAL =
        0x000000; ///< Default reset value for RAMPER_A_ACTUAL.
  };

  /**
   * @brief Actual Ramp Position Register (MCC_RAMPER_X_ACTUAL)
   * @details Block 0, Address: 0x150
   *
   * Multi-turn position output of the ramp generator.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:0    | X_ACTUAL      | R      | Actual ramp position output (signed)
   *
   * @note This register provides the actual position output of the ramp generator
   */
  struct RAMP_X_ACTUAL {
    static constexpr uint16_t ADDRESS = 0x150;
    int32_t X_ACTUAL; ///< Actual ramp position output (signed 32-bit).
    static constexpr int32_t RESET_RAMPER_X_ACTUAL =
        0x00000000; ///< Default reset value for RAMPER_X_ACTUAL.
  };

  /**
   * @brief Actual Ramp Velocity Register (MCC_RAMPER_V_ACTUAL)
   * @details Block 0, Address: 0x151
   *
   * Current velocity output of the ramp generator.
   *
   * Register Map:
   * Bits    | Name              | Access | Description
   * --------|-------------------|--------|-------------
   * 27:0    | RAMPER_V_ACTUAL   | R      | Actual ramp velocity (signed)
   *
   * @note This register provides the actual velocity output of the ramp generator
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

  /**
   * @brief Ramp Target Position Register (MCC_RAMPER_X_TARGET)
   * @details Block 0, Address: 0x152
   *
   * Multi-turn target position of the ramp controller.
   *
   * Register Map:
   * Bits    | Name              | Access | Description
   * --------|-------------------|--------|-------------
   * 31:0    | RAMPER_X_TARGET   | RW     | Target position of ramp controller
   *
   * @note This register provides the target position for the ramp controller
   */
  struct RAMP_X_TARGET {
    static constexpr uint16_t ADDRESS = 0x152;
    int32_t RAMPER_X_TARGET;
  };

  /**
   * @brief Ramp Electrical Angle Register (MCC_RAMPER_PHI_E)
   * @details Block 0, Address: 0x153
   *
   * PHI_E of the ramp controller.
   *
   * Register Map:
   * Bits    | Name              | Access | Description
   * --------|-------------------|--------|-------------
   * 15:0    | RAMPER_PHI_E      | R      | PHI_E calculated from RAMPER_X_ACTUAL × N_POLE_PAIRS +
   * Offset
   *
   * @note This register provides the electrical angle of the ramp controller
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

  /**
   * @brief Ramp Feedforward Acceleration Register (MCC_RAMPER_ACC_FF)
   * @details Block 0, Address: 0x155
   *
   * Gain and shift factor for acceleration feedforward.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 15:0    | GAIN          | RW     | Gain factor for acceleration feedforward
   * 18:16   | SHIFT         | RW     | Shift factor for acceleration feedforward
   *
   * @note This register provides the gain and shift factor for acceleration feedforward
   */
  struct RAMP_ACC_FF {
    static constexpr uint16_t ADDRESS = 0x155;
    union {
      uint32_t value;
      struct {
        uint32_t GAIN : 16; ///< Gain factor for acceleration feedforward.
        uint32_t SHIFT : 3; ///< Shift factor for acceleration feedforward.
        uint32_t : 13;
      } bits;
    };
  };

  /**
   * @brief Latched Multi-turn Position Register (MCC_RAMPER_X_ACTUAL_LATCH)
   * @details Block 0, Address: 0x156
   *
   * Latches RAMPER_X_ACTUAL on left or right switch or encoder trigger.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 31:0    | RAMPER_X_ACTUAL_LATCH     | R      | Latched X-Actual value at stop switch event
   *
   * @note This register provides the latched actual position of the ramp generator
   */
  struct RAMP_X_ACTUAL_LATCH {
    static constexpr uint16_t ADDRESS = 0x156;
    int32_t RAMPER_X_ACTUAL_LATCH;
  };

  /**
   * @brief Latched Actual Position Register (MCC_POSITION_ACTUAL_LATCH)
   * @details Block 0, Address: 0x157
   *
   * Latches PID_POSITION_ACTUAL on left or right switch or encoder trigger.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 31:0    | POSITION_ACTUAL_LATCH     | R      | Actual feedback position latch at stop switch
   * event
   *
   * @note This register provides the latched actual position of the PID controller
   */
  struct POSITION_ACTUAL_LATCH {
    static constexpr uint16_t ADDRESS = 0x157;
    int32_t POSITION_ACTUAL_LATCH;
  };

  /**
   * @brief PRBS Amplitude Register (MCC_PRBS_AMPLITUDE)
   * @details Block 0, Address: 0x160
   *
   * Set the amplitude of the PRBS (Pseudo-Random Binary Sequence) signal used by some settings of
   * MCC_MOTION_CONFIG -> MOTION_MODE.
   *
   * Register Map:
   * Bits    | Name              | Access | Description
   * --------|-------------------|--------|-------------
   * 31:0    | PRBS_AMPLITUDE    | RW     | Amplitude of the PRBS signal
   *
   * @note This register provides the amplitude of the PRBS signal
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

  /**
   * @brief PRBS Down-Sampling Ratio Register (MCC_PRBS_DOWN_SAMPLING_RATIO)
   * @details Block 0, Address: 0x161
   *
   * Set the downsampling rate of the PWM frequency to trigger new PRBS value generation.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 7:0     | PRBS_DOWN_SAMPLING_RATIO  | RW     | Downsampling rate for PRBS generation
   *
   * @note This register provides the downsampling rate for PRBS generation
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

  /**
   * @brief PID Controller Configuration Register (MCC_PID_CONFIG)
   * @details Block 0, Address: 0x180
   *
   * General configuration for on-chip PI(D) controllers (torque/flux, velocity, position).
   *
   * Register Map:
   * Bits    | Name                | Access | Description
   * --------|---------------------|--------|-------------
   * 0       | KEEP_POS_TARGET     | RW     | Do not overwrite position target on position actual
   * write 2       | CURRENT_NORM_P      | RW     | Normalization of P Factor of Current Control 3
   * | CURRENT_NORM_I      | RW     | Normalization of I Factor of Current Control 5:4     |
   * VELOCITY_NORM_P     | RW     | Normalization of P Factor of Velocity Control 7:6     |
   * VELOCITY_NORM_I     | RW     | Normalization of I Factor of Velocity Control 9:8     |
   * POSITION_NORM_P     | RW     | Normalization of P Factor of Position Control 11:10   |
   * POSITION_NORM_I     | RW     | Normalization of I Factor of Position Control 15:12   |
   * VEL_SCALE           | RW     | Output right shift factor of the velocity controller 22:16   |
   * POS_SMPL            | RW     | Downsampling factor for Position controller 30:24   | VEL_SMPL
   * | RW     | Downsampling factor for Velocity controller
   *
   * @note This register provides the general configuration for the PID controllers
   */
  struct PID_CONFIG {
    static constexpr uint16_t ADDRESS = 0x180;
    union {
      uint32_t value;
      struct {
        uint32_t
            KEEP_POS_TARGET : 1;     ///< Do not overwrite position target on position actual write.
        uint32_t CURRENT_NORM_P : 1; ///< Normalization of P Factor of Current Control.
        uint32_t CURRENT_NORM_I : 1; ///< Normalization of I Factor of Current Control.
        uint32_t VELOCITY_NORM_P : 2; ///< Normalization of P Factor of Velocity Control.
        uint32_t VELOCITY_NORM_I : 2; ///< Normalization of I Factor of Velocity Control.
        uint32_t POSITION_NORM_P : 2; ///< Normalization of P Factor of Position Control.
        uint32_t POSITION_NORM_I : 2; ///< Normalization of I Factor of Position Control.
        uint32_t VEL_SCALE : 4;       ///< Output right shift factor of the velocity controller.
        uint32_t POS_SMPL : 7;        ///< Downsampling factor for Position controller.
        uint32_t VEL_SMPL : 7;        ///< Downsampling factor for Velocity controller.
        uint32_t : 5;
      } bits;
    };
    static constexpr uint32_t RESET_PID_CONFIG = 0x00000800; ///< Default reset value.
  };

  /**
   * @brief Flux PI Controller Coefficients Register (MCC_PID_FLUX_COEFF)
   * @details Block 0, Address: 0x181
   *
   * Configuration of the PI Flux controller gains.
   *
   * Register Map:
   * Bits    | Name      | Access | Description
   * --------|-----------|--------|-------------
   * 31:16   | P         | RW     | Proportional gain for the PI Flux controller (signed)
   * 15:0    | I         | RW     | Integral gain for the PI Flux controller (signed)
   *
   * @note This register provides the gains for the PI Flux controller
   */
  struct PID_FLUX_COEFF {
    static constexpr uint16_t ADDRESS = 0x181;
    union {
      uint32_t value;
      struct {
        int16_t P; ///< Proportional gain for the PI Flux controller.
        int16_t I; ///< Integral gain for the PI Flux controller.
      } bits;
    };
  };

  /**
   * @brief Torque PI Controller Coefficients Register (MCC_PID_TORQUE_COEFF)
   * @details Block 0, Address: 0x182
   *
   * Configuration of the PI Torque controller gains.
   *
   * Register Map:
   * Bits    | Name      | Access | Description
   * --------|-----------|--------|-------------
   * 31:16   | P         | RW     | Proportional gain for the PI Torque controller (signed)
   * 15:0    | I         | RW     | Integral gain for the PI Torque controller (signed)
   *
   * @note This register provides the gains for the PI Torque controller
   */
  struct PID_TORQUE_COEFF {
    static constexpr uint16_t ADDRESS = 0x182;
    union {
      uint32_t value;
      struct {
        int16_t P; ///< Proportional gain for the PI Torque controller.
        int16_t I; ///< Integral gain for the PI Torque controller.
      } bits;
    };
  };

  /**
   * @brief Field Weakening PI Controller Coefficients Register (MCC_PID_FIELDWEAK_COEFF)
   * @details Block 0, Address: 0x183
   *
   * Configuration of the PI Fieldweakening controller gains.
   *
   * Register Map:
   * Bits    | Name      | Access | Description
   * --------|-----------|--------|-------------
   * 31:16   | P         | RW     | Proportional gain for the PI Fieldweakening controller (signed)
   * 15:0    | I         | RW     | Integral gain for the PI Fieldweakening controller (signed)
   *
   * @note This register provides the gains for the PI Fieldweakening controller
   */
  struct PID_FIELDWEAK_COEFF {
    static constexpr uint16_t ADDRESS = 0x183;
    union {
      uint32_t value;
      struct {
        int16_t P; ///< Proportional gain for the PI Fieldweakening controller.
        int16_t I; ///< Integral gain for the PI Fieldweakening controller.
      } bits;
    };
  };

  /**
   * @brief Maximum Voltage (U<sub>S</sub>) Register (MCC_PID_U_S_MAX)
   * @details Block 0, Address: 0x184
   *
   * Maximum voltage allowed for fieldweakening.
   *
   * Register Map:
   * Bits    | Name      | Access | Description
   * --------|-----------|--------|-------------
   * 15:0    | U_S_MAX   | RW     | Maximum voltage allowed for fieldweakening
   *
   * @note This register provides the maximum voltage allowed for fieldweakening
   */
  struct PID_U_S_MAX {
    static constexpr uint16_t ADDRESS = 0x184;
    uint16_t U_S_MAX; ///< Maximum voltage allowed for fieldweakening.
    uint16_t _reserved;
  };

  /**
   * @brief Velocity PI Controller Coefficients Register (MCC_PID_VELOCITY_COEFF)
   * @details Block 0, Address: 0x185
   *
   * Configuration of the PI Velocity controller gains.
   *
   * Register Map:
   * Bits    | Name      | Access | Description
   * --------|-----------|--------|-------------
   * 31:16   | P         | RW     | Proportional gain for the PI Velocity controller (signed)
   * 15:0    | I         | RW     | Integral gain for the PI Velocity controller (signed)
   *
   * @note This register provides the gains for the PI Velocity controller
   */
  struct PID_VELOCITY_COEFF {
    static constexpr uint16_t ADDRESS = 0x185;
    union {
      uint32_t value;
      struct {
        int16_t P; ///< Proportional gain for the PI Velocity controller.
        int16_t I; ///< Integral gain for the PI Velocity controller.
      } bits;
    };
  };

  /**
   * @brief Position PI Controller Coefficients Register (MCC_PID_POSITION_COEFF)
   * @details Block 0, Address: 0x186
   *
   * Configuration of the PI Position controller gains.
   *
   * Register Map:
   * Bits    | Name      | Access | Description
   * --------|-----------|--------|-------------
   * 31:16   | P         | RW     | Proportional gain for the PI Position controller (signed)
   * 15:0    | I         | RW     | Integral gain for the PI Position controller (signed)
   *
   * @note This register provides the gains for the PI Position controller
   */
  struct PID_POSITION_COEFF {
    static constexpr uint16_t ADDRESS = 0x186;
    union {
      uint32_t value;
      struct {
        int16_t P; ///< Proportional gain for the PI Position controller.
        int16_t I; ///< Integral gain for the PI Position controller.
      } bits;
    };
    static constexpr int16_t RESET_P = 0x0000; ///< Default reset value for P.
    static constexpr int16_t RESET_I = 0x0000; ///< Default reset value for I.
  };

  /**
   * @brief Position Tolerance Register (MCC_PID_POSITION_TOLERANCE)
   * @details Block 0, Address: 0x187
   *
   * Position controller ignores position errors smaller than PID_POSITION_TOLERANCE if
   * EVENT_POS_REACHED and after (PID_POSITION_TOLERANCE_DELAY × PWM period).
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 30:0    | PID_POSITION_TOLERANCE    | RW     | Position error tolerance
   *
   * @note This register provides the position error tolerance for the position controller
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

  /**
   * @brief Position Tolerance Delay Register (MCC_PID_POSITION_TOLERANCE_DELAY)
   * @details Block 0, Address: 0x188
   *
   * Number of PWM periods the abs(PID_POSITION_ERROR) must stay within PID_POSITION_TOLERANCE
   * after EVENT_POS_REACHED to disable the controller.
   *
   * Register Map:
   * Bits    | Name                          | Access | Description
   * --------|-------------------------------|--------|-------------
   * 15:0    | PID_POSITION_TOLERANCE_DELAY  | RW     | PWM periods to hold within tolerance
   *
   * @note This register provides the position tolerance delay for the position controller
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

  /**
   * @brief Voltage Limit Register (MCC_PID_UQ_UD_LIMITS)
   * @details Block 0, Address: 0x189
   *
   * Set maximum output voltage limit value.
   *
   * Register Map:
   * Bits    | Name              | Access | Description
   * --------|-------------------|--------|-------------
   * 15:0    | PID_UQ_UD_LIMITS  | RW     | Maximum voltage limit
   *
   * @note This register provides the maximum voltage limit for the PID controller
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

  /**
   * @brief Torque/Flux Current Limit Register (MCC_PID_TORQUE_FLUX_LIMITS)
   * @details Block 0, Address: 0x18A
   *
   * Set maximum target absolute current for torque and flux PI controller.
   *
   * Register Map:
   * Bits    | Name              | Access | Description
   * --------|-------------------|--------|-------------
   * 30:16   | PID_TORQUE_LIMIT  | RW     | Maximum torque current target
   * 14:0    | PID_FLUX_LIMIT    | RW     | Maximum flux current target
   *
   * @note This register provides the maximum current limits for the torque and flux PI controllers
   */
  struct PID_TORQUE_FLUX_LIMITS {
    static constexpr uint16_t ADDRESS = 0x18A;
    union {
      uint32_t value;
      struct {
        uint32_t PID_TORQUE_LIMIT : 15; ///< Maximum torque current target.
        uint32_t : 1;
        uint32_t PID_FLUX_LIMIT : 15; ///< Maximum flux current target.
        uint32_t : 1;
      } bits;
    };
  };

  /**
   * @brief Velocity Limit Register (MCC_PID_VELOCITY_LIMIT)
   * @details Block 0, Address: 0x18B
   *
   * Set maximum absolute velocity for velocity PI controller.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 30:0    | PID_VELOCITY_LIMIT    | RW     | Maximum velocity limit
   *
   * @note This register provides the maximum velocity limit for the velocity PI controller
   */
  struct PID_VELOCITY_LIMIT {
    static constexpr uint16_t ADDRESS = 0x18B;
    uint32_t PID_VELOCITY_LIMIT : 31;
    uint32_t : 1;
  };

  /**
   * @brief Position Limit Low Register (MCC_PID_POSITION_LIMIT_LOW)
   * @details Block 0, Address: 0x18C
   *
   * Set minimum target position for position PI controller.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 31:0    | PID_POSITION_LIMIT_LOW    | RW     | Minimum position limit
   *
   * @note This register provides the minimum position limit for the position PI controller
   */
  struct PID_POSITION_LIMIT_LOW {
    static constexpr uint16_t ADDRESS = 0x18C;
    int32_t PID_POSITION_LIMIT_LOW;
  };

  /**
   * @brief Position Limit High Register (MCC_PID_POSITION_LIMIT_HIGH)
   * @details Block 0, Address: 0x18D
   *
   * Set maximum target position for position PI controller.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 31:0    | PID_POSITION_LIMIT_HIGH   | RW     | Maximum position limit
   *
   * @note This register provides the maximum position limit for the position PI controller
   */
  struct PID_POSITION_LIMIT_HIGH {
    static constexpr uint16_t ADDRESS = 0x18D;
    int32_t PID_POSITION_LIMIT_HIGH;
  };

  /**
   * @brief Torque/Flux Target Register (MCC_PID_TORQUE_FLUX_TARGET)
   * @details Block 0, Address: 0x18E
   *
   * PID target torque and target flux (for torque mode).
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 31:16   | PID_TORQUE_TARGET     | RW     | Target torque
   * 15:0    | PID_FLUX_TARGET       | RW     | Target flux
   *
   * @note This register provides the target torque and flux for the PID controller
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

  /**
   * @brief Torque/Flux Offset Register (MCC_PID_TORQUE_FLUX_OFFSET)
   * @details Block 0, Address: 0x18F
   *
   * PID torque and flux offset.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 31:16   | PID_TORQUE_OFFSET     | RW     | Torque offset for feedforward control
   * 15:0    | PID_FLUX_OFFSET       | RW     | Flux offset for feedforward control
   *
   * @note This register provides the torque and flux offset for the PID controller
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

  /**
   * @brief Velocity Target Register (MCC_PID_VELOCITY_TARGET)
   * @details Block 0, Address: 0x190
   *
   * PID Target velocity (for velocity mode).
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 31:0    | PID_VELOCITY_TARGET   | RW     | Target velocity
   *
   * @note This register provides the target velocity for the PID controller
   */
  struct PID_VELOCITY_TARGET {
    static constexpr uint16_t ADDRESS = 0x190;
    int32_t PID_VELOCITY_TARGET;
  };

  /**
   * @brief Velocity Offset Register (MCC_PID_VELOCITY_OFFSET)
   * @details Block 0, Address: 0x191
   *
   * PID velocity offset for feedforward control.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 31:0    | PID_VELOCITY_OFFSET   | RW     | Velocity offset
   *
   * @note This register provides the velocity offset for the PID controller
   */
  struct PID_VELOCITY_OFFSET {
    static constexpr uint16_t ADDRESS = 0x191;
    int32_t PID_VELOCITY_OFFSET;
  };

  /**
   * @brief Position Target Register (MCC_PID_POSITION_TARGET)
   * @details Block 0, Address: 0x192
   *
   * Target position register (for position mode).
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 31:0    | PID_POSITION_TARGET   | RW     | Target position
   *
   * @note This register provides the target position for the PID controller
   */
  struct PID_POSITION_TARGET {
    static constexpr uint16_t ADDRESS = 0x192;
    int32_t PID_POSITION_TARGET;
  };

  /**
   * @brief Torque/Flux Actual Register (MCC_PID_TORQUE_FLUX_ACTUAL)
   * @details Block 0, Address: 0x193
   *
   * PID actual torque and flux.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 31:16   | PID_TORQUE_ACTUAL     | R      | Actual torque
   * 15:0    | PID_FLUX_ACTUAL       | R      | Actual flux
   *
   * @note This register provides the actual torque and flux for the PID controller
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

  /**
   * @brief Velocity Actual Register (MCC_PID_VELOCITY_ACTUAL)
   * @details Block 0, Address: 0x194
   *
   * PID actual velocity.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 31:0    | PID_VELOCITY_ACTUAL   | R      | Actual velocity
   *
   * @note This register provides the actual velocity for the PID controller
   */
  struct PID_VELOCITY_ACTUAL {
    static constexpr uint16_t ADDRESS = 0x194;
    int32_t PID_VELOCITY_ACTUAL;
  };

  /**
   * @brief Position Actual Register (MCC_PID_POSITION_ACTUAL)
   * @details Block 0, Address: 0x195
   *
   * PID actual position.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 31:0    | PID_POSITION_ACTUAL   | RW     | Actual position
   *
   * @note This register provides the actual position for the PID controller
   */
  struct PID_POSITION_ACTUAL {
    static constexpr uint16_t ADDRESS = 0x195;
    int32_t PID_POSITION_ACTUAL;
  };

  /**
   * @brief Position Actual Offset Register (MCC_PID_POSITION_ACTUAL_OFFSET)
   * @details Block 0, Address: 0x196
   *
   * Offset for actual position.
   *
   * Register Map:
   * Bits    | Name                          | Access | Description
   * --------|-------------------------------|--------|-------------
   * 31:0    | PID_POSITION_ACTUAL_OFFSET    | RW     | Position offset
   *
   * @note This register provides the position offset for the PID controller
   */
  struct PID_POSITION_ACTUAL_OFFSET {
    static constexpr uint16_t ADDRESS = 0x196;
    int32_t PID_POSITION_ACTUAL_OFFSET;
  };

  /**
   * @brief Torque Error Register (MCC_PID_TORQUE_ERROR)
   * @details Block 0, Address: 0x197
   *
   * PID torque error.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 15:0    | PID_TORQUE_ERROR      | R      | Torque error
   *
   * @note This register provides the torque error for the PID controller
   */
  struct PID_TORQUE_ERROR {
    static constexpr uint16_t ADDRESS = 0x197;
    int16_t PID_TORQUE_ERROR;
  };

  /**
   * @brief Flux Error Register (MCC_PID_FLUX_ERROR)
   * @details Block 0, Address: 0x198
   *
   * PID flux error.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 15:0    | PID_FLUX_ERROR        | R      | Flux error
   *
   * @note This register provides the flux error for the PID controller
   */
  struct PID_FLUX_ERROR {
    static constexpr uint16_t ADDRESS = 0x198;
    int16_t PID_FLUX_ERROR;
  };

  /**
   * @brief Velocity Error Register (MCC_PID_VELOCITY_ERROR)
   * @details Block 0, Address: 0x199
   *
   * PID velocity error.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 31:0    | PID_VELOCITY_ERROR    | R      | Velocity error
   *
   * @note This register provides the velocity error for the PID controller
   */
  struct PID_VELOCITY_ERROR {
    static constexpr uint16_t ADDRESS = 0x199;
    int32_t PID_VELOCITY_ERROR;
  };

  /**
   * @brief Position Error Register (MCC_PID_POSITION_ERROR)
   * @details Block 0, Address: 0x19A
   *
   * PID position error.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 31:0    | PID_POSITION_ERROR    | R      | Position error
   *
   * @note This register provides the position error for the PID controller
   */
  struct PID_POSITION_ERROR {
    static constexpr uint16_t ADDRESS = 0x19A;
    int32_t PID_POSITION_ERROR;
  };

  /**
   * @brief Torque Integrator Register (MCC_PID_TORQUE_INTEGRATOR)
   * @details Block 0, Address: 0x19B
   *
   * PID torque integrator.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 31:0    | PID_TORQUE_INTEGRATOR     | RW     | Torque integrator
   *
   * @note This register provides the torque integrator for the PID controller
   */
  struct PID_TORQUE_INTEGRATOR {
    static constexpr uint16_t ADDRESS = 0x19B;
    int32_t PID_TORQUE_INTEGRATOR;
  };

  /**
   * @brief Flux Integrator Register (MCC_PID_FLUX_INTEGRATOR)
   * @details Block 0, Address: 0x19C
   *
   * PID flux integrator.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 31:0    | PID_FLUX_INTEGRATOR   | RW     | Flux integrator
   *
   * @note This register provides the flux integrator for the PID controller
   */
  struct PID_FLUX_INTEGRATOR {
    static constexpr uint16_t ADDRESS = 0x19C;
    int32_t PID_FLUX_INTEGRATOR;
  };

  /**
   * @brief Velocity Integrator Register (MCC_PID_VELOCITY_INTEGRATOR)
   * @details Block 0, Address: 0x19D
   *
   * PID velocity integrator.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 31:0    | PID_VELOCITY_INTEGRATOR   | RW     | Velocity integrator
   *
   * @note This register provides the velocity integrator for the PID controller
   */
  struct PID_VELOCITY_INTEGRATOR {
    static constexpr uint16_t ADDRESS = 0x19D;
    int32_t PID_VELOCITY_INTEGRATOR;
  };

  /**
   * @brief Position Integrator Register (MCC_PID_POSITION_INTEGRATOR)
   * @details Block 0, Address: 0x19E
   *
   * PID position integrator.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 31:0    | PID_POSITION_INTEGRATOR   | RW     | Position integrator
   *
   * @note This register provides the position integrator for the PID controller
   */
  struct PID_POSITION_INTEGRATOR {
    static constexpr uint16_t ADDRESS = 0x19E;
    int32_t PID_POSITION_INTEGRATOR;
  };

  /**
   * @brief PID Input - Torque/Flux Target Register (MCC_PIDIN_TORQUE_FLUX_TARGET)
   * @details Block 0, Address: 0x1A0
   *
   * PID target torque and target flux for readback.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 31:16   | PIDIN_TORQUE_TARGET       | R      | Torque target before filtering/limiting
   * 15:0    | PIDIN_FLUX_TARGET         | R      | Flux target before filtering/limiting
   *
   * @note This register provides the target torque and flux for readback
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

  /**
   * @brief PID Input - Velocity Target Register (MCC_PIDIN_VELOCITY_TARGET)
   * @details Block 0, Address: 0x1A1
   *
   * PID target velocity for readback.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 31:0    | PIDIN_VELOCITY_TARGET     | R      | Velocity target before filtering/limiting
   *
   * @note This register provides the target velocity for readback
   */
  struct PIDIN_VELOCITY_TARGET {
    static constexpr uint16_t ADDRESS = 0x1A1;
    int32_t PIDIN_VELOCITY_TARGET; ///< Velocity target before filtering/limiting.
  };

  /**
   * @brief PID Input - Position Target Register (MCC_PIDIN_POSITION_TARGET)
   * @details Block 0, Address: 0x1A2
   *
   * PID target position for readback.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 31:0    | PIDIN_POSITION_TARGET     | R      | Position target before filtering/limiting
   *
   * @note This register provides the target position for readback
   */
  struct PIDIN_POSITION_TARGET {
    static constexpr uint16_t ADDRESS = 0x1A2;
    int32_t PIDIN_POSITION_TARGET; ///< Position target before filtering/limiting.
  };

  /**
   * @brief PID Input - Limited Torque/Flux Target Register (MCC_PIDIN_TORQUE_FLUX_TARGET_LIMITED)
   * @details Block 0, Address: 0x1A3
   *
   * PID target torque and target flux after PID_TORQUE_FLUX_LIMITS applied.
   *
   * Register Map:
   * Bits    | Name                          | Access | Description
   * --------|-------------------------------|--------|-------------
   * 31:16   | PIDIN_TORQUE_TARGET_LIMITED   | R      | Torque target after limiter
   * 15:0    | PIDIN_FLUX_TARGET_LIMITED     | R      | Flux target after limiter
   *
   * @note This register provides the limited target torque and flux for readback
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

  /**
   * @brief PID Input - Limited Velocity Target Register (MCC_PIDIN_VELOCITY_TARGET_LIMITED)
   * @details Block 0, Address: 0x1A4
   *
   * PID target velocity after PID_VELOCITY_LIMIT applied.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 31:0    | PIDIN_VELOCITY_TARGET_LIMITED | R   | Velocity target after limiter
   *
   * @note This register provides the limited target velocity for readback
   */
  struct PIDIN_VELOCITY_TARGET_LIMITED {
    static constexpr uint16_t ADDRESS = 0x1A4;
    int32_t PIDIN_VELOCITY_TARGET_LIMITED; ///< Velocity target after limiter.
  };

  /**
   * @brief PID Input - Limited Position Target Register (MCC_PIDIN_POSITION_TARGET_LIMITED)
   * @details Block 0, Address: 0x1A5
   *
   * PID target position after PID_POSITION_LIMIT_LOW and PID_POSITION_LIMIT_HIGH applied.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 31:0    | PIDIN_POSITION_TARGET_LIMITED | R   | Position target after limiter
   *
   * @note This register provides the limited target position for readback
   */
  struct PIDIN_POSITION_TARGET_LIMITED {
    static constexpr uint16_t ADDRESS = 0x1A5;
    int32_t PIDIN_POSITION_TARGET_LIMITED; ///< Limited position target.
  };

  /**
   * @brief FOC Interim Result - IALPHA and IBETA Register (MCC_FOC_IBETA_IALPHA)
   * @details Block 0, Address: 0x1A6
   *
   * Interim result of the FOC, IALPHA, and IBETA term.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | IBETA         | R      | Interim result of the FOC, IBETA term
   * 15:0    | IALPHA        | R      | Interim result of the FOC, IALPHA term
   *
   * @note This register provides the interim result of the FOC, IALPHA, and IBETA term
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

  /**
   * @brief FOC Interim Result - ID and IQ Register (MCC_FOC_IQ_ID)
   * @details Block 0, Address: 0x1A7
   *
   * Interim result of the FOC, ID, and IQ term.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | IQ            | R      | Interim result of the FOC, IQ term
   * 15:0    | ID            | R      | Interim result of the FOC, ID term
   *
   * @note This register provides the interim result of the FOC, ID, and IQ term
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

  /**
   * @brief FOC Interim Result - UD and UQ Register (MCC_FOC_UQ_UD)
   * @details Block 0, Address: 0x1A8
   *
   * Interim result of the FOC, UD, and UQ term.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | UQ            | R      | Interim result of the FOC, UQ term
   * 15:0    | UD            | R      | Interim result of the FOC, UD term
   *
   * @note This register provides the interim result of the FOC, UD, and UQ term
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

  /**
   * @brief FOC Interim Result - Limited UD and UQ Register (MCC_FOC_UQ_UD_LIMITED)
   * @details Block 0, Address: 0x1A9
   *
   * Interim result of the FOC, UD, and UQ term after PID_UQ_UD_LIMITS applied.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | UQ            | R      | Interim result of the FOC, UQ term limited
   * 15:0    | UD            | R      | Interim result of the FOC, UD term limited
   *
   * @note This register provides the limited interim result of the FOC, UD, and UQ term
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

  /**
   * @brief FOC Interim Result - UALPHA and UBETA Register (MCC_FOC_UBETA_UALPHA)
   * @details Block 0, Address: 0x1AA
   *
   * Interim result of the FOC, UALPHA, and UBETA term.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | UBETA         | R      | Interim result of the FOC, UBETA term
   * 15:0    | UALPHA        | R      | Interim result of the FOC, UALPHA term
   *
   * @note This register provides the interim result of the FOC, UALPHA, and UBETA term
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

  /**
   * @brief FOC Interim Result - UUX and UWY Register (MCC_FOC_UWY_UUX)
   * @details Block 0, Address: 0x1AB
   *
   * Interim result of the FOC, UUX, and UWY term.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | UWY           | R      | Interim result of the FOC, UWY term
   * 15:0    | UUX           | R      | Interim result of the FOC, UUX term
   *
   * @note This register provides the interim result of the FOC, UUX, and UWY term
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

  /**
   * @brief FOC Interim Result - UV Register (MCC_FOC_UV)
   * @details Block 0, Address: 0x1AC
   *
   * Interim result of the FOC, UV term.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 15:0    | UV            | R      | Interim result of the FOC, UV term
   *
   * @note This register provides the interim result of the FOC, UV term
   */
  struct FOC_UV {
    static constexpr uint16_t ADDRESS = 0x1AC;
    int16_t UV; ///< Interim result of the FOC, UV term.
  };

  /**
   * @brief PWM Duty Cycle - UX1 and VX2 Register (MCC_PWM_VX2_UX1)
   * @details Block 0, Address: 0x1AD
   *
   * Interim result PWM duty cycle UX1 and VX2.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | VX2           | R      | Interim result PWM VX2
   * 15:0    | UX1           | R      | Interim result PWM UX1
   *
   * @note This register provides the interim result PWM duty cycle for UX1 and VX2
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

  /**
   * @brief PWM Duty Cycle - WY1 and Y2 Register (MCC_PWM_Y2_WY1)
   * @details Block 0, Address: 0x1AE
   *
   * Interim result PWM duty cycle WY1 and Y2.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | Y2            | R      | Interim result PWM Y2
   * 15:0    | WY1           | R      | Interim result PWM WY1
   *
   * @note This register provides the interim result PWM duty cycle for WY1 and Y2
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

  /**
   * @brief Velocity (Fixed Frequency) Measurement Register (MCC_VELOCITY_FRQ)
   * @details Block 0, Address: 0x1AF
   *
   * Actual velocity measured by fixed frequency sampling.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:0    | VELOCITY_FRQ  | R      | Actual velocity measured by fixed frequency sampling
   *
   * @note This register provides the actual velocity measured by fixed frequency sampling
   */
  struct VELOCITY_FRQ {
    static constexpr uint16_t ADDRESS = 0x1AF;
    int32_t VELOCITY_FRQ; ///< Actual velocity measured by fixed frequency sampling.
  };

  /**
   * @brief Velocity (Period) Measurement Register (MCC_VELOCITY_PER)
   * @details Block 0, Address: 0x1B0
   *
   * Actual velocity measured using period measurement (time between increments).
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:0    | VELOCITY_PER  | R      | Actual velocity measured by period measurement
   *
   * @note This register provides the actual velocity measured by period measurement
   */
  struct VELOCITY_PER {
    static constexpr uint16_t ADDRESS = 0x1B0;
    int32_t VELOCITY_PER_VAL; ///< Actual velocity measured by period measurement.
  };

  /**
   * @brief Motor Voltage & Current Actual Values Register (MCC_U_S_ACTUAL_I_S_ACTUAL)
   * @details Block 0, Address: 0x1C0
   *
   * Real-time values of the magnitude of voltage (U<sub>S</sub>) and current (I<sub>S</sub>)
   * vectors.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | U_S_ACTUAL    | R      | Actual motor voltage magnitude
   * 15:0    | I_S_ACTUAL    | R      | Actual motor current magnitude
   *
   * @note This register provides the real-time values of the motor voltage and current magnitudes
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

  /**
   * @brief Motor Power (Mechanical) Measurement Register (MCC_P_MOTOR)
   * @details Block 0, Address: 0x1C1
   *
   * Estimated mechanical output power of the motor.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:0    | P_MOTOR       | R      | Actual power applied to the motor
   *
   * @note This register provides the estimated mechanical output power of the motor
   */
  struct P_MOTOR {
    static constexpr uint16_t ADDRESS = 0x1C1;
    uint32_t P_MECH; ///< Actual power applied to the motor.
  };

  /**
   * @brief Raw Digital Inputs Register (MCC_INPUTS_RAW)
   * @details Block 0, Address: 0x1C2
   *
   * Represents the raw input signals for the TMC9660 driver.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 0       | ENC_A         | R      | Encoder signal A directly from pin
   * 1       | ENC_B         | R      | Encoder signal B directly from pin
   * 2       | ENC_N         | R      | Encoder signal N directly from pin
   * 8       | HALL_U        | R      | Hall signal U directly from pin
   * 9       | HALL_V        | R      | Hall signal V directly from pin
   * 10      | HALL_W        | R      | Hall signal W directly from pin
   * 12      | REF_SW_R      | R      | Right reference switch value directly from pin
   * 13      | REF_SW_L      | R      | Left reference switch value directly from pin
   * 14      | REF_SW_H      | R      | Home reference switch value directly from pin
   * 15      | ENI           | R      | DRV_ENABLE pin value
   * 20      | HALL_U_FILT   | R      | Hall signal U after filter and reordering
   * 21      | HALL_V_FILT   | R      | Hall signal V after filter and reordering
   * 22      | HALL_W_FILT   | R      | Hall signal W after filter and reordering
   *
   * @note This register provides the raw input signals for the TMC9660 driver
   */
  struct INPUTS_RAW {
    static constexpr uint16_t ADDRESS = 0x1C2;
    union {
      uint32_t value;
      struct {
        uint32_t ENC_A : 1; ///< Encoder signal A directly from pin.
        uint32_t ENC_B : 1; ///< Encoder signal B directly from pin.
        uint32_t ENC_N : 1; ///< Encoder signal N directly from pin.
        uint32_t : 5;
        uint32_t HALL_U : 1; ///< Hall signal U directly from pin.
        uint32_t HALL_V : 1; ///< Hall signal V directly from pin.
        uint32_t HALL_W : 1; ///< Hall signal W directly from pin.
        uint32_t : 1;
        uint32_t REF_SW_R : 1; ///< Right reference switch value directly from pin.
        uint32_t REF_SW_L : 1; ///< Left reference switch value directly from pin.
        uint32_t REF_SW_H : 1; ///< Home reference switch value directly from pin.
        uint32_t ENI : 1;      ///< DRV_ENABLE pin value.
        uint32_t : 4;
        uint32_t HALL_U_FILT : 1; ///< Hall signal U after filter and reordering.
        uint32_t HALL_V_FILT : 1; ///< Hall signal V after filter and reordering.
        uint32_t HALL_W_FILT : 1; ///< Hall signal W after filter and reordering.
        uint32_t : 9;
      } bits;
    };
  };

  /**
   * @brief Raw Digital Outputs Register (MCC_OUTPUTS_RAW)
   * @details Block 0, Address: 0x1C3
   *
   * Represents the raw output signals for the TMC9660 driver.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 0       | PWM_UX1_L     | R      | Value of PWM phase UX1 low side
   * 1       | PWM_UX1_H     | R      | Value of PWM phase UX1 high side
   * 2       | PWM_VX2_L     | R      | Value of PWM phase VX2 low side
   * 3       | PWM_VX2_H     | R      | Value of PWM phase VX2 high side
   * 4       | PWM_WY1_L     | R      | Value of PWM phase WY1 low side
   * 5       | PWM_WY1_H     | R      | Value of PWM phase WY1 high side
   * 6       | PWM_Y2_L      | R      | Value of PWM phase Y2 low side
   * 7       | PWM_Y2_H      | R      | Value of PWM phase Y2 high side
   *
   * @note This register provides the raw output signals for the TMC9660 driver
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
        uint32_t PWM_Y2_L : 1;  ///< Value of PWM phase Y2 low side.
        uint32_t PWM_Y2_H : 1;  ///< Value of PWM phase Y2 high side.
        uint32_t : 24;
      } bits;
    };
  };

  /**
   * @brief General Status Flags Register (MCC_STATUS_FLAGS)
   * @details Block 0, Address: 0x1C4
   *
   * Represents the general status flags and events for the TMC9660 driver.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 0       | PID_X_TARGET_LIMIT        | R      | PI position controller target value limiter
   * active 1       | PID_X_OUTPUT_LIMIT        | R      | PI position controller output limiter
   * active 2       | PID_V_TARGET_LIMIT        | R      | PI velocity controller target value
   * limiter active 3       | PID_V_OUTPUT_LIMIT        | R      | PI velocity controller output
   * limiter active 4       | PID_ID_TARGET_LIMIT       | R      | PI flux controller target value
   * limiter active 5       | PID_ID_OUTPUT_LIMIT       | R      | PI flux controller output limiter
   * active 6       | PID_IQ_TARGET_LIMIT       | R      | PI torque controller target value limiter
   * active 7       | PID_IQ_OUTPUT_LIMIT       | R      | PI torque controller output limiter
   * active 8       | IPARK_VOLTLIM_LIMIT_U     | R      | Ud or Uq limited by PID_UQ_UD_LIMITS or
   * internal max value 9       | PWM_SWITCH_LIMIT_ACTIVE   | R      | PWM switch limit active 10 |
   * HALL_ERROR                | R      | Hall vector error (000 or 111) 11      |
   * POSITION_TRACKING_ERROR   | R      | PI position controller error exceeds MAX_POS_DEVIATION 12
   * | VELOCITY_TRACKING_ERROR   | R      | PI velocity controller error exceeds MAX_VEL_DEVIATION
   * 13      | PID_FW_OUTPUT_LIMIT       | R      | Field weakening PI controller output limit
   * active 16      | SHORT                     | R      | HS_FAULT or LS_FAULT triggered 20      |
   * REF_SW_L                  | R      | Left reference switch active 21      | REF_SW_R | R      |
   * Right reference switch active 22      | REF_SW_H                  | R      | Home reference
   * switch active 23      | POSITION_REACHED          | R      | Ramper position reached 26      |
   * ADC_I_CLIPPED             | R      | ADC current measurement clipped 28      | ENC_N | R      |
   * Filtered encoder signal N active 31      | ENI                       | R      | Change on
   * DRV_ENABLE pin detected
   *
   * @note This register provides the general status flags and events for the TMC9660 driver
   */
  struct STATUS_FLAGS {
    static constexpr uint16_t ADDRESS = 0x1C4;
    union {
      uint32_t value;
      struct {
        uint32_t PID_X_TARGET_LIMIT : 1;    ///< PI position controller target value limiter active.
        uint32_t PID_X_OUTPUT_LIMIT : 1;    ///< PI position controller output limiter active.
        uint32_t PID_V_TARGET_LIMIT : 1;    ///< PI velocity controller target value limiter active.
        uint32_t PID_V_OUTPUT_LIMIT : 1;    ///< PI velocity controller output limiter active.
        uint32_t PID_ID_TARGET_LIMIT : 1;   ///< PI flux controller target value limiter active.
        uint32_t PID_ID_OUTPUT_LIMIT : 1;   ///< PI flux controller output limiter active.
        uint32_t PID_IQ_TARGET_LIMIT : 1;   ///< PI torque controller target value limiter active.
        uint32_t PID_IQ_OUTPUT_LIMIT : 1;   ///< PI torque controller output limiter active.
        uint32_t IPARK_VOLTLIM_LIMIT_U : 1; ///< Ud or Uq limited by PID_UQ_UD_LIMITS or internal
                                            ///< max value.
        uint32_t PWM_SWITCH_LIMIT_ACTIVE : 1; ///< PWM switch limit active.
        uint32_t HALL_ERROR : 1;              ///< Hall vector error (000 or 111).
        uint32_t POSITION_TRACKING_ERROR : 1; ///< PI position controller error exceeds
                                              ///< MAX_POS_DEVIATION.
        uint32_t VELOCITY_TRACKING_ERROR : 1; ///< PI velocity controller error exceeds
                                              ///< MAX_VEL_DEVIATION.
        uint32_t PID_FW_OUTPUT_LIMIT : 1; ///< Field weakening PI controller output limit active.
        uint32_t : 2;
        uint32_t SHORT : 1; ///< HS_FAULT or LS_FAULT triggered.
        uint32_t : 2;
        uint32_t REF_SW_L : 1;         ///< Left reference switch active.
        uint32_t REF_SW_R : 1;         ///< Right reference switch active.
        uint32_t REF_SW_H : 1;         ///< Home reference switch active.
        uint32_t POSITION_REACHED : 1; ///< Ramper position reached.
        uint32_t : 2;
        uint32_t ADC_I_CLIPPED : 1; ///< ADC current measurement clipped.
        uint32_t : 1;
        uint32_t ENC_N : 1; ///< Filtered encoder signal N active.
        uint32_t : 2;
        uint32_t ENI : 1; ///< Change on DRV_ENABLE pin detected.
      } bits;
    };
  };

  /**
   * @brief Gate Driver Hardware Configuration Register (MCC_HW_CONFIG)
   * @details Block 0, Address: 0x1E3
   *
   * Controls top-level gate driver enable bits for each half-bridge.
   *
   * Register Map:
   * Bits    | Name              | Access | Description
   * --------|-------------------|--------|-------------
   * 0       | BRIDGE_ENABLE_U   | RW     | Enable bridge U (UX1 low-side & corresponding high-side)
   * 1       | BRIDGE_ENABLE_V   | RW     | Enable bridge V (VX2 half-bridge)
   *
   * @note This register provides the top-level gate driver enable bits for each half-bridge
   */
  struct HW_CONFIG {
    static constexpr uint16_t ADDRESS = 0x1E3;
    enum class BridgeEnable : uint8_t {
      Disabled = 0,
      Enabled = 1
    }; ///< Enumerated state for bridge enable bits.
    union {
      uint32_t value;
      struct {
        BridgeEnable BRIDGE_ENABLE_U : 1; ///< Enable Bridge U (phase U half-bridge).
        BridgeEnable BRIDGE_ENABLE_V : 1; ///< Enable Bridge V (phase V half-bridge).
        uint32_t : 30;
      } bits;
    };
  };

  /**
   * @brief Gate Driver Configuration Register (MCC_GDRV_CFG)
   * @details Block 0, Address: 0x1E4
   *
   * Configures gate driver behavior and strengths.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 23:20   | VS_UVLO_LVL           | RW     | Sets the VS undervoltage lockout threshold level
   * 17      | ADAPTIVE_MODE_Y2      | RW     | Enables adaptive gate drive mode for phase Y2
   * 16      | ADAPTIVE_MODE_UVW     | RW     | Enables adaptive gate drive mode for phases U, V, W
   * 15:12   | IGATE_SOURCE_Y2       | RW     | Gate source current limit for high-side Y2 FET
   * 11:8    | IGATE_SINK_Y2         | RW     | Gate sink current limit for low-side Y2 FET
   * 7:4     | IGATE_SOURCE_UVW      | RW     | Gate source current limit for high-side U/V/W FETs
   * 3:0     | IGATE_SINK_UVW        | RW     | Gate sink current limit for low-side U/V/W FETs
   *
   * @note This register provides the configuration for gate driver behavior and strengths
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
        GateSinkCurrent IGATE_SINK_UVW : 4;     ///< Gate sink current setting for UVW.
        GateSourceCurrent IGATE_SOURCE_UVW : 4; ///< Gate source current setting for UVW.
        GateSinkCurrent IGATE_SINK_Y2 : 4;      ///< Gate sink current setting for Y2.
        GateSourceCurrent IGATE_SOURCE_Y2 : 4;  ///< Gate source current setting for Y2.
        uint32_t ADAPTIVE_MODE_UVW : 1;         ///< Adaptive gate discharge for UVW (1=enabled).
        uint32_t ADAPTIVE_MODE_Y2 : 1;          ///< Adaptive gate discharge for Y2 (1=enabled).
        uint32_t : 2;
        VsUvloLevel VS_UVLO_LVL : 4; ///< VS undervoltage lockout threshold setting.
        uint32_t : 12;
      } bits;
    };
  };

  /**
   * @brief Gate Driver Timing Register (MCC_GDRV_TIMING)
   * @details Block 0, Address: 0x1E9
   *
   * Sets the T_DRIVE sink and source times for all channels.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 31:24   | T_DRIVE_SOURCE_Y2     | RW     | Charge time of the MOSFET for Y2
   * 23:16   | T_DRIVE_SINK_Y2       | RW     | Discharge time of the MOSFET for Y2
   * 15:8    | T_DRIVE_SOURCE_UVW    | RW     | Charge time of the MOSFET for UVW
   * 7:0     | T_DRIVE_SINK_UVW      | RW     | Discharge time of the MOSFET for UVW
   *
   * @note This register provides the T_DRIVE sink and source times for all channels
   */
  struct TIMING {
    static constexpr uint16_t ADDRESS = 0x1E9;
    union {
      uint32_t value;
      struct {
        uint8_t T_DRIVE_SINK_UVW;   ///< Low-side drive (sink) time for UVW FETs.
        uint8_t T_DRIVE_SOURCE_UVW; ///< High-side drive (source) time for UVW FETs.
        uint8_t T_DRIVE_SINK_Y2;    ///< Low-side drive time for Y2 FET.
        uint8_t T_DRIVE_SOURCE_Y2;  ///< High-side drive time for Y2 FET.
      } bits;
    };
  };

  /**
   * @brief Gate Driver Blanking and Deadtime Register (MCC_GDRV_BBM)
   * @details Block 0, Address: 0x1EA
   *
   * Controls the BBM_L and BBM_H times for all channels.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 31:24   | BBM_H_Y2              | RW     | Break Before Make time for high-side Y2
   * 23:16   | BBM_L_Y2              | RW     | Break Before Make time for low-side Y2
   * 15:8    | BBM_H_UVW             | RW     | Break Before Make time for high-side UVW
   * 7:0     | BBM_L_UVW             | RW     | Break Before Make time for low-side UVW
   *
   * @note This register provides the BBM_L and BBM_H times for all channels
   */
  struct BBM {
    static constexpr uint16_t ADDRESS = 0x1EA;
    union {
      uint32_t value;
      struct {
        uint8_t BBM_L_UVW; ///< Deadtime (off time) after turning off UVW low-side before high-side
                           ///< can turn on.
        uint8_t BBM_H_UVW; ///< Deadtime for UVW high-side.
        uint8_t BBM_L_Y2;  ///< Deadtime for Y2 low-side.
        uint8_t BBM_H_Y2;  ///< Deadtime for Y2 high-side.
      } bits;
    };
  };

  /**
   * @brief Gate Driver Protection Register (MCC_GDRV_PROT)
   * @details Block 0, Address: 0x1EB
   *
   * Protection settings for MCC_GDRV_PROT (Block 0, Address 0x1EB).
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 28      | TERM_PWM_ON_SHORT     | RW     | Terminate PWM on other phases if a fault occurs
   * 23:22   | HS_RETRIES_Y2         | RW     | High-side Y2 retry count on fault
   * 21:20   | LS_RETRIES_Y2         | RW     | Low-side Y2 retry count on fault
   * 19:18   | HS_RETRIES_UVW        | RW     | High-side UVW retry count on fault
   * 17:16   | LS_RETRIES_UVW        | RW     | Low-side UVW retry count on fault
   * 13:12   | VGS_BLANKING_Y2       | RW     | VGS short blanking time for Y2
   * 10:8    | VGS_DEGLITCH_Y2       | RW     | VGS short deglitch time for Y2
   * 5:4     | VGS_BLANKING_UVW      | RW     | VGS short blanking time for UVW
   * 2:0     | VGS_DEGLITCH_UVW      | RW     | VGS short deglitch time for UVW
   *
   * @note This register provides the protection settings for the gate driver
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
      OFF = 0,  ///< No retries.
      ONE = 1,  ///< 1 retry.
      TWO = 2,  ///< 2 retries.
      THREE = 3 ///< 3 retries.
    };

    /// Enum for VGS blanking time.
    enum class VgsBlanking : uint8_t {
      BLK_OFF = 0,   ///< Off.
      BLK_250NS = 1, ///< 0.25 µs.
      BLK_500NS = 2, ///< 0.5 µs.
      BLK_1000NS = 3 ///< 1 µs.
    };

    /// Enum for VGS deglitch time.
    enum class VgsDeglitch : uint8_t {
      DEG_OFF = 0,    ///< Off.
      DEG_250NS = 1,  ///< 0.25 µs.
      DEG_500NS = 2,  ///< 0.5 µs.
      DEG_1000NS = 3, ///< 1 µs.
      DEG_2000NS = 4, ///< 2 µs.
      DEG_4000NS = 5, ///< 4 µs.
      DEG_6000NS = 6, ///< 6 µs.
      DEG_8000NS = 7  ///< 8 µs.
    };

    union {
      uint32_t value;
      struct {
        VgsDeglitch VGS_DEGLITCH_UVW : 3; ///< VGS short deglitch time for UVW.
        VgsBlanking VGS_BLANKING_UVW : 2; ///< VGS short blanking time for UVW.
        uint32_t : 1;
        VgsDeglitch VGS_DEGLITCH_Y2 : 3; ///< VGS short deglitch time for Y2.
        VgsBlanking VGS_BLANKING_Y2 : 2; ///< VGS short blanking time for Y2.
        uint32_t : 2;
        RetryCount LS_RETRIES_UVW : 2; ///< Low-side UVW retry count on fault.
        RetryCount HS_RETRIES_UVW : 2; ///< High-side UVW retry count.
        RetryCount LS_RETRIES_Y2 : 2;  ///< Low-side Y2 retry count.
        RetryCount HS_RETRIES_Y2 : 2;  ///< High-side Y2 retry count.
        uint32_t : 3;
        TermPwmOnShort TERM_PWM_ON_SHORT : 1; ///< Terminate PWM on other phases if fault occurs.
        uint32_t : 3;
      } bits;
    };
  };

  /**
   * @brief Overcurrent Protection (OCP) UVW Register (MCC_GDRV_OCP_UVW)
   * @details Block 0, Address: 0x1EC
   *
   * Configures the overcurrent protection for phases U, V, and W.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 27:24   | HS_OCP_THRES_UVW      | RW     | Threshold of the high-side overcurrent protection
   * 22:20   | HS_OCP_BLANKING_UVW   | RW     | OCP blanking time for high-side
   * 18:16   | HS_OCP_DEGLITCH_UVW   | RW     | OCP deglitch time for high-side
   * 15      | LS_OCP_USE_VDS_UVW    | RW     | Switches between shunt and RDSon measurement
   * 11:8    | LS_OCP_THRES_UVW      | RW     | Threshold of the low-side overcurrent protection
   * 6:4     | LS_OCP_BLANKING_UVW   | RW     | OCP blanking time for low-side
   * 2:0     | LS_OCP_DEGLITCH_UVW   | RW     | OCP deglitch time for low-side
   *
   * @note This register provides the overcurrent protection settings for phases U, V, and W
   */
  struct OCP_UVW {
    static constexpr uint16_t ADDRESS = 0x1EC;

    /// Enum for OCP thresholds.
    enum class OcpThreshold : uint8_t {
      THRES_63MV = 0,
      THRES_125MV,
      THRES_187MV,
      THRES_248MV,
      THRES_312MV,
      THRES_374MV,
      THRES_434MV,
      THRES_504MV,
      THRES_705MV,
      THRES_940MV,
      THRES_1180MV,
      THRES_1410MV,
      THRES_1650MV,
      THRES_1880MV,
      THRES_2110MV,
      THRES_2350MV
    };

    /// Enum for blanking times.
    enum class BlankingTime : uint8_t {
      BLK_OFF = 0,
      BLK_250NS,
      BLK_500NS,
      BLK_1000NS,
      BLK_2000NS,
      BLK_4000NS,
      BLK_6000NS,
      BLK_8000NS
    };

    /// Enum for deglitch times.
    enum class DeglitchTime : uint8_t {
      DEG_OFF = 0,
      DEG_250NS,
      DEG_500NS,
      DEG_1000NS,
      DEG_2000NS,
      DEG_4000NS,
      DEG_6000NS,
      DEG_8000NS
    };

    union {
      uint32_t value;
      struct {
        DeglitchTime LS_OCP_DEGLITCH_UVW : 3; ///< Low-side OCP deglitch time.
        BlankingTime LS_OCP_BLANKING_UVW : 3; ///< Low-side OCP blanking time.
        uint32_t : 2;
        OcpThreshold LS_OCP_THRES_UVW : 4; ///< Low-side OCP threshold.
        uint32_t : 1;
        uint32_t LS_OCP_USE_VDS_UVW : 1; ///< Use RDSon measurement for low-side OCP.
        uint32_t : 1;
        DeglitchTime HS_OCP_DEGLITCH_UVW : 3; ///< High-side OCP deglitch time.
        BlankingTime HS_OCP_BLANKING_UVW : 3; ///< High-side OCP blanking time.
        uint32_t : 1;
        OcpThreshold HS_OCP_THRES_UVW : 4; ///< High-side OCP threshold.
        uint32_t : 4;
      } bits;
    };
  };

  /**
   * @brief Overcurrent Protection (OCP) Y2 Register (MCC_GDRV_OCP_Y2)
   * @details Block 0, Address: 0x1ED
   *
   * Represents the configuration for Overcurrent Protection (OCP) settings.
   *
   * Register Map:
   * Bits    | Name                  | Access | Description
   * --------|-----------------------|--------|-------------
   * 2:0     | LS_OCP_DEGLITCH_Y2    | RW     | Low-side OCP deglitch time
   * 6:4     | LS_OCP_BLANKING_Y2    | RW     | Low-side OCP blanking time
   * 11:8    | LS_OCP_THRES_Y2       | RW     | Low-side OCP threshold
   * 15      | LS_OCP_USE_VDS_Y2     | RW     | Use RDSon measurement for low-side OCP
   * 18:16   | HS_OCP_DEGLITCH_Y2    | RW     | High-side OCP deglitch time
   * 22:20   | HS_OCP_BLANKING_Y2    | RW     | High-side OCP blanking time
   * 27:24   | HS_OCP_THRES_Y2       | RW     | High-side OCP threshold
   *
   * @note This register provides the overcurrent protection settings for phase Y2
   */
  struct OCP_Y2 {
    static constexpr uint16_t ADDRESS = 0x1ED;

    /// Enum representing the OCP threshold levels in millivolts (mV).
    enum class OcpThreshold : uint8_t {
      THRES_63MV = 0, ///< Threshold: 63 mV
      THRES_125MV,    ///< Threshold: 125 mV
      THRES_187MV,    ///< Threshold: 187 mV
      THRES_248MV,    ///< Threshold: 248 mV
      THRES_312MV,    ///< Threshold: 312 mV
      THRES_374MV,    ///< Threshold: 374 mV
      THRES_434MV,    ///< Threshold: 434 mV
      THRES_504MV,    ///< Threshold: 504 mV
      THRES_705MV,    ///< Threshold: 705 mV
      THRES_940MV,    ///< Threshold: 940 mV
      THRES_1180MV,   ///< Threshold: 1180 mV
      THRES_1410MV,   ///< Threshold: 1410 mV
      THRES_1650MV,   ///< Threshold: 1650 mV
      THRES_1880MV,   ///< Threshold: 1880 mV
      THRES_2110MV,   ///< Threshold: 2110 mV
      THRES_2350MV    ///< Threshold: 2350 mV
    };

    /// Enum representing the blanking times in nanoseconds (ns).
    enum class BlankingTime : uint8_t {
      BLK_OFF = 0, ///< Blanking time: Off
      BLK_250NS,   ///< Blanking time: 250 ns
      BLK_500NS,   ///< Blanking time: 500 ns
      BLK_1000NS,  ///< Blanking time: 1000 ns
      BLK_2000NS,  ///< Blanking time: 2000 ns
      BLK_4000NS,  ///< Blanking time: 4000 ns
      BLK_6000NS,  ///< Blanking time: 6000 ns
      BLK_8000NS   ///< Blanking time: 8000 ns
    };

    /// Enum representing the deglitch times in nanoseconds (ns).
    enum class DeglitchTime : uint8_t {
      DEG_OFF = 0, ///< Deglitch time: Off
      DEG_250NS,   ///< Deglitch time: 250 ns
      DEG_500NS,   ///< Deglitch time: 500 ns
      DEG_1000NS,  ///< Deglitch time: 1000 ns
      DEG_2000NS,  ///< Deglitch time: 2000 ns
      DEG_4000NS,  ///< Deglitch time: 4000 ns
      DEG_6000NS,  ///< Deglitch time: 6000 ns
      DEG_8000NS   ///< Deglitch time: 8000 ns
    };

    union {
      uint32_t value;
      struct {
        DeglitchTime LS_OCP_DEGLITCH_Y2 : 3; ///< Low-side OCP deglitch time.
        BlankingTime LS_OCP_BLANKING_Y2 : 3; ///< Low-side OCP blanking time.
        uint32_t : 2;                        ///< Reserved bits.
        OcpThreshold LS_OCP_THRES_Y2 : 4;    ///< Low-side OCP threshold.
        uint32_t : 1;                        ///< Reserved bit.
        uint32_t LS_OCP_USE_VDS_Y2 : 1;      ///< Use RDSon measurement for low-side OCP.
        uint32_t : 1;                        ///< Reserved bit.
        DeglitchTime HS_OCP_DEGLITCH_Y2 : 3; ///< High-side OCP deglitch time.
        BlankingTime HS_OCP_BLANKING_Y2 : 3; ///< High-side OCP blanking time.
        uint32_t : 1;                        ///< Reserved bit.
        OcpThreshold HS_OCP_THRES_Y2 : 4;    ///< High-side OCP threshold.
        uint32_t : 4;                        ///< Reserved bits.
      } bits;
    };
  };

  /**
   * @brief Protection Enable Register (MCC_GDRV_PROT_EN)
   * @details Block 0, Address: 0x1EE
   *
   * Enables the protection mechanism for various fault events.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 31      | VS_UVLO_PROT              | RW     | Disables the gate driver on VS undervoltage
   * event 29      | VDRV_UVLO_PROT            | RW     | Disables the gate driver on VDRV
   * undervoltage event 27      | HS_VGS_ON_SHORT_PROT_Y2   | RW     | Disables the gate driver if a
   * gate short event occurs while the gate is turned on (Y2 phase) 26      | HS_VGS_ON_SHORT_PROT_W
   * | RW     | Disables the gate driver if a gate short event occurs while the gate is turned on (W
   * phase) 25      | HS_VGS_ON_SHORT_PROT_V    | RW     | Disables the gate driver if a gate short
   * event occurs while the gate is turned on (V phase) 24      | HS_VGS_ON_SHORT_PROT_U    | RW |
   * Disables the gate driver if a gate short event occurs while the gate is turned on (U phase) 23
   * | HS_VGS_OFF_SHORT_PROT_Y2  | RW     | Disables the gate driver if a gate short event occurs
   * while the gate is turned off (Y2 phase) 22      | HS_VGS_OFF_SHORT_PROT_W   | RW     | Disables
   * the gate driver if a gate short event occurs while the gate is turned off (W phase) 21      |
   * HS_VGS_OFF_SHORT_PROT_V   | RW     | Disables the gate driver if a gate short event occurs
   * while the gate is turned off (V phase) 20      | HS_VGS_OFF_SHORT_PROT_U   | RW     | Disables
   * the gate driver if a gate short event occurs while the gate is turned off (U phase) 19      |
   * HS_SHORT_PROT_Y2          | RW     | Disables the channel if an overcurrent event occurs (Y2
   * phase) 18      | HS_SHORT_PROT_W           | RW     | Disables the channel if an overcurrent
   * event occurs (W phase) 17      | HS_SHORT_PROT_V           | RW     | Disables the channel if
   * an overcurrent event occurs (V phase) 16      | HS_SHORT_PROT_U           | RW     | Disables
   * the channel if an overcurrent event occurs (U phase) 15      | BST_UVLO_PROT_Y2          | RW
   * | Disables the affected phase if a bootstrap capacitor undervoltage event occurs (Y2 phase) 14
   * | BST_UVLO_PROT_W           | RW     | Disables the affected phase if a bootstrap capacitor
   * undervoltage event occurs (W phase) 13      | BST_UVLO_PROT_V           | RW     | Disables the
   * affected phase if a bootstrap capacitor undervoltage event occurs (V phase) 12      |
   * BST_UVLO_PROT_U           | RW     | Disables the affected phase if a bootstrap capacitor
   * undervoltage event occurs (U phase) 11      | LS_VGS_ON_SHORT_PROT_Y2   | RW     | Disables the
   * gate driver if a gate short event occurs while the gate is turned on (Y2 phase) 10      |
   * LS_VGS_ON_SHORT_PROT_W    | RW     | Disables the gate driver if a gate short event occurs
   * while the gate is turned on (W phase) 9       | LS_VGS_ON_SHORT_PROT_V    | RW     | Disables
   * the gate driver if a gate short event occurs while the gate is turned on (V phase) 8       |
   * LS_VGS_ON_SHORT_PROT_U    | RW     | Disables the gate driver if a gate short event occurs
   * while the gate is turned on (U phase) 7       | LS_VGS_OFF_SHORT_PROT_Y2  | RW     | Disables
   * the gate driver if a gate short event occurs while the gate is turned off (Y2 phase) 6       |
   * LS_VGS_OFF_SHORT_PROT_W   | RW     | Disables the gate driver if a gate short event occurs
   * while the gate is turned off (W phase) 5       | LS_VGS_OFF_SHORT_PROT_V   | RW     | Disables
   * the gate driver if a gate short event occurs while the gate is turned off (V phase) 4       |
   * LS_VGS_OFF_SHORT_PROT_U   | RW     | Disables the gate driver if a gate short event occurs
   * while the gate is turned off (U phase) 3       | LS_SHORT_PROT_Y2          | RW     | Disables
   * the channel if an overcurrent event occurs (Y2 phase) 2       | LS_SHORT_PROT_W           | RW
   * | Disables the channel if an overcurrent event occurs (W phase) 1       | LS_SHORT_PROT_V | RW
   * | Disables the channel if an overcurrent event occurs (V phase) 0       | LS_SHORT_PROT_U | RW
   * | Disables the channel if an overcurrent event occurs (U phase)
   *
   * @note This register provides the protection enable settings for various fault events
   */
  struct PROT_ENABLE {
    static constexpr uint16_t ADDRESS = 0x1EE;
    union {
      uint32_t value;
      struct {
        uint32_t
            LS_SHORT_PROT_U : 1; ///< Disables the channel if an overcurrent event occurs (U phase).
        uint32_t
            LS_SHORT_PROT_V : 1; ///< Disables the channel if an overcurrent event occurs (V phase).
        uint32_t
            LS_SHORT_PROT_W : 1; ///< Disables the channel if an overcurrent event occurs (W phase).
        uint32_t LS_SHORT_PROT_Y2 : 1; ///< Disables the channel if an overcurrent event occurs (Y2
                                       ///< phase).
        uint32_t LS_VGS_OFF_SHORT_PROT_U : 1;  ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned off (U phase).
        uint32_t LS_VGS_OFF_SHORT_PROT_V : 1;  ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned off (V phase).
        uint32_t LS_VGS_OFF_SHORT_PROT_W : 1;  ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned off (W phase).
        uint32_t LS_VGS_OFF_SHORT_PROT_Y2 : 1; ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned off (Y2 phase).
        uint32_t LS_VGS_ON_SHORT_PROT_U : 1;   ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned on (U phase).
        uint32_t LS_VGS_ON_SHORT_PROT_V : 1;   ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned on (V phase).
        uint32_t LS_VGS_ON_SHORT_PROT_W : 1;   ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned on (W phase).
        uint32_t LS_VGS_ON_SHORT_PROT_Y2 : 1;  ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned on (Y2 phase).
        uint32_t BST_UVLO_PROT_U : 1;  ///< Disables the affected phase if a bootstrap capacitor
                                       ///< undervoltage event occurs (U phase).
        uint32_t BST_UVLO_PROT_V : 1;  ///< Disables the affected phase if a bootstrap capacitor
                                       ///< undervoltage event occurs (V phase).
        uint32_t BST_UVLO_PROT_W : 1;  ///< Disables the affected phase if a bootstrap capacitor
                                       ///< undervoltage event occurs (W phase).
        uint32_t BST_UVLO_PROT_Y2 : 1; ///< Disables the affected phase if a bootstrap capacitor
                                       ///< undervoltage event occurs (Y2 phase).
        uint32_t
            HS_SHORT_PROT_U : 1; ///< Disables the channel if an overcurrent event occurs (U phase).
        uint32_t
            HS_SHORT_PROT_V : 1; ///< Disables the channel if an overcurrent event occurs (V phase).
        uint32_t
            HS_SHORT_PROT_W : 1; ///< Disables the channel if an overcurrent event occurs (W phase).
        uint32_t HS_SHORT_PROT_Y2 : 1; ///< Disables the channel if an overcurrent event occurs (Y2
                                       ///< phase).
        uint32_t HS_VGS_OFF_SHORT_PROT_U : 1;  ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned off (U phase).
        uint32_t HS_VGS_OFF_SHORT_PROT_V : 1;  ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned off (V phase).
        uint32_t HS_VGS_OFF_SHORT_PROT_W : 1;  ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned off (W phase).
        uint32_t HS_VGS_OFF_SHORT_PROT_Y2 : 1; ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned off (Y2 phase).
        uint32_t HS_VGS_ON_SHORT_PROT_U : 1;   ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned on (U phase).
        uint32_t HS_VGS_ON_SHORT_PROT_V : 1;   ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned on (V phase).
        uint32_t HS_VGS_ON_SHORT_PROT_W : 1;   ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned on (W phase).
        uint32_t HS_VGS_ON_SHORT_PROT_Y2 : 1;  ///< Disables the gate driver if a gate short event
                                               ///< occurs while the gate is turned on (Y2 phase).
        uint32_t VDRV_UVLO_PROT : 1; ///< Disables the gate driver on VDRV undervoltage event.
        uint32_t : 1;
        uint32_t VS_UVLO_PROT : 1; ///< Disables the gate driver on VS undervoltage event.
      } bits;
    };
  };

  /**
   * @brief Gate Driver Status Enable Register (MCC_GDRV_STATUS_EN)
   * @details Block 0, Address: 0x1EF
   *
   * Represents the STATUS_INT_ENABLE register configuration for enabling reporting or actions for
   * various fault conditions in a motor driver.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 0       | LS_SHORT_EN_U             | RW     | Enables reporting/action for overcurrent event
   * (U phase) 1       | LS_SHORT_EN_V             | RW     | Enables reporting/action for
   * overcurrent event (V phase) 2       | LS_SHORT_EN_W             | RW     | Enables
   * reporting/action for overcurrent event (W phase) 3       | LS_SHORT_EN_Y2            | RW     |
   * Enables reporting/action for overcurrent event (Y2 phase) 4       | LS_VGS_OFF_SHORT_EN_U     |
   * RW     | Enables reporting/action for gate short event while off (U phase) 5       |
   * LS_VGS_OFF_SHORT_EN_V     | RW     | Enables reporting/action for gate short event while off (V
   * phase) 6       | LS_VGS_OFF_SHORT_EN_W     | RW     | Enables reporting/action for gate short
   * event while off (W phase) 7       | LS_VGS_OFF_SHORT_EN_Y2    | RW     | Enables
   * reporting/action for gate short event while off (Y2 phase) 8       | LS_VGS_ON_SHORT_EN_U | RW
   * | Enables reporting/action for gate short event while on (U phase) 9       |
   * LS_VGS_ON_SHORT_EN_V      | RW     | Enables reporting/action for gate short event while on (V
   * phase) 10      | LS_VGS_ON_SHORT_EN_W      | RW     | Enables reporting/action for gate short
   * event while on (W phase) 11      | LS_VGS_ON_SHORT_EN_Y2     | RW     | Enables
   * reporting/action for gate short event while on (Y2 phase) 12      | BST_UVLO_EN_U             |
   * RW     | Enables reporting/action for bootstrap undervoltage (U phase) 13      | BST_UVLO_EN_V
   * | RW     | Enables reporting/action for bootstrap undervoltage (V phase) 14      |
   * BST_UVLO_EN_W             | RW     | Enables reporting/action for bootstrap undervoltage (W
   * phase) 15      | BST_UVLO_EN_Y2            | RW     | Enables reporting/action for bootstrap
   * undervoltage (Y2 phase) 16      | HS_SHORT_EN_U             | RW     | Enables reporting/action
   * for overcurrent event (U phase) 17      | HS_SHORT_EN_V             | RW     | Enables
   * reporting/action for overcurrent event (V phase) 18      | HS_SHORT_EN_W             | RW     |
   * Enables reporting/action for overcurrent event (W phase) 19      | HS_SHORT_EN_Y2            |
   * RW     | Enables reporting/action for overcurrent event (Y2 phase) 20      |
   * HS_VGS_OFF_SHORT_EN_U     | RW     | Enables reporting/action for gate short event while off (U
   * phase) 21      | HS_VGS_OFF_SHORT_EN_V     | RW     | Enables reporting/action for gate short
   * event while off (V phase) 22      | HS_VGS_OFF_SHORT_EN_W     | RW     | Enables
   * reporting/action for gate short event while off (W phase) 23      | HS_VGS_OFF_SHORT_EN_Y2    |
   * RW     | Enables reporting/action for gate short event while off (Y2 phase) 24      |
   * HS_VGS_ON_SHORT_EN_U      | RW     | Enables reporting/action for gate short event while on (U
   * phase) 25      | HS_VGS_ON_SHORT_EN_V      | RW     | Enables reporting/action for gate short
   * event while on (V phase) 26      | HS_VGS_ON_SHORT_EN_W      | RW     | Enables
   * reporting/action for gate short event while on (W phase) 27      | HS_VGS_ON_SHORT_EN_Y2     |
   * RW     | Enables reporting/action for gate short event while on (Y2 phase) 28      |
   * VDRV_UVLO_EN              | RW     | Enables reporting/action for VDRV undervoltage event 29 |
   * VDRV_UVLWRN_EN            | RW     | Enables reporting/action for VDRV undervoltage warning 30
   * | VS_UVLO_EN                | RW     | Enables reporting/action for VS undervoltage event
   *
   * @note This register provides the status enable settings for various fault conditions
   */
  struct STATUS_INT_ENABLE {
    static constexpr uint16_t ADDRESS = 0x1EF;
    union {
      uint32_t value;
      struct {
        uint32_t LS_SHORT_EN_U : 1;  ///< Enables reporting/action for overcurrent event (U phase).
        uint32_t LS_SHORT_EN_V : 1;  ///< Enables reporting/action for overcurrent event (V phase).
        uint32_t LS_SHORT_EN_W : 1;  ///< Enables reporting/action for overcurrent event (W phase).
        uint32_t LS_SHORT_EN_Y2 : 1; ///< Enables reporting/action for overcurrent event (Y2 phase).
        uint32_t LS_VGS_OFF_SHORT_EN_U : 1; ///< Enables reporting/action for gate short event while
                                            ///< off (U phase).
        uint32_t LS_VGS_OFF_SHORT_EN_V : 1; ///< Enables reporting/action for gate short event while
                                            ///< off (V phase).
        uint32_t LS_VGS_OFF_SHORT_EN_W : 1; ///< Enables reporting/action for gate short event while
                                            ///< off (W phase).
        uint32_t LS_VGS_OFF_SHORT_EN_Y2 : 1; ///< Enables reporting/action for gate short event
                                             ///< while off (Y2 phase).
        uint32_t LS_VGS_ON_SHORT_EN_U : 1;  ///< Enables reporting/action for gate short event while
                                            ///< on (U phase).
        uint32_t LS_VGS_ON_SHORT_EN_V : 1;  ///< Enables reporting/action for gate short event while
                                            ///< on (V phase).
        uint32_t LS_VGS_ON_SHORT_EN_W : 1;  ///< Enables reporting/action for gate short event while
                                            ///< on (W phase).
        uint32_t LS_VGS_ON_SHORT_EN_Y2 : 1; ///< Enables reporting/action for gate short event while
                                            ///< on (Y2 phase).
        uint32_t
            BST_UVLO_EN_U : 1; ///< Enables reporting/action for bootstrap undervoltage (U phase).
        uint32_t
            BST_UVLO_EN_V : 1; ///< Enables reporting/action for bootstrap undervoltage (V phase).
        uint32_t
            BST_UVLO_EN_W : 1; ///< Enables reporting/action for bootstrap undervoltage (W phase).
        uint32_t
            BST_UVLO_EN_Y2 : 1; ///< Enables reporting/action for bootstrap undervoltage (Y2 phase).
        uint32_t HS_SHORT_EN_U : 1;  ///< Enables reporting/action for overcurrent event (U phase).
        uint32_t HS_SHORT_EN_V : 1;  ///< Enables reporting/action for overcurrent event (V phase).
        uint32_t HS_SHORT_EN_W : 1;  ///< Enables reporting/action for overcurrent event (W phase).
        uint32_t HS_SHORT_EN_Y2 : 1; ///< Enables reporting/action for overcurrent event (Y2 phase).
        uint32_t HS_VGS_OFF_SHORT_EN_U : 1; ///< Enables reporting/action for gate short event while
                                            ///< off (U phase).
        uint32_t HS_VGS_OFF_SHORT_EN_V : 1; ///< Enables reporting/action for gate short event while
                                            ///< off (V phase).
        uint32_t HS_VGS_OFF_SHORT_EN_W : 1; ///< Enables reporting/action for gate short event while
                                            ///< off (W phase).
        uint32_t HS_VGS_OFF_SHORT_EN_Y2 : 1; ///< Enables reporting/action for gate short event
                                             ///< while off (Y2 phase).
        uint32_t HS_VGS_ON_SHORT_EN_U : 1;  ///< Enables reporting/action for gate short event while
                                            ///< on (U phase).
        uint32_t HS_VGS_ON_SHORT_EN_V : 1;  ///< Enables reporting/action for gate short event while
                                            ///< on (V phase).
        uint32_t HS_VGS_ON_SHORT_EN_W : 1;  ///< Enables reporting/action for gate short event while
                                            ///< on (W phase).
        uint32_t HS_VGS_ON_SHORT_EN_Y2 : 1; ///< Enables reporting/action for gate short event while
                                            ///< on (Y2 phase).
        uint32_t VDRV_UVLO_EN : 1;   ///< Enables reporting/action for VDRV undervoltage event.
        uint32_t VDRV_UVLWRN_EN : 1; ///< Enables reporting/action for VDRV undervoltage warning.
        uint32_t VS_UVLO_EN : 1;     ///< Enables reporting/action for VS undervoltage event.
      } bits;
    };
  };

  /**
   * @brief Gate Driver Status Register (MCC_GDRV_STATUS)
   * @details Block 0, Address: 0x1F0
   *
   * Represents the status register of the MCC_GDRV_STATUS block.
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 0       | LS_SHORT_U                | R      | Status of the overcurrent protection (low-side,
   * U phase) 1       | LS_SHORT_V                | R      | Status of the overcurrent protection
   * (low-side, V phase) 2       | LS_SHORT_W                | R      | Status of the overcurrent
   * protection (low-side, W phase) 3       | LS_SHORT_Y2               | R      | Status of the
   * overcurrent protection (low-side, Y2 phase) 4       | LS_VGS_OFF_SHORT_U        | R      | Gate
   * short detection while off (low-side, U phase) 5       | LS_VGS_OFF_SHORT_V        | R      |
   * Gate short detection while off (low-side, V phase) 6       | LS_VGS_OFF_SHORT_W        | R |
   * Gate short detection while off (low-side, W phase) 7       | LS_VGS_OFF_SHORT_Y2       | R |
   * Gate short detection while off (low-side, Y2 phase) 8       | LS_VGS_ON_SHORT_U         | R |
   * Gate short detection while on (low-side, U phase) 9       | LS_VGS_ON_SHORT_V         | R |
   * Gate short detection while on (low-side, V phase) 10      | LS_VGS_ON_SHORT_W         | R |
   * Gate short detection while on (low-side, W phase) 11      | LS_VGS_ON_SHORT_Y2        | R |
   * Gate short detection while on (low-side, Y2 phase) 12      | BST_UVLO_U                | R |
   * Undervoltage condition of the bootstrap cap (U phase) 13      | BST_UVLO_V                | R
   * | Undervoltage condition of the bootstrap cap (V phase) 14      | BST_UVLO_W                | R
   * | Undervoltage condition of the bootstrap cap (W phase) 15      | BST_UVLO_Y2               | R
   * | Undervoltage condition of the bootstrap cap (Y2 phase) 16      | HS_SHORT_U                |
   * R      | Status of the overcurrent protection (high-side, U phase) 17      | HS_SHORT_V | R |
   * Status of the overcurrent protection (high-side, V phase) 18      | HS_SHORT_W                |
   * R      | Status of the overcurrent protection (high-side, W phase) 19      | HS_SHORT_Y2 | R |
   * Status of the overcurrent protection (high-side, Y2 phase) 20      | HS_VGS_OFF_SHORT_U | R |
   * Gate short detection while off (high-side, U phase) 21      | HS_VGS_OFF_SHORT_V        | R |
   * Gate short detection while off (high-side, V phase) 22      | HS_VGS_OFF_SHORT_W        | R |
   * Gate short detection while off (high-side, W phase) 23      | HS_VGS_OFF_SHORT_Y2       | R |
   * Gate short detection while off (high-side, Y2 phase) 24      | HS_VGS_ON_SHORT_U         | R |
   * Gate short detection while on (high-side, U phase) 25      | HS_VGS_ON_SHORT_V         | R |
   * Gate short detection while on (high-side, V phase) 26      | HS_VGS_ON_SHORT_W         | R |
   * Gate short detection while on (high-side, W phase) 27      | HS_VGS_ON_SHORT_Y2        | R |
   * Gate short detection while on (high-side, Y2 phase) 28      | VDRV_UVLO                 | R |
   * Undervoltage condition of VDRV (gate drive voltage) 29      | VDRV_UVLWRN               | R |
   * Low voltage warning condition of VDRV (gate drive voltage) 30      | VS_UVLO | R      |
   * Undervoltage condition of VS (supply voltage)
   *
   * @note This register provides the status of various fault events in the system
   */
  struct STATUS {
    static constexpr uint16_t ADDRESS = 0x1F0;
    union {
      uint32_t value;
      struct {
        uint32_t LS_SHORT_U : 1;  ///< Status of the overcurrent protection (low-side, U phase).
        uint32_t LS_SHORT_V : 1;  ///< Status of the overcurrent protection (low-side, V phase).
        uint32_t LS_SHORT_W : 1;  ///< Status of the overcurrent protection (low-side, W phase).
        uint32_t LS_SHORT_Y2 : 1; ///< Status of the overcurrent protection (low-side, Y2 phase).
        uint32_t LS_VGS_OFF_SHORT_U : 1;  ///< Gate short detection while off (low-side, U phase).
        uint32_t LS_VGS_OFF_SHORT_V : 1;  ///< Gate short detection while off (low-side, V phase).
        uint32_t LS_VGS_OFF_SHORT_W : 1;  ///< Gate short detection while off (low-side, W phase).
        uint32_t LS_VGS_OFF_SHORT_Y2 : 1; ///< Gate short detection while off (low-side, Y2 phase).
        uint32_t LS_VGS_ON_SHORT_U : 1;   ///< Gate short detection while on (low-side, U phase).
        uint32_t LS_VGS_ON_SHORT_V : 1;   ///< Gate short detection while on (low-side, V phase).
        uint32_t LS_VGS_ON_SHORT_W : 1;   ///< Gate short detection while on (low-side, W phase).
        uint32_t LS_VGS_ON_SHORT_Y2 : 1;  ///< Gate short detection while on (low-side, Y2 phase).
        uint32_t BST_UVLO_U : 1;  ///< Undervoltage condition of the bootstrap cap (U phase).
        uint32_t BST_UVLO_V : 1;  ///< Undervoltage condition of the bootstrap cap (V phase).
        uint32_t BST_UVLO_W : 1;  ///< Undervoltage condition of the bootstrap cap (W phase).
        uint32_t BST_UVLO_Y2 : 1; ///< Undervoltage condition of the bootstrap cap (Y2 phase).
        uint32_t HS_SHORT_U : 1;  ///< Status of the overcurrent protection (high-side, U phase).
        uint32_t HS_SHORT_V : 1;  ///< Status of the overcurrent protection (high-side, V phase).
        uint32_t HS_SHORT_W : 1;  ///< Status of the overcurrent protection (high-side, W phase).
        uint32_t HS_SHORT_Y2 : 1; ///< Status of the overcurrent protection (high-side, Y2 phase).
        uint32_t HS_VGS_OFF_SHORT_U : 1;  ///< Gate short detection while off (high-side, U phase).
        uint32_t HS_VGS_OFF_SHORT_V : 1;  ///< Gate short detection while off (high-side, V phase).
        uint32_t HS_VGS_OFF_SHORT_W : 1;  ///< Gate short detection while off (high-side, W phase).
        uint32_t HS_VGS_OFF_SHORT_Y2 : 1; ///< Gate short detection while off (high-side, Y2 phase).
        uint32_t HS_VGS_ON_SHORT_U : 1;   ///< Gate short detection while on (high-side, U phase).
        uint32_t HS_VGS_ON_SHORT_V : 1;   ///< Gate short detection while on (high-side, V phase).
        uint32_t HS_VGS_ON_SHORT_W : 1;   ///< Gate short detection while on (high-side, W phase).
        uint32_t HS_VGS_ON_SHORT_Y2 : 1;  ///< Gate short detection while on (high-side, Y2 phase).
        uint32_t VDRV_UVLO : 1;           ///< Undervoltage condition of VDRV (gate drive voltage).
        uint32_t VDRV_UVLWRN : 1; ///< Low voltage warning condition of VDRV (gate drive voltage).
        uint32_t VS_UVLO : 1;     ///< Undervoltage condition of VS (supply voltage).
      } bits;
    };
  };

  /**
   * @brief Gate Driver Fault Register (MCC_GDRV_FAULT)
   * @details Block 0, Address: 0x1F1
   *
   * Represents the MCC_GDRV_FAULT register (0x1F1, Block 0).
   *
   * Register Map:
   * Bits    | Name                      | Access | Description
   * --------|---------------------------|--------|-------------
   * 31      | VS_UVLO_STS               | R      | Undervoltage condition of VS (supply voltage)
   * 30      | VDRV_UVLWRN_STS           | R      | Low voltage warning condition of VDRV (gate
   * drive voltage) 29      | VDRV_UVLO_STS             | R      | Undervoltage condition of VDRV
   * (gate drive voltage) 19      | HS_FAULT_ACTIVE_Y2        | RW     | Set if any fault occurred
   * on phase Y2 that triggered hardware protection 18      | HS_FAULT_ACTIVE_W         | RW     |
   * Set if any fault occurred on phase W that triggered hardware protection 17      |
   * HS_FAULT_ACTIVE_V         | RW     | Set if any fault occurred on phase V that triggered
   * hardware protection 16      | HS_FAULT_ACTIVE_U         | RW     | Set if any fault occurred on
   * phase U that triggered hardware protection 15      | BST_UVLO_STS_Y2           | R      |
   * Undervoltage condition of the bootstrap capacitor on phase Y2 14      | BST_UVLO_STS_W | R |
   * Undervoltage condition of the bootstrap capacitor on phase W 13      | BST_UVLO_STS_V | R |
   * Undervoltage condition of the bootstrap capacitor on phase V 12      | BST_UVLO_STS_U | R |
   * Undervoltage condition of the bootstrap capacitor on phase U 3       | LS_FAULT_ACTIVE_Y2 | RW
   * | Set if any fault occurred on phase Y2 that triggered hardware protection 2       |
   * LS_FAULT_ACTIVE_W         | RW     | Set if any fault occurred on phase W that triggered
   * hardware protection 1       | LS_FAULT_ACTIVE_V         | RW     | Set if any fault occurred on
   * phase V that triggered hardware protection 0       | LS_FAULT_ACTIVE_U         | RW     | Set
   * if any fault occurred on phase U that triggered hardware protection
   *
   * @note This register provides the status of various fault events in the system
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
        uint32_t VDRV_UVLO_STS : 1;   ///< Undervoltage condition of VDRV.
        uint32_t VDRV_UVLWRN_STS : 1; ///< Low voltage warning condition of VDRV.
        uint32_t VS_UVLO_STS : 1;     ///< Undervoltage condition of VS.
      } bits;
    };
  };

  /**
   * @brief External Writable ADC Values Register (MCC_ADC_I1_I0_EXT)
   * @details Block 0, Address: 0x200
   *
   * External writable ADC values for phase currents I1 and I0.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | I1            | RW     | External writable ADC value for phase I1
   * 15:0    | I0            | RW     | External writable ADC value for phase I0
   *
   * @note This register provides the external writable ADC values for phase currents I1 and I0
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

  /**
   * @brief External Current Input Register (MCC_ADC_I2_EXT)
   * @details Block 0, Address: 0x201
   *
   * External writable ADC value for phase I2.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 15:0    | I2            | RW     | External writable ADC value for phase I2
   *
   * @note This register provides the external writable ADC value for phase I2
   */
  struct ADC_I2_EXT {
    static constexpr uint16_t ADDRESS = 0x201;
    union {
      uint32_t value;
      struct {
        int16_t I2 : 16; ///< External writable ADC value for phase I2 (signed).
        uint16_t : 16;
      } bits;
    };
  };

  /**
   * @brief External PWM Duty Cycle Register (MCC_PWM_VX2_UX1_EXT)
   * @details Block 0, Address: 0x202
   *
   * External writable PWM value.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | VX2           | RW     | External writable PWM value for phase VX2
   * 15:0    | UX1           | RW     | External writable PWM value for phase UX1
   *
   * @note This register provides the external writable PWM values for phases VX2 and UX1
   */
  struct PWM_VX2_UX1_EXT {
    static constexpr uint16_t ADDRESS = 0x202;
    union {
      uint32_t value;
      struct {
        uint16_t UX1 : 16; ///< External writable PWM value for phase UX1.
        uint16_t VX2 : 16; ///< External writable PWM value for phase VX2.
      } bits;
    };
  };

  /**
   * @brief External PWM Duty Cycle Register (MCC_PWM_Y2_WY1_EXT)
   * @details Block 0, Address: 0x203
   *
   * External writable PWM value.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | Y2            | RW     | External writable PWM value for phase Y2
   * 15:0    | WY1           | RW     | External writable PWM value for phase WY1
   *
   * @note This register provides the external writable PWM values for phases Y2 and WY1
   */
  struct PWM_Y2_WY1_EXT {
    static constexpr uint16_t ADDRESS = 0x203;
    union {
      uint32_t value;
      struct {
        uint16_t WY1 : 16; ///< External writable PWM value for phase WY1.
        uint16_t Y2 : 16;  ///< External writable PWM value for phase Y2.
      } bits;
    };
  };

  /**
   * @brief External PWM Alternate Channel Register (MCC_PWM_EXT_Y2_ALT)
   * @details Block 0, Address: 0x204
   *
   * External writable PWM compare value for phase Y2_ALT.
   *
   * Register Map:
   * Bits    | Name              | Access | Description
   * --------|-------------------|--------|-------------
   * 15:0    | PWM_EXT_Y2_ALT    | RW     | Duty cycle to be used in independent Y2 mode
   *
   * @note This register provides the external writable PWM compare value for phase Y2_ALT
   */
  struct PWM_EXT_Y2_ALT {
    static constexpr uint16_t ADDRESS = 0x204;
    union {
      uint32_t value;
      struct {
        uint16_t Y2_ALT : 16; ///< Duty cycle for independent Y2 mode.
        uint16_t : 16;
      } bits;
    };
  };

  /**
   * @brief External Voltage Sense Register (MCC_VOLTAGE_EXT)
   * @details Block 0, Address: 0x205
   *
   * External writable parameter for open-loop voltage control mode, useful during system setup.
   *
   * Register Map:
   * Bits    | Name      | Access | Description
   * --------|-----------|--------|-------------
   * 31:16   | UQ        | RW     | U_Q component in Voltage mode
   * 15:0    | UD        | RW     | U_D component in Voltage mode
   *
   * @note This register provides the external writable parameters for open-loop voltage control
   * mode
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

  /**
   * @brief External Angle/Position Input Register (MCC_PHI_EXT)
   * @details Block 0, Address: 0x206
   *
   * Angle phi_e_ext and phi_m_ext for external writing into this register.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:16   | PHI_M_EXT     | RW     | Angle phi_m_ext for external input
   * 15:0    | PHI_E_EXT     | RW     | Angle phi_e_ext for external input
   *
   * @note This register provides the external writable angles phi_e_ext and phi_m_ext
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

  /**
   * @brief External Velocity Input Register (MCC_VELOCITY_EXT)
   * @details Block 0, Address: 0x208
   *
   * Actual velocity for external override.
   *
   * Register Map:
   * Bits    | Name          | Access | Description
   * --------|---------------|--------|-------------
   * 31:0    | VELOCITY_EXT  | RW     | Actual velocity for SW override
   *
   * @note This register provides the external writable velocity for software override
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
