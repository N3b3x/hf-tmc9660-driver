#pragma once
#include <cstdint>

/**
 * @file tmc9660_sys_ctrl.hpp
 * @brief TMC9660 System Control and Status registers (Register Mode).
 *
 * Contains definitions for system-level registers such as Chip ID and fault status.
 */

namespace TMC9660 {
namespace SYS_CTRL {

/**
 * @brief Fault Status Register (FAULT_STATUS)
 * @details Block 2, Address: 0x008
 *
 * Reflects general system status flags for power and fault monitoring.
 * Each bit indicates a specific fault or status condition. Bits are set and cleared automatically by hardware.
 *
 * Register Map:
 * Bits    | Name             | Access | Reset | Description
 * --------|------------------|--------|-------|----------------------------------------------------------
 * 12      | LDO2_READY       | R      | 0x0   | LDO2 (V_EXT2) has completed soft-start (1 = ready)
 * 11      | LDO1_READY       | R      | 0x0   | LDO1 (V_EXT1) has completed soft-start (1 = ready)
 * 10      | VCCIO_UVLO       | R      | 0x0   | VCCIO undervoltage lockout detected
 * 9       | VDDA_UVLO        | R      | 0x0   | VDDA undervoltage lockout detected
 * 8       | VDD_UVLO         | R      | 0x0   | VDD undervoltage lockout detected
 * 7       | VSA_UVLO         | R      | 0x0   | VSA undervoltage lockout detected
 * 6       | CHGP_SHORT       | R      | 0x0   | VDRV charge pump short status
 * 5       | CHGP_OK          | R      | 0x0   | VDRV charge pump power-up is currently okay (1 = OK)
 * 4       | LDOEXT2_SHORT    | R      | 0x0   | LDO2 (V_EXT2) is shorted
 * 3       | LDOEXT1_SHORT    | R      | 0x0   | LDO1 (V_EXT1) is shorted
 * 2       | LDOEXT_TSD       | R      | 0x0   | LDO thermal shutdown status
 * 1       | BCK_SHORT        | R      | 0x0   | VBUCK shorted
 * 0       | BCK_UVLO         | R      | 0x0   | VBUCK undervoltage lockout detected
 *
 * @note All bits are read-only and reflect real-time hardware status.
 */
struct FAULT_STATUS {
    static constexpr uint8_t ADDRESS = 0x008;  ///< Register address (Block 2)
    union {
        uint32_t value;
        struct {
            uint32_t BCK_UVLO        : 1; ///< [0]  VBUCK undervoltage lockout
            uint32_t BCK_SHORT       : 1; ///< [1]  VBUCK shorted
            uint32_t LDOEXT_TSD      : 1; ///< [2]  LDO thermal shutdown
            uint32_t LDOEXT1_SHORT   : 1; ///< [3]  LDO1 (V_EXT1) shorted
            uint32_t LDOEXT2_SHORT   : 1; ///< [4]  LDO2 (V_EXT2) shorted
            uint32_t CHGP_OK         : 1; ///< [5]  VDRV charge pump power-up OK (1 = OK)
            uint32_t CHGP_SHORT      : 1; ///< [6]  VDRV charge pump short status
            uint32_t VSA_UVLO        : 1; ///< [7]  VSA undervoltage lockout
            uint32_t VDD_UVLO        : 1; ///< [8]  VDD undervoltage lockout
            uint32_t VDDA_UVLO       : 1; ///< [9]  VDDA undervoltage lockout
            uint32_t VCCIO_UVLO      : 1; ///< [10] VCCIO undervoltage lockout
            uint32_t LDO1_READY      : 1; ///< [11] LDO1 (V_EXT1) soft-start complete (1 = ready)
            uint32_t LDO2_READY      : 1; ///< [12] LDO2 (V_EXT2) soft-start complete (1 = ready)
            uint32_t                 : 19;///< Reserved
        } bits;
    };
};

/**
 * @brief Latched Fault Flags Register (FAULT_STATUS_LATCHED / FAULT_R_INT)
 * @details Block 2, Address: 0x009
 *
 * Read/Write register of latched fault flags. Bits are set automatically if the corresponding bit in FAULT_STATUS is set.
 * Each bit must be cleared manually by writing 1 (W1C: Write 1 to Clear). Bits remain set until explicitly cleared by software.
 *
 * Register Map:
 * Bits    | Name               | Access   | Reset | Description
 * --------|--------------------|----------|-------|----------------------------------------------------------
 * 15      | UC_FAULT           | RW       | 0x0   | Force fault pin assertion.
 * 12      | LDO2_READY_LTC     | RW, W1C  | 0x0   | LDO2READY latched bit. Write 1 to clear.
 * 11      | LDO1_READY_LTC     | RW, W1C  | 0x0   | LDO1READY latched bit. Write 1 to clear.
 * 10      | VCCIO_UVLO_LTC     | RW, W1C  | 0x0   | VCCIO_UVLO latched bit. Write 1 to clear.
 * 9       | VDDA_UVLO_LTC      | RW, W1C  | 0x0   | VDDA_UVLO latched bit. Write 1 to clear.
 * 8       | VDD_UVLO_LTC       | RW, W1C  | 0x0   | VDD_UVLO latched bit. Write 1 to clear.
 * 7       | VSA_UVLO_LTC       | RW, W1C  | 0x0   | VSA_UVLO latched bit. Write 1 to clear.
 * 6       | CHGP_SHORT_LTC     | RW, W1C  | 0x0   | CHGP_SHORT latched bit. Write 1 to clear.
 * 5       | CHGP_OK_LTC        | RW, W1C  | 0x0   | CHGP_OK latched bit. Write 1 to clear.
 * 4       | LDOEXT2_SHORT_LTC  | RW, W1C  | 0x0   | LDO2EXT_SHORT latched bit. Write 1 to clear.
 * 3       | LDOEXT1_SHORT_LTC  | RW, W1C  | 0x0   | LDO1EXT_SHORT latched bit. Write 1 to clear.
 * 2       | LDOEXT_TSD_LTC     | RW, W1C  | 0x0   | LDOEXT_TSD latched bit. Write 1 to clear.
 * 1       | BCK_SHORT_LTC      | RW, W1C  | 0x0   | BUCK_SHORT latched bit. Write 1 to clear.
 * 0       | BCK_UVLO_LTC       | RW, W1C  | 0x0   | BUCK_UVLO latched bit. Write 1 to clear.
 *
 * @note All bits are set by hardware and must be cleared by writing 1. UC_FAULT can be set/cleared by software.
 */
struct FAULT_STATUS_LATCHED {
    static constexpr uint8_t ADDRESS = 0x009;  ///< Register address (Block 2)
    union {
        uint32_t value;
        struct {
            uint32_t BCK_UVLO_LTC       : 1;  ///< [0]  BUCK_UVLO latched bit (W1C)
            uint32_t BCK_SHORT_LTC      : 1;  ///< [1]  BUCK_SHORT latched bit (W1C)
            uint32_t LDOEXT_TSD_LTC     : 1;  ///< [2]  LDOEXT_TSD latched bit (W1C)
            uint32_t LDOEXT1_SHORT_LTC  : 1;  ///< [3]  LDO1EXT_SHORT latched bit (W1C)
            uint32_t LDOEXT2_SHORT_LTC  : 1;  ///< [4]  LDO2EXT_SHORT latched bit (W1C)
            uint32_t CHGP_OK_LTC        : 1;  ///< [5]  CHGP_OK latched bit (W1C)
            uint32_t CHGP_SHORT_LTC     : 1;  ///< [6]  CHGP_SHORT latched bit (W1C)
            uint32_t VSA_UVLO_LTC       : 1;  ///< [7]  VSA_UVLO latched bit (W1C)
            uint32_t VDD_UVLO_LTC       : 1;  ///< [8]  VDD_UVLO latched bit (W1C)
            uint32_t VDDA_UVLO_LTC      : 1;  ///< [9]  VDDA_UVLO latched bit (W1C)
            uint32_t VCCIO_UVLO_LTC     : 1;  ///< [10] VCCIO_UVLO latched bit (W1C)
            uint32_t LDO1_READY_LTC     : 1;  ///< [11] LDO1READY latched bit (W1C)
            uint32_t LDO2_READY_LTC     : 1;  ///< [12] LDO2READY latched bit (W1C)
            uint32_t                    : 2;  ///< [13:14] Reserved
            uint32_t UC_FAULT           : 1;  ///< [15] Force fault pin assertion (RW)
            uint32_t                    : 16; ///< [16:31] Reserved
        } bits;
    };
};

/**
 * @brief Fault Interrupt Enable Mask Register (FAULT_R_ENA_F)
 * @details Block 2, Address: 0x00A
 *
 * Mask register for FAULT_STATUS_LATCHED (FAULT_R_INT). If a bit in this register is set and the corresponding
 * latched fault flag is set, the FAULTN pin will be asserted. Each bit enables or disables the fault pin response
 * for the corresponding latched fault. Default reset values are shown below.
 *
 * Register Map:
 * Bits    | Name                   | Access | Reset | Description
 * --------|------------------------|--------|-------|----------------------------------------------------------
 * 12      | LDO2_READY_ENA_F       | RW     | 0x0   | LDO2_READY_LTC mask bit for fault pin.
 * 11      | LDO1_READY_ENA_F       | RW     | 0x0   | LDO1_READY_LTC mask bit for fault pin.
 * 10      | VCCIO_UVLO_ENA_F       | RW     | 0x1   | VCCIO_UVLO_LTC mask bit for fault pin.
 * 9       | VDDA_UVLO_ENA_F        | RW     | 0x1   | VDDA_UVLO_LTC mask bit for fault pin.
 * 8       | VDD_UVLO_ENA_F         | RW     | 0x1   | VDD_UVLO_LTC mask bit for fault pin.
 * 7       | VSA_UVLO_ENA_F         | RW     | 0x1   | VSA_UVLO_LTC mask bit for fault pin.
 * 6       | CHGP_SHORT_ENA_F       | RW     | 0x0   | CHGP_SHORT_LTC mask bit for fault pin.
 * 5       | CHGP_OK_ENA_F          | RW     | 0x0   | CHGP_OK_LTC mask bit for fault pin.
 * 4       | LDOEXT2_SHORT_ENA_F    | RW     | 0x0   | LDO2EXT_SHORT_LTC mask bit for fault pin.
 * 3       | LDOEXT1_SHORT_ENA_F    | RW     | 0x0   | LDO1EXT_SHORT_LTC mask bit for fault pin.
 * 2       | LDOEXT_TSD_ENA_F       | RW     | 0x0   | LDOEXT_LTC thermal shutdown mask bit for fault pin.
 * 1       | BCK_SHORT_ENA_F        | RW     | 0x1   | BUCK_SHORT mask bit for fault pin.
 * 0       | BCK_UVLO_ENA_F         | RW     | 0x1   | BUCK_UVLO mask bit for fault pin.
 *
 * @note Setting a bit to 1 enables the FAULTN pin assertion for the corresponding latched fault.
 */
struct FAULT_INT_ENABLE {
    static constexpr uint8_t ADDRESS = 0x00A;  ///< Register address (Block 2)
    union {
        uint32_t value;
        struct {
            uint32_t BCK_UVLO_ENA_F         : 1;  ///< [0]  BUCK_UVLO mask bit for fault pin (reset 1)
            uint32_t BCK_SHORT_ENA_F        : 1;  ///< [1]  BUCK_SHORT mask bit for fault pin (reset 1)
            uint32_t LDOEXT_TSD_ENA_F       : 1;  ///< [2]  LDOEXT_LTC thermal shutdown mask bit (reset 0)
            uint32_t LDOEXT1_SHORT_ENA_F    : 1;  ///< [3]  LDO1EXT_SHORT_LTC mask bit (reset 0)
            uint32_t LDOEXT2_SHORT_ENA_F    : 1;  ///< [4]  LDO2EXT_SHORT_LTC mask bit (reset 0)
            uint32_t CHGP_OK_ENA_F          : 1;  ///< [5]  CHGP_OK_LTC mask bit (reset 0)
            uint32_t CHGP_SHORT_ENA_F       : 1;  ///< [6]  CHGP_SHORT_LTC mask bit (reset 0)
            uint32_t VSA_UVLO_ENA_F         : 1;  ///< [7]  VSA_UVLO_LTC mask bit (reset 1)
            uint32_t VDD_UVLO_ENA_F         : 1;  ///< [8]  VDD_UVLO_LTC mask bit (reset 1)
            uint32_t VDDA_UVLO_ENA_F        : 1;  ///< [9]  VDDA_UVLO_LTC mask bit (reset 1)
            uint32_t VCCIO_UVLO_ENA_F       : 1;  ///< [10] VCCIO_UVLO_LTC mask bit (reset 1)
            uint32_t LDO1_READY_ENA_F       : 1;  ///< [11] LDO1_READY_LTC mask bit (reset 0)
            uint32_t LDO2_READY_ENA_F       : 1;  ///< [12] LDO2_READY_LTC mask bit (reset 0)
            uint32_t                        : 19; ///< [13:31] Reserved
        } bits;
    };
};

} // namespace SYS_CTRL
} // namespace TMC9660
