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

/// Chip ID Register (Address 0x000, Block 0).
/**
 * Provides the chip identification number. This is a read-only constant register.
 */
struct CHIP_ID {
    static constexpr uint32_t ADDRESS = 0x000;  ///< Register address (Block 0)
    /// Register value and bitfields.
    union {
        uint32_t value;  ///< Full 32-bit register value.
        struct {
            uint32_t ID : 32;  ///< Chip identifier (should read 0x544D0001 for TMC9660:contentReference[oaicite:0]{index=0}).
        } bits;
    };
};

/// Fault Status Register (Address 0x008, Block 2).
/**
 * Read-only register reflecting general system status flags. Bits are set automatically when corresponding faults occur.
 * Each bit indicates a specific fault condition (undervoltage, short, thermal, etc.). Cleared when the condition resolves or via reset.
 */
struct FAULT_STATUS {
    static constexpr uint8_t ADDRESS = 0x008;  ///< Register address (Block 2)
    /// Individual fault flags (1 = fault active).
    union {
        uint32_t value;
        struct {
            uint32_t LDO2_READY : 1;      ///< LDO2 regulator ready fault (0 = OK, 1 = LDO2 not ready).
            uint32_t LDO1_READY : 1;      ///< LDO1 regulator ready fault.
            uint32_t VCCIO_UVLO : 1;      ///< VCCIO undervoltage lockout (UVLO) fault.
            uint32_t VDDA_UVLO : 1;       ///< VDDA undervoltage lockout fault.
            uint32_t VDD_UVLO : 1;        ///< VDD undervoltage lockout fault.
            uint32_t VSA_UVLO : 1;        ///< VSA undervoltage lockout fault.
            uint32_t CHGP_SHORT : 1;      ///< Charge pump short fault.
            uint32_t CHGP_OK : 1;         ///< Charge pump voltage OK flag (fault if not OK).
            uint32_t LDOEXT2_SHORT : 1;   ///< External LDO2 short fault.
            uint32_t LDOEXT1_SHORT : 1;   ///< External LDO1 short fault.
            uint32_t LDOEXT_TSD : 1;      ///< External LDO thermal shutdown (TSD) fault.
            uint32_t BCK_SHORT : 1;       ///< Buck converter short fault.
            uint32_t BCK_UVLO : 1;        ///< Buck converter undervoltage lockout fault.
            uint32_t : 19;                ///< Reserved bits.
        } bits;
    };
};

/// Latched Fault Flags Register (Address 0x009, Block 2).
/**
 * Read-only register of latched fault flags. Bits mirror FAULT_STATUS but latch on fault occurrence (remain set until explicitly cleared).
 * Each bit is set when the corresponding fault occurs and stays set until cleared by software (e.g., by writing to a clear register or resetting).
 */
struct FAULT_STATUS_LATCHED {
    static constexpr uint8_t ADDRESS = 0x009;  ///< Register address (Block 2)
    union {
        uint32_t value;
        struct {
            uint32_t LDO2_READY_LATCH : 1;    ///< Latched LDO2 ready fault flag.
            uint32_t LDO1_READY_LATCH : 1;    ///< Latched LDO1 ready fault flag.
            uint32_t VCCIO_UVLO_LATCH : 1;    ///< Latched VCCIO UVLO fault.
            uint32_t VDDA_UVLO_LATCH : 1;     ///< Latched VDDA UVLO fault.
            uint32_t VDD_UVLO_LATCH : 1;      ///< Latched VDD UVLO fault.
            uint32_t VSA_UVLO_LATCH : 1;      ///< Latched VSA UVLO fault.
            uint32_t CHGP_SHORT_LATCH : 1;    ///< Latched charge pump short fault.
            uint32_t CHGP_OK_LATCH : 1;       ///< Latched charge pump OK status (fault if not OK).
            uint32_t LDOEXT2_SHORT_LATCH : 1; ///< Latched external LDO2 short fault.
            uint32_t LDOEXT1_SHORT_LATCH : 1; ///< Latched external LDO1 short fault.
            uint32_t LDOEXT_TSD_LATCH : 1;    ///< Latched external LDO thermal shutdown fault.
            uint32_t BCK_SHORT_LATCH : 1;     ///< Latched buck converter short fault.
            uint32_t BCK_UVLO_LATCH : 1;      ///< Latched buck converter UVLO fault.
            uint32_t : 19;
        } bits;
    };
};

/// Fault Enable Mask Register (Address 0x00A, Block 2).
/**
 * Read/Write register to mask (disable) or enable specific fault interrupts. Each bit corresponds to a fault flag in FAULT_STATUS_LATCHED.
 * When a bit is 0, the corresponding fault will not trigger an interrupt; when 1, the fault will trigger an interrupt (if such mechanism exists).
 * Default 0x000 (all fault interrupts disabled by default):contentReference[oaicite:1]{index=1}:contentReference[oaicite:2]{index=2}.
 */
struct FAULT_INT_ENABLE {
    static constexpr uint8_t ADDRESS = 0x00A;  ///< Register address (Block 2)
    union {
        uint32_t value;
        struct {
            uint32_t LDO2_READY_INT_EN : 1;    ///< Enable interrupt for LDO2 ready fault.
            uint32_t LDO1_READY_INT_EN : 1;    ///< Enable interrupt for LDO1 ready fault.
            uint32_t VCCIO_UVLO_INT_EN : 1;    ///< Enable interrupt for VCCIO UVLO fault.
            uint32_t VDDA_UVLO_INT_EN : 1;     ///< Enable interrupt for VDDA UVLO fault.
            uint32_t VDD_UVLO_INT_EN : 1;      ///< Enable interrupt for VDD UVLO fault.
            uint32_t VSA_UVLO_INT_EN : 1;      ///< Enable interrupt for VSA UVLO fault.
            uint32_t CHGP_SHORT_INT_EN : 1;    ///< Enable interrupt for charge pump short fault.
            uint32_t CHGP_OK_INT_EN : 1;       ///< Enable interrupt for charge pump OK status (loss of CP voltage).
            uint32_t LDOEXT2_SHORT_INT_EN : 1; ///< Enable interrupt for external LDO2 short fault.
            uint32_t LDOEXT1_SHORT_INT_EN : 1; ///< Enable interrupt for external LDO1 short fault.
            uint32_t LDOEXT_TSD_INT_EN : 1;    ///< Enable interrupt for external LDO thermal shutdown.
            uint32_t BCK_SHORT_INT_EN : 1;     ///< Enable interrupt for buck converter short fault.
            uint32_t BCK_UVLO_INT_EN : 1;      ///< Enable interrupt for buck converter UVLO fault.
            uint32_t : 19;
        } bits;
    };
};

} // namespace SYS_CTRL
} // namespace TMC9660
