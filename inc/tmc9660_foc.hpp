#pragma once
#include <cstdint>

/**
 * @file tmc9660_foc.hpp
 * @brief TMC9660 Field-Oriented Control (FOC) internal registers (Register Mode).
 *
 * Contains definitions for registers holding FOC algorithm intermediate results (currents and voltages in dq and alpha-beta frames).
 */

namespace TMC9660 {
namespace FOC {

/// FOC Current (Alpha/Beta Frame) Register (Address 0x1A6, Block 0).
/**
 * Interim result of the FOC current transformation in the alpha-beta coordinate system:contentReference[oaicite:28]{index=28}:contentReference[oaicite:29]{index=29}.
 * - **I_BETA**: Beta-axis current (bits 31:16, signed).
 * - **I_ALPHA**: Alpha-axis current (bits 15:0, signed).
 */
struct I_BETA_I_ALPHA {
    static constexpr uint16_t ADDRESS = 0x1A6;
    union {
        uint32_t value;
        struct {
            int16_t I_BETA  : 16;  ///< FOC I<sub>β</sub> current (Beta component):contentReference[oaicite:30]{index=30}.
            int16_t I_ALPHA : 16;  ///< FOC I<sub>α</sub> current (Alpha component):contentReference[oaicite:31]{index=31}.
        } bits;
    };
};

/// FOC Current (DQ Frame) Register (Address 0x1A7, Block 0).
/**
 * Interim result of the FOC current in the D-Q frame (torque and flux components):contentReference[oaicite:32]{index=32}:contentReference[oaicite:33]{index=33}.
 * - **I_Q**: Q-axis (torque-producing) current (bits 31:16, signed).
 * - **I_D**: D-axis (flux-producing) current (bits 15:0, signed).
 */
struct I_Q_I_D {
    static constexpr uint16_t ADDRESS = 0x1A7;
    union {
        uint32_t value;
        struct {
            int16_t I_Q : 16;  ///< FOC I<sub>q</sub> current (torque component):contentReference[oaicite:34]{index=34}.
            int16_t I_D : 16;  ///< FOC I<sub>d</sub> current (flux component):contentReference[oaicite:35]{index=35}.
        } bits;
    };
};

/// FOC Voltage (DQ Frame) Register (Address 0x1A8, Block 0).
/**
 * Interim result of the FOC output voltages in the D-Q frame:contentReference[oaicite:36]{index=36}:contentReference[oaicite:37]{index=37}.
 * - **U_Q**: Q-axis voltage output (bits 31:16, signed).
 * - **U_D**: D-axis voltage output (bits 15:0, signed).
 */
struct U_Q_U_D {
    static constexpr uint16_t ADDRESS = 0x1A8;
    union {
        uint32_t value;
        struct {
            int16_t U_Q : 16;  ///< FOC U<sub>q</sub> voltage (Q-axis):contentReference[oaicite:38]{index=38}.
            int16_t U_D : 16;  ///< FOC U<sub>d</sub> voltage (D-axis):contentReference[oaicite:39]{index=39}.
        } bits;
    };
};

/// FOC Voltage (DQ Frame) Limited Register (Address 0x1A9, Block 0).
/**
 * FOC output voltages (D-Q frame) after limiting (e.g., saturation due to maximum duty cycle).
 * - **U_Q_LIMITED**: Limited Q-axis voltage.
 * - **U_D_LIMITED**: Limited D-axis voltage.
 */
struct U_Q_U_D_LIMITED {
    static constexpr uint16_t ADDRESS = 0x1A9;
    union {
        uint32_t value;
        struct {
            int16_t U_Q_LIMITED : 16;  ///< Limited U<sub>q</sub> voltage.
            int16_t U_D_LIMITED : 16;  ///< Limited U<sub>d</sub> voltage.
        } bits;
    };
};

/// FOC Voltage (Alpha/Beta Frame) Register (Address 0x1AA, Block 0).
/**
 * Interim result of the FOC output voltages in the alpha-beta frame.
 * - **U_BETA**: Beta-axis voltage (bits 31:16, signed).
 * - **U_ALPHA**: Alpha-axis voltage (bits 15:0, signed).
 */
struct U_BETA_U_ALPHA {
    static constexpr uint16_t ADDRESS = 0x1AA;
    union {
        uint32_t value;
        struct {
            int16_t U_BETA  : 16;  ///< FOC U<sub>β</sub> voltage (Beta component).
            int16_t U_ALPHA : 16;  ///< FOC U<sub>α</sub> voltage (Alpha component).
        } bits;
    };
};

/// FOC Voltage (UVW Frame) Register (Address 0x1AB, Block 0).
/**
 * Interim result of the FOC output voltages in the three-phase (UVW) frame prior to PWM mapping:contentReference[oaicite:40]{index=40}:contentReference[oaicite:41]{index=41}.
 * - **U_WY**: Combined voltage for W-Y half-bridge (bits 31:16, signed).
 * - **U_UX**: Combined voltage for U-X half-bridge (bits 15:0, signed).
 */
struct U_WY_U_UX {
    static constexpr uint16_t ADDRESS = 0x1AB;
    union {
        uint32_t value;
        struct {
            int16_t U_WY : 16;  ///< FOC combined voltage for W/Y phases:contentReference[oaicite:42]{index=42}.
            int16_t U_UX : 16;  ///< FOC combined voltage for U/X phases:contentReference[oaicite:43]{index=43}.
        } bits;
    };
};

/// FOC Voltage Single (Address 0x1AC, Block 0).
/**
 * Interim result of the FOC output voltage for a single component (e.g., the neutral or combined UV value):contentReference[oaicite:44]{index=44}:contentReference[oaicite:45]{index=45}.
 * - **U_V**: A single voltage component (bits 15:0, signed).
 * (Bits 31:16 are not used in this register.)
 */
struct U_V_SINGLE {
    static constexpr uint16_t ADDRESS = 0x1AC;
    union {
        uint32_t value;
        struct {
            int16_t : 16;
            int16_t U_V : 16;  ///< FOC UV combined voltage term (or other single-axis voltage):contentReference[oaicite:46]{index=46}.
        } bits;
    };
};

} // namespace FOC
} // namespace TMC9660
