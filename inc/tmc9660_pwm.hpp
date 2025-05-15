#pragma once
#include <cstdint>

/**
 * @file tmc9660_pwm.hpp
 * @brief TMC9660 PWM configuration registers (Register Mode).
 *
 * Contains definitions for PWM generation settings and related registers.
 */

namespace TMC9660 {
namespace PWM {



/// Internal PWM Duty Result (Address 0x1AD, Block 0).
/**
 * Interim result for PWM duty cycles of half-bridges UX1 and VX2:contentReference[oaicite:22]{index=22}:contentReference[oaicite:23]{index=23}.
 * This register provides read-only values of the PWM compare registers as determined by the control loop.
 * - **VX2**: PWM duty for phase V high-side (bits 31:16, unsigned).
 * - **UX1**: PWM duty for phase U low-side (bits 15:0, unsigned).
 */
struct DUTY_VX2_UX1 {
    static constexpr uint16_t ADDRESS = 0x1AD;
    union {
        uint32_t value;
        struct {
            uint16_t VX2 : 16;  ///< PWM duty cycle for bridge VX2 (as a count out of MAXCNT):contentReference[oaicite:24]{index=24}.
            uint16_t UX1 : 16;  ///< PWM duty cycle for bridge UX1:contentReference[oaicite:25]{index=25}.
        } bits;
    };
};

/// Internal PWM Duty Result (Address 0x1AE, Block 0).
/**
 * Interim result for PWM duty cycles of half-bridges WY1 and Y2:contentReference[oaicite:26]{index=26}:contentReference[oaicite:27]{index=27}.
 * - **Y2**: PWM duty for phase Y high-side (bits 31:16, unsigned).
 * - **WY1**: PWM duty for phase W low-side (bits 15:0, unsigned).
 */
struct DUTY_Y2_WY1 {
    static constexpr uint16_t ADDRESS = 0x1AE;
    union {
        uint32_t value;
        struct {
            uint16_t Y2  : 16;  ///< PWM duty cycle for bridge Y2 (half-bridge of phase V for instance).
            uint16_t WY1 : 16;  ///< PWM duty cycle for bridge WY1.
        } bits;
    };
};

} // namespace PWM
} // namespace TMC9660
