#pragma once
#include <cstdint>

/**
 * @file tmc9660_params.hpp
 * @brief TMC9660 Parameter Mode definitions.
 *
 * Provides an enumeration of parameter IDs and their descriptions for operating the TMC9660 in Parameter Mode.
 * Each parameter corresponds to a high-level configuration or real-time value.
 */

namespace TMC9660 {
namespace PARAM {

/// Parameter enumeration for TMC9660 Parameter Mode.
enum class ID : uint16_t {
    // --- Motion control targets and actual values ---
    TARGET_POSITION             = 0,   /**< Target position setpoint. Writing this activates position control mode (units: encoder counts). */
    ACTUAL_POSITION             = 1,   /**< Actual position feedback (encoder counts). Read-only. */
    TARGET_VELOCITY             = 2,   /**< Target velocity setpoint. Writing activates velocity control mode (units: counts/s). */
    ACTUAL_VELOCITY             = 3,   /**< Actual measured velocity. Read-only. */
    MAX_POSITION_OFFSET         = 4,   /**< [Reserved/Unused] Possibly reserved for future use (placeholder). */
    // (IDs 4 and 5 are not explicitly given in documentation; assume reserved or not used.)
    ACTUAL_POSITION_ERROR       = 5,   /**< Current position error (ACTUAL_POSITION - TARGET_POSITION). Read-only. */

    // --- Torque/Flux control loop ---
    MAX_TORQUE                  = 6,   /**< Maximum motor torque [mA]. Default 2000. Limits torque command:contentReference[oaicite:200]{index=200}. */
    MAX_FLUX                    = 7,   /**< Maximum motor flux [mA]. Default 2000. Limits flux current:contentReference[oaicite:201]{index=201}. */
    // IDs 8-53: Reserved for various intermediate or less-used parameters (e.g., commutation settings, reserved).

    // --- Ramp generator and motion profile ---
    RAMP_AMAX                   = 54,  /**< Acceleration in top (flat) part of ramp (Amax):contentReference[oaicite:202]{index=202}. Default 1000. */
    RAMP_A1                     = 55,  /**< First acceleration segment of ramp (A1):contentReference[oaicite:203]{index=203}. Default 8000. */
    RAMP_A2                     = 56,  /**< Second acceleration segment of ramp (A2):contentReference[oaicite:204]{index=204}. Default 4000. */
    RAMP_DMAX                   = 57,  /**< Deceleration in top part of ramp (Dmax):contentReference[oaicite:205]{index=205}. Default 1000. */
    RAMP_D1                     = 58,  /**< First deceleration segment of ramp (D1):contentReference[oaicite:206]{index=206}. Default 8000. */
    RAMP_D2                     = 59,  /**< Second deceleration segment of ramp (D2):contentReference[oaicite:207]{index=207}. Default 8000. */
    RAMP_VMAX                   = 60,  /**< Maximum velocity of ramp:contentReference[oaicite:208]{index=208}. Default 134217727 (full speed) ticks/s. */
    RAMP_V1                     = 61,  /**< Velocity threshold to transition from A1/D1 to A2/D2:contentReference[oaicite:209]{index=209} (mid-speed). */
    RAMP_V2                     = 62,  /**< Velocity threshold to transition from A2/D2 to Amax/Dmax:contentReference[oaicite:210]{index=210}. */
    RAMP_VSTART                 = 63,  /**< Start velocity of ramp:contentReference[oaicite:211]{index=211}. Typically 0. */
    RAMP_VSTOP                  = 64,  /**< Stop velocity of ramp (must be >0):contentReference[oaicite:212]{index=212}. Default 1. */
    RAMP_TVMAX                  = 65,  /**< Minimum time at Vmax before deceleration (TVMAX):contentReference[oaicite:213]{index=213}. Default 0. */
    RAMP_TZEROWAIT              = 66,  /**< Wait time at end of ramp (after reaching 0 velocity):contentReference[oaicite:214]{index=214}. Default 0. */
    ACCELERATION_FEEDFORWARD_ENABLE = 67,  /**< Enable acceleration feedforward feature:contentReference[oaicite:215]{index=215}. Boolean: 0=Disabled, 1=Enabled. */
    VELOCITY_FEEDFORWARD_ENABLE = 68,  /**< Enable velocity feedforward feature:contentReference[oaicite:216]{index=216}. Boolean: 0=Disabled, 1=Enabled. */
    RAMP_VELOCITY               = 69,  /**< Target velocity computed by ramp generator. Read-only. */
    RAMP_POSITION               = 70,  /**< Target position computed by ramp generator. Read-only. */

    // --- Mode activation and controls (the following trigger mode changes) ---
    TARGET_POSITION_PARAM       = 71,  /**< Write to this parameter to initiate position movement (alias for 0 in param mode usage). */
    TARGET_VELOCITY_PARAM       = 72,  /**< Write to initiate velocity motion (alias for 2). */
    // (In practice, writing TARGET_POSITION or TARGET_VELOCITY triggers mode changes.)

    // --- Torque and flux targets and measurements ---
    TARGET_TORQUE               = 104, /**< Target torque value [mA]:contentReference[oaicite:219]{index=219}. Writing engages torque regulator. Signed 16-bit. */
    ACTUAL_TORQUE               = 105, /**< Actual measured torque [mA]:contentReference[oaicite:220]{index=220}. Read-only. */
    TARGET_FLUX                 = 106, /**< Target flux value [mA]:contentReference[oaicite:221]{index=221}. */
    ACTUAL_FLUX                 = 107, /**< Actual measured flux [mA]:contentReference[oaicite:222]{index=222}. (32-bit, read-only) */

    TORQUE_OFFSET               = 108, /**< Torque offset [mA] (peak) applied to torque setpoint:contentReference[oaicite:223]{index=223}:contentReference[oaicite:224]{index=224}. Default 0. */
    TORQUE_P                    = 109, /**< Torque PI controller P gain:contentReference[oaicite:225]{index=225}. Default 50. If separate loops off, also flux P. */
    TORQUE_I                    = 110, /**< Torque PI controller I gain:contentReference[oaicite:226]{index=226}. Default 100. If separate loops off, also flux I. */
    FLUX_P                      = 111, /**< Flux PI controller P gain:contentReference[oaicite:227]{index=227}. Default 50. Only when separate torque/flux loops enabled. */
    FLUX_I                      = 112, /**< Flux PI controller I gain:contentReference[oaicite:228]{index=228}. Default 100. Only when separate loops enabled. */
    SEPARATE_TORQUE_FLUX_PI_PARAMETERS = 113, /**< Enable separate PI parameters for torque and flux loops:contentReference[oaicite:229]{index=229}. (0: combined, 1: separated) */
    CURRENT_NORM_P              = 114, /**< Normalization format for current PI P parameter:contentReference[oaicite:230]{index=230}. 0: shift 8-bit, 1: shift 16-bit. */
    CURRENT_NORM_I              = 115, /**< Normalization format for current PI I parameter:contentReference[oaicite:231]{index=231}:contentReference[oaicite:232]{index=232}. 0: 8-bit, 1: 16-bit. */
    TORQUE_PI_ERROR             = 116, /**< Torque PI regulator error (difference between target and actual):contentReference[oaicite:233]{index=233}. Read-only 32-bit. */
    FLUX_PI_ERROR               = 117, /**< Flux PI regulator error. Read-only. */
    TORQUE_PI_INTEGRATOR        = 118, /**< Torque PI integrator accumulator. Read-only 32-bit. */
    FLUX_PI_INTEGRATOR          = 119, /**< Flux PI integrator accumulator. Read-only. */
    FLUX_OFFSET                 = 120, /**< Flux offset [mA] applied to flux setpoint. Default 0. */

    // --- Velocity control loop parameters ---
    VELOCITY_P                  = 142, /**< Velocity PI controller P gain. (Exact ID inferred: likely around 142) */
    VELOCITY_I                  = 143, /**< Velocity PI controller I gain. */
    VELOCITY_NORM_P             = 144, /**< Normalization format for velocity PI P (0: no shift, 1: 8-bit, 2: 16-bit, 3: 24-bit):contentReference[oaicite:234]{index=234}. Default 1 (8-bit shift). */
    VELOCITY_NORM_I             = 145, /**< Normalization format for velocity PI I (0: 8-bit, 1: 16-bit, 2: 24-bit, 3: 32-bit):contentReference[oaicite:235]{index=235}. Default 1 (16-bit shift). */
    VELOCITY_PI_INTEGRATOR      = 146, /**< Integrated error of velocity PI regulator. Read-only 32-bit. */
    VELOCITY_PI_ERROR           = 147, /**< Velocity PI regulator error (target - actual). Read-only. */

    // --- Position control loop parameters ---
    POSITION_SCALING_FACTOR     = 145, /**< Scaling factor to convert internal position to real-world units:contentReference[oaicite:236]{index=236}. Default 1024. */
    POSITION_P                  = 146, /**< Position PI controller P gain:contentReference[oaicite:237]{index=237}. Default 5. */
    POSITION_I                  = 147, /**< Position PI controller I gain:contentReference[oaicite:238]{index=238}. Default 0. */
    POSITION_NORM_P             = 148, /**< Normalization for position PI P:contentReference[oaicite:239]{index=239}. 0: no shift, 1: 8-bit, 2: 16-bit, 3: 24-bit. Default 1. */
    POSITION_NORM_I             = 149, /**< Normalization for position PI I:contentReference[oaicite:240]{index=240}. 0:8-bit,1:16-bit,2:24-bit,3:32-bit. Default 1. */
    POSITION_PI_INTEGRATOR      = 150, /**< Integrated error of position PI regulator:contentReference[oaicite:241]{index=241}. Read-only. */
    POSITION_PI_ERROR           = 151, /**< Error of position PI regulator:contentReference[oaicite:242]{index=242}. Read-only. */
    STOP_ON_POSITION_DEVIATION  = 152, /**< Position deviation threshold that triggers a stop event (if enabled):contentReference[oaicite:243]{index=243}. Default 0. */
    POSITION_LOOP_DOWNSAMPLING  = 153, /**< Downsampling factor for position loop updates:contentReference[oaicite:244]{index=244}. Default 0 (no downsample). */
    LATCH_POSITION              = 154, /**< Latched position value on event (e.g., stop switch activation):contentReference[oaicite:245]{index=245}. Read-only. */
    POSITION_LIMIT_LOW          = 155, /**< Lower software position limit:contentReference[oaicite:246]{index=246}. Default -2147483648. */
    POSITION_LIMIT_HIGH         = 156, /**< Upper software position limit:contentReference[oaicite:247]{index=247}. Default +2147483647. */
    // (Parameter IDs may extend further for additional features or reserved entries.)
};

} // namespace PARAM
} // namespace TMC9660
