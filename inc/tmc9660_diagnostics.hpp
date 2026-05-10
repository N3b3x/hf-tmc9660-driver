/**
 * @file tmc9660_diagnostics.hpp
 * @brief Compact and full diagnostic snapshots for the TMC9660.
 *
 * Two structs, two access patterns:
 *
 *   - `MotorSummary`  — small (≈ 64 B), populated from one round of reads.
 *                       Use when polling at high rate or shipping over a
 *                       constrained transport. Contains: bus voltage, chip
 *                       temperature, commutation mode, target/actual velocity
 *                       in RPM, regulating-velocity flag, fault summary.
 *
 *   - `MotorSnapshot` — full state dump. Every read-only state on the chip
 *                       in both raw and engineering-unit forms. Use for
 *                       debugging, expectation-checking, structured logging,
 *                       and upstream health reports.
 *
 * Population is delegated to the `Diagnostics` subsystem on `TMC9660`
 * (see `tmc9660.hpp`), which is the only thing that knows how to talk to
 * the chip. This header is pure plain-old-data; it depends only on the
 * units module and the chip enums.
 */
#pragma once

#include "tmc9660_units.hpp"
#include "parameter_mode/tmc9660_param_mode_tmcl.hpp"

#include <cstdint>

namespace tmc9660 {
namespace diagnostics {

namespace tmcl  = ::tmc9660::tmcl;
namespace units = ::tmc9660::units;

/**
 * @brief Compact health summary — single-screen worth of state.
 *
 * Every field is populated best-effort: a chip read failure leaves the
 * default value. `valid_*` flags flip true only when the corresponding
 * read succeeded (so a downstream consumer can ignore stale fields).
 */
struct MotorSummary {
    // Bus / thermal
    float    vbus_volts                 = 0.0f;
    float    chip_temp_c                = 0.0f;

    // Mode
    tmcl::CommutationMode commutation_mode = tmcl::CommutationMode::SYSTEM_OFF;
    bool                  valid_commutation_mode = false;

    // Velocity (motor shaft, mechanical RPM)
    double   target_velocity_rpm        = 0.0;
    double   actual_velocity_rpm        = 0.0;
    bool     valid_velocity             = false;

    // Headline status bits
    bool     regulating_velocity        = false;
    bool     velocity_reached           = false;

    // Fault summary (true = any error bit set in the corresponding register)
    bool     has_general_error          = false;
    bool     has_gate_driver_error      = false;
    bool     has_adc_clipping           = false;

    // Raw register values (kept so callers can decode without re-polling)
    uint32_t general_error_flags        = 0;
    uint32_t gate_driver_error_flags    = 0;
    uint32_t adc_status_flags           = 0;
};

/**
 * @brief Full read-only state dump from the chip.
 *
 * Velocity / position fields carry both the raw int (`*_internal` /
 * `*_counts` / `*_phi_e`) and the engineering-unit form. The unit-form
 * fields are populated using the `MotorContext` passed to the populating
 * call, so they are only meaningful when `context.valid()`.
 */
struct MotorSnapshot {
    // ---- Context (echoed for the consumer) --------------------------------
    units::MotorContext context{};

    // ---- Bus / thermal ----------------------------------------------------
    float    vbus_volts        = 0.0f;
    float    chip_temp_c       = 0.0f;
    uint16_t external_temp_raw = 0;

    // ---- Mode -------------------------------------------------------------
    tmcl::CommutationMode commutation_mode       = tmcl::CommutationMode::SYSTEM_OFF;
    bool                  valid_commutation_mode = false;

    // ---- Velocity (motor shaft) ------------------------------------------
    int32_t  target_velocity_internal = 0;
    int32_t  actual_velocity_internal = 0;
    int32_t  ramp_velocity_internal   = 0;
    double   target_velocity_rpm      = 0.0;
    double   actual_velocity_rpm      = 0.0;
    double   ramp_velocity_rpm        = 0.0;

    // ---- Position --------------------------------------------------------
    int32_t  actual_position_counts   = 0;
    double   actual_position_revs     = 0.0;
    double   actual_position_deg_mech = 0.0;

    // ---- Electrical angle (PHI_E) ----------------------------------------
    int16_t  phi_e_internal           = 0;
    double   phi_e_deg_elec           = 0.0;
    double   phi_e_deg_mech           = 0.0;

    // ---- FOC currents / voltages -----------------------------------------
    int16_t  motor_current_ma         = 0;
    int16_t  iq_ma                    = 0;
    int16_t  i_ux                     = 0;
    int16_t  i_v                      = 0;
    int16_t  i_wy                     = 0;
    int16_t  uq                       = 0;
    int16_t  u_ux                     = 0;
    int16_t  u_v                      = 0;
    int16_t  u_wy                     = 0;

    // ---- Status / error registers ----------------------------------------
    uint32_t general_status_flags     = 0;
    uint32_t general_error_flags      = 0;
    uint32_t gate_driver_error_flags  = 0;
    uint32_t adc_status_flags         = 0;

    // ---- Decoded convenience flags (subset of general_status_flags) -----
    bool     regulating_torque        = false;
    bool     regulating_velocity      = false;
    bool     velocity_reached         = false;
};

}  // namespace diagnostics
}  // namespace tmc9660
