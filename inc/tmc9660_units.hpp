/**
 * @file tmc9660_units.hpp
 * @brief Strongly-typed engineering-unit conversions for the TMC9660.
 *
 * The TMC9660 stores velocity, acceleration, position and angle in opaque
 * "internal units" whose meaning depends on the active velocity-feedback
 * source and the motor's pole-pair count. This header turns the raw
 * formulas from the TMC9660 Parameter Mode Reference Manual into a
 * compile-time-safe conversion API.
 *
 * Type-safety design
 * ------------------
 * Each physical dimension has its **own** strongly-typed enum class:
 *   - `VelocityUnit`     — never collides with `PositionUnit`
 *   - `PositionUnit`     — never collides with `AccelerationUnit`
 *   - `AccelerationUnit` — never collides with `AngleUnit`
 *   - `AngleUnit`
 *
 * The chip-side setters take `(double value, <DimensionUnit> unit, MotorContext const&)`
 * so a stray `setTargetVelocity(80, PositionUnit::DegMech, ctx)` is a
 * compile error. Convenience wrappers like `setTargetVelocityRpm(double)`
 * are thin inlines that hard-code the unit.
 *
 * Datasheet sources
 * -----------------
 *   TMC9660 Parameter Mode Reference Manual, "VELOCITY MODE":
 *     k_RPM = CPR · 2^24 / (40 MHz · 60)       (internal vel units / mech RPM)
 *     SAME_AS_COMMUTATION: CPR = 2^16 · pole_pairs
 *     DIGITAL_HALL:        CPR = 6     · pole_pairs
 *     ABN / SPI encoder:   CPR = encoder counts per mechanical revolution
 *
 *   Stepper note: a stepper's "pole pairs" equals `full_steps_per_rev / 4`
 *   (200-step stepper → 50 PP). The same `k_RPM` formula then applies.
 *
 *   PHI_E (NR 45 OPENLOOP_ANGLE, NR 74 HALL_PHI_E, NR 89 ABN_1_PHI_E …):
 *     int16, full-scale 65 536 LSBs == 360° electrical.
 *
 *   ACTUAL_POSITION (NR 142):
 *     1 mechanical revolution = `cpr` counts.
 */
#pragma once

#include "parameter_mode/tmc9660_param_mode_tmcl.hpp"

#include <cmath>
#include <cstdint>

namespace tmc9660 {
namespace units {

namespace tmcl = ::tmc9660::tmcl;

// ============================================================================
// Strongly-typed unit enums — one per physical dimension
// ============================================================================

/** @brief Velocity engineering units. `Internal` is the chip's raw int32. */
enum class VelocityUnit : uint8_t {
    Rpm,         ///< Revolutions per minute (mechanical shaft).
    Rps,         ///< Revolutions per second (mechanical shaft).
    RadPerSec,   ///< Radians per second (mechanical shaft).
    DegPerSec,   ///< Degrees per second (mechanical shaft).
    Internal,    ///< Raw TMC9660 internal velocity units (int32).
};

/** @brief Position engineering units. `Counts` is the chip's raw int32. */
enum class PositionUnit : uint8_t {
    MechRevs,    ///< Mechanical revolutions (float-friendly, 1.0 = full turn).
    DegMech,     ///< Mechanical degrees.
    RadMech,     ///< Mechanical radians.
    Counts,      ///< Raw TMC9660 ACTUAL_POSITION counts (1 rev = `cpr` counts).
};

/** @brief Acceleration engineering units. `Internal` is the chip's raw uint32. */
enum class AccelerationUnit : uint8_t {
    RpmPerSec,    ///< RPM per second (mechanical).
    RpsPerSec,    ///< RPS per second (mechanical).
    RadPerSec2,   ///< Rad/s² (mechanical).
    DegPerSec2,   ///< Deg/s² (mechanical).
    Internal,     ///< Raw TMC9660 internal acceleration (units/s², same scaling as velocity).
};

/** @brief Angle engineering units. `PhiERaw` is the chip's int16 (65536 ≡ 360° elec). */
enum class AngleUnit : uint8_t {
    DegElec,     ///< Electrical degrees (independent of pole pairs).
    RadElec,     ///< Electrical radians.
    DegMech,     ///< Mechanical degrees (= elec / pole_pairs).
    RadMech,     ///< Mechanical radians.
    PhiERaw,     ///< Raw int16 PHI_E (signed, 65536 LSBs = full electrical revolution).
};

// ============================================================================
// Math constants (private)
// ============================================================================
namespace detail {
inline constexpr double kPi             = 3.14159265358979323846;
inline constexpr double kTwoPi          = 2.0 * kPi;
inline constexpr double kRpmPerRps      = 60.0;
inline constexpr double kRpmPerRadPerSec = 60.0 / kTwoPi;
inline constexpr double kRpmPerDegPerSec = 60.0 / 360.0;
/// Datasheet numerator: 2^24 / (40 MHz · 60) = internal velocity units per (CPR · mech RPM)
inline constexpr double kVelScalePerCpr = 16777216.0 / (40e6 * 60.0);
/// Default chip clock (TMC9660 fCLK). Used in the ramp acceleration scaling.
inline constexpr double kFclkHz = 40.0e6;
/// Datasheet ramper acceleration scaling:
///   RAMPER_A = (ΔV_internal / Δt) × 2^17 / fCLK
/// i.e. `internal_acceleration_register = (dV_internal/dt) * kAccelInternalPerVdt`.
/// Parameter Mode reference manual p. 72 / Register Mode reference manual p. 44.
inline constexpr double kAccelInternalPerVdt = 131072.0 / kFclkHz;  // = 2^17 / fCLK
/// PHI_E full-scale: 65536 LSBs = 1 electrical revolution.
inline constexpr double kPhiEPerElecRev = 65536.0;
/// Chip register saturation limits (NR 124 TARGET_VELOCITY / NR 60 RAMP_VMAX):
///   velocity is signed 28-bit, range -2^27 .. 2^27-1.
inline constexpr int32_t  kVelocityInternalMin = -134217728;  // -2^27
inline constexpr int32_t  kVelocityInternalMax =  134217727;  //  2^27 - 1
/// Ramper acceleration register (NR 54..59) is unsigned 23-bit, range 1 .. 2^23-1.
inline constexpr uint32_t kAccelInternalMin    = 1u;
inline constexpr uint32_t kAccelInternalMax    = 8388607u;    //  2^23 - 1
}  // namespace detail

// ============================================================================
// MotorContext — chip facts needed to interpret internal units
// ============================================================================

/**
 * @brief Bundle of motor + feedback facts needed to interpret the chip's
 *        opaque internal units. Construct once per motor and reuse.
 *
 * For the BLDC case `pole_pairs` is the rotor pole-pair count. For the
 * STEPPER case the same field carries `full_steps_per_rev / 4` (so a
 * standard 200-step stepper has `pole_pairs = 50`); the unit math is then
 * identical to BLDC. The motor type is stored explicitly for callers that
 * need to dispatch on it (e.g. step/dir-only flows).
 *
 * For SPI/ABN encoder feedback selections, set `encoder_cpr` to the
 * encoder's mechanical CPR. For SAME_AS_COMMUTATION and DIGITAL_HALL the
 * chip derives CPR from `pole_pairs` and `encoder_cpr` is ignored.
 */
struct MotorContext {
    /// Motor topology. Affects only the conceptual interpretation of
    /// `pole_pairs`; the unit math is identical for all FOC-driven types.
    tmcl::MotorType motor_type = tmcl::MotorType::BLDC_MOTOR;

    /// Rotor pole pairs (BLDC) or `full_steps_per_rev/4` (stepper).
    /// 0 is treated as 1 to avoid divides-by-zero in derived quantities.
    uint8_t pole_pairs = 1;

    /// Active velocity-feedback source (NR 132 VELOCITY_SENSOR_SELECTION).
    tmcl::VelocitySensorSelection velocity_sensor =
        tmcl::VelocitySensorSelection::SAME_AS_COMMUTATION;

    /// Counts-per-mechanical-rev for an attached ABN/SPI encoder.
    /// Ignored when `velocity_sensor` is SAME_AS_COMMUTATION or DIGITAL_HALL.
    uint32_t encoder_cpr = 0;

    /// Effective counts-per-mechanical-revolution used by the chip's
    /// velocity scaling. Returns 0 if the sensor selection is one we
    /// cannot derive a CPR for here (caller should treat unit conversions
    /// involving velocity as undefined and fall back to internal units).
    [[nodiscard]] constexpr uint32_t cpr() const noexcept {
        const uint32_t pp = (pole_pairs == 0u) ? 1u : pole_pairs;
        switch (velocity_sensor) {
            case tmcl::VelocitySensorSelection::SAME_AS_COMMUTATION:
                return 65536u * pp;
            case tmcl::VelocitySensorSelection::DIGITAL_HALL:
                return 6u * pp;
            case tmcl::VelocitySensorSelection::ABN1_ENCODER:
            case tmcl::VelocitySensorSelection::ABN2_ENCODER:
            case tmcl::VelocitySensorSelection::SPI_ENCODER:
                return encoder_cpr;
        }
        return 0u;
    }

    /// Internal velocity units per mechanical RPM (`k_RPM` in the datasheet).
    ///
    /// This is the chip *meter* scale, not always true shaft speed. Three
    /// digital Halls that double-fire one edge per electrical revolution make
    /// the period meter read high by 7/6; `ACTUAL_POSITION` stays exact.
    /// Product firmware applies `hall_velocity_meter_scale` on top of this.
    /// Returns 0 for sensor selections lacking a derivable CPR.
    [[nodiscard]] double k_rpm() const noexcept {
        return static_cast<double>(cpr()) * detail::kVelScalePerCpr;
    }

    /// True when the context can produce meaningful unit conversions.
    [[nodiscard]] bool valid() const noexcept { return cpr() != 0u; }
};

// ============================================================================
// Velocity conversions
// ============================================================================

/// Convert a velocity from any unit to mech RPM (intermediate canonical form).
[[nodiscard]] inline double velocityToRpm(double value, VelocityUnit unit,
                                          MotorContext const& ctx) noexcept {
    switch (unit) {
        case VelocityUnit::Rpm:        return value;
        case VelocityUnit::Rps:        return value * detail::kRpmPerRps;
        case VelocityUnit::RadPerSec:  return value * detail::kRpmPerRadPerSec;
        case VelocityUnit::DegPerSec:  return value * detail::kRpmPerDegPerSec;
        case VelocityUnit::Internal: {
            const double k = ctx.k_rpm();
            return (k > 0.0) ? (value / k) : 0.0;
        }
    }
    return 0.0;
}

/// Convert a velocity from mech RPM (canonical) to any unit.
[[nodiscard]] inline double velocityFromRpm(double rpm, VelocityUnit unit,
                                            MotorContext const& ctx) noexcept {
    switch (unit) {
        case VelocityUnit::Rpm:        return rpm;
        case VelocityUnit::Rps:        return rpm / detail::kRpmPerRps;
        case VelocityUnit::RadPerSec:  return rpm / detail::kRpmPerRadPerSec;
        case VelocityUnit::DegPerSec:  return rpm / detail::kRpmPerDegPerSec;
        case VelocityUnit::Internal:   return rpm * ctx.k_rpm();
    }
    return 0.0;
}

/// Convert a velocity between any two units.
[[nodiscard]] inline double convertVelocity(double value, VelocityUnit from, VelocityUnit to,
                                            MotorContext const& ctx) noexcept {
    if (from == to) return value;
    return velocityFromRpm(velocityToRpm(value, from, ctx), to, ctx);
}

/// Convert a velocity in any unit to the chip's int32 internal-unit representation.
/// Saturates to the chip's signed 28-bit range ±(2^27-1) per NR 124 TARGET_VELOCITY
/// (Parameter Mode reference manual, table for parameter 124).
[[nodiscard]] inline int32_t velocityToInternal(double value, VelocityUnit unit,
                                                MotorContext const& ctx) noexcept {
    const double internal_d = velocityFromRpm(velocityToRpm(value, unit, ctx),
                                              VelocityUnit::Internal, ctx);
    if (internal_d > static_cast<double>(detail::kVelocityInternalMax)) return detail::kVelocityInternalMax;
    if (internal_d < static_cast<double>(detail::kVelocityInternalMin)) return detail::kVelocityInternalMin;
    return static_cast<int32_t>(internal_d >= 0.0 ? internal_d + 0.5 : internal_d - 0.5);
}

// ============================================================================
// Acceleration conversions
//
// `AccelerationUnit::Internal` is the **raw RAMPER_A register value** as
// stored in NR 54..59 (RAMP_AMAX/A1/A2/DMAX/D1/D2). The chip's scaling is:
//     RAMPER_A_register = (dV_internal/dt) * 2^17 / fCLK
// (Parameter Mode reference manual, p.72; Register Mode p.44).
// ============================================================================

[[nodiscard]] inline double accelerationToRpmPerSec(double value, AccelerationUnit unit,
                                                    MotorContext const& ctx) noexcept {
    switch (unit) {
        case AccelerationUnit::RpmPerSec:    return value;
        case AccelerationUnit::RpsPerSec:    return value * detail::kRpmPerRps;
        case AccelerationUnit::RadPerSec2:   return value * detail::kRpmPerRadPerSec;
        case AccelerationUnit::DegPerSec2:   return value * detail::kRpmPerDegPerSec;
        case AccelerationUnit::Internal: {
            // value is the register integer; recover dV/dt in internal
            // velocity units per second, then divide by k_rpm to get RPM/s.
            const double k = ctx.k_rpm();
            if (k <= 0.0 || detail::kAccelInternalPerVdt <= 0.0) return 0.0;
            const double dvdt_internal = value / detail::kAccelInternalPerVdt;
            return dvdt_internal / k;
        }
    }
    return 0.0;
}

[[nodiscard]] inline double accelerationFromRpmPerSec(double rpm_per_sec, AccelerationUnit unit,
                                                      MotorContext const& ctx) noexcept {
    switch (unit) {
        case AccelerationUnit::RpmPerSec:    return rpm_per_sec;
        case AccelerationUnit::RpsPerSec:    return rpm_per_sec / detail::kRpmPerRps;
        case AccelerationUnit::RadPerSec2:   return rpm_per_sec / detail::kRpmPerRadPerSec;
        case AccelerationUnit::DegPerSec2:   return rpm_per_sec / detail::kRpmPerDegPerSec;
        case AccelerationUnit::Internal: {
            // Forward direction: RAMPER_A_register = rpm/s * k_rpm * (2^17 / fCLK)
            return rpm_per_sec * ctx.k_rpm() * detail::kAccelInternalPerVdt;
        }
    }
    return 0.0;
}

[[nodiscard]] inline double convertAcceleration(double value, AccelerationUnit from,
                                                AccelerationUnit to,
                                                MotorContext const& ctx) noexcept {
    if (from == to) return value;
    return accelerationFromRpmPerSec(accelerationToRpmPerSec(value, from, ctx), to, ctx);
}

[[nodiscard]] inline uint32_t accelerationToInternal(double value, AccelerationUnit unit,
                                                     MotorContext const& ctx) noexcept {
    // Convert to the raw RAMPER_A register integer and saturate to the
    // chip's unsigned 23-bit range (1 .. 2^23-1).
    const double reg_d = accelerationFromRpmPerSec(
        accelerationToRpmPerSec(value, unit, ctx), AccelerationUnit::Internal, ctx);
    if (reg_d <= 0.0) return detail::kAccelInternalMin;
    if (reg_d > static_cast<double>(detail::kAccelInternalMax)) return detail::kAccelInternalMax;
    const uint32_t reg = static_cast<uint32_t>(reg_d + 0.5);
    return (reg < detail::kAccelInternalMin) ? detail::kAccelInternalMin : reg;
}

// ============================================================================
// Position conversions
// ============================================================================

[[nodiscard]] inline double positionToMechRevs(double value, PositionUnit unit,
                                               MotorContext const& ctx) noexcept {
    switch (unit) {
        case PositionUnit::MechRevs:  return value;
        case PositionUnit::DegMech:   return value / 360.0;
        case PositionUnit::RadMech:   return value / detail::kTwoPi;
        case PositionUnit::Counts: {
            const uint32_t c = ctx.cpr();
            return (c > 0u) ? (value / static_cast<double>(c)) : 0.0;
        }
    }
    return 0.0;
}

[[nodiscard]] inline double positionFromMechRevs(double revs, PositionUnit unit,
                                                 MotorContext const& ctx) noexcept {
    switch (unit) {
        case PositionUnit::MechRevs:  return revs;
        case PositionUnit::DegMech:   return revs * 360.0;
        case PositionUnit::RadMech:   return revs * detail::kTwoPi;
        case PositionUnit::Counts:    return revs * static_cast<double>(ctx.cpr());
    }
    return 0.0;
}

[[nodiscard]] inline double convertPosition(double value, PositionUnit from, PositionUnit to,
                                            MotorContext const& ctx) noexcept {
    if (from == to) return value;
    return positionFromMechRevs(positionToMechRevs(value, from, ctx), to, ctx);
}

[[nodiscard]] inline int32_t positionToCounts(double value, PositionUnit unit,
                                              MotorContext const& ctx) noexcept {
    const double counts_d = positionFromMechRevs(positionToMechRevs(value, unit, ctx),
                                                 PositionUnit::Counts, ctx);
    if (counts_d >  2147483647.0) return  2147483647;
    if (counts_d < -2147483648.0) return -2147483648;
    return static_cast<int32_t>(counts_d >= 0.0 ? counts_d + 0.5 : counts_d - 0.5);
}

// ============================================================================
// Angle conversions
// ============================================================================

[[nodiscard]] inline double angleToDegElec(double value, AngleUnit unit,
                                           MotorContext const& ctx) noexcept {
    const double pp = (ctx.pole_pairs == 0u) ? 1.0 : static_cast<double>(ctx.pole_pairs);
    switch (unit) {
        case AngleUnit::DegElec:   return value;
        case AngleUnit::RadElec:   return value * (180.0 / detail::kPi);
        case AngleUnit::DegMech:   return value * pp;
        case AngleUnit::RadMech:   return value * pp * (180.0 / detail::kPi);
        case AngleUnit::PhiERaw:   return value * (360.0 / detail::kPhiEPerElecRev);
    }
    return 0.0;
}

[[nodiscard]] inline double angleFromDegElec(double deg_elec, AngleUnit unit,
                                             MotorContext const& ctx) noexcept {
    const double pp = (ctx.pole_pairs == 0u) ? 1.0 : static_cast<double>(ctx.pole_pairs);
    switch (unit) {
        case AngleUnit::DegElec:   return deg_elec;
        case AngleUnit::RadElec:   return deg_elec * (detail::kPi / 180.0);
        case AngleUnit::DegMech:   return deg_elec / pp;
        case AngleUnit::RadMech:   return deg_elec / pp * (detail::kPi / 180.0);
        case AngleUnit::PhiERaw:   return deg_elec * (detail::kPhiEPerElecRev / 360.0);
    }
    return 0.0;
}

[[nodiscard]] inline double convertAngle(double value, AngleUnit from, AngleUnit to,
                                         MotorContext const& ctx) noexcept {
    if (from == to) return value;
    return angleFromDegElec(angleToDegElec(value, from, ctx), to, ctx);
}

[[nodiscard]] inline int16_t angleToPhiE(double value, AngleUnit unit,
                                         MotorContext const& ctx) noexcept {
    double phi_d = angleFromDegElec(angleToDegElec(value, unit, ctx), AngleUnit::PhiERaw, ctx);
    // Wrap into [-32768, 32767]
    while (phi_d >  32767.0) phi_d -= 65536.0;
    while (phi_d < -32768.0) phi_d += 65536.0;
    return static_cast<int16_t>(phi_d >= 0.0 ? phi_d + 0.5 : phi_d - 0.5);
}

}  // namespace units
}  // namespace tmc9660
