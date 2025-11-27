# Automatic Stall Detection Tuning

## Overview

The TMC9660 driver includes an automatic stall detection tuning subsystem that systematically calibrates velocity and position deviation thresholds for reliable stall detection without false triggers.

## Features

- **Automatic Threshold Scanning**: Systematically tests threshold ranges to find optimal values
- **Safe Current Margin**: Optionally reduces motor current during tuning for safer operation
- **Multi-Velocity Validation**: Tests thresholds at target, minimum, and maximum velocities
- **State Preservation**: Automatically saves and restores original settings
- **Comprehensive Results**: Provides detailed tuning results including measured values and validation status

## Usage

### Basic Example

```cpp
#include "tmc9660.hpp"

// Create driver instance
tmc9660::TMC9660<MyCommInterface> driver(comm_interface);

// Initialize driver (required before use)
tmc9660::BootloaderConfig cfg{};
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
cfg.boot.startMotorControl = true;
driver.bootloaderInit(&cfg);

// Perform automatic tuning
tmc9660::TMC9660<MyCommInterface>::StallDetectionTuning::TuningResult result;
if (driver.stallDetectionTuning.autoTune(
    5000,   // target velocity: 5000 steps/s
    result,
    100,    // min velocity deviation threshold
    5000,   // max velocity deviation threshold
    100,    // min position deviation threshold
    5000,   // max position deviation threshold
    1000,   // acceleration: 1000 steps/s²
    100,    // min velocity to validate (0 to skip)
    10000,  // max velocity to validate (0 to skip)
    400     // safe current margin: 400 mA (0 to disable)
)) {
    // Apply optimal thresholds
    driver.velocityControl.setStopOnVelocityDeviation(
        result.optimal_velocity_deviation, true); // true = soft stop
    driver.positionControl.setStopOnPositionDeviation(
        result.optimal_position_deviation, true);
    
    // Check validation results
    if (!result.min_velocity_success) {
        // Stall detection may not work reliably at low speeds
    }
    if (!result.max_velocity_success) {
        // Stall detection may not work reliably at high speeds
    }
}
```

### Advanced Example with Current Margin

```cpp
// Reduce motor current by 20% during tuning for safer operation
uint16_t current_margin = driver.motor.getMaxTorqueCurrent() * 0.2f;

tmc9660::TMC9660<MyCommInterface>::StallDetectionTuning::TuningResult result;
if (driver.stallDetectionTuning.autoTune(
    5000, result,
    100, 5000,  // velocity deviation range
    100, 5000,  // position deviation range
    1000,       // acceleration
    100, 10000, // velocity validation range
    current_margin // safe current margin
)) {
    // Use results...
}
```

## Tuning Algorithm

The tuning process follows these steps:

1. **Preparation**: 
   - Saves original deviation thresholds and motor current
   - Disables automatic stop-on-deviation (to observe without stopping)
   - Optionally reduces motor current for safer operation

2. **Velocity Ramp**:
   - Configures velocity mode
   - Ramps motor to target velocity
   - Waits for steady-state operation

3. **Threshold Scanning**:
   - Tests velocity deviation thresholds from min to max
   - Tests position deviation thresholds from min to max
   - Measures actual deviation/error at each threshold
   - Identifies thresholds that avoid false stalls
   - Selects optimal thresholds closest to target values

4. **Validation**:
   - Tests optimal thresholds at minimum velocity (if specified)
   - Tests optimal thresholds at maximum velocity (if specified)
   - Records whether thresholds work reliably at each speed

5. **Cleanup**:
   - Stops motor
   - Restores original deviation thresholds
   - Restores original motor current

## Result Structure

The `TuningResult` structure contains:

```cpp
struct TuningResult {
    bool tuning_success;                    // Overall success flag
    
    // Optimal thresholds found
    uint32_t optimal_velocity_deviation;    // Recommended velocity threshold
    uint32_t optimal_position_deviation;    // Recommended position threshold
    
    // Measured values at target velocity
    uint32_t target_velocity_deviation_reading;
    int32_t target_velocity_position_error;
    
    // Validation results
    bool min_velocity_success;              // Works at min velocity?
    bool max_velocity_success;              // Works at max velocity?
    uint32_t min_velocity_deviation_reading;
    uint32_t max_velocity_deviation_reading;
    
    // Velocity range limits
    int32_t actual_min_velocity;            // Lowest working velocity
    int32_t actual_max_velocity;            // Highest working velocity
    
    // Original settings (for reference)
    uint32_t original_velocity_deviation;
    uint32_t original_position_deviation;
    uint16_t original_current_mA;
};
```

## Adapting for TMC51x0 StallGuard

While this implementation is designed for TMC9660's velocity/position deviation-based stall detection, the same principles can be applied to TMC51x0 StallGuard tuning:

### Key Differences

1. **Threshold Type**: TMC51x0 uses SGT (StallGuard Threshold) values (-64 to +63) instead of deviation thresholds
2. **Reading**: TMC51x0 reads SG_RESULT (0-1023) instead of velocity/position errors
3. **Target Values**: StallGuard targets SG_RESULT around 100-500 at no-load, dropping to 0 at stall

### Adaptation Steps

1. Replace deviation threshold scanning with SGT scanning (-10 to +10 range recommended)
2. Replace `measureVelocityDeviation()` with `GetStallGuard()` to read SG_RESULT
3. Adjust target values: look for SG_RESULT around 200 at no-load
4. Modify false stall detection: SG_RESULT = 0 at no-load indicates false stall
5. Update threshold selection logic to work with SGT values

### Example Pseudo-Code for TMC51x0

```cpp
// Scan SGT range
for (int8_t sgt = 0; sgt <= max_sgt; ++sgt) {
    driver.diagnostics.ConfigureStallGuard(sgt, false); // SFILT=0
    delay(10);
    
    // Measure SG_RESULT
    uint16_t sg_result = 0;
    for (int i = 0; i < 8; ++i) {
        uint16_t sg;
        driver.diagnostics.GetStallGuard(sg);
        if (sg == 0) {
            // False stall detected - too sensitive
            break;
        }
        sg_result += sg;
        delay(5);
    }
    sg_result /= 8;
    
    // Check if close to target (~200)
    if (sg_result >= 100 && sg_result <= 500) {
        // Good candidate
    }
}
```

## Best Practices

1. **Run tuning with no load**: Ensure motor can spin freely during tuning
2. **Use safe current margin**: Reduce current by 20-30% for safer operation
3. **Choose appropriate target velocity**: Use a mid-range velocity (not too low, not too high)
4. **Validate at operating range**: Test at min/max velocities you'll actually use
5. **Re-tune if conditions change**: Motor current, load, or speed changes may require re-tuning

## Troubleshooting

### Tuning Fails

- **Check motor can spin**: Ensure no mechanical obstruction
- **Verify target velocity**: Too high or too low may cause issues
- **Check current settings**: Motor may be underpowered
- **Review threshold ranges**: Expand min/max ranges if needed

### False Stalls After Tuning

- Thresholds may be too sensitive: Increase thresholds slightly
- Re-run tuning with higher target velocity
- Check for mechanical issues (binding, excessive friction)

### No Stall Detection

- Thresholds may be too high: Decrease thresholds
- Motor may be overpowered: Reduce current or re-tune with margin
- Check velocity range: Stall detection may not work at very low speeds

## References

- TMC9660 Datasheet: Velocity/Position Deviation Detection
- StallGuard Application Note AN-002 (for TMC51x0 adaptation)
- Klipper Sensorless Homing Guide (for StallGuard best practices)
