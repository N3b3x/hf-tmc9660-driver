---
layout: default
title: "⚡ Quick Start"
description: "Get up and running with the TMC9660 driver in minutes"
nav_order: 2
parent: "📚 Documentation"
permalink: /docs/quickstart/
---

# Quick Start

This guide will get you up and running with the TMC9660 driver in just a few steps.

## Prerequisites

- [Driver installed](installation.md)
- [Hardware wired](hardware_setup.md)
- Communication interface implemented (see [Platform Integration](platform_integration.md))

## Minimal Example

Here's a complete working example:

```cpp
#include "inc/tmc9660.hpp"

// 1. Implement your communication interface (see platform_integration.md)
class MySPI : public tmc9660::SpiCommInterface {
    // ... implement spiTransfer() method
};

int main() {
    // 2. Create communication interface
    MySPI spi;
    
    // 3. Create driver instance
    tmc9660::TMC9660 driver(spi);
    
    // 4. CRITICAL: Initialize bootloader for Parameter Mode
    tmc9660::BootloaderConfig cfg{};
    cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    cfg.boot.start_motor_control = true;
    
    auto result = driver.bootloaderInit(&cfg);
    if (result != tmc9660::TMC9660::BootloaderInitResult::Success) {
        // Handle initialization failure
        return -1;
    }
    
    // 5. Configure motor type
    driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7); // 7 pole pairs
    
    // 6. Start motor control
    driver.focControl.setTargetVelocity(1000); // 1000 RPM
    
    return 0;
}
```

## Step-by-Step Explanation

### Step 1: Implement Communication Interface

You must implement either `SpiCommInterface` or `UartCommInterface` for your platform. See [Platform Integration](platform_integration.md) for detailed examples.

### Step 2: Create Driver Instance

```cpp
tmc9660::TMC9660 driver(spi);
```

The driver takes a reference to your communication interface.

### Step 3: Initialize Bootloader

**This step is CRITICAL!** The TMC9660 must be configured for Parameter Mode before any motor control will work.

```cpp
tmc9660::BootloaderConfig cfg{};
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
cfg.boot.start_motor_control = true;

auto result = driver.bootloaderInit(&cfg);
```

See [Bootloader Initialization](special_feature_bootloader.md) for complete details.

### Step 4: Configure Motor

```cpp
driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
```

Set the motor type and pole pairs. Supported types:
- `BLDC_MOTOR` - Brushless DC motor
- `STEPPER_MOTOR` - Stepper motor
- `DC_MOTOR` - Brushed DC motor

### Step 5: Control Motor

```cpp
driver.focControl.setTargetVelocity(1000); // Set target velocity
```

## Next Steps

- **[Hardware Setup](hardware_setup.md)** - Complete wiring guide
- **[Platform Integration](platform_integration.md)** - Implement communication interface
- **[Bootloader Initialization](special_feature_bootloader.md)** - Critical bootloader setup
- **[Examples](examples.md)** - More detailed examples

