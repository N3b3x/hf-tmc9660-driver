---
layout: default
title: "📦 Namespace Structure"
description: "Complete guide to the TMC9660 library namespace organization and usage patterns"
nav_order: 10
parent: "📚 Documentation"
permalink: /docs/NamespaceStructure/
---

## 📦 TMC9660 Library Namespace Structure

This guide documents the complete namespace structure of the TMC9660 library, explaining how namespaces are organized, when to use them, and best practices for working with the library's API.

---

## 🎯 Overview

The TMC9660 library uses a **hierarchical namespace structure** that groups related functionality logically. All namespaces use **lowercase naming** and follow C++ best practices for namespace organization.

### Key Principles

- ✅ **Lowercase namespace names** - All namespaces use lowercase (`tmc9660`, not `TMC9660`)
- ✅ **Logical grouping** - Related functionality is grouped together
- ✅ **Proper nesting** - Sub-namespaces are nested under the main namespace
- ✅ **Clear hierarchy** - Easy to understand and navigate

---

## 📊 Complete Namespace Hierarchy

```
tmc9660                                    (main namespace)
├── TMC9660                                (main driver class)
├── CommInterface                          (communication interface)
├── TMC9660Bootloader                     (bootloader class)
│
├── bootcfg                                (bootloader configuration)
│   ├── BootMode
│   ├── BaudRate
│   ├── SPIInterface
│   ├── LDOVoltage
│   └── ... (all bootloader config enums)
│
├── bootaddr                               (bootloader addresses)
│
├── tmcl                                   (TMCL parameter mode)
│   ├── Op                                 (TMCL operations)
│   ├── Status                             (TMCL status codes)
│   ├── Parameters                         (TMCL parameters)
│   ├── MotorType
│   ├── CommutationMode
│   └── ... (all TMCL types)
│
└── register_mode                          (register mode)
    ├── ADC                                (ADC registers)
    ├── MCC                                (Motor Control Core registers)
    ├── SYS_CTRL                           (System Control registers)
    └── RAMDebug                           (RAMDebug registers)
```

---

## 🔍 Detailed Namespace Reference

### Main Namespace: `tmc9660`

The root namespace containing all main classes and functionality.

**Location**: `inc/TMC9660.hpp`, `inc/TMC9660CommInterface.hpp`, `inc/bootloader/TMC9660Bootloader.hpp`

**Contents**:
- `tmc9660::TMC9660` - Main driver class
- `tmc9660::CommInterface` - Abstract communication interface
- `tmc9660::TMC9660Bootloader` - Bootloader management class
- `tmc9660::BootloaderConfig` - Bootloader configuration structure

**Usage Example**:
```cpp
#include "TMC9660.hpp"

// Option 1: Fully qualified names
tmc9660::TMC9660 driver(commInterface);
tmc9660::BootloaderConfig cfg{};

// Option 2: Using declaration (recommended in implementation files)
using namespace tmc9660;
TMC9660 driver(commInterface);
BootloaderConfig cfg{};
```

---

### Bootloader Configuration: `tmc9660::bootcfg`

Contains all bootloader configuration enumerations and related types.

**Location**: `inc/bootloader/bootloader_config.hpp`

**Key Types**:
- `bootcfg::BootMode` - Boot mode selection (Parameter, Register)
- `bootcfg::BaudRate` - UART baud rate settings
- `bootcfg::SPIInterface` - SPI interface selection
- `bootcfg::LDOVoltage` - LDO voltage settings
- `bootcfg::ClockSource` - Clock source configuration
- `bootcfg::MemStorage` - External memory storage options

**Usage Example**:
```cpp
#include "bootloader/TMC9660Bootloader.hpp"
using namespace tmc9660;

BootloaderConfig cfg{};
cfg.boot.boot_mode = bootcfg::BootMode::Parameter;
cfg.uart.baud_rate = bootcfg::BaudRate::BR115200;
cfg.spiComm.boot_spi_iface = bootcfg::SPIInterface::SPI0;
cfg.clock.use_external = bootcfg::ClockSource::External;
```

**Complete Enumeration List**:
- `BootMode` - Boot mode selection
- `BaudRate` - UART baud rates
- `SPIInterface` - SPI interface selection
- `LDOVoltage` - LDO output voltage levels
- `LDOSlope` - LDO ramp-up time
- `UartRxPin`, `UartTxPin` - UART pin assignments
- `RS485TxEnPin` - RS485 transmit enable pin
- `SPI0SckPin` - SPI0 clock pin selection
- `I2CSdaPin`, `I2CSclPin` - I2C pin assignments
- `I2CFreq` - I2C clock frequency
- `ClockSource` - Clock source (Internal/External)
- `ExtSourceType` - External clock source type
- `XtalDrive` - Crystal drive strength
- `SysClkSource` - System clock source
- `SysClkDiv` - System clock divider
- `HallUPin`, `HallVPin`, `HallWPin` - Hall sensor pins
- `ABN1APin`, `ABN1BPin`, `ABN1NPin` - ABN encoder 1 pins
- `ABN2APin`, `ABN2BPin` - ABN encoder 2 pins
- `RefLPin`, `RefRPin`, `RefHPin` - Reference switch pins
- `BrakeChopperOutput` - Brake chopper output pin
- `MechBrakeOutput` - Mechanical brake output pin
- `MemStorage` - External memory storage type
- `SPIFlashFreq` - SPI flash frequency divider

---

### Bootloader Addresses: `tmc9660::bootaddr`

Contains bootloader register address definitions.

**Location**: `inc/bootloader/bootloader_config.hpp`

**Usage**: Typically used internally by the bootloader implementation. Most users won't need to reference this namespace directly.

---

### TMCL Parameter Mode: `tmc9660::tmcl`

Contains all TMCL (Trinamic Motion Control Language) related definitions for parameter mode operation.

**Location**: `inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`

**Key Types**:
- `tmcl::Op` - TMCL operation codes
- `tmcl::Status` - TMCL status codes
- `tmcl::Parameters` - TMCL parameter enumerations
- `tmcl::MotorType` - Motor type definitions
- `tmcl::CommutationMode` - Commutation mode settings
- `tmcl::Direction` - Direction settings
- `tmcl::EnableDisable` - Enable/disable flags

**Usage Example**:
```cpp
#include "TMC9660.hpp"
using namespace tmc9660;

// Configure motor type
driver.motorConfig.setType(tmcl::MotorType::BLDC_MOTOR, 7);

// Set commutation mode
driver.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_ABN);

// Read parameter
uint32_t value;
driver.readParameter(tmcl::Parameters::ACTUAL_POSITION, value);

// Send TMCL command
driver.sendCommand(tmcl::Op::ROR, 0, 0, 1000, nullptr);
```

**Common Enumerations**:
- `MotorType` - NO_MOTOR, DC_MOTOR, STEPPER_MOTOR, BLDC_MOTOR
- `CommutationMode` - SYSTEM_OFF, FOC_HALL_SENSOR, FOC_ABN, FOC_OPENLOOP_CURRENT_MODE, etc.
- `Parameters` - All 300+ TMCL parameters (ACTUAL_POSITION, TARGET_POSITION, etc.)
- `Op` - TMCL operation codes (ROR, ROL, MST, MVP, etc.)
- `Status` - TMCL status codes (SUCCESS, WRONG_CHECKSUM, INVALID_VALUE, etc.)

---

### Register Mode: `tmc9660::register_mode`

Contains register definitions for direct register access mode (not commonly used in parameter mode applications).

**Location**: `inc/register_mode/*.hpp`

**Sub-namespaces**:
- `register_mode::ADC` - ADC and analog measurement registers
- `register_mode::MCC` - Motor Control Core registers
- `register_mode::SYS_CTRL` - System Control registers
- `register_mode::RAMDebug` - RAMDebug system registers

**Usage Example**:
```cpp
#include "register_mode/tmc9660_mcc.hpp"
using namespace tmc9660;

// Access register definitions
register_mode::MCC::CHIP_ID chipId;
register_mode::ADC::ADC_SOURCES adcConfig;
register_mode::SYS_CTRL::FAULT_STATUS faultStatus;
```

**Note**: Register mode is typically used for low-level debugging or when direct register access is required. Most applications use parameter mode (`tmcl`) instead.

---

## 💡 Usage Patterns

### Pattern 1: Fully Qualified Names (Headers)

In header files, use fully qualified names to avoid namespace pollution:

```cpp
// MyDriver.hpp
#pragma once
#include "TMC9660.hpp"

class MyDriver {
public:
    void configure(tmc9660::BootloaderConfig& cfg);
    void setMotorType(tmc9660::tmcl::MotorType type);
};
```

### Pattern 2: Using Declaration (Implementation Files)

In `.cpp` files, use `using namespace` for convenience:

```cpp
// MyDriver.cpp
#include "MyDriver.hpp"
#include "TMC9660.hpp"

using namespace tmc9660;

void MyDriver::configure(BootloaderConfig& cfg) {
    cfg.boot.boot_mode = bootcfg::BootMode::Parameter;
    cfg.uart.baud_rate = bootcfg::BaudRate::BR115200;
}

void MyDriver::setMotorType(tmcl::MotorType type) {
    driver.motorConfig.setType(type, 7);
}
```

### Pattern 3: Selective Using (Recommended)

For better clarity, use selective `using` declarations:

```cpp
#include "TMC9660.hpp"

using tmc9660::TMC9660;
using tmc9660::BootloaderConfig;
using tmc9660::bootcfg::BootMode;
using tmc9660::bootcfg::BaudRate;
using tmc9660::tmcl::MotorType;

BootloaderConfig cfg{};
cfg.boot.boot_mode = BootMode::Parameter;
cfg.uart.baud_rate = BaudRate::BR115200;
```

### Pattern 4: Namespace Aliases

For frequently used nested namespaces, create aliases:

```cpp
#include "TMC9660.hpp"

namespace tmc = tmc9660;
namespace cfg = tmc9660::bootcfg;
namespace tmcl = tmc9660::tmcl;

tmc::BootloaderConfig cfg{};
cfg.boot.boot_mode = cfg::BootMode::Parameter;
cfg.uart.baud_rate = cfg::BaudRate::BR115200;
tmc::TMC9660 driver(comm);
driver.motorConfig.setType(tmcl::MotorType::BLDC_MOTOR, 7);
```

---

## 🎓 Best Practices

### ✅ Do

1. **Use `using namespace tmc9660;` in implementation files** - Makes code cleaner and easier to read
2. **Use fully qualified names in headers** - Prevents namespace pollution
3. **Group related includes** - Keep namespace-related includes together
4. **Use namespace aliases for deep nesting** - Improves readability

### ❌ Don't

1. **Don't use `using namespace` in headers** - Can cause namespace conflicts
2. **Don't pollute global namespace** - Always scope types properly
3. **Don't mix namespace styles** - Be consistent within a file
4. **Don't use uppercase namespace names** - All namespaces use lowercase

---

## 📝 Common Code Patterns

### Initialization Pattern

```cpp
#include "TMC9660.hpp"
using namespace tmc9660;

// 1. Create bootloader configuration
BootloaderConfig cfg{};
cfg.boot.boot_mode = bootcfg::BootMode::Parameter;
cfg.boot.start_motor_control = true;
cfg.uart.baud_rate = bootcfg::BaudRate::BR115200;
cfg.clock.use_external = bootcfg::ClockSource::External;

// 2. Initialize driver
TMC9660 driver(commInterface);
auto result = driver.bootloaderInit(&cfg);

// 3. Configure motor
driver.motorConfig.setType(tmcl::MotorType::BLDC_MOTOR, 7);
driver.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_ABN);
```

### Configuration Pattern

```cpp
#include "TMC9660.hpp"
using namespace tmc9660;

void configureBootloader(BootloaderConfig& cfg) {
    // Boot mode
    cfg.boot.boot_mode = bootcfg::BootMode::Parameter;
    cfg.boot.start_motor_control = true;
    
    // UART settings
    cfg.uart.baud_rate = bootcfg::BaudRate::BR115200;
    cfg.uart.rx_pin = bootcfg::UartRxPin::GPIO7;
    cfg.uart.tx_pin = bootcfg::UartTxPin::GPIO6;
    
    // Clock configuration
    cfg.clock.use_external = bootcfg::ClockSource::External;
    cfg.clock.xtal_drive = bootcfg::XtalDrive::Freq16MHz;
    cfg.clock.pll_selection = bootcfg::SysClkSource::PLL;
}
```

### Motor Control Pattern

```cpp
#include "TMC9660.hpp"
using namespace tmc9660;

void controlMotor(TMC9660& driver) {
    // Set motor type
    driver.motorConfig.setType(tmcl::MotorType::BLDC_MOTOR, 7);
    
    // Set commutation mode
    driver.motorConfig.setCommutationMode(tmcl::CommutationMode::FOC_ABN);
    
    // Control motor
    driver.velocityControl.setTargetVelocity(1000);
    driver.motorConfig.enable();
    
    // Read status
    int32_t position;
    driver.positionControl.getActualPosition(position);
}
```

---

## 🔗 Related Documentation

- **[Setup Guide](setup_guide.md)** - Getting started with the library
- **[Bootloader Initialization Guide](bootloader_initialization_guide.md)** - Bootloader configuration
- **[TMCL Protocol Guide](tmcl_protocol_guide.md)** - Parameter mode usage
- **[Common Operations](common_operations.md)** - Everyday usage patterns
- **[Hardware-Agnostic Examples](hardware_agnostic_examples.md)** - Complete examples

---

## 📋 Quick Reference

### Main Classes
- `tmc9660::TMC9660` - Main driver
- `tmc9660::TMC9660CommInterface` - Communication interface
- `tmc9660::TMC9660Bootloader` - Bootloader class

### Configuration Namespaces
- `tmc9660::bootcfg` - Bootloader configuration
- `tmc9660::bootaddr` - Bootloader addresses

### Parameter Mode
- `tmc9660::tmcl` - TMCL commands and parameters

### Register Mode
- `tmc9660::register_mode::ADC` - ADC registers
- `tmc9660::register_mode::MCC` - Motor Control Core
- `tmc9660::register_mode::SYS_CTRL` - System Control
- `tmc9660::register_mode::RAMDebug` - RAMDebug

---

<div style="text-align: center; margin: 2em 0; padding: 1em; background: #f8f9fa; border-radius: 8px;">
  <strong>💡 Tip</strong><br>
  Use <code>using namespace tmc9660;</code> in implementation files for cleaner code,<br>
  but always use fully qualified names in header files.
</div>

---

