---
layout: default
title: "🔧 HardFOC TMC9660 Driver"
description: "Hardware-Agnostic TMC9660 Motor Controller Driver - Universal C++20 driver with FOC control, telemetry, and TMCL scripting"
nav_order: 1
permalink: /
has_children: true
---

# 🔧 HardFOC TMC9660 Driver:
**Hardware-Agnostic Motor Controller Driver**

![TMC9660](https://img.shields.io/badge/TMC9660-Motor%20Controller-blue?style=for-the-badge&logo=microchip)
![C++20](https://img.shields.io/badge/C%2B%2B-20-blue?style=for-the-badge&logo=cplusplus)
![Hardware-Agnostic](https://img.shields.io/badge/Hardware--Agnostic-Universal-green?style=for-the-badge&logo=hardware)
![License](https://img.shields.io/badge/License-GPL--3.0-green?style=for-the-badge&logo=opensourceinitiative)

## 🎯 Universal Motor Controller Interface for Multi-MCU Development

*A professional hardware-agnostic driver enabling seamless TMC9660 motor control across multiple MCU platforms - designed for the HardFOC board ecosystem*

---

## 📚 **Table of Contents**

- [🎯 **Overview**](#-overview)
- [🏗️ **Architecture**](#-architecture)
- [🔌 **Motor Types**](#-motor-types)
- [🚀 **Quick Start**](#-quick-start)
- [📖 **API Documentation**](#-api-documentation)
- [🔧 **Building**](#-building)
- [📊 **Examples**](#-examples)
- [🤝 **Contributing**](#-contributing)
- [📄 **License**](#-license)

---

## 🎯 **Overview**

> **📖 [📚🌐 Live Complete Documentation](https://n3b3x.github.io/hf-tmc9660-driver/)** - 
> Interactive guides, examples, and step-by-step tutorials

**HF-TMC9660** is a portable C++20 driver for the **TMC9660** motor controller from Trinamic. It exposes the full parameter mode interface with FOC control, telemetry readback and TMCL scripting. The driver is transport agnostic – implement `TMC9660CommInterface` for SPI or UART and run it on any MCU or host.

### 🏆 **Core Benefits**

- **🔄 Hardware Portability** - Write once, run on any MCU with SPI or UART
- **🎯 Unified API** - Consistent interface across all communication transports
- **⚡ FOC Control** - Advanced Field-Oriented Control for BLDC, stepper, and DC motors
- **🛡️ Safety Features** - Comprehensive protection systems and fault monitoring
- **📈 Telemetry** - Real-time temperature, current, and voltage monitoring
- **🔌 Complete Coverage** - Access to all 300+ TMC9660 parameters

### 🎨 **Design Philosophy**

```cpp
// Write hardware-agnostic motor control code
TMC9660 driver(commInterface);  // SPI or UART interface

// Configure for Parameter Mode
driver.bootloaderInit(&config);

// Control any motor type - same API
driver.motorConfig.setType(MotorType::BLDC_MOTOR, 7);
driver.focControl.setTargetVelocity(1000);
```

---

## 🏗️ **Architecture**

### **Transport-Agnostic Design**

```text
📦 TMC9660 Driver Architecture
├── 🎯 Driver Layer (TMC9660 class)     # High-level motor control API
│   ├── Motor Configuration             # Motor type, commutation, sensors
│   ├── FOC Control                     # Torque, velocity, position loops
│   ├── Telemetry                       # Temperature, current, voltage
│   ├── Bootloader                      # Parameter mode initialization
│   └── TMCL Scripting                  # Command execution
│
├── 🔌 Communication Interface          # Abstract transport layer
│   ├── TMC9660CommInterface            # Base interface
│   ├── SPITMC9660CommInterface         # SPI implementation
│   └── UARTTMC9660CommInterface        # UART implementation
│
└── 🔧 Hardware Layer                   # Platform-specific implementations
    ├── ESP32 SPI/UART                  # ESP32 family support
    ├── STM32 SPI/UART                  # STM32 support
    └── Any MCU with SPI/UART           # Your implementation
```

### **Abstraction Benefits**

#### **1. MCU Independence**
```cpp
// Application code remains the same across MCUs
class MotorController {
    TMC9660* driver;
    
public:
    void Initialize() {
        // Platform-specific interface, same driver code
        YourSPIInterface spi;  // ESP32, STM32, or any MCU
        driver = new TMC9660(spi);
        
        // Same configuration regardless of MCU
        driver->bootloaderInit(&config);
        driver->motorConfig.setType(MotorType::BLDC_MOTOR, 7);
    }
};
```

---

## 🔌 **Motor Types**

### **Supported Motor Types**

| **Motor Type** | **Control Mode** | **Key Features** |
|----------------|-----------------|------------------|
| **BLDC Motor** | FOC with Hall sensors | High efficiency, smooth operation |
| **BLDC Motor** | FOC with encoder | Precise position and velocity control |
| **Stepper Motor** | FOC position control | High torque, microstepping |
| **Stepper Motor** | STEP/DIR interface | Traditional stepper control |
| **DC Motor** | Velocity control | Simple brushed motor control |
| **DC Motor** | Current control | Open-loop current drive |

### **Communication Modes**

| **Mode** | **Transport** | **Use Case** |
|----------|---------------|--------------|
| **Parameter Mode** | SPI or UART | Full motor control, TMCL scripting |
| **Register Mode** | SPI only | Direct register access |

---

## 🚀 **Quick Start**

### **1. Clone Repository**
```bash
git clone https://github.com/n3b3x/hf-tmc9660-driver.git
cd hf-tmc9660-driver
```

### **2. Implement Communication Interface**
```cpp
#include "inc/TMC9660CommInterface.hpp"

class YourSPIInterface : public SPITMC9660CommInterface {
public:
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        // Your SPI transfer implementation
        return true;
    }
};
```

### **3. Basic Motor Control**
```cpp
#include "inc/TMC9660.hpp"

YourSPIInterface spi;
TMC9660 driver(spi);

// CRITICAL: Configure bootloader for Parameter Mode
tmc9660::BootloaderConfig cfg{};
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
cfg.boot.start_motor_control = true;

auto result = driver.bootloaderInit(&cfg);
if (result != TMC9660::BootloaderInitResult::Success) {
    return -1;
}

// Configure and start motor
driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
driver.focControl.setTargetVelocity(1000);
```

### **4. Build Example**
```bash
cd examples/esp32
./scripts/setup_repo.sh
./scripts/build_app.sh bldc_test Release
```

---

## 📖 **API Documentation**

### **Generated Documentation**
- **[📚 Complete Documentation](https://n3b3x.github.io/hf-tmc9660-driver/)** - Interactive guides and tutorials
- **[API Reference](docs/index.md)** - Complete driver API documentation
- **[Setup Guide](docs/SetupGuide.md)** - Installation and configuration

### **Key Concepts**

#### **Bootloader Initialization**
```cpp
// CRITICAL: Must configure for Parameter Mode first
tmc9660::BootloaderConfig cfg{};
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
cfg.boot.start_motor_control = true;
cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;

auto result = driver.bootloaderInit(&cfg);
```

#### **Motor Configuration**
```cpp
// Set motor type and pole pairs
driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);

// Configure current limits
driver.motorConfig.setMaxTorqueCurrent(2000);  // 2A limit

// Setup feedback sensors
driver.feedbackSense.configureHall();
```

#### **FOC Control**
```cpp
// Configure control gains
driver.focControl.setCurrentLoopGains(50, 100);
driver.focControl.setVelocityLoopGains(800, 1);

// Set target velocity
driver.focControl.setTargetVelocity(1000);
```

---

## 🔧 **Building**

### **Build System Features**
- **Multi-MCU Support** - Works with any MCU providing SPI or UART
- **ESP32 Examples** - Comprehensive test applications
- **Automated Testing** - Hardware validation suites
- **CI/CD Integration** - Automated builds and validation

### **Build Commands**
```bash
## For ESP32 development
cd examples/esp32
./scripts/setup_repo.sh
./scripts/build_app.sh <app_name> <build_type>

## Examples:
./scripts/build_app.sh bldc_test Release
./scripts/build_app.sh stepper_test Debug
./scripts/build_app.sh telemetry_test Release
```

---

## 📊 **Examples**

### **Available Test Applications**

| **Application** | **Tests** | **Purpose** |
|------------------|-----------|-------------|
| **bldc_test** | BLDC motor with Hall sensors | FOC commutation validation |
| **stepper_test** | Stepper motor control | Position and velocity control |
| **dc_test** | DC motor control | Simple velocity loop |
| **telemetry_test** | Temperature, current, voltage | Monitoring and diagnostics |
| **bootloader_test** | Bootloader configuration | Parameter mode setup |

### **Usage Examples**
```cpp
// BLDC Motor with Hall Sensors
driver.motorConfig.setType(MotorType::BLDC_MOTOR, 7);
driver.feedbackSense.configureHall();
driver.motorConfig.setCommutationMode(CommutationMode::FOC_HALL_SENSOR);
driver.focControl.setTargetVelocity(1000);

// Stepper Motor Position Control
driver.motorConfig.setType(MotorType::STEPPER_MOTOR, 200);
driver.focControl.setTargetPosition(10000);
```

---

## 🤝 **Contributing**

### **Development Workflow**
1. **Fork** the repository
2. **Create** feature branch (`feature/new-feature`)
3. **Implement** following coding standards
4. **Test** with existing applications
5. **Document** your changes
6. **Submit** pull request

### **Coding Standards**
- **Functions**: PascalCase (`SetTargetVelocity`, `GetChipTemperature`)
- **Types**: snake_case (`tmc9660::tmcl::MotorType`)
- **Error Handling**: Return status codes
- **Code Formatting**: Use `clang-format`

---

## 📄 **License**

This project is licensed under the **GNU General Public License v3.0**.

See [LICENSE](LICENSE) for full details.

---

## 🔗 **Quick Links**

### **Documentation**
- 📚 [Complete Documentation](https://n3b3x.github.io/hf-tmc9660-driver/) - Interactive guides and tutorials
- 📋 [API Reference](docs/index.md) - Complete driver API documentation
- 🔧 [Setup Guide](docs/SetupGuide.md) - Installation and build instructions
- 🔌 [Communication Interface Guide](docs/ImplementingCommInterface.md) - Platform integration
- ⚡ [Hardware Examples](docs/HardwareAgnosticExamples.md) - Practical usage scenarios

### **Development**
- 🚀 [Examples](examples/esp32/) - Test applications and usage examples
- 🧪 [Test Documentation](examples/esp32/docs/README.md) - Comprehensive test documentation
- 🔧 [Scripts](examples/esp32/scripts/) - Build, flash, and development tools
- 📊 [Configuration](examples/esp32/app_config.yml) - Application and build settings

### **Community**
- 🤝 [Contributing](CONTRIBUTING.md) - Development guidelines
- 🐛 [Issue Tracker](https://github.com/n3b3x/hf-tmc9660-driver/issues)
- 💬 [Discussions](https://github.com/n3b3x/hf-tmc9660-driver/discussions)

---

**Built for the HardFOC ecosystem - Enabling seamless motor control**

*Hardware-agnostic motor control that just works™*
