# TMC9660 Parameter Mode Driver - Complete Analysis

## Executive Summary

The hf-tmc9660-driver is a comprehensive C++20 hardware-agnostic driver library for the TMC9660 motor controller operating in **Parameter Mode**. This analysis covers the complete initialization sequence from bootloader configuration through parameter mode setup, inner workings, and operational details.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Bootloader Configuration](#bootloader-configuration)
3. [Parameter Mode Initialization](#parameter-mode-initialization)
4. [Driver Inner Workings](#driver-inner-workings)
5. [Communication Layer](#communication-layer)
6. [Motor Configuration Workflow](#motor-configuration-workflow)
7. [FOC Control System](#foc-control-system)
8. [Telemetry and Monitoring](#telemetry-and-monitoring)
9. [Error Handling and Diagnostics](#error-handling-and-diagnostics)
10. [Best Practices](#best-practices)

---

## Architecture Overview

### Core Components

```
┌─────────────────────────────────────────────────────────────┐
│                        TMC9660 Driver                       │
├─────────────────────────────────────────────────────────────┤
│  MotorConfig  │  CurrentSensing  │  GateDriver  │  FOCControl │
│  FeedbackSense│  Protection      │  Telemetry   │  Ramp       │
├─────────────────────────────────────────────────────────────┤
│              TMC9660CommInterface (Abstract)                │
├─────────────────────────────────────────────────────────────┤
│        SPI Implementation  │  UART Implementation           │
└─────────────────────────────────────────────────────────────┘
```

### Key Classes:
- **TMC9660**: Main driver class with subsystem interfaces
- **TMC9660Bootloader**: Handles initial hardware configuration
- **TMC9660CommInterface**: Abstract communication layer
- **Subsystems**: Specialized interfaces for motor control aspects

---

## Bootloader Configuration

### 1. Bootloader Initialization Sequence

The bootloader phase is critical for setting up the TMC9660 hardware before entering parameter mode:

```cpp
// Bootloader configuration structure
tmc9660::BootloaderConfig cfg{};

// UART Communication Setup
cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;
cfg.uart.device_address = 1;
cfg.uart.host_address = 255;
cfg.uart.rx_pin = tmc9660::bootcfg::UartRxPin::GPIO7;
cfg.uart.tx_pin = tmc9660::bootcfg::UartTxPin::GPIO6;

// SPI Communication Setup
cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
cfg.spiComm.spi0_sck_pin = tmc9660::bootcfg::SPI0SckPin::GPIO6;
cfg.spiComm.disable_spi = false;

// Clock Configuration
cfg.clock.use_external = tmc9660::bootcfg::ClockSource::Internal;
cfg.clock.xtal_drive = tmc9660::bootcfg::XtalDrive::Freq16MHz;
cfg.clock.pll_selection = tmc9660::bootcfg::SysClkSource::PLL;
```

### 2. Bootloader Register Programming

The bootloader uses a specific command set to program configuration registers:

| Command | Opcode | Function |
|---------|--------|----------|
| SET_BANK | 0x01 | Select register bank |
| SET_ADDRESS | 0x02 | Set address within bank |
| WRITE_8 | 0x03 | Write 8-bit value |
| WRITE_16 | 0x04 | Write 16-bit value |
| WRITE_32 | 0x05 | Write 32-bit value |
| OTP_BURN | 0x06 | Burn configuration to OTP |

### 3. Critical Bootloader Settings

**Boot Mode Selection:**
```cpp
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter; // Key setting!
cfg.boot.start_motor_control = true; // Auto-start parameter mode
```

**GPIO Configuration:**
```cpp
cfg.gpio.outputMask = 0x00000000;     // GPIO output levels
cfg.gpio.directionMask = 0x00000000;  // GPIO directions
cfg.gpio.pullUpMask = 0x00000000;     // Pull-up enables
cfg.gpio.pullDownMask = 0x00000000;   // Pull-down enables
```

---

## Parameter Mode Initialization

### 1. TMCL Command Structure

Parameter mode uses TMCL (Trinamic Motion Control Language) commands:

```cpp
struct TMCLFrame {
    uint8_t opcode;    // Operation code
    uint16_t type;     // Parameter ID or command type
    uint8_t motor;     // Motor/bank index
    uint32_t value;    // Parameter value
};
```

### 2. Key TMCL Operations

| Operation | Code | Description |
|-----------|------|-------------|
| SAP | 5 | Set Axis Parameter |
| GAP | 6 | Get Axis Parameter |
| SGP | 9 | Set Global Parameter |
| GGP | 10 | Get Global Parameter |
| MST | 3 | Motor Stop |
| RFS | 13 | Reference Search |

### 3. Parameter Mode Setup Sequence

```cpp
TMC9660 driver(commInterface);

// 1. Initialize bootloader
auto result = driver.bootloaderInit(&cfg);
if (result != TMC9660::BootloaderInitResult::Success) {
    // Handle bootloader failure
}

// 2. Configure motor type
driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);

// 3. Set commutation mode
driver.motorConfig.setCommutationMode(
    tmc9660::tmcl::CommutationMode::FOC_HALL_SENSOR);

// 4. Configure current limits
driver.motorConfig.setMaxTorqueCurrent(2000); // 2A max
driver.motorConfig.setMaxFluxCurrent(1000);   // 1A flux

// 5. Setup feedback sensors
driver.feedbackSense.configureHall();

// 6. Enable FOC control
driver.focControl.setTargetVelocity(1000);
```

---

## Driver Inner Workings

### 1. Core Parameter Access

The driver implements a clean abstraction over TMCL commands:

```cpp
bool TMC9660::writeParameter(tmc9660::tmcl::Parameters id, 
                           uint32_t value, 
                           uint8_t motorIndex) {
    return sendCommand(tmc9660::tmcl::Op::SAP, 
                      static_cast<uint16_t>(id), 
                      motorIndex, 
                      value, 
                      nullptr);
}

bool TMC9660::readParameter(tmc9660::tmcl::Parameters id, 
                          uint32_t &value, 
                          uint8_t motorIndex) {
    return sendCommand(tmc9660::tmcl::Op::GAP, 
                      static_cast<uint16_t>(id), 
                      motorIndex, 
                      0, 
                      &value);
}
```

### 2. Subsystem Organization

Each subsystem provides a focused interface:

**Motor Configuration:**
- Motor type (DC, BLDC, Stepper)
- Pole pairs and direction
- PWM frequency and switching scheme
- Current limits

**FOC Control:**
- Torque, velocity, and position control
- PI loop configuration
- Reference switches and limits
- Open-loop operation

**Current Sensing:**
- ADC configuration and scaling
- Shunt selection and amplifier settings
- Offset calibration

**Feedback Sensing:**
- ABN encoder setup
- Hall sensor configuration
- SPI encoder interface

### 3. Communication Layer Details

The abstract `TMC9660CommInterface` enables hardware portability:

```cpp
class TMC9660CommInterface {
public:
    virtual bool transfer(const TMCLFrame& tx,
                         TMCLReply& reply,
                         uint8_t address) noexcept = 0;
    virtual CommMode mode() const noexcept = 0;
};
```

**SPI Implementation Example:**
```cpp
class SPIInterface : public SPITMC9660CommInterface {
public:
    bool spiTransfer(std::array<uint8_t,8>& tx,
                    std::array<uint8_t,8>& rx) noexcept override {
        // Hardware-specific SPI transfer
        // Toggle CS, send 8 bytes, receive 8 bytes
        return performSPIExchange(tx, rx);
    }
};
```

---

## Motor Configuration Workflow

### 1. BLDC Motor Setup

```cpp
// Step 1: Motor type and parameters
driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
driver.motorConfig.setDirection(tmc9660::tmcl::MotorDirection::FORWARD);
driver.motorConfig.setPWMFrequency(25000); // 25 kHz

// Step 2: Current limits
driver.motorConfig.setMaxTorqueCurrent(2000); // 2A
driver.motorConfig.setMaxFluxCurrent(1000);   // 1A
driver.motorConfig.setOutputVoltageLimit(8000);

// Step 3: Commutation setup
driver.motorConfig.setCommutationMode(
    tmc9660::tmcl::CommutationMode::FOC_HALL_SENSOR);
driver.motorConfig.setPWMSwitchingScheme(
    tmc9660::tmcl::PwmSwitchingScheme::SVPWM);

// Step 4: Feedback configuration
driver.feedbackSense.configureHall();
```

### 2. Stepper Motor Setup

```cpp
// Stepper-specific configuration
driver.motorConfig.setType(tmc9660::tmcl::MotorType::STEPPER_MOTOR, 1);
driver.motorConfig.setCommutationMode(
    tmc9660::tmcl::CommutationMode::FOC_ENCODER);

// Configure encoder for position feedback
driver.feedbackSense.configureABNEncoder(1024); // 1024 CPR
```

### 3. DC Motor Setup

```cpp
// DC motor configuration
driver.motorConfig.setType(tmc9660::tmcl::MotorType::DC_MOTOR);
driver.motorConfig.setCommutationMode(
    tmc9660::tmcl::CommutationMode::FOC_OPENLOOP_CURRENT_MODE);
```

---

## FOC Control System

### 1. Control Loop Hierarchy

```
Position Loop (outermost)
    ↓
Velocity Loop (middle)
    ↓
Current Loop (innermost, fastest)
```

### 2. Torque Control

```cpp
// Direct torque control
driver.focControl.setTargetTorque(1500); // 1.5A torque current

// Configure current PI gains
driver.focControl.setCurrentLoopGains(50, 100); // P=50, I=100
```

### 3. Velocity Control

```cpp
// Velocity control setup
driver.focControl.setTargetVelocity(1000); // Internal units

// Configure velocity PI gains
driver.focControl.setVelocityLoopGains(800, 1); // P=800, I=1

// Set velocity sensor
driver.focControl.setVelocitySensor(
    tmc9660::tmcl::VelocitySensorSelection::ABN_ENCODER);
```

### 4. Position Control

```cpp
// Position control
driver.focControl.setTargetPosition(50000); // Target position

// Configure position PI gains
driver.focControl.setPositionLoopGains(100, 0); // P=100, I=0

// Set position limits
driver.focControl.setPositionLimitLow(-100000);
driver.focControl.setPositionLimitHigh(100000);
```

### 5. Motion Ramp Control

```cpp
// Hardware acceleration/deceleration ramps
driver.ramp.enable(true);
driver.ramp.setAcceleration(1000, 2000, 5000); // A1, A2, Amax
driver.ramp.setDeceleration(1000, 2000, 5000); // D1, D2, Dmax
driver.ramp.setVelocities(0, 0, 500, 1000, 2000); // Vstart, Vstop, V1, V2, Vmax
```

---

## Telemetry and Monitoring

### 1. Real-time Telemetry

```cpp
// Read chip temperature
float chipTemp = driver.telemetry.getChipTemperature();

// Read motor current
int16_t motorCurrent = driver.telemetry.getMotorCurrent();

// Read supply voltage
float supplyVolt = driver.telemetry.getSupplyVoltage();

// Read actual position and velocity
int32_t actualPos, actualVel;
driver.focControl.getActualPosition(actualPos);
driver.focControl.getActualVelocity(actualVel);
```

### 2. RAMDebug System

The TMC9660 includes a powerful RAMDebug system for high-speed data logging:

```cpp
// Configure RAMDebug for current monitoring
driver.sendCommand(tmc9660::tmcl::Op::RamDebug, 
                  0x01, 0, 0); // Start logging
                  
// Read logged data
driver.sendCommand(tmc9660::tmcl::Op::RamDebug, 
                  0x02, 0, 0); // Stop logging
```

---

## Error Handling and Diagnostics

### 1. Communication Error Detection

```cpp
bool success = driver.motorConfig.setType(
    tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
if (!success) {
    // Handle communication failure
    // Check bus connection, power, etc.
}
```

### 2. Motor Protection

```cpp
// Configure protection limits
driver.protection.setOverVoltageLimit(48000); // 48V max
driver.protection.setUnderVoltageLimit(8000);  // 8V min
driver.protection.setOverTemperatureLimit(85); // 85°C max

// Monitor protection status
uint32_t faultStatus;
driver.readParameter(tmc9660::tmcl::Parameters::FAULT_STATUS, faultStatus);
```

### 3. Fault Recovery

```cpp
// Clear fault conditions
driver.sendCommand(tmc9660::tmcl::Op::CLE, 0xFF, 0, 0);

// Reset to safe state
driver.focControl.stop();
driver.motorConfig.setCommutationMode(
    tmc9660::tmcl::CommutationMode::SYSTEM_OFF);
```

---

## Best Practices

### 1. Initialization Sequence

1. **Configure bootloader** with proper communication settings
2. **Verify communication** with simple parameter read/write
3. **Set motor type** before any other motor parameters
4. **Configure feedback sensors** before enabling commutation
5. **Set current limits** before enabling motor drive
6. **Enable commutation mode** last

### 2. Real-time Operation

- **Check return values** of all driver calls
- **Monitor telemetry** for temperature and current
- **Implement watchdog** for communication timeouts
- **Use proper timing** for control loop updates

### 3. Safety Considerations

- **Always set current limits** before motor operation
- **Monitor chip temperature** continuously
- **Implement emergency stop** functionality
- **Validate sensor feedback** before closed-loop operation

### 4. Performance Optimization

- **Tune PI gains** systematically (current → velocity → position)
- **Use appropriate PWM frequency** for your application
- **Configure proper ADC settings** for current measurement
- **Minimize communication overhead** in real-time loops

---

## Conclusion

The hf-tmc9660-driver provides a comprehensive, hardware-agnostic interface to the TMC9660's parameter mode operation. The driver's architecture enables:

- **Clean separation** between hardware interface and motor control logic
- **Type-safe parameter access** through enumerated types
- **Modular configuration** via specialized subsystem interfaces
- **Robust error handling** and diagnostic capabilities

The initialization sequence from bootloader configuration through parameter mode setup is well-defined and enables reliable motor control across different hardware platforms. The driver's abstraction of TMCL commands provides a modern C++ interface while maintaining full access to the TMC9660's advanced features.

For optimal results, follow the documented initialization sequence, monitor telemetry data, and implement proper safety measures including current limits and temperature monitoring.