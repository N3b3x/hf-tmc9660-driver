# API Reference

Complete reference documentation for all public methods and types in the TMC9660 driver.

## Source Code

- **Main Header**: [`inc/tmc9660.hpp`](../inc/tmc9660.hpp)
- **Communication Interface**: [`inc/tmc9660_comm_interface.hpp`](../inc/tmc9660_comm_interface.hpp)
- **Bootloader**: [`inc/bootloader/tmc9660_bootloader.hpp`](../inc/bootloader/tmc9660_bootloader.hpp)
- **Parameter Mode**: [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp)

## TMC9660 Class

Main driver class for TMC9660 motor controller.

**Location**: [`inc/tmc9660.hpp#L117`](../inc/tmc9660.hpp#L117)

### Constructor

| Method | Signature | Location |
|--------|-----------|----------|
| `TMC9660()` | `TMC9660(CommInterface& comm, uint8_t address = 0, const BootloaderConfig* bootCfg = nullptr)` | [`inc/tmc9660.hpp#L130`](../inc/tmc9660.hpp#L130) |

### Core Methods

| Method | Signature | Location |
|--------|-----------|----------|
| `comm()` | `CommInterface& comm()` | [`inc/tmc9660.hpp#L139`](../inc/tmc9660.hpp#L139) |
| `bootloaderInit()` | `BootloaderInitResult bootloaderInit(const BootloaderConfig* cfg, bool performReset = true, bool retrieveBootloaderInfo = false, bool failOnVerifyError = true)` | [`inc/tmc9660.hpp#L212`](../inc/tmc9660.hpp#L212) |
| `getBootloader()` | `TMC9660Bootloader* getBootloader()` | [`inc/tmc9660.hpp#L248`](../inc/tmc9660.hpp#L248) |
| `enterBootloaderMode()` | `bool enterBootloaderMode()` | [`inc/tmc9660.hpp#L275`](../inc/tmc9660.hpp#L275) |
| `writeParameter()` | `bool writeParameter(tmcl::Parameters id, uint32_t value)` | [`inc/tmc9660.hpp#L296`](../inc/tmc9660.hpp#L296) |
| `readParameter()` | `bool readParameter(tmcl::Parameters id, uint32_t& value)` | [`inc/tmc9660.hpp#L306`](../inc/tmc9660.hpp#L306) |
| `writeGlobalParameter()` | `bool writeGlobalParameter(GlobalParamBankVariant id, uint8_t bank, uint32_t value)` | [`inc/tmc9660.hpp#L320`](../inc/tmc9660.hpp#L320) |
| `readGlobalParameter()` | `bool readGlobalParameter(GlobalParamBankVariant id, uint8_t bank, uint32_t& value)` | [`inc/tmc9660.hpp#L330`](../inc/tmc9660.hpp#L330) |
| `sendCommand()` | `bool sendCommand(tmcl::Op opcode, uint16_t type = 0, uint8_t motor = 0, uint32_t value = 0, uint32_t* reply = nullptr)` | [`inc/tmc9660.hpp#L334`](../inc/tmc9660.hpp#L334) |

### Enums

| Type | Values | Location |
|------|--------|----------|
| `BootloaderInitResult` | `Success`, `NoConfig`, `Failure` | [`inc/tmc9660.hpp#L153`](../inc/tmc9660.hpp#L153) |

## MotorConfig Subsystem

Motor configuration and control subsystem.

**Location**: [`inc/tmc9660.hpp#L361`](../inc/tmc9660.hpp#L361)

### Methods

| Method | Signature | Location |
|--------|-----------|----------|
| `setType()` | `bool setType(tmcl::MotorType type, uint8_t polePairs = 1)` | [`inc/tmc9660.hpp#L375`](../inc/tmc9660.hpp#L375) |
| `setDirection()` | `bool setDirection(tmcl::MotorDirection direction)` | [`inc/tmc9660.hpp#L384`](../inc/tmc9660.hpp#L384) |
| `setPWMFrequency()` | `bool setPWMFrequency(uint32_t frequencyHz)` | [`inc/tmc9660.hpp#L391`](../inc/tmc9660.hpp#L391) |
| `setCommutationMode()` | `bool setCommutationMode(tmcl::CommutationMode mode)` | [`inc/tmc9660.hpp#L423`](../inc/tmc9660.hpp#L423) |
| `setOutputVoltageLimit()` | `bool setOutputVoltageLimit(uint16_t limit)` | [`inc/tmc9660.hpp#L433`](../inc/tmc9660.hpp#L433) |
| `getOutputVoltageLimit()` | `bool getOutputVoltageLimit(uint16_t& limit)` | [`inc/tmc9660.hpp#L440`](../inc/tmc9660.hpp#L440) |
| `setMaxTorqueCurrent()` | `bool setMaxTorqueCurrent(uint16_t milliamps)` | [`inc/tmc9660.hpp#L452`](../inc/tmc9660.hpp#L452) |
| `setMaxFluxCurrent()` | `bool setMaxFluxCurrent(uint16_t milliamps)` | [`inc/tmc9660.hpp#L465`](../inc/tmc9660.hpp#L465) |
| `setPWMSwitchingScheme()` | `bool setPWMSwitchingScheme(tmcl::PwmSwitchingScheme scheme)` | [`inc/tmc9660.hpp#L484`](../inc/tmc9660.hpp#L484) |
| `setIdleMotorPWMBehavior()` | `bool setIdleMotorPWMBehavior(tmcl::IdleMotorPwmBehavior pwmOffWhenIdle)` | [`inc/tmc9660.hpp#L575`](../inc/tmc9660.hpp#L575) |
| `configureAuto()` | `bool configureAuto(const MotorProfile& profile)` | [`inc/tmc9660.hpp#L639`](../inc/tmc9660.hpp#L639) |

## FOCControl Subsystem

Field-Oriented Control subsystem.

**Location**: [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp)

### Methods

| Method | Signature | Location |
|--------|-----------|----------|
| `setTargetVelocity()` | `bool setTargetVelocity(int32_t velocity)` | See source |
| `setTargetTorque()` | `bool setTargetTorque(int32_t torque)` | See source |
| `setTargetPosition()` | `bool setTargetPosition(int32_t position)` | See source |

## Telemetry Subsystem

Telemetry and monitoring subsystem.

**Location**: [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp)

### Methods

| Method | Signature | Location |
|--------|-----------|----------|
| `getTemperature()` | `bool getTemperature(int16_t& temp)` | See source |
| `getCurrent()` | `bool getCurrent(int16_t& current)` | See source |
| `getVoltage()` | `bool getVoltage(uint16_t& voltage)` | See source |
| `getPosition()` | `bool getPosition(int32_t& position)` | See source |

## Communication Interfaces

### SpiCommInterface

SPI communication interface base class.

**Location**: [`inc/tmc9660_comm_interface.hpp`](../inc/tmc9660_comm_interface.hpp)

| Method | Signature | Location |
|--------|-----------|----------|
| `spiTransfer()` | `bool spiTransfer(std::array<uint8_t,8>& tx, std::array<uint8_t,8>& rx)` | See source |

### UartCommInterface

UART communication interface base class.

**Location**: [`inc/tmc9660_comm_interface.hpp`](../inc/tmc9660_comm_interface.hpp)

| Method | Signature | Location |
|--------|-----------|----------|
| `uartTransfer()` | `bool uartTransfer(const TMCLFrame& tx, TMCLReply& reply, uint8_t address)` | See source |

## Types

### Enums

| Type | Location |
|------|----------|
| `tmcl::MotorType` | [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp) |
| `tmcl::CommutationMode` | [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp) |
| `tmcl::MotorDirection` | [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp) |
| `tmcl::PwmSwitchingScheme` | [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp) |
| `tmcl::Parameters` | [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp) |
| `tmcl::Op` | [`inc/parameter_mode/tmc9660_param_mode_tmcl.hpp`](../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp) |
| `bootcfg::BootMode` | [`inc/bootloader/bootloader_config.hpp`](../inc/bootloader/bootloader_config.hpp) |

### Structs

| Type | Location |
|------|----------|
| `BootloaderConfig` | [`inc/bootloader/bootloader_config.hpp`](../inc/bootloader/bootloader_config.hpp) |
| `MotorProfile` | [`inc/tmc9660.hpp`](../inc/tmc9660.hpp) |
| `TMCLFrame` | [`inc/tmc9660_comm_interface.hpp`](../inc/tmc9660_comm_interface.hpp) |
| `TMCLReply` | [`inc/tmc9660_comm_interface.hpp`](../inc/tmc9660_comm_interface.hpp) |

