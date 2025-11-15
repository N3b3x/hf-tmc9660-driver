---
layout: default
title: "⚙️ Configuration"
description: "Configuration options for the TMC9660 driver"
nav_order: 5
parent: "📚 Documentation"
permalink: /docs/configuration/
---

# Configuration

This guide covers configuration options for the TMC9660 driver.

## Bootloader Configuration

The bootloader must be configured for Parameter Mode operation. This is **CRITICAL** and must be done before any motor control.

```cpp
tmc9660::BootloaderConfig cfg{};

// CRITICAL: Set boot mode to Parameter
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;

// Start motor control after configuration
cfg.boot.start_motor_control = true;

// SPI configuration
cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;

// UART configuration (if using UART)
cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;
cfg.uart.tx_pin = tmc9660::bootcfg::UartTxPin::GPIO6;
cfg.uart.rx_pin = tmc9660::bootcfg::UartRxPin::GPIO7;

// Clock configuration
cfg.clock.use_external = tmc9660::bootcfg::ClockSource::Internal;

// Initialize bootloader
auto result = driver.bootloaderInit(&cfg);
```

## Motor Configuration

### Motor Type and Pole Pairs

```cpp
// BLDC motor with 7 pole pairs
driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);

// Stepper motor
driver.motorConfig.setType(tmc9660::tmcl::MotorType::STEPPER_MOTOR);

// DC motor
driver.motorConfig.setType(tmc9660::tmcl::MotorType::DC_MOTOR);
```

### Current Limits

```cpp
// Set maximum torque current (2A)
driver.motorConfig.setMaxTorqueCurrent(2000);

// Set maximum flux current (1A)
driver.motorConfig.setMaxFluxCurrent(1000);
```

### PWM Configuration

```cpp
// Set PWM frequency (20 kHz)
driver.motorConfig.setPWMFrequency(20000);

// Set PWM switching scheme
driver.motorConfig.setPWMSwitchingScheme(
    tmc9660::tmcl::PwmSwitchingScheme::SVPWM);
```

### Commutation Mode

```cpp
// FOC with Hall sensors
driver.motorConfig.setCommutationMode(
    tmc9660::tmcl::CommutationMode::FOC_HALL);

// FOC with encoder
driver.motorConfig.setCommutationMode(
    tmc9660::tmcl::CommutationMode::FOC_ENCODER);

// Open-loop
driver.motorConfig.setCommutationMode(
    tmc9660::tmcl::CommutationMode::OPEN_LOOP);
```

## FOC Control Configuration

### Control Loop Gains

```cpp
// Set current loop gains (Kp, Ki)
driver.focControl.setCurrentLoopGains(50, 100);

// Set velocity loop gains
driver.focControl.setVelocityLoopGains(100, 200);

// Set position loop gains
driver.focControl.setPositionLoopGains(10, 20);
```

## Sensor Configuration

### Hall Sensors

```cpp
driver.feedbackSense.configureHall();
```

### ABN Encoder

```cpp
// Configure incremental encoder (2048 counts per revolution)
driver.feedbackSense.configureABNEncoder(2048);
```

## Protection Configuration

### Undervoltage Protection

```cpp
driver.protection.configureUndervoltageProtection(
    tmc9660::tmcl::UndervoltageLevel::Level_12V,
    tmc9660::tmcl::UndervoltageEnable::ENABLED,
    tmc9660::tmcl::UndervoltageEnable::ENABLED,
    tmc9660::tmcl::UndervoltageEnable::ENABLED);
```

### Overcurrent Protection

```cpp
driver.protection.configureOvercurrentProtection(
    tmc9660::tmcl::OvercurrentEnable::ENABLED,
    tmc9660::tmcl::OvercurrentEnable::ENABLED,
    tmc9660::tmcl::OvercurrentEnable::ENABLED);
```

## Default Values

Most configuration methods have sensible defaults. Refer to the [API Reference](api_reference.md) for specific default values.

## Next Steps

- **[Examples](examples.md)** - See configuration in action
- **[Bootloader Initialization](special_feature_bootloader.md)** - Critical bootloader setup

