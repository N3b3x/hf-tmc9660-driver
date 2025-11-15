---
layout: default
title: "💡 Examples"
description: "Complete example walkthroughs for the TMC9660 driver"
nav_order: 7
parent: "📚 Documentation"
permalink: /docs/examples/
---

# Examples

This guide provides detailed examples for various TMC9660 use cases.

## Basic BLDC Motor Control

```cpp
#include "inc/tmc9660.hpp"

class MySPI : public tmc9660::SpiCommInterface<MySPI> {
    // ... implement required methods (spiTransferTMCL, gpioSet, etc.) ...
};

int main() {
    MySPI spi;
    tmc9660::TMC9660<MySPI> driver(spi);
    
    // 1. Initialize bootloader
    tmc9660::BootloaderConfig cfg{};
    cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    cfg.boot.start_motor_control = true;
    
    if (driver.bootloaderInit(&cfg) != 
        tmc9660::TMC9660::BootloaderInitResult::Success) {
        return -1;
    }
    
    // 2. Configure motor
    driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
    driver.motorConfig.setMaxTorqueCurrent(2000);
    driver.motorConfig.setPWMFrequency(20000);
    
    // 3. Configure feedback
    driver.feedbackSense.configureHall();
    driver.motorConfig.setCommutationMode(
        tmc9660::tmcl::CommutationMode::FOC_HALL);
    
    // 4. Start motor
    driver.focControl.setTargetVelocity(1000); // 1000 RPM
    
    return 0;
}
```

## BLDC with ABN Encoder

```cpp
// Configure BLDC motor with incremental encoder
driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
driver.feedbackSense.configureABNEncoder(2048); // 2048 counts/rev
driver.motorConfig.setCommutationMode(
    tmc9660::tmcl::CommutationMode::FOC_ENCODER);

// Set target velocity
driver.focControl.setTargetVelocity(1500);
```

## Stepper Motor Control

```cpp
// Configure stepper motor
driver.motorConfig.setType(tmc9660::tmcl::MotorType::STEPPER_MOTOR);
driver.feedbackSense.configureABNEncoder(4000);
driver.motorConfig.setCommutationMode(
    tmc9660::tmcl::CommutationMode::FOC_ENCODER);

// Move to position
driver.position.moveTo(1000);
```

## Step/Dir Interface

```cpp
// Configure stepper for step/direction interface
driver.motorConfig.setType(tmc9660::tmcl::MotorType::STEPPER_MOTOR);
driver.stepDir.setMicrosteps(8); // 1/8 microstepping
driver.stepDir.enable();
```

## Telemetry Reading

```cpp
// Read motor temperature
int16_t temperature;
if (driver.telemetry.getTemperature(temperature)) {
    printf("Temperature: %d°C\n", temperature);
}

// Read motor current
int16_t current;
if (driver.telemetry.getCurrent(current)) {
    printf("Current: %d mA\n", current);
}

// Read position
int32_t position;
if (driver.telemetry.getPosition(position)) {
    printf("Position: %ld\n", position);
}
```

## Error Handling

```cpp
bool configureMotor(tmc9660::TMC9660& driver) {
    // Set motor type
    if (!driver.motorConfig.setType(
            tmc9660::tmcl::MotorType::BLDC_MOTOR, 7)) {
        printf("Failed to set motor type\n");
        return false;
    }
    
    // Set current limits
    if (!driver.motorConfig.setMaxTorqueCurrent(2000)) {
        printf("Failed to set torque current\n");
        return false;
    }
    
    return true;
}
```

## Advanced: Custom Parameter Access

```cpp
// Write custom parameter
if (!driver.writeParameter(
        tmc9660::tmcl::Parameters::MOTOR_TYPE, 
        static_cast<uint32_t>(tmc9660::tmcl::MotorType::BLDC_MOTOR))) {
    printf("Failed to write parameter\n");
}

// Read custom parameter
uint32_t value;
if (driver.readParameter(tmc9660::tmcl::Parameters::MOTOR_TYPE, value)) {
    printf("Motor type: %lu\n", value);
}
```

## Complete Motor Configuration Examples

### BLDC Motor with Hall Sensors

```cpp
bool configureBLDCMotor(tmc9660::TMC9660& driver) {
    // Step 1: Set motor type and pole pairs
    if (!driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7)) {
        return false;
    }
    
    // Step 2: Configure current limits (SAFETY FIRST!)
    if (!driver.motorConfig.setMaxTorqueCurrent(2000)) {  // 2A max
        return false;
    }
    
    if (!driver.motorConfig.setMaxFluxCurrent(1000)) {    // 1A flux
        return false;
    }
    
    // Step 3: Configure feedback sensors
    if (!driver.feedbackSense.configureHall()) {
        return false;
    }
    
    // Step 4: Set PWM frequency and switching scheme
    if (!driver.motorConfig.setPWMFrequency(25000)) {     // 25 kHz
        return false;
    }
    
    if (!driver.motorConfig.setPWMSwitchingScheme(
            tmc9660::tmcl::PwmSwitchingScheme::SVPWM)) {
        return false;
    }
    
    // Step 5: Configure FOC control gains
    if (!driver.focControl.setCurrentLoopGains(50, 100)) {
        return false;
    }
    
    if (!driver.focControl.setVelocityLoopGains(800, 1)) {
        return false;
    }
    
    // Step 6: Enable commutation (LAST STEP!)
    if (!driver.motorConfig.setCommutationMode(
            tmc9660::tmcl::CommutationMode::FOC_HALL)) {
        return false;
    }
    
    return true;
}
```

### Stepper Motor with Encoder

```cpp
bool configureStepperMotor(tmc9660::TMC9660& driver) {
    // Step 1: Set motor type (steppers use 1 pole pair)
    if (!driver.motorConfig.setType(
            tmc9660::tmcl::MotorType::STEPPER_MOTOR, 1)) {
        return false;
    }
    
    // Step 2: Configure encoder for feedback
    if (!driver.feedbackSense.configureABNEncoder(1024)) {  // 1024 CPR
        return false;
    }
    
    // Step 3: Set current limits
    if (!driver.motorConfig.setMaxTorqueCurrent(1500)) {    // 1.5A
        return false;
    }
    
    // Step 4: Configure for FOC with encoder feedback
    if (!driver.motorConfig.setCommutationMode(
            tmc9660::tmcl::CommutationMode::FOC_ENCODER)) {
        return false;
    }
    
    return true;
}
```

### Velocity Control

```cpp
bool setVelocityControl(tmc9660::TMC9660& driver, int32_t target_rpm) {
    // Configure velocity sensor
    if (!driver.focControl.setVelocitySensor(
            tmc9660::tmcl::VelocitySensorSelection::ABN1_ENCODER)) {
        return false;
    }
    
    // Set velocity PI gains (tune these for your system)
    if (!driver.focControl.setVelocityLoopGains(800, 1)) {
        return false;
    }
    
    // Command target velocity
    if (!driver.focControl.setTargetVelocity(target_rpm)) {
        return false;
    }
    
    return true;
}
```

### Position Control

```cpp
bool setPositionControl(tmc9660::TMC9660& driver, int32_t target_position) {
    // Configure position sensor
    if (!driver.focControl.setPositionSensor(
            tmc9660::tmcl::PositionSensorSelection::ABN1_ENCODER)) {
        return false;
    }
    
    // Set position PI gains
    if (!driver.focControl.setPositionLoopGains(100, 0)) {  // P=100, I=0
        return false;
    }
    
    // Set position limits for safety
    if (!driver.focControl.setPositionLimitLow(-100000)) {
        return false;
    }
    
    if (!driver.focControl.setPositionLimitHigh(100000)) {
        return false;
    }
    
    // Command target position
    if (!driver.focControl.setTargetPosition(target_position)) {
        return false;
    }
    
    return true;
}
```

## Next Steps

- **[Quick Start](quickstart.md)** - Minimal working example
- **[API Reference](api_reference.md)** - Complete API documentation
- **[Hardware Setup](hardware_setup.md)** - Wiring guide

