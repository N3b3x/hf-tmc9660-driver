---
layout: default
title: Common HF-TMC9660 Operations
---

## 🛠️ Common HF-TMC9660 Operations

This guide covers the most frequently used operations when working with the
HF-TMC9660 driver. Each example includes proper error handling and follows best
practices for robust motor control applications.

---

## 🎯 What You'll Learn

- ✅ Essential initialization patterns
- ✅ Motor configuration for different types
- ✅ FOC control operations (torque, velocity, position)
- ✅ Telemetry and monitoring
- ✅ Safety and protection setup
- ✅ Error handling best practices

---

## 🚀 Essential Initialization Pattern

**Every application must follow this initialization sequence:**

```cpp
#include "TMC9660.hpp"

// Your communication interface implementation
YourCommInterface bus;
TMC9660 driver(bus);

// STEP 1: CRITICAL - Configure for Parameter Mode
tmc9660::BootloaderConfig cfg{};
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;  // ESSENTIAL!
cfg.boot.start_motor_control = true;
cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;

auto result = driver.bootloaderInit(&cfg);
if (result != TMC9660::BootloaderInitResult::Success) {
    // Handle bootloader failure - motor control won't work without this
    return -1;
}

// Now you can use all motor control functions...
```

---

## 🔧 Motor Configuration

### BLDC Motor Setup

```cpp
bool configureBLDCMotor(TMC9660& driver) {
    // Step 1: Set motor type and pole pairs
    if (!driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7)) {
        std::cerr << "Failed to set BLDC motor type" << std::endl;
        return false;
    }
    
    // Step 2: Configure current limits (SAFETY FIRST!)
    if (!driver.motorConfig.setMaxTorqueCurrent(2000)) {  // 2A max
        std::cerr << "Failed to set torque current limit" << std::endl;
        return false;
    }
    
    if (!driver.motorConfig.setMaxFluxCurrent(1000)) {    // 1A flux
        std::cerr << "Failed to set flux current limit" << std::endl;
        return false;
    }
    
    // Step 3: Configure feedback sensors
    if (!driver.feedbackSense.configureHall()) {
        std::cerr << "Failed to configure Hall sensors" << std::endl;
        return false;
    }
    
    // Step 4: Set PWM frequency and switching scheme
    if (!driver.motorConfig.setPWMFrequency(25000)) {     // 25 kHz
        std::cerr << "Failed to set PWM frequency" << std::endl;
        return false;
    }
    
    if (!driver.motorConfig.setPWMSwitchingScheme(
            tmc9660::tmcl::PwmSwitchingScheme::SVPWM)) {
        std::cerr << "Failed to set SVPWM scheme" << std::endl;
        return false;
    }
    
    // Step 5: Configure FOC control gains
    if (!driver.focControl.setCurrentLoopGains(50, 100)) {
        std::cerr << "Failed to set current PI gains" << std::endl;
        return false;
    }
    
    if (!driver.focControl.setVelocityLoopGains(800, 1)) {
        std::cerr << "Failed to set velocity PI gains" << std::endl;
        return false;
    }
    
    // Step 6: Enable commutation (LAST STEP!)
    if (!driver.motorConfig.setCommutationMode(
            tmc9660::tmcl::CommutationMode::FOC_HALL_SENSOR)) {
        std::cerr << "Failed to enable FOC commutation" << std::endl;
        return false;
    }
    
    std::cout << "✅ BLDC motor configured successfully" << std::endl;
    return true;
}
```

### Stepper Motor Setup

```cpp
bool configureStepperMotor(TMC9660& driver) {
    // Step 1: Set motor type (steppers use 1 pole pair)
    if (!driver.motorConfig.setType(tmc9660::tmcl::MotorType::STEPPER_MOTOR, 1)) {
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

### DC Motor Setup

```cpp
bool configureDCMotor(TMC9660& driver) {
    // Step 1: Set motor type
    if (!driver.motorConfig.setType(tmc9660::tmcl::MotorType::DC_MOTOR)) {
        return false;
    }
    
    // Step 2: Set current limit
    if (!driver.motorConfig.setMaxTorqueCurrent(1500)) {    // 1.5A
        return false;
    }
    
    // Step 3: Configure encoder for velocity feedback
    if (!driver.feedbackSense.configureABNEncoder(1024)) {
        return false;
    }
    
    // Step 4: Enable open-loop current mode
    if (!driver.motorConfig.setCommutationMode(
            tmc9660::tmcl::CommutationMode::FOC_OPENLOOP_CURRENT_MODE)) {
        return false;
    }
    
    return true;
}
```

---

## ⚡ FOC Control Operations

### Velocity Control

```cpp
bool setVelocityControl(TMC9660& driver, int32_t target_rpm) {
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
    
    std::cout << "Velocity control: " << target_rpm << " RPM" << std::endl;
    return true;
}

// Monitor velocity control
void monitorVelocity(TMC9660& driver) {
    int32_t actual_velocity = 0;
    if (driver.focControl.getActualVelocity(actual_velocity)) {
        std::cout << "Current velocity: " << actual_velocity << " RPM" << std::endl;
    }
    
    // Check if velocity target is reached
    uint32_t threshold = 0;
    driver.focControl.getVelocityReachedThreshold(threshold);
    // Implement your logic based on threshold
}
```

### Position Control

```cpp
bool setPositionControl(TMC9660& driver, int32_t target_position) {
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
    
    std::cout << "Position control: " << target_position << " units" << std::endl;
    return true;
}
```

### Torque Control

```cpp
bool setTorqueControl(TMC9660& driver, int16_t target_current_ma) {
    // Set current PI gains
    if (!driver.focControl.setCurrentLoopGains(50, 100)) {
        return false;
    }
    
    // Command target torque (current)
    if (!driver.focControl.setTargetTorque(target_current_ma)) {
        return false;
    }
    
    std::cout << "Torque control: " << target_current_ma << " mA" << std::endl;
    return true;
}
```

---

## 📊 Telemetry and Monitoring

### Basic Telemetry Reading

```cpp
struct TelemetryData {
    float chip_temperature;
    int16_t motor_current;
    float supply_voltage;
    int32_t actual_position;
    int32_t actual_velocity;
};

bool readTelemetry(TMC9660& driver, TelemetryData& data) {
    // Read chip temperature
    data.chip_temperature = driver.telemetry.getChipTemperature();
    
    // Read motor current
    data.motor_current = driver.telemetry.getMotorCurrent();
    
    // Read supply voltage
    data.supply_voltage = driver.telemetry.getSupplyVoltage();
    
    // Read actual position and velocity
    bool pos_ok = driver.focControl.getActualPosition(data.actual_position);
    bool vel_ok = driver.focControl.getActualVelocity(data.actual_velocity);
    
    return pos_ok && vel_ok;
}

void printTelemetry(const TelemetryData& data) {
    std::cout << "Telemetry:" << std::endl;
    std::cout << "  Temperature: " << data.chip_temperature << " °C" << std::endl;
    std::cout << "  Current:     " << data.motor_current << " mA" << std::endl;
    std::cout << "  Voltage:     " << data.supply_voltage << " V" << std::endl;
    std::cout << "  Position:    " << data.actual_position << " units" << std::endl;
    std::cout << "  Velocity:    " << data.actual_velocity << " RPM" << std::endl;
}
```

### Advanced Current Sensing

```cpp
bool configureCurrentSensing(TMC9660& driver) {
    // Set ADC shunt configuration
    if (!driver.currentSensing.setShuntType(
            tmc9660::tmcl::AdcShuntType::THREE_SHUNT)) {
        return false;
    }
    
    // Configure current sense amplifier gains
    if (!driver.currentSensing.setCSAGain(
            tmc9660::tmcl::CsaGain::GAIN_20, 
            tmc9660::tmcl::CsaGain::GAIN_20)) {
        return false;
    }
    
    // Set scaling factor
    if (!driver.currentSensing.setScalingFactor(1000)) {
        return false;
    }
    
    // Calibrate ADC offsets (motor must be stationary!)
    if (!driver.currentSensing.calibrateOffsets(true, 1000)) {
        std::cerr << "ADC calibration failed" << std::endl;
        return false;
    }
    
    std::cout << "✅ Current sensing configured and calibrated" << std::endl;
    return true;
}

void readDetailedCurrents(TMC9660& driver) {
    // Read raw ADC values
    int16_t adc0, adc1, adc2, adc3;
    if (driver.currentSensing.readRaw(adc0, adc1, adc2, adc3)) {
        std::cout << "Raw ADC: " << adc0 << ", " << adc1 << ", " 
                  << adc2 << ", " << adc3 << std::endl;
    }
    
    // Read scaled and offset-compensated values
    if (driver.currentSensing.readScaledAndOffset(adc0, adc1, adc2, adc3)) {
        std::cout << "Scaled ADC: " << adc0 << ", " << adc1 << ", " 
                  << adc2 << ", " << adc3 << std::endl;
    }
}
```

---

## 🛡️ Safety and Protection

### Configure Protection Limits

```cpp
bool configureProtection(TMC9660& driver) {
    // Voltage protection
    if (!driver.protection.setOverVoltageLimit(48000)) {    // 48V max
        return false;
    }
    
    if (!driver.protection.setUnderVoltageLimit(8000)) {    // 8V min
        return false;
    }
    
    // Temperature protection
    if (!driver.protection.setOverTemperatureLimit(85)) {   // 85°C max
        return false;
    }
    
    // Current protection (already set in motor config, but can be adjusted)
    if (!driver.motorConfig.setMaxTorqueCurrent(2000)) {    // 2A max
        return false;
    }
    
    std::cout << "✅ Protection limits configured" << std::endl;
    return true;
}

bool checkFaultStatus(TMC9660& driver) {
    uint32_t fault_status = 0;
    if (!driver.readParameter(tmc9660::tmcl::Parameters::FAULT_STATUS, fault_status)) {
        return false;
    }
    
    if (fault_status != 0) {
        std::cout << "⚠️  Fault detected: 0x" << std::hex << fault_status << std::endl;
        
        // Clear faults if appropriate
        if (driver.sendCommand(tmc9660::tmcl::Op::CLE, 0xFF, 0, 0)) {
            std::cout << "Faults cleared" << std::endl;
        }
        
        return false; // Indicate fault condition
    }
    
    return true; // No faults
}
```

---

## 🎛️ Motion Ramp Control

### Configure Hardware Ramps

```cpp
bool configureMotionRamps(TMC9660& driver) {
    // Enable ramp generator
    if (!driver.ramp.enable(true)) {
        return false;
    }
    
    // Set acceleration parameters (µ units/s²)
    if (!driver.ramp.setAcceleration(1000, 2000, 5000)) {  // A1, A2, Amax
        return false;
    }
    
    // Set deceleration parameters (µ units/s²)  
    if (!driver.ramp.setDeceleration(1000, 2000, 5000)) {  // D1, D2, Dmax
        return false;
    }
    
    // Set velocity parameters (µ units/s)
    // Vstart, Vstop, V1, V2, Vmax
    if (!driver.ramp.setVelocities(0, 0, 500, 1000, 2000)) {
        return false;
    }
    
    // Enable feedforward control
    if (!driver.ramp.enableFeedForward(
            true, true, 1000, 
            tmc9660::tmcl::AccelerationFFShift::SHIFT_0)) {
        return false;
    }
    
    std::cout << "✅ Motion ramps configured" << std::endl;
    return true;
}

void monitorRampStatus(TMC9660& driver) {
    int32_t ramp_velocity = 0;
    int32_t ramp_position = 0;
    
    if (driver.ramp.getRampVelocity(ramp_velocity)) {
        std::cout << "Ramp velocity: " << ramp_velocity << std::endl;
    }
    
    if (driver.ramp.getRampPosition(ramp_position)) {
        std::cout << "Ramp position: " << ramp_position << std::endl;
    }
}
```

---

## 🚨 Emergency Stop and Safe Shutdown

### Emergency Stop Procedure

```cpp
bool emergencyStop(TMC9660& driver) {
    std::cout << "🚨 EMERGENCY STOP ACTIVATED!" << std::endl;
    
    // Immediate motor stop
    if (!driver.focControl.stop()) {
        std::cerr << "Failed to stop motor!" << std::endl;
        return false;
    }
    
    // Disable commutation
    if (!driver.motorConfig.setCommutationMode(
            tmc9660::tmcl::CommutationMode::SYSTEM_OFF)) {
        std::cerr << "Failed to disable commutation!" << std::endl;
        return false;
    }
    
    // Check for faults
    checkFaultStatus(driver);
    
    std::cout << "✅ Emergency stop completed" << std::endl;
    return true;
}
```

### Graceful Shutdown

```cpp
bool gracefulShutdown(TMC9660& driver) {
    std::cout << "Initiating graceful shutdown..." << std::endl;
    
    // Ramp down to zero velocity
    if (!driver.focControl.setTargetVelocity(0)) {
        return false;
    }
    
    // Wait for motor to stop (implement your own timeout logic)
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    // Disable commutation
    if (!driver.motorConfig.setCommutationMode(
            tmc9660::tmcl::CommutationMode::SYSTEM_OFF)) {
        return false;
    }
    
    std::cout << "✅ Graceful shutdown completed" << std::endl;
    return true;
}
```

---

## 📋 Best Practices Checklist

### Initialization

- [ ] **Bootloader configured** for Parameter Mode
- [ ] **Communication tested** with simple parameter read/write
- [ ] **Motor type set** before other motor parameters
- [ ] **Current limits configured** before enabling commutation
- [ ] **Feedback sensors configured** properly

### Operation

- [ ] **Error checking** on all driver calls
- [ ] **Telemetry monitoring** for temperature and current
- [ ] **Fault status checking** periodically
- [ ] **Emergency stop** procedure implemented
- [ ] **Graceful shutdown** on application exit

### Safety

- [ ] **Voltage limits** set appropriately
- [ ] **Current limits** set conservatively
- [ ] **Temperature monitoring** enabled
- [ ] **Position limits** configured if applicable
- [ ] **Watchdog timer** implemented in your application

---

## 🎯 Next Steps

With these common operations mastered, explore advanced features:

**👉 [Hardware-Agnostic Examples](HardwareAgnosticExamples.html)** - Complete
application scenarios

**👉 [API Reference](annotated.html)** - Complete function documentation

---

[⬅️ Building Examples](BuildingExamples.html) | [⬆️ Back to Index](index.html) | [Next ➡️ GitHub Pages](HostingDocsWithGitHubPages.html)

---

*Remember: Always check return values and implement proper error handling in
production code!*
