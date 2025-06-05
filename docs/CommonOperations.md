# Common HF-TMC9660 Operations

The following steps outline typical tasks when working with the driver
library.  They build upon the setup and interface documents.

---

## 1. Configure a Motor

Choose the motor type and commutation mode.  The driver exposes helper
functions for these parameters.

```cpp
TMC9660 driver(myBus);

driver.motor.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
driver.motor.setCommutationMode(
    tmc9660::tmcl::CommutationMode::FOC_HALL);
```

Adjust the parameters as required for your hardware.

---

## 2. Set Motion Targets

Use the ramp generator utilities to command velocity or position moves.

```cpp
driver.rampGenerator.setTargetVelocity(1500); // velocity in rpm
```

For position control:

```cpp
driver.position.moveTo(1000); // encoder units
```

All calls return a boolean indicating communication success.

---

## 3. Read Telemetry Data

The library can query temperature, current and voltage from the chip.

```cpp
float temp = 0.0f;
if (driver.monitor.getTemperature(temp)) {
    std::cout << "Chip temperature: " << temp << " C" << std::endl;
}
```

Refer to `inc/TMC9660.hpp` for the complete API.

