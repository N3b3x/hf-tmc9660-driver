# Common HF-TMC9660 Operations 🔧

The snippets below highlight common patterns when working with the driver.
They assume you have already followed the setup guide and created your own
communication interface.

---

## 1. Configure a Motor

Choose the motor type and commutation mode. The driver exposes helper
functions for these parameters so you rarely need to write raw register
values. A typical initialization looks like this:

```cpp
TMC9660 driver(myBus);

driver.motor.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
driver.motor.setCommutationMode(
    tmc9660::tmcl::CommutationMode::FOC_HALL);
```

Adjust the parameters as required for your hardware. For example you may choose
`FOC_ENCODER` when using an incremental encoder or set the number of motor
poles to match your BLDC.

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

All calls return a boolean indicating communication success. You can poll the
status or read back the position to monitor progress.

---

## 3. Read Telemetry Data

The library can query temperature, current and voltage from the chip.

```cpp
float temp = 0.0f;
if (driver.monitor.getTemperature(temp)) {
    std::cout << "Chip temperature: " << temp << " C" << std::endl;
}
```

Refer to `inc/TMC9660.hpp` for the complete API and additional helper
functions. With these building blocks you can integrate the driver into your
motion control software and start experimenting right away.


---

[⬅️ Prev](HardwareAgnosticExamples.md) | [⬆️ Back to Index](index.md)
