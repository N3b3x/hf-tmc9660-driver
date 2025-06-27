# Extensive TMC9660 Examples

This document collects hardware agnostic walkthroughs showing how to configure
the TMC9660 from bootloader setup to running a variety of motors.  All code
examples use a very small "DummyBus" that merely echoes SPI transfers.  When
porting the library to real hardware you must implement your own class derived
from `TMC9660CommInterface` and replace the dummy bus with that implementation.

---

## Bootloader Configuration

The bootloader registers are configured via `TMC9660Bootloader`. The snippet
below writes a minimal configuration enabling SPI and UART communication.  The
`DummyBus` seen in all snippets simply echoes data and serves as a placeholder
for your real SPI or UART implementation.

```cpp
#include "TMC9660.hpp"
#include <iostream>

class DummyBus : public SPITMC9660CommInterface {
public:
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        rx = tx; // echo back for demonstration
        return true;
    }
};

int main() {
    DummyBus bus;
    TMC9660 driver(bus);

    tmc9660::BootloaderConfig cfg{};
    cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;
    cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
    cfg.clock.use_external = tmc9660::bootcfg::ClockSource::Internal;

    auto result = driver.bootloaderInit(&cfg);
    if (result == TMC9660::BootloaderInitResult::Success)
        std::cout << "Bootloader configured" << std::endl;
    else
        std::cout << "Bootloader configuration failed" << std::endl;
}
```

Compile with:

```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    docs/bootloader_example.cpp -o bootloader_demo
```

---

## BLDC with ABN Encoder

This example configures a BLDC motor using an incremental ABN encoder for closed
loop FOC control.  Again the `DummyBus` merely mimics the real transport layer.

```cpp
#include "TMC9660.hpp"
#include <iostream>

class DummyBus : public SPITMC9660CommInterface {
public:
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        rx = tx;
        return true;
    }
};

int main() {
    DummyBus bus;
    TMC9660 driver(bus);

    driver.motor.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
    driver.motor.configureABNEncoder(2048);
    driver.motor.setCommutationMode(
        tmc9660::tmcl::CommutationMode::FOC_ENCODER);
    driver.motor.setMaxTorqueCurrent(2000);
    driver.rampGenerator.setTargetVelocity(1500);

    std::cout << "Motor running with ABN feedback" << std::endl;
}
```

---

## Stepper Closed‑Loop Control

The library can control a stepper motor using field‑oriented control. The
following snippet sets up a stepper with an ABN encoder and commands a position
move.  As before, `DummyBus` is just a stand‑in for a real bus driver.

```cpp
#include "TMC9660.hpp"
#include <iostream>

class DummyBus : public SPITMC9660CommInterface {
public:
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        rx = tx;
        return true;
    }
};

int main() {
    DummyBus bus;
    TMC9660 driver(bus);

    driver.motor.setType(tmc9660::tmcl::MotorType::STEPPER_MOTOR);
    driver.motor.configureABNEncoder(4000);
    driver.motor.setCommutationMode(
        tmc9660::tmcl::CommutationMode::FOC_ENCODER);
    driver.position.moveTo(1000);
    std::cout << "Stepper moving to position 1000" << std::endl;
}
```

---

## Step/Dir Interface

External controllers can drive the STEP/DIR pins while the TMC9660 extrapolates
between pulses. Here a stepper motor is configured for 1/8 micro‑steps and the
interface is enabled.  `DummyBus` once again represents the application‑specific
communication layer.

```cpp
#include "TMC9660.hpp"
#include <iostream>

class DummyBus : public SPITMC9660CommInterface {
public:
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        rx = tx;
        return true;
    }
};

int main() {
    DummyBus bus;
    TMC9660 driver(bus);

    driver.motor.setType(tmc9660::tmcl::MotorType::STEPPER_MOTOR);
    driver.stepDir.setMicrostepResolution(
        tmc9660::tmcl::StepDirStepDividerShift::DIV8);
    driver.stepDir.enableInterface(true);
    driver.stepDir.enableExtrapolation(true);

    std::cout << "STEP/DIR interface enabled" << std::endl;
}
```

---

## DC Motor Current Control

A brushed DC motor can be driven using open‑loop current mode. The following
example sets a current limit and commands a constant current.  Replace
`DummyBus` with your hardware specific implementation when integrating.

```cpp
#include "TMC9660.hpp"
#include <iostream>

class DummyBus : public SPITMC9660CommInterface {
public:
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override {
        rx = tx;
        return true;
    }
};

int main() {
    DummyBus bus;
    TMC9660 driver(bus);

    driver.motor.setType(tmc9660::tmcl::MotorType::DC_MOTOR);
    driver.motor.setCommutationMode(
        tmc9660::tmcl::CommutationMode::FOC_OPENLOOP_CURRENT);
    driver.motor.setMaxTorqueCurrent(1000);
    driver.rampGenerator.setTargetCurrent(500);

    std::cout << "DC motor running in current mode" << std::endl;
}
```

These snippets are minimal but cover the main features of the library. Refer to `inc/TMC9660.hpp` for all available parameters and helper methods.


---

[⬅️ Prev](BuildingExamples.md) | [⬆️ Back to Index](index.md) | [Next ➡️](CommonOperations.md)
