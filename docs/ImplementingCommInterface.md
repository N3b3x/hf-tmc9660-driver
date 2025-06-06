# Implementing a Custom Communication Interface

To keep the driver portable it does not ship with any hardware specific code.
Instead you plug in your own class that knows how to talk over SPI or UART. The
steps below show how to write that class and hook it up to the library.

The library itself does not contain any hardware specific transfer code.
Instead you provide a class derived from `TMC9660CommInterface` that
delivers the 8 byte datagrams over SPI or UART.

This document explains the required functions and provides a skeleton
implementation you can adapt for your platform.

---

## 1. Derive from `TMC9660CommInterface`

Create a new C++ class that inherits from either
`SPITMC9660CommInterface` or `UARTTMC9660CommInterface`. The choice depends on
whether your hardware uses SPI or UART for communication.

```cpp
#include "TMC9660CommInterface.hpp"

class MyInterface : public SPITMC9660CommInterface {
public:
    bool spiTransfer(std::array<uint8_t,8>& tx,
                     std::array<uint8_t,8>& rx) noexcept override;
};
```

Only a single method must be implemented. It should transmit the contents of
`tx` and fill `rx` with the reply from the device. Return `true` on success and
`false` if any error occurred. If communication fails you can use this return
value to trigger a retry or error message in your application.

---

## 2. Provide the Transfer Logic

Inside `spiTransfer()` perform the actual byte exchange.  For example on
an embedded MCU you might toggle chip select, call into your SPI driver
and then return the received bytes.

```cpp
bool MyInterface::spiTransfer(std::array<uint8_t,8>& tx,
                              std::array<uint8_t,8>& rx) noexcept {
    // Start the transaction
    csLow();
    hardwareSpiTransmitReceive(tx.data(), rx.data(), tx.size());
    csHigh();
    return true; // return false on failure
}
```

For UART interfaces implement `uartTransfer()` instead.  The method
signatures are documented in `TMC9660CommInterface.hpp`.

---

## 3. Use the Interface with `TMC9660`

Instantiate your interface and pass it to the driver:

```cpp
MyInterface bus;
TMC9660 driver(bus);
```

All library calls now communicate through your implementation.

---

## 4. Next Steps 🚀

After implementing the interface you can build any of the example
programs or integrate the driver into your own application:

* Compile one of the demos from [BuildingExamples.md](BuildingExamples.md)
  to verify transfers work.
* Hook the `TMC9660` object into your existing control loop or RTOS task.
* Consider wrapping the driver in a small C API if your larger project is
  in C rather than C++.


---

[⬅️ Prev](SetupGuide.md) | [⬆️ Back to Index](index.md) | [Next ➡️](BuildingExamples.md)
