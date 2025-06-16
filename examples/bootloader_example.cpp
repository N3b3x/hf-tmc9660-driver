/**
 * @file bootloader_example.cpp
 * @brief Demonstrates how to configure the bootloader registers.
 *
 * The library itself does not implement any communication back‑end.  Users must
 * provide their own class deriving from ::TMC9660CommInterface.  This example
 * uses a minimal "DummyBus" that simply echoes all transfers so the program can
 * be compiled and run on a desktop machine without hardware.
 */

#include "TMC9660.hpp"
#include <iostream>

/**
 * @brief Minimal SPI bus stub used for demonstration.
 *
 * Replace this with your real SPI or UART implementation when running on the
 * target hardware.
 */
class DummyBus : public SPITMC9660CommInterface {
public:
  bool spiTransfer(std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept override {
    rx = tx; //!< Echo data back to emulate a device
    return true;
  }
};

int main() {
  DummyBus bus;        //!< Replace with your real bus
  TMC9660 driver(bus); //!< Driver communicating over that bus

  // Build a bootloader configuration structure.
  tmc9660::BootloaderConfig cfg{};
  cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;           ///< Example UART speed
  cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0; ///< Use SPI0

  // Write the configuration to the device.
  auto res = driver.bootloaderInit(&cfg);
  if (res == TMC9660::BootloaderInitResult::Success)
    std::cout << "Bootloader configured successfully" << std::endl;
  else
    std::cout << "Bootloader configuration failed" << std::endl;

  return 0;
}
