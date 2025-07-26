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

  // Build a comprehensive bootloader configuration structure.
  tmc9660::BootloaderConfig cfg{};
  
  // CRITICAL: Set parameter mode (required for motor control)
  cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
  cfg.boot.start_motor_control = true;
  cfg.boot.bl_exit_fault = true;
  
  // Communication configuration
  cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;
  cfg.uart.device_address = 1;
  cfg.uart.host_address = 255;
  cfg.uart.rx_pin = tmc9660::bootcfg::UartRxPin::GPIO7;
  cfg.uart.tx_pin = tmc9660::bootcfg::UartTxPin::GPIO6;
  
  cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
  cfg.spiComm.spi0_sck_pin = tmc9660::bootcfg::SPI0SckPin::GPIO6;
  cfg.spiComm.disable_spi = false;
  
  // Clock configuration
  cfg.clock.use_external = tmc9660::bootcfg::ClockSource::Internal;
  cfg.clock.xtal_drive = tmc9660::bootcfg::XtalDrive::Freq16MHz;
  cfg.clock.pll_selection = tmc9660::bootcfg::SysClkSource::PLL;
  cfg.clock.rdiv = 14;
  cfg.clock.sysclk_div = tmc9660::bootcfg::SysClkDiv::Div1;

  // Write the configuration to the device.
  auto res = driver.bootloaderInit(&cfg);
  if (res == TMC9660::BootloaderInitResult::Success) {
    std::cout << "[OK] Bootloader configured successfully for parameter mode" << std::endl;
    std::cout << "  - Boot mode: Parameter" << std::endl;
    std::cout << "  - Motor control: Enabled" << std::endl;
    std::cout << "  - Communication: SPI/UART configured" << std::endl;
    std::cout << "  - Clock: Internal 16MHz with PLL" << std::endl;
  } else {
    std::cout << "[ERROR] Bootloader configuration failed: ";
    if (res == TMC9660::BootloaderInitResult::NoConfig)
      std::cout << "No configuration provided" << std::endl;
    else
      std::cout << "Communication failure" << std::endl;
    return 1;
  }

  // Verify parameter mode is accessible with a simple parameter read
  uint32_t version = 0;
  if (driver.readParameter(tmc9660::tmcl::Parameters::MOTOR_TYPE, version)) {
    std::cout << "[OK] Parameter mode communication verified" << std::endl;
  } else {
    std::cout << "[ERROR] Parameter mode communication failed" << std::endl;
    return 1;
  }

  return 0;
}
