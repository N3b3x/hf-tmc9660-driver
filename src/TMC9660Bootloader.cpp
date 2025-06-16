#include "TMC9660Bootloader.hpp"
#include <array>

using namespace tmc9660;

TMC9660Bootloader::TMC9660Bootloader(TMC9660CommInterface &comm, uint8_t address) noexcept
    : comm_(comm), address_(address & 0x7F) {}

bool TMC9660Bootloader::sendCommand(uint8_t op, uint16_t type, uint8_t bank, uint32_t value,
                                    uint32_t *reply) noexcept {
  TMCLFrame tx{};
  tx.opcode = op;
  tx.type = type;
  tx.motor = bank;
  tx.value = value;

  TMCLReply rep{};
  if (!comm_.transfer(tx, rep, address_))
    return false;
  if (!rep.isOK())
    return false;
  if (reply)
    *reply = rep.value;
  return true;
}

bool TMC9660Bootloader::setBank(uint8_t bank) noexcept {
  return sendCommand(0x01, 0, bank, 0);
}

bool TMC9660Bootloader::setAddress(uint32_t addr) noexcept {
  return sendCommand(0x02, static_cast<uint16_t>(addr >> 16), static_cast<uint8_t>(addr >> 8),
                     addr & 0xFF);
}

bool TMC9660Bootloader::write8(uint8_t v) noexcept {
  return sendCommand(0x03, 0, 0, v);
}

bool TMC9660Bootloader::write16(uint16_t v) noexcept {
  return sendCommand(0x04, 0, 0, v);
}

bool TMC9660Bootloader::write32(uint32_t v) noexcept {
  return sendCommand(0x05, 0, 0, v);
}

bool TMC9660Bootloader::write32Inc(const uint32_t *vals, size_t count) noexcept {
  for (size_t i = 0; i < count; ++i) {
    if (!write32(vals[i]))
      return false;
  }
  return true;
}

bool TMC9660Bootloader::otpBurn(uint8_t page) noexcept {
  return sendCommand(0x06, 0, page, 0);
}

bool TMC9660Bootloader::applyConfiguration(const BootloaderConfig &cfg) noexcept {
  if (!setBank(5))
    return false;

  // UART addresses
  if (!setAddress(bootaddr::UART_ADDR))
    return false;
  uint16_t addrWord = static_cast<uint16_t>(cfg.uart.device_address) |
                      (static_cast<uint16_t>(cfg.uart.host_address) << 8);
  if (!write16(addrWord))
    return false;

  // RS485 delays
  if (!setAddress(bootaddr::RS485_DELAY))
    return false;
  uint32_t rs485 = static_cast<uint32_t>(cfg.rs485.txen_post_delay) |
                   (static_cast<uint32_t>(cfg.rs485.txen_pre_delay) << 8);
  if (!write32(rs485))
    return false;

  // Communication configuration (UART/SPI/RS485)
  if (!setAddress(bootaddr::COMM_CONFIG))
    return false;
  uint32_t comm = 0;
  comm |= (static_cast<uint32_t>(cfg.rs485.txen_pin) & 0x3) << 5;         // BL_UART_TXEN
  comm |= (cfg.spiComm.disable_spi ? 1u : 0u) << 1;                       // BL_DISABLE_SPI
  comm |= (static_cast<uint32_t>(cfg.spiComm.boot_spi_iface) & 0x1) << 2; // BL_SPI_SELECT
  comm |= (static_cast<uint32_t>(cfg.spiComm.spi0_sck_pin) & 0x1) << 10;  // BL_SPI0_SCK
  if (!write32(comm))
    return false;

  // SPI flash configuration
  if (!setAddress(bootaddr::SPI_FLASH))
    return false;
  uint32_t flash = (cfg.spiFlash.enable_flash ? 1u : 0u);
  flash |= (static_cast<uint32_t>(cfg.spiFlash.cs_pin & 0x1F) << 3);
  flash |= (static_cast<uint32_t>(cfg.spiFlash.freq_div) & 0xF) << 8;
  if (!write32(flash))
    return false;

  // I2C EEPROM configuration
  if (!setAddress(bootaddr::I2C_CONFIG))
    return false;
  uint32_t i2c = (cfg.i2c.enable_eeprom ? 1u : 0u);
  i2c |= (static_cast<uint32_t>(cfg.i2c.sda_pin) & 0x3) << 1;
  i2c |= (static_cast<uint32_t>(cfg.i2c.scl_pin) & 0x3) << 3;
  i2c |= (static_cast<uint32_t>(cfg.i2c.address_bits & 0x7) << 5);
  i2c |= (static_cast<uint32_t>(cfg.i2c.freq_code) & 0x7) << 8;
  if (!write32(i2c))
    return false;

  // GPIO configuration
  if (!setAddress(bootaddr::GPIO_OUT) || !write32(cfg.gpio.outputMask))
    return false;
  if (!setAddress(bootaddr::GPIO_DIR) || !write32(cfg.gpio.directionMask))
    return false;
  if (!setAddress(bootaddr::GPIO_PU) || !write32(cfg.gpio.pullUpMask))
    return false;
  if (!setAddress(bootaddr::GPIO_PD) || !write32(cfg.gpio.pullDownMask))
    return false;
  if (!setAddress(bootaddr::GPIO_ANALOG) || !write32(cfg.gpio.analogMask))
    return false;

  // Clock configuration
  if (!setAddress(bootaddr::CLOCK_CONFIG))
    return false;
  uint32_t clk = 0;
  clk |= (static_cast<uint32_t>(cfg.clock.use_external) & 0x1) << 8;     // EXT_NOT_INT
  clk |= (static_cast<uint32_t>(cfg.clock.xtal_drive) & 0x7) << 9;       // XTAL_CFG
  clk |= (cfg.clock.xtal_boost ? 1u : 0u) << 12;                         // XTAL_BOOST
  clk |= (static_cast<uint32_t>(cfg.clock.ext_source_type) & 0x1) << 13; // EXT_NOT_XTAL
  clk |= (static_cast<uint32_t>(cfg.clock.pll_selection) & 0x3) << 16;   // PLL_OUT_SEL
  clk |= (cfg.clock.rdiv & 0x1F) << 18;                                  // RDIV
  clk |= (static_cast<uint32_t>(cfg.clock.sysclk_div) & 0x3) << 23;      // SYS_CLK_DIV
  if (!write32(clk))
    return false;

  return true;
}
