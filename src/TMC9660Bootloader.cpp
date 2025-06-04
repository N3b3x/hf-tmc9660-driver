#include "TMC9660Bootloader.hpp"
#include <array>

using namespace tmc9660;

TMC9660Bootloader::TMC9660Bootloader(TMC9660CommInterface &comm, uint8_t address) noexcept
  : comm_(comm), address_(address & 0x7F) {}

bool TMC9660Bootloader::sendCommand(uint8_t op, uint16_t type, uint8_t bank, uint32_t value, uint32_t *reply) noexcept {
  TMCLFrame tx{};
  tx.opcode = op;
  tx.type   = type;
  tx.motor  = bank;
  tx.value  = value;

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
  return sendCommand(0x02, static_cast<uint16_t>(addr >> 16), static_cast<uint8_t>(addr >> 8), addr & 0xFF);
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
  // Very simplified implementation: write only UART device/host address
  if (!setBank(5)) return false; // CONFIG bank
  if (!setAddress(0x00020002)) return false; // offset 2 for UART addresses
  uint16_t word = static_cast<uint16_t>(cfg.uart.device_address) | (static_cast<uint16_t>(cfg.uart.host_address) << 8);
  if (!write16(word)) return false;
  return true;
}

