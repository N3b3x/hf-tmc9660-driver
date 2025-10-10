#include "../inc/TMC9660Bootloader.hpp"

using namespace tmc9660;

TMC9660Bootloader::TMC9660Bootloader(TMC9660CommInterface &comm) noexcept
    : comm_(comm), deviceAddr_(1), hostAddr_(255) {
  // Default UART addresses: device=1, host=255 (as per spec)
}

bool TMC9660Bootloader::sendCommand(uint8_t cmd, uint32_t value, uint32_t *reply) noexcept {
  // Dispatch to appropriate protocol based on interface type
  if (comm_.mode() == CommMode::SPI) {
    return sendCommandSPI(cmd, value, reply);
  } else if (comm_.mode() == CommMode::UART) {
    return sendCommandUART(cmd, value, reply);
  } else {
    comm_.logDebug(0, "TMC9660Bootloader", "Unsupported interface mode for bootloader");
    return false;
  }
}

bool TMC9660Bootloader::sendCommandSPI(uint8_t cmd, uint32_t value, uint32_t *reply) noexcept {
  auto* spiComm = static_cast<SPITMC9660CommInterface*>(&comm_);
  BootloaderCommandSPI tx{cmd, value};
  
  comm_.logDebug(4, "TMC9660Bootloader", "Sending SPI bootloader command: cmd=0x%02X, value=0x%08X", 
                 cmd, value);

  std::array<uint8_t, 5> txBuf, rxBuf;
  tx.toBuffer(txBuf);
  
  // Step 1: Send the command
  // NOTE: In SPI, the reply to THIS command will come in the NEXT transaction.
  // The rxBuf here contains the reply from the PREVIOUS command (we ignore it).
  if (!spiComm->spiTransfer5(txBuf, rxBuf)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to send SPI command (cmd=0x%02X)", cmd);
    return false;
  }
  
  // Step 2: Send dummy frame (all zeros) to clock out the reply to our command
  // This is standard SPI behavior - replies are delayed by one transaction.
  std::array<uint8_t, 5> dummyTx = {0, 0, 0, 0, 0};
  std::array<uint8_t, 5> replyBuf;
  
  if (!spiComm->spiTransfer5(dummyTx, replyBuf)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to receive SPI reply (cmd=0x%02X)", cmd);
    return false;
  }
  
  // Parse reply from the second transaction
  BootloaderReplySPI rep = BootloaderReplySPI::fromBuffer(replyBuf);
  
  if (!rep.isOK()) {
    comm_.logDebug(0, "TMC9660Bootloader", "SPI command failed: status=0x%02X (cmd=0x%02X)", 
                   rep.status, cmd);
    return false;
  }
  
  if (reply)
    *reply = rep.value;
  
  comm_.logDebug(4, "TMC9660Bootloader", "SPI command successful: status=0x%02X, value=0x%08X (cmd=0x%02X)", 
                 rep.status, rep.value, cmd);
  return true;
}

bool TMC9660Bootloader::sendCommandUART(uint8_t cmd, uint32_t value, uint32_t *reply) noexcept {
  auto* uartComm = static_cast<UARTTMC9660CommInterface*>(&comm_);
  BootloaderCommandUART tx{deviceAddr_, cmd, value};
  
  comm_.logDebug(4, "TMC9660Bootloader", "Sending UART bootloader command: cmd=0x%02X, value=0x%08X", 
                 cmd, value);

  std::array<uint8_t, 8> txBuf, rxBuf;
  tx.toBuffer(txBuf);
  
  // UART: Send command and receive reply in same transaction (no delayed reply like SPI)
  if (!uartComm->uartTransfer(txBuf, rxBuf)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to send UART command (cmd=0x%02X)", cmd);
    return false;
  }
  
  // Parse reply
  BootloaderReplyUART rep = BootloaderReplyUART::fromBuffer(rxBuf);
  
  // Verify CRC
  if (!rep.verifyCRC(rxBuf)) {
    comm_.logDebug(0, "TMC9660Bootloader", "UART reply CRC mismatch (cmd=0x%02X)", cmd);
    return false;
  }
  
  if (!rep.isOK()) {
    comm_.logDebug(0, "TMC9660Bootloader", "UART command failed: status=0x%02X (cmd=0x%02X)", 
                   rep.status, cmd);
    return false;
  }
  
  if (reply)
    *reply = rep.value;
  
  comm_.logDebug(4, "TMC9660Bootloader", "UART command successful: status=0x%02X, value=0x%08X (cmd=0x%02X)", 
                 rep.status, rep.value, cmd);
  return true;
}

//==================================================
// BASIC MEMORY OPERATIONS
//==================================================

bool TMC9660Bootloader::setBank(uint8_t bank) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::SET_BANK), bank);
}

bool TMC9660Bootloader::setAddress(uint32_t addr) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::SET_ADDRESS), addr);
}

bool TMC9660Bootloader::write8(uint8_t v) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::WRITE_8), v);
}

bool TMC9660Bootloader::write16(uint16_t v) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::WRITE_16), v);
}

bool TMC9660Bootloader::write32(uint32_t v) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::WRITE_32), v);
}

bool TMC9660Bootloader::write8Inc(uint8_t v) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::WRITE_8_INC), v);
}

bool TMC9660Bootloader::write16Inc(uint16_t v) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::WRITE_16_INC), v);
}

bool TMC9660Bootloader::write32Inc(uint32_t v) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::WRITE_32_INC), v);
}

bool TMC9660Bootloader::write32IncMultiple(const uint32_t *vals, size_t count) noexcept {
  for (size_t i = 0; i < count; ++i) {
    if (!write32Inc(vals[i]))
      return false;
  }
  return true;
}

bool TMC9660Bootloader::noOp(uint32_t *reply) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::NO_OP), 0, reply);
}

//==================================================
// OTP OPERATIONS
//==================================================

bool TMC9660Bootloader::otpLoad(uint8_t page, uint8_t *errorCount, uint8_t *pageTag) noexcept {
  uint32_t reply = 0;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::OTP_LOAD), page, &reply))
    return false;
  
  if (errorCount)
    *errorCount = static_cast<uint8_t>((reply >> 8) & 0xFF);
  if (pageTag)
    *pageTag = static_cast<uint8_t>(reply & 0xFF);
  
  return true;
}

bool TMC9660Bootloader::otpBurn(uint8_t page, uint8_t pageAddr) noexcept {
  uint32_t value = (static_cast<uint32_t>(pageAddr) << 8) | page;
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::OTP_BURN), value);
}

//==================================================
// EXTERNAL MEMORY OPERATIONS
//==================================================

bool TMC9660Bootloader::memIsConfigured(MemoryBank bank, bool *isConfigured) noexcept {
  uint32_t reply = 0;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::MEM_IS_CONFIGURED), 
                   static_cast<uint32_t>(bank), &reply))
    return false;
  
  if (isConfigured)
    *isConfigured = (reply != 0);
  
  return true;
}

bool TMC9660Bootloader::memIsConnected(MemoryBank bank, bool *isConnected) noexcept {
  uint32_t reply = 0;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::MEM_IS_CONNECTED), 
                   static_cast<uint32_t>(bank), &reply))
    return false;
  
  if (isConnected)
    *isConnected = (reply != 0);
  
  return true;
}

bool TMC9660Bootloader::memIsBusy(MemoryBank bank, bool *isBusy) noexcept {
  uint32_t reply = 0;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::MEM_IS_BUSY), 
                   static_cast<uint32_t>(bank), &reply))
    return false;
  
  if (isBusy)
    *isBusy = (reply != 0);
  
  return true;
}

//==================================================
// SPI FLASH OPERATIONS
//==================================================

bool TMC9660Bootloader::flashLoadBuffer(uint8_t offset, uint32_t data) noexcept {
  // Bits 31-28 = 0 (load), bits 27-24 = offset, bits 23-0 = data
  uint32_t value = (static_cast<uint32_t>(offset & 0x0F) << 24) | (data & 0x00FFFFFF);
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::FLASH_SEND_CMD), value);
}

bool TMC9660Bootloader::flashReadBuffer(uint8_t offset, uint32_t *data) noexcept {
  // Bits 31-28 = 1 (read), bits 27-24 = offset
  uint32_t value = (1U << 28) | (static_cast<uint32_t>(offset & 0x0F) << 24);
  uint32_t reply = 0;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::FLASH_SEND_CMD), value, &reply))
    return false;
  
  if (data)
    *data = reply & 0x00FFFFFF;  // Bits 23-0 contain the data
  
  return true;
}

bool TMC9660Bootloader::flashSendDatagram(uint8_t numBytes) noexcept {
  // Bits 31-28 = 2 (send), bits 27-24 = number of bytes
  uint32_t value = (2U << 28) | (static_cast<uint32_t>(numBytes & 0x0F) << 24);
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::FLASH_SEND_CMD), value);
}

bool TMC9660Bootloader::flashEraseSector(uint32_t address) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::FLASH_ERASE_SECTOR), address & 0x00FFFFFF);
}

bool TMC9660Bootloader::flashReadJedecId(uint8_t *manufacturerId) noexcept {
  // Example from documentation: Read JEDEC ID
  // 1. Load buffer with 0x9F, 0x00
  if (!flashLoadBuffer(0, 0x009F0000))
    return false;
  
  // 2. Send 2 bytes
  if (!flashSendDatagram(2))
    return false;
  
  // 3. Read buffer to get manufacturer ID (bits 15-8)
  uint32_t data = 0;
  if (!flashReadBuffer(0, &data))
    return false;
  
  if (manufacturerId)
    *manufacturerId = static_cast<uint8_t>((data >> 8) & 0xFF);
  
  return true;
}

//==================================================
// RS485 CONFIGURATION
//==================================================

bool TMC9660Bootloader::bootstrapRS485(uint8_t txEnPin, uint8_t preDelay, 
                                       uint8_t hostAddr, uint8_t deviceAddr) noexcept {
  // Byte 0: TX_EN pin, Byte 1: pre-delay, Byte 2: host address, Byte 3: device address
  uint32_t value = (static_cast<uint32_t>(deviceAddr) << 24) |
                   (static_cast<uint32_t>(hostAddr) << 16) |
                   (static_cast<uint32_t>(preDelay) << 8) |
                   txEnPin;
  
  bool result = sendCommand(static_cast<uint8_t>(BootloaderCommand::BOOTSTRAP_RS485), value);
  
  // Update internal addresses if successful
  if (result) {
    deviceAddr_ = deviceAddr;
    hostAddr_ = hostAddr;
  }
  
  return result;
}

//==================================================
// INFORMATION QUERIES
//==================================================

bool TMC9660Bootloader::getInfo(InfoQuery query, uint32_t *value) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::GET_INFO), 
                    static_cast<uint32_t>(query), value);
}

bool TMC9660Bootloader::applyConfiguration(const BootloaderConfig &cfg) noexcept {
  comm_.logDebug(3, "TMC9660Bootloader", "Starting bootloader configuration application");
  
  if (!setBank(5)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to set bank 5");
    return false;
  }
  comm_.logDebug(3, "TMC9660Bootloader", "Successfully set bank 5 (CONFIG memory bank)");

  // LDO configuration (offset 0x00)
  comm_.logDebug(3, "TMC9660Bootloader", "Configuring LDO settings");
  if (!setAddress(bootaddr::LDO_CONFIG)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to set LDO config register");
    return false;
  }
  uint32_t ldo = 0;
  ldo |= static_cast<uint32_t>(cfg.ldo.vext1) & 0x3;               // Bits 0-1: VEXT1
  ldo |= (static_cast<uint32_t>(cfg.ldo.vext2) & 0x3) << 2;        // Bits 2-3: VEXT2
  ldo |= (static_cast<uint32_t>(cfg.ldo.slope_vext1) & 0x3) << 4;  // Bits 4-5: SS_VEXT1
  ldo |= (static_cast<uint32_t>(cfg.ldo.slope_vext2) & 0x3) << 6;  // Bits 6-7: SS_VEXT2
  ldo |= (cfg.ldo.ldo_short_fault ? 1u : 0u) << 8;                 // Bit 8: LDO_SHORT_FAULT
  if (!write32(ldo)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to write LDO config (0x%08X)", ldo);
    return false;
  }
  comm_.logDebug(3, "TMC9660Bootloader", "LDO configured (VEXT1: %d, VEXT2: %d)", 
                 static_cast<int>(cfg.ldo.vext1), static_cast<int>(cfg.ldo.vext2));

  // UART addresses (offset 0x02)
  comm_.logDebug(3, "TMC9660Bootloader", "Configuring UART addresses");
  if (!setAddress(bootaddr::UART_ADDR)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to set UART address register");
    return false;
  }
  uint16_t addrWord = static_cast<uint16_t>(cfg.uart.device_address) |
                      (static_cast<uint16_t>(cfg.uart.host_address) << 8);
  if (!write16(addrWord)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to write UART address word (0x%04X)", addrWord);
    return false;
  }
  comm_.logDebug(3, "TMC9660Bootloader", "UART addresses configured (device: %d, host: %d)", 
                 cfg.uart.device_address, cfg.uart.host_address);

  // RS485 delays
  comm_.logDebug(3, "TMC9660Bootloader", "Configuring RS485 delays");
  if (!setAddress(bootaddr::RS485_DELAY)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to set RS485 delay register");
    return false;
  }
  uint32_t rs485 = static_cast<uint32_t>(cfg.rs485.txen_post_delay) |
                   (static_cast<uint32_t>(cfg.rs485.txen_pre_delay) << 8);
  if (!write32(rs485)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to write RS485 delay config (0x%08X)", rs485);
    return false;
  }
  comm_.logDebug(3, "TMC9660Bootloader", "RS485 delays configured (pre: %d, post: %d)", 
                 cfg.rs485.txen_pre_delay, cfg.rs485.txen_post_delay);

  // Communication configuration (UART/SPI/RS485)
  comm_.logDebug(3, "TMC9660Bootloader", "Configuring communication settings");
  if (!setAddress(bootaddr::COMM_CONFIG)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to set communication config register");
    return false;
  }
  uint32_t comm = 0;
  comm |= (static_cast<uint32_t>(cfg.rs485.txen_pin) & 0x3) << 5;         // BL_UART_TXEN
  comm |= (cfg.spiComm.disable_spi ? 1u : 0u) << 1;                       // BL_DISABLE_SPI
  comm |= (static_cast<uint32_t>(cfg.spiComm.boot_spi_iface) & 0x1) << 2; // BL_SPI_SELECT
  comm |= (static_cast<uint32_t>(cfg.spiComm.spi0_sck_pin) & 0x1) << 10;  // BL_SPI0_SCK
  if (!write32(comm)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to write communication config (0x%08X)", comm);
    return false;
  }
  comm_.logDebug(3, "TMC9660Bootloader", "Communication config written (SPI disabled: %s, TXEN pin: %d)", 
                 cfg.spiComm.disable_spi ? "yes" : "no", static_cast<int>(cfg.rs485.txen_pin));

  // SPI flash configuration
  comm_.logDebug(3, "TMC9660Bootloader", "Configuring SPI flash settings");
  if (!setAddress(bootaddr::SPI_FLASH)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to set SPI flash register");
    return false;
  }
  uint32_t flash = (cfg.spiFlash.enable_flash ? 1u : 0u);
  flash |= (static_cast<uint32_t>(cfg.spiFlash.cs_pin & 0x1F) << 3);
  flash |= (static_cast<uint32_t>(cfg.spiFlash.freq_div) & 0xF) << 8;
  if (!write32(flash)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to write SPI flash config (0x%08X)", flash);
    return false;
  }
  comm_.logDebug(3, "TMC9660Bootloader", "SPI flash configured (enabled: %s, CS pin: %d, freq_div: %d)", 
                 cfg.spiFlash.enable_flash ? "yes" : "no", cfg.spiFlash.cs_pin, static_cast<int>(cfg.spiFlash.freq_div));

  // I2C EEPROM configuration
  comm_.logDebug(3, "TMC9660Bootloader", "Configuring I2C EEPROM settings");
  if (!setAddress(bootaddr::I2C_CONFIG)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to set I2C config register");
    return false;
  }
  uint32_t i2c = (cfg.i2c.enable_eeprom ? 1u : 0u);
  i2c |= (static_cast<uint32_t>(cfg.i2c.sda_pin) & 0x3) << 1;
  i2c |= (static_cast<uint32_t>(cfg.i2c.scl_pin) & 0x3) << 3;
  i2c |= (static_cast<uint32_t>(cfg.i2c.address_bits & 0x7) << 5);
  i2c |= (static_cast<uint32_t>(cfg.i2c.freq_code) & 0x7) << 8;
  if (!write32(i2c)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to write I2C config (0x%08X)", i2c);
    return false;
  }
  comm_.logDebug(3, "TMC9660Bootloader", "I2C EEPROM configured (enabled: %s, SDA: %d, SCL: %d, addr_bits: %d)", 
                 cfg.i2c.enable_eeprom ? "yes" : "no", static_cast<int>(cfg.i2c.sda_pin), 
                 static_cast<int>(cfg.i2c.scl_pin), cfg.i2c.address_bits);

  // GPIO configuration
  comm_.logDebug(3, "TMC9660Bootloader", "Configuring GPIO settings");
  if (!setAddress(bootaddr::GPIO_OUT) || !write32(cfg.gpio.outputMask)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to configure GPIO output mask (0x%08X)", cfg.gpio.outputMask);
    return false;
  }
  if (!setAddress(bootaddr::GPIO_DIR) || !write32(cfg.gpio.directionMask)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to configure GPIO direction mask (0x%08X)", cfg.gpio.directionMask);
    return false;
  }
  if (!setAddress(bootaddr::GPIO_PU) || !write32(cfg.gpio.pullUpMask)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to configure GPIO pull-up mask (0x%08X)", cfg.gpio.pullUpMask);
    return false;
  }
  if (!setAddress(bootaddr::GPIO_PD) || !write32(cfg.gpio.pullDownMask)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to configure GPIO pull-down mask (0x%08X)", cfg.gpio.pullDownMask);
    return false;
  }
  if (!setAddress(bootaddr::GPIO_ANALOG) || !write32(cfg.gpio.analogMask)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to configure GPIO analog mask (0x%08X)", cfg.gpio.analogMask);
    return false;
  }
  comm_.logDebug(3, "TMC9660Bootloader", "GPIO configuration completed");

  // Clock configuration (offset 0x18)
  comm_.logDebug(3, "TMC9660Bootloader", "Configuring clock settings");
  if (!setAddress(bootaddr::CLOCK_CONFIG)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to set clock config register");
    return false;
  }
  uint32_t clk = 0;
  clk |= (static_cast<uint32_t>(cfg.clock.use_external) & 0x1) << 8;     // EXT_NOT_INT
  clk |= (static_cast<uint32_t>(cfg.clock.xtal_drive) & 0x7) << 9;       // XTAL_CFG
  clk |= (cfg.clock.xtal_boost ? 1u : 0u) << 12;                         // XTAL_BOOST
  clk |= (static_cast<uint32_t>(cfg.clock.ext_source_type) & 0x1) << 13; // EXT_NOT_XTAL
  clk |= (static_cast<uint32_t>(cfg.clock.pll_selection) & 0x3) << 16;   // PLL_OUT_SEL
  clk |= (cfg.clock.rdiv & 0x1F) << 18;                                  // RDIV
  clk |= (static_cast<uint32_t>(cfg.clock.sysclk_div) & 0x3) << 23;      // SYS_CLK_DIV
  if (!write32(clk)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to write clock config (0x%08X)", clk);
    return false;
  }
  comm_.logDebug(3, "TMC9660Bootloader", "Clock configuration completed (external: %s, xtal_drive: %d, rdiv: %d)", 
                 cfg.clock.use_external == bootcfg::ClockSource::External ? "yes" : "no", 
                 static_cast<int>(cfg.clock.xtal_drive), cfg.clock.rdiv);

  // ⚠️ CRITICAL: Boot configuration (offset 0x08) - WRITE THIS LAST!
  // When START_MOTOR_CTRL=1 is written, the bootloader IMMEDIATELY exits and starts motor control.
  // Any commands sent after this will fail because the bootloader is no longer running.
  comm_.logDebug(3, "TMC9660Bootloader", "Configuring boot settings (FINAL STEP)");
  if (!setAddress(bootaddr::BOOT_CONFIG)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to set boot config register");
    return false;
  }
  uint32_t boot = 0;
  boot |= static_cast<uint32_t>(cfg.boot.boot_mode) & 0x3;                // Bits 0-1: BOOT_MODE
  boot |= (cfg.boot.bl_ready_fault ? 1u : 0u) << 2;                       // Bit 2: BL_READY_FAULT
  boot |= (cfg.boot.bl_exit_fault ? 1u : 0u) << 3;                        // Bit 3: BL_EXIT_FAULT
  boot |= (cfg.boot.disable_selftest ? 1u : 0u) << 8;                     // Bit 8: DISABLE_SELFTEST
  boot |= (cfg.boot.bl_config_fault ? 1u : 0u) << 9;                      // Bit 9: BL_CONFIG_FAULT
  boot |= (cfg.boot.start_motor_control ? 1u : 0u) << 12;                 // Bit 12: START_MOTOR_CTRL
  if (!write32(boot)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to write boot config (0x%08X)", boot);
    return false;
  }
  comm_.logDebug(3, "TMC9660Bootloader", 
                 "Boot config written: mode=%d, start_motor_ctrl=%s, bl_exit_fault=%s", 
                 static_cast<int>(cfg.boot.boot_mode),
                 cfg.boot.start_motor_control ? "TRUE" : "FALSE",
                 cfg.boot.bl_exit_fault ? "TRUE" : "FALSE");

  if (cfg.boot.start_motor_control) {
    comm_.logDebug(1, "TMC9660Bootloader", 
                   "⚠️  START_MOTOR_CTRL=1 written - Bootloader will EXIT and motor control will START!");
    comm_.logDebug(1, "TMC9660Bootloader", 
                   "⚠️  No further bootloader commands will be accepted after this point!");
  } else {
    comm_.logDebug(2, "TMC9660Bootloader", 
                   "START_MOTOR_CTRL=0 - Bootloader remains active, motor control NOT started");
  }

  comm_.logDebug(3, "TMC9660Bootloader", "Bootloader configuration application completed successfully");
  return true;
}

//==================================================
// MOTOR CONTROL STARTUP
//==================================================

bool TMC9660Bootloader::startMotorControl(bootcfg::BootMode bootMode) noexcept {
  comm_.logDebug(2, "TMC9660Bootloader", "Starting motor control system (mode: %d)...", 
                 static_cast<int>(bootMode));
  
  // Set bank to CONFIG (bank 5)
  if (!setBank(5)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to set CONFIG bank before starting motor control");
    return false;
  }
  
  // Set address to BOOT_CONFIG register (offset 0x08)
  if (!setAddress(bootaddr::BOOT_CONFIG)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to set BOOT_CONFIG address");
    return false;
  }
  
  // Read current BOOT_CONFIG to preserve other settings
  // We'll use a dummy write to get the current value via SPI reply mechanism
  // Note: This is optional - if we don't care about preserving other bits, we can skip this
  
  // Construct BOOT_CONFIG value with START_MOTOR_CTRL=1
  uint32_t bootConfig = 0;
  bootConfig |= static_cast<uint32_t>(bootMode) & 0x3;  // Bits 0-1: BOOT_MODE
  bootConfig |= (1u << 12);                             // Bit 12: START_MOTOR_CTRL = 1
  
  // Optional: Set BL_EXIT_FAULT to get FAULTN signal when exiting (helps debugging)
  bootConfig |= (1u << 3);  // Bit 3: BL_EXIT_FAULT = 1
  
  comm_.logDebug(2, "TMC9660Bootloader", 
                 "Writing BOOT_CONFIG=0x%08X (mode=%d, START_MOTOR_CTRL=1)", 
                 bootConfig, static_cast<int>(bootMode));
  
  // Write the boot configuration
  // ⚠️ CRITICAL: After this write completes, the bootloader will EXIT!
  if (!write32(bootConfig)) {
    comm_.logDebug(0, "TMC9660Bootloader", "Failed to write BOOT_CONFIG");
    return false;
  }
  
  comm_.logDebug(1, "TMC9660Bootloader", 
                 "⚠️  START_MOTOR_CTRL command sent - Bootloader is EXITING NOW!");
  comm_.logDebug(1, "TMC9660Bootloader", 
                 "⚠️  Motor control system starting in %s mode", 
                 bootMode == bootcfg::BootMode::Parameter ? "PARAMETER" : "REGISTER");
  comm_.logDebug(1, "TMC9660Bootloader", 
                 "⚠️  Allow 100-150ms for motor control to fully initialize");
  comm_.logDebug(1, "TMC9660Bootloader", 
                 "⚠️  Bootloader commands will NO LONGER work after this point");
  
  return true;
}
