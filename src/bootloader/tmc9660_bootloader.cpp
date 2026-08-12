/**
 * @file tmc9660_bootloader.cpp
 * @brief Tmc9660 Bootloader definitions
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 */
#ifndef TMC9660_BOOTLOADER_IMPL
#define TMC9660_BOOTLOADER_IMPL

#ifdef TMC9660_BOOTLOADER_HEADER_INCLUDED
#include "../../inc/bootloader/tmc9660_bootloader.hpp"
#else
#include "../../inc/bootloader/tmc9660_bootloader.hpp"
#endif

#include "../../inc/tmc9660_comm_interface.hpp"
#include <type_traits>

using namespace tmc9660;

// Helper type traits to check if CommType inherits from SpiCommInterface or UartCommInterface
template<typename T>
struct is_spi_interface : std::is_base_of<SpiCommInterface<T>, T> {};

template<typename T>
struct is_uart_interface : std::is_base_of<UartCommInterface<T>, T> {};

namespace {

inline bool IsOptionalInfoQuery(uint32_t query_value) noexcept {
  const auto query = static_cast<InfoQuery>(query_value);
  switch (query) {
    case InfoQuery::PARTITION_VERSION:
    case InfoQuery::SPI_MEM_SIZE:
    case InfoQuery::I2C_MEM_SIZE:
    case InfoQuery::SPI_MEM_PARTITIONS:
    case InfoQuery::I2C_MEM_PARTITIONS:
      return true;
    default:
      return false;
  }
}

inline bool IsExpectedOptionalInfoFailure(uint8_t cmd, uint32_t value, uint8_t status) noexcept {
  const auto st = static_cast<BootloaderStatus>(status);
  const bool is_expected_status = (st == BootloaderStatus::INVALID_VALUE) ||
                                  (st == BootloaderStatus::INVALID_BANK) ||
                                  (st == BootloaderStatus::MEM_UNCONFIGURED) ||
                                  (st == BootloaderStatus::CMD_NOT_AVAILABLE);
  if (!is_expected_status) {
    return false;
  }

  if (cmd == static_cast<uint8_t>(BootloaderCommand::GET_INFO)) {
    return IsOptionalInfoQuery(value);
  }

  if (cmd == static_cast<uint8_t>(BootloaderCommand::SET_BANK)) {
    const auto bank = static_cast<MemoryBank>(value);
    return (bank == MemoryBank::SPI_FLASH) || (bank == MemoryBank::I2C_EEPROM);
  }

  return false;
}

} // namespace

// tmc9660_bootloader.cpp - Template implementation of TMC9660Bootloader
// This file is included at the end of tmc9660_bootloader.hpp
// DO NOT include this file directly - include tmc9660_bootloader.hpp instead
//
// Note: This file contains all template function implementations for the TMC9660Bootloader.
// It follows the same pattern as other HardFOC drivers (AS5047U, BNO08x) where template
// implementations are in a .cpp file that is included at the end of the header.

// tmc9660_bootloader.tpp - Template implementation of TMC9660Bootloader
// This file is included at the end of tmc9660_bootloader.hpp
// DO NOT include this file directly - include tmc9660_bootloader.hpp instead

// Note: The header (tmc9660_bootloader.hpp) must be included before this file
// All necessary includes and namespace declarations are in the header

/**
 * @brief Construct a TMC9660Bootloader instance.
 *
 * Initializes the bootloader with the specified communication interface
 * and sets default UART addresses as per the TMC9660 specification.
 *
 * @param comm Reference to the communication interface for bootloader commands
 */
template <typename CommType>
TMC9660Bootloader<CommType>::TMC9660Bootloader(CommType& comm) noexcept
    : comm_(comm), deviceAddr_(1), hostAddr_(255) {
  // Default UART addresses: device=1, host=255 (as per spec)
}

/**
 * @brief Send a bootloader command and optionally receive a reply.
 *
 * This method dispatches the command to the appropriate protocol handler
 * based on the communication interface type (SPI or UART). It provides
 * a unified interface for all bootloader commands.
 *
 * @param cmd Bootloader command code to send
 * @param value 32-bit value to send with the command
 * @param reply Optional pointer to store the 32-bit reply value
 * @return true if command was sent successfully, false on error
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::sendCommand(uint8_t cmd, uint32_t value, uint32_t* reply) noexcept {
  // Dispatch to appropriate protocol based on interface type
  // Use compile-time type checking to only instantiate the appropriate function
  if constexpr (std::is_base_of_v<SpiCommInterface<CommType>, CommType>) {
    if (comm_.mode() == CommMode::SPI) {
      return sendCommandSPI(cmd, value, reply);
    }
  }
  if constexpr (std::is_base_of_v<UartCommInterface<CommType>, CommType>) {
    if (comm_.mode() == CommMode::UART) {
      return sendCommandUART(cmd, value, reply);
    }
  }
  TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Unsupported interface mode for bootloader");
  return false;
}

/**
 * @brief Send a bootloader command via SPI interface.
 *
 * Implements the SPI bootloader protocol which requires two transactions:
 * 1. Send command (receive previous reply, ignored)
 * 2. Send NO_OP command (receive current command's reply)
 *
 * This is standard SPI behavior where replies are delayed by one transaction.
 *
 * @param cmd Bootloader command code to send
 * @param value 32-bit value to send with the command
 * @param reply Optional pointer to store the 32-bit reply value
 * @return true if command was sent successfully, false on error
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::sendCommandSPI(uint8_t cmd, uint32_t value, uint32_t* reply) noexcept {
  if (comm_.mode() != CommMode::SPI) {
    return false;
  }
  // At this point, we know comm_.mode() == CommMode::SPI, so CommType must inherit from
  // SpiCommInterface<CommType>. We cast to the CRTP base class to access the public method,
  // which will use CRTP (static_cast<Derived*>(this)) internally to call the derived implementation.
  // This is safe because:
  // 1. Runtime check ensures mode() == SPI, so CommType is an SPI interface
  // 2. CommType (e.g., Esp32SPITMC9660CommInterface) inherits from SpiCommInterface<CommType>
  // 3. The CRTP base class method will properly dispatch to the derived implementation
  BootloaderCommandSPI tx{cmd, value};

  TMC9660_LOG_DEBUG(comm_, 4, "TMC9660Bootloader",
                    "Sending SPI bootloader command: cmd=0x%02X, value=0x%08X", cmd, value);

  std::array<uint8_t, 5> txBuf, rxBuf;
  tx.toBuffer(txBuf);

  // Same verbosity as [BL TX NOOP]: raw 5-byte command frame (host→device) for SPI bring-up.
  TMC9660_LOG_DEBUG(comm_, 4, "TMC9660Bootloader", "[BL TX CMD ] %02X %02X %02X %02X %02X",
                    txBuf[0], txBuf[1], txBuf[2], txBuf[3], txBuf[4]);

  // Call the method directly on comm_ - since CommType inherits from SpiCommInterface<CommType>,
  // it has spiTransferBootloader() available through inheritance. The CRTP base class method
  // will internally use static_cast<Derived*>(this) to call the derived implementation.
  // Step 1: Send the command
  // NOTE: In SPI, the reply to THIS command will come in the NEXT transaction.
  // The rxBuf here contains the reply from the PREVIOUS command (we ignore it).
  if (!comm_.spiTransferBootloader(txBuf, rxBuf)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to send SPI command (cmd=0x%02X)",
                      cmd);
    return false;
  }

  // TMC9660_LOG_DEBUG(comm_,2, "TMC9660Bootloader", "[BL RX PREV] %02X %02X %02X %02X %02X
  // (ignored)",
  //                rxBuf[0], rxBuf[1], rxBuf[2], rxBuf[3], rxBuf[4]);

  // Step 2: Send NO_OP command to clock out the reply to our command
  // This is standard SPI behavior - replies are delayed by one transaction.
  std::array<uint8_t, 5> dummyTx = {
      static_cast<uint8_t>(BootloaderCommand::NO_OP), // NO_OP command
      0, 0, 0, 0                                      // Dummy value (all zeros)
  };
  std::array<uint8_t, 5> replyBuf;

  TMC9660_LOG_DEBUG(comm_, 4, "TMC9660Bootloader", "[BL TX NOOP] %02X %02X %02X %02X %02X",
                    dummyTx[0], dummyTx[1], dummyTx[2], dummyTx[3], dummyTx[4]);

  // Step 2: Send NO_OP to receive the reply to our command
  if (!comm_.spiTransferBootloader(dummyTx, replyBuf)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to receive SPI reply (cmd=0x%02X)",
                      cmd);
    return false;
  }

  TMC9660_LOG_DEBUG(comm_, 4, "TMC9660Bootloader", "[BL RX RPLY] %02X %02X %02X %02X %02X",
                    replyBuf[0], replyBuf[1], replyBuf[2], replyBuf[3], replyBuf[4]);

  // Parse reply from the second transaction
  BootloaderReplySPI rep = BootloaderReplySPI::fromBuffer(replyBuf);

  if (!rep.isOK()) {
    if (IsExpectedOptionalInfoFailure(cmd, value, rep.status)) {
      TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                        "Optional bootloader info unavailable: status=0x%02X (cmd=0x%02X, query=0x%08X)",
                        rep.status, cmd, value);
    } else {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                        "SPI command failed: status=0x%02X (cmd=0x%02X)", rep.status, cmd);
    }
    return false;
  }

  if (reply)
    *reply = rep.value;

  TMC9660_LOG_DEBUG(comm_, 4, "TMC9660Bootloader",
                    "SPI command successful: status=0x%02X, value=0x%08X (cmd=0x%02X)", rep.status,
                    rep.value, cmd);
  return true;
}

/**
 * @brief Send a bootloader command via UART interface.
 *
 * Implements the UART bootloader protocol which uses a single transaction:
 * Send command and receive reply immediately. The UART protocol includes
 * CRC-8 checksum verification for data integrity.
 *
 * @param cmd Bootloader command code to send
 * @param value 32-bit value to send with the command
 * @param reply Optional pointer to store the 32-bit reply value
 * @return true if command was sent successfully, false on error
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::sendCommandUART(uint8_t cmd, uint32_t value, uint32_t* reply) noexcept {
  if (comm_.mode() != CommMode::UART) {
    return false;
  }
  // At this point, we know comm_.mode() == CommMode::UART, so CommType must be a UART interface
  // Call uartTransferBootloader directly on comm_ - if CommType is UART, it will have this method
  // We use a helper function pointer to avoid template instantiation issues
  BootloaderCommandUART tx{deviceAddr_, cmd, value};

  TMC9660_LOG_DEBUG(comm_, 4, "TMC9660Bootloader",
                    "Sending UART bootloader command: cmd=0x%02X, value=0x%08X", cmd, value);

  std::array<uint8_t, 8> txBuf, rxBuf;
  tx.toBuffer(txBuf);

  // Log transmitted bytes
  // TMC9660_LOG_DEBUG(comm_,2, "TMC9660Bootloader", "[UART BL TX] %02X %02X %02X %02X %02X %02X
  // %02X %02X",
  //                txBuf[0], txBuf[1], txBuf[2], txBuf[3], txBuf[4], txBuf[5], txBuf[6], txBuf[7]);

  // UART: Send command and receive reply in same transaction (no delayed reply like SPI)
  // Call the method directly on comm_ - since CommType inherits from UartCommInterface<CommType>,
  // it has uartTransferBootloader() available through inheritance. The CRTP base class method
  // will internally use static_cast<Derived*>(this) to call the derived implementation.
  if (!comm_.uartTransferBootloader(txBuf, rxBuf)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to send UART command (cmd=0x%02X)",
                      cmd);
    return false;
  }

  // Log received bytes
  // TMC9660_LOG_DEBUG(comm_,2, "TMC9660Bootloader", "[UART BL RX] %02X %02X %02X %02X %02X %02X
  // %02X %02X",
  //                rxBuf[0], rxBuf[1], rxBuf[2], rxBuf[3], rxBuf[4], rxBuf[5], rxBuf[6], rxBuf[7]);

  // Parse reply
  BootloaderReplyUART rep = BootloaderReplyUART::fromBuffer(rxBuf);

  // Verify CRC
  if (!rep.verifyCRC(rxBuf)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "UART reply CRC mismatch (cmd=0x%02X)", cmd);
    return false;
  }

  if (!rep.isOK()) {
    if (IsExpectedOptionalInfoFailure(cmd, value, rep.status)) {
      TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                        "Optional bootloader info unavailable: status=0x%02X (cmd=0x%02X, query=0x%08X)",
                        rep.status, cmd, value);
    } else {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                        "UART command failed: status=0x%02X (cmd=0x%02X)", rep.status, cmd);
    }
    return false;
  }

  if (reply)
    *reply = rep.value;

  TMC9660_LOG_DEBUG(comm_, 4, "TMC9660Bootloader",
                    "UART command successful: status=0x%02X, value=0x%08X (cmd=0x%02X)", rep.status,
                    rep.value, cmd);
  return true;
}

//==================================================
// BASIC MEMORY OPERATIONS
//==================================================

/**
 * @brief Select the target memory bank for bootloader operations.
 *
 * Sets the memory bank that subsequent read/write operations will target.
 * Valid banks include RAM, OTP, SPI Flash, I2C EEPROM, and CONFIG.
 *
 * @param bank Memory bank number (0-5)
 * @return true if bank was set successfully, false on error
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::setBank(uint8_t bank) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::SET_BANK), bank);
}

/**
 * @brief Set the address within the current memory bank.
 *
 * Sets the address pointer for subsequent read/write operations within
 * the currently selected memory bank. The address is bank-specific.
 *
 * @param addr 32-bit address within the current bank
 * @return true if address was set successfully, false on error
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::setAddress(uint32_t addr) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::SET_ADDRESS), addr);
}

/**
 * @brief Write a single byte to the previously selected address.
 *
 * Writes an 8-bit value to the current address in the selected memory bank.
 * The address pointer is not incremented after the write operation.
 *
 * @param v 8-bit value to write
 * @return true if write was successful, false on error
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::write8(uint8_t v) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::WRITE_8), v);
}

/**
 * @brief Write a 16-bit word to the previously selected address.
 *
 * Writes a 16-bit value to the current address in the selected memory bank.
 * The address pointer is not incremented after the write operation.
 *
 * @param v 16-bit value to write
 * @return true if write was successful, false on error
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::write16(uint16_t v) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::WRITE_16), v);
}

/**
 * @brief Write a 32-bit word to the previously selected address.
 *
 * Writes a 32-bit value to the current address in the selected memory bank.
 * The address pointer is not incremented after the write operation.
 *
 * @param v 32-bit value to write
 * @return true if write was successful, false on error
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::write32(uint32_t v) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::WRITE_32), v);
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::write8Inc(uint8_t v) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::WRITE_8_INC), v);
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::write16Inc(uint16_t v) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::WRITE_16_INC), v);
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::write32Inc(uint32_t v) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::WRITE_32_INC), v);
}

/**
 * @brief Write multiple 32-bit words starting at the current address.
 *
 * Writes an array of 32-bit values using the WRITE_32_INC command,
 * which automatically increments the address pointer after each write.
 * This is useful for writing configuration blocks or data arrays.
 *
 * @param values Pointer to array of 32-bit values to write
 * @param count Number of 32-bit values to write
 * @return true if all writes were successful, false on error
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::write32IncMultiple(const uint32_t* values, size_t count) noexcept {
  for (size_t i = 0; i < count; ++i) {
    if (!write32Inc(values[i]))
      return false;
  }
  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::noOp(uint32_t* reply) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::NO_OP), 0, reply);
}

//==================================================
// OTP OPERATIONS
//==================================================

/**
 * @brief Load an OTP page into the OTP memory bank.
 *
 * Loads the specified OTP (One-Time Programmable) page into the OTP memory bank
 * for reading or verification. The OTP memory contains factory-programmed data
 * that cannot be modified by the user.
 *
 * @param page OTP page number to load (0-255)
 * @param result Pointer to store the OTP load result including error count and page tag
 * @return true if command was sent successfully, false on communication error
 * @note Check result.errorCount for bit errors, result.pageTag for page tag
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::otpLoad(uint8_t page, OtpLoadResult* result) noexcept {
  if (!result) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "otpLoad: null result pointer");
    return false;
  }

  uint32_t reply = 0;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::OTP_LOAD), page, &reply)) {
    return false;
  }

  *result = OtpLoadResult::fromValue(reply);
  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::otpLoad(uint8_t page, uint8_t* errorCount, uint8_t* pageTag) noexcept {
  OtpLoadResult result;
  if (!otpLoad(page, &result)) {
    return false;
  }

  if (errorCount)
    *errorCount = result.errorCount;
  if (pageTag)
    *pageTag = result.pageTag;

  return true;
}

/**
 * @brief Permanently burn the given OTP page.
 *
 * Burns (programs) the specified OTP page with the given page address.
 * This operation is irreversible and can only be performed once per page.
 * The OTP memory is used for storing permanent configuration data.
 *
 * @param page OTP page number to burn (0-255)
 * @param page_addr OTP page address to write (0-255)
 * @param result Pointer to store the detailed burn result with error information
 * @return true if command was sent successfully, false on communication error
 * @note Check result.isSuccess and result.error_code for burn operation status
 * @warning This operation is irreversible! Use with extreme caution.
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::otpBurn(uint8_t page, uint8_t page_addr, OtpBurnResult* result) noexcept {
  if (!result) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "otpBurn: null result pointer");
    return false;
  }

  uint32_t value = (static_cast<uint32_t>(page_addr) << 8) | page;
  uint32_t reply = 0;

  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::OTP_BURN), value, &reply)) {
    return false;
  }

  // Check if the command returned an error status
  if (reply == 0) {
    // Success case - no error code
    *result = OtpBurnResult::createSuccess();
  } else {
    // Error case - parse error code from reply
    int8_t error_code = static_cast<int8_t>(reply);
    *result = OtpBurnResult::createError(static_cast<OtpBurnError>(error_code));
  }

  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::otpBurn(uint8_t page, uint8_t page_addr) noexcept {
  OtpBurnResult result;
  if (!otpBurn(page, page_addr, &result)) {
    return false;
  }

  return result.isSuccess;
}

/**
 * @brief Perform OTP burn operation with Erratum 1 workaround for TMC9660.
 *
 * This method implements a complex workaround for TMC9660 Erratum 1, which affects
 * OTP (One-Time Programmable) memory burning operations. The workaround involves
 * a specific sequence of register operations to ensure proper OTP programming.
 *
 * The workaround sequence:
 * 1. Set memory bank to 0
 * 2. Set address to 0x4801B010 (specific register for workaround)
 * 3. Read 32-bit value from the register
 * 4. Clear bit 0 of the read value
 * 5. Write the modified value back
 * 6. Wait for VDRV voltage to drop below 8.4V (critical timing)
 * 7. Perform the actual OTP burn operation
 * 8. Verify the burn operation success
 *
 * This workaround is essential for reliable OTP programming and must be used
 * instead of the standard OTP burn procedure when working with affected TMC9660
 * silicon revisions.
 *
 * @param page OTP page number to burn
 * @param page_addr Address within the OTP page
 * @param result Pointer to store burn operation result
 * @param vdrv_wait_ms Milliseconds to wait for VDRV voltage drop (typically 100-200ms)
 * @return true if workaround sequence completed successfully, false on error
 * @note This method implements TMC9660 Erratum 1 workaround - do not use standard OTP burn
 * @warning VDRV voltage timing is critical - incorrect timing may cause burn failure
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::otpBurnWithWorkaround(uint8_t page, uint8_t page_addr, OtpBurnResult* result,
                                              uint32_t vdrv_wait_ms) noexcept {
  if (!result) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "otpBurnWithWorkaround: null result pointer");
    return false;
  }

  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                    "Starting OTP burn with Erratum 1 workaround (page=%d, addr=0x%02X)", page,
                    page_addr);

  // Step 1: Send SET_BANK, value 0
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Step 1: Setting bank to 0");
  if (!setBank(0)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set bank 0");
    *result = OtpBurnResult::createError(OtpBurnError::BURN_PROCEDURE_FAILED);
    return false;
  }

  // Step 2: Send SET_ADDRESS, value 0x4801B010
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Step 2: Setting address to 0x4801B010");
  if (!setAddress(0x4801B010)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set address 0x4801B010");
    *result = OtpBurnResult::createError(OtpBurnError::BURN_PROCEDURE_FAILED);
    return false;
  }

  // Step 3: Send READ_32
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Step 3: Reading 32-bit value");
  uint32_t read_value;
  if (!read32(&read_value)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to read 32-bit value");
    *result = OtpBurnResult::createError(OtpBurnError::BURN_PROCEDURE_FAILED);
    return false;
  }

  // Step 4: Clear bit 0 of the read value (0x00000001)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Step 4: Clearing bit 0 (read=0x%08X)",
                    read_value);
  uint32_t modified_value = read_value & ~0x00000001;

  // Step 5: Send WRITE_32 with the modified read value
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Step 5: Writing modified value 0x%08X",
                    modified_value);
  if (!write32(modified_value)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write modified value");
    *result = OtpBurnResult::createError(OtpBurnError::BURN_PROCEDURE_FAILED);
    return false;
  }

  // Step 6: Wait for VDRV voltage to drop below 8.4V
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                    "Step 6: Waiting %dms for VDRV voltage to drop below 8.4V", vdrv_wait_ms);
  comm_.delayMs(vdrv_wait_ms);

  // Step 7: Send OTP_BURN
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Step 7: Sending OTP_BURN command");
  uint32_t value = (static_cast<uint32_t>(page_addr) << 8) | page;
  uint32_t reply = 0;

  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::OTP_BURN), value, &reply)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to send OTP_BURN command");
    *result = OtpBurnResult::createError(OtpBurnError::BURN_PROCEDURE_FAILED);
    return false;
  }

  // Parse the result
  if (reply == 0) {
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "OTP burn command completed successfully");
    *result = OtpBurnResult::createSuccess();
  } else {
    int8_t error_code = static_cast<int8_t>(reply);
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "OTP burn command failed with error code: %d",
                      error_code);
    *result = OtpBurnResult::createError(static_cast<OtpBurnError>(error_code));
  }

  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::checkOtpBurnStatus(bool* result) noexcept {
  if (!result) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "checkOtpBurnStatus: null result pointer");
    return false;
  }

  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                    "Checking OTP burn status using Erratum 1 workaround");

  // Step 1: Configure clock settings to have PLL active, SYS_CLK_DIV set to 3 (15MHz)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "Step 1: Configuring clock for 15MHz system clock");
  if (!setBank(5)) { // CONFIG bank
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set CONFIG bank");
    return false;
  }

  // Read current clock configuration
  if (!setAddress(0x00020018)) { // Clock config offset
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set clock config address");
    return false;
  }

  uint32_t clock_config;
  if (!read32(&clock_config)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to read clock configuration");
    return false;
  }

  // Set SYS_CLK_DIV to 3 (15MHz system clock)
  uint32_t modified_clock_config = (clock_config & ~0x03000000) | (3 << 24);
  if (!write32(modified_clock_config)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                      "Failed to write modified clock configuration");
    return false;
  }

  // Wait for clock configuration to take effect
  comm_.delayMs(100);

  // Step 2: Send SET_BANK, value 0
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Step 2: Setting bank to 0");
  if (!setBank(0)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set bank 0");
    return false;
  }

  // Step 3: Send SET_ADDRESS, value 0x48020014
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Step 3: Setting address to 0x48020014");
  if (!setAddress(0x48020014)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set address 0x48020014");
    return false;
  }

  // Step 4: Send READ_16
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Step 4: Reading 16-bit value");
  uint16_t read_value;
  if (!read16(&read_value)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to read 16-bit value");
    return false;
  }

  // Step 5: Check if read value is 0x80 or 0x84 (successful burn)
  *result = (read_value == 0x80) || (read_value == 0x84);
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "OTP burn status check: read=0x%04X, success=%s",
                    read_value, *result ? "true" : "false");

  // Step 6: Set SYS_CLK_DIV back to 0 (40MHz)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Step 6: Restoring clock to 40MHz");
  if (!setBank(5)) { // CONFIG bank
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set CONFIG bank for clock restore");
    return false;
  }

  if (!setAddress(0x00020018)) { // Clock config offset
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                      "Failed to set clock config address for restore");
    return false;
  }

  if (!write32(clock_config)) { // Restore original clock config
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                      "Failed to restore original clock configuration");
    return false;
  }

  // Wait for clock configuration to take effect
  comm_.delayMs(100);

  return true;
}

//==================================================
// EXTERNAL MEMORY OPERATIONS
//==================================================

template <typename CommType>

bool TMC9660Bootloader<CommType>::memIsConfigured(MemoryBank bank, bool* is_configured) noexcept {
  uint32_t reply = 0;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::MEM_IS_CONFIGURED),
                   static_cast<uint32_t>(bank), &reply))
    return false;

  if (is_configured)
    *is_configured = (reply != 0);

  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::memIsConnected(MemoryBank bank, bool* is_connected) noexcept {
  uint32_t reply = 0;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::MEM_IS_CONNECTED),
                   static_cast<uint32_t>(bank), &reply))
    return false;

  if (is_connected)
    *is_connected = (reply != 0);

  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::memIsBusy(MemoryBank bank, bool* is_busy) noexcept {
  uint32_t reply = 0;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::MEM_IS_BUSY),
                   static_cast<uint32_t>(bank), &reply))
    return false;

  if (is_busy)
    *is_busy = (reply != 0);

  return true;
}

//==================================================
// SPI FLASH OPERATIONS
//==================================================

template <typename CommType>

bool TMC9660Bootloader<CommType>::flashLoadBuffer(uint8_t offset, uint32_t data) noexcept {
  // Bits 31-28 = 0 (load), bits 27-24 = offset, bits 23-0 = data
  uint32_t value = (static_cast<uint32_t>(offset & 0x0F) << 24) | (data & 0x00FFFFFF);
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::FLASH_SEND_CMD), value);
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::flashReadBuffer(uint8_t offset, uint32_t* data) noexcept {
  // Bits 31-28 = 1 (read), bits 27-24 = offset
  uint32_t value = (1U << 28) | (static_cast<uint32_t>(offset & 0x0F) << 24);
  uint32_t reply = 0;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::FLASH_SEND_CMD), value, &reply))
    return false;

  if (data)
    *data = reply & 0x00FFFFFF; // Bits 23-0 contain the data

  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::flashSendDatagram(uint8_t numBytes) noexcept {
  // Bits 31-28 = 2 (send), bits 27-24 = number of bytes
  uint32_t value = (2U << 28) | (static_cast<uint32_t>(numBytes & 0x0F) << 24);
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::FLASH_SEND_CMD), value);
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::flashEraseSector(uint32_t address) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::FLASH_ERASE_SECTOR),
                     address & 0x00FFFFFF);
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::flashReadJedecId(uint8_t* manufacturerId) noexcept {
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

template <typename CommType>
bool TMC9660Bootloader<CommType>::bootstrapRS485(uint8_t txEnPin, uint8_t preDelay, uint8_t hostAddr,
                                       uint8_t deviceAddr) noexcept {
  // Byte 0: TX_EN pin, Byte 1: pre-delay, Byte 2: host address, Byte 3: device address
  uint32_t value = (static_cast<uint32_t>(deviceAddr) << 24) |
                   (static_cast<uint32_t>(hostAddr) << 16) |
                   (static_cast<uint32_t>(preDelay) << 8) | txEnPin;

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

/**
 * @brief Get bootloader information using the specified query type.
 *
 * Queries various information about the TMC9660 bootloader including
 * version, features, memory sizes, and chip identification. This is
 * useful for debugging and verifying the bootloader configuration.
 *
 * @param query Information type to query (see InfoQuery enum)
 * @param value Pointer to store the queried value
 * @return true if query was successful, false on error
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::getInfo(InfoQuery query, uint32_t* value) noexcept {
  return sendCommand(static_cast<uint8_t>(BootloaderCommand::GET_INFO),
                     static_cast<uint32_t>(query), value);
}

/**
 * @brief Get bootloader version information.
 *
 * Retrieves the bootloader version information including major and minor
 * version numbers. This is useful for compatibility checking and debugging.
 *
 * @param version Pointer to store the parsed version information
 * @return true if version was retrieved successfully, false on error
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::getBootloaderVersion(BootloaderVersion* version) noexcept {
  if (!version) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "getBootloaderVersion: null pointer");
    return false;
  }

  uint32_t value;
  if (!getInfo(InfoQuery::BL_VERSION, &value)) {
    return false;
  }

  *version = BootloaderVersion::fromValue(value);
  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::getFeatures(BootloaderFeatures* features) noexcept {
  if (!features) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "getFeatures: null pointer");
    return false;
  }

  uint32_t value;
  if (!getInfo(InfoQuery::FEATURES, &value)) {
    return false;
  }

  *features = BootloaderFeatures::fromValue(value);
  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::getGitInfo(GitInfo* gitInfo) noexcept {
  if (!gitInfo) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "getGitInfo: null pointer");
    return false;
  }

  uint32_t value;
  if (!getInfo(InfoQuery::GIT_INFO, &value)) {
    return false;
  }

  *gitInfo = GitInfo::fromValue(value);
  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::getPartitionVersion(PartitionVersion* version) noexcept {
  if (!version) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "getPartitionVersion: null pointer");
    return false;
  }

  uint32_t value;
  if (!getInfo(InfoQuery::PARTITION_VERSION, &value)) {
    return false;
  }

  *version = PartitionVersion::fromValue(value);
  return true;
}

/**
 * @brief Apply all fields of a BootloaderConfig to the TMC9660.
 *
 * This method performs the complete bootloader configuration sequence by
 * writing all configuration registers in the correct order. It handles
 * the complex register dependencies and ensures proper initialization.
 *
 * The configuration is applied in the following order:
 * 1. LDO configuration
 * 2. Clock configuration (CRITICAL - must be done before motor control)
 * 3. UART addresses and RS485 delays
 * 4. Communication interface selection
 * 5. SPI flash and I2C EEPROM configuration
 * 6. GPIO configuration (output levels, directions, pull settings)
 * 7. Encoder and sensor configuration
 * 8. Brake and mechanical configuration
 * 9. Memory storage configuration
 * 10. Boot configuration (FINAL - may exit bootloader)
 *
 * @param cfg Complete bootloader configuration to apply
 * @param fail_on_verify_error If true, return false on read-back verification failure;
 *                          if false, log warning but continue
 * @return true if configuration was applied successfully, false on error
 * @note If cfg.boot.start_motor_control is true, bootloader will exit after this call
 * @warning Clock reconfiguration takes time and the TMC9660 cannot respond during this process
 */
/**
 * @brief Apply complete bootloader configuration with proper register sequencing.
 *
 * This method performs the complete bootloader configuration sequence by writing
 * all configuration registers in the correct order. It handles complex register
 * dependencies and ensures proper initialization of the TMC9660 system.
 *
 * The configuration is applied in a specific order to handle dependencies:
 * 1. LDO configuration - Power supply settings
 * 2. Clock configuration - CRITICAL timing setup (may cause temporary unresponsiveness)
 * 3. UART addresses and RS485 delays - Communication parameters
 * 4. Communication interface selection - SPI/UART mode
 * 5. SPI flash and I2C EEPROM configuration - External memory settings
 * 6. GPIO configuration - Pin directions, levels, and pull settings
 * 7. Encoder and sensor configuration - Feedback system setup
 * 8. Brake and mechanical configuration - Safety and mechanical systems
 * 9. Memory storage configuration - Data storage parameters
 * 10. Boot configuration - FINAL step (may exit bootloader mode)
 *
 * @param cfg Complete bootloader configuration structure to apply
 * @param fail_on_verify_error If true, return false on read-back verification failure;
 *                          if false, log warning but continue with configuration
 * @return true if configuration was applied successfully, false on error
 * @note If cfg.boot.start_motor_control is true, bootloader will exit after this call
 * @warning Clock reconfiguration takes time and the TMC9660 cannot respond during this process
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::applyConfiguration(const BootloaderConfig& cfg,
                                           bool fail_on_verify_error) noexcept {
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Starting bootloader configuration application");

  if (!setBank(5)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set bank 5");
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Successfully set bank 5 (CONFIG memory bank)");

  // LDO configuration (offset 0x00)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Configuring LDO settings");
  if (!setAddress(bootaddr::LDO_CONFIG)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set LDO config register");
    return false;
  }
  uint16_t ldo = 0;
  ldo |= static_cast<uint16_t>(cfg.ldo.vext1) & 0x3;              // Bits 0-1: VEXT1
  ldo |= (static_cast<uint16_t>(cfg.ldo.vext2) & 0x3) << 2;       // Bits 2-3: VEXT2
  ldo |= (static_cast<uint16_t>(cfg.ldo.slope_vext1) & 0x3) << 4; // Bits 4-5: SS_VEXT1
  ldo |= (static_cast<uint16_t>(cfg.ldo.slope_vext2) & 0x3) << 6; // Bits 6-7: SS_VEXT2
  ldo |= (cfg.ldo.ldo_short_fault ? 1u : 0u) << 8;                // Bit 8: LDO_SHORT_FAULT
  if (!write16(ldo)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write LDO config (0x%04X)", ldo);
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "LDO configured (VEXT1: %d, VEXT2: %d)",
                    static_cast<int>(cfg.ldo.vext1), static_cast<int>(cfg.ldo.vext2));

  // Verify LDO configuration was written correctly
  if (!readAndVerify16(ldo, "LDO config")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "LDO configuration verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  LDO configuration verification failed (continuing)");
    }
  }

  // ⚠️ CRITICAL: Clock configuration (offset 0x18) - MUST BE DONE RIGHT BEFORE MOTOR CONTROL START!
  // Clock reconfiguration takes time and the TMC9660 cannot respond during this process.
  // This must happen just before starting motor control to ensure proper timing.
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "Configuring clock settings (FINAL STEP BEFORE MOTOR CONTROL)");
  if (!setAddress(bootaddr::CLOCK_CONFIG)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set clock config register");
    return false;
  }

  // Test: Read current clock configuration before writing
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Reading current clock configuration...");
  uint32_t current_clock;
  if (!read32(&current_clock)) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                      "⚠️  Failed to read current clock config (continuing)");
  } else {
    TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Current clock config: 0x%08X (PLL_STATUS=%s)",
                      current_clock, (current_clock & (1u << 30)) ? "SET" : "CLEAR");
  }
  // ⚠️ CRITICAL: Bits 0-6 MUST be set to 99 (0x63) per datasheet requirement!
  // "RESERVED_1: Bits 0-6. Reserved. Must always stay set to 99."
  uint32_t clk = 99u; // Start with required reserved value
  clk |= (static_cast<uint32_t>(cfg.clock.use_external) & 0x1) << 8;     // EXT_NOT_INT [0:6]
  clk |= (static_cast<uint32_t>(cfg.clock.xtal_drive) & 0x7) << 9;       // XTAL_CFG [7:9]
  clk |= (cfg.clock.xtal_boost ? 1u : 0u) << 12;                         // XTAL_BOOST [12:12]
  clk |= (static_cast<uint32_t>(cfg.clock.ext_source_type) & 0x1) << 13; // EXT_NOT_XTAL [13:13]
  clk |= (static_cast<uint32_t>(cfg.clock.pll_selection) & 0x3) << 16;   // PLL_OUT_SEL [16:17]
  clk |= (cfg.clock.rdiv & 0x1F) << 18;                                  // RDIV [18:22]
  clk |= (static_cast<uint32_t>(cfg.clock.sysclk_div) & 0x3) << 23;      // SYS_CLK_DIV [23:24]
  if (!write32(clk)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write clock config (0x%08X)", clk);
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "Clock configuration written (external: %s, xtal_drive: %d, rdiv: %d)",
                    cfg.clock.use_external == bootcfg::ClockSource::External ? "yes" : "no",
                    static_cast<int>(cfg.clock.xtal_drive), cfg.clock.rdiv);

  // ⚠️ CRITICAL: Wait for clock reconfiguration to complete
  // According to datasheet: "Any change to the clock configuration takes multiple milliseconds to
  // apply. The TMC9660 cannot respond to commands during clock reconfiguration."
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "Waiting for clock reconfiguration to complete...");

  // Fixed delay to allow clock reconfiguration to complete
  // The TMC9660 cannot respond to commands during this process
  comm_.delayMs(100); // 100ms should be sufficient for clock reconfiguration

  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Clock reconfiguration delay completed");

  // Verify clock config was written correctly and PLL_STATUS is SET
  uint32_t actual_clock;
  if (!read32(&actual_clock)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to read back clock config");
    if (fail_on_verify_error) {
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "⚠️  Clock config read failed (continuing)");
    }
  } else {
    // First check: PLL_STATUS must be SET after clock reconfiguration
    if (!(actual_clock & (1u << 30))) {
      if (fail_on_verify_error) {
        TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                          "❌ PLL_STATUS not set after clock reconfiguration: 0x%08X", actual_clock);
        TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                          "   Clock reconfiguration may have failed or needs more time");
        return false;
      } else {
        TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                          "⚠️  PLL_STATUS not set after clock reconfiguration: 0x%08X (continuing)",
                          actual_clock);
      }
    } else {
      TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "✅ PLL_STATUS confirmed SET: 0x%08X",
                        actual_clock);
    }

    // Second check: Verify the actual configuration matches expected (excluding PLL_STATUS)
    uint32_t expected_config = clk;
    uint32_t actual_config = actual_clock & ~(1u << 30); // Mask out PLL_STATUS bit for comparison

    if (actual_config != expected_config) {
      if (fail_on_verify_error) {
        TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                          "❌ Clock config verification failed: expected=0x%08X, actual=0x%08X",
                          expected_config, actual_config);
        TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                          "   Full actual value: 0x%08X (PLL_STATUS=%s)", actual_clock,
                          (actual_clock & (1u << 30)) ? "SET" : "CLEAR");
        return false;
      } else {
        TMC9660_LOG_DEBUG(
            comm_, 1, "TMC9660Bootloader",
            "⚠️  Clock config verification failed: expected=0x%08X, actual=0x%08X (continuing)",
            expected_config, actual_config);
      }
    } else {
      TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                        "✅ Clock config verified: 0x%08X (PLL_STATUS=%s)", actual_clock,
                        (actual_clock & (1u << 30)) ? "SET" : "CLEAR");
    }
  }

  // UART addresses (offset 0x02)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Configuring UART addresses");
  if (!setAddress(bootaddr::UART_ADDR)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set UART address register");
    return false;
  }
  uint16_t addr_word = static_cast<uint16_t>(cfg.uart.device_address) |
                      (static_cast<uint16_t>(cfg.uart.host_address) << 8);
  if (!write16(addr_word)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write UART address word (0x%04X)",
                      addr_word);
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "UART addresses configured (device: %d, host: %d)", cfg.uart.device_address,
                    cfg.uart.host_address);

  // Verify UART addresses were written correctly
  if (!readAndVerify16(addr_word, "UART addresses")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "UART addresses verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  UART addresses verification failed (continuing)");
    }
  }

  // RS485 delays
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Configuring RS485 delays");
  if (!setAddress(bootaddr::RS485_DELAY)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set RS485 delay register");
    return false;
  }
  uint16_t rs485 = static_cast<uint16_t>(cfg.rs485.txen_post_delay) |
                   (static_cast<uint16_t>(cfg.rs485.txen_pre_delay) << 8);
  if (!write16(rs485)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write RS485 delay config (0x%08X)",
                      rs485);
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "RS485 delays configured (pre: %d, post: %d)",
                    cfg.rs485.txen_pre_delay, cfg.rs485.txen_post_delay);

  // Verify RS485 delays were written correctly
  if (!readAndVerify16(rs485, "RS485 delays")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "RS485 delays verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  RS485 delays verification failed (continuing)");
    }
  }

  // Communication configuration (UART/SPI/RS485) - SHARED REGISTER with SPI Flash
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "Configuring communication settings (shared register with SPI flash)");
  if (!setAddress(bootaddr::COMM_CONFIG)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set communication config register");
    return false;
  }

  // Read current communication config before overwriting
  uint16_t current_comm = 0;
  if (read16(&current_comm)) {
    TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Current COMM_CONFIG before overwrite: 0x%04X",
                      current_comm);
    TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  Current Bit 0 (UART disabled): %d",
                      (current_comm >> 0) & 0x1);
    TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  Current Bit 1 (SPI disabled): %d",
                      (current_comm >> 1) & 0x1);
    TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  Current Bit 2 (SPI SELECT): %d",
                      (current_comm >> 2) & 0x1);
    TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  Current Bit 3 (UART RX): %d",
                      (current_comm >> 3) & 0x1);
    TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  Current Bit 4 (UART TX): %d",
                      (current_comm >> 4) & 0x1);
    TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  Current Bits 5-6 (UART TXEN): %d",
                      (current_comm >> 5) & 0x3);
    TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  Current Bits 7-9 (UART BAUD): %d",
                      (current_comm >> 7) & 0x7);
    TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  Current Bit 10 (SPI0 SCK): %d",
                      (current_comm >> 10) & 0x1);
  } else {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                      "⚠️  Failed to read current COMM_CONFIG (continuing)");
  }

  // Build the shared register value (offset 6) - 16-bit write (highest bit is 10)
  uint16_t comm = 0;

  // UART Configuration
  comm |= (cfg.uart.disable_uart ? 1u : 0u) << 0; // BL_DISABLE_UART (bit 0)
  comm |= (static_cast<uint16_t>(cfg.uart.rx_pin) & 0x1)
          << 3; // BL_UART_RX (bit 3): 0=GPIO7, 1=GPIO1
  comm |= (static_cast<uint16_t>(cfg.uart.tx_pin) & 0x1)
          << 4; // BL_UART_TX (bit 4): 0=GPIO6, 1=GPIO0
  comm |= (static_cast<uint16_t>(cfg.rs485.txen_pin) & 0x3) << 5; // BL_UART_TXEN (bits 5-6)
  comm |= (static_cast<uint16_t>(cfg.uart.baud_rate) & 0x7) << 7; // BL_UART_BAUDRATE (bits 7-9)

  // SPI Configuration
  comm |= (cfg.spiComm.disable_spi ? 1u : 0u) << 1; // BL_DISABLE_SPI (bit 1)

  // ============================================================================
  // CRITICAL: Handle shared BL_SPI_SELECT (bit 2) and BL_SPI0_SCK (bit 10)
  // ============================================================================
  // These bits are SHARED between bootloader comm (offset 6) and flash (offset 10).
  // Per datasheet: "If both flash and bootloader SPI communication are used,
  // they must always be on separate SPI interfaces."
  //
  // Priority rules:
  //   1. If bootloader SPI enabled: bootloader config controls BL_SPI_SELECT
  //   2. If bootloader SPI disabled but flash enabled: flash config controls it
  //   3. Bootloader communication ALWAYS takes priority over flash
  // ============================================================================

  // Determine which physical SPI interface each component wants to use
  bool bootloader_uses_spi0 = !cfg.spiComm.disable_spi &&
                              (cfg.spiComm.boot_spi_iface == tmc9660::bootcfg::SPIInterface::SPI0);
  bool flash_uses_spi0 = cfg.spiFlash.enable_flash &&
                         (cfg.spiFlash.flash_spi_iface == tmc9660::bootcfg::SPIInterface::SPI0);

  // Validate configuration: both cannot use the same SPI interface
  if (!cfg.spiComm.disable_spi && cfg.spiFlash.enable_flash) {
    if (bootloader_uses_spi0 == flash_uses_spi0) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "⚠️  CONFIGURATION ERROR:");
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                        "    Both bootloader and flash are configured for %s!",
                        bootloader_uses_spi0 ? "SPI0" : "SPI1");
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                        "    Per datasheet: they MUST be on separate interfaces");
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "    Current config:");
      TMC9660_LOG_DEBUG(
          comm_, 0, "TMC9660Bootloader", "      - Bootloader SPI: %s (disable_spi=%d)",
          cfg.spiComm.boot_spi_iface == tmc9660::bootcfg::SPIInterface::SPI0 ? "SPI0" : "SPI1",
          cfg.spiComm.disable_spi ? 1 : 0);
      TMC9660_LOG_DEBUG(
          comm_, 0, "TMC9660Bootloader", "      - Flash SPI: %s (enable_flash=%d)",
          cfg.spiFlash.flash_spi_iface == tmc9660::bootcfg::SPIInterface::SPI0 ? "SPI0" : "SPI1",
          cfg.spiFlash.enable_flash ? 1 : 0);
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                        "    Fix: Set one to SPI0 and the other to SPI1");
      return false;
    }
  }

  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "SPI interface allocation:");
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "  - Bootloader: %s (enabled=%d)",
                    bootloader_uses_spi0 ? "SPI0" : "SPI1", !cfg.spiComm.disable_spi ? 1 : 0);
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "  - Flash: %s (enabled=%d)",
                    flash_uses_spi0 ? "SPI0" : "SPI1", cfg.spiFlash.enable_flash ? 1 : 0);

  // ============================================================================
  // Set BL_SPI_SELECT (bit 2) with correct bit inversion logic
  // ============================================================================
  // CRITICAL: BL_SPI_SELECT has OPPOSITE meanings for bootloader vs flash!
  //   For Bootloader: 0=SPI0, 1=SPI1
  //   For Flash:      0=SPI1, 1=SPI0 (inverted!)
  //
  // Priority: Bootloader config takes precedence over flash config
  // ============================================================================
  uint8_t bl_spi_select_bit;
  if (!cfg.spiComm.disable_spi) {
    // Bootloader SPI enabled - use bootloader's interface selection directly
    bl_spi_select_bit =
        (cfg.spiComm.boot_spi_iface == tmc9660::bootcfg::SPIInterface::SPI0) ? 0 : 1;
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "  - BL_SPI_SELECT=%d (bootloader on %s)",
                      bl_spi_select_bit, bootloader_uses_spi0 ? "SPI0" : "SPI1");
  } else if (cfg.spiFlash.enable_flash) {
    // Only flash enabled - use INVERTED flash interface selection
    bl_spi_select_bit =
        (cfg.spiFlash.flash_spi_iface == tmc9660::bootcfg::SPIInterface::SPI0) ? 1 : 0;
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                      "  - BL_SPI_SELECT=%d (flash on %s, inverted bit)", bl_spi_select_bit,
                      flash_uses_spi0 ? "SPI0" : "SPI1");
  } else {
    // Neither enabled - default to 0
    bl_spi_select_bit = 0;
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                      "  - BL_SPI_SELECT=%d (default, neither enabled)", bl_spi_select_bit);
  }
  comm |= (bl_spi_select_bit & 0x1) << 2;

  // Handle shared BL_SPI0_SCK bit (bit 10) - only relevant if SPI0 is being used
  if (bootloader_uses_spi0) {
    comm |= (static_cast<uint16_t>(cfg.spiComm.spi0_sck_pin) & 0x1)
            << 10; // BL_SPI0_SCK (bit 10): 0=GPIO6, 1=GPIO11
    TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                      "BL_SPI0_SCK set from bootloader config (SPI0_SCK pin: %d)",
                      static_cast<int>(cfg.spiComm.spi0_sck_pin));
  } else if (flash_uses_spi0) {
    // Flash uses SPI0, so set SCK pin based on flash config
    comm |= (static_cast<uint32_t>(cfg.spiFlash.spi0_sck_pin) & 0x1) << 10;
    TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                      "BL_SPI0_SCK set from flash config (SPI0_SCK pin: %d)",
                      static_cast<int>(cfg.spiFlash.spi0_sck_pin));
  }
  // If neither uses SPI0, leave BL_SPI0_SCK as 0 (default)

  // Debug: Show detailed bit breakdown of NEW configuration
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "NEW COMM_CONFIG to be written: 0x%04X", comm);
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  New Bit 0 (UART disabled): %d",
                    (comm >> 0) & 0x1);
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  New Bit 1 (SPI disabled): %d",
                    (comm >> 1) & 0x1);
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  New Bit 2 (SPI SELECT): %d",
                    (comm >> 2) & 0x1);
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  New Bit 3 (UART RX): %d", (comm >> 3) & 0x1);
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  New Bit 4 (UART TX): %d", (comm >> 4) & 0x1);
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  New Bits 5-6 (UART TXEN): %d",
                    (comm >> 5) & 0x3);
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  New Bits 7-9 (UART BAUD): %d (expected: %d)",
                    (comm >> 7) & 0x7, static_cast<int>(cfg.uart.baud_rate));
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "  New Bit 10 (SPI0 SCK): %d",
                    (comm >> 10) & 0x1);

  if (!write16(comm)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                      "Failed to write communication config (0x%04X)", comm);
    return false;
  }
  // COMM_CONFIG can drive internal SPI0 pin mux (BL_SPI0_SCK bit 10). Allow a short settle
  // and re-arm the address pointer before readback — avoids spurious 0x0000 verify on some boards.
  if (!setAddress(bootaddr::COMM_CONFIG)) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                      "⚠️  Failed to re-set COMM_CONFIG address before verify (continuing)");
  }
  comm_.delayMs(2);
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Communication config written (0x%04X):", comm);
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "  UART: disabled=%s, RX=GPIO%d, TX=GPIO%d, TXEN=%s, baud=%d",
                    cfg.uart.disable_uart ? "yes" : "no",
                    cfg.uart.rx_pin == tmc9660::bootcfg::UartRxPin::GPIO7 ? 7 : 1,
                    cfg.uart.tx_pin == tmc9660::bootcfg::UartTxPin::GPIO6 ? 6 : 0,
                    cfg.rs485.txen_pin == tmc9660::bootcfg::RS485TxEnPin::GPIO8   ? "GPIO8"
                    : cfg.rs485.txen_pin == tmc9660::bootcfg::RS485TxEnPin::GPIO2 ? "GPIO2"
                                                                                  : "None",
                    static_cast<int>(cfg.uart.baud_rate));
  TMC9660_LOG_DEBUG(
      comm_, 3, "TMC9660Bootloader",
      "  SPI: disabled=%s, bootloader_SPI0=%s, flash_SPI0=%s, SPI0_SCK=GPIO%d",
      cfg.spiComm.disable_spi ? "yes" : "no", bootloader_uses_spi0 ? "yes" : "no",
      flash_uses_spi0 ? "yes" : "no",
      bootloader_uses_spi0
          ? (cfg.spiComm.spi0_sck_pin == tmc9660::bootcfg::SPI0SckPin::GPIO6 ? 6 : 11)
      : flash_uses_spi0
          ? (cfg.spiFlash.spi0_sck_pin == tmc9660::bootcfg::SPI0SckPin::GPIO6 ? 6 : 11)
          : 0);

  // Verify communication config was written correctly
  if (!readAndVerify16(comm, "Communication config")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Communication config verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  Communication config verification failed (continuing)");
    }
  }

  // ============================================================================
  // SPI Flash Configuration (offset 10)
  // ============================================================================
  // NOTE: This register mirrors some bits from COMM_CONFIG (offset 6):
  //   - Bit 1: SPI_IFACE_SELECT (mirrors BL_SPI_SELECT from COMM_CONFIG bit 2)
  //   - Bit 2: SPI0_SCK_SELECT (mirrors BL_SPI0_SCK from COMM_CONFIG bit 10)
  //
  // These bits were determined above based on priority:
  //   - If bootloader SPI enabled: bootloader config takes priority
  //   - If bootloader SPI disabled but flash enabled: flash config is used
  //
  // This register (offset 10) contains:
  //   - Bit 0: SPI_FLASH_EN
  //   - Bit 1: SPI_IFACE_SELECT (mirrors COMM_CONFIG bit 2)
  //   - Bit 2: SPI0_SCK_SELECT (mirrors COMM_CONFIG bit 10)
  //   - Bits 3-7: SPI_FLASH_CS (chip select pin)
  //   - Bits 8-11: SPI_FLASH_FREQ (frequency divider)
  // ============================================================================

  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "Configuring SPI flash settings");
  if (!setAddress(bootaddr::SPI_FLASH)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set SPI flash register");
    return false;
  }

  // Determine SCK pin bit value (mirrors BL_SPI0_SCK from COMM_CONFIG bit 10)
  uint8_t sck_pin_bit = 0;
  if (bootloader_uses_spi0) {
    sck_pin_bit = (cfg.spiComm.spi0_sck_pin == tmc9660::bootcfg::SPI0SckPin::GPIO6) ? 0 : 1;
  } else if (flash_uses_spi0) {
    sck_pin_bit = (cfg.spiFlash.spi0_sck_pin == tmc9660::bootcfg::SPI0SckPin::GPIO6) ? 0 : 1;
  }
  // If neither uses SPI0, sck_pin_bit remains 0 (default)

  uint16_t flash = (cfg.spiFlash.enable_flash ? 1u : 0u); // SPI_FLASH_EN (bit 0)
  flash |= (static_cast<uint16_t>(bl_spi_select_bit & 0x1))
           << 1; // SPI_IFACE_SELECT (bit 1, mirrors COMM_CONFIG bit 2)
  flash |= (static_cast<uint16_t>(sck_pin_bit & 0x1))
           << 2; // SPI0_SCK_SELECT (bit 2, mirrors COMM_CONFIG bit 10)
  flash |= (static_cast<uint16_t>(cfg.spiFlash.cs_pin & 0x1F)) << 3;  // SPI_FLASH_CS (bits 3-7)
  flash |= (static_cast<uint16_t>(cfg.spiFlash.freq_div) & 0xF) << 8; // SPI_FLASH_FREQ (bits 8-11)

  if (!write16(flash)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write SPI flash config (0x%04X)",
                      flash);
    return false;
  }

  // Determine which interface and SCK pin are actually being used (from COMM_CONFIG)
  const char* actual_iface = flash_uses_spi0 ? "SPI0" : "SPI1";
  const char* actual_sck =
      flash_uses_spi0
          ? (cfg.spiFlash.spi0_sck_pin == tmc9660::bootcfg::SPI0SckPin::GPIO6 ? "GPIO6" : "GPIO11")
          : "N/A";

  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "SPI flash config written: 0x%04X", flash);
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "  - SPI_FLASH_EN (bit 0): %d",
                    cfg.spiFlash.enable_flash ? 1 : 0);
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                    "  - SPI_IFACE_SELECT (bit 1): %d (mirrors COMM_CONFIG bit 2, %s)",
                    bl_spi_select_bit, actual_iface);
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                    "  - SPI0_SCK_SELECT (bit 2): %d (mirrors COMM_CONFIG bit 10, %s)", sck_pin_bit,
                    actual_sck);
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "  - CS_PIN (bits 3-7): GPIO%d",
                    cfg.spiFlash.cs_pin);
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "  - FREQ_DIV (bits 8-11): Div%d",
                    1 << static_cast<int>(cfg.spiFlash.freq_div));

  // Verify SPI flash config — re-arm address after write (UART historically needed this; SPI
  // benefits when COMM_CONFIG and flash share mux-related fields).
  if (!setAddress(bootaddr::SPI_FLASH)) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                      "⚠️  Failed to set SPI_FLASH address before verify (continuing)");
  }
  comm_.delayMs(1);

  if (!readAndVerify16(flash, "SPI flash config")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "SPI flash config verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  SPI flash config verification failed (continuing)");
    }
  }

  // I2C EEPROM configuration
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Configuring I2C EEPROM settings");
  if (!setAddress(bootaddr::I2C_CONFIG)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set I2C config register");
    return false;
  }
  uint16_t i2c = (cfg.i2c.enable_eeprom ? 1u : 0u);                // I2C_EN (bit 0)
  i2c |= (static_cast<uint16_t>(cfg.i2c.sda_pin) & 0x3) << 1;      // I2C_SDA (bits 1-2)
  i2c |= (static_cast<uint16_t>(cfg.i2c.scl_pin) & 0x3) << 3;      // I2C_SCL (bits 3-4)
  i2c |= (static_cast<uint16_t>(cfg.i2c.address_bits & 0x7)) << 5; // I2C_ADDR_BITS (bits 5-7)
  i2c |= (static_cast<uint16_t>(cfg.i2c.freq_code) & 0x7) << 8;    // I2C_FREQ (bits 8-10)
  if (!write16(i2c)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write I2C config (0x%04X)", i2c);
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "I2C EEPROM configured (enabled: %s, SDA: %d, SCL: %d, addr_bits: %d)",
                    cfg.i2c.enable_eeprom ? "yes" : "no", static_cast<int>(cfg.i2c.sda_pin),
                    static_cast<int>(cfg.i2c.scl_pin), cfg.i2c.address_bits);

  // Verify I2C config was written correctly
  // UART mode: address has an issue if not reset
  if (comm_.mode() == CommMode::UART) {
    if (!setAddress(bootaddr::I2C_CONFIG)) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                        "Failed to reset I2C config register address for verification");
      if (fail_on_verify_error)
        return false;
    }
  }

  if (!readAndVerify16(i2c, "I2C config")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "I2C config verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  I2C config verification failed (continuing)");
    }
  }

  // GPIO configuration (according to datasheet specification)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Configuring GPIO settings");

  // GPIO output levels for GPIOs 0-15 (offset 14)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Configuring GPIO output levels (GPIOs 0-15)");
  if (!setAddress(bootaddr::GPIO_OUT)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set GPIO output register");
    return false;
  }
  if (!write16(cfg.gpio.outputMask_0_15)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write GPIO output mask (0x%04X)",
                      cfg.gpio.outputMask_0_15);
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "GPIO output levels configured (GPIOs 0-15: 0x%04X)", cfg.gpio.outputMask_0_15);

  // Verify GPIO output levels were written correctly
  if (!readAndVerify16(cfg.gpio.outputMask_0_15, "GPIO output levels")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "GPIO output levels verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  GPIO output levels verification failed (continuing)");
    }
  }

  // GPIO direction (output enable) for GPIOs 0-15 (offset 16)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Configuring GPIO direction (GPIOs 0-15)");
  if (!setAddress(bootaddr::GPIO_DIR)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set GPIO direction register");
    return false;
  }
  if (!write16(cfg.gpio.directionMask_0_15)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write GPIO direction mask (0x%04X)",
                      cfg.gpio.directionMask_0_15);
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "GPIO direction configured (GPIOs 0-15: 0x%04X)",
                    cfg.gpio.directionMask_0_15);

  // Verify GPIO direction was written correctly
  if (!readAndVerify16(cfg.gpio.directionMask_0_15, "GPIO direction")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "GPIO direction verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  GPIO direction verification failed (continuing)");
    }
  }

  // GPIO pull-up enable for GPIOs 0-15 (offset 18)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Configuring GPIO pull-up (GPIOs 0-15)");
  if (!setAddress(bootaddr::GPIO_PU)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set GPIO pull-up register");
    return false;
  }
  if (!write16(cfg.gpio.pullUpMask_0_15)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write GPIO pull-up mask (0x%04X)",
                      cfg.gpio.pullUpMask_0_15);
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "GPIO pull-up configured (GPIOs 0-15: 0x%04X)",
                    cfg.gpio.pullUpMask_0_15);

  // Verify GPIO pull-up was written correctly
  if (!readAndVerify16(cfg.gpio.pullUpMask_0_15, "GPIO pull-up")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "GPIO pull-up verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  GPIO pull-up verification failed (continuing)");
    }
  }

  // GPIO pull-down enable for GPIOs 0-15 (offset 20)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Configuring GPIO pull-down (GPIOs 0-15)");
  if (!setAddress(bootaddr::GPIO_PD)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set GPIO pull-down register");
    return false;
  }
  if (!write16(cfg.gpio.pullDownMask_0_15)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write GPIO pull-down mask (0x%04X)",
                      cfg.gpio.pullDownMask_0_15);
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "GPIO pull-down configured (GPIOs 0-15: 0x%04X)",
                    cfg.gpio.pullDownMask_0_15);

  // Verify GPIO pull-down was written correctly
  if (!readAndVerify16(cfg.gpio.pullDownMask_0_15, "GPIO pull-down")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "GPIO pull-down verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  GPIO pull-down verification failed (continuing)");
    }
  }

  // GPIO extended configuration (offset 22) - GPIOs 16-18 + analog GPIOs 2-5
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "Configuring GPIO extended settings (GPIOs 16-18 + analog 2-5)");
  if (!setAddress(bootaddr::GPIO_EXT)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set GPIO extended register");
    return false;
  }
  uint16_t gpio_ext = 0;
  gpio_ext |= (cfg.gpio.outputMask_16_18 & 0x7);         // Bits 0-2: GPIO16-18 output levels
  gpio_ext |= (cfg.gpio.directionMask_16_18 & 0x7) << 3; // Bits 3-5: GPIO16-18 direction
  gpio_ext |= (cfg.gpio.pullDownMask_16_18 & 0x7) << 6;  // Bits 6-8: GPIO16-18 pull-down
  gpio_ext |= (cfg.gpio.pullUpMask_16_18 & 0x7) << 9;    // Bits 9-11: GPIO16-18 pull-up
  gpio_ext |= (cfg.gpio.analogMask_2_5 & 0xF) << 12;     // Bits 12-15: GPIO2-5 analog enable
  if (!write16(gpio_ext)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                      "Failed to write GPIO extended config (0x%04X)", gpio_ext);
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "GPIO extended configured (output_16_18:0x%X, dir_16_18:0x%X, pd_16_18:0x%X, "
                    "pu_16_18:0x%X, analog_2_5:0x%X)",
                    cfg.gpio.outputMask_16_18, cfg.gpio.directionMask_16_18,
                    cfg.gpio.pullDownMask_16_18, cfg.gpio.pullUpMask_16_18,
                    cfg.gpio.analogMask_2_5);

  // Verify GPIO extended config was written correctly
  if (!readAndVerify16(gpio_ext, "GPIO extended config")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "GPIO extended config verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  GPIO extended config verification failed (continuing)");
    }
  }

  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "GPIO configuration completed");

  // Hall encoder and ABN encoder 1 configuration (offset 0x20 - shared register)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "Configuring Hall encoder and ABN encoder 1 settings");
  if (!setAddress(bootaddr::HALL_CONFIG)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set Hall/ABN1 config register");
    return false;
  }
  uint16_t hall_abn1 = 0;
  // Hall encoder configuration
  hall_abn1 |= (cfg.hall.enable ? 1u : 0u);                        // Bit 0: HALL_ENABLE
  hall_abn1 |= (static_cast<uint16_t>(cfg.hall.u_pin) & 0x3) << 4; // Bits 4-5: HALL_U
  hall_abn1 |= (static_cast<uint16_t>(cfg.hall.v_pin) & 0x3) << 6; // Bits 6-7: HALL_V
  hall_abn1 |= (static_cast<uint16_t>(cfg.hall.w_pin) & 0x3) << 8; // Bits 8-9: HALL_W
  // ABN encoder 1 configuration
  hall_abn1 |= (cfg.abn1.enable ? 1u : 0u) << 1;                    // Bit 1: ABN1_ENABLE
  hall_abn1 |= (static_cast<uint16_t>(cfg.abn1.a_pin) & 0x3) << 10; // Bits 10-11: ABN1_A
  hall_abn1 |= (static_cast<uint16_t>(cfg.abn1.b_pin) & 0x3) << 12; // Bits 12-13: ABN1_B
  hall_abn1 |= (static_cast<uint16_t>(cfg.abn1.n_pin) & 0x3) << 14; // Bits 14-15: ABN1_N
  if (!write16(hall_abn1)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write Hall/ABN1 config (0x%08X)",
                      hall_abn1);
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "Hall/ABN1 configured - Hall(enabled:%s, U:%d, V:%d, W:%d), ABN1(enabled:%s, "
                    "A:%d, B:%d, N:%d)",
                    cfg.hall.enable ? "yes" : "no", static_cast<int>(cfg.hall.u_pin),
                    static_cast<int>(cfg.hall.v_pin), static_cast<int>(cfg.hall.w_pin),
                    cfg.abn1.enable ? "yes" : "no", static_cast<int>(cfg.abn1.a_pin),
                    static_cast<int>(cfg.abn1.b_pin), static_cast<int>(cfg.abn1.n_pin));

  // Verify Hall/ABN1 config was written correctly
  if (!readAndVerify16(hall_abn1, "Hall/ABN1 config")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Hall/ABN1 config verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  Hall/ABN1 config verification failed (continuing)");
    }
  }

  // ABN encoder 2, Reference switches, and Step/Direction configuration (offset 0x22)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "Configuring ABN2, REF switches, and Step/Dir settings");
  if (!setAddress(bootaddr::ABN2_CONFIG)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                      "Failed to set ABN2/REF/StepDir config register");
    return false;
  }
  uint16_t abn2_ref_stepdir = 0;
  // Reference switches (bits 0-6)
  abn2_ref_stepdir |= static_cast<uint16_t>(cfg.ref.ref_l_pin) & 0x3;        // Bits 0-1: REF_L_PIN
  abn2_ref_stepdir |= (static_cast<uint16_t>(cfg.ref.ref_r_pin) & 0x3) << 2; // Bits 2-3: REF_R_PIN
  abn2_ref_stepdir |= (static_cast<uint16_t>(cfg.ref.ref_h_pin) & 0x7) << 4; // Bits 4-6: REF_H_PIN
  // Step/Direction (bits 8-11)
  abn2_ref_stepdir |= (cfg.stepDir.enable ? 1u : 0u) << 8; // Bit 8: STEPDIR_ENABLE
  abn2_ref_stepdir |= (static_cast<uint16_t>(cfg.stepDir.step_pin) & 0x3)
                      << 9; // Bits 9-10: STEP_PIN
  abn2_ref_stepdir |= (static_cast<uint16_t>(cfg.stepDir.dir_pin) & 0x1) << 11; // Bit 11: DIR_PIN
  // ABN encoder 2 (bits 12-15)
  abn2_ref_stepdir |= (cfg.abn2.enable ? 1u : 0u) << 12;                   // Bit 12: ABN2_ENABLE
  abn2_ref_stepdir |= (static_cast<uint16_t>(cfg.abn2.a_pin) & 0x1) << 13; // Bit 13: ABN2_A
  abn2_ref_stepdir |= (static_cast<uint16_t>(cfg.abn2.b_pin) & 0x3) << 14; // Bits 14-15: ABN2_B
  if (!write16(abn2_ref_stepdir)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                      "Failed to write ABN2/REF/StepDir config (0x%08X)", abn2_ref_stepdir);
    return false;
  }
  TMC9660_LOG_DEBUG(
      comm_, 3, "TMC9660Bootloader",
      "ABN2/REF/StepDir configured (ABN2:%s, REF_L:%d, REF_R:%d, REF_H:%d, StepDir:%s)",
      cfg.abn2.enable ? "enabled" : "disabled", static_cast<int>(cfg.ref.ref_l_pin),
      static_cast<int>(cfg.ref.ref_r_pin), static_cast<int>(cfg.ref.ref_h_pin),
      cfg.stepDir.enable ? "enabled" : "disabled");

  // Verify ABN2/REF/StepDir config was written correctly
  if (!readAndVerify16(abn2_ref_stepdir, "ABN2/REF/StepDir config")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                        "ABN2/REF/StepDir config verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  ABN2/REF/StepDir config verification failed (continuing)");
    }
  }

  // Mechanical brake and Brake chopper configuration (offset 0x24)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "Configuring mechanical brake and brake chopper settings");
  if (!setAddress(bootaddr::MECH_BRAKE_CONFIG)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set brake config register");
    return false;
  }
  uint16_t brake = 0;
  // Brake chopper (bits 4-9)
  brake |= (cfg.brakeChopper.enable ? 1u : 0u) << 4; // Bit 4: BRAKECHOPPER_ENABLE
  brake |= (static_cast<uint16_t>(cfg.brakeChopper.output_pin) & 0x1F)
           << 5; // Bits 5-9: BRAKECHOPPER_OUTPUT
  // Mechanical brake (bits 12-14)
  brake |= (cfg.mechBrake.enable ? 1u : 0u) << 12; // Bit 12: MECH_BRAKE_ENABLE
  brake |= (static_cast<uint16_t>(cfg.mechBrake.output_pin) & 0x3)
           << 13; // Bits 13-14: MECH_BRAKE_OUTPUT
  if (!write16(brake)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write brake config (0x%04X)",
                      brake);
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "Brake configuration completed (mech_brake:%s, brake_chopper:%s)",
                    cfg.mechBrake.enable ? "enabled" : "disabled",
                    cfg.brakeChopper.enable ? "enabled" : "disabled");

  // Verify brake config was written correctly
  if (!readAndVerify16(brake, "Brake config")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Brake config verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  Brake config verification failed (continuing)");
    }
  }

  // SPI encoder configuration (offset 0x26)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Configuring SPI encoder settings");
  if (!setAddress(bootaddr::SPI_ENC_CONFIG)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set SPI encoder config register");
    return false;
  }
  uint16_t spiEnc = (cfg.spiEnc.enable ? 1u : 0u);                       // Bit 0: SPI_ENC_ENABLE
  spiEnc |= (static_cast<uint16_t>(cfg.spiEnc.spi_block) & 0x1) << 1;    // Bit 1: SPI_ENC_BLOCK
  spiEnc |= (static_cast<uint16_t>(cfg.spiEnc.spi_mode) & 0x3) << 2;     // Bits 2-3: SPI_ENC_MODE
  spiEnc |= (static_cast<uint16_t>(cfg.spiEnc.spi_freq) & 0xF) << 4;     // Bits 4-7: SPI_ENC_FREQ
  spiEnc |= (static_cast<uint16_t>(cfg.spiEnc.cs_pin) & 0x3) << 8;       // Bits 8-9: SPI_ENC_CS_PIN
  spiEnc |= (static_cast<uint16_t>(cfg.spiEnc.cs_polarity) & 0x1) << 10; // Bit 10: SPI_ENC_CS_POL
  if (!write16(spiEnc)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write SPI encoder config (0x%04X)",
                      spiEnc);
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "SPI encoder configured (enabled: %s, block:%d, mode:%d, freq:%d)",
                    cfg.spiEnc.enable ? "yes" : "no", static_cast<int>(cfg.spiEnc.spi_block),
                    static_cast<int>(cfg.spiEnc.spi_mode), static_cast<int>(cfg.spiEnc.spi_freq));

  // Verify SPI encoder config was written correctly
  if (!readAndVerify16(spiEnc, "SPI encoder config")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "SPI encoder config verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  SPI encoder config verification failed (continuing)");
    }
  }

  // External memory storage configuration (offset 0x28)
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Configuring external memory storage settings");
  if (!setAddress(bootaddr::MEM_STORAGE_CONFIG)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                      "Failed to set memory storage config register");
    return false;
  }
  uint8_t mem_storage =
      static_cast<uint8_t>(cfg.memStorage.tmcl_script) & 0x3; // Bits 0-1: MEM_TMCL_SCRIPT
  mem_storage |= (static_cast<uint8_t>(cfg.memStorage.parameters) & 0x3)
                << 2; // Bits 2-3: MEM_PARAMETERS
  if (!write8(mem_storage)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                      "Failed to write memory storage config (0x%02X)", mem_storage);
    return false;
  }
  TMC9660_LOG_DEBUG(
      comm_, 3, "TMC9660Bootloader", "Memory storage configured (TMCL_script:%d, parameters:%d)",
      static_cast<int>(cfg.memStorage.tmcl_script), static_cast<int>(cfg.memStorage.parameters));

  // Verify memory storage config was written correctly
  if (!readAndVerify8(mem_storage, "Memory storage config")) {
    if (fail_on_verify_error) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Memory storage config verification failed");
      return false;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  Memory storage config verification failed (continuing)");
    }
  }

  // ========================================================================
  // External Memory Verification (if enabled)
  // ========================================================================

  // Check SPI Flash if enabled
  if (cfg.spiFlash.enable_flash) {
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "========================================");
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "Verifying SPI Flash Memory Configuration");
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "========================================");
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "Expected config:");
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "  - SPI_FLASH_EN: %s (bit 0)",
                      cfg.spiFlash.enable_flash ? "ENABLED" : "DISABLED");
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "  - CS Pin: GPIO%d (bits 3-7)",
                      cfg.spiFlash.cs_pin);
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "  - Frequency: Div%d (bits 8-11)",
                      1 << static_cast<int>(cfg.spiFlash.freq_div));
    TMC9660_LOG_DEBUG(
        comm_, 2, "TMC9660Bootloader", "  - SPI Interface: %s (bit 12)",
        cfg.spiFlash.flash_spi_iface == tmc9660::bootcfg::SPIInterface::SPI0 ? "SPI0" : "SPI1");
    TMC9660_LOG_DEBUG(
        comm_, 2, "TMC9660Bootloader", "  - SCK Pin: %s (bit 13)",
        cfg.spiFlash.spi0_sck_pin == tmc9660::bootcfg::SPI0SckPin::GPIO6 ? "GPIO6" : "GPIO11");

    // Check if SPI Flash is configured
    uint32_t is_configured = 0;
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                      "Checking MEM_IS_CONFIGURED (bank=1, SPI Flash)...");
    if (sendCommand(static_cast<uint8_t>(BootloaderCommand::MEM_IS_CONFIGURED),
                    static_cast<uint32_t>(MemoryBank::SPI_FLASH), &is_configured)) {
      TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "MEM_IS_CONFIGURED reply: %u", is_configured);
      if (is_configured == 1) {
        TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                          "✅ SPI Flash is CONFIGURED (bootloader sees SPI_FLASH_EN=1)");

        // Check if SPI Flash is connected
        uint32_t is_connected = 0;
        TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                          "Checking MEM_IS_CONNECTED (probing flash chip)...");
        if (sendCommand(static_cast<uint8_t>(BootloaderCommand::MEM_IS_CONNECTED),
                        static_cast<uint32_t>(MemoryBank::SPI_FLASH), &is_connected)) {
          TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "MEM_IS_CONNECTED reply: %u",
                            is_connected);
          if (is_connected == 1) {
            TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                              "✅ SPI Flash is CONNECTED (chip responded to probe)");

            // Check if SPI Flash is busy
            uint32_t is_busy = 0;
            TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "Checking MEM_IS_BUSY...");
            if (sendCommand(static_cast<uint8_t>(BootloaderCommand::MEM_IS_BUSY),
                            static_cast<uint32_t>(MemoryBank::SPI_FLASH), &is_busy)) {
              TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "MEM_IS_BUSY reply: %u", is_busy);
              if (is_busy == 0) {
                TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                                  "✅ SPI Flash is READY (not busy)");
              } else {
                TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                                  "⚠️  SPI Flash is BUSY (write in progress)");
              }
            } else {
              TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                                "⚠️  Failed to check SPI Flash busy status");
            }

            // Query partition count
            uint32_t partition_count = 0;
            TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                              "Querying partition count (GET_INFO SPI_MEM_PARTITIONS)...");
            if (sendCommand(static_cast<uint8_t>(BootloaderCommand::GET_INFO),
                            static_cast<uint32_t>(InfoQuery::SPI_MEM_PARTITIONS),
                            &partition_count)) {
              TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "SPI Flash partition count: %u",
                                partition_count);
              if (partition_count == 0) {
                TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "⚠️  SPI Flash has NO partitions");
                TMC9660_LOG_DEBUG(
                    comm_, 1, "TMC9660Bootloader",
                    "    Flash is connected but not partitioned (no partition table)");
                TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                                  "    See datasheet Table 21 for partition header format");
              } else {
                TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "✅ SPI Flash has %u partition(s)",
                                  partition_count);
              }
            } else {
              TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                                "⚠️  Failed to query SPI Flash partition count");
            }
          } else {
            TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "⚠️  SPI Flash is NOT CONNECTED");
            TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "    Possible causes:");
            TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                              "    1. No flash chip physically connected");
            TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                              "    2. Wrong CS pin configured (expected GPIO%d)",
                              cfg.spiFlash.cs_pin);
            TMC9660_LOG_DEBUG(
                comm_, 1, "TMC9660Bootloader", "    3. Wrong SPI interface (configured: %s)",
                cfg.spiFlash.flash_spi_iface == tmc9660::bootcfg::SPIInterface::SPI0 ? "SPI0"
                                                                                     : "SPI1");
            TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                              "    4. Flash chip not responding to probe (0xAB, 0x90, 0x9F)");
          }
        } else {
          TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                            "⚠️  Failed to send MEM_IS_CONNECTED command");
        }
      } else {
        TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "⚠️  SPI Flash is NOT CONFIGURED");
        TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                          "    Bootloader reports SPI_FLASH_EN=0 (bit 0 of flash config)");
        TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                          "    This means the config write may have failed or been overwritten");
        TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                          "    Check that cfg.spiFlash.enable_flash = true");
      }
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  Failed to send MEM_IS_CONFIGURED command");
    }
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "========================================");
  }

  // Check I2C EEPROM if enabled
  if (cfg.i2c.enable_eeprom) {
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "Verifying I2C EEPROM memory...");

    // Check if I2C EEPROM is configured
    uint32_t is_configured = 0;
    if (sendCommand(static_cast<uint8_t>(BootloaderCommand::MEM_IS_CONFIGURED),
                    static_cast<uint32_t>(MemoryBank::I2C_EEPROM), &is_configured)) {
      if (is_configured == 1) {
        TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "✅ I2C EEPROM is CONFIGURED");

        // Check if I2C EEPROM is connected
        uint32_t is_connected = 0;
        if (sendCommand(static_cast<uint8_t>(BootloaderCommand::MEM_IS_CONNECTED),
                        static_cast<uint32_t>(MemoryBank::I2C_EEPROM), &is_connected)) {
          if (is_connected == 1) {
            TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "✅ I2C EEPROM is CONNECTED");

            // Check if I2C EEPROM is busy
            uint32_t is_busy = 0;
            if (sendCommand(static_cast<uint8_t>(BootloaderCommand::MEM_IS_BUSY),
                            static_cast<uint32_t>(MemoryBank::I2C_EEPROM), &is_busy)) {
              if (is_busy == 0) {
                TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                                  "✅ I2C EEPROM is READY (not busy)");
              } else {
                TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "⚠️  I2C EEPROM is BUSY");
              }
            } else {
              TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                                "⚠️  Failed to check I2C EEPROM busy status");
            }

            // Query partition count
            uint32_t partition_count = 0;
            if (sendCommand(static_cast<uint8_t>(BootloaderCommand::GET_INFO),
                            static_cast<uint32_t>(InfoQuery::I2C_MEM_PARTITIONS),
                            &partition_count)) {
              TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "I2C EEPROM partition count: %u",
                                partition_count);
              if (partition_count == 0) {
                TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                                  "⚠️  I2C EEPROM has NO partitions (not partitioned)");
              }
            } else {
              TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                                "⚠️  Failed to query I2C EEPROM partition count");
            }
          } else {
            TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                              "⚠️  I2C EEPROM is NOT CONNECTED (check wiring)");
          }
        } else {
          TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                            "⚠️  Failed to check I2C EEPROM connection status");
        }
      } else {
        TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                          "⚠️  I2C EEPROM is NOT CONFIGURED (check config settings)");
      }
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "⚠️  Failed to check I2C EEPROM configuration status");
    }
  }

  // ⚠️ CRITICAL: Boot configuration (offset 0x08) - WRITE THIS LAST!
  // When START_MOTOR_CTRL=1 is written, the bootloader IMMEDIATELY exits and starts motor control.
  // Any commands sent after this will fail because the bootloader is no longer running.
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "Configuring boot settings (FINAL STEP)");
  if (!setAddress(bootaddr::BOOT_CONFIG)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set boot config register");
    return false;
  }
  uint16_t boot = 0;
  boot |= static_cast<uint16_t>(cfg.boot.boot_mode) & 0x3; // Bits 0-1: BOOT_MODE
  boot |= (cfg.boot.bl_ready_fault ? 1u : 0u) << 2;        // Bit 2: BL_READY_FAULT
  boot |= (cfg.boot.bl_exit_fault ? 1u : 0u) << 3;         // Bit 3: BL_EXIT_FAULT
  boot |= (cfg.boot.disable_selftest ? 1u : 0u) << 8;      // Bit 8: DISABLE_SELFTEST
  boot |= (cfg.boot.bl_config_fault ? 1u : 0u) << 9;       // Bit 9: BL_CONFIG_FAULT
  boot |= (cfg.boot.start_motor_control ? 1u : 0u) << 12;  // Bit 12: START_MOTOR_CTRL
  if (!write16(boot)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write boot config (0x%04X)", boot);
    return false;
  }
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "Boot config written: mode=%d, start_motor_ctrl=%s, bl_exit_fault=%s",
                    static_cast<int>(cfg.boot.boot_mode),
                    cfg.boot.start_motor_control ? "TRUE" : "FALSE",
                    cfg.boot.bl_exit_fault ? "TRUE" : "FALSE");

  // CAN'T VERIFY BOOT CONFIG BECAUSE THE BOOTLOADER WILL EXIT IMMEDIATELY AFTER START_MOTOR_CTRL=1
  // IS WRITTEN
  // // Verify boot config was written correctly
  // if (!readAndVerify16(boot, "Boot config")) {
  //   if (fail_on_verify_error) {
  //     TMC9660_LOG_DEBUG(comm_,0, "TMC9660Bootloader", "Boot config verification failed");
  //     return false;
  //   } else {
  //     TMC9660_LOG_DEBUG(comm_,1, "TMC9660Bootloader", "⚠️  Boot config verification failed
  //     (continuing)");
  //   }
  // }

  if (cfg.boot.start_motor_control) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                      "⚠️  START_MOTOR_CTRL=1 written - Bootloader will hopefully EXIT and motor "
                      "control will START!");
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                      "⚠️  No further bootloader commands will be accepted after this point!");
  } else {
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                      "START_MOTOR_CTRL=0 - Bootloader remains active, motor control NOT started");
  }

  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader",
                    "Bootloader configuration application completed successfully");
  return true;
}

//==================================================
// MOTOR CONTROL STARTUP
//==================================================

/**
 * @brief Start the motor control system and exit bootloader mode.
 *
 * This function writes START_MOTOR_CTRL=1 to the BOOT_CONFIG register,
 * causing the bootloader to immediately exit and launch the motor control
 * application based on the current BOOT_MODE setting.
 *
 * @warning CRITICAL: After calling this function, the bootloader will no longer
 * respond to commands. All bootloader configuration must be completed BEFORE
 * calling this function.
 *
 * @param boot_mode Motor control mode to start (Register or Parameter mode)
 * @return true if command sent successfully, false on error
 * @note The bootloader exits immediately after this command
 * @note Allow 100-150ms for motor control to fully initialize after calling
 *
 * @code{.cpp}
 * // Configure everything first
 * bootloader.setBank(5);
 * bootloader.setAddress(0x00020002);
 * bootloader.write16(uart_config);
 * // ... more configuration ...
 *
 * // Finally, start motor control (bootloader exits here)
 * bootloader.startMotorControl(tmc9660::bootcfg::BootMode::Parameter);
 *
 * vTaskDelay(pdMS_TO_TICKS(150));  // Wait for motor control to start
 * // Now use motor control commands
 * @endcode
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::startMotorControl(bootcfg::BootMode boot_mode) noexcept {
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader", "Starting motor control system (mode: %d)...",
                    static_cast<int>(boot_mode));

  // Set bank to CONFIG (bank 5)
  if (!setBank(5)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                      "Failed to set CONFIG bank before starting motor control");
    return false;
  }

  // Set address to BOOT_CONFIG register (offset 0x08)
  if (!setAddress(bootaddr::BOOT_CONFIG)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to set BOOT_CONFIG address");
    return false;
  }

  // Read current BOOT_CONFIG to preserve other settings
  // We'll use a dummy write to get the current value via SPI reply mechanism
  // Note: This is optional - if we don't care about preserving other bits, we can skip this

  // Construct BOOT_CONFIG value with START_MOTOR_CTRL=1
  uint32_t boot_config = 0;
  boot_config |= static_cast<uint32_t>(boot_mode) & 0x3; // Bits 0-1: BOOT_MODE
  boot_config |= (1u << 12);                            // Bit 12: START_MOTOR_CTRL = 1

  // Optional: Set BL_EXIT_FAULT to get FAULTN signal when exiting (helps debugging)
  boot_config |= (1u << 3); // Bit 3: BL_EXIT_FAULT = 1

  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660Bootloader",
                    "Writing BOOT_CONFIG=0x%08X (mode=%d, START_MOTOR_CTRL=1)", boot_config,
                    static_cast<int>(boot_mode));

  // Write the boot configuration
  // ⚠️ CRITICAL: After this write completes, the bootloader will EXIT!
  if (!write32(boot_config)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to write BOOT_CONFIG");
    return false;
  }

  TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                    "⚠️  START_MOTOR_CTRL command sent - Bootloader is EXITING NOW!");
  TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "⚠️  Motor control system starting in %s mode",
                    boot_mode == bootcfg::BootMode::Parameter ? "PARAMETER" : "REGISTER");
  TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                    "⚠️  Allow 100-150ms for motor control to fully initialize");
  TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                    "⚠️  Bootloader commands will NO LONGER work after this point");

  return true;
}

//==================================================
// READ OPERATIONS
//==================================================

/**
 * @brief Read a single byte from the previously selected address.
 *
 * Reads an 8-bit value from the current address in the selected memory bank.
 * The address pointer is not incremented after the read operation.
 *
 * @param value Pointer to store the 8-bit value read
 * @return true if read was successful, false on error
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::read8(uint8_t* value) noexcept {
  if (!value) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "read8: null pointer");
    return false;
  }

  uint32_t reply;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::READ_8), 0, &reply)) {
    return false;
  }

  *value = static_cast<uint8_t>(reply);
  return true;
}

/**
 * @brief Read a 16-bit word from the previously selected address.
 *
 * Reads a 16-bit value from the current address in the selected memory bank.
 * The address pointer is not incremented after the read operation.
 *
 * @param value Pointer to store the 16-bit value read
 * @return true if read was successful, false on error
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::read16(uint16_t* value) noexcept {
  if (!value) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "read16: null pointer");
    return false;
  }

  uint32_t reply;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::READ_16), 0, &reply)) {
    return false;
  }

  *value = static_cast<uint16_t>(reply);
  return true;
}

/**
 * @brief Read a 32-bit word from the previously selected address.
 *
 * Reads a 32-bit value from the current address in the selected memory bank.
 * The address pointer is not incremented after the read operation.
 *
 * @param value Pointer to store the 32-bit value read
 * @return true if read was successful, false on error
 */
template <typename CommType>
bool TMC9660Bootloader<CommType>::read32(uint32_t* value) noexcept {
  if (!value) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "read32: null pointer");
    return false;
  }

  return sendCommand(static_cast<uint8_t>(BootloaderCommand::READ_32), 0, value);
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::read8Inc(uint8_t* value) noexcept {
  if (!value) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "read8Inc: null pointer");
    return false;
  }

  uint32_t reply;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::READ_8_INC), 0, &reply)) {
    return false;
  }

  *value = static_cast<uint8_t>(reply);
  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::read16Inc(uint16_t* value) noexcept {
  if (!value) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "read16Inc: null pointer");
    return false;
  }

  uint32_t reply;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::READ_16_INC), 0, &reply)) {
    return false;
  }

  *value = static_cast<uint16_t>(reply);
  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::read32Inc(uint32_t* value) noexcept {
  if (!value) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "read32Inc: null pointer");
    return false;
  }

  return sendCommand(static_cast<uint8_t>(BootloaderCommand::READ_32_INC), 0, value);
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::getBank(uint8_t* bank) noexcept {
  if (!bank) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "getBank: null pointer");
    return false;
  }

  uint32_t reply;
  if (!sendCommand(static_cast<uint8_t>(BootloaderCommand::GET_BANK), 0, &reply)) {
    return false;
  }

  *bank = static_cast<uint8_t>(reply);
  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::readAndVerify8(uint8_t expected, const char* configName) noexcept {
  uint8_t actual;
  if (!read8(&actual)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to read back %s", configName);
    return false;
  }

  if (actual != expected) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                      "❌ %s verification failed: expected=0x%02X, actual=0x%02X", configName,
                      expected, actual);
    return false;
  }

  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "✅ %s verified: 0x%02X", configName, actual);
  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::readAndVerify16(uint16_t expected, const char* configName) noexcept {
  uint16_t actual;
  if (!read16(&actual)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to read back %s", configName);
    return false;
  }

  if (actual != expected) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                      "❌ %s verification failed: expected=0x%04X, actual=0x%04X", configName,
                      expected, actual);
    if (actual == 0 && expected != 0) {
      TMC9660_LOG_DEBUG(
          comm_, 1, "TMC9660Bootloader",
          "   Hint: readback 0x0000 with OK status is often a timing/address-pointer issue, or "
          "COMM_CONFIG BL_SPI0_SCK (bit 10) / OTP not matching bootcfg SPI0SckPin (GPIO6 vs GPIO11) "
          "vs PCB routing.");
    }
    return false;
  }

  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "✅ %s verified: 0x%04X", configName, actual);
  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::readAndVerify32(uint32_t expected, const char* configName) noexcept {
  uint32_t actual;
  if (!read32(&actual)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Failed to read back %s", configName);
    return false;
  }

  if (actual != expected) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader",
                      "❌ %s verification failed: expected=0x%08X, actual=0x%08X", configName,
                      expected, actual);
    return false;
  }

  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660Bootloader", "✅ %s verified: 0x%08X", configName, actual);
  return true;
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::getAddress(uint32_t* address) noexcept {
  if (!address) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "getAddress: null pointer");
    return false;
  }

  return sendCommand(static_cast<uint8_t>(BootloaderCommand::GET_ADDRESS), 0, address);
}

template <typename CommType>

bool TMC9660Bootloader<CommType>::getAllBootloaderInfo() noexcept {
  TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "");
  TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                    "╔══════════════════════════════════════════════════════════════╗");
  TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                    "║          TMC9660 BOOTLOADER INFORMATION                      ║");
  TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                    "╚══════════════════════════════════════════════════════════════╝");

  bool success = true;
  uint32_t value = 0;

  // 1. Chip Type
  if (getChipType(&value)) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "Chip Type:           0x%08X %s", value,
                      (value == 0x544D0001) ? "(TMC9660 ✓)" : "(Unknown!)");
  } else {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Chip Type:           FAILED TO READ");
    success = false;
  }

  // 2. Bootloader Version
  BootloaderVersion blVer;
  if (getBootloaderVersion(&blVer)) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "Bootloader Version:  v%u.%u", blVer.major,
                      blVer.minor);
  } else {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Bootloader Version:  FAILED TO READ");
  }

  // 3. Chip Version (Silicon Revision)
  if (getChipVersion(&value)) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "Silicon Revision:    %u", value);
  } else {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Silicon Revision:    FAILED TO READ");
  }

  // 4. Chip Variant
  if (getChipVariant(&value)) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "Chip Variant:        %u %s", value,
                      (value == 2) ? "(TMC9660 ✓)" : "(Unexpected!)");
  } else {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Chip Variant:        FAILED TO READ");
  }

  // 5. System Frequency
  if (getChipFrequency(&value)) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "System Frequency:    %u MHz", value);
  } else {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "System Frequency:    FAILED TO READ");
  }

  // 6. Git Info
  GitInfo gitInfo;
  if (getGitInfo(&gitInfo)) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "Git Commit:          %07X%s",
                      gitInfo.commit_hash, gitInfo.dirty ? " (dirty)" : "");
  } else {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Git Commit:          FAILED TO READ");
  }

  // 7. Features
  BootloaderFeatures features;
  if (getFeatures(&features)) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "Features:");
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "  - SRAM Support:    %s",
                      features.sram_support ? "YES" : "NO");
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "  - ROM:             %s",
                      features.rom ? "YES" : "NO");
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "  - OTP:             %s",
                      features.otp ? "YES" : "NO");
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "  - SPI Flash:       %s",
                      features.spi_flash ? "YES" : "NO");
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "  - I2C EEPROM:      %s",
                      features.i2c_eeprom ? "YES" : "NO");
  } else {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660Bootloader", "Features:            FAILED TO READ");
  }

  // 8. CONFIG Memory Info
  uint32_t config_start = 0, config_size = 0;
  if (getConfigMemStart(&config_start) && getConfigMemSize(&config_size)) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                      "CONFIG Memory:       Start=0x%08X, Size=%u bytes", config_start, config_size);
  } else {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "CONFIG Memory:       Info not available");
  }

  // 9. OTP Memory Size
  if (getOtpMemSize(&value)) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "OTP Page Size:       %u bytes", value);
  } else {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "OTP Page Size:       Not available");
  }

  // 10. External Memory Partition Version
  PartitionVersion partVer;
  if (getPartitionVersion(&partVer)) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "Partition Version:   v%u.%u", partVer.major,
                      partVer.minor);
  } else {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "Partition Version:   Not available");
  }

  // 11. SPI Flash Info (may fail if no flash connected)
  uint32_t spi_size = 0, spi_partitions = 0;
  bool has_spi_flash = getSpiMemSize(&spi_size);
  if (has_spi_flash) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "SPI Flash Size:      %u bytes (%.2f KB)",
                      spi_size, spi_size / 1024.0f);

    // Try to get partition count (requires SPI bank to be selected)
    uint8_t current_bank = 0;
    getBank(&current_bank);
    setBank(MemoryBank::SPI_FLASH);

    if (getSpiMemPartitions(&spi_partitions)) {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "SPI Partitions:      %u", spi_partitions);
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "SPI Partitions:      Not available (may need partitioning)");
    }

    // Restore original bank
    setBank(current_bank);
  } else {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                      "SPI Flash:           Not connected or not configured");
  }

  // 12. I2C EEPROM Info (may fail if no EEPROM connected)
  uint32_t i2c_size = 0, i2c_partitions = 0;
  bool has_i2c_eeprom = getI2cMemSize(&i2c_size);
  if (has_i2c_eeprom) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "I2C EEPROM Size:     %u bytes (%.2f KB)",
                      i2c_size, i2c_size / 1024.0f);

    // Try to get partition count (requires I2C bank to be selected)
    uint8_t current_bank = 0;
    getBank(&current_bank);
    setBank(MemoryBank::I2C_EEPROM);

    if (getI2cMemPartitions(&i2c_partitions)) {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "I2C Partitions:      %u", i2c_partitions);
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                        "I2C Partitions:      Not available (may need partitioning)");
    }

    // Restore original bank
    setBank(current_bank);
  } else {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                      "I2C EEPROM:          Not connected or not configured");
  }

  TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader",
                    "╚══════════════════════════════════════════════════════════════╝");
  TMC9660_LOG_DEBUG(comm_, 1, "TMC9660Bootloader", "");

  return success;
}

// tmc9660_bootloader.tpp - Template implementation ends here

#endif // TMC9660_BOOTLOADER_IMPL
