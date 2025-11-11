/**
 * @file bootloader_protocol.hpp
 * @brief TMC9660 bootloader communication protocol structures and enumerations.
 * 
 * This file contains all protocol-related structures, enumerations, and
 * definitions for communicating with the TMC9660 bootloader. These define
 * command codes, status codes, memory banks, information queries, and
 * communication packet structures for both SPI and UART protocols.
 * 
 * @defgroup TMC9660_BootloaderProtocol Communication Protocol
 * @brief SPI/UART protocol structures and command definitions
 */

#pragma once

#include "bootloader_utils.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace tmc9660 {

/**
 * @brief Bootloader status codes returned by command replies.
 * @ingroup TMC9660_BootloaderProtocol
 * 
 * These status codes are returned by the bootloader to indicate the result
 * of each command. Status OK (0) indicates successful command execution.
 * Other status codes indicate various error conditions or special states.
 * 
 * @note Per TMC9660 datasheet Table 23
 */
enum class BootloaderStatus : uint8_t {
  OK = 0,                    ///< Command executed successfully
  CMD_NOT_FOUND = 1,         ///< The request has an invalid command number
  INVALID_ADDR = 3,          ///< The memory address is not valid for the requested command
  INVALID_VALUE = 4,         ///< The request has an invalid value
  INVALID_BANK = 14,         ///< The memory bank is not valid for the requested command
  BUSY = 15,                 ///< Bootloader has not yet finished processing the last command (SPI only)
  MEM_UNCONFIGURED = 17,     ///< The external memory is not configured
  OTP_ERROR = 18,            ///< The OTP command has failed
  SESSION_START = 19,        ///< First SPI datagram after power-on (SPI only)
  CMD_NOT_AVAILABLE = 20,    ///< The command is currently not available
  BOOTLOADER_RESUMED = 21    ///< First SPI datagram after returning to bootloader from motor control (SPI only)
};

/**
 * @brief Bootloader command codes for memory and configuration operations.
 * @ingroup TMC9660_BootloaderProtocol
 * 
 * These commands allow reading and writing to memory banks, configuring
 * external devices, and querying system information. Each command may be
 * followed by command-specific parameters in the 32-bit value field.
 * 
 * @note Per TMC9660 datasheet Table 23
 */
enum class BootloaderCommand : uint8_t {
  GET_INFO = 0,               ///< Get various basic information about the connected TMC9660
  GET_BANK = 8,               ///< Get the currently selected memory bank
  SET_BANK = 9,               ///< Set the memory bank
  GET_ADDRESS = 10,           ///< Get the current memory address
  SET_ADDRESS = 11,           ///< Set the memory address
  READ_32 = 12,               ///< Read 32-bit data from selected memory bank/address
  READ_32_INC = 13,           ///< Read 32-bit data and increment address
  READ_16 = 14,               ///< Read 16-bit data from selected memory bank/address
  READ_16_INC = 15,           ///< Read 16-bit data and increment address
  READ_8 = 16,                ///< Read 8-bit data from selected memory bank/address
  READ_8_INC = 17,            ///< Read 8-bit data and increment address
  WRITE_32 = 18,              ///< Write 32-bit data to selected memory bank/address
  WRITE_32_INC = 19,          ///< Write 32-bit data and increment address
  WRITE_16 = 20,              ///< Write 16-bit data to selected memory bank/address
  WRITE_16_INC = 21,          ///< Write 16-bit data and increment address
  WRITE_8 = 22,               ///< Write 8-bit data to selected memory bank/address
  WRITE_8_INC = 23,           ///< Write 8-bit data and increment address
  NO_OP = 29,                 ///< No operation (for retrieving previous reply in SPI)
  OTP_LOAD = 30,              ///< Read a programmed OTP page
  OTP_BURN = 31,              ///< Burn an OTP page
  MEM_IS_CONFIGURED = 32,     ///< Check whether an external memory bank is configured
  MEM_IS_CONNECTED = 33,      ///< Check whether an external memory is connected
  FLASH_SEND_CMD = 36,        ///< Send arbitrary commands to an external flash
  FLASH_ERASE_SECTOR = 37,    ///< Send a sector erase command to an external flash
  MEM_IS_BUSY = 40,           ///< Check whether an external memory is busy
  BOOTSTRAP_RS485 = 255       ///< Set up RS485 settings
};

/**
 * @brief Memory bank identifiers for bootloader operations.
 * @ingroup TMC9660_BootloaderProtocol
 * 
 * The TMC9660 organizes memory into several banks for different purposes.
 * The bootloader allows reading from and writing to these memory banks
 * for configuration and program storage.
 */
enum class MemoryBank : uint8_t {
  RAM = 0,           ///< Internal RAM
  OTP = 1,           ///< OTP memory
  SPI_FLASH = 2,     ///< External SPI Flash
  I2C_EEPROM = 3,    ///< External I2C EEPROM
  RESERVED = 4,      ///< Reserved
  CONFIG = 5         ///< Configuration memory (runtime reconfiguration)
};

/**
 * @brief GET_INFO query types for retrieving system information.
 * @ingroup TMC9660_BootloaderProtocol
 * 
 * These query types retrieve various system information including chip type,
 * bootloader version, available features, memory sizes, and hardware capabilities.
 * Use ::TMC9660Bootloader::getInfo() to query these values.
 * 
 * @note Per TMC9660 datasheet Table 24
 */
enum class InfoQuery : uint32_t {
  CHIP_TYPE = 0,              ///< Get the Chip type (returns 0x544D0001)
  BL_VERSION = 1,             ///< Get bootloader version (major in upper 16 bits, minor in lower 16 bits)
  FEATURES = 2,               ///< Get available feature groups (bit flags)
  GIT_INFO = 12,              ///< Get Git version control information
  CHIP_VERSION = 13,          ///< Get silicon revision (TMC9660 reports revision 1)
  CHIP_FREQUENCY = 14,        ///< Get system frequency in MHz
  CONFIG_MEM_START = 17,      ///< Get starting address of CONFIG memory
  CONFIG_MEM_SIZE = 18,       ///< Get size of CONFIG memory
  OTP_MEM_SIZE = 19,          ///< Get size of one OTP memory page
  I2C_MEM_SIZE = 20,          ///< Get memory size of connected I2C memory
  SPI_MEM_SIZE = 21,          ///< Get memory size of connected SPI memory
  PARTITION_VERSION = 22,     ///< Get version of external memory partition format
  SPI_MEM_PARTITIONS = 25,    ///< Get number of SPI memory partitions
  I2C_MEM_PARTITIONS = 26,    ///< Get number of I2C memory partitions
  CHIP_VARIANT = 28           ///< Get chip variant (TMC9660 reports value 2)
};

/**
 * @brief Bootloader version information structure.
 * 
 * Contains the bootloader version in major.minor format. The version is retrieved
 * from the bootloader using GET_INFO query type ::InfoQuery::BL_VERSION.
 * 
 * Parses the 32-bit version value where the upper 16 bits contain the major version
 * and the lower 16 bits contain the minor version.
 */
struct BootloaderVersion {
  uint16_t major;  ///< Major version number
  uint16_t minor;  ///< Minor version number
  
  /// Parse from 32-bit value (major in upper 16 bits, minor in lower 16 bits)
  static BootloaderVersion fromValue(uint32_t value) noexcept {
    BootloaderVersion v;
    v.major = static_cast<uint16_t>(value >> 16);
    v.minor = static_cast<uint16_t>(value & 0xFFFF);
    return v;
  }
};

/**
 * @brief Feature flags indicating available bootloader capabilities.
 * 
 * These flags indicate which memory types and features are supported by the
 * bootloader hardware. Retrieved via GET_INFO query type ::InfoQuery::FEATURES.
 */
struct BootloaderFeatures {
  bool sram_support;      ///< Bit 0: SRAM support
  bool rom;               ///< Bit 1: ROM
  bool otp;               ///< Bit 2: OTP
  bool spi_flash;         ///< Bit 3: SPI flash external memory
  bool i2c_eeprom;        ///< Bit 4: I2C EEPROM external memory
  
  /// Parse from 32-bit value
  static BootloaderFeatures fromValue(uint32_t value) noexcept {
    BootloaderFeatures f;
    f.sram_support = (value & (1u << 0)) != 0;
    f.rom = (value & (1u << 1)) != 0;
    f.otp = (value & (1u << 2)) != 0;
    f.spi_flash = (value & (1u << 3)) != 0;
    f.i2c_eeprom = (value & (1u << 4)) != 0;
    return f;
  }
};

/**
 * @brief Git version control information from bootloader firmware.
 * 
 * Contains Git commit hash and dirty bit indicating the firmware build state.
 * The dirty bit indicates if there were uncommitted changes when the firmware
 * was built. Retrieved via GET_INFO query type ::InfoQuery::GIT_INFO.
 */
struct GitInfo {
  bool dirty;              ///< Bit 28: Dirty bit - uncommitted changes
  uint32_t commit_hash;    ///< Bits 27-0: 7-digit hex commit hash
  
  /// Parse from 32-bit value
  static GitInfo fromValue(uint32_t value) noexcept {
    GitInfo g;
    g.dirty = (value & (1u << 28)) != 0;
    g.commit_hash = value & 0x0FFFFFFF;
    return g;
  }
};

/**
 * @brief External memory partition version information.
 * 
 * Indicates the version of the external memory partition format used by the
 * TMC9660. This version affects how TMCL scripts and parameters are stored
 * and retrieved from external memory (SPI flash or I2C EEPROM).
 */
struct PartitionVersion {
  uint8_t major;  ///< Bits 15-8: Major version
  uint8_t minor;  ///< Bits 7-0: Minor version
  
  /// Parse from 32-bit value
  static PartitionVersion fromValue(uint32_t value) noexcept {
    PartitionVersion v;
    v.major = static_cast<uint8_t>((value >> 8) & 0xFF);
    v.minor = static_cast<uint8_t>(value & 0xFF);
    return v;
  }
};

/**
 * @brief OTP load operation result information.
 * 
 * Contains the result of loading data from OTP (One-Time Programmable) memory.
 * The error count indicates how many bit errors were detected during load,
 * while the page tag identifies the loaded OTP page.
 */
struct OtpLoadResult {
  uint8_t errorCount;  ///< OTP bit error count (bits 15-8)
  uint8_t pageTag;     ///< OTP page tag (bits 7-0)
  
  /// Parse from 32-bit value
  static OtpLoadResult fromValue(uint32_t value) noexcept {
    OtpLoadResult result;
    result.errorCount = static_cast<uint8_t>((value >> 8) & 0xFF);
    result.pageTag = static_cast<uint8_t>(value & 0xFF);
    return result;
  }
};

/**
 * @brief OTP burn operation error codes.
 * 
 * These error codes are returned when an OTP burn operation fails. Negative
 * values indicate specific failure modes during the programming process.
 * Positive values (0 or greater) indicate successful operations.
 * 
 * @note Per TMC9660 datasheet
 */
enum class OtpBurnError : int8_t {
  INVALID_PAGE = -1,           ///< The OTP page number is invalid
  NO_MORE_BURNS = -2,          ///< Last OTP page burnt, no more burns possible
  CHARGE_PUMP_FAILED = -3,     ///< Setting up internal OTP charge pump failed
  BURN_PROCEDURE_FAILED = -4,  ///< The burn procedure failed
  CLOCK_SETUP_FAILED = -5,     ///< Internal clock setup for OTP operation failed
  CLOCK_RESTORE_FAILED = -6    ///< Restoring original clock setup after OTP operation failed
};

/**
 * @brief OTP burn operation result information.
 * 
 * Contains the result of burning data to OTP (One-Time Programmable) memory.
 * The structure includes success status, error code, and human-readable
 * error description for debugging failed burn operations.
 */
struct OtpBurnResult {
  bool isSuccess;                  ///< Whether the burn operation succeeded
  OtpBurnError errorCode;          ///< Error code if operation failed
  const char* errorDescription;    ///< Human-readable error description
  
  /// Create success result
  static OtpBurnResult createSuccess() noexcept {
    OtpBurnResult result;
    result.isSuccess = true;
    result.errorCode = static_cast<OtpBurnError>(0);
    result.errorDescription = "Success";
    return result;
  }
  
  /// Create error result
  static OtpBurnResult createError(OtpBurnError code) noexcept {
    OtpBurnResult result;
    result.isSuccess = false;
    result.errorCode = code;
    
    switch (code) {
      case OtpBurnError::INVALID_PAGE:
        result.errorDescription = "Invalid OTP page number";
        break;
      case OtpBurnError::NO_MORE_BURNS:
        result.errorDescription = "Last OTP page burnt, no more burns possible";
        break;
      case OtpBurnError::CHARGE_PUMP_FAILED:
        result.errorDescription = "Setting up internal OTP charge pump failed";
        break;
      case OtpBurnError::BURN_PROCEDURE_FAILED:
        result.errorDescription = "The burn procedure failed";
        break;
      case OtpBurnError::CLOCK_SETUP_FAILED:
        result.errorDescription = "Internal clock setup for OTP operation failed";
        break;
      case OtpBurnError::CLOCK_RESTORE_FAILED:
        result.errorDescription = "Restoring original clock setup after OTP operation failed";
        break;
      default:
        result.errorDescription = "Unknown error";
        break;
    }
    
    return result;
  }
};

/**
 * @brief Bootloader command structure for SPI (40-bit / 5-byte protocol).
 * @ingroup TMC9660_BootloaderProtocol
 * 
 * Format: [COMMAND(8)] [VALUE(32)]
 * 
 * All bytes are bit-flipped during SPI transmission. Serialization is handled
 * automatically by the ::TMC9660Bootloader::sendCommand() methods.
 */
struct BootloaderCommandSPI {
  uint8_t command;  ///< Command byte
  uint32_t value;   ///< 32-bit data value
  
  /// Serialize to 5-byte buffer for SPI transmission
  void toBuffer(std::array<uint8_t, 5> &out) const noexcept {
    out[0] = command;
    out[1] = static_cast<uint8_t>(value >> 24);
    out[2] = static_cast<uint8_t>(value >> 16);
    out[3] = static_cast<uint8_t>(value >> 8);
    out[4] = static_cast<uint8_t>(value);
  }
};

/**
 * @brief Bootloader command structure for UART (64-bit / 8-byte protocol).
 * @ingroup TMC9660_BootloaderProtocol
 * 
 * Format: [SYNC(0x55)] [DEVICE_ADDR] [COMMAND] [VALUE(32 MSB first)] [CRC8]
 * 
 * No bit-flipping (unlike SPI). CRC-8 checksum with polynomial x^8 + x^2 + x^1 + x^0.
 * Serialization is handled automatically by the ::TMC9660Bootloader::sendCommand() methods.
 */
struct BootloaderCommandUART {
  uint8_t deviceAddr;  ///< Device address
  uint8_t command;     ///< Command byte
  uint32_t value;      ///< 32-bit data value

  /// Serialize to 8-byte buffer for UART transmission
  void toBuffer(std::array<uint8_t, 8> &out) const noexcept {
    out[0] = 0x55;  // Sync byte
    out[1] = deviceAddr;
    out[2] = command;
    out[3] = static_cast<uint8_t>(value >> 24);  // MSB first
    out[4] = static_cast<uint8_t>(value >> 16);
    out[5] = static_cast<uint8_t>(value >> 8);
    out[6] = static_cast<uint8_t>(value);
    out[7] = crc8Bootloader(out.data(), 7);  // CRC over first 7 bytes
  }
};

/**
 * @brief Bootloader reply structure for SPI (40-bit / 5-byte protocol).
 * @ingroup TMC9660_BootloaderProtocol
 * 
 * Format: [STATUS(8)] [VALUE(32)]
 * 
 * All bytes are bit-flipped during SPI reception. Replies are delayed by one
 * SPI transaction (standard SPI behavior), requiring two transactions per command.
 */
struct BootloaderReplySPI {
  uint8_t status;   ///< Status byte
  uint32_t value;   ///< 32-bit data value

  /// Deserialize from 5-byte SPI buffer
  static BootloaderReplySPI fromBuffer(const std::array<uint8_t, 5> &in) noexcept {
    BootloaderReplySPI r;
    r.status = in[0];
    r.value = (static_cast<uint32_t>(in[1]) << 24) |
              (static_cast<uint32_t>(in[2]) << 16) |
              (static_cast<uint32_t>(in[3]) << 8) |
              static_cast<uint32_t>(in[4]);
    return r;
  }

  /// Check if reply indicates success
  bool isOK() const noexcept {
    return status == static_cast<uint8_t>(BootloaderStatus::OK) ||
           status == static_cast<uint8_t>(BootloaderStatus::SESSION_START) ||
           status == static_cast<uint8_t>(BootloaderStatus::BOOTLOADER_RESUMED);
  }
  
  /// Get status as enum
  BootloaderStatus getStatus() const noexcept {
    return static_cast<BootloaderStatus>(status);
  }
};

/**
 * @brief Bootloader reply structure for UART (64-bit / 8-byte protocol).
 * @ingroup TMC9660_BootloaderProtocol
 * 
 * Format: [HOST_ADDR] [DEVICE_ADDR] [STATUS] [VALUE(32 MSB first)] [CRC8]
 * 
 * Note: No SYNC byte in reply (unlike request which has 0x55 as first byte).
 * CRC8 verification is automatically performed during deserialization.
 */
struct BootloaderReplyUART {
  uint8_t hostAddr;    ///< Host address
  uint8_t deviceAddr;  ///< Device address
  uint8_t status;      ///< Status byte
  uint32_t value;      ///< 32-bit data value

  /// Deserialize from 8-byte UART buffer
  static BootloaderReplyUART fromBuffer(const std::array<uint8_t, 8> &in) noexcept {
    BootloaderReplyUART r;
    // UART protocol: [HOST_ADDR] [DEVICE_ADDR] [STATUS] [VALUE(32)] [CRC8]
    // (No SYNC byte in reply, unlike the request which has 0x55 as first byte)
    r.hostAddr = in[0];
    r.deviceAddr = in[1];
    r.status = in[2];
    r.value = (static_cast<uint32_t>(in[3]) << 24) |  // MSB first
              (static_cast<uint32_t>(in[4]) << 16) |
              (static_cast<uint32_t>(in[5]) << 8) |
              static_cast<uint32_t>(in[6]);
    // Note: in[7] is CRC8, should be verified
    return r;
  }

  /// Verify CRC8 checksum
  bool verifyCRC(const std::array<uint8_t, 8> &in) const noexcept {
    uint8_t calculatedCRC = crc8Bootloader(in.data(), 7);
    return calculatedCRC == in[7];
  }

  /// Check if reply indicates success
  bool isOK() const noexcept {
    return status == static_cast<uint8_t>(BootloaderStatus::OK);
  }
  
  /// Get status as enum
  BootloaderStatus getStatus() const noexcept {
    return static_cast<BootloaderStatus>(status);
  }
};

} // namespace tmc9660