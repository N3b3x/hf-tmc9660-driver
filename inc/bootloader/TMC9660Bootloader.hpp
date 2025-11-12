/**
 * @file TMC9660Bootloader.hpp
 * @brief Main TMC9660 bootloader interface class
 * @ingroup TMC9660_Bootloader
 *
 * @copyright Copyright (c) 2024
 * @license MIT License
 */

#pragma once

#include "bootloader_config.hpp"
#include "bootloader_protocol.hpp"
#include <cstdint>

// Forward declaration for communication interface
namespace tmc9660 {
class TMC9660CommInterface;
}

namespace tmc9660 {

/**
 * @brief Main TMC9660 bootloader interface class.
 * @ingroup TMC9660_Bootloader
 *
 * This class provides comprehensive bootloader functionality for the TMC9660
 * including memory operations, configuration management, OTP programming,
 * and system initialization. The bootloader is essential for configuring
 * the TMC9660 before transitioning to parameter mode operation.
 *
 * ## Key Features
 *
 * - **Memory Operations**: Read/write to RAM, OTP, SPI Flash, I2C EEPROM
 * - **Configuration Management**: Complete bootloader configuration application
 * - **OTP Programming**: One-time programmable memory burning with workarounds
 * - **System Information**: Retrieve bootloader version, features, and capabilities
 * - **Protocol Support**: Both SPI and UART communication interfaces
 * - **Error Handling**: Comprehensive error reporting and verification
 *
 * ## Usage Pattern
 *
 * @code{.cpp}
 * // Create bootloader instance
 * TMC9660Bootloader bootloader(commInterface);
 *
 * // Configure the system
 * BootloaderConfig cfg{};
 * cfg.ldo.vext1_voltage = bootcfg::LDOVoltage::V3_3;
 * cfg.boot.boot_mode = bootcfg::BootMode::Parameter;
 * cfg.boot.start_motor_control = true;
 *
 * // Apply configuration and start motor control
 * bootloader.applyConfiguration(cfg);
 * @endcode
 */
class TMC9660Bootloader {
public:
  explicit TMC9660Bootloader(TMC9660CommInterface& comm) noexcept;

  //==================================================
  // BASIC MEMORY OPERATIONS
  //==================================================

  /**
   * @brief Select the target memory bank for subsequent operations.
   *
   * Sets which memory bank (RAM, OTP, SPI Flash, I2C EEPROM, or CONFIG)
   * will be accessed by subsequent read/write operations. The bank must be
   * selected before performing any memory access operations.
   *
   * @param bank Memory bank number (0-5, see ::MemoryBank enum)
   * @return true if command successful
   * @note Common banks: 0=RAM, 1=OTP, 5=CONFIG
   */
  bool setBank(uint8_t bank) noexcept;

  /**
   * @brief Select the target register bank using enum (overloaded).
   *
   * Convenience overload that accepts a ::MemoryBank enum instead of raw
   * bank number. Automatically converts to the numeric value.
   *
   * @param bank Memory bank enum value
   * @return true if command successful
   */
  bool setBank(MemoryBank bank) noexcept {
    return setBank(static_cast<uint8_t>(bank));
  }

  /**
   * @brief Set the address within the current memory bank.
   *
   * Sets the memory address for subsequent read/write operations within the
   * currently selected bank. Addresses are bank-relative (offset within bank).
   *
   * @param addr Memory address within current bank
   * @return true if command successful
   */
  bool setAddress(uint32_t addr) noexcept;

  /**
   * @brief Write a single byte to the previously selected address.
   *
   * Writes an 8-bit value to the current bank and address. Does not increment
   * the address pointer.
   *
   * @param v 8-bit value to write
   * @return true if write successful
   */
  bool write8(uint8_t v) noexcept;

  /**
   * @brief Write a 16-bit word to the previously selected address.
   *
   * Writes a 16-bit value to the current bank and address. Little-endian byte
   * order. Does not increment the address pointer.
   *
   * @param v 16-bit value to write
   * @return true if write successful
   */
  bool write16(uint16_t v) noexcept;

  /**
   * @brief Write a 32-bit word to the previously selected address.
   *
   * Writes a 32-bit value to the current bank and address. Little-endian byte
   * order. Does not increment the address pointer.
   *
   * @param v 32-bit value to write
   * @return true if write successful
   */
  bool write32(uint32_t v) noexcept;

  /**
   * @brief Write a single byte and increment address by 1.
   *
   * Writes an 8-bit value and automatically increments the address pointer.
   * Useful for writing sequential data arrays.
   *
   * @param v 8-bit value to write
   * @return true if write successful
   */
  bool write8Inc(uint8_t v) noexcept;

  /**
   * @brief Write a 16-bit word and increment address by 2.
   *
   * Writes a 16-bit value and automatically increments the address pointer by 2.
   * Useful for writing sequential data arrays.
   *
   * @param v 16-bit value to write
   * @return true if write successful
   */
  bool write16Inc(uint16_t v) noexcept;

  /**
   * @brief Write a 32-bit word and increment address by 4.
   *
   * Writes a 32-bit value and automatically increments the address pointer by 4.
   * Commonly used for writing configuration registers and data arrays.
   *
   * @param v 32-bit value to write
   * @return true if write successful
   */
  bool write32Inc(uint32_t v) noexcept;

  /**
   * @brief Write multiple 32-bit words starting at the current address.
   *
   * Writes multiple consecutive 32-bit words using WRITE_32_INC commands.
   * The address pointer is incremented automatically for each word.
   *
   * @param values Pointer to array of 32-bit values to write
   * @param count Number of 32-bit words to write
   * @return true if all writes successful
   */
  bool write32IncMultiple(const uint32_t* values, size_t count) noexcept;

  //==================================================
  // READ OPERATIONS
  //==================================================

  /**
   * @brief Read a single byte from the previously selected address.
   *
   * Reads an 8-bit value from the current bank and address without
   * modifying the address pointer.
   *
   * @param value Pointer to receive the 8-bit value read
   * @return true if read successful
   */
  bool read8(uint8_t* value) noexcept;

  /**
   * @brief Read a 16-bit word from the previously selected address.
   *
   * Reads a 16-bit value from the current bank and address without
   * modifying the address pointer. Little-endian byte order.
   *
   * @param value Pointer to receive the 16-bit value read
   * @return true if read successful
   */
  bool read16(uint16_t* value) noexcept;

  /**
   * @brief Read a 32-bit word from the previously selected address.
   *
   * Reads a 32-bit value from the current bank and address without
   * modifying the address pointer. Little-endian byte order.
   *
   * @param value Pointer to receive the 32-bit value read
   * @return true if read successful
   */
  bool read32(uint32_t* value) noexcept;

  /**
   * @brief Read a single byte and increment address by 1.
   *
   * Reads an 8-bit value and automatically increments the address pointer.
   * Useful for reading sequential data arrays.
   *
   * @param value Pointer to receive the 8-bit value read
   * @return true if read successful
   */
  bool read8Inc(uint8_t* value) noexcept;

  /**
   * @brief Read a 16-bit word and increment address by 2.
   *
   * Reads a 16-bit value and automatically increments the address pointer by 2.
   * Useful for reading sequential data arrays.
   *
   * @param value Pointer to receive the 16-bit value read
   * @return true if read successful
   */
  bool read16Inc(uint16_t* value) noexcept;

  /**
   * @brief Read a 32-bit word and increment address by 4.
   *
   * Reads a 32-bit value and automatically increments the address pointer by 4.
   * Commonly used for reading configuration registers and data arrays.
   *
   * @param value Pointer to receive the 32-bit value read
   * @return true if read successful
   */
  bool read32Inc(uint32_t* value) noexcept;

  /**
   * @brief Get the currently selected memory bank.
   *
   * Retrieves the memory bank number that was last selected via ::setBank().
   * Useful for verifying the current bank configuration.
   *
   * @param bank Pointer to receive the current bank number (0-5)
   * @return true if query successful
   */
  bool getBank(uint8_t* bank) noexcept;

  /**
   * @brief Get the currently selected memory address.
   *
   * Retrieves the memory address that was last set via ::setAddress() within
   * the currently selected bank. Useful for tracking address pointer state.
   *
   * @param address Pointer to receive the current address
   * @return true if query successful
   */
  bool getAddress(uint32_t* address) noexcept;

  /**
   * @brief No operation - retrieve reply from previous command (SPI only).
   *
   * Due to SPI protocol, replies are delayed by one transaction. This command
   * sends a NO_OP to retrieve the reply from the previous command. Primarily
   * used internally by the bootloader implementation.
   *
   * @param reply Optional pointer to store the 32-bit reply value
   * @return true if command successful
   * @note Not needed when using UART protocol
   */
  bool noOp(uint32_t* reply = nullptr) noexcept;

  //==================================================
  // VERIFICATION OPERATIONS
  //==================================================

  /**
   * @brief Read back and verify an 8-bit value matches the expected value.
   *
   * Performs a read operation and compares the returned value with the expected
   * value. Useful for verification after configuration writes. Logs the result
   * for debugging purposes.
   *
   * @param expected The expected 8-bit value to match
   * @param configName Human-readable configuration name for logging
   * @return true if read successful and value matches
   */
  bool readAndVerify8(uint8_t expected, const char* configName) noexcept;

  /**
   * @brief Read back and verify a 16-bit value matches the expected value.
   *
   * Performs a read operation and compares the returned value with the expected
   * value. Useful for verification after configuration writes. Logs the result
   * for debugging purposes.
   *
   * @param expected The expected 16-bit value to match
   * @param configName Human-readable configuration name for logging
   * @return true if read successful and value matches
   */
  bool readAndVerify16(uint16_t expected, const char* configName) noexcept;

  /**
   * @brief Read back and verify a 32-bit value matches the expected value.
   *
   * Performs a read operation and compares the returned value with the expected
   * value. Useful for verification after configuration writes. Logs the result
   * for debugging purposes.
   *
   * @param expected The expected 32-bit value to match
   * @param configName Human-readable configuration name for logging
   * @return true if read successful and value matches
   */
  bool readAndVerify32(uint32_t expected, const char* configName) noexcept;

  //==================================================
  // OTP OPERATIONS
  //==================================================

  /**
   * @brief Load an OTP (One-Time Programmable) page into the OTP memory bank.
   *
   * Loads a previously programmed OTP page into RAM for reading. Returns information
   * about the loaded page including the page tag and any bit errors detected.
   *
   * @param page OTP page number to load (0-based)
   * @param result Pointer to receive parsed OTP load result (error count + page tag)
   * @return true if command executed successfully (check result.errorCount for bit errors)
   * @note Check result.errorCount for bit errors, result.pageTag for page identification
   */
  bool otpLoad(uint8_t page, OtpLoadResult* result) noexcept;

  /**
   * @brief Load an OTP page (legacy method with separate parameters).
   *
   * Legacy overload that returns error count and page tag as separate parameters
   * instead of a structured result object.
   *
   * @param page OTP page number to load
   * @param errorCount Optional pointer to receive bit error count (bits 15-8)
   * @param pageTag Optional pointer to receive page tag (bits 7-0)
   * @return true if successful
   */
  bool otpLoad(uint8_t page, uint8_t* errorCount = nullptr, uint8_t* pageTag = nullptr) noexcept;

  /**
   * @brief Permanently burn data to OTP (One-Time Programmable) memory.
   *
   * **WARNING:** OTP burn operations are IRREVERSIBLE! Once a page is burned,
   * it cannot be modified. The burn operation requires specific voltage conditions
   * and timing to succeed.
   *
   * @param page OTP page number to burn (bits 7-0)
   * @param pageAddr OTP page address to write (bits 15-8)
   * @param result Pointer to receive detailed burn result with error information
   * @return true if command executed (check result.isSuccess and result.errorCode)
   * @note ⚠️ This operation is PERMANENT and IRREVERSIBLE!
   */
  bool otpBurn(uint8_t page, uint8_t pageAddr, OtpBurnResult* result) noexcept;

  /**
   * @brief Permanently burn OTP page (legacy method without detailed results).
   *
   * Simplified version that only returns boolean success/failure without error details.
   * Use the overload with OtpBurnResult for detailed error information.
   *
   * @param page OTP page number to burn (bits 7-0)
   * @param pageAddr OTP page address to write (bits 15-8)
   * @return true if successful
   * @note ⚠️ This operation is PERMANENT and IRREVERSIBLE!
   */
  bool otpBurn(uint8_t page, uint8_t pageAddr = 0) noexcept;

  /**
   * @brief Burn OTP page with Erratum 1 workaround for reliable operation.
   *
   * Implements a workaround for silicon Erratum 1 that improves reliability of
   * OTP burn operations. The workaround waits for VDRV voltage to stabilize
   * before and after the burn operation.
   *
   * @param page OTP page number to burn (bits 7-0)
   * @param pageAddr OTP page address to write (bits 15-8)
   * @param result Pointer to receive detailed burn result with error information
   * @param vdrvWaitMs Wait time for VDRV voltage to drop (default 1000ms for 10uF capacitor)
   * @return true if command executed (check result.isSuccess and result.errorCode)
   * @note ⚠️ This operation is PERMANENT and IRREVERSIBLE!
   * @note Recommended method for production OTP burning
   */
  bool otpBurnWithWorkaround(uint8_t page, uint8_t pageAddr, OtpBurnResult* result,
                             uint32_t vdrvWaitMs = 1000) noexcept;

  /**
   * @brief Check OTP burn status using Erratum 1 workaround verification.
   *
   * Verifies the result of an OTP burn operation by checking multiple status
   * indicators. Implements Erratum 1 workaround for reliable status verification.
   *
   * @param result Pointer to receive true if burn was successful
   * @return true if status check completed successfully
   * @note This method implements the Erratum 1 status check workaround
   */
  bool checkOtpBurnStatus(bool* result) noexcept;

  //==================================================
  // EXTERNAL MEMORY OPERATIONS
  //==================================================

  /**
   * @brief Check if external memory is configured.
   *
   * Verifies whether the specified external memory bank has been properly
   * configured in the bootloader settings. Configuration is required before
   * the memory can be accessed.
   *
   * @param bank Memory bank to check
   * @param isConfigured Returns true if configured
   * @return true if command successful
   */
  bool memIsConfigured(MemoryBank bank, bool* isConfigured) noexcept;

  /**
   * @brief Check if external memory is connected.
   *
   * Verifies whether the specified external memory device is physically
   * connected and responding to communication attempts.
   *
   * @param bank Memory bank to check
   * @param isConnected Returns true if connected
   * @return true if command successful
   */
  bool memIsConnected(MemoryBank bank, bool* isConnected) noexcept;

  /**
   * @brief Check if external memory is busy.
   *
   * Verifies whether the specified external memory device is currently
   * busy with an operation (e.g., write cycle, erase operation).
   *
   * @param bank Memory bank to check
   * @param isBusy Returns true if busy
   * @return true if command successful
   */
  bool memIsBusy(MemoryBank bank, bool* isBusy) noexcept;

  //==================================================
  // SPI FLASH OPERATIONS
  //==================================================

  /**
   * @brief Load data into internal 6-byte flash command buffer.
   *
   * Loads data into the TMC9660's internal flash command buffer, which is used
   * to prepare SPI flash commands before transmission. The buffer can hold up
   * to 6 bytes of command and address data.
   *
   * @param offset Byte offset in buffer (0-5)
   * @param data 24-bit data to load (bits 23-0)
   * @return true if successful
   */
  bool flashLoadBuffer(uint8_t offset, uint32_t data) noexcept;

  /**
   * @brief Read data from internal 6-byte flash command buffer.
   *
   * Reads data from the TMC9660's internal flash command buffer. This can be
   * used to verify buffer contents or read response data from flash operations.
   *
   * @param offset Byte offset in buffer (0-5)
   * @param data Returns 24-bit data read (bits 23-0)
   * @return true if successful
   */
  bool flashReadBuffer(uint8_t offset, uint32_t* data) noexcept;

  /**
   * @brief Send datagram to SPI flash and receive reply.
   *
   * Transmits the specified number of bytes from the internal flash command
   * buffer to the external SPI flash device and receives the response.
   *
   * @param numBytes Number of bytes to transmit (1-6)
   * @return true if successful
   */
  bool flashSendDatagram(uint8_t numBytes) noexcept;

  /**
   * @brief Erase a sector on external SPI flash.
   *
   * Performs a sector erase operation on the external SPI flash memory at
   * the specified address. This prepares the sector for new data programming.
   *
   * @param address 24-bit sector address
   * @return true if successful
   */
  bool flashEraseSector(uint32_t address) noexcept;

  /**
   * @brief Read JEDEC manufacturer ID from SPI flash.
   *
   * Reads the JEDEC manufacturer ID from the external SPI flash device.
   * This can be used to identify the flash chip type and verify connectivity.
   *
   * @param manufacturerId Returns manufacturer ID
   * @return true if successful
   */
  bool flashReadJedecId(uint8_t* manufacturerId) noexcept;

  //==================================================
  // RS485 CONFIGURATION
  //==================================================

  /**
   * @brief Configure RS485 communication (must be first command for RS485).
   *
   * Initializes RS485 half-duplex communication parameters. This command must
   * be sent as the first command when using RS485 communication interface.
   * It configures the transmit enable pin and timing parameters.
   *
   * @param txEnPin GPIO pin for TX_EN (1=GPIO8, 2=GPIO2)
   * @param preDelay Delay between TX_EN assertion and TX start
   * @param hostAddr Host address
   * @param deviceAddr Device address
   * @return true if successful
   */
  bool bootstrapRS485(uint8_t txEnPin, uint8_t preDelay, uint8_t hostAddr,
                      uint8_t deviceAddr) noexcept;

  //==================================================
  // INFORMATION QUERIES
  //==================================================

  /**
   * @brief Get bootloader information.
   *
   * Generic function to query various types of system information from the
   * bootloader. Use the specific convenience methods for typed access to
   * parsed information structures.
   *
   * @param query Information type to query
   * @param value Returns queried value
   * @return true if successful
   */
  bool getInfo(InfoQuery query, uint32_t* value) noexcept;

  //==================================================
  // CONVENIENCE INFO METHODS
  //==================================================

  /**
   * @brief Get chip type (should return 0x544D0001 for TMC9660).
   *
   * Retrieves the chip type identifier. For TMC9660, this should return
   * 0x544D0001, which can be used to verify correct chip identification.
   *
   * @param chipType Returns chip type
   * @return true if successful
   */
  bool getChipType(uint32_t* chipType) noexcept {
    return getInfo(InfoQuery::CHIP_TYPE, chipType);
  }

  /**
   * @brief Get bootloader version information.
   *
   * Retrieves and parses the bootloader version information from the TMC9660.
   * The version is returned in major.minor format for easy interpretation.
   *
   * @param version Returns parsed version information
   * @return true if successful
   */
  bool getBootloaderVersion(BootloaderVersion* version) noexcept;

  /**
   * @brief Get available feature flags.
   *
   * Retrieves feature flags indicating which capabilities are supported by
   * the bootloader hardware, including available memory types and interfaces.
   *
   * @param features Returns parsed feature flags
   * @return true if successful
   */
  bool getFeatures(BootloaderFeatures* features) noexcept;

  /**
   * @brief Get Git version control information.
   *
   * Retrieves Git commit hash and dirty bit from the bootloader firmware,
   * indicating the exact firmware build version and development state.
   *
   * @param gitInfo Returns parsed Git information
   * @return true if successful
   */
  bool getGitInfo(GitInfo* gitInfo) noexcept;

  /**
   * @brief Get chip version (silicon revision).
   *
   * Retrieves the silicon revision number of the TMC9660 chip. This information
   * is useful for identifying chip variants and applying silicon-specific workarounds.
   *
   * @param version Returns chip version
   * @return true if successful
   */
  bool getChipVersion(uint32_t* version) noexcept {
    return getInfo(InfoQuery::CHIP_VERSION, version);
  }

  /**
   * @brief Get system frequency in MHz.
   *
   * Retrieves the current system clock frequency in MHz. This reflects the
   * actual operating frequency after PLL and divider configurations.
   *
   * @param frequency Returns frequency in MHz
   * @return true if successful
   */
  bool getChipFrequency(uint32_t* frequency) noexcept {
    return getInfo(InfoQuery::CHIP_FREQUENCY, frequency);
  }

  /**
   * @brief Get CONFIG memory bank start address.
   *
   * Retrieves the starting address of the CONFIG memory bank (bank 5).
   * This is typically 0x00020000 for TMC9660.
   *
   * @param address Returns start address
   * @return true if successful
   */
  bool getConfigMemStart(uint32_t* address) noexcept {
    return getInfo(InfoQuery::CONFIG_MEM_START, address);
  }

  /**
   * @brief Get CONFIG memory bank size.
   *
   * Retrieves the size of the CONFIG memory bank in bytes. This indicates
   * how much configuration space is available for bootloader settings.
   *
   * @param size Returns memory size
   * @return true if successful
   */
  bool getConfigMemSize(uint32_t* size) noexcept {
    return getInfo(InfoQuery::CONFIG_MEM_SIZE, size);
  }

  /**
   * @brief Get OTP memory page size.
   *
   * Retrieves the size of a single OTP (One-Time Programmable) memory page
   * in bytes. This is the unit size for OTP burn operations.
   *
   * @param size Returns OTP page size
   * @return true if successful
   */
  bool getOtpMemSize(uint32_t* size) noexcept {
    return getInfo(InfoQuery::OTP_MEM_SIZE, size);
  }

  /**
   * @brief Get I2C memory size.
   *
   * Retrieves the total size of connected I2C EEPROM memory in bytes.
   * Returns 0 if no I2C EEPROM is configured or connected.
   *
   * @param size Returns I2C memory size
   * @return true if successful
   */
  bool getI2cMemSize(uint32_t* size) noexcept {
    return getInfo(InfoQuery::I2C_MEM_SIZE, size);
  }

  /**
   * @brief Get SPI memory size.
   *
   * Retrieves the total size of connected SPI flash memory in bytes.
   * Returns 0 if no SPI flash is configured or connected.
   *
   * @param size Returns SPI memory size
   * @return true if successful
   */
  bool getSpiMemSize(uint32_t* size) noexcept {
    return getInfo(InfoQuery::SPI_MEM_SIZE, size);
  }

  /**
   * @brief Get partition version information.
   *
   * Retrieves the version of the external memory partition format used
   * for storing TMCL scripts and parameters in external memory.
   *
   * @param version Returns parsed partition version
   * @return true if successful
   */
  bool getPartitionVersion(PartitionVersion* version) noexcept;

  /**
   * @brief Get number of SPI memory partitions.
   *
   * Retrieves the number of partitions available in the connected SPI flash
   * memory for storing TMCL scripts and configuration data.
   *
   * @param count Returns number of partitions
   * @return true if successful
   */
  bool getSpiMemPartitions(uint32_t* count) noexcept {
    return getInfo(InfoQuery::SPI_MEM_PARTITIONS, count);
  }

  /**
   * @brief Get number of I2C memory partitions.
   *
   * Retrieves the number of partitions available in the connected I2C EEPROM
   * memory for storing TMCL scripts and configuration data.
   *
   * @param count Returns number of partitions
   * @return true if successful
   */
  bool getI2cMemPartitions(uint32_t* count) noexcept {
    return getInfo(InfoQuery::I2C_MEM_PARTITIONS, count);
  }

  /**
   * @brief Get chip variant (TMC9660 reports value 2).
   *
   * Retrieves the chip variant identifier. For TMC9660, this should return
   * value 2, which can be used to verify correct chip identification.
   *
   * @param variant Returns chip variant
   * @return true if successful
   */
  bool getChipVariant(uint32_t* variant) noexcept {
    return getInfo(InfoQuery::CHIP_VARIANT, variant);
  }

  /**
   * @brief Retrieve and log all available bootloader information.
   *
   * This function queries all GET_INFO commands and logs the results.
   * Useful for debugging and verifying chip configuration.
   *
   * Information retrieved:
   * - Chip type, version, variant
   * - Bootloader version and Git info
   * - System frequency
   * - Available features (SRAM, ROM, OTP, SPI flash, I2C EEPROM)
   * - Memory sizes and partition info
   *
   * @return true if at least basic info was retrieved successfully
   * @note Some queries may fail if features are not available (e.g., no SPI flash)
   */
  bool getAllBootloaderInfo() noexcept;

  //==================================================
  // HIGH-LEVEL CONFIGURATION
  //==================================================

  /**
   * @brief Apply all fields of a ::BootloaderConfig.
   *
   * This function applies a complete bootloader configuration by writing all
   * configuration structures to their respective registers in the CONFIG memory
   * bank. It provides a convenient single-call interface for comprehensive
   * system configuration.
   *
   * @param cfg Configuration to apply
   * @param failOnVerifyError If true, return false on read-back verification failure;
   *                          if false, log warning but continue
   * @return true if successful
   * @note If cfg.boot.start_motor_control is true, bootloader will exit after this call
   */
  bool applyConfiguration(const BootloaderConfig& cfg, bool failOnVerifyError = true) noexcept;

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
   * ⚠️ CRITICAL: After calling this function, the bootloader will no longer
   * respond to commands. All bootloader configuration must be completed BEFORE
   * calling this function.
   *
   * @param bootMode Motor control mode to start (Register or Parameter mode)
   * @return true if command sent successfully
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
  bool startMotorControl(bootcfg::BootMode bootMode = bootcfg::BootMode::Parameter) noexcept;

private:
  bool sendCommand(uint8_t cmd, uint32_t value, uint32_t* reply = nullptr) noexcept;
  bool sendCommandSPI(uint8_t cmd, uint32_t value, uint32_t* reply) noexcept;
  bool sendCommandUART(uint8_t cmd, uint32_t value, uint32_t* reply) noexcept;

  TMC9660CommInterface& comm_;
  uint8_t deviceAddr_; ///< Device address for UART protocol
  uint8_t hostAddr_;   ///< Host address for UART protocol
};

} // namespace tmc9660