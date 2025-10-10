/**
 * @file TMC9660Bootloader.hpp
 * @brief Helper for configuring the TMC9660 bootloader registers.
 */

#pragma once

#include "TMC9660CommInterface.hpp"
#include <cstddef>
#include <cstdint>

namespace tmc9660 {

namespace bootcfg {

/// Enumerations describing bootloader configuration options.

enum class LDOVoltage : uint8_t {
  Disabled = 0,
  V2_5 = 1,
  V3_3 = 2,
  V5_0 = 3,
};

enum class LDOSlope : uint8_t {
  Slope3ms = 0,
  Slope1_5ms = 1,
  Slope0_75ms = 2,
  Slope0_37ms = 3,
};

enum class BootMode : uint8_t {
  Register = 1,
  Parameter = 2,
};

enum class UartRxPin : uint8_t { GPIO7 = 0, GPIO1 = 1 };
enum class UartTxPin : uint8_t { GPIO6 = 0, GPIO0 = 1 };

enum class BaudRate : uint8_t {
  BR9600 = 0,
  BR19200,
  BR38400,
  BR57600,
  BR115200,
  BR1000000,
  Auto8x,
  Auto16x,
};

enum class RS485TxEnPin : uint8_t { None = 0, GPIO8 = 1, GPIO2 = 2 };

enum class SPIInterface : uint8_t { IFACE0 = 0, IFACE1 = 1 };

enum class SPI0SckPin : uint8_t { GPIO6 = 0, GPIO11 = 1 };

enum class SPIFlashFreq : uint8_t { Div1 = 0, Div2 = 1, Div4 = 3 };

enum class I2CSdaPin : uint8_t { GPIO5 = 0, GPIO11 = 1, GPIO14 = 2 };
enum class I2CSclPin : uint8_t { GPIO4 = 0, GPIO12 = 1, GPIO13 = 2 };

enum class I2CFreq : uint8_t { Freq100k = 0, Freq200k, Freq400k, Freq800k };

enum class ClockSource : uint8_t { Internal = 0, External = 1 };

enum class ExtSourceType : uint8_t { Oscillator = 0, Clock = 1 };

enum class XtalDrive : uint8_t {
  Freq8MHz = 1,
  Freq16MHz = 3,
  Freq24MHz = 5,
  Freq32MHz = 6,
};

enum class SysClkSource : uint8_t { IntOsc = 0, PLL = 1 };

enum class SysClkDiv : uint8_t { Div1 = 0, Div15MHz = 3 };

} // namespace bootcfg

namespace bootaddr {
/// Base offset of the configuration registers inside bank 5.
constexpr uint32_t BASE = 0x00020000;
/// LDO configuration register (VEXT1, VEXT2, slopes, faults).
constexpr uint32_t LDO_CONFIG = BASE + 0x00;
/// UART device/host address register.
constexpr uint32_t UART_ADDR = BASE + 0x02;
/// RS485 TXEN delay configuration.
constexpr uint32_t RS485_DELAY = BASE + 0x04;
/// Communication selection (UART/SPI/RS485).
constexpr uint32_t COMM_CONFIG = BASE + 0x06;
/// Boot configuration register (BOOT_MODE, START_MOTOR_CTRL, BL_READY_FAULT, BL_EXIT_FAULT, etc.).
/// ⚠️ CRITICAL: Write this LAST with START_MOTOR_CTRL=1 to exit bootloader!
constexpr uint32_t BOOT_CONFIG = BASE + 0x08;
/// SPI flash configuration register.
constexpr uint32_t SPI_FLASH = BASE + 0x0A;
/// I2C EEPROM configuration register.
constexpr uint32_t I2C_CONFIG = BASE + 0x0C;
/// GPIO output level register.
constexpr uint32_t GPIO_OUT = BASE + 0x0E;
/// GPIO direction register.
constexpr uint32_t GPIO_DIR = BASE + 0x10;
/// GPIO pull-up register.
constexpr uint32_t GPIO_PU = BASE + 0x12;
/// GPIO pull-down register.
constexpr uint32_t GPIO_PD = BASE + 0x14;
/// GPIO analog enable register.
constexpr uint32_t GPIO_ANALOG = BASE + 0x16;
/// Clock configuration register.
constexpr uint32_t CLOCK_CONFIG = BASE + 0x18;
} // namespace bootaddr

/// Configuration of the on-chip LDO regulators.
struct LDOConfig {
  bootcfg::LDOVoltage vext1{bootcfg::LDOVoltage::Disabled};
  bootcfg::LDOVoltage vext2{bootcfg::LDOVoltage::Disabled};
  bootcfg::LDOSlope slope_vext1{bootcfg::LDOSlope::Slope3ms};
  bootcfg::LDOSlope slope_vext2{bootcfg::LDOSlope::Slope3ms};
  bool ldo_short_fault{false};
};

/// Bootloader behaviour configuration.
struct BootConfig {
  bootcfg::BootMode boot_mode{bootcfg::BootMode::Register};
  bool bl_ready_fault{false};
  bool bl_exit_fault{true};
  bool disable_selftest{false};
  bool bl_config_fault{false};
  bool start_motor_control{false};
};

/// UART communication settings for the bootloader.
struct UARTConfig {
  uint8_t device_address{1};
  uint8_t host_address{255};
  bool disable_uart{false};
  bootcfg::UartRxPin rx_pin{bootcfg::UartRxPin::GPIO7};
  bootcfg::UartTxPin tx_pin{bootcfg::UartTxPin::GPIO6};
  bootcfg::BaudRate baud_rate{bootcfg::BaudRate::BR115200};
};

/// Optional RS485 transceiver control via the UART_TXEN pin.
struct RS485Config {
  bool enable_rs485{false};
  bootcfg::RS485TxEnPin txen_pin{bootcfg::RS485TxEnPin::None};
  uint8_t txen_pre_delay{0};
  uint8_t txen_post_delay{0};
};

/// SPI interface used for bootloader commands.
struct SPIBootConfig {
  bool disable_spi{false};
  bootcfg::SPIInterface boot_spi_iface{bootcfg::SPIInterface::IFACE0};
  bootcfg::SPI0SckPin spi0_sck_pin{bootcfg::SPI0SckPin::GPIO6};
};

/// External SPI flash configuration.
struct SPIFlashConfig {
  bool enable_flash{false};
  bootcfg::SPIInterface flash_spi_iface{bootcfg::SPIInterface::IFACE1};
  bootcfg::SPI0SckPin spi0_sck_pin{bootcfg::SPI0SckPin::GPIO6};
  uint8_t cs_pin{0};
  bootcfg::SPIFlashFreq freq_div{bootcfg::SPIFlashFreq::Div4};
};

/// External I2C EEPROM configuration.
struct I2CConfig {
  bool enable_eeprom{false};
  bootcfg::I2CSdaPin sda_pin{bootcfg::I2CSdaPin::GPIO5};
  bootcfg::I2CSclPin scl_pin{bootcfg::I2CSclPin::GPIO4};
  uint8_t address_bits{0};
  bootcfg::I2CFreq freq_code{bootcfg::I2CFreq::Freq100k};
};

/// System clock selection parameters.
struct ClockConfig {
  bootcfg::ClockSource use_external{bootcfg::ClockSource::Internal};
  bootcfg::ExtSourceType ext_source_type{bootcfg::ExtSourceType::Oscillator};
  bootcfg::XtalDrive xtal_drive{bootcfg::XtalDrive::Freq16MHz};
  bool xtal_boost{false};
  bootcfg::SysClkSource pll_selection{bootcfg::SysClkSource::PLL};
  uint8_t rdiv{14};
  bootcfg::SysClkDiv sysclk_div{bootcfg::SysClkDiv::Div1};
};

/// Initial state of the general purpose pins during boot.
struct GPIOConfig {
  uint32_t outputMask{0};
  uint32_t directionMask{0};
  uint32_t pullUpMask{0};
  uint32_t pullDownMask{0};
  uint32_t analogMask{0};
};

/// Aggregated bootloader configuration written by ::TMC9660Bootloader.
struct BootloaderConfig {
  LDOConfig ldo;
  BootConfig boot;
  UARTConfig uart;
  RS485Config rs485;
  SPIBootConfig spiComm;
  SPIFlashConfig spiFlash;
  I2CConfig i2c;
  ClockConfig clock;
  GPIOConfig gpio;
};

/// Bootloader status codes
enum class BootloaderStatus : uint8_t {
  SESSION_START = 0x01,  ///< First reply after power-up
  OK = 0x64,             ///< Command executed successfully (100)
  INVALID_ADDR = 0x65,   ///< Invalid memory address (101)
  INVALID_VALUE = 0x66,  ///< Invalid value (102)
  OTP_ERROR = 0x67,      ///< OTP operation error (103)
  MEM_UNCONFIGURED = 0x68, ///< External memory not configured (104)
  ERROR = 0xFF           ///< Command execution failed
};

/// Bootloader command codes
enum class BootloaderCommand : uint8_t {
  SET_BANK = 0x01,           ///< Select memory bank
  SET_ADDRESS = 0x02,        ///< Set memory address
  WRITE_8 = 0x03,            ///< Write 8-bit value
  WRITE_16 = 0x04,           ///< Write 16-bit value
  WRITE_32 = 0x05,           ///< Write 32-bit value
  OTP_BURN = 0x06,           ///< Burn OTP page
  WRITE_8_INC = 0x07,        ///< Write 8-bit and increment address
  WRITE_16_INC = 0x08,       ///< Write 16-bit and increment address
  WRITE_32_INC = 0x09,       ///< Write 32-bit and increment address
  NO_OP = 0x0A,              ///< No operation (for retrieving previous reply)
  OTP_LOAD = 0x0B,           ///< Load OTP page
  MEM_IS_CONFIGURED = 0x0C,  ///< Check if external memory is configured
  MEM_IS_CONNECTED = 0x0D,   ///< Check if external memory is connected
  FLASH_SEND_CMD = 0x0E,     ///< Send command to SPI flash
  FLASH_ERASE_SECTOR = 0x0F, ///< Erase SPI flash sector
  MEM_IS_BUSY = 0x10,        ///< Check if external memory is busy
  BOOTSTRAP_RS485 = 0x11,    ///< Configure RS485 communication
  GET_INFO = 0x12            ///< Get bootloader information
};

/// Memory bank identifiers
enum class MemoryBank : uint8_t {
  RAM = 0,           ///< Internal RAM
  OTP = 1,           ///< OTP memory
  SPI_FLASH = 2,     ///< External SPI Flash
  I2C_EEPROM = 3,    ///< External I2C EEPROM
  RESERVED = 4,      ///< Reserved
  CONFIG = 5         ///< Configuration memory (runtime reconfiguration)
};

/// GET_INFO query types
enum class InfoQuery : uint32_t {
  CONFIG_MEM_START = 0,  ///< Get CONFIG memory bank start address
  CONFIG_MEM_SIZE = 1    ///< Get CONFIG memory bank size
};

/// Helper function to bit-flip a byte (reverse all 8 bits) - used for SPI only
static constexpr uint8_t bitFlip(uint8_t byte) noexcept {
  byte = ((byte & 0xF0) >> 4) | ((byte & 0x0F) << 4);
  byte = ((byte & 0xCC) >> 2) | ((byte & 0x33) << 2);
  byte = ((byte & 0xAA) >> 1) | ((byte & 0x55) << 1);
  return byte;
}

/// CRC-8 calculation for UART protocol
/// Polynomial: x^8 + x^2 + x^1 + x^0
/// Input and output bytes are bit-flipped
static constexpr uint8_t crc8Bootloader(const uint8_t* data, size_t len) noexcept {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    uint8_t byte = bitFlip(data[i]);  // Input byte is bit-flipped
    crc ^= byte;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;  // Polynomial: x^8 + x^2 + x^1 + x^0
      } else {
        crc <<= 1;
      }
    }
  }
  return bitFlip(crc);  // Result is bit-flipped
}

/// Bootloader command structure for SPI (40-bit / 5-byte protocol)
/// NOTE: All bytes are bit-flipped in the SPI bootloader protocol
struct BootloaderCommandSPI {
  uint8_t command;  ///< Command byte
  uint32_t value;   ///< 32-bit data value

  /// Serialize to 5-byte buffer for SPI transmission (with bit-flipping)
  void toBuffer(std::array<uint8_t, 5> &out) const noexcept {
    // All bytes are bit-flipped in the bootloader protocol
    out[0] = bitFlip(command);
    out[1] = bitFlip(static_cast<uint8_t>(value >> 24));
    out[2] = bitFlip(static_cast<uint8_t>(value >> 16));
    out[3] = bitFlip(static_cast<uint8_t>(value >> 8));
    out[4] = bitFlip(static_cast<uint8_t>(value));
  }
};

/// Bootloader command structure for UART (64-bit / 8-byte protocol)
/// Format: [SYNC(0x55)] [DEVICE_ADDR] [COMMAND] [VALUE(32)] [CRC8]
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

/// Bootloader reply structure for SPI (40-bit / 5-byte protocol)
/// NOTE: All bytes are bit-flipped in the SPI bootloader protocol
struct BootloaderReplySPI {
  uint8_t status;   ///< Status byte
  uint32_t value;   ///< 32-bit data value

  /// Deserialize from 5-byte SPI buffer (with bit-flipping)
  static BootloaderReplySPI fromBuffer(const std::array<uint8_t, 5> &in) noexcept {
    BootloaderReplySPI r;
    // All bytes are bit-flipped in the bootloader protocol
    r.status = bitFlip(in[0]);
    r.value = (static_cast<uint32_t>(bitFlip(in[1])) << 24) |
              (static_cast<uint32_t>(bitFlip(in[2])) << 16) |
              (static_cast<uint32_t>(bitFlip(in[3])) << 8) |
              static_cast<uint32_t>(bitFlip(in[4]));
    return r;
  }

  /// Check if reply indicates success
  bool isOK() const noexcept {
    return status == static_cast<uint8_t>(BootloaderStatus::OK) ||
           status == static_cast<uint8_t>(BootloaderStatus::SESSION_START);
  }
  
  /// Get status as enum
  BootloaderStatus getStatus() const noexcept {
    return static_cast<BootloaderStatus>(status);
  }
};

/// Bootloader reply structure for UART (64-bit / 8-byte protocol)
/// Format: [SYNC(0x55)] [HOST_ADDR] [STATUS] [VALUE(32)] [CRC8]
struct BootloaderReplyUART {
  uint8_t hostAddr;  ///< Host address
  uint8_t status;    ///< Status byte
  uint32_t value;    ///< 32-bit data value

  /// Deserialize from 8-byte UART buffer
  static BootloaderReplyUART fromBuffer(const std::array<uint8_t, 8> &in) noexcept {
    BootloaderReplyUART r;
    // UART protocol: [SYNC] [HOST_ADDR] [STATUS] [VALUE(32)] [CRC8]
    r.hostAddr = in[1];
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
    return status == static_cast<uint8_t>(BootloaderStatus::OK) ||
           status == static_cast<uint8_t>(BootloaderStatus::SESSION_START);
  }
  
  /// Get status as enum
  BootloaderStatus getStatus() const noexcept {
    return static_cast<BootloaderStatus>(status);
  }
};

/**
 * @brief Convenience wrapper around the bootloader commands for SPI and UART.
 *
 * The bootloader supports two different protocols:
 * 
 * **SPI Protocol (40-bit / 5-byte):**
 * - TX: [COMMAND(8)] [VALUE(32)] - All bytes are bit-flipped
 * - RX: [STATUS(8)] [VALUE(32)] - All bytes are bit-flipped
 * - Replies are delayed by one SPI transaction (standard SPI behavior)
 * - Each command requires TWO SPI transactions:
 *   1. Send command (receive previous reply, ignored)
 *   2. Send dummy frame (receive current command's reply)
 * 
 * **UART Protocol (64-bit / 8-byte):**
 * - TX: [SYNC(0x55)] [DEVICE_ADDR] [COMMAND] [VALUE(32 MSB first)] [CRC8]
 * - RX: [SYNC(0x55)] [HOST_ADDR] [STATUS] [VALUE(32 MSB first)] [CRC8]
 * - No bit-flipping (unlike SPI)
 * - CRC-8 checksum with polynomial x^8 + x^2 + x^1 + x^0
 * - Reply comes immediately (no delayed transaction like SPI)
 */
class TMC9660Bootloader {
public:
  explicit TMC9660Bootloader(TMC9660CommInterface &comm) noexcept;

  //==================================================
  // BASIC MEMORY OPERATIONS
  //==================================================

  /// Select the target register bank.
  bool setBank(uint8_t bank) noexcept;
  
  /// Select the target register bank using enum.
  bool setBank(MemoryBank bank) noexcept {
    return setBank(static_cast<uint8_t>(bank));
  }
  
  /// Set the address within the current bank.
  bool setAddress(uint32_t addr) noexcept;
  
  /// Write a single byte to the previously selected address.
  bool write8(uint8_t v) noexcept;
  
  /// Write a 16 bit word.
  bool write16(uint16_t v) noexcept;
  
  /// Write a 32 bit word.
  bool write32(uint32_t v) noexcept;
  
  /// Write a single byte and increment address by 1.
  bool write8Inc(uint8_t v) noexcept;
  
  /// Write a 16 bit word and increment address by 2.
  bool write16Inc(uint16_t v) noexcept;
  
  /// Write a 32 bit word and increment address by 4.
  bool write32Inc(uint32_t v) noexcept;
  
  /// Write multiple 32 bit words starting at the current address (uses WRITE_32_INC).
  bool write32IncMultiple(const uint32_t *values, size_t count) noexcept;
  
  /// No operation - used to retrieve reply from previous command (SPI only).
  bool noOp(uint32_t *reply = nullptr) noexcept;
  
  //==================================================
  // OTP OPERATIONS
  //==================================================
  
  /// Load an OTP page into the OTP memory bank.
  /// @param page OTP page number to load
  /// @param errorCount Returns bit error count (bits 15-8 of reply)
  /// @param pageTag Returns page tag (bits 7-0 of reply)
  /// @return true if successful
  bool otpLoad(uint8_t page, uint8_t *errorCount = nullptr, uint8_t *pageTag = nullptr) noexcept;
  
  /// Permanently burn the given OTP page.
  /// @param page OTP page number to burn (bits 7-0)
  /// @param pageAddr OTP page address to write (bits 15-8)
  /// @return true if successful
  bool otpBurn(uint8_t page, uint8_t pageAddr = 0) noexcept;
  
  //==================================================
  // EXTERNAL MEMORY OPERATIONS
  //==================================================
  
  /// Check if external memory is configured.
  /// @param bank Memory bank to check
  /// @param isConfigured Returns true if configured
  /// @return true if command successful
  bool memIsConfigured(MemoryBank bank, bool *isConfigured) noexcept;
  
  /// Check if external memory is connected.
  /// @param bank Memory bank to check
  /// @param isConnected Returns true if connected
  /// @return true if command successful
  bool memIsConnected(MemoryBank bank, bool *isConnected) noexcept;
  
  /// Check if external memory is busy.
  /// @param bank Memory bank to check
  /// @param isBusy Returns true if busy
  /// @return true if command successful
  bool memIsBusy(MemoryBank bank, bool *isBusy) noexcept;
  
  //==================================================
  // SPI FLASH OPERATIONS
  //==================================================
  
  /// Load data into internal 6-byte flash command buffer.
  /// @param offset Byte offset in buffer (0-5)
  /// @param data 24-bit data to load (bits 23-0)
  /// @return true if successful
  bool flashLoadBuffer(uint8_t offset, uint32_t data) noexcept;
  
  /// Read data from internal 6-byte flash command buffer.
  /// @param offset Byte offset in buffer (0-5)
  /// @param data Returns 24-bit data read (bits 23-0)
  /// @return true if successful
  bool flashReadBuffer(uint8_t offset, uint32_t *data) noexcept;
  
  /// Send datagram to SPI flash and receive reply.
  /// @param numBytes Number of bytes to transmit (1-6)
  /// @return true if successful
  bool flashSendDatagram(uint8_t numBytes) noexcept;
  
  /// Erase a sector on external SPI flash.
  /// @param address 24-bit sector address
  /// @return true if successful
  bool flashEraseSector(uint32_t address) noexcept;
  
  /// Read JEDEC manufacturer ID from SPI flash.
  /// @param manufacturerId Returns manufacturer ID
  /// @return true if successful
  bool flashReadJedecId(uint8_t *manufacturerId) noexcept;
  
  //==================================================
  // RS485 CONFIGURATION
  //==================================================
  
  /// Configure RS485 communication (must be first command for RS485).
  /// @param txEnPin GPIO pin for TX_EN (1=GPIO8, 2=GPIO2)
  /// @param preDelay Delay between TX_EN assertion and TX start
  /// @param hostAddr Host address
  /// @param deviceAddr Device address
  /// @return true if successful
  bool bootstrapRS485(uint8_t txEnPin, uint8_t preDelay, uint8_t hostAddr, uint8_t deviceAddr) noexcept;
  
  //==================================================
  // INFORMATION QUERIES
  //==================================================
  
  /// Get bootloader information.
  /// @param query Information type to query
  /// @param value Returns queried value
  /// @return true if successful
  bool getInfo(InfoQuery query, uint32_t *value) noexcept;
  
  /// Get CONFIG memory bank start address.
  /// @param address Returns start address
  /// @return true if successful
  bool getConfigMemStart(uint32_t *address) noexcept {
    return getInfo(InfoQuery::CONFIG_MEM_START, address);
  }
  
  /// Get CONFIG memory bank size.
  /// @param size Returns memory size
  /// @return true if successful
  bool getConfigMemSize(uint32_t *size) noexcept {
    return getInfo(InfoQuery::CONFIG_MEM_SIZE, size);
  }
  
  //==================================================
  // HIGH-LEVEL CONFIGURATION
  //==================================================

  /// Apply all fields of a ::BootloaderConfig.
  /// @param cfg Configuration to apply
  /// @return true if successful
  /// @note If cfg.boot.start_motor_control is true, bootloader will exit after this call
  bool applyConfiguration(const BootloaderConfig &cfg) noexcept;

  //==================================================
  // MOTOR CONTROL STARTUP
  //==================================================

  /// Start the motor control system and exit bootloader mode.
  /// 
  /// This function writes START_MOTOR_CTRL=1 to the BOOT_CONFIG register,
  /// causing the bootloader to immediately exit and launch the motor control
  /// application based on the current BOOT_MODE setting.
  /// 
  /// ⚠️ CRITICAL: After calling this function, the bootloader will no longer
  /// respond to commands. All bootloader configuration must be completed BEFORE
  /// calling this function.
  /// 
  /// @param bootMode Motor control mode to start (Register or Parameter mode)
  /// @return true if command sent successfully
  /// @note The bootloader exits immediately after this command
  /// @note Allow 100-150ms for motor control to fully initialize after calling
  /// 
  /// @code{.cpp}
  /// // Configure everything first
  /// bootloader.setBank(5);
  /// bootloader.setAddress(0x00020002);
  /// bootloader.write16(uart_config);
  /// // ... more configuration ...
  /// 
  /// // Finally, start motor control (bootloader exits here)
  /// bootloader.startMotorControl(tmc9660::bootcfg::BootMode::Parameter);
  /// 
  /// vTaskDelay(pdMS_TO_TICKS(150));  // Wait for motor control to start
  /// // Now use motor control commands
  /// @endcode
  bool startMotorControl(bootcfg::BootMode bootMode = bootcfg::BootMode::Parameter) noexcept;

private:
  bool sendCommand(uint8_t cmd, uint32_t value, uint32_t *reply = nullptr) noexcept;
  bool sendCommandSPI(uint8_t cmd, uint32_t value, uint32_t *reply) noexcept;
  bool sendCommandUART(uint8_t cmd, uint32_t value, uint32_t *reply) noexcept;

  TMC9660CommInterface &comm_;
  uint8_t deviceAddr_;  ///< Device address for UART protocol
  uint8_t hostAddr_;    ///< Host address for UART protocol
};

} // namespace tmc9660
