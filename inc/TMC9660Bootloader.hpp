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
/// UART device/host address register.
constexpr uint32_t UART_ADDR = BASE + 0x02;
/// RS485 TXEN delay configuration.
constexpr uint32_t RS485_DELAY = BASE + 0x04;
/// Communication selection (UART/SPI/RS485).
constexpr uint32_t COMM_CONFIG = BASE + 0x06;
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

/**
 * @brief Convenience wrapper around the bootloader TMCL commands.
 *
 * The bootloader uses a small command set for writing configuration words
 * into an internal RAM.  This class provides helpers to build these commands
 * and write a complete ::BootloaderConfig structure.
 */
class TMC9660Bootloader {
public:
  explicit TMC9660Bootloader(TMC9660CommInterface &comm, uint8_t address = 0) noexcept;

  /// Select the target register bank.
  bool setBank(uint8_t bank) noexcept;
  /// Set the address within the current bank.
  bool setAddress(uint32_t addr) noexcept;
  /// Write a single byte to the previously selected address.
  bool write8(uint8_t v) noexcept;
  /// Write a 16 bit word.
  bool write16(uint16_t v) noexcept;
  /// Write a 32 bit word.
  bool write32(uint32_t v) noexcept;
  /// Write multiple 32 bit words starting at the current address.
  bool write32Inc(const uint32_t *values, size_t count) noexcept;
  /// Permanently burn the given OTP page.
  bool otpBurn(uint8_t page) noexcept;

  /// Apply all fields of a ::BootloaderConfig.
  bool applyConfiguration(const BootloaderConfig &cfg) noexcept;

private:
  bool sendCommand(uint8_t op, uint16_t type, uint8_t bank, uint32_t value,
                   uint32_t *reply = nullptr) noexcept;

  TMC9660CommInterface &comm_;
  uint8_t address_;
};

} // namespace tmc9660
