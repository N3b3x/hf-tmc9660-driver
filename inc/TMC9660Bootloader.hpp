/**
 * @file TMC9660Bootloader.hpp
 * @brief Helper for configuring the TMC9660 bootloader registers.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "TMC9660CommInterface.hpp"

namespace tmc9660 {

namespace bootcfg {

/// Constants used when building a ::BootloaderConfig.

constexpr uint8_t LDO_DISABLED = 0;
constexpr uint8_t LDO_2V5     = 1;
constexpr uint8_t LDO_3V3     = 2;
constexpr uint8_t LDO_5V0     = 3;

constexpr uint8_t LDO_SLOPE_3MS    = 0;
constexpr uint8_t LDO_SLOPE_1_5MS  = 1;
constexpr uint8_t LDO_SLOPE_0_75MS = 2;
constexpr uint8_t LDO_SLOPE_0_37MS = 3;

constexpr uint8_t BOOT_MODE_REGISTER  = 1;
constexpr uint8_t BOOT_MODE_PARAMETER = 2;

constexpr uint8_t UART_RX_GPIO7 = 0;
constexpr uint8_t UART_RX_GPIO1 = 1;
constexpr uint8_t UART_TX_GPIO6 = 0;
constexpr uint8_t UART_TX_GPIO0 = 1;

constexpr uint8_t UART_BAUD_9600   = 0;
constexpr uint8_t UART_BAUD_19200  = 1;
constexpr uint8_t UART_BAUD_38400  = 2;
constexpr uint8_t UART_BAUD_57600  = 3;
constexpr uint8_t UART_BAUD_115200 = 4;
constexpr uint8_t UART_BAUD_1000000 = 5;
constexpr uint8_t UART_BAUD_AUTO_8X  = 6;
constexpr uint8_t UART_BAUD_AUTO_16X = 7;

constexpr uint8_t RS485_TXEN_NONE = 0;
constexpr uint8_t RS485_TXEN_GPIO8 = 1;
constexpr uint8_t RS485_TXEN_GPIO2 = 2;

constexpr uint8_t SPI_IFACE0 = 0;
constexpr uint8_t SPI_IFACE1 = 1;

constexpr uint8_t SPI0_SCK_GPIO6  = 0;
constexpr uint8_t SPI0_SCK_GPIO11 = 1;

constexpr uint8_t SPI_FLASH_FREQ_DIV1  = 0;
constexpr uint8_t SPI_FLASH_FREQ_DIV2  = 1;
constexpr uint8_t SPI_FLASH_FREQ_DIV4  = 3;

constexpr uint8_t I2C_SDA_GPIO5  = 0;
constexpr uint8_t I2C_SDA_GPIO11 = 1;
constexpr uint8_t I2C_SDA_GPIO14 = 2;

constexpr uint8_t I2C_SCL_GPIO4  = 0;
constexpr uint8_t I2C_SCL_GPIO12 = 1;
constexpr uint8_t I2C_SCL_GPIO13 = 2;

constexpr uint8_t I2C_FREQ_100KHZ = 0;
constexpr uint8_t I2C_FREQ_200KHZ = 1;
constexpr uint8_t I2C_FREQ_400KHZ = 2;
constexpr uint8_t I2C_FREQ_800KHZ = 3;

constexpr uint8_t CLK_USE_INTERNAL = 0;
constexpr uint8_t CLK_USE_EXTERNAL = 1;

constexpr uint8_t EXT_SOURCE_OSCILLATOR = 0;
constexpr uint8_t EXT_SOURCE_CLOCK     = 1;

constexpr uint8_t XTAL_FREQ_8MHZ   = 1;
constexpr uint8_t XTAL_FREQ_16MHZ  = 3;
constexpr uint8_t XTAL_FREQ_24MHZ  = 5;
constexpr uint8_t XTAL_FREQ_32MHZ  = 6;

constexpr uint8_t SYSCLK_USE_INTOSC = 0;
constexpr uint8_t SYSCLK_USE_PLL    = 1;

constexpr uint8_t SYSCLK_DIV_1   = 0;
constexpr uint8_t SYSCLK_DIV_2_3 = 3;

} // namespace bootcfg

namespace bootaddr {
/// Base offset of the configuration registers inside bank 5.
constexpr uint32_t BASE = 0x00020000;
/// UART device/host address register.
constexpr uint32_t UART_ADDR      = BASE + 0x02;
/// RS485 TXEN delay configuration.
constexpr uint32_t RS485_DELAY    = BASE + 0x04;
/// Communication selection (UART/SPI/RS485).
constexpr uint32_t COMM_CONFIG    = BASE + 0x06;
/// SPI flash configuration register.
constexpr uint32_t SPI_FLASH      = BASE + 0x0A;
/// I2C EEPROM configuration register.
constexpr uint32_t I2C_CONFIG     = BASE + 0x0C;
/// GPIO output level register.
constexpr uint32_t GPIO_OUT       = BASE + 0x0E;
/// GPIO direction register.
constexpr uint32_t GPIO_DIR       = BASE + 0x10;
/// GPIO pull-up register.
constexpr uint32_t GPIO_PU        = BASE + 0x12;
/// GPIO pull-down register.
constexpr uint32_t GPIO_PD        = BASE + 0x14;
/// GPIO analog enable register.
constexpr uint32_t GPIO_ANALOG    = BASE + 0x16;
/// Clock configuration register.
constexpr uint32_t CLOCK_CONFIG   = BASE + 0x18;
} // namespace bootaddr

/// Configuration of the on-chip LDO regulators.
struct LDOConfig {
  uint8_t vext1{bootcfg::LDO_DISABLED};
  uint8_t vext2{bootcfg::LDO_DISABLED};
  uint8_t slope_vext1{bootcfg::LDO_SLOPE_3MS};
  uint8_t slope_vext2{bootcfg::LDO_SLOPE_3MS};
  bool    ldo_short_fault{false};
};

/// Bootloader behaviour configuration.
struct BootConfig {
  uint8_t boot_mode{bootcfg::BOOT_MODE_REGISTER};
  bool    bl_ready_fault{false};
  bool    bl_exit_fault{true};
  bool    disable_selftest{false};
  bool    bl_config_fault{false};
  bool    start_motor_control{false};
};

/// UART communication settings for the bootloader.
struct UARTConfig {
  uint8_t device_address{1};
  uint8_t host_address{255};
  bool    disable_uart{false};
  uint8_t rx_pin{bootcfg::UART_RX_GPIO7};
  uint8_t tx_pin{bootcfg::UART_TX_GPIO6};
  uint8_t baud_rate{bootcfg::UART_BAUD_115200};
};

/// Optional RS485 transceiver control via the UART_TXEN pin.
struct RS485Config {
  bool    enable_rs485{false};
  uint8_t txen_pin{bootcfg::RS485_TXEN_NONE};
  uint8_t txen_pre_delay{0};
  uint8_t txen_post_delay{0};
};

/// SPI interface used for bootloader commands.
struct SPIBootConfig {
  bool    disable_spi{false};
  uint8_t boot_spi_iface{bootcfg::SPI_IFACE0};
  uint8_t spi0_sck_pin{bootcfg::SPI0_SCK_GPIO6};
};

/// External SPI flash configuration.
struct SPIFlashConfig {
  bool    enable_flash{false};
  uint8_t flash_spi_iface{bootcfg::SPI_IFACE1};
  uint8_t spi0_sck_pin{bootcfg::SPI0_SCK_GPIO6};
  uint8_t cs_pin{0};
  uint8_t freq_div{bootcfg::SPI_FLASH_FREQ_DIV4};
};

/// External I2C EEPROM configuration.
struct I2CConfig {
  bool    enable_eeprom{false};
  uint8_t sda_pin{bootcfg::I2C_SDA_GPIO5};
  uint8_t scl_pin{bootcfg::I2C_SCL_GPIO4};
  uint8_t address_bits{0};
  uint8_t freq_code{bootcfg::I2C_FREQ_100KHZ};
};

/// System clock selection parameters.
struct ClockConfig {
  uint8_t use_external{bootcfg::CLK_USE_INTERNAL};
  uint8_t ext_source_type{bootcfg::EXT_SOURCE_OSCILLATOR};
  uint8_t xtal_drive{bootcfg::XTAL_FREQ_16MHZ};
  bool    xtal_boost{false};
  uint8_t pll_selection{bootcfg::SYSCLK_USE_PLL};
  uint8_t rdiv{14};
  uint8_t sysclk_div{bootcfg::SYSCLK_DIV_1};
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
  LDOConfig     ldo;
  BootConfig    boot;
  UARTConfig    uart;
  RS485Config   rs485;
  SPIBootConfig spiComm;
  SPIFlashConfig spiFlash;
  I2CConfig     i2c;
  ClockConfig   clock;
  GPIOConfig    gpio;
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
  bool sendCommand(uint8_t op, uint16_t type, uint8_t bank, uint32_t value, uint32_t *reply = nullptr) noexcept;

  TMC9660CommInterface &comm_;
  uint8_t address_;
};

} // namespace tmc9660

