/**
 * @file bootloader_config.hpp
 * @brief TMC9660 bootloader configuration structures and enumerations
 * @copyright Copyright (c) 2024-2025 HardFOC. All rights reserved.
 */
#pragma once
#include <cstdint>

namespace tmc9660 {

/**
 * @brief Bootloader configuration namespace containing all configuration structures and enums.
 * @ingroup TMC9660_BootloaderConfig
 *
 * This namespace contains all the configuration structures, enumerations, and
 * constants needed to configure the TMC9660 bootloader. These structures
 * define hardware pin assignments, communication parameters, power supply
 * settings, and system behavior options.
 */
namespace bootcfg {

/**
 * @brief Enumerations describing bootloader configuration options.
 * @ingroup TMC9660_BootloaderConfig
 *
 * These enumerations define the various configuration options available
 * for the TMC9660 bootloader, including voltage settings, communication
 * parameters, and hardware pin configurations.
 */

/**
 * @brief LDO voltage configuration options for external power supplies.
 *
 * Configures the on-chip LDO regulators for VEXT1 and VEXT2 outputs.
 * These voltages power external components connected to the TMC9660.
 */
enum class LDOVoltage : uint8_t {
  Disabled = 0, ///< LDO output disabled
  V2_5 = 1,     ///< 2.5V output voltage
  V3_3 = 2,     ///< 3.3V output voltage
  V5_0 = 3,     ///< 5.0V output voltage
};

/**
 * @brief LDO output slope control for power-up characteristics.
 *
 * Controls the rise time of the LDO output voltage during power-up.
 * Slower slopes reduce inrush current but increase power-up time.
 */
enum class LDOSlope : uint8_t {
  Slope3ms = 0,    ///< 3ms rise time (slowest, lowest inrush current)
  Slope1_5ms = 1,  ///< 1.5ms rise time
  Slope0_75ms = 2, ///< 0.75ms rise time
  Slope0_37ms = 3, ///< 0.37ms rise time (fastest, highest inrush current)
};

/**
 * @brief Boot mode selection for motor control system startup.
 *
 * Determines which motor control mode the TMC9660 will enter after
 * bootloader configuration is complete and START_MOTOR_CTRL is set.
 */
enum class BootMode : uint8_t {
  Register = 1,  ///< Register mode - direct register access for motor control
  Parameter = 2, ///< Parameter mode - TMCL command-based motor control (recommended)
};

/**
 * @brief UART receive pin selection for bootloader communication.
 *
 * Selects which GPIO pin is used for UART RX signal during bootloader mode.
 */
enum class UartRxPin : uint8_t {
  GPIO7 = 0, ///< Use GPIO7 for UART RX (default)
  GPIO1 = 1  ///< Use GPIO1 for UART RX (alternative)
};

/**
 * @brief UART transmit pin selection for bootloader communication.
 *
 * Selects which GPIO pin is used for UART TX signal during bootloader mode.
 */
enum class UartTxPin : uint8_t {
  GPIO6 = 0, ///< Use GPIO6 for UART TX (default)
  GPIO0 = 1  ///< Use GPIO0 for UART TX (alternative)
};

/**
 * @brief UART baud rate configuration for bootloader communication.
 *
 * Configures the UART communication speed. Auto modes detect the baud rate
 * automatically by analyzing the received data pattern.
 */
enum class BaudRate : uint8_t {
  BR9600 = 0, ///< 9600 baud (slowest, most reliable)
  BR19200,    ///< 19200 baud
  BR38400,    ///< 38400 baud
  BR57600,    ///< 57600 baud
  BR115200,   ///< 115200 baud (recommended for most applications)
  BR1000000,  ///< 1000000 baud (1 Mbps, fastest)
  Auto8x,     ///< Auto-detect with 8x oversampling
  Auto16x,    ///< Auto-detect with 16x oversampling (most robust)
};

/**
 * @brief RS485 transmit enable pin selection for half-duplex communication.
 *
 * Selects which GPIO pin controls the RS485 transceiver's transmit enable.
 * When None is selected, RS485 mode is disabled.
 */
enum class RS485TxEnPin : uint8_t {
  None = 0,  ///< No RS485 TX enable pin (RS485 disabled)
  GPIO8 = 1, ///< Use GPIO8 for RS485 TX enable
  GPIO2 = 2  ///< Use GPIO2 for RS485 TX enable
};

/**
 * @brief SPI interface selection for bootloader and flash communication.
 *
 * The TMC9660 has two physical SPI interfaces: SPI0 and SPI1.
 *
 * **CRITICAL: BL_SPI_SELECT (bit 2 of COMM_CONFIG) has OPPOSITE meanings:**
 * - For **Bootloader**: BL_SPI_SELECT=0 means SPI0, BL_SPI_SELECT=1 means SPI1
 * - For **Flash**: BL_SPI_SELECT=0 means SPI1, BL_SPI_SELECT=1 means SPI0 (inverted!)
 *
 * **This enum uses the PHYSICAL interface names (SPI0/SPI1).**
 * The driver handles the bit inversion automatically in applyConfiguration().
 *
 * **Constraint:** If both bootloader and flash are enabled, they MUST use
 * different SPI interfaces (one on SPI0, the other on SPI1).
 */
enum class SPIInterface : uint8_t {
  SPI0 = 0, ///< Physical SPI0 interface
  SPI1 = 1  ///< Physical SPI1 interface
};

/**
 * @brief SPI0 clock pin selection for bootloader communication.
 *
 * Selects which GPIO pin is used for SPI0 SCK (serial clock) signal.
 * This affects the bootloader communication interface.
 */
enum class SPI0SckPin : uint8_t {
  GPIO6 = 0, ///< Use GPIO6 for SPI0 SCK (default)
  GPIO11 = 1 ///< Use GPIO11 for SPI0 SCK (alternative)
};

/**
 * @brief SPI flash frequency divider configuration.
 *
 * Controls the SPI clock frequency for external flash communication.
 * Higher divider values result in slower, more reliable communication.
 */
enum class SPIFlashFreq : uint8_t {
  Div1 = 0, ///< No division (fastest, may be unreliable)
  Div2 = 1, ///< Divide by 2 (medium speed)
  Div4 = 3  ///< Divide by 4 (slowest, most reliable)
};

/**
 * @brief I2C data pin selection for external EEPROM communication.
 *
 * Selects which GPIO pin is used for I2C SDA (serial data) signal.
 * Used for communication with external I2C EEPROM devices.
 */
enum class I2CSdaPin : uint8_t {
  GPIO5 = 0,  ///< Use GPIO5 for I2C SDA (default)
  GPIO11 = 1, ///< Use GPIO11 for I2C SDA
  GPIO14 = 2  ///< Use GPIO14 for I2C SDA
};

/**
 * @brief I2C clock pin selection for external EEPROM communication.
 *
 * Selects which GPIO pin is used for I2C SCL (serial clock) signal.
 * Used for communication with external I2C EEPROM devices.
 */
enum class I2CSclPin : uint8_t {
  GPIO4 = 0,  ///< Use GPIO4 for I2C SCL (default)
  GPIO12 = 1, ///< Use GPIO12 for I2C SCL
  GPIO13 = 2  ///< Use GPIO13 for I2C SCL
};

/**
 * @brief I2C communication frequency configuration.
 *
 * Sets the I2C clock frequency for external EEPROM communication.
 * Higher frequencies enable faster data transfer but may be less reliable.
 */
enum class I2CFreq : uint8_t {
  Freq100k = 0, ///< 100 kHz (standard mode, most compatible)
  Freq200k,     ///< 200 kHz (fast mode)
  Freq400k,     ///< 400 kHz (fast mode plus)
  Freq800k      ///< 800 kHz (high speed mode, fastest)
};

/**
 * @brief System clock source selection.
 *
 * Determines whether the TMC9660 uses its internal oscillator or an external
 * clock source for system timing.
 */
enum class ClockSource : uint8_t {
  Internal = 0, ///< Use internal oscillator (default, no external components needed)
  External = 1  ///< Use external clock source (requires external crystal/oscillator)
};

/**
 * @brief External clock source type configuration.
 *
 * Specifies the type of external clock source when ClockSource::External is selected.
 * This affects the input buffer configuration and clock processing.
 */
enum class ExtSourceType : uint8_t {
  Oscillator = 0, ///< External crystal oscillator (requires external crystal)
  Clock = 1       ///< External clock signal (digital clock input)
};

/**
 * @brief Crystal drive strength configuration for external oscillators.
 *
 * Configures the drive strength of the internal oscillator when using
 * an external crystal. Higher drive strength is needed for higher frequency crystals.
 */
enum class XtalDrive : uint8_t {
  Freq8MHz = 1,  ///< 8 MHz crystal drive strength
  Freq16MHz = 3, ///< 16 MHz crystal drive strength (recommended)
  Freq24MHz = 5, ///< 24 MHz crystal drive strength
  Freq32MHz = 6, ///< 32 MHz crystal drive strength (highest frequency)
};

/**
 * @brief System clock source selection after initial oscillator.
 *
 * Determines whether the system clock is derived directly from the oscillator
 * or through the PLL (Phase-Locked Loop) for frequency multiplication.
 */
enum class SysClkSource : uint8_t {
  IntOsc = 0, ///< Use internal oscillator directly (lower power, fixed frequency)
  PLL = 1     ///< Use PLL for frequency multiplication (higher performance, configurable)
};

/**
 * @brief System clock divider configuration.
 *
 * Divides the system clock to achieve the desired operating frequency.
 * Lower divider values result in higher system clock frequencies.
 */
enum class SysClkDiv : uint8_t {
  Div1 = 0,    ///< No division (40 MHz system clock)
  Div15MHz = 3 ///< Divide by 3 (15 MHz system clock, lower power)
};

/**
 * @brief Hall encoder U-phase pin selection for BLDC motor feedback.
 *
 * Selects which GPIO pin is used for the Hall sensor U-phase signal.
 * Used for BLDC motor commutation and position feedback.
 */
enum class HallUPin : uint8_t {
  GPIO2 = 0, ///< Use GPIO2 for Hall U-phase (default)
  GPIO7 = 1, ///< Use GPIO7 for Hall U-phase
  GPIO9 = 2  ///< Use GPIO9 for Hall U-phase
};

/**
 * @brief Hall encoder V-phase pin selection for BLDC motor feedback.
 *
 * Selects which GPIO pin is used for the Hall sensor V-phase signal.
 * Used for BLDC motor commutation and position feedback.
 */
enum class HallVPin : uint8_t {
  GPIO3 = 0, ///< Use GPIO3 for Hall V-phase (default)
  GPIO15 = 1 ///< Use GPIO15 for Hall V-phase
};

/**
 * @brief Hall encoder W-phase pin selection for BLDC motor feedback.
 *
 * Selects which GPIO pin is used for the Hall sensor W-phase signal.
 * Used for BLDC motor commutation and position feedback.
 */
enum class HallWPin : uint8_t {
  GPIO4 = 0, ///< Use GPIO4 for Hall W-phase (default)
  GPIO8 = 1, ///< Use GPIO8 for Hall W-phase
  GPIO10 = 2 ///< Use GPIO10 for Hall W-phase
};

/**
 * @brief ABN encoder 1 A-phase pin selection for incremental encoder feedback.
 *
 * Selects which GPIO pin is used for the ABN encoder 1 A-phase signal.
 * Used for precise position and velocity feedback from incremental encoders.
 */
enum class ABN1APin : uint8_t {
  GPIO5 = 0, ///< Use GPIO5 for ABN1 A-phase (default)
  GPIO8 = 1, ///< Use GPIO8 for ABN1 A-phase
  GPIO17 = 2 ///< Use GPIO17 for ABN1 A-phase
};

/**
 * @brief ABN encoder 1 B-phase pin selection for incremental encoder feedback.
 *
 * Selects which GPIO pin is used for the ABN encoder 1 B-phase signal.
 * Used for precise position and velocity feedback from incremental encoders.
 */
enum class ABN1BPin : uint8_t {
  GPIO1 = 0,  ///< Use GPIO1 for ABN1 B-phase (default)
  GPIO13 = 1, ///< Use GPIO13 for ABN1 B-phase
  GPIO18 = 2  ///< Use GPIO18 for ABN1 B-phase
};

/**
 * @brief ABN encoder 1 index pin selection for incremental encoder feedback.
 *
 * Selects which GPIO pin is used for the ABN encoder 1 index (Z) signal.
 * The index signal provides a reference position for absolute positioning.
 */
enum class ABN1NPin : uint8_t {
  Disabled = 0, ///< No index pin (index signal not used)
  GPIO14 = 1,   ///< Use GPIO14 for ABN1 index signal
  GPIO16 = 2    ///< Use GPIO16 for ABN1 index signal
};

/**
 * @brief ABN encoder 2 A-phase pin selection for second incremental encoder.
 *
 * Selects which GPIO pin is used for the ABN encoder 2 A-phase signal.
 * Used for dual-encoder applications or secondary position feedback.
 */
enum class ABN2APin : uint8_t {
  GPIO6 = 0, ///< Use GPIO6 for ABN2 A-phase (default)
  GPIO15 = 1 ///< Use GPIO15 for ABN2 A-phase
};

/**
 * @brief ABN encoder 2 B-phase pin selection for second incremental encoder.
 *
 * Selects which GPIO pin is used for the ABN encoder 2 B-phase signal.
 * Used for dual-encoder applications or secondary position feedback.
 */
enum class ABN2BPin : uint8_t {
  GPIO7 = 0,  ///< Use GPIO7 for ABN2 B-phase (default)
  GPIO11 = 1, ///< Use GPIO11 for ABN2 B-phase
  GPIO16 = 2  ///< Use GPIO16 for ABN2 B-phase
};

/**
 * @brief Reference switch left pin selection for limit switch detection.
 *
 * Selects which GPIO pin is used for the left reference switch signal.
 * Used for detecting mechanical limits and home position.
 */
enum class RefLPin : uint8_t {
  Disabled = 0, ///< No left reference switch (disabled)
  GPIO2 = 1,    ///< Use GPIO2 for left reference switch
  GPIO12 = 2,   ///< Use GPIO12 for left reference switch
  GPIO16 = 3    ///< Use GPIO16 for left reference switch
};

/**
 * @brief Reference switch right pin selection for limit switch detection.
 *
 * Selects which GPIO pin is used for the right reference switch signal.
 * Used for detecting mechanical limits and home position.
 */
enum class RefRPin : uint8_t {
  Disabled = 0, ///< No right reference switch (disabled)
  GPIO3 = 1,    ///< Use GPIO3 for right reference switch
  GPIO18 = 2    ///< Use GPIO18 for right reference switch
};

/**
 * @brief Reference switch home pin selection for home position detection.
 *
 * Selects which GPIO pin is used for the home reference switch signal.
 * Used for detecting the home position during homing sequences.
 */
enum class RefHPin : uint8_t {
  Disabled = 0, ///< No home reference switch (disabled)
  GPIO4 = 1,    ///< Use GPIO4 for home reference switch
  GPIO7 = 2,    ///< Use GPIO7 for home reference switch
  GPIO15 = 3,   ///< Use GPIO15 for home reference switch
  GPIO17 = 4    ///< Use GPIO17 for home reference switch
};

/**
 * @brief Step pin selection for step/direction interface.
 *
 * Selects which GPIO pin is used for the step signal in step/direction mode.
 * Used for controlling stepper motors or providing step pulses to external controllers.
 */
enum class StepPin : uint8_t {
  GPIO7 = 0,  ///< Use GPIO7 for step signal (default)
  GPIO11 = 1, ///< Use GPIO11 for step signal
  GPIO16 = 2  ///< Use GPIO16 for step signal
};

/**
 * @brief Direction pin selection for step/direction interface.
 *
 * Selects which GPIO pin is used for the direction signal in step/direction mode.
 * Used for controlling stepper motors or providing direction signals to external controllers.
 */
enum class DirPin : uint8_t {
  GPIO6 = 0, ///< Use GPIO6 for direction signal (default)
  GPIO15 = 1 ///< Use GPIO15 for direction signal
};

/**
 * @brief SPI encoder interface block selection.
 *
 * Selects which SPI interface is used for communication with external
 * SPI-based encoders or position sensors.
 */
enum class SPIEncBlock : uint8_t {
  SPI0 = 0, ///< Use SPI interface 0 for encoder communication
  SPI1 = 1  ///< Use SPI interface 1 for encoder communication
};

/**
 * @brief SPI encoder communication mode configuration.
 *
 * Configures the SPI mode for encoder communication. Different encoders
 * may require different SPI modes for proper communication.
 */
enum class SPIEncMode : uint8_t {
  Mode0 = 0, ///< SPI Mode 0 (CPOL=0, CPHA=0)
  Mode1 = 1, ///< SPI Mode 1 (CPOL=0, CPHA=1)
  Mode2 = 2, ///< SPI Mode 2 (CPOL=1, CPHA=0)
  Mode3 = 3  ///< SPI Mode 3 (CPOL=1, CPHA=1)
};

/**
 * @brief SPI encoder clock frequency divider configuration.
 *
 * Controls the SPI clock frequency for encoder communication.
 * Higher divider values result in slower, more reliable communication.
 */
enum class SPIEncFreq : uint8_t {
  Div4 = 0,   ///< Divide by 4 (fastest)
  Div5 = 1,   ///< Divide by 5
  Div6 = 2,   ///< Divide by 6
  Div7 = 3,   ///< Divide by 7
  Div8 = 4,   ///< Divide by 8
  Div9 = 5,   ///< Divide by 9
  Div10 = 6,  ///< Divide by 10
  Div11 = 7,  ///< Divide by 11
  Div12 = 8,  ///< Divide by 12
  Div13 = 9,  ///< Divide by 13
  Div14 = 10, ///< Divide by 14
  Div15 = 11, ///< Divide by 15
  Div16 = 12, ///< Divide by 16
  Div17 = 13, ///< Divide by 17
  Div18 = 14, ///< Divide by 18
  Div19 = 15  ///< Divide by 19 (slowest, most reliable)
};

/**
 * @brief SPI encoder chip select pin selection.
 *
 * Selects which GPIO pin is used for the SPI encoder chip select signal.
 * The available pins depend on which SPI interface is selected.
 */
enum class SPIEncCSPin : uint8_t {
  // SPI0 options
  GPIO8 = 0,  ///< Use GPIO8 for SPI0 encoder CS
  GPIO12 = 1, ///< Use GPIO12 for SPI0 encoder CS
  GPIO13 = 2, ///< Use GPIO13 for SPI0 encoder CS
  GPIO16 = 3, ///< Use GPIO16 for SPI0 encoder CS
  // SPI1 options (same enum values, context determines which)
  GPIO15_SPI1 = 0 ///< Use GPIO15 for SPI1 encoder CS
};

/**
 * @brief SPI encoder chip select polarity configuration.
 *
 * Configures the active level of the chip select signal for SPI encoder communication.
 * Most SPI devices use active-low chip select.
 */
enum class SPIEncCSPol : uint8_t {
  ActiveHigh = 0, ///< Chip select active high (CS high = selected)
  ActiveLow = 1   ///< Chip select active low (CS low = selected, most common)
};

/**
 * @brief Mechanical brake output pin selection.
 *
 * Selects which output is used to control an external mechanical brake.
 * Used for holding motor position when power is removed.
 */
enum class MechBrakeOutput : uint8_t {
  GPIO8 = 0,  ///< Use GPIO8 for mechanical brake control
  GPIO10 = 1, ///< Use GPIO10 for mechanical brake control
  GPIO18 = 2, ///< Use GPIO18 for mechanical brake control
  Y2_LS = 3   ///< Use Y2_LS (low-side output) for mechanical brake control
};

/**
 * @brief Brake chopper output pin selection for dynamic braking.
 *
 * Selects which output is used for the brake chopper circuit.
 * Used for dynamic braking to quickly stop the motor by shorting the windings.
 */
enum class BrakeChopperOutput : uint8_t {
  GPIO0 = 0,   ///< Use GPIO0 for brake chopper output
  GPIO1 = 1,   ///< Use GPIO1 for brake chopper output
  GPIO2 = 2,   ///< Use GPIO2 for brake chopper output
  GPIO3 = 3,   ///< Use GPIO3 for brake chopper output
  GPIO4 = 4,   ///< Use GPIO4 for brake chopper output
  GPIO5 = 5,   ///< Use GPIO5 for brake chopper output
  GPIO6 = 6,   ///< Use GPIO6 for brake chopper output
  GPIO7 = 7,   ///< Use GPIO7 for brake chopper output
  GPIO8 = 8,   ///< Use GPIO8 for brake chopper output
  GPIO9 = 9,   ///< Use GPIO9 for brake chopper output
  GPIO10 = 10, ///< Use GPIO10 for brake chopper output
  GPIO11 = 11, ///< Use GPIO11 for brake chopper output
  GPIO12 = 12, ///< Use GPIO12 for brake chopper output
  GPIO13 = 13, ///< Use GPIO13 for brake chopper output
  GPIO14 = 14, ///< Use GPIO14 for brake chopper output
  GPIO15 = 15, ///< Use GPIO15 for brake chopper output
  GPIO16 = 16, ///< Use GPIO16 for brake chopper output
  GPIO17 = 17, ///< Use GPIO17 for brake chopper output
  GPIO18 = 18, ///< Use GPIO18 for brake chopper output
  Y2_HS = 19   ///< Use Y2_HS (high-side output) for brake chopper output
};

/**
 * @brief External memory storage type selection.
 *
 * Configures which type of external memory is used for storing
 * TMCL scripts and parameter data.
 */
enum class MemStorage : uint8_t {
  Disabled = 0, ///< No external memory (disabled)
  SPIFlash = 1, ///< Use SPI flash memory for storage
  I2CEEPROM = 2 ///< Use I2C EEPROM for storage
};

} // namespace bootcfg

/**
 * @brief Bootloader configuration register addresses.
 *
 * This namespace contains the memory addresses for all bootloader configuration
 * registers within bank 5 (CONFIG memory bank). These addresses are used to
 * configure the TMC9660's bootloader behavior and hardware settings.
 */
namespace bootaddr {
/**
 * @brief Base offset of the configuration registers inside bank 5.
 *
 * All bootloader configuration registers are located in memory bank 5,
 * starting at this base address.
 */
constexpr uint32_t BASE = 0x00020000;

/**
 * @brief LDO configuration register address.
 *
 * Configures the on-chip LDO regulators for VEXT1 and VEXT2 outputs,
 * including voltage levels, slope control, and fault detection.
 */
constexpr uint32_t LDO_CONFIG = BASE + 0x00;

/**
 * @brief UART device/host address register.
 *
 * Configures the UART communication addresses for device and host
 * identification in multi-device systems.
 */
constexpr uint32_t UART_ADDR = BASE + 0x02;

/**
 * @brief RS485 TXEN delay configuration register.
 *
 * Configures the timing delays for RS485 transmit enable signal
 * to ensure proper half-duplex communication.
 */
constexpr uint32_t RS485_DELAY = BASE + 0x04;

/**
 * @brief Communication interface selection register.
 *
 * Configures which communication interfaces are enabled (UART/SPI/RS485)
 * and their associated pin assignments and parameters.
 */
constexpr uint32_t COMM_CONFIG = BASE + 0x06;

/**
 * @brief Boot configuration register.
 *
 * Controls the bootloader behavior including boot mode selection,
 * motor control startup, and fault handling options.
 *
 * @warning CRITICAL: Write this register LAST with START_MOTOR_CTRL=1 to exit bootloader!
 */
constexpr uint32_t BOOT_CONFIG = BASE + 0x08;

/**
 * @brief SPI flash configuration register.
 *
 * Configures external SPI flash memory interface including
 * chip select pin, frequency divider, and enable settings.
 */
constexpr uint32_t SPI_FLASH = BASE + 0x0A;

/**
 * @brief I2C EEPROM configuration register.
 *
 * Configures external I2C EEPROM interface including
 * pin assignments, address bits, and communication frequency.
 */
constexpr uint32_t I2C_CONFIG = BASE + 0x0C;

/**
 * @brief GPIO output level register.
 *
 * Sets the initial output levels for GPIO pins 0-15
 * during bootloader operation.
 */
constexpr uint32_t GPIO_OUT = BASE + 0x0E;

/**
 * @brief GPIO direction register.
 *
 * Configures GPIO pins 0-15 as inputs or outputs
 * during bootloader operation.
 */
constexpr uint32_t GPIO_DIR = BASE + 0x10;

/**
 * @brief GPIO pull-up enable register.
 *
 * Enables internal pull-up resistors for GPIO pins 0-15
 * during bootloader operation.
 */
constexpr uint32_t GPIO_PU = BASE + 0x12;

/**
 * @brief GPIO pull-down enable register.
 *
 * Enables internal pull-down resistors for GPIO pins 0-15
 * during bootloader operation.
 */
constexpr uint32_t GPIO_PD = BASE + 0x14;

/**
 * @brief GPIO extended configuration register.
 *
 * Configures GPIO pins 16-18 and analog GPIOs 2-5,
 * including output levels, directions, and pull settings.
 */
constexpr uint32_t GPIO_EXT = BASE + 0x16;

/**
 * @brief Clock configuration register.
 *
 * Configures the system clock source, PLL settings,
 * and frequency dividers for optimal performance.
 */
constexpr uint32_t CLOCK_CONFIG = BASE + 0x18;

/**
 * @brief Hall encoder configuration register.
 *
 * Configures Hall sensor pins and enable settings for BLDC motor feedback.
 * This register is shared with ABN1 configuration at offset 0x20.
 */
constexpr uint32_t HALL_CONFIG = BASE + 0x20;

/**
 * @brief ABN encoder 1 configuration register.
 *
 * Configures ABN encoder 1 pins and enable settings for incremental encoder feedback.
 * This register is shared with Hall encoder configuration at offset 0x20.
 */
constexpr uint32_t ABN1_CONFIG = BASE + 0x20;

/**
 * @brief ABN encoder 2 configuration register.
 *
 * Configures ABN encoder 2 pins and enable settings for second incremental encoder.
 * This register is shared with reference switches and step/direction at offset 0x22.
 */
constexpr uint32_t ABN2_CONFIG = BASE + 0x22;

/**
 * @brief Reference switches configuration register.
 *
 * Configures reference switch pins for limit detection and homing.
 * This register is shared with ABN2 and step/direction at offset 0x22.
 */
constexpr uint32_t REF_CONFIG = BASE + 0x22;

/**
 * @brief Step/Direction interface configuration register.
 *
 * Configures step and direction pins for stepper motor control.
 * This register is shared with ABN2 and reference switches at offset 0x22.
 */
constexpr uint32_t STEPDIR_CONFIG = BASE + 0x22;

/**
 * @brief SPI encoder configuration register.
 *
 * Configures SPI encoder interface including block selection,
 * communication mode, frequency, and chip select settings.
 */
constexpr uint32_t SPI_ENC_CONFIG = BASE + 0x26;

/**
 * @brief Mechanical brake configuration register.
 *
 * Configures mechanical brake output pin and enable settings.
 * This register is shared with brake chopper at offset 0x24.
 */
constexpr uint32_t MECH_BRAKE_CONFIG = BASE + 0x24;

/**
 * @brief Brake chopper configuration register.
 *
 * Configures brake chopper output pin and enable settings for dynamic braking.
 * This register is shared with mechanical brake at offset 0x24.
 */
constexpr uint32_t BRAKECHOPPER_CONFIG = BASE + 0x24;

/**
 * @brief External memory storage selection register.
 *
 * Configures which external memory types are used for storing
 * TMCL scripts and parameter data.
 */
constexpr uint32_t MEM_STORAGE_CONFIG = BASE + 0x28;
} // namespace bootaddr

/**
 * @brief Configuration structure for on-chip LDO regulators.
 *
 * Configures the TMC9660's internal LDO (Low Dropout) regulators
 * that provide power to external components. These regulators can
 * be configured for different voltage levels and power-up characteristics.
 */
struct LDOConfig {
  bootcfg::LDOVoltage vext1{bootcfg::LDOVoltage::Disabled};   ///< VEXT1 output voltage setting
  bootcfg::LDOVoltage vext2{bootcfg::LDOVoltage::Disabled};   ///< VEXT2 output voltage setting
  bootcfg::LDOSlope slope_vext1{bootcfg::LDOSlope::Slope3ms}; ///< VEXT1 power-up slope control
  bootcfg::LDOSlope slope_vext2{bootcfg::LDOSlope::Slope3ms}; ///< VEXT2 power-up slope control
  bool ldo_short_fault{false}; ///< Enable LDO short-circuit fault detection
};

/**
 * @brief Bootloader behavior configuration structure.
 *
 * Controls the bootloader's behavior including boot mode selection,
 * fault handling, self-test options, and motor control startup.
 */
struct BootConfig {
  bootcfg::BootMode boot_mode{
      bootcfg::BootMode::Parameter}; ///< Target boot mode (Register or Parameter)
  bool bl_ready_fault{false};        ///< Enable bootloader ready fault detection
  bool bl_exit_fault{true};          ///< Enable bootloader exit fault detection
  bool disable_selftest{false};      ///< Disable power-on self-test
  bool bl_config_fault{false};       ///< Enable bootloader configuration fault detection
  bool start_motor_control{false};   ///< Start motor control after configuration (CRITICAL!)
};

/**
 * @brief UART communication configuration structure.
 *
 * Configures UART communication parameters for bootloader operation
 * including addresses, pin assignments, and baud rate settings.
 */
struct UARTConfig {
  uint8_t device_address{1};                            ///< Device address for UART communication
  uint8_t host_address{255};                            ///< Host address for UART communication
  bool disable_uart{false};                             ///< Disable UART interface
  bootcfg::UartRxPin rx_pin{bootcfg::UartRxPin::GPIO7}; ///< UART receive pin selection
  bootcfg::UartTxPin tx_pin{bootcfg::UartTxPin::GPIO6}; ///< UART transmit pin selection
  bootcfg::BaudRate baud_rate{bootcfg::BaudRate::BR115200}; ///< UART baud rate setting
};

/**
 * @brief RS485 transceiver configuration structure.
 *
 * Configures RS485 half-duplex communication including transmit enable
 * pin selection and timing delays for proper transceiver control.
 */
struct RS485Config {
  bool enable_rs485{false}; ///< Enable RS485 half-duplex mode
  bootcfg::RS485TxEnPin txen_pin{
      bootcfg::RS485TxEnPin::None}; ///< RS485 transmit enable pin selection
  uint8_t txen_pre_delay{0};        ///< Pre-transmission delay (microseconds)
  uint8_t txen_post_delay{0};       ///< Post-transmission delay (microseconds)
};

/**
 * @brief SPI interface configuration for bootloader commands.
 *
 * Configures the SPI interface used for bootloader communication
 * including interface selection and pin assignments.
 */
struct SPIBootConfig {
  bool disable_spi{false};                                           ///< Disable SPI interface
  bootcfg::SPIInterface boot_spi_iface{bootcfg::SPIInterface::SPI0}; ///< SPI interface selection
  bootcfg::SPI0SckPin spi0_sck_pin{bootcfg::SPI0SckPin::GPIO11};     ///< SPI0 clock pin selection
};

/**
 * @brief External SPI flash memory configuration structure.
 *
 * Configures external SPI flash memory interface including
 * enable settings, interface selection, and communication parameters.
 */
struct SPIFlashConfig {
  bool enable_flash{false}; ///< Enable external SPI flash interface
  bootcfg::SPIInterface flash_spi_iface{bootcfg::SPIInterface::SPI1}; ///< SPI interface for flash
  bootcfg::SPI0SckPin spi0_sck_pin{bootcfg::SPI0SckPin::GPIO6};       ///< SPI0 clock pin for flash
  uint8_t cs_pin{0};                                                  ///< Chip select pin for flash
  bootcfg::SPIFlashFreq freq_div{bootcfg::SPIFlashFreq::Div4}; ///< SPI clock frequency divider
};

/**
 * @brief External I2C EEPROM configuration structure.
 *
 * Configures external I2C EEPROM interface including
 * enable settings, pin assignments, and communication parameters.
 */
struct I2CConfig {
  bool enable_eeprom{false};                              ///< Enable external I2C EEPROM interface
  bootcfg::I2CSdaPin sda_pin{bootcfg::I2CSdaPin::GPIO5};  ///< I2C data pin selection
  bootcfg::I2CSclPin scl_pin{bootcfg::I2CSclPin::GPIO4};  ///< I2C clock pin selection
  uint8_t address_bits{0};                                ///< I2C address bit configuration
  bootcfg::I2CFreq freq_code{bootcfg::I2CFreq::Freq100k}; ///< I2C communication frequency
};

/**
 * @brief System clock configuration structure.
 *
 * Configures the TMC9660's system clock including source selection,
 * PLL settings, and frequency dividers for optimal performance.
 */
struct ClockConfig {
  bootcfg::ClockSource use_external{
      bootcfg::ClockSource::Internal}; ///< Use external or internal clock source
  bootcfg::ExtSourceType ext_source_type{
      bootcfg::ExtSourceType::Oscillator}; ///< External source type (crystal/clock)
  bootcfg::XtalDrive xtal_drive{bootcfg::XtalDrive::Freq16MHz}; ///< Crystal drive strength
  bool xtal_boost{false};                                       ///< Enable crystal boost mode
  bootcfg::SysClkSource pll_selection{
      bootcfg::SysClkSource::PLL};                         ///< System clock source (oscillator/PLL)
  uint8_t rdiv{14};                                        ///< PLL reference divider
  bootcfg::SysClkDiv sysclk_div{bootcfg::SysClkDiv::Div1}; ///< System clock divider
};

/**
 * @brief GPIO configuration structure for bootloader operation.
 *
 * Configures the initial state of all GPIO pins during bootloader operation
 * including output levels, directions, pull settings, and analog mode.
 */
struct GPIOConfig {
  uint16_t outputMask_0_15{0};    ///< GPIO output levels for GPIOs 0-15 (16-bit mask)
  uint8_t outputMask_16_18{0};    ///< GPIO output levels for GPIOs 16-18 (3-bit mask)
  uint16_t directionMask_0_15{0}; ///< GPIO direction (output enable) for GPIOs 0-15 (16-bit mask)
  uint8_t directionMask_16_18{0}; ///< GPIO direction (output enable) for GPIOs 16-18 (3-bit mask)
  uint16_t pullUpMask_0_15{0};    ///< GPIO pull-up enable for GPIOs 0-15 (16-bit mask)
  uint8_t pullUpMask_16_18{0};    ///< GPIO pull-up enable for GPIOs 16-18 (3-bit mask)
  uint16_t pullDownMask_0_15{0};  ///< GPIO pull-down enable for GPIOs 0-15 (16-bit mask)
  uint8_t pullDownMask_16_18{0};  ///< GPIO pull-down enable for GPIOs 16-18 (3-bit mask)
  uint8_t analogMask_2_5{0};      ///< GPIO analog enable for GPIOs 2-5 (4-bit mask)
};

/**
 * @brief Hall encoder configuration for BLDC motor feedback.
 *
 * Configures Hall sensor pins and enable settings for BLDC motor feedback.
 * Hall sensors provide 60-degree position information for proper motor commutation.
 */
struct HallConfig {
  bool enable{false};
  bootcfg::HallUPin u_pin{bootcfg::HallUPin::GPIO2};
  bootcfg::HallVPin v_pin{bootcfg::HallVPin::GPIO3};
  bootcfg::HallWPin w_pin{bootcfg::HallWPin::GPIO4};
};

/**
 * @brief ABN encoder 1 configuration for incremental encoder feedback.
 *
 * Configures ABN encoder 1 pins and enable settings for incremental encoder feedback.
 * ABN encoders provide high-resolution position and velocity feedback for precise motor control.
 */
struct ABN1Config {
  bool enable{false};
  bootcfg::ABN1APin a_pin{bootcfg::ABN1APin::GPIO5};
  bootcfg::ABN1BPin b_pin{bootcfg::ABN1BPin::GPIO1};
  bootcfg::ABN1NPin n_pin{bootcfg::ABN1NPin::Disabled};
};

/**
 * @brief ABN encoder 2 configuration for second incremental encoder.
 *
 * Configures ABN encoder 2 pins and enable settings for dual-encoder applications
 * or secondary position feedback. Enables applications requiring redundancy
 * or separate position measurement systems.
 */
struct ABN2Config {
  bool enable{false};
  bootcfg::ABN2APin a_pin{bootcfg::ABN2APin::GPIO6};
  bootcfg::ABN2BPin b_pin{bootcfg::ABN2BPin::GPIO7};
};

/**
 * @brief Reference switches configuration for limit detection and homing.
 *
 * Configures reference switch pins for detecting mechanical limits and home position.
 * Left and right switches detect travel limits, while the home switch provides
 * a reference position for absolute positioning.
 */
struct RefConfig {
  bootcfg::RefLPin ref_l_pin{bootcfg::RefLPin::Disabled};
  bootcfg::RefRPin ref_r_pin{bootcfg::RefRPin::Disabled};
  bootcfg::RefHPin ref_h_pin{bootcfg::RefHPin::Disabled};
};

/**
 * @brief Step/Direction interface configuration for stepper motor control.
 *
 * Configures step and direction pins for controlling stepper motors or providing
 * external step/direction signals. The step pin generates pulses for position steps,
 * while the direction pin controls the movement direction.
 */
struct StepDirConfig {
  bool enable{false};
  bootcfg::StepPin step_pin{bootcfg::StepPin::GPIO7};
  bootcfg::DirPin dir_pin{bootcfg::DirPin::GPIO6};
};

/**
 * @brief SPI encoder configuration for SPI-based position sensors.
 *
 * Configures SPI encoder interface including block selection, communication mode,
 * frequency, chip select settings, and polarity. Used for communicating with
 * external SPI-based encoders or position sensors.
 */
struct SPIEncConfig {
  bool enable{false};
  bootcfg::SPIEncBlock spi_block{bootcfg::SPIEncBlock::SPI0};
  bootcfg::SPIEncMode spi_mode{bootcfg::SPIEncMode::Mode0};
  bootcfg::SPIEncFreq spi_freq{bootcfg::SPIEncFreq::Div4};
  bootcfg::SPIEncCSPin cs_pin{bootcfg::SPIEncCSPin::GPIO8};
  bootcfg::SPIEncCSPol cs_polarity{bootcfg::SPIEncCSPol::ActiveLow};
};

/**
 * @brief Mechanical brake configuration for holding motor position.
 *
 * Configures mechanical brake output pin and enable settings. Mechanical brakes
 * hold motor position when power is removed, commonly used in servo applications
 * where the motor must maintain position after power loss.
 */
struct MechBrakeConfig {
  bool enable{false};
  bootcfg::MechBrakeOutput output_pin{bootcfg::MechBrakeOutput::GPIO8};
};

/**
 * @brief Brake chopper configuration for dynamic braking.
 *
 * Configures brake chopper output pin and enable settings for dynamic braking.
 * The brake chopper allows rapid motor deceleration by shorting the motor windings,
 * providing faster stops than passive braking alone.
 */
struct BrakeChopperConfig {
  bool enable{false};
  bootcfg::BrakeChopperOutput output_pin{bootcfg::BrakeChopperOutput::GPIO0};
};

/**
 * @brief External memory storage configuration for TMCL scripts and parameters.
 *
 * Configures which external memory types are used for storing TMCL scripts
 * and parameter data. The bootloader can store TMCL application code and
 * configuration parameters in external SPI flash or I2C EEPROM for persistence.
 */
struct MemStorageConfig {
  bootcfg::MemStorage tmcl_script{bootcfg::MemStorage::Disabled};
  bootcfg::MemStorage parameters{bootcfg::MemStorage::Disabled};
};

/**
 * @brief Complete bootloader configuration structure.
 *
 * This structure aggregates all bootloader configuration settings into a single
 * object for convenient configuration. Use ::TMC9660Bootloader::applyConfiguration()
 * to write these settings to the TMC9660 bootloader registers.
 */
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
  HallConfig hall;
  ABN1Config abn1;
  ABN2Config abn2;
  RefConfig ref;
  StepDirConfig stepDir;
  SPIEncConfig spiEnc;
  MechBrakeConfig mechBrake;
  BrakeChopperConfig brakeChopper;
  MemStorageConfig memStorage;
};

} // namespace tmc9660
