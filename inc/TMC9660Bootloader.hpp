/**
 * @file TMC9660Bootloader.hpp
 * @brief TMC9660 bootloader interface and configuration structures.
 * 
 * This file provides comprehensive bootloader functionality for the TMC9660
 * including memory operations, configuration management, OTP programming,
 * and system initialization. The bootloader is essential for configuring
 * the TMC9660 before transitioning to parameter mode operation.
 * 
 * @defgroup TMC9660_Bootloader Bootloader Interface
 * @brief Core bootloader functionality and memory operations
 * 
 * @defgroup TMC9660_BootloaderConfig Configuration Structures
 * @brief Bootloader configuration data structures and enums
 * 
 * @defgroup TMC9660_BootloaderProtocol Communication Protocol
 * @brief SPI/UART protocol structures and command definitions
 * 
 * @defgroup TMC9660_BootloaderUtils Utility Functions
 * @brief Helper functions for bootloader operations
 */

#pragma once

#include "TMC9660CommInterface.hpp"
#include <cstddef>
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
  Disabled = 0,  ///< LDO output disabled
  V2_5 = 1,      ///< 2.5V output voltage
  V3_3 = 2,      ///< 3.3V output voltage  
  V5_0 = 3,      ///< 5.0V output voltage
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
  GPIO7 = 0,  ///< Use GPIO7 for UART RX (default)
  GPIO1 = 1   ///< Use GPIO1 for UART RX (alternative)
};

/**
 * @brief UART transmit pin selection for bootloader communication.
 * 
 * Selects which GPIO pin is used for UART TX signal during bootloader mode.
 */
enum class UartTxPin : uint8_t { 
  GPIO6 = 0,  ///< Use GPIO6 for UART TX (default)
  GPIO0 = 1   ///< Use GPIO0 for UART TX (alternative)
};

/**
 * @brief UART baud rate configuration for bootloader communication.
 * 
 * Configures the UART communication speed. Auto modes detect the baud rate
 * automatically by analyzing the received data pattern.
 */
enum class BaudRate : uint8_t {
  BR9600 = 0,    ///< 9600 baud (slowest, most reliable)
  BR19200,       ///< 19200 baud
  BR38400,       ///< 38400 baud
  BR57600,       ///< 57600 baud
  BR115200,      ///< 115200 baud (recommended for most applications)
  BR1000000,     ///< 1000000 baud (1 Mbps, fastest)
  Auto8x,        ///< Auto-detect with 8x oversampling
  Auto16x,       ///< Auto-detect with 16x oversampling (most robust)
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
 * @brief SPI interface selection for bootloader communication.
 * 
 * The TMC9660 supports two SPI interfaces. IFACE0 is typically used for
 * bootloader communication, while IFACE1 can be used for external flash.
 */
enum class SPIInterface : uint8_t { 
  IFACE0 = 0,  ///< SPI interface 0 (primary bootloader interface)
  IFACE1 = 1   ///< SPI interface 1 (secondary interface, often for flash)
};

/**
 * @brief SPI0 clock pin selection for bootloader communication.
 * 
 * Selects which GPIO pin is used for SPI0 SCK (serial clock) signal.
 * This affects the bootloader communication interface.
 */
enum class SPI0SckPin : uint8_t { 
  GPIO6 = 0,  ///< Use GPIO6 for SPI0 SCK (default)
  GPIO11 = 1  ///< Use GPIO11 for SPI0 SCK (alternative)
};

/**
 * @brief SPI flash frequency divider configuration.
 * 
 * Controls the SPI clock frequency for external flash communication.
 * Higher divider values result in slower, more reliable communication.
 */
enum class SPIFlashFreq : uint8_t { 
  Div1 = 0,  ///< No division (fastest, may be unreliable)
  Div2 = 1,  ///< Divide by 2 (medium speed)
  Div4 = 3   ///< Divide by 4 (slowest, most reliable)
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
  Freq100k = 0,  ///< 100 kHz (standard mode, most compatible)
  Freq200k,      ///< 200 kHz (fast mode)
  Freq400k,      ///< 400 kHz (fast mode plus)
  Freq800k       ///< 800 kHz (high speed mode, fastest)
};

/**
 * @brief System clock source selection.
 * 
 * Determines whether the TMC9660 uses its internal oscillator or an external
 * clock source for system timing.
 */
enum class ClockSource : uint8_t { 
  Internal = 0,  ///< Use internal oscillator (default, no external components needed)
  External = 1   ///< Use external clock source (requires external crystal/oscillator)
};

/**
 * @brief External clock source type configuration.
 * 
 * Specifies the type of external clock source when ClockSource::External is selected.
 * This affects the input buffer configuration and clock processing.
 */
enum class ExtSourceType : uint8_t { 
  Oscillator = 0,  ///< External crystal oscillator (requires external crystal)
  Clock = 1        ///< External clock signal (digital clock input)
};

/**
 * @brief Crystal drive strength configuration for external oscillators.
 * 
 * Configures the drive strength of the internal oscillator when using
 * an external crystal. Higher drive strength is needed for higher frequency crystals.
 */
enum class XtalDrive : uint8_t {
  Freq8MHz = 1,   ///< 8 MHz crystal drive strength
  Freq16MHz = 3,  ///< 16 MHz crystal drive strength (recommended)
  Freq24MHz = 5,  ///< 24 MHz crystal drive strength
  Freq32MHz = 6,  ///< 32 MHz crystal drive strength (highest frequency)
};

/**
 * @brief System clock source selection after initial oscillator.
 * 
 * Determines whether the system clock is derived directly from the oscillator
 * or through the PLL (Phase-Locked Loop) for frequency multiplication.
 */
enum class SysClkSource : uint8_t { 
  IntOsc = 0,  ///< Use internal oscillator directly (lower power, fixed frequency)
  PLL = 1      ///< Use PLL for frequency multiplication (higher performance, configurable)
};

/**
 * @brief System clock divider configuration.
 * 
 * Divides the system clock to achieve the desired operating frequency.
 * Lower divider values result in higher system clock frequencies.
 */
enum class SysClkDiv : uint8_t { 
  Div1 = 0,      ///< No division (40 MHz system clock)
  Div15MHz = 3   ///< Divide by 3 (15 MHz system clock, lower power)
};

/**
 * @brief Hall encoder U-phase pin selection for BLDC motor feedback.
 * 
 * Selects which GPIO pin is used for the Hall sensor U-phase signal.
 * Used for BLDC motor commutation and position feedback.
 */
enum class HallUPin : uint8_t { 
  GPIO2 = 0,  ///< Use GPIO2 for Hall U-phase (default)
  GPIO7 = 1,  ///< Use GPIO7 for Hall U-phase
  GPIO9 = 2   ///< Use GPIO9 for Hall U-phase
};

/**
 * @brief Hall encoder V-phase pin selection for BLDC motor feedback.
 * 
 * Selects which GPIO pin is used for the Hall sensor V-phase signal.
 * Used for BLDC motor commutation and position feedback.
 */
enum class HallVPin : uint8_t { 
  GPIO3 = 0,   ///< Use GPIO3 for Hall V-phase (default)
  GPIO15 = 1   ///< Use GPIO15 for Hall V-phase
};

/**
 * @brief Hall encoder W-phase pin selection for BLDC motor feedback.
 * 
 * Selects which GPIO pin is used for the Hall sensor W-phase signal.
 * Used for BLDC motor commutation and position feedback.
 */
enum class HallWPin : uint8_t { 
  GPIO4 = 0,   ///< Use GPIO4 for Hall W-phase (default)
  GPIO8 = 1,   ///< Use GPIO8 for Hall W-phase
  GPIO10 = 2   ///< Use GPIO10 for Hall W-phase
};

/**
 * @brief ABN encoder 1 A-phase pin selection for incremental encoder feedback.
 * 
 * Selects which GPIO pin is used for the ABN encoder 1 A-phase signal.
 * Used for precise position and velocity feedback from incremental encoders.
 */
enum class ABN1APin : uint8_t { 
  GPIO5 = 0,   ///< Use GPIO5 for ABN1 A-phase (default)
  GPIO8 = 1,   ///< Use GPIO8 for ABN1 A-phase
  GPIO17 = 2   ///< Use GPIO17 for ABN1 A-phase
};

/**
 * @brief ABN encoder 1 B-phase pin selection for incremental encoder feedback.
 * 
 * Selects which GPIO pin is used for the ABN encoder 1 B-phase signal.
 * Used for precise position and velocity feedback from incremental encoders.
 */
enum class ABN1BPin : uint8_t { 
  GPIO1 = 0,   ///< Use GPIO1 for ABN1 B-phase (default)
  GPIO13 = 1,  ///< Use GPIO13 for ABN1 B-phase
  GPIO18 = 2   ///< Use GPIO18 for ABN1 B-phase
};

/**
 * @brief ABN encoder 1 index pin selection for incremental encoder feedback.
 * 
 * Selects which GPIO pin is used for the ABN encoder 1 index (Z) signal.
 * The index signal provides a reference position for absolute positioning.
 */
enum class ABN1NPin : uint8_t { 
  Disabled = 0,  ///< No index pin (index signal not used)
  GPIO14 = 1,    ///< Use GPIO14 for ABN1 index signal
  GPIO16 = 2     ///< Use GPIO16 for ABN1 index signal
};

/**
 * @brief ABN encoder 2 A-phase pin selection for second incremental encoder.
 * 
 * Selects which GPIO pin is used for the ABN encoder 2 A-phase signal.
 * Used for dual-encoder applications or secondary position feedback.
 */
enum class ABN2APin : uint8_t { 
  GPIO6 = 0,   ///< Use GPIO6 for ABN2 A-phase (default)
  GPIO15 = 1   ///< Use GPIO15 for ABN2 A-phase
};

/**
 * @brief ABN encoder 2 B-phase pin selection for second incremental encoder.
 * 
 * Selects which GPIO pin is used for the ABN encoder 2 B-phase signal.
 * Used for dual-encoder applications or secondary position feedback.
 */
enum class ABN2BPin : uint8_t { 
  GPIO7 = 0,   ///< Use GPIO7 for ABN2 B-phase (default)
  GPIO11 = 1,  ///< Use GPIO11 for ABN2 B-phase
  GPIO16 = 2   ///< Use GPIO16 for ABN2 B-phase
};

/**
 * @brief Reference switch left pin selection for limit switch detection.
 * 
 * Selects which GPIO pin is used for the left reference switch signal.
 * Used for detecting mechanical limits and home position.
 */
enum class RefLPin : uint8_t { 
  Disabled = 0,  ///< No left reference switch (disabled)
  GPIO2 = 1,     ///< Use GPIO2 for left reference switch
  GPIO12 = 2,    ///< Use GPIO12 for left reference switch
  GPIO16 = 3     ///< Use GPIO16 for left reference switch
};

/**
 * @brief Reference switch right pin selection for limit switch detection.
 * 
 * Selects which GPIO pin is used for the right reference switch signal.
 * Used for detecting mechanical limits and home position.
 */
enum class RefRPin : uint8_t { 
  Disabled = 0,  ///< No right reference switch (disabled)
  GPIO3 = 1,     ///< Use GPIO3 for right reference switch
  GPIO18 = 2     ///< Use GPIO18 for right reference switch
};

/**
 * @brief Reference switch home pin selection for home position detection.
 * 
 * Selects which GPIO pin is used for the home reference switch signal.
 * Used for detecting the home position during homing sequences.
 */
enum class RefHPin : uint8_t { 
  Disabled = 0,  ///< No home reference switch (disabled)
  GPIO4 = 1,     ///< Use GPIO4 for home reference switch
  GPIO7 = 2,     ///< Use GPIO7 for home reference switch
  GPIO15 = 3,    ///< Use GPIO15 for home reference switch
  GPIO17 = 4     ///< Use GPIO17 for home reference switch
};

/**
 * @brief Step pin selection for step/direction interface.
 * 
 * Selects which GPIO pin is used for the step signal in step/direction mode.
 * Used for controlling stepper motors or providing step pulses to external controllers.
 */
enum class StepPin : uint8_t { 
  GPIO7 = 0,   ///< Use GPIO7 for step signal (default)
  GPIO11 = 1,  ///< Use GPIO11 for step signal
  GPIO16 = 2   ///< Use GPIO16 for step signal
};

/**
 * @brief Direction pin selection for step/direction interface.
 * 
 * Selects which GPIO pin is used for the direction signal in step/direction mode.
 * Used for controlling stepper motors or providing direction signals to external controllers.
 */
enum class DirPin : uint8_t { 
  GPIO6 = 0,   ///< Use GPIO6 for direction signal (default)
  GPIO15 = 1   ///< Use GPIO15 for direction signal
};

/**
 * @brief SPI encoder interface block selection.
 * 
 * Selects which SPI interface is used for communication with external
 * SPI-based encoders or position sensors.
 */
enum class SPIEncBlock : uint8_t { 
  SPI0 = 0,  ///< Use SPI interface 0 for encoder communication
  SPI1 = 1   ///< Use SPI interface 1 for encoder communication
};

/**
 * @brief SPI encoder communication mode configuration.
 * 
 * Configures the SPI mode for encoder communication. Different encoders
 * may require different SPI modes for proper communication.
 */
enum class SPIEncMode : uint8_t { 
  Mode0 = 0,  ///< SPI Mode 0 (CPOL=0, CPHA=0)
  Mode1 = 1,  ///< SPI Mode 1 (CPOL=0, CPHA=1)
  Mode2 = 2,  ///< SPI Mode 2 (CPOL=1, CPHA=0)
  Mode3 = 3   ///< SPI Mode 3 (CPOL=1, CPHA=1)
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
  GPIO8 = 0,   ///< Use GPIO8 for SPI0 encoder CS
  GPIO12 = 1,  ///< Use GPIO12 for SPI0 encoder CS
  GPIO13 = 2,  ///< Use GPIO13 for SPI0 encoder CS
  GPIO16 = 3,  ///< Use GPIO16 for SPI0 encoder CS
  // SPI1 options (same enum values, context determines which)
  GPIO15_SPI1 = 0  ///< Use GPIO15 for SPI1 encoder CS
};

/**
 * @brief SPI encoder chip select polarity configuration.
 * 
 * Configures the active level of the chip select signal for SPI encoder communication.
 * Most SPI devices use active-low chip select.
 */
enum class SPIEncCSPol : uint8_t { 
  ActiveHigh = 0,  ///< Chip select active high (CS high = selected)
  ActiveLow = 1    ///< Chip select active low (CS low = selected, most common)
};

/**
 * @brief Mechanical brake output pin selection.
 * 
 * Selects which output is used to control an external mechanical brake.
 * Used for holding motor position when power is removed.
 */
enum class MechBrakeOutput : uint8_t { 
  GPIO8 = 0,   ///< Use GPIO8 for mechanical brake control
  GPIO10 = 1,  ///< Use GPIO10 for mechanical brake control
  GPIO18 = 2,  ///< Use GPIO18 for mechanical brake control
  Y2_LS = 3    ///< Use Y2_LS (low-side output) for mechanical brake control
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
  Disabled = 0,   ///< No external memory (disabled)
  SPIFlash = 1,   ///< Use SPI flash memory for storage
  I2CEEPROM = 2   ///< Use I2C EEPROM for storage
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
  bootcfg::LDOVoltage vext1{bootcfg::LDOVoltage::Disabled};      ///< VEXT1 output voltage setting
  bootcfg::LDOVoltage vext2{bootcfg::LDOVoltage::Disabled};      ///< VEXT2 output voltage setting
  bootcfg::LDOSlope slope_vext1{bootcfg::LDOSlope::Slope3ms};    ///< VEXT1 power-up slope control
  bootcfg::LDOSlope slope_vext2{bootcfg::LDOSlope::Slope3ms};    ///< VEXT2 power-up slope control
  bool ldo_short_fault{false};                                    ///< Enable LDO short-circuit fault detection
};

/**
 * @brief Bootloader behavior configuration structure.
 * 
 * Controls the bootloader's behavior including boot mode selection,
 * fault handling, self-test options, and motor control startup.
 */
struct BootConfig {
  bootcfg::BootMode boot_mode{bootcfg::BootMode::Parameter};  ///< Target boot mode (Register or Parameter)
  bool bl_ready_fault{false};                                 ///< Enable bootloader ready fault detection
  bool bl_exit_fault{true};                                   ///< Enable bootloader exit fault detection
  bool disable_selftest{false};                               ///< Disable power-on self-test
  bool bl_config_fault{false};                                ///< Enable bootloader configuration fault detection
  bool start_motor_control{false};                            ///< Start motor control after configuration (CRITICAL!)
};

/**
 * @brief UART communication configuration structure.
 * 
 * Configures UART communication parameters for bootloader operation
 * including addresses, pin assignments, and baud rate settings.
 */
struct UARTConfig {
  uint8_t device_address{1};                                    ///< Device address for UART communication
  uint8_t host_address{255};                                    ///< Host address for UART communication
  bool disable_uart{false};                                     ///< Disable UART interface
  bootcfg::UartRxPin rx_pin{bootcfg::UartRxPin::GPIO7};         ///< UART receive pin selection
  bootcfg::UartTxPin tx_pin{bootcfg::UartTxPin::GPIO6};         ///< UART transmit pin selection
  bootcfg::BaudRate baud_rate{bootcfg::BaudRate::BR115200};     ///< UART baud rate setting
};

/**
 * @brief RS485 transceiver configuration structure.
 * 
 * Configures RS485 half-duplex communication including transmit enable
 * pin selection and timing delays for proper transceiver control.
 */
struct RS485Config {
  bool enable_rs485{false};                                     ///< Enable RS485 half-duplex mode
  bootcfg::RS485TxEnPin txen_pin{bootcfg::RS485TxEnPin::None};  ///< RS485 transmit enable pin selection
  uint8_t txen_pre_delay{0};                                    ///< Pre-transmission delay (microseconds)
  uint8_t txen_post_delay{0};                                   ///< Post-transmission delay (microseconds)
};

/**
 * @brief SPI interface configuration for bootloader commands.
 * 
 * Configures the SPI interface used for bootloader communication
 * including interface selection and pin assignments.
 */
struct SPIBootConfig {
  bool disable_spi{false};                                      ///< Disable SPI interface
  bootcfg::SPIInterface boot_spi_iface{bootcfg::SPIInterface::IFACE0};  ///< SPI interface selection
  bootcfg::SPI0SckPin spi0_sck_pin{bootcfg::SPI0SckPin::GPIO11};       ///< SPI0 clock pin selection
};

/**
 * @brief External SPI flash memory configuration structure.
 * 
 * Configures external SPI flash memory interface including
 * enable settings, interface selection, and communication parameters.
 */
struct SPIFlashConfig {
  bool enable_flash{false};                                     ///< Enable external SPI flash interface
  bootcfg::SPIInterface flash_spi_iface{bootcfg::SPIInterface::IFACE1};  ///< SPI interface for flash
  bootcfg::SPI0SckPin spi0_sck_pin{bootcfg::SPI0SckPin::GPIO6};         ///< SPI0 clock pin for flash
  uint8_t cs_pin{0};                                            ///< Chip select pin for flash
  bootcfg::SPIFlashFreq freq_div{bootcfg::SPIFlashFreq::Div4};  ///< SPI clock frequency divider
};

/**
 * @brief External I2C EEPROM configuration structure.
 * 
 * Configures external I2C EEPROM interface including
 * enable settings, pin assignments, and communication parameters.
 */
struct I2CConfig {
  bool enable_eeprom{false};                                    ///< Enable external I2C EEPROM interface
  bootcfg::I2CSdaPin sda_pin{bootcfg::I2CSdaPin::GPIO5};       ///< I2C data pin selection
  bootcfg::I2CSclPin scl_pin{bootcfg::I2CSclPin::GPIO4};       ///< I2C clock pin selection
  uint8_t address_bits{0};                                      ///< I2C address bit configuration
  bootcfg::I2CFreq freq_code{bootcfg::I2CFreq::Freq100k};      ///< I2C communication frequency
};

/**
 * @brief System clock configuration structure.
 * 
 * Configures the TMC9660's system clock including source selection,
 * PLL settings, and frequency dividers for optimal performance.
 */
struct ClockConfig {
  bootcfg::ClockSource use_external{bootcfg::ClockSource::Internal};        ///< Use external or internal clock source
  bootcfg::ExtSourceType ext_source_type{bootcfg::ExtSourceType::Oscillator}; ///< External source type (crystal/clock)
  bootcfg::XtalDrive xtal_drive{bootcfg::XtalDrive::Freq16MHz};             ///< Crystal drive strength
  bool xtal_boost{false};                                                   ///< Enable crystal boost mode
  bootcfg::SysClkSource pll_selection{bootcfg::SysClkSource::PLL};          ///< System clock source (oscillator/PLL)
  uint8_t rdiv{14};                                                         ///< PLL reference divider
  bootcfg::SysClkDiv sysclk_div{bootcfg::SysClkDiv::Div1};                  ///< System clock divider
};

/**
 * @brief GPIO configuration structure for bootloader operation.
 * 
 * Configures the initial state of all GPIO pins during bootloader operation
 * including output levels, directions, pull settings, and analog mode.
 */
struct GPIOConfig {
  uint16_t outputMask_0_15{0};      ///< GPIO output levels for GPIOs 0-15 (16-bit mask)
  uint8_t outputMask_16_18{0};      ///< GPIO output levels for GPIOs 16-18 (3-bit mask)
  uint16_t directionMask_0_15{0};   ///< GPIO direction (output enable) for GPIOs 0-15 (16-bit mask)
  uint8_t directionMask_16_18{0};   ///< GPIO direction (output enable) for GPIOs 16-18 (3-bit mask)
  uint16_t pullUpMask_0_15{0};      ///< GPIO pull-up enable for GPIOs 0-15 (16-bit mask)
  uint8_t pullUpMask_16_18{0};      ///< GPIO pull-up enable for GPIOs 16-18 (3-bit mask)
  uint16_t pullDownMask_0_15{0};    ///< GPIO pull-down enable for GPIOs 0-15 (16-bit mask)
  uint8_t pullDownMask_16_18{0};    ///< GPIO pull-down enable for GPIOs 16-18 (3-bit mask)
  uint8_t analogMask_2_5{0};        ///< GPIO analog enable for GPIOs 2-5 (4-bit mask)
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

/**
 * @brief Bootloader status codes returned by command replies.
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
 * @brief Helper function for CRC-8 calculation (UART only).
 * 
 * These functions are used by the UART bootloader protocol for checksum
 * calculation. They are automatically called by the ::BootloaderCommandUART
 * serialization methods.
 */

/**
 * @brief Bit-reverse a byte (LSB ↔ MSB).
 * 
 * Helper function used by the bootloader CRC-8 calculation to reverse
 * the bit order of each input byte before polynomial division.
 */
static constexpr uint8_t reverseByte(uint8_t b) noexcept {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

/**
 * @brief CRC-8 calculation for UART protocol (TMC9660 datasheet method).
 */
/// Polynomial: x^8 + x^2 + x^1 + x^0 (9-bit: 0b100000111)
/// Algorithm: Bit-reverse each input byte, perform polynomial division, return MSB-first result
/// Note: This is NOT a standard CRC-8! Each byte is bit-reversed before processing.
static constexpr uint8_t crc8Bootloader(const uint8_t* data, size_t len) noexcept {
  uint16_t crc = 0;  // 9-bit register for polynomial division
  const uint16_t POLY = 0x107;  // x^8 + x^2 + x^1 + x^0 in 9-bit form
  
  // Process each byte (bit-reversed, LSB-first)
  for (size_t i = 0; i < len; i++) {
    uint8_t byte_reversed = reverseByte(data[i]);
    
    // Feed 8 bits into CRC register (MSB-first from reversed byte)
    for (int bit = 7; bit >= 0; bit--) {
      crc = (crc << 1) | ((byte_reversed >> bit) & 1);
      if (crc & 0x100) {  // If bit 8 is set
        crc ^= POLY;
      }
    }
  }
  
  // Append 8 zero bits (flush remaining data through CRC)
  for (int i = 0; i < 8; i++) {
    crc <<= 1;
    if (crc & 0x100) {
      crc ^= POLY;
    }
  }
  
  return static_cast<uint8_t>(crc & 0xFF);
}

/**
 * @brief Bootloader command structure for SPI (40-bit / 5-byte protocol).
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
  explicit TMC9660Bootloader(TMC9660CommInterface &comm) noexcept;

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
  bool write32IncMultiple(const uint32_t *values, size_t count) noexcept;
  
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
  bool read8(uint8_t *value) noexcept;
  
  /**
   * @brief Read a 16-bit word from the previously selected address.
   * 
   * Reads a 16-bit value from the current bank and address without 
   * modifying the address pointer. Little-endian byte order.
   * 
   * @param value Pointer to receive the 16-bit value read
   * @return true if read successful
   */
  bool read16(uint16_t *value) noexcept;
  
  /**
   * @brief Read a 32-bit word from the previously selected address.
   * 
   * Reads a 32-bit value from the current bank and address without 
   * modifying the address pointer. Little-endian byte order.
   * 
   * @param value Pointer to receive the 32-bit value read
   * @return true if read successful
   */
  bool read32(uint32_t *value) noexcept;
  
  /**
   * @brief Read a single byte and increment address by 1.
   * 
   * Reads an 8-bit value and automatically increments the address pointer.
   * Useful for reading sequential data arrays.
   * 
   * @param value Pointer to receive the 8-bit value read
   * @return true if read successful
   */
  bool read8Inc(uint8_t *value) noexcept;
  
  /**
   * @brief Read a 16-bit word and increment address by 2.
   * 
   * Reads a 16-bit value and automatically increments the address pointer by 2.
   * Useful for reading sequential data arrays.
   * 
   * @param value Pointer to receive the 16-bit value read
   * @return true if read successful
   */
  bool read16Inc(uint16_t *value) noexcept;
  
  /**
   * @brief Read a 32-bit word and increment address by 4.
   * 
   * Reads a 32-bit value and automatically increments the address pointer by 4.
   * Commonly used for reading configuration registers and data arrays.
   * 
   * @param value Pointer to receive the 32-bit value read
   * @return true if read successful
   */
  bool read32Inc(uint32_t *value) noexcept;
  
  /**
   * @brief Get the currently selected memory bank.
   * 
   * Retrieves the memory bank number that was last selected via ::setBank().
   * Useful for verifying the current bank configuration.
   * 
   * @param bank Pointer to receive the current bank number (0-5)
   * @return true if query successful
   */
  bool getBank(uint8_t *bank) noexcept;
  
  /**
   * @brief Get the currently selected memory address.
   * 
   * Retrieves the memory address that was last set via ::setAddress() within
   * the currently selected bank. Useful for tracking address pointer state.
   * 
   * @param address Pointer to receive the current address
   * @return true if query successful
   */
  bool getAddress(uint32_t *address) noexcept;
  
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
  bool noOp(uint32_t *reply = nullptr) noexcept;
  
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
  bool otpLoad(uint8_t page, OtpLoadResult *result) noexcept;
  
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
  bool otpLoad(uint8_t page, uint8_t *errorCount = nullptr, uint8_t *pageTag = nullptr) noexcept;
  
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
  bool otpBurn(uint8_t page, uint8_t pageAddr, OtpBurnResult *result) noexcept;
  
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
  bool otpBurnWithWorkaround(uint8_t page, uint8_t pageAddr, OtpBurnResult *result, 
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
  bool checkOtpBurnStatus(bool *result) noexcept;
  
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
  
  //==================================================
  // CONVENIENCE INFO METHODS
  //==================================================
  
  /// Get chip type (should return 0x544D0001 for TMC9660).
  /// @param chipType Returns chip type
  /// @return true if successful
  bool getChipType(uint32_t *chipType) noexcept {
    return getInfo(InfoQuery::CHIP_TYPE, chipType);
  }
  
  /// Get bootloader version information.
  /// @param version Returns parsed version information
  /// @return true if successful
  bool getBootloaderVersion(BootloaderVersion *version) noexcept;
  
  /// Get available feature flags.
  /// @param features Returns parsed feature flags
  /// @return true if successful
  bool getFeatures(BootloaderFeatures *features) noexcept;
  
  /// Get Git version control information.
  /// @param gitInfo Returns parsed Git information
  /// @return true if successful
  bool getGitInfo(GitInfo *gitInfo) noexcept;
  
  /// Get chip version (silicon revision).
  /// @param version Returns chip version
  /// @return true if successful
  bool getChipVersion(uint32_t *version) noexcept {
    return getInfo(InfoQuery::CHIP_VERSION, version);
  }
  
  /// Get system frequency in MHz.
  /// @param frequency Returns frequency in MHz
  /// @return true if successful
  bool getChipFrequency(uint32_t *frequency) noexcept {
    return getInfo(InfoQuery::CHIP_FREQUENCY, frequency);
  }
  
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
  
  /// Get OTP memory page size.
  /// @param size Returns OTP page size
  /// @return true if successful
  bool getOtpMemSize(uint32_t *size) noexcept {
    return getInfo(InfoQuery::OTP_MEM_SIZE, size);
  }
  
  /// Get I2C memory size.
  /// @param size Returns I2C memory size
  /// @return true if successful
  bool getI2cMemSize(uint32_t *size) noexcept {
    return getInfo(InfoQuery::I2C_MEM_SIZE, size);
  }
  
  /// Get SPI memory size.
  /// @param size Returns SPI memory size
  /// @return true if successful
  bool getSpiMemSize(uint32_t *size) noexcept {
    return getInfo(InfoQuery::SPI_MEM_SIZE, size);
  }
  
  /// Get partition version information.
  /// @param version Returns parsed partition version
  /// @return true if successful
  bool getPartitionVersion(PartitionVersion *version) noexcept;
  
  /// Get number of SPI memory partitions.
  /// @param count Returns number of partitions
  /// @return true if successful
  bool getSpiMemPartitions(uint32_t *count) noexcept {
    return getInfo(InfoQuery::SPI_MEM_PARTITIONS, count);
  }
  
  /// Get number of I2C memory partitions.
  /// @param count Returns number of partitions
  /// @return true if successful
  bool getI2cMemPartitions(uint32_t *count) noexcept {
    return getInfo(InfoQuery::I2C_MEM_PARTITIONS, count);
  }
  
  /// Get chip variant (TMC9660 reports value 2).
  /// @param variant Returns chip variant
  /// @return true if successful
  bool getChipVariant(uint32_t *variant) noexcept {
    return getInfo(InfoQuery::CHIP_VARIANT, variant);
  }
  
  /// Retrieve and log all available bootloader information.
  /// 
  /// This function queries all GET_INFO commands and logs the results.
  /// Useful for debugging and verifying chip configuration.
  /// 
  /// @return true if at least basic info was retrieved successfully
  /// @note Some queries may fail if features are not available (e.g., no SPI flash)
  /// 
  /// Information retrieved:
  /// - Chip type, version, variant
  /// - Bootloader version and Git info
  /// - System frequency
  /// - Available features (SRAM, ROM, OTP, SPI flash, I2C EEPROM)
  /// - Memory sizes and partition info
  bool getAllBootloaderInfo() noexcept;
  
  //==================================================
  // HIGH-LEVEL CONFIGURATION
  //==================================================

  /// Apply all fields of a ::BootloaderConfig.
  /// @param cfg Configuration to apply
  /// @param failOnVerifyError If true, return false on read-back verification failure;
  ///                          if false, log warning but continue
  /// @return true if successful
  /// @note If cfg.boot.start_motor_control is true, bootloader will exit after this call
  bool applyConfiguration(const BootloaderConfig &cfg, bool failOnVerifyError = true) noexcept;

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
