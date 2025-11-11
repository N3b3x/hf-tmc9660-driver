/**
 * @file tmc9660_ramdebug.hpp
 * @brief RAMDebug register definitions and utilities for TMC9660.
 * 
 * The RAMDebug block uses a composite register offset where the upper 4 bits
 * encode the subcommand and the lower 6 bits encode the index. All offsets
 * are within block number 31 for SPI (command 142 for UART).
 * 
 * @defgroup TMC9660_RAMDebug RAMDebug System
 * @brief Real-time data capture and debugging capabilities
 * 
 * @defgroup TMC9660_RAMDebugCommands RAMDebug Commands
 * @brief Subcommands and operations for RAMDebug system
 * 
 * @defgroup TMC9660_RAMDebugTypes RAMDebug Type Definitions
 * @brief Enums and structures for RAMDebug operations
 */

#ifndef TMC9660_RAMDEBUG_HPP
#define TMC9660_RAMDEBUG_HPP

#include <cstdint>

/**
 * @file tmc9660_ramdebug.hpp
 * @brief RAMDebug register definitions and utilities for TMC9660.
 *
 * The RAMDebug block uses a composite register offset where the upper 4 bits
 * encode the subcommand and the lower 6 bits encode the index. All offsets
 * are within block number 31 for SPI (command 142 for UART).
 */

namespace tmc9660 {
namespace register_mode {
/**
 * @brief RAMDebug system namespace for TMC9660.
 * @ingroup TMC9660_RAMDebug
 * 
 * This namespace contains all RAMDebug-related definitions, commands, and
 * utilities for real-time data capture and debugging of the TMC9660 motor
 * control system. The RAMDebug feature allows capturing internal register
 * values and signals for analysis and debugging purposes.
 */
namespace RAMDebug {

/**
 * @brief Block number identifier for RAMDebug operations via SPI interface.
 * 
 * Used when accessing RAMDebug registers through the SPI interface.
 * All RAMDebug operations target block 31 in the SPI register map.
 */
static constexpr uint8_t RAMDEBUG_BLOCK = 31;
/**
 * @brief UART command number for RAMDebug operations via UART interface.
 * 
 * Used when accessing RAMDebug registers through the UART interface.
 * Command 142 provides access to the RAMDebug functionality over UART.
 */
static constexpr uint8_t RAMDEBUG_UART_CMD = 142;

/// RAMDebug subcommands
/**
 * @brief RAMDebug subcommands for configuring and operating the debug system.
 * 
 * These subcommands control the RAMDebug feature which allows capturing
 * real-time data from internal registers and signals. The system supports
 * configurable trigger conditions, multiple capture channels, and flexible
 * sample timing for debugging motor control algorithms and system behavior.
 * 
 * Each subcommand performs a specific operation such as initialization,
 * configuration, triggering, or data retrieval from the debug system.
 */
enum class RamDebugSub : uint8_t {
  Init = 0,                ///< Initialize and reset RAMDebug.
  SetSampleCount = 1,      ///< Set the total number of samples to collect.
  SetPrescaler = 3,        ///< Set the prescaler (divider = value + 1).
  SetChannel = 4,          ///< Configure a capture channel.
  SetTriggerChannel = 5,   ///< Configure the trigger channel.
  SetTriggerMaskShift = 6, ///< Set mask and shift before trigger evaluation.
  TriggerStart = 7,        ///< Set trigger type and start measurement.
  GetState = 8,            ///< Read the current RAMDebug state.
  ReadSample = 9,          ///< Read a captured sample by index.
  GetInfo = 10,            ///< Read general RAMDebug information.
  GetChannelType = 11,     ///< Read the configured channel type.
  GetChannelAddress = 12,  ///< Read the configured channel address.
  SetPreTriggerCount = 13, ///< Set the total number of pretrigger samples.
  GetPreTriggerCount = 14  ///< Read the total number of pretrigger samples.
};

/// RAMDebug info selections (for subcommand GetInfo)
/**
 * @brief RAMDebug information selection options for retrieving system capabilities.
 * 
 * These options specify what type of information to retrieve when using
 * the GetInfo subcommand. Each option returns different system parameters
 * such as maximum supported channels, sample counts, operating frequency,
 * and current capture status.
 */
enum class InfoSelect : uint8_t {
  MaxChannels = 0,       ///< Maximum number of channels supported.
  MaxSamples = 1,        ///< Maximum number of samples supported.
  FrequencyHz = 2,       ///< RAMDebug frequency in Hz.
  CapturedSamples = 3,   ///< Number of samples already captured.
  PrescalerOnTrigger = 4 ///< Prescaler value at the trigger event.
};

/// RAMDebug states (returned by GetState)
/**
 * @brief RAMDebug system states indicating current operational mode.
 * 
 * The RAMDebug system operates in different states depending on the
 * current activity. These states are returned by the GetState subcommand
 * and indicate whether the system is idle, waiting for triggers,
 * actively capturing data, or has completed a capture operation.
 */
enum class RamDebugState : uint8_t {
  Idle = 0,      ///< RAMDebug is not running and can be configured.
  Trigger = 1,   ///< Waiting for the trigger event.
  Capture = 2,   ///< Capturing samples after the trigger.
  Complete = 3,  ///< Capture is complete, samples can be downloaded.
  PreTrigger = 4 ///< Capturing pretrigger samples.
};

/// Trigger types (for TriggerStart)
/**
 * @brief Trigger types for initiating RAMDebug capture operations.
 * 
 * Defines the various trigger conditions that can start a data capture
 * sequence. Triggers can be based on rising edges, falling edges, or
 * any edge detection, with support for both signed and unsigned data
 * interpretation depending on the signal characteristics.
 */
enum class TriggerType : uint8_t {
  NoTrigger = 0,    ///< No trigger.
  RisingEdgeS = 1,  ///< Trigger on signed rising edge.
  FallingEdgeS = 2, ///< Trigger on signed falling edge.
  AnyEdgeS = 3,     ///< Trigger on any signed edge.
  RisingEdgeU = 4,  ///< Trigger on unsigned rising edge.
  FallingEdgeU = 5, ///< Trigger on unsigned falling edge.
  AnyEdgeU = 6      ///< Trigger on any unsigned edge.
};

/**
 * @brief Construct RAMDebug register offset from subcommand and index parameters.
 * 
 * The RAMDebug system uses a composite addressing scheme where register
 * offsets are constructed by combining a subcommand (upper 4 bits) with
 * an index parameter (lower 6 bits). This allows a single register
 * block to handle multiple operations and parameters efficiently.
 * 
 * @param sub Subcommand specifying the operation type
 * @param index Index or parameter value (0-63, masked to 6 bits)
 * @return 10-bit register offset for RAMDebug operations
 */
static inline uint16_t ramDebugOffset(RamDebugSub sub, uint8_t index = 0) {
  return static_cast<uint16_t>((static_cast<uint8_t>(sub) << 6) | (index & 0x3F));
}

} // namespace RAMDebug
} // namespace register_mode
} // namespace tmc9660

#endif // TMC9660_RAMDEBUG_HPP
