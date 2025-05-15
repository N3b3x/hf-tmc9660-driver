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

namespace TMC9660 {
namespace RAMDebug {

/// Block number for RAMDebug (SPI interface)
static constexpr uint8_t RAMDEBUG_BLOCK = 31;
/// UART command number for RAMDebug
static constexpr uint8_t RAMDEBUG_UART_CMD = 142;

/// RAMDebug subcommands
/**
 * @brief Subcommands used to configure and operate the RAMDebug feature.
 */
enum class RamDebugSub : uint8_t {
    Init                =  0, ///< Initialize and reset RAMDebug.
    SetSampleCount      =  1, ///< Set the total number of samples to collect.
    SetPrescaler        =  3, ///< Set the prescaler (divider = value + 1).
    SetChannel          =  4, ///< Configure a capture channel.
    SetTriggerChannel   =  5, ///< Configure the trigger channel.
    SetTriggerMaskShift =  6, ///< Set mask and shift before trigger evaluation.
    TriggerStart        =  7, ///< Set trigger type and start measurement.
    GetState            =  8, ///< Read the current RAMDebug state.
    ReadSample          =  9, ///< Read a captured sample by index.
    GetInfo             = 10, ///< Read general RAMDebug information.
    GetChannelType      = 11, ///< Read the configured channel type.
    GetChannelAddress   = 12, ///< Read the configured channel address.
    SetPreTriggerCount  = 13, ///< Set the total number of pretrigger samples.
    GetPreTriggerCount  = 14  ///< Read the total number of pretrigger samples.
};

/// RAMDebug info selections (for subcommand GetInfo)
/**
 * @brief Options for retrieving general RAMDebug information.
 */
enum class InfoSelect : uint8_t {
    MaxChannels       = 0, ///< Maximum number of channels supported.
    MaxSamples        = 1, ///< Maximum number of samples supported.
    FrequencyHz       = 2, ///< RAMDebug frequency in Hz.
    CapturedSamples   = 3, ///< Number of samples already captured.
    PrescalerOnTrigger= 4  ///< Prescaler value at the trigger event.
};

/// RAMDebug states (returned by GetState)
/**
 * @brief Represents the current state of the RAMDebug system.
 */
enum class RamDebugState : uint8_t {
    Idle       = 0, ///< RAMDebug is not running and can be configured.
    Trigger    = 1, ///< Waiting for the trigger event.
    Capture    = 2, ///< Capturing samples after the trigger.
    Complete   = 3, ///< Capture is complete, samples can be downloaded.
    PreTrigger = 4  ///< Capturing pretrigger samples.
};

/// Trigger types (for TriggerStart)
/**
 * @brief Defines the trigger types for starting a measurement.
 */
enum class TriggerType : uint8_t {
    NoTrigger    = 0, ///< No trigger.
    RisingEdgeS  = 1, ///< Trigger on signed rising edge.
    FallingEdgeS = 2, ///< Trigger on signed falling edge.
    AnyEdgeS     = 3, ///< Trigger on any signed edge.
    RisingEdgeU  = 4, ///< Trigger on unsigned rising edge.
    FallingEdgeU = 5, ///< Trigger on unsigned falling edge.
    AnyEdgeU     = 6  ///< Trigger on any unsigned edge.
};

/**
 * @brief Construct the RAMDebug register offset for a given subcommand and index.
 *
 * The offset is: (subcommand << 6) | (index & 0x3F).
 * @param sub Subcommand value.
 * @param index Index or value parameter (0-63).
 * @return 10-bit register offset (upper 4 bits sub, lower 6 bits index).
 */
static inline uint16_t ramDebugOffset(RamDebugSub sub, uint8_t index = 0) {
    return static_cast<uint16_t>((static_cast<uint8_t>(sub) << 6) | (index & 0x3F));
}

} // namespace RAMDebug
} // namespace TMC9660

#endif // TMC9660_RAMDEBUG_HPP
