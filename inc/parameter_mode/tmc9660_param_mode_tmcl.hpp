//====================================================================================================================
//  @file TMC9660TmclCommands.hpp
//  @brief Enumerations and helpers for **TMCL® operation & reply codes** when the TMC9660 is used in *Parameter Mode*.
//
//  This header complements **TMC9660GlobalParameters.hpp**.
//  It translates Tables 15‑19 (Rev‑0, Feb‑2025 reference manual) into strongly‑typed C++21 constructs:
//    • ::tmcl::Operation — every TMCL command supported by Parameter Mode (Table 18)
//    • ::tmcl::ReplyCode  — all possible TMCL reply / status codes (Table 19)
//    • ::tmcl::RamDebugType — sub‑command numbers for the **RAMDebug** operation (Table 16)
//    • ::tmcl::RamDebugState — state machine values returned by type‑8 “Get state” (Table 17)
//
//  Each enum entry is fully documented with Doxygen, including the **TYPE**, **MOTOR/BANK** and **VALUE** fields.  This
//  allows the code to serve as an executable summary of the protocol.
//
//  --------------------------------------------------------------------------------------------------
//  © 2025 <Your Company>. Released under the MIT License.
//====================================================================================================================
#pragma once

#include <cstdint>

namespace tmc9660::tmcl {

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ████████╗███╗   ███╗ ██████╗██╗          █████╗ ██████╗ ██╗                                                   //
//    ╚══██╔══╝████╗ ████║██╔════╝██║         ██╔══██╗██╔══██╗██║                                                   //
//       ██║   ██╔████╔██║██║     ██║         ███████║██████╔╝██║                                                   //
//       ██║   ██║╚██╔╝██║██║     ██║         ██╔══██║██╔═══╝ ██║                                                   //
//       ██║   ██║ ╚═╝ ██║╚██████╗███████╗    ██║  ██║██║     ██║                                                   //
//       ╚═╝   ╚═╝     ╚═╝ ╚═════╝╚══════╝    ╚═╝  ╚═╝╚═╝     ╚═╝                                                   //
//                                                                                                                  //
//==================================================================================================================//
//                                                  TMCL API SECTION                                                //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//--------------------------------------
//  TMCL operation codes (Table 18)
//--------------------------------------
/**
 * @brief Operation codes accepted by the TMC9660 Parameter Mode (Table 18).
 *
 * Table 18 — TMCL commands:
 *  NUMBER | OPERATION      | TYPE                | MOTOR/BANK         | VALUE                | DESCRIPTION
 *  ------ | -------------- | ------------------- | ------------------ | -------------------- | ---------------------------------------------------------------
 *     3   | MST            | -                   | -                  | -                    | Stops motor movement
 *     5   | SAP            | parameter           | 0                  | value                | Sets axis parameter (motion control specific settings)
 *     6   | GAP            | parameter           | 0                  | -                    | Gets axis parameter (read out motion control specific settings)
 *     7   | STAP           | 0xFFF               | 0xF                | 0xFFFFFFFF           | Stores all storable parameters to external memory
 *     9   | SGP            | parameter           | 0,2,3              | value                | Sets global parameter (module/user settings)
 *    10   | GGP            | parameter           | -                  | -                    | Gets global parameter (module/user settings)
 *    13   | RFS            | START|STOP|STATUS   | 0                  | -                    | Reference search
 *    14   | SIO            | port number         | 0                  | 0,1                  | Sets digital output to specified value
 *    15   | GIO            | port number         | 0: digital, 1: ana | -                    | Gets value of analog/digital input
 *    19   | CALC           | operation           | -                  | value                | Arithmetic operation (accumulator and value)
 *    20   | COMP           | -                   | -                  | value                | Compares accumulator with value
 *    21   | JC             | condition           | -                  | address              | Jump conditional
 *    22   | JA             | -                   | -                  | address              | Jump absolute
 *    23   | CSUB           | -                   | -                  | address              | Calls subroutine
 *    24   | RSUB           | -                   | -                  | -                    | Returns from subroutine
 *    25   | EI             | -                   | -                  | interrupt number     | Enables interrupt
 *    26   | DI             | -                   | -                  | interrupt number     | Disables interrupt
 *    27   | WAIT           | condition           | -                  | ticks                | Waits with further program execution
 *    28   | STOP           | -                   | -                  | -                    | Stops program execution
 *    33   | CALCX          | type                | -                  | -                    | Arithmetic operation (accumulator and X-register)
 *    34   | AAP            | parameter           | 0                  | -                    | Accumulator to axis parameter
 *    35   | AGP            | parameter           | 0,2,3              | -                    | Accumulator to global parameter
 *    36   | CLE            | flag                | -                  | -                    | Clears an error flag
 *    37   | VECT           | interrupt number    | -                  | address              | Defines interrupt vector
 *    38   | RETI           | -                   | -                  | -                    | Returns from interrupt
 *    40   | CALCVV         | type                | user var 1         | user var 2           | Arithmetic: user variable and user variable
 *    41   | CALCVA         | type                | user variable      | -                    | Arithmetic: user variable and accumulator
 *    42   | CALCAV         | type                | user variable      | -                    | Arithmetic: accumulator and user variable
 *    43   | CALCVX         | type                | user variable      | -                    | Arithmetic: user variable and X register
 *    44   | CALCXV         | type                | user variable      | -                    | Arithmetic: X register and user variable
 *    45   | CALCV          | type                | -                  | value                | Arithmetic: user variable and direct value
 *    48   | RST            | -                   | -                  | address              | Restarts the program from the given address
 *    49   | DJNZ           | user variable       | -                  | address              | Decrement and jump if not zero
 *    55   | SIV            | -                   | -                  | value                | Sets indexed variable
 *    56   | GIV            | -                   | -                  | -                    | Gets indexed variable
 *    57   | AIV            | -                   | -                  | -                    | Accumulator to indexed variable
 *   128   | ApplStop        | -                  | -                  | -                    | Stops a running TMCL program
 *   129   | ApplRun         | 0: current, 1: addr| -                  | address              | Starts/continues TMCL program
 *   130   | ApplStep        | -                  | -                  | -                    | Executes only the next TMCL command
 *   131   | ApplReset       | -                  | -                  | -                    | Stops and resets program counter
 *   132   | DownloadStart   | -                  | -                  | -                    | Enter download mode (store script)
 *   133   | DownloadEnd     | -                  | -                  | -                    | End download mode
 *   134   | ReadMem         | -                  | -                  | address              | Returns script command at address
 *   135   | GetStatusScript | -                  | -                  | -                    | Gets status of script
 *   136   | GetVersion      | -                  | -                  | -                    | Gets version of system
 *   137   | FactoryDefault  | -                  | -                  | -                    | Clears settings stored in external memory
 *   141   | Breakpoint      | 0:add,1:del,2:all,3:max| -              | address              | Manage breakpoints
 *   142   | RamDebug        | see Table 16       | see Table 16       | see Table 16         | Access to RAMDebug control
 *   157   | GetInfo         | 0:ID, 1:Version    | -                  | -                    | Gets ID and version info
 *   242   | Boot            | 0x81               | 0x92               | 0xA3B4C5D6           | Returns the system to the bootloader
 *
 * Each command lists the *TYPE*, *MOTOR/BANK*, and *VALUE* fields as per Table 18.
 * For scripting-related commands, see the manual for further details.
 */
enum class Op : std::uint8_t {
    MST                 =   3,  //!< Stop motor movement. TYPE: -, MOTOR/BANK: -, VALUE: -.
    SAP                 =   5,  //!< Set Axis Parameter. TYPE: parameter, MOTOR/BANK: 0, VALUE: value.
    GAP                 =   6,  //!< Get Axis Parameter. TYPE: parameter, MOTOR/BANK: 0, VALUE: -.
    STAP                =   7,  //!< Store All Parameters. TYPE: 0xFFF, MOTOR/BANK: 0xF, VALUE: 0xFFFFFFFF.
    SGP                 =   9,  //!< Set Global Parameter. TYPE: parameter, MOTOR/BANK: 0,2,3, VALUE: value.
    GGP                 =  10,  //!< Get Global Parameter. TYPE: parameter, MOTOR/BANK: -, VALUE: -.
    RFS                 =  13,  //!< Reference Search. TYPE: START|STOP|STATUS, MOTOR/BANK: 0, VALUE: -.
    SIO                 =  14,  //!< Set IO. TYPE: port number, MOTOR/BANK: 0, VALUE: 0,1.
    GIO                 =  15,  //!< Get IO. TYPE: port number, MOTOR/BANK: 0 (digital) or 1 (analog), VALUE: -.
    CALC                =  19,  //!< Arithmetic operation. TYPE: operation, MOTOR/BANK: -, VALUE: value.
    COMP                =  20,  //!< Compare accumulator. TYPE: -, MOTOR/BANK: -, VALUE: value.
    JC                  =  21,  //!< Jump Conditional. TYPE: condition, MOTOR/BANK: -, VALUE: address.
    JA                  =  22,  //!< Jump Absolute. TYPE: -, MOTOR/BANK: -, VALUE: address.
    CSUB                =  23,  //!< Call Subroutine. TYPE: -, MOTOR/BANK: -, VALUE: address.
    RSUB                =  24,  //!< Return from Subroutine. TYPE: -, MOTOR/BANK: -, VALUE: -.
    EI                  =  25,  //!< Enable Interrupt. TYPE: -, MOTOR/BANK: -, VALUE: interrupt number.
    DI                  =  26,  //!< Disable Interrupt. TYPE: -, MOTOR/BANK: -, VALUE: interrupt number.
    WAIT                =  27,  //!< Wait. TYPE: condition, MOTOR/BANK: -, VALUE: ticks.
    STOP                =  28,  //!< Stop Script Execution. TYPE: -, MOTOR/BANK: -, VALUE: -.
    CALCX               =  33,  //!< Arithmetic accumulator <-> X-register. TYPE: type, MOTOR/BANK: -, VALUE: -.
    AAP                 =  34,  //!< Accumulator to Axis Parameter. TYPE: parameter, MOTOR/BANK: 0, VALUE: -.
    AGP                 =  35,  //!< Accumulator to Global Parameter. TYPE: parameter, MOTOR/BANK: 0,2,3, VALUE: -.
    CLE                 =  36,  //!< Clear Error Flag. TYPE: flag, MOTOR/BANK: -, VALUE: -.
    VECT                =  37,  //!< Define Interrupt Vector. TYPE: interrupt number, MOTOR/BANK: -, VALUE: address.
    RETI                =  38,  //!< Return from Interrupt. TYPE: -, MOTOR/BANK: -, VALUE: -.
    CALCVV              =  40,  //!< UserVar ∘ UserVar arithmetic. TYPE: type, MOTOR/BANK: user variable 1, VALUE: user variable 2.
    CALCVA              =  41,  //!< UserVar ∘ Accumulator. TYPE: type, MOTOR/BANK: user variable, VALUE: -.
    CALCAV              =  42,  //!< Accumulator ∘ UserVar. TYPE: type, MOTOR/BANK: user variable, VALUE: -.
    CALCVX              =  43,  //!< UserVar ∘ X-register. TYPE: type, MOTOR/BANK: user variable, VALUE: -.
    CALCXV              =  44,  //!< X-register ∘ UserVar. TYPE: type, MOTOR/BANK: user variable, VALUE: -.
    CALCV               =  45,  //!< UserVar ∘ literal value. TYPE: type, MOTOR/BANK: -, VALUE: value.
    RST                 =  48,  //!< Restart script from address. TYPE: -, MOTOR/BANK: -, VALUE: address.
    DJNZ                =  49,  //!< Decrement-and-Jump if not zero. TYPE: user variable, MOTOR/BANK: -, VALUE: address.
    SIV                 =  55,  //!< Set Indexed Variable. TYPE: -, MOTOR/BANK: -, VALUE: value.
    GIV                 =  56,  //!< Get Indexed Variable. TYPE: -, MOTOR/BANK: -, VALUE: -.
    AIV                 =  57,  //!< Accumulator to Indexed Variable. TYPE: -, MOTOR/BANK: -, VALUE: -.

    // High-level application control
    ApplStop            = 128, //!< Stop running TMCL program. TYPE: -, MOTOR/BANK: -, VALUE: -.
    ApplRun             = 129, //!< Run/continue TMCL program. TYPE: 0 (current addr) or 1 (specified), MOTOR/BANK: -, VALUE: address.
    ApplStep            = 130, //!< Execute single TMCL instruction. TYPE: -, MOTOR/BANK: -, VALUE: -.
    ApplReset           = 131, //!< Reset program counter. TYPE: -, MOTOR/BANK: -, VALUE: -.
    DownloadStart       = 132, //!< Enter download (script upload) mode. TYPE: -, MOTOR/BANK: -, VALUE: -.
    DownloadEnd         = 133, //!< Leave download mode. TYPE: -, MOTOR/BANK: -, VALUE: -.
    ReadMem             = 134, //!< Read script word at address. TYPE: -, MOTOR/BANK: -, VALUE: address.
    GetStatusScript     = 135, //!< Get script status. TYPE: -, MOTOR/BANK: -, VALUE: -.
    GetVersion          = 136, //!< Get firmware version string. TYPE: -, MOTOR/BANK: -, VALUE: -.
    FactoryDefault      = 137, //!< Erase stored config & reset. TYPE: -, MOTOR/BANK: -, VALUE: -.
    Breakpoint          = 141, //!< Manage breakpoints. TYPE: 0 (add), 1 (del), 2 (del all), 3 (get max), MOTOR/BANK: -, VALUE: address.
    RamDebug            = 142, //!< RAMDebug Control. TYPE/MOTOR/BANK/VALUE: see Table 16.
    GetInfo             = 157, //!< Generic info (ID, version, etc.). TYPE: 0 (ID), 1 (Version), MOTOR/BANK: -, VALUE: -.
    Boot                = 242  //!< Exit to bootloader. TYPE: 0x81, MOTOR/BANK: 0x92, VALUE: 0xA3B4C5D6.
};

//--------------------------------------
//  TMCL reply codes (Table 19)
//--------------------------------------
/**
 * @brief Status codes returned in the *status* byte of every TMCL reply.
 *
 * Table 19 — TMCL status codes:
 *  NUMBER | NAME                        | DESCRIPTION
 *  ------ | --------------------------- | ---------------------------------------------------------------
 *   100   | REPLY_OK                    | Command executed successfully.
 *   101   | REPLY_CMD_LOADED            | Command loaded successfully.
 *     1   | REPLY_CHKERR                | Check error occurred (e.g., checksum error).
 *     2   | REPLY_INVALID_CMD           | Invalid command received (unknown command number).
 *     3   | REPLY_WRONG_TYPE            | Wrong type of data received (TYPE field invalid).
 *     4   | REPLY_INVALID_VALUE         | Invalid value received (VALUE field out of range).
 *     6   | REPLY_CMD_NOT_AVAILABLE     | Command not available (not supported in this mode).
 *     7   | REPLY_CMD_LOAD_ERROR        | Error occurred while loading command (storage error).
 *     9   | REPLY_MAX_EXCEEDED          | Maximum limit exceeded (e.g., too many breakpoints).
 *    10   | REPLY_DOWNLOAD_NOT_POSSIBLE | Download operation not possible (misuse or memory full).
 */
enum class ReplyCode : std::uint8_t {
    REPLY_OK                       = 100, //!< Command executed successfully.
    REPLY_CMD_LOADED               = 101, //!< Command loaded successfully.
    REPLY_CHKERR                   =   1, //!< Check error occurred (e.g., checksum error).
    REPLY_INVALID_CMD              =   2, //!< Invalid command received (unknown command number).
    REPLY_WRONG_TYPE               =   3, //!< Wrong type of data received (TYPE field invalid).
    REPLY_INVALID_VALUE            =   4, //!< Invalid value received (VALUE field out of range).
    REPLY_CMD_NOT_AVAILABLE        =   6, //!< Command not available (not supported in this mode).
    REPLY_CMD_LOAD_ERROR           =   7, //!< Error occurred while loading command (storage error).
    REPLY_MAX_EXCEEDED             =   9, //!< Maximum limit exceeded (e.g., too many breakpoints).
    REPLY_DOWNLOAD_NOT_POSSIBLE    =  10  //!< Download operation not possible (misuse or memory full).
};

//--------------------------------------
//  RAMDebug sub‑commands / type codes (Table 16)
//--------------------------------------
/**
 * @brief TYPE field values when **Operation::RamDebug** is issued.
 *
 * Table 16 — RAMDebug type commands:
 *  NUMBER | TYPE (subcode)      | MOTOR/BANK                                      | VALUE                                                                 | DESCRIPTION
 *  ------ | ------------------- | ----------------------------------------------- | --------------------------------------------------------------------- | ----------------------------------------------------------
 *     0   | INITIALISE_RESET    | -                                               | -                                                                     | Initialize and reset RAMDebug configuration & buffers.
 *     1   | SET_SAMPLE_COUNT    | -                                               | Number                                                               | Set total number of samples to collect (not per-channel).
 *     3   | SET_PRESCALER       | -                                               | Prescale                                                             | Set divider for sampling rate (divider = VALUE+1).
 *     4   | SET_CHANNEL         | Type: 0=Disabled, 1=Parameter, 3=Global param   | Motor/Bank: 0xFF000000, AP/GP number: 0x00000FFF                     | Configure capture channel.
 *     5   | SET_TRIGGER_CHANNEL | Type: 0=Disabled, 1=Parameter, 3=Global param   | Motor/Bank: 0xFF000000, AP/GP number: 0x00000FFF                     | Specify source of trigger data.
 *     6   | SET_TRIGGER_MASK_SHIFT| Shift                                         | Mask                                                                 | Specify mask and shift for trigger value.
 *     7   | ENABLE_TRIGGER      | Type: 0=Uncond, 1=RiseS, 2=FallS, 3=DualS,      | Threshold                                                            | Start measurement by enabling trigger.
 *         |                    | 4=RiseU, 5=FallU, 6=BothU                        |                                                                     |
 *     8   | GET_STATE           | -                                               | -                                                                     | Request state of RAMDebug (see RamDebugState).
 *     9   | READ_SAMPLE         | Index                                           | -                                                                     | Download sampled values at index.
 *    10   | GET_INFO            | -                                               | 0=Max channels, 1=Buffer size, 2=RAMDebug freq, 3=Captured count,    | Read general info.
 *         |                    |                                                 | 4=Prescaler on trigger                                               |
 *    11   | GET_CHANNEL_TYPE    | Index                                           | -                                                                     | Read channel type info.
 *    12   | GET_CHANNEL_ADDRESS | Index                                           | -                                                                     | Read channel address.
 *    13   | SET_PRETRIGGER_COUNT| -                                               | Number                                                               | Set total number of pretrigger samples (not per-channel).
 *    14   | GET_PRETRIGGER_COUNT| -                                               | -                                                                     | Get total number of pretrigger samples.
 */
enum class RamDebugType : std::uint8_t {
    INITIALISE_RESET            =  0, //!< Initialize and reset RAMDebug configuration & buffers.
    SET_SAMPLE_COUNT            =  1, //!< VALUE: Number of samples to collect in total (not per-channel).
    SET_PRESCALER               =  3, //!< VALUE: Prescale value. Sets divider for sampling rate (divider = VALUE+1).
    SET_CHANNEL                 =  4, //!< Configure capture channel. TYPE: 0=Disabled, 1=Parameter, 3=Global parameter. MOTOR/BANK: 0xFF000000. VALUE: AP/GP number (0x00000FFF).
    SET_TRIGGER_CHANNEL         =  5, //!< Specify trigger source. TYPE: 0=Disabled, 1=Parameter, 3=Global parameter. MOTOR/BANK: 0xFF000000. VALUE: AP/GP number (0x00000FFF).
    SET_TRIGGER_MASK_SHIFT      =  6, //!< Specify mask and shift for trigger value. MOTOR/BANK: Shift. VALUE: Mask.
    ENABLE_TRIGGER              =  7, //!< Start measurement by enabling trigger. TYPE: 0=Unconditional, 1=Rising edge signed, 2=Falling edge signed, 3=Dual edge signed, 4=Rising edge unsigned, 5=Falling edge unsigned, 6=Both edge unsigned. VALUE: Threshold.
    GET_STATE                   =  8, //!< Request state of RAMDebug (see RamDebugState).
    READ_SAMPLE                 =  9, //!< Download sampled values. MOTOR/BANK: Index.
    GET_INFO                    = 10, //!< Read general info. VALUE: 0=Max channels, 1=Buffer size, 2=RAMDebug frequency, 3=Captured sample count, 4=Prescaler value on trigger event.
    GET_CHANNEL_TYPE            = 11, //!< Read channel type info. MOTOR/BANK: Index.
    GET_CHANNEL_ADDRESS         = 12, //!< Read channel address. MOTOR/BANK: Index.
    SET_PRETRIGGER_COUNT        = 13, //!< Set total number of pretrigger samples (not per-channel). VALUE: Number of samples.
    GET_PRETRIGGER_COUNT        = 14  //!< Get total number of pretrigger samples.
};

//--------------------------------------
//  RAMDebug state machine (Table 17)
//--------------------------------------
/**
 * @brief Values returned by ::RamDebugType::GET_STATE.
 *
 * Table 17 — List of RAMDebug states:
 *  NUMBER | NAME       | DESCRIPTION
 *  ------ | ---------- | -------------------------------------------------------------------------------------------
 *     0   | Idle       | RAMDebug is not running and can be configured. Use type code 0 to enter this state.
 *     1   | Trigger    | RAMDebug is waiting for the trigger event to happen. When updating a value that RAMDebug
 *                      | is triggering on, ensure this state is reached before updating.
 *     2   | Capture    | RAMDebug has been triggered and is capturing samples.
 *     3   | Complete   | RAMDebug has finished capturing samples. The data can now be downloaded using type code 9.
 *     4   | Pretrigger | RAMDebug is capturing samples for the pretrigger.
 */
enum class RamDebugState : std::uint8_t {
    IDLE       = 0, //!< Idle: RAMDebug is not running and can be configured. Use type code 0 to enter this state.
    TRIGGER    = 1, //!< Trigger: Waiting for the trigger event. Ensure this state before updating trigger value.
    CAPTURE    = 2, //!< Capture: RAMDebug has been triggered and is capturing samples.
    COMPLETE   = 3, //!< Complete: Finished capturing samples. Data can be downloaded using type code 9.
    PRETRIGGER = 4  //!< Pretrigger: Capturing samples for the pretrigger.
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██████╗  █████╗ ████████╗███████╗    ██████╗ ██████╗ ██╗██╗   ██╗███████╗██████╗ ███████╗                     //
//   ██╔════╝ ██╔══██╗╚══██╔══╝██╔════╝    ██╔══██╗██╔══██╗██║██║   ██║██╔════╝██╔══██╗██╔════╝                     //
//   ██║  ███╗███████║   ██║   █████╗      ██║  ██║██████╔╝██║██║   ██║█████╗  ██████╔╝███████╗                     //
//   ██║   ██║██╔══██║   ██║   ██╔══╝      ██║  ██║██╔══██╗██║╚██╗ ██╔╝██╔══╝  ██╔══██╗╚════██║                     //
//   ╚██████╔╝██║  ██║   ██║   ███████╗    ██████╔╝██║  ██║██║ ╚████╔╝ ███████╗██║  ██║███████║                     //
//    ╚═════╝ ╚═╝  ╚═╝   ╚═╝   ╚══════╝    ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═══╝  ╚══════╝╚═╝  ╚═╝╚══════╝                     //
//                                                                                                                  //
//==================================================================================================================//
//                                              GATE DRIVER SECTION                                                 //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//--------------------------------------
//  Gate Driver Parameters (Table 20)
//--------------------------------------
/**
 * @brief Gate driver timer and current settings (Table 20).
 *
 * These parameters configure the MOSFET gate driver for the TMC9660, including output polarity, break-before-make timing,
 * adaptive drive time, drive times, current limits, and bootstrap current limit.
 *
 * Table 20 — Gate driver timer and current settings:
 *  NR   | PARAMETER                        | DESCRIPTION
 *  ---- | -------------------------------- | ---------------------------------------------------------------------------
 *  233  | PWM_L_OUTPUT_POLARITY            | PWM_L output polarity. 0: ACTIVE_HIGH, 1: ACTIVE_LOW
 *  234  | PWM_H_OUTPUT_POLARITY            | PWM_H output polarity. 0: ACTIVE_HIGH, 1: ACTIVE_LOW
 *  235  | BREAK_BEFORE_MAKE_TIME_LOW_UVW   | Break-before-make time for low side UVW [8.33ns units] 0...255
 *  236  | BREAK_BEFORE_MAKE_TIME_HIGH_UVW  | Break-before-make time for high side UVW [8.33ns units] 0...255
 *  237  | BREAK_BEFORE_MAKE_TIME_LOW_Y2    | Break-before-make time for low side Y2 [8.33ns units] 0...255
 *  238  | BREAK_BEFORE_MAKE_TIME_HIGH_Y2   | Break-before-make time for high side Y2 [8.33ns units] 0...255
 *  239  | USE_ADAPTIVE_DRIVE_TIME_UVW      | Adaptive drive time UVW. 0: DISABLED, 1: ENABLED
 *  240  | USE_ADAPTIVE_DRIVE_TIME_Y2       | Adaptive drive time Y2. 0: DISABLED, 1: ENABLED
 *  241  | DRIVE_TIME_SINK_UVW              | Discharge time UVW [0...255] (1s/120MHz) × (2×value+3)
 *  242  | DRIVE_TIME_SOURCE_UVW            | Charge time UVW [0...255] (1s/120MHz) × (2×value+3)
 *  243  | DRIVE_TIME_SINK_Y2               | Discharge time Y2 [0...255] (1s/120MHz) × (2×value+3)
 *  244  | DRIVE_TIME_SOURCE_Y2             | Charge time Y2 [0...255] (1s/120MHz) × (2×value+3)
 *  245  | UVW_SINK_CURRENT                 | See ::tmc9660::tmcl::GateCurrentSink
 *  246  | UVW_SOURCE_CURRENT               | See ::tmc9660::tmcl::GateCurrentSource
 *  247  | Y2_SINK_CURRENT                  | See ::tmc9660::tmcl::GateCurrentSink
 *  248  | Y2_SOURCE_CURRENT                | See ::tmc9660::tmcl::GateCurrentSource
 *  249  | BOOTSTRAP_CURRENT_LIMIT          | See ::tmc9660::tmcl::BootstrapCurrentLimit
 */
enum class GateDriver : uint16_t {
    PWM_L_OUTPUT_POLARITY = 233,           ///< 0: ACTIVE_HIGH, 1: ACTIVE_LOW
    PWM_H_OUTPUT_POLARITY = 234,           ///< 0: ACTIVE_HIGH, 1: ACTIVE_LOW
    BREAK_BEFORE_MAKE_TIME_LOW_UVW = 235,  ///< [8.33ns units] 0...255
    BREAK_BEFORE_MAKE_TIME_HIGH_UVW = 236, ///< [8.33ns units] 0...255
    BREAK_BEFORE_MAKE_TIME_LOW_Y2 = 237,   ///< [8.33ns units] 0...255
    BREAK_BEFORE_MAKE_TIME_HIGH_Y2 = 238,  ///< [8.33ns units] 0...255
    USE_ADAPTIVE_DRIVE_TIME_UVW = 239,     ///< 0: DISABLED, 1: ENABLED
    USE_ADAPTIVE_DRIVE_TIME_Y2 = 240,      ///< 0: DISABLED, 1: ENABLED
    DRIVE_TIME_SINK_UVW = 241,             ///< [0...255] (1s/120MHz) × (2×value+3)
    DRIVE_TIME_SOURCE_UVW = 242,           ///< [0...255] (1s/120MHz) × (2×value+3)
    DRIVE_TIME_SINK_Y2 = 243,              ///< [0...255] (1s/120MHz) × (2×value+3)
    DRIVE_TIME_SOURCE_Y2 = 244,            ///< [0...255] (1s/120MHz) × (2×value+3)
    UVW_SINK_CURRENT = 245,                ///< See GateCurrentSink
    UVW_SOURCE_CURRENT = 246,              ///< See GateCurrentSource
    Y2_SINK_CURRENT = 247,                 ///< See GateCurrentSink
    Y2_SOURCE_CURRENT = 248,               ///< See GateCurrentSource
    BOOTSTRAP_CURRENT_LIMIT = 249          ///< See BootstrapCurrentLimit
};

/**
 * @brief Enumerates possible values for UVW_SINK_CURRENT and Y2_SINK_CURRENT (50–2000mA).
 *
 * 0: 50mA, 1: 100mA, 2: 160mA, 3: 210mA, 4: 270mA, 5: 320mA, 6: 380mA, 7: 430mA,
 * 8: 580mA, 9: 720mA, 10: 860mA, 11: 1000mA, 12: 1250mA, 13: 1510mA, 14: 1770mA, 15: 2000mA
 */
enum class GateCurrentSink : uint8_t {
    CUR_50_MA = 0,    ///< 50 mA
    CUR_100_MA = 1,   ///< 100 mA
    CUR_160_MA = 2,   ///< 160 mA
    CUR_210_MA = 3,   ///< 210 mA
    CUR_270_MA = 4,   ///< 270 mA
    CUR_320_MA = 5,   ///< 320 mA
    CUR_380_MA = 6,   ///< 380 mA
    CUR_430_MA = 7,   ///< 430 mA
    CUR_580_MA = 8,   ///< 580 mA
    CUR_720_MA = 9,   ///< 720 mA
    CUR_860_MA = 10,  ///< 860 mA
    CUR_1000_MA = 11, ///< 1000 mA
    CUR_1250_MA = 12, ///< 1250 mA
    CUR_1510_MA = 13, ///< 1510 mA
    CUR_1770_MA = 14, ///< 1770 mA
    CUR_2000_MA = 15  ///< 2000 mA
};

/**
 * @brief Enumerates possible values for UVW_SOURCE_CURRENT and Y2_SOURCE_CURRENT (25–1000mA).
 *
 * 0: 25mA, 1: 50mA, 2: 80mA, 3: 105mA, 4: 135mA, 5: 160mA, 6: 190mA, 7: 215mA,
 * 8: 290mA, 9: 360mA, 10: 430mA, 11: 500mA, 12: 625mA, 13: 755mA, 14: 855mA, 15: 1000mA
 */
enum class GateCurrentSource : uint8_t {
    CUR_25_MA = 0,    ///< 25 mA
    CUR_50_MA = 1,    ///< 50 mA
    CUR_80_MA = 2,    ///< 80 mA
    CUR_105_MA = 3,   ///< 105 mA
    CUR_135_MA = 4,   ///< 135 mA
    CUR_160_MA = 5,   ///< 160 mA
    CUR_190_MA = 6,   ///< 190 mA
    CUR_215_MA = 7,   ///< 215 mA
    CUR_290_MA = 8,   ///< 290 mA
    CUR_360_MA = 9,   ///< 360 mA
    CUR_430_MA = 10,  ///< 430 mA
    CUR_500_MA = 11,  ///< 500 mA
    CUR_625_MA = 12,  ///< 625 mA
    CUR_755_MA = 13,  ///< 755 mA
    CUR_855_MA = 14,  ///< 855 mA
    CUR_1000_MA = 15  ///< 1000 mA
};

/**
 * @brief Enumerates possible values for BOOTSTRAP_CURRENT_LIMIT (45–391mA).
 *
 * 0: 45mA, 1: 91mA, 2: 141mA, 3: 191mA, 4: 267mA, 5: 292mA, 6: 341mA, 7: 391mA
 */
enum class BootstrapCurrentLimit : uint8_t {
    CUR_45_MA = 0,   ///< 45 mA
    CUR_91_MA = 1,   ///< 91 mA
    CUR_141_MA = 2,  ///< 141 mA
    CUR_191_MA = 3,  ///< 191 mA
    CUR_267_MA = 4,  ///< 267 mA
    CUR_292_MA = 5,  ///< 292 mA
    CUR_341_MA = 6,  ///< 341 mA
    CUR_391_MA = 7   ///< 391 mA
};

/// @name Gate Driver Overcurrent Protection Parameters
/// @{
/**
 * @brief Parameters for configuring gate driver overcurrent protection.
 * 
 * These parameters configure the overcurrent protection thresholds, blanking times,
 * and deglitch times for the UVW and Y2 phases of the TMC9660 gate driver.
 */
enum class OvercurrentProtection : uint16_t {
    UVW_HIGH_SIDE_THRESHOLD = 259, ///< Threshold for UVW phases high side [0-15]. Default: 0.
    Y2_LOW_SIDE_THRESHOLD = 260, ///< Threshold for Y2 phase low side [0-15]. Default: 0.
    Y2_HIGH_SIDE_THRESHOLD = 261, ///< Threshold for Y2 phase high side [0-15]. Default: 0.
    UVW_LOW_SIDE_BLANKING = 262, ///< Blanking time for UVW phases low side [0-7]. Default: 2.
    UVW_HIGH_SIDE_BLANKING = 263, ///< Blanking time for UVW phases high side [0-7]. Default: 2.
    Y2_LOW_SIDE_BLANKING = 264, ///< Blanking time for Y2 phase low side [0-7]. Default: 2.
    Y2_HIGH_SIDE_BLANKING = 265, ///< Blanking time for Y2 phase high side [0-7]. Default: 2.
    UVW_LOW_SIDE_DEGLITCH = 266, ///< Deglitch time for UVW phases low side [0-7]. Default: 6.
    UVW_HIGH_SIDE_DEGLITCH = 267, ///< Deglitch time for UVW phases high side [0-7]. Default: 6.
    Y2_LOW_SIDE_DEGLITCH = 268, ///< Deglitch time for Y2 phase low side [0-7]. Default: 6.
    Y2_HIGH_SIDE_DEGLITCH = 269, ///< Deglitch time for Y2 phase high side [0-7]. Default: 6.
    UVW_LOW_SIDE_USE_VDS = 270, ///< Use VDS measurement for UVW phases low side. 0: DISABLED, 1: ENABLED. Default: 1.
    Y2_LOW_SIDE_USE_VDS = 271 ///< Use VDS measurement for Y2 phase low side. 0: DISABLED, 1: ENABLED. Default: 1.
};
/// @}

/// @name Overcurrent Protection Thresholds
/// @{
/**
 * @brief Enumerates possible threshold values for overcurrent protection.
 * 
 * Used with UVW_HIGH_SIDE_THRESHOLD, Y2_LOW_SIDE_THRESHOLD (with VDS enabled),
 * and Y2_HIGH_SIDE_THRESHOLD parameters.
 */
enum class OvercurrentThreshold : uint8_t {
    V_63_MILLIVOLT = 0,    ///< 63 mV
    V_125_MILLIVOLT = 1,   ///< 125 mV
    V_187_MILLIVOLT = 2,   ///< 187 mV
    V_248_MILLIVOLT = 3,   ///< 248 mV
    V_312_MILLIVOLT = 4,   ///< 312 mV
    V_374_MILLIVOLT = 5,   ///< 374 mV
    V_434_MILLIVOLT = 6,   ///< 434 mV
    V_504_MILLIVOLT = 7,   ///< 504 mV
    V_705_MILLIVOLT = 8,   ///< 705 mV
    V_940_MILLIVOLT = 9,   ///< 940 mV
    V_1180_MILLIVOLT = 10, ///< 1180 mV
    V_1410_MILLIVOLT = 11, ///< 1410 mV
    V_1650_MILLIVOLT = 12, ///< 1650 mV
    V_1880_MILLIVOLT = 13, ///< 1880 mV
    V_2110_MILLIVOLT = 14, ///< 2110 mV
    V_2350_MILLIVOLT = 15  ///< 2350 mV
};

/**
 * @brief Enumerates possible threshold values for Y2 low side when VDS_USE is disabled.
 * 
 * Used with Y2_LOW_SIDE_THRESHOLD when UVW_LOW_SIDE_USE_VDS = 0.
 */
enum class Y2LowSideNonVdsThreshold : uint8_t {
    V_80_MILLIVOLT = 0,    ///< 80 mV
    V_165_MILLIVOLT = 1,   ///< 165 mV
    V_250_MILLIVOLT = 2,   ///< 250 mV
    V_330_MILLIVOLT = 3,   ///< 330 mV
    V_415_MILLIVOLT = 4,   ///< 415 mV
    V_500_MILLIVOLT = 5,   ///< 500 mV
    V_582_MILLIVOLT = 6,   ///< 582 mV
    V_660_MILLIVOLT = 7,   ///< 660 mV
    V_125_MILLIVOLT = 8,   ///< 125 mV (different scale)
    V_250_MILLIVOLT_ALT = 9,   ///< 250 mV (different scale)
    V_375_MILLIVOLT = 10,  ///< 375 mV
    V_500_MILLIVOLT_ALT = 11,  ///< 500 mV (different scale)
    V_625_MILLIVOLT = 12,  ///< 625 mV
    V_750_MILLIVOLT = 13,  ///< 750 mV
    V_875_MILLIVOLT = 14,  ///< 875 mV
    V_1000_MILLIVOLT = 15  ///< 1000 mV
};
/// @}

/// @name Overcurrent Protection Blanking and Deglitch Times
/// @{
/**
 * @brief Enumerates possible blanking and deglitch times for overcurrent protection.
 * 
 * Used for all blanking and deglitch time parameters.
 */
enum class OvercurrentTiming : uint8_t {
    OFF = 0,              ///< No blanking or deglitching
    T_0_25_MICROSEC = 1,  ///< 0.25 µs
    T_0_5_MICROSEC = 2,   ///< 0.5 µs
    T_1_MICROSEC = 3,     ///< 1.0 µs
    T_2_MICROSEC = 4,     ///< 2.0 µs
    T_4_MICROSEC = 5,     ///< 4.0 µs
    T_6_MICROSEC = 6,     ///< 6.0 µs
    T_8_MICROSEC = 7      ///< 8.0 µs
};
/// @}

/// @name VDS Measurement Enable/Disable
/// @{
/**
 * @brief Enumerates options for VDS measurement usage in overcurrent protection.
 */
enum class VdsUsage : uint8_t {
    DISABLED = 0,  ///< VDS measurement disabled
    ENABLED = 1    ///< VDS measurement enabled
};
/// @}

/// @name Undervoltage Protection Parameters
/// @{
/**
 * @brief Parameters for configuring undervoltage lockout (UVLO) protection.
 * 
 * These parameters configure the undervoltage protection for the supply voltage (VS),
 * driver voltage (VDRV), and bootstrap capacitors for UVW and Y2 phases.
 */
enum class UndervoltageProtection : uint16_t {
    SUPPLY_LEVEL = 250, ///< Protection level for VS (Supply voltage) [0-16]. Default: 0.
    VDRV_ENABLE = 251, ///< Enable protection for VDRV (Driver voltage) [0, 1]. Default: 1.
    BST_UVW_ENABLE = 252, ///< Enable protection for UVW bootstrap capacitors [0, 1]. Default: 1.
    BST_Y2_ENABLE = 253 ///< Enable protection for Y2 bootstrap capacitor [0, 1]. Default: 1.
};
/// @}

/// @name Undervoltage Protection Levels
/// @{
/**
 * @brief Enumerates possible levels for undervoltage protection.
 * 
 * Used with the SUPPLY_LEVEL parameter to configure the VS undervoltage threshold.
 */
enum class UndervoltageLevel : uint8_t {
    DISABLED = 0, ///< Comparator disabled.
    LEVEL_0  = 1, ///< Hardware level 0.
    LEVEL_1  = 2, ///< Hardware level 1.
    LEVEL_2  = 3, ///< Hardware level 2.
    LEVEL_3  = 4, ///< Hardware level 3.
    LEVEL_4  = 5, ///< Hardware level 4.
    LEVEL_5  = 6, ///< Hardware level 5.
    LEVEL_6  = 7, ///< Hardware level 6.
    LEVEL_7  = 8, ///< Hardware level 7.
    LEVEL_8  = 9, ///< Hardware level 8.
    LEVEL_9  = 10, ///< Hardware level 9.
    LEVEL_10 = 11, ///< Hardware level 10.
    LEVEL_11 = 12, ///< Hardware level 11.
    LEVEL_12 = 13, ///< Hardware level 12.
    LEVEL_13 = 14, ///< Hardware level 13.
    LEVEL_14 = 15, ///< Hardware level 14.
    LEVEL_15 = 16  ///< Hardware level 15.
};

/// @name Undervoltage Protection Enable/Disable
/// @{
/**
 * @brief Enumerates options for enabling or disabling undervoltage protection.
 */
enum class UndervoltageEnable : uint8_t {
    DISABLED = 0, ///< Protection disabled.
    ENABLED  = 1  ///< Protection enabled.
};
/// @}

/// @name Gate Short (VGS) Protection Parameters
/// @{
/**
 * @brief Parameters for configuring gate-to-source (VGS) short circuit protection.
 * 
 * These parameters enable/disable protection against gate-to-source voltage shorts for
 * different phases (UVW, Y2) and transitions (ON, OFF) of both low and high side drivers.
 * The protection includes configurable blanking and deglitch times.
 */
enum class VgsShortProtection : uint16_t {
    UVW_LOW_SIDE_ON_ENABLE = 272,  ///< Enable VGS short protection for ON transition of UVW low side [0, 1]. Default: 1.
    UVW_LOW_SIDE_OFF_ENABLE = 273, ///< Enable VGS short protection for OFF transition of UVW low side [0, 1]. Default: 1.
    UVW_HIGH_SIDE_ON_ENABLE = 274, ///< Enable VGS short protection for ON transition of UVW high side [0, 1]. Default: 1.
    UVW_HIGH_SIDE_OFF_ENABLE = 275,///< Enable VGS short protection for OFF transition of UVW high side [0, 1]. Default: 1.
    Y2_LOW_SIDE_ON_ENABLE = 276,   ///< Enable VGS short protection for ON transition of Y2 low side [0, 1]. Default: 1.
    Y2_LOW_SIDE_OFF_ENABLE = 277,  ///< Enable VGS short protection for OFF transition of Y2 low side [0, 1]. Default: 1.
    Y2_HIGH_SIDE_ON_ENABLE = 278,  ///< Enable VGS short protection for ON transition of Y2 high side [0, 1]. Default: 1.
    Y2_HIGH_SIDE_OFF_ENABLE = 279, ///< Enable VGS short protection for OFF transition of Y2 high side [0, 1]. Default: 1.
    UVW_BLANKING = 280,            ///< VGS short protection blanking time for UVW phases [0-3]. Default: 1.
    Y2_BLANKING = 281,             ///< VGS short protection blanking time for Y2 phase [0-3]. Default: 1.
    UVW_DEGLITCH = 282,            ///< VGS short protection deglitch time for UVW phases [0-7]. Default: 1.
    Y2_DEGLITCH = 283              ///< VGS short protection deglitch time for Y2 phase [0-7]. Default: 1.
};
/// @}

/// @name VGS Protection Enable/Disable
/// @{
/**
 * @brief Enumerates options for enabling or disabling VGS short protection.
 */
enum class VgsShortEnable : uint8_t {
    DISABLED = 0, ///< Protection disabled.
    ENABLED = 1   ///< Protection enabled.
};
/// @}

/// @name VGS Protection Blanking Time
/// @{
/**
 * @brief Enumerates possible blanking times for VGS short protection.
 * 
 * Used with UVW_BLANKING and Y2_BLANKING parameters.
 */
enum class VgsBlankingTime : uint8_t {
    OFF = 0,               ///< No blanking.
    T_0_25_MICROSEC = 1,   ///< 0.25 µs blanking time.
    T_0_5_MICROSEC = 2,    ///< 0.5 µs blanking time.
    T_1_MICROSEC = 3       ///< 1.0 µs blanking time.
};
/// @}

/// @name VGS Protection Deglitch Time
/// @{
/**
 * @brief Enumerates possible deglitch times for VGS short protection.
 * 
 * Used with UVW_DEGLITCH and Y2_DEGLITCH parameters.
 * Determines minimum duration of short-circuit before recognized as fault.
 */
enum class VgsDeglitchTime : uint8_t {
    OFF = 0,               ///< No deglitching.
    T_0_25_MICROSEC = 1,   ///< 0.25 µs deglitch time.
    T_0_5_MICROSEC = 2,    ///< 0.5 µs deglitch time.
    T_1_MICROSEC = 3,      ///< 1.0 µs deglitch time.
    T_2_MICROSEC = 4,      ///< 2.0 µs deglitch time.
    T_4_MICROSEC = 5,      ///< 4.0 µs deglitch time.
    T_6_MICROSEC = 6,      ///< 6.0 µs deglitch time.
    T_8_MICROSEC = 7       ///< 8.0 µs deglitch time.
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ███╗   ███╗ ██████╗ ████████╗ ██████╗ ██████╗      ██████╗ ██████╗ ███╗   ██╗███████╗██╗ ██████╗              //
//    ████╗ ████║██╔═══██╗╚══██╔══╝██╔═══██╗██╔══██╗    ██╔════╝██╔═══██╗████╗  ██║██╔════╝██║██╔════╝              //
//    ██╔████╔██║██║   ██║   ██║   ██║   ██║██████╔╝    ██║     ██║   ██║██╔██╗ ██║███████╗██║██║  ███╗             //
//    ██║╚██╔╝██║██║   ██║   ██║   ██║   ██║██╔═══╝     ██║     ██║   ██║██║╚██╗██║╚════██║██║██║   ██║             //
//    ██║ ╚═╝ ██║╚██████╔╝   ██║   ╚██████╔╝██║         ╚██████╗╚██████╔╝██║ ╚████║███████║██║╚██████╔╝             //
//    ╚═╝     ╚═╝ ╚═════╝    ╚═╝    ╚═════╝ ╚═╝          ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝╚═╝ ╚═════╝              //
//                                                                                                                  //
//==================================================================================================================//
//                                               MOTOR CONFIG SECTION                                               //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @name Motor Configuration Parameters
/// @{
/**
 * @brief Parameters for configuring motor characteristics and drive settings.
 * 
 * These parameters configure the fundamental motor properties, control modes,
 * and operational settings for the TMC9660 motor controller.
 */
enum class MotorConfig : uint16_t {
    MOTOR_TYPE = 300,           ///< Motor type selection. See MotorType enum. Default: 0 (DC).
    POLE_PAIRS = 301,           ///< Number of pole pairs for BLDC/PMSM motors [1-255]. Default: 1.
    ENCODER_MODE = 302,         ///< Encoder feedback mode. See EncoderMode enum. Default: 0 (NONE).
    ENCODER_RESOLUTION = 303,   ///< Encoder resolution in counts per revolution [1-65535]. Default: 4096.
    HALL_POLARITY = 304,        ///< Hall sensor polarity. 0: NORMAL, 1: INVERTED. Default: 0.
    CONTROL_MODE = 305,         ///< Motor control mode. See ControlMode enum. Default: 0 (VOLTAGE).
    COMMUTATION_MODE = 306,     ///< Commutation mode for BLDC/PMSM. See CommutationMode enum. Default: 0 (BLOCK).
    PWM_FREQUENCY = 307,        ///< PWM frequency in Hz [1000-100000]. Default: 20000.
    CURRENT_LIMIT = 308,        ///< Maximum motor current in mA [0-65535]. Default: 2000.
    VOLTAGE_LIMIT = 309,        ///< Maximum motor voltage in mV [0-65535]. Default: 24000.
    ACCELERATION = 310,         ///< Acceleration in RPM/s [1-65535]. Default: 1000.
    DECELERATION = 311,         ///< Deceleration in RPM/s [1-65535]. Default: 1000.
    MAX_SPEED = 312,            ///< Maximum speed in RPM [1-65535]. Default: 3000.
    MIN_SPEED = 313,            ///< Minimum speed in RPM [0-65535]. Default: 0.
    DIRECTION = 314,            ///< Rotation direction. 0: FORWARD, 1: REVERSE. Default: 0.
    PHASE_OFFSET = 315,         ///< Electrical phase offset in degrees [0-359]. Default: 0.
    TORQUE_P = 316,             ///< Torque controller P coefficient [0-65535]. Default: 100.
    TORQUE_I = 317,             ///< Torque controller I coefficient [0-65535]. Default: 100.
    VELOCITY_P = 318,           ///< Velocity controller P coefficient [0-65535]. Default: 100.
    VELOCITY_I = 319,           ///< Velocity controller I coefficient [0-65535]. Default: 100.
    POSITION_P = 320,           ///< Position controller P coefficient [0-65535]. Default: 100.
    POSITION_I = 321,           ///< Position controller I coefficient [0-65535]. Default: 0.
    POSITION_D = 322            ///< Position controller D coefficient [0-65535]. Default: 0.
};
/// @}

/// @name Motor Type Selection
/// @{
/**
 * @brief Enumerates supported motor types.
 * 
 * Used with MOTOR_TYPE parameter to configure the type of motor being controlled.
 */
enum class MotorType : uint8_t {
    DC = 0,            ///< DC motor
    BLDC = 1,          ///< Brushless DC motor
    STEPPER = 2,       ///< Stepper motor
    PMSM = 3           ///< Permanent Magnet Synchronous Motor
};
/// @}

/// @name Encoder Feedback Modes
/// @{
/**
 * @brief Enumerates supported encoder and feedback modes.
 * 
 * Used with ENCODER_MODE parameter to configure position feedback.
 */
enum class EncoderMode : uint8_t {
    NONE = 0,          ///< No encoder feedback
    INCREMENTAL = 1,   ///< Incremental encoder (ABN)
    HALL = 2,          ///< Hall sensors (UVW)
    SSI = 3,           ///< SSI absolute encoder
    SPI = 4,           ///< SPI absolute encoder
    BISS = 5           ///< BiSS absolute encoder
};
/// @}

/// @name Hall Sensor Polarity
/// @{
/**
 * @brief Enumerates hall sensor polarity options.
 * 
 * Used with HALL_POLARITY parameter to configure polarity of hall sensors.
 */
enum class HallPolarity : uint8_t {
    NORMAL = 0,        ///< Normal polarity
    INVERTED = 1       ///< Inverted polarity
};
/// @}

/// @name Motor Control Modes
/// @{
/**
 * @brief Enumerates supported motor control modes.
 * 
 * Used with CONTROL_MODE parameter to configure the primary control method.
 */
enum class ControlMode : uint8_t {
    VOLTAGE = 0,       ///< Voltage mode (open-loop)
    CURRENT = 1,       ///< Current mode (torque control)
    VELOCITY = 2,      ///< Velocity mode (speed control)
    POSITION = 3       ///< Position mode (positioning)
};
/// @}

/// @name Commutation Modes
/// @{
/**
 * @brief Enumerates supported commutation strategies.
 * 
 * Used with COMMUTATION_MODE parameter to configure field-oriented control.
 */
enum class CommutationMode : uint8_t {
    BLOCK = 0,         ///< Block (trapezoidal) commutation
    SINUSOIDAL = 1,    ///< Sinusoidal commutation
    FOC = 2            ///< Field Oriented Control
};
/// @}

/// @name Direction Control
/// @{
/**
 * @brief Enumerates motor rotation directions.
 * 
 * Used with DIRECTION parameter to set motor rotation direction.
 */
enum class Direction : uint8_t {
    FORWARD = 0,       ///< Forward rotation (CCW or CW depending on wiring)
    REVERSE = 1        ///< Reverse rotation
};
/// @}


} // namespace tmc9660::tmcl
