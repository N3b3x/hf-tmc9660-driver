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

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

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
#define OP_LIST(X) \
    X(MST, 3,  /*!< Stop motor movement. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(SAP, 5,  /*!< Set Axis Parameter. TYPE: parameter, MOTOR/BANK: 0, VALUE: value. */) \
    X(GAP, 6,  /*!< Get Axis Parameter. TYPE: parameter, MOTOR/BANK: 0, VALUE: -. */) \
    X(STAP, 7, /*!< Store All Parameters. TYPE: 0xFFF, MOTOR/BANK: 0xF, VALUE: 0xFFFFFFFF. */) \
    X(SGP, 9,  /*!< Set Global Parameter. TYPE: parameter, MOTOR/BANK: 0,2,3, VALUE: value. */) \
    X(GGP, 10, /*!< Get Global Parameter. TYPE: parameter, MOTOR/BANK: -, VALUE: -. */) \
    X(RFS, 13, /*!< Reference Search. TYPE: START|STOP|STATUS, MOTOR/BANK: 0, VALUE: -. */) \
    X(SIO, 14, /*!< Set IO. TYPE: port number, MOTOR/BANK: 0, VALUE: 0,1. */) \
    X(GIO, 15, /*!< Get IO. TYPE: port number, MOTOR/BANK: 0 (digital) or 1 (analog), VALUE: -. */) \
    X(CALC, 19, /*!< Arithmetic operation. TYPE: operation, MOTOR/BANK: -, VALUE: value. */) \
    X(COMP, 20, /*!< Compare accumulator. TYPE: -, MOTOR/BANK: -, VALUE: value. */) \
    X(JC, 21,   /*!< Jump Conditional. TYPE: condition, MOTOR/BANK: -, VALUE: address. */) \
    X(JA, 22,   /*!< Jump Absolute. TYPE: -, MOTOR/BANK: -, VALUE: address. */) \
    X(CSUB, 23, /*!< Call Subroutine. TYPE: -, MOTOR/BANK: -, VALUE: address. */) \
    X(RSUB, 24, /*!< Return from Subroutine. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(EI, 25,   /*!< Enable Interrupt. TYPE: -, MOTOR/BANK: -, VALUE: interrupt number. */) \
    X(DI, 26,   /*!< Disable Interrupt. TYPE: -, MOTOR/BANK: -, VALUE: interrupt number. */) \
    X(WAIT, 27, /*!< Wait. TYPE: condition, MOTOR/BANK: -, VALUE: ticks. */) \
    X(STOP, 28, /*!< Stop Script Execution. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(CALCX, 33,/*!< Arithmetic accumulator <-> X-register. TYPE: type, MOTOR/BANK: -, VALUE: -. */) \
    X(AAP, 34,  /*!< Accumulator to Axis Parameter. TYPE: parameter, MOTOR/BANK: 0, VALUE: -. */) \
    X(AGP, 35,  /*!< Accumulator to Global Parameter. TYPE: parameter, MOTOR/BANK: 0,2,3, VALUE: -. */) \
    X(CLE, 36,  /*!< Clear Error Flag. TYPE: flag, MOTOR/BANK: -, VALUE: -. */) \
    X(VECT, 37, /*!< Define Interrupt Vector. TYPE: interrupt number, MOTOR/BANK: -, VALUE: address. */) \
    X(RETI, 38, /*!< Return from Interrupt. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(CALCVV, 40,/*!< UserVar ∘ UserVar arithmetic. TYPE: type, MOTOR/BANK: user variable 1, VALUE: user variable 2. */) \
    X(CALCVA, 41,/*!< UserVar ∘ Accumulator. TYPE: type, MOTOR/BANK: user variable, VALUE: -. */) \
    X(CALCAV, 42,/*!< Accumulator ∘ UserVar. TYPE: type, MOTOR/BANK: user variable, VALUE: -. */) \
    X(CALCVX, 43,/*!< UserVar ∘ X-register. TYPE: type, MOTOR/BANK: user variable, VALUE: -. */) \
    X(CALCXV, 44,/*!< X-register ∘ UserVar. TYPE: type, MOTOR/BANK: user variable, VALUE: -. */) \
    X(CALCV, 45, /*!< UserVar ∘ literal value. TYPE: type, MOTOR/BANK: -, VALUE: value. */) \
    X(RST, 48,  /*!< Restart script from address. TYPE: -, MOTOR/BANK: -, VALUE: address. */) \
    X(DJNZ, 49, /*!< Decrement-and-Jump if not zero. TYPE: user variable, MOTOR/BANK: -, VALUE: address. */) \
    X(SIV, 55,  /*!< Set Indexed Variable. TYPE: -, MOTOR/BANK: -, VALUE: value. */) \
    X(GIV, 56,  /*!< Get Indexed Variable. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(AIV, 57,  /*!< Accumulator to Indexed Variable. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(ApplStop, 128, /*!< Stop running TMCL program. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(ApplRun, 129,  /*!< Run/continue TMCL program. TYPE: 0 (current addr) or 1 (specified), MOTOR/BANK: -, VALUE: address. */) \
    X(ApplStep, 130, /*!< Execute single TMCL instruction. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(ApplReset, 131,/*!< Reset program counter. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(DownloadStart, 132, /*!< Enter download (script upload) mode. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(DownloadEnd, 133,   /*!< Leave download mode. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(ReadMem, 134,       /*!< Read script word at address. TYPE: -, MOTOR/BANK: -, VALUE: address. */) \
    X(GetStatusScript, 135, /*!< Get script status. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(GetVersion, 136,    /*!< Get firmware version string. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(FactoryDefault, 137,/*!< Erase stored config & reset. TYPE: -, MOTOR/BANK: -, VALUE: -. */) \
    X(Breakpoint, 141,    /*!< Manage breakpoints. TYPE: 0 (add), 1 (del), 2 (del all), 3 (get max), MOTOR/BANK: -, VALUE: address. */) \
    X(RamDebug, 142,      /*!< RAMDebug Control. TYPE/MOTOR/BANK/VALUE: see Table 16. */) \
    X(GetInfo, 157,       /*!< Generic info (ID, version, etc.). TYPE: 0 (ID), 1 (Version), MOTOR/BANK: -, VALUE: -. */) \
    X(Boot, 242,          /*!< Exit to bootloader. TYPE: 0x81, MOTOR/BANK: 0x92, VALUE: 0xA3B4C5D6. */)

enum class Op : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    OP_LIST(X)
    #undef X
};

inline const char* to_string(Op op) {
    switch(op) {
        #define X(NAME, VALUE, DOC) case Op::NAME: return #NAME;
        OP_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef OP_LIST

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
#define REPLY_CODE_LIST(X) \
    X(REPLY_OK, 100,                   /*!< Command executed successfully. */) \
    X(REPLY_CMD_LOADED, 101,           /*!< Command loaded successfully. */) \
    X(REPLY_CHKERR, 1,                 /*!< Check error occurred (e.g., checksum error). */) \
    X(REPLY_INVALID_CMD, 2,            /*!< Invalid command received (unknown command number). */) \
    X(REPLY_WRONG_TYPE, 3,             /*!< Wrong type of data received (TYPE field invalid). */) \
    X(REPLY_INVALID_VALUE, 4,          /*!< Invalid value received (VALUE field out of range). */) \
    X(REPLY_CMD_NOT_AVAILABLE, 6,      /*!< Command not available (not supported in this mode). */) \
    X(REPLY_CMD_LOAD_ERROR, 7,         /*!< Error occurred while loading command (storage error). */) \
    X(REPLY_MAX_EXCEEDED, 9,           /*!< Maximum limit exceeded (e.g., too many breakpoints). */) \
    X(REPLY_DOWNLOAD_NOT_POSSIBLE, 10, /*!< Download operation not possible (misuse or memory full). */)

enum class ReplyCode : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    REPLY_CODE_LIST(X)
    #undef X
};

inline const char* to_string(ReplyCode rc) {
    switch(rc) {
        #define X(NAME, VALUE, DOC) case ReplyCode::NAME: return #NAME;
        REPLY_CODE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef REPLY_CODE_LIST

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
 *     1   | SET_SAMPLE_COUNT    | -                                               | Number                                                                | Set total number of samples to collect (not per-channel).
 *     3   | SET_PRESCALER       | -                                               | Prescale                                                              | Set divider for sampling rate (divider = VALUE+1).
 *     4   | SET_CHANNEL         | Type: 0=Disabled, 1=Parameter, 3=Global param   | Motor/Bank: 0xFF000000, AP/GP number: 0x00000FFF                      | Configure capture channel.
 *     5   | SET_TRIGGER_CHANNEL | Type: 0=Disabled, 1=Parameter, 3=Global param   | Motor/Bank: 0xFF000000, AP/GP number: 0x00000FFF                      | Specify source of trigger data.
 *     6   | SET_TRIGGER_MASK_SHIFT| Shift                                         | Mask                                                                  | Specify mask and shift for trigger value.
 *     7   | ENABLE_TRIGGER      | Type: 0=Uncond, 1=RiseS, 2=FallS, 3=DualS,      | Threshold                                                             | Start measurement by enabling trigger.
 *         |                     | 4=RiseU, 5=FallU, 6=BothU                       |                                                                       |
 *     8   | GET_STATE           | -                                               | -                                                                     | Request state of RAMDebug (see RamDebugState).
 *     9   | READ_SAMPLE         | Index                                           | -                                                                     | Download sampled values at index.
 *    10   | GET_INFO            | -                                               | 0=Max channels, 1=Buffer size, 2=RAMDebug freq, 3=Captured count,     | Read general info.
 *         |                     |                                                 | 4=Prescaler on trigger                                                |
 *    11   | GET_CHANNEL_TYPE    | Index                                           | -                                                                     | Read channel type info.
 *    12   | GET_CHANNEL_ADDRESS | Index                                           | -                                                                     | Read channel address.
 *    13   | SET_PRETRIGGER_COUNT| -                                               | Number                                                                | Set total number of pretrigger samples (not per-channel).
 *    14   | GET_PRETRIGGER_COUNT| -                                               | -                                                                     | Get total number of pretrigger samples.
 */
#define RAMDEBUG_TYPE_LIST(X) \
    X(INITIALISE_RESET, 0,       /*!< Initialize and reset RAMDebug configuration & buffers. */) \
    X(SET_SAMPLE_COUNT, 1,       /*!< VALUE: Number of samples to collect in total (not per-channel). */) \
    X(SET_PRESCALER, 3,          /*!< VALUE: Prescale value. Sets divider for sampling rate (divider = VALUE+1). */) \
    X(SET_CHANNEL, 4,            /*!< Configure capture channel. TYPE: 0=Disabled, 1=Parameter, 3=Global parameter. MOTOR/BANK: 0xFF000000. VALUE: AP/GP number (0x00000FFF). */) \
    X(SET_TRIGGER_CHANNEL, 5,    /*!< Specify trigger source. TYPE: 0=Disabled, 1=Parameter, 3=Global parameter. MOTOR/BANK: 0xFF000000. VALUE: AP/GP number (0x00000FFF). */) \
    X(SET_TRIGGER_MASK_SHIFT, 6, /*!< Specify mask and shift for trigger value. MOTOR/BANK: Shift. VALUE: Mask. */) \
    X(ENABLE_TRIGGER, 7,         /*!< Start measurement by enabling trigger. TYPE: 0=Unconditional, 1=Rising edge signed, 2=Falling edge signed, 3=Dual edge signed, 4=Rising edge unsigned, 5=Falling edge unsigned, 6=Both edge unsigned. VALUE: Threshold. */) \
    X(GET_STATE, 8,              /*!< Request state of RAMDebug (see RamDebugState). */) \
    X(READ_SAMPLE, 9,            /*!< Download sampled values. MOTOR/BANK: Index. */) \
    X(GET_INFO, 10,              /*!< Read general info. VALUE: 0=Max channels, 1=Buffer size, 2=RAMDebug frequency, 3=Captured sample count, 4=Prescaler value on trigger event. */) \
    X(GET_CHANNEL_TYPE, 11,      /*!< Read channel type info. MOTOR/BANK: Index. */) \
    X(GET_CHANNEL_ADDRESS, 12,   /*!< Read channel address. MOTOR/BANK: Index. */) \
    X(SET_PRETRIGGER_COUNT, 13,  /*!< Set total number of pretrigger samples (not per-channel). VALUE: Number of samples. */) \
    X(GET_PRETRIGGER_COUNT, 14,  /*!< Get total number of pretrigger samples. */)

enum class RamDebugType : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    RAMDEBUG_TYPE_LIST(X)
    #undef X
};

inline const char* to_string(RamDebugType t) {
    switch(t) {
        #define X(NAME, VALUE, DOC) case RamDebugType::NAME: return #NAME;
        RAMDEBUG_TYPE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef RAMDEBUG_TYPE_LIST

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
#define RAMDEBUG_STATE_LIST(X) \
    X(IDLE, 0,       /*!< Idle: RAMDebug is not running and can be configured. Use type code 0 to enter this state. */) \
    X(TRIGGER, 1,    /*!< Trigger: Waiting for the trigger event. Ensure this state before updating trigger value. */) \
    X(CAPTURE, 2,    /*!< Capture: RAMDebug has been triggered and is capturing samples. */) \
    X(COMPLETE, 3,   /*!< Complete: Finished capturing samples. Data can be downloaded using type code 9. */) \
    X(PRETRIGGER, 4, /*!< Pretrigger: Capturing samples for the pretrigger. */)

enum class RamDebugState : std::uint8_t {
    #define X(NAME, VALUE, DOC) NAME = VALUE DOC,
    RAMDEBUG_STATE_LIST(X)
    #undef X
};

inline const char* to_string(RamDebugState s) {
    switch(s) {
        #define X(NAME, VALUE, DOC) case RamDebugState::NAME: return #NAME;
        RAMDEBUG_STATE_LIST(X)
        #undef X
        default: return "UNKNOWN";
    }
}
#undef RAMDEBUG_STATE_LIST


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//     ██████╗ ██╗      ██████╗ ██████╗  █████╗ ██╗         ██████╗  █████╗ ██████╗  █████╗ ███╗   ███╗███████╗    //
//    ██╔════╝ ██║     ██╔═══██╗██╔══██╗██╔══██╗██║         ██╔══██╗██╔══██╗██╔══██╗██╔══██╗████╗ ████║██╔════╝    //
//    ██║  ███╗██║     ██║   ██║██████╔╝███████║██║         ██████╔╝███████║██████╔╝███████║██╔████╔██║███████╗    //
//    ██║   ██║██║     ██║   ██║██╔══██╗██╔══██║██║         ██╔═══╝ ██╔══██║██╔══██╗██╔══██║██║╚██╔╝██║╚════██║    //
//    ╚██████╔╝███████╗╚██████╔╝██████╔╝██║  ██║███████╗    ██║     ██║  ██║██║  ██║██║  ██║██║ ╚═╝ ██║███████║    //
//     ╚═════╝ ╚══════╝ ╚═════╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝    ╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚══════╝    //
//                                                                                                                  //
//==================================================================================================================//
//                                            GLOBAL PARAMETERS SECTION                                             //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Global Parameters – Bank 0 (System Settings)
/// @{
/**
 * @brief Non-motion parameters in bank 0: communication, I/O, heartbeat, hibernation, loops, auto-start, etc.
 *
 * To persist changes, use STAP after setting RWE parameters.
 */
enum class GlobalParamBank0 : uint16_t {
    SERIAL_ADDRESS                              =   1, ///< RS485/UART module address [1…255 odd]. Default: 1. RWE
    SERIAL_HOST_ADDRESS                         =   2, ///< RS485/UART host address [1…255]. Default: 2. RWE
    HEARTBEAT_MONITORING_CONFIG                 =   3, ///< 0: DISABLED, 1: UART, 2: SPI, 3: UART+SPI. Default: 0. RWE
    HEARTBEAT_MONITORING_TIMEOUT                =   4, ///< Heartbeat timeout [ms] [1…4294967295]. Default: 100. RWE
    IO_DIRECTION_MASK                           =   5, ///< GPIO direction mask [bit=1→output]. Default: 0. RWE
    IO_INPUT_PULLUP_PULLDOWN_ENABLE_MASK        =   6, ///< GPIO pull-enable mask [bit=1→pull enabled]. Default: 0. RWE
    IO_INPUT_PULLUP_PULLDOWN_DIRECTION_MASK     =   7, ///< GPIO pull-dir mask [bit=1→pull-up]. Default: 0. RWE
    WAKE_PIN_CONTROL_ENABLE                     =  10, ///< 0: DISABLED, 1: ENABLED. Default: 0. RWE
    GO_TO_TIMEOUT_POWER_DOWN_STATE              =  11, ///< See PowerDownTimeout. Default: 0. W
    MAIN_LOOPS                                  =  12, ///< Main loops/sec [0…4294967295]. Default: 0. R
    TORQUE_LOOPS                                =  13, ///< Torque loops/sec [0…4294967295]. Default: 0. R
    VELOCITY_LOOPS                              =  14, ///< Velocity loops/sec [0…4294967295]. Default: 0. R
    AUTO_START_ENABLE                           =  77, ///< 0: DISABLED, 1: ENABLED. Default: 1. RWE
    CLEAR_USER_VARIABLES                        =  85  ///< 0: TRY_LOAD_FROM_STORAGE, 1: CLEAR. Default: 0. RWE
};
/// @}

/// @name Global Parameters – Bank 2 (User Variables)
/// @{
/**
 * @brief User-script variables 0…15. RWE.
 */
enum class GlobalParamBank2 : uint16_t {
    USER_VARIABLE_0  =   0,
    USER_VARIABLE_1  =   1,
    USER_VARIABLE_2  =   2,
    USER_VARIABLE_3  =   3,
    USER_VARIABLE_4  =   4,
    USER_VARIABLE_5  =   5,
    USER_VARIABLE_6  =   6,
    USER_VARIABLE_7  =   7,
    USER_VARIABLE_8  =   8,
    USER_VARIABLE_9  =   9,
    USER_VARIABLE_10 =  10,
    USER_VARIABLE_11 =  11,
    USER_VARIABLE_12 =  12,
    USER_VARIABLE_13 =  13,
    USER_VARIABLE_14 =  14,
    USER_VARIABLE_15 =  15
};
/// @}

/// @name Global Parameters – Bank 3 (Interrupt & Trigger Configuration)
/// @{
/**
 * @brief Timer periods and input-trigger transitions for scripting interrupts.
 */
enum class GlobalParamBank3 : uint16_t {
    TIMER_0_PERIOD                  =   0, ///< [ms] 0…2147483647. R/W
    TIMER_1_PERIOD                  =   1, ///< [ms] 0…2147483647. R/W
    TIMER_2_PERIOD                  =   2, ///< [ms] 0…2147483647. R/W
    STOP_LEFT_TRIGGER_TRANSITION    =  10, ///< See TriggerTransition. Default: 0. R/W
    STOP_RIGHT_TRIGGER_TRANSITION   =  11, ///< See TriggerTransition. Default: 0. R/W
    HOME_RIGHT_TRIGGER_TRANSITION   =  12, ///< See TriggerTransition. Default: 0. R/W
    INPUT_0_TRIGGER_TRANSITION      =  13, ///< See TriggerTransition. Default: 0. R/W
    INPUT_1_TRIGGER_TRANSITION      =  14, ///< See TriggerTransition. Default: 0. R/W
    INPUT_2_TRIGGER_TRANSITION      =  15, ///< See TriggerTransition. Default: 0. R/W
    INPUT_3_TRIGGER_TRANSITION      =  16, ///< See TriggerTransition. Default: 0. R/W
    INPUT_4_TRIGGER_TRANSITION      =  17, ///< See TriggerTransition. Default: 0. R/W
    INPUT_5_TRIGGER_TRANSITION      =  18, ///< See TriggerTransition. Default: 0. R/W
    INPUT_6_TRIGGER_TRANSITION      =  19, ///< See TriggerTransition. Default: 0. R/W
    INPUT_7_TRIGGER_TRANSITION      =  20, ///< See TriggerTransition. Default: 0. R/W
    INPUT_8_TRIGGER_TRANSITION      =  21, ///< See TriggerTransition. Default: 0. R/W
    INPUT_9_TRIGGER_TRANSITION      =  22, ///< See TriggerTransition. Default: 0. R/W
    INPUT_10_TRIGGER_TRANSITION     =  23, ///< See TriggerTransition. Default: 0. R/W
    INPUT_11_TRIGGER_TRANSITION     =  24, ///< See TriggerTransition. Default: 0. R/W
    INPUT_12_TRIGGER_TRANSITION     =  25, ///< See TriggerTransition. Default: 0. R/W
    INPUT_13_TRIGGER_TRANSITION     =  26, ///< See TriggerTransition. Default: 0. R/W
    INPUT_14_TRIGGER_TRANSITION     =  27, ///< See TriggerTransition. Default: 0. R/W
    INPUT_15_TRIGGER_TRANSITION     =  28, ///< See TriggerTransition. Default: 0. R/W
    INPUT_16_TRIGGER_TRANSITION     =  29, ///< See TriggerTransition. Default: 0. R/W
    INPUT_17_TRIGGER_TRANSITION     =  30, ///< See TriggerTransition. Default: 0. R/W
    INPUT_18_TRIGGER_TRANSITION     =  31  ///< See TriggerTransition. Default: 0. R/W
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Trigger Transition Options
/// @{
/**
 * @brief For all “_TRIGGER_TRANSITION” params: 0=OFF, 1=RISING, 2=FALLING, 3=BOTH.
 */
enum class TriggerTransition : uint8_t {
    OFF     = 0,
    RISING  = 1,
    FALLING = 2,
    BOTH    = 3
};
/// @}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//                              ██████╗  █████╗ ██████╗  █████╗ ███╗   ███╗███████╗                                 //
//                              ██╔══██╗██╔══██╗██╔══██╗██╔══██╗████╗ ████║██╔════╝                                 //
//                              ██████╔╝███████║██████╔╝███████║██╔████╔██║███████╗                                 //
//                              ██╔═══╝ ██╔══██║██╔══██╗██╔══██║██║╚██╔╝██║╚════██║                                 //
//                              ██║     ██║  ██║██║  ██║██║  ██║██║ ╚═╝ ██║███████║                                 //
//                              ╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚══════╝                                 //
//                                                                                                                  //
//==================================================================================================================//
//                                                  PARAMETERS                                                      //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


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

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

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

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

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

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////


/// @name Gate Driver Overcurrent Protection Parameters
/// @{
/**
 * @brief Parameters for configuring gate driver overcurrent protection.
 * 
 * These parameters configure the overcurrent protection thresholds, blanking times,
 * deglitch times, and enable/disable settings for the UVW and Y2 phases of the TMC9660 gate driver.
 */
enum class OvercurrentProtection : uint16_t {
    UVW_LOW_SIDE_ENABLE = 254,  ///< Enable overcurrent protection for UVW phases low side. 0: DISABLED, 1: ENABLED. Default: 1.
    UVW_HIGH_SIDE_ENABLE = 255, ///< Enable overcurrent protection for UVW phases high side. 0: DISABLED, 1: ENABLED. Default: 1.
    Y2_LOW_SIDE_ENABLE = 256,   ///< Enable overcurrent protection for Y2 phase low side. 0: DISABLED, 1: ENABLED. Default: 1.
    Y2_HIGH_SIDE_ENABLE = 257,  ///< Enable overcurrent protection for Y2 phase high side. 0: DISABLED, 1: ENABLED. Default: 1.
    UVW_LOW_SIDE_THRESHOLD = 258,///< Threshold for UVW phases low side [0-15]. Default: 0.
    UVW_HIGH_SIDE_THRESHOLD = 259,///< Threshold for UVW phases high side [0-15]. Default: 0.
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

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Overcurrent Protection Enable/Disable
/// @{
/**
 * @brief Enumerates options for enabling or disabling overcurrent protection.
 */
enum class OvercurrentEnable : uint8_t {
    DISABLED = 0, ///< Protection disabled.
    ENABLED = 1   ///< Protection enabled.
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

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

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

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

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
/// @}

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

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

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

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

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
//    ██╔████╔██║██║   ██║   ██║   ██║   ██║██████╔╝    ██║     ██║   ██║██╔██╗ ██║█████╗  ██║██║  ███╗             //
//    ██║╚██╔╝██║██║   ██║   ██║   ██║   ██║██╔══██╗    ██║     ██║   ██║██║╚██╗██║██╔══╝  ██║██║   ██║             //
//    ██║ ╚═╝ ██║╚██████╔╝   ██║   ╚██████╔╝██║  ██║    ╚██████╗╚██████╔╝██║ ╚████║██║     ██║╚██████╔╝             //
//    ╚═╝     ╚═╝ ╚═════╝    ╚═╝    ╚═════╝ ╚═╝  ╚═╝     ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝     ╚═╝ ╚═════╝              //
//                                                                                                                  //
//==================================================================================================================//
//                                               MOTOR CONFIG SECTION                                               //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Motor Configuration Parameters
/// @{
/**
 * @brief Parameters for configuring motor characteristics and drive settings.
 */
enum class MotorConfig : uint16_t {
    MOTOR_TYPE = 0,                ///< Motor type selection. See MotorType enum. Default: 0 (NO_MOTOR). RWE
    MOTOR_POLE_PAIRS = 1,          ///< Number of pole pairs for motor [0-127]. Default: 1. RWE
    MOTOR_DIRECTION = 2,           ///< Motor direction. 0: NOT_INVERTED, 1: INVERTED. Default: 0. RWE
    MOTOR_PWM_FREQUENCY = 3,       ///< PWM frequency in Hz [10000-100000]. Default: 25000. RWE
    COMMUTATION_MODE = 4,          ///< Motor commutation mode. See CommutationMode enum. Default: 0 (SYSTEM_OFF). RW
    OUTPUT_VOLTAGE_LIMIT = 5,      ///< PID UQ/UD output limit for circular limiter [0-32767]. Default: 8000. RWE
    PWM_SWITCHING_SCHEME = 8     ///< PWM switching scheme. See PwmSwitchingScheme enum. Default: 1 (SVPWM). RWE
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Motor Types
/// @{
/**
 * @brief Enumerates supported motor types.
 */
enum class MotorType : uint8_t {
    DC = 0,            ///< DC motor
    BLDC = 1,          ///< Brushless DC motor
    STEPPER = 2        ///< Stepper motor
};
/// @}

/// @name Motor Direction
/// @{
/**
 * @brief Enumerates motor rotation directions.
 */
enum class MotorDirection : uint8_t {
    FORWARD = 0,       ///< Forward rotation
    REVERSE = 1        ///< Reverse rotation
};
/// @}

/// @name PWM Switching Schemes
/// @{
/**
 * @brief Enumerates PWM switching schemes.
 */
enum class PwmSwitchingScheme : uint8_t {
    STANDARD = 0,      ///< Standard modulation
    SVPWM = 1,         ///< Space Vector PWM
    FLAT_BOTTOM = 2    ///< Flat bottom modulation
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name ADC Configuration Parameters
/// @{
/**
 * @brief Parameters for configuring ADCs for motor current measurement.
 */
enum class AdcConfig : uint16_t {
    ADC_SHUNT_TYPE = 12, ///< Shunt type for ADC measurements. See AdcShuntType enum. Default: 4 (BOTTOM_SHUNTS).
    ADC_I0_RAW = 13,     ///< Raw ADC measurement for I0 shunt [-32768, 32767]. Read-only.
    ADC_I1_RAW = 14,     ///< Raw ADC measurement for I1 shunt [-32768, 32767]. Read-only.
    ADC_I2_RAW = 15,     ///< Raw ADC measurement for I2 shunt [-32768, 32767]. Read-only.
    ADC_I3_RAW = 16      ///< Raw ADC measurement for I3 shunt [-32768, 32767]. Read-only.
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name ADC Shunt Types
/// @{
/**
 * @brief Enumerates shunt types for ADC measurements.
 */
enum class AdcShuntType : uint8_t {
    INLINE_UVW = 0,      ///< Inline shunts for U, V, W phases
    INLINE_VW = 1,       ///< Inline shunts for V, W phases
    INLINE_UW = 2,       ///< Inline shunts for U, W phases
    INLINE_UV = 3,       ///< Inline shunts for U, V phases
    BOTTOM_SHUNTS = 4    ///< Bottom shunts
};
/// @}

/// @name PWM Frequency Configuration
/// @{
/**
 * @brief Enumerates recommended PWM frequencies for different motor types.
 * 
 * The optimal PWM frequency depends on the motor's winding time constant Ts = Ls/Rs.
 * For best performance, the PWM period (1/fPWM) should be at least 5x smaller than Ts.
 * Using a frequency that's too low causes high current ripple and reduced efficiency,
 * while frequencies that are too high increase switching losses.
 */
enum class PwmFrequency : uint32_t {
    STANDARD_BLDC = 25000,      ///< Standard frequency for typical BLDC motors
    STANDARD_STEPPER = 20000,   ///< Standard frequency for high-inductance stepper motors
    FAST_BLDC = 50000,          ///< For fast-spinning BLDC motors (>10,000 RPM)
    ULTRA_FAST_BLDC = 100000,   ///< For very high-speed or low-inductance BLDC motors
    MINIMUM_SILENT = 20000      ///< Minimum frequency to avoid audible switching noise
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ███████╗███████╗███████╗██████╗ ██████╗  █████╗  ██████╗██╗  ██╗                                              //
//    ██╔════╝██╔════╝██╔════╝██╔══██╗██╔══██╗██╔══██╗██╔════╝██║ ██╔╝                                              //
//    █████╗  █████╗  █████╗  ██║  ██║██████╔╝███████║██║     █████╔╝                                               //
//    ██╔══╝  ██╔══╝  ██╔══╝  ██║  ██║██╔══██╗██╔══██║██║     ██╔═██╗                                               //
//    ██║     ███████╗███████╗██████╔╝██████╔╝██║  ██║╚██████╗██║  ██╗                                              //
//    ╚══════╝╚══════╝╚══════╝╚═════╝ ╚═════╝ ╚═╝  ╚═╝ ╚═════╝╚═╝  ╚═╝                                              //
//                                                                                                                  //
//     ███████╗███████╗███╗   ██╗███████╗ ██████╗ ██████╗ ███████╗                                                  //
//     ██╔════╝██╔════╝████╗  ██║██╔════╝██╔═══██╗██╔══██╗██╔════╝                                                  //
//     ███████╗█████╗  ██╔██╗ ██║███████╗██║   ██║██████╔╝███████╗                                                  //
//     ╚════██║██╔══╝  ██║╚██╗██║╚════██║██║   ██║██╔══██╗╚════██║                                                  //
//     ███████║███████╗██║ ╚████║███████║╚██████╔╝██║  ██║███████║                                                  //
//     ╚══════╝╚══════╝╚═╝  ╚═══╝╚══════╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝                                                  //
//                                                                                                                  //
//==================================================================================================================//
//                                              FEEDBACK SENSOR SECTION                                             //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Feedback Sensor Configuration Parameters
/// @{
/**
 * @brief Parameters for configuring feedback sensors (ABN, Hall, SPI encoders).
 */
enum class FeedbackSensorConfig : uint16_t {
    // ABN Encoder Parameters
    ABN_1_PHI_E                   = 89, ///< Phi_e calculated from ABN feedback [-32768, 32767]. Read-only.
    ABN_1_STEPS                   = 90, ///< ABN encoder steps per rotation (CPR) [0, 16777215]. Default: 65536.
    ABN_1_DIRECTION               = 91, ///< ABN encoder rotation direction. 0: NOT_INVERTED, 1: INVERTED. Default: 0.
    ABN_1_INIT_METHOD             = 92, ///< ABN initialization method. See AbnInitMethod enum. Default: 0.
    ABN_1_INIT_STATE              = 93, ///< ABN initialization state. See AbnInitState enum. Read-only.
    ABN_1_INIT_DELAY              = 94, ///< Delay for forced phi_e initialization [1000, 10000 ms]. Default: 1000.
    ABN_1_INIT_VELOCITY           = 95, ///< Init velocity for N-channel offset [-200000, 200000]. Default: 5.
    ABN_1_N_CHANNEL_PHI_E_OFFSET  = 96, ///< Offset between phi_e zero and ABN index pulse [-32768, 32767]. Default: 0.
    ABN_1_N_CHANNEL_INVERTED      = 97, ///< ABN N-channel inversion. 0: ACTIVE_HIGH, 1: ACTIVE_LOW. Default: 0.
    ABN_1_N_CHANNEL_FILTERING     = 98, ///< ABN N-channel filtering. See AbnNChannelFiltering enum. Default: 0.
    ABN_1_CLEAR_ON_NEXT_NULL      = 99, ///< Clear position on next N-channel event. 0: DISABLED, 1: ENABLED. Default: 0.
    ABN_1_VALUE                   = 100, ///< Raw ABN encoder counter value [0, 16777215]. Read-only.

    // Hall Encoder Parameters
    HALL_PHI_E                    = 74, ///< Phi_e calculated from Hall feedback [-32768, 32767]. Read-only.
    HALL_SECTOR_OFFSET            = 75, ///< Hall sector offset. See HallSectorOffset enum. Default: 0.
    HALL_FILTER_LENGTH            = 76, ///< Hall signal filter length [0, 255]. Default: 0.
    HALL_POSITION_0_OFFSET        = 77, ///< Hall offset for 0-degree position [-32768, 32767]. Default: 0.
    HALL_POSITION_60_OFFSET       = 78, ///< Hall offset for 60-degree position [-32768, 32767]. Default: 10922.
    HALL_POSITION_120_OFFSET      = 79, ///< Hall offset for 120-degree position [-32768, 32767]. Default: 21845.
    HALL_POSITION_180_OFFSET      = 80, ///< Hall offset for 180-degree position [-32768, 32767]. Default: -32768.
    HALL_POSITION_240_OFFSET      = 81, ///< Hall offset for 240-degree position [-32768, 32767]. Default: -21846.
    HALL_POSITION_300_OFFSET      = 82, ///< Hall offset for 300-degree position [-32768, 32767]. Default: -10923.
    HALL_INVERT_DIRECTION         = 83, ///< Invert Hall angle direction. 0: NOT_INVERTED, 1: INVERTED. Default: 0.
    HALL_EXTRAPOLATION_ENABLE     = 84, ///< Enable Hall extrapolation. 0: DISABLED, 1: ENABLED. Default: 0.
    HALL_PHI_E_OFFSET             = 85, ///< Hall sensor mounting tolerance compensation [-32768, 32767]. Default: 0.

    // SPI Encoder Parameters
    SPI_ENCODER_CS_SETTLE_DELAY_TIME    = 181, ///< CS settle delay time [0, 6375 ns]. Default: 0.
    SPI_ENCODER_CS_IDLE_DELAY_TIME      = 182, ///< CS idle delay time between frames [0, 102 us]. Default: 0.
    SPI_ENCODER_MAIN_TRANSFER_CMD_SIZE  = 183, ///< Size of first SPI transfer frame [1, 16]. Default: 1.
    SPI_ENCODER_SECONDARY_TRANSFER_CMD_SIZE = 184, ///< Size of optional secondary SPI transfer [0, 15]. Default: 0.
    SPI_ENCODER_TRANSFER_DATA_3_0       = 185, ///< Transmit data and read received data (bytes 0-3). Default: 0.
    SPI_ENCODER_TRANSFER_DATA_7_4       = 186, ///< Transmit data and read received data (bytes 4-7). Default: 0.
    SPI_ENCODER_TRANSFER_DATA_11_8      = 187, ///< Transmit data and read received data (bytes 8-11). Default: 0.
    SPI_ENCODER_TRANSFER_DATA_15_12     = 188, ///< Transmit data and read received data (bytes 12-15). Default: 0.
    SPI_ENCODER_TRANSFER                = 189, ///< SPI interface control. See SpiEncoderTransfer enum. Default: 0.
    SPI_ENCODER_POSITION_COUNTER_MASK   = 190, ///< Mask to extract position from received data [0, 4294967295]. Default: 0.
    SPI_ENCODER_POSITION_COUNTER_SHIFT  = 191, ///< Right bit shift for position value before mask [0, 127]. Default: 0.
    SPI_ENCODER_POSITION_COUNTER_VALUE  = 192, ///< Actual SPI encoder position value [0, 4294967295]. Read-only.
    SPI_ENCODER_COMMUTATION_ANGLE       = 193, ///< Actual absolute encoder angle value [-32768, 32767]. Read-only.
    SPI_ENCODER_INITIALIZATION_METHOD   = 194, ///< Encoder initialization method. See SpiInitMethod enum. Default: 0.
    SPI_ENCODER_DIRECTION               = 195, ///< SPI encoder direction. 0: NOT_INVERTED, 1: INVERTED. Default: 0.
    SPI_ENCODER_OFFSET                  = 196, ///< Internal commutation offset [0, 4294967295]. Default: 0.
    SPI_LUT_CORRECTION_ENABLE           = 197, ///< Enable lookup table correction. 0: DISABLED, 1: ENABLED. Default: 0.
    SPI_LUT_ADDRESS_SELECT              = 198, ///< Address to read/write in lookup table [0, 255]. Default: 0.
    SPI_LUT_DATA                        = 199, ///< Data to read/write to lookup table address [-128, 127]. Default: 0.
    SPI_LUT_COMMON_SHIFT_FACTOR         = 201, ///< LUT entries are multiplied with 2^SHIFT_FACTOR [-128, 127]. Default: 0.

    // ABN 2 Encoder Parameters
    ABN_2_STEPS                         = 174, ///< ABN 2 encoder steps per rotation (CPR) [0, 16777215]. Default: 1024.
    ABN_2_DIRECTION                     = 175, ///< ABN 2 encoder rotation direction. 0: NORMAL, 1: INVERTED. Default: 0.
    ABN_2_GEAR_RATIO                    = 176, ///< ABN 2 encoder gear ratio [1, 255]. Default: 1.
    ABN_2_ENABLE                        = 177, ///< Enable ABN 2 encoder. 0: DISABLED, 1: ENABLED. Default: 0.
    ABN_2_VALUE                         = 178  ///< Raw ABN 2 encoder counter value [0, 4294967295]. Read-only.
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name ABN Initialization Methods
/// @{
/**
 * @brief Enumerates ABN encoder initialization methods.
 */
enum class AbnInitMethod : uint8_t {
    FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING = 0, ///< Force phi_e zero with active swing.
    FORCED_PHI_E_90_ZERO               = 1,  ///< Force phi_e 90-degree then zero.
    USE_HALL                           = 2,  ///< Use Hall sensor for alignment.
    USE_N_CHANNEL_OFFSET               = 3   ///< Use N-channel offset for alignment.
};
/// @}

/// @name ABN Initialization States
/// @{
/**
 * @brief Enumerates ABN encoder initialization states.
 */
enum class AbnInitState : uint8_t {
    IDLE = 0,  ///< Initialization idle.
    BUSY = 1,  ///< Initialization in progress.
    WAIT = 2,  ///< Waiting for completion.
    DONE = 3   ///< Initialization completed.
};
/// @}

/// @name ABN N-Channel Filtering Modes
/// @{
/**
 * @brief Enumerates ABN N-channel filtering modes.
 */
enum class AbnNChannelFiltering : uint8_t {
    FILTERING_OFF            = 0,  ///< No filtering.
    N_EVENT_ON_A_HIGH_B_HIGH = 1,  ///< Event on A high, B high.
    N_EVENT_ON_A_HIGH_B_LOW  = 2,  ///< Event on A high, B low.
    N_EVENT_ON_A_LOW_B_HIGH  = 3,  ///< Event on A low, B high.
    N_EVENT_ON_A_LOW_B_LOW   = 4   ///< Event on A low, B low.
};
/// @}

/// @name Hall Sector Offsets
/// @{
/**
 * @brief Enumerates Hall sector offsets.
 */
enum class HallSectorOffset : uint8_t {
    DEG_0   = 0,  ///< 0 degrees.
    DEG_60  = 1,  ///< 60 degrees.
    DEG_120 = 2,  ///< 120 degrees.
    DEG_180 = 3,  ///< 180 degrees.
    DEG_240 = 4,  ///< 240 degrees.
    DEG_300 = 5   ///< 300 degrees.
};
/// @}

/// @name SPI Encoder Transfer Modes
/// @{
/**
 * @brief Enumerates SPI encoder transfer modes.
 */
enum class SpiEncoderTransfer : uint8_t {
    OFF                             = 0, ///< SPI encoder interface off.
    TRIGGER_SINGLE_TRANSFER         = 1, ///< Trigger a single SPI transfer.
    CONTINUOUS_POSITION_COUNTER_READ = 2 ///< Continuously read position counter.
};
/// @}

/// @name SPI Encoder Initialization Methods
/// @{
/**
 * @brief Enumerates SPI encoder initialization methods.
 */
enum class SpiInitMethod : uint8_t {
    FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING = 0, ///< Force rotor into PHI_E zero with active swing.
    FORCED_PHI_E_90_ZERO                = 1, ///< Force rotor into PHI_E 90° then 0° position.
    USE_OFFSET                          = 2  ///< Use the offset value stored in SPI_ENCODER_OFFSET.
};
/// @}

/// @name Enable/Disable Settings
/// @{
/**
 * @brief Generic enable/disable settings used by various parameters.
 */
enum class EnableDisable : uint8_t {
    DISABLED = 0, ///< Feature disabled.
    ENABLED  = 1  ///< Feature enabled.
};
/// @}

/// @name Direction Settings
/// @{
/**
 * @brief Direction settings used by various encoder parameters.
 */
enum class Direction : uint8_t {
    NOT_INVERTED = 0, ///< Normal direction.
    INVERTED     = 1  ///< Inverted direction.
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//     ██████╗ ██████╗ ███╗   ███╗███╗   ███╗██╗   ██╗████████╗ █████╗ ████████╗██╗ ██████╗ ███╗   ██╗              //
//    ██╔════╝██╔═══██╗████╗ ████║████╗ ████║██║   ██║╚══██╔══╝██╔══██╗╚══██╔══╝██║██╔═══██╗████╗  ██║              //
//    ██║     ██║   ██║██╔████╔██║██╔████╔██║██║   ██║   ██║   ███████║   ██║   ██║██║   ██║██╔██╗ ██║              //
//    ██║     ██║   ██║██║╚██╔╝██║██║╚██╔╝██║██║   ██║   ██║   ██╔══██║   ██║   ██║██║   ██║██║╚██╗██║              //
//    ╚██████╗╚██████╔╝██║ ╚═╝ ██║██║ ╚═╝ ██║╚██████╔╝   ██║   ██║  ██║   ██║   ██║╚██████╔╝██║ ╚████║              //
//     ╚═════╝ ╚═════╝ ╚═╝     ╚═╝╚═╝     ╚═╝ ╚═════╝    ╚═╝   ╚═╝  ╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝              //
//                                                                                                                  //
//==================================================================================================================//
//                                             COMMUTATION MODES SECTION                                            //
//==================================================================================================================//

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Commutation Modes
/// @{
/**
 * @brief Commutation modes define how the motor shaft angle is determined and the system's state.
 *
 * To turn a motor, the commutation mode must be selected using the parameter `COMMUTATION_MODE`.
 * This specifies the feedback system used to determine the motor's shaft angle. It is also used to
 * enable/disable the system or switch to a state with shorted motor coils.
 *
 * ### Commutation Modes and Their Behavior:
 *
 * #### System Off
 * - Default state after power-on and reset events.
 * - Certain operations like changing the motor type require this mode to be active.
 * - PWM behavior is determined by the `IDLE_MOTOR_PWM_BEHAVIOR` parameter:
 *   - `1`: PWM signals are switched off, leaving the motor electrically floating (default).
 *   - `0`: PWM signals remain active.
 * - Note: This mode must be manually selected when exiting from parameter mode to the bootloader.
 *
 * #### System Off, Low-Side FETs On
 * - Charges all low-side motor FETs, shorting the motor coils to ground.
 * - Not used during normal operation.
 *
 * #### System Off, High-Side FETs On
 * - Charges all high-side motor FETs, shorting the motor coils to the supply voltage.
 * - Not used during normal operation.
 *
 * #### FOC (Openloop, Voltage Mode)
 * - Applies a constant duty cycle to the motor.
 * - Voltage is defined by the `OPENLOOP_VOLTAGE` parameter (PWM duty cycle relative to supply voltage).
 * - Intended for initial setup purposes only.
 * - Current is not limited, so configure the `OPENLOOP_VOLTAGE` parameter with care.
 *
 * #### FOC (Openloop, Current Mode)
 * - Applies a constant current to the motor.
 * - Current is defined by the `OPENLOOP_CURRENT` parameter and regulated by the flux control loop.
 * - For stepper/BLDC motors:
 *   - Commutation angle is calculated by the internal hardware ramper block.
 *   - Maximum current is limited by the `MAX_FLUX` parameter.
 *   - Requires setting a velocity target using the `TARGET_VELOCITY` parameter.
 * - For DC motors:
 *   - No velocity input is needed.
 *   - Maximum current is limited by the `MAX_TORQUE` parameter.
 *
 * #### FOC (ABN), FOC (Hall Sensor), FOC (SPI Encoder)
 * - Uses sensor feedback to calculate the motor shaft position.
 * - Feedback source must be configured before activating this mode.
 * - For stepper/BLDC motors:
 *   - Commutation angle is calculated from the selected feedback method.
 *   - Supports torque, velocity, and position control.
 *   - Torque is applied to the motor, and the flux value is assumed zero unless field weakening is active.
 *   - Maximum torque is limited by the `MAX_TORQUE` parameter.
 * - For DC motors:
 *   - Supports velocity or position control.
 *   - Maximum torque is limited by the `MAX_TORQUE` parameter.
 *
 * ### Parameters to Set Up Commutation Mode:
 * - `COMMUTATION_MODE` (Parameter ID: 4)
 *   - 0: SYSTEM_OFF
 *   - 1: SYSTEM_OFF_LOW_SIDE_FETS_ON
 *   - 2: SYSTEM_OFF_HIGH_SIDE_FETS_ON
 *   - 3: FOC_OPENLOOP_VOLTAGE_MODE
 *   - 4: FOC_OPENLOOP_CURRENT_MODE
 *   - 5: FOC_ABN
 *   - 6: FOC_HALL_SENSOR
 *   - 7: RESERVED
 *   - 8: FOC_SPI_ENC
 * - `IDLE_MOTOR_PWM_BEHAVIOR` (Parameter ID: 9)
 *   - 0: PWM_ON_WHEN_MOTOR_IDLE
 *   - 1: PWM_OFF_WHEN_MOTOR_IDLE (default)
 */
/// @}
enum class CommutationMode : uint8_t {
    SYSTEM_OFF = 0,                   ///< System off (default after power-on/reset)
    SYSTEM_OFF_LOW_SIDE_FETS_ON = 1,  ///< All low-side FETs on (coils shorted to ground)
    SYSTEM_OFF_HIGH_SIDE_FETS_ON = 2, ///< All high-side FETs on (coils shorted to supply)
    FOC_OPENLOOP_VOLTAGE_MODE = 3,    ///< Open-loop voltage mode (constant duty cycle)
    FOC_OPENLOOP_CURRENT_MODE = 4,    ///< Open-loop current mode (constant current)
    FOC_ABN = 5,                      ///< FOC with ABN encoder feedback
    FOC_HALL_SENSOR = 6,              ///< FOC with Hall sensor feedback
    RESERVED = 7,                     ///< Reserved
    FOC_SPI_ENC = 8                   ///< FOC with SPI encoder feedback
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Idle Motor PWM Behavior
/// @{
/**
 * @brief PWM behavior in commutation mode "System Off" (Parameter ID: 9).
 *
 * Configures if the PWM should be off (high-Z) or on (all motor phases same voltage) in commutation mode "System Off".
 * - 0: PWM_ON_WHEN_MOTOR_IDLE — PWM stays on when motor is idle (all phases same voltage)
 * - 1: PWM_OFF_WHEN_MOTOR_IDLE — PWM off (high-Z, motor floating) [default]
 */
enum class IdleMotorPwmBehavior : uint8_t {
    PWM_ON_WHEN_MOTOR_IDLE = 0,  ///< PWM stays on when motor is idle (all phases same voltage)
    PWM_OFF_WHEN_MOTOR_IDLE = 1  ///< PWM off (high-Z, motor floating) [default]
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ████████╗ ██████╗ ██████╗  ██████╗ ██╗   ██╗███████╗     █████╗ ███╗   ██╗██████╗                             //
//    ╚══██╔══╝██╔═══██╗██╔══██╗██╔═══██╗██║   ██║██╔════╝    ██╔══██╗████╗  ██║██╔══██╗                            //
//       ██║   ██║   ██║██████╔╝██║   ██║██║   ██║█████╗      ███████║██╔██╗ ██║██║  ██║                            //
//       ██║   ██║   ██║██╔══██╗██║▄▄ ██║██║   ██║██╔══╝      ██╔══██║██║╚██╗██║██║  ██║                            //
//       ██║   ╚██████╔╝██║  ██║╚██████╔╝╚██████╔╝███████╗    ██║  ██║██║ ╚████║██████╔╝                            //
//       ╚═╝    ╚═════╝ ╚═╝  ╚═╝ ╚══▀▀═╝  ╚═════╝ ╚══════╝    ╚═╝  ╚═╝╚═╝  ╚═══╝╚═════╝                             //
//                                                                                                                  //
//    ███████╗██╗     ██╗   ██╗██╗  ██╗     ██████╗ ██████╗ ███╗   ██╗████████╗██████╗  ██████╗ ██╗                 //
//    ██╔════╝██║     ██║   ██║╚██╗██╔╝    ██╔════╝██╔═══██╗████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██║                 //
//    █████╗  ██║     ██║   ██║ ╚███╔╝     ██║     ██║   ██║██╔██╗ ██║   ██║   ██████╔╝██║   ██║██║                 //
//    ██╔══╝  ██║     ██║   ██║ ██╔██╗     ██║     ██║   ██║██║╚██╗██║   ██║   ██╔══██╗██║   ██║██║                 //
//    ██║     ███████╗╚██████╔╝██╔╝ ██╗    ╚██████╗╚██████╔╝██║ ╚████║   ██║   ██║  ██║╚██████╔╝███████╗            //
//    ╚═╝     ╚══════╝ ╚═════╝ ╚═╝  ╚═╝     ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝            //
//                                                                                                                  //
//==================================================================================================================//
//                                  TORQUE AND FLUX CONTROL SECTION                                                 // 
//==================================================================================================================//
//==================================================================================================================//

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Torque and Flux Control Parameters
/// @{
/**
 * @brief Parameters for configuring torque and flux control loops.
 *
 * These parameters configure the PI controllers for torque and flux, target/actual values, offsets, and field weakening.
 */
enum class TorqueFluxControl : uint16_t {
    MAX_TORQUE = 6,           ///< Maximum motor torque [mA]. 0...65535. Default: 2000. RWE
    MAX_FLUX = 7,             ///< Maximum motor flux [mA]. 0...65535. Default: 2000. RWE
    TARGET_TORQUE = 104,      ///< Target torque [mA]. Write to activate torque regulation. -32768...32767. Default: 0. RW
    ACTUAL_TORQUE = 105,      ///< Actual motor torque [mA]. -32767...32768. Default: 0. R
    TARGET_FLUX = 106,        ///< Target flux [mA]. -10000...10000. Default: 0. RW
    ACTUAL_FLUX = 107,        ///< Actual motor flux [mA]. -2147483648...2147483647. Default: 0. R
    TORQUE_OFFSET = 108,      ///< Offset applied to torque value [mA]. -4700...4700. Default: 0. RW
    TORQUE_P = 109,           ///< P parameter for torque PI regulator. 0...32767. Default: 50. RWE
    TORQUE_I = 110,           ///< I parameter for torque PI regulator. 0...32767. Default: 100. RWE
    FLUX_P = 111,             ///< P parameter for flux PI regulator (if separated). 0...32767. Default: 50. RWE
    FLUX_I = 112,             ///< I parameter for flux PI regulator (if separated). 0...32767. Default: 100. RWE
    SEPARATE_TORQUE_FLUX_PI_PARAMETERS = 113, ///< Enable separate PI values for torque/flux. 0: COMBINED, 1: SEPARATED. Default: 0. RWE
    CURRENT_NORM_P = 114,     ///< P normalization format for current PI. 0: SHIFT_8_BIT, 1: SHIFT_16_BIT. Default: 0. RWE
    CURRENT_NORM_I = 115,     ///< I normalization format for current PI. 0: SHIFT_8_BIT, 1: SHIFT_16_BIT. Default: 1. RWE
    TORQUE_PI_ERROR = 116,    ///< Torque PI regulator error. -2147483648...2147483647. Default: 0. R
    FLUX_PI_ERROR = 117,      ///< Flux PI regulator error. -2147483648...2147483647. Default: 0. R
    TORQUE_PI_INTEGRATOR = 118,///< Integrated error of torque PI. -2147483648...2147483647. Default: 0. R
    FLUX_PI_INTEGRATOR = 119,  ///< Integrated error of flux PI. -2147483648...2147483647. Default: 0. R
    FLUX_OFFSET = 120,        ///< Offset applied to flux value [mA]. -4700...4700. Default: 0. RW
    FIELDWEAKENING_I = 308,   ///< I parameter for field weakening controller. 0...32767. Default: 0. RWE
    FIELDWEAKENING_VOLTAGE_THRESHOLD = 310 ///< Max voltage for field weakening. 0...32767. Default: 32767. RWE
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Torque/Flux PI Separation
/// @{
/**
 * @brief Selects if torque and flux PI controllers use separate parameters.
 */
enum class TorqueFluxPiSeparation : uint8_t {
    TORQUE_FLUX_PI_COMBINED = 0,   ///< Use same PI parameters for torque and flux
    TORQUE_FLUX_PI_SEPARATED = 1   ///< Use separate PI parameters for torque and flux
};
/// @}

/// @name Current PI Normalization Format
/// @{
/**
 * @brief Normalization format for current PI controller output.
 */
enum class CurrentPiNormalization : uint8_t {
    SHIFT_8_BIT = 0,   ///< Output shifted right by 8 bits
    SHIFT_16_BIT = 1   ///< Output shifted right by 16 bits
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██╗   ██╗███████╗██╗      ██████╗  ██████╗██╗████████╗██╗   ██╗    ███╗   ███╗ ██████╗ ██████╗ ███████╗       //
//    ██║   ██║██╔════╝██║     ██╔═══██╗██╔════╝██║╚══██╔══╝╚██╗ ██╔╝    ████╗ ████║██╔═══██╗██╔══██╗██╔════╝       //
//    ██║   ██║█████╗  ██║     ██║   ██║██║     ██║   ██║    ╚████╔╝     ██╔████╔██║██║   ██║██║  ██║█████╗         //
//    ╚██╗ ██╔╝██╔══╝  ██║     ██║   ██║██║     ██║   ██║     ╚██╔╝      ██║╚██╔╝██║██║   ██║██║  ██║██╔══╝         //
//     ╚████╔╝ ███████╗███████╗╚██████╔╝╚██████╗██║   ██║      ██║       ██║ ╚═╝ ██║╚██████╔╝██████╔╝███████╗       //
//      ╚═══╝  ╚══════╝╚══════╝ ╚═════╝  ╚═════╝╚═╝   ╚═╝      ╚═╝       ╚═╝     ╚═╝ ╚═════╝ ╚═════╝ ╚══════╝       //
//                                                                                                                  //
//==================================================================================================================//
//                                        VELOCITY MODE SECTION                                                     // 
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Velocity Mode Parameters
/// @{
/**
 * @brief Parameters for configuring velocity control and ramping.
 *
 * These parameters configure the velocity PI controller, velocity feedback, scaling, and the hardware ramper block.
 */
enum class VelocityControl : uint16_t {
    VELOCITY_SENSOR_SELECTION = 123,      ///< Feedback source for velocity PI regulator. See VelocitySensorSelection. Default: 0 (SAME_AS_COMMUTATION). RWE
    TARGET_VELOCITY = 124,                ///< Target velocity value. Write to activate velocity regulation. -134217728...134217727. Default: 0. RW
    ACTUAL_VELOCITY = 125,                ///< Actual velocity value. -2147483648...2147483647. Default: 0. R
    VELOCITY_P = 127,                     ///< P parameter for velocity PI regulator. 0...32767. Default: 800. RWE
    VELOCITY_I = 128,                     ///< I parameter for velocity PI regulator. 0...32767. Default: 1. RWE
    VELOCITY_NORM_P = 129,                ///< P normalization for velocity PI. See VelocityPiNorm. Default: 2. RWE
    VELOCITY_NORM_I = 130,                ///< I normalization for velocity PI. See VelocityPiNorm. Default: 2. RWE
    VELOCITY_PI_INTEGRATOR = 131,         ///< Integrated error of velocity PI regulator. -2147483648...2147483647. Default: 0. R
    VELOCITY_PI_ERROR = 132,              ///< Velocity PI regulator error. -2147483648...2147483647. Default: 0. R
    VELOCITY_SCALING_FACTOR = 133,        ///< Scaling factor for velocity to real-world units. 1...2047. Default: 1. RWE
    VELOCITY_LOOP_DOWNSAMPLING = 135,     ///< Downsampling factor for velocity controller. 0...127. Default: 5. RWE
    VELOCITY_METER_SWITCH_THRESHOLD = 137,///< Threshold for switching from period to frequency velocity meter. 0...134217727. Default: 2000. RWE
    VELOCITY_METER_SWITCH_HYSTERESIS = 138,///< Hysteresis for switching back to period meter. 0...65535. Default: 500. RWE
    VELOCITY_METER_MODE = 139,            ///< Currently used velocity meter mode. See VelocityMeterMode. Default: 0. R
    // Ramper block parameters
    OPENLOOP_ANGLE = 45,                  ///< Phi_e calculated by ramper hardware (openloop modes). -32768...32767. Default: 0. R
    ACCELERATION_FF_GAIN = 50,            ///< Gain for acceleration feedforward. 0...65535. Default: 8. RWE
    ACCELERATION_FF_SHIFT = 51,           ///< Shift for acceleration feedforward. See AccelerationFfShift. Default: 4. RWE
    RAMP_ENABLE = 52,                     ///< Enable acceleration/deceleration ramps. 0: DISABLED, 1: ENABLED. Default: 0. RWE
    DIRECT_VELOCITY_MODE = 53,            ///< Direct velocity control mode. 0: DISABLED, 1: ENABLED. Default: 1. RWE
    RAMP_AMAX = 54,                       ///< Max acceleration (top part of ramp). 1...8388607. Default: 1000. RWE
    RAMP_A1 = 55,                         ///< First acceleration in ramp. 1...8388607. Default: 8000. RWE
    RAMP_A2 = 56,                         ///< Second acceleration in ramp. 1...8388607. Default: 4000. RWE
    RAMP_DMAX = 57,                       ///< Max deceleration (top part of ramp). 1...8388607. Default: 1000. RWE
    RAMP_D1 = 58,                         ///< Second deceleration in ramp. 1...8388607. Default: 8000. RWE
    RAMP_D2 = 59,                         ///< First deceleration in ramp. 1...8388607. Default: 8000. RWE
    RAMP_VMAX = 60,                       ///< Max velocity of ramp. 0...134217727. Default: 134217727. RWE
    RAMP_V1 = 61,                         ///< Velocity threshold for A1/D1 to A2/D2. 0...134217727. Default: 0. RWE
    RAMP_V2 = 62,                         ///< Velocity threshold for A2/D2 to AMAX/DMAX. 0...134217727. Default: 0. RWE
    RAMP_VSTART = 63,                     ///< Start velocity of ramp. 0...8388607. Default: 0. RWE
    RAMP_VSTOP = 64,                      ///< Stop velocity of ramp. 1...8388607. Default: 1. RWE
    RAMP_TVMAX = 65,                      ///< Min time at VMAX before deceleration. 0...65535. Default: 0. RWE
    RAMP_TZEROWAIT = 66,                  ///< Wait time at end of ramp. 0...65535. Default: 0. RWE
    ACCELERATION_FEEDFORWARD_ENABLE = 67, ///< Enable acceleration feedforward. 0: DISABLED, 1: ENABLED. Default: 0. RWE
    VELOCITY_FEEDFORWARD_ENABLE = 68,     ///< Enable velocity feedforward. 0: DISABLED, 1: ENABLED. Default: 0. RWE
    RAMP_VELOCITY = 69                    ///< Target velocity calculated by ramp controller. -134217727...134217727. Default: 0. R
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Velocity Sensor Selection
/// @{
/**
 * @brief Enumerates feedback sources for velocity PI regulator.
 */
enum class VelocitySensorSelection : uint8_t {
    SAME_AS_COMMUTATION = 0, ///< Use same feedback as commutation mode.
    DIGITAL_HALL = 1,        ///< Use digital Hall sensors.
    ABN1_ENCODER = 2,        ///< Use ABN1 encoder.
    ABN2_ENCODER = 3,        ///< Use ABN2 encoder.
    SPI_ENCODER = 4          ///< Use SPI encoder.
};
/// @}

/// @name Velocity PI Normalization
/// @{
/**
 * @brief Enumerates normalization formats for velocity PI controller.
 */
enum class VelocityPiNorm : uint8_t {
    NO_SHIFT = 0,    ///< No shift.
    SHIFT_8_BIT = 1, ///< Shift right by 8 bits.
    SHIFT_16_BIT = 2,///< Shift right by 16 bits.
    SHIFT_24_BIT = 3 ///< Shift right by 24 bits.
};
/// @}

/// @name Velocity Meter Modes
/// @{
/**
 * @brief Enumerates velocity meter modes.
 */
enum class VelocityMeterMode : uint8_t {
    PERIOD_METER = 0,    ///< Period-based measurement.
    FREQUENCY_METER = 1, ///< Frequency-based measurement.
    SOFTWARE_METER = 2   ///< Software-based measurement.
};
/// @}

/// @name Acceleration Feedforward Shift
/// @{
/**
 * @brief Enumerates shift values for acceleration feedforward.
 */
enum class AccelerationFfShift : uint8_t {
    NO_SHIFT = 0,    ///< No shift.
    SHIFT_4_BIT = 1, ///< Shift right by 4 bits.
    SHIFT_8_BIT = 2, ///< Shift right by 8 bits.
    SHIFT_12_BIT = 3,///< Shift right by 12 bits.
    SHIFT_16_BIT = 4,///< Shift right by 16 bits.
    SHIFT_20_BIT = 5,///< Shift right by 20 bits.
    SHIFT_24_BIT = 6 ///< Shift right by 24 bits.
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██████╗  ██████╗ ███████╗██╗████████╗██╗ ██████╗ ███╗   ██╗    ███╗   ███╗ ██████╗ ██████╗ ███████╗           //
//    ██╔══██╗██╔═══██╗██╔════╝██║╚══██╔══╝██║██╔═══██╗████╗  ██║    ████╗ ████║██╔═══██╗██╔══██╗██╔════╝           //
//    ██████╔╝██║   ██║███████╗██║   ██║   ██║██║   ██║██╔██╗ ██║    ██╔████╔██║██║   ██║██║  ██║█████╗             //
//    ██╔═══╝ ██║   ██║╚════██║██║   ██║   ██║██║   ██║██║╚██╗██║    ██║╚██╔╝██║██║   ██║██║  ██║██╔══╝             //
//    ██║     ╚██████╔╝███████║██║   ██║   ██║╚██████╔╝██║ ╚████║    ██║ ╚═╝ ██║╚██████╔╝██████╔╝███████╗           //
//    ╚═╝      ╚═════╝ ╚══════╝╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝    ╚═╝     ╚═╝ ╚═════╝ ╚═════╝ ╚══════╝           //
//                                                                                                                  //
//==================================================================================================================//
//                                         POSITION MODE SECTION                                                    // 
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Position Mode Parameters
/// @{
/**
 * @brief Parameters for configuring position control and ramping.
 *
 * These parameters configure the position PI controller, feedback selection, scaling, and ramp generator.
 */
enum class PositionControl : uint16_t {
    POSITION_SENSOR_SELECTION = 142, ///< Feedback source for position PI regulator. See PositionSensorSelection. Default: 0 (SAME_AS_COMMUTATION). RWE
    TARGET_POSITION = 143,           ///< Target position value. Write to activate position regulation. -2147483648...2147483647. Default: 0. RW
    ACTUAL_POSITION = 144,           ///< Actual position value. -2147483648...2147483647. Default: 0. RW
    POSITION_SCALING_FACTOR = 145,   ///< Scaling factor for position to real-world units. 1024...65535. Default: 1024. RWE
    POSITION_P = 146,                ///< P parameter for position PI regulator. 0...32767. Default: 5. RWE
    POSITION_I = 147,                ///< I parameter for position PI regulator. 0...32767. Default: 0. RWE
    POSITION_NORM_P = 148,           ///< P normalization for position PI. See PositionPiNorm. Default: 1. RWE
    POSITION_NORM_I = 149,           ///< I normalization for position PI. See PositionPiNorm. Default: 1. RWE
    POSITION_PI_INTEGRATOR = 150,    ///< Integrated error of position PI regulator. -2147483648...2147483647. Default: 0. R
    POSITION_PI_ERROR = 151          ///< Error of position PI regulator. -2147483648...2147483647. Default: 0. R
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Position Sensor Selection
/// @{
/**
 * @brief Enumerates feedback sources for position PI regulator.
 */
enum class PositionSensorSelection : uint8_t {
    SAME_AS_COMMUTATION = 0, ///< Use same feedback as commutation mode.
    DIGITAL_HALL = 1,        ///< Use digital Hall sensors.
    ABN1_ENCODER = 2,        ///< Use ABN1 encoder.
    ABN2_ENCODER = 3,        ///< Use ABN2 encoder.
    SPI_ENCODER = 4          ///< Use SPI encoder.
};
/// @}

/// @name Position PI Normalization
/// @{
/**
 * @brief Enumerates normalization formats for position PI controller.
 */
enum class PositionPiNorm : uint8_t {
    NO_SHIFT = 0,    ///< No shift.
    SHIFT_8_BIT = 1, ///< Shift right by 8 bits.
    SHIFT_16_BIT = 2,///< Shift right by 16 bits.
    SHIFT_24_BIT = 3 ///< Shift right by 24 bits.
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██████╗  █████╗ ███╗   ███╗██████╗ ███████╗██████╗     ███████╗████████╗ ██████╗ ██████╗                      //
//    ██╔══██╗██╔══██╗████╗ ████║██╔══██╗██╔════╝██╔══██╗    ██╔════╝╚══██╔══╝██╔═══██╗██╔══██╗                     //
//    ██████╔╝███████║██╔████╔██║██████╔╝█████╗  ██████╔╝    ███████╗   ██║   ██║   ██║██████╔╝                     //
//    ██╔══██╗██╔══██║██║╚██╔╝██║██╔═══╝ ██╔══╝  ██╔══██╗    ╚════██║   ██║   ██║   ██║██╔═══╝                      //
//    ██║  ██║██║  ██║██║ ╚═╝ ██║██║     ███████╗██║  ██║    ███████║   ██║   ╚██████╔╝██║                          //
//    ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚═╝     ╚══════╝╚═╝  ╚═╝    ╚══════╝   ╚═╝    ╚═════╝ ╚═╝                          //
//                                                                                                                  //
//     ██████╗ ██████╗ ███╗   ██╗██████╗ ██╗████████╗██╗ ██████╗ ███╗   ██╗███████╗      ██╗                        //
//    ██╔════╝██╔═══██╗████╗  ██║██╔══██╗██║╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝     ██╔╝                        //
//    ██║     ██║   ██║██╔██╗ ██║██║  ██║██║   ██║   ██║██║   ██║██╔██╗ ██║███████╗    ██╔╝                         //
//    ██║     ██║   ██║██║╚██╗██║██║  ██║██║   ██║   ██║██║   ██║██║╚██╗██║╚════██║   ██╔╝                          //
//    ╚██████╗╚██████╔╝██║ ╚████║██████╔╝██║   ██║   ██║╚██████╔╝██║ ╚████║███████║  ██╔╝                           //
//     ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝╚═════╝ ╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝  ╚═╝                            //
//                                                                                                                  //
//    ██████╗ ███████╗███████╗███████╗██████╗ ███████╗███╗   ██╗ ██████╗███████╗                                    //
//    ██╔══██╗██╔════╝██╔════╝██╔════╝██╔══██╗██╔════╝████╗  ██║██╔════╝██╔════╝                                    //
//    ██████╔╝█████╗  █████╗  █████╗  ██████╔╝█████╗  ██╔██╗ ██║██║     █████╗                                      //
//    ██╔══██╗██╔══╝  ██╔══╝  ██╔══╝  ██╔══██╗██╔══╝  ██║╚██╗██║██║     ██╔══╝                                      //
//    ██║  ██║███████╗██║     ███████╗██║  ██║███████╗██║ ╚████║╚██████╗███████╗                                    //
//    ╚═╝  ╚═╝╚══════╝╚═╝     ╚══════╝╚═╝  ╚═╝╚══════╝╚═╝  ╚═══╝ ╚═════╝╚══════╝                                    //
//                                                                                                                  //
//    ███████╗██╗    ██╗██╗████████╗ ██████╗██╗  ██╗███████╗███████╗                                                //
//    ██╔════╝██║    ██║██║╚══██╔══╝██╔════╝██║  ██║██╔════╝██╔════╝                                                //
//    ███████╗██║ █╗ ██║██║   ██║   ██║     ███████║█████╗  ███████╗                                                //
//    ╚════██║██║███╗██║██║   ██║   ██║     ██╔══██║██╔══╝  ╚════██║                                                //
//    ███████║╚███╔███╔╝██║   ██║   ╚██████╗██║  ██║███████╗███████║                                                //
//    ╚══════╝ ╚══╝╚══╝ ╚═╝   ╚═╝    ╚═════╝╚═╝  ╚═╝╚══════╝╚══════╝                                                //
//                                                                                                                  //
//==================================================================================================================//
//                            RAMPER STOP CONDITIONS AND REFERENCE SWITCHES                                         //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Ramper Stop Condition and Reference Switch Parameters
/// @{
/**
 * @brief Parameters for configuring ramper stop conditions and reference switch behavior.
 *
 * These parameters allow stopping the ramper based on velocity/position deviation or external switch events.
 * Includes configuration for soft/hard stop, switch polarity, swapping, and latch behavior.
 */
enum class RamperStopConfig : uint16_t {
    STOP_ON_VELOCITY_DEVIATION = 134,   ///< Max velocity deviation before stop event [0...200000]. Default: 0. RW
    STOP_ON_POSITION_DEVIATION = 152,   ///< Max position deviation before stop event [0...2147483647]. Default: 0. RWE
    LATCH_POSITION = 154,               ///< Latched position at switch event [-2147483648...2147483647]. Default: 0. R
    REFERENCE_SWITCH_ENABLE = 161,      ///< Bitwise enable for stopping on reference switch. See ReferenceSwitchEnable. Default: 0. RWE
    REFERENCE_SWITCH_POLARITY_AND_SWAP = 162, ///< Bitwise config for switch polarity/swap. See ReferenceSwitchPolaritySwap. Default: 0. RWE
    REFERENCE_SWITCH_LATCH_SETTINGS = 163,    ///< Bitwise config for latch behavior. See ReferenceSwitchLatchSettings. Default: 0. RWE
    EVENT_STOP_SETTINGS = 164           ///< Bitwise config for stop conditions. See EventStopSettings. Default: 0. RWE
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Reference Switch Enable
/// @{
/**
 * @brief Bitwise enable for stopping when reference switch input is triggered.
 *
 * Bit 2: Stop on home, Bit 1: Stop on right, Bit 0: Stop on left.
 */
enum class ReferenceSwitchEnable : uint8_t {
    NO_STOP_ON_SWITCH_TRIGGERED = 0, ///< No stop on switch.
    STOP_ON_L = 1,                   ///< Stop on left switch.
    STOP_ON_R = 2,                   ///< Stop on right switch.
    STOP_ON_R_AND_L = 3,             ///< Stop on right and left switches.
    STOP_ON_H = 4,                   ///< Stop on home switch.
    STOP_ON_H_AND_L = 5,             ///< Stop on home and left switches.
    STOP_ON_H_AND_R = 6,             ///< Stop on home and right switches.
    STOP_ON_H_R_AND_L = 7            ///< Stop on home, right, and left switches.
};
/// @}

/// @name Reference Switch Polarity and Swap
/// @{
/**
 * @brief Bitwise configuration for reference switch polarity and swapping.
 *
 * Bit 3: Swap left/right, Bit 2: Invert home, Bit 1: Invert right, Bit 0: Invert left.
 */
enum class ReferenceSwitchPolaritySwap : uint8_t {
    NOT_SWAPPED_NOT_INVERTED = 0,
    L_INVERTED = 1,
    R_INVERTED = 2,
    R_AND_L_INVERTED = 3,
    H_INVERTED = 4,
    H_AND_L_INVERTED = 5,
    H_AND_R_INVERTED = 6,
    H_R_AND_L_INVERTED = 7,
    L_R_SWAPPED_L_INVERTED = 8,
    L_R_SWAPPED_R_INVERTED = 9,
    L_R_SWAPPED_R_AND_L_INVERTED = 10,
    L_R_SWAPPED_H_INVERTED = 11,
    L_R_SWAPPED_H_AND_L_INVERTED = 12,
    L_R_SWAPPED = 13,
    L_R_SWAPPED_H_AND_R_INVERTED = 14,
    L_R_SWAPPED_H_R_AND_L_INVERTED = 15
};
/// @}

/// @name Reference Switch Latch Settings
/// @{
/**
 * @brief Bitwise configuration for reference switch latch behavior.
 *
 * Bit 3: Latch on falling home, Bit 2: Latch on rising home,
 * Bit 1: Latch on falling left/right, Bit 0: Latch on rising left/right.
 */
enum class ReferenceSwitchLatchSettings : uint8_t {
    NO_TRIGGER = 0,
    L_R_RISING_EDGE = 1,
    L_R_FALLING_EDGE = 2,
    L_R_BOTH_EDGES = 3,
    H_RISING_EDGE = 4,
    H_L_R_RISING_EDGE = 5,
    H_RISING_L_R_FALLING_EDGE = 6,
    H_RISING_L_R_BOTH_EDGES = 7,
    H_FALLING_EDGE = 8,
    H_FALLING_L_R_RISING_EDGE = 9,
    H_L_R_FALLING_EDGE = 10,
    H_FALLING_L_R_BOTH_EDGES = 11,
    H_BOTH_EDGES = 12,
    H_BOTH_L_R_RISING_EDGE = 13,
    H_BOTH_L_R_FALLING_EDGE = 14,
    H_L_R_BOTH_EDGES = 15
};
/// @}

/// @name Event Stop Settings
/// @{
/**
 * @brief Bitwise configuration for stop conditions.
 *
 * Bit 2: Stop on velocity deviation, Bit 1: Stop on position deviation, Bit 0: Soft stop (ramp down).
 */
enum class EventStopSettings : uint8_t {
    DO_HARD_STOP = 0,                        ///< Hard stop on event.
    DO_SOFT_STOP = 1,                        ///< Soft stop (ramp down) on event.
    STOP_ON_POS_DEVIATION = 2,               ///< Stop on position deviation.
    STOP_ON_POS_DEVIATION_SOFT_STOP = 3,     ///< Stop on position deviation, soft stop.
    STOP_ON_VEL_DEVIATION = 4,               ///< Stop on velocity deviation.
    STOP_ON_VEL_DEVIATION_SOFT_STOP = 5,     ///< Stop on velocity deviation, soft stop.
    STOP_ON_POS_VEL_DEVIATION = 6,           ///< Stop on position or velocity deviation.
    STOP_ON_POS_VEL_DEVIATION_SOFT_STOP = 7  ///< Stop on position or velocity deviation, soft stop.
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██████╗ ██╗ ██████╗ ██╗   ██╗ █████╗ ██████╗     ███████╗██╗██╗  ████████╗███████╗██████╗                     //
//    ██╔══██╗██║██╔═══██╗██║   ██║██╔══██╗██╔══██╗    ██╔════╝██║██║  ╚══██╔══╝██╔════╝██╔══██╗                    //
//    ██████╔╝██║██║   ██║██║   ██║███████║██║  ██║    █████╗  ██║██║     ██║   █████╗  ██████╔╝                    //
//    ██╔══██╗██║██║▄▄ ██║██║   ██║██╔══██║██║  ██║    ██╔══╝  ██║██║     ██║   ██╔══╝  ██╔══██╗                    //
//    ██████╔╝██║╚██████╔╝╚██████╔╝██║  ██║██████╔╝    ██║     ██║███████╗██║   ███████╗██║  ██║                    //
//    ╚═════╝ ╚═╝ ╚══▀▀═╝  ╚═════╝ ╚═╝  ╚═╝╚═════╝     ╚═╝     ╚═╝╚══════╝╚═╝   ╚══════╝╚═╝  ╚═╝                    //
//                                                                                                                  //
//==================================================================================================================//
//                                            BIQUAD FILTER SETUP SECTION                                           //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Biquad Filter Parameters
/// @{
/**
 * @brief Parameters for configuring biquad filters for torque and velocity control loops.
 *
 * The TMC9660 Parameter Mode includes biquad filters to enhance the performance of its velocity and torque control loops.
 * The biquad filter is a second-order digital filter, operating on current and past input/output samples using configurable coefficients.
 * 
 * The filter output Y(n) is calculated as:
 *   Y(n) = X(n)*b0 + X(n-1)*b1 + X(n-2)*b2 + Y(n-1)*a1 + Y(n-2)*a2
 * where:
 *   - X(n): current input sample
 *   - X(n-1), X(n-2): previous input samples
 *   - Y(n-1), Y(n-2): previous output samples
 *   - a1, a2, b0, b1, b2: filter coefficients (Q4.20 format, 24-bit)
 *
 * The velocity biquad filter (enabled by default) filters the actual velocity used as input for the velocity controller.
 * The torque biquad filter can be enabled to filter the target torque value.
 */
enum class BiquadFilter : uint16_t {
    TARGET_TORQUE_BIQUAD_FILTER_ENABLE = 318,      ///< Enable target torque biquad filter. 0: DISABLED, 1: ENABLED. Default: 0. RWE
    TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_1 = 319,    ///< Target torque biquad filter aCoeff_1 [-2147483648, 2147483647]. Default: 0. RWE
    TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_2 = 320,    ///< Target torque biquad filter aCoeff_2 [-2147483648, 2147483647]. Default: 0. RWE
    TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_0 = 321,    ///< Target torque biquad filter bCoeff_0 [-2147483648, 2147483647]. Default: 1048576. RWE
    TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_1 = 322,    ///< Target torque biquad filter bCoeff_1 [-2147483648, 2147483647]. Default: 0. RWE
    TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_2 = 323,    ///< Target torque biquad filter bCoeff_2 [-2147483648, 2147483647]. Default: 0. RWE
    ACTUAL_VELOCITY_BIQUAD_FILTER_ENABLE = 324,    ///< Enable actual velocity biquad filter. 0: DISABLED, 1: ENABLED. Default: 1. RWE
    ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_1 = 325,  ///< Actual velocity biquad filter aCoeff_1 [-2147483648, 2147483647]. Default: 1849195. RWE
    ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_2 = 326,  ///< Actual velocity biquad filter aCoeff_2 [-2147483648, 2147483647]. Default: 15961938. RWE
    ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_0 = 327,  ///< Actual velocity biquad filter bCoeff_0 [-2147483648, 2147483647]. Default: 3665. RWE
    ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_1 = 328,  ///< Actual velocity biquad filter bCoeff_1 [-2147483648, 2147483647]. Default: 7329. RWE
    ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_2 = 329   ///< Actual velocity biquad filter bCoeff_2 [-2147483648, 2147483647]. Default: 3665. RWE
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Biquad Filter Enable/Disable
/// @{
/**
 * @brief Enumerates enable/disable options for biquad filters.
 */
enum class BiquadFilterEnable : uint8_t {
    DISABLED = 0, ///< Filter disabled.
    ENABLED  = 1  ///< Filter enabled.
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ███████╗ █████╗ ██╗   ██╗██╗  ████████╗    ██╗  ██╗ █████╗ ███╗   ██╗██████╗ ██╗     ██╗███╗   ██╗ ██████╗    //
//    ██╔════╝██╔══██╗██║   ██║██║  ╚══██╔══╝    ██║  ██║██╔══██╗████╗  ██║██╔══██╗██║     ██║████╗  ██║██╔════╝    //
//    █████╗  ███████║██║   ██║██║     ██║       ███████║███████║██╔██╗ ██║██║  ██║██║     ██║██╔██╗ ██║██║  ███╗   //
//    ██╔══╝  ██╔══██║██║   ██║██║     ██║       ██╔══██║██╔══██║██║╚██╗██║██║  ██║██║     ██║██║╚██╗██║██║   ██║   //
//    ██║     ██║  ██║╚██████╔╝███████╗██║       ██║  ██║██║  ██║██║ ╚████║██████╔╝███████╗██║██║ ╚████║╚██████╔╝   //
//    ╚═╝     ╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚═╝       ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═══╝╚═════╝ ╚══════╝╚═╝╚═╝  ╚═══╝ ╚═════╝    //
//                                                                                                                  //
//==================================================================================================================//
//                                               FAULT HANDLING SECTION                                             //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Fault Handling Parameters
/// @{
/**
 * @brief Parameters for configuring system behavior on fault conditions.
 *
 * The TMC9660 supports advanced fault handling for overtemperature, I²T, and gate driver faults.
 * The system can be configured to react with open-circuit, electrical braking, and/or mechanical braking.
 * For gate driver faults, a retry mechanism is available. The number of retries and retry behavior are configurable.
 * If all retries fail, the standard drive fault behavior is applied.
 */
enum class FaultHandling : uint16_t {
    GDRV_RETRY_BEHAVIOUR = 286,         ///< State after a gate driver fault. See GdrvRetryBehaviour. Default: 0 (OPEN_CIRCUIT). RWE
    DRIVE_FAULT_BEHAVIOUR = 287,        ///< State after all retries fail. See DriveFaultBehaviour. Default: 0 (OPEN_CIRCUIT). RWE
    FAULT_HANDLER_NUMBER_OF_RETRIES = 288 ///< Max number of retries per detected fault [0...255]. Default: 5. RWE
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Gate Driver Retry Behaviour
/// @{
/**
 * @brief System state after a gate driver fault occurs.
 */
enum class GdrvRetryBehaviour : uint8_t {
    OPEN_CIRCUIT = 0,        ///< Switch off and discharge gates; motor spins freely.
    ELECTRICAL_BRAKING = 1   ///< Switch off and, if possible, enable LS or HS gates for electrical braking.
};
/// @}

/// @name Drive Fault Behaviour
/// @{
/**
 * @brief System state after all retries fail following a fault.
 */
enum class DriveFaultBehaviour : uint8_t {
    OPEN_CIRCUIT = 0,                       ///< Switch off and discharge LS/HS gates; motor spins freely.
    ELECTRICAL_BRAKING = 1,                 ///< Switch off and, if possible, enable LS/HS gates for electrical braking.
    MECHANICAL_BRAKING_AND_OPEN_CIRCUIT = 2,///< Switch off, discharge LS/HS gates, and engage mechanical brake if configured.
    MECHANICAL_AND_ELECTRICAL_BRAKING = 3   ///< Switch off, enable LS/HS gates if possible, and engage mechanical brake if configured.
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██╗██╗████████╗    ███╗   ███╗ ██████╗ ███╗   ██╗██╗████████╗ ██████╗ ██████╗                                 //
//    ██║██║╚══██╔══╝    ████╗ ████║██╔═══██╗████╗  ██║██║╚══██╔══╝██╔═══██╗██╔══██╗                                //
//    ██║██║   ██║       ██╔████╔██║██║   ██║██╔██╗ ██║██║   ██║   ██║   ██║██████╔╝                                //
//    ██║██║   ██║       ██║╚██╔╝██║██║   ██║██║╚██╗██║██║   ██║   ██║   ██║██╔══██╗                                //
//    ██║██║   ██║       ██║ ╚═╝ ██║╚██████╔╝██║ ╚████║██║   ██║   ╚██████╔╝██║  ██║                                //
//    ╚═╝╚═╝   ╚═╝       ╚═╝     ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝                                //
//                                                                                                                  //
//==================================================================================================================//
//                                               IIT MONITOR SECTION                                                //
//==================================================================================================================//

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name IIT Monitor Parameters
/// @{
/**
 * @brief Parameters for configuring IIT (I²t) monitoring and protection.
 *
 * The IIT monitor protects against overtemperature in sustained load conditions by monitoring the total current
 * flowing into the motor (torque and flux). Two independent IIT monitoring windows are supported, each with
 * its own time constant and limit. If the squared current sum exceeds the limit, an emergency stop is triggered.
 * The behavior is defined by DRIVE_FAULT_BEHAVIOUR. Status flags indicate active protection and limit exceeded.
 *
 * - The total motor current is available via ACTUAL_TOTAL_MOTOR_CURRENT.
 * - Each window accumulates squared current over a time set by THERMAL_WINDING_TIME_CONSTANT_1/2.
 * - The sum for each window is available via IIT_SUM_1/2.
 * - Limits are set by IIT_LIMIT_1/2. If not set to max, protection is active.
 * - If the sum exceeds the limit, the corresponding IIT_EXCEEDED flag is set and the motor is stopped.
 * - Both sums can be reset with RESET_IIT_SUMS.
 * - The update rate depends on MOTOR_PWM_FREQUENCY.
 */
enum class IitMonitor : uint16_t {
    THERMAL_WINDING_TIME_CONSTANT_1 = 224, ///< Time constant for IIT window 1 [ms]. 1000...60000. Default: 3000. RWE
    IIT_LIMIT_1 = 225,                     ///< IIT limit for window 1 [A^2 x ms]. 0...4294967295. Default: 4294967295. RWE
    IIT_SUM_1 = 226,                       ///< Actual IIT sum for window 1 [A^2 x ms]. 0...4294967295. Default: 0. R
    THERMAL_WINDING_TIME_CONSTANT_2 = 227, ///< Time constant for IIT window 2 [ms]. 1000...60000. Default: 6000. RWE
    IIT_LIMIT_2 = 228,                     ///< IIT limit for window 2 [A^2 x ms]. 0...4294967295. Default: 4294967295. RWE
    IIT_SUM_2 = 229,                       ///< Actual IIT sum for window 2 [A^2 x ms]. 0...4294967295. Default: 0. R
    RESET_IIT_SUMS = 230,                  ///< Write to reset both IIT sums. 0. W
    ACTUAL_TOTAL_MOTOR_CURRENT = 231       ///< Total current through motor windings [mA]. 0...65535. Default: 0. R
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ████████╗███████╗███╗   ███╗██████╗ ███████╗██████╗  █████╗ ████████╗██╗   ██╗██████╗ ███████╗                //
//    ╚══██╔══╝██╔════╝████╗  ██║██╔══██╗██╔════╝██╔══██╗██╔══██╗╚══██╔══╝██║   ██║██╔══██╗██╔════╝                //
//       ██║   █████╗  ██╔██╗ ██║██████╔╝█████╗  ██████╔╝███████║   ██║   ██║   ██║██████╔╝█████╗                  //
//       ██║   ██╔══╝  ██║╚██╗██║██╔═══╝ ██╔══╝  ██╔══██╗██╔══██║   ██║   ██║   ██║██╔══██╗██╔══╝                  //
//       ██║   ███████╗██║ ╚████║██║     ███████╗██║  ██║██║  ██║   ██║   ╚██████╔╝██║  ██║███████╗                //
//       ╚═╝   ╚══════╝╚═╝  ╚═══╝╚═╝     ╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝╚══════╝                //
//                                                                                                                  //
//    ██████╗ ██████╗  ██████╗ ████████╗███████╗ ██████╗████████╗██╗ ██████╗ ███╗   ██╗███████╗                     //
//    ██╔══██╗██╔══██╗██╔═══██╗╚══██╔══╝██╔════╝██╔════╝╚══██╔══╝██║██╔═══██╗████╗  ██║██╔════╝                     //
//    ██████╔╝██████╔╝██║   ██║   ██║   █████╗  ██║        ██║   ██║██║   ██║██╔██╗ ██║███████╗                     //
//    ██╔═══╝ ██╔══██╗██║   ██║   ██║   ██╔══╝  ██║        ██║   ██║██║   ██║██║╚██╗██║╚════██║                     //
//    ██║     ██║  ██║╚██████╔╝   ██║   ███████╗╚██████╗   ██║   ██║╚██████╔╝██║ ╚████║███████║                     //
//    ╚═╝     ╚═╝  ╚═╝ ╚═════╝    ╚═╝   ╚══════╝ ╚═════╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚══════╝                     //
//                                                                                                                  //
//==================================================================================================================//
//                                   TEMPERATURE PROTECTIONS SECTION                                                //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Temperature Protection Parameters
/// @{
/**
 * @brief Parameters for configuring temperature protection using external and internal sensors.
 *
 * The TMC9660 integrates protections using both an external analog temperature sensor (AIN3) and an internal chip sensor.
 * - EXTERNAL_TEMPERATURE can be converted to voltage: Voltage[V] = par(EXTERNAL_TEMPERATURE) × 2.5V / (2^16 - 1)
 * - CHIP_TEMPERATURE can be converted to °C: Temperature[°C] = par(CHIP_TEMPERATURE) × 0.01615 - 268.15
 * - Warning and shutdown thresholds can be set for both sensors. Exceeding a warning threshold sets a warning flag.
 *   Exceeding a shutdown threshold initiates a motor shutdown, as defined by DRIVE_FAULT_BEHAVIOUR.
 * - To restart after shutdown, clear the corresponding error flag.
 */
enum class TemperatureProtection : uint16_t {
    EXTERNAL_TEMPERATURE = 293,                  ///< External temperature sensor value [0...65535]. R
    EXTERNAL_TEMPERATURE_SHUTDOWN_THRESHOLD = 294,///< Shutdown threshold for external temperature [0...65535]. Default: 65535. RWE
    EXTERNAL_TEMPERATURE_WARNING_THRESHOLD = 295, ///< Warning threshold for external temperature [0...65535]. Default: 65535. RWE
    CHIP_TEMPERATURE = 296,                      ///< Internal chip temperature value [0...65535]. R
    CHIP_TEMPERATURE_SHUTDOWN_THRESHOLD = 297,   ///< Shutdown threshold for chip temperature [0...65535]. Default: 65535. RWE
    CHIP_TEMPERATURE_WARNING_THRESHOLD = 298     ///< Warning threshold for chip temperature [0...65535]. Default: 65535. RWE
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██╗  ██╗███████╗ █████╗ ██████╗ ████████╗██████╗ ███████╗ █████╗ ████████╗                                    //
//    ██║  ██║██╔════╝██╔══██╗██╔══██╗╚══██╔══╝██╔══██╗██╔════╝██╔══██╗╚══██╔══╝                                    //
//    ███████║█████╗  ███████║██████╔╝   ██║   ██████╔╝█████╗  ███████║   ██║                                       //
//    ██╔══██║██╔══╝  ██╔══██║██╔══██╗   ██║   ██╔══██╗██╔══╝  ██╔══██║   ██║                                       //
//    ██║  ██║███████╗██║  ██║██║  ██║   ██║   ██████╔╝███████╗██║  ██║   ██║                                       //
//    ╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝   ╚═╝   ╚═════╝ ╚══════╝╚═╝  ╚═╝   ╚═╝                                       //
//                                                                                                                  //
//    ███╗   ███╗ ██████╗ ███╗   ██╗██╗████████╗ ██████╗ ██████╗ ██╗███╗   ██╗ ██████╗                              //
//    ████╗ ████║██╔═══██╗████╗  ██║██║╚══██╔══╝██╔═══██╗██╔══██╗██║████╗  ██║██╔════╝                              //
//    ██╔████╔██║██║   ██║██╔██╗ ██║██║   ██║   ██║   ██║██████╔╝██║██╔██╗ ██║██║  ███╗                             //
//    ██║╚██╔╝██║██║   ██║██║╚██╗██║██║   ██║   ██║   ██║██╔══██╗██║██║╚██╗██║██║   ██║                             //
//    ██║ ╚═╝ ██║╚██████╔╝██║ ╚████║██║   ██║   ╚██████╔╝██║  ██║██║██║ ╚████║╚██████╔╝                             //
//    ╚═╝     ╚═╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝╚═╝╚═╝  ╚═══╝ ╚═════╝                              //
//                                                                                                                  //
//==================================================================================================================//
//                                   HEARTBEAT MONITORING SECTION                                                   //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Heartbeat Monitoring Parameters
/// @{
/**
 * @brief Parameters for configuring heartbeat monitoring.
 *
 * The heartbeat monitor checks for regular communication on UART and/or SPI. If no datagram is received within
 * the configured timeout, the system initiates a motor shutdown (behavior defined by DRIVE_FAULT_BEHAVIOUR).
 * The status flag HEARTBEAT_STOPPED is raised.
 */
enum class HeartbeatMonitoring : uint16_t {
    HEARTBEAT_MONITORING_CONFIG = 3,   ///< Heartbeat monitoring config. 0: DISABLED, 1: UART, 2: SPI, 3: UART+SPI. Default: 0. RWE
    HEARTBEAT_MONITORING_TIMEOUT = 4   ///< Heartbeat timeout in ms [1...4294967295]. Default: 100. RWE
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Heartbeat Monitoring Config
/// @{
/**
 * @brief Enumerates heartbeat monitoring interface selection.
 */
enum class HeartbeatMonitoringConfig : uint8_t {
    DISABLED = 0,                     ///< Heartbeat monitoring disabled.
    TMCL_UART_INTERFACE = 1,          ///< Monitor TMCL UART interface.
    SPI_INTERFACE = 2,                ///< Monitor SPI interface.
    TMCL_UART_AND_SPI_INTERFACE = 3   ///< Monitor both UART and SPI.
};
/// @}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██████╗ ██████╗  █████╗ ██╗  ██╗███████╗     ██████╗██╗  ██╗ ██████╗ ██████╗ ██████╗ ███████╗██████╗          //
//    ██╔══██╗██╔══██╗██╔══██╗██║ ██╔╝██╔════╝    ██╔════╝██║  ██║██╔═══██╗██╔══██╗██╔══██╗██╔════╝██╔══██╗         //
//    ██████╔╝██████╔╝███████║█████╔╝ █████╗      ██║     ███████║██║   ██║██████╔╝██████╔╝█████╗  ██████╔╝         //
//    ██╔══██╗██╔══██╗██╔══██║██╔═██╗ ██╔══╝      ██║     ██╔══██║██║   ██║██╔═══╝ ██╔═══╝ ██╔══╝  ██╔══██╗         //
//    ██████╔╝██║  ██║██║  ██║██║  ██╗███████╗    ╚██████╗██║  ██║╚██████╔╝██║     ██║     ███████╗██║  ██║         //
//    ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝     ╚═════╝╚═╝  ╚═╝ ╚═════╝ ╚═╝     ╚═╝     ╚══════╝╚═╝  ╚═╝         //
//                                                                                                                  //
//==================================================================================================================//
//                                               BRAKE CHOPPER SECTION                                              //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Brake Chopper Parameters
/// @{
/**
 * @brief Parameters for configuring the brake chopper functionality.
 *
 * The brake chopper dissipates excess energy via an external brake resistor and MOSFET when supply voltage exceeds a set limit.
 * - Enable with BRAKE_CHOPPER_ENABLE.
 * - When supply voltage exceeds BRAKE_CHOPPER_VOLTAGE_LIMIT, the brake chopper MOSFET is activated.
 * - The MOSFET is deactivated when voltage drops below (limit - hysteresis).
 */
enum class BrakeChopper : uint16_t {
    BRAKE_CHOPPER_ENABLE = 212,           ///< Enable brake chopper. 0: DISABLED, 1: ENABLED. Default: 0. RWE
    BRAKE_CHOPPER_VOLTAGE_LIMIT = 213,    ///< Voltage limit [0.1V] to activate brake chopper. 50...1000. Default: 260. RWE
    BRAKE_CHOPPER_HYSTERESIS = 214        ///< Hysteresis [0.1V] for deactivation. 0...50. Default: 5. RWE
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ███╗   ███╗███████╗ ██████╗██╗  ██╗ █████╗ ███╗   ██╗██╗ ██████╗ █████╗ ██╗                                   //
//    ████╗ ████║██╔════╝██╔════╝██║  ██║██╔══██╗████╗  ██║██║██╔════╝██╔══██╗██║                                   //
//    ██╔████╔██║█████╗  ██║     ███████║███████║██╔██╗ ██║██║██║     ███████║██║                                   //
//    ██║╚██╔╝██║██╔══╝  ██║     ██╔══██║██╔══██║██║╚██╗██║██║██║     ██╔══██║██║                                   //
//    ██║ ╚═╝ ██║███████╗╚██████╗██║  ██║██║  ██║██║ ╚████║██║╚██████╗██║  ██║███████╗                              //
//    ╚═╝     ╚═╝╚══════╝ ╚═════╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═══╝╚═╝ ╚═════╝╚═╝  ╚═╝╚══════╝                              //
//                                                                                                                  //
//    ██████╗ ██████╗  █████╗ ██╗  ██╗███████╗                                                                      //
//    ██╔══██╗██╔══██╗██╔══██╗██║ ██╔╝██╔════╝                                                                      //
//    ██████╔╝██████╔╝███████║█████╔╝ █████╗                                                                        //
//    ██╔══██╗██╔══██╗██╔══██║██╔═██╗ ██╔══╝                                                                        //
//    ██████╔╝██║  ██║██║  ██║██║  ██╗███████╗                                                                      //
//    ╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝                                                                      //
//                                                                                                                  //
//==================================================================================================================//
//                                              MECHANICAL BRAKE SECTION                                            //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Mechanical Brake Parameters
/// @{
/**
 * @brief Parameters for controlling an external mechanical brake.
 *
 * The mechanical brake is controlled by a PWM output. Release is triggered by setting RELEASE_BRAKE.
 * - BRAKE_RELEASING_DUTY_CYCLE: PWM duty cycle during release phase [%].
 * - BRAKE_HOLDING_DUTY_CYCLE: PWM duty cycle during holding phase [%].
 * - BRAKE_RELEASING_DURATION: Duration of release phase [ms].
 * - INVERT_BRAKE_OUTPUT: Invert brake output polarity (0: NORMAL, 1: INVERTED).
 */
enum class MechanicalBrake : uint16_t {
    RELEASE_BRAKE = 216,                  ///< Release brake (apply PWM). 0: DEACTIVATED, 1: ACTIVATED. Default: 0. RWE
    BRAKE_RELEASING_DUTY_CYCLE = 217,     ///< Duty cycle [%] for releasing phase. 0...99. Default: 75. RWE
    BRAKE_HOLDING_DUTY_CYCLE = 218,       ///< Duty cycle [%] for holding phase. 0...99. Default: 11. RWE
    BRAKE_RELEASING_DURATION = 219,       ///< Duration [ms] for releasing phase. 0...65535. Default: 80. RWE
    INVERT_BRAKE_OUTPUT = 221             ///< Invert brake output. 0: NORMAL, 1: INVERTED. Default: 0. RWE
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//     █████╗ ██╗   ██╗████████╗ ██████╗ ███╗   ███╗ █████╗ ████████╗██╗ ██████╗                                    //
//    ██╔══██╗██║   ██║╚══██╔══╝██╔═══██╗████╗ ████║██╔══██╗╚══██╔══╝██║██╔════╝                                    //
//    ███████║██║   ██║   ██║   ██║   ██║██╔████╔██║███████║   ██║   ██║██║                                         //
//    ██╔══██║██║   ██║   ██║   ██║   ██║██║╚██╔╝██║██╔══██║   ██║   ██║██║                                         //
//    ██║  ██║╚██████╔╝   ██║   ╚██████╔╝██║ ╚═╝ ██║██║  ██║   ██║   ██║╚██████╗                                    //
//    ╚═╝  ╚═╝ ╚═════╝    ╚═╝    ╚═════╝ ╚═╝     ╚═╝╚═╝  ╚═╝   ╚═╝   ╚═╝ ╚═════╝                                    //
//                                                                                                                  //
//    ██╗  ██╗ ██████╗ ███╗   ███╗██╗███╗   ██╗ ██████╗                                                             //
//    ██║  ██║██╔═══██╗████╗ ████║██║████╗  ██║██╔════╝                                                             //
//    ███████║██║   ██║██╔████╔██║██║██╔██╗ ██║██║  ███╗                                                            //
//    ██╔══██║██║   ██║██║╚██╔╝██║██║██║╚██╗██║██║   ██║                                                            //
//    ██║  ██║╚██████╔╝██║ ╚═╝ ██║██║██║ ╚████║╚██████╔╝                                                            //
//    ╚═╝  ╚═╝ ╚═════╝ ╚═╝     ╚═╝╚═╝╚═╝  ╚═══╝ ╚═════╝                                                             //
//                                                                                                                  //
//==================================================================================================================//
//                                              AUTOMATIC HOMING SECTION                                            //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Reference Search (Automatic Homing) Parameters
/// @{
/**
 * @brief Parameters for configuring automatic homing/reference search routines.
 *
 * The TMC9660 supports eight different reference search patterns, configurable via REFERENCE_SWITCH_SEARCH_MODE.
 * Two speeds can be set: REFERENCE_SWITCH_SEARCH_SPEED (fast search) and REFERENCE_SWITCH_SPEED (slow, for accuracy).
 * Switch positions can be read out after search. The TMCL command RFS (13) is used to start/stop/query the search.
 */
enum class ReferenceSearch : uint16_t {
    REFERENCE_SWITCH_SEARCH_MODE = 165,   ///< Reference search mode. See ReferenceSearchMode. Default: 0. RWE
    REFERENCE_SWITCH_SEARCH_SPEED = 166,  ///< Speed for reference search [-134217728...134217727]. Default: 0. RWE
    REFERENCE_SWITCH_SPEED = 167,         ///< Lower speed for accurate switch positioning [-134217728...134217727]. Default: 0. RWE
    RIGHT_LIMIT_SWITCH_POSITION = 168,    ///< Position of right limit switch [-2147483648...2147483647]. Default: 0. R
    HOME_SWITCH_POSITION = 169,           ///< Position of home switch [-2147483648...2147483647]. Default: 0. R
    LAST_REFERENCE_POSITION = 170         ///< Last reference position [-2147483648...2147483647]. Default: 0. R
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Reference Search Modes
/// @{
/**
 * @brief Enumerates reference search (homing) patterns.
 */
enum class ReferenceSearchMode : uint8_t {
    LEFT_SWITCH = 1,                        ///< Search for left limit switch.
    RIGHT_SWITCH_LEFT_SWITCH = 2,           ///< Search right limit, then left limit switch.
    RIGHT_SWITCH_LEFT_SWITCH_BOTH_SIDES = 3,///< Right limit, then approach left limit from both sides.
    LEFT_SWITCH_BOTH_SIDES = 4,             ///< Approach left limit from both sides.
    HOME_SWITCH_NEG_DIR_LEFT_END_SWITCH = 5,///< Search home switch in negative direction, turn if left end detected.
    HOME_SWITCH_POS_DIR_RIGHT_END_SWITCH = 6,///< Search home switch in positive direction, turn if right end detected.
    HOME_SWITCH_NEG_DIR_IGNORE_END_SWITCH = 7,///< Search home switch in negative direction, ignore end switch.
    HOME_SWITCH_POS_DIR_IGNORE_END_SWITCH = 8 ///< Search home switch in positive direction, ignore end switch.
};
/// @}

/// @name Reference Search TMCL Command Types
/// @{
/**
 * @brief Enumerates TMCL RFS (13) command types for reference search.
 */
enum class ReferenceSearchCommand : uint8_t {
    START = 0,   ///< Start reference search.
    STOP = 1,    ///< Stop reference search.
    STATUS = 2   ///< Return reference search status.
};
/// @}

/// @name Reference Search Status Codes
/// @{
/**
 * @brief Enumerates status codes for reference search state machine.
 */
enum class ReferenceSearchStatus : uint8_t {
    IDLE = 0,
    START_REFERENCE_DRIVE = 1,
    START_DRIVE_TO_RIGHT_LIMIT_FAST = 2,
    WAIT_UNTIL_RIGHT_SWITCH_REACHED = 3,
    START_DRIVE_TO_LEFT_LIMIT_FAST = 4,
    WAIT_UNTIL_LEFT_SWITCH_REACHED = 5,
    DRIVE_OUT_OF_LEFT_SWITCH_SLOWLY = 6,
    WAIT_UNTIL_LEFT_SWITCH_EXITED_DRIVE_IN_AGAIN = 7,
    WAIT_UNTIL_LEFT_SWITCH_REACHED_AGAIN_DRIVE_TO_POSITION = 8,
    WAIT_UNTIL_POSITION_REACHED_SET_ZERO = 9,
    WAIT_UNTIL_SWITCH_PUSHED_AGAIN = 10,
    WAIT_UNTIL_OTHER_SIDE_SWITCH_REACHED = 11,
    RESERVED = 12,
    WAIT_UNTIL_CENTER_SWITCH_REACHED = 13,
    REFERENCE_DRIVE_FINISHED_RESTORE_SETTINGS = 14,
    STOP_REFERENCE_DRIVE = 15
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ███████╗████████╗███████╗██████╗     ██╗██████╗ ██╗██████╗                                                    //
//    ██╔════╝╚══██╔══╝██╔════╝██╔══██╗   ██╔╝██╔══██╗██║██╔══██╗                                                   //
//    ███████╗   ██║   █████╗  ██████╔╝  ██╔╝ ██║  ██║██║██████╔╝                                                   //
//    ╚════██║   ██║   ██╔══╝  ██╔═══╝  ██╔╝  ██║  ██║██║██╔══██╗                                                   //
//    ███████║   ██║   ███████╗██║     ██╔╝   ██████╔╝██║██║  ██║                                                   //
//    ╚══════╝   ╚═╝   ╚══════╝╚═╝     ╚═╝    ╚═════╝ ╚═╝╚═╝  ╚═╝                                                   //
//                                                                                                                  //
//==================================================================================================================//
//                                              STEP/DIR SECTION                                                    //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Step/Dir Interface Parameters
/// @{
/**
 * @brief Parameters for configuring the STEP/DIR target movement interface.
 *
 * The STEP/DIR interface allows position control via step and direction signals. Supports micro-stepping,
 * extrapolation, and velocity feedforward. Micro-stepping is set by STEP_DIR_STEP_DIVIDER_SHIFT (1/1 to 1/1024).
 * Extrapolation smooths movement between steps and can be limited by velocity. Timeout and correction are configurable.
 */
enum class StepDir : uint16_t {
    VELOCITY_FEEDFORWARD_ENABLE = 68,           ///< Enable velocity feedforward. 0: DISABLED, 1: ENABLED. Default: 0. RWE
    STEP_DIR_STEP_DIVIDER_SHIFT = 205,          ///< Micro-step divider shift (see StepDirStepDividerShift). Default: 0 (full step). RWE
    STEP_DIR_ENABLE = 206,                      ///< Enable STEP/DIR input. 0: DISABLED, 1: ENABLED. Default: 0. RW
    STEP_DIR_EXTRAPOLATION_ENABLE = 207,        ///< Enable extrapolation. 0: DISABLED, 1: ENABLED. Default: 0. RW
    STEP_DIR_STEP_SIGNAL_TIMEOUT_LIMIT = 208,   ///< Step signal timeout limit [ms]. 1...2000. Default: 1000. RW
    STEP_DIR_MAXIMUM_EXTRAPOLATION_VELOCITY = 209 ///< Max velocity for extrapolation [eRPM]. 0...2147483647. Default: 2147483647. RW
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Step/Dir Microstep Divider Shift
/// @{
/**
 * @brief Enumerates micro-step divider shift settings for STEP/DIR interface.
 *
 * Determines the number of micro-steps per full step (1/1 to 1/1024).
 */
enum class StepDirStepDividerShift : uint8_t {
    STEP_MODE_FULL = 0,      ///< Full step (1/1)
    STEP_MODE_HALF = 1,      ///< Half step (1/2)
    STEP_MODE_QUARTER = 2,   ///< Quarter step (1/4)
    STEP_MODE_1_8TH = 3,     ///< 1/8 step
    STEP_MODE_1_16TH = 4,    ///< 1/16 step
    STEP_MODE_1_32ND = 5,    ///< 1/32 step
    STEP_MODE_1_64TH = 6,    ///< 1/64 step
    STEP_MODE_1_128TH = 7,   ///< 1/128 step
    STEP_MODE_1_256TH = 8,   ///< 1/256 step
    STEP_MODE_1_512TH = 9,   ///< 1/512 step
    STEP_MODE_1_1024TH = 10  ///< 1/1024 step
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██╗  ██╗██╗██████╗ ███████╗██████╗ ███╗   ██╗ █████╗ ████████╗██╗ ██████╗ ███╗   ██╗                         //
//    ██║  ██║██║██╔══██╗██╔════╝██╔══██╗████╗  ██║██╔══██╗╚══██╔══╝██║██╔═══██╗████╗  ██║                         //
//    ███████║██║██████╔╝█████╗  ██████╔╝██╔██╗ ██║███████║   ██║   ██║██║   ██║██╔██╗ ██║                         //
//    ██╔══██║██║██╔══██╗██╔══╝  ██╔══██╗██║╚██╗██║██╔══██║   ██║   ██║██║   ██║██║╚██╗██║                         //
//    ██║  ██║██║██████╔╝███████╗██║  ██║██║ ╚████║██║  ██║   ██║   ██║╚██████╔╝██║ ╚████║                         //
//    ╚═╝  ╚═╝╚═╝╚═════╝ ╚══════╝╚═╝  ╚═╝╚═╝  ╚═══╝╚═╝  ╚═╝   ╚═╝   ╚═╝ ╚═════╝ ╚═╝  ╚═══╝                         //
//                                                                                                                  //
//     █████╗ ███╗   ██╗██████╗     ██╗    ██╗ █████╗ ██╗  ██╗███████╗██╗   ██╗██████╗                             //
//    ██╔══██╗████╗  ██║██╔══██╗    ██║    ██║██╔══██╗██║ ██╔╝██╔════╝██║   ██║██╔══██╗                            //
//    ███████║██╔██╗ ██║██║  ██║    ██║ █╗ ██║███████║█████╔╝ █████╗  ██║   ██║██████╔╝                            //
//    ██╔══██║██║╚██╗██║██║  ██║    ██║███╗██║██╔══██║██╔═██╗ ██╔══╝  ██║   ██║██╔═══╝                             //
//    ██║  ██║██║ ╚████║██████╔╝    ╚███╔███╔╝██║  ██║██║  ██╗███████╗╚██████╔╝██║                                 //
//    ╚═╝  ╚═╝╚═╝  ╚═══╝╚═════╝      ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝ ╚═╝                                 //
//                                                                                                                  //
//==================================================================================================================//
//                                      HIBERNATION AND WAKEUP SECTION                                              //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Hibernation and Wakeup Parameters
/// @{
/**
 * @brief Parameters for configuring hibernation (low-power) and wakeup behavior.
 *
 * The TMC9660 can be sent to a low-power hibernation state using WAKE_PIN_CONTROL_ENABLE or GO_TO_TIMEOUT_POWER_DOWN_STATE.
 * - WAKE_PIN_CONTROL_ENABLE enables the WAKE pin for entering/exiting power-down state.
 * - GO_TO_TIMEOUT_POWER_DOWN_STATE puts the device into hibernation for a predefined time.
 *   If the WAKE pin is configured, pulling it low powers down the device; pulling it high wakes it up.
 */
enum class HibernationWakeup : uint16_t {
    WAKE_PIN_CONTROL_ENABLE = 10,         ///< Enable WAKE pin control. 0: DISABLED, 1: ENABLED. Default: 0. RWE
    GO_TO_TIMEOUT_POWER_DOWN_STATE = 11   ///< Enter power-down for a predefined time. See PowerDownTimeout. Default: 0. W
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////

/// @name Power Down Timeout
/// @{
/**
 * @brief Enumerates timeout durations for power-down state.
 */
enum class PowerDownTimeout : uint8_t {
    T_250_MILLISEC = 0,   ///< 250 ms
    T_500_MILLISEC = 1,   ///< 500 ms
    T_1_SEC = 2,          ///< 1 second
    T_2_SEC = 3,          ///< 2 seconds
    T_4_SEC = 4,          ///< 4 seconds
    T_8_SEC = 5,          ///< 8 seconds
    T_16_SEC = 6,         ///< 16 seconds
    T_32_SEC = 7          ///< 32 seconds
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ██╗███╗   ██╗████████╗███████╗██████╗ ███╗   ██╗ █████╗ ██╗                                                   //
//    ██║████╗  ██║╚══██╔══╝██╔════╝██╔══██╗████╗  ██║██╔══██╗██║                                                   //
//    ██║██╔██╗ ██║   ██║   █████╗  ██████╔╝██╔██╗ ██║███████║██║                                                   //
//    ██║██║╚██╗██║   ██║   ██╔══╝  ██╔══██╗██║╚██╗██║██╔══██║██║                                                   //
//    ██║██║ ╚████║   ██║   ███████╗██║  ██║██║ ╚████║██║  ██║███████╗                                              //
//    ╚═╝╚═╝  ╚═══╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚═╝  ╚═══╝╚═╝  ╚═╝╚══════╝                                              //
//                                                                                                                  //
//    ███╗   ███╗███████╗ █████╗ ███████╗██╗   ██╗██████╗ ███████╗███╗   ███╗███████╗███╗   ██╗████████╗███████╗   //
//    ████╗ ████║██╔════╝██╔══██╗██╔════╝██║   ██║██╔══██╗██╔════╝████╗ ████║██╔════╝████╗  ██║╚══██╔══╝██╔════╝   //
//    ██╔████╔██║█████╗  ███████║███████╗██║   ██║██████╔╝█████╗  ██╔████╔██║█████╗  ██╔██╗ ██║   ██║   ███████╗   //
//    ██║╚██╔╝██║██╔══╝  ██╔══██║╚════██║██║   ██║██╔══██╗██╔══╝  ██║╚██╔╝██║██╔══╝  ██║╚██╗██║   ██║   ╚════██║   //
//    ██║ ╚═╝ ██║███████╗██║  ██║███████║╚██████╔╝██║  ██║███████╗██║ ╚═╝ ██║███████╗██║ ╚████║   ██║   ███████║   //
//    ╚═╝     ╚═╝╚══════╝╚═╝  ╚═╝╚══════╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝╚═╝     ╚═╝╚══════╝╚═╝  ╚═══╝   ╚═╝   ╚══════╝   //
//                                                                                                                  //
//==================================================================================================================//
//                                           INTERNAL MEASUREMENTS SECTION                                          //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name System Supply Parameters
/// @{
/**
 * @brief Parameters for supply voltage warnings.
 */
enum class SystemStatusSupply : uint16_t {
    SUPPLY_VOLTAGE = 290,                       ///< Actual supply voltage in 0.1 V units. Read-only.
    SUPPLY_OVERVOLTAGE_WARNING_THRESHOLD = 291, ///< Supply overvoltage warning threshold [0…1000]. RWE
    SUPPLY_UNDERVOLTAGE_WARNING_THRESHOLD = 292 ///< Supply undervoltage warning threshold [0…1000]. RWE
};
/// @}

/// @name Internal Measurement Parameters
/// @{
/**
 * @brief Raw diagnostic values and FOC internal measurements.
 * 
 * These parameters provide access to interim results and measurements from the Field-Oriented Control (FOC)
 * algorithm, as well as raw input states. They are useful for debugging and diagnostics.
 */
enum class InternalMeasurement : uint16_t {
    MCC_INPUTS_RAW                 = 304, ///< Raw inputs for ABN, hall, reference switches, driver enabled, hall filtered and ABN2 or Step/Dir [0...32767]. Read-only.
    FOC_VOLTAGE_UX                 = 305, ///< Interim result of the FOC for phase U (X in case of stepper motor) [-32768...32767]. Read-only.
    FOC_VOLTAGE_WY                 = 306, ///< Interim result of the FOC for phase W (Y in case of stepper motor) [-32768...32767]. Read-only.
    FOC_VOLTAGE_V                  = 307, ///< Interim result of the FOC for phase V (BLDC motor only) [-32768...32767]. Read-only.
    FIELDWEAKENING_I               = 308, ///< I parameter for field weakening controller [0...32767]. Default: 0. RWE
    FIELDWEAKENING_VOLTAGE_THRESHOLD = 310, ///< Maximum motor voltage allowed for field weakening [0...32767]. Default: 32767. RWE
    FOC_CURRENT_UX                 = 311, ///< Interim measurement of the FOC for phase UX [-32768...32767]. Read-only.
    FOC_CURRENT_V                  = 312, ///< Interim measurement of the FOC for phase V [-32768...32767]. Read-only.
    FOC_CURRENT_WY                 = 313, ///< Interim measurement of the FOC for phase WY [-32768...32767]. Read-only.
    FOC_VOLTAGE_UQ                 = 314, ///< Interim measurement of the FOC for Uq [-32768...32767]. Read-only.
    FOC_CURRENT_IQ                 = 315  ///< Interim measurement of the FOC for Iq [-32768...32767]. Read-only.
};
/// @}

/// @name Combined Diagnostic Values
/// @{
/**
 * @brief Simplified combined measurement registers used during tuning.
 * 
 * These parameters provide compact diagnostic values primarily used during motor tuning operations.
 * They combine multiple measurements into single 32-bit values or provide integrated measurements
 * to facilitate data collection at lower sampling rates.
 */
enum class CombinedDiagnosticValues : uint16_t {
    TORQUE_FLUX_COMBINED_TARGET_VALUES = 330, ///< Raw (unscaled) torque and flux target values combined into one 32-bit value. Used for simplified compact measurement during tuning. [0...4294967295]. Read-only.
    TORQUE_FLUX_COMBINED_ACTUAL_VALUES = 331, ///< Raw (unscaled) torque and flux actual values combined into one 32-bit value. Used for simplified compact measurement during tuning. [0...4294967295]. Read-only.
    VOLTAGE_D_Q_COMBINED_ACTUAL_VALUES = 332, ///< Raw (unscaled) voltage actual values combined into one 32-bit value. Used for simplified compact measurement during tuning. [0...4294967295]. Read-only.
    INTEGRATED_ACTUAL_TORQUE_VALUE     = 333, ///< Periodically summed up actual torque value. Used for simplified measurement with low measurement frequency during tuning operations. [0...4294967295]. Read-only.
    INTEGRATED_ACTUAL_VELOCITY_VALUE   = 334  ///< Periodically summed up actual velocity value. Used for simplified measurement with low measurement frequency during tuning operations. [0...4294967295]. Read-only.
};
/// @}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                  //
//    ███████╗██████╗ ██████╗  ██████╗ ██████╗ ███████╗         ███████╗██╗      █████╗  ██████╗ ███████╗           //
//    ██╔════╝██╔══██╗██╔══██╗██╔═══██╗██╔══██╗██╔════╝         ██╔════╝██║     ██╔══██╗██╔════╝ ██╔════╝           //
//    █████╗  ██████╔╝██████╔╝██║   ██║██████╔╝███████╗  █████╗ █████╗  ██║     ███████║██║  ███╗███████╗           //
//    ██╔══╝  ██╔══██╗██╔══██╗██║   ██║██╔══██╗╚════██║  ╚════╝ ██╔══╝  ██║     ██╔══██║██║   ██║╚════██║           //
//    ███████╗██║  ██║██║  ██║╚██████╔╝██║  ██║███████║         ██║     ███████╗██║  ██║╚██████╔╝███████║           //
//    ╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═╝╚══════╝         ╚═╝     ╚══════╝╚═╝  ╚═╝ ╚═════╝ ╚══════╝           //
//                                                                                                                  //
//==================================================================================================================//
//                                              ERRORS & FLAGS SECTION                                              //
//==================================================================================================================//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////
//    ╔═╗╔═╗╦═╗╔═╗╔╦╗╔═╗╔╦╗╔═╗╦═╗╔═╗       //
//    ╠═╝╠═╣╠╦╝╠═╣║║║║╣  ║ ║╣ ╠╦╝╚═╗       //
//    ╩  ╩ ╩╩╚═╩ ╩╩ ╩╚═╝ ╩ ╚═╝╩╚═╚═╝       //
/////////////////////////////////////////////

/// @name Temperature and Error Flags Parameters
/// @{
/**
 * @brief Parameters for temperature monitoring and error flags.
 */
enum class ErrorsAndFlags : uint16_t {
    GENERAL_STATUS_FLAGS                    = 289, ///< General status flags. See GeneralStatusFlags enum. Read-only.
    EXTERNAL_TEMPERATURE                    = 293, ///< External temperature sensor reading [0-65535]. Read-only.
    EXTERNAL_TEMPERATURE_SHUTDOWN_THRESHOLD = 294, ///< Shutdown threshold for external temperature [0-65535]. Default: 65535.
    EXTERNAL_TEMPERATURE_WARNING_THRESHOLD  = 295, ///< Warning threshold for external temperature [0-65535]. Default: 65535.
    CHIP_TEMPERATURE                        = 296, ///< Chip temperature reading [0-65535]. Read-only.
    CHIP_TEMPERATURE_SHUTDOWN_THRESHOLD     = 297, ///< Shutdown threshold for chip temperature [0-65535]. Default: 65535.
    CHIP_TEMPERATURE_WARNING_THRESHOLD      = 298, ///< Warning threshold for chip temperature [0-65535]. Default: 65535.
    GENERAL_ERROR_FLAGS                     = 299, ///< General error flags. See GeneralErrorFlags enum. Read-only.
    GDRV_ERROR_FLAGS                        = 300, ///< Gate driver error flags. See GateDriverErrorFlags enum. Read-only.
    ADC_STATUS_FLAGS                        = 301  ///< ADC status flags. See AdcStatusFlags enum. Write-to-clear.
};
/// @}

/////////////////////////////////////////////
//    ╔═╗╔╗╔╦ ╦╔╦╗╔═╗╦═╗╔═╗╔╦╗╦╔═╗╔╗╔╔═╗   //
//    ║╣ ║║║║ ║║║║║╣ ╠╦╝╠═╣ ║ ║║ ║║║║╚═╗   //
//    ╚═╝╝╚╝╚═╝╩ ╩╚═╝╩╚═╩ ╩ ╩ ╩╚═╝╝╚╝╚═╝   //
/////////////////////////////////////////////
/// @name General Status Flags
/// @{
/**
 * @brief General status flags indicating system state, events, and hardware availability.
 * 
 * This 32-bit status register contains flags that provide information about the current system state,
 * regulation modes, configuration status, hardware events, and available peripherals.
 * 
 * Most flags are read-only (R), while some can be read, written, and cleared (RWC).
 * Flags marked as RWC can be cleared by writing a 1 to the corresponding bit.
 */
enum class GeneralStatusFlags : uint32_t {
    REGULATION_STOPPED              = 0x00000001, ///< System does not regulate motion. Read-only.
    REGULATION_TORQUE               = 0x00000002, ///< System is regulating mode torque. Read-only.
    REGULATION_VELOCITY             = 0x00000004, ///< System is regulating mode velocity. Read-only.
    REGULATION_POSITION             = 0x00000008, ///< System is regulating mode position. Read-only.
    CONFIG_STORED                   = 0x00000010, ///< Config was stored successfully. Read-write-clear.
    CONFIG_LOADED                   = 0x00000020, ///< Config was loaded successfully. Read-write-clear.
    CONFIG_READ_ONLY                = 0x00000040, ///< Memory for config is read only. Read-only.
    TMCL_SCRIPT_READ_ONLY           = 0x00000080, ///< Memory for TMCL script is read only. Read-only.
    BRAKE_CHOPPER_ACTIVE            = 0x00000100, ///< Brake chopper is active. Read-only.
    POSITION_REACHED                = 0x00000200, ///< Actual velocity and target velocity are below POSITION_REACHED_THRESHOLD. Read-only.
    VELOCITY_REACHED                = 0x00000400, ///< Actual velocity and target velocity are below VELOCITY_REACHED_THRESHOLD. Read-only.
    ADC_OFFSET_CALIBRATED           = 0x00000800, ///< The ADC offset was calibrated automatically (clear to recalibrate). Read-write-clear.
    RAMPER_LATCHED                  = 0x00001000, ///< The ramper latched a position. Read-write-clear.
    RAMPER_EVENT_STOP_SWITCH        = 0x00002000, ///< Ramper had a switch stop event. Read-only.
    RAMPER_EVENT_STOP_DEVIATION     = 0x00004000, ///< Ramper had a deviation stop event. Read-write-clear.
    RAMPER_VELOCITY_REACHED         = 0x00008000, ///< The ramper reached its velocity target. Read-only.
    RAMPER_POSITION_REACHED         = 0x00010000, ///< The ramper reached its position target. Read-only.
    RAMPER_SECOND_MOVE              = 0x00020000, ///< The ramper needed a second move to reach target. Read-write-clear.
    IIT_1_ACTIVE                    = 0x00040000, ///< IIT 1 active. Read-only.
    IIT_2_ACTIVE                    = 0x00080000, ///< IIT 2 active. Read-only.
    REFSEARCH_FINISHED              = 0x00100000, ///< Reference search finished. Read-only.
    Y2_USED_FOR_BRAKING             = 0x00200000, ///< Fourth phase used for braking. Read-only.
    STEPDIR_INPUT_AVAILABLE         = 0x00800000, ///< Signals that StepDir is available. Read-only.
    RIGHT_REF_SWITCH_AVAILABLE      = 0x01000000, ///< Signals that REF_R is available. Read-only.
    HOME_REF_SWITCH_AVAILABLE       = 0x02000000, ///< Signals that REF_H is available. Read-only.
    LEFT_REF_SWITCH_AVAILABLE       = 0x04000000, ///< Signals that REF_L is available. Read-only.
    ABN2_FEEDBACK_AVAILABLE         = 0x08000000, ///< Signals that ABN2 feedback is available. Read-only.
    HALL_FEEDBACK_AVAILABLE         = 0x10000000, ///< Signals that hall feedback is available. Read-only.
    ABN1_FEEDBACK_AVAILABLE         = 0x20000000, ///< Signals that ABN1 feedback is available. Read-only.
    SPI_FLASH_AVAILABLE             = 0x40000000, ///< Signals that an external SPI flash is available. Read-only.
    I2C_EEPROM_AVAILABLE            = 0x80000000  ///< Signals that an external I2C EEPROM is available. Read-only.
};
/// @}

/// @name General Error Flags
/// @{
/**
 * @brief Enumerates general error flags for GENERAL_ERROR_FLAGS.
 * 
 * These flags indicate various error conditions in the system. Most of these flags
 * are Read-Write-Clear (RWC), meaning they can be cleared by writing a 1 to the 
 * corresponding bit position.
 */
enum class GeneralErrorFlags : uint32_t {
    CONFIG_ERROR                 = 0x00000001, ///< Verification of config storage failed. Read-only.
    TMCL_SCRIPT_ERROR            = 0x00000002, ///< TMCL Script not available. Read-only.
    HOMESWITCH_NOT_FOUND         = 0x00000004, ///< Reference search for home switch failed. Read-only.
    HALL_ERROR                   = 0x00000020, ///< Signals an invalid hall state. Read-write-clear.
    WATCHDOG_EVENT               = 0x00000200, ///< Watchdog reset indication. Read-write-clear.
    EXT_TEMP_EXCEEDED            = 0x00002000, ///< External temperature exceeded. Read-write-clear.
    CHIP_TEMP_EXCEEDED           = 0x00004000, ///< Chip temperature threshold exceeded. Read-write-clear.
    I2T_1_EXCEEDED               = 0x00010000, ///< Signals that I²t limit 1 was exceeded. Read-write-clear.
    I2T_2_EXCEEDED               = 0x00020000, ///< Signals that I²t limit 2 was exceeded. Read-write-clear.
    EXT_TEMP_WARNING             = 0x00040000, ///< External temperature warning threshold exceeded. Read-write-clear.
    SUPPLY_OVERVOLTAGE_WARNING   = 0x00080000, ///< Supply overvoltage warning threshold exceeded. Read-write-clear.
    SUPPLY_UNDERVOLTAGE_WARNING  = 0x00100000, ///< Supply voltage below undervoltage warning threshold. Read-write-clear.
    ADC_IN_OVERVOLTAGE           = 0x00200000, ///< ADC IN over 2V while ADC enabled. Read-write-clear.
    FAULT_RETRY_HAPPENED         = 0x00400000, ///< The set number of max. retries was exceeded without recovering. Read-write-clear.
    FAULT_RETRIES_FAILED         = 0x00800000, ///< All retries of a detected fault failed. Read-write-clear.
    CHIP_TEMP_WARNING            = 0x01000000, ///< Chip temperature warning threshold exceeded. Read-write-clear.
    HEARTBEAT_STOPPED            = 0x04000000  ///< Heartbeat stopped. Read-write-clear.
};
/// @}

/// @name Gate Driver Error Flags
/// @{
/**
 * @brief Enumerates gate driver error flags for GDRV_ERROR_FLAGS.
 */
enum class GateDriverErrorFlags : uint32_t {
    U_LOW_SIDE_OVERCURRENT        = 0x00000001, ///< U low side overcurrent.
    V_LOW_SIDE_OVERCURRENT        = 0x00000002, ///< V low side overcurrent.
    W_LOW_SIDE_OVERCURRENT        = 0x00000004, ///< W low side overcurrent.
    Y2_LOW_SIDE_OVERCURRENT       = 0x00000008, ///< Y2 low side overcurrent.
    U_LOW_SIDE_DISCHARGE_SHORT    = 0x00000010, ///< U low side discharge short.
    V_LOW_SIDE_DISCHARGE_SHORT    = 0x00000020, ///< V low side discharge short.
    W_LOW_SIDE_DISCHARGE_SHORT    = 0x00000040, ///< W low side discharge short.
    Y2_LOW_SIDE_DISCHARGE_SHORT   = 0x00000080, ///< Y2 low side discharge short.
    U_LOW_SIDE_CHARGE_SHORT       = 0x00000100, ///< U low side charge short.
    V_LOW_SIDE_CHARGE_SHORT       = 0x00000200, ///< V low side charge short.
    W_LOW_SIDE_CHARGE_SHORT       = 0x00000400, ///< W low side charge short.
    Y2_LOW_SIDE_CHARGE_SHORT      = 0x00000800, ///< Y2 low side charge short.
    U_BOOTSTRAP_UNDERVOLTAGE      = 0x00001000, ///< U bootstrap undervoltage.
    V_BOOTSTRAP_UNDERVOLTAGE      = 0x00002000, ///< V bootstrap undervoltage.
    W_BOOTSTRAP_UNDERVOLTAGE      = 0x00004000, ///< W bootstrap undervoltage.
    Y2_BOOTSTRAP_UNDERVOLTAGE     = 0x00008000, ///< Y2 bootstrap undervoltage.
    U_HIGH_SIDE_OVERCURRENT       = 0x00010000, ///< U high side overcurrent.
    V_HIGH_SIDE_OVERCURRENT       = 0x00020000, ///< V high side overcurrent.
    W_HIGH_SIDE_OVERCURRENT       = 0x00040000, ///< W high side overcurrent.
    Y2_HIGH_SIDE_OVERCURRENT      = 0x00080000, ///< Y2 high side overcurrent.
    U_HIGH_SIDE_DISCHARGE_SHORT   = 0x00100000, ///< U high side discharge short.
    V_HIGH_SIDE_DISCHARGE_SHORT   = 0x00200000, ///< V high side discharge short.
    W_HIGH_SIDE_DISCHARGE_SHORT   = 0x00400000, ///< W high side discharge short.
    Y2_HIGH_SIDE_DISCHARGE_SHORT  = 0x00800000, ///< Y2 high side discharge short.
    U_HIGH_SIDE_CHARGE_SHORT      = 0x01000000, ///< U high side charge short.
    V_HIGH_SIDE_CHARGE_SHORT      = 0x02000000, ///< V high side charge short.
    W_HIGH_SIDE_CHARGE_SHORT      = 0x04000000, ///< W high side charge short.
    Y2_HIGH_SIDE_CHARGE_SHORT     = 0x08000000, ///< Y2 high side charge short.
    GDRV_UNDERVOLTAGE             = 0x20000000, ///< Gate driver undervoltage.
    GDRV_LOW_VOLTAGE              = 0x40000000, ///< Gate driver low voltage.
    GDRV_SUPPLY_UNDERVOLTAGE      = 0x80000000  ///< Gate driver supply undervoltage.
};
/// @}

/// @name ADC Status Flags
/// @{
/**
 * @brief Bit flags reported via parameter ADC_STATUS_FLAGS.
 */
enum class AdcStatusFlags : uint32_t {
    I0_CLIPPED   = 0x00000001,
    I1_CLIPPED   = 0x00000002,
    I2_CLIPPED   = 0x00000004,
    I3_CLIPPED   = 0x00000008,
    U0_CLIPPED   = 0x00000010,
    U1_CLIPPED   = 0x00000020,
    U2_CLIPPED   = 0x00000040,
    U3_CLIPPED   = 0x00000080,
    AIN0_CLIPPED = 0x00000100,
    AIN1_CLIPPED = 0x00000200,
    AIN2_CLIPPED = 0x00000400,
    AIN3_CLIPPED = 0x00000800,
    VM_CLIPPED   = 0x00001000,
    TEMP_CLIPPED = 0x00002000
};
/// @}

} // namespace tmc9660::tmcl





