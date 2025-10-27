/**
 * @file TMC9660CommInterface.hpp
 * @brief Communication interfaces for TMC9660 Parameter Mode devices using TMCL protocol over SPI
 * and UART.
 *
 * For parameter read/write access, either UART or SPI may be used. Both interfaces share the same
 * TMCL command structure and follow a strict command/reply order.
 *
 * ## SPI Command Format
 * Table 3: Command format for parameter read/write access through SPI
 * | BYTE | 0         | 1-2       | 3          | 4-6       | 7        |
 * |------|-----------|-----------|------------|-----------|----------|
 * | Bits | 0-7       | 8-19      | 20-23      | 24-55     | 56-63    |
 * | Desc | Operation | Type      | Motor/Bank | Data      | Checksum |
 *
 * ## SPI Reply Format
 * Table 4: Reply format for parameter read/write access through SPI
 * | BYTE | 0          | 1          | 2          | 3-6       | 7        |
 * |------|------------|------------|------------|-----------|----------|
 * | Bits | 0-7        | 8-15       | 16-23      | 24-55     | 56-63    |
 * | Desc | SPI Status | TMCL Status| Operation  | Data      | Checksum |
 *
 * ## UART Command Format
 * | BYTE | 0               | 1        | 2-3       | 4          | 5-8       | 9        |
 * |------|----------------|----------|-----------|------------|-----------|----------|
 * | Desc | Sync+Address   | Command  | Type      | Motor/Bank | Data      | Checksum |
 *
 * ## UART Reply Format
 * | BYTE | 0            | 1             | 2          | 3          | 4-7       | 8        |
 * |------|--------------|---------------|------------|------------|-----------|----------|
 * | Desc | Host Address | Sync+Address  | TMCL Status| Operation  | Data      | Checksum |
 *
 * ## GPIO Control Interface
 * The TMC9660 provides several GPIO pins for system control and status monitoring.
 * The interface uses board-agnostic naming and signal levels to accommodate different
 * board implementations (direct connection, inverters, etc.).
 *
 * ### Control Pins (Output from Host)
 * - **RST (Pin 22)**: External System Reset Input
 *   - Device remains in reset while this pin is in its active state
 *   - Has internal pull-down resistor
 *   - Active level depends on board implementation (configure via setPinActiveLevel)
 *
 * - **DRV_EN (Pin 21)**: Driver Enable Input
 *   - Enables the motor driver outputs
 *   - Has internal pull-down resistor
 *   - Active level depends on board implementation (configure via setPinActiveLevel)
 *
 * - **WAKE (Pin 19)**: Wake Input
 *   - Drive to active state to enable power-up and exit from hibernation mode
 *   - When not shorted to VSA, external pull-down resistor recommended
 *   - Active level depends on board implementation (configure via setPinActiveLevel)
 *
 * ### Status Pins (Input to Host)
 * - **FAULTN (Pin 20)**: FAULT Output Signal (open drain)
 *   - Indicates busy state during bootstrapping or severe error
 *   - Active level depends on board implementation (configure via setPinActiveLevel)
 *   - Use gpioRead(TMC9660CtrlPin::FAULTN, signal) to check fault status
 */

#pragma once
#include <array>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <span>
#include <string>

/**
 * @brief Supported physical communication modes for TMC9660.
 * 
 * Defines the available communication interfaces that can be used
 * to communicate with the TMC9660 motor driver.
 */
enum class CommMode { 
  SPI,   ///< SPI (Serial Peripheral Interface) mode - 4-wire synchronous communication
  UART   ///< UART (Universal Asynchronous Receiver-Transmitter) mode - 2-wire asynchronous communication
};

/**
 * @brief TMC9660 control pin identifiers with board-agnostic naming.
 * 
 * These pin identifiers abstract the physical pin assignments to provide
 * a consistent interface regardless of board implementation (direct connection,
 * inverters, level shifters, etc.).
 */
enum class TMC9660CtrlPin {
  RST,     ///< External System Reset Input (pin 22) - Active high reset signal
  DRV_EN,  ///< Driver enable input (pin 21) - Enables motor driver outputs
  FAULTN,  ///< FAULT output signal (pin 20) - Open drain fault indication
  WAKE     ///< Wake input (pin 19) - Wake from hibernation mode
};

/**
 * @brief GPIO signal states with board-agnostic naming.
 * 
 * These signal states abstract the physical voltage levels to provide
 * a consistent interface regardless of board implementation (active-high
 * or active-low signals, inverters, etc.).
 */
enum class GpioSignal {
  INACTIVE = 0,  ///< Inactive signal state (logical low)
  ACTIVE = 1     ///< Active signal state (logical high)
};

/**
 * @brief SPI status codes as per TMC9660 Parameter Mode specification.
 * 
 * These status codes are returned in the first byte of SPI replies
 * to indicate the communication status and any errors that occurred.
 */
enum class SPIStatus : uint8_t {
  OK = 0xFF,            ///< Operation successful
  CHECKSUM_ERROR = 0x00, ///< Checksum verification failed
  FIRST_CMD = 0x0C,     ///< First command after initialization
  NOT_READY = 0xF0,     ///< System busy, resend datagram
};

/**
 * @brief Calculate 8-bit checksum using sum of bytes method.
 * 
 * This function implements the TMCL checksum algorithm which is simply
 * the sum of all bytes in the datagram. The checksum is used to verify
 * data integrity during communication.
 * 
 * @param bytes Pointer to the data bytes to checksum
 * @param n Number of bytes to include in the checksum calculation
 * @param start Starting byte offset for checksum calculation (default: 0)
 * @return 8-bit checksum value
 */
static constexpr uint8_t tmclChecksum(const uint8_t *bytes, size_t n, uint8_t start = 0) noexcept {
  uint8_t sum = 0;
  for (size_t i = start; i < n; ++i)
    sum += bytes[i];
  return sum;
}

// -----------------------------------------------------------------------
// TMCL scripting support structures and enums
// -----------------------------------------------------------------------

/// Reply structure returned by sendCommand()
struct TMCLReply {
  SPIStatus spiStatus = SPIStatus::OK; ///< SPI status byte
  uint8_t status = 0;                  ///< TMCL status code (100=OK, 101=LOADED)
  uint8_t opcode = 0;                  ///< Echoed operation code
  uint32_t value = 0;                  ///< Optional returned value
  
  // ✅ NEW: Raw bytes for advanced parsing (handle protocol mismatches)
  std::array<uint8_t, 8> rawBytes = {}; ///< Raw SPI bytes (8-byte) or UART bytes mapped to 8-byte format

  [[nodiscard]] bool isOK() const noexcept {
    return spiStatus == SPIStatus::OK && (status == 100 || status == 101);
  }
  
  /**
   * @brief Extract 32-bit value from raw bytes at specified offset (MSB-first)
   * @param offset Starting byte offset (0-4 for 32-bit value)
   * @return 32-bit value extracted from rawBytes[offset..offset+3]
   * 
   * Useful for extracting misaligned data when protocol mismatch occurs
   * (e.g., bootloader 5-byte SESSION_START parsed as 8-byte TMCL).
   */
  [[nodiscard]] uint32_t extractRawValue(size_t offset) const noexcept {
    if (offset + 3 >= rawBytes.size())
      return 0;
    return (static_cast<uint32_t>(rawBytes[offset]) << 24) |
           (static_cast<uint32_t>(rawBytes[offset + 1]) << 16) |
           (static_cast<uint32_t>(rawBytes[offset + 2]) << 8) |
           static_cast<uint32_t>(rawBytes[offset + 3]);
  }
  
  /**
   * @brief Extract version string from GetVersion reply (Command 136, Type 0)
   * @return Version string (up to 8 characters) or empty string if not a version reply
   * 
   * GetVersion with Type=0 returns: [Host Address][Version String 8 chars]
   * This method extracts the version string portion
   */
  [[nodiscard]] std::string getVersionString() const noexcept {
    if (opcode != 136) return "";  // Not a GetVersion reply
    
    std::string version;
    // Extract version string from bytes 1-8 (skip host address at byte 0)
    for (size_t i = 1; i < 8; ++i) {
      if (rawBytes[i] >= 0x20 && rawBytes[i] <= 0x7E) {  // Printable ASCII
        version += static_cast<char>(rawBytes[i]);
      } else {
        break;  // Stop at first non-printable character
      }
    }
    return version;
  }

  /// Decode reply from SPI datagram
  static bool fromSpi(std::span<const uint8_t, 8> in, TMCLReply &r) noexcept {
    // Store raw bytes for advanced parsing (ALWAYS, even if parsing fails)
    std::copy(in.begin(), in.end(), r.rawBytes.begin());
    
    // Check for SESSION_START status codes (bootloader or parameter mode)
    uint8_t rawStatus = in[0];
    if (rawStatus == 0x13) {
      // SESSION_START from bootloader (0x13)
      // These use different protocols, so TMCL checksum will fail
      // But we still want to return success so caller can use rawBytes
      r.spiStatus = static_cast<SPIStatus>(rawStatus);
      r.status = in[1];
      r.opcode = in[2];
      r.value = (static_cast<uint32_t>(in[3]) << 24) | (static_cast<uint32_t>(in[4]) << 16) |
                (static_cast<uint32_t>(in[5]) << 8) | static_cast<uint32_t>(in[6]);
      return true;  // Allow SESSION_START even if checksum fails
    }
    
    // Check for GetVersion string format (Command 136, Type 0)
    // GetVersion with Type=0 returns a special string format without checksum
    // Format: [Host Address][Version String 8 chars]
    // We detect this by checking if the reply looks like ASCII characters
    bool looksLikeVersionString = true;
    for (size_t i = 1; i < 8; ++i) {  // Check bytes 1-7 for ASCII printable characters
      if (in[i] < 0x20 || in[i] > 0x7E) {  // Not printable ASCII
        looksLikeVersionString = false;
        break;
      }
    }
    
    if (looksLikeVersionString && in[0] >= 0x20 && in[0] <= 0x7E) {
      // This looks like a GetVersion string reply
      // Format: [Host Address][Version String 8 chars] - no checksum!
      r.spiStatus = SPIStatus::OK;  // Assume OK for version string
      r.status = 100;  // REPLY_OK for successful version retrieval
      r.opcode = 136;  // GetVersion command
      r.value = 0;     // No value field for string format
      return true;     // Success - no checksum validation needed
    }
    
    // Standard TMCL reply validation
    if (tmclChecksum(in.data(), 7) != in[7])
      return false;
    r.spiStatus = static_cast<SPIStatus>(in[0]);
    if (r.spiStatus == SPIStatus::NOT_READY || r.spiStatus == SPIStatus::CHECKSUM_ERROR)
      return false;
    r.status = in[1];
    r.opcode = in[2];
    r.value = (static_cast<uint32_t>(in[3]) << 24) | (static_cast<uint32_t>(in[4]) << 16) |
              (static_cast<uint32_t>(in[5]) << 8) | static_cast<uint32_t>(in[6]);
    return true;
  }

  /// Decode reply from UART datagram
  static bool fromUart(std::span<const uint8_t, 9> in, uint8_t addr, TMCLReply &r) noexcept {
    // Store raw bytes (map 9-byte UART to 8-byte format, skip first byte)
    std::copy(in.begin() + 1, in.end(), r.rawBytes.begin());
    
    // UART Reply Format (per datasheet):
    // byte0: Host Address (8 bits)
    // byte1: Sync bit (bit 0) + Module Address (bits 1-7)
    // byte2: TMCL Status
    // byte3: Operation
    // byte4-7: Data (big-endian)
    // byte8: Checksum
    
    // Verify module address in byte 1 (bits 1-7)
    // "The module address reuses the upper 7 bits of the bootloader device address"
    // Compare the upper 7 bits of byte 1 with the upper 7 bits of the address
    if ((in[1] & 0xFE) != (addr & 0xFE))
      return false;
    
    // Verify checksum over first 8 bytes
    if (tmclChecksum(in.data(), 8) != in[8])
      return false;
    
    r.spiStatus = SPIStatus::OK; // UART has no SPI status field
    r.status = in[2];            // TMCL Status at byte 2
    r.opcode = in[3];            // Operation at byte 3
    r.value = (static_cast<uint32_t>(in[4]) << 24) | (static_cast<uint32_t>(in[5]) << 16) |
              (static_cast<uint32_t>(in[6]) << 8) | static_cast<uint32_t>(in[7]);
    return true;
  }
};

/**
 * @brief Frame structure for TMCL commands and replies.
 *
 * Supports conversion to/from SPI (8 bytes) and UART (9 bytes) formats.
 */
struct TMCLFrame {
  uint8_t opcode = 0; ///< Operation code field (BYTE 0, bits 0-7).
  uint16_t type = 0;  ///< Parameter or command type (BYTE 1-2, bits 8-19).
  uint8_t motor = 0;  ///< Motor or bank identifier (BYTE 3, bits 20-23).
  uint32_t value = 0; ///< 32-bit data value (BYTE 4-7, bits 24-55).

  /**
   * @brief Serialize frame into 8-byte SPI buffer.
   * @param out Span of 8 bytes to fill: opcode, type (2), motor, value (4).
   */
  void toSpi(std::span<uint8_t, 8> out) const noexcept {
    out[0] = opcode;
    out[1] = static_cast<uint8_t>(type >> 4);
    out[2] = static_cast<uint8_t>((type << 4) | (motor & 0x0F));
    out[3] = static_cast<uint8_t>(value >> 24);
    out[4] = static_cast<uint8_t>(value >> 16);
    out[5] = static_cast<uint8_t>(value >> 8);
    out[6] = static_cast<uint8_t>(value);
    out[7] = tmclChecksum(out.data(), 7);
  }

  /**
   * @brief Serialize frame into 9-byte UART buffer, including sync bit and checksum.
   * @param addr 7-bit module address; MSB (sync bit) set automatically.
   * @param out Span of 9 bytes to fill: sync+addr, fields, checksum.
   */
  void toUart(uint8_t addr, std::span<uint8_t, 9> out) const noexcept {
    // Byte 0: [Sync bit (bit 0)] [Module Address (bits 1-7)]
    // "The module address reuses the upper 7 bits of the bootloader device address"
    // This means we take bits 7-1 of the address and place them in bits 7-1 of byte 0
    // Then set bit 0 (sync bit) to 1
    out[0] = (addr & 0xFE) | 0x01; // Keep upper 7 bits of address, set sync bit
    out[1] = opcode;
    out[2] = static_cast<uint8_t>(type & 0xFF);  // Type field (8 bits)
    out[3] = static_cast<uint8_t>(motor & 0x0F); // Motor/Bank field (4 bits)
    out[4] = static_cast<uint8_t>(value >> 24);
    out[5] = static_cast<uint8_t>(value >> 16);
    out[6] = static_cast<uint8_t>(value >> 8);
    out[7] = static_cast<uint8_t>(value);
    out[8] = tmclChecksum(out.data(), 8);
  }

  /**
   * @brief Deserialize an SPI buffer into a TMCLFrame without status check.
   * @param in Span of 8 received bytes.
   * @return Populated TMCLFrame.
   */
  static TMCLFrame fromSpi(std::span<const uint8_t, 8> in) noexcept {
    TMCLFrame f;
    f.opcode = in[0];
    f.type = static_cast<uint16_t>(in[1]) << 4 | (in[2] >> 4);
    f.motor = in[2] & 0x0F;
    f.value = (static_cast<uint32_t>(in[3]) << 24) | (static_cast<uint32_t>(in[4]) << 16) |
              (static_cast<uint32_t>(in[5]) << 8) | static_cast<uint32_t>(in[6]);
    return f;
  }

  /**
   * @brief Deserialize an SPI buffer with status and checksum validation.
   * @param in Span of 8 received bytes.
   * @param outFrame Reference to store the valid frame.
   * @return false if NOT_READY or CHECKSUM_ERROR or invalid status; true otherwise.
   */
  static bool fromSpiChecked(std::span<const uint8_t, 8> in, TMCLFrame &outFrame) noexcept {
    TMCLReply reply{};
    if (!TMCLReply::fromSpi(in, reply))
      return false;
    outFrame.opcode = reply.opcode;
    outFrame.value = reply.value;
    return true;
  }

  /**
   * @brief Deserialize a UART buffer with address and checksum validation.
   * @param in Span of 9 received bytes.
   * @param expectedAddr 7-bit expected address of host.
   * @param outFrame Reference to store the valid frame.
   * @return true if address and checksum match.
   */
  static bool fromUart(std::span<const uint8_t, 9> in, uint8_t expectedAddr,
                       TMCLFrame &outFrame) noexcept {
    TMCLReply rep{};
    if (!TMCLReply::fromUart(in, expectedAddr, rep))
      return false;
    outFrame.opcode = rep.opcode;
    outFrame.value = rep.value;
    return true;
  }

  /**
   * @brief Calculate 8-bit checksum (sum of bytes).
   * @param bytes Pointer to data bytes to sum.
   * @param n Number of bytes to include in sum.
   * @return 8-bit checksum value.
   */
  static constexpr uint8_t calculateChecksum(const uint8_t *bytes, size_t n) noexcept {
    uint8_t sum = 0;
    for (size_t i = 0; i < n; ++i)
      sum += bytes[i];
    return sum;
  }
};

/**
 * @brief SPI status codes as per TMC9660 Parameter Mode.
 *
 * Table 4: SPI status codes:
 * - 0xFF: OK (operation successful)
 * - 0x00: CHECKSUM_ERROR
 * - 0x0C: FIRST_CMD (initial response after initialization)
 * - 0xF0: NOT_READY (system busy, resend datagram)
 */
/**
 * @brief Abstract communication interface for sending/receiving TMCL datagrams.
 *
 * Defines the common API for higher-level code to send and receive 64-bit TMCL frames
 * (via SPI) or 72-bit (via UART) without knowledge of the underlying transport.
 * Also provides GPIO control interface for TMC9660 control pins.
 */
class TMC9660CommInterface {
public:
  /**
   * @brief Construct communication interface with pin active level configuration
   * @param rstActiveLevel Physical GPIO level for RST pin when ACTIVE (true=HIGH, false=LOW)
   * @param drvEnActiveLevel Physical GPIO level for DRV_EN pin when ACTIVE (true=HIGH, false=LOW)
   * @param wakeActiveLevel Physical GPIO level for WAKE pin when ACTIVE (true=HIGH, false=LOW)
   * @param faultnActiveLevel Physical GPIO level for FAULTN pin when ACTIVE (true=HIGH, false=LOW)
   */
  TMC9660CommInterface(bool rstActiveLevel, bool drvEnActiveLevel, 
                       bool wakeActiveLevel, bool faultnActiveLevel) noexcept
    : pinActiveLevels_{rstActiveLevel, drvEnActiveLevel, wakeActiveLevel, faultnActiveLevel} {}

  virtual ~TMC9660CommInterface() noexcept = default;

  /// Return underlying communication mode
  virtual CommMode mode() const noexcept = 0;

  /**
   * @brief Perform a full duplex TMCL transfer for parameter mode communication.
   *
   * For SPI: Performs two transactions - sends command, receives first reply,
   * then sends second command (or NO_OP if not provided), receives second reply.
   * 
   * For UART: Single transaction - sends command, receives reply.
   *
   * @param tx TMCL command frame to transmit
   * @param reply Reference to store the final reply
   * @param address TMC9660 module address (UART only, ignored for SPI)
   * @param firstReply Optional pointer to capture first reply (SPI only)
   * @param secondCommand Optional second command to send (SPI only, defaults to NO_OP)
   * @return true if transfer succeeded
   * @note This is for TMCL parameter mode communication (8-byte SPI, 9-byte UART)
   */
  virtual bool transferTMCL(const TMCLFrame &tx, TMCLReply &reply, uint8_t address,
                           TMCLReply *firstReply = nullptr,
                           const TMCLFrame *secondCommand = nullptr) noexcept = 0;


  /**
   * @brief Set GPIO pin signal state (output control).
   * 
   * Used for controlling TMC9660 control pins like RST, DRV_EN, and WAKE.
   * The implementation must handle the specific hardware GPIO configuration and
   * map ACTIVE/INACTIVE to the appropriate physical levels based on board design.
   * 
   * @param pin The TMC9660 control pin to control
   * @param signal The desired signal state (ACTIVE or INACTIVE)
   * @return true if the GPIO was set successfully, false otherwise
   */
  virtual bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept = 0;

  /**
   * @brief Read GPIO pin signal state (input state).
   * 
   * Used for reading TMC9660 status pins like FAULTN.
   * The implementation must handle the specific hardware GPIO configuration and
   * map physical levels to ACTIVE/INACTIVE based on board design.
   * 
   * @param pin The TMC9660 control pin to read
   * @param signal Reference to store the current signal state
   * @return true if the GPIO was read successfully, false otherwise
   */
  virtual bool gpioRead(TMC9660CtrlPin pin, GpioSignal &signal) noexcept = 0;

  /**
   * @brief Convert signal state to physical GPIO level
   * @param pin The TMC9660 control pin
   * @param signal The signal state (ACTIVE or INACTIVE)
   * @return Physical GPIO level (true=HIGH, false=LOW)
   */
  bool signalToGpioLevel(TMC9660CtrlPin pin, GpioSignal signal) const noexcept {
    bool activeLevel = pinActiveLevels_[static_cast<int>(pin)];
    return (signal == GpioSignal::ACTIVE) ? activeLevel : !activeLevel;
  }

  /**
   * @brief Convert physical GPIO level to signal state
   * @param pin The TMC9660 control pin
   * @param gpioLevel Physical GPIO level (true=HIGH, false=LOW)
   * @return Signal state (ACTIVE or INACTIVE)
   */
  GpioSignal gpioLevelToSignal(TMC9660CtrlPin pin, bool gpioLevel) const noexcept {
    bool activeLevel = pinActiveLevels_[static_cast<int>(pin)];
    return (gpioLevel == activeLevel) ? GpioSignal::ACTIVE : GpioSignal::INACTIVE;
  }

  /**
   * @brief Set GPIO pin to active state (convenience method).
   * 
   * @param pin The TMC9660 control pin to set active
   * @return true if the GPIO was set successfully, false otherwise
   */
  bool gpioSetActive(TMC9660CtrlPin pin) noexcept {
    return gpioSet(pin, GpioSignal::ACTIVE);
  }

  /**
   * @brief Set GPIO pin to inactive state (convenience method).
   * 
   * @param pin The TMC9660 control pin to set inactive
   * @return true if the GPIO was set successfully, false otherwise
   */
  bool gpioSetInactive(TMC9660CtrlPin pin) noexcept {
    return gpioSet(pin, GpioSignal::INACTIVE);
  }

  /**
   * @brief Configure the active level for a specific pin.
   * 
   * This method allows the user to configure which physical GPIO level (HIGH or LOW)
   * corresponds to the ACTIVE state for each pin. This accommodates different board
   * implementations (direct connection, inverters, etc.).
   * 
   * @param pin The TMC9660 control pin to configure
   * @param activeLevel The physical GPIO level that represents ACTIVE state
   * @return true if the configuration was successful, false otherwise
   */
  bool setPinActiveLevel(TMC9660CtrlPin pin, bool activeLevel) noexcept {
    pinActiveLevels_[static_cast<int>(pin)] = activeLevel;
    return true;
  }

protected:
  /// Pin active level configuration [RST, DRV_EN, WAKE, FAULTN]
  bool pinActiveLevels_[4];

  /**
   * @brief Debug logging function for detailed debugging information.
   * 
   * This function allows the TMC9660 driver to output debug information
   * through the communication interface, which can be routed to platform-specific
   * logging systems (e.g., ESP-IDF logging for ESP32).
   * 
   * @param level Log level (0=Error, 1=Warning, 2=Info, 3=Debug, 4=Verbose)
   * @param tag Log tag for categorization
   * @param format printf-style format string
   * @param args Variable arguments list
   */
  virtual void debugLog(int level, const char* tag, const char* format, va_list args) noexcept = 0;
  
public:
  /**
   * @brief Delay execution for specified milliseconds
   * @param ms Milliseconds to delay
   * @note This function should be implemented by the user to provide platform-specific delay
   * @note Used by bootloader operations that require timing (e.g., VDRV voltage drop wait)
   */
  virtual void delayMs(uint32_t ms) noexcept = 0;
  /**
   * @brief Public debug logging wrapper for external classes.
   * 
   * This function allows external classes like TMC9660Bootloader to output debug information
   * through the communication interface. Automatically ensures newlines are present.
   * 
   * @param level Log level (0=Error, 1=Warning, 2=Info, 3=Debug, 4=Verbose)
   * @param tag Log tag for categorization
   * @param format printf-style format string
   * @param ... Variable arguments for format string
   */
  void logDebug(int level, const char* tag, const char* format, ...) noexcept {
    va_list args;
    va_start(args, format);
    
    // Check if format string already ends with newline
    size_t format_len = strlen(format);
    const char* final_format = format;
    char* modified_format = nullptr;
    
    if (format_len == 0 || format[format_len - 1] != '\n') {
      // Allocate buffer for format + "\n"
      modified_format = new char[format_len + 2];
      strcpy(modified_format, format);
      strcat(modified_format, "\n");
      final_format = modified_format;
    }
    
    debugLog(level, tag, final_format, args);
    
    if (modified_format) {
      delete[] modified_format;
    }
    
    va_end(args);
  }
};

/**
 * @brief SPI implementation of TMC9660CommInterface.
 *
 * Uses a 4-wire SPI bus (mode 3) to exchange 64-bit datagrams (8 bytes).
 * Data is sent MSB-first, big-endian. Replies match the previous command; initial reply uses
 * FIRST_CMD status.
 */
class SPITMC9660CommInterface : public TMC9660CommInterface {
public:
  /**
   * @brief Construct SPI communication interface with pin active level configuration
   * @param rstActiveLevel Physical GPIO level for RST pin when ACTIVE (true=HIGH, false=LOW)
   * @param drvEnActiveLevel Physical GPIO level for DRV_EN pin when ACTIVE (true=HIGH, false=LOW)
   * @param wakeActiveLevel Physical GPIO level for WAKE pin when ACTIVE (true=HIGH, false=LOW)
   * @param faultnActiveLevel Physical GPIO level for FAULTN pin when ACTIVE (true=HIGH, false=LOW)
   */
  SPITMC9660CommInterface(bool rstActiveLevel, bool drvEnActiveLevel, 
                          bool wakeActiveLevel, bool faultnActiveLevel) noexcept
    : TMC9660CommInterface(rstActiveLevel, drvEnActiveLevel, wakeActiveLevel, faultnActiveLevel) {}

  CommMode mode() const noexcept override {
    return CommMode::SPI;
  }
  /**
   * @brief Low-level SPI transfer for TMCL parameter mode communication.
   * @param tx Buffer containing 8 bytes to transmit (TMCL format).
   * @param rx Buffer to receive 8 bytes from device (TMCL format).
   * @return true if the SPI transfer completed successfully.
   * @note This is for TMCL parameter mode (8-byte frames)
   */
  virtual bool spiTransferTMCL(std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept = 0;

  /**
   * @brief Low-level SPI transfer for bootloader communication.
   * 
   * The bootloader uses a 40-bit (5-byte) protocol:
   * - TX: [COMMAND(8)] [VALUE(32)]
   * - RX: [STATUS(8)] [VALUE(32)]
   * 
   * @param tx Buffer containing 5 bytes to transmit (bootloader format).
   * @param rx Buffer to receive 5 bytes from device (bootloader format).
   * @return true if the SPI transfer completed successfully.
   * @note This is for bootloader mode (5-byte frames)
   */
  virtual bool spiTransferBootloader(std::array<uint8_t, 5> &tx, std::array<uint8_t, 5> &rx) noexcept = 0;

  /**
   * @brief Set GPIO pin signal state for SPI interface.
   * @param pin The TMC9660 control pin to control
   * @param signal The desired signal state
   * @return true if the GPIO was set successfully
   */
  bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept override = 0;

  /**
   * @brief Read GPIO pin signal state for SPI interface.
   * @param pin The TMC9660 control pin to read
   * @param signal Reference to store the current signal state
   * @return true if the GPIO was read successfully
   */
  bool gpioRead(TMC9660CtrlPin pin, GpioSignal &signal) noexcept override = 0;


  bool transferTMCL(const TMCLFrame &tx, TMCLReply &reply, uint8_t,
                   TMCLReply *firstReply, const TMCLFrame *secondCommand) noexcept override {
    std::array<uint8_t, 8> txBuf, rxBuf;
    tx.toSpi(txBuf);
    
    // Log raw SPI bytes being transmitted
    logDebug(2, "SPI_TMCL", "[TMCL TX 1 ] %02X %02X %02X %02X %02X %02X %02X %02X",
             txBuf[0], txBuf[1], txBuf[2], txBuf[3], 
             txBuf[4], txBuf[5], txBuf[6], txBuf[7]);
    
    // Transaction 1: Send command, receive first reply
    if (!spiTransferTMCL(txBuf, rxBuf))
      return false;
    
    logDebug(2, "SPI_TMCL", "[TMCL RX 1 ] %02X %02X %02X %02X %02X %02X %02X %02X",
             rxBuf[0], rxBuf[1], rxBuf[2], rxBuf[3], 
             rxBuf[4], rxBuf[5], rxBuf[6], rxBuf[7]);
    
    // Optionally capture first reply
    if (firstReply) {
      bool firstParseOk = TMCLReply::fromSpi(rxBuf, *firstReply);
      logDebug(2, "SPI_TMCL", "          └─> Status=0x%02X, Value=0x%08X (parse %s)",
               static_cast<uint8_t>(firstReply->spiStatus), firstReply->value,
               firstParseOk ? "OK" : "failed");
      // Note: We don't return false here because SESSION_START may have non-standard format
      // The caller can check rawBytes[0] for SESSION_START status codes
    }
    
    // Transaction 2: Send second command (or NO_OP), receive final reply
    TMCLFrame cmd2 = secondCommand ? *secondCommand : TMCLFrame{}; // NO_OP if not provided
    cmd2.toSpi(txBuf);
    
    logDebug(2, "SPI_TMCL", "[TMCL TX 2 ] %02X %02X %02X %02X %02X %02X %02X %02X",
             txBuf[0], txBuf[1], txBuf[2], txBuf[3], 
             txBuf[4], txBuf[5], txBuf[6], txBuf[7]);
    
    if (!spiTransferTMCL(txBuf, rxBuf))
      return false;
    
    logDebug(2, "SPI_TMCL", "[TMCL RX 2 ] %02X %02X %02X %02X %02X %02X %02X %02X",
             rxBuf[0], rxBuf[1], rxBuf[2], rxBuf[3], 
             rxBuf[4], rxBuf[5], rxBuf[6], rxBuf[7]);
    
    return TMCLReply::fromSpi(rxBuf, reply);
  }

};

/**
 * @brief UART implementation of TMC9660CommInterface.
 *
 * Uses UART_TXD and UART_RXD signals; supports external transceivers via UART_TXEN.
 * Frames consist of 9 bytes: sync+address, command, type, motor, 4-byte data, checksum.
 * LSB-first transmission; checksum is 8-bit sum of first 8 bytes.
 */
class UARTTMC9660CommInterface : public TMC9660CommInterface {
public:
  /**
   * @brief Construct UART communication interface with pin active level configuration
   * @param rstActiveLevel Physical GPIO level for RST pin when ACTIVE (true=HIGH, false=LOW)
   * @param drvEnActiveLevel Physical GPIO level for DRV_EN pin when ACTIVE (true=HIGH, false=LOW)
   * @param wakeActiveLevel Physical GPIO level for WAKE pin when ACTIVE (true=HIGH, false=LOW)
   * @param faultnActiveLevel Physical GPIO level for FAULTN pin when ACTIVE (true=HIGH, false=LOW)
   */
  UARTTMC9660CommInterface(bool rstActiveLevel, bool drvEnActiveLevel, 
                          bool wakeActiveLevel, bool faultnActiveLevel) noexcept
    : TMC9660CommInterface(rstActiveLevel, drvEnActiveLevel, wakeActiveLevel, faultnActiveLevel) {}

  CommMode mode() const noexcept override {
    return CommMode::UART;
  }
  /**
   * @brief Send raw 9-byte UART TMCL datagram for parameter mode communication.
   * @param data Array of 9 bytes including sync, fields, and checksum (TMCL format).
   * @return true if transmission succeeded.
   * @note This is for TMCL parameter mode (9-byte frames)
   */
  virtual bool uartSendTMCL(const std::array<uint8_t, 9> &data) noexcept = 0;

  /**
   * @brief Receive raw 9-byte UART TMCL datagram for parameter mode communication.
   * @param data Array to store 9 received bytes (TMCL format).
   * @return true if reception succeeded.
   * @note This is for TMCL parameter mode (9-byte frames)
   */
  virtual bool uartReceiveTMCL(std::array<uint8_t, 9> &data) noexcept = 0;

  /**
   * @brief Transfer 8-byte UART bootloader datagram (send and receive).
   * 
   * The bootloader uses a different 8-byte protocol:
   * - TX: [SYNC(0x55)] [DEVICE_ADDR] [COMMAND] [VALUE(32)] [CRC8]
   * - RX: [SYNC(0x55)] [HOST_ADDR] [STATUS] [VALUE(32)] [CRC8]
   * 
   * @param tx Buffer containing 8 bytes to transmit (bootloader format).
   * @param rx Buffer to receive 8 bytes from device (bootloader format).
   * @return true if transfer succeeded.
   * @note This is for bootloader mode (8-byte frames)
   */
  virtual bool uartTransferBootloader(const std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept = 0;

  /**
   * @brief Set GPIO pin signal state for UART interface.
   * @param pin The TMC9660 control pin to control
   * @param signal The desired signal state
   * @return true if the GPIO was set successfully
   */
  bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept override = 0;

  /**
   * @brief Read GPIO pin signal state for UART interface.
   * @param pin The TMC9660 control pin to read
   * @param signal Reference to store the current signal state
   * @return true if the GPIO was read successfully
   */
  bool gpioRead(TMC9660CtrlPin pin, GpioSignal &signal) noexcept override = 0;


  bool transferTMCL(const TMCLFrame &tx, TMCLReply &reply, uint8_t address,
                   TMCLReply *firstReply, const TMCLFrame *secondCommand) noexcept override {
    // UART doesn't use the two-transaction pattern, so firstReply and secondCommand are ignored
    (void)firstReply;      // Suppress unused parameter warning
    (void)secondCommand;   // Suppress unused parameter warning
    
    std::array<uint8_t, 9> frame;
    tx.toUart(address, frame);
    if (!uartSendTMCL(frame))
      return false;
    if (!uartReceiveTMCL(frame))
      return false;
    return TMCLReply::fromUart(frame, address, reply);
  }
};
