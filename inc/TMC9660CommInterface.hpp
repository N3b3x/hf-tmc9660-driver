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
 * The TMC9660 provides several GPIO pins for system control and status monitoring:
 *
 * ### Control Pins (Output from Host)
 * - **RSTN (Pin 22)**: External System Reset Input (active low)
 *   - Device remains in reset while this pin is in its active state
 *   - Has internal pull-down resistor
 *   - Use gpioSet(TMC9660CtrlPin::RSTN, GpioLevel::LOW) to reset device
 *
 * - **DRV_EN (Pin 21)**: Driver Enable Input (active high)
 *   - Enables the motor driver outputs
 *   - Has internal pull-down resistor
 *   - Use gpioSet(TMC9660CtrlPin::DRV_EN, GpioLevel::HIGH) to enable drivers
 *
 * - **WAKE (Pin 19)**: Wake Input (active high)
 *   - Drive high to enable power-up and exit from hibernation mode
 *   - When not shorted to VSA, external pull-down resistor recommended
 *   - Use gpioSet(TMC9660CtrlPin::WAKE, GpioLevel::HIGH) to wake device
 *
 * ### Status Pins (Input to Host)
 * - **FAULTN (Pin 20)**: FAULT Output Signal (open drain)
 *   - Indicates busy state during bootstrapping or severe error
 *   - Use gpioRead(TMC9660CtrlPin::FAULTN, level) to check fault status
 *   - LOW indicates fault condition
 */

#pragma once
#include <array>
#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <span>

/// Supported physical communication modes
enum class CommMode { 
  SPI,  ///< SPI mode
  UART  ///< UART mode
};

/// TMC9660 control pin identifiers
enum class TMC9660CtrlPin {
  RSTN,    ///< External System Reset Input (active low, pin 22)
  DRV_EN,  ///< Driver enable input (active high, pin 21)
  FAULTN,  ///< FAULT output signal (open drain, pin 20)
  WAKE     ///< Wake input (active high, pin 19)
};

/// GPIO level states
enum class GpioLevel {
  LOW = 0,  ///< Logic low level
  HIGH = 1  ///< Logic high level
};

/// SPI status codes as per TMC9660 Parameter Mode
enum class SPIStatus : uint8_t {
  OK = 0xFF,
  CHECKSUM_ERROR = 0x00,
  FIRST_CMD = 0x0C,
  NOT_READY = 0xF0,
};

/// Calculate 8-bit checksum (sum of bytes)
static constexpr uint8_t tmclChecksum(const uint8_t *bytes, size_t n) noexcept {
  uint8_t sum = 0;
  for (size_t i = 0; i < n; ++i)
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

  [[nodiscard]] bool isOK() const noexcept {
    return spiStatus == SPIStatus::OK && (status == 100 || status == 101);
  }

  /// Decode reply from SPI datagram
  static bool fromSpi(std::span<const uint8_t, 8> in, TMCLReply &r) noexcept {
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
    // byte0 : host address (ignored)
    // byte1 : sync bit + module address (7-bit)
    if ((in[1] & 0x7F) != (addr & 0x7F))
      return false;
    if (tmclChecksum(in.data(), 8) != in[8])
      return false;              // checksum over first 8 bytes
    r.spiStatus = SPIStatus::OK; // UART has no SPI status field
    r.status = in[2];
    r.opcode = in[3];
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
    out[0] = addr | 0x80; // sync bit set
    out[1] = opcode;
    out[2] = static_cast<uint8_t>(type >> 4);
    out[3] = static_cast<uint8_t>((type << 4) | (motor & 0x0F));
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
  virtual ~TMC9660CommInterface() noexcept = default;

  /// Return underlying communication mode
  virtual CommMode mode() const noexcept = 0;

  /**
   * @brief Perform a full duplex TMCL transfer.
   *
   * Implementations encode @p tx according to the active mode (SPI or UART),
   * transmit it and decode the reply into @p reply. The @p address parameter
   * is only used for UART transfers and ignored for SPI.
   */
  virtual bool transfer(const TMCLFrame &tx, TMCLReply &reply, uint8_t address) noexcept = 0;

  /**
   * @brief Set GPIO pin level (output control).
   * 
   * Used for controlling TMC9660 control pins like RSTN, DRV_EN, and WAKE.
   * The implementation must handle the specific hardware GPIO configuration.
   * 
   * @param pin The TMC9660 control pin to control
   * @param level The desired GPIO level (HIGH or LOW)
   * @return true if the GPIO was set successfully, false otherwise
   */
  virtual bool gpioSet(TMC9660CtrlPin pin, GpioLevel level) noexcept = 0;

  /**
   * @brief Read GPIO pin level (input state).
   * 
   * Used for reading TMC9660 status pins like FAULTN.
   * The implementation must handle the specific hardware GPIO configuration.
   * 
   * @param pin The TMC9660 control pin to read
   * @param level Reference to store the current GPIO level
   * @return true if the GPIO was read successfully, false otherwise
   */
  virtual bool gpioRead(TMC9660CtrlPin pin, GpioLevel &level) noexcept = 0;

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
   * @param ... Variable arguments for format string
   */
  virtual void debugLog(int level, const char* tag, const char* format, ...) noexcept = 0;
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
  CommMode mode() const noexcept override {
    return CommMode::SPI;
  }
  /**
   * @brief Low-level SPI transfer of 8 bytes.
   * @param tx Buffer containing 8 bytes to transmit.
   * @param rx Buffer to receive 8 bytes from device.
   * @return true if the SPI transfer completed successfully.
   */
  virtual bool spiTransfer(std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept = 0;

  /**
   * @brief Set GPIO pin level for SPI interface.
   * @param pin The TMC9660 control pin to control
   * @param level The desired GPIO level
   * @return true if the GPIO was set successfully
   */
  bool gpioSet(TMC9660CtrlPin pin, GpioLevel level) noexcept override = 0;

  /**
   * @brief Read GPIO pin level for SPI interface.
   * @param pin The TMC9660 control pin to read
   * @param level Reference to store the current GPIO level
   * @return true if the GPIO was read successfully
   */
  bool gpioRead(TMC9660CtrlPin pin, GpioLevel &level) noexcept override = 0;

  bool transfer(const TMCLFrame &tx, TMCLReply &reply, uint8_t) noexcept override {
    std::array<uint8_t, 8> txBuf, rxBuf;
    tx.toSpi(txBuf);
    if (!spiTransfer(txBuf, rxBuf))
      return false;
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
  CommMode mode() const noexcept override {
    return CommMode::UART;
  }
  /**
   * @brief Send raw 9-byte UART TMCL datagram.
   * @param data Array of 9 bytes including sync, fields, and checksum.
   * @return true if transmission succeeded.
   */
  virtual bool sendUartDatagram(const std::array<uint8_t, 9> &data) noexcept = 0;

  /**
   * @brief Receive raw 9-byte UART TMCL datagram.
   * @param data Array to store 9 received bytes.
   * @return true if reception succeeded.
   */
  virtual bool receiveUartDatagram(std::array<uint8_t, 9> &data) noexcept = 0;

  /**
   * @brief Set GPIO pin level for UART interface.
   * @param pin The TMC9660 control pin to control
   * @param level The desired GPIO level
   * @return true if the GPIO was set successfully
   */
  bool gpioSet(TMC9660CtrlPin pin, GpioLevel level) noexcept override = 0;

  /**
   * @brief Read GPIO pin level for UART interface.
   * @param pin The TMC9660 control pin to read
   * @param level Reference to store the current GPIO level
   * @return true if the GPIO was read successfully
   */
  bool gpioRead(TMC9660CtrlPin pin, GpioLevel &level) noexcept override = 0;

  bool transfer(const TMCLFrame &tx, TMCLReply &reply, uint8_t address) noexcept override {
    std::array<uint8_t, 9> frame;
    tx.toUart(address, frame);
    if (!sendUartDatagram(frame))
      return false;
    if (!receiveUartDatagram(frame))
      return false;
    return TMCLReply::fromUart(frame, address, reply);
  }
};
