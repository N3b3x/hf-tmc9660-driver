/**
 * @file TMC9660CommInterface.hpp
 * @brief Communication interfaces for TMC9660 Parameter Mode devices using TMCL protocol over SPI and UART.
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
 */

#pragma once
#include <cstdint>
#include <cstdlib>
#include <array>
#include <span>

/// Supported physical communication modes
enum class CommMode { SPI, UART };

/// SPI status codes as per TMC9660 Parameter Mode
enum class SPIStatus : uint8_t {
    OK             = 0xFF,
    CHECKSUM_ERROR = 0x00,
    FIRST_CMD      = 0x0C,
    NOT_READY      = 0xF0,
};

/// Calculate 8-bit checksum (sum of bytes)
static constexpr uint8_t tmclChecksum(const uint8_t* bytes, size_t n) noexcept {
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
    SPIStatus spiStatus = SPIStatus::OK;  ///< SPI status byte
    uint8_t   status    = 0;              ///< TMCL status code (100=OK, 101=LOADED)
    uint8_t   opcode    = 0;              ///< Echoed operation code
    uint32_t  value     = 0;              ///< Optional returned value

    [[nodiscard]] bool isOK() const noexcept {
        return spiStatus == SPIStatus::OK && (status == 100 || status == 101);
    }

    /// Decode reply from SPI datagram
    static bool fromSpi(std::span<const uint8_t,8> in, TMCLReply& r) noexcept {
        if (tmclChecksum(in.data(),7) != in[7]) return false;
        r.spiStatus = static_cast<SPIStatus>(in[0]);
        if (r.spiStatus == SPIStatus::NOT_READY || r.spiStatus == SPIStatus::CHECKSUM_ERROR)
            return false;
        r.status = in[1];
        r.opcode = in[2];
        r.value  = (static_cast<uint32_t>(in[3]) << 24) |
                   (static_cast<uint32_t>(in[4]) << 16) |
                   (static_cast<uint32_t>(in[5]) << 8)  |
                   static_cast<uint32_t>(in[6]);
        return true;
    }

    /// Decode reply from UART datagram
    static bool fromUart(std::span<const uint8_t,9> in, uint8_t addr, TMCLReply& r) noexcept {
        // byte0 : host address (ignored)
        // byte1 : sync bit + module address (7-bit)
        if ((in[1] & 0x7F) != (addr & 0x7F)) return false;
        if (tmclChecksum(in.data(),8) != in[8]) return false;  // checksum over first 8 bytes
        r.spiStatus = SPIStatus::OK; // UART has no SPI status field
        r.status = in[2];
        r.opcode = in[3];
        r.value  = (static_cast<uint32_t>(in[4]) << 24) |
                   (static_cast<uint32_t>(in[5]) << 16) |
                   (static_cast<uint32_t>(in[6]) << 8)  |
                   static_cast<uint32_t>(in[7]);
        return true;
    }
};

/**
 * @brief Frame structure for TMCL commands and replies.
 *
 * Supports conversion to/from SPI (8 bytes) and UART (9 bytes) formats.
 */
struct TMCLFrame {
    uint8_t opcode = 0;   ///< Operation code field (BYTE 0, bits 0-7).
    uint16_t type = 0;    ///< Parameter or command type (BYTE 1-2, bits 8-19).
    uint8_t motor = 0;    ///< Motor or bank identifier (BYTE 3, bits 20-23).
    uint32_t value = 0;   ///< 32-bit data value (BYTE 4-7, bits 24-55).

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
        out[7] = tmclChecksum(out.data(),7);
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
        out[8] = tmclChecksum(out.data(),8);
    }

    /**
     * @brief Deserialize an SPI buffer into a TMCLFrame without status check.
     * @param in Span of 8 received bytes.
     * @return Populated TMCLFrame.
     */
    static TMCLFrame fromSpi(std::span<const uint8_t, 8> in) noexcept {
        TMCLFrame f;
        f.opcode = in[0];
        f.type   = static_cast<uint16_t>(in[1]) << 4 | (in[2] >> 4);
        f.motor  = in[2] & 0x0F;
        f.value  = (static_cast<uint32_t>(in[3]) << 24) |
                   (static_cast<uint32_t>(in[4]) << 16) |
                   (static_cast<uint32_t>(in[5]) << 8)  |
                   static_cast<uint32_t>(in[6]);
        return f;
    }

    /**
     * @brief Deserialize an SPI buffer with status and checksum validation.
     * @param in Span of 8 received bytes.
     * @param outFrame Reference to store the valid frame.
     * @return false if NOT_READY or CHECKSUM_ERROR or invalid status; true otherwise.
     */
    static bool fromSpiChecked(std::span<const uint8_t, 8> in, TMCLFrame& outFrame) noexcept {
        TMCLReply reply{};
        if(!TMCLReply::fromSpi(in, reply)) return false;
        outFrame.opcode = reply.opcode;
        outFrame.value  = reply.value;
        return true;
    }

    /**
     * @brief Deserialize a UART buffer with address and checksum validation.
     * @param in Span of 9 received bytes.
     * @param expectedAddr 7-bit expected address of host.
     * @param outFrame Reference to store the valid frame.
     * @return true if address and checksum match.
     */
    static bool fromUart(std::span<const uint8_t, 9> in, uint8_t expectedAddr, TMCLFrame& outFrame) noexcept {
        TMCLReply rep{};
        if(!TMCLReply::fromUart(in, expectedAddr, rep)) return false;
        outFrame.opcode = rep.opcode;
        outFrame.value  = rep.value;
        return true;
    }

    /**
     * @brief Calculate 8-bit checksum (sum of bytes).
     * @param bytes Pointer to data bytes to sum.
     * @param n Number of bytes to include in sum.
     * @return 8-bit checksum value.
     */
    static constexpr uint8_t calculateChecksum(const uint8_t* bytes, size_t n) noexcept {
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
    virtual bool transfer(const TMCLFrame& tx, TMCLReply& reply, uint8_t address) noexcept = 0;
};

/**
 * @brief SPI implementation of TMC9660CommInterface.
 *
 * Uses a 4-wire SPI bus (mode 3) to exchange 64-bit datagrams (8 bytes).
 * Data is sent MSB-first, big-endian. Replies match the previous command; initial reply uses FIRST_CMD status.
 */
class SPITMC9660CommInterface : public TMC9660CommInterface {
public:
    CommMode mode() const noexcept override { return CommMode::SPI; }
    /**
     * @brief Low-level SPI transfer of 8 bytes.
     * @param tx Buffer containing 8 bytes to transmit.
     * @param rx Buffer to receive 8 bytes from device.
     * @return true if the SPI transfer completed successfully.
     */
    virtual bool spiTransfer(std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept = 0;

    bool transfer(const TMCLFrame& tx, TMCLReply& reply, uint8_t) noexcept override {
        std::array<uint8_t, 8> txBuf, rxBuf;
        tx.toSpi(txBuf);
        if (!spiTransfer(txBuf, rxBuf)) return false;
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
    CommMode mode() const noexcept override { return CommMode::UART; }
    /**
     * @brief Send raw 9-byte UART TMCL datagram.
     * @param data Array of 9 bytes including sync, fields, and checksum.
     * @return true if transmission succeeded.
     */
    virtual bool sendUartDatagram(const std::array<uint8_t, 9>& data) noexcept = 0;

    /**
     * @brief Receive raw 9-byte UART TMCL datagram.
     * @param data Array to store 9 received bytes.
     * @return true if reception succeeded.
     */
    virtual bool receiveUartDatagram(std::array<uint8_t, 9>& data) noexcept = 0;

    bool transfer(const TMCLFrame& tx, TMCLReply& reply, uint8_t address) noexcept override {
        std::array<uint8_t, 9> frame;
        tx.toUart(address, frame);
        if (!sendUartDatagram(frame)) return false;
        if (!receiveUartDatagram(frame)) return false;
        return TMCLReply::fromUart(frame, address, reply);
    }
};
