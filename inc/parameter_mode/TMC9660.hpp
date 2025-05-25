#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "../tmc9660_params.hpp" // for ITransport and ModuleAddress
#include "tmc9660_param_mode_tmcl.hpp"

namespace tmc9660 {

/**
 * @brief High level BLDC FOC driver for the TMC9660.
 *
 * The class encapsulates TMCL datagram handling and exposes a small
 * convenience API for sending parameter mode commands.  It relies on an
 * external transport implementation (SPI or UART) provided via
 * @ref ITransport.
 */
class TMC9660 {
public:
    /// Construct a driver for the given transport and module address.
    explicit TMC9660(ITransport &transport, ModuleAddress addr = 0) noexcept;

    /// Return the currently configured module address.
    [[nodiscard]] ModuleAddress address() const noexcept { return addr_; }
    /// Change the module address used when sending TMCL datagrams.
    void address(ModuleAddress a) noexcept { addr_ = a; }

    // ----- Parameter access -------------------------------------------------
    [[nodiscard]] bool writeParameter(uint16_t id, uint32_t value,
                                      uint8_t motor = 0) noexcept;
    [[nodiscard]] bool readParameter(uint16_t id, uint32_t &value,
                                     uint8_t motor = 0) noexcept;

    [[nodiscard]] bool writeGlobal(uint16_t id, uint8_t bank, uint32_t value) noexcept;
    [[nodiscard]] bool readGlobal(uint16_t id, uint8_t bank, uint32_t &value) noexcept;

    // Typed overloads for enums defined in tmcl namespace
    template <class Enum>
    [[nodiscard]] bool writeParameter(Enum e, uint32_t value, uint8_t motor = 0) noexcept {
        return writeParameter(static_cast<uint16_t>(e), value, motor);
    }
    template <class Enum>
    [[nodiscard]] bool readParameter(Enum e, uint32_t &value, uint8_t motor = 0) noexcept {
        return readParameter(static_cast<uint16_t>(e), value, motor);
    }

    /// Issue a generic TMCL command.
    [[nodiscard]] bool command(tmcl::Op op, uint16_t type = 0, uint8_t motor = 0,
                               uint32_t value = 0) noexcept;

    /// Convenience: stop motor movement using Op::MST.
    [[nodiscard]] bool stopMotor() noexcept { return command(tmcl::Op::MST); }
    /// Convenience: return to bootloader using Op::Boot.
    [[nodiscard]] bool bootloader() noexcept {
        return command(tmcl::Op::Boot, 0x81, 0x92, 0xA3B4C5D6u);
    }

private:
    ITransport &bus_;
    ModuleAddress addr_{};

    struct Datagram {
        uint8_t op{};
        uint16_t type{};
        uint8_t motor{};
        uint32_t value{};

        void toSpi(std::span<uint8_t, 8> out) const noexcept {
            out[0] = op;
            out[1] = static_cast<uint8_t>(type >> 8);
            out[2] = static_cast<uint8_t>(type);
            out[3] = motor;
            out[4] = static_cast<uint8_t>(value >> 24);
            out[5] = static_cast<uint8_t>(value >> 16);
            out[6] = static_cast<uint8_t>(value >> 8);
            out[7] = static_cast<uint8_t>(value);
        }

        static constexpr uint8_t checksum(const uint8_t *bytes, size_t n) noexcept {
            uint8_t sum = 0;
            for (size_t i = 0; i < n; ++i)
                sum += bytes[i];
            return sum;
        }

        void toUart(ModuleAddress addr, std::span<uint8_t, 9> out) const noexcept {
            out[0] = addr & 0x7Fu; // sync bit cleared
            out[1] = op;
            out[2] = static_cast<uint8_t>(type >> 8);
            out[3] = static_cast<uint8_t>(type);
            out[4] = motor;
            out[5] = static_cast<uint8_t>(value >> 24);
            out[6] = static_cast<uint8_t>(value >> 16);
            out[7] = static_cast<uint8_t>(value >> 8);
            out[8] = static_cast<uint8_t>(value);
            out[8] = checksum(out.data(), 8);
        }
    };

    [[nodiscard]] bool send(tmcl::Op op, uint16_t type, uint8_t motor, uint32_t value,
                            uint32_t *reply = nullptr) noexcept;
};

} // namespace tmc9660

// Implementation ---------------------------------------------------------------

inline tmc9660::TMC9660::TMC9660(ITransport &transport, ModuleAddress addr) noexcept
    : bus_(transport), addr_(addr) {}

inline bool tmc9660::TMC9660::command(tmcl::Op op, uint16_t type, uint8_t motor,
                                      uint32_t value) noexcept {
    return send(op, type, motor, value, nullptr);
}

inline bool tmc9660::TMC9660::writeParameter(uint16_t id, uint32_t value,
                                             uint8_t motor) noexcept {
    return send(tmcl::Op::SAP, id, motor, value, nullptr);
}

inline bool tmc9660::TMC9660::readParameter(uint16_t id, uint32_t &value,
                                            uint8_t motor) noexcept {
    return send(tmcl::Op::GAP, id, motor, 0, &value);
}

inline bool tmc9660::TMC9660::writeGlobal(uint16_t id, uint8_t bank, uint32_t value) noexcept {
    return send(tmcl::Op::SGP, id, bank, value, nullptr);
}

inline bool tmc9660::TMC9660::readGlobal(uint16_t id, uint8_t bank, uint32_t &value) noexcept {
    return send(tmcl::Op::GGP, id, bank, 0, &value);
}

inline bool tmc9660::TMC9660::send(tmcl::Op op, uint16_t type, uint8_t motor,
                                   uint32_t value, uint32_t *reply) noexcept {
    Datagram d{static_cast<uint8_t>(op), type, motor, value};
    std::array<uint8_t, 8> tx{};
    d.toSpi(tx);
    std::array<uint8_t, 8> rx{};
    if (!bus_.transfer(tx, rx))
        return false;

    if (reply) {
        *reply = (static_cast<uint32_t>(rx[4]) << 24) |
                  (static_cast<uint32_t>(rx[5]) << 16) |
                  (static_cast<uint32_t>(rx[6]) << 8) |
                  (static_cast<uint32_t>(rx[7]));
    }
    return true;
}

