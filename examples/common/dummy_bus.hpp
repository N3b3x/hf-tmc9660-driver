#pragma once
#include <array>
#include "TMC9660.hpp"

class DummyBus : public SPITMC9660CommInterface {
public:
    bool spiTransfer(std::array<uint8_t,8>& tx, std::array<uint8_t,8>& rx) noexcept override {
        rx = tx;
        return true;
    }
};
