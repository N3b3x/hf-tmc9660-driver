/**
 * @file Telemetry_monitor.cpp
 * @brief Polling telemetry data from the TMC9660.
 *
 * The library does not ship with a specific bus implementation.  The DummyBus
 * below merely echoes transfers so the example can run anywhere.
 */

#include <iostream>
#include <thread>
#include <chrono>
#include "TMC9660.hpp"

class DummyBus : public SPITMC9660CommInterface {
public:
    bool spiTransfer(std::array<uint8_t,8>& tx, std::array<uint8_t,8>& rx) noexcept override {
        rx = tx; // echo back
        return true;
    }
};

int main() {
    DummyBus bus;       //!< Replace with your communication layer
    TMC9660 driver(bus);

    // Assume motor is configured and running. We will poll telemetry.
    for (int i = 0; i < 5; ++i) {
        float chipTemp = driver.telemetry.getChipTemperature();
        int16_t motorCurrent = driver.telemetry.getMotorCurrent();
        float supplyVolt = driver.telemetry.getSupplyVoltage();
        std::cout << "Read " << i+1 << ": Temp = " << chipTemp << " °C, "
                  << "Current = " << motorCurrent << " mA, "
                  << "Voltage = " << supplyVolt << " V" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
