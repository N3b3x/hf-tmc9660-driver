#include <iostream>
#include <thread>
#include <chrono>
#include "TMC9660.hpp"

// Example: Using telemetry APIs to read temperature, current, and voltage continuously.

class MySPIInterface : public TMC9660CommInterface {
public:
    bool transferDatagram(const std::array<uint8_t,8>& tx, std::array<uint8_t,8>& rx) override {
        // In real code, perform SPI transfer. Here, simulate device response.
        rx = tx;
        // If this were a read operation, rx[4-7] would contain actual values from the device.
        // (For demonstration, we leave it echoing the sent data.)
        return true;
    }
};

int main() {
    MySPIInterface spiBus;
    TMC9660 driver(spiBus);

    // Assume motor is configured and running. We will poll telemetry.
    for (int i = 0; i < 5; ++i) {
        float chipTemp = driver.getChipTemperature();
        int16_t motorCurrent = driver.getMotorCurrent();
        float supplyVolt = driver.getSupplyVoltage();
        std::cout << "Read " << i+1 << ": Temp = " << chipTemp << " °C, "
                  << "Current = " << motorCurrent << " mA, "
                  << "Voltage = " << supplyVolt << " V" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
