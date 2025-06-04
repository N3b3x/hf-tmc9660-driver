/**
 * @file BLDC_velocity_control.cpp
 * @brief Brushed DC motor velocity control example.
 *
 * This sample illustrates how the driver API might be used from a host program.
 * Replace the DummyBus class with your actual communication transport.
 */

#include <iostream>
#include "TMC9660.hpp"

/// Stub communication bus for documentation purposes
class DummyBus : public SPITMC9660CommInterface {
public:
    bool spiTransfer(std::array<uint8_t,8>& tx, std::array<uint8_t,8>& rx) noexcept override {
        rx = tx; // echo back for demo purposes
        return true;
    }
};

int main() {
    DummyBus bus;        //!< Replace with your communication driver
    TMC9660 driver(bus);

    // 1. Configure motor type as DC.
    if (!driver.configureMotorType(TMC9660::MotorType::DC)) {
        std::cerr << "Failed to configure DC motor type." << std::endl;
        return 1;
    }

    // 2. Set maximum current limit for the DC motor (e.g., 1500 mA).
    driver.setMaxCurrent(1500);

    // 3. Configure an encoder for velocity feedback (e.g., 1024 counts per revolution).
    driver.configureABNEncoder(1024);

    // 4. For DC motor, use open-loop current mode to drive the H-bridge.
    driver.setCommutationMode(TMC9660::CommutationMode::FOC_OPENLOOP_CURRENT);

    // (Optional: tune velocity loop gains if necessary)
    // driver.setVelocityLoopGains(500, 5);

    // 5. Set a target velocity (requires encoder feedback for closed-loop speed control).
    driver.setTargetVelocity(500);
    std::cout << "DC motor running at target velocity 500 (internal units)." << std::endl;

    // ... (In a real application, the motor would accelerate to the target speed and maintain it) ...

    // 6. Stop the motor when done.
    driver.stopMotor();
    std::cout << "Motor stopped." << std::endl;

    return 0;
}
