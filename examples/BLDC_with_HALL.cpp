#include <iostream>
#include "TMC9660.hpp"

// Example: Initialize and run a BLDC motor with Hall sensors using the TMC9660.

class MySPIInterface : public TMC9660CommInterface {
public:
    bool transferDatagram(const std::array<uint8_t,8>& tx, std::array<uint8_t,8>& rx) override {
        // TODO: Implement actual SPI transfer. Here we simulate a perfect transfer (echo back for demo).
        rx = tx;
        return true;
    }
};

int main() {
    MySPIInterface spiBus;
    TMC9660 driver(spiBus);

    // 1. Configure motor type as BLDC (3-phase) with specified pole pairs.
    uint8_t polePairs = 7;  // example pole pair count
    if (!driver.configureMotorType(TMC9660::MotorType::BLDC, polePairs)) {
        std::cerr << "Failed to set motor type!" << std::endl;
        return 1;
    }

    // 2. Configure Hall sensor feedback (assuming standard hall sequence, not inverted).
    driver.configureHallSensors(0, false);

    // 3. Set commutation mode to FOC with Hall feedback.
    driver.setCommutationMode(TMC9660::CommutationMode::FOC_HALL);

    // 4. Set maximum motor current (torque limit) to 2000 mA.
    driver.setMaxCurrent(2000);

    // (Optional) Tune FOC control gains if necessary:
    // driver.setCurrentLoopGains(50, 100);       // example current PI gains
    // driver.setVelocityLoopGains(800, 1);       // example velocity PI gains

    // 5. Command a velocity. For example, target velocity = 1000 (internal units).
    driver.setTargetVelocity(1000);
    std::cout << "Motor started with target velocity 1000." << std::endl;

    // ... Motor would now ramp up to the target speed and hold it ...

    // Simulate reading telemetry after some time:
    float tempC = driver.getChipTemperature();
    int16_t current_mA = driver.getMotorCurrent();
    float busVolt = driver.getSupplyVoltage();
    std::cout << "Telemetry - Temp: " << tempC << " °C, Current: " 
              << current_mA << " mA, Bus Voltage: " << busVolt << " V" << std::endl;

    // 6. Stop the motor.
    driver.stopMotor();
    std::cout << "Motor stop command issued." << std::endl;

    return 0;
}
