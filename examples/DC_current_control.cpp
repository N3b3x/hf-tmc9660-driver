/**
 * @file DC_current_control.cpp
 * @brief Demonstrates open-loop current mode for a brushed DC motor.
 *
 * The bus implementation is left to the user; here we provide a minimal stub so
 * the example can compile without hardware.
 */

#include "TMC9660.hpp"
#include <iostream>

/// Dummy communication bus echoing transfers back
class DummyBus : public SPITMC9660CommInterface {
public:
  bool spiTransfer(std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept override {
    rx = tx;
    return true;
  }
};

int main() {
  DummyBus bus;        //!< Replace with actual transport
  TMC9660 driver(bus); //!< Driver communicating with the TMC9660

  // Configure motor and drive settings
  driver.motorConfig.setType(tmc9660::tmcl::MotorType::DC_MOTOR);
  driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC_OPENLOOP_CURRENT_MODE);
  driver.motorConfig.setMaxTorqueCurrent(1000); // peak current in mA
  driver.focControl.setTargetTorque(500);       // hold 500 mA

  std::cout << "DC motor current control active" << std::endl;
}
