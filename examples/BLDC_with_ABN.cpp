/**
 * @file BLDC_with_ABN.cpp
 * @brief Example showing closed-loop FOC on a BLDC motor using an ABN encoder.
 *
 * As with all examples, the "DummyBus" simply echoes SPI transfers so the code
 * can be compiled on any machine.  Implement a real bus by deriving from
 * ::TMC9660CommInterface when running on hardware.
 */

#include "TMC9660.hpp"
#include <iostream>

/// Stub communication bus used for demonstration only
class DummyBus : public SPITMC9660CommInterface {
public:
  bool spiTransfer(std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept override {
    rx = tx;
    return true;
  }
};

int main() {
  DummyBus bus;        //!< Replace with your hardware bus
  TMC9660 driver(bus); //!< Driver instance using that bus

  // Configure motor parameters
  driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
  driver.feedbackSense.configureABNEncoder(2048); // 2048 line incremental encoder
  driver.motorConfig.setCommutationMode(
      tmc9660::tmcl::CommutationMode::FOC_ABN); // use encoder based FOC
  driver.motorConfig.setMaxTorqueCurrent(2000); // limit phase current to 2 A

  // Start rotating
  driver.focControl.setTargetVelocity(1000);
  std::cout << "BLDC motor running with ABN encoder" << std::endl;
}
