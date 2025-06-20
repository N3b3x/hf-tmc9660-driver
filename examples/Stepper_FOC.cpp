/**
 * @file Stepper_FOC.cpp
 * @brief Closed-loop FOC position control of a stepper motor.
 *
 * The communication bus must be provided by the integrator.  Here a small dummy
 * implementation is used purely for documentation and unit tests.
 */

#include "TMC9660.hpp"
#include <iostream>

/// Example bus that echoes traffic back
class DummyBus : public SPITMC9660CommInterface {
public:
  bool spiTransfer(std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept override {
    rx = tx;
    return true;
  }
};

int main() {
  DummyBus bus;        //!< Replace with real bus
  TMC9660 driver(bus); //!< Motor driver instance

  driver.motorConfig.setType(tmc9660::tmcl::MotorType::STEPPER_MOTOR);
  driver.feedbackSense.configureABNEncoder(4000); // encoder resolution
  driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC_ABN);

  driver.focControl.setTargetPosition(500); // command position
  std::cout << "Stepper moving to position 500" << std::endl;
}
