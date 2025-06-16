/**
 * @file Stepper_step_dir.cpp
 * @brief Using the STEP/DIR interface with extrapolation.
 *
 * A dummy communication bus is provided for illustrative purposes only.  You
 * should replace this with your own SPI/UART implementation.
 */

#include "TMC9660.hpp"
#include <iostream>

/// Stub bus used by the example
class DummyBus : public SPITMC9660CommInterface {
public:
  bool spiTransfer(std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept override {
    rx = tx;
    return true;
  }
};

int main() {
  DummyBus bus;        //!< Replace with actual transport
  TMC9660 driver(bus); //!< Driver instance

  driver.motorConfig.setType(tmc9660::tmcl::MotorType::STEPPER_MOTOR);
  driver.stepDir.setMicrostepResolution(tmc9660::tmcl::StepDirStepDividerShift::STEP_MODE_1_8TH);
  driver.stepDir.enableInterface(true);     // enable STEP/DIR pins
  driver.stepDir.enableExtrapolation(true); // smooth interpolation

  std::cout << "STEP/DIR interface active" << std::endl;
}
