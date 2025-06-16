/**
 * @file BLDC_with_HALL.cpp
 * @brief Run a BLDC motor using Hall sensor feedback.
 *
 * The DummyBus defined below is a simple echo implementation so this example can
 * be compiled without hardware.  Replace it with your own communication layer
 * derived from ::TMC9660CommInterface.
 */

#include "TMC9660.hpp"
#include <iostream>

class DummyBus : public SPITMC9660CommInterface {
public:
  bool spiTransfer(std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept override {
    rx = tx; // echo back for demo
    return true;
  }
};

int main() {
  DummyBus bus; //!< Replace with your comms driver
  TMC9660 driver(bus);

  // 1. Configure motor type as BLDC (3-phase) with specified pole pairs.
  uint8_t polePairs = 7; // example pole pair count
  if (!driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, polePairs)) {
    std::cerr << "Failed to set motor type!" << std::endl;
    return 1;
  }

  // 2. Configure Hall sensor feedback (assuming standard hall sequence, not inverted).
  driver.feedbackSense.configureHall();

  // 3. Set commutation mode to FOC with Hall feedback.
  driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC_HALL_SENSOR);

  // 4. Set maximum motor current (torque limit) to 2000 mA.
  driver.motorConfig.setMaxTorqueCurrent(2000);

  // (Optional) Tune FOC control gains if necessary:
  // driver.setCurrentLoopGains(50, 100);       // example current PI gains
  // driver.setVelocityLoopGains(800, 1);       // example velocity PI gains

  // 5. Command a velocity. For example, target velocity = 1000 (internal units).
  driver.focControl.setTargetVelocity(1000);
  std::cout << "Motor started with target velocity 1000." << std::endl;

  // ... Motor would now ramp up to the target speed and hold it ...

  // Simulate reading telemetry after some time:
  float tempC = driver.telemetry.getChipTemperature();
  int16_t current_mA = driver.telemetry.getMotorCurrent();
  float busVolt = driver.telemetry.getSupplyVoltage();
  std::cout << "Telemetry - Temp: " << tempC << " °C, Current: " << current_mA
            << " mA, Bus Voltage: " << busVolt << " V" << std::endl;

  // 6. Stop the motor.
  driver.focControl.stop();
  std::cout << "Motor stop command issued." << std::endl;

  return 0;
}
