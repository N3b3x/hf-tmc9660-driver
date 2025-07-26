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

  // STEP 1: Configure bootloader for parameter mode (CRITICAL!)
  tmc9660::BootloaderConfig cfg{};
  cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
  cfg.boot.start_motor_control = true;
  cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
  cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;
  cfg.clock.use_external = tmc9660::bootcfg::ClockSource::Internal;
  cfg.clock.pll_selection = tmc9660::bootcfg::SysClkSource::PLL;

  auto result = driver.bootloaderInit(&cfg);
  if (result != TMC9660::BootloaderInitResult::Success) {
    std::cerr << "[ERROR] Bootloader initialization failed!" << std::endl;
    return 1;
  }
    std::cout << "[OK] Bootloader configured for parameter mode" << std::endl;

  // STEP 2: Configure motor type as BLDC (3-phase) with specified pole pairs.
  uint8_t polePairs = 7; // example pole pair count
  if (!driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, polePairs)) {
    std::cerr << "[ERROR] Failed to set motor type!" << std::endl;
    return 1;
  }
      std::cout << "[OK] Motor type: BLDC with " << (int)polePairs << " pole pairs" << std::endl;

  // STEP 3: Set current limits BEFORE enabling commutation
  if (!driver.motorConfig.setMaxTorqueCurrent(2000)) {
    std::cerr << "[ERROR] Failed to set torque current limit!" << std::endl;
    return 1;
  }
  if (!driver.motorConfig.setMaxFluxCurrent(1000)) {
    std::cerr << "[ERROR] Failed to set flux current limit!" << std::endl;
    return 1;
  }
      std::cout << "[OK] Current limits: 2A torque, 1A flux" << std::endl;

  // STEP 4: Configure Hall sensor feedback (assuming standard hall sequence, not inverted).
  if (!driver.feedbackSense.configureHall()) {
    std::cerr << "[ERROR] Failed to configure Hall sensors!" << std::endl;
    return 1;
  }
      std::cout << "[OK] Hall sensors configured" << std::endl;

  // STEP 5: Configure FOC control gains
  if (!driver.focControl.setCurrentLoopGains(50, 100)) {
    std::cerr << "[ERROR] Failed to set current loop gains!" << std::endl;
    return 1;
  }
  if (!driver.focControl.setVelocityLoopGains(800, 1)) {
    std::cerr << "[ERROR] Failed to set velocity loop gains!" << std::endl;
    return 1;
  }
      std::cout << "[OK] FOC gains configured (Current: P=50, I=100 | Velocity: P=800, I=1)" << std::endl;

  // STEP 6: Set commutation mode to FOC with Hall feedback.
  if (!driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC_HALL_SENSOR)) {
    std::cerr << "[ERROR] Failed to set commutation mode!" << std::endl;
    return 1;
  }
      std::cout << "[OK] FOC commutation with Hall sensors enabled" << std::endl;

  // STEP 7: Command a velocity. For example, target velocity = 1000 (internal units).
  if (!driver.focControl.setTargetVelocity(1000)) {
    std::cerr << "[ERROR] Failed to set target velocity!" << std::endl;
    return 1;
  }
      std::cout << "[OK] Motor started with target velocity 1000" << std::endl;

  // ... Motor would now ramp up to the target speed and hold it ...

  // Simulate reading telemetry after some time:
  float tempC = driver.telemetry.getChipTemperature();
  int16_t current_mA = driver.telemetry.getMotorCurrent();
  float busVolt = driver.telemetry.getSupplyVoltage();
      std::cout << "Telemetry - Temp: " << tempC << " C, Current: " << current_mA
            << " mA, Bus Voltage: " << busVolt << " V" << std::endl;

  // 6. Stop the motor.
  driver.focControl.stop();
  std::cout << "Motor stop command issued." << std::endl;

  return 0;
}
