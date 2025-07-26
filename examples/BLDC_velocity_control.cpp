/**
 * @file BLDC_velocity_control.cpp
 * @brief Brushed DC motor velocity control example.
 *
 * This sample illustrates how the driver API might be used from a host program.
 * Replace the DummyBus class with your actual communication transport.
 */

#include "TMC9660.hpp"
#include <iostream>

/// Stub communication bus for documentation purposes
class DummyBus : public SPITMC9660CommInterface {
public:
  bool spiTransfer(std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept override {
    rx = tx; // echo back for demo purposes
    return true;
  }
};

int main() {
  DummyBus bus; //!< Replace with your communication driver
  TMC9660 driver(bus);

  // STEP 1: Initialize bootloader for parameter mode
  tmc9660::BootloaderConfig cfg{};
  cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
  cfg.boot.start_motor_control = true;
  cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
  cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;

  auto result = driver.bootloaderInit(&cfg);
  if (result != TMC9660::BootloaderInitResult::Success) {
    std::cerr << "✗ Bootloader initialization failed!" << std::endl;
    return 1;
  }
  std::cout << "✓ Parameter mode initialized" << std::endl;

  // STEP 2: Configure motor type as DC.
  if (!driver.motorConfig.setType(tmc9660::tmcl::MotorType::DC_MOTOR)) {
    std::cerr << "✗ Failed to configure DC motor type." << std::endl;
    return 1;
  }
  std::cout << "✓ DC motor type configured" << std::endl;

  // STEP 3: Set maximum current limit for the DC motor (e.g., 1500 mA).
  if (!driver.motorConfig.setMaxTorqueCurrent(1500)) {
    std::cerr << "✗ Failed to set current limit." << std::endl;
    return 1;
  }
  std::cout << "✓ Current limit: 1.5A" << std::endl;

  // STEP 4: Configure an encoder for velocity feedback (e.g., 1024 counts per revolution).
  if (!driver.feedbackSense.configureABNEncoder(1024)) {
    std::cerr << "✗ Failed to configure encoder." << std::endl;
    return 1;
  }
  std::cout << "✓ ABN encoder configured (1024 CPR)" << std::endl;

  // STEP 5: Configure velocity control gains
  if (!driver.focControl.setVelocityLoopGains(500, 5)) {
    std::cerr << "✗ Failed to set velocity gains." << std::endl;
    return 1;
  }
  std::cout << "✓ Velocity loop gains: P=500, I=5" << std::endl;

  // STEP 6: For DC motor, use open-loop current mode to drive the H-bridge.
  if (!driver.motorConfig.setCommutationMode(tmc9660::tmcl::CommutationMode::FOC_OPENLOOP_CURRENT_MODE)) {
    std::cerr << "✗ Failed to set commutation mode." << std::endl;
    return 1;
  }
  std::cout << "✓ Open-loop current mode enabled" << std::endl;

  // STEP 7: Set a target velocity (requires encoder feedback for closed-loop speed control).
  if (!driver.focControl.setTargetVelocity(500)) {
    std::cerr << "✗ Failed to set target velocity." << std::endl;
    return 1;
  }
  std::cout << "✓ DC motor running at target velocity 500 (internal units)" << std::endl;

  // ... (In a real application, the motor would accelerate to the target speed and maintain it) ...

  // 6. Stop the motor when done.
  driver.focControl.stop();
  std::cout << "Motor stopped." << std::endl;

  return 0;
}
