/**
 * @file Telemetry_monitor.cpp
 * @brief Polling telemetry data from the TMC9660.
 *
 * The library does not ship with a specific bus implementation.  The DummyBus
 * below merely echoes transfers so the example can run anywhere.
 */

#include "TMC9660.hpp"
#include <chrono>
#include <iostream>
#include <iomanip>
#include <thread>

class DummyBus : public SPITMC9660CommInterface {
public:
  bool spiTransfer(std::array<uint8_t, 8> &tx, std::array<uint8_t, 8> &rx) noexcept override {
    rx = tx; // echo back
    return true;
  }
};

int main() {
  DummyBus bus; //!< Replace with your communication layer
  TMC9660 driver(bus);

  // Initialize parameter mode for telemetry access
  tmc9660::BootloaderConfig cfg{};
  cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
  cfg.boot.start_motor_control = true;
  cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
  cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;

  auto result = driver.bootloaderInit(&cfg);
  if (result != TMC9660::BootloaderInitResult::Success) {
    std::cerr << "✗ Failed to initialize parameter mode for telemetry" << std::endl;
    return 1;
  }
  std::cout << "✓ Parameter mode initialized - starting telemetry monitoring" << std::endl;

  // Basic motor setup for meaningful telemetry readings
  driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
  driver.motorConfig.setMaxTorqueCurrent(2000);
  std::cout << "✓ Basic motor configuration applied" << std::endl;

  // Poll telemetry data continuously
  std::cout << "\nTelemetry monitoring (Ctrl+C to stop):" << std::endl;
  std::cout << "----------------------------------------" << std::endl;
  
  for (int i = 0; i < 5; ++i) {
    float chipTemp = driver.telemetry.getChipTemperature();
    int16_t motorCurrent = driver.telemetry.getMotorCurrent();
    float supplyVolt = driver.telemetry.getSupplyVoltage();
    
    std::cout << "Sample " << std::setw(2) << i + 1 << ": ";
    std::cout << "Temp=" << std::setw(5) << std::fixed << std::setprecision(1) << chipTemp << "°C, ";
    std::cout << "Current=" << std::setw(5) << motorCurrent << "mA, ";
    std::cout << "Voltage=" << std::setw(5) << std::fixed << std::setprecision(2) << supplyVolt << "V";
    
    // Add status indicators
    if (chipTemp > 70.0f) std::cout << " ⚠️ HIGH TEMP";
    if (motorCurrent > 1500) std::cout << " ⚠️ HIGH CURRENT";
    if (supplyVolt < 10.0f || supplyVolt > 50.0f) std::cout << " ⚠️ VOLTAGE";
    
    std::cout << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  std::cout << "\n✓ Telemetry monitoring completed" << std::endl;
  return 0;
}
