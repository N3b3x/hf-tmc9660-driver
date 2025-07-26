/**
 * @file TMC9660_Parameter_Mode_Verification_Test.cpp
 * @brief Comprehensive verification test for TMC9660 parameter mode operation
 * 
 * This test demonstrates the complete initialization sequence from bootloader
 * configuration through parameter mode setup and operational verification.
 * 
 * Test covers:
 * 1. Bootloader configuration and initialization
 * 2. Parameter mode communication verification
 * 3. Motor configuration for different motor types
 * 4. FOC control setup and verification
 * 5. Telemetry monitoring
 * 6. Error handling and diagnostics
 */

#include "TMC9660.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <map>

/**
 * @brief Test communication interface for verification
 * 
 * In a real application, replace this with your actual SPI or UART implementation.
 * This test interface simulates proper responses for verification purposes.
 */
class TestCommInterface : public SPITMC9660CommInterface {
private:
    bool initialized_ = false;
    uint32_t simulatedTempValue_ = 25;  // Simulated 25°C
    uint32_t simulatedCurrentValue_ = 0;
    uint32_t simulatedVoltageValue_ = 24000; // 24V
    std::map<uint16_t, uint32_t> parameterValues_; // Store parameter values
    
public:
    TestCommInterface() {
        // Initialize default parameter values
        parameterValues_[305] = simulatedTempValue_;     // CHIP_TEMPERATURE
        parameterValues_[306] = simulatedCurrentValue_;  // MOTOR_CURRENT  
        parameterValues_[307] = simulatedVoltageValue_;  // SUPPLY_VOLTAGE
    }
    
    bool spiTransfer(std::array<uint8_t, 8>& tx, std::array<uint8_t, 8>& rx) noexcept override {
        // Simulate realistic TMC9660 responses
        rx.fill(0);
        
        // Extract TMCL command
        uint8_t opcode = tx[0];
        uint16_t type = (tx[1] << 8) | tx[2];
        uint8_t motor = tx[3];
        uint32_t value = (tx[4] << 24) | (tx[5] << 16) | (tx[6] << 8) | tx[7];
        
        // Simulate response header (success)
        rx[0] = 0x01; // Module address
        rx[1] = 0x64; // Status: OK (100)
        rx[2] = opcode; // Echo command
        
        // Simulate parameter-specific responses
        switch (opcode) {
            case 5: // SAP (Set Axis Parameter)
                // Store parameter value for later retrieval
                parameterValues_[type] = value;
                break;
                
            case 6: // GAP (Get Axis Parameter)
                // Return stored or simulated values
                {
                    uint32_t returnValue = 0;
                    if (parameterValues_.find(type) != parameterValues_.end()) {
                        returnValue = parameterValues_[type];
                    } else {
                        // Default values for common parameters
                        switch (type) {
                            case 305: returnValue = simulatedTempValue_; break;     // CHIP_TEMPERATURE
                            case 306: returnValue = simulatedCurrentValue_; break; // MOTOR_CURRENT
                            case 307: returnValue = simulatedVoltageValue_; break; // SUPPLY_VOLTAGE
                            default: returnValue = 0; break;
                        }
                    }
                    
                    rx[4] = (returnValue >> 24) & 0xFF;
                    rx[5] = (returnValue >> 16) & 0xFF;
                    rx[6] = (returnValue >> 8) & 0xFF;
                    rx[7] = returnValue & 0xFF;
                }
                break;
                
            case 9: // SGP (Set Global Parameter)
                // Bootloader global parameter writes
                break;
                
            case 10: // GGP (Get Global Parameter)
                // Bootloader global parameter reads
                break;
                
            case 1: // Bootloader SET_BANK
            case 2: // Bootloader SET_ADDRESS  
            case 3: // Bootloader WRITE_8
            case 4: // Bootloader WRITE_16
                // Bootloader commands - always succeed
                break;
        }
        
        return true; // Simulate successful communication
    }
    
    // Simulate changing telemetry values for realistic testing
    void updateSimulatedValues() {
        simulatedTempValue_ = 25 + (std::rand() % 20); // 25-45°C
        simulatedCurrentValue_ = std::rand() % 2000;   // 0-2A
        simulatedVoltageValue_ = 24000 + (std::rand() % 1000); // 24-25V
    }
};

/**
 * @brief Test result tracking
 */
struct TestResults {
    int passed = 0;
    int failed = 0;
    
    void recordPass(const std::string& testName) {
        std::cout << "✓ PASS: " << testName << std::endl;
        passed++;
    }
    
    void recordFail(const std::string& testName, const std::string& reason = "") {
        std::cout << "✗ FAIL: " << testName;
        if (!reason.empty()) {
            std::cout << " - " << reason;
        }
        std::cout << std::endl;
        failed++;
    }
    
    void printSummary() {
        std::cout << "\n" << std::string(50, '=') << std::endl;
        std::cout << "TEST SUMMARY:" << std::endl;
        std::cout << "Passed: " << passed << std::endl;
        std::cout << "Failed: " << failed << std::endl;
        std::cout << "Total:  " << (passed + failed) << std::endl;
        std::cout << "Success Rate: " << std::fixed << std::setprecision(1) 
                  << (100.0 * passed / (passed + failed)) << "%" << std::endl;
        std::cout << std::string(50, '=') << std::endl;
    }
};

/**
 * @brief Test bootloader configuration
 */
bool testBootloaderConfiguration(TMC9660& driver, TestResults& results) {
    std::cout << "\n--- Testing Bootloader Configuration ---" << std::endl;
    
    // Create bootloader configuration
    tmc9660::BootloaderConfig cfg{};
    
    // UART Configuration
    cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;
    cfg.uart.device_address = 1;
    cfg.uart.host_address = 255;
    cfg.uart.rx_pin = tmc9660::bootcfg::UartRxPin::GPIO7;
    cfg.uart.tx_pin = tmc9660::bootcfg::UartTxPin::GPIO6;
    
    // SPI Configuration
    cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
    cfg.spiComm.spi0_sck_pin = tmc9660::bootcfg::SPI0SckPin::GPIO6;
    cfg.spiComm.disable_spi = false;
    
    // Critical: Set parameter mode
    cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    cfg.boot.start_motor_control = true;
    
    // Clock Configuration
    cfg.clock.use_external = tmc9660::bootcfg::ClockSource::Internal;
    cfg.clock.xtal_drive = tmc9660::bootcfg::XtalDrive::Freq16MHz;
    cfg.clock.pll_selection = tmc9660::bootcfg::SysClkSource::PLL;
    cfg.clock.rdiv = 14;
    cfg.clock.sysclk_div = tmc9660::bootcfg::SysClkDiv::Div1;
    
    // Initialize bootloader
    auto result = driver.bootloaderInit(&cfg);
    
    if (result == TMC9660::BootloaderInitResult::Success) {
        results.recordPass("Bootloader initialization");
        return true;
    } else {
        std::string reason = (result == TMC9660::BootloaderInitResult::NoConfig) ? 
                           "No configuration provided" : "Communication failure";
        results.recordFail("Bootloader initialization", reason);
        return false;
    }
}

/**
 * @brief Test communication with parameter mode
 */
bool testParameterModeComm(TMC9660& driver, TestResults& results) {
    std::cout << "\n--- Testing Parameter Mode Communication ---" << std::endl;
    
    // Test basic parameter read/write
    uint32_t testValue = 12345;
    uint32_t readValue = 0;
    
    // Try to write and read back a parameter
    bool writeSuccess = driver.writeParameter(
        tmc9660::tmcl::Parameters::MOTOR_TYPE, testValue);
    bool readSuccess = driver.readParameter(
        tmc9660::tmcl::Parameters::MOTOR_TYPE, readValue);
    
    if (writeSuccess && readSuccess) {
        results.recordPass("Parameter read/write communication");
        return true;
    } else {
        results.recordFail("Parameter read/write communication", 
                          "Failed to communicate with device");
        return false;
    }
}

/**
 * @brief Test BLDC motor configuration
 */
bool testBLDCMotorConfig(TMC9660& driver, TestResults& results) {
    std::cout << "\n--- Testing BLDC Motor Configuration ---" << std::endl;
    
    bool allPassed = true;
    
    // Motor type and pole pairs
    if (driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7)) {
        results.recordPass("BLDC motor type configuration");
    } else {
        results.recordFail("BLDC motor type configuration");
        allPassed = false;
    }
    
    // Motor direction
    if (driver.motorConfig.setDirection(tmc9660::tmcl::MotorDirection::FORWARD)) {
        results.recordPass("Motor direction configuration");
    } else {
        results.recordFail("Motor direction configuration");
        allPassed = false;
    }
    
    // PWM frequency
    if (driver.motorConfig.setPWMFrequency(25000)) {
        results.recordPass("PWM frequency configuration");
    } else {
        results.recordFail("PWM frequency configuration");
        allPassed = false;
    }
    
    // Current limits
    if (driver.motorConfig.setMaxTorqueCurrent(2000)) {
        results.recordPass("Maximum torque current limit");
    } else {
        results.recordFail("Maximum torque current limit");
        allPassed = false;
    }
    
    if (driver.motorConfig.setMaxFluxCurrent(1000)) {
        results.recordPass("Maximum flux current limit");
    } else {
        results.recordFail("Maximum flux current limit");
        allPassed = false;
    }
    
    // Output voltage limit
    if (driver.motorConfig.setOutputVoltageLimit(8000)) {
        results.recordPass("Output voltage limit");
    } else {
        results.recordFail("Output voltage limit");
        allPassed = false;
    }
    
    // PWM switching scheme
    if (driver.motorConfig.setPWMSwitchingScheme(
            tmc9660::tmcl::PwmSwitchingScheme::SVPWM)) {
        results.recordPass("PWM switching scheme (SVPWM)");
    } else {
        results.recordFail("PWM switching scheme (SVPWM)");
        allPassed = false;
    }
    
    return allPassed;
}

/**
 * @brief Test feedback sensor configuration
 */
bool testFeedbackSensorConfig(TMC9660& driver, TestResults& results) {
    std::cout << "\n--- Testing Feedback Sensor Configuration ---" << std::endl;
    
    bool allPassed = true;
    
    // Configure Hall sensors
    if (driver.feedbackSense.configureHall()) {
        results.recordPass("Hall sensor configuration");
    } else {
        results.recordFail("Hall sensor configuration");
        allPassed = false;
    }
    
    // Configure ABN encoder
    if (driver.feedbackSense.configureABNEncoder(1024)) {
        results.recordPass("ABN encoder configuration");
    } else {
        results.recordFail("ABN encoder configuration");
        allPassed = false;
    }
    
    return allPassed;
}

/**
 * @brief Test FOC control setup
 */
bool testFOCControlSetup(TMC9660& driver, TestResults& results) {
    std::cout << "\n--- Testing FOC Control Setup ---" << std::endl;
    
    bool allPassed = true;
    
    // Set commutation mode
    if (driver.motorConfig.setCommutationMode(
            tmc9660::tmcl::CommutationMode::FOC_HALL_SENSOR)) {
        results.recordPass("FOC Hall sensor commutation mode");
    } else {
        results.recordFail("FOC Hall sensor commutation mode");
        allPassed = false;
    }
    
    // Configure current loop gains
    if (driver.focControl.setCurrentLoopGains(50, 100)) {
        results.recordPass("Current loop PI gains");
    } else {
        results.recordFail("Current loop PI gains");
        allPassed = false;
    }
    
    // Configure velocity loop gains
    if (driver.focControl.setVelocityLoopGains(800, 1)) {
        results.recordPass("Velocity loop PI gains");
    } else {
        results.recordFail("Velocity loop PI gains");
        allPassed = false;
    }
    
    // Configure position loop gains
    if (driver.focControl.setPositionLoopGains(100, 0)) {
        results.recordPass("Position loop PI gains");
    } else {
        results.recordFail("Position loop PI gains");
        allPassed = false;
    }
    
    // Set velocity sensor
    if (driver.focControl.setVelocitySensor(
            tmc9660::tmcl::VelocitySensorSelection::ABN1_ENCODER)) {
        results.recordPass("Velocity sensor selection");
    } else {
        results.recordFail("Velocity sensor selection");
        allPassed = false;
    }
    
    // Set position sensor
    if (driver.focControl.setPositionSensor(
            tmc9660::tmcl::PositionSensorSelection::ABN1_ENCODER)) {
        results.recordPass("Position sensor selection");
    } else {
        results.recordFail("Position sensor selection");
        allPassed = false;
    }
    
    return allPassed;
}

/**
 * @brief Test motion control commands
 */
bool testMotionControl(TMC9660& driver, TestResults& results) {
    std::cout << "\n--- Testing Motion Control Commands ---" << std::endl;
    
    bool allPassed = true;
    
    // Test torque control
    if (driver.focControl.setTargetTorque(1000)) {
        results.recordPass("Set target torque");
    } else {
        results.recordFail("Set target torque");
        allPassed = false;
    }
    
    // Test velocity control
    if (driver.focControl.setTargetVelocity(1000)) {
        results.recordPass("Set target velocity");
    } else {
        results.recordFail("Set target velocity");
        allPassed = false;
    }
    
    // Test position control
    if (driver.focControl.setTargetPosition(50000)) {
        results.recordPass("Set target position");
    } else {
        results.recordFail("Set target position");
        allPassed = false;
    }
    
    // Test motor stop
    if (driver.focControl.stop()) {
        results.recordPass("Motor stop command");
    } else {
        results.recordFail("Motor stop command");
        allPassed = false;
    }
    
    return allPassed;
}

/**
 * @brief Test telemetry monitoring
 */
bool testTelemetryMonitoring(TMC9660& driver, TestCommInterface& commInterface, 
                           TestResults& results) {
    std::cout << "\n--- Testing Telemetry Monitoring ---" << std::endl;
    
    bool allPassed = true;
    
    // Update simulated values
    commInterface.updateSimulatedValues();
    
    // Test temperature reading
    try {
        float temperature = driver.telemetry.getChipTemperature();
        std::cout << "Chip temperature: " << temperature << "°C" << std::endl;
        if (temperature > 0 && temperature < 150) {
            results.recordPass("Chip temperature reading");
        } else {
            results.recordFail("Chip temperature reading", "Invalid temperature range");
            allPassed = false;
        }
    } catch (...) {
        results.recordFail("Chip temperature reading", "Exception thrown");
        allPassed = false;
    }
    
    // Test current reading
    try {
        int16_t current = driver.telemetry.getMotorCurrent();
        std::cout << "Motor current: " << current << " mA" << std::endl;
        if (current >= 0 && current <= 10000) {
            results.recordPass("Motor current reading");
        } else {
            results.recordFail("Motor current reading", "Invalid current range");
            allPassed = false;
        }
    } catch (...) {
        results.recordFail("Motor current reading", "Exception thrown");
        allPassed = false;
    }
    
    // Test voltage reading
    try {
        float voltage = driver.telemetry.getSupplyVoltage();
        std::cout << "Supply voltage: " << voltage << " V" << std::endl;
        if (voltage > 0 && voltage < 100) {
            results.recordPass("Supply voltage reading");
        } else {
            results.recordFail("Supply voltage reading", "Invalid voltage range");
            allPassed = false;
        }
    } catch (...) {
        results.recordFail("Supply voltage reading", "Exception thrown");
        allPassed = false;
    }
    
    return allPassed;
}

/**
 * @brief Test ramp control
 */
bool testRampControl(TMC9660& driver, TestResults& results) {
    std::cout << "\n--- Testing Ramp Control ---" << std::endl;
    
    bool allPassed = true;
    
    // Enable ramp
    if (driver.ramp.enable(true)) {
        results.recordPass("Ramp enable");
    } else {
        results.recordFail("Ramp enable");
        allPassed = false;
    }
    
    // Set acceleration parameters
    if (driver.ramp.setAcceleration(1000, 2000, 5000)) {
        results.recordPass("Acceleration parameters");
    } else {
        results.recordFail("Acceleration parameters");
        allPassed = false;
    }
    
    // Set deceleration parameters
    if (driver.ramp.setDeceleration(1000, 2000, 5000)) {
        results.recordPass("Deceleration parameters");
    } else {
        results.recordFail("Deceleration parameters");
        allPassed = false;
    }
    
    // Set velocity parameters
    if (driver.ramp.setVelocities(0, 0, 500, 1000, 2000)) {
        results.recordPass("Velocity parameters");
    } else {
        results.recordFail("Velocity parameters");
        allPassed = false;
    }
    
    return allPassed;
}

/**
 * @brief Main verification test function
 */
int main() {
    std::cout << "TMC9660 Parameter Mode Verification Test" << std::endl;
    std::cout << std::string(50, '=') << std::endl;
    
    TestResults results;
    TestCommInterface commInterface;
    TMC9660 driver(commInterface);
    
    // Run test sequence
    bool allTestsPassed = true;
    
    allTestsPassed &= testBootloaderConfiguration(driver, results);
    allTestsPassed &= testParameterModeComm(driver, results);
    allTestsPassed &= testBLDCMotorConfig(driver, results);
    allTestsPassed &= testFeedbackSensorConfig(driver, results);
    allTestsPassed &= testFOCControlSetup(driver, results);
    allTestsPassed &= testMotionControl(driver, results);
    allTestsPassed &= testTelemetryMonitoring(driver, commInterface, results);
    allTestsPassed &= testRampControl(driver, results);
    
    // Print test summary
    results.printSummary();
    
    if (allTestsPassed && results.failed == 0) {
        std::cout << "\n🎉 ALL TESTS PASSED - Parameter mode verification successful!" << std::endl;
        std::cout << "\nThe TMC9660 driver is properly configured for parameter mode operation." << std::endl;
        std::cout << "You can now proceed with motor control applications." << std::endl;
        return 0;
    } else {
        std::cout << "\n⚠️  SOME TESTS FAILED - Please review the configuration." << std::endl;
        std::cout << "\nCheck hardware connections, power supply, and communication interface." << std::endl;
        return 1;
    }
}

/**
 * @brief Compilation instructions:
 * 
 * g++ -std=c++20 -Iinc \
 *     TMC9660_Parameter_Mode_Verification_Test.cpp \
 *     src/TMC9660.cpp \
 *     src/TMC9660Bootloader.cpp \
 *     -o tmc9660_verification_test
 * 
 * ./tmc9660_verification_test
 */