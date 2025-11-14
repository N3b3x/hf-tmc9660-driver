# Troubleshooting

Common issues and solutions for the TMC9660 driver.

## Bootloader Initialization Fails

### Symptom
`bootloaderInit()` returns `BootloaderInitResult::Failure`

### Solutions

1. **Check Hardware Reset**
   - Ensure RST pin is properly toggled
   - Verify FAULTN pin monitoring during reset
   - Wait at least 100ms after reset

2. **Verify Communication Interface**
   - Check SPI/UART wiring
   - Verify CS pin (SPI) or TX/RX pins (UART)
   - Confirm clock speed ≤ 4 MHz (SPI)

3. **Check Bootloader Configuration**
   ```cpp
   // CRITICAL: Must set boot mode to Parameter
   cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
   ```

4. **Verify Power Supply**
   - Ensure stable power to TMC9660
   - Check voltage levels (12V-48V for motor supply)

## No Reply from Device

### Symptom
Communication timeouts, no response to commands

### Solutions

1. **SPI Issues**
   - Verify MOSI/MISO not swapped
   - Check CS pin polarity (active LOW)
   - Confirm SPI mode (Mode 0 or 3)
   - Reduce clock speed

2. **UART Issues**
   - Verify TX/RX pins not swapped
   - Check baud rate matches configuration
   - Ensure GPIO pins match bootloader config
   - Try autobaud mode

3. **Check Connections**
   - Verify all connections are secure
   - Check for loose wires
   - Verify common ground

## Motor Not Starting

### Symptom
Motor configured but not rotating

### Solutions

1. **Verify Bootloader Initialization**
   ```cpp
   // Must initialize bootloader first
   cfg.boot.start_motor_control = true;
   auto result = driver.bootloaderInit(&cfg);
   ```

2. **Check Motor Configuration**
   ```cpp
   // Verify motor type is set
   driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
   
   // Verify commutation mode
   driver.motorConfig.setCommutationMode(
       tmc9660::tmcl::CommutationMode::FOC_HALL);
   ```

3. **Check Current Limits**
   ```cpp
   // Ensure current limits are set
   driver.motorConfig.setMaxTorqueCurrent(2000);
   ```

4. **Verify Target Values**
   ```cpp
   // Set target velocity or torque
   driver.focControl.setTargetVelocity(1000);
   ```

## Communication Errors

### Symptom
CRC errors, checksum failures, invalid replies

### Solutions

1. **Check Timing**
   - Add delays between commands
   - Respect SPI delayed reply timing
   - Wait for UART reply before next command

2. **Verify Protocol**
   - Check CRC/checksum calculation
   - Verify frame format (8 bytes SPI, 9 bytes UART)
   - Confirm address encoding

3. **Reduce Noise**
   - Use shorter cables
   - Add pull-up/pull-down resistors
   - Check for ground loops

## Sensor Issues

### Symptom
Hall sensors or encoder not working

### Solutions

1. **Check Sensor Wiring**
   - Verify Hall sensor connections
   - Check encoder A/B signals
   - Confirm sensor power supply

2. **Verify Sensor Configuration**
   ```cpp
   // For Hall sensors
   driver.feedbackSense.configureHall();
   
   // For encoder
   driver.feedbackSense.configureABNEncoder(2048);
   ```

3. **Check Commutation Mode**
   ```cpp
   // Must match sensor type
   driver.motorConfig.setCommutationMode(
       tmc9660::tmcl::CommutationMode::FOC_HALL); // or FOC_ENCODER
   ```

## Protection Faults

### Symptom
Overcurrent, undervoltage, or overtemperature faults

### Solutions

1. **Check Current Limits**
   ```cpp
   // Reduce current limits if overcurrent
   driver.motorConfig.setMaxTorqueCurrent(1500);
   ```

2. **Verify Power Supply**
   - Check voltage levels
   - Ensure adequate current capacity
   - Check for voltage drops

3. **Check Temperature**
   - Ensure adequate cooling
   - Monitor temperature via telemetry
   - Reduce load if overheating

## Debugging Tips

1. **Enable Debug Logging**
   - Remove `TMC9660_DISABLE_DEBUG_LOGGING` define
   - Check serial output for detailed logs

2. **Verify Communication**
   ```cpp
   // Test basic communication
   auto version = driver.bootloaderGetVersion();
   if (version.has_value()) {
       printf("Communication OK\n");
   }
   ```

3. **Check Telemetry**
   ```cpp
   // Monitor device status
   int16_t temp;
   driver.telemetry.getTemperature(temp);
   printf("Temperature: %d\n", temp);
   ```

## Getting Help

- Check the [Bootloader Initialization](special_feature_bootloader.md) guide
- Review [Platform Integration](platform_integration.md) for interface issues
- See [Examples](examples.md) for working code
- Consult the [API Reference](api_reference.md) for method details

