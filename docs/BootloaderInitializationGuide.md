---
layout: default
title: "⭐ Bootloader Initialization Guide"
description: "Critical guide for proper bootloader initialization sequence, OTP vs Runtime, and command ordering"
nav_order: 2
parent: "📚 Documentation"
permalink: /docs/BootloaderInitializationGuide/
---

# TMC9660 Bootloader Initialization Guide

## 🎯 **Critical Understanding: OTP vs Runtime Configuration**

The TMC9660 bootloader has **two distinct operational modes** depending on where `START_MOTOR_CONTROL` is set:

### **Mode 1: OTP Configuration (Production)**

When `START_MOTOR_CONTROL = 1` is **burned into OTP memory**:

```
Power-Up → Bootloader reads OTP → Sees START_MOTOR_CTRL=1 → Immediately starts motor control
                                                              ↓
                                                    ❌ NO bootloader commands accepted!
```

**Characteristics:**
- ✅ Autonomous operation - no host needed
- ✅ Fastest startup time
- ❌ **Cannot send bootloader commands after power-up**
- ❌ **Cannot reconfigure without hardware reset**
- ✅ Perfect for production deployments

**Use case:** Final product where TMC9660 operates independently.

---

### **Mode 2: Runtime Configuration (Development)**

When `START_MOTOR_CONTROL = 1` is **written to CONFIG bank at runtime**:

```
Power-Up → Bootloader active → Accept commands → ... → Write START_MOTOR_CTRL=1 → Exit bootloader
                                                                                   ↓
                                                                    Motor control starts
```

**Characteristics:**
- ✅ **Can send multiple bootloader commands** before exiting
- ✅ **Flexible configuration** during development
- ✅ Test different settings without burning OTP
- ⚠️ Requires host to send configuration
- ⚠️ Longer startup time

**Use case:** Development, testing, prototyping.

---

## 🚨 **CRITICAL: Command Ordering**

According to the TMC9660 datasheet:

> "Each WRITE_* command triggers a reconfiguration, do not split up bigger writes into smaller ones if it would create an invalid intermediate configuration."

**When you write `START_MOTOR_CTRL = 1` to CONFIG bank, the bootloader EXITS IMMEDIATELY!**

### **❌ WRONG: START_MOTOR_CTRL written early**

```cpp
// WRONG! This will fail!
setBank(5);                           // ✅ OK
setAddress(0x00020008);               // ✅ OK (BOOT_CONFIG offset)
write32(boot_config_with_start=1);    // ⚠️ Bootloader exits HERE!

// ❌ All subsequent commands FAIL because bootloader is gone!
setAddress(0x00020002);               // ❌ FAILS - motor control is running
write16(uart_addresses);              // ❌ FAILS - bootloader not listening
setAddress(0x00020018);               // ❌ FAILS
write32(clock_config);                // ❌ FAILS
```

### **✅ CORRECT: START_MOTOR_CTRL written LAST**

```cpp
// CORRECT! Write all other configs first
setBank(5);                           // CONFIG memory bank

// 1. Write LDO config (offset 0x00)
setAddress(0x00020000);
write32(ldo_config);

// 2. Write UART addresses (offset 0x02)
setAddress(0x00020002);
write16(uart_addresses);

// 3. Write RS485 delays (offset 0x04)
setAddress(0x00020004);
write16(rs485_delays);

// 4. Write communication config (offset 0x06)
setAddress(0x00020006);
write32(comm_config);

// 5. Write SPI flash config (offset 0x0A)
setAddress(0x0002000A);
write32(flash_config);

// 6. Write I2C config (offset 0x0C)
setAddress(0x0002000C);
write32(i2c_config);

// 7. Write GPIO configs (offsets 0x0E - 0x16)
setAddress(0x0002000E);
write32(gpio_out);
// ... more GPIO writes ...

// 8. Write clock config (offset 0x18)
setAddress(0x00020018);
write32(clock_config);

// 9. ⚠️ FINAL STEP: Write BOOT_CONFIG with START_MOTOR_CTRL=1 (offset 0x08)
setAddress(0x00020008);
uint32_t boot = 0;
boot |= (boot_mode & 0x3);              // Bits 0-1
boot |= (1u << 12);                     // Bit 12: START_MOTOR_CTRL = 1
write32(boot);                          // 🚪 Bootloader exits HERE!

// Motor control is now running - bootloader commands no longer work
```

---

## 📝 **Correct Initialization Sequence**

### **Step 1: Hardware Reset**

```cpp
// Assert RST pin (active)
gpioSetActive(TMC9660CtrlPin::RST);
vTaskDelay(pdMS_TO_TICKS(100));  // Hold reset for 100ms

// Release RST pin (inactive)
gpioSetInactive(TMC9660CtrlPin::RST);
vTaskDelay(pdMS_TO_TICKS(100));  // Wait for chip to stabilize
```

**Timing notes:**
- Minimum reset pulse: 100ms recommended
- Post-reset stabilization: 100ms minimum
- **Do NOT rely solely on FAULTN** - it may not assert with default settings

### **Step 2: Apply Configuration**

```cpp
TMC9660 driver(spi_interface);

// Configure bootloader settings
tmc9660::BootloaderConfig cfg{};

// LDO settings
cfg.ldo.vext1 = tmc9660::bootcfg::LDOVoltage::V3_3;
cfg.ldo.vext2 = tmc9660::bootcfg::LDOVoltage::V5_0;

// UART settings
cfg.uart.device_address = 1;
cfg.uart.host_address = 255;
cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;

// Boot settings
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
cfg.boot.bl_ready_fault = false;  // Don't assert FAULTN when ready
cfg.boot.bl_exit_fault = true;    // DO assert FAULTN when exiting (helps debugging)

// ⚠️ CRITICAL: Set this to TRUE to exit bootloader after configuration
cfg.boot.start_motor_control = true;  // MUST be TRUE!

// SPI settings
cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
cfg.spiComm.disable_spi = false;

// Clock settings
cfg.clock.use_external = tmc9660::bootcfg::ClockSource::Internal;
cfg.clock.pll_selection = tmc9660::bootcfg::SysClkSource::PLL;
cfg.clock.rdiv = 14;  // Internal 15MHz oscillator: RDIV = freq_MHz - 1

// Apply configuration (this will write START_MOTOR_CTRL=1 LAST)
auto result = driver.bootloaderInit(&cfg);
if (result != TMC9660::BootloaderInitResult::Success) {
    ESP_LOGE(TAG, "Bootloader initialization failed!");
    return false;
}
```

### **Step 3: Wait for Motor Control to Start**

```cpp
// After bootloaderInit() returns with start_motor_control=true:
// - Bootloader has exited
// - Motor control application is starting
// - Wait for it to fully initialize

vTaskDelay(pdMS_TO_TICKS(150));  // Give motor control time to start

ESP_LOGI(TAG, "Motor control application should now be running");
```

### **Step 4: Verify Motor Control is Active**

```cpp
// Try to send a motor control command
bool success = driver.motorConfig.setType(
    tmc9660::tmcl::MotorType::BLDC_MOTOR, 7
);

if (success) {
    ESP_LOGI(TAG, "✅ Motor control is active - bootloader exited successfully!");
} else {
    ESP_LOGE(TAG, "❌ Motor control not responding - bootloader may not have exited");
}
```

---

## ⚠️ **Understanding FAULTN Pin Behavior**

The FAULTN pin behavior is controlled by **three bits** in the Bootstrap Configuration:

| Bit | Name | Default | Behavior |
|-----|------|---------|----------|
| Bit 2 | `BL_READY_FAULT` | `0` | Controls FAULTN when bootloader is ready to communicate |
| Bit 3 | `BL_EXIT_FAULT` | `1` | Controls FAULTN when bootloader exits to motor control |
| Bit 9 | `BL_CONFIG_FAULT` | `0` | Controls FAULTN during configuration updates |

### **Official FAULTN Pin Behavior (Per TMC9660 Datasheet):**

#### **Power-On Sequence:**
```
Power-Up → FAULTN ASSERTED (Active) → Reset Released → Bootloader Initializing
```

#### **Bootloader Ready (Default: BL_READY_FAULT=0):**
```
Bootloader Ready → FAULTN DEASSERTS (Inactive) → Ready for Commands
```

#### **Motor Control Start (Default: BL_EXIT_FAULT=1):**
```
Write START_MOTOR_CTRL=1 → Bootloader Exits → FAULTN ASSERTS (Active) → Motor Control Starts
```

### **Complete Sequence with Default Settings:**

```
Power-Up → FAULTN ACTIVE → Reset Released → Bootloader Initializing
                                                       ↓
                                              Bootloader Ready
                                                       ↓
                                              FAULTN goes INACTIVE ✅
                                                       ↓
                                              Send configurations
                                                       ↓
                                       Write START_MOTOR_CTRL=1
                                                       ↓
                                           Bootloader exits
                                                       ↓
                                           FAULTN goes ACTIVE! ⚠️
                                                       ↓
                                      Motor control starts
                                                       ↓
                                Motor control deasserts FAULTN when ready
```

### **Key Points:**

1. **✅ Power-On:** FAULTN starts **ACTIVE** (asserted)
2. **✅ Bootloader Ready:** FAULTN goes **INACTIVE** (deasserted) with default settings
3. **✅ Motor Start:** FAULTN goes **ACTIVE** (asserted) when bootloader exits
4. **✅ Motor Ready:** Motor control system deasserts FAULTN when fully initialized

**⚠️ IMPORTANT:** FAULTN going ACTIVE when exiting bootloader is **NORMAL** with `BL_EXIT_FAULT=1`!

This is **NOT an error** - it's a signal that the bootloader has exited.

### **FAULTN Pin Configuration Options:**

#### **Option 1: Default Configuration (Recommended)**
```cpp
cfg.boot.bl_ready_fault = false;   // FAULTN deasserts when bootloader ready
cfg.boot.bl_exit_fault = true;     // FAULTN asserts when bootloader exits
cfg.boot.bl_config_fault = false;  // No FAULTN during config updates
```

**Timeline:**
```
Power-Up → FAULTN ACTIVE → Bootloader Ready → FAULTN INACTIVE → 
Config Commands → START_MOTOR_CTRL=1 → Bootloader Exit → FAULTN ACTIVE → 
Motor Control Ready → FAULTN INACTIVE
```

#### **Option 2: Signal Configuration Completion**
```cpp
cfg.boot.bl_ready_fault = false;   // FAULTN deasserts when bootloader ready
cfg.boot.bl_exit_fault = false;    // No FAULTN on bootloader exit
cfg.boot.bl_config_fault = true;   // FAULTN asserts during config updates
```

**Timeline:**
```
Power-Up → FAULTN ACTIVE → Bootloader Ready → FAULTN INACTIVE → 
Config Write → FAULTN ACTIVE → Config Complete → FAULTN INACTIVE → 
START_MOTOR_CTRL=1 → Motor Control Starts (no FAULTN change)
```

#### **Option 3: Signal Bootloader Readiness**
```cpp
cfg.boot.bl_ready_fault = true;    // FAULTN asserts when bootloader ready
cfg.boot.bl_exit_fault = true;     // FAULTN asserts when bootloader exits
cfg.boot.bl_config_fault = false;  // No FAULTN during config updates
```

**Timeline:**
```
Power-Up → FAULTN ACTIVE → Bootloader Ready → FAULTN STAYS ACTIVE → 
Config Commands → START_MOTOR_CTRL=1 → Bootloader Exit → FAULTN STAYS ACTIVE → 
Motor Control Ready → FAULTN INACTIVE
```

### **When to Use Each Configuration:**

| Use Case | BL_READY_FAULT | BL_EXIT_FAULT | BL_CONFIG_FAULT | Reason |
|----------|----------------|---------------|-----------------|---------|
| **Standard Operation** | `false` | `true` | `false` | Clear bootloader ready signal, exit signal |
| **Slow Config Operations** | `false` | `false` | `true` | Signal config completion for slow operations |
| **Debug/Development** | `true` | `true` | `false` | Always know when bootloader is active |
| **Production Line** | `false` | `true` | `true` | Signal both config completion and exit |

### **Monitoring FAULTN Pin in Code:**

```cpp
// Example: Wait for bootloader to be ready
void wait_for_bootloader_ready() {
    // With default settings (BL_READY_FAULT=0):
    // FAULTN should go INACTIVE when bootloader is ready
    
    while (gpioGetLevel(TMC9660CtrlPin::FAULTN) == 1) {  // Active = 1, Inactive = 0
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "Bootloader ready (FAULTN inactive)");
}

// Example: Wait for motor control to start
void wait_for_motor_control_start() {
    // With default settings (BL_EXIT_FAULT=1):
    // FAULTN should go ACTIVE when bootloader exits
    
    while (gpioGetLevel(TMC9660CtrlPin::FAULTN) == 0) {  // Wait for it to go active
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "Bootloader exited (FAULTN active)");
    
    // Wait for motor control to deassert FAULTN
    vTaskDelay(pdMS_TO_TICKS(150));  // Give motor control time to initialize
    
    while (gpioGetLevel(TMC9660CtrlPin::FAULTN) == 1) {  // Wait for it to go inactive
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "Motor control ready (FAULTN inactive)");
}
```

---

## 🔄 **Hybrid Workflow: Best of Both Worlds**

The **most flexible approach** combines `applyConfiguration()` with `startMotorControl()`:

### **Why Use This Approach?**

✅ **Best for:**
- Manufacturing/production setup requiring OTP programming
- Verification workflows (configure → test → start)
- Conditional motor startup based on diagnostics
- Multi-stage initialization processes
- Development/debug scenarios

### **The Workflow:**

```
┌─────────────────────────────────────────────────────────────┐
│ 1. applyConfiguration() with start_motor_control = FALSE   │
│    → Applies ALL standard settings (LDO, UART, Clock, etc) │
│    → Bootloader stays active                                │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ 2. Get bootloader access via getBootloader()               │
│    → Returns pointer to bootloader instance                 │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ 3. Perform custom bootloader operations                     │
│    → OTP programming                                        │
│    → Flash verification                                     │
│    → Memory checks                                          │
│    → Custom diagnostics                                     │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│ 4. Call startMotorControl() when ready                     │
│    → Bootloader exits                                       │
│    → Motor control starts                                   │
└─────────────────────────────────────────────────────────────┘
```

### **Complete Example: Production OTP Programming**

```cpp
#include "TMC9660.hpp"
#include "TMC9660Bootloader.hpp"

bool production_initialize(TMC9660& driver) {
    // ═══════════════════════════════════════════════════════════
    // STEP 1: Apply standard configuration (stay in bootloader)
    // ═══════════════════════════════════════════════════════════
    
    tmc9660::BootloaderConfig cfg{};
    
    // Boot settings
    cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
    cfg.boot.start_motor_control = false;  // ← CRITICAL: Stay in bootloader!
    
    // LDO settings
    cfg.ldo.vext1 = tmc9660::bootcfg::LDOVoltage::V3_3;
    cfg.ldo.vext2 = tmc9660::bootcfg::LDOVoltage::V5_0;
    
    // UART settings
    cfg.uart.device_address = 1;
    cfg.uart.host_address = 255;
    cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;
    
    // Clock settings
    cfg.clock.use_external = tmc9660::bootcfg::ClockSource::Internal;
    cfg.clock.pll_selection = tmc9660::bootcfg::SysClkSource::PLL;
    cfg.clock.rdiv = 14;  // 15MHz internal oscillator
    
    // Apply configuration (bootloader stays active)
    auto result = driver.bootloaderInit(&cfg);
    if (result != TMC9660::BootloaderInitResult::Success) {
        ESP_LOGE(TAG, "Configuration failed!");
        return false;
    }
    
    ESP_LOGI(TAG, "✓ Standard configuration applied, bootloader still active");
    
    // ═══════════════════════════════════════════════════════════
    // STEP 2: Get bootloader access for custom operations
    // ═══════════════════════════════════════════════════════════
    
    auto* bootloader = driver.getBootloader();
    if (!bootloader) {
        ESP_LOGE(TAG, "Failed to get bootloader access");
        return false;
    }
    
    // ═══════════════════════════════════════════════════════════
    // STEP 3: Perform OTP programming (production only)
    // ═══════════════════════════════════════════════════════════
    
    // Check if OTP page 0 is already programmed
    uint8_t errorCount, pageTag;
    if (!bootloader->otpLoad(0, &errorCount, &pageTag)) {
        ESP_LOGE(TAG, "Failed to load OTP page 0");
        return false;
    }
    
    if (pageTag == 0xFF) {
        ESP_LOGI(TAG, "OTP page 0 is empty, programming production config...");
        
        // Switch to OTP memory bank
        if (!bootloader->setBank(1)) {
            ESP_LOGE(TAG, "Failed to set OTP bank");
            return false;
        }
        
        // Write production configuration to OTP
        bootloader->setAddress(0x00010000);
        uint32_t production_config[] = {
            0x12345678,  // Motor parameters
            0xABCDEF00,  // Current limits
            0x11223344,  // Safety settings
            // ... more config data ...
        };
        
        if (!bootloader->write32IncMultiple(production_config, 
                                            sizeof(production_config)/sizeof(uint32_t))) {
            ESP_LOGE(TAG, "Failed to write OTP data");
            return false;
        }
        
        // Burn to OTP page 0 with config tag (4)
        if (!bootloader->otpBurn(0, 4)) {
            ESP_LOGE(TAG, "OTP burn failed!");
            return false;
        }
        
        ESP_LOGI(TAG, "✓ OTP programmed successfully");
        vTaskDelay(pdMS_TO_TICKS(100));  // Let OTP settle
        
    } else if (pageTag == 4) {
        ESP_LOGI(TAG, "OTP already programmed with config (tag=4)");
    } else {
        ESP_LOGW(TAG, "OTP has unexpected tag: %d", pageTag);
    }
    
    // ═══════════════════════════════════════════════════════════
    // STEP 4: Verify external flash (if present)
    // ═══════════════════════════════════════════════════════════
    
    bool flashConfigured;
    if (bootloader->memIsConfigured(tmc9660::MemoryBank::SPI_FLASH, &flashConfigured)) {
        if (flashConfigured) {
            bool flashConnected;
            if (bootloader->memIsConnected(tmc9660::MemoryBank::SPI_FLASH, &flashConnected)) {
                if (flashConnected) {
                    // Read JEDEC ID
                    uint8_t jedecId;
                    if (bootloader->flashReadJedecId(&jedecId)) {
                        ESP_LOGI(TAG, "✓ Flash detected, JEDEC ID: 0x%02X", jedecId);
                    }
                } else {
                    ESP_LOGW(TAG, "Flash configured but not connected");
                }
            }
        }
    }
    
    // ═══════════════════════════════════════════════════════════
    // STEP 5: Run diagnostics before starting motor control
    // ═══════════════════════════════════════════════════════════
    
    // Query bootloader info
    uint32_t configMemStart, configMemSize;
    if (bootloader->getConfigMemStart(&configMemStart) &&
        bootloader->getConfigMemSize(&configMemSize)) {
        ESP_LOGI(TAG, "Config memory: 0x%08X, size: %u bytes", 
                 configMemStart, configMemSize);
    }
    
    // Check EEPROM if configured
    bool eepromConfigured;
    if (bootloader->memIsConfigured(tmc9660::MemoryBank::EEPROM, &eepromConfigured)) {
        if (eepromConfigured) {
            ESP_LOGI(TAG, "✓ EEPROM is configured");
        }
    }
    
    // ═══════════════════════════════════════════════════════════
    // STEP 6: All checks passed - start motor control!
    // ═══════════════════════════════════════════════════════════
    
    ESP_LOGI(TAG, "All pre-flight checks passed, starting motor control...");
    
    if (!bootloader->startMotorControl(tmc9660::bootcfg::BootMode::Parameter)) {
        ESP_LOGE(TAG, "Failed to start motor control");
        return false;
    }
    
    // Wait for motor control to initialize
    vTaskDelay(pdMS_TO_TICKS(150));
    
    ESP_LOGI(TAG, "✓ Motor control system started successfully!");
    
    return true;
}

// Usage in main:
void app_main() {
    auto spi_interface = createSPIInterface();
    TMC9660 driver(*spi_interface);
    
    if (production_initialize(driver)) {
        // Now ready for motor control
        driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
        driver.focControl.setTargetVelocity(1000);
    }
}
```

### **Simpler Example: Conditional Startup**

```cpp
// Configure everything standard
tmc9660::BootloaderConfig cfg{};
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
cfg.boot.start_motor_control = false;  // Stay in bootloader
// ... set other config ...

driver.bootloaderInit(&cfg);

// Get bootloader access
auto* bootloader = driver.getBootloader();
if (bootloader) {
    // Check external conditions
    if (external_enable_switch_pressed()) {
        // Verify system health
        bool flashOK = verify_flash_health(bootloader);
        bool configOK = verify_configuration(bootloader);
        
        if (flashOK && configOK) {
            ESP_LOGI(TAG, "System healthy, starting motor control");
            bootloader->startMotorControl();
            vTaskDelay(pdMS_TO_TICKS(150));
        } else {
            ESP_LOGE(TAG, "System check failed, staying in bootloader");
            // Can continue debugging via bootloader
        }
    } else {
        ESP_LOGI(TAG, "Enable switch not pressed, waiting in bootloader");
        // Can monitor or wait for condition to change
    }
}
```

### **Key Advantages of Hybrid Approach:**

| Feature | Advantage |
|---------|-----------|
| **Configuration** | ✅ Easy - use `applyConfiguration()` for all standard settings |
| **Flexibility** | ✅ High - can add custom bootloader operations |
| **Safety** | ✅ Can verify system health before starting motor |
| **OTP Programming** | ✅ Perfect for production manufacturing flow |
| **Debugging** | ✅ Can stay in bootloader for diagnostics |
| **Code Clarity** | ✅ Clear separation: config → verify → start |

---

## 🚀 **Using `startMotorControl()` Function**

For scenarios where you want fine-grained control over bootloader operations, use the dedicated `startMotorControl()` function.

### **When to Use This Function:**

✅ **Use `startMotorControl()` when:**
- You want to send custom bootloader commands
- You need precise control over the configuration sequence
- You're implementing advanced bootloader features (OTP programming, flash operations)
- You want to configure specific registers manually
- You're debugging bootloader behavior

❌ **Use `applyConfiguration()` when:**
- You want a simple, all-in-one configuration
- You're doing standard motor control setup
- You want the driver to handle ordering automatically

### **Function Signature:**

```cpp
bool TMC9660Bootloader::startMotorControl(
    bootcfg::BootMode bootMode = bootcfg::BootMode::Parameter
) noexcept;
```

**Parameters:**
- `bootMode`: Motor control mode to start
  - `bootcfg::BootMode::Parameter` (default) - Parameter mode (TMCL)
  - `bootcfg::BootMode::Register` - Register mode (direct register access)

**Returns:**
- `true` if the command was sent successfully
- `false` if there was a communication error

### **Usage Example 1: Basic Manual Configuration**

```cpp
#include "TMC9660Bootloader.hpp"

// After reset, create bootloader instance
TMC9660Bootloader bootloader(spi_interface);

// Configure UART addresses
bootloader.setBank(5);                    // CONFIG bank
bootloader.setAddress(0x00020002);        // UART_ADDR offset
bootloader.write16(0xFF01);               // Host=255, Device=1

// Configure clock for 40MHz operation
bootloader.setAddress(0x00020018);        // CLOCK_CONFIG offset
uint32_t clk = (1 << 16) | (14 << 18);   // PLL enabled, RDIV=14
bootloader.write32(clk);

// Configure LDO outputs
bootloader.setAddress(0x00020000);        // LDO_CONFIG offset
bootloader.write32(0x0000000A);           // VEXT1=3.3V, VEXT2=3.3V

// Done configuring - start motor control!
if (bootloader.startMotorControl(tmc9660::bootcfg::BootMode::Parameter)) {
    ESP_LOGI(TAG, "Motor control starting...");
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_LOGI(TAG, "Motor control should be active now");
} else {
    ESP_LOGE(TAG, "Failed to start motor control!");
}
```

### **Usage Example 2: OTP Programming Then Start**

```cpp
// Program OTP with configuration
TMC9660Bootloader bootloader(spi_interface);

// 1. Load OTP page 0 to check if empty
uint8_t errorCount, pageTag;
if (bootloader.otpLoad(0, &errorCount, &pageTag)) {
    if (pageTag == 0xFF) {
        ESP_LOGI(TAG, "OTP page 0 is empty, programming...");
        
        // 2. Write configuration to OTP memory bank
        bootloader.setBank(1);  // OTP bank
        bootloader.setAddress(0x00010000);
        // Write configuration data...
        bootloader.write32(config_data);
        
        // 3. Burn to OTP
        bootloader.otpBurn(0, 4);  // Page 0, tag 4 (config)
        
        ESP_LOGI(TAG, "OTP programmed successfully");
    }
}

// 4. Start motor control with the OTP configuration
bootloader.startMotorControl(tmc9660::bootcfg::BootMode::Parameter);
vTaskDelay(pdMS_TO_TICKS(150));
```

### **Usage Example 3: Conditional Start Based on External Input**

```cpp
TMC9660Bootloader bootloader(spi_interface);

// Configure based on DIP switches or other external input
if (read_dip_switch(0)) {
    // Configure for BLDC motor
    bootloader.setBank(5);
    bootloader.setAddress(0x00020000);
    bootloader.write32(bldc_config);
} else {
    // Configure for stepper motor
    bootloader.setBank(5);
    bootloader.setAddress(0x00020000);
    bootloader.write32(stepper_config);
}

// Check if user button is pressed to skip motor start
if (!button_pressed()) {
    // Start motor control
    bootloader.startMotorControl(tmc9660::bootcfg::BootMode::Parameter);
    vTaskDelay(pdMS_TO_TICKS(150));
} else {
    ESP_LOGI(TAG, "Motor start skipped, staying in bootloader mode");
    // Can continue sending bootloader commands
}
```

### **Important Notes:**

1. **⚠️ Bootloader Exits Immediately:**
   ```cpp
   bootloader.startMotorControl();  // Bootloader exits here
   bootloader.setBank(5);           // ❌ FAILS - bootloader is gone!
   ```

2. **✅ Always Wait After Calling:**
   ```cpp
   bootloader.startMotorControl();
   vTaskDelay(pdMS_TO_TICKS(150));  // REQUIRED - motor control needs time to initialize
   ```

3. **✅ Configure Everything First:**
   ```cpp
   // ✅ CORRECT order
   configure_ldo();
   configure_uart();
   configure_clock();
   configure_gpios();
   startMotorControl();  // Last step
   ```

4. **🔍 Check Return Value:**
   ```cpp
   if (!bootloader.startMotorControl()) {
       ESP_LOGE(TAG, "Failed to send start command - check SPI/UART connection");
       return false;
   }
   ```

---

## 🔍 **Debugging Tips**

### **Problem: Motor control commands fail after bootloaderInit()**

**Likely cause:** `start_motor_control = false`

**Fix:**
```cpp
cfg.boot.start_motor_control = true;  // MUST be TRUE!
```

### **Problem: Subsequent bootloader commands fail**

**Likely cause:** You wrote `START_MOTOR_CTRL=1` too early

**Fix:** The driver's `applyConfiguration()` already handles this correctly by writing BOOT_CONFIG **LAST**. Just ensure you're using `bootloaderInit()` and not sending manual bootloader commands out of order.

### **Problem: FAULTN is ACTIVE after initialization**

**Likely cause:** This is **NORMAL** if `BL_EXIT_FAULT=1` (default)

**Fix:** Not needed - this is expected behavior. FAULTN asserts when bootloader exits. Check if motor control commands work despite FAULTN being active.

### **Problem: Device doesn't respond at all**

**Possible causes:**
1. Insufficient post-reset delay
2. Wrong SPI/UART configuration
3. Hardware issue

**Fix:**
```cpp
// Increase delays
vTaskDelay(pdMS_TO_TICKS(200));  // After reset release
vTaskDelay(pdMS_TO_TICKS(200));  // After bootloaderInit()

// Verify SPI settings match hardware
cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;

// Check clock configuration
cfg.clock.use_external = tmc9660::bootcfg::ClockSource::Internal;
cfg.clock.pll_selection = tmc9660::bootcfg::SysClkSource::PLL;
```

---

## 📊 **Configuration Memory Map**

CONFIG bank (Bank 5) at base address `0x00020000`:

| Offset | Size | Register | Description |
|--------|------|----------|-------------|
| `0x00` | 4 bytes | `LDO_CONFIG` | LDO voltages and slopes |
| `0x02` | 2 bytes | `UART_ADDR` | Device and host addresses |
| `0x04` | 2 bytes | `RS485_DELAY` | RS485 TX_EN delays |
| `0x06` | 4 bytes | `COMM_CONFIG` | UART/SPI/RS485 selection |
| **`0x08`** | **4 bytes** | **`BOOT_CONFIG`** | **⚠️ BOOT_MODE, START_MOTOR_CTRL** |
| `0x0A` | 4 bytes | `SPI_FLASH` | SPI flash configuration |
| `0x0C` | 4 bytes | `I2C_CONFIG` | I2C EEPROM configuration |
| `0x0E` | 4 bytes | `GPIO_OUT` | GPIO output levels |
| `0x10` | 4 bytes | `GPIO_DIR` | GPIO directions |
| `0x12` | 4 bytes | `GPIO_PU` | GPIO pull-ups |
| `0x14` | 4 bytes | `GPIO_PD` | GPIO pull-downs |
| `0x16` | 4 bytes | `GPIO_ANALOG` | GPIO analog enables |
| `0x18` | 4 bytes | `CLOCK_CONFIG` | Clock and PLL settings |

**⚠️ Write order:**
1. Write `0x00` - `0x06` (LDO, UART, RS485, COMM)
2. Write `0x0A` - `0x18` (Flash, I2C, GPIO, Clock)
3. **FINALLY** write `0x08` (BOOT_CONFIG with START_MOTOR_CTRL=1)

---

## ✅ **Summary**

### **For Development (Runtime Configuration):**

#### **Method 1: Using `applyConfiguration()` (Recommended for full setup)**

```cpp
// 1. Reset chip
gpioSetActive(TMC9660CtrlPin::RST);
vTaskDelay(100ms);
gpioSetInactive(TMC9660CtrlPin::RST);
vTaskDelay(100ms);

// 2. Configure with start_motor_control = TRUE
cfg.boot.start_motor_control = true;  // ← CRITICAL!
driver.bootloaderInit(&cfg);          // Writes START_MOTOR_CTRL=1 LAST
vTaskDelay(150ms);

// 3. Use motor control
driver.motorConfig.setType(...);
```

#### **Method 2: Using `startMotorControl()` (Full manual control)**

```cpp
// 1. Reset chip
gpioSetActive(TMC9660CtrlPin::RST);
vTaskDelay(100ms);
gpioSetInactive(TMC9660CtrlPin::RST);
vTaskDelay(100ms);

// 2. Get bootloader access
TMC9660Bootloader bootloader(comm_interface);

// 3. Manually configure individual settings
bootloader.setBank(5);  // CONFIG bank
bootloader.setAddress(0x00020002);
bootloader.write16(uart_config);
bootloader.setAddress(0x00020006);
bootloader.write32(comm_config);
// ... configure everything you need ...

// 4. Explicitly start motor control (exits bootloader)
bootloader.startMotorControl(tmc9660::bootcfg::BootMode::Parameter);
vTaskDelay(150ms);

// 5. Use motor control
driver.motorConfig.setType(...);
```

#### **Method 3: Hybrid Approach** ⭐ **RECOMMENDED for Advanced Workflows**

```cpp
// 1. Reset chip
gpioSetActive(TMC9660CtrlPin::RST);
vTaskDelay(100ms);
gpioSetInactive(TMC9660CtrlPin::RST);
vTaskDelay(100ms);

// 2. Configure with applyConfiguration (start_motor_control = FALSE)
TMC9660 driver(spi_interface);
tmc9660::BootloaderConfig cfg{};
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
cfg.boot.start_motor_control = false;  // ← Stay in bootloader!
cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;
cfg.clock.use_external = tmc9660::bootcfg::ClockSource::Internal;
cfg.clock.pll_selection = tmc9660::bootcfg::SysClkSource::PLL;
// ... all other settings ...

driver.bootloaderInit(&cfg);  // Applies config but stays in bootloader

// 3. Now do additional bootloader operations
auto* bootloader = driver.getBootloader();  // Get bootloader access
if (bootloader) {
    // Program OTP
    bootloader->setBank(1);
    bootloader->write32Inc(otp_data);
    bootloader->otpBurn(0, 4);
    
    // Verify flash connection
    uint8_t jedecId;
    bootloader->flashReadJedecId(&jedecId);
    ESP_LOGI(TAG, "Flash JEDEC ID: 0x%02X", jedecId);
    
    // Check external memory
    bool isConnected;
    bootloader->memIsConnected(tmc9660::MemoryBank::EEPROM, &isConnected);
    
    // ... any other bootloader operations ...
}

// 4. When ready, explicitly start motor control
bootloader->startMotorControl(tmc9660::bootcfg::BootMode::Parameter);
vTaskDelay(150ms);

// 5. Use motor control
driver.motorConfig.setType(...);
```

### **For Production (OTP Configuration):**

```cpp
// Burn OTP with START_MOTOR_CTRL=1 once during manufacturing
// Then, every power-up:
// → Motor control starts automatically
// → No bootloader commands needed
// → Fastest startup
```

### **Three Initialization Methods Compared:**

| Aspect | **Method 1: Automatic** | **Method 2: Manual** | **Method 3: Hybrid** ⭐ |
|--------|-------------------------|----------------------|------------------------|
| **Complexity** | Low | High | Medium |
| **Configuration** | Automatic | Manual | Automatic + Custom |
| **Flexibility** | Low | High | **Very High** |
| **Use Case** | Simple setup | Full control | **Production/Advanced** |
| **Code** | `cfg.start=true`<br>`bootloaderInit()` | Manual commands<br>`startMotorControl()` | `cfg.start=false`<br>`bootloaderInit()`<br>`getBootloader()`<br>Custom ops<br>`startMotorControl()` |
| **Best For** | ✅ Quick start<br>✅ Standard config | ✅ Custom commands<br>✅ Low-level control | ✅ **OTP programming**<br>✅ **Diagnostics**<br>✅ **Conditional start** |

### **Key Rules:**

1. ✅ **Runtime config:** Set `start_motor_control = true` to exit bootloader (or use `startMotorControl()`)
2. ✅ **Hybrid approach:** Use `start_motor_control = false` + `getBootloader()` + `startMotorControl()`
3. ✅ **Write order:** Always write `BOOT_CONFIG` (offset 0x08) **LAST**
4. ✅ **Timing:** Wait 100-150ms after reset and after starting motor control
5. ✅ **FAULTN:** Expecting it to assert with `BL_EXIT_FAULT=1` is normal
6. ❌ **Never** write `START_MOTOR_CTRL=1` before other configurations
7. ❌ **Never** try to send bootloader commands after `START_MOTOR_CTRL=1` or `startMotorControl()`

---

## 🔗 **Related Documentation**

- [Bootloader Guide](BootloaderGuide.md) - Complete bootloader command reference
- [Bootloader Quick Reference](BootloaderQuickReference.md) - Quick lookup
- [Setup Guide](SetupGuide.md) - Hardware and software setup

---

**Last Updated:** October 10, 2025  
**Driver Version:** v1.0.0

