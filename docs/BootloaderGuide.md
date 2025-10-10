---
layout: default
title: TMC9660 Bootloader Complete Guide
---

## 🔧 TMC9660 Bootloader Complete Guide

This comprehensive guide covers **all** bootloader functionality for the TMC9660, including detailed protocol specifications, command reference, and practical examples for both SPI and UART interfaces.

---

## 📋 Table of Contents

1. [Overview](#overview)
2. [Communication Protocols](#communication-protocols)
3. [Command Reference](#command-reference)
4. [Memory Banks](#memory-banks)
5. [OTP Operations](#otp-operations)
6. [External Memory](#external-memory)
7. [Configuration Management](#configuration-management)
8. [Practical Examples](#practical-examples)
9. [Troubleshooting](#troubleshooting)

---

## 🎯 Overview

The TMC9660 bootloader provides low-level access to configure the chip before starting motor control. It supports:

- ✅ **Dual Protocol Support**: SPI (5-byte) and UART (8-byte)
- ✅ **18 Commands**: Complete control over chip configuration
- ✅ **6 Memory Banks**: RAM, OTP, SPI Flash, I2C EEPROM, Reserved, CONFIG
- ✅ **Runtime Configuration**: Modify settings without OTP burns
- ✅ **OTP Management**: Burn and verify one-time programmable memory
- ✅ **External Memory**: Full SPI Flash and I2C EEPROM support

---

## 📡 Communication Protocols

### SPI Protocol (40-bit / 5-byte)

```
┌─────────────────────────────────────────────────────────────┐
│                    SPI BOOTLOADER PROTOCOL                   │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Transaction 1: SEND COMMAND                                 │
│  ┌──────────┬──────────────────────────────────────────┐    │
│  │ TX (5B)  │ [CMD_FLIP] [VALUE(32)_FLIP]              │    │
│  │ RX (5B)  │ [PREV_STATUS_FLIP] [PREV_VALUE(32)_FLIP] │    │
│  └──────────┴──────────────────────────────────────────┘    │
│                         ↓                                     │
│  Transaction 2: RECEIVE REPLY                                │
│  ┌──────────┬──────────────────────────────────────────┐    │
│  │ TX (5B)  │ [0x00] [0x00] [0x00] [0x00] [0x00]       │    │
│  │ RX (5B)  │ [STATUS_FLIP] [VALUE(32)_FLIP]           │    │
│  └──────────┴──────────────────────────────────────────┘    │
│                                                               │
│  ⚠️  CRITICAL: All bytes are BIT-FLIPPED!                    │
│  ⚠️  Reply comes in NEXT transaction (standard SPI)          │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

**Key Features:**
- **Frame Size**: 5 bytes (40 bits)
- **Bit-Flipping**: Every byte is bit-reversed (MSB↔LSB)
- **Reply Delay**: Reply to command N comes during transaction N+1
- **Dummy Frame**: Send all zeros to clock out reply

**Bit-Flip Example:**
```
Original:  0xA5 = 10100101
Flipped:   0xA5 = 10100101 (reversed bits)
```

### UART Protocol (64-bit / 8-byte)

```
┌─────────────────────────────────────────────────────────────┐
│                   UART BOOTLOADER PROTOCOL                   │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  COMMAND FRAME (8 bytes):                                    │
│  ┌────┬────┬────┬────────────────────────┬────┐             │
│  │0x55│DEVA│CMD │    VALUE (32-bit)      │CRC8│             │
│  │    │DDR │    │   [MSB ... ... LSB]    │    │             │
│  └────┴────┴────┴────────────────────────┴────┘             │
│   Sync  Dev  Cmd    Byte3 Byte2 Byte1 Byte0  Checksum       │
│                                                               │
│  REPLY FRAME (8 bytes):                                      │
│  ┌────┬────┬────┬────────────────────────┬────┐             │
│  │0x55│HOST│STAT│    VALUE (32-bit)      │CRC8│             │
│  │    │ADDR│US  │   [MSB ... ... LSB]    │    │             │
│  └────┴────┴────┴────────────────────────┴────┘             │
│   Sync  Host Stat   Byte3 Byte2 Byte1 Byte0  Checksum       │
│                                                               │
│  ✅ NO bit-flipping (unlike SPI)                             │
│  ✅ Immediate reply (no delay like SPI)                      │
│  ✅ CRC-8 checksum (polynomial: x⁸+x²+x¹+x⁰)                 │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

**Key Features:**
- **Frame Size**: 8 bytes (64 bits)
- **No Bit-Flipping**: Bytes transmitted as-is
- **Immediate Reply**: Reply comes immediately (no dummy frame needed)
- **CRC-8 Protection**: Polynomial x⁸+x²+x¹+x⁰ over first 7 bytes
- **Addressing**: Device address (default 1) and Host address (default 255)

---

## 📚 Command Reference

### Command Summary Table

| Command | Code | Description | Parameters | Reply |
|---------|------|-------------|------------|-------|
| `SET_BANK` | 0x01 | Select memory bank | Bank ID (0-5) | Status |
| `SET_ADDRESS` | 0x02 | Set memory address | 32-bit address | Status |
| `WRITE_8` | 0x03 | Write 8-bit value | 8-bit value | Status |
| `WRITE_16` | 0x04 | Write 16-bit value | 16-bit value | Status |
| `WRITE_32` | 0x05 | Write 32-bit value | 32-bit value | Status |
| `OTP_BURN` | 0x06 | Burn OTP page | Page + Addr | Status |
| `WRITE_8_INC` | 0x07 | Write 8-bit + increment | 8-bit value | Status |
| `WRITE_16_INC` | 0x08 | Write 16-bit + increment | 16-bit value | Status |
| `WRITE_32_INC` | 0x09 | Write 32-bit + increment | 32-bit value | Status |
| `NO_OP` | 0x0A | No operation | - | Previous reply |
| `OTP_LOAD` | 0x0B | Load OTP page | Page number | Errors + Tag |
| `MEM_IS_CONFIGURED` | 0x0C | Check memory config | Bank ID | 0 or 1 |
| `MEM_IS_CONNECTED` | 0x0D | Check memory connection | Bank ID | 0 or 1 |
| `FLASH_SEND_CMD` | 0x0E | SPI Flash command | See below | Varies |
| `FLASH_ERASE_SECTOR` | 0x0F | Erase flash sector | 24-bit address | Status |
| `MEM_IS_BUSY` | 0x10 | Check memory busy | Bank ID | 0 or 1 |
| `BOOTSTRAP_RS485` | 0x11 | Configure RS485 | Config bytes | Status |
| `GET_INFO` | 0x12 | Query information | Query type | Info value |

### Status Codes

```cpp
enum class BootloaderStatus : uint8_t {
  SESSION_START = 0x01,      // First reply after power-up
  OK = 0x64,                 // Command successful (100)
  INVALID_ADDR = 0x65,       // Invalid memory address (101)
  INVALID_VALUE = 0x66,      // Invalid parameter value (102)
  OTP_ERROR = 0x67,          // OTP operation failed (103)
  MEM_UNCONFIGURED = 0x68,   // External memory not configured (104)
  ERROR = 0xFF               // General error (255)
};
```

---

## 🗄️ Memory Banks

```
┌─────────────────────────────────────────────────────────────┐
│                     MEMORY BANK LAYOUT                       │
├──────┬──────────────────────────────────────────────────────┤
│ ID   │ Name         │ Description                           │
├──────┼──────────────┼───────────────────────────────────────┤
│ 0    │ RAM          │ Internal RAM (temporary storage)      │
│ 1    │ OTP          │ One-Time Programmable memory          │
│ 2    │ SPI_FLASH    │ External SPI Flash memory             │
│ 3    │ I2C_EEPROM   │ External I2C EEPROM memory            │
│ 4    │ RESERVED     │ Reserved for future use               │
│ 5    │ CONFIG       │ Runtime configuration (0x00020000)    │
└──────┴──────────────┴───────────────────────────────────────┘
```

### CONFIG Memory Bank (Bank 5)

The CONFIG bank provides **runtime reconfiguration** without burning OTP:

```
Base Address: 0x00020000
Size: 64 bytes

┌──────────┬─────────────────────────────────────────┐
│ Offset   │ Configuration                           │
├──────────┼─────────────────────────────────────────┤
│ 0x00     │ LDO Configuration (VEXT1, VEXT2)        │
│ 0x02     │ Device/Host Address (UART)              │
│ 0x04     │ RS485 Delays                            │
│ 0x06     │ Communication Interface Settings        │
│ 0x08     │ Boot Configuration                      │
│ 0x0A     │ SPI Flash Configuration                 │
│ 0x0C     │ I2C EEPROM Configuration                │
│ 0x0E-0x15│ GPIO Configuration                      │
│ 0x18     │ Clock Configuration                     │
└──────────┴─────────────────────────────────────────┘
```

**Example: Change UART Addresses at Runtime**
```cpp
bootloader.setBank(MemoryBank::CONFIG);
bootloader.setAddress(0x00020002);  // Device/Host address offset
bootloader.write16(0x0403);         // Device=3, Host=4
```

---

## 🔐 OTP Operations

### OTP Memory Structure

```
┌─────────────────────────────────────────────────────────────┐
│                    OTP PAGE STRUCTURE                        │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Pages 0-3: Configuration Storage (checked in reverse)       │
│  ┌──────────┬──────────┬──────────┬──────────┐              │
│  │ Page 0   │ Page 1   │ Page 2   │ Page 3   │              │
│  │ (First)  │ (Second) │ (Third)  │ (Latest) │              │
│  └──────────┴──────────┴──────────┴──────────┘              │
│       ↑          ↑          ↑          ↑                     │
│       └──────────┴──────────┴──────────┘                     │
│           Bootloader checks 3→2→1→0                          │
│           (uses first valid config found)                    │
│                                                               │
│  Page Tag = 4: Configuration page                            │
│  Other tags: Application-specific data                       │
│                                                               │
│  ⚠️  OTP can only be written ONCE per page!                  │
│  ⚠️  Use pages 0-3 sequentially for config updates           │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### OTP Configuration Workflow

```
┌─────────────────────────────────────────────────────────────┐
│              OTP CONFIGURATION WORKFLOW                      │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Step 1: Test Configuration (Runtime)                        │
│  ┌────────────────────────────────────────┐                 │
│  │ Use CONFIG bank (Bank 5) to test       │                 │
│  │ settings without burning OTP            │                 │
│  └────────────────────────────────────────┘                 │
│                    ↓                                          │
│  Step 2: Prepare Configuration                               │
│  ┌────────────────────────────────────────┐                 │
│  │ Write configuration to RAM/OTP bank     │                 │
│  │ Verify all settings are correct         │                 │
│  └────────────────────────────────────────┘                 │
│                    ↓                                          │
│  Step 3: Burn to OTP                                         │
│  ┌────────────────────────────────────────┐                 │
│  │ otpBurn(page, pageTag=4)                │                 │
│  │ ⚠️  IRREVERSIBLE - Cannot undo!         │                 │
│  └────────────────────────────────────────┘                 │
│                    ↓                                          │
│  Step 4: Verify                                              │
│  ┌────────────────────────────────────────┐                 │
│  │ otpLoad(page, &errors, &tag)            │                 │
│  │ Check error count and page tag          │                 │
│  └────────────────────────────────────────┘                 │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### OTP Code Examples

**Load and Verify OTP Page:**
```cpp
uint8_t errorCount, pageTag;
if (bootloader.otpLoad(0, &errorCount, &pageTag)) {
    printf("OTP Page 0:\n");
    printf("  Bit Errors: %d\n", errorCount);
    printf("  Page Tag: %d\n", pageTag);
    
    if (errorCount > 0) {
        printf("⚠️  Warning: %d bit errors detected!\n", errorCount);
    }
    if (pageTag == 4) {
        printf("✅ Configuration page detected\n");
    }
}
```

**Burn Configuration to OTP:**
```cpp
// STEP 1: Prepare configuration in OTP bank
bootloader.setBank(MemoryBank::OTP);
bootloader.setAddress(0x00000000);

// Write your configuration data...
bootloader.write32(configData1);
bootloader.write32Inc(configData2);
// ... more writes ...

// STEP 2: Burn to OTP page 0 with tag 4 (configuration)
if (bootloader.otpBurn(0, 4)) {
    printf("✅ Configuration burned to OTP page 0\n");
    
    // STEP 3: Verify
    uint8_t errors, tag;
    bootloader.otpLoad(0, &errors, &tag);
    if (errors == 0 && tag == 4) {
        printf("✅ Verification successful!\n");
    }
} else {
    printf("❌ OTP burn failed!\n");
}
```

---

## 💾 External Memory

### SPI Flash Operations

```
┌─────────────────────────────────────────────────────────────┐
│              SPI FLASH COMMAND WORKFLOW                      │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Internal 6-byte buffer for flash commands:                  │
│  ┌────┬────┬────┬────┬────┬────┐                            │
│  │ B0 │ B1 │ B2 │ B3 │ B4 │ B5 │                            │
│  └────┴────┴────┴────┴────┴────┘                            │
│                                                               │
│  Operation Flow:                                             │
│  1. flashLoadBuffer(offset, data)  - Load bytes into buffer │
│  2. flashSendDatagram(numBytes)    - Send to flash          │
│  3. flashReadBuffer(offset, &data) - Read reply from buffer │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

**Read JEDEC Manufacturer ID:**
```cpp
uint8_t manufacturerId;
if (bootloader.flashReadJedecId(&manufacturerId)) {
    printf("Flash Manufacturer ID: 0x%02X\n", manufacturerId);
    
    // Common IDs:
    // 0xEF - Winbond
    // 0xC2 - Macronix
    // 0x20 - Micron
}
```

**Manual Flash Command (Advanced):**
```cpp
// Example: Read Status Register (command 0x05)
// 1. Load command byte
bootloader.flashLoadBuffer(0, 0x05000000);

// 2. Send 2 bytes (command + dummy byte for reply)
bootloader.flashSendDatagram(2);

// 3. Read reply (status in byte 1)
uint32_t data;
bootloader.flashReadBuffer(0, &data);
uint8_t status = (data >> 8) & 0xFF;
printf("Flash Status: 0x%02X\n", status);
```

**Erase Flash Sector:**
```cpp
// Erase sector at address 0x010000
if (bootloader.flashEraseSector(0x010000)) {
    printf("✅ Sector erased\n");
    
    // Wait for erase to complete
    bool busy;
    do {
        bootloader.memIsBusy(MemoryBank::SPI_FLASH, &busy);
        delay_ms(10);
    } while (busy);
    
    printf("✅ Erase complete\n");
}
```

### I2C EEPROM Operations

**Check EEPROM Status:**
```cpp
bool isConfigured, isConnected, isBusy;

// Check if EEPROM is configured in bootloader
bootloader.memIsConfigured(MemoryBank::I2C_EEPROM, &isConfigured);

if (isConfigured) {
    // Check if physically connected
    bootloader.memIsConnected(MemoryBank::I2C_EEPROM, &isConnected);
    
    if (isConnected) {
        // Check if busy with operation
        bootloader.memIsBusy(MemoryBank::I2C_EEPROM, &isBusy);
        
        if (!isBusy) {
            printf("✅ EEPROM ready for operations\n");
        }
    }
}
```

**Write to EEPROM:**
```cpp
// Select I2C EEPROM bank
bootloader.setBank(MemoryBank::I2C_EEPROM);
bootloader.setAddress(0x00000100);  // Address 0x100

// Write data
bootloader.write32(0x12345678);
bootloader.write32Inc(0xABCDEF00);

// Wait for write to complete
bool busy;
do {
    bootloader.memIsBusy(MemoryBank::I2C_EEPROM, &busy);
    delay_ms(5);
} while (busy);
```

---

## ⚙️ Configuration Management

### Runtime Configuration Example

```cpp
// Change device and host addresses without OTP burn
bootloader.setBank(MemoryBank::CONFIG);

// Device address at offset 2 (byte 0), Host at offset 2 (byte 1)
bootloader.setAddress(0x00020002);
bootloader.write16(0x04FF);  // Device=4, Host=255

// Change UART baud rate
bootloader.setAddress(0x00020006);  // Communication config offset
uint32_t commConfig = 0;
// Set BL_UART_BAUDRATE bits [7:9] = 4 (115200 baud)
commConfig |= (4 << 7);
bootloader.write32(commConfig);
```

### Clock Configuration

```cpp
// Configure for external 16MHz crystal with PLL
bootloader.setBank(MemoryBank::CONFIG);
bootloader.setAddress(0x00020018);  // Clock config offset

uint32_t clockConfig = 0;
clockConfig |= 99;              // RESERVED_1 must be 99
clockConfig |= (1 << 8);        // EXT_NOT_INT: Use external clock
clockConfig |= (3 << 9);        // XTAL_CFG: 16MHz crystal
clockConfig |= (1 << 16);       // PLL_OUT_SEL: Use PLL
clockConfig |= (15 << 18);      // RDIV: 16MHz - 1 = 15

bootloader.write32(clockConfig);

// Wait for PLL to lock (check PLL_STATUS bit 30)
delay_ms(10);
uint32_t status;
// Read back to verify
// ... (would need read command - not shown in basic bootloader)
```

### RS485 Configuration

```cpp
// Configure RS485 as FIRST command (before any other communication)
if (bootloader.bootstrapRS485(
    1,    // TX_EN on GPIO8
    10,   // 10 unit pre-delay
    255,  // Host address
    1     // Device address
)) {
    printf("✅ RS485 configured\n");
    // All subsequent commands will use RS485 protocol
}
```

---

## 💡 Practical Examples

### Example 1: Complete Initialization Sequence

```cpp
#include "TMC9660.hpp"
#include "TMC9660Bootloader.hpp"

// Your SPI interface
YourSPIInterface spiInterface;

// Create driver
TMC9660 driver(spiInterface);

// Get bootloader instance (automatically created for SPI/UART)
// Access through driver's bootloader initialization

// Configure for Parameter Mode with custom settings
tmc9660::BootloaderConfig cfg{};

// Boot configuration
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
cfg.boot.start_motor_control = true;
cfg.boot.disable_selftest = false;  // Keep self-test for safety

// Communication settings
cfg.spiComm.boot_spi_iface = tmc9660::bootcfg::SPIInterface::IFACE0;
cfg.uart.baud_rate = tmc9660::bootcfg::BaudRate::BR115200;
cfg.uart.device_address = 1;
cfg.uart.host_address = 255;

// Clock settings (external 16MHz crystal)
cfg.clock.use_external = tmc9660::bootcfg::ClockSource::External;
cfg.clock.pll_selection = tmc9660::bootcfg::SysClkSource::PLL;

// LDO outputs
cfg.ldo.vext1 = tmc9660::bootcfg::LDOVoltage::V3_3;
cfg.ldo.vext2 = tmc9660::bootcfg::LDOVoltage::V5_0;

// Apply configuration
auto result = driver.bootloaderInit(&cfg);
if (result != TMC9660::BootloaderInitResult::Success) {
    printf("❌ Bootloader initialization failed!\n");
    return -1;
}

printf("✅ TMC9660 initialized and ready for motor control\n");
```

### Example 2: OTP Configuration Backup

```cpp
// Save current configuration to OTP for automatic boot
void saveConfigurationToOTP(TMC9660Bootloader& bootloader) {
    printf("Saving configuration to OTP...\n");
    
    // First, test configuration in CONFIG bank
    printf("Step 1: Testing configuration...\n");
    // ... test your settings ...
    
    // Prepare OTP data
    printf("Step 2: Preparing OTP data...\n");
    bootloader.setBank(MemoryBank::OTP);
    bootloader.setAddress(0x00000000);
    
    // Write configuration (example - adjust for your needs)
    bootloader.write32(0x00000000);  // LDO config
    bootloader.write32Inc(0x01FF);   // Device/Host addresses
    // ... write more configuration ...
    
    // Burn to next available OTP page
    uint8_t nextPage = findNextAvailableOTPPage(bootloader);
    if (nextPage >= 4) {
        printf("❌ No OTP pages available!\n");
        return;
    }
    
    printf("Step 3: Burning to OTP page %d...\n", nextPage);
    if (bootloader.otpBurn(nextPage, 4)) {  // Tag 4 = configuration
        printf("✅ Configuration saved to OTP\n");
        
        // Verify
        uint8_t errors, tag;
        bootloader.otpLoad(nextPage, &errors, &tag);
        printf("Verification: %d errors, tag=%d\n", errors, tag);
    } else {
        printf("❌ OTP burn failed!\n");
    }
}

uint8_t findNextAvailableOTPPage(TMC9660Bootloader& bootloader) {
    // Check pages 0-3 to find first empty one
    for (uint8_t page = 0; page < 4; page++) {
        uint8_t errors, tag;
        if (bootloader.otpLoad(page, &errors, &tag)) {
            if (tag == 0) {  // Empty page
                return page;
            }
        }
    }
    return 4;  // No pages available
}
```

### Example 3: External Flash Management

```cpp
// Complete flash management example
class FlashManager {
public:
    FlashManager(TMC9660Bootloader& bl) : bootloader(bl) {}
    
    bool initialize() {
        // Check if flash is configured
        bool configured, connected;
        bootloader.memIsConfigured(MemoryBank::SPI_FLASH, &configured);
        
        if (!configured) {
            printf("❌ SPI Flash not configured in bootloader\n");
            return false;
        }
        
        bootloader.memIsConnected(MemoryBank::SPI_FLASH, &connected);
        if (!connected) {
            printf("❌ SPI Flash not connected\n");
            return false;
        }
        
        // Read manufacturer ID
        uint8_t mfgId;
        if (bootloader.flashReadJedecId(&mfgId)) {
            printf("✅ Flash detected: MFG ID = 0x%02X\n", mfgId);
            return true;
        }
        
        return false;
    }
    
    bool eraseSector(uint32_t address) {
        printf("Erasing sector at 0x%06X...\n", address);
        
        if (!bootloader.flashEraseSector(address)) {
            return false;
        }
        
        // Wait for erase to complete
        return waitForFlashReady();
    }
    
    bool writePage(uint32_t address, const uint8_t* data, size_t len) {
        // Implementation would use flashLoadBuffer and flashSendDatagram
        // This is a simplified example
        printf("Writing %zu bytes to 0x%06X...\n", len, address);
        
        // Select flash bank
        bootloader.setBank(MemoryBank::SPI_FLASH);
        bootloader.setAddress(address);
        
        // Write data in chunks
        for (size_t i = 0; i < len; i += 4) {
            uint32_t word = 0;
            for (size_t j = 0; j < 4 && (i + j) < len; j++) {
                word |= (data[i + j] << (j * 8));
            }
            bootloader.write32Inc(word);
        }
        
        return waitForFlashReady();
    }
    
private:
    TMC9660Bootloader& bootloader;
    
    bool waitForFlashReady(uint32_t timeout_ms = 1000) {
        uint32_t start = millis();
        bool busy;
        
        do {
            bootloader.memIsBusy(MemoryBank::SPI_FLASH, &busy);
            if (!busy) return true;
            delay_ms(10);
        } while ((millis() - start) < timeout_ms);
        
        printf("❌ Flash timeout\n");
        return false;
    }
};
```

---

## 🔧 Troubleshooting

### Common Issues and Solutions

#### Issue 1: "Failed to send command"

**Symptoms:**
```
Failed to send command (op=0x01, type=0x0000)
Failed to set bank 5
```

**Causes & Solutions:**

1. **Wrong Protocol**: Using TMCL (8-byte) instead of bootloader (5-byte) protocol
   - ✅ **Solution**: Use `TMC9660Bootloader` class, not raw TMCL commands

2. **Missing Bit-Flipping (SPI)**: Bytes not bit-flipped
   - ✅ **Solution**: Bootloader class handles this automatically

3. **Missing Dummy Frame (SPI)**: Not sending second transaction for reply
   - ✅ **Solution**: Bootloader class handles two-transaction protocol automatically

4. **Wrong CRC (UART)**: CRC-8 calculation incorrect
   - ✅ **Solution**: Use provided `crc8Bootloader()` function

#### Issue 2: "OTP_ERROR" Status

**Possible Causes:**
- Page already burned
- Invalid page number (>3 for config)
- Clock configuration failed
- Hardware issue

**Debug Steps:**
```cpp
uint8_t errors, tag;
if (bootloader.otpLoad(page, &errors, &tag)) {
    printf("Page %d: errors=%d, tag=%d\n", page, errors, tag);
    if (tag != 0) {
        printf("⚠️  Page already burned!\n");
    }
}
```

#### Issue 3: "MEM_UNCONFIGURED" Status

**Cause**: External memory not configured in bootloader settings

**Solution**:
```cpp
// Check configuration
bool configured;
bootloader.memIsConfigured(MemoryBank::SPI_FLASH, &configured);

if (!configured) {
    // Configure in CONFIG bank
    bootloader.setBank(MemoryBank::CONFIG);
    bootloader.setAddress(0x0002000A);  // SPI Flash config offset
    
    uint32_t flashConfig = 0;
    flashConfig |= (1 << 0);   // SPI_FLASH_EN
    flashConfig |= (7 << 3);   // CS pin = GPIO7
    flashConfig |= (3 << 8);   // Frequency divider
    
    bootloader.write32(flashConfig);
}
```

#### Issue 4: SPI vs UART Confusion

**Remember:**

| Feature | SPI | UART |
|---------|-----|------|
| Frame Size | 5 bytes | 8 bytes |
| Bit-Flipping | ✅ Yes | ❌ No |
| Reply Delay | ✅ Yes (next transaction) | ❌ No (immediate) |
| Checksum | ❌ No | ✅ CRC-8 |
| Addressing | ❌ No | ✅ Device/Host |

### Debug Logging

Enable verbose logging to see all bootloader operations:

```cpp
// In your communication interface implementation
void debugLog(int level, const char* tag, const char* format, va_list args) noexcept override {
    // Level 4 = Verbose (shows all bootloader commands)
    if (level >= 4) {
        vprintf(format, args);
        printf("\n");
    }
}
```

**Example Output:**
```
[TMC9660Bootloader] Sending SPI bootloader command: cmd=0x01, value=0x00000005
[TMC9660Bootloader] SPI command successful: status=0x64, value=0x00000000 (cmd=0x01)
```

---

## 📖 API Quick Reference

### Basic Operations
```cpp
// Memory operations
bootloader.setBank(MemoryBank::CONFIG);
bootloader.setAddress(0x00020000);
bootloader.write8(value);
bootloader.write16(value);
bootloader.write32(value);
bootloader.write8Inc(value);   // Auto-increment
bootloader.write16Inc(value);
bootloader.write32Inc(value);
bootloader.noOp(&reply);        // Get previous reply (SPI)
```

### OTP Operations
```cpp
// Load OTP page
uint8_t errors, tag;
bootloader.otpLoad(page, &errors, &tag);

// Burn OTP page
bootloader.otpBurn(page, pageTag);
```

### External Memory
```cpp
// Check status
bool configured, connected, busy;
bootloader.memIsConfigured(bank, &configured);
bootloader.memIsConnected(bank, &connected);
bootloader.memIsBusy(bank, &busy);

// Flash operations
bootloader.flashLoadBuffer(offset, data);
bootloader.flashReadBuffer(offset, &data);
bootloader.flashSendDatagram(numBytes);
bootloader.flashEraseSector(address);
bootloader.flashReadJedecId(&mfgId);
```

### Configuration
```cpp
// RS485 setup
bootloader.bootstrapRS485(txEnPin, preDelay, hostAddr, devAddr);

// Information queries
uint32_t value;
bootloader.getInfo(InfoQuery::CONFIG_MEM_START, &value);
bootloader.getConfigMemStart(&address);
bootloader.getConfigMemSize(&size);
```

---

## 🎓 Best Practices

### 1. Always Test Before Burning OTP
```cpp
// ✅ GOOD: Test in CONFIG bank first
bootloader.setBank(MemoryBank::CONFIG);
// ... test settings ...
// If everything works, then burn to OTP

// ❌ BAD: Burn directly to OTP without testing
bootloader.setBank(MemoryBank::OTP);
bootloader.otpBurn(0, 4);  // Can't undo if wrong!
```

### 2. Verify OTP After Burning
```cpp
// ✅ GOOD: Always verify
if (bootloader.otpBurn(page, tag)) {
    uint8_t errors, readTag;
    bootloader.otpLoad(page, &errors, &readTag);
    if (errors > 0 || readTag != tag) {
        printf("❌ Verification failed!\n");
    }
}
```

### 3. Check Memory Status Before Operations
```cpp
// ✅ GOOD: Check before using
bool configured, connected;
bootloader.memIsConfigured(MemoryBank::SPI_FLASH, &configured);
bootloader.memIsConnected(MemoryBank::SPI_FLASH, &connected);

if (configured && connected) {
    // Safe to use flash
}
```

### 4. Use Enums Instead of Magic Numbers
```cpp
// ✅ GOOD: Type-safe and readable
bootloader.setBank(MemoryBank::CONFIG);

// ❌ BAD: Magic numbers
bootloader.setBank(5);
```

### 5. Handle All Error Cases
```cpp
// ✅ GOOD: Comprehensive error handling
if (!bootloader.write32(value)) {
    BootloaderStatus status = /* get from reply */;
    switch (status) {
        case BootloaderStatus::INVALID_ADDR:
            printf("Invalid address\n");
            break;
        case BootloaderStatus::OTP_ERROR:
            printf("OTP operation failed\n");
            break;
        // ... handle all cases ...
    }
}
```

---

## 📚 Additional Resources

- **[⚡ Bootloader Quick Reference](BootloaderQuickReference.html)** - Quick lookup card for commands and codes
- **[Common Operations Guide](CommonOperations.html)** - Motor control after bootloader init
- **[API Reference](annotated.html)** - Complete class documentation
- **[Setup Guide](SetupGuide.html)** - Getting started with the driver

---

## 🎯 Summary Checklist

Before moving to motor control, ensure:

- ✅ Understand SPI vs UART protocol differences
- ✅ Know how to use CONFIG bank for testing
- ✅ Understand OTP page structure and burning process
- ✅ Can check external memory status
- ✅ Know how to configure RS485 if needed
- ✅ Familiar with all 18 bootloader commands
- ✅ Have proper error handling in place
- ✅ Enabled debug logging for troubleshooting

---

<div style="text-align: center; margin: 2em 0; padding: 1em; background: #f8f9fa; border-radius: 8px;">
  <strong>🎉 Ready to control motors?</strong><br>
  <a href="CommonOperations.html" style="display: inline-block; margin-top: 0.5em; padding: 0.5em 1em; background: #28a745; color: white; text-decoration: none; border-radius: 4px;">Motor Control Guide →</a>
</div>

---

**Last Updated**: 2024 | TMC9660 Bootloader Complete Guide v1.0

