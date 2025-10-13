---
layout: default
title: TMC9660 Bootloader Quick Reference
---

## 🚀 TMC9660 Bootloader Quick Reference Card

**Quick lookup for common bootloader operations**

---

## 📡 Protocol Comparison

| Feature | SPI | UART |
|---------|-----|------|
| **Frame Size** | 5 bytes (40-bit) | 8 bytes (64-bit) |
| **Bit-Flipping** | ✅ All bytes | ❌ None |
| **Reply Delay** | ✅ Next transaction | ❌ Immediate |
| **Checksum** | ❌ None | ✅ CRC-8 |
| **Addressing** | ❌ None | ✅ Device/Host |
| **Dummy Frame** | ✅ Required | ❌ Not needed |

---

## 📋 Command Codes

```
┌──────┬───────────────────┬─────────────────────────────┐
│ Code │ Command           │ Purpose                     │
├──────┼───────────────────┼─────────────────────────────┤
│ 0x00 │ GET_INFO          │ Get bootloader info         │
│ 0x08 │ GET_BANK          │ Get current bank            │
│ 0x09 │ SET_BANK          │ Select memory bank          │
│ 0x0A │ GET_ADDRESS       │ Get current address         │
│ 0x0B │ SET_ADDRESS       │ Set memory address          │
│ 0x0C │ READ_32           │ Read 32-bit data            │
│ 0x0D │ READ_32_INC       │ Read 32-bit + increment     │
│ 0x0E │ READ_16           │ Read 16-bit data            │
│ 0x0F │ READ_16_INC       │ Read 16-bit + increment     │
│ 0x10 │ READ_8            │ Read 8-bit data             │
│ 0x11 │ READ_8_INC        │ Read 8-bit + increment      │
│ 0x12 │ WRITE_32          │ Write 32-bit value          │
│ 0x13 │ WRITE_32_INC      │ Write 32-bit + increment    │
│ 0x14 │ WRITE_16          │ Write 16-bit value          │
│ 0x15 │ WRITE_16_INC      │ Write 16-bit + increment    │
│ 0x16 │ WRITE_8           │ Write 8-bit value           │
│ 0x17 │ WRITE_8_INC       │ Write 8-bit + increment     │
│ 0x1D │ NO_OP             │ No operation (get reply)    │
│ 0x1E │ OTP_LOAD          │ Load OTP page               │
│ 0x1F │ OTP_BURN          │ Burn OTP page               │
│ 0x20 │ MEM_IS_CONFIGURED │ Check memory config         │
│ 0x21 │ MEM_IS_CONNECTED  │ Check memory connection     │
│ 0x24 │ FLASH_SEND_CMD    │ SPI Flash command           │
│ 0x25 │ FLASH_ERASE_SECTOR│ Erase flash sector          │
│ 0x28 │ MEM_IS_BUSY       │ Check memory busy           │
│ 0xFF │ BOOTSTRAP_RS485   │ Configure RS485             │
└──────┴───────────────────┴─────────────────────────────┘
```

## Status Codes

| Code | Name | Description |
|------|------|-------------|
| 0 | OK | Command executed successfully |
| 1 | CMD_NOT_FOUND | Invalid command number |
| 3 | INVALID_ADDR | Invalid memory address |
| 4 | INVALID_VALUE | Invalid request value |
| 14 | INVALID_BANK | Invalid memory bank |
| 15 | BUSY | Bootloader busy (SPI only) |
| 17 | MEM_UNCONFIGURED | External memory not configured |
| 18 | OTP_ERROR | OTP command failed |
| 19 | SESSION_START | First SPI datagram after power-on (SPI only) |
| 20 | CMD_NOT_AVAILABLE | Command not available |
| 21 | BOOTLOADER_RESUMED | Returned to bootloader from motor control (SPI only) |

---

## 📊 GET_INFO Queries

| Code | Query | Description |
|------|-------|-------------|
| 0 | CHIP_TYPE | Chip type (0x544D0001) |
| 1 | BL_VERSION | Bootloader version |
| 2 | FEATURES | Available features |
| 12 | GIT_INFO | Git version info |
| 13 | CHIP_VERSION | Silicon revision |
| 14 | CHIP_FREQUENCY | System frequency (MHz) |
| 17 | CONFIG_MEM_START | CONFIG memory start |
| 18 | CONFIG_MEM_SIZE | CONFIG memory size |
| 19 | OTP_MEM_SIZE | OTP page size |
| 20 | I2C_MEM_SIZE | I2C memory size |
| 21 | SPI_MEM_SIZE | SPI memory size |
| 22 | PARTITION_VERSION | Partition format version |
| 25 | SPI_MEM_PARTITIONS | SPI partition count |
| 26 | I2C_MEM_PARTITIONS | I2C partition count |
| 28 | CHIP_VARIANT | Chip variant |

---

## 🗄️ Memory Banks

```
0 = RAM           Internal RAM
1 = OTP           One-Time Programmable
2 = SPI_FLASH     External SPI Flash
3 = I2C_EEPROM    External I2C EEPROM
4 = RESERVED      Reserved
5 = CONFIG        Runtime config (0x00020000)
```

---

## ⚡ Common Code Snippets

### Initialize Bootloader
```cpp
tmc9660::BootloaderConfig cfg{};
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;
cfg.boot.start_motor_control = true;
auto result = driver.bootloaderInit(&cfg);
```

### Change Runtime Config
```cpp
bootloader.setBank(MemoryBank::CONFIG);
bootloader.setAddress(0x00020002);
bootloader.write16(0x0403);  // Dev=3, Host=4
```

### Check External Memory
```cpp
bool configured, connected;
bootloader.memIsConfigured(MemoryBank::SPI_FLASH, &configured);
bootloader.memIsConnected(MemoryBank::SPI_FLASH, &connected);
```

### Load OTP Page
```cpp
uint8_t errors, tag;
bootloader.otpLoad(0, &errors, &tag);
```

### Burn OTP Page
```cpp
bootloader.setBank(MemoryBank::OTP);
bootloader.setAddress(0x00000000);
// ... write data ...
bootloader.otpBurn(0, 4);  // Page 0, tag 4
```

### Read Flash ID
```cpp
uint8_t mfgId;
bootloader.flashReadJedecId(&mfgId);
```

### Erase Flash Sector
```cpp
bootloader.flashEraseSector(0x010000);
bool busy;
do {
    bootloader.memIsBusy(MemoryBank::SPI_FLASH, &busy);
} while (busy);
```

### Configure RS485
```cpp
bootloader.bootstrapRS485(
    1,    // TX_EN on GPIO8
    10,   // Pre-delay
    255,  // Host address
    1     // Device address
);
```

---

## 🔍 CONFIG Bank Offsets

```
0x00020000 + Offset:
  0x00 = LDO Config
  0x02 = Device/Host Address
  0x04 = RS485 Delays
  0x06 = Comm Interface Settings
  0x08 = Boot Configuration
  0x0A = SPI Flash Config
  0x0C = I2C EEPROM Config
  0x0E = GPIO Config (start)
  0x18 = Clock Configuration
```

---

## ⚠️ Critical Reminders

1. **SPI**: Always send dummy frame to get reply
2. **SPI**: All bytes are bit-flipped
3. **UART**: CRC-8 over first 7 bytes
4. **OTP**: Can only burn once per page!
5. **Test**: Use CONFIG bank before burning OTP
6. **Verify**: Always check OTP after burning

---

## 🔗 Full Documentation

---

## 🚀 **High-Level Functions**

### **`startMotorControl()`** - Start Motor Control & Exit Bootloader

```cpp
bool startMotorControl(bootcfg::BootMode bootMode = bootcfg::BootMode::Parameter)
```

**Purpose:** Manually start motor control and exit bootloader mode

**When to use:**
- ✅ After manual bootloader configuration
- ✅ For conditional motor startup
- ✅ When you need precise control over timing

**Example:**
```cpp
TMC9660Bootloader bootloader(spi_interface);

// Configure manually
bootloader.setBank(5);
bootloader.setAddress(0x00020002);
bootloader.write16(uart_config);
// ... more config ...

// Start motor control
bootloader.startMotorControl(tmc9660::bootcfg::BootMode::Parameter);
vTaskDelay(150);  // Wait for motor control to initialize
```

**⚠️ CRITICAL:** Bootloader exits immediately after this call!

---

For complete details, examples, and troubleshooting:

👉 **[Bootloader Initialization Guide](BootloaderInitializationGuide.html)** - START HERE for proper initialization  
👉 **[Bootloader Complete Guide](BootloaderGuide.html)** - Full command reference

---

**Quick Reference v1.0** | [Back to Index](index.html)

