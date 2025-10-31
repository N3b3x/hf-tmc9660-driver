---
layout: default
title: "🛠️ Bootloader Troubleshooting"
description: "Complete troubleshooting guide with real-world debugging solutions for bootloader issues"
nav_order: 5
parent: "📚 Documentation"
permalink: /docs/BootloaderTroubleshooting/
---

# 🔧 TMC9660 Bootloader Troubleshooting Guide

Comprehensive troubleshooting guide for TMC9660 bootloader initialization and communication issues, based on real-world debugging experiences.

---

## 📋 Table of Contents

1. [Quick Diagnostic Checklist](#quick-diagnostic-checklist)
2. [Common Issues & Solutions](#common-issues--solutions)
3. [Protocol-Specific Problems](#protocol-specific-problems)
4. [Configuration Issues](#configuration-issues)
5. [Advanced Debugging](#advanced-debugging)

---

## 🚀 Quick Diagnostic Checklist

Use this checklist to quickly identify the most common issues:

```text
Hardware Reset
├─ ✅ RST pin toggled correctly?
├─ ✅ FAULTN monitored during reset?
├─ ✅ Adequate delay after reset (100ms recommended)?
└─ ✅ Power supply stable?

Communication Interface
├─ SPI
│   ├─ ✅ Clock speed ≤ 10MHz?
│   ├─ ✅ CS active low?
│   ├─ ✅ MOSI/MISO not swapped?
│   └─ ✅ Clock polarity/phase correct (Mode 3)?
│
└─ UART
    ├─ ✅ TX/RX pins correct (not swapped)?
    ├─ ✅ Baud rate matches config or auto?
    ├─ ✅ GPIO pins match bootloader config?
    └─ ✅ Half-duplex timing respected?

Protocol
├─ ✅ Using correct sync byte/bit?
├─ ✅ CRC8/checksum calculated correctly?
├─ ✅ Reply timing understood (SPI delayed)?
└─ ✅ Address encoding correct (UART module addr)?

Configuration
├─ ✅ Clock config before communication?
├─ ✅ START_MOTOR_CTRL written last?
├─ ✅ Delays after clock configuration?
└─ ✅ SPI disabled when using UART?
```text

---

## 🐛 Common Issues & Solutions

### Issue 1: No Reply from Bootloader

#### Symptoms
```text
[UART BL TX] 55 01 00 00 00 00 01 1D
E (xxxx) TMC9660_Bus: UART bootloader read failed: expected 8, read 0
```text

#### Root Causes & Solutions

**Cause 1: TX/RX Pin Mismatch**
```text
❌ WRONG Configuration:
ESP32 UART:  TX=GPIO4, RX=GPIO5
TMC9660 BL:  TX=GPIO6, RX=GPIO7
                 ↑         ↑
         Not connected! Pins don't match!

✅ CORRECT Configuration:
ESP32 UART:  TX=GPIO5 ───► RX=GPIO7 (TMC9660)
             RX=GPIO4 ◄─── TX=GPIO6 (TMC9660)

// In bootloader config
cfg.uart.tx_pin = tmc9660::bootcfg::UartTxPin::GPIO6;
cfg.uart.rx_pin = tmc9660::bootcfg::UartRxPin::GPIO7;

// In ESP32 UART config
uart_config.uart.tx_pin = GPIO_NUM_5;  // Connects to TMC9660 RX
uart_config.uart.rx_pin = GPIO_NUM_4;  // Connects to TMC9660 TX
```text

**Cause 2: Wrong CRC8 Algorithm**
```text
❌ Using standard CRC8:
uint8_t crc = crc8_standard(data, 7);  // ❌ WRONG!

✅ Must use bit-reversed algorithm:
uint8_t reverseByte(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

uint8_t crc8Bootloader(const uint8_t* data, size_t len) {
  uint16_t crc = 0;
  const uint16_t POLY = 0x107;
  for (size_t i = 0; i < len; i++) {
    uint8_t byte_reversed = reverseByte(data[i]);
    for (int bit = 7; bit >= 0; bit--) {
      crc = (crc << 1) | ((byte_reversed >> bit) & 1);
      if (crc & 0x100) crc ^= POLY;
    }
  }
  for (int i = 0; i < 8; i++) {
    crc <<= 1;
    if (crc & 0x100) crc ^= POLY;
  }
  return static_cast<uint8_t>(crc & 0xFF);
}
```text

**Cause 3: Incorrect Bootloader Reply Parsing**
```text
❌ WRONG: Missing device address field
struct BootloaderReplyUART {
  uint8_t hostAddr;
  // uint8_t deviceAddr;  ← MISSING!
  uint8_t status;
  uint32_t value;
};

✅ CORRECT: All fields present
struct BootloaderReplyUART {
  uint8_t hostAddr;     // Byte 0
  uint8_t deviceAddr;   // Byte 1 ← CRITICAL!
  uint8_t status;       // Byte 2
  uint32_t value;       // Bytes 3-6
};
```text

---

### Issue 2: Bootloader Reply CRC Failure

#### Symptoms
```text
[UART BL RX] FF 01 00 00 00 00 01 FD
✅ CRC verified: 0xFD
[UART BL RX] FF 01 00 00 02 00 0A 99
❌ CRC failed: expected 0x99, got 0xXX
```text

#### Solutions

**Verify CRC Implementation**
```cpp
// Test with known good examples from datasheet
const uint8_t testFrame[] = {0x55, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01};
uint8_t crc = crc8Bootloader(testFrame, 7);
assert(crc == 0x1D);  // From datasheet example

// Test bootloader reply
const uint8_t reply[] = {0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01};
crc = crc8Bootloader(reply, 7);
// Compare with byte 7 of actual reply
```text

**Common Mistake: Wrong Byte Count**
```text
❌ WRONG: CRC over 8 bytes
crc8Bootloader(frame, 8);  // Includes CRC byte itself!

✅ CORRECT: CRC over first 7 bytes
crc8Bootloader(frame, 7);  // Excludes CRC byte
```text

---

### Issue 3: TMCL Address Mismatch (UART Only)

#### Symptoms
```text
[TMCL TX] Op=0x03, Type=0x0000, Motor=0x00, Value=0x00000000
[UART TX] 03 03 00 00 00 00 00 00 06
E (xxxx) TMC9660_Bus: UART read failed: expected 9, read 0
❌ TMCL communication failed (MST command failed)
```text

#### Root Cause: Incorrect Module Address Encoding

```text
Device Address = 0x01 (bootloader config)

❌ WRONG: Shifted left
out[0] = (addr << 1) | 0x01;
       = (0x01 << 1) | 0x01
       = 0x02 | 0x01
       = 0x03  ← TMC9660 rejects this!

✅ CORRECT: Upper 7 bits reused
out[0] = (addr & 0xFE) | 0x01;
       = (0x01 & 0xFE) | 0x01
       = 0x00 | 0x01
       = 0x01  ← TMC9660 accepts this!
```text

**Detailed Explanation:**
```text
Datasheet: "The module address reuses the upper 7 bits of the 
            bootloader device address"

Device Address: 0x01 = [0][0][0][0][0][0][0][1]
                        └─────────┬─────────┘
                          Bits 7-1 = 0

Byte 0 Format:  [Module Address bits 1-7][Sync bit 0]
                [0][0][0][0][0][0][0]     [1]
                └─────────┬─────────┘     └─ Always 1
                  From device addr bits 7-1

Result: 0x01 (NOT 0x03!)
```text

**Fix in Code:**
```cpp
// TX: Command serialization
void toUart(uint8_t addr, std::span<uint8_t, 9> out) const noexcept {
  // ❌ WRONG
  // out[0] = (addr << 1) | 0x01;
  
  // ✅ CORRECT
  out[0] = (addr & 0xFE) | 0x01;  // Keep upper 7 bits, set sync bit
  // ... rest of frame
}

// RX: Reply validation
static bool fromUart(std::span<const uint8_t, 9> in, uint8_t addr, TMCLReply &r) {
  // ❌ WRONG
  // if ((in[1] >> 1) != (addr & 0x7F)) return false;
  
  // ✅ CORRECT
  if ((in[1] & 0xFE) != (addr & 0xFE)) return false;
  // ... rest of validation
}
```text

---

### Issue 4: Clock Configuration Failures

#### Symptoms
```text
Clock configuration written (external: yes, xtal_drive: 3, rdiv: 15)
Waiting for clock reconfiguration to complete...
E (xxxx) TMC9660Bootloader: Failed to read back clock config
❌ PLL_STATUS not set after clock reconfiguration
```text

#### Solutions

**Solution 1: Adequate Delay After Clock Write**
```cpp
// Write clock configuration
if (!write32(clk)) return false;

// ❌ WRONG: No delay or insufficient delay
comm_.delayMs(10);  // Too short!

// ✅ CORRECT: Adequate delay for PLL lock
comm_.delayMs(100);  // 100ms recommended

// TMC9660 cannot respond during clock reconfiguration!
```text

**Solution 2: Skip PLL Status Polling in Bootloader**
```cpp
// During bootloader phase
if (!write32(clk)) return false;
comm_.delayMs(100);

// ❌ WRONG: Try to poll PLL_STATUS immediately
// (TMC9660 may still be reconfiguring)

// ✅ CORRECT: Fixed delay, verify later via TMCL
// PLL status will be stable in parameter mode
```text

**Solution 3: Correct Clock Configuration Order**
```cpp
// ❌ WRONG: Clock config at end
configure_ldo();
configure_uart();
configure_spi();
configure_clock();  // Too late!

// ✅ CORRECT: Clock config early
configure_ldo();
configure_clock();   // Right after LDO
comm_.delayMs(100);
configure_uart();
configure_spi();
// ... other configs
```text

---

### Issue 5: SPI/UART Conflict

#### Symptoms
```text
Configuring communication settings (shared register with SPI flash)
Current COMM_CONFIG before overwrite: 0x0780
  Current Bit 1 (SPI disabled): 0  ← SPI enabled
NEW COMM_CONFIG to be written: 0x0600
  New Bit 1 (SPI disabled): 0      ← Still enabled!
  
// Later: UART stops working
```text

#### Root Cause: Both SPI and UART Enabled

```text
❌ WRONG: Conflicting configuration
cfg.spiComm.disable_spi = false;   // SPI enabled
cfg.uart.disable_uart = false;     // UART also enabled
                                   ↑
                        Conflict! Only one can be active!

✅ CORRECT: Mutually exclusive
if (use_uart) {
  cfg.spiComm.disable_spi = true;   // Disable SPI
  cfg.uart.disable_uart = false;    // Enable UART
} else {
  cfg.spiComm.disable_spi = false;  // Enable SPI
  cfg.uart.disable_uart = true;     // Disable UART
}
```text

**Additional Fix: Flash Configuration**
```cpp
// When using UART + Flash
cfg.spiComm.disable_spi = true;              // Bootloader uses UART
cfg.spiFlash.enable_flash = true;            // Flash is enabled
cfg.spiFlash.flash_spi_iface = SPIInterface::IFACE1;  // Flash uses SPI1

// COMM_CONFIG register:
// Bit 1: BL_DISABLE_SPI = 1
// Bit 2: BL_SPI_SELECT = 1 (flash uses SPI0)
```text

---

### Issue 6: Memory Bank Address Pointer Reset

#### Symptoms
```text
Configuring SPI flash settings
[UART BL TX] 55 01 0B 00 02 00 0A C8   ← SET_ADDRESS(0x0A)
[UART BL RX] FF 01 00 00 02 00 0A 99   ← OK
[UART BL TX] 55 01 14 00 00 03 61 9F   ← WRITE_16(0x0361)
[UART BL RX] FF 01 00 00 00 03 61 DE   ← OK
[UART BL TX] 55 01 0E 00 00 00 00 0A   ← READ_16
[UART BL RX] FF 01 00 00 00 00 0B 96   ← Wrong value! (0x000B instead of 0x0361)
❌ SPI flash config verification failed: expected=0x0361, actual=0x000B
```text

#### Root Cause: Address Pointer Resetting

The address pointer may reset to 0x00 after certain operations.

**Solution: Re-set Address Before Each Read**
```cpp
// ❌ WRONG: Assume address stays set
setAddress(0x0A);
write16(value);
// Address may have changed!
uint16_t readback = read16();  // Reading from wrong address!

// ✅ CORRECT: Re-set address before read
setAddress(0x0A);
write16(value);
setAddress(0x0A);  // Re-set address!
uint16_t readback = read16();
```text

---

### Issue 7: GetVersion String Format

#### Symptoms
```text
[TMCL RX] 30 30 35 31 56 31 30 30
Checksum validation failed!  ← Byte 2 is '0', not status 100
```text

#### Root Cause: Special Format for Type=0

The `GetVersion` command returns an **ASCII string**, not a normal TMCL reply!

**Solution: Context-Aware Reply Parsing**
```cpp
bool TMCLFrame::fromSpi(std::span<const uint8_t, 8> in, TMCLReply &r,
                        uint8_t tx_opcode, uint16_t tx_type) noexcept {
  r.spiStatus = static_cast<SPIStatus>(in[0]);
  
  // Special case: GetVersion (Type=0) returns ASCII string
  if (tx_opcode == static_cast<uint8_t>(TMCLOperation::GAP) && tx_type == 0) {
    // Bytes 1-8 are version string (e.g., "051V100\0")
    std::copy(in.begin() + 1, in.end(), r.rawBytes.begin());
    r.tmclStatus = TMCLStatus::Success;  // Assume success
    return true;  // Skip checksum validation
  }
  
  // Normal reply: check status byte
  r.tmclStatus = static_cast<TMCLStatus>(in[2]);
  // ... rest of normal parsing
}
```text

---

## 🔍 Protocol-Specific Problems

### SPI Issues

**Problem: All Replies are 0x63 (NOT_READY)**
```text
[SPI TX] 64 00 00 00 00 01 00 00
[SPI RX] 63 00 00 00 00 00 00 00  ← STATUS_NOT_READY

Possible causes:
1. Clock not locked (PLL not ready)
2. Insufficient delay after clock config
3. Commands sent too fast (chip still processing)

Solutions:
• Add 100ms delay after clock configuration
• Implement retry logic (max 100 attempts)
• Reduce SPI clock speed (<= 10MHz)
```text

**Problem: SPI Delayed Replies Not Handled**
```text
❌ WRONG: Expect immediate reply
spi_transfer(GET_INFO);
reply = spi_transfer(dummy_byte);  // Gets previous reply!

✅ CORRECT: Account for 1-command delay
spi_transfer(GET_INFO);
spi_transfer(NO_OP);  // Dummy command
reply = spi_transfer(NO_OP);  // Gets GET_INFO reply
```text

### UART Issues

**Problem: Autobaud Not Working**
```text
// Autobaud requires sync byte 0x55
cfg.uart.baud_rate = BaudRate::Auto16x;

// ❌ WRONG: First byte not 0x55
tx[0] = device_addr;  // Variable value

// ✅ CORRECT: Bootloader automatically uses 0x55
// (Bootloader protocol always starts with 0x55)
```text

**Problem: UART Stops After Configuration**
```text
// Check COMM_CONFIG register bits
Current: 0x0780
  Bit 0 (UART disabled): 0  ← UART enabled
  Bit 7-9 (Baud rate): 7    ← Auto16x

New: 0x0600
  Bit 0 (UART disabled): 0  ← Still enabled ✓
  Bit 7-9 (Baud rate): 3    ← Changed to 57600!
  
Solution: Match baud rate in config and ESP32 UART
```text

---

## ⚙️ Configuration Issues

### Issue: `read32` Failures on UART

#### Symptoms
```text
[UART BL TX] 55 01 0C 00 00 00 00 77  ← READ_32
[UART BL RX] FF 01 00 00 00 00 00 00  ← Returns 0 or fails
```text

#### Solution: Use Two `READ_16` Operations
```cpp
// ❌ PROBLEMATIC: READ_32 on UART
bool read32(uint32_t *value) {
  return sendCommand(READ_32, 0, value);
}

// ✅ WORKAROUND: Use two READ_16_INC
bool read32(uint32_t *value) {
  if (comm_.mode() == CommMode::UART) {
    uint16_t lower16, upper16;
    if (!read16Inc(&lower16)) return false;  // Read lower, increment
    if (!read16(&upper16)) return false;     // Read upper
    *value = (static_cast<uint32_t>(upper16) << 16) | lower16;
    return true;
  }
  // SPI: use normal READ_32
  return sendCommand(READ_32, 0, value);
}
```text

---

## 🔬 Advanced Debugging

### Enable Comprehensive Logging

```cpp
// Set maximum log level
comm_.setLogLevel(4);  // 0=Error, 1=Warn, 2=Info, 3=Debug, 4=Verbose

// Example output for every transaction:
[UART BL TX] 55 01 00 00 00 00 01 1D
[UART BL RX] FF 01 00 00 00 00 01 FD
✅ CRC verified: 0xFD
Chip Type: 0x544D0001 (TMC9660 ✓)
```text

### Decode Raw Bytes

```text
UART Bootloader Command: [55 01 00 00 00 00 01 1D]
  [0] 0x55 = Sync byte
  [1] 0x01 = Device address
  [2] 0x00 = Command (GET_INFO)
  [3-6] 0x00000001 = Value (SW_VERSION)
  [7] 0x1D = CRC8

UART Bootloader Reply: [FF 01 00 00 00 00 01 FD]
  [0] 0xFF = Host address
  [1] 0x01 = Device address
  [2] 0x00 = Status (OK)
  [3-6] 0x00000001 = Version 1.0
  [7] 0xFD = CRC8

UART TMCL Command: [01 06 00 00 00 00 00 00 07]
  [0] 0x01 = Sync bit + address (0x01)
  [1] 0x06 = Operation (GAP)
  [2] 0x00 = Type (axis parameter)
  [3] 0x00 = Motor/Bank
  [4-7] 0x00000000 = Value (param 0)
  [8] 0x07 = Checksum

UART TMCL Reply: [FF 01 64 06 00 00 00 00 ...]
  [0] 0xFF = Host address
  [1] 0x01 = Sync bit + address
  [2] 0x64 = Status (100 = Success)
  [3] 0x06 = Operation (GAP echoed)
  [4-7] 0x00000000 = Value
  [8] 0x... = Checksum
```text

### Logic Analyzer Capture Points

```text
Critical Signals to Monitor:
├─ SPI
│   ├─ CLK: Check frequency (<= 10MHz)
│   ├─ CS: Active low, proper timing
│   ├─ MOSI: Command bytes
│   └─ MISO: Reply bytes (delayed by 1)
│
└─ UART
    ├─ TX: Command frames (check sync byte)
    ├─ RX: Reply frames (check CRC)
    └─ Baud Rate: Must match config

Timing Checks:
├─ Reset pulse: >= 1ms
├─ Post-reset delay: >= 100ms
├─ Clock config delay: >= 100ms
└─ UART command spacing: >= 10ms (autobaud)
```text

---

## 📚 Related Documentation

- **[Communication Protocol Guide](CommunicationProtocolGuide.html)** - Detailed protocol specs
- **[Bootloader Initialization Guide](BootloaderInitializationGuide.html)** - Complete setup process
- **[TMCL Protocol Guide](TMCLProtocolGuide.html)** - Parameter mode operations
- **[Common Operations](CommonOperations.html)** - Practical usage examples

---

## 🆘 Still Having Issues?

If you've tried everything in this guide and still have problems:

1. **Check Hardware First**
   - Verify all connections with multimeter
   - Check power supply voltage and stability
   - Ensure crystal oscillator is working (if using external)

2. **Compare With Working Example**
   - Run the provided ESP32 examples
   - Compare your timing with example code
   - Check if SPI mode works when UART doesn't (or vice versa)

3. **Enable Maximum Logging**
   - Set log level to 4 (verbose)
   - Capture all TX/RX bytes
   - Compare with examples in this guide

4. **Test Individual Components**
   - Test CRC8 calculation with known values
   - Test address encoding with examples
   - Verify reply parsing with captured data

---

*Last updated: 2025 | HF-TMC9660 Driver v1.0*

