---
layout: default
title: TMC9660 Communication Protocol Guide
---

# 📡 TMC9660 Communication Protocol Guide

Complete guide to SPI and UART communication protocols for the TMC9660, including bootloader and parameter mode (TMCL) protocols.

---

## 📋 Table of Contents

1. [Protocol Overview](#protocol-overview)
2. [Bootloader Protocol](#bootloader-protocol)
3. [Parameter Mode (TMCL) Protocol](#parameter-mode-tmcl-protocol)
4. [CRC & Checksum Calculations](#crc--checksum-calculations)
5. [Common Pitfalls & Solutions](#common-pitfalls--solutions)

---

## 🎯 Protocol Overview

The TMC9660 supports two communication interfaces:
- **SPI**: Full-duplex, high-speed, delayed replies
- **UART**: Half-duplex, command/reply order, autobaud support

Each interface has two protocols:
- **Bootloader Protocol**: **5-byte frames (40 bits)** for SPI, 8-byte for UART
- **Parameter Mode Protocol (TMCL)**: **8-byte frames (64 bits)** for SPI, 9-byte for UART

```
┌──────────────────────────────────────────────────────────────────┐
│                    TMC9660 COMMUNICATION                         │
├────────────────────────────────┬─────────────────────────────────┤
│         SPI Interface          │      UART Interface             │
├────────────────────────────────┼─────────────────────────────────┤
│  • Full-duplex                 │  • Half-duplex                  │
│  • SPI Mode 3 (CPOL=1, CPHA=1) │  • Sync bit in each frame       │
│  • MSB first, big-endian       │  • Autobaud support             │
│  • Delayed replies (1 cmd)     │  • Immediate replies            │
│  • Bootloader: 40 bits (5 B)   │  • Bootloader: 8 bytes + CRC8   │
│  • TMCL: 64 bits (8 B)         │  • TMCL: 9 bytes + checksum     │
│  • Status: 0xFF/0x64/0xF0      │  • CRC8: Bit-reversed algorithm │
└────────────────────────────────┴─────────────────────────────────┘
```

---

## 🔧 Bootloader Protocol

Used for **initial configuration**, **OTP programming**, and **external memory** operations.

### SPI Bootloader Protocol

**⚠️ CRITICAL: 40-bit Datagrams (5 bytes), NOT 64-bit!**

The bootloader uses **5-byte frames** (40 bits), while parameter mode uses 8-byte frames (64 bits).

#### Command Frame (5 bytes, TX)
```
┌──────┬──────┬────────────────────────┐
│ Byte │  0   │         1-4            │
├──────┼──────┼────────────────────────┤
│ Desc │ CMD  │   VALUE (32-bit)       │
│ Bits │ 8    │   MSB first            │
└──────┴──────┴────────────────────────┘

Example: GET_INFO (SW_VERSION)
TX: [00 00 00 00 01]
     │  └────┬────┘
     │     Value: 1 (SW_VERSION)
     └─ Command: 0 (GET_INFO)

Example: NO_OP (to retrieve reply)
TX: [1D 00 00 00 00]
     │  └────┬────┘
     │     Value: 0
     └─ Command: 0x1D (NO_OP)
```

#### Reply Frame (5 bytes, RX - Delayed by 1 command!)
```
┌──────┬──────┬────────────────────────┐
│ Byte │  0   │         1-4            │
├──────┼──────┼────────────────────────┤
│ Desc │ STAT │   VALUE (32-bit)       │
│ Bits │ 8    │   MSB first            │
└──────┴──────┴────────────────────────┘

Status Codes (Bootloader):
  0x13 (19) = SESSION_START (first reply after power-on, value = SW version)
                Upper 16 bits: major version
                Lower 16 bits: minor version
  0x00 (0)  = OK (command executed successfully)
  0x01 (1)  = CMD_NOT_FOUND (invalid command number)
  0x03 (3)  = INVALID_ADDR (memory address not valid)
  0x04 (4)  = INVALID_VALUE (request has invalid value)
  0x0E (14) = INVALID_BANK (memory bank not valid)
  0x0F (15) = BUSY (still processing last command - SPI only)
  0x11 (17) = MEM_UNCONFIGURED (external memory not configured)
  0x12 (18) = OTP_ERROR (OTP command failed)
  0x14 (20) = CMD_NOT_AVAILABLE (command currently not available)
  0x15 (21) = BOOTLOADER_RESUMED (first datagram after returning from motor control)

Example: First reply (SESSION_START with version 1.0)
RX: [13 00 00 01 00]
     │  └────┬────┘
     │   Version: 0x00010000 = v1.0
     └─ Status: 0x13 (SESSION_START)
```

**⚠️ CRITICAL: SPI Reply Timing**
```
TX Cmd 1 ──────►  [Process]
                     │
TX Cmd 2 ──────►     │          Reply 1 ◄──
                     │
TX Cmd 3 ──────►     │          Reply 2 ◄──
                                    │
                                Reply 3 ◄──

Each TX receives the reply from the PREVIOUS command!
```

### UART Bootloader Protocol

#### Command Frame (8 bytes, TX)
```
┌──────┬──────┬──────┬──────┬────────────────┬──────┐
│ Byte │  0   │  1   │  2   │     3-6        │  7   │
├──────┼──────┼──────┼──────┼────────────────┼──────┤
│ Desc │ SYNC │ DADDR│ CMD  │ VALUE (32-bit) │ CRC8 │
│ Bits │ 8    │ 8    │ 8    │  MSB first     │  8   │
└──────┴──────┴──────┴──────┴────────────────┴──────┘

SYNC  = 0x55 (fixed, for autobaud detection)
DADDR = Device address (default: 1)
CMD   = Bootloader command
VALUE = 32-bit parameter (big-endian)
CRC8  = Non-standard bit-reversed CRC8 (see below)

Example: GET_INFO (SW_VERSION)
TX: [55 01 00 00 00 00 01 1D]
     │  │  │  └──────────┘ │
     │  │  │    Value: 1   │
     │  │  └─ Command: 0   │
     │  └─ Device addr: 1  │
     └─ Sync byte         └─ CRC8
```

#### Reply Frame (8 bytes, RX)
```
┌──────┬──────┬──────┬──────┬────────────────┬──────┐
│ Byte │  0   │  1   │  2   │     3-6        │  7   │
├──────┼──────┼──────┼──────┼────────────────┼──────┤
│ Desc │ HADDR│ DADDR│ STAT │ VALUE (32-bit) │ CRC8 │
│ Bits │ 8    │ 8    │ 8    │  MSB first     │  8   │
└──────┴──────┴──────┴──────┴────────────────┴──────┘

HADDR = Host address (default: 255)
DADDR = Device address (echoed back)
STAT  = Status/error code (0 = OK)
VALUE = 32-bit return value
CRC8  = Calculated over first 7 bytes
```

### Bootloader Commands

| Command | Code | Description | Value In | Value Out |
|---------|------|-------------|----------|-----------|
| `GET_INFO` | 0x00 | Get chip/bootloader info | Info type | Info value |
| `GET_BANK` | 0x08 | Get current memory bank | - | Bank number |
| `SET_BANK` | 0x09 | Set memory bank | Bank number | Bank number |
| `GET_ADDRESS` | 0x0A | Get current address | - | Address |
| `SET_ADDRESS` | 0x0B | Set memory address | Address | Address |
| `READ_32` | 0x0C | Read 32-bit value | - | Read data |
| `READ_32_INC` | 0x0D | Read + increment | - | Read data |
| `READ_16` | 0x0E | Read 16-bit value | - | Read data |
| `READ_16_INC` | 0x0F | Read 16-bit + increment | - | Read data |
| `READ_8` | 0x10 | Read 8-bit value | - | Read data |
| `READ_8_INC` | 0x11 | Read 8-bit + increment | - | Read data |
| `WRITE_32` | 0x12 | Write 32-bit value | Write data | Write data |
| `WRITE_32_INC` | 0x13 | Write 32-bit + increment | Write data | Write data |
| `WRITE_16` | 0x14 | Write 16-bit value | Write data | Write data |
| `WRITE_16_INC` | 0x15 | Write 16-bit + increment | Write data | Write data |
| `WRITE_8` | 0x16 | Write 8-bit value | Write data | Write data |
| `WRITE_8_INC` | 0x17 | Write 8-bit + increment | Write data | Write data |

---

## 📨 Parameter Mode (TMCL) Protocol

Used for **motor control**, **parameter access**, and **real-time operation** after bootloader initialization.

### SPI TMCL Protocol

#### Command Frame (8 bytes / 64 bits, TX)
```
┌──────┬──────┬──────────┬──────┬────────────────┬──────┐
│ Byte │  0   │   1-2    │  3   │     4-7        │  8   │
├──────┼──────┼──────────┼──────┼────────────────┼──────┤
│ Desc │ OP   │ TYPE(12) │MOTOR │ DATA (32-bit)  │ CSUM │
│ Bits │ 0-7  │  8-19    │20-23 │    24-55       │56-63 │
└──────┴──────┴──────────┴──────┴────────────────┴──────┘

⚠️ Note: Type field is 12 bits (not 8 bits!)
  Byte 1: Bits 8-15 of type
  Byte 2: Bits 16-19 of type (upper 4 bits)
          Bits 20-23 contain motor/bank (lower 4 bits)

Example: Read parameter 0 (GetVersion)
TX: [06 00 00 00 00 00 00 06]
     │  └──┬──┘ │  └────┬────┘ │
     │   Type:0 │    Data: 0   │
     │          └─ Motor: 0     │
     └─ Operation: 6 (GAP)    └─ Checksum
```

#### Reply Frame (8 bytes / 64 bits, RX - Delayed by 1 command!)
```
┌──────┬──────┬──────────┬──────┬────────────────┬──────┐
│ Byte │  0   │   1-2    │  3   │     4-7        │  8   │
├──────┼──────┼──────────┼──────┼────────────────┼──────┤
│ Desc │SPIST │  TMCL    │ OP   │ DATA (32-bit)  │ CSUM │
│ Bits │ 0-7  │  STATUS  │20-23 │    24-55       │56-63 │
│      │      │  8-19    │      │                │      │
└──────┴──────┴──────────┴──────┴────────────────┴──────┘

SPIST = SPI Status Code:
  0xFF = SPI_STATUS_OK (normal operation)
  0x00 = SPI_STATUS_CHECKSUM_ERROR
  0x0C = SPI_STATUS_FIRST_CMD (initial response after init)
  0xF0 = SPI_STATUS_NOT_READY (busy, resend command)

TMCL STATUS = TMCL Status Code (12 bits):
  100 = Success
  Other values = Error codes

OP = Operation echoed back (4 bits, bits 20-23)
DATA = Return value (32 bits, big-endian)
CSUM = Checksum (8 bits)
```

**⚠️ SPI_STATUS_NOT_READY Handling:**
When you receive `0xF0` (SPI_STATUS_NOT_READY):
- The system is still busy processing
- The reply is NOT valid
- The command you just sent was IGNORED
- **Solution:** Resend the same command until status changes

### UART TMCL Protocol

#### Command Frame (9 bytes, TX)
```
┌──────┬──────┬──────┬──────┬──────┬────────────────┬──────┐
│ Byte │  0   │  1   │  2   │  3   │     4-7        │  8   │
├──────┼──────┼──────┼──────┼──────┼────────────────┼──────┤
│ Desc │S+ADR │ OP   │ TYPE │MOTOR │ VALUE (32-bit) │ CSUM │
│ Bits │1+7   │ 8    │ 8    │ 4+4  │  MSB first     │  8   │
└──────┴──────┴──────┴──────┴──────┴────────────────┴──────┘

Byte 0 = [Sync bit (0)] [Module Address (bits 1-7)]
  - Sync bit: Always 1
  - Module Address: REUSES upper 7 bits of device address
  
⚠️ CRITICAL: Module address encoding
  Device Address: 0x01 (binary: 00000001)
  Byte 0 Format:  [x][x][x][x][x][x][x][1]
                   └─────────┬─────────┘ └─ Sync
                      Bits 1-7 of device addr
  Result: 0x01 (NOT 0x03!)
  
Checksum = Sum of first 8 bytes (8-bit addition)

Example: Read parameter 0
TX: [01 06 00 00 00 00 00 00 07]
     │  │  │  │              │
     │  │  │  └─ Motor: 0    │
     │  │  └─ Type: 0        │
     │  └─ Operation: 6     │
     └─ 0x01 = address 1   └─ Checksum
        with sync bit set
```

#### Reply Frame (9 bytes, RX)
```
┌──────┬──────┬──────┬──────┬──────┬────────────────┬──────┐
│ Byte │  0   │  1   │  2   │  3   │     4-7        │  8   │
├──────┼──────┼──────┼──────┼──────┼────────────────┼──────┤
│ Desc │ HADDR│S+ADR │STATUS│ OP   │ VALUE (32-bit) │ CSUM │
│ Bits │ 8    │1+7   │ 8    │ 8    │  MSB first     │  8   │
└──────┴──────┴──────┴──────┴──────┴────────────────┴──────┘

HADDR  = Host address (from bootloader config)
S+ADR  = [Sync bit] [Module address]
STATUS = TMCL status (100 = success)
OP     = Operation echoed back
VALUE  = Return value
CSUM   = Sum of first 8 bytes
```

### TMCL Operations

| Operation | Code | Description |
|-----------|------|-------------|
| `ROR` | 0x01 | Rotate Right |
| `ROL` | 0x02 | Rotate Left |
| `MST` | 0x03 | Motor Stop |
| `MVP` | 0x04 | Move to Position |
| `SAP` | 0x05 | Set Axis Parameter |
| `GAP` | 0x06 | Get Axis Parameter |
| `STAP` | 0x07 | Store Axis Parameter |
| `RSAP` | 0x08 | Restore Axis Parameter |
| `SGP` | 0x09 | Set Global Parameter |
| `GGP` | 0x0A | Get Global Parameter |
| `STGP` | 0x0B | Store Global Parameter |
| `RSGP` | 0x0C | Restore Global Parameter |

---

## 🔐 CRC & Checksum Calculations

### Bootloader CRC8 (Non-Standard!)

**⚠️ CRITICAL**: This is a **non-standard bit-reversed CRC8** algorithm!

```
Polynomial: x^8 + x^2 + x^1 + x^0 = 0x107 (9-bit form)
Initial value: 0
Final XOR: None
Reflection: EACH INPUT BYTE is bit-reversed!
```

#### Algorithm
```cpp
// Step 1: Bit-reverse helper
uint8_t reverseByte(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;  // Swap nibbles
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;  // Swap pairs
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;  // Swap bits
  return b;
}

// Step 2: CRC8 calculation
uint8_t crc8Bootloader(const uint8_t* data, size_t len) {
  uint16_t crc = 0;
  const uint16_t POLY = 0x107;  // 9-bit polynomial
  
  // Process each input byte (bit-reversed!)
  for (size_t i = 0; i < len; i++) {
    uint8_t byte_reversed = reverseByte(data[i]);
    
    // Process each bit (MSB first after reversal)
    for (int bit = 7; bit >= 0; bit--) {
      crc = (crc << 1) | ((byte_reversed >> bit) & 1);
      if (crc & 0x100) {
        crc ^= POLY;
      }
    }
  }
  
  // Final 8 shifts
  for (int i = 0; i < 8; i++) {
    crc <<= 1;
    if (crc & 0x100) {
      crc ^= POLY;
    }
  }
  
  return static_cast<uint8_t>(crc & 0xFF);
}
```

#### Example Calculation
```
Input: [55 01 00 00 00 00 01]
       (SYNC, ADDR=1, CMD=0, VALUE=1)

Step 1: Bit-reverse each byte
  0x55 -> 0xAA
  0x01 -> 0x80
  0x00 -> 0x00
  0x00 -> 0x00
  0x00 -> 0x00
  0x00 -> 0x00
  0x01 -> 0x80

Step 2: Process through CRC algorithm
  Result: 0x1D

Final frame: [55 01 00 00 00 00 01 1D]
```

### TMCL Checksum (Simple 8-bit Addition)

```cpp
uint8_t tmclChecksum(const uint8_t* bytes, size_t n) {
  uint8_t sum = 0;
  for (size_t i = 0; i < n; ++i) {
    sum += bytes[i];  // 8-bit wraparound addition
  }
  return sum;
}
```

#### Example Calculation
```
Input: [01 06 00 00 00 00 00 00]

Checksum = 0x01 + 0x06 + 0x00 + ... = 0x07

Final frame: [01 06 00 00 00 00 00 00 07]
```

---

## ⚠️ Common Pitfalls & Solutions

### 1. Module Address Encoding (UART TMCL)

❌ **WRONG:**
```cpp
// DON'T shift the address left!
out[0] = (addr << 1) | 0x01;  // ❌ WRONG!
```

✅ **CORRECT:**
```cpp
// Module address REUSES upper 7 bits of device address
out[0] = (addr & 0xFE) | 0x01;  // ✅ CORRECT!
```

**Why?** The datasheet says:
> "The module address reuses the upper 7 bits of the bootloader device address."

This means if `device_address = 0x01`, then:
- Byte 0 = `[0][0][0][0][0][0][0][1]` → bits 7-1 are 0, bit 0 is sync
- Result: `0x01` (NOT `0x03`!)

### 2. SPI Delayed Replies

❌ **WRONG:**
```cpp
// Expecting reply immediately
spi_transfer(cmd);
reply = spi_transfer(dummy);  // ❌ Gets previous reply!
```

✅ **CORRECT:**
```cpp
// Send dummy command to get reply
spi_transfer(cmd1);
reply1 = spi_transfer(cmd2).reply;  // Gets reply for cmd1
reply2 = spi_transfer(NO_OP).reply; // Gets reply for cmd2
```

### 3. Bootloader CRC8

❌ **WRONG:**
```cpp
// Using standard CRC8
crc8_standard(data, len);  // ❌ WRONG algorithm!
```

✅ **CORRECT:**
```cpp
// Must use bit-reversed algorithm
crc8Bootloader(data, len);  // ✅ Custom algorithm required
```

### 4. UART Command/Reply Order

❌ **WRONG:**
```cpp
// Sending commands without waiting for replies
uart_send(cmd1);
uart_send(cmd2);  // ❌ Violates protocol!
```

✅ **CORRECT:**
```cpp
// Strict command/reply order
uart_send(cmd1);
uart_receive(reply1);  // Must receive before next command
uart_send(cmd2);
uart_receive(reply2);
```

### 5. GetVersion Special Format

The `GetVersion` command (Type=0) returns an **ASCII string** instead of a status code!

❌ **WRONG:**
```cpp
// Checking byte 2 as status
if (reply[2] != 100) return false;  // ❌ Not a status!
```

✅ **CORRECT:**
```cpp
// Special handling for GetVersion
if (is_get_version_command) {
  // Bytes 1-8 are ASCII string (e.g., "051V100")
  extract_version_string(reply + 1, 8);
} else {
  // Normal status check
  if (reply[2] != 100) return false;
}
```

---

## 📊 Protocol Comparison Table

| Feature | SPI Bootloader | UART Bootloader | SPI TMCL | UART TMCL |
|---------|----------------|-----------------|----------|-----------|
| **Frame Size** | **5 bytes (40 bits)** | 8 bytes | **8 bytes (64 bits)** | 9 bytes |
| **Sync** | Status byte | 0x55 | SPI status | Bit in byte 0 |
| **Reply Timing** | Delayed (1 cmd) | Immediate | Delayed (1 cmd) | Immediate |
| **Checksum** | None | CRC8 (custom) | 8-bit sum | 8-bit sum |
| **Addressing** | No address | Device addr | No address | Module addr |
| **Autobaud** | No | Yes | No | Yes |
| **Half/Full Duplex** | Full | Half | Full | Half |
| **Type Field** | N/A | N/A | **12 bits** | 8 bits |
| **NOT_READY Status** | 0x63 | N/A | 0xF0 | N/A |

---

## 🔍 Debugging Tips

### Enable Debug Logging
```cpp
// Set log level to see all communication
comm.setLogLevel(4);  // 0=Error, 1=Warn, 2=Info, 3=Debug, 4=Verbose

// Example output:
// [UART BL TX] 55 01 00 00 00 00 01 1D
// [UART BL RX] FF 01 00 00 00 00 01 FD
```

### Common Error Patterns

**Pattern 1: All CRC failures**
→ Check: Are you using the bit-reversed CRC8 algorithm?

**Pattern 2: UART no reply**
→ Check: Are pins correct? Is autobaud working? Try fixed baud rate.

**Pattern 3: SPI status always 0x63**
→ Check: Clock configuration, PLL lock, delays after config

**Pattern 4: TMCL address mismatch**
→ Check: Module address uses upper 7 bits (don't shift left!)

---

## 📚 Additional Resources

- **[Bootloader Initialization Guide](BootloaderInitializationGuide.html)** - Complete bootloader setup
- **[TMCL Protocol Guide](TMCLProtocolGuide.html)** - Detailed TMCL operations
- **TMC9660 Datasheet** - Official protocol specifications
- **[Common Operations](CommonOperations.html)** - Practical usage examples

---

*Last updated: 2025 | HF-TMC9660 Driver v1.0*

