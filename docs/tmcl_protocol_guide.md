---
layout: default
title: "📨 TMCL Protocol Guide"
description: "Parameter mode command reference and usage patterns for TMCL scripting"
nav_order: 7
parent: "📚 Documentation"
permalink: /docs/TMCLProtocolGuide/
---

# TMC9660 TMCL Protocol Guide

## Table of Contents
1. [Overview](#overview)
2. [Communication Modes](#communication-modes)
3. [Command Structure](#command-structure)
4. [Address Encoding](#address-encoding)
5. [Checksum Calculation](#checksum-calculation)
6. [SPI Protocol Details](#spi-protocol-details)
7. [UART Protocol Details](#uart-protocol-details)
8. [Special Cases](#special-cases)
9. [Common Pitfalls](#common-pitfalls)

---

## Overview

The TMC9660 Parameter Mode uses the TMCL (Trinamic Motion Control Language) protocol
for communication over both SPI and UART interfaces. This guide provides detailed
information about the protocol implementation, including critical details learned
through extensive debugging.

### Key Characteristics
- **Strict Command/Reply Order**: Never send a new command before receiving the
  reply to the previous command
- **Shared Structure**: SPI and UART use the same command format with interface-specific framing
- **Checksum Protected**: All datagrams include a checksum for data integrity
- **Special Reply Formats**: Some commands (like GetVersion) return non-standard formats

---

## Communication Modes

### SPI Mode
- **Interface**: 4-wire synchronous (MOSI, MISO, SCK, CS)
- **Speed**: Up to 4 MHz
- **Data Size**: 8 bytes per transaction
- **Reply Delay**: Reply to command N is received when sending command N+1

### UART Mode
- **Interface**: 2-wire asynchronous (TX, RD)
- **Speed**: 9600 to 1000000 baud, or autobaud
- **Data Size**: 9 bytes per transaction (includes sync bit and host address)
- **Reply Order**: Immediate reply after command

---

## Command Structure

Both SPI and UART share the same core TMCL command structure:

### Core Fields (8 bytes for SPI, 9 bytes for UART)

| Field | Size | Description | Notes |
|-------|------|-------------|-------|
| **Operation** | 1 byte | TMCL operation code | e.g., 0x05 for SAP, 0x06 for GAP, 0x88 for GetVersion |
| **Type** | 12 bits | Parameter type or address | SPI: split across bytes 1-2, UART: 8 bits in byte 2 |
| **Motor/Bank** | 4 bits | Motor number or memory bank | SPI: in byte 2 upper nibble, UART: in byte 3 lower nibble |
| **Value** | 4 bytes | Data value (big-endian) | MSB first |

### SPI Frame Format (8 bytes)

⚠️ **CRITICAL: Type field is 12 bits, not 8 bits! Motor shares Byte 2 with Type!**

```text
┌──────┬──────┬─────────────┬──────────────────┬──────┐
│ Byte │  0   │      1-2     │      3-6         │  7   │
├──────┼──────┼─────────────┼──────────────────┼──────┤
│ Desc │ OP   │ TYPE+MOTOR   │ DATA (32-bit)    │ CSUM │
│ Bits │ 0-7  │  8-19 + 20-23│    24-55         │56-63 │
└──────┴──────┴─────────────┴──────────────────┴──────┘

Detailed Byte Layout:
┌──────┬──────┬──────────────────┬──────────────────┐
│ Byte │  0   │       1          │       2          │
├──────┼──────┼──────────────────┼──────────────────┤
│ Field│ OP   │ Type bits 0-7    │ Type 8-11 | Motor│
│ Bits │ 0-7  │    8-15          │  16-19   | 20-23 │
│      │      │ (lower 8 bits)   │(lower)   |(upper)│
└──────┴──────┴──────────────────┴──────────────────┘

Note: Byte 2 lower nibble = Type bits 16-19 (frame bits), Byte 2 upper nibble = Motor bits 20-23 (frame bits)

Frame Bit Positions (from datasheet Table 3):
- Byte 1: Frame bits 8-15 = Type bits 8-15 (Type lower 8 bits of 12-bit field)
- Byte 2 lower nibble: Frame bits 16-19 = Type bits 16-19 (Type upper 4 bits of 12-bit field)
- Byte 2 upper nibble: Frame bits 20-23 = Motor/Bank (4 bits)
- Bytes 3-6: Frame bits 24-55 = 32-bit Value (big-endian)
```

**Encoding Example (Type=110=0x006E, Motor=0):**
- Byte 0 (frame bits 0-7): Opcode = 0x05 (SAP)
- Byte 1 (frame bits 8-15): `type & 0xFF` = `0x006E & 0xFF` = `0x6E` (Type lower 8 bits)
- Byte 2 (frame bits 16-23):
  - Lower nibble (bits 0-3, frame bits 16-19): `(type & 0xF00) >> 8` = `(0x006E & 0xF00) >> 8` = `0x00`
  - Upper nibble (bits 4-7, frame bits 20-23): `(motor & 0x0F) << 4` = `(0 & 0x0F) << 4` = `0x00`
  - Result: `0x00 | 0x00` = `0x00`
- Bytes 3-6 (frame bits 24-55): Value (big-endian, 32-bit)
- Byte 7 (frame bits 56-63): Checksum (sum of bytes 0-6)

**Decoding:**
- Type = `in[1] | ((in[2] & 0x0F) << 8)`
  - From Byte 1 (frame bits 8-15): Type lower 8 bits
  - From Byte 2 lower nibble (frame bits 16-19): Type upper 4 bits, shifted left by 8
  - Example: `0x6E | (0x00 << 8)` = `0x006E` = 110
- Motor = `(in[2] >> 4) & 0x0F`
  - From Byte 2 upper nibble (frame bits 20-23)
  - Example: `(0x00 >> 4) & 0x0F` = `0x00` = 0

### UART Frame Format (9 bytes)

```yaml
Byte:  0             1        2        3        4-7      8
Field: [Sync+Addr  ] [OpCode] [Type  ] [Motor ] [Value (32-bit) ] [Checksum]
       └─ Bit 0: Sync (always 1)
       └─ Bits 1-7: Module Address
       
Note: UART Type field is 8 bits (byte 2), Motor is 4 bits in byte 3 lower nibble
```

---

## Address Encoding

### ⚠️ CRITICAL: Module Address Encoding

The datasheet states: **"The module address reuses the upper 7 bits of the bootloader device address"**

This is **NOT** a shift operation! Here's the correct implementation:

#### UART TX (Command)
```yaml
// INCORRECT (common mistake):
out[0] = (addr << 1) | 0x01;  // ❌ WRONG - shifts the address

// CORRECT:
out[0] = (addr & 0xFE) | 0x01;  // ✅ Keeps upper 7 bits, sets sync bit
```

#### UART RX (Reply)
```yaml
// INCORRECT (common mistake):
if ((in[1] >> 1) != (addr & 0x7F))  // ❌ WRONG - shifts for comparison

// CORRECT:
if ((in[1] & 0xFE) != (addr & 0xFE))  // ✅ Compares upper 7 bits directly
```

#### Why This Matters
- **Byte 0 Format**: `[Sync bit (bit 0)] [Module Address (bits 1-7)]`
- **Address = 0x01** (binary: `00000001`):
  - **Wrong method** produces: `0x03` (binary: `00000011`) - module address becomes `0x01` in bits 1-7
  - **Correct method** produces: `0x01` (binary: `00000001`) - module address is `0x00` in bits 1-7
  
The device address `0x01` means module address `0x00` (upper 7 bits = `0000000`), NOT
module address `0x01`!

### Example Address Mappings

| Device Address | Binary | Upper 7 Bits | Byte 0 (UART TX) | Module Address (bits 1-7) |
|----------------|---------|--------------|------------------|---------------------------|
| 0x00 | 00000000 | 0000000 | 0x01 | 0x00 |
| 0x01 | 00000001 | 0000000 | 0x01 | 0x00 |
| 0x02 | 00000010 | 0000001 | 0x03 | 0x01 |
| 0xFE | 11111110 | 1111111 | 0xFF | 0x7F |
| 0xFF | 11111111 | 1111111 | 0xFF | 0x7F |

---

## Checksum Calculation

### TMCL Checksum (8-bit Addition)

The TMCL checksum is calculated by summing all bytes (excluding the checksum byte itself)
using 8-bit arithmetic:

```cpp
uint8_t tmclChecksum(const uint8_t *bytes, size_t n) {
    uint8_t sum = 0;
    for (size_t i = 0; i < n; ++i)
        sum += bytes[i];
    return sum;
}
```

#### For UART
- **Checksum covers**: Bytes 0-7 (including sync+address byte)
- **Checksum stored**: Byte 8

#### For SPI
- **Checksum covers**: Bytes 0-6 (operation through value)
- **Checksum stored**: Byte 7

### Example Calculation

Command: `SAP 4, 0, 1000` (Set Axis Parameter: Type=4, Motor=0, Value=1000)
```yaml
Byte 0: 0x05 (SAP opcode = 0x05)
Byte 1: 0x04 (Type lower 8 bits = 0x04)
Byte 2: 0x00 ((Motor << 4) | (Type upper 4 bits)) = (0 << 4) | 0 = 0x00
Byte 3: 0x00 (Value MSB)
Byte 4: 0x00
Byte 5: 0x03 (Value)
Byte 6: 0xE8 (Value LSB = 1000 decimal)
Checksum: 0x05 + 0x04 + 0x00 + 0x00 + 0x00 + 0x03 + 0xE8 = 0xF4
```

---

## SPI Protocol Details

### Transaction Flow

```text
[Master] → Command N → [Slave]
         ← Reply to Command N-1 ←
```

The SPI interface has a **one-transaction delay**:
- First command: Send command, receive `SESSION_START` (0x13) or `FIRST_CMD` (0x0C)
- Subsequent commands: Send command N, receive reply to command N-1

### SPI Status Byte

The first byte of every SPI reply contains a status code:

| Status | Value | Meaning |
|--------|-------|---------|
| `SPI_STATUS_OK` | 0xFF | Operation successful, valid data follows |
| `SESSION_START` | 0x13 | First communication after reset/power-on |
| `FIRST_CMD` | 0x0C | First command in session |
| `SPI_STATUS_NOT_READY` | 0xF0 | Device busy, resend command |
| `CHECKSUM_ERROR` | 0x00 | Checksum verification failed |

### SPI Reply Format

```yaml
Byte 0: SPI Status (0xFF, 0x13, 0x0C, 0xF0, or 0x00)
Byte 1: TMCL Status (100 = REPLY_OK, errors vary)
Byte 2: Operation (echoed from command)
Byte 3-6: Value (big-endian, 32-bit)
Byte 7: Checksum (sum of bytes 0-6)
```

**⚠️ Important:** The SPI reply format does NOT include a separate host address byte. The
first byte is always the SPI status code.

### SPI_STATUS_NOT_READY Retry Logic

The driver includes **automatic retry logic** for `SPI_STATUS_NOT_READY` (0xF0) responses:

```cpp
// Configurable retry settings (defaults):
comm.setSpiRetryMaxCount(3);        // Default: 3 retries
comm.setSpiRetryInterval(100);      // Default: 100 microseconds

// When SPI_STATUS_NOT_READY is received:
// 1. Command is automatically resent after delay
// 2. Retries continue up to maxRetryCount
// 3. If max retries exceeded, returns false with NOT_READY status
```

**Retry Behavior:**
- Retry interval is in **microseconds** (not milliseconds) for precise timing
- Each retry sends the **same command** again
- Logging shows retry attempts: `⚠️  SPI_STATUS_NOT_READY received, retrying (attempt 1/4)
  after 100 us`
- If all retries fail, the reply structure is manually populated and function returns `false`

### Polling for SPI_STATUS_OK

After bootloader exits and motor control starts:
```cpp
const int MAX_ATTEMPTS = 10;
for (int i = 0; i < MAX_ATTEMPTS; i++) {
    sendCommand(NO_OP, 0, 0);  // Send dummy command
    if (reply.spiStatus == SPIStatus::OK)
        break;
    delayMs(10);
}
```

---

## UART Protocol Details

### Frame Format

#### Command (Host → TMC9660)
```yaml
Byte 0: Sync bit (bit 0 = 1) + Module Address (bits 1-7)
Byte 1: Operation
Byte 2: Type
Byte 3: Motor/Bank
Byte 4-7: Value (big-endian)
Byte 8: Checksum (sum of bytes 0-7)
```

#### Reply (TMC9660 → Host)
```yaml
Byte 0: Host Address
Byte 1: Sync bit (bit 0 = 1) + Module Address (bits 1-7)
Byte 2: TMCL Status
Byte 3: Operation (echoed)
Byte 4-7: Value (big-endian)
Byte 8: Checksum (sum of bytes 0-7)
```

### Autobaud Detection

The sync bit (always 1 in byte 0) is used for automatic baud rate detection:

**Autobaud Modes:**
- `Auto8x` (value 6): 8x oversampling - less precise, faster detection
- `Auto16x` (value 7): 16x oversampling - more precise, slower detection

**Autobaud Restart Conditions:**
- After sending each reply
- After receiving an invalid datagram (wrong sync, address, or checksum)
- After 10ms timeout with incomplete data

### UART Timing Requirements

1. **After Hardware Reset**: Wait 100ms for UART stabilization
2. **Command/Reply Order**: Wait for complete reply before sending next command
3. **Incomplete Data Timeout**: 10ms - incomplete datagrams are dropped

---

## Special Cases

### GetVersion Command (0x88)

The GetVersion command has **TWO different reply formats** depending on the Type field:

#### Type 0: String Format (8-character version)
```yaml
SPI Format:  [Host Addr] [8-character ASCII string]
UART Format: [Host Addr] [Sync+Addr] [7-character ASCII string]

Example: "051V100\0" (firmware version 0.51, HW version 1.00)
```

**⚠️ CRITICAL**: String format replies do **NOT** include a valid checksum! The checksum byte
is part of the version string. Your parser must:
1. Detect GetVersion Type=0 was sent
2. Skip checksum validation
3. Extract ASCII string from reply bytes

```cpp
// Correct implementation:
if (sentOpcode == 136 && sentType == 0) {
    // Skip checksum validation for string format
    extractVersionString(reply);
    return true;
}
```

#### Type 1: Binary Format (standard TMCL)
```text
Standard TMCL reply with 32-bit value and valid checksum
Value contains version info encoded as integers
```

### SESSION_START Reply (SPI Only)

When first communicating after reset:
```text
[Master] → First Command → [Slave]
         ← SESSION_START (0x13) ←
```

The SESSION_START reply format:
```yaml
Byte 0: 0x13 (SESSION_START)
Byte 1: 0x00
Byte 2: Bootloader version (e.g., 0x01 for v1.0)
Byte 3-7: Don't care
```

---

## Common Pitfalls

### 1. Address Encoding Error
**Problem**: Shifting the address left instead of keeping upper 7 bits
```cpp
// WRONG:
out[0] = (addr << 1) | 0x01;

// CORRECT:
out[0] = (addr & 0xFE) | 0x01;
```

### 2. GetVersion String Format Not Handled
**Problem**: Trying to validate checksum on string-format GetVersion reply
**Solution**: Pass command context (`sentOpcode`, `sentType`) to reply parser

### 3. Not Waiting for SPI_STATUS_OK
**Problem**: Sending commands immediately after bootloader exits
**Solution**: Poll with NO_OP commands until `SPI_STATUS_OK` is received

### 4. UART Address Mismatch
**Problem**: Module address in reply doesn't match expected address
**Solution**: Ensure device address in bootloader config matches driver address

### 5. Checksum Includes Wrong Bytes
**Problem**: Calculating checksum over wrong byte range
**Solution**:
- **UART**: Checksum covers bytes 0-7 (including sync+address)
- **SPI**: Checksum covers bytes 0-6 (operation through value)

### 6. SPI Reply Delay Not Handled
**Problem**: Expecting immediate reply to first SPI command
**Solution**: Handle `SESSION_START` or `FIRST_CMD` status codes, understand one-transaction delay

### 7. Incomplete UART Data Timeout
**Problem**: Waiting forever for incomplete UART datagram
**Solution**: Implement 10ms timeout, drop incomplete data

### 8. Incorrect SPI Type Field Encoding
**Problem**: Treating Type field as 8 bits instead of 12 bits, or incorrect byte packing
**Solution**:
- **BYTE 1**: Type bits 0-7 (lower 8 bits) = `type & 0xFF`
- **BYTE 2**: Upper nibble = Motor, Lower nibble = Type bits 8-11 = `((motor & 0x0F) << 4)
  | ((type & 0xF00) >> 8)`
- Always use `TMCLFrame::toSpi()` for encoding and `TMCLReply::fromSpi()` for decoding

### 9. SPI_STATUS_NOT_READY Not Handled
**Problem**: Not retrying commands that return `SPI_STATUS_NOT_READY`
**Solution**: Use the built-in retry logic with `setSpiRetryMaxCount()` and
`setSpiRetryInterval()`, or implement manual retry in your application code

---

## Implementation Checklist

### UART Communication
- [ ] Address encoding uses upper 7 bits (not shifted)
- [ ] Checksum covers all 8 bytes (including sync+address)
- [ ] Module address validation compares upper 7 bits
- [ ] 100ms delay after hardware reset
- [ ] GetVersion string format handled (no checksum validation)
- [ ] 10ms timeout for incomplete datagrams

### SPI Communication
- [ ] Handle `SESSION_START` on first command
- [ ] Handle one-transaction reply delay
- [ ] Poll for `SPI_STATUS_OK` after bootloader exit
- [ ] Configure retry logic for `SPI_STATUS_NOT_READY` (`setSpiRetryMaxCount`, `setSpiRetryInterval`)
- [ ] Checksum covers bytes 0-6 only
- [ ] Type field correctly encoded as 12 bits across bytes 1-2
- [ ] Motor field in BYTE2 upper nibble, Type upper 4 bits in BYTE2 lower nibble
- [ ] GetVersion string format handled
- [ ] Use `TMCLReply::fromSpi()` for parsing (includes context for special replies)

### Both Interfaces
- [ ] Strict command/reply order enforced
- [ ] Command context passed to reply parser
- [ ] TMCL status codes checked
- [ ] Checksum validated on standard replies
- [ ] Special reply formats handled

---

## References

- TMC9660 Parameter Mode Reference Manual
- TMC9660 Datasheet (Bootloader section)
- `inc/TMC9660CommInterface.hpp` - Communication interface (contains `CommInterface`, `SpiCommInterface`, `UartCommInterface`)
- `docs/bootloader_guide.md` - Bootloader configuration guide

---
