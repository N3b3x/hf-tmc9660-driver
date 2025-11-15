---
layout: default
title: "📡 Communication Protocols"
description: "Detailed protocol specifications for TMC9660"
nav_order: 9
parent: "📚 Documentation"
permalink: /docs/special_feature_protocols/
---

# Communication Protocols

This guide covers the detailed communication protocols for the TMC9660, including bootloader and parameter mode (TMCL) protocols for both SPI and UART interfaces.

## Protocol Overview

The TMC9660 supports two communication interfaces:
- **SPI**: Full-duplex, high-speed, delayed replies
- **UART**: Half-duplex, command/reply order, autobaud support

Each interface has two protocols:
- **Bootloader Protocol**: 5-byte frames (40 bits) for SPI, 8-byte for UART
- **Parameter Mode Protocol (TMCL)**: 8-byte frames (64 bits) for SPI, 9-byte for UART

## Bootloader Protocol

### SPI Bootloader Protocol (40-bit / 5-byte)

```
Transaction 1: SEND COMMAND
┌──────────┬──────────────────────────────────────────┐
│ TX (5B)  │ [CMD] [VALUE(32)]                        │
│ RX (5B)  │ [PREV_STATUS] [PREV_VALUE(32)]           │
└──────────┴──────────────────────────────────────────┘
                    ↓
Transaction 2: RECEIVE REPLY
┌──────────┬──────────────────────────────────────────┐
│ TX (5B)  │ [NO_OP] [0x00] [0x00] [0x00] [0x00]      │
│ RX (5B)  │ [STATUS] [VALUE(32)]                     │
└──────────┴──────────────────────────────────────────┘
```

**Key Features:**
- Frame Size: 5 bytes (40 bits)
- No Bit-Flipping: Data sent and received as-is
- Reply Delay: Reply to command N comes during transaction N+1
- Dummy Frame: Send all zeros to clock out reply

### UART Bootloader Protocol (64-bit / 8-byte)

```
COMMAND FRAME (8 bytes):
┌────┬────┬────┬────────────────────────┬────┐
│0x55│DEVA│CMD │    VALUE (32-bit)      │CRC8│
│    │DDR │    │   [MSB ... ... LSB]    │    │
└────┴────┴────┴────────────────────────┴────┘
 Sync  Dev  Cmd    Byte3 Byte2 Byte1 Byte0  Checksum

REPLY FRAME (8 bytes):
┌────┬────┬────┬────────────────────────┬────┐
│0x55│HOST│STAT│    VALUE (32-bit)      │CRC8│
│    │ADDR│US  │   [MSB ... ... LSB]    │    │
└────┴────┴────┴────────────────────────┴────┘
 Sync  Host Stat   Byte3 Byte2 Byte1 Byte0  Checksum
```

**Key Features:**
- Frame Size: 8 bytes (64 bits)
- No Bit-Flipping: Bytes transmitted as-is
- Immediate Reply: Reply comes immediately (no dummy frame needed)
- CRC-8 Protection: Polynomial x⁸+x²+x¹+x⁰ over first 7 bytes
- Addressing: Device address (default 1) and Host address (default 255)

## Parameter Mode (TMCL) Protocol

### SPI TMCL Protocol (64-bit / 8-byte)

```
Command Format:
┌──────┬──────────┬──────────┬──────────────┬──────────┐
│ Byte │ 0        │ 1-2      │ 3            │ 4-6      │ 7        │
├──────┼──────────┼──────────┼──────────────┼──────────┼──────────┤
│ Bits │ 0-7      │ 8-19     │ 20-23        │ 24-55     │ 56-63    │
│ Desc │ Operation│ Type     │ Motor/Bank   │ Data      │ Checksum │
└──────┴──────────┴──────────┴──────────────┴──────────┴──────────┘

Reply Format:
┌──────┬──────────┬──────────┬──────────┬──────────────┬──────────┐
│ Byte │ 0        │ 1        │ 2        │ 3-6          │ 7        │
├──────┼──────────┼──────────┼──────────┼──────────────┼──────────┤
│ Bits │ 0-7      │ 8-15     │ 16-23    │ 24-55        │ 56-63    │
│ Desc │ SPI Status│ TMCL Status│ Operation│ Data      │ Checksum │
└──────┴──────────┴──────────┴──────────┴──────────────┴──────────┘
```

**Key Features:**
- Frame Size: 8 bytes (64 bits)
- Reply Delay: Reply to command N is received when sending command N+1
- Checksum: 8-bit checksum over first 7 bytes
- Status: SPI status byte (0xFF = OK, 0x64 = error, 0xF0 = busy)

### UART TMCL Protocol (72-bit / 9-byte)

```
Command Format:
┌──────┬──────────────┬──────────┬──────────┬──────────────┬──────────┐
│ Byte │ 0            │ 1        │ 2-3      │ 4            │ 5-8      │ 9        │
├──────┼──────────────┼──────────┼──────────┼──────────────┼──────────┼──────────┤
│ Desc │ Sync+Address │ Command  │ Type     │ Motor/Bank   │ Data     │ Checksum │
└──────┴──────────────┴──────────┴──────────┴──────────────┴──────────┴──────────┘

Reply Format:
┌──────┬──────────────┬──────────────┬──────────┬──────────┬──────────────┬──────────┐
│ Byte │ 0            │ 1            │ 2        │ 3        │ 4-7          │ 8        │
├──────┼──────────────┼──────────────┼──────────┼──────────┼──────────────┼──────────┤
│ Desc │ Host Address │ Sync+Address │ TMCL Status│ Operation│ Data         │ Checksum │
└──────┴──────────────┴──────────────┴──────────┴──────────┴──────────────┴──────────┘
```

**Key Features:**
- Frame Size: 9 bytes (72 bits)
- Immediate Reply: Reply comes immediately after command
- Checksum: 8-bit checksum over first 8 bytes
- Addressing: Module address in sync byte

## CRC & Checksum Calculations

### Bootloader CRC-8 (UART only)

Polynomial: x⁸ + x² + x¹ + x⁰ (0x07)

```cpp
uint8_t calculateCRC8(const uint8_t* data, size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
```

### TMCL Checksum

Simple 8-bit sum over first 7 bytes (SPI) or 8 bytes (UART):

```cpp
uint8_t calculateChecksum(const uint8_t* data, size_t length) {
    uint8_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}
```

## Common Pitfalls

### SPI Delayed Replies

**Issue**: Reply to command N comes during transaction N+1

**Solution**: Always send a dummy frame (NO_OP) to receive the previous reply:

```cpp
// Send command
spiTransfer(command_frame, dummy_rx);

// Get reply by sending dummy frame
uint8_t dummy[8] = {0};
spiTransfer(dummy, reply_frame);
```

### UART Half-Duplex Timing

**Issue**: Must wait for complete reply before sending next command

**Solution**: Wait for all expected bytes before sending next command:

```cpp
// Send command
uartWrite(command_frame, 9);

// Wait for complete reply
uint8_t reply[9];
size_t received = 0;
while (received < 9) {
    if (uartAvailable()) {
        reply[received++] = uartRead();
    }
}
```

### Checksum Errors

**Issue**: Checksum calculation mismatch

**Solution**: 
- Verify checksum algorithm matches specification
- Check byte order (big-endian vs little-endian)
- Ensure all bytes are included in calculation

## Next Steps

- **[Platform Integration](platform_integration.md)** - Implement communication interface
- **[Bootloader Initialization](special_feature_bootloader.md)** - Bootloader setup
- **[Troubleshooting](troubleshooting.md)** - Common issues and solutions

