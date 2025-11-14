---
layout: default
title: "📖 GetVersion Command Reference"
description: "Reference documentation for the GetVersion command and version information"
nav_order: 11
parent: "📚 Documentation"
permalink: /docs/GetVersionCommandReference/
---

# GetVersion Command Reference

## Overview

The `GetVersion` command (Opcode 136, 0x88) retrieves firmware version information from
the TMC9660 chip in Parameter Mode.

## Command Format

```yaml
Opcode: 136 (0x88)
Type: 0 (returns string format)
Motor/Bank: 0 (not used)
Value: 0 (not used)
```

## Type Parameter Values

### Type = 0: Firmware Version String (Recommended)

**Request:**
```text
GetVersion (Op=0x88), Type=0x0000, Motor=0x00, Value=0x00000000
```

**Response Format:**
- **SPI Mode**: Returns 8 bytes containing a version string in ASCII format
  - Byte 0: SPI Status (typically 0xFF)
  - Bytes 1-8: Version string (up to 8 characters, null-padded)
  - Format: `[SPI_STATUS][VERSION_STRING_8_CHARS]`

- **UART Mode**: Returns 9 bytes containing a version string
  - Byte 0: Host Address
  - Byte 1: Sync bit + Module Address
  - Bytes 2-8: Version string (up to 7 characters)
  - Format: `[HOST_ADDR][SYNC+ADDR][VERSION_STRING_7_CHARS]`

**Example Response:**
```yaml
Raw bytes (UART): FF 01 64 88 30 30 35 31 56 31 30 30 [CRC]
Decoded: "051V100"
```

**Version String Format:**
The firmware version string follows the pattern:
```yaml
[PREFIX][VERSION_NUMBER]V[BUILD_NUMBER]
```

Example: `051V100`
- `051`: Prefix or version identifier
- `V`: Version indicator
- `100`: Build number

**Implementation Notes:**
- The version string is ASCII-encoded
- For SPI: 8-byte version string (typically from bytes 1-8 after status)
- For UART: 7-byte version string (bytes 2-8 after address)
- Checksum validation is typically skipped for Type=0 responses due to the string format
- The driver's `getVersionString()` method extracts printable ASCII characters (0x20-0x7E)

### Other Type Values

**Current Implementation:**
Based on codebase analysis and testing:
- **Type = 0**: Returns firmware version as ASCII string format (confirmed working)
- **Type ≠ 0**: Not currently implemented/tested in the driver; behavior unknown

**TMCL Protocol Notes:**
- The TMC9660 Parameter Mode Reference Manual (Table 18) lists GetVersion with Type as "-",
  indicating Type may not be standardized in the base TMCL specification
- However, the TMC9660 firmware clearly supports `Type=0` for string format returns
- Other Type values may exist but are not documented in available sources

**Alternative: GetInfo Command**
If you need other version information formats, consider using the `GetInfo` command (Opcode 157):
- **Type = 0**: Get Module ID (returns numeric ID)
- **Type = 1**: Get Version (returns numeric version, format depends on firmware)

The `GetInfo` command provides a more standardized way to query multiple information types.

## Implementation in Driver

The driver handles GetVersion with Type=0 specially:

```cpp
// In src/TMC9660.cpp
if (opcode == tmc9660::tmcl::Op::GetVersion && type == 0) {
    std::string versionString = rep.getVersionString();
    if (!versionString.empty()) {
        // Success - version string extracted
        return true;
    }
}
```

The `TMCLReply::getVersionString()` method:
- Checks if opcode is 136 (GetVersion)
- Extracts ASCII characters from raw reply bytes
- Filters printable characters (0x20-0x7E)
- Returns version string

## Usage Example

```cpp
// Get firmware version string
uint32_t version = 0;
bool success = driver.sendCommand(
    tmc9660::tmcl::Op::GetVersion,
    0,  // Type = 0 for string format
    0,  // Motor (not used)
    0,  // Value (not used)
    &version  // Reply will be 0 for string format
);

// Version string is logged in sendCommand()
// Output: "✅ GetVersion successful - Firmware version: 051V100"
```

## Sources

- TMC9660 Parameter Mode Reference Manual (Table 18 - TMCL Commands)
- Driver source code: `src/TMC9660.cpp`, `inc/TMC9660CommInterface.hpp` (contains `CommInterface`, `SpiCommInterface`, `UartCommInterface`)
- TMCL Protocol Standard (Trinamic Motion Control Language)
- Analog Devices TMC9660 documentation

## References

- **TMCL Protocol Reference**: See [tmcl_protocol_guide.md](./tmcl_protocol_guide.md)
- **Parameter Mode Manual**: See TMC9660 Parameter Mode Reference Manual PDF in `Datasheet/`
  directory
