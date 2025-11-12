/**
 * @file bootloader_utils.hpp
 * @brief TMC9660 bootloader utility functions and helpers.
 *
 * This file contains utility functions and helper routines used by the
 * TMC9660 bootloader protocol implementation. These include CRC calculation
 * functions, bit manipulation utilities, and other support functions.
 *
 * @defgroup TMC9660_BootloaderUtils Utility Functions
 * @brief Helper functions for bootloader operations
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace tmc9660 {

/**
 * @brief Helper function for CRC-8 calculation (UART only).
 *
 * These functions are used by the UART bootloader protocol for checksum
 * calculation. They are automatically called by the ::BootloaderCommandUART
 * serialization methods.
 */

/**
 * @brief Bit-reverse a byte (LSB ↔ MSB).
 * @ingroup TMC9660_BootloaderUtils
 *
 * Helper function used by the bootloader CRC-8 calculation to reverse
 * the bit order of each input byte before polynomial division.
 *
 * @param b Input byte to reverse
 * @return Bit-reversed byte
 */
static constexpr uint8_t reverseByte(uint8_t b) noexcept {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

/**
 * @brief CRC-8 calculation for UART protocol (TMC9660 datasheet method).
 * @ingroup TMC9660_BootloaderUtils
 *
 * Calculates CRC-8 checksum using the TMC9660-specific algorithm. This is NOT
 * a standard CRC-8 implementation - each input byte is bit-reversed before
 * processing through the polynomial division.
 *
 * Polynomial: x^8 + x^2 + x^1 + x^0 (9-bit: 0b100000111)
 * Algorithm: Bit-reverse each input byte, perform polynomial division, return MSB-first result
 *
 * @param data Pointer to data bytes to calculate CRC over
 * @param len Number of data bytes
 * @return Calculated CRC-8 value
 * @note This is NOT a standard CRC-8! Each byte is bit-reversed before processing.
 */
static constexpr uint8_t crc8Bootloader(const uint8_t* data, size_t len) noexcept {
  uint16_t crc = 0;            // 9-bit register for polynomial division
  const uint16_t POLY = 0x107; // x^8 + x^2 + x^1 + x^0 in 9-bit form

  // Process each byte (bit-reversed, LSB-first)
  for (size_t i = 0; i < len; i++) {
    uint8_t byte_reversed = reverseByte(data[i]);

    // Feed 8 bits into CRC register (MSB-first from reversed byte)
    for (int bit = 7; bit >= 0; bit--) {
      crc = (crc << 1) | ((byte_reversed >> bit) & 1);
      if (crc & 0x100) { // If bit 8 is set
        crc ^= POLY;
      }
    }
  }

  // Append 8 zero bits (flush remaining data through CRC)
  for (int i = 0; i < 8; i++) {
    crc <<= 1;
    if (crc & 0x100) {
      crc ^= POLY;
    }
  }

  return static_cast<uint8_t>(crc & 0xFF);
}

} // namespace tmc9660