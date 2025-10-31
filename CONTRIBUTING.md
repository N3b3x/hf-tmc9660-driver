---
layout: default
title: "🤝 Contributing"
description: "Guidelines and information for contributing to the HardFOC TMC9660 Driver"
nav_order: 5
parent: "🔧 HardFOC TMC9660 Driver"
permalink: /CONTRIBUTING/
---

# 🤝 Contributing to HardFOC TMC9660 Driver

Thank you for your interest in contributing to the HardFOC TMC9660 Driver! This document
provides guidelines and information for contributors.

## 📋 **Code Standards**

### 🎯 **Coding Style and Best Practices for HardFOC Development**

- **C++17 Standard Compliance** - All code must be compatible with C++17
- **Consistent Naming** - Follow the established naming conventions:
  - Classes: `PascalCase` (e.g., `TMC9660`, `TMC9660Bootloader`)
  - Functions: `PascalCase` (e.g., `Initialize`, `SetMotorCurrent`)
  - Variables: `snake_case` with trailing underscore for members (e.g., `motor_enable_`, `current_sensor_`)
  - Constants: `UPPER_SNAKE_CASE` (e.g., `TMC9660_MAX_CURRENT`)
  - Types: Platform-agnostic types where applicable

### 🏗️ **Architecture Guidelines**

- **Hardware Abstraction** - Use hardware-agnostic interfaces where possible
- **Error Handling** - All functions must return appropriate error codes or use `std::expected`
- **Thread Safety** - Consider thread safety implications and document any limitations
- **Platform Agnostic Types** - Use HardFOC type system where applicable

## 🧪 **Testing**

### 🔧 **Unit Tests and Hardware Validation Requirements**

- **Unit Tests** - Write comprehensive unit tests for all new functionality
- **Hardware Testing** - Test on actual TMC9660 hardware with ESP32
- **Integration Tests** - Verify compatibility with existing HardFOC systems
- **Performance Tests** - Ensure real-time performance requirements are met
- **Safety Tests** - Validate safety features and error handling

## 📖 **Documentation**

### 📚 **Documentation Standards and Updates**

- **API Documentation** - Update documentation for all public interfaces
- **User Guides** - Create or update guides for new features
- **Example Code** - Provide working examples for motor control applications
- **Architecture Documentation** - Document design decisions and patterns

## 🐛 **Bug Reports**

### 🔍 **How to Report Bugs Effectively**

When reporting bugs, please include:

1. **Hardware Information**: TMC9660 board, ESP32 version, motor type
2. **Environment Details**: ESP-IDF version, compiler version, operating system
3. **Reproduction Steps**: Minimal code example, configuration settings
4. **Hardware Configuration**: Connected peripherals, pin assignments
5. **Debugging Information**: Error messages, log output, stack traces

## ✨ **Feature Requests**

### 🚀 **Proposing New Features and Enhancements**

When proposing new features:

1. **Use Case** - Describe the specific motor control use case
2. **Technical Specification** - Provide detailed technical requirements
3. **API Design** - Propose the interface design following established patterns
4. **Implementation Plan** - Outline the implementation approach
5. **Testing Strategy** - Describe how the feature will be tested

## 🔄 **Development Workflow**

### 📋 **Step-by-Step Development Process**

1. **Fork the Repository**
2. **Create a Feature Branch**
3. **Implement Your Changes with HardFOC-Specific Tests**
4. **Document Your Changes with HardFOC Examples**
5. **Submit a Pull Request**

## 📋 **Code Quality Standards for HardFOC**

- **C++17 Compliance** - Code compiles without warnings
- **HardFOC Compatibility** - Tested on HardFOC boards
- **Error Handling** - All error conditions handled appropriately
- **Documentation** - All public APIs documented
- **Tests** - Adequate test coverage provided
- **Performance** - Real-time requirements met

---

## 🚀 Thank You for Contributing to HardFOC

Your contributions help make HardFOC motor controller boards more accessible and powerful
for everyone.
