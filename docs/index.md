---
layout: default
title: HF-TMC9660 Driver Documentation
---

## 🚀 HF-TMC9660 Driver Documentation

Welcome to the comprehensive documentation for the **HF-TMC9660** driver library
– a modern C++20 hardware-agnostic driver for the TMC9660 motor controller
operating in **Parameter Mode**.

## 📖 What You'll Learn

This documentation will guide you through everything from initial setup to
advanced motor control applications. Whether you're new to TMC9660 or an
experienced embedded developer, you'll find practical examples and detailed
explanations for every aspect of the driver.

---

## 🎯 Quick Start Path

**New to TMC9660?** Follow this recommended learning path:

```mermaid
graph LR
    A[📋 Setup Guide] --> B[🔌 Communication Interface]
    B --> C[🏗️ Building Examples]
    C --> D[⚡ Hardware Examples]
    D --> E[🛠️ Common Operations]
    E --> F[🚀 Advanced Usage]
```

---

## 📚 Documentation Structure

### **Foundation & Setup**

1. **[📋 Setup Guide](SetupGuide.html)** - Get started with installation and
   compilation
2. **[🔌 Implementing Communication Interface](ImplementingCommInterface.html)**
   - Create your hardware-specific communication layer
3. **[🏗️ Building Examples](BuildingExamples.html)** - Compile and run the provided
   examples

### **Practical Application**

1. **[⚡ Hardware-Agnostic Examples](HardwareAgnosticExamples.html)** - Complete
   motor control scenarios
2. **[🛠️ Common Operations](CommonOperations.html)** - Everyday driver usage
   patterns
3. **[📖 API Reference](annotated.html)** - Complete C++ class documentation

### **Advanced Topics**

1. **[🌐 GitHub Pages Hosting](HostingDocsWithGitHubPages.html)** - Host your own
   documentation

---

## ⚠️ Critical Requirements

> **PARAMETER MODE SETUP IS MANDATORY**
>
> The TMC9660 must be properly configured for **Parameter Mode** operation via
> bootloader initialization before any motor control functions will work. This is
> the #1 source of issues for new users.

### Essential Setup Sequence

```cpp
// 1. CRITICAL: Configure for Parameter Mode
tmc9660::BootloaderConfig cfg{};
cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;  // ESSENTIAL!
cfg.boot.start_motor_control = true;

// 2. Initialize bootloader
auto result = driver.bootloaderInit(&cfg);

// 3. Configure motor and start control
driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
driver.focControl.setTargetVelocity(1000);
```

---

## 🌟 Key Features

| Feature | Description |
|---------|-------------|
| **🔧 Hardware Agnostic** | Abstract communication interface for any SPI/UART implementation |
| **⚡ FOC Control** | Advanced Field-Oriented Control for BLDC, stepper, and DC motors |
| **📊 Real-time Telemetry** | Temperature, current, voltage monitoring with logging |
| **🛡️ Protection Systems** | Comprehensive safety features and fault monitoring |
| **🎛️ Complete Configuration** | Access to all 300+ TMC9660 parameters |
| **📱 Modern C++** | Clean C++20 API with type safety and RAII principles |

---

## 🚦 Getting Started

Ready to begin? Start with the **Setup Guide** to prepare your development environment:

### **First Time Users**

👉 **[Start Here: Setup Guide](SetupGuide.html)**

### **Experienced Developers**

👉 **[Jump to Examples](HardwareAgnosticExamples.html)**

### **API Reference**

👉 **[Browse API Documentation](annotated.html)**

---

## 💡 Need Help?

- **🐛 Found a bug?** Check the troubleshooting sections in each guide
- **❓ Have questions?** Review the Common Operations guide
- **📝 Want to contribute?** See the implementation guides

---

<div style="text-align: center; margin: 2em 0; padding: 1em; background: #f8f9fa; border-radius: 8px;">
  <strong>🎯 Ready to control some motors?</strong><br>
  <a href="SetupGuide.html" style="display: inline-block; margin-top: 0.5em; padding: 0.5em 1em; background: #007bff; color: white; text-decoration: none; border-radius: 4px;">Get Started →</a>
</div>

---

## Last updated: 2024 | HF-TMC9660 Driver v1.0
