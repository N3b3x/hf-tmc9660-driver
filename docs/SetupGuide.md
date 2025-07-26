---
layout: default
title: HF-TMC9660 Setup Guide
---

# 📋 HF-TMC9660 Setup Guide

This comprehensive guide walks you through setting up the HF-TMC9660 driver library on your development machine. By the end of this guide, you'll have a working build environment and understand the critical parameter mode configuration requirements.

---

## 🎯 What You'll Accomplish

- ✅ Clone and set up the HF-TMC9660 driver library
- ✅ Install and verify C++20 toolchain compatibility  
- ✅ Build the library and example programs
- ✅ Understand parameter mode initialization requirements
- ✅ Test your setup with working examples

---

## 📋 Prerequisites

Before starting, ensure you have:

- **Git** for cloning the repository
- **C++20 compatible compiler** (GCC 10+, Clang 11+, or MSVC 2019+)
- **Command line access** (Terminal, PowerShell, or Command Prompt)
- **Basic C++ knowledge** for understanding examples

---

## 🚀 Step 1: Clone the Repository

Choose one of these methods based on your project needs:

### Option A: Standalone Clone
```bash
git clone https://github.com/n3b3x/hf-tmc9660-driver.git
cd hf-tmc9660-driver
```

### Option B: Git Submodule (Recommended for existing projects)
```bash
# From your project root
git submodule add https://github.com/n3b3x/hf-tmc9660-driver.git external/hf-tmc9660
cd external/hf-tmc9660
git submodule update --init --recursive
```

### Option C: Download ZIP
If you prefer not to use Git, download the ZIP file from GitHub and extract it to your preferred location.

---

## 🛠️ Step 2: Verify C++20 Toolchain

The HF-TMC9660 driver requires **C++20** support. Verify your compiler meets the requirements:

### Linux/macOS
```bash
# Check GCC version (needs 10+)
g++ --version

# Check Clang version (needs 11+)  
clang++ --version

# Test C++20 support
echo '#include <span>' | g++ -std=c++20 -x c++ -c - 2>/dev/null && echo "✅ C++20 supported" || echo "❌ C++20 not supported"
```

### Windows
```cmd
REM Visual Studio 2019+ with C++20
cl.exe

REM Or MinGW/MSYS2
g++ --version
```

### Installing/Updating Compilers

**Ubuntu/Debian:**
```bash
sudo apt update
sudo apt install build-essential gcc-10 g++-10
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-10 100
```

**Windows (MSYS2):**
```bash
pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-cmake
```

**macOS:**
```bash
# Install Xcode command line tools
xcode-select --install

# Or use Homebrew
brew install gcc
```

---

## 🏗️ Step 3: Build the Library

The library uses a simple compilation model without complex build systems.

### Basic Library Compilation
```bash
# Compile the core library
g++ -std=c++20 -Iinc -c src/TMC9660.cpp -o TMC9660.o

# Compile bootloader support
g++ -std=c++20 -Iinc -c src/TMC9660Bootloader.cpp -o TMC9660Bootloader.o

# Verify object files were created
ls -la *.o
```

### Create Static Library (Optional)
```bash
# Create a reusable static library
ar rcs libTMC9660.a TMC9660.o TMC9660Bootloader.o

# Verify the library
ar -t libTMC9660.a
```

---

## 🧪 Step 4: Build and Test Examples

Test your setup by building the provided examples:

### Bootloader Configuration Example
```bash
g++ -std=c++20 -Iinc \
    src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/bootloader_example.cpp \
    -o bootloader_test

# Run the test
./bootloader_test
```

**Expected Output:**
```
✓ Bootloader configured successfully for parameter mode
  - Boot mode: Parameter
  - Motor control: Enabled
  - Communication: SPI/UART configured
  - Clock: Internal 16MHz with PLL
✓ Parameter mode communication verified
```

### BLDC Motor Control Example
```bash
g++ -std=c++20 -Iinc \
    src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/BLDC_with_HALL.cpp \
    -o bldc_hall_test

# Run the test
./bldc_hall_test
```

### Telemetry Monitoring Example
```bash
g++ -std=c++20 -Iinc \
    src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/Telemetry_monitor.cpp \
    -o telemetry_test

# Run the test
./telemetry_test
```

---

## ⚠️ Step 5: Understanding Parameter Mode Requirements

**CRITICAL:** The TMC9660 must be initialized for Parameter Mode operation before any motor control functions will work.

### Why Parameter Mode?
The TMC9660 can operate in two modes:
- **Register Mode**: Low-level register access (not covered by this driver)
- **Parameter Mode**: High-level TMCL command interface (required for this driver)

### Essential Initialization Pattern
Every application using the HF-TMC9660 driver MUST follow this pattern:

```cpp
#include "TMC9660.hpp"

int main() {
    // 1. Create communication interface (your hardware-specific implementation)
    YourCommInterface bus;  // Replace with your SPI/UART implementation
    TMC9660 driver(bus);
    
    // 2. CRITICAL: Configure bootloader for Parameter Mode
    tmc9660::BootloaderConfig cfg{};
    cfg.boot.boot_mode = tmc9660::bootcfg::BootMode::Parameter;  // ESSENTIAL!
    cfg.boot.start_motor_control = true;
    
    // 3. Initialize bootloader
    auto result = driver.bootloaderInit(&cfg);
    if (result != TMC9660::BootloaderInitResult::Success) {
        // Handle initialization failure
        return -1;
    }
    
    // 4. Now you can use motor control functions
    driver.motorConfig.setType(tmc9660::tmcl::MotorType::BLDC_MOTOR, 7);
    driver.focControl.setTargetVelocity(1000);
    
    return 0;
}
```

---

## 🔧 Step 6: Integration with Your Project

### Method 1: Direct File Integration
Copy the `inc/` and `src/` directories into your project and compile the sources along with your application:

```cmake
# CMake example
add_executable(my_motor_app
    src/main.cpp
    external/hf-tmc9660/src/TMC9660.cpp
    external/hf-tmc9660/src/TMC9660Bootloader.cpp
)

target_include_directories(my_motor_app PRIVATE 
    external/hf-tmc9660/inc
)

target_compile_features(my_motor_app PRIVATE cxx_std_20)
```

### Method 2: Static Library
Use the static library you created in Step 3:

```makefile
# Makefile example
CXXFLAGS = -std=c++20 -Iexternal/hf-tmc9660/inc
LDLIBS = -Lexternal/hf-tmc9660 -lTMC9660

my_motor_app: src/main.cpp
	$(CXX) $(CXXFLAGS) $< $(LDLIBS) -o $@
```

---

## ✅ Verification Checklist

Before proceeding to the next documentation sections, verify:

- [ ] **Repository cloned successfully** and all files present
- [ ] **C++20 compiler working** and version requirements met
- [ ] **Library compiles cleanly** without errors or warnings
- [ ] **Examples build and run** with expected output
- [ ] **Parameter mode initialization** understood and tested
- [ ] **Integration method chosen** for your project

---

## 🚨 Troubleshooting

### Compilation Errors

**Problem:** `error: no matching function for call to 'span'`
**Solution:** Your compiler doesn't support C++20. Update to GCC 10+, Clang 11+, or MSVC 2019+.

**Problem:** `fatal error: TMC9660.hpp: No such file or directory`
**Solution:** Add the include path with `-Iinc` or verify the file structure.

### Runtime Issues

**Problem:** All motor control functions return `false`
**Solution:** Ensure bootloader is configured for Parameter Mode (see Step 5).

**Problem:** Communication timeouts or failures
**Solution:** Verify your communication interface implementation and hardware connections.

---

## 🎯 Next Steps

Congratulations! Your HF-TMC9660 development environment is ready. Continue with:

**👉 [Implementing Communication Interface](ImplementingCommInterface.html)** - Create your hardware-specific communication layer

---

[⬅️ Back to Index](index.html) | [Next ➡️ Communication Interface](ImplementingCommInterface.html)

---

*Need help? Check the troubleshooting sections or review the example implementations.*
