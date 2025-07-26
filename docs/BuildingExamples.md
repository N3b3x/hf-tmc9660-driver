---
layout: default
title: Building and Running the Example Programs
---

# 🏗️ Building and Running the Example Programs

The HF-TMC9660 driver includes comprehensive examples that demonstrate every aspect of motor control from basic setup to advanced FOC algorithms. This guide shows you how to build, run, and understand these examples.

---

## 🎯 What You'll Learn

- ✅ Build all example programs from command line
- ✅ Understand what each example demonstrates
- ✅ Run examples with proper output interpretation
- ✅ Use examples as templates for your projects
- ✅ Troubleshoot common build and runtime issues

---

## 📋 Prerequisites

Before building examples, ensure you have:

- **✅ HF-TMC9660 library** set up ([Setup Guide](SetupGuide.html))
- **✅ C++20 compiler** working (GCC 10+, Clang 11+, MSVC 2019+)
- **✅ Communication interface** understanding ([Comm Interface Guide](ImplementingCommInterface.html))

---

## 🚀 Quick Start - Build All Examples

### Method 1: Individual Compilation

```bash
# Navigate to project root
cd hf-tmc9660-driver

# Build bootloader configuration example
g++ -std=c++20 -Iinc \
    src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/bootloader_example.cpp \
    -o bootloader_demo

# Build BLDC with Hall sensors example
g++ -std=c++20 -Iinc \
    src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/BLDC_with_HALL.cpp \
    -o bldc_hall_demo

# Build telemetry monitoring example
g++ -std=c++20 -Iinc \
    src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/Telemetry_monitor.cpp \
    -o telemetry_demo
```

### Method 2: Batch Build Script

Create a build script for convenience:

```bash
#!/bin/bash
# build_examples.sh

COMPILER="g++"
STD="-std=c++20"
INCLUDES="-Iinc"
SOURCES="src/TMC9660.cpp src/TMC9660Bootloader.cpp"

echo "🏗️  Building HF-TMC9660 Examples..."

# Array of examples
declare -a examples=(
    "bootloader_example:bootloader_demo"
    "BLDC_with_HALL:bldc_hall_demo"
    "BLDC_with_ABN:bldc_abn_demo"
    "BLDC_velocity_control:bldc_vel_demo"
    "DC_current_control:dc_current_demo"
    "Stepper_FOC:stepper_foc_demo"
    "Stepper_step_dir:stepper_stepdir_demo"
    "Telemetry_monitor:telemetry_demo"
)

for example in "${examples[@]}"; do
    IFS=':' read -r source output <<< "$example"
    echo "Building $source..."
    
    $COMPILER $STD $INCLUDES $SOURCES "examples/$source.cpp" -o "$output"
    
    if [ $? -eq 0 ]; then
        echo "✅ $output built successfully"
    else
        echo "❌ Failed to build $source"
        exit 1
    fi
done

echo "🎉 All examples built successfully!"
```

Make it executable and run:
```bash
chmod +x build_examples.sh
./build_examples.sh
```

---

## 📚 Example Programs Reference

### 🔧 Foundation Examples

#### 1. **Bootloader Configuration** (`bootloader_example.cpp`)
**Purpose:** Demonstrates essential parameter mode setup
```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/bootloader_example.cpp -o bootloader_demo
./bootloader_demo
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

**Key Learning:** This example shows the **critical** bootloader setup required for all motor control operations.

#### 2. **Telemetry Monitoring** (`Telemetry_monitor.cpp`)
**Purpose:** Real-time monitoring of chip status
```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/Telemetry_monitor.cpp -o telemetry_demo
./telemetry_demo
```

**Expected Output:**
```
✓ Parameter mode initialized - starting telemetry monitoring
✓ Basic motor configuration applied

Telemetry monitoring (Ctrl+C to stop):
----------------------------------------
Sample  1: Temp= 25.0°C, Current=    0mA, Voltage=24.00V
Sample  2: Temp= 26.1°C, Current=  150mA, Voltage=23.98V
...
```

---

### ⚡ BLDC Motor Control Examples

#### 3. **BLDC with Hall Sensors** (`BLDC_with_HALL.cpp`)
**Purpose:** Complete BLDC setup with Hall sensor feedback
```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/BLDC_with_HALL.cpp -o bldc_hall_demo
./bldc_hall_demo
```

**Expected Output:**
```
✓ Bootloader configured for parameter mode
✓ Motor type: BLDC with 7 pole pairs
✓ Current limits: 2A torque, 1A flux
✓ Hall sensors configured
✓ FOC gains configured (Current: P=50, I=100 | Velocity: P=800, I=1)
✓ FOC commutation with Hall sensors enabled
✓ Motor started with target velocity 1000
```

**Key Learning:** Demonstrates complete BLDC setup sequence with safety checks.

#### 4. **BLDC with ABN Encoder** (`BLDC_with_ABN.cpp`)
**Purpose:** High-precision BLDC control with incremental encoder
```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/BLDC_with_ABN.cpp -o bldc_abn_demo
./bldc_abn_demo
```

**Key Learning:** Shows encoder-based feedback for precision applications.

#### 5. **BLDC Velocity Control** (`BLDC_velocity_control.cpp`)
**Purpose:** DC motor with velocity feedback control
```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/BLDC_velocity_control.cpp -o bldc_vel_demo
./bldc_vel_demo
```

---

### 🔄 Stepper Motor Examples

#### 6. **Stepper FOC Control** (`Stepper_FOC.cpp`)
**Purpose:** Field-oriented control of stepper motors
```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/Stepper_FOC.cpp -o stepper_foc_demo
./stepper_foc_demo
```

**Key Learning:** Advanced stepper control with smooth operation and high precision.

#### 7. **Stepper Step/Dir Interface** (`Stepper_step_dir.cpp`)
**Purpose:** Traditional step/direction interface with extrapolation
```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/Stepper_step_dir.cpp -o stepper_stepdir_demo
./stepper_stepdir_demo
```

---

### 🔌 DC Motor Examples

#### 8. **DC Current Control** (`DC_current_control.cpp`)
**Purpose:** Open-loop current drive for DC motors
```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/DC_current_control.cpp -o dc_current_demo
./dc_current_demo
```

---

## 🔧 Build Configurations

### Debug Build (Recommended for Development)
```bash
g++ -std=c++20 -Iinc -g -O0 -DDEBUG \
    src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/BLDC_with_HALL.cpp \
    -o bldc_hall_debug
```

### Release Build (Optimized for Production)
```bash
g++ -std=c++20 -Iinc -O3 -DNDEBUG \
    src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/BLDC_with_HALL.cpp \
    -o bldc_hall_release
```

### Static Library Build
```bash
# First create the static library
g++ -std=c++20 -Iinc -c src/TMC9660.cpp -o TMC9660.o
g++ -std=c++20 -Iinc -c src/TMC9660Bootloader.cpp -o TMC9660Bootloader.o
ar rcs libTMC9660.a TMC9660.o TMC9660Bootloader.o

# Then link examples against it
g++ -std=c++20 -Iinc examples/BLDC_with_HALL.cpp -L. -lTMC9660 -o bldc_hall_demo
```

---

## 🛠️ Platform-Specific Build Instructions

### Linux/macOS
```bash
# Standard build
g++ -std=c++20 -Iinc src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/bootloader_example.cpp -o bootloader_demo

# With threading support (if using std::thread)
g++ -std=c++20 -Iinc -pthread src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/bootloader_example.cpp -o bootloader_demo
```

### Windows (MinGW/MSYS2)
```bash
# Standard build
g++ -std=c++20 -Iinc src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/bootloader_example.cpp -o bootloader_demo.exe

# Static linking (avoid DLL dependencies)
g++ -std=c++20 -Iinc -static src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/bootloader_example.cpp -o bootloader_demo.exe
```

### Windows (Visual Studio)
```cmd
REM Use Developer Command Prompt
cl /std:c++20 /Iinc /EHsc src/TMC9660.cpp src/TMC9660Bootloader.cpp ^
   examples/bootloader_example.cpp /Fe:bootloader_demo.exe
```

---

## 🧪 Testing Your Builds

### Verification Checklist

Run this verification for each built example:

```bash
# 1. Verify the executable was created
ls -la *_demo

# 2. Check it's executable
file bootloader_demo

# 3. Run basic test
./bootloader_demo

# 4. Verify expected output appears
echo $?  # Should return 0 for success
```

### Expected Behavior

**With DummyBus (Default):**
- Examples should run without errors
- Configuration messages should appear
- Simulated telemetry values should be displayed
- Return code should be 0

**With Real Hardware:**
- Motors should respond according to example logic
- Telemetry should show real sensor values
- Communication should be stable

---

## 🚨 Troubleshooting

### Compilation Issues

**Problem:** `error: no matching function for call to 'span'`
```bash
# Solution: Verify C++20 support
g++ --version  # Should be 10+ for GCC, 11+ for Clang
echo '#include <span>' | g++ -std=c++20 -x c++ -c -
```

**Problem:** `fatal error: TMC9660.hpp: No such file or directory`
```bash
# Solution: Verify include path and file structure
ls -la inc/TMC9660.hpp  # Should exist
# Add -Iinc flag to compilation command
```

**Problem:** `undefined reference to TMC9660Bootloader`
```bash
# Solution: Include bootloader source
# Add src/TMC9660Bootloader.cpp to compilation command
```

### Runtime Issues

**Problem:** Examples compile but return error codes
```bash
# Check the actual error message
./bootloader_demo
echo "Exit code: $?"

# Common issues:
# - Parameter mode not configured (most common)
# - Communication interface not working
# - Hardware not connected
```

**Problem:** "Communication failure" messages
```bash
# Verify your communication interface implementation
# Check hardware connections
# Ensure TMC9660 is powered and in parameter mode
```

### Build System Integration

#### CMake Example
```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(TMC9660Examples)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directory
include_directories(external/hf-tmc9660/inc)

# Library sources
set(TMC9660_SOURCES
    external/hf-tmc9660/src/TMC9660.cpp
    external/hf-tmc9660/src/TMC9660Bootloader.cpp
)

# Create library
add_library(TMC9660 STATIC ${TMC9660_SOURCES})

# Example executables
add_executable(bootloader_demo external/hf-tmc9660/examples/bootloader_example.cpp)
target_link_libraries(bootloader_demo TMC9660)

add_executable(bldc_hall_demo external/hf-tmc9660/examples/BLDC_with_HALL.cpp)
target_link_libraries(bldc_hall_demo TMC9660)
```

#### Makefile Example
```makefile
# Makefile
CXX = g++
CXXFLAGS = -std=c++20 -Iinc -O2
SOURCES = src/TMC9660.cpp src/TMC9660Bootloader.cpp

EXAMPLES = bootloader_demo bldc_hall_demo telemetry_demo

all: $(EXAMPLES)

bootloader_demo: examples/bootloader_example.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

bldc_hall_demo: examples/BLDC_with_HALL.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

telemetry_demo: examples/Telemetry_monitor.cpp $(SOURCES)
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -f $(EXAMPLES) *.o

.PHONY: all clean
```

---

## 📋 Example Usage Checklist

Before using examples as templates:

- [ ] **Built successfully** without warnings
- [ ] **Runs without errors** (exit code 0)
- [ ] **Output messages appear** as expected
- [ ] **Parameter mode setup** understood and verified
- [ ] **Communication interface** adapted for your hardware
- [ ] **Motor parameters** adjusted for your motor
- [ ] **Safety limits** configured appropriately

---

## 🎯 Next Steps

With working examples, you're ready for advanced topics:

**👉 [Hardware-Agnostic Examples](HardwareAgnosticExamples.html)** - Detailed motor control scenarios

**👉 [Common Operations](CommonOperations.html)** - Everyday driver usage patterns

---

[⬅️ Communication Interface](ImplementingCommInterface.html) | [⬆️ Back to Index](index.html) | [Next ➡️ Hardware Examples](HardwareAgnosticExamples.html)

---

*Having build issues? Check the troubleshooting section or review your C++20 compiler setup.*
