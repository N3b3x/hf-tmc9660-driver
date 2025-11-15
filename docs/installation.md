---
layout: default
title: "🛠️ Installation"
description: "Installation and integration instructions for the TMC9660 driver"
nav_order: 1
parent: "📚 Documentation"
permalink: /docs/installation/
---

# Installation

This guide covers how to obtain and integrate the HF-TMC9660 driver into your project.

## Prerequisites

- **C++20 compatible compiler** (GCC 10+, Clang 11+, or MSVC 2019+)
- **Git** (for cloning)
- **CMake** (optional, for building examples)

## Obtaining the Source

### Option 1: Git Clone

```bash
git clone https://github.com/n3b3x/hf-tmc9660-driver.git
cd hf-tmc9660-driver
```

### Option 2: Git Submodule (Recommended)

```bash
# From your project root
git submodule add https://github.com/n3b3x/hf-tmc9660-driver.git external/hf-tmc9660
cd external/hf-tmc9660
git submodule update --init --recursive
```

### Option 3: Download ZIP

Download the ZIP file from GitHub and extract it to your project.

## Integration Methods

### Manual Integration

1. **Copy the driver files** to your project:
   - `inc/` directory (header files)
   - `src/` directory (source files, if any)

2. **Add include path** to your build system:
   ```cpp
   #include "inc/tmc9660.hpp"
   ```

3. **Enable C++20** in your compiler:
   - GCC/Clang: `-std=c++20`
   - MSVC: `/std:c++20`

### CMake Integration

If your project uses CMake:

```cmake
# Add the driver as a subdirectory
add_subdirectory(external/hf-tmc9660-driver)

# Link to your target
target_link_libraries(your_target PRIVATE hf_tmc9660_driver)

# Enable C++20
set_target_properties(your_target PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)
```

### ESP-IDF Component

For ESP-IDF projects:

1. **Add as component**:
   ```bash
   # In your project's components directory
   git submodule add https://github.com/n3b3x/hf-tmc9660-driver.git hf-tmc9660
   ```

2. **Update CMakeLists.txt**:
   ```cmake
   idf_component_register(
       INCLUDE_DIRS "inc"
       PRIV_INCLUDE_DIRS "src"
       REQUIRES driver
   )
   ```

## Building Examples

### ESP32 Examples

```bash
cd examples/esp32
idf.py build
idf.py flash monitor
```

### Generic CMake Examples

```bash
mkdir build && cd build
cmake ..
make
```

## Verification

After installation, verify your setup:

```cpp
#include "inc/tmc9660.hpp"

int main() {
    // If this compiles, installation is successful
    return 0;
}
```

## Next Steps

- **[Quick Start](quickstart.md)** - Get a minimal example running
- **[Platform Integration](platform_integration.md)** - Implement the communication interface
- **[Hardware Setup](hardware_setup.md)** - Wire your hardware

