# 🚀 HardFOC CI/CD Pipeline Documentation

<div align="center">

![CI/CD](https://img.shields.io/badge/CI%2FCD-Advanced%20Workflows-blue?style=for-the-badge&logo=github-actions)
![ESP32](https://img.shields.io/badge/ESP32-C6%20Support-green?style=for-the-badge&logo=espressif)
![Security](https://img.shields.io/badge/Security-First%20Approach-orange?style=for-the-badge&logo=security)

**📋 Multi-MCU Interface Wrapper CI/CD Pipeline**

*Comprehensive testing and release automation for HardFOC's multi-MCU interface wrapper library*

</div>

---

## 📋 Table of Contents

- [🎯 Overview](#-overview)
- [🏗️ Workflow Architecture](#-workflow-architecture)
- [🚀 Available Workflows](#-available-workflows)
- [🔧 Configuration](#-configuration)
- [📊 Workflow Selection Guide](#-workflow-selection-guide)
- [🛡️ Security Features](#-security-features)
- [⚡ Performance Optimization](#-performance-optimization)
- [🔍 Troubleshooting](#-troubleshooting)

---

## 🎯 Overview

The HardFOC CI/CD pipeline provides comprehensive automated testing, building, and
deployment capabilities for our multi-MCU interface wrapper library. The pipeline is
designed to test examples across multiple MCU platforms and create unified releases of
the complete library.

### **Key Features**
- 🔄 **Multi-MCU Support** - Test examples across different MCU platforms (ESP32, future MCUs)
- 🛡️ **Security-First** - Comprehensive security scanning and vulnerability detection
- 📊 **Smart Caching** - Optimized for fast builds and minimal resource usage
- 🎯 **Library-Focused** - Core library (`inc/` and `src/`) with MCU-specific examples
- 🔧 **MCU-Specific Tools** - Each MCU has its own project tools and CI pipelines

---

## 🏗️ Workflow Architecture

### **Multi-MCU Architecture**

The library provides a unified interface across multiple MCU platforms:

```text
hf-internal-interface-wrap/
├── inc/                          # Core library headers (MCU-agnostic and )
├── src/                          # Core library implementation (MCU-agnostic)
├── examples/                     # MCU-specific example implementations
│   ├── esp32/                    # ESP32 examples (current)
│   │   ├── main/                 # Example applications
│   │   ├── scripts/              # ESP32-specific tools
│   │   └── app_config.yml        # ESP32 configuration
│   ├── stm32/                    # STM32 examples (future)
│   │   ├── main/
│   │   ├── scripts/              # STM32-specific tools
│   │   └── app_config.yml
│   └── ...                       # Additional MCU support
└── docs/                         # Unified documentation
```

### **MCU-Specific Tools Integration**

Each MCU has its own project tools and CI pipelines:

```text
examples/esp32/scripts/  # Git submodule: hf-espidf-project-tools
├── build_app.sh         # ESP32 build automation
├── generate_matrix.py   # ESP32 matrix generation
├── setup_ci.sh          # ESP32 CI environment setup
└── ...

examples/stm32/scripts/  # Future: hf-stm32-project-tools
├── build_app.sh         # STM32 build automation
├── generate_matrix.py   # STM32 matrix generation
└── ...
```

### **Multi-MCU Workflow Dependencies**

```text
┌─────────────────────────────────────────────────────────────────────────────────┐
│                        🚀 Multi-MCU CI Workflows Layer                          │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │ ESP32       │  │ STM32       │  │ Future      │  │ Release     │             │
│  │ Examples    │  │ Examples    │  │ MCU         │  │ CI          │             │
│  │ Build CI    │  │ Build CI    │  │ Examples    │  │             │             │
│  │             │  │             │  │ Build CI    │  │ • Waits for │             │
│  │ • Matrix    │  │ • Matrix    │  │             │  │   ALL MCU   │             │
│  │   Builds    │  │   Builds    │  │ • Matrix    │  │   Examples  │             │
│  │ • Linting   │  │ • Linting   │  │   Builds    │  │   Build CI  │             │
│  │ • Security  │  │ • Security  │  │ • Linting   │  │ • Packages  │             │
│  │ • Testing   │  │ • Testing   │  │ • Security  │  │   Complete  │             │
│  │             │  │             │  │ • Testing   │  │   Library   │             │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘             │
│         │                 │                 │                 │                 │
│         └─────────────────┼─────────────────┼─────────────────┘                 │
│                           │                 │                                   │
│  ┌─────────────────────────────────────────────────────────────────────────────┐│
│  │                    📦 Unified Release Artifacts                             ││
│  │                                                                             ││
│  │  • Complete Library Package (inc/ + src/ + all MCU examples)                ││
│  │  • MCU-Specific Firmware Packages (ESP32, STM32, etc.)                      ││
│  │  │  • Unified Documentation (all MCU APIs)                                  ││
│  │  • GitHub Release with comprehensive multi-MCU documentation                ││
│  └─────────────────────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## 🚀 Available Workflows

### **1. 🚀 ESP32 Examples Build CI** ([`esp32-examples-build-ci.yml`](esp32-examples-build-ci.yml))

**Purpose**: Test and validate ESP32-specific examples of the interface wrapper library

**Triggers**:
- Push to `main`, `develop`, `release/*`, `feature/*`, `bugfix/*` branches
- Pull requests to `main`, `develop`
- Manual trigger with options

**Features**:
- ✅ **Matrix Generation**: Dynamic build matrix from ESP32 `app_config.yml`
- ✅ **Parallel Builds**: All ESP32 examples built simultaneously
- ✅ **Code Linting**: Automated formatting and style checking
- ✅ **Security Scanning**: Comprehensive vulnerability detection
- ✅ **Static Analysis**: Code quality and complexity analysis
- ✅ **Link Checking**: Documentation link validation

**Note**: This is the first MCU implementation. Future MCUs will have similar workflows (e.g., `stm32-examples-build-ci.yml`)

**Workflow Process**:

```text
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Validate      │    │   Build         │    │   Lint          │
│   Configuration │───▶│   MCU           │───▶│   Code          │
│                 │    │   Examples      │    │   Formatting    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Security      │    │   Static        │    │   Link          │
│   Audit         │    │   Analysis      │    │   Check         │
│                 │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 ▼
                    ┌─────────────────┐
                    │   Summary       │
                    │   Report        │
                    └─────────────────┘
```

### **2. 📦 Release CI** ([`release-ci.yml`](release-ci.yml))

**Purpose**: Create unified releases of the complete multi-MCU interface wrapper library

**Triggers**:
- Push to `release/*` branches
- Push to version tags (`v*`)
- Manual trigger with version input

**Features**:
- ✅ **Waits for ALL MCU Examples Build CI** to complete successfully
- ✅ **Packages Complete Library** (`inc/` + `src/` + all MCU examples)
- ✅ **MCU-Specific Firmware Packages** (ESP32, future STM32, etc.)
- ✅ **Unified Documentation** (all MCU APIs in one place)
- ✅ **GitHub Release Creation** with comprehensive multi-MCU documentation
- ✅ **Automatic Changelog** generation

**Release Package Contents**:

```text
hardfoc-interface-wrapper-v1.0.0-complete.zip
├── inc/                          # Core library headers (MCU-agnostic)
├── src/                          # Core library implementation (MCU-agnostic)
├── examples/                     # MCU-specific example implementations
│   ├── esp32/                    # ESP32 examples (current)
│   │   ├── main/                 # ESP32 example applications
│   │   ├── scripts/              # ESP32 build and flash scripts
│   │   ├── docs/                 # ESP32-specific documentation
│   │   └── app_config.yml        # ESP32 configuration
│   ├── stm32/                    # STM32 examples (future)
│   │   ├── main/                 # STM32 example applications
│   │   ├── scripts/              # STM32 build and flash scripts
│   │   └── app_config.yml        # STM32 configuration
│   └── ...                       # Additional MCU support
├── docs/                         # Unified API documentation (all MCUs)
├── firmware-binaries/            # Pre-built firmware by MCU
│   ├── esp32/                    # ESP32 firmware binaries
│   │   ├── gpio_test_Debug.bin
│   │   ├── gpio_test_Release.bin
│   │   └── ... (all ESP32 apps)
│   ├── stm32/                    # STM32 firmware binaries (future)
│   │   └── ... (all STM32 apps)
│   └── ...                       # Additional MCU firmware
├── CMakeLists.txt                # Main build configuration
├── RELEASE_README.md             # Multi-MCU release instructions
├── CHANGELOG.md                  # Automatic changelog
└── LICENSE                       # License information
```

**Release Process**:

```text
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Wait for      │    │   Package       │    │   Create        │
│   ALL MCU       │───▶│   Complete      │───▶│   Unified       │
│   Examples      │    │   Library       │    │   Release       │
│   Build CI      │    │   (inc+src+     │    │   Package       │
│   (ESP32, etc.) │    │    examples)    │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                 │                       │
                                 ▼                       ▼
                    ┌─────────────────┐    ┌─────────────────┐
                    │   Create        │    │   Upload        │
                    │   GitHub        │    │   Multi-MCU     │
                    │   Release       │    │   Assets        │
                    └─────────────────┘    └─────────────────┘
```

### **3. 📚 Documentation CI** ([`docs.yml`](docs.yml))

**Purpose**: Generate unified documentation for the multi-MCU interface wrapper library

**Triggers**:
- Push to `main`, `release/*` branches
- Pull requests to `main` (docs changes only)

**Features**:
- ✅ **Unified API Documentation** (all MCU implementations)
- ✅ **Comprehensive Link Checking**
- ✅ **Markdown Linting**
- ✅ **Spell Checking**
- ✅ **GitHub Pages Deployment**

**Documentation Process**:

```text
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Check         │    │   Generate      │    │   Deploy to     │
│   Links         │───▶│   Doxygen       │───▶│   GitHub        │
│                 │    │   Docs          │    │   Pages         │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Lint          │    │   Spell         │    │   Validate      │
│   Markdown      │    │   Check         │    │   Output        │
│                 │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

---

## 🔧 Configuration

### **Environment Variables**

All workflows use consistent environment variables:

```yaml
env:
  PROJECT_DIR: examples/esp32
  TOOLS_DIR: examples/esp32/scripts
  CACHE_VERSION: v1
```

### **Custom Tools Configuration**

The workflows automatically use the custom tools from the Git submodule:

```yaml
# Matrix generation
python3 ${{ env.TOOLS_DIR }}/generate_matrix.py --validate --verbose

# Build automation
./${{ env.TOOLS_DIR }}/build_app.sh "${{ matrix.app_name }}" "${{ matrix.build_type }}"
```

### **External Actions Integration**

All workflows leverage the custom ESP-IDF CI tools:

```yaml
# Build workflow
uses: N3b3x/hf-espidf-ci-tools/.github/workflows/build.yml@main

# Lint workflow
uses: N3b3x/hf-espidf-ci-tools/.github/workflows/lint.yml@main

# Security workflow
uses: N3b3x/hf-espidf-ci-tools/.github/workflows/security.yml@main
```

---

## 📊 Workflow Selection Guide

| Use Case | Recommended Workflow | Key Features |
|----------|---------------------|--------------|
| **ESP32 Development** | ESP32 Examples Build CI | ESP32-specific validation |
| **STM32 Development** | STM32 Examples Build CI | STM32-specific validation (future) |
| **Multi-MCU Releases** | Release CI | Waits for ALL MCU Examples Build CI + packages complete library |
| **Documentation** | Documentation CI | Unified multi-MCU documentation |

### **Workflow Triggers Summary**

| Workflow | Main | Develop | Feature/* | Release/* | Tags | Manual |
|----------|------|---------|-----------|-----------|------|--------|
| ESP32 Examples Build CI | ✅ | ✅ | ✅ | ✅ | ❌ | ✅ |
| STM32 Examples Build CI | ✅ | ✅ | ✅ | ✅ | ❌ | ✅ |
| STM32 Examples Build CI (future) | ✅ | ✅ | ✅ | ✅ | ❌ | ✅ |
| Release CI | ❌ | ❌ | ❌ | ✅ | ✅ | ✅ |
| Documentation CI | ✅ | ❌ | ❌ | ✅ | ❌ | ❌ |

---

## 🛡️ Security Features

### **Comprehensive Security Scanning**

```text
┌─────────────────────────────────────────────────────────────────┐
│                    🛡️ Security Pipeline                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │   CodeQL    │  │  Dependency │  │   Secret    │              │
│  │   Analysis  │  │   Scanning  │  │   Scanning  │              │
│  │             │  │             │  │             │              │
│  │ • C++       │  │ • CVE       │  │ • API Keys  │              │
│  │ • Python    │  │   Detection │  │ • Tokens    │              │
│  │ • Security  │  │ • License   │  │ • Passwords │              │
│  │   Patterns  │  │   Compliance│  │ • Certificates│            │
│  └─────────────┘  └─────────────┘  └─────────────┘              │
│         │                 │                 │                   │
│         └─────────────────┼─────────────────┘                   │
│                           │                                     │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                Security Report Generation                   ││
│  │                                                             ││
│  │  • Vulnerability Summary                                    ││
│  │  • Risk Assessment                                          ││
│  │  • Remediation Recommendations                              ││
│  │  • Compliance Status                                        ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

### **Security Checks Include**
- **CodeQL Analysis**: Static code analysis for security vulnerabilities
- **Dependency Scanning**: CVE detection in dependencies
- **Secret Scanning**: Detection of exposed secrets and credentials
- **License Compliance**: Open source license compatibility checking
- **SAST/DAST**: Static and dynamic application security testing

---

## ⚡ Performance Optimization

### **Parallel Execution Strategy**

```text
┌─────────────────────────────────────────────────────────────────┐
│                    ⚡ Performance Pipeline                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │   Matrix    │  │   Build     │  │   Lint      │              │
│  │ Generation  │  │   Jobs      │  │   Jobs      │              │
│  │             │  │             │  │             │              │
│  │ • 32 builds │  │ • Parallel  │  │ • Parallel  │              │
│  │ • 2 configs │  │   execution │  │   execution │              │
│  │ • 16 apps   │  │ • 4 cores   │  │ • 2 cores   │              │
│  └─────────────┘  └─────────────┘  └─────────────┘              │
│         │                 │                 │                   │
│         ▼                 ▼                 ▼                   │
│  ┌─────────────────────────────────────────────────────────────┐│
│  │                Smart Caching Strategy                       ││
│  │                                                             ││
│  │  • ESP-IDF Cache (2GB)                                      ││
│  │  • Build Artifacts Cache (1GB)                              ││
│  │  • Dependency Cache (500MB)                                 ││
│  │  • Incremental Builds                                       ││
│  └─────────────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

### **Caching Strategy**
- **ESP-IDF Cache**: Reuses ESP-IDF installation across runs
- **Build Artifacts**: Caches compiled objects and libraries
- **Dependency Cache**: Caches Python packages and tools
- **Incremental Builds**: Only rebuilds changed components

### **Resource Optimization**
- **Parallel Matrix Builds**: Up to 32 simultaneous builds
- **Smart Job Distribution**: Load balancing across available runners
- **Efficient Resource Usage**: Optimized memory and CPU utilization
- **Fast Failure Detection**: Early termination on critical failures

---

## 🔍 Troubleshooting

### **Common Issues and Solutions**

#### **1. Submodule Issues**

```text
Problem: "fatal: remote error: upload-pack: not our ref"
Solution: Update submodule reference to latest commit
Commands:
git submodule update --init --recursive
  git add examples/esp32/scripts
  git commit -m "Update submodule to latest commit"
```

#### **2. Permission Errors**

```text
Problem: "The workflow is requesting 'contents: write', but is only allowed 'contents: read'"
Solution: Add required permissions to workflow
YAML Fix:
  permissions:
    contents: write
    actions: read
```

#### **3. Script Path Issues**

```text
Problem: "python3: can't open file '.../generate_matrix.py': [Errno 2] No such file or directory"
Solution: Use correct working directory
Commands:
  cd examples/esp32/scripts
  python3 generate_matrix.py --validate --verbose
```

#### **4. Build Failures**

```text
Problem: ESP32 build failures
Solution: Check ESP-IDF environment and dependencies
Commands:
  cd examples/esp32
  ./scripts/setup_repo.sh
  ./scripts/build_app.sh gpio_test Release
```

### **Debug Commands**

```text
# Check submodule status
git submodule status

# Validate workflow syntax
yamllint .github/workflows/

# Test scripts locally
cd examples/esp32/scripts
python3 generate_matrix.py --validate --verbose

# Check ESP-IDF environment
idf.py --version
```

### **Log Analysis**

```text
┌─────────────────────────────────────────────────────────────────┐
│                    🔍 Log Analysis Guide                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Build Logs:                                                    │
│  • Look for "ERROR" or "FAILED" keywords                        │
│  • Check ESP-IDF version compatibility                          │
│  • Verify target configuration (ESP32-C6)                       │
│                                                                 │
│  Security Logs:                                                 │
│  • Review CodeQL findings                                       │
│  • Check dependency vulnerabilities                             │
│  • Verify secret scanning results                               │
│                                                                 │
│  Release Logs:                                                  │
│  • Confirm package creation success                             │
│  • Verify GitHub release creation                               │
│  • Check asset upload status                                    │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📈 Monitoring and Metrics

### **Success Metrics**
- **Build Success Rate**: >95% for main branch
- **Security Scan Pass Rate**: 100% for releases
- **Documentation Coverage**: 100% for public APIs
- **Release Frequency**: Weekly for development, monthly for stable

### **Performance Metrics**
- **Average Build Time**: <15 minutes for full matrix
- **Cache Hit Rate**: >80% for ESP-IDF and dependencies
- **Parallel Efficiency**: >90% resource utilization
- **Failure Recovery**: <5 minutes average

---

## 🎯 Best Practices

### **Development Workflow**
1. **Feature Development**: Use `feature/*` branches
2. **Testing**: Ensure ESP32 Examples Build CI passes
3. **Documentation**: Update docs for new features
4. **Release**: Create `release/*` branch and tag

### **Release Workflow**
1. **Quality Gate**: Wait for ESP32 Examples Build CI success
2. **Clean Build**: Use `clean_build: true` for releases
3. **Comprehensive Package**: Include source, docs, and firmware
4. **Documentation**: Update release notes and changelog

### **Maintenance**
1. **Regular Updates**: Keep submodules and dependencies current
2. **Security Monitoring**: Review security scan results regularly
3. **Performance Tuning**: Monitor build times and optimize caching
4. **Documentation**: Keep README and workflow docs current

---

<div align="center">

**🚀 HardFOC CI/CD Pipeline - Powering Precise Control in ESP32 Systems**

*Built with ❤️ for the HardFOC community*

</div>