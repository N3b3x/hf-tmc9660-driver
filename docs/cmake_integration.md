# CMake Integration — TMC9660 Driver


## Quick start (ESP-IDF)

The ESP-IDF component wrapper lives at:

```
examples/esp32/components/hf_tmc9660/CMakeLists.txt
```

It imports `cmake/hf_tmc9660_build_settings.cmake`, which is the single
source of truth for sources, includes, and version information.

## Layer overview

| Layer | File | Role |
|-------|------|------|
| 1 — Settings | `cmake/hf_tmc9660_build_settings.cmake` | Sources, includes, version |
| 2 — Root | `CMakeLists.txt` | Desktop / CI static library |
| 3 — ESP-IDF | `examples/esp32/components/…/CMakeLists.txt` | IDF wrapper |

## Version

The version is defined once in `hf_tmc9660_build_settings.cmake` and
stamped into `inc/tmc9660_version.h.in` → `tmc9660_version.h` via
`configure_file()`.

Access at runtime:

```cpp
using namespace tmc9660;
TMC9660<MySpi>::GetDriverVersion();      // "1.0.0"
TMC9660<MySpi>::GetDriverVersionMajor(); // 1
tmc9660::GetDriverVersion();             // "1.0.0" (namespace-level)
```
