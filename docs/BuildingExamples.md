# Building and Running the Example Programs

Want to see the driver in action? These examples are designed for quick
standalone tests but you can also use them as references when folding the code
into a larger project. Each program compiles to a tiny executable that talks to
your interface implementation.

A collection of small examples demonstrates how to use different parts of
the library.  This document explains how to build each one on a desktop
machine using the GNU toolchain.  The same commands apply to other
platforms with a compatible C++20 compiler.

---

## 1. Basic Command Line

All examples are built by compiling `src/TMC9660.cpp` together with the chosen
example source file. For instance, to build the BLDC example that uses Hall
sensors run:

```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp examples/BLDC_with_HALL.cpp -o hall_demo
```

Execute `./hall_demo` to run it. If your transport layer only echoes data (like
the `DummyBus` used in the docs) you’ll simply see some debug prints. When
hooked up to real hardware the same binary will drive the motor.

---

## 2. List of Examples

| File                     | Description                                       |
|-------------------------|---------------------------------------------------|
| `bootloader_example.cpp` | Configure bootloader registers.                  |
| `BLDC_with_HALL.cpp`     | BLDC motor using Hall sensor feedback.          |
| `BLDC_with_ABN.cpp`      | BLDC motor with incremental encoder.            |
| `BLDC_velocity_control.cpp` | Basic velocity loop for a DC motor.          |
| `DC_current_control.cpp`   | Open loop current drive for a DC motor.        |
| `Stepper_FOC.cpp`          | Closed loop FOC control of a stepper motor.    |
| `Stepper_step_dir.cpp`     | Enable the STEP/DIR interface.                 |
| `Telemetry_monitor.cpp`    | Continuously read telemetry values.            |

Use the same compilation pattern for each file, replacing the source name
and output executable.

---

## 3. Including Bootloader Support

Some examples require the bootloader helper class.  Add
`src/TMC9660Bootloader.cpp` to the command line when building those
programs:

```bash
g++ -std=c++20 -Iinc src/TMC9660.cpp src/TMC9660Bootloader.cpp \
    examples/bootloader_example.cpp -o boot_demo
```

---

## 4. Cleaning Up 🧹

The repository does not include a Makefile, so compiled objects are left
in the working directory. Remove them with `rm` or create your own build
scripts once you have verified the commands above work on your system.
When integrating into a larger project you will most likely add these
source files to your existing `Makefile` or `CMakeLists.txt`.


---

[⬅️ Prev](ImplementingCommInterface.md) | [⬆️ Back to Index](index.md) | [Next ➡️](HardwareAgnosticExamples.md)
