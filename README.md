# HF-TMC9660
Hardware Agnostic TMC9660 library - as used in the HardFOC-V1 controller

# TMC9660 C++ Library

This is a C++ library for the **TMC9660** smart gate driver and motor controller IC, providing high-level access to all of its Parameter Mode features. The library abstracts the device's TMCL (Trinamic Motion Control Language) protocol and allows configuration and control of DC, BLDC, and stepper motors with Field-Oriented Control (FOC) and various feedback options. 

## Features

- **Abstract Communication Interface:** An abstract base class `TMC9660CommInterface` defines the transport layer (e.g., SPI or UART). This allows the library to be hardware-agnostic. Users implement this interface for their specific communication bus.
- **Motor Configuration:** Set up motor type (brushed DC, BLDC/PMSM, or stepper), motor parameters (pole pairs, etc.), and commutation modes (open-loop or FOC with Hall, encoder, etc.).
- **FOC Tuning:** Adjust current (torque/flux) PI controller gains, velocity PI gains, and position controller gain for fine-tuning closed-loop performance.
- **Gate Driver Settings:** Configure gate driver output current (drive strength), dead-time (break-before-make), adaptive gate timing, and output polarity for external MOSFETs.
- **Feedback Sensors:** Support for digital Hall sensors, incremental ABN encoders, and SPI-based absolute encoders. Functions to configure sensor parameters (e.g., encoder counts per revolution, hall alignment offsets).
- **Protection Features:** Set thresholds for over-voltage/under-voltage warnings, configure over-temperature limits (both chip and external sensor), and enable/disable overcurrent protection on driver outputs.
- **Script Execution:** Upload and execute TMCL scripts on the TMC9660's internal microcontroller for standalone operation or custom behavior. Functions are provided to enter download mode, send a script, start/stop the script.
- **RAM Debug (Data Logging):** Control the on-chip data logging (RAMDebug) feature to capture runtime data (currents, velocities, etc.) at high speed for tuning or diagnostics.
- **Telemetry:** Read real-time telemetry such as supply voltage, driver IC temperature, motor current, actual velocity, and position.

All functionality is accessed through a single `TMC9660` class, which uses the communication interface to send commands and retrieve data from the device. The library is thoroughly documented with **Doxygen** comments for all classes and methods.

## File Organization

```text
├── include/
│   └── TMC9660.hpp        # Main library header (public API and class definitions)
├── src/
│   └── TMC9660.cpp        # Implementation of TMC9660 methods
├── examples/
│   ├── BLDC_with_Hall.cpp       # Example: BLDC motor with Hall sensor feedback
│   ├── DC_velocity_control.cpp  # Example: DC motor in velocity control mode
│   └── Telemetry_monitor.cpp    # Example: Reading telemetry data (temperature, current, voltage)
└── docs/
    └── UsageGuide.md      # Additional usage instructions and integration guide
