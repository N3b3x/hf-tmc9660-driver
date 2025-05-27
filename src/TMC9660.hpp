#pragma once

#include <cstdint>
#include <cstdlib>
#include <array>
#include <vector>

/**
 * @brief Abstract communication interface for sending/receiving TMCL datagrams (64-bit) to the TMC9660.
 * 
 * Users should derive from this class and implement the pure virtual methods to interface with actual 
 * hardware (e.g., via SPI or UART). The library will use this interface to send commands and receive responses
 * from the TMC9660. All data exchanges are 64-bit TMCL frames (typically 8 bytes).
 */
class TMC9660CommInterface {
public:
    /**
     * @brief Virtual destructor for interface.
     */
    virtual ~TMC9660CommInterface() = default;
    /**
     * @brief Send a 64-bit TMCL datagram to the device.
     * 
     * This function should send an 8-byte command frame out via the physical interface. For SPI, this might
     * be implemented as a single 8-byte full-duplex transfer. For UART, this might send the 9-byte frame 
     * (including address and checksum) and not wait for the reply.
     * 
     * @param data 8-byte array containing the command datagram to send (without any trailing checksum).
     * @return true if the datagram was successfully transmitted, false if an error occurred.
     */
    virtual bool sendDatagram(const std::array<uint8_t, 8>& data) = 0;
    /**
     * @brief Receive a 64-bit TMCL datagram from the device.
     * 
     * This function should read an 8-byte response frame from the interface. For SPI, this may be filled 
     * by the same transaction as sendDatagram if full-duplex, or by a subsequent transaction if required.
     * For UART, this should read the 9-byte response (address + 7 bytes + checksum) and return the core 8 bytes.
     * 
     * @param data 8-byte array to store the received datagram.
     * @return true if a response datagram was received successfully, false if an error or timeout occurred.
     */
    virtual bool receiveDatagram(std::array<uint8_t, 8>& data) = 0;
    /**
     * @brief Optional combined transmit/receive for a 64-bit datagram.
     * 
     * Default implementation uses sendDatagram() then receiveDatagram(), but it can be overridden to perform
     * a single atomic transaction if the interface supports it (e.g. SPI).
     * 
     * @param txData 8-byte array containing the command datagram to send.
     * @param rxData 8-byte array to store the received datagram.
     * @return true if the transfer was successful and rxData contains the response.
     */
    virtual bool transferDatagram(const std::array<uint8_t, 8>& txData, std::array<uint8_t, 8>& rxData) {
        if (!sendDatagram(txData)) {
            return false;
        }
        return receiveDatagram(rxData);
    }
};

/**
 * @brief Main class representing a TMC9660 motor driver in Parameter Mode.
 * 
 * Provides high-level functions to configure and control the TMC9660's features. The class uses a 
 * TMC9660CommInterface for communication, making it agnostic to the physical layer.
 * 
 * All configuration and control is done by sending TMCL commands (Set/Get Parameter, etc.) and reading/writing
 * parameter values. The library covers motor type setup, FOC control tuning, gate driver settings, sensor feedback
 * configuration, protection (voltage, temperature, current) settings, executing scripts on the device's microcontroller,
 * and reading telemetry data like temperature, current, and voltage.
 */
class TMC9660 {
public:
    /// Motor types supported by the TMC9660.
    enum class MotorType : uint8_t {
        DC = 0,      /**< Brushed DC motor. */
        BLDC = 1,    /**< Brushless DC (3-phase) motor. */
        STEPPER = 2  /**< Two-phase bipolar stepper motor. */
    };
    /// Commutation modes for motor control (for BLDC/stepper primarily).
    enum class CommutationMode : uint8_t {
        SYSTEM_OFF = 0,                    /**< System off (outputs disabled, motor free). */
        FOC_OPENLOOP_VOLTAGE = 3,          /**< Open-loop voltage mode (constant duty cycle output). */
        FOC_OPENLOOP_CURRENT = 4,          /**< Open-loop current mode (constant current to motor). */
        FOC_ENCODER = 5,                   /**< FOC with ABN encoder feedback. */
        FOC_HALL = 6,                      /**< FOC with Hall sensor feedback. */
        FOC_SPI_ENCODER = 8                /**< FOC with SPI-based encoder feedback. */
        // Note: Modes 1,2 are special states for all FETs on (low or high side) typically for idle/braking; mode 7 is reserved.
    };
    /// Motor direction setting (for DC motors or direction inversion).
    enum class MotorDirection : uint8_t {
        FORWARD = 0,  /**< Normal rotation direction. */
        REVERSE = 1   /**< Inverted rotation direction. */
    };

    /**
     * @brief Construct a TMC9660 driver instance.
     * 
     * @param comm Reference to a user-implemented communication interface (SPI, UART, etc).
     * @param address (Optional) Module address if multiple TMC9660 devices are on one bus. For SPI, this is typically 0.
     */
    TMC9660(TMC9660CommInterface& comm, uint8_t address = 0);

    // -------- Core Parameter Access Methods --------
    /**
     * @brief Set (write) an axis (motor-specific) parameter on the TMC9660.
     * @param id Parameter ID number (see TMC9660 documentation for the full list).
     * @param value 32-bit value to write to the parameter.
     * @param motorIndex Index of the motor/axis (0 or 1). Typically 0 unless the device controls multiple axes.
     * @return true if the parameter was successfully written (acknowledged by the device), false if an error occurred.
     */
    bool writeParameter(uint16_t id, uint32_t value, uint8_t motorIndex = 0);
    /**
     * @brief Read an axis (motor-specific) parameter from the TMC9660.
     * @param id Parameter ID number to read.
     * @param[out] value Reference to store the 32-bit parameter value read.
     * @param motorIndex Index of the motor/axis (0 or 1).
     * @return true if the parameter was successfully read (device responded), false on error.
     */
    bool readParameter(uint16_t id, uint32_t& value, uint8_t motorIndex = 0);
    /**
     * @brief Set (write) a global parameter on the TMC9660.
     * @param id Global parameter ID number.
     * @param bank Bank number for global parameter (some global parameters are grouped in banks, otherwise 0).
     * @param value 32-bit value to write.
     * @return true if successfully written, false if an error occurred.
     */
    bool writeGlobalParameter(uint16_t id, uint8_t bank, uint32_t value);
    /**
     * @brief Read a global parameter from the TMC9660.
     * @param id Global parameter ID number.
     * @param bank Bank number or index associated with the parameter.
     * @param[out] value Reference to store the read 32-bit value.
     * @return true if read successfully, false on error.
     */
    bool readGlobalParameter(uint16_t id, uint8_t bank, uint32_t& value);

    // -------- Motor Configuration and Control Methods --------
    /**
     * @brief Configure the motor type (DC, BLDC, or stepper) and basic motor settings.
     * 
     * This sets the MOTOR_TYPE parameter and optionally related parameters like pole pairs for BLDC or microstep settings for steppers.
     * @param type MotorType (DC, BLDC, STEPPER).
     * @param polePairs For BLDC motors, number of pole pairs. For stepper or DC, this can be set to 1.
     * @return true if the motor type was set successfully, false if communication or device error.
     */
    bool configureMotorType(MotorType type, uint8_t polePairs = 1);
    /**
     * @brief Set the motor direction inversion.
     * 
     * This configures the MOTOR_DIRECTION parameter which inverts the meaning of "forward" direction for the motor.
     * @param direction MotorDirection (FORWARD or REVERSE).
     * @return true if successfully set.
     */
    bool setMotorDirection(MotorDirection direction);
    /**
     * @brief Set the PWM frequency for the motor driver.
     * @param frequencyHz PWM frequency in Hertz (allowed range 10kHz to 100kHz).
     * @return true if set successfully, false if an error occurred.
     */
    bool setPWMFrequency(uint32_t frequencyHz);
    /**
     * @brief Configure the commutation mode for the motor.
     * 
     * This sets how the motor is driven: e.g., open-loop or closed-loop FOC with various sensor feedback options.
     * Typically used for BLDC or stepper motors. For DC motors, commutation modes are not applicable (the motor is driven via H-bridge).
     * @param mode A CommutationMode value (e.g. FOC_HALL, FOC_ENCODER, FOC_OPENLOOP_CURRENT, etc).
     * @return true if the mode was applied successfully.
     */
    bool setCommutationMode(CommutationMode mode);
    /**
     * @brief Set the maximum allowed motor current (torque limit).
     * 
     * This sets the MAX_TORQUE parameter which limits the peak current/torque that the controller will deliver to the motor.
     * @param milliamps Maximum current in milliamps.
     * @return true on success, false on error.
     */
    bool setMaxCurrent(uint16_t milliamps);
    /**
     * @brief Stop any motor motion immediately.
     * 
     * Sends the "MST" (motor stop) command which should stop any ongoing movement or ramp. This does not disable the motor driver, 
     * but halts the velocity/position motion.
     * @return true if the stop command was acknowledged, false if not.
     */
    bool stopMotor();
    /**
     * @brief Set a target torque (current) for torque control mode.
     * 
     * Writing a target torque will engage the torque control loop (current control) of the TMC9660. The motor will attempt to output 
     * the requested current (torque) up to the maximum limits.
     * @param milliamps Desired motor current in mA (positive for forward torque, negative for reverse torque).
     * @return true if the target was sent successfully.
     */
    bool setTargetTorque(int16_t milliamps);
    /**
     * @brief Set a target velocity for velocity control mode.
     * 
     * Writing a target velocity engages the velocity control loop. The motor will ramp to and hold the specified velocity if possible.
     * An appropriate sensor (like Hall or encoder) is required for closed-loop velocity mode on BLDC/stepper. For DC motors, an encoder is needed for velocity feedback.
     * @param velocity Target velocity value. The unit is in internal ticks (position counts per second) unless scaled via parameters.
     * @return true if the target velocity was sent successfully.
     */
    bool setTargetVelocity(int32_t velocity);
    /**
     * @brief Set a target position for position control mode.
     * 
     * Writing a target position engages the position control loop. The motor will move to the specified position using the internal motion controller and ramp generator.
     * Requires a position sensor (incremental encoder or equivalent) to be configured.
     * @param position Target position in internal units (ticks).
     * @return true if the target position was sent successfully.
     */
    bool setTargetPosition(int32_t position);

    // -------- FOC Tuning and Controller Gains --------
    /**
     * @brief Set the tuning parameters for the current (torque/flux) PI controller.
     * @param p Gain P for current (torque) controller.
     * @param i Gain I for current (torque) controller.
     * @param separateFlux If true, assume separate flux PI controller and set its gains similarly (requires chip support).
     * @param fluxP (Optional) P gain for flux controller, if separateFlux is true.
     * @param fluxI (Optional) I gain for flux controller, if separateFlux is true.
     * @return true if the parameters were set successfully.
     */
    bool setCurrentLoopGains(uint16_t p, uint16_t i, bool separateFlux = false, uint16_t fluxP = 0, uint16_t fluxI = 0);
    /**
     * @brief Set the tuning parameters for the velocity PI controller.
     * @param p Gain P for velocity controller.
     * @param i Gain I for velocity controller.
     * @return true if the parameters were set successfully.
     */
    bool setVelocityLoopGains(uint16_t p, uint16_t i);
    /**
     * @brief Set the tuning parameters for the position control loop (P controller).
     * @param p Gain P for position controller.
     * @return true if successfully set.
     */
    bool setPositionGain(uint16_t p);

    // -------- Gate Driver Settings --------
    /**
     * @brief Configure the gate driver output currents and timing (drive strength and dead time).
     * 
     * This function adjusts the gate driver for external MOSFETs. It sets the sink/source current limits and the deadtime (break-before-make).
     * @param sinkLevel Drive sink current level (0-15 corresponding to ~50mA up to 2000mA).
     * @param sourceLevel Drive source current level (0-15 corresponding to ~25mA up to 1000mA).
     * @param deadTimeNs Dead-time in nanoseconds (approximately). The value will be quantized to the nearest supported step (8.33ns units).
     * @return true if the configuration commands were sent successfully.
     */
    bool configureGateDriver(uint8_t sinkLevel, uint8_t sourceLevel, uint16_t deadTimeNs);
    /**
     * @brief Set the gate driver output polarity.
     * 
     * By default, the PWM_L and PWM_H outputs are active-high. This function can invert those outputs if needed by external gate circuits.
     * @param lowActive True to invert the low-side gate outputs (active low), false for active high.
     * @param highActive True to invert the high-side gate outputs, false for active high.
     * @return true if the polarity was set successfully.
     */
    bool setGateOutputPolarity(bool lowActive, bool highActive);

    // -------- Feedback Sensors Configuration --------
    /**
     * @brief Configure digital Hall sensors for BLDC commutation.
     * 
     * This enables Hall sensor inputs as the feedback for commutation. Typically used with CommutationMode::FOC_HALL.
     * @param hallOrder An optional parameter defining the expected Hall sensor sequence order (if needed for alignment).
     * @param inverted If true, invert the interpretation of hall sensor signals.
     * @return true if Hall sensor feedback is configured successfully.
     */
    bool configureHallSensors(uint8_t hallOrder = 0, bool inverted = false);
    /**
     * @brief Configure an ABN incremental encoder for feedback.
     * 
     * Sets up an incremental quadrature encoder with optional index (N) channel for position and velocity feedback.
     * @param countsPerRev Encoder resolution (counts per revolution).
     * @param inverted If true, invert the encoder direction.
     * @return true if encoder parameters were set successfully.
     */
    bool configureEncoder(uint32_t countsPerRev, bool inverted = false);
    /**
     * @brief Configure a SPI-based encoder for feedback.
     * 
     * Sets up a digital SPI encoder (e.g., absolute magnetic encoder) for position feedback.
     * @param mode SPI encoder mode/format selection (depends on specific encoder supported by TMC9660).
     * @return true if configured successfully.
     */
    bool configureSPIEncoder(uint8_t mode);

    // -------- Protection and Fault Handling --------
    /**
     * @brief Configure overvoltage and undervoltage protection thresholds.
     * @param overVoltThreshold Over-voltage warning threshold in units of 0.1V.
     * @param underVoltThreshold Under-voltage warning threshold in units of 0.1V.
     * @return true if thresholds were set successfully.
     */
    bool configureVoltageProtection(uint16_t overVoltThreshold, uint16_t underVoltThreshold);
    /**
     * @brief Configure over-temperature protection thresholds.
     * 
     * The TMC9660 has an internal temperature sensor and supports an external analog temperature sensor.
     * This function sets warning and shutdown thresholds for the internal sensor.
     * @param warningDegC Warning threshold in °C for chip temperature.
     * @param shutdownDegC Shutdown (fault) threshold in °C for chip temperature.
     * @return true if set successfully.
     */
    bool configureTemperatureProtection(float warningDegC, float shutdownDegC);
    /**
     * @brief Enable or disable overcurrent protection on the driver outputs.
     * 
     * This controls the internal gate driver overcurrent detection (e.g., comparators) for various FETs.
     * @param enabled True to enable overcurrent protection (shut down drivers on overcurrent), false to disable.
     * @return true if command was sent successfully.
     */
    bool setOvercurrentProtectionEnabled(bool enabled);

    // -------- Script Execution Control --------
    /**
     * @brief Upload a TMCL script to the TMC9660's internal memory.
     * 
     * This enters download mode, writes a series of instructions to the device memory, and exits download mode.
     * The script will typically run on device startup or when triggered.
     * @param scriptData Vector of 32-bit instructions representing the TMCL script.
     * @return true if the script was uploaded successfully.
     */
    bool uploadScript(const std::vector<uint32_t>& scriptData);
    /**
     * @brief Start or restart execution of the stored script.
     * @param address The address from which to start execution (usually 0 for beginning of script).
     * @return true if the command to start the script was sent.
     */
    bool startScript(uint16_t address = 0);
    /**
     * @brief Stop execution of the running script.
     * @return true if the stop command was sent successfully.
     */
    bool stopScriptExecution();

    // -------- RAM Debugging (Data Logging) --------
    /**
     * @brief Initialize and configure the RAMDebug feature (data logging).
     * 
     * This sends a command to initialize the RAM debug system and reset any previous configurations.
     * @param sampleCount Number of samples to collect in the buffer.
     * @return true if the RAM debug was initialized properly.
     */
    bool initRAMDebug(uint32_t sampleCount);
    /**
     * @brief Start capturing data using RAMDebug.
     * @return true if the capture started successfully.
     */
    bool startRAMDebugCapture();
    /**
     * @brief Read a captured sample from RAMDebug buffer.
     * @param index Sample index to read (0-based).
     * @param[out] data Variable to store the 32-bit sample data.
     * @return true if the data was read successfully.
     */
    bool readRAMDebugData(uint32_t index, uint32_t& data);
    /**
     * @brief Get the current state of the RAM debug engine.
     * @param[out] isRunning Will be set to true if capture is ongoing, false if stopped or idle.
     * @return true if the status was retrieved successfully.
     */
    bool getRAMDebugStatus(bool& isRunning);

    // -------- Telemetry and Status Reading --------
    /**
     * @brief Read the internal chip temperature.
     * @return Chip temperature in degrees Celsius.
     * 
     * If reading the temperature fails, this returns a negative value (e.g. -273) to indicate an error.
     * The temperature is computed from the raw sensor reading using the formula: T(°C) = value * 0.01615 - 268.15.
     */
    float getChipTemperature();
    /**
     * @brief Read the current motor current (torque current).
     * @return Motor current in milliamps (mA). Returns 0 if unable to read.
     */
    int16_t getMotorCurrent();
    /**
     * @brief Read the current supply (bus) voltage.
     * @return Supply voltage in volts. Returns a negative value if unable to read.
     */
    float getSupplyVoltage();
    /**
     * @brief Read the measured actual velocity of the motor.
     * @return The actual velocity in internal units (typically encoder counts per second). Returns 0 if not available.
     */
    int32_t getActualVelocity();
    /**
     * @brief Read the measured actual position of the motor.
     * @return The actual position in internal units (encoder counts). Returns 0 if not available.
     */
    int32_t getActualPosition();

private:
    TMC9660CommInterface& comm_;  ///< Communication interface (transport) for sending/receiving data.
    uint8_t address_;            ///< Module address (0-127). Used primarily for UART multi-drop addressing.

    // Helper: pack a TMCL command into 8 bytes and send/receive.
    bool sendCommand(uint8_t opcode, uint16_t type, uint8_t motor, uint32_t value, uint32_t* reply = nullptr);
};
