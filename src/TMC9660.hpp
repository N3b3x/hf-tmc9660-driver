#pragma once

#include <cstdint>
#include <cstdlib>
#include <array>
#include <vector>
#include <span>

#include "../inc/parameter_mode/tmc9660_param_mode_tmcl.hpp"

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
    virtual ~TMC9660CommInterface() noexcept = default;
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
    virtual bool sendDatagram(const std::array<uint8_t, 8>& data) noexcept = 0;
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
    virtual bool receiveDatagram(std::array<uint8_t, 8>& data) noexcept = 0;
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
    virtual bool transferDatagram(const std::array<uint8_t, 8>& txData, std::array<uint8_t, 8>& rxData) noexcept {
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
    /// Heartbeat watchdog modes for communication monitoring
    enum class HeartbeatMode : uint8_t { WATCHDOG_DISABLE = 0, WATCHDOG_ENABLE = 1 };
    /// Power-down sleep periods
    enum class PowerDownPeriod : uint8_t { PERIOD_1 = 0, PERIOD_2, PERIOD_3, PERIOD_4, PERIOD_5, PERIOD_6 };
    /// Fault retry and final actions
    enum class FaultRetryAction : uint8_t { NO_RETRY = 0, RETRY = 1 };
    enum class FaultFinalAction : uint8_t { DISABLE_MOTOR = 0, KEEP_RUNNING = 1 };

    /**
     * @brief Construct a TMC9660 driver instance.
     * 
     * @param comm Reference to a user-implemented communication interface (SPI, UART, etc).
     * @param address (Optional) Module address if multiple TMC9660 devices are on one bus. For SPI, this is typically 0.
     */
    TMC9660(TMC9660CommInterface& comm, uint8_t address = 0) noexcept;

    //***************************************************************************
    //**               CORE PARAMETER ACCESS METHODS                         **//
    //***************************************************************************

    /**
     * @brief Set (write) an axis (motor-specific) parameter on the TMC9660.
     * @param id Parameter ID number (see TMC9660 documentation for the full list).
     * @param value 32-bit value to write to the parameter.
     * @param motorIndex Index of the motor/axis (0 or 1). Typically 0 unless the device controls multiple axes.
     * @return true if the parameter was successfully written (acknowledged by the device), false if an error occurred.
     */
    [[nodiscard]] bool writeParameter(tmc9660::tmcl::Parameters id, uint32_t value, uint8_t motorIndex = 0) noexcept;
    /**
     * @brief Read an axis (motor-specific) parameter from the TMC9660.
     * @param id Parameter ID number to read.
     * @param[out] value Reference to store the 32-bit parameter value read.
     * @param motorIndex Index of the motor/axis (0 or 1).
     * @return true if the parameter was successfully read (device responded), false on error.
     */
    [[nodiscard]] bool readParameter(tmc9660::tmcl::Parameters id, uint32_t& value, uint8_t motorIndex = 0) noexcept;
    /**
     * @brief Set (write) a global parameter on the TMC9660.
     * @param id Global parameter ID number.
     * @param bank Bank number for global parameter (some global parameters are grouped in banks, otherwise 0).
     * @param value 32-bit value to write.
     * @return true if successfully written, false if an error occurred.
     */
    [[nodiscard]] bool writeGlobalParameter(tmc9660::tmcl::Parameters id, uint8_t bank, uint32_t value) noexcept;
    /**
     * @brief Read a global parameter from the TMC9660.
     * @param id Global parameter ID number.
     * @param bank Bank number or index associated with the parameter.
     * @param[out] value Reference to store the read 32-bit value.
     * @return true if read successfully, false on error.
     */
    [[nodiscard]] bool readGlobalParameter(tmc9660::tmcl::Parameters id, uint8_t bank, uint32_t& value) noexcept;

    //***************************************************************************
    //**                  SUBSYSTEM: Motor Configuration                       **//
    //***************************************************************************
    /**
     * @brief Subsystem for configuring motor type and basic settings
     */
    struct MotorConfig {
        /**
         * @brief Configure the motor type (DC, BLDC, or stepper) and basic motor settings.
         * 
         * This sets the MOTOR_TYPE parameter and optionally related parameters like pole pairs for BLDC or microstep settings for steppers.
         * @param type MotorType (DC, BLDC, STEPPER).
         * @param polePairs For BLDC motors, number of pole pairs. For stepper or DC, this can be set to 1.
         * @return true if the motor type was set successfully, false if communication or device error.
         */
        bool setType(tmc9660::tmcl::MotorType type, uint8_t polePairs = 1) noexcept;

        /**
         * @brief Set the motor direction inversion.
         * 
         * This configures the MOTOR_DIRECTION parameter which inverts the meaning of "forward" direction for the motor.
         * @param direction MotorDirection (FORWARD or REVERSE).
         * @return true if successfully set.
         */
        bool setDirection(tmc9660::tmcl::MotorDirection direction) noexcept;

        /**
         * @brief Set the PWM frequency for the motor driver.
         * @param frequencyHz PWM frequency in Hertz (allowed range 10kHz to 100kHz).
         * @return true if set successfully, false if an error occurred.
         */
        bool setPWMFrequency(uint32_t frequencyHz) noexcept;
        
        /**
         * @brief Set the PWM switching scheme for the motor driver.
         * 
         * This configures how the PWM signals are generated for driving the motor phases.
         * Different schemes offer varying trade-offs between voltage utilization, switching losses,
         * and current measurement windows.
         * 
         * @param scheme PWM switching scheme:
         *               0: STANDARD - Standard PWM modulation (86% max voltage for BLDC)
         *               1: SVPWM - Space Vector PWM (100% max voltage for BLDC, balanced load on FETs)
         *               2: FLAT_BOTTOM - Flat bottom PWM (100% max voltage for BLDC, extended current measurement window)
         * @return true if set successfully, false if an error occurred.
         * 
         * @note For BLDC motors, SVPWM and Flat Bottom schemes allow full voltage utilization.
         *       For Stepper/DC motors, only standard and flat bottom modes are available with 100% duty cycle.
         */
        bool setPWMSwitchingScheme(uint8_t scheme) noexcept;
        
        /**
         * @brief Configure the commutation mode for the motor.
         * 
         * This sets how the motor is driven: e.g., open-loop or closed-loop FOC with various sensor feedback options.
         * Typically used for BLDC or stepper motors. For DC motors, commutation modes are not applicable except for
         * sensor feedback in velocity/position control modes.
         * 
         * @param mode A CommutationMode value defining the motor control strategy:
         *             - SYSTEM_OFF: Motor disabled (default state after power-on/reset)
         *             - FOC_OPENLOOP_VOLTAGE: Constant duty cycle (voltage) control without feedback
         *             - FOC_OPENLOOP_CURRENT: Constant current control without position feedback
         *             - FOC_ENCODER: Field-oriented control with ABN encoder feedback
         *             - FOC_HALL: Field-oriented control with Hall sensor feedback
         *             - FOC_SPI_ENCODER: Field-oriented control with SPI encoder feedback
         * 
         * @return true if the mode was applied successfully.
         * 
         * @note When using SYSTEM_OFF, the behavior is controlled by IDLE_MOTOR_PWM_BEHAVIOR parameter.
         *       For open-loop modes, additional parameters must be set (OPENLOOP_VOLTAGE or OPENLOOP_CURRENT).
         *       For sensor-based modes, the appropriate sensor must be configured before enabling this mode.
         */
        bool setCommutationMode(tmc9660::tmcl::CommutationMode mode) noexcept;

        /**
         * @brief Set the maximum allowed motor current (torque limit).
         * 
         * This sets the MAX_TORQUE parameter which limits the peak current/torque that the controller will deliver to the motor.
         * @param milliamps Maximum current in milliamps.
         * @return true on success, false on error.
         */
        bool setMaxCurrent(uint16_t milliamps) noexcept;

        /**
         * @brief Set the maximum allowed flux current for BLDC/stepper motors.
         * 
         * This sets the MAX_FLUX parameter which limits the flux-producing current component.
         * Important for field-weakening operation and stepper motor control.
         * @param milliamps Maximum flux current in milliamps.
         * @return true on success, false on error.
         */
        bool setMaxFluxCurrent(uint16_t milliamps) noexcept;

        /**
         * @brief Limit the motor’s peak flux (|Ψ|max) used by the torque controller.
         *
         * Maps to the register pair MAX_FLUX / FLUX_LIMIT_ENABLE and is essential
         * for high-speed field-weakening operation.                                   
         * @param mWb  Maximum flux in milli-Weber (0 … 32767).
         * @return true on success.
         */
        bool setMaxFlux(uint16_t mWb) noexcept;

        /**
         * @brief Configure field-weakening (automatic flux reduction above a knee speed).
         *
         * @param startRPM   Electrical RPM where weakening starts.
         * @param slope      Weakening slope (dΨ/dω) as signed 16-bit value.
         * @param minFlux    Minimum allowed flux in mWb.
         * @return true on success.
         *
         * Registers: WEAKENING_FACTOR, FIELD_WEAKENING_ENABLE :contentReference[oaicite:0]{index=0}
         */
        bool configureFieldWeakening(uint16_t startRPM,
                                    int16_t  slope,
                                    uint16_t minFlux) noexcept;


        /**
         * @brief Configure the ADC shunt type for current measurement.
         * 
         * Sets the current sense resistor configuration for motor phase current sensing.
         * @param shuntType Type of shunt configuration:
         *                  0: INLINE_UVW - Inline shunts on all phases
         *                  1: INLINE_VW - Inline shunts on V and W phases only
         *                  2: INLINE_UW - Inline shunts on U and W phases only
         *                  3: INLINE_UV - Inline shunts on U and V phases only
         *                  4: BOTTOM_SHUNTS - Low-side (bottom) shunts configuration
         * @return true if successfully configured.
         */
        bool setADCShuntType(uint8_t shuntType) noexcept;

        /**
         * @brief Set the current scaling factor for ADC measurements.
         * 
         * The scaling factor converts ADC readings to real-world current values in milliamps.
         * When set correctly, all current-related parameters (target torque, actual torque, etc.) are in mA.
         * 
         * @param scalingFactor Scaling factor calculated using:
         *                      For RMS: Factor ≈ 27.62 / (GCSA × RShunt)
         *                      For Peak: Factor ≈ 39.06 / (GCSA × RShunt)
         * @return true if successfully set.
         */
        bool setCurrentScalingFactor(float scalingFactor) noexcept;

        /**
         * @brief Configure the ADC inversion settings for each phase.
         * 
         * Sets whether each ADC input should be inverted based on motor type and shunt configuration.
         * See Table 24 in the documentation for recommended settings.
         * 
         * @param invertADC0 True to invert ADC0 readings
         * @param invertADC1 True to invert ADC1 readings
         * @param invertADC2 True to invert ADC2 readings
         * @param invertADC3 True to invert ADC3 readings (if used)
         * @return true if successfully configured.
         */
        bool configureADCInversion(bool invertADC0, bool invertADC1, bool invertADC2, bool invertADC3 = false) noexcept;

        /**
         * @brief Set the gain for the current sense amplifiers.
         * 
         * Configures the gain for the internal current sense amplifiers to match your shunt resistor values.
         * 
         * @param gainADC0_1 Gain setting for ADC channels I0 and I1 (0-31)
         * @param gainADC2_3 Gain setting for ADC channels I2 and I3 (0-31)
         * @return true if successfully set.
         */
        bool setCurrentSenseAmplifierGain(uint8_t gainADC0_1, uint8_t gainADC2_3) noexcept;

        /**
         * @brief Set scaling factors for individual ADC channels to account for shunt resistor tolerances.
         * 
         * Fine-tunes the scaling for each ADC channel to compensate for component variations.
         * 
         * @param scaleADC0 Scaling factor for ADC I0 (default: 1.0)
         * @param scaleADC1 Scaling factor for ADC I1 (default: 1.0)
         * @param scaleADC2 Scaling factor for ADC I2 (default: 1.0)
         * @param scaleADC3 Scaling factor for ADC I3 (default: 1.0)
         * @return true if successfully set.
         */
        bool setADCScalingFactors(float scaleADC0 = 1.0f, float scaleADC1 = 1.0f, 
                                  float scaleADC2 = 1.0f, float scaleADC3 = 1.0f) noexcept;

        /**
         * @brief Configure velocity loop downsampling to adjust control loop frequency.
         * 
         * Slows down the velocity control loop relative to the PWM frequency by the specified factor.
         * 
         * @param divider Downsampling divider (1 = no downsampling, 2 = half frequency, etc.)
         * @return true if successfully configured.
         */
        bool setVelocityLoopDownsampling(uint8_t divider) noexcept;

        /**
         * @brief Configure position loop downsampling relative to velocity loop.
         * 
         * Further slows down the position control loop relative to the velocity loop by the specified factor.
         * 
         * @param divider Downsampling divider (1 = same as velocity loop, 2 = half frequency, etc.)
         * @return true if successfully configured.
         */
        bool setPositionLoopDownsampling(uint8_t divider) noexcept;

        /**
         * @brief Calibrate the ADC offsets for current measurement.
         * 
         * Initiates a calibration sequence for the current sensing ADCs.
         * Motor must be stationary and commutation mode set to off before calibration.
         * 
         * @param waitForCompletion If true, wait until calibration is completed
         * @param timeoutMs Timeout in milliseconds if waiting for completion
         * @return true if calibration was started (and completed if waitForCompletion is true)
         */
        bool calibrateADCOffsets(bool waitForCompletion = false, uint32_t timeoutMs = 1000) noexcept;

    private:
        friend class TMC9660;
        explicit MotorConfig(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } motorConfig{*this};
    
    //***************************************************************************
    //**                  SUBSYSTEM: Gate Driver                              **//
    //***************************************************************************
    /**
     * @brief Subsystem for configuring the MOSFET gate driver
     */
    struct GateDriver {
        /**
         * @brief Configure the gate driver output currents and timing (drive strength and dead time).
         * 
         * This function adjusts the gate driver for external MOSFETs. It sets the sink/source current levels and the deadtime (break-before-make).
         * @param sinkLevel Drive sink current level (0-15 corresponding to ~50mA up to 2000mA).
         * @param sourceLevel Drive source current level (0-15 corresponding to ~25mA up to 1000mA).
         * @param deadTimeNs Dead-time in nanoseconds (approximately). The value will be quantized to the nearest supported step (8.33ns units).
         * @return true if the configuration commands were sent successfully.
         */
        bool configure(uint8_t sinkLevel, uint8_t sourceLevel, uint16_t deadTimeNs) noexcept;
        
        /**
         * @brief Set the gate driver output polarity.
         * 
         * By default, the PWM_L and PWM_H outputs are active-high. This function can invert those outputs if needed by external gate circuits.
         * @param lowActive True to invert the low-side gate outputs (active low), false for active high.
         * @param highActive True to invert the high-side gate outputs, false for active high.
         * @return true if the polarity was set successfully.
         */
        bool setOutputPolarity(bool lowActive, bool highActive) noexcept;

        /**
         * @brief Configure the overcurrent protection blanking time for UVW phases.
         * 
         * Sets the blanking time for overcurrent protection to filter out transient spikes during switching events.
         * 
         * @param lowSideTime Blanking time for the low side of UVW phases:
         *                    0: OFF, 1: 0.25μs, 2: 0.5μs, 3: 1μs, 4: 2μs, 5: 4μs, 6: 6μs, 7: 8μs
         * @param highSideTime Blanking time for the high side of UVW phases (same range as lowSideTime)
         * @return true if successfully configured
         */
        bool setOvercurrentBlankingUVW(uint8_t lowSideTime, uint8_t highSideTime) noexcept;

        /**
         * @brief Configure the overcurrent protection blanking time for Y2 phase.
         * 
         * @param lowSideTime Blanking time for the low side of Y2 phase (0-7)
         * @param highSideTime Blanking time for the high side of Y2 phase (0-7)
         * @return true if successfully configured
         */
        bool setOvercurrentBlankingY2(uint8_t lowSideTime, uint8_t highSideTime) noexcept;

        /**
         * @brief Configure the overcurrent protection deglitch time for UVW phases.
         * 
         * Sets how long an overcurrent condition must persist before triggering protection.
         * 
         * @param lowSideTime Deglitch time for the low side of UVW phases:
         *                    0: OFF, 1: 0.25μs, 2: 0.5μs, 3: 1μs, 4: 2μs, 5: 4μs, 6: 6μs, 7: 8μs
         * @param highSideTime Deglitch time for the high side of UVW phases (same range)
         * @return true if successfully configured
         */
        bool setOvercurrentDeglitchUVW(uint8_t lowSideTime, uint8_t highSideTime) noexcept;

        /**
         * @brief Configure the overcurrent protection deglitch time for Y2 phase.
         * 
         * @param lowSideTime Deglitch time for the low side of Y2 phase (0-7)
         * @param highSideTime Deglitch time for the high side of Y2 phase (0-7)
         * @return true if successfully configured
         */
        bool setOvercurrentDeglitchY2(uint8_t lowSideTime, uint8_t highSideTime) noexcept;

        /**
         * @brief Enable or disable VDS monitoring for overcurrent protection on UVW low side.
         * 
         * @param enable True to enable VDS measurement for overcurrent protection
         * @return true if successfully configured
         */
        bool enableVdsMonitoringUVWLow(bool enable) noexcept;

        /**
         * @brief Enable or disable VDS monitoring for overcurrent protection on Y2 low side.
         * 
         * @param enable True to enable VDS measurement for overcurrent protection
         * @return true if successfully configured
         */
        bool enableVdsMonitoringY2Low(bool enable) noexcept;

        /**
         * @brief Configure undervoltage protection settings.
         * 
         * @param supplyLevel Supply voltage (VS) protection level (0-16, 0 disables)
         * @param enableVdrv Enable driver voltage (VDRV) protection
         * @param enableBstUVW Enable bootstrap capacitor protection for UVW phases
         * @param enableBstY2 Enable bootstrap capacitor protection for Y2 phase
         * @return true if successfully configured
         */
        bool configureUndervoltageProtection(uint8_t supplyLevel, bool enableVdrv, 
                                            bool enableBstUVW, bool enableBstY2) noexcept;

        /**
         * @brief Configure gate-to-source short protection for UVW phases.
         * 
         * @param enableLowSideOn Enable protection for ON transition of low side
         * @param enableLowSideOff Enable protection for OFF transition of low side
         * @param enableHighSideOn Enable protection for ON transition of high side
         * @param enableHighSideOff Enable protection for OFF transition of high side
         * @return true if successfully configured
         */
        bool configureVgsShortProtectionUVW(bool enableLowSideOn, bool enableLowSideOff,
                                           bool enableHighSideOn, bool enableHighSideOff) noexcept;

        /**
         * @brief Configure gate-to-source short protection for Y2 phase.
         * 
         * @param enableLowSideOn Enable protection for ON transition of low side
         * @param enableLowSideOff Enable protection for OFF transition of low side
         * @param enableHighSideOn Enable protection for ON transition of high side
         * @param enableHighSideOff Enable protection for OFF transition of high side
         * @return true if successfully configured
         */
        bool configureVgsShortProtectionY2(bool enableLowSideOn, bool enableLowSideOff,
                                          bool enableHighSideOn, bool enableHighSideOff) noexcept;

        /**
         * @brief Set gate-to-source short protection blanking time.
         * 
         * @param uvwTime Blanking time for UVW phases:
         *               0: OFF, 1: 0.25μs, 2: 0.5μs, 3: 1μs
         * @param y2Time Blanking time for Y2 phase (same range)
         * @return true if successfully configured
         */
        bool setVgsShortBlankingTime(uint8_t uvwTime, uint8_t y2Time) noexcept;

        /**
         * @brief Set gate-to-source short protection deglitch time.
         * 
         * @param uvwTime Deglitch time for UVW phases:
         *               0: OFF, 1: 0.25μs, 2: 0.5μs, 3: 1μs, 4: 2μs, 5: 4μs, 6: 6μs, 7: 8μs
         * @param y2Time Deglitch time for Y2 phase (same range)
         * @return true if successfully configured
         */
        bool setVgsShortDeglitchTime(uint8_t uvwTime, uint8_t y2Time) noexcept;

    private:
        friend class TMC9660;
        explicit GateDriver(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } gateDriver{*this};

    //***************************************************************************
    //**                  SUBSYSTEM: Motor Control                            **//
    //***************************************************************************
    /**
     * @brief Subsystem for controlling motor motion
     */
    struct MotorControl {
        /**
         * @brief Stop any motor motion immediately.
         * 
         * Sends the "MST" (motor stop) command which should stop any ongoing movement or ramp. This does not disable the motor driver, 
         * but halts the velocity/position motion.
         * @return true if the stop command was acknowledged, false if not.
         */
        bool stop() noexcept;

        /**
         * @brief Set a target torque (current) for torque control mode.
         * 
         * Writing a target torque will engage the torque control loop (current control) of the TMC9660. The motor will attempt to output 
         * the requested current (torque) up to the maximum limits.
         * @param milliamps Desired motor current in mA (positive for forward torque, negative for reverse torque).
         * @return true if the target was sent successfully.
         */
        bool setTargetTorque(int16_t milliamps) noexcept;

        /**
         * @brief Set a target velocity for velocity control mode.
         * 
         * Writing a target velocity engages the velocity control loop. The motor will ramp to and hold the specified velocity if possible.
         * An appropriate sensor (like Hall or encoder) is required for closed-loop velocity mode on BLDC/stepper. For DC motors, an encoder is needed for velocity feedback.
         * @param velocity Target velocity value. The unit is in internal ticks (position counts per second) unless scaled via parameters.
         * @return true if the target velocity was sent successfully.
         */
        bool setTargetVelocity(int32_t velocity) noexcept;

        /**
         * @brief Set a target position for position control mode.
         * 
         * Writing a target position engages the position control loop. The motor will move to the specified position using the internal motion controller and ramp generator.
         * Requires a position sensor (incremental encoder or equivalent) to be configured.
         * @param position Target position in internal units (ticks).
         * @return true if the target position was sent successfully.
         */
        bool setTargetPosition(int32_t position) noexcept;

    private:
        friend class TMC9660;
        explicit MotorControl(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } motorControl{*this};

    //***************************************************************************
    //**                  SUBSYSTEM: Current Measurement                      **//
    //***************************************************************************
    /**
     * @brief Subsystem for configuring ADC-based current measurement
     */
    struct CurrentSensing {
        /**
         * @brief Configure the ADC shunt type for current measurement.
         * 
         * Sets the shunt resistor configuration used for motor current sensing.
         * 
         * @param shuntType Shunt configuration:
         *        0: INLINE_UVW - Inline shunts on all phases
         *        1: INLINE_VW - Inline shunts on V and W phases only
         *        2: INLINE_UW - Inline shunts on U and W phases only
         *        3: INLINE_UV - Inline shunts on U and V phases only
         *        4: BOTTOM_SHUNTS - Low-side (bottom) shunts configuration
         * @return true if configuration was successful
         */
        bool setShuntType(uint8_t shuntType) noexcept;

        /**
         * @brief Set the current scaling factor for accurate current measurement.
         * 
         * The scaling factor converts ADC readings to real-world current values in milliamps.
         * Calculated using: Factor = 1024 × 2.5 × 1000 / ((2^16-1) × GCSA × RShunt)
         * 
         * @param scalingFactor Scaling factor to convert ADC readings to mA.
         *        For RMS current: Factor ≈ 27.62 / (GCSA × RShunt)
         *        For Peak current: Factor ≈ 39.06 / (GCSA × RShunt)
         * @return true if set successfully
         */
        bool setScalingFactor(float scalingFactor) noexcept;

        /**
         * @brief Configure the ADC mapping and inversion for motor phases.
         * 
         * Maps ADC inputs to motor phases and sets inversion flags based on motor type
         * and current sensing configuration.
         * 
         * @param invertADC0 Set true to invert ADC0 readings
         * @param invertADC1 Set true to invert ADC1 readings
         * @param invertADC2 Set true to invert ADC2 readings
         * @param invertADC3 Set true to invert ADC3 readings (if used)
         * @return true if configuration was successful
         */
        bool configureADCInversion(bool invertADC0, bool invertADC1, bool invertADC2, bool invertADC3 = false) noexcept;

        /**
         * @brief Configure the current sense amplifier (CSA) gain.
         * 
         * Sets the gain for the internal current sense amplifiers.
         * 
         * @param gainADC0_1 Gain setting for ADC I0 and I1 channels (0-31)
         * @param gainADC2_3 Gain setting for ADC I2 and I3 channels (0-31)
         * @return true if gains were set successfully
         */
        bool setCSAGain(uint8_t gainADC0_1, uint8_t gainADC2_3) noexcept;

        /**
         * @brief Calibrate the ADC offsets for current measurement.
         * 
         * Initiates a calibration sequence for the ADCs. This should be done:
         * 1. With the motor stationary
         * 2. With the commutation mode set to off
         * 
         * @param waitForCompletion If true, wait until calibration is completed
         * @param timeoutMs Timeout in milliseconds if waiting for completion
         * @return true if calibration was started (and completed if waitForCompletion is true)
         */
        bool calibrateOffsets(bool waitForCompletion = false, uint32_t timeoutMs = 1000) noexcept;

        /**
         * @brief Check if ADC offset calibration has been completed.
         * 
         * @param[out] isCalibrated Set to true if calibration is complete
         * @return true if the status was read successfully
         */
        bool getCalibrationStatus(bool& isCalibrated) noexcept;

        /**
         * @brief Set individual ADC scaling factors to account for shunt resistor tolerances.
         * 
         * @param scaleADC0 Scaling factor for ADC I0 (default: 1.0)
         * @param scaleADC1 Scaling factor for ADC I1 (default: 1.0)
         * @param scaleADC2 Scaling factor for ADC I2 (default: 1.0)
         * @param scaleADC3 Scaling factor for ADC I3 (default: 1.0)
         * @return true if scales were set successfully
         */
        bool setADCScalingFactors(float scaleADC0 = 1.0f, float scaleADC1 = 1.0f, 
                                  float scaleADC2 = 1.0f, float scaleADC3 = 1.0f) noexcept;

        /**
         * @brief Read the raw ADC values for current measurement.
         * 
         * @param[out] adc0 Raw value from ADC I0
         * @param[out] adc1 Raw value from ADC I1
         * @param[out] adc2 Raw value from ADC I2
         * @param[out] adc3 Raw value from ADC I3
         * @return true if all values were read successfully
         */
        bool readRawValues(int16_t& adc0, int16_t& adc1, int16_t& adc2, int16_t& adc3) noexcept;

    private:
        friend class TMC9660;
        explicit CurrentSensing(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } currentSensing{*this};

    //***************************************************************************
    //**                  SUBSYSTEM: FOC Control                              **//
    //***************************************************************************
    /**
     * @brief Subsystem for Field-Oriented Control tuning
     */
    struct FOCControl {
        /**
         * @brief Set the tuning parameters for the current (torque/flux) PI controller.
         * @param p Gain P for current (torque) controller.
         * @param i Gain I for current (torque) controller.
         * @param separateFlux If true, assume separate flux PI controller and set its gains similarly (requires chip support).
         * @param fluxP (Optional) P gain for flux controller, if separateFlux is true.
         * @param fluxI (Optional) I gain for flux controller, if separateFlux is true.
         * @return true if the parameters were set successfully.
         */
        bool setCurrentLoopGains(uint16_t p, uint16_t i, bool separateFlux = false, uint16_t fluxP = 0, uint16_t fluxI = 0) noexcept;
        
        /**
         * @brief Set the tuning parameters for the velocity PI controller.
         * @param p Gain P for velocity controller.
         * @param i Gain I for velocity controller.
         * @return true if the parameters were set successfully.
         */
        bool setVelocityLoopGains(uint16_t p, uint16_t i) noexcept;

        /**
         * @brief Full PI tuning for the position loop.
         * @param p  Proportional gain.
         * @param i  Integral gain.
         */
        bool setPositionLoopGains(uint16_t p, uint16_t i) noexcept;   ///< POSITION_P / POSITION_I :contentReference[oaicite:1]{index=1}

        /**
         * @brief Normalisation (bit-shift) for velocity PI terms.
         * @param pShift  0–3 ⇒ {NO_SHIFT,8,16,24 bit}.
         * @param iShift  0–3 ⇒ {8,16,24,32 bit}.
         */
        bool setVelocityNormalization(uint8_t pShift,
                                    uint8_t iShift) noexcept;       ///< VELOCITY_NORM_P / VELOCITY_NORM_I :contentReference[oaicite:2]{index=2}

        /**
         * @brief Normalisation (bit-shift) for position PI terms.
         */
        bool setPositionNormalization(uint8_t pShift,
                                    uint8_t iShift) noexcept;       ///< POSITION_NORM_P / POSITION_NORM_I :contentReference[oaicite:3]{index=3}

        /**
         * @brief Set the tuning parameters for the position control loop (P controller).
         * @param p Gain P for position controller.
         * @return true if successfully set.
         */
        bool setPositionGain(uint16_t p) noexcept;

    private:
        friend class TMC9660;
        explicit FOCControl(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } focControl{*this};

    //***************************************************************************
    //**                  SUBSYSTEM: Motion Ramp                               **//
    //***************************************************************************
    /**
     * @brief Hardware 8-segment acceleration/dec-acc profile controller.
     */
    struct Ramp {
        /**
         * @brief Enable or disable the ramp generator block.
         */
        bool enable(bool on) noexcept;                             ///< RAMP_ENABLE :contentReference[oaicite:4]{index=4}

        /**
         * @brief Set acceleration segments A1, A2, Amax (µ units/s²).
         */
        bool setAcceleration(uint32_t a1,
                             uint32_t a2,
                             uint32_t aMax) noexcept;             ///< RAMP_A1/A2/AMAX :contentReference[oaicite:5]{index=5}

        /**
         * @brief Set deceleration segments D1, D2, Dmax (µ units/s²).
         */
        bool setDeceleration(uint32_t d1,
                             uint32_t d2,
                             uint32_t dMax) noexcept;             ///< RAMP_D1/D2/DMAX :contentReference[oaicite:6]{index=6}

        /**
         * @brief Configure velocity thresholds and limits.
         */
        bool setVelocities(uint32_t vStart,
                           uint32_t vStop,
                           uint32_t v1,
                           uint32_t v2,
                           uint32_t vMax) noexcept;               ///< RAMP_V* set :contentReference[oaicite:7]{index=7}

        /**
         * @brief Timing constraints at Vmax and between moves.
         */
        bool setTiming(uint16_t tVmaxCycles,
                       uint16_t tZeroWaitCycles) noexcept;        ///< RAMP_TVMAX / RAMP_TZEROWAIT :contentReference[oaicite:8]{index=8}

        /**
         * @brief Enable hardware feed-forward terms.
         * @param accelGain  ACCELERATION_FF_GAIN (0…65535)
         * @param accelShift ACCELERATION_FF_SHIFT enum 0…6
         * @param enableVelFF Enable the VELOCITY_FEEDFORWARD feature.
         */
        bool enableFeedForward(uint16_t accelGain,
                               uint8_t  accelShift,
                               bool     enableVelFF) noexcept;     ///< ACCELERATION_FF_* & VELOCITY_FEEDFORWARD_ENABLE :contentReference[oaicite:9]{index=9}

        /**
         * @brief Direct-velocity mode instead of classic PI velocity loop.
         */
        bool setDirectVelocityMode(bool enable) noexcept;          ///< DIRECT_VELOCITY_MODE :contentReference[oaicite:10]{index=10}

    private:
        friend class TMC9660;
        explicit Ramp(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } ramp{*this};

    //***************************************************************************
    //**                  SUBSYSTEM: Sensors                                  **//
    //***************************************************************************
    /**
     * @brief Subsystem for feedback sensor configuration
     */
    struct FeedbackSense {
        /**
         * @brief Select which sensor supplies the velocity loop.
         *        SAME_AS_COMMUTATION | DIGITAL_HALL | ABN1 | ABN2 | SPI.
         */
        bool selectVelocitySensor(uint8_t sel) noexcept;               ///< VELOCITY_SENSOR_SELECTION :contentReference[oaicite:14]{index=14}

        /**
         * @brief Select which sensor supplies the position loop.
         *        SAME_AS_COMMUTATION | DIGITAL_HALL | ABN1 | ABN2 | SPI.
         */
        bool selectPositionSensor(uint8_t sel) noexcept;               ///< POSITION_SENSOR_SELECTION :contentReference[oaicite:15]{index=15}

        /**
         * @brief Configure digital Hall sensors for BLDC commutation.
         * 
         * This enables Hall sensor inputs as the feedback for commutation. Typically used with CommutationMode::FOC_HALL.
         * @param sectorOffset Hall sensor 60-degree/sector offset (0-5):
         *                     0: 0°, 1: 60°, 2: 120°, 3: 180°, 4: 240°, 5: 300°
         *                     This combines both the 120° order offset and 180° polarity offset.
         * @param inverted If true, invert the interpretation of hall sensor signals.
         * @param enableExtrapolation If true, enable hall extrapolation for higher resolution position signal.
         * @param filterLength Digital filter length (0-255) for hall sensor inputs.
         * @return true if Hall sensor feedback is configured successfully.
         */
        bool configureHall(uint8_t sectorOffset = 0, bool inverted = false, 
                           bool enableExtrapolation = false, uint8_t filterLength = 0) noexcept;

        /**
         * @brief Set Hall sensor position offsets for improved accuracy.
         * 
         * Compensates for Hall sensor mounting tolerances by setting precise electrical angle offsets.
         * 
         * @param offset0 Offset for 0° Hall position (-32768 to 32767)
         * @param offset60 Offset for 60° Hall position (-32768 to 32767)
         * @param offset120 Offset for 120° Hall position (-32768 to 32767)
         * @param offset180 Offset for 180° Hall position (-32768 to 32767)
         * @param offset240 Offset for 240° Hall position (-32768 to 32767)
         * @param offset300 Offset for 300° Hall position (-32768 to 32767)
         * @param globalOffset Additional global offset applied to all positions (-32768 to 32767)
         * @return true if Hall position offsets were set successfully.
         */
        bool setHallPositionOffsets(int16_t offset0 = 0, int16_t offset60 = 10922, int16_t offset120 = 21845,
                                   int16_t offset180 = -32768, int16_t offset240 = -21846, int16_t offset300 = -10923,
                                   int16_t globalOffset = 0) noexcept;

        /**
         * @brief Configure an ABN incremental encoder for feedback.
         * 
         * Sets up an incremental quadrature encoder with optional index (N) channel for position and velocity feedback.
         * @param countsPerRev Encoder resolution (counts per revolution, 0-16777215).
         * @param inverted If true, invert the encoder direction.
         * @param nChannelInverted If true, invert the N-channel signal (active low instead of active high).
         * @return true if encoder parameters were set successfully.
         */
        bool configureABNEncoder(uint32_t countsPerRev, bool inverted = false, bool nChannelInverted = false) noexcept;

        /**
         * @brief Configure ABN encoder initialization method.
         * 
         * Sets the method used to align the ABN encoder with the rotor's absolute position.
         * 
         * @param initMethod Initialization method:
         *                   0: FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING - Forces rotor to phi_e zero with active swing
         *                   1: FORCED_PHI_E_90_ZERO - Forces rotor to 90° then 0° positions
         *                   2: USE_HALL - Uses Hall sensor transitions to align encoder
         *                   3: USE_N_CHANNEL_OFFSET - Uses N-channel with offset to align encoder
         * @param initDelay Delay in milliseconds to wait for mechanical oscillations to stop (1000-10000)
         * @param initVelocity Velocity used during N-channel initialization (-200000 to 200000)
         * @param nChannelOffset Offset between phi_e zero and encoder index pulse position (-32768 to 32767)
         * @return true if ABN initialization parameters were set successfully.
         */
        bool configureABNInitialization(uint8_t initMethod = 0, uint16_t initDelay = 1000,
                                       int32_t initVelocity = 5, int16_t nChannelOffset = 0) noexcept;

        /**
         * @brief Configure N-channel filtering for ABN encoder.
         * 
         * Sets up filtering for the N-channel (index pulse) to handle imprecise encoders.
         * 
         * @param filterMode N-channel filtering mode:
         *                   0: FILTERING_OFF - No filtering
         *                   1: N_EVENT_ON_A_HIGH_B_HIGH - N event only when A and B are high
         *                   2: N_EVENT_ON_A_HIGH_B_LOW - N event only when A is high and B is low
         *                   3: N_EVENT_ON_A_LOW_B_HIGH - N event only when A is low and B is high
         *                   4: N_EVENT_ON_A_LOW_B_LOW - N event only when A and B are low
         * @param clearOnNextNull If true, clear position counter on next N-channel event.
         * @return true if N-channel settings were applied successfully.
         */
        bool configureABNNChannel(uint8_t filterMode = 0, bool clearOnNextNull = false) noexcept;

        /**
         * @brief Configure a SPI-based encoder for feedback.
         * 
         * Sets up a digital SPI encoder (e.g., absolute magnetic encoder) for position feedback.
         * 
         * @param cmdSize Size of SPI transfer frame (1-16 bytes).
         * @param csSettleTimeNs CS settle time in nanoseconds (0-6375).
         * @param csIdleTimeUs CS idle time between frames in microseconds (0-102).
         * @return true if configured successfully.
         */
        bool configureSPIEncoder(uint8_t cmdSize, uint16_t csSettleTimeNs = 0, uint8_t csIdleTimeUs = 0) noexcept;

        /**
         * @brief Configure SPI encoder data format and processing.
         * 
         * Sets up how the position data is extracted from the SPI encoder response.
         * 
         * @param positionMask Bit mask to extract position from SPI response.
         * @param positionShift Right shift value to apply to position counter.
         * @param invertDirection If true, invert the direction of the SPI encoder.
         * @return true if configuration was successful.
         */
        bool configureSPIEncoderDataFormat(uint32_t positionMask, uint8_t positionShift = 0, bool invertDirection = false) noexcept;

        /**
         * @brief Set up SPI encoder request data for continuous transfer mode.
         * 
         * Sets the data to be sent to the SPI encoder during position acquisition.
         * 
         * @param requestData Array of data bytes to send to the encoder.
         * @param size Size of the request data (1-16 bytes).
         * @return true if transfer data was set successfully.
         */
        bool setSPIEncoderRequestData(const uint8_t* requestData, uint8_t size) noexcept;

        /**
         * @brief Configure SPI encoder initialization method.
         * 
         * Sets how the SPI encoder is initialized for commutation.
         * 
         * @param initMethod Initialization method (0-3, similar to ABN initialization methods).
         * @param offset Manual offset value if using offset-based initialization.
         * @return true if initialization method was set successfully.
         */
        bool configureSPIEncoderInitialization(uint8_t initMethod, int16_t offset = 0) noexcept;

        /**
         * @brief Enable or disable SPI encoder lookup table correction.
         * 
         * Enables the lookup table-based correction for encoder nonlinearity.
         * 
         * @param enable If true, enable LUT correction.
         * @param shiftFactor Common shift factor for all LUT entries.
         * @return true if LUT settings were applied successfully.
         */
        bool setSPIEncoderLUTCorrection(bool enable, int8_t shiftFactor = 0) noexcept;

        /**
         * @brief Upload a single entry to the SPI encoder correction lookup table.
         * 
         * @param index Index in the LUT (0-255).
         * @param value Correction value (-128 to 127).
         * @return true if the entry was uploaded successfully.
         */
        bool uploadSPIEncoderLUTEntry(uint8_t index, int8_t value) noexcept;

    private:
        friend class TMC9660;
        explicit FeedbackSense(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } feedbackSense{*this};

    //***************************************************************************
    //**                  SUBSYSTEM: Stop / Event                              **//
    //***************************************************************************
    /**
     * @brief Configure automatic stop/latch behaviour for deviation, switches.
     */
    struct StopEvents {
        /**
         * @brief Stop when ramp target deviates from actual > thresholds.
         */
        bool enableDeviationStop(uint32_t maxVelError,
                                 uint32_t maxPosError,
                                 bool     softStop=true) noexcept; ///< STOP_ON_*_DEVIATION + EVENT_STOP_SETTINGS :contentReference[oaicite:11]{index=11}

        /**
         * @brief Configure reference / limit-switch inputs.
         * @param mask  Bit-mask 0…7 ; see REFERENCE_SWITCH_ENABLE.
         * @param invertL,R,H    invert individual polarities.
         * @param swapLR         swap left/right wiring.
         */
        bool configureReferenceSwitches(uint8_t mask,
                                        bool invertL,
                                        bool invertR,
                                        bool invertH,
                                        bool swapLR) noexcept;    ///< REFERENCE_SWITCH_* :contentReference[oaicite:12]{index=12}

        /**
         * @brief Read and clear the latched position from a switch event.
         */
        bool getAndClearLatchedPosition(int32_t& pos) noexcept;    ///< LATCH_POSITION / RAMPER_LATCHED :contentReference[oaicite:13]{index=13}

    private:
        friend class TMC9660;
        explicit StopEvents(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } stopEvents{*this};

    //***************************************************************************
    //**                  SUBSYSTEM: Protection                               **//
    //***************************************************************************
    /**
     * @brief Subsystem for motor protection features
     */
    struct Protection {
        /**
         * @brief Configure overvoltage and undervoltage protection thresholds.
         * @param overVoltThreshold Over-voltage warning threshold in units of 0.1V.
         * @param underVoltThreshold Under-voltage warning threshold in units of 0.1V.
         * @return true if thresholds were set successfully.
         */
        bool configureVoltage(uint16_t overVoltThreshold, uint16_t underVoltThreshold) noexcept;

        /**
         * @brief Configure over-temperature protection thresholds.
         * 
         * The TMC9660 has an internal temperature sensor and supports an external analog temperature sensor.
         * This function sets warning and shutdown thresholds for the internal sensor.
         * @param warningDegC Warning threshold in °C for chip temperature.
         * @param shutdownDegC Shutdown (fault) threshold in °C for chip temperature.
         * @return true if set successfully.
         */
        bool configureTemperature(float warningDegC, float shutdownDegC) noexcept;

        /**
         * @brief Enable or disable overcurrent protection on the driver outputs.
         * 
         * This controls the internal gate driver overcurrent detection (e.g., comparators) for various FETs.
         * @param enabled True to enable overcurrent protection (shut down drivers on overcurrent), false to disable.
         * @return true if command was sent successfully.
         */
        bool setOvercurrentEnabled(bool enabled) noexcept;

        /**
         * @brief Configure the two I²t monitoring windows for motor current.
         * @param timeConstant1_ms Time constant for first window in milliseconds.
         * @param continuousCurrent1_A Continuous current limit for first window in amps.
         * @param timeConstant2_ms Time constant for second window in milliseconds.
         * @param continuousCurrent2_A Continuous current limit for second window in amps.
         * @return true if configuration was successful.
         */
        bool configureI2t(uint16_t timeConstant1_ms, float continuousCurrent1_A,
                          uint16_t timeConstant2_ms, float continuousCurrent2_A) noexcept;
        
        /**
         * @brief Reset the integrated I²t sum accumulators.
         * @return true if reset was successful.
         */
        bool resetI2tState() noexcept;

    private:
        friend class TMC9660;
        explicit Protection(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } protection{*this};

    //***************************************************************************
    //**                  SUBSYSTEM: Script                                   **//
    //***************************************************************************
    /**
     * @brief Subsystem for TMCL script execution control
     */
    struct Script {
        /**
         * @brief Upload a TMCL script to the TMC9660's internal memory.
         * 
         * This enters download mode, writes a series of instructions to the device memory, and exits download mode.
         * The script will typically run on device startup or when triggered.
         * @param scriptData Vector of 32-bit instructions representing the TMCL script.
         * @return true if the script was uploaded successfully.
         */
        bool upload(const std::vector<uint32_t>& scriptData) noexcept;

        /**
         * @brief Start or restart execution of the stored script.
         * @param address The address from which to start execution (usually 0 for beginning of script).
         * @return true if the command to start the script was sent.
         */
        bool start(uint16_t address = 0) noexcept;
        
        /**
         * @brief Stop execution of the running script.
         * @return true if the stop command was sent successfully.
         */
        bool stop() noexcept;

    private:
        friend class TMC9660;
        explicit Script(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } script{*this};

    //***************************************************************************
    //**                  SUBSYSTEM: Debug                                    **//
    //***************************************************************************
    /**
     * @brief Subsystem for debug and data logging features
     */
    struct Debug {
        /**
         * @brief Initialize and configure the RAMDebug feature (data logging).
         * 
         * This sends a command to initialize the RAM debug system and reset any previous configurations.
         * @param sampleCount Number of samples to collect in the buffer.
         * @return true if the RAM debug was initialized properly.
         */
        bool initRAMDebug(uint32_t sampleCount) noexcept;

        /**
         * @brief Start capturing data using RAMDebug.
         * @return true if the capture started successfully.
         */
        bool startRAMDebugCapture() noexcept;

        /**
         * @brief Read a captured sample from RAMDebug buffer.
         * @param index Sample index to read (0-based).
         * @param[out] data Variable to store the 32-bit sample data.
         * @return true if the data was read successfully.
         */
        bool readRAMDebugData(uint32_t index, uint32_t& data) noexcept;

        /**
         * @brief Get the current state of the RAM debug engine.
         * @param[out] isRunning Will be set to true if capture is ongoing, false if stopped or idle.
         * @return true if the status was retrieved successfully.
         */
        bool getRAMDebugStatus(bool& isRunning) noexcept;

    private:
        friend class TMC9660;
        explicit Debug(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } debug{*this};
    
    //***************************************************************************
    //**                  SUBSYSTEM: Telemetry & Status                        **//
    //***************************************************************************
    /**
     * @brief Subsystem for reading various telemetry and status information from the driver.
     */
    struct Telemetry {
        /**
         * @brief Read the internal chip temperature.
         * @return Chip temperature in degrees Celsius.
         * 
         * If reading the temperature fails, this returns a negative value (e.g. -273) to indicate an error.
         * The temperature is computed from the raw sensor reading using the formula: T(°C) = value * 0.01615 - 268.15.
         */
        float getChipTemperature() noexcept;

        /**
         * @brief Read the current motor current (torque current).
         * @return Motor current in milliamps (mA). Returns 0 if unable to read.
         */
        int16_t getMotorCurrent() noexcept;

        /**
         * @brief Read the current supply (bus) voltage.
         * @return Supply voltage in volts. Returns a negative value if unable to read.
         */
        float getSupplyVoltage() noexcept;

        /**
         * @brief Read the measured actual velocity of the motor.
         * @return The actual velocity in internal units (typically encoder counts per second). Returns 0 if not available.
         */
        int32_t getActualVelocity() noexcept;

        /**
         * @brief Read the measured actual position of the motor.
         * @return The actual position in internal units (encoder counts). Returns 0 if not available.
         */
        int32_t getActualPosition() noexcept;

    private:
        friend class TMC9660;
        explicit Telemetry(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } telemetry{*this};

    //***************************************************************************
    //**                      SUBSYSTEM: Brake Chopper                         **//
    //***************************************************************************
    /**
     * @brief Subsystem controlling the brake chopper and mechanical brake features.
     */
    struct Brake {
        /**
         * @brief Enable or disable the brake chopper functionality.
         * @param enable True to enable the brake chopper, false to disable it.
         * @return True if the command was sent and acknowledged.
         */
        bool enableChopper(bool enable) noexcept;

        /**
         * @brief Set the overvoltage threshold for the brake chopper.
         * @param voltage Threshold voltage in volts (5.0 to 100.0 V).
         * @return True if the parameter was written successfully.
         */
        bool setVoltageLimit(float voltage) noexcept;

        /**
         * @brief Set the hysteresis for the brake chopper threshold.
         * @param voltage Hysteresis in volts (0.0 to 5.0 V).
         * @return True if the parameter was written successfully.
         */
        bool setHysteresis(float voltage) noexcept;

        /**
         * @brief Trigger a release of the mechanical brake.
         * @return True if the command was sent successfully.
         */
        bool release() noexcept;

        /**
         * @brief Engage (lock) the mechanical brake.
         * @return True if the command was sent successfully.
         */
        bool engage() noexcept;

        /**
         * @brief Set the PWM duty cycle for releasing the brake.
         * @param percent Duty cycle (0 to 99%).
         * @return True if the parameter was written successfully.
         */
        bool setReleasingDutyCycle(uint8_t percent) noexcept;

        /**
         * @brief Set the PWM duty cycle for holding the brake released.
         * @param percent Duty cycle (0 to 99%).
         * @return True if the parameter was written successfully.
         */
        bool setHoldingDutyCycle(uint8_t percent) noexcept;

        /**
         * @brief Set the duration of the brake release initial phase.
         * @param milliseconds Duration in ms (0 to 65535).
         * @return True if the parameter was written successfully.
         */
        bool setReleasingDuration(uint16_t milliseconds) noexcept;

        /**
         * @brief Invert or normalize the brake output signal polarity.
         * @param invert True to invert the brake output, false for normal.
         * @return True if the parameter was written successfully.
         */
        bool invertOutput(bool invert) noexcept;

    private:
        friend class TMC9660;
        explicit Brake(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } brake{*this};

    //***************************************************************************
    //**                   SUBSYSTEM: I²t Overload Protection                  **//
    //***************************************************************************
    /**
     * @brief Subsystem for motor overload (I²t) protection configuration.
     */
    struct IIT {
        /**
         * @brief Configure the two I²t monitoring windows for motor current.
         */
        bool configure(uint16_t timeConstant1_ms, float continuousCurrent1_A,
                       uint16_t timeConstant2_ms, float continuousCurrent2_A) noexcept;
        /**
         * @brief Reset the integrated I²t sum accumulators.
         */
        bool resetIntegralState() noexcept;
    private:
        friend class TMC9660;
        explicit IIT(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } iit{*this};

    //***************************************************************************
    //**              SUBSYSTEM: Step/Dir Input Extrapolation                  **//
    //***************************************************************************
    /**
     * @brief Subsystem for Step/Dir input interface configuration.
     */
    struct StepDir {
        /**
         * @brief Enable/disable the STEP/DIR hardware interface.
         */
        bool enableInterface(bool on) noexcept;                        ///< BOOT_CONFIG.STEPDIR_ENABLE (datasheet)

        /**
         * @brief Configure micro-step resolution (full-step divided by 1…256).
         */
        bool setMicrostepResolution(uint16_t µSteps) noexcept;         ///< MICROSTEP_RESOLUTION (Parameter-mode DS)

        bool enableExtrapolation(bool enable) noexcept;
        bool setSignalTimeout(uint16_t timeout_ms) noexcept;
        bool setMaxExtrapolationVelocity(uint32_t eRPM) noexcept;
    private:
        friend class TMC9660;
        explicit StepDir(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } stepDir{*this};


    //***************************************************************************
    //**                SUBSYSTEM: FLASH STORAGE                             **//
    //***************************************************************************

    /**
     * @brief Persist all RAM parameters to on-chip flash.
     *
     *      Wraps PARAMETER_STORAGE_* commands so your application can
     *      survive resets, matching the “Save to NVM” button in GUI-tools.
     */
    struct NvmStorage {
        bool storeToFlash()            noexcept;   ///< PARAM_SAVE_ALL
        bool recallFromFlash()         noexcept;   ///< PARAM_RESTORE_ALL
        bool eraseFlashBank(uint8_t n) noexcept;   ///< PARAM_ERASE_BANK
    } nvm{};


    //***************************************************************************
    //**                SUBSYSTEM: Heartbeat (Watchdog)                        **//
    //***************************************************************************
    /**
     * @brief Subsystem for the communication heartbeat monitor (watchdog).
     */
    struct Heartbeat {
        bool configure(HeartbeatMode mode, uint32_t timeout_ms) noexcept;
    private:
        friend class TMC9660;
        explicit Heartbeat(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } heartbeat{*this};

    //***************************************************************************
    //**        SUBSYSTEM: General-purpose GPIO (Digital/Analog I/O)           **//
    //***************************************************************************
    struct GPIO {
        bool setMode(uint8_t pin, bool output, bool pullEnable = false, bool pullUp = true) noexcept;
        bool writePin(uint8_t pin, bool value) noexcept;
        bool readDigital(uint8_t pin, bool &value) noexcept;
        bool readAnalog(uint8_t pin, uint16_t &value) noexcept;
    private:
        friend class TMC9660;
        explicit GPIO(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } gpio{*this};

    //***************************************************************************
    //**               SUBSYSTEM: Power Management                          **//
    //***************************************************************************
    struct Power {
        bool enableWakePin(bool enable) noexcept;
        bool enterPowerDown(PowerDownPeriod period) noexcept;
    private:
        friend class TMC9660;
        explicit Power(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } power{*this};

    //***************************************************************************
    //**                SUBSYSTEM: Fault Handling and Retry                    **//
    //***************************************************************************
    struct Fault {
        bool configure(FaultRetryAction retryAction, FaultFinalAction finalAction, uint8_t retries) noexcept;
    private:
        friend class TMC9660;
        explicit Fault(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } fault{*this};

    //==================================================
    //==================================================
private:
    TMC9660CommInterface& comm_;  ///< Communication interface (transport) for sending/receiving data.
    uint8_t address_;            ///< Module address (0-127). Used primarily for UART multi-drop addressing.

    struct Datagram {
        uint8_t  op{};
        uint16_t type{};
        uint8_t  motor{};
        uint32_t value{};

        void toSpi(std::span<uint8_t, 8> out) const noexcept {
            out[0] = op;
            out[1] = static_cast<uint8_t>(type >> 8);
            out[2] = static_cast<uint8_t>(type);
            out[3] = motor;
            out[4] = static_cast<uint8_t>(value >> 24);
            out[5] = static_cast<uint8_t>(value >> 16);
            out[6] = static_cast<uint8_t>(value >> 8);
            out[7] = static_cast<uint8_t>(value);
        }

        static constexpr uint8_t checksum(const uint8_t* bytes, size_t n) noexcept {
            uint8_t sum = 0;
            for (size_t i = 0; i < n; ++i)
                sum += bytes[i];
            return sum;
        }

        void toUart(uint8_t addr, std::span<uint8_t, 9> out) const noexcept {
            out[0] = addr & 0x7Fu; // sync bit cleared
            out[1] = op;
            out[2] = static_cast<uint8_t>(type >> 8);
            out[3] = static_cast<uint8_t>(type);
            out[4] = motor;
            out[5] = static_cast<uint8_t>(value >> 24);
            out[6] = static_cast<uint8_t>(value >> 16);
            out[7] = static_cast<uint8_t>(value >> 8);
            out[8] = static_cast<uint8_t>(value);
            out[8] = checksum(out.data(), 8);
        }
    };

    // Helper: pack a TMCL command into 8 bytes and send/receive.
    bool sendCommand(uint8_t opcode, uint16_t type, uint8_t motor, uint32_t value, uint32_t* reply = nullptr) noexcept;
};
