#pragma once

#include <cstdint>
#include <cstdlib>
#include <array>
#include <vector>
#include <span>

#include "parameter_mode/tmc9660_param_mode_tmcl.hpp"
#include "TMC9660CommInterface.hpp"

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
         * Note that the BLDC option also covers PMSM motors since they use the same three-phase commutation scheme.
         * @param type MotorType (DC, BLDC/PMSM, STEPPER).
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
         * @brief Configure the commutation mode for the motor.
         * 
         * This sets how the motor is driven: e.g., open-loop or closed-loop FOC with various sensor feedback options.
         * Typically used for BLDC or stepper motors. For DC motors, commutation modes are not applicable except for
         * sensor feedback in velocity/position control modes.
         * 
         * @param mode A CommutationMode value defining the motor control strategy:
         *             - SYSTEM_OFF: Motor disabled (default state after power-on/reset)
         *             - SYSTEM_OFF_LOW_SIDE_FETS_ON: All low-side FETs ON (brake)
         *             - SYSTEM_OFF_HIGH_SIDE_FETS_ON: All high-side FETs ON
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
         * @brief Set the output voltage limit for the FOC controller.
         * 
         * This parameter limits the maximum Uq/ Ud output of the FOC controller (PID output circular limiter).
         * 
         * @param limit Output voltage limit (0 ... 32767, default 8000).
         * @return true if the parameter was set successfully.
         */
        bool setOutputVoltageLimit(uint16_t limit) noexcept;

        /**
         * @brief Set the maximum allowed motor current (torque limit).
         * 
         * This sets the MAX_TORQUE parameter which limits the peak current/torque that the controller will deliver to the motor.
         * @param milliamps Maximum current in milliamps.
         * @return true on success, false on error.
         * 
         * @note This value can be temporarily exceeded marginally due to the operation of the current regulator.
         */
        bool setMaxTorqueCurrent(uint16_t milliamps) noexcept;

        /**
         * @brief Set the maximum allowed flux current for BLDC/stepper motors.
         * 
         * This sets the MAX_FLUX parameter which limits the flux-producing current component.
         * Important for field-weakening operation and stepper motor control.
         * @param milliamps Maximum flux current in milliamps.
         * @return true on success, false on error.
         * 
         * @note This value can be temporarily exceeded marginally due to the operation of the current regulator.
         */
        bool setMaxFluxCurrent(uint16_t milliamps) noexcept;
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
        bool setPWMSwitchingScheme(tmc9660::tmcl::PwmSwitchingScheme scheme) noexcept;
            
        /**
         * @brief Configure PWM behavior when the motor is idle (System Off mode).
         * 
         * Controls whether motor phases are driven or high-impedance when commutation is disabled.
         * 
         * @param pwmOffWhenIdle True: high-Z/disconnected phases, False: all phases driven equally
         * @return true if the parameter was set successfully.
         * 
         * @note Selection guide by motor type:
         * | Motor type   | Typical goal when idle         | Best-fit setting     | Why it's usually preferred                   |
         * | ------------ | ------------------------------ | -------------------- | -------------------------------------------- |
         * | **Stepper**  | Hold a fixed shaft position    | **PWM on**          | • Energizing both windings gives full static |
         * |              | (avoid back-driving)           | (all coils          | torque                                       |
         * |              |                                | energized)          | • Prevents the load or gravity from shifting |
         * |              |                                |                      | the rotor                                    |
         * |              |                                |                      | • Draws continuous current, heating the      |
         * |              |                                |                      | motor and wasting power—turn it off if you   |
         * |              |                                |                      | don't need holding torque                    |
         * | ------------ | ------------------------------ | -------------------- | -------------------------------------------- |
         * | **BLDC**     | Usually want the rotor to      | **PWM off**         | • With the bridge inactive the stator        |
         * | **(3-phase)**| coast freely or be gently      | (high-Z)            | doesn't pull current, avoiding battery       |
         * |              | braked by external friction    |                      | drain and heating                            |
         * |              |                                |                      | • If you need static holding/braking torque  |
         * |              |                                |                      | (rare for sensorless BLDC), keep PWM on      |
         * |              |                                |                      | or switch to active braking instead          |
         * | ------------ | ------------------------------ | -------------------- | -------------------------------------------- |
         * | **Brushed**  | Let the armature coast; no     | **PWM off**         | • Disconnecting the H-bridge removes any     |
         * | **DC**       | holding torque exists anyway   | (high-Z)            | quiescent current                            |
         * |              |                                |                      | • Keeping PWM on shorts the terminals        |
         * |              |                                |                      | (dynamic braking) and stops the shaft        |
         * |              |                                |                      | quickly but wastes energy and can cause      |
         * |              |                                |                      | regen currents—use only if you need that     |
         * |              |                                |                      | fast passive brake                           |
         * 
         * There isn't a single "best" choice that fits every motor type or application—you're really deciding between:
         * 
         * * **High-Z (PWM off → `PWM_OFF_WHEN_MOTOR_IDLE`)**
         *   All phases are tri-stated, so the motor is electrically disconnected.
         * 
         * * **All-phases driven (PWM on → `PWM_ON_WHEN_MOTOR_IDLE`)**
         *   Every phase is held at the same potential, keeping the bridge active.
         * 
         * ### Quick decision tree
         * 
         * 1. **Do you need the shaft locked in place at idle?**
         *    *Yes →* **PWM on**.
         *    *No →* continue.
         * 
         * 2. **Is extra heat, quiescent current, or battery life a concern?**
         *    *Yes →* **PWM off** is safer.
         * 
         * 3. **Do you actually want "freewheel" coasting?**
         *    *Yes →* **PWM off** (high-Z).
         *    *No, I want dynamic braking →* leave PWM on **or** enable a dedicated brake parameter/mode.
         * 
         * **Rule of thumb**
         * 
         * * **Steppers**: keep coils powered only if you genuinely need holding torque; otherwise cut them to save power and heat.
         * * **BLDC & brushed DC**: default to PWM off; only energize at idle if you're deliberately braking or holding.
         * 
         * So, "most ideal" depends entirely on your idle behavior requirements: holding torque = **ON**, low power coast = **OFF**.
         */
        bool setIdleMotorPWMBehavior(tmc9660::tmcl::IdleMotorPwmBehavior pwmOffWhenIdle = tmc9660::tmcl::IdleMotorPwmBehavior::PWM_OFF_WHEN_MOTOR_IDLE) noexcept;

        // /**
        //  * @brief Configure velocity loop downsampling to adjust control loop frequency.
        //  * 
        //  * Slows down the velocity control loop relative to the PWM frequency by the specified factor.
        //  * 
        //  * @param divider Downsampling divider (1 = no downsampling, 2 = half frequency, etc.)
        //  * @return true if successfully configured.
        //  */
        // bool setVelocityLoopDownsampling(uint8_t divider) noexcept;

        // /**
        //  * @brief Configure position loop downsampling relative to velocity loop.
        //  * 
        //  * Further slows down the position control loop relative to the velocity loop by the specified factor.
        //  * 
        //  * @param divider Downsampling divider (1 = same as velocity loop, 2 = half frequency, etc.)
        //  * @return true if successfully configured.
        //  */
        // bool setPositionLoopDownsampling(uint8_t divider) noexcept;

        // /**
        //  * @brief Configure field-weakening (automatic flux reduction above a knee speed).
        //  *
        //  * @param startRPM   Electrical RPM where weakening starts.
        //  * @param slope      Weakening slope (dΨ/dω) as signed 16-bit value.
        //  * @param minFlux    Minimum allowed flux in mWb.
        //  * @return true on success.
        //  *
        //  * Registers: WEAKENING_FACTOR, FIELD_WEAKENING_ENABLE :contentReference[oaicite:0]{index=0}
        //  */
        // bool configureFieldWeakening(uint16_t startRPM,
        //                             int16_t  slope,
        //                             uint16_t minFlux) noexcept;

    private:
        friend class TMC9660;
        explicit MotorConfig(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } motorConfig{*this};

    //***************************************************************************
    //**                  SUBSYSTEM: Current Measurement                      **//
    //***************************************************************************
    /**
     * @brief Subsystem for configuring ADC-based current measurement
     */
    struct CurrentSensing {
        /**
         * @brief Set the ADC shunt type (Parameter 12: ADC_SHUNT_TYPE).
         * @param shuntType AdcShuntType enum value
         * @return true if successful
         */
        bool setShuntType(tmc9660::tmcl::AdcShuntType shuntType) noexcept;

        /**
         * @brief Get the ADC shunt type (Parameter 12: ADC_SHUNT_TYPE).
         * @param[out] shuntType AdcShuntType enum value
         * @return true if successful
         */
        bool getShuntType(tmc9660::tmcl::AdcShuntType& shuntType) noexcept;

        /**
         * @brief Read raw ADC values (Parameters 13-16: ADC_I0_RAW ... ADC_I3_RAW).
         * @param[out] adc0 Raw ADC I0
         * @param[out] adc1 Raw ADC I1
         * @param[out] adc2 Raw ADC I2
         * @param[out] adc3 Raw ADC I3
         * @return true if all values were read successfully
         */
        bool readRaw(int16_t& adc0, int16_t& adc1, int16_t& adc2, int16_t& adc3) noexcept;

        /**
         * @brief Set current sense amplifier gain (Parameters 17/18: CSA_GAIN_ADC_I0_TO_ADC_I2, CSA_GAIN_ADC_I3).
         * @param gain012 CsaGain enum value for ADC I0/I1/I2
         * @param gain3 CsaGain enum value for ADC I3
         * @return true if successful
         */
        bool setCSAGain(tmc9660::tmcl::CsaGain gain012, tmc9660::tmcl::CsaGain gain3) noexcept;

        /**
         * @brief Get current sense amplifier gain (Parameters 17/18: CSA_GAIN_ADC_I0_TO_ADC_I2, CSA_GAIN_ADC_I3).
         * @param[out] gain012 CsaGain enum value for ADC I0/I1/I2
         * @param[out] gain3 CsaGain enum value for ADC I3
         * @return true if successful
         */
        bool getCSAGain(tmc9660::tmcl::CsaGain& gain012, tmc9660::tmcl::CsaGain& gain3) noexcept;

        /**
         * @brief Set current sense amplifier filter (Parameters 19/20: CSA_FILTER_ADC_I0_TO_ADC_I2, CSA_FILTER_ADC_I3).
         * @param filter012 CsaFilter enum value for ADC I0/I1/I2
         * @param filter3 CsaFilter enum value for ADC I3
         * @return true if successful
         */
        bool setCSAFilter(tmc9660::tmcl::CsaFilter filter012, tmc9660::tmcl::CsaFilter filter3) noexcept;

        /**
         * @brief Get current sense amplifier filter (Parameters 19/20: CSA_FILTER_ADC_I0_TO_ADC_I2, CSA_FILTER_ADC_I3).
         * @param[out] filter012 CsaFilter enum value for ADC I0/I1/I2
         * @param[out] filter3 CsaFilter enum value for ADC I3
         * @return true if successful
         */
        bool getCSAFilter(tmc9660::tmcl::CsaFilter& filter012, tmc9660::tmcl::CsaFilter& filter3) noexcept;

        /**
         * @brief Set current scaling factor (Parameter 21: CURRENT_SCALING_FACTOR).
         * @param scalingFactor Scaling factor (1...65535)
         * @return true if successful
         */
        bool setScalingFactor(uint16_t scalingFactor) noexcept;

        /**
         * @brief Get current scaling factor (Parameter 21: CURRENT_SCALING_FACTOR).
         * @param[out] scalingFactor Scaling factor (1...65535)
         * @return true if successful
         */
        bool getScalingFactor(uint16_t& scalingFactor) noexcept;

        /**
         * @brief Set ADC mapping for each phase (Parameters 22-25: PHASE_UX1_ADC_MAPPING ... PHASE_Y2_ADC_MAPPING).
         * @param ux1 AdcMapping enum value for UX1
         * @param vx2 AdcMapping enum value for VX2
         * @param wy1 AdcMapping enum value for WY1
         * @param y2 AdcMapping enum value for Y2
         * @return true if all mappings were set successfully
         */
        bool setPhaseAdcMapping(tmc9660::tmcl::AdcMapping ux1, tmc9660::tmcl::AdcMapping vx2, 
                    tmc9660::tmcl::AdcMapping wy1, tmc9660::tmcl::AdcMapping y2) noexcept;

        /**
         * @brief Get ADC mapping for each phase (Parameters 22-25: PHASE_UX1_ADC_MAPPING ... PHASE_Y2_ADC_MAPPING).
         * @param[out] ux1 AdcMapping enum value for UX1
         * @param[out] vx2 AdcMapping enum value for VX2
         * @param[out] wy1 AdcMapping enum value for WY1
         * @param[out] y2 AdcMapping enum value for Y2
         * @return true if all mappings were retrieved successfully
         */
        bool getPhaseAdcMapping(tmc9660::tmcl::AdcMapping& ux1, tmc9660::tmcl::AdcMapping& vx2, 
                    tmc9660::tmcl::AdcMapping& wy1, tmc9660::tmcl::AdcMapping& y2) noexcept;

        /**
         * @brief Set individual ADC scaling factors (Parameters 26-29: ADC_I0_SCALE ... ADC_I3_SCALE).
         * @param scale0 Scaling factor for ADC I0 (1...32767)
         * @param scale1 Scaling factor for ADC I1 (1...32767)
         * @param scale2 Scaling factor for ADC I2 (1...32767)
         * @param scale3 Scaling factor for ADC I3 (1...32767)
         * @return true if all scales were set successfully
         */
        bool setScalingFactors(uint16_t scale0, uint16_t scale1, uint16_t scale2, uint16_t scale3) noexcept;

        /**
         * @brief Get individual ADC scaling factors (Parameters 26-29: ADC_I0_SCALE ... ADC_I3_SCALE).
         * @param[out] scale0 Scaling factor for ADC I0 (1...32767)
         * @param[out] scale1 Scaling factor for ADC I1 (1...32767)
         * @param[out] scale2 Scaling factor for ADC I2 (1...32767)
         * @param[out] scale3 Scaling factor for ADC I3 (1...32767)
         * @return true if all scales were retrieved successfully
         */
        bool getScalingFactors(uint16_t& scale0, uint16_t& scale1, uint16_t& scale2, uint16_t& scale3) noexcept;

        /**
         * @brief Set ADC inversion (Parameters 30-33: ADC_I0_INVERTED ... ADC_I3_INVERTED).
         * @param inv0 AdcInversion enum value for ADC I0
         * @param inv1 AdcInversion enum value for ADC I1
         * @param inv2 AdcInversion enum value for ADC I2
         * @param inv3 AdcInversion enum value for ADC I3
         * @return true if all inversion flags were set successfully
         */
        bool setInversion(tmc9660::tmcl::AdcInversion inv0, tmc9660::tmcl::AdcInversion inv1, 
                    tmc9660::tmcl::AdcInversion inv2, tmc9660::tmcl::AdcInversion inv3) noexcept;

        /**
         * @brief Get ADC inversion (Parameters 30-33: ADC_I0_INVERTED ... ADC_I3_INVERTED).
         * @param[out] inv0 AdcInversion enum value for ADC I0
         * @param[out] inv1 AdcInversion enum value for ADC I1
         * @param[out] inv2 AdcInversion enum value for ADC I2
         * @param[out] inv3 AdcInversion enum value for ADC I3
         * @return true if all inversion flags were retrieved successfully
         */
        bool getInversion(tmc9660::tmcl::AdcInversion& inv0, tmc9660::tmcl::AdcInversion& inv1, 
                    tmc9660::tmcl::AdcInversion& inv2, tmc9660::tmcl::AdcInversion& inv3) noexcept;

        /**
         * @brief Set ADC offset (Parameters 34-37: ADC_I0_OFFSET ... ADC_I3_OFFSET).
         * @param offset0 Offset for ADC I0 (-32768...32767)
         * @param offset1 Offset for ADC I1 (-32768...32767)
         * @param offset2 Offset for ADC I2 (-32768...32767)
         * @param offset3 Offset for ADC I3 (-32768...32767)
         * @return true if all offsets were set successfully
         */
        bool setOffsets(int16_t offset0, int16_t offset1, int16_t offset2, int16_t offset3) noexcept;

        /**
         * @brief Get ADC offset (Parameters 34-37: ADC_I0_OFFSET ... ADC_I3_OFFSET).
         * @param[out] offset0 Offset for ADC I0 (-32768...32767)
         * @param[out] offset1 Offset for ADC I1 (-32768...32767)
         * @param[out] offset2 Offset for ADC I2 (-32768...32767)
         * @param[out] offset3 Offset for ADC I3 (-32768...32767)
         * @return true if all offsets were retrieved successfully
         */
        bool getOffsets(int16_t& offset0, int16_t& offset1, int16_t& offset2, int16_t& offset3) noexcept;

        /**
         * @brief Read scaled and offset-compensated ADC values (Parameters 38-41: ADC_I0 ... ADC_I3).
         * @param[out] adc0 Scaled/offset ADC I0
         * @param[out] adc1 Scaled/offset ADC I1
         * @param[out] adc2 Scaled/offset ADC I2
         * @param[out] adc3 Scaled/offset ADC I3
         * @return true if all values were read successfully
         */
        bool readScaledAndOffset(int16_t& adc0, int16_t& adc1, int16_t& adc2, int16_t& adc3) noexcept;

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

    private:
        friend class TMC9660;
        explicit CurrentSensing(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } currentSensing{*this};
    
    //***************************************************************************
    //**                  SUBSYSTEM: Gate Driver                              **//
    //***************************************************************************
    /**
     * @brief Subsystem for configuring the MOSFET gate driver
     */
    struct GateDriver {
        /**
         * @brief Set the gate driver output polarity.
         * 
         * Configures the PWM_L and PWM_H output polarity.
         * @param lowSide Polarity for the low-side gate outputs (tmc9660::tmcl::PwmOutputPolarity enum).
         * @param highSide Polarity for the high-side gate outputs (tmc9660::tmcl::PwmOutputPolarity enum).
         * @return true if the polarity was set successfully.
         */
        bool setOutputPolarity(tmc9660::tmcl::PwmOutputPolarity lowSide, tmc9660::tmcl::PwmOutputPolarity highSide) noexcept;

        /**
         * @brief Configure the break-before-make timing for the gate driver.
         * 
         * Sets the timing for switching between high and low sides of the gate driver.
         * @param lowSideUVW Break-before-make time for UVW low side (0-255, 8.33ns units).
         * @param highSideUVW Break-before-make time for UVW high side (0-255, 8.33ns units).
         * @param lowSideY2 Break-before-make time for Y2 low side (0-255, 8.33ns units).
         * @param highSideY2 Break-before-make time for Y2 high side (0-255, 8.33ns units).
         * @return true if successfully configured.
         */
        bool configureBreakBeforeMakeTiming(uint8_t lowSideUVW, uint8_t highSideUVW,
                                             uint8_t lowSideY2, uint8_t highSideY2) noexcept;

        /**
         * @brief Enable or disable adaptive drive time for UVW and Y2 phases.
         * 
         * Adaptive drive time shortens the discharge cycle by monitoring gate voltages.
         * @param enableUVW True to enable adaptive drive time for UVW phases.
         * @param enableY2 True to enable adaptive drive time for Y2 phase.
         * @return true if successfully configured.
         */
        bool enableAdaptiveDriveTime(bool enableUVW, bool enableY2) noexcept;

        /**
         * @brief Configure drive times for UVW and Y2 phases.
         * 
         * Sets the discharge and charge times for the gate driver.
         * @param sinkTimeUVW Discharge time for UVW phases (0 ... 255). Default: 255.
         * @param sourceTimeUVW Charge time for UVW phases (0 ... 255). Default: 255.
         * @param sinkTimeY2 Discharge time for Y2 phase (0 ... 255). Default: 255.
         * @param sourceTimeY2 Charge time for Y2 phase (0 ... 255). Default: 255.
         * @return true if successfully configured.
         */
        bool configureDriveTimes(uint8_t sinkTimeUVW, uint8_t sourceTimeUVW,
                                  uint8_t sinkTimeY2, uint8_t sourceTimeY2) noexcept;

        /**
         * @brief Configure gate driver current limits for UVW and Y2 phases.
         * 
         * Sets the maximum sink and source currents for the gate driver.
         * @param sinkCurrentUVW Sink current for UVW phases (GateCurrentSink enum).
         * @param sourceCurrentUVW Source current for UVW phases (GateCurrentSource enum).
         * @param sinkCurrentY2 Sink current for Y2 phase (GateCurrentSink enum).
         * @param sourceCurrentY2 Source current for Y2 phase (GateCurrentSource enum).
         * @return true if successfully configured.
         */
        bool configureCurrentLimits(tmc9660::tmcl::GateCurrentSink sinkCurrentUVW,
                                    tmc9660::tmcl::GateCurrentSource sourceCurrentUVW,
                                    tmc9660::tmcl::GateCurrentSink sinkCurrentY2,
                                    tmc9660::tmcl::GateCurrentSource sourceCurrentY2) noexcept;

        /**
         * @brief Configure bootstrap current limit.
         * 
         * Sets the maximum current for the bootstrap capacitor.
         * @param limit Bootstrap current limit (BootstrapCurrentLimit enum).
         * @return true if successfully configured.
         */
        bool configureBootstrapCurrentLimit(tmc9660::tmcl::BootstrapCurrentLimit limit) noexcept;

        /**
         * @brief Configure undervoltage protection settings.
         * 
         * @param supplyLevel Supply voltage (VS) protection level (tmc9660::tmcl::UndervoltageLevel enum).
         * @param enableVdrv Enable driver voltage (VDRV) protection (tmc9660::tmcl::UndervoltageEnable enum).
         * @param enableBstUVW Enable bootstrap capacitor protection for UVW phases (tmc9660::tmcl::UndervoltageEnable enum).
         * @param enableBstY2 Enable bootstrap capacitor protection for Y2 phase (tmc9660::tmcl::UndervoltageEnable enum).
         * @return true if successfully configured.
         */
        bool configureUndervoltageProtection(tmc9660::tmcl::UndervoltageLevel supplyLevel, tmc9660::tmcl::UndervoltageEnable enableVdrv, 
                                             tmc9660::tmcl::UndervoltageEnable enableBstUVW, tmc9660::tmcl::UndervoltageEnable enableBstY2) noexcept;

        /**
         * @brief Enable or disable overcurrent protection for UVW and Y2 phases.
         * 
         * @param enableUVWLowSide Enable protection for UVW low side (tmc9660::tmcl::OvercurrentEnable enum).
         * @param enableUVWHighSide Enable protection for UVW high side (tmc9660::tmcl::OvercurrentEnable enum).
         * @param enableY2LowSide Enable protection for Y2 low side (tmc9660::tmcl::OvercurrentEnable enum).
         * @param enableY2HighSide Enable protection for Y2 high side (tmc9660::tmcl::OvercurrentEnable enum).
         * @return true if successfully configured.
         */
        bool enableOvercurrentProtection(tmc9660::tmcl::OvercurrentEnable enableUVWLowSide, tmc9660::tmcl::OvercurrentEnable enableUVWHighSide,
                                         tmc9660::tmcl::OvercurrentEnable enableY2LowSide, tmc9660::tmcl::OvercurrentEnable enableY2HighSide) noexcept;

        /**
         * @brief Configure overcurrent protection thresholds for UVW and Y2 phases.
         * 
         * @param uvwLowSideThreshold Threshold for UVW low side (tmc9660::tmcl::OvercurrentThreshold enum).
         * @param uvwHighSideThreshold Threshold for UVW high side (tmc9660::tmcl::OvercurrentThreshold enum).
         * @param y2LowSideThreshold Threshold for Y2 low side (tmc9660::tmcl::OvercurrentThreshold enum).
         * @param y2HighSideThreshold Threshold for Y2 high side (tmc9660::tmcl::OvercurrentThreshold enum).
         * @return true if successfully configured.
         */
        bool setOvercurrentThresholds(tmc9660::tmcl::OvercurrentThreshold uvwLowSideThreshold, tmc9660::tmcl::OvercurrentThreshold uvwHighSideThreshold,
                                      tmc9660::tmcl::OvercurrentThreshold y2LowSideThreshold, tmc9660::tmcl::OvercurrentThreshold y2HighSideThreshold) noexcept;

        /**
         * @brief Configure the overcurrent protection blanking time for UVW and Y2 phases.
         * 
         * Sets the blanking time for overcurrent protection to filter out transient spikes during switching events.
         * @param uvwLowSideTime Blanking time for the low side of UVW phases (tmc9660::tmcl::OvercurrentTiming enum).
         * @param uvwHighSideTime Blanking time for the high side of UVW phases (tmc9660::tmcl::OvercurrentTiming enum).
         * @param y2LowSideTime Blanking time for the low side of Y2 phase (tmc9660::tmcl::OvercurrentTiming enum).
         * @param y2HighSideTime Blanking time for the high side of Y2 phase (tmc9660::tmcl::OvercurrentTiming enum).
         * @return true if successfully configured.
         */
        bool setOvercurrentBlanking(tmc9660::tmcl::OvercurrentTiming uvwLowSideTime, tmc9660::tmcl::OvercurrentTiming uvwHighSideTime,
                                    tmc9660::tmcl::OvercurrentTiming y2LowSideTime, tmc9660::tmcl::OvercurrentTiming y2HighSideTime) noexcept;

        /**
         * @brief Configure the overcurrent protection deglitch time for UVW and Y2 phases.
         * 
         * Sets how long an overcurrent condition must persist before triggering protection.
         * @param uvwLowSideTime Deglitch time for the low side of UVW phases (tmc9660::tmcl::OvercurrentTiming enum).
         * @param uvwHighSideTime Deglitch time for the high side of UVW phases (tmc9660::tmcl::OvercurrentTiming enum).
         * @param y2LowSideTime Deglitch time for the low side of Y2 phase (tmc9660::tmcl::OvercurrentTiming enum).
         * @param y2HighSideTime Deglitch time for the high side of Y2 phase (tmc9660::tmcl::OvercurrentTiming enum).
         * @return true if successfully configured.
         */
        bool setOvercurrentDeglitch(tmc9660::tmcl::OvercurrentTiming uvwLowSideTime, tmc9660::tmcl::OvercurrentTiming uvwHighSideTime,
                                    tmc9660::tmcl::OvercurrentTiming y2LowSideTime, tmc9660::tmcl::OvercurrentTiming y2HighSideTime) noexcept;

        /**
         * @brief Enable or disable VDS monitoring for overcurrent protection on UVW and Y2 low sides.
         * 
         * @param uvwEnable True to enable VDS measurement for overcurrent protection on UVW low side (tmc9660::tmcl::VdsUsage enum).
         * @param y2Enable True to enable VDS measurement for overcurrent protection on Y2 low side (tmc9660::tmcl::VdsUsage enum).
         * @return true if successfully configured.
         */
        bool enableVdsMonitoringLow(tmc9660::tmcl::VdsUsage uvwEnable, tmc9660::tmcl::VdsUsage y2Enable) noexcept;

        /**
         * @brief Configure gate-to-source short protection for UVW phases.
         * 
         * @param enableLowSideOn Enable protection for ON transition of low side (tmc9660::tmcl::VgsShortEnable enum).
         * @param enableLowSideOff Enable protection for OFF transition of low side (tmc9660::tmcl::VgsShortEnable enum).
         * @param enableHighSideOn Enable protection for ON transition of high side (tmc9660::tmcl::VgsShortEnable enum).
         * @param enableHighSideOff Enable protection for OFF transition of high side (tmc9660::tmcl::VgsShortEnable enum).
         * @return true if successfully configured.
         */
        bool configureVgsShortProtectionUVW(tmc9660::tmcl::VgsShortEnable enableLowSideOn, tmc9660::tmcl::VgsShortEnable enableLowSideOff,
                                            tmc9660::tmcl::VgsShortEnable enableHighSideOn, tmc9660::tmcl::VgsShortEnable enableHighSideOff) noexcept;

        /**
         * @brief Configure gate-to-source short protection for Y2 phase.
         * 
         * @param enableLowSideOn Enable protection for ON transition of low side (tmc9660::tmcl::VgsShortEnable enum).
         * @param enableLowSideOff Enable protection for OFF transition of low side (tmc9660::tmcl::VgsShortEnable enum).
         * @param enableHighSideOn Enable protection for ON transition of high side (tmc9660::tmcl::VgsShortEnable enum).
         * @param enableHighSideOff Enable protection for OFF transition of high side (tmc9660::tmcl::VgsShortEnable enum).
         * @return true if successfully configured.
         */
        bool configureVgsShortProtectionY2(tmc9660::tmcl::VgsShortEnable enableLowSideOn, tmc9660::tmcl::VgsShortEnable enableLowSideOff,
                                           tmc9660::tmcl::VgsShortEnable enableHighSideOn, tmc9660::tmcl::VgsShortEnable enableHighSideOff) noexcept;

        /**
         * @brief Set gate-to-source short protection blanking time.
         * 
         * @param uvwTime Blanking time for UVW phases (tmc9660::tmcl::VgsBlankingTime enum).
         * @param y2Time Blanking time for Y2 phase (tmc9660::tmcl::VgsBlankingTime enum).
         * @return true if successfully configured.
         */
        bool setVgsShortBlankingTime(tmc9660::tmcl::VgsBlankingTime uvwTime, tmc9660::tmcl::VgsBlankingTime y2Time) noexcept;

        /**
         * @brief Set gate-to-source short protection deglitch time.
         * 
         * @param uvwTime Deglitch time for UVW phases (tmc9660::tmcl::VgsDeglitchTime enum).
         * @param y2Time Deglitch time for Y2 phase (tmc9660::tmcl::VgsDeglitchTime enum).
         * @return true if successfully configured.
         */
        bool setVgsShortDeglitchTime(tmc9660::tmcl::VgsDeglitchTime uvwTime, tmc9660::tmcl::VgsDeglitchTime y2Time) noexcept;

        /**
         * @brief Configure fault retry behavior.
         * 
         * @param retryBehavior Retry behavior after a fault (tmc9660::tmcl::GdrvRetryBehaviour enum).
         * @return true if successfully configured.
         */
        bool setRetryBehavior(tmc9660::tmcl::GdrvRetryBehaviour retryBehavior) noexcept;

        /**
         * @brief Configure drive fault behavior.
         * 
         * @param faultBehavior Behavior after all retries fail (tmc9660::tmcl::DriveFaultBehavior enum).
         * @return true if successfully configured.
         */
        bool setDriveFaultBehavior(tmc9660::tmcl::DriveFaultBehaviour faultBehavior) noexcept;

        /**
         * @brief Set the maximum number of retries for fault handling.
         * 
         * @param retries Maximum number of retries (0-255).
         * @return true if successfully configured.
         */
        bool setFaultHandlerRetries(uint8_t retries) noexcept;

    private:
        friend class TMC9660;
        explicit GateDriver(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } gateDriver{*this};

    //***************************************************************************
    //**                  SUBSYSTEM: FOC Control                              **//
    //***************************************************************************
    /**
     * @brief Subsystem for FOC control: torque/flux, velocity, position loops,
     *        open‐loop support, and reference switch / stop-event parameters.
     *
     * Covers TMCL parameters:
     *  - Torque/Flux PI   (104–120)
     *  - Velocity PI      (123–139)
     *  - Position PI      (142–157)
     *  - Open‐loop        (45–47)
     *  - Ref switch & stop (161–170)
     */
    struct FOCControl {
        //-------------------------------------------------------------------------
        // Torque / Flux control (104–120)
        //-------------------------------------------------------------------------
        /**
         * @brief Stop torque/flux control (SYSTEM_OFF).
         * @return true on success.
         */
        bool stop() noexcept;

        /**
         * @brief Set desired torque.
         * @param milliamps Target torque in mA.
         * @return true if written.
         */
        bool setTargetTorque(int16_t milliamps) noexcept;
        /**
         * @brief Read actual torque.
         * @param[out] milliamps Actual torque in mA.
         * @return true if read.
         */
        bool getActualTorque(int16_t& milliamps) noexcept;

        /**
         * @brief Set desired flux current.
         * @param milliamps Target flux in mA.
         * @return true if written.
         */
        bool setTargetFlux(int16_t milliamps) noexcept;
        /**
         * @brief Read actual flux current.
         * @param[out] milliamps Actual flux in mA.
         * @return true if read.
         */
        bool getActualFlux(int32_t& milliamps) noexcept;

        /**
         * @brief Set torque offset (feed-forward).
         * @param milliamps Offset in mA.
         * @return true if written.
         */
        bool setTorqueOffset(int16_t milliamps) noexcept;
        /**
         * @brief Read torque offset.
         * @param[out] milliamps Offset in mA.
         * @return true if read.
         */
        bool getTorqueOffset(int16_t& milliamps) noexcept;

        /**
         * @brief Set flux offset (feed-forward).
         * @param milliamps Offset in mA.
         * @return true if written.
         */
        bool setFluxOffset(int16_t milliamps) noexcept;
        /**
         * @brief Read flux offset.
         * @param[out] milliamps Offset in mA.
         * @return true if read.
         */
        bool getFluxOffset(int16_t& milliamps) noexcept;

        /**
         * @brief Configure current-loop PI gains.
         * @param p          Proportional gain for torque (and flux if not separate).
         * @param i          Integral gain for torque (and flux if not separate).
         * @param separate   true to use separate flux gains.
         * @param fluxP      Proportional gain for flux loop.
         * @param fluxI      Integral gain for flux loop.
         * @return true if written.
         */
        bool setCurrentLoopGains(uint16_t p, uint16_t i,
                                 bool separate = false,
                                 uint16_t fluxP = 0,
                                 uint16_t fluxI = 0) noexcept;
        /**
         * @brief Select combined or separate torque/flux PI parameters.
         * @param sep Separation mode.
         * @return true if written.
         */
        bool setTorqueFluxPiSeparation(tmc9660::tmcl::TorqueFluxPiSeparation sep) noexcept;

        /**
         * @brief Set normalization for current-PI outputs.
         * @param pNorm Normalization for P-term.
         * @param iNorm Normalization for I-term.
         * @return true if written.
         */
        bool setCurrentNormalization(tmc9660::tmcl::CurrentPiNormalization pNorm,
                                     tmc9660::tmcl::CurrentPiNormalization iNorm) noexcept;

        /**
         * @brief Read torque PI error.
         * @param[out] error Current torque-PI error.
         * @return true if read.
         */
        bool getTorquePiError(int32_t& error) noexcept;
        /**
         * @brief Read flux PI error.
         * @param[out] error Current flux-PI error.
         * @return true if read.
         */
        bool getFluxPiError(int32_t& error) noexcept;
        /**
         * @brief Read torque-PI integrator state.
         * @param[out] integrator Integrator value.
         * @return true if read.
         */
        bool getTorquePiIntegrator(int32_t& integrator) noexcept;
        /**
         * @brief Read flux-PI integrator state.
         * @param[out] integrator Integrator value.
         * @return true if read.
         */
        bool getFluxPiIntegrator(int32_t& integrator) noexcept;


        //-------------------------------------------------------------------------
        // Velocity control (123–139)
        //-------------------------------------------------------------------------
        /**
         * @brief Select velocity feedback sensor.
         * @param sel Sensor selection.
         * @return true if written.
         */
        bool setVelocitySensor(tmc9660::tmcl::VelocitySensorSelection sel) noexcept;
        /**
         * @brief Read velocity feedback sensor.
         * @param[out] sel Sensor selection.
         * @return true if read.
         */
        bool getVelocitySensor(tmc9660::tmcl::VelocitySensorSelection& sel) noexcept;

        /**
         * @brief Set target velocity.
         * @param velocity Target velocity (internal units).
         * @return true if written.
         */
        bool setTargetVelocity(int32_t velocity) noexcept;
        /**
         * @brief Read actual velocity.
         * @param[out] velocity Measured velocity.
         * @return true if read.
         */
        bool getActualVelocity(int32_t& velocity) noexcept;

        /**
         * @brief Set velocity offset.
         * @param offset Offset in RPM.
         * @return true if written.
         */
        bool setVelocityOffset(int32_t offset) noexcept;
        /**
         * @brief Read velocity offset.
         * @param[out] offset Offset in RPM.
         * @return true if read.
         */
        bool getVelocityOffset(int32_t& offset) noexcept;

        /**
         * @brief Configure velocity PI gains.
         * @param p P gain.
         * @param i I gain.
         * @return true if written.
         */
        bool setVelocityLoopGains(uint16_t p, uint16_t i) noexcept;
        /**
         * @brief Set velocity PI normalization.
         * @param pNorm P-term norm.
         * @param iNorm I-term norm.
         * @return true if written.
         */
        bool setVelocityNormalization(tmc9660::tmcl::VelocityPiNorm pNorm,
                                      tmc9660::tmcl::VelocityPiNorm iNorm) noexcept;

        /**
         * @brief Read velocity-PI integrator.
         * @param[out] integrator Integrator value.
         * @return true if read.
         */
        bool getVelocityPiIntegrator(int32_t& integrator) noexcept;
        /**
         * @brief Read velocity-PI error.
         * @param[out] error PI error.
         * @return true if read.
         */
        bool getVelocityPiError(int32_t& error) noexcept;

        /**
         * @brief Set velocity scaling factor.
         * @param factor Scale factor.
         * @return true if written.
         */
        bool setVelocityScalingFactor(uint16_t factor) noexcept;
        /**
         * @brief Read velocity scaling factor.
         * @param[out] factor Scale factor.
         * @return true if read.
         */
        bool getVelocityScalingFactor(uint16_t& factor) noexcept;

        /**
         * @brief Configure stop-on-velocity-deviation.
         * @param maxError Max allowed deviation.
         * @param softStop true for ramp down, false for hard stop.
         * @return true if written.
         */
        bool setStopOnVelocityDeviation(uint32_t maxError, bool softStop = true) noexcept;
        /**
         * @brief Read stop-on-velocity-deviation settings.
         * @param[out] maxError Configured max deviation.
         * @param[out] softStop Soft/hard stop flag.
         * @return true if read.
         */
        bool getStopOnVelocityDeviation(uint32_t& maxError, bool& softStop) noexcept;

        /**
         * @brief Set velocity loop downsampling.
         * @param divider Downsample factor.
         * @return true if written.
         */
        bool setVelocityLoopDownsampling(uint8_t divider) noexcept;
        /**
         * @brief Read velocity loop downsampling.
         * @param[out] divider Factor.
         * @return true if read.
         */
        bool getVelocityLoopDownsampling(uint8_t& divider) noexcept;

        /**
         * @brief Set velocity reached threshold.
         * @param threshold Threshold value.
         * @return true if written.
         */
        bool setVelocityReachedThreshold(uint32_t threshold) noexcept;
        /**
         * @brief Read velocity reached threshold.
         * @param[out] threshold Threshold.
         * @return true if read.
         */
        bool getVelocityReachedThreshold(uint32_t& threshold) noexcept;

        /**
         * @brief Set velocity meter switch threshold.
         * @param threshold Threshold value.
         * @return true if written.
         */
        bool setVelocityMeterSwitchThreshold(uint32_t threshold) noexcept;
        /**
         * @brief Read velocity meter switch threshold.
         * @param[out] threshold Threshold.
         * @return true if read.
         */
        bool getVelocityMeterSwitchThreshold(uint32_t& threshold) noexcept;

        /**
         * @brief Set velocity meter hysteresis.
         * @param hysteresis Hysteresis value.
         * @return true if written.
         */
        bool setVelocityMeterSwitchHysteresis(uint16_t hysteresis) noexcept;
        /**
         * @brief Read velocity meter hysteresis.
         * @param[out] hysteresis Hysteresis.
         * @return true if read.
         */
        bool getVelocityMeterSwitchHysteresis(uint16_t& hysteresis) noexcept;

        /**
         * @brief Read current velocity meter mode.
         * @param[out] mode Current mode.
         * @return true if read.
         */
        bool getVelocityMeterMode(tmc9660::tmcl::VelocityMeterMode& mode) noexcept;


        //-------------------------------------------------------------------------
        // Position control (142–157)
        //-------------------------------------------------------------------------
        /**
         * @brief Select position feedback sensor.
         * @param sel Sensor selection.
         * @return true if written.
         */
        bool setPositionSensor(tmc9660::tmcl::VelocitySensorSelection sel) noexcept;
        /**
         * @brief Read position feedback sensor.
         * @param[out] sel Sensor selection.
         * @return true if read.
         */
        bool getPositionSensor(tmc9660::tmcl::VelocitySensorSelection& sel) noexcept;

        /**
         * @brief Set target position.
         * @param position Desired position (internal units).
         * @return true if written.
         */
        bool setTargetPosition(int32_t position) noexcept;
        /**
         * @brief Read actual position.
         * @param[out] position Measured position.
         * @return true if read.
         */
        bool getActualPosition(int32_t& position) noexcept;

        /**
         * @brief Set position scaling factor.
         * @param factor Scale factor.
         * @return true if written.
         */
        bool setPositionScalingFactor(uint16_t factor) noexcept;
        /**
         * @brief Read position scaling factor.
         * @param[out] factor Scale factor.
         * @return true if read.
         */
        bool getPositionScalingFactor(uint16_t& factor) noexcept;

        /**
         * @brief Configure position PI gains.
         * @param p P gain.
         * @param i I gain.
         * @return true if written.
         */
        bool setPositionLoopGains(uint16_t p, uint16_t i) noexcept;
        /**
         * @brief Set position PI normalization.
         * @param pNorm P-term norm.
         * @param iNorm I-term norm.
         * @return true if written.
         */
        bool setPositionNormalization(tmc9660::tmcl::VelocityPiNorm pNorm,
                                      tmc9660::tmcl::VelocityPiNorm iNorm) noexcept;

        /**
         * @brief Read position-PI integrator.
         * @param[out] integrator Integrator value.
         * @return true if read.
         */
        bool getPositionPiIntegrator(int32_t& integrator) noexcept;
        /**
         * @brief Read position-PI error.
         * @param[out] error PI error.
         * @return true if read.
         */
        bool getPositionPiError(int32_t& error) noexcept;

        /**
         * @brief Configure stop-on-position-deviation.
         * @param maxError Max allowed deviation.
         * @param softStop true for ramp down, false for hard stop.
         * @return true if written.
         */
        bool setStopOnPositionDeviation(uint32_t maxError, bool softStop = true) noexcept;
        /**
         * @brief Read stop-on-position-deviation settings.
         * @param[out] maxError Configured max deviation.
         * @param[out] softStop Soft/hard stop flag.
         * @return true if read.
         */
        bool getStopOnPositionDeviation(uint32_t& maxError, bool& softStop) noexcept;

        /**
         * @brief Set position loop downsampling.
         * @param divider Downsample factor.
         * @return true if written.
         */
        bool setPositionLoopDownsampling(uint8_t divider) noexcept;
        /**
         * @brief Read position loop downsampling.
         * @param[out] divider Factor.
         * @return true if read.
         */
        bool getPositionLoopDownsampling(uint8_t& divider) noexcept;

        /**
         * @brief Set low position limit.
         * @param limit Minimum allowed position.
         * @return true if written.
         */
        bool setPositionLimitLow(int32_t limit) noexcept;
        /**
         * @brief Read low position limit.
         * @param[out] limit Minimum allowed position.
         * @return true if read.
         */
        bool getPositionLimitLow(int32_t& limit) noexcept;

        /**
         * @brief Set high position limit.
         * @param limit Maximum allowed position.
         * @return true if written.
         */
        bool setPositionLimitHigh(int32_t limit) noexcept;
        /**
         * @brief Read high position limit.
         * @param[out] limit Maximum allowed position.
         * @return true if read.
         */
        bool getPositionLimitHigh(int32_t& limit) noexcept;

        /**
         * @brief Set position reached threshold.
         * @param threshold Latch threshold.
         * @return true if written.
         */
        bool setPositionReachedThreshold(uint32_t threshold) noexcept;
        /**
         * @brief Read position reached threshold.
         * @param[out] threshold Latch threshold.
         * @return true if read.
         */
        bool getPositionReachedThreshold(uint32_t& threshold) noexcept;


        //-------------------------------------------------------------------------
        // Open‐loop support (45–47)
        //-------------------------------------------------------------------------
        /**
         * @brief Read open-loop angle.
         * @param[out] angle Electrical angle.
         * @return true if read.
         */
        bool getOpenloopAngle(int16_t& angle) noexcept;

        /**
         * @brief Set open-loop current.
         * @param milliamps Current in mA.
         * @return true if written.
         */
        bool setOpenloopCurrent(uint16_t milliamps) noexcept;
        /**
         * @brief Read open-loop current.
         * @param[out] milliamps Current in mA.
         * @return true if read.
         */
        bool getOpenloopCurrent(uint16_t& milliamps) noexcept;

        /**
         * @brief Set open-loop voltage.
         * @param voltage Voltage unit (0…32767).
         * @return true if written.
         */
        bool setOpenloopVoltage(uint16_t voltage) noexcept;
        /**
         * @brief Read open-loop voltage.
         * @param[out] voltage Voltage unit.
         * @return true if read.
         */
        bool getOpenloopVoltage(uint16_t& voltage) noexcept;


        //-------------------------------------------------------------------------
        // Ref switch & stop-event (161–170)
        //-------------------------------------------------------------------------
        /**
         * @brief Enable/disable reference switch stops.
         * @param enable Bitmask of switch stops.
         * @return true if written.
         */
        bool setReferenceSwitchEnable(tmc9660::tmcl::ReferenceSwitchEnable enable) noexcept;
        /**
         * @brief Read reference switch enable mask.
         * @param[out] enable Mask.
         * @return true if read.
         */
        bool getReferenceSwitchEnable(tmc9660::tmcl::ReferenceSwitchEnable& enable) noexcept;

        /**
         * @brief Configure switch polarity and swap.
         * @param config Polarity/swap config.
         * @return true if written.
         */
        bool setReferenceSwitchPolaritySwap(tmc9660::tmcl::ReferenceSwitchPolaritySwap config) noexcept;
        /**
         * @brief Read switch polarity/swap config.
         * @param[out] config Config.
         * @return true if read.
         */
        bool getReferenceSwitchPolaritySwap(tmc9660::tmcl::ReferenceSwitchPolaritySwap& config) noexcept;

        /**
         * @brief Configure switch latch settings.
         * @param setting Latch behavior.
         * @return true if written.
         */
        bool setReferenceSwitchLatchSettings(tmc9660::tmcl::ReferenceSwitchLatchSettings setting) noexcept;
        /**
         * @brief Read switch latch settings.
         * @param[out] setting Latch behavior.
         * @return true if read.
         */
        bool getReferenceSwitchLatchSettings(tmc9660::tmcl::ReferenceSwitchLatchSettings& setting) noexcept;

        /**
         * @brief Configure event-stop settings.
         * @param settings Stop conditions.
         * @return true if written.
         */
        bool setEventStopSettings(tmc9660::tmcl::EventStopSettings settings) noexcept;
        /**
         * @brief Read event-stop settings.
         * @param[out] settings Stop conditions.
         * @return true if read.
         */
        bool getEventStopSettings(tmc9660::tmcl::EventStopSettings& settings) noexcept;

        /**
         * @brief Set reference search mode.
         * @param mode Search sequence.
         * @return true if written.
         */
        bool setReferenceSwitchSearchMode(tmc9660::tmcl::ReferenceSwitchSearchMode mode) noexcept;
        /**
         * @brief Read reference search mode.
         * @param[out] mode Search sequence.
         * @return true if read.
         */
        bool getReferenceSwitchSearchMode(tmc9660::tmcl::ReferenceSwitchSearchMode& mode) noexcept;

        /**
         * @brief Set reference search speed.
         * @param speed Search velocity.
         * @return true if written.
         */
        bool setReferenceSwitchSearchSpeed(int32_t speed) noexcept;
        /**
         * @brief Read reference search speed.
         * @param[out] speed Search velocity.
         * @return true if read.
         */
        bool getReferenceSwitchSearchSpeed(int32_t& speed) noexcept;

        /**
         * @brief Set reference positioning speed.
         * @param speed Approach speed.
         * @return true if written.
         */
        bool setReferenceSwitchSpeed(int32_t speed) noexcept;
        /**
         * @brief Read reference positioning speed.
         * @param[out] speed Approach speed.
         * @return true if read.
         */
        bool getReferenceSwitchSpeed(int32_t& speed) noexcept;

        /**
         * @brief Read right-limit-switch position.
         * @param[out] position Position value.
         * @return true if read.
         */
        bool getRightLimitSwitchPosition(int32_t& position) noexcept;
        /**
         * @brief Read home-switch position.
         * @param[out] position Position value.
         * @return true if read.
         */
        bool getHomeSwitchPosition(int32_t& position) noexcept;
        /**
         * @brief Read last reference position.
         * @param[out] position Position value.
         * @return true if read.
         */
        bool getLastReferencePosition(int32_t& position) noexcept;

    private:
        friend class TMC9660;
        explicit FOCControl(TMC9660& parent) noexcept
            : driver(parent) {}
        TMC9660& driver;
    } focControl{*this};

    //***************************************************************************
    //**                  SUBSYSTEM: Motion Ramp                               **//
    //***************************************************************************
    /**
     * @brief Hardware 8-segment acceleration/dec-acc profile controller.
     *
     * Covers all ramp generator and feedforward parameters:
     * - ACCELERATION_FF_GAIN (50)
     * - ACCELERATION_FF_SHIFT (51)
     * - RAMP_ENABLE (52)
     * - DIRECT_VELOCITY_MODE (53)
     * - RAMP_AMAX/A1/A2 (54/55/56)
     * - RAMP_DMAX/D1/D2 (57/58/59)
     * - RAMP_VMAX/V1/V2/VSTART/VSTOP (60/61/62/63/64)
     * - RAMP_TVMAX/TZEROWAIT (65/66)
     * - ACCELERATION_FEEDFORWARD_ENABLE (67)
     * - VELOCITY_FEEDFORWARD_ENABLE (68)
     * - RAMP_VELOCITY (69)
     * - RAMP_POSITION (70)
     */
    struct Ramp {
        /**
         * @brief Enable or disable the ramp generator block.
         * @param on True to enable, false to disable (RAMP_ENABLE).
         */
        bool enable(bool on) noexcept;

        /**
         * @brief Set acceleration segments A1, A2, Amax (µ units/s²).
         * @param a1 First acceleration (RAMP_A1)
         * @param a2 Second acceleration (RAMP_A2)
         * @param aMax Top acceleration (RAMP_AMAX)
         */
        bool setAcceleration(uint32_t a1,
                             uint32_t a2,
                             uint32_t aMax) noexcept;

        /**
         * @brief Set deceleration segments D1, D2, Dmax (µ units/s²).
         * @param d1 Second deceleration (RAMP_D1)
         * @param d2 First deceleration (RAMP_D2)
         * @param dMax Top deceleration (RAMP_DMAX)
         */
        bool setDeceleration(uint32_t d1,
                             uint32_t d2,
                             uint32_t dMax) noexcept;

        /**
         * @brief Configure velocity thresholds and limits.
         * @param vStart Start velocity (RAMP_VSTART)
         * @param vStop Stop velocity (RAMP_VSTOP)
         * @param v1 Velocity threshold 1 (RAMP_V1)
         * @param v2 Velocity threshold 2 (RAMP_V2)
         * @param vMax Maximum velocity (RAMP_VMAX)
         */
        bool setVelocities(uint32_t vStart,
                           uint32_t vStop,
                           uint32_t v1,
                           uint32_t v2,
                           uint32_t vMax) noexcept;

        /**
         * @brief Timing constraints at Vmax and between moves.
         * @param tVmaxCycles Minimum time at VMAX (RAMP_TVMAX)
         * @param tZeroWaitCycles Wait time at end of ramp (RAMP_TZEROWAIT)
         */
        bool setTiming(uint16_t tVmaxCycles,
                       uint16_t tZeroWaitCycles) noexcept;

        /**
         * @brief Enable hardware feed-forward terms and set gain/shift.
         *
         * This allows the ramp generator to use feed-forward terms for velocity and acceleration.
         * @param enableVelFF Enable the VELOCITY_FEEDFORWARD feature (VELOCITY_FEEDFORWARD_ENABLE)
         * @param enableAccelFF Enable the ACCELERATION_FEEDFORWARD feature (ACCELERATION_FEEDFORWARD_ENABLE)
         * @param accelFFGain  ACCELERATION_FF_GAIN (0…65535)
         * @param accelFFShift ACCELERATION_FF_SHIFT enum (tmc9660::tmcl::AccelerationFFShift)
         */
        bool enableFeedForward(bool enableVelFF,
                               bool enableAccelFF,
                               uint16_t accelFFGain,
                               tmc9660::tmcl::AccelerationFFShift accelFFShift) noexcept;

        /**
         * @brief Direct-velocity mode instead of classic PI velocity loop.
         * @param enable True to enable direct velocity mode (DIRECT_VELOCITY_MODE)
         */
        bool setDirectVelocityMode(bool enable) noexcept;

        /**
         * @brief Get the current target velocity calculated by the ramp controller.
         * @param[out] velocity The current ramp target velocity (RAMP_VELOCITY, param 69)
         * @return true if the value was read successfully.
         */
        bool getRampVelocity(int32_t& velocity) noexcept;

        /**
         * @brief Get the current target position calculated by the ramp controller.
         * @param[out] position The current ramp target position (RAMP_POSITION, param 70)
         * @return true if the value was read successfully.
         */
        bool getRampPosition(int32_t& position) noexcept;

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
         * This enables Hall sensor inputs as the feedback for commutation. Typically used with tmc9660::tmcl::CommutationMode::FOC_HALL.
         * @param sectorOffset Hall sensor 60-degree/sector offset (tmc9660::tmcl::HallSectorOffset):
         *                     tmc9660::tmcl::HallSectorOffset::DEG_0, DEG_60, DEG_120, DEG_180, DEG_240, DEG_300
         *                     This combines both the 120° order offset and 180° polarity offset.
         * @param inverted If true, invert the interpretation of hall sensor signals (tmc9660::tmcl::Direction).
         * @param enableExtrapolation If true, enable hall extrapolation for higher resolution position signal (tmc9660::tmcl::EnableDisable).
         * @param filterLength Digital filter length (0-255) for hall sensor inputs.
         * @return true if Hall sensor feedback is configured successfully.
         */
        bool configureHall(tmc9660::tmcl::HallSectorOffset sectorOffset = tmc9660::tmcl::HallSectorOffset::DEG_0, 
                   tmc9660::tmcl::Direction inverted = tmc9660::tmcl::Direction::NOT_INVERTED, 
                   tmc9660::tmcl::EnableDisable enableExtrapolation = tmc9660::tmcl::EnableDisable::DISABLED, 
                   uint8_t filterLength = 0) noexcept;

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
         * @brief Read the electrical angle (phi_e) calculated from Hall feedback.
         * @param[out] phiE Electrical angle (-32768 to 32767).
         * @return true if the value was read successfully.
         */
        bool getHallPhiE(int16_t& phiE) noexcept;

        /**
         * @brief Configure an ABN incremental encoder for feedback.
         * 
         * Sets up an incremental quadrature encoder with optional index (N) channel for position and velocity feedback.
         * @param countsPerRev Encoder resolution (counts per revolution, 0-16777215).
         * @param inverted If true, invert the encoder direction (tmc9660::tmcl::Direction).
         * @param nChannelInverted If true, invert the N-channel signal (active low instead of active high) (tmc9660::tmcl::EnableDisable).
         * @return true if encoder parameters were set successfully.
         */
        bool configureABNEncoder(uint32_t countsPerRev, tmc9660::tmcl::Direction inverted = tmc9660::tmcl::Direction::NOT_INVERTED, 
                     tmc9660::tmcl::EnableDisable nChannelInverted = tmc9660::tmcl::EnableDisable::DISABLED) noexcept;

        /**
         * @brief Configure the secondary ABN encoder input.
         *
         * This allows the use of a second incremental encoder or a geared
         * encoder setup. It writes ABN_2_* parameters to set the resolution,
         * direction and optional gear ratio.
         *
         * @param countsPerRev Encoder resolution in counts per revolution.
         * @param inverted     True to invert the encoder direction (tmc9660::tmcl::Direction).
         * @param gearRatio    Gear ratio between the second encoder and the
         *                     motor shaft. Use 1 if directly coupled.
         * @return true if all parameters were written successfully.
         */
        bool configureSecondaryABNEncoder(uint32_t countsPerRev,
                          tmc9660::tmcl::Direction inverted = tmc9660::tmcl::Direction::NOT_INVERTED,
                          uint8_t gearRatio = 1) noexcept;

        /**
         * @brief Enable or disable the secondary ABN encoder.
         * @param enable True to enable, false to disable.
         * @return true if the operation was successful.
         */
        bool setSecondaryABNEncoderEnabled(bool enable) noexcept;

        /**
         * @brief Read the raw ABN2 encoder internal counter value.
         * @param[out] value Raw counter value (0-4294967295).
         * @return true if the value was read successfully.
         */
        bool getSecondaryABNEncoderValue(uint32_t& value) noexcept;

        /**
         * @brief Configure ABN encoder initialization method.
         * 
         * Sets the method used to align the ABN encoder with the rotor's absolute position.
         * 
         * @param initMethod Initialization method (tmc9660::tmcl::AbnInitMethod):
         *                   FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING, FORCED_PHI_E_90_ZERO, USE_HALL, USE_N_CHANNEL_OFFSET
         * @param initDelay Delay in milliseconds to wait for mechanical oscillations to stop (1000-10000)
         * @param initVelocity Velocity used during N-channel initialization (-200000 to 200000)
         * @param nChannelOffset Offset between phi_e zero and encoder index pulse position (-32768 to 32767)
         * @return true if ABN initialization parameters were set successfully.
         */
        bool configureABNInitialization(tmc9660::tmcl::AbnInitMethod initMethod = tmc9660::tmcl::AbnInitMethod::FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING, 
                        uint16_t initDelay = 1000, int32_t initVelocity = 5, int16_t nChannelOffset = 0) noexcept;

        /**
         * @brief Read the electrical angle (phi_e) calculated from ABN feedback.
         * @param[out] phiE Electrical angle (-32768 to 32767).
         * @return true if the value was read successfully.
         */
        bool getABNPhiE(int16_t& phiE) noexcept;

        /**
         * @brief Configure N-channel filtering for ABN encoder.
         * 
         * Sets up filtering for the N-channel (index pulse) to handle imprecise encoders.
         * 
         * @param filterMode N-channel filtering mode (tmc9660::tmcl::AbnNChannelFiltering):
         *                   FILTERING_OFF, N_EVENT_ON_A_HIGH_B_HIGH, N_EVENT_ON_A_HIGH_B_LOW, N_EVENT_ON_A_LOW_B_HIGH, N_EVENT_ON_A_LOW_B_LOW
         * @param clearOnNextNull If true, clear position counter on next N-channel event (tmc9660::tmcl::EnableDisable).
         * @return true if N-channel settings were applied successfully.
         */
        bool configureABNNChannel(tmc9660::tmcl::AbnNChannelFiltering filterMode = tmc9660::tmcl::AbnNChannelFiltering::FILTERING_OFF, 
                      tmc9660::tmcl::EnableDisable clearOnNextNull = tmc9660::tmcl::EnableDisable::DISABLED) noexcept;

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
         * @param invertDirection If true, invert the direction of the SPI encoder (tmc9660::tmcl::Direction).
         * @return true if configuration was successful.
         */
        bool configureSPIEncoderDataFormat(uint32_t positionMask, uint8_t positionShift = 0, 
                           tmc9660::tmcl::Direction invertDirection = tmc9660::tmcl::Direction::NOT_INVERTED) noexcept;

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
         * @param initMethod Initialization method (tmc9660::tmcl::SpiInitMethod):
         *                   FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING, FORCED_PHI_E_90_ZERO, USE_OFFSET
         * @param offset Manual offset value if using offset-based initialization.
         * @return true if initialization method was set successfully.
         */
        bool configureSPIEncoderInitialization(tmc9660::tmcl::SpiInitMethod initMethod, int16_t offset = 0) noexcept;

        /**
         * @brief Enable or disable SPI encoder lookup table correction.
         * 
         * Enables the lookup table-based correction for encoder nonlinearity.
         * 
         * @param enable If true, enable LUT correction (tmc9660::tmcl::EnableDisable).
         * @param shiftFactor Common shift factor for all LUT entries.
         * @return true if LUT settings were applied successfully.
         */
        bool setSPIEncoderLUTCorrection(tmc9660::tmcl::EnableDisable enable, int8_t shiftFactor = 0) noexcept;

        /**
         * @brief Upload a single entry to the SPI encoder correction lookup table.
         * 
         * @param index Index in the LUT (0-255).
         * @param value Correction value (-128 to 127).
         * @return true if the entry was uploaded successfully.
         */
        bool uploadSPIEncoderLUTEntry(uint8_t index, int8_t value) noexcept;

        /**
         * @brief Read the current state of ABN encoder initialization.
         * @param[out] state Current initialization state (tmc9660::tmcl::AbnInitState):
         *                   IDLE, BUSY, WAIT, DONE
         * @return true if the state was read successfully.
         */
        bool getABNInitializationState(tmc9660::tmcl::AbnInitState& state) noexcept;

        /**
         * @brief Read the raw ABN encoder internal counter value.
         * @param[out] value Raw counter value (0-16777215).
         * @return true if the value was read successfully.
         */
        bool getABNRawValue(uint32_t& value) noexcept;

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
    struct RamDebug {
        /**
         * @brief Initialize and configure the RAMDebug feature (data logging).
         * 
         * This sends a command to initialize the RAM debug system and reset any previous configurations.
         * @param sampleCount Number of samples to collect in the buffer.
         * @return true if the RAM debug was initialized properly.
         */
        bool init(uint32_t sampleCount) noexcept;

        /**
         * @brief Start capturing data using RAMDebug.
         * @return true if the capture started successfully.
         */
        bool startCapture() noexcept;

        /**
         * @brief Read a captured sample from RAMDebug buffer.
         * @param index Sample index to read (0-based).
         * @param[out] data Variable to store the 32-bit sample data.
         * @return true if the data was read successfully.
         */
        bool readData(uint32_t index, uint32_t& data) noexcept;

        /**
         * @brief Get the current state of the RAM debug engine.
         * @param[out] isRunning Will be set to true if capture is ongoing, false if stopped or idle.
         * @return true if the status was retrieved successfully.
         */
        bool getStatus(bool& isRunning) noexcept;

    private:
        friend class TMC9660;
        explicit RamDebug(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } ramDebug{*this};

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

        /**
         * @brief Read the GENERAL_STATUS_FLAGS register.
         * @param[out] flags Bit mask of current status flags.
         * @return true if the flags were read successfully.
         */
        bool getGeneralStatusFlags(uint32_t& flags) noexcept;

        /**
         * @brief Read the GENERAL_ERROR_FLAGS register.
         * @param[out] flags Bit mask of current error flags.
         * @return true if the flags were read successfully.
         */
        bool getGeneralErrorFlags(uint32_t& flags) noexcept;

        /**
         * @brief Read the GDRV_ERROR_FLAGS register.
         * @param[out] flags Bit mask of current gate driver error flags.
         * @return true if the flags were read successfully.
         */
        bool getGateDriverErrorFlags(uint32_t& flags) noexcept;

        /**
         * @brief Clear bits in the GENERAL_ERROR_FLAGS register.
         * @param mask Bit mask of flags to clear (write-1-to-clear).
         * @return true if the mask was written successfully.
         */
        bool clearGeneralErrorFlags(uint32_t mask) noexcept;

        /**
         * @brief Clear bits in the GDRV_ERROR_FLAGS register.
         * @param mask Bit mask of flags to clear.
         * @return true if the mask was written successfully.
         */
        bool clearGateDriverErrorFlags(uint32_t mask) noexcept;

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
     * @brief Subsystem for motor thermal overload protection via I²t integration.
     *
     * Configures two independent I²t windows that monitor integrated current over time
     * (in A²·ms) to detect thermal overloads. If either limit is exceeded, a fault is triggered.
     *
     * - Refer to: Parameters #224–#228 (Table 41)
     * - Manual: “IIT” section, p. 86:contentReference[oaicite:1]{index=1}
     * - Related fault flags: `IIT_1_EXCEEDED`, `IIT_2_EXCEEDED`
     */
    struct IIT {
        /**
         * @brief Configure the two I²t monitoring windows for motor current.
         *
         * @param timeConstant1_ms Time constant for fast window [ms]
         * @param continuousCurrent1_A Continuous current limit for window 1 [A]
         * @param timeConstant2_ms Time constant for slow window [ms]
         * @param continuousCurrent2_A Continuous current limit for window 2 [A]
         * @return true if parameters written successfully
         *
         * Parameters:
         * - `THERMAL_WINDING_TIME_CONSTANT_1/2`
         * - `IIT_LIMIT_1/2`
         */
        bool configure(uint16_t timeConstant1_ms, float continuousCurrent1_A,
                    uint16_t timeConstant2_ms, float continuousCurrent2_A) noexcept;

        /**
         * @brief Reset the integrated current monitoring accumulators.
         *
         * Clears the internal counters of both I²t windows to 0.
         * Use this after a thermal fault is acknowledged.
         *
         * - Parameter: `RESET_IIT_SUMS`
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
     * @brief Subsystem for controlling the STEP/DIR pulse input interface.
     *
     * Enables stepper-style control using external STEP and DIR pulses, with support for:
     * - Signal extrapolation to interpolate between pulses
     * - Velocity feed-forward
     * - Microstep resolution configuration
     *
     * Refer to Parameter IDs #205–#209 (Table 48), and STEP/DIR section, p. 95:contentReference[oaicite:2]{index=2}
     */
    struct StepDir {
        /**
         * @brief Enable or disable the STEP/DIR interface.
         *
         * @param on true = enable, false = disable
         * @return true on success
         *
         * - Parameter: `STEPDIR_ENABLE`
         * - Boot option: Table 11:contentReference[oaicite:3]{index=3}
         */
        bool enableInterface(bool on) noexcept;

        /**
         * @brief Configure microstep resolution for each STEP pulse.
         *
         * @param µSteps Microsteps per full step (e.g. 256 = 1/256 resolution)
         * @return true on success
         *
         * - Parameter: `STEPDIR_STEP_DIVIDER_SHIFT`
         *   (shift of incoming step pulse count)
         */
        bool setMicrostepResolution(uint16_t µSteps) noexcept;

        /**
         * @brief Enable or disable velocity feed-forward calculation.
         *
         * @param enable true = enable, false = disable
         * @return true on success
         *
         * - Parameter: `VELOCITY_FEEDFORWARD_ENABLE`
         */
        bool enableVelocityFeedforward(bool enable) noexcept;

        /**
         * @brief Enable signal extrapolation between STEP pulses.
         *
         * @param enable true = enable extrapolation
         * @return true on success
         *
         * - Parameter: `STEPDIR_EXTRAPOLATE`
         * - Behavior described on p. 96, Fig. 27:contentReference[oaicite:4]{index=4}
         */
        bool enableExtrapolation(bool enable) noexcept;

        /**
         * @brief Timeout before extrapolated motion stops after last pulse.
         *
         * @param timeout_ms Timeout in milliseconds
         * @return true on success
         *
         * - Parameter: `STEPDIR_STEP_SIGNAL_TIMEOUT_LIMIT`
         */
        bool setSignalTimeout(uint16_t timeout_ms) noexcept;

        /**
         * @brief Set maximum allowed extrapolation velocity.
         *
         * @param eRPM Max electrical RPM before extrapolation is disabled
         * @return true on success
         *
         * - Parameter: `STEPDIR_MAXIMUM_EXTRAPOLATION_VELOCITY`
         */
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
     * @brief Subsystem for storing and recalling parameters from nonvolatile flash.
     *
     * This API wraps STAP (0xFFF) and FactoryDefault commands. Use it to
     * persist all RWE parameters across power cycles, matching the functionality of
     * the “Save to Flash” button in GUI tools.
     *
     * WARNING: Requires that external memory is configured via BOOT_CONFIG.
     * Refer to: “Storing System Settings” section, p. 15:contentReference[oaicite:5]{index=5}
     */
    struct NvmStorage {
        /**
         * @brief Store all writable parameters to flash or EEPROM.
         *
         * @return true if the operation completed without error.
         *
         * Internally sends `STAP` (type=0xFFF) to save parameters.
         */
        bool storeToFlash() noexcept;

        /**
         * @brief Restore parameters previously saved to NVM.
         *
         * @return true if configuration was successfully recalled.
         *
         * Sets CONFIG_LOADED flag in GENERAL_STATUS_FLAGS if success.
         */
        bool recallFromFlash() noexcept;

        /**
         * @brief Erase a configuration bank from external memory.
         *
         * @param n Index of the flash bank to erase (typically 0)
         * @return true if erase command sent successfully.
         *
         * Use before re-storing to flash if stale config causes issues.
         */
        bool eraseFlashBank(uint8_t n) noexcept;

    private:
        friend class TMC9660;
        explicit NvmStorage(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } nvmStorage{*this};


    //***************************************************************************
    //**                SUBSYSTEM: Heartbeat (Watchdog)                        **//
    //***************************************************************************
    /**
     * @brief Subsystem for configuring the communication watchdog (heartbeat).
     *
     * If enabled, the TMC9660 monitors the time since the last command.
     * If no communication occurs before timeout expires, the chip faults or disables motor outputs.
     *
     * Refer to Parameters #10 & #11 in Global Bank 0 (Table 43), p. 89:contentReference[oaicite:6]{index=6}
     */
    struct Heartbeat {
        /**
         * @brief Enable the heartbeat monitor and set timeout.
         *
         * @param mode ENABLE or DISABLE the watchdog
         * @param timeout_ms Timeout in milliseconds
         * @return true if both values written successfully
         *
         * - Parameters:
         *   - `HEARTBEAT_MONITORING_CONFIG`
         *   - `HEARTBEAT_MONITORING_TIMEOUT`
         */
        bool configure(HeartbeatMode mode, uint32_t timeout_ms) noexcept;

    private:
        friend class TMC9660;
        explicit Heartbeat(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } heartbeat{*this};


    //***************************************************************************
    //**        SUBSYSTEM: General-purpose GPIO (Digital/Analog I/O)           **//
    //***************************************************************************
    /**
    * @brief Subsystem for configuring general-purpose IOs (GPIOs).
    *
    * Pins can be configured as digital inputs, digital outputs, or analog inputs.
    * The input pull-up/down resistors and output state can also be controlled.
    *
    * Refer to GPIO section and TMCL commands SIO / GIO (Table 18), p. 19:contentReference[oaicite:7]{index=7}
    */
    struct GPIO {
        /**
         * @brief Configure a GPIO pin as input or output.
         *
         * @param pin GPIO index (e.g. GPIO0..GPIO18)
         * @param output Set to true to make the pin output, false = input
         * @param pullEnable Enable pull resistor
         * @param pullUp true = pull-up, false = pull-down
         * @return true if configuration applied successfully
         */
        bool setMode(uint8_t pin, bool output,
                    bool pullEnable = false,
                    bool pullUp = true) noexcept;

        /**
         * @brief Write a digital value to a configured output pin.
         * @param pin GPIO pin index
         * @param value true = high, false = low
         * @return true if write succeeded
         */
        bool writePin(uint8_t pin, bool value) noexcept;

        /**
         * @brief Read a digital input pin.
         * @param pin GPIO pin index
         * @param[out] value Logic level read from the pin
         * @return true on successful read
         */
        bool readDigital(uint8_t pin, bool &value) noexcept;

        /**
         * @brief Read an analog input (e.g. external temperature or potentiometer).
         * @param pin ADC input index
         * @param[out] value Raw ADC value (typically 0–65535)
         * @return true on success
         *
         * - Analog reads typically apply to AIN3, used with external thermistors.
         */
        bool readAnalog(uint8_t pin, uint16_t &value) noexcept;

    private:
        friend class TMC9660;
        explicit GPIO(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } gpio{*this};

    //***************************************************************************
    //**               SUBSYSTEM: Power Management                          **//
    //***************************************************************************
    /**
     * @brief Subsystem for entering low-power hibernation mode and configuring wake.
     *
     * The TMC9660 supports timed power-down and wake-on-pin. These features reduce power when idle.
     *
     * - See Section “Hibernation and Wakeup” p. 101:contentReference[oaicite:8]{index=8}
     * - Wake behavior depends on BOOT_CONFIG + pin wiring
     */
    struct Power {
        /**
         * @brief Enable or disable the external wake-up pin.
         * @param enable true = enable pin
         * @return true on success
         *
         * - Parameter: `ENABLE_WAKE_PIN`
         */
        bool enableWakePin(bool enable) noexcept;

        /**
         * @brief Put the chip into power-down mode for a set duration.
         * @param period Enum selecting one of 6 durations (e.g. PERIOD_1 = 250 ms)
         * @return true on success
         *
         * - Parameter: `GO_TO_TIMEOUT_POWER_DOWN_STATE`
         */
        bool enterPowerDown(PowerDownPeriod period) noexcept;

    private:
        friend class TMC9660;
        explicit Power(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    } power{*this};


    //***************************************************************************
    //**                SUBSYSTEM: Fault Handling and Retry                    **//
    //***************************************************************************
    /**
     * @brief Subsystem for configuring automatic fault retry logic.
     *
     * When a protection fault occurs (e.g., OCP, UVLO), the TMC9660 can either:
     * - retry commutation (up to a configured limit)
     * - or disable the motor permanently
     *
     * Refer to Table 40 (p. 84), parameters:
     * - FAULT_RECOVERY_RETRIES
     * - FAULT_RETRY_ACTION
     * - FAULT_FINAL_ACTION:contentReference[oaicite:9]{index=9}
     */
    struct Fault {
        /**
         * @brief Configure retry behavior after a motor fault.
         *
         * @param retryAction Whether to retry once or continue immediately
         * @param finalAction Behavior after final fault (disable outputs vs. keep driving)
         * @param retries Number of retry attempts before giving up (0–255)
         * @return true on success
         */
        bool configure(FaultRetryAction retryAction,
                    FaultFinalAction finalAction,
                    uint8_t retries) noexcept;

    private:
        friend class TMC9660;
        explicit Fault(TMC9660& parent) noexcept : driver(parent) {}
        TMC9660& driver;
    };


    //==================================================
    // PRIVATE MEMBERS
    //==================================================
private:

    TMC9660CommInterface& comm_;  ///< Communication interface (transport) for sending/receiving data.
    uint8_t address_;            ///< Module address (0-127). Used primarily for UART multi-drop addressing.

    #ifdef TMC_API_EXTERNAL_CRC_TABLE
    extern const uint8_t tmcCRCTable_Poly7Reflected[256];
    #else
    const uint8_t tmcCRCTable_Poly7Reflected[256] = {
        0x00, 0x91, 0xE3, 0x72, 0x07, 0x96, 0xE4, 0x75, 0x0E, 0x9F, 0xED, 0x7C, 0x09, 0x98, 0xEA, 0x7B, 0x1C, 0x8D, 0xFF,
        0x6E, 0x1B, 0x8A, 0xF8, 0x69, 0x12, 0x83, 0xF1, 0x60, 0x15, 0x84, 0xF6, 0x67, 0x38, 0xA9, 0xDB, 0x4A, 0x3F, 0xAE,
        0xDC, 0x4D, 0x36, 0xA7, 0xD5, 0x44, 0x31, 0xA0, 0xD2, 0x43, 0x24, 0xB5, 0xC7, 0x56, 0x23, 0xB2, 0xC0, 0x51, 0x2A,
        0xBB, 0xC9, 0x58, 0x2D, 0xBC, 0xCE, 0x5F, 0x70, 0xE1, 0x93, 0x02, 0x77, 0xE6, 0x94, 0x05, 0x7E, 0xEF, 0x9D, 0x0C,
        0x79, 0xE8, 0x9A, 0x0B, 0x6C, 0xFD, 0x8F, 0x1E, 0x6B, 0xFA, 0x88, 0x19, 0x62, 0xF3, 0x81, 0x10, 0x65, 0xF4, 0x86,
        0x17, 0x48, 0xD9, 0xAB, 0x3A, 0x4F, 0xDE, 0xAC, 0x3D, 0x46, 0xD7, 0xA5, 0x34, 0x41, 0xD0, 0xA2, 0x33, 0x54, 0xC5,
        0xB7, 0x26, 0x53, 0xC2, 0xB0, 0x21, 0x5A, 0xCB, 0xB9, 0x28, 0x5D, 0xCC, 0xBE, 0x2F, 0xE0, 0x71, 0x03, 0x92, 0xE7,
        0x76, 0x04, 0x95, 0xEE, 0x7F, 0x0D, 0x9C, 0xE9, 0x78, 0x0A, 0x9B, 0xFC, 0x6D, 0x1F, 0x8E, 0xFB, 0x6A, 0x18, 0x89,
        0xF2, 0x63, 0x11, 0x80, 0xF5, 0x64, 0x16, 0x87, 0xD8, 0x49, 0x3B, 0xAA, 0xDF, 0x4E, 0x3C, 0xAD, 0xD6, 0x47, 0x35,
        0xA4, 0xD1, 0x40, 0x32, 0xA3, 0xC4, 0x55, 0x27, 0xB6, 0xC3, 0x52, 0x20, 0xB1, 0xCA, 0x5B, 0x29, 0xB8, 0xCD, 0x5C,
        0x2E, 0xBF, 0x90, 0x01, 0x73, 0xE2, 0x97, 0x06, 0x74, 0xE5, 0x9E, 0x0F, 0x7D, 0xEC, 0x99, 0x08, 0x7A, 0xEB, 0x8C,
        0x1D, 0x6F, 0xFE, 0x8B, 0x1A, 0x68, 0xF9, 0x82, 0x13, 0x61, 0xF0, 0x85, 0x14, 0x66, 0xF7, 0xA8, 0x39, 0x4B, 0xDA,
        0xAF, 0x3E, 0x4C, 0xDD, 0xA6, 0x37, 0x45, 0xD4, 0xA1, 0x30, 0x42, 0xD3, 0xB4, 0x25, 0x57, 0xC6, 0xB3, 0x22, 0x50,
        0xC1, 0xBA, 0x2B, 0x59, 0xC8, 0xBD, 0x2C, 0x5E, 0xCF,
    };
    #endif
};
