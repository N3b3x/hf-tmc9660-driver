#ifndef TMC9660_PARAM_MODE_TMCL_XMACROS_HPP
#define TMC9660_PARAM_MODE_TMCL_XMACROS_HPP

#include <cstdint>

namespace tmc9660::tmcl {

/**
 * @brief Non-motion parameters in bank 0: communication, I/O, heartbeat, hibernation, loops, auto-start, etc.
 *
 * To persist changes, use STAP after setting RWE parameters.
 */
#define GLOBAL_PARAM_BANK0_LIST \
    X(SERIAL_ADDRESS, 1)                          ///< RS485/UART module address [1…255 odd]. Default: 1. RWE \
    X(SERIAL_HOST_ADDRESS, 2)                     ///< RS485/UART host address [1…255]. Default: 2. RWE \
    X(HEARTBEAT_MONITORING_CONFIG, 3)             ///< 0: DISABLED, 1: UART, 2: SPI, 3: UART+SPI. Default: 0. RWE \
    X(HEARTBEAT_MONITORING_TIMEOUT, 4)            ///< Heartbeat timeout [ms] [1…4294967295]. Default: 100. RWE \
    X(IO_DIRECTION_MASK, 5)                       ///< GPIO direction mask [bit=1→output]. Default: 0. RWE \
    X(IO_INPUT_PULLUP_PULLDOWN_ENABLE_MASK, 6)    ///< GPIO pull resistor enable mask [bit=1→enable]. Default: 0. RWE \
    X(IO_INPUT_PULLUP_PULLDOWN_DIRECTION_MASK, 7) ///< GPIO pull resistor direction mask [bit=1→pull-up]. Default: 0. RWE \
    X(WAKE_PIN_CONTROL_ENABLE, 10)               ///< Enable TMC9660 WAKE pin control. 0: DISABLED, 1: ENABLED. Default: 0. RWE \
    X(GO_TO_TIMEOUT_POWER_DOWN_STATE, 11)        ///< Enter power-down state for predefined time. See PowerDownTimeout. Default: 0. W \
    X(MAIN_LOOPS, 12)                            ///< Main control loops per second. [0…4294967295]. Default: 0. R \
    X(VELOCITY_LOOPS, 14)                        ///< Velocity control loops per second. [0…4294967295]. Default: 0. R \
    X(AUTO_START_ENABLE, 77)                     ///< Automatic TMCL application start on power-up. 0: DISABLED, 1: ENABLED. Default: 1. RWE \
    X(CLEAR_USER_VARIABLES, 85)                  ///< Clear user variables on startup. 0: TRY_LOAD_FROM_STORAGE, 1: CLEAR. Default: 0. RWE

enum class GlobalParamBank0 : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    GLOBAL_PARAM_BANK0_LIST
    #undef X
};

inline const char* to_string(GlobalParamBank0 id) {
    switch(id) {
    #define X(NAME, VALUE) case GlobalParamBank0::NAME: return #NAME;
    GLOBAL_PARAM_BANK0_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef GLOBAL_PARAM_BANK0_LIST

/**
 * @brief User variables (global parameters in bank 2).
 */
#define GLOBAL_PARAM_BANK2_LIST \
    X(USER_VARIABLE_0, 0) \
    X(USER_VARIABLE_1, 1) \
    X(USER_VARIABLE_2, 2) \
    X(USER_VARIABLE_3, 3) \
    X(USER_VARIABLE_4, 4) \
    X(USER_VARIABLE_5, 5) \
    X(USER_VARIABLE_6, 6) \
    X(USER_VARIABLE_7, 7) \
    X(USER_VARIABLE_8, 8) \
    X(USER_VARIABLE_9, 9) \
    X(USER_VARIABLE_10, 10) \
    X(USER_VARIABLE_11, 11) \
    X(USER_VARIABLE_12, 12) \
    X(USER_VARIABLE_13, 13) \
    X(USER_VARIABLE_14, 14) \
    X(USER_VARIABLE_15, 15)

enum class GlobalParamBank2 : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    GLOBAL_PARAM_BANK2_LIST
    #undef X
};

inline const char* to_string(GlobalParamBank2 id) {
    switch(id) {
    #define X(NAME, VALUE) case GlobalParamBank2::NAME: return #NAME;
    GLOBAL_PARAM_BANK2_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef GLOBAL_PARAM_BANK2_LIST

/**
 * @brief Script interrupt configuration (global parameters in bank 3).
 */
#define GLOBAL_PARAM_BANK3_LIST \
    X(TIMER_0_PERIOD, 0)      ///< [ms] 0…2147483647. R/W \
    X(TIMER_1_PERIOD, 1)      ///< [ms] 0…2147483647. R/W \
    X(TIMER_2_PERIOD, 2)      ///< [ms] 0…2147483647. R/W \
    X(STOP_LEFT_TRIGGER_TRANSITION, 10)   ///< Stop left trigger edge: 0: OFF, 1: RISING, 2: FALLING, 3: BOTH. Default: 0. R/W \
    X(STOP_RIGHT_TRIGGER_TRANSITION, 11)  ///< Stop right trigger edge: 0: OFF, 1: RISING, 2: FALLING, 3: BOTH. Default: 0. R/W \
    X(HOME_RIGHT_TRIGGER_TRANSITION, 12)  ///< Home trigger edge: 0: OFF, 1: RISING, 2: FALLING, 3: BOTH. Default: 0. R/W \
    X(INPUT_0_TRIGGER_TRANSITION, 13)     ///< Input 0 trigger edge: 0: OFF, 1: RISING, 2: FALLING, 3: BOTH. Default: 0. R/W \
    X(INPUT_1_TRIGGER_TRANSITION, 14)     ///< Input 1 trigger edge: 0: OFF, 1: RISING, 2: FALLING, 3: BOTH. Default: 0. R/W \
    X(INPUT_2_TRIGGER_TRANSITION, 15)     ///< Input 2 trigger edge: 0: OFF, 1: RISING, 2: FALLING, 3: BOTH. Default: 0. R/W

enum class GlobalParamBank3 : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    GLOBAL_PARAM_BANK3_LIST
    #undef X
};

inline const char* to_string(GlobalParamBank3 id) {
    switch(id) {
    #define X(NAME, VALUE) case GlobalParamBank3::NAME: return #NAME;
    GLOBAL_PARAM_BANK3_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef GLOBAL_PARAM_BANK3_LIST

/**
 * @brief Gate driver configuration parameters (timings, polarities, etc.).
 */
#define GATE_DRIVER_LIST \
    X(PWM_L_OUTPUT_POLARITY, 233)           ///< Low-side PWM output polarity. 0: ACTIVE_HIGH, 1: ACTIVE_LOW. R/W \
    X(PWM_H_OUTPUT_POLARITY, 234)           ///< High-side PWM output polarity. 0: ACTIVE_HIGH, 1: ACTIVE_LOW. R/W \
    X(BREAK_BEFORE_MAKE_TIME_LOW_UVW, 235)  ///< Break-before-make time for low-side UVW [in 8.33ns units]. 0…255. R/W \
    X(BREAK_BEFORE_MAKE_TIME_HIGH_UVW, 236) ///< Break-before-make time for high-side UVW [8.33ns units]. 0…255. R/W \
    X(BREAK_BEFORE_MAKE_TIME_LOW_Y2, 237)   ///< Break-before-make time for low-side Y2 [8.33ns units]. 0…255. R/W \
    X(BREAK_BEFORE_MAKE_TIME_HIGH_Y2, 238)  ///< Break-before-make time for high-side Y2 [8.33ns units]. 0…255. R/W \
    X(PWM_DEAD_TIME_COMPENSATION, 239)      ///< PWM dead-time compensation setting. 0…255. Default: 0. R/W \
    X(PWM_SYMMETRY_SHIFT, 240)              ///< PWM symmetry shift (for duty cycle linearity). 0…255. Default: 0. R/W \
    X(ADC_VDS_CONVERSION_DELAY, 241)        ///< ADC VDS conversion delay [in 8.33ns units]. 0…255. Default: 0. R/W \
    X(VGS_FILTER_CONSTANT, 242)             ///< VGS (gate short) detection filter constant. 0…15. Default: 0. R/W

enum class GateDriver : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    GATE_DRIVER_LIST
    #undef X
};

inline const char* to_string(GateDriver id) {
    switch(id) {
    #define X(NAME, VALUE) case GateDriver::NAME: return #NAME;
    GATE_DRIVER_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef GATE_DRIVER_LIST

/**
 * @brief Overcurrent protection parameters.
 */
#define OVERCURRENT_PROTECTION_LIST \
    X(UVW_LOW_SIDE_ENABLE, 254)              ///< Overcurrent protect enable for UVW low-side. 0: DISABLED, 1: ENABLED. Default: 0. R/W \
    X(UVW_HIGH_SIDE_ENABLE, 255)             ///< Overcurrent protect enable for UVW high-side. 0: DISABLED, 1: ENABLED. Default: 0. R/W \
    X(UVW_LOW_SIDE_THRESHOLD, 258)           ///< UVW low-side overcurrent threshold. See OvercurrentThreshold enum. Default: 0 (63mV). R/W \
    X(UVW_HIGH_SIDE_THRESHOLD, 259)          ///< UVW high-side overcurrent threshold. See OvercurrentThreshold enum. Default: 0 (63mV). R/W \
    X(Y2_LOW_SIDE_THRESHOLD, 260)            ///< Y2 low-side overcurrent threshold. See Y2LowSideNonVdsThreshold enum. Default: 0. R/W \
    X(Y2_HIGH_SIDE_THRESHOLD, 261)           ///< Y2 high-side overcurrent threshold. See OvercurrentThreshold enum. Default: 0 (63mV). R/W \
    X(Y2_LOW_SIDE_VDS_MONITOR_ENABLE, 262)   ///< Y2 low-side VDS monitor enable. 0: USE_NON_VDS_THRESHOLD_LIST, 1: USE_VDS. Default: 0. R/W \
    X(OVERCURRENT_REACTION, 263)             ///< Overcurrent reaction (latched or auto-retry). See OvercurrentTiming enum. Default: 0. R/W

enum class OvercurrentProtection : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    OVERCURRENT_PROTECTION_LIST
    #undef X
};

inline const char* to_string(OvercurrentProtection id) {
    switch(id) {
    #define X(NAME, VALUE) case OvercurrentProtection::NAME: return #NAME;
    OVERCURRENT_PROTECTION_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef OVERCURRENT_PROTECTION_LIST

/**
 * @brief Undervoltage protection parameters.
 */
#define UNDERVOLTAGE_PROTECTION_LIST \
    X(SUPPLY_LEVEL, 250)      ///< Undervoltage protection threshold level (VS undervoltage comparator). See UndervoltageLevel enum. Default: 0. R/W \
    X(UNDERVOLTAGE_REACTION, 251)   ///< Undervoltage protection reaction. 0: DISABLED, 1: ENABLED (shut down). Default: 0. R/W \
    X(UNDERVOLTAGE_WARNING_LEVEL, 252)       ///< Undervoltage warning threshold level. See UndervoltageLevel enum. Default: 0. R/W \
    X(WARNING_TO_SHUTDOWN_TIME, 253)        ///< Delay from undervoltage warning to shutdown [ms]. 0…65535. Default: 0. R/W

enum class UndervoltageProtection : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    UNDERVOLTAGE_PROTECTION_LIST
    #undef X
};

inline const char* to_string(UndervoltageProtection id) {
    switch(id) {
    #define X(NAME, VALUE) case UndervoltageProtection::NAME: return #NAME;
    UNDERVOLTAGE_PROTECTION_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef UNDERVOLTAGE_PROTECTION_LIST

/**
 * @brief VGS (gate short) protection parameters.
 */
#define VGS_SHORT_PROTECTION_LIST \
    X(VGS_LOW_SIDE_REACTION, 264)    ///< VGS low-side reaction (gate short protection). 0: DISABLED, 1: DISABLED_RESET, 2: ENABLED. Default: 0. R/W \
    X(VGS_HIGH_SIDE_REACTION, 265)   ///< VGS high-side reaction. 0: DISABLED, 1: DISABLED_RESET, 2: ENABLED. Default: 0. R/W \
    X(VGS_BLANKING_TIME, 266)        ///< VGS short blanking time. See VgsBlankingTime enum. Default: 0. R/W \
    X(VGS_DEGLITCH_TIME, 267)        ///< VGS short deglitch time. See VgsDeglitchTime enum. Default: 0. R/W

enum class VgsShortProtection : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    VGS_SHORT_PROTECTION_LIST
    #undef X
};

inline const char* to_string(VgsShortProtection id) {
    switch(id) {
    #define X(NAME, VALUE) case VgsShortProtection::NAME: return #NAME;
    VGS_SHORT_PROTECTION_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef VGS_SHORT_PROTECTION_LIST

/**
 * @brief Motor configuration parameters (motor type, PWM settings, etc.).
 */
#define MOTOR_CONFIG_LIST \
    X(MOTOR_TYPE, 0)           ///< Motor type selection. See MotorType enum. Default: 0 (NO_MOTOR). RWE \
    X(MOTOR_POLE_PAIRS, 1)     ///< Number of pole pairs for motor [0-127]. Default: 1. RWE \
    X(MOTOR_DIRECTION, 2)      ///< Motor direction. 0: NOT_INVERTED, 1: INVERTED. Default: 0. RWE \
    X(MOTOR_PWM_FREQUENCY, 3)  ///< PWM frequency in Hz [10000-100000]. Default: 25000. RWE \
    X(COMMUTATION_MODE, 4)     ///< Motor commutation mode. See CommutationMode enum. Default: 0 (SYSTEM_OFF). R/W \
    X(OUTPUT_VOLTAGE_LIMIT, 5) ///< PID UQ/UD output limit (circular limiter) [0-32767]. Default: 8000. RWE \
    X(PWM_SWITCHING_SCHEME, 8) ///< PWM switching scheme. See PwmSwitchingScheme enum. Default: 1 (SVPWM). RWE

enum class MotorConfig : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    MOTOR_CONFIG_LIST
    #undef X
};

inline const char* to_string(MotorConfig id) {
    switch(id) {
    #define X(NAME, VALUE) case MotorConfig::NAME: return #NAME;
    MOTOR_CONFIG_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef MOTOR_CONFIG_LIST

/**
 * @brief ADC configuration parameters.
 */
#define ADC_CONFIG_LIST \
    X(ADC_I0_TO_VS_VOLTAGE_RATIO, 16)   ///< ADC I0-to-VS voltage ratio. [0…4095]. Default: 0. R/W \
    X(ADC_I0_OFFSET, 17)                ///< ADC I0 offset calibration. [0…4095]. Default: 0. R/W \
    X(ADC_I1_TO_VS_VOLTAGE_RATIO, 18)   ///< ADC I1-to-VS voltage ratio. [0…4095]. Default: 0. R/W \
    X(ADC_I1_OFFSET, 19)                ///< ADC I1 offset calibration. [0…4095]. Default: 0. R/W \
    X(ADC_I2_TO_VS_VOLTAGE_RATIO, 20)   ///< ADC I2-to-VS voltage ratio. [0…4095]. Default: 0. R/W \
    X(ADC_I2_OFFSET, 21)                ///< ADC I2 offset calibration. [0…4095]. Default: 0. R/W \
    X(ADC_VBAT_OFFSET, 22)              ///< ADC VBAT (battery voltage) offset calibration. [0…4095]. Default: 0. R/W \
    X(ADC_VDS_OFFSET, 23)               ///< ADC VDS offset calibration. [0…4095]. Default: 0. R/W

enum class AdcConfig : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    ADC_CONFIG_LIST
    #undef X
};

inline const char* to_string(AdcConfig id) {
    switch(id) {
    #define X(NAME, VALUE) case AdcConfig::NAME: return #NAME;
    ADC_CONFIG_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef ADC_CONFIG_LIST

/**
 * @brief Feedback sensor configuration parameters (Hall/ABN encoder/SPI encoder settings).
 */
#define FEEDBACK_SENSOR_CONFIG_LIST \
    X(HALL_SENSOR_POLARITY, 24)      ///< Hall sensor polarity. 0: STANDARD, 1: INVERTED. Default: 0. R/W \
    X(ABN_ENCODER_RESOLUTION, 25)    ///< ABN encoder resolution (ticks per rotation). [0…16777215]. Default: 0. R/W \
    X(ABN_ENCODER_DIRECTION, 26)     ///< ABN encoder direction inversion. 0: STANDARD, 1: INVERTED. Default: 0. R/W \
    X(ABN_ENCODER_INIT_MODE, 27)     ///< ABN encoder initialization mode. See AbnInitMethod enum. Default: 0. R/W \
    X(ABN_ENCODER_INIT_STATE, 28)    ///< ABN encoder initialization state. See AbnInitState enum. Default: 0. R/W \
    X(ABN_ENCODER_N_CHANNEL_FILTER, 29) ///< ABN encoder N channel filtering. See AbnNChannelFiltering enum. Default: 0. R/W \
    X(HALL_SENSOR_SECTOR_OFFSET, 30) ///< Hall sensor sector offset. See HallSectorOffset enum. Default: 0. R/W \
    X(SPI_ENCODER_DATA_CONFIG, 31)   ///< SPI encoder data transfer config. See SpiEncoderTransfer enum. Default: 0. R/W \
    X(SPI_ENCODER_INIT_MODE, 32)     ///< SPI encoder initialization mode. See SpiInitMethod enum. Default: 0. R/W

enum class FeedbackSensorConfig : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    FEEDBACK_SENSOR_CONFIG_LIST
    #undef X
};

inline const char* to_string(FeedbackSensorConfig id) {
    switch(id) {
    #define X(NAME, VALUE) case FeedbackSensorConfig::NAME: return #NAME;
    FEEDBACK_SENSOR_CONFIG_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef FEEDBACK_SENSOR_CONFIG_LIST

/**
 * @brief Torque and flux control parameters (limits, setpoints, PI gains).
 */
#define TORQUE_FLUX_CONTROL_LIST \
    X(MAX_TORQUE, 6)           ///< Maximum motor torque [mA]. 0…65535. Default: 2000. RWE \
    X(MAX_FLUX, 7)             ///< Maximum motor flux [mA]. 0…65535. Default: 2000. RWE \
    X(TARGET_TORQUE, 104)      ///< Target torque [mA]. Writing activates torque control. -32768…32767. Default: 0. R/W \
    X(ACTUAL_TORQUE, 105)      ///< Actual motor torque [mA]. -32768…32767 (signed). Default: 0. R \
    X(TARGET_FLUX, 106)        ///< Target flux [mA]. -10000…10000. Default: 0. R/W \
    X(ACTUAL_FLUX, 107)        ///< Actual motor flux [mA]. -2147483648…2147483647. Default: 0. R \
    X(TORQUE_OFFSET, 108)      ///< Torque offset [mA]. -4700…4700. Default: 0. R/W \
    X(TORQUE_P, 109)           ///< Torque PI controller P parameter. 0…32767. Default: 32767. R/W \
    X(TORQUE_I, 110)           ///< Torque PI controller I parameter. 0…32767. Default: 32767. R/W \
    X(FLUX_P, 111)             ///< Flux PI controller P parameter. 0…32767. Default: 32767. R/W \
    X(FLUX_I, 112)             ///< Flux PI controller I parameter. 0…32767. Default: 32767. R/W \
    X(FIELDWEAKENING_I, 308)   ///< I parameter for field weakening. 0…32767. Default: 32767. R/W

enum class TorqueFluxControl : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    TORQUE_FLUX_CONTROL_LIST
    #undef X
};

inline const char* to_string(TorqueFluxControl id) {
    switch(id) {
    #define X(NAME, VALUE) case TorqueFluxControl::NAME: return #NAME;
    TORQUE_FLUX_CONTROL_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef TORQUE_FLUX_CONTROL_LIST

/**
 * @brief Velocity control parameters (setpoints, measured value, PI gains, feed-forward, etc.).
 */
#define VELOCITY_CONTROL_LIST \
    X(TARGET_VELOCITY, 100)        ///< Target velocity [tick/s]. Default: 0. R/W \
    X(ACTUAL_VELOCITY, 101)        ///< Actual velocity [tick/s]. Default: 0. R \
    X(VELOCITY_MEASUREMENT_SELECTION, 102) ///< Velocity measurement source selection. See VelocitySensorSelection enum. Default: 0. R/W \
    X(VELOCITY_P, 103)            ///< Velocity PI controller P parameter. 0…32767. Default: 32767. R/W \
    X(VELOCITY_I, 104)            ///< Velocity PI controller I parameter. 0…32767. Default: 32767. R/W \
    X(VELOCITY_FEEDFORWARD_ENABLE, 105) ///< Enable velocity feed-forward. 0: DISABLED, 1: ENABLED. Default: 0. R/W \
    X(VELOCITY_METER_MODE, 106)   ///< Velocity meter mode. See VelocityMeterMode enum. Default: 0. R/W \
    X(ACCELERATION_FEEDFORWARD_SHIFT, 107) ///< Acceleration feed-forward divisor (shift value). 0…15. Default: 0. R/W

enum class VelocityControl : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    VELOCITY_CONTROL_LIST
    #undef X
};

inline const char* to_string(VelocityControl id) {
    switch(id) {
    #define X(NAME, VALUE) case VelocityControl::NAME: return #NAME;
    VELOCITY_CONTROL_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef VELOCITY_CONTROL_LIST

/**
 * @brief Position control parameters (setpoints, actual position, PI gains, ramp behavior).
 */
#define POSITION_CONTROL_LIST \
    X(TARGET_POSITION, 108)         ///< Target position [ticks]. Default: 0. R/W \
    X(ACTUAL_POSITION, 109)         ///< Actual position [ticks]. Default: 0. R \
    X(POSITION_P, 110)             ///< Position PI controller P parameter. 0…32767. Default: 32767. R/W \
    X(POSITION_I, 111)             ///< Position PI controller I parameter. 0…32767. Default: 32767. R/W \
    X(POSITION_SENSOR_SELECTION, 112) ///< Position sensor selection. See PositionSensorSelection enum. Default: 0. R/W \
    X(POSITION_FEEDFORWARD_ENABLE, 113) ///< Enable position feed-forward (normalization). See PositionPiNorm enum. Default: 0. R/W \
    X(STOP_ON_STALL_ENABLE, 114)       ///< Enable stop-on-stall. 0: DISABLED, 1: ENABLED. Default: 0. R/W \
    X(RAMP_STOP_STATE_CONFIG, 115)     ///< Ramp stop switch behavior configuration. See RamperStopConfig enum. Default: 0. R/W

enum class PositionControl : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    POSITION_CONTROL_LIST
    #undef X
};

inline const char* to_string(PositionControl id) {
    switch(id) {
    #define X(NAME, VALUE) case PositionControl::NAME: return #NAME;
    POSITION_CONTROL_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef POSITION_CONTROL_LIST

/**
 * @brief Ramp and reference switch configuration parameters.
 */
#define RAMPER_STOP_CONFIG_LIST \
    X(RAMP_STOP_REFERENCE_MODE, 116)  ///< Reference switch stop behavior. 0: NONE, 1: HARD_STOP, 2: SOFT_STOP. Default: 0. R/W

enum class RamperStopConfig : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    RAMPER_STOP_CONFIG_LIST
    #undef X
};

inline const char* to_string(RamperStopConfig id) {
    switch(id) {
    #define X(NAME, VALUE) case RamperStopConfig::NAME: return #NAME;
    RAMPER_STOP_CONFIG_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef RAMPER_STOP_CONFIG_LIST

/**
 * @brief Biquad filter parameters (user-programmable filter coefficients).
 */
#define BIQUAD_FILTER_LIST \
    X(BIQUAD_0_A1, 144) ///< Biquad filter 0 coefficient A1. Default: 0. R/W \
    X(BIQUAD_0_A2, 145) ///< Biquad filter 0 coefficient A2. Default: 0. R/W \
    X(BIQUAD_0_B0, 146) ///< Biquad filter 0 coefficient B0. Default: 0. R/W \
    X(BIQUAD_0_B1, 147) ///< Biquad filter 0 coefficient B1. Default: 0. R/W \
    X(BIQUAD_0_B2, 148) ///< Biquad filter 0 coefficient B2. Default: 0. R/W \
    X(BIQUAD_1_A1, 149) ///< Biquad filter 1 coefficient A1. Default: 0. R/W \
    X(BIQUAD_1_A2, 150) ///< Biquad filter 1 coefficient A2. Default: 0. R/W \
    X(BIQUAD_1_B0, 151) ///< Biquad filter 1 coefficient B0. Default: 0. R/W \
    X(BIQUAD_1_B1, 152) ///< Biquad filter 1 coefficient B1. Default: 0. R/W \
    X(BIQUAD_1_B2, 153) ///< Biquad filter 1 coefficient B2. Default: 0. R/W

enum class BiquadFilter : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    BIQUAD_FILTER_LIST
    #undef X
};

inline const char* to_string(BiquadFilter id) {
    switch(id) {
    #define X(NAME, VALUE) case BiquadFilter::NAME: return #NAME;
    BIQUAD_FILTER_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef BIQUAD_FILTER_LIST

/**
 * @brief Fault handling configuration parameters.
 */
#define FAULT_HANDLING_LIST \
    X(FAULT_REACTION, 160)          ///< General fault reaction (safe state). 0: DISABLE_PWM, 1: DISABLE_SYSTEM, 2: NOP (ignore). Default: 0. R/W \
    X(GATE_DRIVER_RETRY, 161)       ///< Gate driver retry behavior. See GdrvRetryBehaviour enum. Default: 0. R/W \
    X(DRIVE_FAULT_BEHAVIOR, 162)    ///< Drive fault (error) behavior. See DriveFaultBehaviour enum. Default: 0. R/W

enum class FaultHandling : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    FAULT_HANDLING_LIST
    #undef X
};

inline const char* to_string(FaultHandling id) {
    switch(id) {
    #define X(NAME, VALUE) case FaultHandling::NAME: return #NAME;
    FAULT_HANDLING_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef FAULT_HANDLING_LIST

/**
 * @brief I²t (IIT) monitoring parameters.
 */
#define IIT_MONITOR_LIST \
    X(IIT_MAX_CURRENT, 163)      ///< I²t monitoring: maximum RMS current [mA]. 0…65535. Default: 0 (disabled). R/W \
    X(IIT_TIME_CONSTANT, 164)    ///< I²t time constant [ms]. 0…65535. Default: 0 (use default). R/W

enum class IitMonitor : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    IIT_MONITOR_LIST
    #undef X
};

inline const char* to_string(IitMonitor id) {
    switch(id) {
    #define X(NAME, VALUE) case IitMonitor::NAME: return #NAME;
    IIT_MONITOR_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef IIT_MONITOR_LIST

/**
 * @brief Temperature protection (overtemperature thresholds and readings).
 */
#define TEMPERATURE_PROTECTION_LIST \
    X(EXTERNAL_TEMPERATURE, 293)                  ///< External temperature sensor value [0…65535]. Read-only. \
    X(EXTERNAL_TEMPERATURE_SHUTDOWN_THRESHOLD, 294) ///< External temperature shutdown threshold [0…65535]. Default: 65535. RWE \
    X(EXTERNAL_TEMPERATURE_WARNING_THRESHOLD, 295)  ///< External temperature warning threshold [0…65535]. Default: 65535. RWE \
    X(CHIP_TEMPERATURE, 296)                      ///< Internal chip temperature value [0…65535]. Read-only. \
    X(CHIP_TEMPERATURE_SHUTDOWN_THRESHOLD, 297)   ///< Chip temperature shutdown threshold [0…65535]. Default: 65535. RWE \
    X(CHIP_TEMPERATURE_WARNING_THRESHOLD, 298)    ///< Chip temperature warning threshold [0…65535]. Default: 65535. RWE

enum class TemperatureProtection : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    TEMPERATURE_PROTECTION_LIST
    #undef X
};

inline const char* to_string(TemperatureProtection id) {
    switch(id) {
    #define X(NAME, VALUE) case TemperatureProtection::NAME: return #NAME;
    TEMPERATURE_PROTECTION_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef TEMPERATURE_PROTECTION_LIST

/**
 * @brief Heartbeat monitoring parameters.
 * @note These map to Bank0 global parameters 3 and 4.
 */
#define HEARTBEAT_MONITORING_LIST \
    X(HEARTBEAT_MONITORING_CONFIG, 3)  ///< Heartbeat monitoring configuration. 0: DISABLED, 1: UART, 2: SPI, 3: UART+SPI. Default: 0. RWE \
    X(HEARTBEAT_MONITORING_TIMEOUT, 4) ///< Heartbeat monitoring timeout [ms]. Default: 100. RWE

enum class HeartbeatMonitoring : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    HEARTBEAT_MONITORING_LIST
    #undef X
};

inline const char* to_string(HeartbeatMonitoring id) {
    switch(id) {
    #define X(NAME, VALUE) case HeartbeatMonitoring::NAME: return #NAME;
    HEARTBEAT_MONITORING_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef HEARTBEAT_MONITORING_LIST

/**
 * @brief Brake chopper (energy dissipator) parameters.
 */
#define BRAKE_CHOPPER_LIST \
    X(BRAKE_CHOPPER_ENABLE, 168)      ///< Enable brake chopper. 0: DISABLED, 1: ENABLED. Default: 0. R/W \
    X(BRAKE_CHOPPER_VOLTAGE_THRESHOLD, 169) ///< Brake chopper activation threshold [mV]. Default: 0. R/W

enum class BrakeChopper : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    BRAKE_CHOPPER_LIST
    #undef X
};

inline const char* to_string(BrakeChopper id) {
    switch(id) {
    #define X(NAME, VALUE) case BrakeChopper::NAME: return #NAME;
    BRAKE_CHOPPER_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef BRAKE_CHOPPER_LIST

/**
 * @brief Mechanical brake control parameters.
 */
#define MECHANICAL_BRAKE_LIST \
    X(BRAKE_ENABLE, 170)           ///< Enable mechanical brake control. 0: DISABLED, 1: ENABLED. Default: 0. R/W \
    X(BRAKE_RELEASE_DELAY, 171)    ///< Brake release delay [ms]. Default: 0. R/W \
    X(BRAKE_ENGAGE_DELAY, 172)     ///< Brake engage delay [ms]. Default: 0. R/W

enum class MechanicalBrake : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    MECHANICAL_BRAKE_LIST
    #undef X
};

inline const char* to_string(MechanicalBrake id) {
    switch(id) {
    #define X(NAME, VALUE) case MechanicalBrake::NAME: return #NAME;
    MECHANICAL_BRAKE_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef MECHANICAL_BRAKE_LIST

/**
 * @brief Reference search (homing) parameters.
 */
#define REFERENCE_SEARCH_LIST \
    X(REFERENCE_SWITCH_SEARCH_MODE, 165)   ///< Reference search mode. See ReferenceSearchMode enum. Default: 0. RWE \
    X(REFERENCE_SWITCH_SEARCH_SPEED, 166)  ///< Reference search speed (coarse) [-134217728…134217727]. Default: 0. RWE \
    X(REFERENCE_SWITCH_SPEED, 167)         ///< Reference lower speed for precise positioning [-134217728…134217727]. Default: 0. RWE \
    X(RIGHT_LIMIT_SWITCH_POSITION, 168)    ///< Position captured at right limit switch [-2147483648…2147483647]. Default: 0. R \
    X(HOME_SWITCH_POSITION, 169)           ///< Position captured at home switch [-2147483648…2147483647]. Default: 0. R \
    X(LAST_REFERENCE_POSITION, 170)        ///< Last referenced position (after homing) [-2147483648…2147483647]. Default: 0. R

enum class ReferenceSearch : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    REFERENCE_SEARCH_LIST
    #undef X
};

inline const char* to_string(ReferenceSearch id) {
    switch(id) {
    #define X(NAME, VALUE) case ReferenceSearch::NAME: return #NAME;
    REFERENCE_SEARCH_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef REFERENCE_SEARCH_LIST

/**
 * @brief Step/Dir interface parameters.
 */
#define STEP_DIR_LIST \
    X(STEP_DIRECTION_MODE, 176)           ///< Step/Dir mode (microstep resolution etc). Default: 0. R/W \
    X(STEP_PULSE_DIVIDER, 177)           ///< Step pulse divisor (frequency scaling). Default: 0. R/W \
    X(DIRECTION_FILT_ENABLE, 178)        ///< Direction change filtering enable. 0: DISABLED, 1: ENABLED. Default: 0. R/W \
    X(VELOCITY_FEEDFORWARD_ENABLE, 105)  ///< Duplicate of VelocityControl VELOCITY_FEEDFORWARD_ENABLE for Step/Dir mode. R/W

enum class StepDir : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    STEP_DIR_LIST
    #undef X
};

inline const char* to_string(StepDir id) {
    switch(id) {
    #define X(NAME, VALUE) case StepDir::NAME: return #NAME;
    STEP_DIR_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef STEP_DIR_LIST

/**
 * @brief Hibernation and wake-up parameters.
 * @note These map to Bank0 global parameters 10 and 11.
 */
#define HIBERNATION_WAKEUP_LIST \
    X(WAKE_PIN_CONTROL_ENABLE, 10)         ///< Enable WAKE pin control (hibernation). 0: DISABLED, 1: ENABLED. Default: 0. RWE \
    X(GO_TO_TIMEOUT_POWER_DOWN_STATE, 11)  ///< Enter power-down state for a predefined time. See PowerDownTimeout enum. Default: 0. W

enum class HibernationWakeup : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    HIBERNATION_WAKEUP_LIST
    #undef X
};

inline const char* to_string(HibernationWakeup id) {
    switch(id) {
    #define X(NAME, VALUE) case HibernationWakeup::NAME: return #NAME;
    HIBERNATION_WAKEUP_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef HIBERNATION_WAKEUP_LIST

/**
 * @brief System status supply readings (voltage, etc.).
 */
#define SYSTEM_STATUS_SUPPLY_LIST \
    X(VBAT_VOLTAGE, 176)       ///< Battery supply voltage [mV]. Read-only. \
    X(VREG_VOLTAGE, 177)       ///< Regulator (VREG) voltage [mV]. Read-only. \
    X(TEMP_ANALOG_VOLTAGE, 178) ///< Analog temperature sensor voltage [mV]. Read-only.

enum class SystemStatusSupply : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    SYSTEM_STATUS_SUPPLY_LIST
    #undef X
};

inline const char* to_string(SystemStatusSupply id) {
    switch(id) {
    #define X(NAME, VALUE) case SystemStatusSupply::NAME: return #NAME;
    SYSTEM_STATUS_SUPPLY_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef SYSTEM_STATUS_SUPPLY_LIST

/**
 * @brief Internal measurement values.
 */
#define INTERNAL_MEASUREMENT_LIST \
    X(DRIVE_FET_VOLTAGE_UVW, 179)    ///< Drive FET voltage UVW [mV]. Read-only. \
    X(DRIVE_FET_VOLTAGE_Y2, 180)     ///< Drive FET voltage Y2 [mV]. Read-only. \
    X(LOW_SIDE_SHUNT_VOLTAGE, 181)   ///< Low-side shunt voltage [mV]. Read-only. \
    X(HIGH_SIDE_SHUNT_VOLTAGE, 182)  ///< High-side shunt voltage [mV]. Read-only. \
    X(PHASE_U_CURRENT, 183)          ///< Phase U current [mA]. Read-only. \
    X(PHASE_V_CURRENT, 184)          ///< Phase V current [mA]. Read-only. \
    X(PHASE_W_CURRENT, 185)          ///< Phase W current [mA]. Read-only. \
    X(PHASE_Y2_CURRENT, 186)         ///< Phase Y2 current [mA]. Read-only. \
    X(LOW_SIDE_VDS_STATUS_MASK, 187) ///< Low-side MOSFET VDS status flags (overcurrent). Read-only. \
    X(HIGH_SIDE_VDS_STATUS_MASK, 188)///< High-side MOSFET VDS status flags. Read-only. \
    X(DRIVER_ERROR_FLAGS, 189)       ///< Gate driver error flags. See GateDriverErrorFlags enum. Read-only. \
    X(DRIVER_STATUS_FLAGS, 190)      ///< Gate driver status flags. See AdcStatusFlags enum. Read-only. \
    X(FIELDWEAKENING_VOLTAGE_THRESHOLD, 309) ///< Field weakening voltage threshold. [mV]. Default: 0. R/W

enum class InternalMeasurement : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    INTERNAL_MEASUREMENT_LIST
    #undef X
};

inline const char* to_string(InternalMeasurement id) {
    switch(id) {
    #define X(NAME, VALUE) case InternalMeasurement::NAME: return #NAME;
    INTERNAL_MEASUREMENT_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef INTERNAL_MEASUREMENT_LIST

/**
 * @brief Combined diagnostic values (packed 32-bit readings for tuning).
 */
#define COMBINED_DIAGNOSTIC_VALUES_LIST \
    X(TORQUE_FLUX_COMBINED_TARGET_VALUES, 330) ///< Combined raw torque/flux target values (32-bit). Read-only. \
    X(TORQUE_FLUX_COMBINED_ACTUAL_VALUES, 331) ///< Combined raw torque/flux actual values (32-bit). Read-only. \
    X(VOLTAGE_D_Q_COMBINED_ACTUAL_VALUES, 332) ///< Combined raw D/Q voltage actual values (32-bit). Read-only. \
    X(INTEGRATED_ACTUAL_TORQUE_VALUE, 333)     ///< Integrated (summed) actual torque value (for low-frequency measurement). Read-only. \
    X(INTEGRATED_ACTUAL_VELOCITY_VALUE, 334)   ///< Integrated (summed) actual velocity value. Read-only.

enum class CombinedDiagnosticValues : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    COMBINED_DIAGNOSTIC_VALUES_LIST
    #undef X
};

inline const char* to_string(CombinedDiagnosticValues id) {
    switch(id) {
    #define X(NAME, VALUE) case CombinedDiagnosticValues::NAME: return #NAME;
    COMBINED_DIAGNOSTIC_VALUES_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef COMBINED_DIAGNOSTIC_VALUES_LIST

/**
 * @brief Errors and flags status parameters.
 */
#define ERRORS_AND_FLAGS_LIST \
    X(GENERAL_STATUS_FLAGS, 289)             ///< General status flags (see GeneralStatusFlags bits). Read-only. \
    X(EXTERNAL_TEMPERATURE, 293)            ///< External temperature sensor reading [0…65535]. Read-only. \
    X(EXTERNAL_TEMPERATURE_SHUTDOWN_THRESHOLD, 294) ///< External temperature shutdown threshold [0…65535]. Read-only. \
    X(EXTERNAL_TEMPERATURE_WARNING_THRESHOLD, 295)  ///< External temperature warning threshold [0…65535]. Read-only. \
    X(CHIP_TEMPERATURE, 296)                ///< Chip temperature reading [0…65535]. Read-only. \
    X(CHIP_TEMPERATURE_SHUTDOWN_THRESHOLD, 297)     ///< Chip temperature shutdown threshold [0…65535]. Read-only. \
    X(CHIP_TEMPERATURE_WARNING_THRESHOLD, 298)      ///< Chip temperature warning threshold [0…65535]. Read-only. \
    X(GENERAL_ERROR_FLAGS, 299)             ///< General error flags (see GeneralErrorFlags bits). Read-only. \
    X(GATE_DRIVER_ERROR_FLAGS, 300)         ///< Gate driver error flags (see GateDriverErrorFlags bits). Read-only. \
    X(ADC_STATUS_FLAGS, 301)               ///< ADC/status flags (see AdcStatusFlags bits). Read-only.

enum class ErrorsAndFlags : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    ERRORS_AND_FLAGS_LIST
    #undef X
};

inline const char* to_string(ErrorsAndFlags id) {
    switch(id) {
    #define X(NAME, VALUE) case ErrorsAndFlags::NAME: return #NAME;
    ERRORS_AND_FLAGS_LIST
    #undef X
    default: return "UNKNOWN";
    }
}
#undef ERRORS_AND_FLAGS_LIST

// Master parameter list (all parameter IDs combined)
#define PARAM_LIST \
    X(USER_VARIABLE_0, 0) \
    X(TIMER_0_PERIOD, 0)                            ///< [ms] 0…2147483647. R/W \
    X(MOTOR_TYPE, 0)                                ///< Motor type selection. See MotorType enum. Default: 0 (NO_MOTOR). RWE \
    X(SERIAL_ADDRESS, 1)                            ///< RS485/UART module address [1…255 odd]. Default: 1. RWE \
    X(USER_VARIABLE_1, 1) \
    X(SERIAL_HOST_ADDRESS, 2)                       ///< RS485/UART host address [1…255]. Default: 2. RWE \
    X(USER_VARIABLE_2, 2) \
    X(USER_VARIABLE_3, 3) \
    X(HEARTBEAT_MONITORING_CONFIG, 3)               ///< 0: DISABLED, 1: UART, 2: SPI, 3: UART+SPI. Default: 0. RWE \
    X(USER_VARIABLE_4, 4) \
    X(HEARTBEAT_MONITORING_TIMEOUT, 4)              ///< Heartbeat timeout [ms] [1…4294967295]. Default: 100. RWE \
    X(USER_VARIABLE_5, 5) \
    X(IO_DIRECTION_MASK, 5)                         ///< GPIO direction mask [bit=1→output]. Default: 0. RWE \
    X(USER_VARIABLE_6, 6) \
    X(IO_INPUT_PULLUP_PULLDOWN_ENABLE_MASK, 6)      ///< GPIO pull resistor enable mask [bit=1→enable]. Default: 0. RWE \
    X(USER_VARIABLE_7, 7) \
    X(IO_INPUT_PULLUP_PULLDOWN_DIRECTION_MASK, 7)   ///< GPIO pull resistor direction mask [bit=1→pull-up]. Default: 0. RWE \
    X(MAX_TORQUE, 6)                                ///< Maximum motor torque [mA]. 0…65535. Default: 2000. RWE \
    X(MAX_FLUX, 7)                                  ///< Maximum motor flux [mA]. 0…65535. Default: 2000. RWE \
    X(WAKE_PIN_CONTROL_ENABLE, 10)                  ///< Enable TMC9660 WAKE pin control. 0: DISABLED, 1: ENABLED. Default: 0. RWE \
    X(GO_TO_TIMEOUT_POWER_DOWN_STATE, 11)           ///< Enter power-down state for predefined time. See PowerDownTimeout. Default: 0. W \
    X(TARGET_VELOCITY, 100)                         ///< Target velocity [tick/s]. Default: 0. R/W \
    X(ACTUAL_VELOCITY, 101)                         ///< Actual velocity [tick/s]. Default: 0. R \
    X(VELOCITY_MEASUREMENT_SELECTION, 102)          ///< Velocity measurement source selection. Default: 0. R/W \
    X(VELOCITY_P, 103)                              ///< Velocity PI controller P parameter. Default: 32767. R/W \
    X(VELOCITY_I, 104)                              ///< Velocity PI controller I parameter. Default: 32767. R/W \
    X(VELOCITY_FEEDFORWARD_ENABLE, 105)             ///< Enable velocity feed-forward. 0: DISABLED, 1: ENABLED. Default: 0. R/W \
    X(VELOCITY_METER_MODE, 106)                     ///< Velocity meter mode. Default: 0. R/W \
    X(ACCELERATION_FEEDFORWARD_SHIFT, 107)          ///< Acceleration feed-forward shift. Default: 0. R/W \
    X(TARGET_POSITION, 108)                         ///< Target position [ticks]. Default: 0. R/W \
    X(ACTUAL_POSITION, 109)                         ///< Actual position [ticks]. Default: 0. R \
    X(POSITION_P, 110)                              ///< Position PI controller P parameter. Default: 32767. R/W \
    X(POSITION_I, 111)                              ///< Position PI controller I parameter. Default: 32767. R/W \
    X(POSITION_SENSOR_SELECTION, 112)               ///< Position sensor selection. Default: 0. R/W \
    X(POSITION_FEEDFORWARD_ENABLE, 113)             ///< Enable position feed-forward (normalization). Default: 0. R/W \
    X(STOP_ON_STALL_ENABLE, 114)                    ///< Enable stop-on-stall. 0: DISABLED, 1: ENABLED. Default: 0. R/W \
    X(RAMP_STOP_REFERENCE_MODE, 116)                ///< Reference switch stop behavior. Default: 0. R/W \
    X(BIQUAD_0_A1, 144)                             ///< Biquad filter 0 coefficient A1. Default: 0. R/W \
    X(BIQUAD_0_A2, 145)                             ///< Biquad filter 0 coefficient A2. Default: 0. R/W \
    X(BIQUAD_0_B0, 146)                             ///< Biquad filter 0 coefficient B0. Default: 0. R/W \
    X(BIQUAD_0_B1, 147)                             ///< Biquad filter 0 coefficient B1. Default: 0. R/W \
    X(BIQUAD_0_B2, 148)                             ///< Biquad filter 0 coefficient B2. Default: 0. R/W \
    X(BIQUAD_1_A1, 149)                             ///< Biquad filter 1 coefficient A1. Default: 0. R/W \
    X(BIQUAD_1_A2, 150)                             ///< Biquad filter 1 coefficient A2. Default: 0. R/W \
    X(BIQUAD_1_B0, 151)                             ///< Biquad filter 1 coefficient B0. Default: 0. R/W \
    X(BIQUAD_1_B1, 152)                             ///< Biquad filter 1 coefficient B1. Default: 0. R/W \
    X(BIQUAD_1_B2, 153)                             ///< Biquad filter 1 coefficient B2. Default: 0. R/W \
    X(FAULT_REACTION, 160)                          ///< General fault reaction (safe state). Default: 0. R/W \
    X(GATE_DRIVER_RETRY, 161)                       ///< Gate driver retry behavior. Default: 0. R/W \
    X(DRIVE_FAULT_BEHAVIOR, 162)                    ///< Drive fault (error) behavior. Default: 0. R/W \
    X(IIT_MAX_CURRENT, 163)                         ///< I²t max RMS current [mA]. Default: 0 (disabled). R/W \
    X(IIT_TIME_CONSTANT, 164)                       ///< I²t time constant [ms]. Default: 0 (use default). R/W \
    X(REFERENCE_SWITCH_SEARCH_MODE, 165)            ///< Reference search mode. Default: 0. RWE \
    X(REFERENCE_SWITCH_SEARCH_SPEED, 166)           ///< Reference search speed (coarse). Default: 0. RWE \
    X(REFERENCE_SWITCH_SPEED, 167)                  ///< Reference search lower speed. Default: 0. RWE \
    X(BRAKE_CHOPPER_ENABLE, 168)                    ///< Enable brake chopper. Default: 0. R/W \
    X(RIGHT_LIMIT_SWITCH_POSITION, 168)             ///< Position at right limit switch. Default: 0. R \
    X(HOME_SWITCH_POSITION, 169)                    ///< Position at home switch. Default: 0. R \
    X(BRAKE_CHOPPER_VOLTAGE_THRESHOLD, 169)         ///< Brake chopper threshold [mV]. Default: 0. R/W \
    X(LAST_REFERENCE_POSITION, 170)                 ///< Last referenced position. Default: 0. R \
    X(BRAKE_ENABLE, 170)                            ///< Enable mechanical brake. Default: 0. R/W \
    X(WAKE_PIN_CONTROL_ENABLE, 10)                  ///< (duplicate) WAKE pin control enable (hibernation). Default: 0. RWE \
    X(BRAKE_RELEASE_DELAY, 171)                     ///< Brake release delay [ms]. Default: 0. R/W \
    X(BRAKE_ENGAGE_DELAY, 172)                      ///< Brake engage delay [ms]. Default: 0. R/W \
    X(VBAT_VOLTAGE, 176)                            ///< Battery supply voltage [mV]. Read-only. \
    X(STEP_DIRECTION_MODE, 176)                     ///< Step/Dir mode (microstep resolution). Default: 0. R/W \
    X(VREG_VOLTAGE, 177)                            ///< VREG (regulator) voltage [mV]. Read-only. \
    X(STEP_PULSE_DIVIDER, 177)                      ///< Step pulse divider. Default: 0. R/W \
    X(TEMP_ANALOG_VOLTAGE, 178)                     ///< Analog temperature sensor voltage [mV]. Read-only. \
    X(DIRECTION_FILT_ENABLE, 178)                   ///< Direction change filter enable. Default: 0. R/W \
    X(DRIVE_FET_VOLTAGE_UVW, 179)                   ///< Drive FET UVW voltage [mV]. Read-only. \
    X(DRIVE_FET_VOLTAGE_Y2, 180)                    ///< Drive FET Y2 voltage [mV]. Read-only. \
    X(LOW_SIDE_SHUNT_VOLTAGE, 181)                  ///< Low-side shunt voltage [mV]. Read-only. \
    X(HIGH_SIDE_SHUNT_VOLTAGE, 182)                 ///< High-side shunt voltage [mV]. Read-only. \
    X(PHASE_U_CURRENT, 183)                         ///< Phase U current [mA]. Read-only. \
    X(PHASE_V_CURRENT, 184)                         ///< Phase V current [mA]. Read-only. \
    X(PHASE_W_CURRENT, 185)                         ///< Phase W current [mA]. Read-only. \
    X(PHASE_Y2_CURRENT, 186)                        ///< Phase Y2 current [mA]. Read-only. \
    X(LOW_SIDE_VDS_STATUS_MASK, 187)                ///< Low-side VDS status flags. Read-only. \
    X(HIGH_SIDE_VDS_STATUS_MASK, 188)               ///< High-side VDS status flags. Read-only. \
    X(DRIVER_ERROR_FLAGS, 189)                      ///< Gate driver error flags. Read-only. \
    X(DRIVER_STATUS_FLAGS, 190)                     ///< Gate driver status flags. Read-only. \
    X(PWM_L_OUTPUT_POLARITY, 233)                   ///< PWM low-side output polarity. Default: 0. R/W \
    X(PWM_H_OUTPUT_POLARITY, 234)                   ///< PWM high-side output polarity. Default: 0. R/W \
    X(BREAK_BEFORE_MAKE_TIME_LOW_UVW, 235)          ///< Break-before-make (low UVW). Default: 0. R/W \
    X(BREAK_BEFORE_MAKE_TIME_HIGH_UVW, 236)         ///< Break-before-make (high UVW). Default: 0. R/W \
    X(BREAK_BEFORE_MAKE_TIME_LOW_Y2, 237)           ///< Break-before-make (low Y2). Default: 0. R/W \
    X(BREAK_BEFORE_MAKE_TIME_HIGH_Y2, 238)          ///< Break-before-make (high Y2). Default: 0. R/W \
    X(PWM_DEAD_TIME_COMPENSATION, 239)              ///< PWM dead-time compensation. Default: 0. R/W \
    X(PWM_SYMMETRY_SHIFT, 240)                      ///< PWM symmetry shift. Default: 0. R/W \
    X(ADC_VDS_CONVERSION_DELAY, 241)                ///< ADC VDS conversion delay. Default: 0. R/W \
    X(VGS_FILTER_CONSTANT, 242)                     ///< VGS filter constant. Default: 0. R/W \
    X(UVW_LOW_SIDE_ENABLE, 254)                     ///< UVW low-side OC protect enable. Default: 0. R/W \
    X(UVW_HIGH_SIDE_ENABLE, 255)                    ///< UVW high-side OC protect enable. Default: 0. R/W \
    X(UVW_LOW_SIDE_THRESHOLD, 258)                  ///< UVW low-side OC threshold (see enum). Default: 0. R/W \
    X(UVW_HIGH_SIDE_THRESHOLD, 259)                 ///< UVW high-side OC threshold. Default: 0. R/W \
    X(Y2_LOW_SIDE_THRESHOLD, 260)                   ///< Y2 low-side OC threshold. Default: 0. R/W \
    X(Y2_HIGH_SIDE_THRESHOLD, 261)                  ///< Y2 high-side OC threshold. Default: 0. R/W \
    X(Y2_LOW_SIDE_VDS_MONITOR_ENABLE, 262)          ///< Y2 low-side VDS monitor enable. Default: 0. R/W \
    X(OVERCURRENT_REACTION, 263)                    ///< Overcurrent reaction mode. Default: 0. R/W \
    X(VGS_LOW_SIDE_REACTION, 264)                   ///< VGS low-side reaction. Default: 0. R/W \
    X(VGS_HIGH_SIDE_REACTION, 265)                  ///< VGS high-side reaction. Default: 0. R/W \
    X(VGS_BLANKING_TIME, 266)                       ///< VGS blanking time (see enum). Default: 0. R/W \
    X(VGS_DEGLITCH_TIME, 267)                       ///< VGS deglitch time. Default: 0. R/W \
    X(TORQUE_FLUX_COMBINED_TARGET_VALUES, 330)      ///< Combined torque/flux target (32-bit). Read-only. \
    X(TORQUE_FLUX_COMBINED_ACTUAL_VALUES, 331)      ///< Combined torque/flux actual (32-bit). Read-only. \
    X(VOLTAGE_D_Q_COMBINED_ACTUAL_VALUES, 332)      ///< Combined D/Q voltage actual (32-bit). Read-only. \
    X(INTEGRATED_ACTUAL_TORQUE_VALUE, 333)          ///< Integrated actual torque (summed). Read-only. \
    X(INTEGRATED_ACTUAL_VELOCITY_VALUE, 334)        ///< Integrated actual velocity (summed). Read-only.

enum class ID : uint16_t {
    #define X(NAME, VALUE) NAME = VALUE,
    PARAM_LIST
    #undef X
};

inline const char* to_string(ID id) {
    // Note: Duplicate numeric values return the first matching name
    #define X(NAME, VALUE) if(id == ID::NAME) return #NAME;
    PARAM_LIST
    #undef X
    return "UNKNOWN";
}
#undef PARAM_LIST

} // namespace tmc9660::tmcl

#endif // TMC9660_PARAM_MODE_TMCL_XMACROS_HPP
