#include "TMC9660.hpp"
#include <cmath>
#include <chrono>
#include <thread>

// Define TMCL operation codes for convenience
static constexpr uint8_t OP_NOP = 0;
static constexpr uint8_t OP_MST = 3;
static constexpr uint8_t OP_SAP = 5;
static constexpr uint8_t OP_GAP = 6;
static constexpr uint8_t OP_STAP = 7; // Store All Parameters (not used here)
static constexpr uint8_t OP_SGP = 9;
static constexpr uint8_t OP_GGP = 10;
static constexpr uint8_t OP_RFS = 13; // Reference Search (not explicitly used)
static constexpr uint8_t OP_SIO = 14; // Set IO (not used)
static constexpr uint8_t OP_GIO = 15; // Get IO (not used)
static constexpr uint8_t OP_RAMDEBUG = 142;
static constexpr uint8_t OP_STOP_SCRIPT = 28;
static constexpr uint8_t OP_CALCX = 33; // Arithmetic operations (script use)
static constexpr uint8_t OP_AAP = 34; // Accumulator to Axis Parameter (script)
static constexpr uint8_t OP_AGP =
    35; // Accumulator to Global Parameter (script)
static constexpr uint8_t OP_RST = 48; // Restart script from address
static constexpr uint8_t OP_DOWNLOAD_START = 132;
static constexpr uint8_t OP_DOWNLOAD_END = 133;
static constexpr uint8_t OP_READ_MEM = 134;
static constexpr uint8_t OP_GET_SCRIPT_STATUS = 135;

TMC9660::TMC9660(TMC9660CommInterface &comm, uint8_t address)
    : comm_(comm), address_(address & 0x7F) // ensure address is 7-bit
{}

bool TMC9660::writeParameter(uint16_t id, uint32_t value, uint8_t motorIndex) {
  return sendCommand(OP_SAP, id, motorIndex, value, nullptr);
}

bool TMC9660::readParameter(uint16_t id, uint32_t &value, uint8_t motorIndex) {
  return sendCommand(OP_GAP, id, motorIndex, 0, &value);
}

bool TMC9660::writeGlobalParameter(uint16_t id, uint8_t bank, uint32_t value) {
  return sendCommand(OP_SGP, id, bank, value, nullptr);
}

bool TMC9660::readGlobalParameter(uint16_t id, uint8_t bank, uint32_t &value) {
  return sendCommand(OP_GGP, id, bank, 0, &value);
}

bool TMC9660::configureMotorType(MotorType type, uint8_t polePairs) {
  // Set motor type
  uint16_t typeVal = static_cast<uint16_t>(type);
  if (!writeParameter(0, typeVal)) {
    return false;
  }
  // If BLDC or Stepper, set pole pairs (for stepper, treat as 1 pole pair by
  // default).
  if (type == MotorType::BLDC || type == MotorType::STEPPER) {
    if (!writeParameter(1, polePairs)) {
      return false;
    }
  }
  return true;
}

bool TMC9660::setMotorDirection(MotorDirection direction) {
  return writeParameter(2, static_cast<uint32_t>(direction));
}

bool TMC9660::setPWMFrequency(uint32_t frequencyHz) {
  // PWM frequency parameter (in Hz).
  return writeParameter(3, frequencyHz);
}

bool TMC9660::setCommutationMode(CommutationMode mode) {
  return writeParameter(4, static_cast<uint32_t>(mode));
}

bool TMC9660::setMaxCurrent(uint16_t milliamps) {
  // MAX_TORQUE parameter (ID 6) in mA
  return writeParameter(6, static_cast<uint32_t>(milliamps));
}

bool TMC9660::stopMotor() {
  // Send an MST (stop) command.
  return sendCommand(OP_MST, 0, 0, 0, nullptr);
}

bool TMC9660::setTargetTorque(int16_t milliamps) {
  int32_t temp = milliamps;
  uint32_t val = static_cast<uint32_t>(temp);
  return writeParameter(104, val);
}

bool TMC9660::setTargetVelocity(int32_t velocity) {
  uint32_t val = static_cast<uint32_t>(velocity);
  return writeParameter(124, val);
}

bool TMC9660::setTargetPosition(int32_t position) {
  uint32_t val = static_cast<uint32_t>(position);
  return writeParameter(143, val);
}

bool TMC9660::setCurrentLoopGains(uint16_t p, uint16_t i, bool separateFlux,
                                  uint16_t fluxP, uint16_t fluxI) {
  bool ok = true;
  if (separateFlux) {
    // Enable separate torque/flux PI control parameters
    ok &= writeParameter(113, 1);
    // Set torque PI gains
    ok &= writeParameter(109, p);
    ok &= writeParameter(110, i);
    // Set flux PI gains (if provided; otherwise use same as torque if fluxP or
    // fluxI is zero)
    uint16_t useFluxP = (fluxP != 0 ? fluxP : p);
    uint16_t useFluxI = (fluxI != 0 ? fluxI : i);
    ok &= writeParameter(111, useFluxP);
    ok &= writeParameter(112, useFluxI);
  } else {
    // Use combined torque/flux PI parameters
    ok &= writeParameter(113, 0);
    ok &= writeParameter(109, p);
    ok &= writeParameter(110, i);
    // When combined, flux PI uses the same values as torque PI (no separate
    // settings needed).
  }
  return ok;
}

bool TMC9660::setVelocityLoopGains(uint16_t p, uint16_t i) {
  bool ok = true;
  ok &= writeParameter(127, p);
  ok &= writeParameter(128, i);
  return ok;
}

bool TMC9660::setPositionGain(uint16_t p) { return writeParameter(146, p); }

bool TMC9660::configureGateDriver(uint8_t sinkLevel, uint8_t sourceLevel,
                                  uint16_t deadTimeNs) {
  if (sinkLevel > 15)
    sinkLevel = 15;
  if (sourceLevel > 15)
    sourceLevel = 15;
  // Calculate dead-time value (in units of ~8.33 ns per step).
  int deadSteps = static_cast<int>(std::lround(deadTimeNs / 8.33));
  if (deadSteps < 0)
    deadSteps = 0;
  if (deadSteps > 255)
    deadSteps = 255;
  uint8_t dt = static_cast<uint8_t>(deadSteps);
  bool ok = true;
  // Set break-before-make time for all FETs (both UVW and Y2, low and high
  // sides) to the same value.
  ok &= writeParameter(235, dt);
  ok &= writeParameter(236, dt);
  ok &= writeParameter(237, dt);
  ok &= writeParameter(238, dt);
  // Set gate driver sink/source current levels for UVW and Y2 phases.
  ok &= writeParameter(245, sinkLevel);
  ok &= writeParameter(246, sourceLevel);
  ok &= writeParameter(247, sinkLevel);
  ok &= writeParameter(248, sourceLevel);
  return ok;
}

bool TMC9660::setGateOutputPolarity(bool lowActive, bool highActive) {
  bool ok = true;
  ok &= writeParameter(233, lowActive ? 1u : 0u);
  ok &= writeParameter(234, highActive ? 1u : 0u);
  return ok;
}

bool TMC9660::configureHall(tmc9660::tmcl::HallSectorOffset sectorOffset,
                            tmc9660::tmcl::Direction inverted,
                            tmc9660::tmcl::EnableDisable enableExtrapolation,
                            uint8_t filterLength) noexcept {
  uint8_t off = static_cast<uint8_t>(sectorOffset);
  if (off > 5)
    off = 0; // valid 0-5
  bool ok = true;
  ok &= writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::HALL_SECTOR_OFFSET), off);
  ok &= writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::HALL_INVERT_DIRECTION),
      inverted == tmc9660::tmcl::Direction::INVERTED ? 1u : 0u);
  ok &= writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::HALL_FILTER_LENGTH),
      filterLength);
  ok &= writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::HALL_EXTRAPOLATION_ENABLE),
      enableExtrapolation == tmc9660::tmcl::EnableDisable::ENABLED ? 1u : 0u);
  return ok;
}

bool TMC9660::configureABNEncoder(
    uint32_t countsPerRev,
    tmc9660::tmcl::Direction inverted,
    tmc9660::tmcl::EnableDisable nChannelInverted) noexcept {
  uint32_t steps = countsPerRev;
  if (steps > 0xFFFFFF)
    steps = 0xFFFFFF;
  bool ok = true;
  ok &= writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_1_STEPS), steps);
  ok &= writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_1_DIRECTION),
      inverted == tmc9660::tmcl::Direction::INVERTED ? 1u : 0u);
  ok &= writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_1_N_CHANNEL_INVERTED),
      nChannelInverted == tmc9660::tmcl::EnableDisable::ENABLED ? 1u : 0u);
  return ok;
}

bool TMC9660::configureSecondaryABNEncoder(uint32_t countsPerRev,
                                           tmc9660::tmcl::Direction inverted,
                                           uint8_t gearRatio) noexcept {
  uint32_t steps = countsPerRev;
  if (steps > 0xFFFFFF)
    steps = 0xFFFFFF;
  if (gearRatio == 0)
    gearRatio = 1;
  bool ok = true;
  ok &= writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_2_STEPS), steps);
  ok &= writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_2_DIRECTION),
      inverted == tmc9660::tmcl::Direction::INVERTED ? 1u : 0u);
  ok &= writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_2_GEAR_RATIO),
      gearRatio);
  ok &= writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_2_ENABLE), 1u);
  return ok;
}

bool TMC9660::FeedbackSense::configureABNInitialization(
    tmc9660::tmcl::AbnInitMethod initMethod, uint16_t initDelay,
    int32_t initVelocity, int16_t nChannelOffset) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_1_INIT_METHOD),
      static_cast<uint32_t>(initMethod));
  ok &= driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_1_INIT_DELAY),
      initDelay);
  ok &= driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_1_INIT_VELOCITY),
      static_cast<uint32_t>(initVelocity));
  ok &= driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_1_N_CHANNEL_PHI_E_OFFSET),
      static_cast<uint32_t>(static_cast<int32_t>(nChannelOffset)));
  return ok;
}

bool TMC9660::FeedbackSense::getABNInitializationState(
    tmc9660::tmcl::AbnInitState &state) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(
          static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_1_INIT_STATE),
          tmp))
    return false;
  state = static_cast<tmc9660::tmcl::AbnInitState>(tmp);
  return true;
}

bool TMC9660::FeedbackSense::getABNPhiE(int16_t &phiE) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(
          static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_1_PHI_E), tmp))
    return false;
  phiE = static_cast<int16_t>(tmp);
  return true;
}

bool TMC9660::FeedbackSense::getABNRawValue(uint32_t &value) noexcept {
  return driver.readParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_1_VALUE), value);
}

bool TMC9660::FeedbackSense::configureABNNChannel(
    tmc9660::tmcl::AbnNChannelFiltering filterMode,
    tmc9660::tmcl::EnableDisable clearOnNextNull) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_1_N_CHANNEL_FILTERING),
      static_cast<uint32_t>(filterMode));
  ok &= driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_1_CLEAR_ON_NEXT_NULL),
      clearOnNextNull == tmc9660::tmcl::EnableDisable::ENABLED ? 1u : 0u);
  return ok;
}

bool TMC9660::FeedbackSense::getSecondaryABNCountsPerRev(
    uint32_t &counts) noexcept {
  return driver.readParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_2_STEPS), counts);
}

bool TMC9660::FeedbackSense::getSecondaryABNDirection(
    tmc9660::tmcl::Direction &dir) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(
          static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_2_DIRECTION),
          tmp))
    return false;
  dir = static_cast<tmc9660::tmcl::Direction>(tmp);
  return true;
}

bool TMC9660::FeedbackSense::getSecondaryABNGearRatio(uint8_t &ratio) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(
          static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_2_GEAR_RATIO),
          tmp))
    return false;
  ratio = static_cast<uint8_t>(tmp);
  return true;
}

bool TMC9660::FeedbackSense::setSecondaryABNEncoderEnabled(bool enable) noexcept {
  return driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_2_ENABLE),
      enable ? 1u : 0u);
}

bool TMC9660::FeedbackSense::getSecondaryABNEncoderValue(
    uint32_t &value) noexcept {
  return driver.readParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::ABN_2_VALUE), value);
}

bool TMC9660::configureSPIEncoder(uint8_t cmdSize, uint16_t csSettleTimeNs,
                                  uint8_t csIdleTimeUs) noexcept {
  bool ok = true;
  ok &= writeParameter(180, cmdSize);
  ok &= writeParameter(181, csSettleTimeNs);
  ok &= writeParameter(182, csIdleTimeUs);
  return ok;
}

bool TMC9660::Protection::configureVoltage(uint16_t overVoltThreshold,
                                           uint16_t underVoltThreshold) {
  bool ok = true;
  ok &= driver.writeGlobalParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::SUPPLY_OVERVOLTAGE_WARNING_THRESHOLD),
      0, overVoltThreshold);
  ok &= driver.writeGlobalParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::SUPPLY_UNDERVOLTAGE_WARNING_THRESHOLD),
      0, underVoltThreshold);
  return ok;
}

bool TMC9660::Protection::configureTemperature(float warningDegC,
                                               float shutdownDegC) {
  // Convert Celsius to raw sensor units: val = (Temp + 268.15) / 0.01615
  float warnVal = (warningDegC + 268.15f) / 0.01615f;
  float shutVal = (shutdownDegC + 268.15f) / 0.01615f;
  if (warnVal < 0)
    warnVal = 0;
  if (warnVal > 65535)
    warnVal = 65535;
  if (shutVal < 0)
    shutVal = 0;
  if (shutVal > 65535)
    shutVal = 65535;
  uint16_t warnRaw = static_cast<uint16_t>(std::lround(warnVal));
  uint16_t shutRaw = static_cast<uint16_t>(std::lround(shutVal));
  bool ok = true;
  ok &= driver.writeGlobalParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::CHIP_TEMPERATURE_WARNING_THRESHOLD),
      0, warnRaw);
  ok &= driver.writeGlobalParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::CHIP_TEMPERATURE_SHUTDOWN_THRESHOLD),
      0, shutRaw);
  return ok;
}

bool TMC9660::Protection::setOvercurrentEnabled(bool enabled) {
  uint8_t val = enabled ? 1 : 0;
  bool ok = true;
  ok &= driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_ENABLE_UVW_LOW),
      val);
  ok &= driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_ENABLE_UVW_HIGH),
      val);
  ok &= driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_ENABLE_Y2_LOW),
      val);
  ok &= driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_ENABLE_Y2_HIGH),
      val);
  return ok;
}

bool TMC9660::Protection::configureI2t(uint16_t timeConstant1_ms,
                                       float continuousCurrent1_A,
                                       uint16_t timeConstant2_ms,
                                       float continuousCurrent2_A) {
  bool ok = true;
  ok &= driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::THERMAL_WINDING_TIME_CONSTANT_1),
      timeConstant1_ms);
  uint32_t limit1 = static_cast<uint32_t>(continuousCurrent1_A * continuousCurrent1_A * timeConstant1_ms);
  ok &= driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::IIT_LIMIT_1), limit1);
  ok &= driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::THERMAL_WINDING_TIME_CONSTANT_2),
      timeConstant2_ms);
  uint32_t limit2 = static_cast<uint32_t>(continuousCurrent2_A * continuousCurrent2_A * timeConstant2_ms);
  ok &= driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::IIT_LIMIT_2), limit2);
  return ok;
}

bool TMC9660::Protection::resetI2tState() {
  return driver.writeParameter(
      static_cast<uint16_t>(tmc9660::tmcl::Parameters::RESET_IIT_SUMS), 1u);
}

bool TMC9660::uploadScript(const std::vector<uint32_t> &scriptData) {
  // Enter script download mode
  if (!sendCommand(OP_DOWNLOAD_START, 0, 0, 0, nullptr)) {
    return false;
  }
  // Send each 32-bit instruction (as a value in a NOP command frame).
  for (uint32_t instr : scriptData) {
    if (!sendCommand(OP_NOP, 0, 0, instr, nullptr)) {
      return false;
    }
  }
  // Exit download mode
  if (!sendCommand(OP_DOWNLOAD_END, 0, 0, 0, nullptr)) {
    return false;
  }
  return true;
}

bool TMC9660::startScript(uint16_t address) {
  // Use RST command to start script at given address
  return sendCommand(OP_RST, 0, 0, address, nullptr);
}

bool TMC9660::stopScriptExecution() {
  return sendCommand(OP_STOP_SCRIPT, 0, 0, 0, nullptr);
}

bool TMC9660::initRAMDebug(uint32_t sampleCount) {
  bool ok = true;
  // Initialize/reset RAM debug
  ok &= sendCommand(OP_RAMDEBUG, 0, 0, 0, nullptr);
  // Set number of samples to capture
  ok &= sendCommand(OP_RAMDEBUG, 1, 0, sampleCount, nullptr);
  return ok;
}

bool TMC9660::startRAMDebugCapture() {
  return sendCommand(OP_RAMDEBUG, 6, 0, 0, nullptr);
}

bool TMC9660::readRAMDebugData(uint32_t index, uint32_t &data) {
  // Use ReadMem command to read memory at given index (assuming index is an
  // address in the debug buffer)
  return sendCommand(OP_READ_MEM, 0, 0, index, &data);
}

bool TMC9660::getRAMDebugStatus(bool &isRunning) {
  uint32_t state = 0;
  if (!sendCommand(OP_RAMDEBUG, 8, 0, 0, &state)) {
    return false;
  }
  // Interpret state: assume 0 = Idle, non-zero = Running
  isRunning = (state != 0);
  return true;
}

float TMC9660::getChipTemperature() {
  uint32_t raw = 0;
  if (!readGlobalParameter(296, 0, raw)) {
    return -273.15f; // return an obviously invalid temperature if read failed
  }
  // raw 16-bit -> temperature in °C
  float tempC = raw * 0.01615f - 268.15f;
  return tempC;
}

int16_t TMC9660::getMotorCurrent() {
  uint32_t raw = 0;
  if (!readParameter(105, raw)) {
    return 0;
  }
  // Actual torque current is 16-bit signed (in mA).
  int16_t current = static_cast<int16_t>(raw & 0xFFFF);
  return current;
}

float TMC9660::getSupplyVoltage() {
  uint32_t raw = 0;
  if (!readGlobalParameter(290, 0, raw)) {
    return -1.0f;
  }
  // raw in 0.1 V units -> convert to V
  float voltage = raw / 10.0f;
  return voltage;
}

int32_t TMC9660::getActualVelocity() {
  uint32_t raw = 0;
  if (!readParameter(125, raw)) {
    return 0;
  }
  int32_t velocity = static_cast<int32_t>(raw);
  return velocity;
}

int32_t TMC9660::getActualPosition() {
  uint32_t raw = 0;
  if (!readParameter(144, raw)) {
    return 0;
  }
  int32_t position = static_cast<int32_t>(raw);
  return position;
}

bool TMC9660::getGeneralStatusFlags(uint32_t &flags) {
  return readParameter(289, flags);
}

bool TMC9660::getGeneralErrorFlags(uint32_t &flags) {
  return readParameter(299, flags);
}

bool TMC9660::getGateDriverErrorFlags(uint32_t &flags) {
  return readParameter(300, flags);
}

bool TMC9660::clearGeneralErrorFlags(uint32_t mask) {
  return writeParameter(299, mask);
}

bool TMC9660::clearGateDriverErrorFlags(uint32_t mask) {
  return writeParameter(300, mask);
}

bool TMC9660::sendCommand(uint8_t opcode, uint16_t type, uint8_t motor,
                          uint32_t value, uint32_t *reply) {
  Datagram d{opcode, type, motor, value};
  std::array<uint8_t, 8> tx{};
  std::array<uint8_t, 8> rx{};
  d.toSpi(tx);
  bool success = comm_.transferDatagram(tx, rx);
  if (!success) {
    return false;
  }
  if (reply != nullptr) {
    uint32_t respVal = (static_cast<uint32_t>(rx[4]) << 24) |
                       (static_cast<uint32_t>(rx[5]) << 16) |
                       (static_cast<uint32_t>(rx[6]) << 8) |
                       (static_cast<uint32_t>(rx[7]));
    *reply = respVal;
  }
  return true;
}

//-------------------------------------------------------------------------
// MotorConfig subsystem implementations
//-------------------------------------------------------------------------

bool TMC9660::MotorConfig::setType(tmc9660::tmcl::MotorType type,
                                   uint8_t polePairs) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::MOTOR_TYPE,
                              static_cast<uint32_t>(type));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::MOTOR_POLE_PAIRS,
                              polePairs);
  return ok;
}

bool TMC9660::MotorConfig::setDirection(
    tmc9660::tmcl::MotorDirection direction) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::MOTOR_DIRECTION,
                               static_cast<uint32_t>(direction));
}

bool TMC9660::MotorConfig::setPWMFrequency(uint32_t frequencyHz) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::MOTOR_PWM_FREQUENCY,
                               frequencyHz);
}

bool TMC9660::MotorConfig::setCommutationMode(
    tmc9660::tmcl::CommutationMode mode) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::COMMUTATION_MODE,
                               static_cast<uint32_t>(mode));
}

bool TMC9660::MotorConfig::setOutputVoltageLimit(uint16_t limit) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::OUTPUT_VOLTAGE_LIMIT,
                               limit);
}

bool TMC9660::MotorConfig::setMaxTorqueCurrent(uint16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::MAX_TORQUE,
                               milliamps);
}

bool TMC9660::MotorConfig::setMaxFluxCurrent(uint16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::MAX_FLUX, milliamps);
}

bool TMC9660::MotorConfig::setPWMSwitchingScheme(
    tmc9660::tmcl::PwmSwitchingScheme scheme) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::PWM_SWITCHING_SCHEME,
                               static_cast<uint32_t>(scheme));
}

bool TMC9660::MotorConfig::setIdleMotorPWMBehavior(
    tmc9660::tmcl::IdleMotorPwmBehavior behavior) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::IDLE_MOTOR_PWM_BEHAVIOR,
      static_cast<uint32_t>(behavior));
}

//-------------------------------------------------------------------------
// CurrentSensing subsystem implementations
//-------------------------------------------------------------------------

bool TMC9660::CurrentSensing::setShuntType(
    tmc9660::tmcl::AdcShuntType shuntType) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::ADC_SHUNT_TYPE,
                               static_cast<uint32_t>(shuntType));
}

bool TMC9660::CurrentSensing::getShuntType(
    tmc9660::tmcl::AdcShuntType &shuntType) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_SHUNT_TYPE, tmp))
    return false;
  shuntType = static_cast<tmc9660::tmcl::AdcShuntType>(tmp);
  return true;
}

bool TMC9660::CurrentSensing::readRaw(int16_t &adc0, int16_t &adc1,
                                      int16_t &adc2, int16_t &adc3) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I0_RAW, v))
    return false;
  adc0 = static_cast<int16_t>(v);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I1_RAW, v))
    return false;
  adc1 = static_cast<int16_t>(v);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I2_RAW, v))
    return false;
  adc2 = static_cast<int16_t>(v);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I3_RAW, v))
    return false;
  adc3 = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::CurrentSensing::setCSAGain(tmc9660::tmcl::CsaGain gain012,
                                         tmc9660::tmcl::CsaGain gain3) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::CSA_GAIN_ADC_I0_TO_ADC_I2,
      static_cast<uint32_t>(gain012));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::CSA_GAIN_ADC_I3,
                              static_cast<uint32_t>(gain3));
  return ok;
}

bool TMC9660::CurrentSensing::getCSAGain(tmc9660::tmcl::CsaGain &gain012,
                                         tmc9660::tmcl::CsaGain &gain3) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::CSA_GAIN_ADC_I0_TO_ADC_I2, tmp))
    return false;
  gain012 = static_cast<tmc9660::tmcl::CsaGain>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::CSA_GAIN_ADC_I3, tmp))
    return false;
  gain3 = static_cast<tmc9660::tmcl::CsaGain>(tmp);
  return true;
}

bool TMC9660::CurrentSensing::setCSAFilter(
    tmc9660::tmcl::CsaFilter filter012,
    tmc9660::tmcl::CsaFilter filter3) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::CSA_FILTER_ADC_I0_TO_ADC_I2,
      static_cast<uint32_t>(filter012));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::CSA_FILTER_ADC_I3,
                              static_cast<uint32_t>(filter3));
  return ok;
}

bool TMC9660::CurrentSensing::getCSAFilter(
    tmc9660::tmcl::CsaFilter &filter012,
    tmc9660::tmcl::CsaFilter &filter3) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::CSA_FILTER_ADC_I0_TO_ADC_I2, tmp))
    return false;
  filter012 = static_cast<tmc9660::tmcl::CsaFilter>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::CSA_FILTER_ADC_I3, tmp))
    return false;
  filter3 = static_cast<tmc9660::tmcl::CsaFilter>(tmp);
  return true;
}

bool TMC9660::CurrentSensing::setScalingFactor(uint16_t scalingFactor) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::CURRENT_SCALING_FACTOR, scalingFactor);
}

bool TMC9660::CurrentSensing::getScalingFactor(uint16_t &scalingFactor) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::CURRENT_SCALING_FACTOR, tmp))
    return false;
  scalingFactor = static_cast<uint16_t>(tmp);
  return true;
}

//-------------------------------------------------------------------------
// FeedbackSense subsystem implementations
//-------------------------------------------------------------------------

bool TMC9660::FeedbackSense::getPhaseAdcMapping(
    tmc9660::tmcl::AdcMapping &ux1, tmc9660::tmcl::AdcMapping &vx2,
    tmc9660::tmcl::AdcMapping &wy1, tmc9660::tmcl::AdcMapping &y2) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::PHASE_UX1_ADC_MAPPING, tmp))
    return false;
  ux1 = static_cast<tmc9660::tmcl::AdcMapping>(tmp);
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::PHASE_VX2_ADC_MAPPING, tmp))
    return false;
  vx2 = static_cast<tmc9660::tmcl::AdcMapping>(tmp);
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::PHASE_WY1_ADC_MAPPING, tmp))
    return false;
  wy1 = static_cast<tmc9660::tmcl::AdcMapping>(tmp);
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::PHASE_Y2_ADC_MAPPING, tmp))
    return false;
  y2 = static_cast<tmc9660::tmcl::AdcMapping>(tmp);
  return true;
}

bool TMC9660::FeedbackSense::setOffsets(int16_t offset0, int16_t offset1,
                                        int16_t offset2,
                                        int16_t offset3) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ADC_I0_OFFSET,
                              static_cast<uint32_t>(static_cast<int32_t>(offset0)));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ADC_I1_OFFSET,
                              static_cast<uint32_t>(static_cast<int32_t>(offset1)));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ADC_I2_OFFSET,
                              static_cast<uint32_t>(static_cast<int32_t>(offset2)));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ADC_I3_OFFSET,
                              static_cast<uint32_t>(static_cast<int32_t>(offset3)));
  return ok;
}

bool TMC9660::FeedbackSense::getOffsets(int16_t &offset0, int16_t &offset1,
                                        int16_t &offset2,
                                        int16_t &offset3) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I0_OFFSET, tmp))
    return false;
  offset0 = static_cast<int16_t>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I1_OFFSET, tmp))
    return false;
  offset1 = static_cast<int16_t>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I2_OFFSET, tmp))
    return false;
  offset2 = static_cast<int16_t>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I3_OFFSET, tmp))
    return false;
  offset3 = static_cast<int16_t>(tmp);
  return true;
}

bool TMC9660::FeedbackSense::readScaledAndOffset(int16_t &adc0, int16_t &adc1,
                                                 int16_t &adc2,
                                                 int16_t &adc3) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I0, tmp))
    return false;
  adc0 = static_cast<int16_t>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I1, tmp))
    return false;
  adc1 = static_cast<int16_t>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I2, tmp))
    return false;
  adc2 = static_cast<int16_t>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I3, tmp))
    return false;
  adc3 = static_cast<int16_t>(tmp);
  return true;
}

bool TMC9660::CurrentSensing::calibrateOffsets(bool waitForCompletion,
                                               uint32_t timeoutMs) noexcept {
  using tmc9660::tmcl::Parameters;
  using tmc9660::tmcl::GeneralStatusFlags;
  // Clear the calibrated flag to trigger a new calibration cycle
  if (!driver.writeParameter(Parameters::GENERAL_STATUS_FLAGS,
                             static_cast<uint32_t>(
                                 GeneralStatusFlags::ADC_OFFSET_CALIBRATED)))
    return false;

  if (!waitForCompletion)
    return true;

  auto start = std::chrono::steady_clock::now();
  bool done = false;
  while (true) {
    if (!getCalibrationStatus(done))
      return false;
    if (done)
      return true;
    if (std::chrono::steady_clock::now() - start >=
        std::chrono::milliseconds(timeoutMs))
      return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool TMC9660::CurrentSensing::getCalibrationStatus(bool &isCalibrated) noexcept {
  uint32_t flags;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::GENERAL_STATUS_FLAGS,
                            flags))
    return false;
  isCalibrated =
      (flags & static_cast<uint32_t>(
                   tmc9660::tmcl::GeneralStatusFlags::ADC_OFFSET_CALIBRATED)) !=
      0;
  return true;
}

//-------------------------------------------------------------------------
// GateDriver subsystem implementations
//-------------------------------------------------------------------------

bool TMC9660::GateDriver::setOutputPolarity(
    tmc9660::tmcl::PwmOutputPolarity lowSide,
    tmc9660::tmcl::PwmOutputPolarity highSide) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::PWM_L_OUTPUT_POLARITY,
                              static_cast<uint32_t>(lowSide));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::PWM_H_OUTPUT_POLARITY,
                              static_cast<uint32_t>(highSide));
  return ok;
}

bool TMC9660::GateDriver::configureBreakBeforeMakeTiming(
    uint8_t lowSideUVW, uint8_t highSideUVW, uint8_t lowSideY2,
    uint8_t highSideY2) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::BREAK_BEFORE_MAKE_TIME_LOW_UVW, lowSideUVW);
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::BREAK_BEFORE_MAKE_TIME_HIGH_UVW, highSideUVW);
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::BREAK_BEFORE_MAKE_TIME_LOW_Y2, lowSideY2);
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::BREAK_BEFORE_MAKE_TIME_HIGH_Y2, highSideY2);
  return ok;
}

bool TMC9660::GateDriver::enableAdaptiveDriveTime(bool enableUVW,
                                                  bool enableY2) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::USE_ADAPTIVE_DRIVE_TIME_UVW,
      enableUVW ? 1u : 0u);
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::USE_ADAPTIVE_DRIVE_TIME_Y2,
      enableY2 ? 1u : 0u);
  return ok;
}

bool TMC9660::GateDriver::configureDriveTimes(uint8_t sinkTimeUVW,
                                              uint8_t sourceTimeUVW,
                                              uint8_t sinkTimeY2,
                                              uint8_t sourceTimeY2) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::DRIVE_TIME_SINK_UVW,
                              sinkTimeUVW);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::DRIVE_TIME_SOURCE_UVW,
                              sourceTimeUVW);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::DRIVE_TIME_SINK_Y2,
                              sinkTimeY2);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::DRIVE_TIME_SOURCE_Y2,
                              sourceTimeY2);
  return ok;
}

bool TMC9660::GateDriver::configureCurrentLimits(
    tmc9660::tmcl::GateCurrentSink sinkCurrentUVW,
    tmc9660::tmcl::GateCurrentSource sourceCurrentUVW,
    tmc9660::tmcl::GateCurrentSink sinkCurrentY2,
    tmc9660::tmcl::GateCurrentSource sourceCurrentY2) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_SINK_CURRENT,
                              static_cast<uint32_t>(sinkCurrentUVW));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_SOURCE_CURRENT,
                              static_cast<uint32_t>(sourceCurrentUVW));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_SINK_CURRENT,
                              static_cast<uint32_t>(sinkCurrentY2));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_SOURCE_CURRENT,
                              static_cast<uint32_t>(sourceCurrentY2));
  return ok;
}

bool TMC9660::GateDriver::configureBootstrapCurrentLimit(
    tmc9660::tmcl::BootstrapCurrentLimit limit) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::BOOTSTRAP_CURRENT_LIMIT,
                               static_cast<uint32_t>(limit));
}

bool TMC9660::GateDriver::configureUndervoltageProtection(
    tmc9660::tmcl::UndervoltageLevel supplyLevel,
    tmc9660::tmcl::UndervoltageEnable enableVdrv,
    tmc9660::tmcl::UndervoltageEnable enableBstUVW,
    tmc9660::tmcl::UndervoltageEnable enableBstY2) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UNDERVOLTAGE_VS_LEVEL,
                              static_cast<uint32_t>(supplyLevel));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UNDERVOLTAGE_VDRV_ENABLE,
                              static_cast<uint32_t>(enableVdrv));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::UNDERVOLTAGE_BST_UVW_ENABLE,
      static_cast<uint32_t>(enableBstUVW));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::UNDERVOLTAGE_BST_Y2_ENABLE,
      static_cast<uint32_t>(enableBstY2));
  return ok;
}

bool TMC9660::GateDriver::enableOvercurrentProtection(
    tmc9660::tmcl::OvercurrentEnable enableUVWLowSide,
    tmc9660::tmcl::OvercurrentEnable enableUVWHighSide,
    tmc9660::tmcl::OvercurrentEnable enableY2LowSide,
    tmc9660::tmcl::OvercurrentEnable enableY2HighSide) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_ENABLE_UVW_LOW,
      static_cast<uint32_t>(enableUVWLowSide));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_ENABLE_UVW_HIGH,
      static_cast<uint32_t>(enableUVWHighSide));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_ENABLE_Y2_LOW,
      static_cast<uint32_t>(enableY2LowSide));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_ENABLE_Y2_HIGH,
      static_cast<uint32_t>(enableY2HighSide));
  return ok;
}

bool TMC9660::GateDriver::setOvercurrentThresholds(
    tmc9660::tmcl::OvercurrentThreshold uvwLowSideThreshold,
    tmc9660::tmcl::OvercurrentThreshold uvwHighSideThreshold,
    tmc9660::tmcl::OvercurrentThreshold y2LowSideThreshold,
    tmc9660::tmcl::OvercurrentThreshold y2HighSideThreshold) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_THRESHOLD_UVW_LOW,
      static_cast<uint32_t>(uvwLowSideThreshold));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_THRESHOLD_UVW_HIGH,
      static_cast<uint32_t>(uvwHighSideThreshold));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_THRESHOLD_Y2_LOW,
      static_cast<uint32_t>(y2LowSideThreshold));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_THRESHOLD_Y2_HIGH,
      static_cast<uint32_t>(y2HighSideThreshold));
  return ok;
}

bool TMC9660::GateDriver::setOvercurrentBlanking(
    tmc9660::tmcl::OvercurrentTiming uvwLowSideTime,
    tmc9660::tmcl::OvercurrentTiming uvwHighSideTime,
    tmc9660::tmcl::OvercurrentTiming y2LowSideTime,
    tmc9660::tmcl::OvercurrentTiming y2HighSideTime) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_BLANKING_UVW_LOW,
      static_cast<uint32_t>(uvwLowSideTime));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_BLANKING_UVW_HIGH,
      static_cast<uint32_t>(uvwHighSideTime));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_BLANKING_Y2_LOW,
      static_cast<uint32_t>(y2LowSideTime));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_BLANKING_Y2_HIGH,
      static_cast<uint32_t>(y2HighSideTime));
  return ok;
}

bool TMC9660::GateDriver::setOvercurrentDeglitch(
    tmc9660::tmcl::OvercurrentTiming uvwLowSideTime,
    tmc9660::tmcl::OvercurrentTiming uvwHighSideTime,
    tmc9660::tmcl::OvercurrentTiming y2LowSideTime,
    tmc9660::tmcl::OvercurrentTiming y2HighSideTime) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_DEGLITCH_UVW_LOW,
      static_cast<uint32_t>(uvwLowSideTime));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_DEGLITCH_UVW_HIGH,
      static_cast<uint32_t>(uvwHighSideTime));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_DEGLITCH_Y2_LOW,
      static_cast<uint32_t>(y2LowSideTime));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::OVERCURRENT_PROTECTION_DEGLITCH_Y2_HIGH,
      static_cast<uint32_t>(y2HighSideTime));
  return ok;
}

bool TMC9660::GateDriver::enableVdsMonitoringLow(
    tmc9660::tmcl::VdsUsage uvwEnable,
    tmc9660::tmcl::VdsUsage y2Enable) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::VDS_MONITORING_USAGE_UVW_LOW,
      static_cast<uint32_t>(uvwEnable));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::VDS_MONITORING_USAGE_Y2_LOW,
      static_cast<uint32_t>(y2Enable));
  return ok;
}

bool TMC9660::GateDriver::configureVgsShortProtectionUVW(
    tmc9660::tmcl::VgsShortEnable enableLowSideOn,
    tmc9660::tmcl::VgsShortEnable enableLowSideOff,
    tmc9660::tmcl::VgsShortEnable enableHighSideOn,
    tmc9660::tmcl::VgsShortEnable enableHighSideOff) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::UVW_LOW_SIDE_ON_ENABLE,
      static_cast<uint32_t>(enableLowSideOn));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::UVW_LOW_SIDE_OFF_ENABLE,
      static_cast<uint32_t>(enableLowSideOff));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::UVW_HIGH_SIDE_ON_ENABLE,
      static_cast<uint32_t>(enableHighSideOn));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::UVW_HIGH_SIDE_OFF_ENABLE,
      static_cast<uint32_t>(enableHighSideOff));
  return ok;
}

bool TMC9660::GateDriver::configureVgsShortProtectionY2(
    tmc9660::tmcl::VgsShortEnable enableLowSideOn,
    tmc9660::tmcl::VgsShortEnable enableLowSideOff,
    tmc9660::tmcl::VgsShortEnable enableHighSideOn,
    tmc9660::tmcl::VgsShortEnable enableHighSideOff) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::Y2_LOW_SIDE_ON_ENABLE,
      static_cast<uint32_t>(enableLowSideOn));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::Y2_LOW_SIDE_OFF_ENABLE,
      static_cast<uint32_t>(enableLowSideOff));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::Y2_HIGH_SIDE_ON_ENABLE,
      static_cast<uint32_t>(enableHighSideOn));
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::Y2_HIGH_SIDE_OFF_ENABLE,
      static_cast<uint32_t>(enableHighSideOff));
  return ok;
}

bool TMC9660::GateDriver::setVgsShortBlankingTime(
    tmc9660::tmcl::VgsBlankingTime uvwTime,
    tmc9660::tmcl::VgsBlankingTime y2Time) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_BLANKING,
                              static_cast<uint32_t>(uvwTime));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_BLANKING,
                              static_cast<uint32_t>(y2Time));
  return ok;
}

bool TMC9660::GateDriver::setVgsShortDeglitchTime(
    tmc9660::tmcl::VgsDeglitchTime uvwTime,
    tmc9660::tmcl::VgsDeglitchTime y2Time) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_DEGLITCH,
                              static_cast<uint32_t>(uvwTime));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_DEGLITCH,
                              static_cast<uint32_t>(y2Time));
  return ok;
}

bool TMC9660::GateDriver::setRetryBehavior(
    tmc9660::tmcl::GdrvRetryBehaviour retryBehavior) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::GDRV_RETRY_BEHAVIOUR,
                               static_cast<uint32_t>(retryBehavior));
}

bool TMC9660::GateDriver::setDriveFaultBehavior(
    tmc9660::tmcl::DriveFaultBehaviour faultBehavior) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::DRIVE_FAULT_BEHAVIOUR,
                               static_cast<uint32_t>(faultBehavior));
}

bool TMC9660::GateDriver::setFaultHandlerRetries(uint8_t retries) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::GDRV_FAULT_HANDLER_RETRIES,
                               retries);
}

//-------------------------------------------------------------------------
// Telemetry subsystem implementations
//-------------------------------------------------------------------------

TMC9660::Telemetry::Telemetry(TMC9660 &parent) noexcept : driver(parent) {}

bool TMC9660::Telemetry::getGeneralStatusFlags(uint32_t &flags) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::GENERAL_STATUS_FLAGS,
                              flags);
}

float TMC9660::Telemetry::getSupplyVoltage() noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::SUPPLY_VOLTAGE, v))
    return -1.0f;
  return static_cast<float>(v) * 0.1f;
}

float TMC9660::Telemetry::getChipTemperature() noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::CHIP_TEMPERATURE, v))
    return -273.0f;
  return static_cast<float>(v) * 0.01615f - 268.15f;
}

int16_t TMC9660::Telemetry::getMotorCurrent() noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::ACTUAL_TOTAL_MOTOR_CURRENT, v))
    return 0;
  return static_cast<int16_t>(v);
}

int32_t TMC9660::Telemetry::getActualVelocity() noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY, v))
    return 0;
  return static_cast<int32_t>(v);
}

int32_t TMC9660::Telemetry::getActualPosition() noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_POSITION, v))
    return 0;
  return static_cast<int32_t>(v);
}

bool TMC9660::Telemetry::getGeneralErrorFlags(uint32_t &flags) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::GENERAL_ERROR_FLAGS,
                              flags);
}

bool TMC9660::Telemetry::getGateDriverErrorFlags(uint32_t &flags) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::GDRV_ERROR_FLAGS,
                              flags);
}

bool TMC9660::Telemetry::clearGeneralErrorFlags(uint32_t mask) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::GENERAL_ERROR_FLAGS,
                               mask);
}

bool TMC9660::Telemetry::clearGateDriverErrorFlags(uint32_t mask) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::GDRV_ERROR_FLAGS,
                               mask);
}

bool TMC9660::Telemetry::getADCStatusFlags(uint32_t &flags) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::ADC_STATUS_FLAGS,
                              flags);
}

bool TMC9660::Telemetry::clearADCStatusFlags(uint32_t mask) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::ADC_STATUS_FLAGS,
                               mask);
}

uint16_t TMC9660::Telemetry::getExternalTemperature() noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::EXTERNAL_TEMPERATURE, v))
    return 0;
  return static_cast<uint16_t>(v);
}

//-------------------------------------------------------------------------
// FOCControl core helpers
//-------------------------------------------------------------------------

bool TMC9660::FOCControl::stop() noexcept {
  return driver.sendCommand(OP_MST, 0, 0, 0, nullptr);
}

bool TMC9660::FOCControl::setTargetTorque(int16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TARGET_TORQUE,
                               static_cast<uint32_t>(static_cast<int32_t>(milliamps)));
}

bool TMC9660::FOCControl::getActualTorque(int16_t &milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_TORQUE, v))
    return false;
  milliamps = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::FOCControl::setTargetFlux(int16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TARGET_FLUX,
                               static_cast<uint32_t>(static_cast<int32_t>(milliamps)));
}

bool TMC9660::FOCControl::getActualFlux(int32_t &milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_FLUX, v))
    return false;
  milliamps = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::setTorqueOffset(int16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TORQUE_OFFSET,
                               static_cast<uint32_t>(static_cast<int32_t>(milliamps)));
}

bool TMC9660::FOCControl::getTorqueOffset(int16_t &milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::TORQUE_OFFSET, v))
    return false;
  milliamps = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::FOCControl::setFluxOffset(int16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::FLUX_OFFSET,
                               static_cast<uint32_t>(static_cast<int32_t>(milliamps)));
}

bool TMC9660::FOCControl::getFluxOffset(int16_t &milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FLUX_OFFSET, v))
    return false;
  milliamps = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::FOCControl::setCurrentLoopGains(uint16_t p, uint16_t i,
                                              bool separate,
                                              uint16_t fluxP,
                                              uint16_t fluxI) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SEPARATE_TORQUE_FLUX_PI_PARAMETERS,
                              separate ? 1u : 0u);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::TORQUE_P, p);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::TORQUE_I, i);
  if (separate) {
    uint16_t useP = fluxP ? fluxP : p;
    uint16_t useI = fluxI ? fluxI : i;
    ok &= driver.writeParameter(tmc9660::tmcl::Parameters::FLUX_P, useP);
    ok &= driver.writeParameter(tmc9660::tmcl::Parameters::FLUX_I, useI);
  }
  return ok;
}

bool TMC9660::FOCControl::setTorqueFluxPiSeparation(
    tmc9660::tmcl::TorqueFluxPiSeparation sep) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::SEPARATE_TORQUE_FLUX_PI_PARAMETERS,
      static_cast<uint32_t>(sep));
}

bool TMC9660::FOCControl::setCurrentNormalization(
    tmc9660::tmcl::CurrentPiNormalization pNorm,
    tmc9660::tmcl::CurrentPiNormalization iNorm) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::TORQUE_NORM_P,
                              static_cast<uint32_t>(pNorm));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::TORQUE_NORM_I,
                              static_cast<uint32_t>(iNorm));
  return ok;
}

bool TMC9660::FOCControl::getTorquePiError(int32_t &error) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::TORQUE_PI_ERROR, v))
    return false;
  error = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::getFluxPiError(int32_t &error) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FLUX_PI_ERROR, v))
    return false;
  error = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::getTorquePiIntegrator(int32_t &integrator) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::TORQUE_PI_INTEGRATOR, v))
    return false;
  integrator = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::getFluxPiIntegrator(int32_t &integrator) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FLUX_PI_INTEGRATOR, v))
    return false;
  integrator = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::setVelocitySensor(
    tmc9660::tmcl::VelocitySensorSelection sel) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::VELOCITY_SENSOR_SELECTION,
                               static_cast<uint32_t>(sel));
}

bool TMC9660::FOCControl::getVelocitySensor(
    tmc9660::tmcl::VelocitySensorSelection &sel) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::VELOCITY_SENSOR_SELECTION, v))
    return false;
  sel = static_cast<tmc9660::tmcl::VelocitySensorSelection>(v);
  return true;
}

bool TMC9660::FOCControl::setTargetVelocity(int32_t velocity) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TARGET_VELOCITY,
                               static_cast<uint32_t>(velocity));
}

bool TMC9660::FOCControl::getActualVelocity(int32_t &velocity) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY, v))
    return false;
  velocity = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::setTargetPosition(int32_t position) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TARGET_POSITION,
                               static_cast<uint32_t>(position));
}

bool TMC9660::FOCControl::getActualPosition(int32_t &position) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_POSITION, v))
    return false;
  position = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::getOpenloopAngle(int16_t &angle) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::OPENLOOP_ANGLE, v))
    return false;
  angle = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::FOCControl::setOpenloopCurrent(uint16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::OPENLOOP_CURRENT,
                               milliamps);
}

bool TMC9660::FOCControl::getOpenloopCurrent(uint16_t &milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::OPENLOOP_CURRENT, v))
    return false;
  milliamps = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::FOCControl::setOpenloopVoltage(uint16_t voltage) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::OPENLOOP_VOLTAGE,
                               voltage);
}

bool TMC9660::FOCControl::getOpenloopVoltage(uint16_t &voltage) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::OPENLOOP_VOLTAGE, v))
    return false;
  voltage = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::FOCControl::setReferenceSwitchEnable(
    tmc9660::tmcl::ReferenceSwitchEnable enable) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_ENABLE,
                               static_cast<uint32_t>(enable));
}

bool TMC9660::FOCControl::getReferenceSwitchEnable(
    tmc9660::tmcl::ReferenceSwitchEnable &enable) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_ENABLE, v))
    return false;
  enable = static_cast<tmc9660::tmcl::ReferenceSwitchEnable>(v);
  return true;
}

bool TMC9660::FOCControl::setReferenceSwitchPolaritySwap(
    tmc9660::tmcl::ReferenceSwitchPolaritySwap config) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::REFERENCE_SWITCH_POLARITY_AND_SWAP,
      static_cast<uint32_t>(config));
}

bool TMC9660::FOCControl::getReferenceSwitchPolaritySwap(
    tmc9660::tmcl::ReferenceSwitchPolaritySwap &config) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::REFERENCE_SWITCH_POLARITY_AND_SWAP, v))
    return false;
  config = static_cast<tmc9660::tmcl::ReferenceSwitchPolaritySwap>(v);
  return true;
}

bool TMC9660::FOCControl::setReferenceSwitchLatchSettings(
    tmc9660::tmcl::ReferenceSwitchLatchSettings setting) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::REFERENCE_SWITCH_LATCH_SETTINGS,
      static_cast<uint32_t>(setting));
}

bool TMC9660::FOCControl::getReferenceSwitchLatchSettings(
    tmc9660::tmcl::ReferenceSwitchLatchSettings &setting) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::REFERENCE_SWITCH_LATCH_SETTINGS, v))
    return false;
  setting = static_cast<tmc9660::tmcl::ReferenceSwitchLatchSettings>(v);
  return true;
}

bool TMC9660::FOCControl::setEventStopSettings(
    tmc9660::tmcl::EventStopSettings settings) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::EVENT_STOP_SETTINGS,
                               static_cast<uint32_t>(settings));
}

bool TMC9660::FOCControl::getEventStopSettings(
    tmc9660::tmcl::EventStopSettings &settings) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::EVENT_STOP_SETTINGS, v))
    return false;
  settings = static_cast<tmc9660::tmcl::EventStopSettings>(v);
  return true;
}

bool TMC9660::FOCControl::setReferenceSwitchSearchMode(
    tmc9660::tmcl::ReferenceSwitchSearchMode mode) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::REFERENCE_SWITCH_SEARCH_MODE,
      static_cast<uint32_t>(mode));
}

bool TMC9660::FOCControl::getReferenceSwitchSearchMode(
    tmc9660::tmcl::ReferenceSwitchSearchMode &mode) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::REFERENCE_SWITCH_SEARCH_MODE, v))
    return false;
  mode = static_cast<tmc9660::tmcl::ReferenceSwitchSearchMode>(v);
  return true;
}

bool TMC9660::FOCControl::setReferenceSwitchSearchSpeed(int32_t speed) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::REFERENCE_SWITCH_SEARCH_SPEED,
      static_cast<uint32_t>(speed));
}

bool TMC9660::FOCControl::getReferenceSwitchSearchSpeed(int32_t &speed) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::REFERENCE_SWITCH_SEARCH_SPEED, v))
    return false;
  speed = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::setReferenceSwitchSpeed(int32_t speed) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_SPEED,
                               static_cast<uint32_t>(speed));
}

bool TMC9660::FOCControl::getReferenceSwitchSpeed(int32_t &speed) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_SPEED, v))
    return false;
  speed = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::getRightLimitSwitchPosition(int32_t &position) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::RIGHT_LIMIT_SWITCH_POSITION, v))
    return false;
  position = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::getHomeSwitchPosition(int32_t &position) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::HOME_SWITCH_POSITION, v))
    return false;
  position = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::getLastReferencePosition(int32_t &position) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::LAST_REFERENCE_POSITION, v))
    return false;
  position = static_cast<int32_t>(v);
  return true;
}

//-------------------------------------------------------------------------
// FOCControl inline helpers moved from header
//-------------------------------------------------------------------------

bool TMC9660::FOCControl::getMccInputsRaw(uint16_t &inputs) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::MCC_INPUTS_RAW, v))
    return false;
  inputs = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::FOCControl::getFocVoltageUx(int16_t &voltage) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_VOLTAGE_UX, v))
    return false;
  voltage = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::FOCControl::getFocVoltageWy(int16_t &voltage) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_VOLTAGE_WY, v))
    return false;
  voltage = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::FOCControl::getFocVoltageV(int16_t &voltage) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_VOLTAGE_V, v))
    return false;
  voltage = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::FOCControl::getFocVoltageUq(int16_t &voltage) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_VOLTAGE_UQ, v))
    return false;
  voltage = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::FOCControl::setFieldWeakeningI(uint16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::FIELDWEAKENING_I,
                               milliamps);
}

bool TMC9660::FOCControl::getFieldWeakeningI(uint16_t &milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FIELDWEAKENING_I, v))
    return false;
  milliamps = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::FOCControl::setFieldWeakeningVoltageThreshold(uint16_t voltage) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::FIELDWEAKENING_VOLTAGE_THRESHOLD, voltage);
}

bool TMC9660::FOCControl::getFieldWeakeningVoltageThreshold(uint16_t &voltage) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::FIELDWEAKENING_VOLTAGE_THRESHOLD, v))
    return false;
  voltage = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::FOCControl::getFocCurrentUx(int16_t &milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_CURRENT_UX, v))
    return false;
  milliamps = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::FOCControl::getFocCurrentV(int16_t &milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_CURRENT_V, v))
    return false;
  milliamps = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::FOCControl::getFocCurrentWy(int16_t &milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_CURRENT_WY, v))
    return false;
  milliamps = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::FOCControl::getFocCurrentIq(int16_t &milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_CURRENT_IQ, v))
    return false;
  milliamps = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::FOCControl::setTargetTorqueBiquadFilterEnable(bool enable) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_ENABLE, enable);
}

bool TMC9660::FOCControl::getTargetTorqueBiquadFilterEnable(bool &enable) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_ENABLE, v))
    return false;
  enable = (v != 0);
  return true;
}

bool TMC9660::FOCControl::setTargetTorqueBiquadFilterACoeff1(int32_t coeff) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_1,
      static_cast<uint32_t>(coeff));
}

bool TMC9660::FOCControl::getTargetTorqueBiquadFilterACoeff1(int32_t &coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_1, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::setTargetTorqueBiquadFilterACoeff2(int32_t coeff) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_2,
      static_cast<uint32_t>(coeff));
}

bool TMC9660::FOCControl::getTargetTorqueBiquadFilterACoeff2(int32_t &coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_2, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::setTargetTorqueBiquadFilterBCoeff0(int32_t coeff) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_0,
      static_cast<uint32_t>(coeff));
}

bool TMC9660::FOCControl::getTargetTorqueBiquadFilterBCoeff0(int32_t &coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_0, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::setTargetTorqueBiquadFilterBCoeff1(int32_t coeff) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_1,
      static_cast<uint32_t>(coeff));
}

bool TMC9660::FOCControl::getTargetTorqueBiquadFilterBCoeff1(int32_t &coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_1, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::setTargetTorqueBiquadFilterBCoeff2(int32_t coeff) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_2,
      static_cast<uint32_t>(coeff));
}

bool TMC9660::FOCControl::getTargetTorqueBiquadFilterBCoeff2(int32_t &coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_2, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::setActualVelocityBiquadFilterEnable(bool enable) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_ENABLE, enable);
}

bool TMC9660::FOCControl::getActualVelocityBiquadFilterEnable(bool &enable) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_ENABLE, v))
    return false;
  enable = (v != 0);
  return true;
}

bool TMC9660::FOCControl::setActualVelocityBiquadFilterACoeff1(int32_t coeff) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_1,
      static_cast<uint32_t>(coeff));
}

bool TMC9660::FOCControl::getActualVelocityBiquadFilterACoeff1(int32_t &coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_1, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::setActualVelocityBiquadFilterACoeff2(int32_t coeff) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_2,
      static_cast<uint32_t>(coeff));
}

bool TMC9660::FOCControl::getActualVelocityBiquadFilterACoeff2(int32_t &coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_2, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::setActualVelocityBiquadFilterBCoeff0(int32_t coeff) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_0,
      static_cast<uint32_t>(coeff));
}

bool TMC9660::FOCControl::getActualVelocityBiquadFilterBCoeff0(int32_t &coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_0, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::setActualVelocityBiquadFilterBCoeff1(int32_t coeff) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_1,
      static_cast<uint32_t>(coeff));
}

bool TMC9660::FOCControl::getActualVelocityBiquadFilterBCoeff1(int32_t &coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_1, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::setActualVelocityBiquadFilterBCoeff2(int32_t coeff) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_2,
      static_cast<uint32_t>(coeff));
}

bool TMC9660::FOCControl::getActualVelocityBiquadFilterBCoeff2(int32_t &coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(
          tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_2, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::FOCControl::getTorqueFluxCombinedTargetValues(uint32_t &value) noexcept {
  return driver.readParameter(
      tmc9660::tmcl::Parameters::TORQUE_FLUX_COMBINED_TARGET_VALUES, value);
}

bool TMC9660::FOCControl::getTorqueFluxCombinedActualValues(uint32_t &value) noexcept {
  return driver.readParameter(
      tmc9660::tmcl::Parameters::TORQUE_FLUX_COMBINED_ACTUAL_VALUES, value);
}

bool TMC9660::FOCControl::getVoltageDqCombinedActualValues(uint32_t &value) noexcept {
  return driver.readParameter(
      tmc9660::tmcl::Parameters::VOLTAGE_D_Q_COMBINED_ACTUAL_VALUES, value);
}

bool TMC9660::FOCControl::getIntegratedActualTorqueValue(uint32_t &value) noexcept {
  return driver.readParameter(
      tmc9660::tmcl::Parameters::INTEGRATED_ACTUAL_TORQUE_VALUE, value);
}

bool TMC9660::FOCControl::getIntegratedActualVelocityValue(uint32_t &value) noexcept {
  return driver.readParameter(
      tmc9660::tmcl::Parameters::INTEGRATED_ACTUAL_VELOCITY_VALUE, value);
}

//-------------------------------------------------------------------------
// Ramp subsystem implementations
//-------------------------------------------------------------------------

bool TMC9660::Ramp::enable(bool on) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_ENABLE,
                               on ? 1u : 0u);
}

bool TMC9660::Ramp::setAcceleration(uint32_t a1, uint32_t a2,
                                    uint32_t aMax) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_A1, a1);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_A2, a2);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_AMAX, aMax);
  return ok;
}

bool TMC9660::Ramp::setDeceleration(uint32_t d1, uint32_t d2,
                                    uint32_t dMax) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_D1, d1);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_D2, d2);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_DMAX, dMax);
  return ok;
}

bool TMC9660::Ramp::setVelocities(uint32_t vStart, uint32_t vStop, uint32_t v1,
                                  uint32_t v2, uint32_t vMax) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_VSTART, vStart);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_VSTOP, vStop);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_V1, v1);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_V2, v2);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_VMAX, vMax);
  return ok;
}

bool TMC9660::Ramp::setTiming(uint16_t tVmaxCycles,
                              uint16_t tZeroWaitCycles) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_TVMAX,
                              tVmaxCycles);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_TZEROWAIT,
                              tZeroWaitCycles);
  return ok;
}

bool TMC9660::Ramp::enableFeedForward(
    bool enableVelFF, bool enableAccelFF, uint16_t accelFFGain,
    tmc9660::tmcl::AccelerationFFShift accelFFShift) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::VELOCITY_FEEDFORWARD_ENABLE,
      enableVelFF ? 1u : 0u);
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::ACCELERATION_FEEDFORWARD_ENABLE,
      enableAccelFF ? 1u : 0u);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ACCELERATION_FF_GAIN,
                              accelFFGain);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ACCELERATION_FF_SHIFT,
                              static_cast<uint32_t>(accelFFShift));
  return ok;
}

bool TMC9660::Ramp::setDirectVelocityMode(bool enable) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::DIRECT_VELOCITY_MODE,
                               enable ? 1u : 0u);
}

bool TMC9660::Ramp::getRampVelocity(int32_t &velocity) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::RAMP_VELOCITY, v))
    return false;
  velocity = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::Ramp::getRampPosition(int32_t &position) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::RAMP_POSITION, v))
    return false;
  position = static_cast<int32_t>(v);
  return true;
}

//-------------------------------------------------------------------------
// StepDir subsystem implementations
//-------------------------------------------------------------------------

bool TMC9660::StepDir::setMicrostepResolution(
    tmc9660::tmcl::StepDirStepDividerShift µSteps) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::STEP_DIR_STEP_DIVIDER_SHIFT,
      static_cast<uint32_t>(µSteps));
}

bool TMC9660::StepDir::enableInterface(bool on) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::STEP_DIR_ENABLE,
                               on ? 1u : 0u);
}

bool TMC9660::StepDir::enableExtrapolation(bool enable) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::STEP_DIR_EXTRAPOLATION_ENABLE,
      enable ? 1u : 0u);
}

bool TMC9660::StepDir::setSignalTimeout(uint16_t timeout_ms) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::STEP_DIR_STEP_SIGNAL_TIMEOUT_LIMIT,
      timeout_ms);
}

bool TMC9660::StepDir::setMaxExtrapolationVelocity(uint32_t eRPM) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::STEP_DIR_MAXIMUM_EXTRAPOLATION_VELOCITY,
      eRPM);
}

bool TMC9660::StepDir::enableVelocityFeedForward(bool enableVelFF) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::VELOCITY_FEEDFORWARD_ENABLE,
      enableVelFF ? 1u : 0u);
}

//-------------------------------------------------------------------------
// Brake subsystem implementations
//-------------------------------------------------------------------------

bool TMC9660::Brake::enableChopper(bool enable) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::BRAKE_CHOPPER_ENABLE,
                               enable ? 1u : 0u);
}

bool TMC9660::Brake::setVoltageLimit(float voltage) noexcept {
  uint32_t raw = static_cast<uint32_t>(voltage * 10.0f + 0.5f);
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::BRAKE_CHOPPER_VOLTAGE_LIMIT, raw);
}

bool TMC9660::Brake::setHysteresis(float voltage) noexcept {
  uint32_t raw = static_cast<uint32_t>(voltage * 10.0f + 0.5f);
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::BRAKE_CHOPPER_HYSTERESIS, raw);
}

bool TMC9660::Brake::release() noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::RELEASE_BRAKE, 1u);
}

bool TMC9660::Brake::engage() noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::RELEASE_BRAKE, 0u);
}

bool TMC9660::Brake::setReleasingDutyCycle(uint8_t percent) noexcept {
  if (percent > 99)
    percent = 99;
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::BRAKE_RELEASING_DUTY_CYCLE, percent);
}

bool TMC9660::Brake::setHoldingDutyCycle(uint8_t percent) noexcept {
  if (percent > 99)
    percent = 99;
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::BRAKE_HOLDING_DUTY_CYCLE, percent);
}

bool TMC9660::Brake::setReleasingDuration(uint16_t milliseconds) noexcept {
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::BRAKE_RELEASING_DURATION, milliseconds);
}

bool TMC9660::Brake::invertOutput(bool invert) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::INVERT_BRAKE_OUTPUT,
                               invert ? 1u : 0u);
}

//-------------------------------------------------------------------------
// StopEvents subsystem implementations
//-------------------------------------------------------------------------

bool TMC9660::StopEvents::enableDeviationStop(uint32_t maxVelError,
                                              uint32_t maxPosError,
                                              bool softStop) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::STOP_ON_VELOCITY_DEVIATION, maxVelError);
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::STOP_ON_POSITION_DEVIATION, maxPosError);

  using tmc9660::tmcl::EventStopSettings;
  bool vel = maxVelError != 0;
  bool pos = maxPosError != 0;
  EventStopSettings setting = EventStopSettings::DO_HARD_STOP;
  if (vel && pos)
    setting = softStop ? EventStopSettings::STOP_ON_POS_VEL_DEVIATION_SOFT_STOP
                       : EventStopSettings::STOP_ON_POS_VEL_DEVIATION;
  else if (pos)
    setting = softStop ? EventStopSettings::STOP_ON_POS_DEVIATION_SOFT_STOP
                       : EventStopSettings::STOP_ON_POS_DEVIATION;
  else if (vel)
    setting = softStop ? EventStopSettings::STOP_ON_VEL_DEVIATION_SOFT_STOP
                       : EventStopSettings::STOP_ON_VEL_DEVIATION;
  else
    setting = softStop ? EventStopSettings::DO_SOFT_STOP
                       : EventStopSettings::DO_HARD_STOP;

  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::EVENT_STOP_SETTINGS,
                              static_cast<uint32_t>(setting));
  return ok;
}

bool TMC9660::StopEvents::configureReferenceSwitches(uint8_t mask, bool invertL,
                                                     bool invertR, bool invertH,
                                                     bool swapLR) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_ENABLE,
                              mask);
  uint8_t cfg = (invertL ? 1u : 0u) | (invertR ? 2u : 0u) |
                (invertH ? 4u : 0u) | (swapLR ? 8u : 0u);
  ok &= driver.writeParameter(
      tmc9660::tmcl::Parameters::REFERENCE_SWITCH_POLARITY_AND_SWAP, cfg);
  return ok;
}

bool TMC9660::StopEvents::getAndClearLatchedPosition(int32_t &pos) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::LATCH_POSITION, v))
    return false;
  pos = static_cast<int32_t>(v);
  driver.writeParameter(
      tmc9660::tmcl::Parameters::GENERAL_STATUS_FLAGS,
      static_cast<uint32_t>(
          tmc9660::tmcl::GeneralStatusFlags::RAMPER_LATCHED));
  return true;
}

//-------------------------------------------------------------------------
// IIT inline helpers moved from header
//-------------------------------------------------------------------------

bool TMC9660::IIT::resetIntegralState() noexcept {
  return driver.writeGlobalParameter(
      tmc9660::tmcl::Parameters::RESET_IIT_SUMS, 0, 0);
}

bool TMC9660::IIT::setThermalWindingTimeConstant1(uint16_t ms) noexcept {
  return driver.writeGlobalParameter(
      tmc9660::tmcl::Parameters::THERMAL_WINDING_TIME_CONSTANT_1, 0, ms);
}

bool TMC9660::IIT::getThermalWindingTimeConstant1(uint16_t &ms) noexcept {
  uint32_t v;
  if (!driver.readGlobalParameter(
          tmc9660::tmcl::Parameters::THERMAL_WINDING_TIME_CONSTANT_1, 0, v))
    return false;
  ms = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::IIT::setLimit1(uint32_t limit) noexcept {
  return driver.writeGlobalParameter(tmc9660::tmcl::Parameters::IIT_LIMIT_1, 0,
                                     limit);
}

bool TMC9660::IIT::getLimit1(uint32_t &limit) noexcept {
  return driver.readGlobalParameter(tmc9660::tmcl::Parameters::IIT_LIMIT_1, 0,
                                    limit);
}

bool TMC9660::IIT::setThermalWindingTimeConstant2(uint16_t ms) noexcept {
  return driver.writeGlobalParameter(
      tmc9660::tmcl::Parameters::THERMAL_WINDING_TIME_CONSTANT_2, 0, ms);
}

bool TMC9660::IIT::getThermalWindingTimeConstant2(uint16_t &ms) noexcept {
  uint32_t v;
  if (!driver.readGlobalParameter(
          tmc9660::tmcl::Parameters::THERMAL_WINDING_TIME_CONSTANT_2, 0, v))
    return false;
  ms = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::IIT::setLimit2(uint32_t limit) noexcept {
  return driver.writeGlobalParameter(tmc9660::tmcl::Parameters::IIT_LIMIT_2, 0,
                                     limit);
}

bool TMC9660::IIT::getLimit2(uint32_t &limit) noexcept {
  return driver.readGlobalParameter(tmc9660::tmcl::Parameters::IIT_LIMIT_2, 0,
                                    limit);
}

bool TMC9660::IIT::getActualTotalMotorCurrent(uint32_t &current,
                                              uint8_t motorIndex) noexcept {
  return driver.readParameter(
      tmc9660::tmcl::Parameters::ACTUAL_TOTAL_MOTOR_CURRENT, current,
      motorIndex);
}

bool TMC9660::IIT::getSum1(uint32_t &sum) noexcept {
  return driver.readGlobalParameter(tmc9660::tmcl::Parameters::IIT_SUM_1, 0,
                                    sum);
}

bool TMC9660::IIT::getSum2(uint32_t &sum) noexcept {
  return driver.readGlobalParameter(tmc9660::tmcl::Parameters::IIT_SUM_2, 0,
                                    sum);
}

//-------------------------------------------------------------------------
// Global parameter helpers
//-------------------------------------------------------------------------

bool TMC9660::Globals::writeBank0(tmc9660::tmcl::GlobalParamBank0 param,
                                  uint32_t value) noexcept {
  return driver.writeGlobalParameter(static_cast<uint16_t>(param), 0, value);
}

bool TMC9660::Globals::readBank0(tmc9660::tmcl::GlobalParamBank0 param,
                                 uint32_t &value) noexcept {
  return driver.readGlobalParameter(static_cast<uint16_t>(param), 0, value);
}

bool TMC9660::Globals::writeBank2(tmc9660::tmcl::GlobalParamBank2 param,
                                  int32_t value) noexcept {
  return driver.writeGlobalParameter(static_cast<uint16_t>(param), 2,
                                     static_cast<uint32_t>(value));
}

bool TMC9660::Globals::readBank2(tmc9660::tmcl::GlobalParamBank2 param,
                                 int32_t &value) noexcept {
  uint32_t tmp;
  if (!driver.readGlobalParameter(static_cast<uint16_t>(param), 2, tmp))
    return false;
  value = static_cast<int32_t>(tmp);
  return true;
}

bool TMC9660::Globals::writeBank3(tmc9660::tmcl::GlobalParamBank3 param,
                                  uint32_t value) noexcept {
  return driver.writeGlobalParameter(static_cast<uint16_t>(param), 3, value);
}

bool TMC9660::Globals::readBank3(tmc9660::tmcl::GlobalParamBank3 param,
                                 uint32_t &value) noexcept {
  return driver.readGlobalParameter(static_cast<uint16_t>(param), 3, value);
}

//-------------------------------------------------------------------------
// High level global parameter helpers
//-------------------------------------------------------------------------

bool TMC9660::Globals::setSerialAddress(uint8_t address) noexcept {
  return writeBank0(tmc9660::tmcl::GlobalParamBank0::SERIAL_ADDRESS, address);
}

bool TMC9660::Globals::getSerialAddress(uint8_t &address) noexcept {
  uint32_t tmp;
  if (!readBank0(tmc9660::tmcl::GlobalParamBank0::SERIAL_ADDRESS, tmp))
    return false;
  address = static_cast<uint8_t>(tmp);
  return true;
}

bool TMC9660::Globals::setHostAddress(uint8_t address) noexcept {
  return writeBank0(tmc9660::tmcl::GlobalParamBank0::SERIAL_HOST_ADDRESS,
                    address);
}

bool TMC9660::Globals::getHostAddress(uint8_t &address) noexcept {
  uint32_t tmp;
  if (!readBank0(tmc9660::tmcl::GlobalParamBank0::SERIAL_HOST_ADDRESS, tmp))
    return false;
  address = static_cast<uint8_t>(tmp);
  return true;
}

bool TMC9660::Globals::configureHeartbeat(
    tmc9660::tmcl::HeartbeatMonitoringConfig iface,
    uint32_t timeout_ms) noexcept {
  bool ok = writeBank0(
      tmc9660::tmcl::GlobalParamBank0::HEARTBEAT_MONITORING_CONFIG,
      static_cast<uint32_t>(iface));
  ok &=
      writeBank0(tmc9660::tmcl::GlobalParamBank0::HEARTBEAT_MONITORING_TIMEOUT,
                 timeout_ms);
  return ok;
}

bool TMC9660::Globals::getHeartbeat(
    tmc9660::tmcl::HeartbeatMonitoringConfig &iface,
    uint32_t &timeout_ms) noexcept {
  uint32_t cfg;
  bool ok = readBank0(
      tmc9660::tmcl::GlobalParamBank0::HEARTBEAT_MONITORING_CONFIG, cfg);
  iface = static_cast<tmc9660::tmcl::HeartbeatMonitoringConfig>(cfg);
  ok &= readBank0(
      tmc9660::tmcl::GlobalParamBank0::HEARTBEAT_MONITORING_TIMEOUT,
      timeout_ms);
  return ok;
}

bool TMC9660::Globals::setIODirectionMask(uint32_t mask) noexcept {
  return writeBank0(tmc9660::tmcl::GlobalParamBank0::IO_DIRECTION_MASK, mask);
}

bool TMC9660::Globals::getIODirectionMask(uint32_t &mask) noexcept {
  return readBank0(tmc9660::tmcl::GlobalParamBank0::IO_DIRECTION_MASK, mask);
}

bool TMC9660::Globals::setPullEnableMask(uint32_t mask) noexcept {
  return writeBank0(
      tmc9660::tmcl::GlobalParamBank0::IO_INPUT_PULLUP_PULLDOWN_ENABLE_MASK,
      mask);
}

bool TMC9660::Globals::getPullEnableMask(uint32_t &mask) noexcept {
  return readBank0(
      tmc9660::tmcl::GlobalParamBank0::IO_INPUT_PULLUP_PULLDOWN_ENABLE_MASK,
      mask);
}

bool TMC9660::Globals::setPullDirectionMask(uint32_t mask) noexcept {
  return writeBank0(
      tmc9660::tmcl::GlobalParamBank0::IO_INPUT_PULLUP_PULLDOWN_DIRECTION_MASK,
      mask);
}

bool TMC9660::Globals::getPullDirectionMask(uint32_t &mask) noexcept {
  return readBank0(
      tmc9660::tmcl::GlobalParamBank0::IO_INPUT_PULLUP_PULLDOWN_DIRECTION_MASK,
      mask);
}

bool TMC9660::Globals::setAutoStart(bool enable) noexcept {
  return writeBank0(tmc9660::tmcl::GlobalParamBank0::AUTO_START_ENABLE,
                    enable ? 1u : 0u);
}

bool TMC9660::Globals::getAutoStart(bool &enable) noexcept {
  uint32_t tmp;
  if (!readBank0(tmc9660::tmcl::GlobalParamBank0::AUTO_START_ENABLE, tmp))
    return false;
  enable = (tmp != 0);
  return true;
}

bool TMC9660::Globals::setClearUserVariables(bool clear) noexcept {
  return writeBank0(tmc9660::tmcl::GlobalParamBank0::CLEAR_USER_VARIABLES,
                    clear ? 1u : 0u);
}

bool TMC9660::Globals::getClearUserVariables(bool &clear) noexcept {
  uint32_t tmp;
  if (!readBank0(tmc9660::tmcl::GlobalParamBank0::CLEAR_USER_VARIABLES, tmp))
    return false;
  clear = (tmp != 0);
  return true;
}

bool TMC9660::Globals::setUserVariable(uint8_t index, int32_t value) noexcept {
  if (index > 15)
    return false;
  auto param = static_cast<tmc9660::tmcl::GlobalParamBank2>(index);
  return writeBank2(param, value);
}

bool TMC9660::Globals::getUserVariable(uint8_t index, int32_t &value) noexcept {
  if (index > 15)
    return false;
  auto param = static_cast<tmc9660::tmcl::GlobalParamBank2>(index);
  return readBank2(param, value);
}

bool TMC9660::Globals::setTimerPeriod(uint8_t timer, uint32_t period_ms) noexcept {
  if (timer > 2)
    return false;
  auto param = static_cast<tmc9660::tmcl::GlobalParamBank3>(timer);
  return writeBank3(param, period_ms);
}

bool TMC9660::Globals::getTimerPeriod(uint8_t timer, uint32_t &period_ms) noexcept {
  if (timer > 2)
    return false;
  auto param = static_cast<tmc9660::tmcl::GlobalParamBank3>(timer);
  return readBank3(param, period_ms);
}

bool TMC9660::Globals::setInputTrigger(
    uint8_t index, tmc9660::tmcl::TriggerTransition transition) noexcept {
  if (index > 18)
    return false;
  auto param = static_cast<tmc9660::tmcl::GlobalParamBank3>(
      static_cast<uint16_t>(tmc9660::tmcl::GlobalParamBank3::INPUT_0_TRIGGER_TRANSITION) +
      index);
  return writeBank3(param, static_cast<uint32_t>(transition));
}

bool TMC9660::Globals::getInputTrigger(
    uint8_t index, tmc9660::tmcl::TriggerTransition &transition) noexcept {
  if (index > 18)
    return false;
  auto param = static_cast<tmc9660::tmcl::GlobalParamBank3>(
      static_cast<uint16_t>(tmc9660::tmcl::GlobalParamBank3::INPUT_0_TRIGGER_TRANSITION) +
      index);
  uint32_t tmp;
  if (!readBank3(param, tmp))
    return false;
  transition = static_cast<tmc9660::tmcl::TriggerTransition>(tmp);
  return true;
}

//-------------------------------------------------------------------------
// Heartbeat and Power helpers (wrapping Globals)
//-------------------------------------------------------------------------

bool TMC9660::Heartbeat::configure(HeartbeatMode mode,
                                   uint32_t timeout_ms) noexcept {
  using tmc9660::tmcl::HeartbeatMonitoringConfig;
  HeartbeatMonitoringConfig cfg =
      (mode == HeartbeatMode::WATCHDOG_ENABLE)
          ? HeartbeatMonitoringConfig::TMCL_UART_AND_SPI_INTERFACE
          : HeartbeatMonitoringConfig::DISABLED;
  return driver.globals.configureHeartbeat(cfg, timeout_ms);
}

bool TMC9660::Power::enableWakePin(bool enable) noexcept {
  return driver.globals.writeBank0(
      tmc9660::tmcl::GlobalParamBank0::WAKE_PIN_CONTROL_ENABLE,
      enable ? 1u : 0u);
}

bool TMC9660::Power::enterPowerDown(PowerDownPeriod period) noexcept {
  return driver.globals.writeBank0(
      tmc9660::tmcl::GlobalParamBank0::GO_TO_TIMEOUT_POWER_DOWN_STATE,
      static_cast<uint32_t>(period));
}

//-------------------------------------------------------------------------
// GPIO helpers
//-------------------------------------------------------------------------

bool TMC9660::GPIO::setMode(uint8_t pin, bool output, bool pullEnable,
                            bool pullUp) noexcept {
  if (pin > 18)
    return false;
  uint32_t bit = 1u << pin;
  uint32_t mask;
  bool ok = driver.globals.getIODirectionMask(mask);
  if (!ok)
    return false;
  mask = output ? (mask | bit) : (mask & ~bit);
  ok &= driver.globals.setIODirectionMask(mask);

  ok &= driver.globals.getPullEnableMask(mask);
  if (!ok)
    return false;
  mask = pullEnable ? (mask | bit) : (mask & ~bit);
  ok &= driver.globals.setPullEnableMask(mask);

  ok &= driver.globals.getPullDirectionMask(mask);
  if (!ok)
    return false;
  mask = pullUp ? (mask | bit) : (mask & ~bit);
  ok &= driver.globals.setPullDirectionMask(mask);
  return ok;
}

bool TMC9660::GPIO::writePin(uint8_t pin, bool value) noexcept {
  if (pin > 18)
    return false;
  return driver.sendCommand(OP_SIO, pin, 0, value ? 1u : 0u, nullptr);
}

bool TMC9660::GPIO::readDigital(uint8_t pin, bool &value) noexcept {
  if (pin > 18)
    return false;
  uint32_t v;
  if (!driver.sendCommand(OP_GIO, pin, 0, 0, &v))
    return false;
  value = (v != 0);
  return true;
}

bool TMC9660::GPIO::readAnalog(uint8_t pin, uint16_t &value) noexcept {
  if (pin > 18)
    return false;
  uint32_t v;
  if (!driver.sendCommand(OP_GIO, pin, 1, 0, &v))
    return false;
  value = static_cast<uint16_t>(v);
  return true;
}

// TMC9660.cpp - Implementation of TMC9660 motor controller interface
