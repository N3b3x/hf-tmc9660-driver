#include "TMC9660.hpp"
#include <cmath>

// Define TMCL operation codes for convenience
static constexpr uint8_t OP_NOP = 0;
static constexpr uint8_t OP_MST = 3;
static constexpr uint8_t OP_SAP = 5;
static constexpr uint8_t OP_GAP = 6;
static constexpr uint8_t OP_STAP = 7; // Store All Parameters (not used here)
static constexpr uint8_t OP_SGP = 9;
static constexpr uint8_t OP_GGP = 10;
static constexpr uint8_t OP_RFS = 13;  // Reference Search (not explicitly used)
static constexpr uint8_t OP_SIO = 14;  // Set IO (not used)
static constexpr uint8_t OP_GIO = 15;  // Get IO (not used)
static constexpr uint8_t OP_RAMDEBUG = 142;
static constexpr uint8_t OP_STOP_SCRIPT = 28;
static constexpr uint8_t OP_CALCX = 33;  // Arithmetic operations (script use)
static constexpr uint8_t OP_AAP = 34;    // Accumulator to Axis Parameter (script)
static constexpr uint8_t OP_AGP = 35;    // Accumulator to Global Parameter (script)
static constexpr uint8_t OP_RST = 48;    // Restart script from address
static constexpr uint8_t OP_DOWNLOAD_START = 132;
static constexpr uint8_t OP_DOWNLOAD_END   = 133;
static constexpr uint8_t OP_READ_MEM      = 134;
static constexpr uint8_t OP_GET_SCRIPT_STATUS = 135;

TMC9660::TMC9660(TMC9660CommInterface& comm, uint8_t address)
    : comm_(comm), address_(address & 0x7F) // ensure address is 7-bit
{
}

bool TMC9660::writeParameter(uint16_t id, uint32_t value, uint8_t motorIndex) {
    return sendCommand(OP_SAP, id, motorIndex, value, nullptr);
}

bool TMC9660::readParameter(uint16_t id, uint32_t& value, uint8_t motorIndex) {
    return sendCommand(OP_GAP, id, motorIndex, 0, &value);
}

bool TMC9660::writeGlobalParameter(uint16_t id, uint8_t bank, uint32_t value) {
    return sendCommand(OP_SGP, id, bank, value, nullptr);
}

bool TMC9660::readGlobalParameter(uint16_t id, uint8_t bank, uint32_t& value) {
    return sendCommand(OP_GGP, id, bank, 0, &value);
}

bool TMC9660::configureMotorType(MotorType type, uint8_t polePairs) {
    // Set motor type
    uint16_t typeVal = static_cast<uint16_t>(type);
    if (!writeParameter(0, typeVal)) {
        return false;
    }
    // If BLDC or Stepper, set pole pairs (for stepper, treat as 1 pole pair by default).
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

bool TMC9660::setCurrentLoopGains(uint16_t p, uint16_t i, bool separateFlux, uint16_t fluxP, uint16_t fluxI) {
    bool ok = true;
    if (separateFlux) {
        // Enable separate torque/flux PI control parameters
        ok &= writeParameter(113, 1);
        // Set torque PI gains
        ok &= writeParameter(109, p);
        ok &= writeParameter(110, i);
        // Set flux PI gains (if provided; otherwise use same as torque if fluxP or fluxI is zero)
        uint16_t useFluxP = (fluxP != 0 ? fluxP : p);
        uint16_t useFluxI = (fluxI != 0 ? fluxI : i);
        ok &= writeParameter(111, useFluxP);
        ok &= writeParameter(112, useFluxI);
    } else {
        // Use combined torque/flux PI parameters
        ok &= writeParameter(113, 0);
        ok &= writeParameter(109, p);
        ok &= writeParameter(110, i);
        // When combined, flux PI uses the same values as torque PI (no separate settings needed).
    }
    return ok;
}

bool TMC9660::setVelocityLoopGains(uint16_t p, uint16_t i) {
    bool ok = true;
    ok &= writeParameter(127, p);
    ok &= writeParameter(128, i);
    return ok;
}

bool TMC9660::setPositionGain(uint16_t p) {
    return writeParameter(146, p);
}

bool TMC9660::configureGateDriver(uint8_t sinkLevel, uint8_t sourceLevel, uint16_t deadTimeNs) {
    if (sinkLevel > 15) sinkLevel = 15;
    if (sourceLevel > 15) sourceLevel = 15;
    // Calculate dead-time value (in units of ~8.33 ns per step).
    int deadSteps = static_cast<int>(std::lround(deadTimeNs / 8.33));
    if (deadSteps < 0) deadSteps = 0;
    if (deadSteps > 255) deadSteps = 255;
    uint8_t dt = static_cast<uint8_t>(deadSteps);
    bool ok = true;
    // Set break-before-make time for all FETs (both UVW and Y2, low and high sides) to the same value.
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

bool TMC9660::configureHallSensors(uint8_t sectorOffset, bool inverted, bool enableExtrapolation, uint8_t filterLength) noexcept {
    if (sectorOffset > 5) sectorOffset = 0; // valid 0-5
    bool ok = true;
    ok &= writeParameter(75, sectorOffset);
    ok &= writeParameter(83, inverted ? 1u : 0u);
    ok &= writeParameter(76, filterLength);
    ok &= writeParameter(84, enableExtrapolation ? 1u : 0u);
    return ok;
}

bool TMC9660::configureEncoder(uint32_t countsPerRev, bool inverted, bool nChannelInverted) noexcept {
    uint32_t steps = countsPerRev;
    if (steps > 0xFFFFFF) steps = 0xFFFFFF;
    bool ok = true;
    ok &= writeParameter(90, steps);
    ok &= writeParameter(91, inverted ? 1u : 0u);
    ok &= writeParameter(92, nChannelInverted ? 1u : 0u);
    return ok;
}

bool TMC9660::configureSPIEncoder(uint8_t cmdSize, uint16_t csSettleTimeNs, uint8_t csIdleTimeUs) noexcept {
    bool ok = true;
    ok &= writeParameter(180, cmdSize);
    ok &= writeParameter(181, csSettleTimeNs);
    ok &= writeParameter(182, csIdleTimeUs);
    return ok;
}

bool TMC9660::configureVoltageProtection(uint16_t overVoltThreshold, uint16_t underVoltThreshold) {
    bool ok = true;
    // Global parameters 291 (OV warning) and 292 (UV warning) in 0.1V units.
    ok &= writeGlobalParameter(291, 0, overVoltThreshold);
    ok &= writeGlobalParameter(292, 0, underVoltThreshold);
    return ok;
}

bool TMC9660::configureTemperatureProtection(float warningDegC, float shutdownDegC) {
    // Convert Celsius to raw sensor units: val = (Temp + 268.15) / 0.01615
    float warnVal = (warningDegC + 268.15f) / 0.01615f;
    float shutVal = (shutdownDegC + 268.15f) / 0.01615f;
    if (warnVal < 0) warnVal = 0;
    if (warnVal > 65535) warnVal = 65535;
    if (shutVal < 0) shutVal = 0;
    if (shutVal > 65535) shutVal = 65535;
    uint16_t warnRaw = static_cast<uint16_t>(std::lround(warnVal));
    uint16_t shutRaw = static_cast<uint16_t>(std::lround(shutVal));
    bool ok = true;
    ok &= writeGlobalParameter(298, 0, warnRaw);
    ok &= writeGlobalParameter(297, 0, shutRaw);
    return ok;
}

bool TMC9660::setOvercurrentProtectionEnabled(bool enabled) {
    uint8_t val = enabled ? 1 : 0;
    bool ok = true;
    ok &= writeParameter(254, val);
    ok &= writeParameter(255, val);
    ok &= writeParameter(256, val);
    ok &= writeParameter(257, val);
    return ok;
}

bool TMC9660::uploadScript(const std::vector<uint32_t>& scriptData) {
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

bool TMC9660::readRAMDebugData(uint32_t index, uint32_t& data) {
    // Use ReadMem command to read memory at given index (assuming index is an address in the debug buffer)
    return sendCommand(OP_READ_MEM, 0, 0, index, &data);
}

bool TMC9660::getRAMDebugStatus(bool& isRunning) {
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

bool TMC9660::sendCommand(uint8_t opcode, uint16_t type, uint8_t motor, uint32_t value, uint32_t* reply) {
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
                           (static_cast<uint32_t>(rx[6]) << 8)  |
                           (static_cast<uint32_t>(rx[7]));
        *reply = respVal;
    }
    return true;
}
// TMC9660.cpp - Implementation of TMC9660 motor controller interface