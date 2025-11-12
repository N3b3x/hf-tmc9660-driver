#include "../inc/TMC9660.hpp"
#include <chrono>
#include <cmath>
#include <thread>

using namespace tmc9660;

/**
 * @brief Construct a TMC9660 driver instance.
 *
 * Initializes the TMC9660 driver with the specified communication interface
 * and optional bootloader configuration. The driver will automatically
 * create a bootloader instance for SPI and UART interfaces.
 *
 * @param comm Reference to a user-implemented communication interface
 * @param address Module address for multi-device systems (0-127, masked to 7 bits)
 * @param bootCfg Optional bootloader configuration (can be set later)
 */
TMC9660::TMC9660(TMC9660CommInterface& comm, uint8_t address,
                 const BootloaderConfig* bootCfg) noexcept
    : comm_(comm), address_(address & 0x7F), bootCfg_(bootCfg) {
  // Initialize bootloader for SPI and UART interfaces
  if (comm.mode() == CommMode::SPI || comm.mode() == CommMode::UART) {
    bootloader_ = std::make_unique<TMC9660Bootloader>(comm);
  }
}

/**
 * @brief Destructor for TMC9660 driver.
 *
 * Cleans up resources and ensures proper shutdown of the motor driver.
 * The destructor will automatically disable the motor if it's currently running.
 */
TMC9660::~TMC9660() noexcept {
  // Destructor does not need to do anything special.
  // The motor should be explicitly disabled before destruction if needed.
}

/**
 * @brief Initialize the TMC9660 bootloader and configure the device for parameter mode operation.
 *
 * This method performs the complete bootloader initialization sequence including hardware reset,
 * bootloader configuration, verification, and transition to parameter mode. It handles the
 * complex multi-step process required to bring the TMC9660 from power-on to ready state.
 *
 * The initialization process includes:
 * - Hardware reset sequence (optional)
 * - Bootloader configuration upload
 * - Configuration verification
 * - Bootloader information retrieval (optional)
 * - Transition to parameter mode
 * - TMCL communication verification
 *
 * @param cfg Bootloader configuration to apply (nullptr uses stored config)
 * @param performReset Whether to perform hardware reset sequence
 * @param retrieveBootloaderInfo Whether to retrieve bootloader version info
 * @param failOnVerifyError Whether to fail if configuration verification fails
 * @return BootloaderInitResult indicating success or failure reason
 */
TMC9660::BootloaderInitResult TMC9660::bootloaderInit(const BootloaderConfig* cfg,
                                                      bool performReset,
                                                      bool retrieveBootloaderInfo,
                                                      bool failOnVerifyError) noexcept {
  const BootloaderConfig* useCfg = cfg ? cfg : bootCfg_;
  if (!useCfg)
    return BootloaderInitResult::NoConfig;
  if (!bootloader_) {
    // Bootloader only works with SPI/UART interface
    return BootloaderInitResult::Failure;
  }

  TMCLFrame statusFrame{};
  statusFrame.opcode =
      static_cast<uint8_t>(tmc9660::tmcl::Op::GetStatusScript); // GetStatusScript command

  // ======================================================================
  // STEP 1: Hardware Reset Sequence (if requested)
  // ======================================================================
  if (performReset) {
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "Performing hardware reset sequence...");

    // 1.1: Assert RST (active high) to enter reset state
    if (!comm_.gpioSetActive(TMC9660CtrlPin::RST)) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660", "Failed to assert RST pin");
      return BootloaderInitResult::Failure;
    }
    comm_.delayMs(10); // Hold reset for 10ms

    // 1.2: Release RST to exit reset state
    if (!comm_.gpioSetInactive(TMC9660CtrlPin::RST)) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660", "Failed to release RST pin");
      return BootloaderInitResult::Failure;
    }

    // 1.3: Wait for FAULTN to go inactive (chip ready)
    TMC9660_LOG_DEBUG(comm_, 3, "TMC9660", "Waiting for FAULTN to go inactive (chip ready)...");
    bool chipReady = false;
    for (int i = 0; i < 100; ++i) { // Timeout after 1 second
      GpioSignal faultnSignal;
      if (!comm_.gpioRead(TMC9660CtrlPin::FAULTN, faultnSignal)) {
        TMC9660_LOG_DEBUG(comm_, 1, "TMC9660", "⚠️  Failed to read FAULTN pin - continuing anyway");
        break; // Don't fail, just continue without FAULTN monitoring
      }

      // FAULTN is active-low, so INACTIVE means chip is ready
      if (faultnSignal == GpioSignal::INACTIVE) {
        chipReady = true;
        TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "✅ FAULTN inactive - chip ready after %d ms",
                          i * 10);
        break;
      }
      comm_.delayMs(10);
    }

    if (!chipReady) {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660",
                        "⚠️  FAULTN did not go inactive within timeout - continuing anyway");
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660",
                        "   (FAULTN may not be configured or chip may be in different state)");
    }

    // Small delay for chip to fully stabilize in bootloader
    comm_.delayMs(50);
  } else {
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "Skipping hardware reset (performReset=false)");
  }

  // ======================================================================
  // STEP 2: Detect Current Mode (Bootloader vs Parameter Mode)
  // ======================================================================
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "Detecting chip mode (bootloader vs parameter)...");

  // Additional delay for UART communication to stabilize after reset
  // UART requires more setup time than SPI
  if (comm_.mode() == CommMode::UART) {
    comm_.delayMs(100);
    TMC9660_LOG_DEBUG(comm_, 3, "TMC9660", "UART mode: Added 100ms stabilization delay");
  }

  bool inBootloaderMode = false;
  bool inParameterMode = false;

  TMCLReply firstReply{}, finalReply{};
  bool tmclSuccess = comm_.transferTMCL(statusFrame, finalReply, address_, &firstReply);

  uint8_t rawStatus = firstReply.rawBytes[0];
  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660", "First reply raw status: 0x%02X (TMCL parse %s)",
                    rawStatus, tmclSuccess ? "succeeded" : "failed");

  // For UART: If TMCL transfer completely failed (no response), chip is likely in bootloader mode
  // For SPI: Always get a response due to full-duplex nature, check status codes
  if (comm_.mode() == CommMode::UART && !tmclSuccess) {
    inBootloaderMode = true;
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "Chip in BOOTLOADER mode (UART: no TMCL response)");
  } else if (rawStatus == 0x13) {
    inBootloaderMode = true;
    uint32_t version = firstReply.extractRawValue(1);
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660",
                      "Chip in BOOTLOADER mode (SESSION_START), version: %d.%d",
                      (version >> 16) & 0xFFFF, version & 0xFFFF);
  } else if (rawStatus == 0x0C) {
    inParameterMode = true;
    uint32_t version = tmclSuccess ? firstReply.value : firstReply.extractRawValue(1);
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "Chip in PARAMETER mode (FIRST_CMD), version: %d.%d",
                      (version >> 16) & 0xFFFF, version & 0xFFFF);
  } else if (rawStatus == 0xFF && tmclSuccess &&
             (firstReply.status == 100 || firstReply.status == 101)) {
    inParameterMode = true;
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "Chip in PARAMETER mode (initialized)");
  } else if (rawStatus == 0x00 && comm_.mode() == CommMode::SPI) {
    // For SPI: 0x00 can indicate bootloader OK status
    inBootloaderMode = true;
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "Chip in BOOTLOADER mode (SPI: OK status 0x00)");
  } else {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660",
                      "Unable to detect chip mode (unknown status: 0x%02X, TMCL %s)", rawStatus,
                      tmclSuccess ? "succeeded" : "failed");
    return BootloaderInitResult::Failure;
  }

  // ======================================================================
  // STEP 3: Handle Parameter Mode Scenarios
  // ======================================================================
  if (inParameterMode && !inBootloaderMode) {
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660",
                      "Chip in parameter mode, transitioning to bootloader...");

    // Send Boot command to enter bootloader
    if (!enterBootloaderMode()) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660", "Failed to send Boot command");
      return BootloaderInitResult::Failure;
    }

    // Wait for transition to complete
    comm_.delayMs(150);

    // Verify we're now in bootloader mode
    uint32_t replyAfterBoot = 0;
    if (!bootloader_->noOp(&replyAfterBoot)) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660", "Failed to verify bootloader mode after Boot command");
      return BootloaderInitResult::Failure;
    }

    uint8_t statusAfterBoot = static_cast<uint8_t>((replyAfterBoot >> 24) & 0xFF);
    if (statusAfterBoot != 0x15) { // BOOTLOADER_RESUMED status
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660", "⚠️  Expected BOOTLOADER_RESUMED (0x15), got 0x%02X",
                        statusAfterBoot);
    } else {
      TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "✅ Successfully entered bootloader mode");
    }

    inBootloaderMode = true;
  }

  // ======================================================================
  // STEP 4: Apply Configuration via Bootloader
  // ======================================================================
  if (inBootloaderMode || !inParameterMode) {

    // ======================================================================
    // STEP 4.5: Retrieve Bootloader Information (if requested)
    // ======================================================================
    if (retrieveBootloaderInfo) {
      TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "Retrieving bootloader information...");
      if (!bootloader_->getAllBootloaderInfo()) {
        TMC9660_LOG_DEBUG(comm_, 1, "TMC9660", "⚠️  Failed to retrieve some bootloader information");
      }
    } else {
      TMC9660_LOG_DEBUG(comm_, 2, "TMC9660",
                        "Skipping bootloader information retrieval (retrieveBootloaderInfo=false)");
    }

    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "Applying bootloader configuration...");
    if (!bootloader_->applyConfiguration(*useCfg, failOnVerifyError)) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660", "Failed to apply bootloader configuration");
      return BootloaderInitResult::Failure;
    }
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "✅ Bootloader configuration applied successfully");
  }

  // ======================================================================
  // STEP 5: Check if motor control should be started
  // ======================================================================
  if (!useCfg->boot.start_motor_control) {
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660",
                      "✅ Bootloader initialization complete (motor control NOT started)");
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660",
                      "   cfg.boot.start_motor_control=false - chip remains in bootloader mode");
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "   Call bootloader_->startMotorControl() when ready");
    return BootloaderInitResult::Success;
  }

  // Motor control will be started automatically by Bootloader after applyConfiguration() when
  // cfg.boot.start_motor_control=true is written to the boot config register
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "✅ Bootloader initialization complete");
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660",
                    "   Motor control started automatically via cfg.boot.start_motor_control=true");

  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "⚠️  Waiting for initialization...");
  comm_.delayMs(100);

  // ======================================================================
  // STEP 7: Wait for Motor Control Initialization and Verify Communication
  // ======================================================================
  // For SPI: Consume SESSION_START and wait for SPI_STATUS_OK
  // For UART: Simply verify TMCL communication works (no status codes)

  if (comm_.mode() == CommMode::SPI) {
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660",
                      "SPI mode: Waiting for SESSION_START and consuming status codes...");

    // Poll for chip readiness and SESSION_START (SPI-specific)
    bool sessionStartReceived = false;
    for (int attempt = 0; attempt < 50; ++attempt) { // Timeout after 5 seconds (50 * 100ms)
      TMCLReply firstReply2{}, finalReply2{};
      comm_.transferTMCL(statusFrame, finalReply2, address_, &firstReply2);

      uint8_t rawStatus2 = finalReply2.rawBytes[0]; // Check the reply to the CURRENT command
      TMC9660_LOG_DEBUG(comm_, 3, "TMC9660", "Attempt %d/50: Current reply raw status: 0x%02X",
                        attempt + 1, rawStatus2);

      if (rawStatus2 == 0x0C) {
        TMC9660_LOG_DEBUG(comm_, 2, "TMC9660",
                          "✅ Parameter mode SESSION_START (0x0C) received after %d attempts",
                          attempt + 1);
        sessionStartReceived = true;
        break;
      } else if (rawStatus2 == 0xFF) {
        TMC9660_LOG_DEBUG(comm_, 2, "TMC9660",
                          "✅ SPI_STATUS_OK (0xFF) received after %d attempts - chip ready",
                          attempt + 1);
        sessionStartReceived = true;
        break;
      } else if (rawStatus2 == 0xF0) {
        TMC9660_LOG_DEBUG(comm_, 3, "TMC9660", "SPI_STATUS_NOT_READY (0xF0) - waiting...");
        comm_.delayMs(100);
        continue;
      } else if (rawStatus2 == 0x13) {
        TMC9660_LOG_DEBUG(comm_, 1, "TMC9660",
                          "⚠️  Still in bootloader mode (0x13) - motor control not ready yet");
        comm_.delayMs(100);
        continue;
      } else if (rawStatus2 == 0x00) {
        TMC9660_LOG_DEBUG(comm_, 1, "TMC9660",
                          "⚠️  Bootloader OK (0x00) - motor control not ready yet");
        comm_.delayMs(100);
        continue;
      } else {
        TMC9660_LOG_DEBUG(comm_, 1, "TMC9660", "⚠️  Unexpected status 0x%02X - waiting...",
                          rawStatus2);
        comm_.delayMs(100);
        continue;
      }
    }

    if (!sessionStartReceived) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660",
                        "❌ Timeout waiting for SESSION_START or SPI_STATUS_OK");
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660", "   Motor control may not have started properly");
      return BootloaderInitResult::Failure;
    }
  } else {
    // UART mode: No SESSION_START or status codes, just wait for motor control to initialize
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660",
                      "UART mode: Waiting for motor control initialization...");
    comm_.delayMs(100); // Give motor control time to start (adjust as needed)
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "✅ Motor control initialization delay completed");
  }

  // ======================================================================
  // STEP 8: Verify TMCL Communication with GetVersion
  // ======================================================================
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "Verifying TMCL communication with GetVersion...");

  // First, for UART mode, let's verify we're NOT still in bootloader mode
  if (comm_.mode() == CommMode::UART) {
    TMC9660_LOG_DEBUG(comm_, 2, "TMC9660",
                      "UART mode: Testing if chip is still in bootloader mode...");
    tmc9660::BootloaderVersion bootloaderVer;
    if (bootloader_->getBootloaderVersion(&bootloaderVer)) {
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660", "❌ ERROR: Chip is STILL in bootloader mode!");
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660", "   Bootloader version: %d.%d", bootloaderVer.major,
                        bootloaderVer.minor);
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660", "   Motor control did NOT start properly");
      TMC9660_LOG_DEBUG(comm_, 0, "TMC9660",
                        "   Check boot configuration: cfg.boot.start_motor_control should be true");
      return BootloaderInitResult::Failure;
    } else {
      TMC9660_LOG_DEBUG(comm_, 2, "TMC9660",
                        "✅ Bootloader commands fail (good - we're in parameter mode)");
    }
  }

  // Try a simple command first (MST - Motor Stop)
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "Testing TMCL communication with MST (Motor Stop)...");
  if (!sendCommand(tmc9660::tmcl::Op::MST, 0, 0, 0, nullptr)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660", "❌ TMCL communication failed (MST command failed)");
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660", "   Motor control is not responding to TMCL commands");
    return BootloaderInitResult::Failure;
  }
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "✅ MST command successful - TMCL communication working!");

  // Now try GetVersion
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "Getting firmware version with GetVersion...");
  uint32_t version = 0;
  if (!sendCommand(tmc9660::tmcl::Op::GetVersion, 0, 0, 0, &version)) {
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660", "❌ GetVersion failed (but MST worked)");
    TMC9660_LOG_DEBUG(comm_, 0, "TMC9660", "   Basic TMCL works, but GetVersion has issues");
    return BootloaderInitResult::Failure;
  }

  // GetVersion success is already logged in sendCommand() with the version string
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "✅ TMCL communication verified - GetVersion successful");
  TMC9660_LOG_DEBUG(comm_, 2, "TMC9660",
                    "✅ Chip fully initialized and ready for motor control commands");

  return BootloaderInitResult::Success;
}

//***************************************************************************
//**               CORE PARAMETER ACCESS METHODS                         **//
//***************************************************************************

/**
 * @brief Write a parameter value to the TMC9660 using TMCL SAP command.
 *
 * Sends a Set Axis Parameter (SAP) command to configure motor-specific parameters.
 * This is the primary method for configuring motor control parameters such as
 * motor type, pole pairs, PWM frequency, and control loop settings.
 *
 * @param id Parameter identifier from tmc9660::tmcl::Parameters enum
 * @param value Parameter value to write
 * @param motorIndex Motor index (typically 0 for single motor systems)
 * @return true if parameter was written successfully
 */
bool TMC9660::writeParameter(tmc9660::tmcl::Parameters id, uint32_t value,
                             uint8_t motorIndex) noexcept {
  return sendCommand(tmc9660::tmcl::Op::SAP, static_cast<uint16_t>(id), motorIndex, value, nullptr);
}

/**
 * @brief Read a parameter value from the TMC9660 using TMCL GAP command.
 *
 * Sends a Get Axis Parameter (GAP) command to retrieve motor-specific parameters.
 * This method is used to read back configuration values and monitor system status.
 *
 * @param id Parameter identifier from tmc9660::tmcl::Parameters enum
 * @param value Reference to store the retrieved parameter value
 * @param motorIndex Motor index (typically 0 for single motor systems)
 * @return true if parameter was read successfully
 */
bool TMC9660::readParameter(tmc9660::tmcl::Parameters id, uint32_t& value,
                            uint8_t motorIndex) noexcept {
  return this->sendCommand(tmc9660::tmcl::Op::GAP, static_cast<uint16_t>(id), motorIndex, 0,
                           &value);
}

/**
 * @brief Write a global parameter value using TMCL SGP command.
 *
 * Sends a Set Global Parameter (SGP) command to configure module-wide settings
 * such as communication parameters, GPIO configuration, and system settings.
 * Global parameters affect the entire TMC9660 module, not just motor control.
 *
 * @param id Global parameter identifier (supports multiple parameter types)
 * @param bank Parameter bank (0=module, 2=user, 3=system)
 * @param value Parameter value to write
 * @return true if parameter was written successfully
 */
bool TMC9660::writeGlobalParameter(GlobalParamBankVariant id, uint8_t bank,
                                   uint32_t value) noexcept {
  uint16_t paramId =
      std::visit([](auto&& arg) -> uint16_t { return static_cast<uint16_t>(arg); }, id);
  return this->sendCommand(tmc9660::tmcl::Op::SGP, paramId, bank, value, nullptr);
}

/**
 * @brief Read a global parameter value using TMCL GGP command.
 *
 * Sends a Get Global Parameter (GGP) command to retrieve module-wide settings
 * and system information. This method is used to read back global configuration
 * values and monitor system status across different parameter banks.
 *
 * @param id Global parameter identifier (supports multiple parameter types)
 * @param bank Parameter bank (0=module, 2=user, 3=system)
 * @param value Reference to store the retrieved parameter value
 * @return true if parameter was read successfully
 */
bool TMC9660::readGlobalParameter(GlobalParamBankVariant id, uint8_t bank,
                                  uint32_t& value) noexcept {
  uint16_t paramId =
      std::visit([](auto&& arg) -> uint16_t { return static_cast<uint16_t>(arg); }, id);
  return this->sendCommand(tmc9660::tmcl::Op::GGP, paramId, bank, 0, &value);
}

/**
 * @brief Send a raw TMCL command and optionally receive a reply.
 *
 * This is the core communication method that handles all TMCL command transmission
 * and reply processing. It provides low-level access to the TMCL protocol for
 * advanced users who need direct command control.
 *
 * The method handles:
 * - TMCL frame assembly and transmission
 * - Reply reception and validation
 * - Special command handling (e.g., GetVersion string format)
 * - Debug logging of command/reply pairs
 * - Error detection and reporting
 *
 * @param opcode TMCL operation code (SAP, GAP, SGP, GGP, etc.)
 * @param type Parameter type or command-specific data
 * @param motor Motor index or bank identifier
 * @param value Command value or parameter data
 * @param reply Optional pointer to store reply value
 * @return true if command was sent and reply was received successfully
 */
bool TMC9660::sendCommand(tmc9660::tmcl::Op opcode, uint16_t type, uint8_t motor, uint32_t value,
                          uint32_t* reply) noexcept {
  TMCLFrame tx{};
  tx.opcode = static_cast<uint8_t>(opcode);
  tx.type = type;
  tx.motor = motor;
  tx.value = value;

  // Debug logging for TMCL frame assembly
  const char* opName = tmc9660::tmcl::to_string(static_cast<tmc9660::tmcl::Op>(tx.opcode));

  // Format type field based on the operation type
  char typeStr[80];
  switch (tx.opcode) {
  case static_cast<uint8_t>(tmc9660::tmcl::Op::SAP): // Set Axis Parameter
  case static_cast<uint8_t>(tmc9660::tmcl::Op::GAP): // Get Axis Parameter
  case static_cast<uint8_t>(tmc9660::tmcl::Op::AAP): // Accumulator to Axis Parameter
  {
    const char* paramName =
        tmc9660::tmcl::to_string(static_cast<tmc9660::tmcl::Parameters>(tx.type));
    snprintf(typeStr, sizeof(typeStr), "%s (0x%04X, %u)", paramName, tx.type, tx.type);
  } break;
  case static_cast<uint8_t>(tmc9660::tmcl::Op::SGP): // Set Global Parameter
  case static_cast<uint8_t>(tmc9660::tmcl::Op::GGP): // Get Global Parameter
  case static_cast<uint8_t>(tmc9660::tmcl::Op::AGP): // Accumulator to Global Parameter
    snprintf(typeStr, sizeof(typeStr), "GlobalParam=0x%04X", tx.type);
    break;
  default:
    snprintf(typeStr, sizeof(typeStr), "0x%04X", tx.type);
    break;
  }

  TMC9660_LOG_DEBUG(comm_, 3, "TMC9660",
                    "[TMCL TX] %s (Op=0x%02X), Type=%s, Motor=0x%02X, Value=0x%08X", opName,
                    tx.opcode, typeStr, tx.motor, tx.value);

  TMCLReply rep{};
  if (!comm_.transferTMCL(tx, rep, address_)) {
    TMC9660_LOG_DEBUG(comm_, 1, "TMC9660", "[TMCL] Transfer failed");
    return false;
  }

  // Debug logging for TMCL reply with decoded status and opcode
  // Note: SPI response format: byte0=SPI_STATUS, byte1=TMCL_STATUS, byte2=OPCODE, byte3-6=VALUE,
  // byte7=CHECKSUM
  const char* statusName =
      tmc9660::tmcl::to_string(static_cast<tmc9660::tmcl::ReplyCode>(rep.status));
  const char* rxOpName = tmc9660::tmcl::to_string(static_cast<tmc9660::tmcl::Op>(rep.opcode));
  TMC9660_LOG_DEBUG(
      comm_, 3, "TMC9660",
      "[TMCL RX] %s (SPI_Status=0x%02X, TMCL_Status=0x%02X), Op=%s (0x%02X), Value=0x%08X",
      statusName, static_cast<uint8_t>(rep.spiStatus), rep.status, rxOpName, rep.opcode, rep.value);

  // Special handling for GetVersion command (Type=0 returns string format)
  if (opcode == tmc9660::tmcl::Op::GetVersion && type == 0) {
    std::string versionString = rep.getVersionString();
    if (!versionString.empty()) {
      TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "✅ GetVersion successful - Firmware version: %s",
                        versionString.c_str());
      if (reply)
        *reply = 0; // No numeric value for string format
      return true;
    } else {
      TMC9660_LOG_DEBUG(comm_, 1, "TMC9660",
                        "⚠️  GetVersion string format not detected, trying standard parsing");
    }
  }

  if (!rep.isOK()) {
    // Distinguish between SPI communication failure and TMCL command failure
    if (rep.spiStatus == SPIStatus::OK) {
      // SPI communication successful, but TMCL command failed
      const char* tmclStatusName =
          tmc9660::tmcl::to_string(static_cast<tmc9660::tmcl::ReplyCode>(rep.status));
      TMC9660_LOG_DEBUG(comm_, 2, "TMC9660", "[TMCL] Command failed: %s (TMCL_Status=0x%02X)",
                        tmclStatusName, rep.status);
    } else {
      // SPI communication failure
      const char* spiStatusStr = "UNKNOWN";
      switch (rep.spiStatus) {
      case SPIStatus::CHECKSUM_ERROR:
        spiStatusStr = "CHECKSUM_ERROR";
        break;
      case SPIStatus::NOT_READY:
        spiStatusStr = "NOT_READY";
        break;
      case SPIStatus::FIRST_CMD:
        spiStatusStr = "FIRST_CMD";
        break;
      default:
        spiStatusStr = "UNKNOWN";
        break;
      }
      TMC9660_LOG_DEBUG(comm_, 2, "TMC9660",
                        "[TMCL] SPI communication failed: %s (SPI_Status=0x%02X)", spiStatusStr,
                        static_cast<uint8_t>(rep.spiStatus));
    }
    return false;
  }
  if (reply)
    *reply = rep.value;
  return true;
}

//***************************************************************************
//**                  SUBSYSTEM: Motor Configuration                     **//
//***************************************************************************

bool TMC9660::MotorConfig::setType(tmc9660::tmcl::MotorType type, uint8_t polePairs) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::MOTOR_TYPE, static_cast<uint32_t>(type));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::MOTOR_POLE_PAIRS, polePairs);
  return ok;
}

bool TMC9660::MotorConfig::setDirection(tmc9660::tmcl::MotorDirection direction) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::MOTOR_DIRECTION,
                               static_cast<uint32_t>(direction));
}

bool TMC9660::MotorConfig::setPWMFrequency(uint32_t frequencyHz) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::MOTOR_PWM_FREQUENCY, frequencyHz);
}

bool TMC9660::MotorConfig::setCommutationMode(tmc9660::tmcl::CommutationMode mode) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::COMMUTATION_MODE,
                               static_cast<uint32_t>(mode));
}

bool TMC9660::MotorConfig::setOutputVoltageLimit(uint16_t limit) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::OUTPUT_VOLTAGE_LIMIT, limit);
}

bool TMC9660::MotorConfig::getOutputVoltageLimit(uint16_t& limit) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::OUTPUT_VOLTAGE_LIMIT, v))
    return false;
  limit = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::MotorConfig::setMaxTorqueCurrent(uint16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::MAX_TORQUE, milliamps);
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
  return driver.writeParameter(tmc9660::tmcl::Parameters::IDLE_MOTOR_PWM_BEHAVIOR,
                               static_cast<uint32_t>(behavior));
}

bool TMC9660::MotorConfig::configureAuto(const MotorProfile& profile) noexcept {
  bool ok = true;

  // --- Safety Check: Ensure critical parameters are valid ---
  if (profile.maxPhaseCurrent_A <= 0.0f || profile.pwmFrequency_Hz < 10000 ||
      profile.pwmFrequency_Hz > 100000 ||
      (profile.motorType == tmc9660::tmcl::MotorType::BLDC_MOTOR && profile.polePairs == 0) ||
      (profile.motorType == tmc9660::tmcl::MotorType::STEPPER_MOTOR && profile.polePairs == 0)) {
    return false;
  }

  // Step 1: Set commutation mode to SYSTEM_OFF (required before changing motor type)
  ok &= setCommutationMode(tmc9660::tmcl::CommutationMode::SYSTEM_OFF);

  // Step 2: Set motor type and pole pairs
  ok &= setType(profile.motorType, profile.polePairs);

  // Step 3: Set motor direction
  ok &= setDirection(profile.direction);

  // Step 4: Set PWM frequency
  ok &= setPWMFrequency(profile.pwmFrequency_Hz);

  // Step 5: Set PWM switching scheme (with smart defaults based on motor type)
  tmc9660::tmcl::PwmSwitchingScheme pwmScheme = profile.pwmSwitchingScheme;
  // Auto-select default based on motor type if SVPWM is set for non-BLDC motors
  if (profile.motorType != tmc9660::tmcl::MotorType::BLDC_MOTOR &&
      pwmScheme == tmc9660::tmcl::PwmSwitchingScheme::SVPWM) {
    // SVPWM is only valid for BLDC motors, use STANDARD for DC/Stepper
    pwmScheme = tmc9660::tmcl::PwmSwitchingScheme::STANDARD;
  }
  ok &= setPWMSwitchingScheme(pwmScheme);

  // Step 6: Set idle motor PWM behavior
  ok &= setIdleMotorPWMBehavior(profile.idlePwmBehavior);

  // Step 7: Calculate and set MAX_TORQUE from maxPhaseCurrent_A
  uint16_t maxTorque_mA = static_cast<uint16_t>(profile.maxPhaseCurrent_A * 1000.0f + 0.5f);
  ok &= setMaxTorqueCurrent(maxTorque_mA);

  // Step 8: Calculate and set MAX_FLUX
  // For DC motors, flux is not used (but we still set it to avoid issues)
  // For BLDC/Stepper, default to 20% of max phase current if not specified
  float maxFlux_A = profile.maxFluxCurrent_A;
  if (std::isnan(maxFlux_A) || maxFlux_A <= 0.0f) {
    if (profile.motorType == tmc9660::tmcl::MotorType::DC_MOTOR) {
      maxFlux_A = 0.0f; // DC motors don't use flux
    } else {
      maxFlux_A = profile.maxPhaseCurrent_A * 0.2f; // 20% default for BLDC/Stepper
    }
  }
  uint16_t maxFlux_mA = static_cast<uint16_t>(maxFlux_A * 1000.0f + 0.5f);
  ok &= setMaxFluxCurrent(maxFlux_mA);

  // Step 9: Set output voltage limit
  ok &= setOutputVoltageLimit(profile.outputVoltageLimit);

  // Step 10: Apply commutation mode if specified (applied last, after all configuration)
  // This allows users to set the desired commutation mode (e.g., FOC_HALL, FOC_ENCODER)
  // which will be applied after all motor parameters are configured
  if (profile.commutationMode.has_value()) {
    ok &= setCommutationMode(profile.commutationMode.value());
  }

  return ok;
}

//***************************************************************************
//**                  SUBSYSTEM: Current Measurement                      **//
//***************************************************************************

bool TMC9660::CurrentSensing::setShuntType(tmc9660::tmcl::AdcShuntType shuntType) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::ADC_SHUNT_TYPE,
                               static_cast<uint32_t>(shuntType));
}

bool TMC9660::CurrentSensing::getShuntType(tmc9660::tmcl::AdcShuntType& shuntType) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_SHUNT_TYPE, tmp))
    return false;
  shuntType = static_cast<tmc9660::tmcl::AdcShuntType>(tmp);
  return true;
}

bool TMC9660::CurrentSensing::readRaw(int16_t& adc0, int16_t& adc1, int16_t& adc2,
                                      int16_t& adc3) noexcept {
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
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::CSA_GAIN_ADC_I0_TO_ADC_I2,
                              static_cast<uint32_t>(gain012));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::CSA_GAIN_ADC_I3,
                              static_cast<uint32_t>(gain3));
  return ok;
}

bool TMC9660::CurrentSensing::getCSAGain(tmc9660::tmcl::CsaGain& gain012,
                                         tmc9660::tmcl::CsaGain& gain3) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::CSA_GAIN_ADC_I0_TO_ADC_I2, tmp))
    return false;
  gain012 = static_cast<tmc9660::tmcl::CsaGain>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::CSA_GAIN_ADC_I3, tmp))
    return false;
  gain3 = static_cast<tmc9660::tmcl::CsaGain>(tmp);
  return true;
}

bool TMC9660::CurrentSensing::setCSAFilter(tmc9660::tmcl::CsaFilter filter012,
                                           tmc9660::tmcl::CsaFilter filter3) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::CSA_FILTER_ADC_I0_TO_ADC_I2,
                              static_cast<uint32_t>(filter012));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::CSA_FILTER_ADC_I3,
                              static_cast<uint32_t>(filter3));
  return ok;
}

bool TMC9660::CurrentSensing::getCSAFilter(tmc9660::tmcl::CsaFilter& filter012,
                                           tmc9660::tmcl::CsaFilter& filter3) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::CSA_FILTER_ADC_I0_TO_ADC_I2, tmp))
    return false;
  filter012 = static_cast<tmc9660::tmcl::CsaFilter>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::CSA_FILTER_ADC_I3, tmp))
    return false;
  filter3 = static_cast<tmc9660::tmcl::CsaFilter>(tmp);
  return true;
}

bool TMC9660::CurrentSensing::setScalingFactor(uint16_t scalingFactor) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::CURRENT_SCALING_FACTOR, scalingFactor);
}

bool TMC9660::CurrentSensing::getScalingFactor(uint16_t& scalingFactor) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::CURRENT_SCALING_FACTOR, tmp))
    return false;
  scalingFactor = static_cast<uint16_t>(tmp);
  return true;
}

bool TMC9660::CurrentSensing::setPhaseAdcMapping(tmc9660::tmcl::AdcMapping ux1,
                                                 tmc9660::tmcl::AdcMapping vx2,
                                                 tmc9660::tmcl::AdcMapping wy1,
                                                 tmc9660::tmcl::AdcMapping y2) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::PHASE_UX1_ADC_MAPPING,
                              static_cast<uint32_t>(ux1));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::PHASE_VX2_ADC_MAPPING,
                              static_cast<uint32_t>(vx2));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::PHASE_WY1_ADC_MAPPING,
                              static_cast<uint32_t>(wy1));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::PHASE_Y2_ADC_MAPPING,
                              static_cast<uint32_t>(y2));
  return ok;
}

bool TMC9660::CurrentSensing::getPhaseAdcMapping(tmc9660::tmcl::AdcMapping& ux1,
                                                 tmc9660::tmcl::AdcMapping& vx2,
                                                 tmc9660::tmcl::AdcMapping& wy1,
                                                 tmc9660::tmcl::AdcMapping& y2) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::PHASE_UX1_ADC_MAPPING, tmp))
    return false;
  ux1 = static_cast<tmc9660::tmcl::AdcMapping>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::PHASE_VX2_ADC_MAPPING, tmp))
    return false;
  vx2 = static_cast<tmc9660::tmcl::AdcMapping>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::PHASE_WY1_ADC_MAPPING, tmp))
    return false;
  wy1 = static_cast<tmc9660::tmcl::AdcMapping>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::PHASE_Y2_ADC_MAPPING, tmp))
    return false;
  y2 = static_cast<tmc9660::tmcl::AdcMapping>(tmp);
  return true;
}

bool TMC9660::CurrentSensing::setScalingFactors(uint16_t scale0, uint16_t scale1, uint16_t scale2,
                                                uint16_t scale3) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ADC_I0_SCALE, scale0);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ADC_I1_SCALE, scale1);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ADC_I2_SCALE, scale2);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ADC_I3_SCALE, scale3);
  return ok;
}

bool TMC9660::CurrentSensing::getScalingFactors(uint16_t& scale0, uint16_t& scale1,
                                                uint16_t& scale2, uint16_t& scale3) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I0_SCALE, tmp))
    return false;
  scale0 = static_cast<uint16_t>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I1_SCALE, tmp))
    return false;
  scale1 = static_cast<uint16_t>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I2_SCALE, tmp))
    return false;
  scale2 = static_cast<uint16_t>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I3_SCALE, tmp))
    return false;
  scale3 = static_cast<uint16_t>(tmp);
  return true;
}

bool TMC9660::CurrentSensing::setInversion(tmc9660::tmcl::AdcInversion inv0,
                                           tmc9660::tmcl::AdcInversion inv1,
                                           tmc9660::tmcl::AdcInversion inv2,
                                           tmc9660::tmcl::AdcInversion inv3) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ADC_I0_INVERTED,
                              static_cast<uint32_t>(inv0));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ADC_I1_INVERTED,
                              static_cast<uint32_t>(inv1));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ADC_I2_INVERTED,
                              static_cast<uint32_t>(inv2));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ADC_I3_INVERTED,
                              static_cast<uint32_t>(inv3));
  return ok;
}

bool TMC9660::CurrentSensing::getInversion(tmc9660::tmcl::AdcInversion& inv0,
                                           tmc9660::tmcl::AdcInversion& inv1,
                                           tmc9660::tmcl::AdcInversion& inv2,
                                           tmc9660::tmcl::AdcInversion& inv3) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I0_INVERTED, tmp))
    return false;
  inv0 = static_cast<tmc9660::tmcl::AdcInversion>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I1_INVERTED, tmp))
    return false;
  inv1 = static_cast<tmc9660::tmcl::AdcInversion>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I2_INVERTED, tmp))
    return false;
  inv2 = static_cast<tmc9660::tmcl::AdcInversion>(tmp);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ADC_I3_INVERTED, tmp))
    return false;
  inv3 = static_cast<tmc9660::tmcl::AdcInversion>(tmp);
  return true;
}

bool TMC9660::CurrentSensing::setOffsets(int16_t offset0, int16_t offset1, int16_t offset2,
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

bool TMC9660::CurrentSensing::getOffsets(int16_t& offset0, int16_t& offset1, int16_t& offset2,
                                         int16_t& offset3) noexcept {
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

bool TMC9660::CurrentSensing::readScaledAndOffset(int16_t& adc0, int16_t& adc1, int16_t& adc2,
                                                  int16_t& adc3) noexcept {
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
  using tmc9660::tmcl::GeneralStatusFlags;
  using tmc9660::tmcl::Parameters;
  // Clear the calibrated flag to trigger a new calibration cycle
  if (!driver.writeParameter(Parameters::GENERAL_STATUS_FLAGS,
                             static_cast<uint32_t>(GeneralStatusFlags::ADC_OFFSET_CALIBRATED)))
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
    if (std::chrono::steady_clock::now() - start >= std::chrono::milliseconds(timeoutMs))
      return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool TMC9660::CurrentSensing::getCalibrationStatus(bool& isCalibrated) noexcept {
  uint32_t flags;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::GENERAL_STATUS_FLAGS, flags))
    return false;
  isCalibrated =
      (flags & static_cast<uint32_t>(tmc9660::tmcl::GeneralStatusFlags::ADC_OFFSET_CALIBRATED)) !=
      0;
  return true;
}

bool TMC9660::CurrentSensing::configureAuto(const AutoConfig& config) noexcept {
  // Convert mΩ to Ω
  const float R_shunt_Ohm = config.shuntResistance_mOhm / 1000.0f;

  // Available CSA gains and their numeric values
  struct CsaGainOption {
    tmc9660::tmcl::CsaGain gain;
    float numericGain;
  };

  const CsaGainOption availableGains[] = {
      {tmc9660::tmcl::CsaGain::GAIN_5X, 5.0f},
      {tmc9660::tmcl::CsaGain::GAIN_10X, 10.0f},
      {tmc9660::tmcl::CsaGain::GAIN_20X, 20.0f},
      {tmc9660::tmcl::CsaGain::GAIN_40X, 40.0f},
  };

  // Calculate full-scale current for each gain: I_FS = 2.5V / (G_CSA * R_shunt)
  // We want I_FS to be at least 1.5x the expected peak current for headroom
  const float minHeadroom = 1.5f;
  const float targetIFS = config.expectedPeakCurrent_A * minHeadroom;

  // Find the best CSA gain (smallest gain that provides sufficient headroom)
  tmc9660::tmcl::CsaGain selectedGain = tmc9660::tmcl::CsaGain::GAIN_40X;
  float selectedGainValue = 40.0f;
  bool foundSuitableGain = false;

  for (const auto& option : availableGains) {
    const float I_FS = 2.5f / (option.numericGain * R_shunt_Ohm);
    if (I_FS >= targetIFS && !foundSuitableGain) {
      selectedGain = option.gain;
      selectedGainValue = option.numericGain;
      foundSuitableGain = true;
    }
  }

  // Calculate CURRENT_SCALING_FACTOR
  // Formula: Factor = 39.06 / (G_CSA * R_shunt_Ohm) for PEAK
  //          Factor = 27.62 / (G_CSA * R_shunt_Ohm) for RMS
  const float scalingFactor = config.usePeakScaling ? (39.06f / (selectedGainValue * R_shunt_Ohm))
                                                    : (27.62f / (selectedGainValue * R_shunt_Ohm));

  // Clamp to valid range [1, 65535]
  uint16_t scalingFactorInt;
  if (scalingFactor < 1.0f) {
    scalingFactorInt = 1;
  } else if (scalingFactor > 65535.0f) {
    scalingFactorInt = 65535;
  } else {
    scalingFactorInt = static_cast<uint16_t>(scalingFactor + 0.5f); // Round to nearest
  }

  // Configure all parameters
  bool ok = true;

  // 1. Set shunt type
  ok &= setShuntType(config.shuntType);

  // 2. Set CSA gain
  ok &= setCSAGain(selectedGain, selectedGain);

  // 3. Set CSA filter
  ok &= setCSAFilter(config.csaFilter, config.csaFilter);

  // 4. Set current scaling factor (this is the key parameter!)
  ok &= setScalingFactor(scalingFactorInt);

  // 5. Set phase ADC mapping (use defaults if not specified: U->I0, V->I1, W->I2, Y2->I3)
  tmc9660::tmcl::AdcMapping mapU =
      config.phaseU_adcMapping.value_or(tmc9660::tmcl::AdcMapping::ADC_I0);
  tmc9660::tmcl::AdcMapping mapV =
      config.phaseV_adcMapping.value_or(tmc9660::tmcl::AdcMapping::ADC_I1);
  tmc9660::tmcl::AdcMapping mapW =
      config.phaseW_adcMapping.value_or(tmc9660::tmcl::AdcMapping::ADC_I2);
  tmc9660::tmcl::AdcMapping mapY2 =
      config.phaseY2_adcMapping.value_or(tmc9660::tmcl::AdcMapping::ADC_I3);
  ok &= setPhaseAdcMapping(mapU, mapV, mapW, mapY2);

  // 6. Calculate and set individual ADC scaling factors based on actual shunt resistances
  // ADC_Ix_SCALE compensates for physical mismatch in shunt resistors and CSA amplifier tolerances.
  // Formula: ADC_Ix_SCALE = 1024 * (R_nominal / R_actual)
  // If actual resistance is not provided (NaN or <= 0), use nominal value (no compensation).
  constexpr float defaultScale = 1024.0f;
  constexpr float minScale = 1.0f;
  constexpr float maxScale = 32767.0f;

  auto calculateScale = [&config, defaultScale, minScale,
                         maxScale](float actualR_mOhm) -> uint16_t {
    // Normalize: use nominal if not provided or invalid
    if (std::isnan(actualR_mOhm) || actualR_mOhm <= 0.0f) {
      actualR_mOhm = config.shuntResistance_mOhm;
    }

    // Calculate and clamp in one step
    float scale = defaultScale * (config.shuntResistance_mOhm / actualR_mOhm);
    if (scale < minScale)
      scale = minScale;
    else if (scale > maxScale)
      scale = maxScale;

    return static_cast<uint16_t>(scale + 0.5f);
  };

  uint16_t scale0 = calculateScale(config.actualShuntR_adc0_mOhm); // ADC_I0
  uint16_t scale1 = calculateScale(config.actualShuntR_adc1_mOhm); // ADC_I1
  uint16_t scale2 = calculateScale(config.actualShuntR_adc2_mOhm); // ADC_I2
  uint16_t scale3 = calculateScale(config.actualShuntR_adc3_mOhm); // ADC_I3

  ok &= setScalingFactors(scale0, scale1, scale2, scale3);

  // 7. Set ADC inversion (use Table 24 defaults if not specified, allow override)
  // Table 24: Standard ADC inversion table for usage with internal CSA
  // DC:      ADC_I0=Inverted, ADC_I1=Not inverted
  // BLDC:    ADC_I0=Inverted, ADC_I1=Inverted, ADC_I2=Inverted
  // Stepper: ADC_I0=Inverted, ADC_I1=Not inverted, ADC_I2=Inverted, ADC_I3=Not inverted
  tmc9660::tmcl::AdcInversion inv0_default, inv1_default, inv2_default, inv3_default;
  switch (config.motorType) {
  case tmc9660::tmcl::MotorType::DC_MOTOR:
    inv0_default = tmc9660::tmcl::AdcInversion::INVERTED;     // ADC_I0 (UX1)
    inv1_default = tmc9660::tmcl::AdcInversion::NOT_INVERTED; // ADC_I1 (VX2)
    inv2_default = tmc9660::tmcl::AdcInversion::NOT_INVERTED; // ADC_I2 (not used for DC)
    inv3_default = tmc9660::tmcl::AdcInversion::NOT_INVERTED; // ADC_I3 (not used for DC)
    break;
  case tmc9660::tmcl::MotorType::BLDC_MOTOR:
    inv0_default = tmc9660::tmcl::AdcInversion::INVERTED;     // ADC_I0 (UX1)
    inv1_default = tmc9660::tmcl::AdcInversion::INVERTED;     // ADC_I1 (VX2)
    inv2_default = tmc9660::tmcl::AdcInversion::INVERTED;     // ADC_I2 (WY1)
    inv3_default = tmc9660::tmcl::AdcInversion::NOT_INVERTED; // ADC_I3 (Y2, not used for BLDC)
    break;
  case tmc9660::tmcl::MotorType::STEPPER_MOTOR:
    inv0_default = tmc9660::tmcl::AdcInversion::INVERTED;     // ADC_I0 (UX1)
    inv1_default = tmc9660::tmcl::AdcInversion::NOT_INVERTED; // ADC_I1 (VX2)
    inv2_default = tmc9660::tmcl::AdcInversion::INVERTED;     // ADC_I2 (WY1)
    inv3_default = tmc9660::tmcl::AdcInversion::NOT_INVERTED; // ADC_I3 (Y2)
    break;
  default:
    // NO_MOTOR or unknown: use BLDC defaults as safe fallback
    inv0_default = tmc9660::tmcl::AdcInversion::INVERTED;
    inv1_default = tmc9660::tmcl::AdcInversion::INVERTED;
    inv2_default = tmc9660::tmcl::AdcInversion::INVERTED;
    inv3_default = tmc9660::tmcl::AdcInversion::NOT_INVERTED;
    break;
  }

  // Use user-specified inversion if provided, otherwise use defaults
  tmc9660::tmcl::AdcInversion inv0 = config.adc0_inverted.value_or(inv0_default);
  tmc9660::tmcl::AdcInversion inv1 = config.adc1_inverted.value_or(inv1_default);
  tmc9660::tmcl::AdcInversion inv2 = config.adc2_inverted.value_or(inv2_default);
  tmc9660::tmcl::AdcInversion inv3 = config.adc3_inverted.value_or(inv3_default);
  ok &= setInversion(inv0, inv1, inv2, inv3);

  // 8. Optionally calibrate ADC offsets
  // Note: Offsets are not set manually - they are calibrated by the TMC9660 hardware.
  // The calibration requires the motor to be stationary and commutation to be off (SYSTEM_OFF).
  if (config.autoCalibrate) {
    // Trigger calibration and wait for completion
    if (!calibrateOffsets(true, config.calibrationTimeoutMs)) {
      return false; // Calibration failed or timed out
    }

    // Verify calibration was successful by checking the ADC_OFFSET_CALIBRATED flag
    bool isCalibrated = false;
    if (!getCalibrationStatus(isCalibrated)) {
      return false; // Failed to read calibration status
    }
    if (!isCalibrated) {
      return false; // Calibration did not complete successfully
    }
  }

  return ok;
}

//***************************************************************************
//**                  SUBSYSTEM: Gate Driver                              **//
//***************************************************************************

bool TMC9660::GateDriver::setOutputPolarity(tmc9660::tmcl::PwmOutputPolarity lowSide,
                                            tmc9660::tmcl::PwmOutputPolarity highSide) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::PWM_L_OUTPUT_POLARITY,
                              static_cast<uint32_t>(lowSide));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::PWM_H_OUTPUT_POLARITY,
                              static_cast<uint32_t>(highSide));
  return ok;
}

bool TMC9660::GateDriver::configureBreakBeforeMakeTiming(uint8_t lowSideUVW, uint8_t highSideUVW,
                                                         uint8_t lowSideY2,
                                                         uint8_t highSideY2) noexcept {
  bool ok = true;
  ok &=
      driver.writeParameter(tmc9660::tmcl::Parameters::BREAK_BEFORE_MAKE_TIME_LOW_UVW, lowSideUVW);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::BREAK_BEFORE_MAKE_TIME_HIGH_UVW,
                              highSideUVW);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::BREAK_BEFORE_MAKE_TIME_LOW_Y2, lowSideY2);
  ok &=
      driver.writeParameter(tmc9660::tmcl::Parameters::BREAK_BEFORE_MAKE_TIME_HIGH_Y2, highSideY2);
  return ok;
}

bool TMC9660::GateDriver::configureBreakBeforeMakeTiming_ns(float lowSideUVW_ns,
                                                            float highSideUVW_ns,
                                                            float lowSideY2_ns,
                                                            float highSideY2_ns) noexcept {
  // Conversion formula: value = time_ns / 8.33
  // Clamp to valid range [0, 255]
  constexpr float ns_per_step = 8.33f;

  auto convertToRegister = [](float time_ns) -> uint8_t {
    float value = time_ns / ns_per_step;
    // Clamp to valid range
    if (value < 0.0f) {
      return 0;
    }
    if (value > 255.0f) {
      return 255;
    }
    return static_cast<uint8_t>(value + 0.5f); // Round to nearest integer
  };

  uint8_t lowUVW = convertToRegister(lowSideUVW_ns);
  uint8_t highUVW = convertToRegister(highSideUVW_ns);
  uint8_t lowY2 = convertToRegister(lowSideY2_ns);
  uint8_t highY2 = convertToRegister(highSideY2_ns);

  return configureBreakBeforeMakeTiming(lowUVW, highUVW, lowY2, highY2);
}

bool TMC9660::GateDriver::enableAdaptiveDriveTime(bool enableUVW, bool enableY2) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::USE_ADAPTIVE_DRIVE_TIME_UVW,
                              enableUVW ? 1u : 0u);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::USE_ADAPTIVE_DRIVE_TIME_Y2,
                              enableY2 ? 1u : 0u);
  return ok;
}

bool TMC9660::GateDriver::configureDriveTimes(uint8_t sinkTimeUVW, uint8_t sourceTimeUVW,
                                              uint8_t sinkTimeY2, uint8_t sourceTimeY2) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::DRIVE_TIME_SINK_UVW, sinkTimeUVW);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::DRIVE_TIME_SOURCE_UVW, sourceTimeUVW);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::DRIVE_TIME_SINK_Y2, sinkTimeY2);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::DRIVE_TIME_SOURCE_Y2, sourceTimeY2);
  return ok;
}

bool TMC9660::GateDriver::configureDriveTimes_ns(float sinkTimeUVW_ns, float sourceTimeUVW_ns,
                                                 float sinkTimeY2_ns,
                                                 float sourceTimeY2_ns) noexcept {
  // Conversion formula per TMC9660 documentation:
  // t = (1s / 120MHz) × (2 × DRIVE_TIME_xxx + 3)
  // Solving for register value: DRIVE_TIME = ((t × 120MHz) - 3) / 2
  // Where: 1s / 120MHz = 8.33ns per step
  // Formula: value = ((desired_time_ns / 8.33) - 3) / 2
  // Clamp to valid range [0, 255]
  constexpr float ns_per_step = 8.33f; // 1s / 120MHz

  auto convertToRegister = [](float time_ns) -> uint8_t {
    // Convert nanoseconds to register value using TMC9660 formula:
    // t = 8.33ns × (2 × REG + 3)  →  REG = ((t / 8.33) - 3) / 2
    float value = ((time_ns / ns_per_step) - 3.0f) / 2.0f;
    // Clamp to valid range [0, 255]
    if (value < 0.0f) {
      return 0;
    }
    if (value > 255.0f) {
      return 255;
    }
    // Round to nearest integer (e.g., 10.5 → 11, 10.4 → 10)
    // This selects the register value that produces the closest actual time
    return static_cast<uint8_t>(value + 0.5f);
  };

  uint8_t sinkUVW = convertToRegister(sinkTimeUVW_ns);
  uint8_t sourceUVW = convertToRegister(sourceTimeUVW_ns);
  uint8_t sinkY2 = convertToRegister(sinkTimeY2_ns);
  uint8_t sourceY2 = convertToRegister(sourceTimeY2_ns);

  return configureDriveTimes(sinkUVW, sourceUVW, sinkY2, sourceY2);
}

bool TMC9660::GateDriver::configureCurrentLimits(
    tmc9660::tmcl::GateCurrentSink sinkCurrentUVW,
    tmc9660::tmcl::GateCurrentSource sourceCurrentUVW, tmc9660::tmcl::GateCurrentSink sinkCurrentY2,
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
    tmc9660::tmcl::UndervoltageLevel supplyLevel, tmc9660::tmcl::UndervoltageEnable enableVdrv,
    tmc9660::tmcl::UndervoltageEnable enableBstUVW,
    tmc9660::tmcl::UndervoltageEnable enableBstY2) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SUPPLY_LEVEL,
                              static_cast<uint32_t>(supplyLevel));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::VDRV_ENABLE,
                              static_cast<uint32_t>(enableVdrv));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::BST_UVW_ENABLE,
                              static_cast<uint32_t>(enableBstUVW));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::BST_Y2_ENABLE,
                              static_cast<uint32_t>(enableBstY2));
  return ok;
}

bool TMC9660::GateDriver::enableOvercurrentProtection(
    tmc9660::tmcl::OvercurrentEnable enableUVWLowSide,
    tmc9660::tmcl::OvercurrentEnable enableUVWHighSide,
    tmc9660::tmcl::OvercurrentEnable enableY2LowSide,
    tmc9660::tmcl::OvercurrentEnable enableY2HighSide) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_LOW_SIDE_ENABLE,
                              static_cast<uint32_t>(enableUVWLowSide));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_HIGH_SIDE_ENABLE,
                              static_cast<uint32_t>(enableUVWHighSide));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_LOW_SIDE_ENABLE,
                              static_cast<uint32_t>(enableY2LowSide));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_HIGH_SIDE_ENABLE,
                              static_cast<uint32_t>(enableY2HighSide));
  return ok;
}

bool TMC9660::GateDriver::setOvercurrentThresholds(
    tmc9660::tmcl::OvercurrentThreshold uvwLowSideThreshold,
    tmc9660::tmcl::OvercurrentThreshold uvwHighSideThreshold,
    tmc9660::tmcl::OvercurrentThreshold y2LowSideThreshold,
    tmc9660::tmcl::OvercurrentThreshold y2HighSideThreshold) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_LOW_SIDE_THRESHOLD,
                              static_cast<uint32_t>(uvwLowSideThreshold));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_HIGH_SIDE_THRESHOLD,
                              static_cast<uint32_t>(uvwHighSideThreshold));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_LOW_SIDE_THRESHOLD,
                              static_cast<uint32_t>(y2LowSideThreshold));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_HIGH_SIDE_THRESHOLD,
                              static_cast<uint32_t>(y2HighSideThreshold));
  return ok;
}

bool TMC9660::GateDriver::setOvercurrentBlanking(
    tmc9660::tmcl::OvercurrentTiming uvwLowSideTime,
    tmc9660::tmcl::OvercurrentTiming uvwHighSideTime,
    tmc9660::tmcl::OvercurrentTiming y2LowSideTime,
    tmc9660::tmcl::OvercurrentTiming y2HighSideTime) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_LOW_SIDE_BLANKING,
                              static_cast<uint32_t>(uvwLowSideTime));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_HIGH_SIDE_BLANKING,
                              static_cast<uint32_t>(uvwHighSideTime));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_LOW_SIDE_BLANKING,
                              static_cast<uint32_t>(y2LowSideTime));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_HIGH_SIDE_BLANKING,
                              static_cast<uint32_t>(y2HighSideTime));
  return ok;
}

bool TMC9660::GateDriver::setOvercurrentDeglitch(
    tmc9660::tmcl::OvercurrentTiming uvwLowSideTime,
    tmc9660::tmcl::OvercurrentTiming uvwHighSideTime,
    tmc9660::tmcl::OvercurrentTiming y2LowSideTime,
    tmc9660::tmcl::OvercurrentTiming y2HighSideTime) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_LOW_SIDE_DEGLITCH,
                              static_cast<uint32_t>(uvwLowSideTime));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_HIGH_SIDE_DEGLITCH,
                              static_cast<uint32_t>(uvwHighSideTime));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_LOW_SIDE_DEGLITCH,
                              static_cast<uint32_t>(y2LowSideTime));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_HIGH_SIDE_DEGLITCH,
                              static_cast<uint32_t>(y2HighSideTime));
  return ok;
}

bool TMC9660::GateDriver::enableVdsMonitoringLow(tmc9660::tmcl::VdsUsage uvwEnable,
                                                 tmc9660::tmcl::VdsUsage y2Enable) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_LOW_SIDE_USE_VDS,
                              static_cast<uint32_t>(uvwEnable));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_LOW_SIDE_USE_VDS,
                              static_cast<uint32_t>(y2Enable));
  return ok;
}

bool TMC9660::GateDriver::configureVgsShortProtectionUVW(
    tmc9660::tmcl::VgsShortEnable enableLowSideOn, tmc9660::tmcl::VgsShortEnable enableLowSideOff,
    tmc9660::tmcl::VgsShortEnable enableHighSideOn,
    tmc9660::tmcl::VgsShortEnable enableHighSideOff) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_LOW_SIDE_ON_ENABLE,
                              static_cast<uint32_t>(enableLowSideOn));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_LOW_SIDE_OFF_ENABLE,
                              static_cast<uint32_t>(enableLowSideOff));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_HIGH_SIDE_ON_ENABLE,
                              static_cast<uint32_t>(enableHighSideOn));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_HIGH_SIDE_OFF_ENABLE,
                              static_cast<uint32_t>(enableHighSideOff));
  return ok;
}

bool TMC9660::GateDriver::configureVgsShortProtectionY2(
    tmc9660::tmcl::VgsShortEnable enableLowSideOn, tmc9660::tmcl::VgsShortEnable enableLowSideOff,
    tmc9660::tmcl::VgsShortEnable enableHighSideOn,
    tmc9660::tmcl::VgsShortEnable enableHighSideOff) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_LOW_SIDE_ON_ENABLE,
                              static_cast<uint32_t>(enableLowSideOn));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_LOW_SIDE_OFF_ENABLE,
                              static_cast<uint32_t>(enableLowSideOff));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_HIGH_SIDE_ON_ENABLE,
                              static_cast<uint32_t>(enableHighSideOn));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::Y2_HIGH_SIDE_OFF_ENABLE,
                              static_cast<uint32_t>(enableHighSideOff));
  return ok;
}

bool TMC9660::GateDriver::setVgsShortBlankingTime(tmc9660::tmcl::VgsBlankingTime uvwTime,
                                                  tmc9660::tmcl::VgsBlankingTime y2Time) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_BLANKING,
                              static_cast<uint32_t>(uvwTime));
  ok &=
      driver.writeParameter(tmc9660::tmcl::Parameters::Y2_BLANKING, static_cast<uint32_t>(y2Time));
  return ok;
}

bool TMC9660::GateDriver::setVgsShortDeglitchTime(tmc9660::tmcl::VgsDeglitchTime uvwTime,
                                                  tmc9660::tmcl::VgsDeglitchTime y2Time) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::UVW_DEGLITCH,
                              static_cast<uint32_t>(uvwTime));
  ok &=
      driver.writeParameter(tmc9660::tmcl::Parameters::Y2_DEGLITCH, static_cast<uint32_t>(y2Time));
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
  return driver.writeParameter(tmc9660::tmcl::Parameters::FAULT_HANDLER_NUMBER_OF_RETRIES, retries);
}

// Helper function to calculate required bootstrap current based on gate charge and PWM frequency
// Physics-based calculation: I_avg = (Qg × N_phases × f_PWM) / duty_cycle × safety_margin
static float calculateBootstrapCurrent_mA(float gateCharge_nC, float pwmFreq_Hz) noexcept {
  constexpr float N_HIGH_SIDE_PHASES = 3.0f; // UVW phases (3 high-side FETs)
  constexpr float AVG_DUTY_CYCLE = 0.5f;     // Average duty cycle assumption
  constexpr float SAFETY_MARGIN = 2.5f;      // 2.5x margin for duty peaks and losses

  // Average current needed: I = (Qg × N_phases × f_PWM) / duty_cycle
  // Convert nC to C, Hz to 1/s: (nC × 1e-9) × N × f_PWM / duty = A
  // Then convert to mA and apply safety margin
  float chargePerCycle_C = gateCharge_nC * 1e-9f; // Convert nC to C
  float avgCurrent_A = (chargePerCycle_C * N_HIGH_SIDE_PHASES * pwmFreq_Hz) / AVG_DUTY_CYCLE;
  return avgCurrent_A * 1000.0f * SAFETY_MARGIN; // Convert to mA and apply margin
}

bool TMC9660::GateDriver::configurePowerStageProtection(const PowerStageProfile& profile) noexcept {
  bool ok = true;

  // --- Safety Check: Ensure critical parameters are valid ---
  if (profile.busVoltage_V <= 0.0f || profile.pwmFrequency_Hz <= 0.0f ||
      profile.expectedPeakCurrent_A <= 0.0f || profile.mosfet_RdsOn_mOhm <= 0.0f ||
      profile.shuntResistance_mOhm <= 0.0f || profile.mosfet_gateCharge_nC <= 0.0f) {
    // Log/Report error: Invalid critical input parameters
    return false;
  }

  // ============================================================================
  // PART 0: GATE DRIVER INTERFACE CONFIGURATION
  // ============================================================================
  // Configure basic hardware interface settings (must be set before timing/protection)

  // Step 0.1: Configure PWM output polarity (critical for hardware compatibility)
  // Wrong polarity = gate driver won't work correctly with external MOSFET stages
  ok &= setOutputPolarity(profile.pwmLowPolarity, profile.pwmHighPolarity);

  // ============================================================================
  // PART 1: GATE DRIVER TIMING CONFIGURATION
  // ============================================================================
  // Configure MOSFET switching characteristics based on gate charge (Qg)
  // These directly affect switching speed and must be set before protection timing

  // Step 1.1: Calculate gate drive times from gate charge
  // Target: ~200ns turn-on (source), ~135ns turn-off (sink) for efficiency/safety balance
  // With adaptive drive time enabled, these are maximum times; driver optimizes down
  // Allow user override via optional profile members for custom switching speed requirements
  constexpr float DEFAULT_TARGET_TURN_ON_NS = 200.0f;  // Default source (turn-on) time target
  constexpr float DEFAULT_TARGET_TURN_OFF_NS = 135.0f; // Default sink (turn-off) time target

  float targetTurnOn_ns = profile.targetTurnOnTime_ns.value_or(DEFAULT_TARGET_TURN_ON_NS);
  float targetTurnOff_ns = profile.targetTurnOffTime_ns.value_or(DEFAULT_TARGET_TURN_OFF_NS);

  // Validate override values are positive
  if (targetTurnOn_ns <= 0.0f || targetTurnOff_ns <= 0.0f) {
    return false; // Invalid override values
  }

  // Calculate required gate currents: I = Qg / t
  // These are theoretical; we'll select closest available enum values
  float requiredSourceCurrent_mA = (profile.mosfet_gateCharge_nC / targetTurnOn_ns) * 1000.0f;
  float requiredSinkCurrent_mA = (profile.mosfet_gateCharge_nC / targetTurnOff_ns) * 1000.0f;

  // Select closest available gate current enum values
  // Note: Selecting next-higher current results in faster switching (shorter actual rise/fall
  // times) This is generally safer (reduces shoot-through risk) but may increase switching losses
  // slightly
  auto selectSourceCurrent = [](float current_mA) -> tmc9660::tmcl::GateCurrentSource {
    if (current_mA <= 37.5f)
      return tmc9660::tmcl::GateCurrentSource::CUR_25_MA;
    else if (current_mA <= 65.0f)
      return tmc9660::tmcl::GateCurrentSource::CUR_50_MA;
    else if (current_mA <= 92.5f)
      return tmc9660::tmcl::GateCurrentSource::CUR_80_MA;
    else if (current_mA <= 120.0f)
      return tmc9660::tmcl::GateCurrentSource::CUR_105_MA;
    else if (current_mA <= 147.5f)
      return tmc9660::tmcl::GateCurrentSource::CUR_135_MA;
    else if (current_mA <= 175.0f)
      return tmc9660::tmcl::GateCurrentSource::CUR_160_MA;
    else if (current_mA <= 202.5f)
      return tmc9660::tmcl::GateCurrentSource::CUR_190_MA;
    else if (current_mA <= 252.5f)
      return tmc9660::tmcl::GateCurrentSource::CUR_215_MA;
    else if (current_mA <= 325.0f)
      return tmc9660::tmcl::GateCurrentSource::CUR_290_MA;
    else if (current_mA <= 395.0f)
      return tmc9660::tmcl::GateCurrentSource::CUR_360_MA;
    else if (current_mA <= 465.0f)
      return tmc9660::tmcl::GateCurrentSource::CUR_430_MA;
    else if (current_mA <= 562.5f)
      return tmc9660::tmcl::GateCurrentSource::CUR_500_MA;
    else if (current_mA <= 690.0f)
      return tmc9660::tmcl::GateCurrentSource::CUR_625_MA;
    else if (current_mA <= 805.0f)
      return tmc9660::tmcl::GateCurrentSource::CUR_755_MA;
    else
      return tmc9660::tmcl::GateCurrentSource::CUR_855_MA;
  };

  auto selectSinkCurrent = [](float current_mA) -> tmc9660::tmcl::GateCurrentSink {
    if (current_mA <= 75.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_50_MA;
    else if (current_mA <= 130.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_100_MA;
    else if (current_mA <= 185.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_160_MA;
    else if (current_mA <= 240.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_210_MA;
    else if (current_mA <= 295.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_270_MA;
    else if (current_mA <= 350.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_320_MA;
    else if (current_mA <= 405.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_380_MA;
    else if (current_mA <= 505.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_430_MA;
    else if (current_mA <= 650.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_580_MA;
    else if (current_mA <= 790.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_720_MA;
    else if (current_mA <= 930.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_860_MA;
    else if (current_mA <= 1125.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_1000_MA;
    else if (current_mA <= 1380.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_1250_MA;
    else if (current_mA <= 1640.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_1510_MA;
    else if (current_mA <= 1890.0f)
      return tmc9660::tmcl::GateCurrentSink::CUR_1770_MA;
    else
      return tmc9660::tmcl::GateCurrentSink::CUR_2000_MA;
  };

  tmc9660::tmcl::GateCurrentSource sourceCurrent = selectSourceCurrent(requiredSourceCurrent_mA);
  tmc9660::tmcl::GateCurrentSink sinkCurrent = selectSinkCurrent(requiredSinkCurrent_mA);

  // Configure gate current limits
  ok &= configureCurrentLimits(sinkCurrent, sourceCurrent, sinkCurrent, sourceCurrent);

  // Step 1.2: Configure drive times (maximum times with adaptive mode)
  // Adaptive drive time will shorten these based on actual gate voltage monitoring
  ok &=
      configureDriveTimes_ns(targetTurnOff_ns, targetTurnOn_ns, targetTurnOff_ns, targetTurnOn_ns);

  // Step 1.3: Enable adaptive drive time for efficiency
  // Adaptive mode monitors gate voltage and shortens drive times automatically
  ok &= enableAdaptiveDriveTime(true, true); // Enable for UVW and Y2

  // Step 1.4: Configure break-before-make timing (dead time)
  // Documentation recommends 0 to let driver use internal optimized timing
  // For special cases (high voltage, slow body diodes), dead time can be explicitly set
  // Default to 0 per documentation recommendation for optimal performance
  // Note: If overrideDeadTime_ns is added to PowerStageProfile in future, use:
  //   float dead_time_ns = profile.overrideDeadTime_ns.value_or(0.0f);
  constexpr float dead_time_ns = 0.0f;
  ok &= configureBreakBeforeMakeTiming_ns(dead_time_ns, dead_time_ns, dead_time_ns, dead_time_ns);

  // Step 1.5: Configure bootstrap current limit
  // Bootstrap must supply gate charge for high-side FETs at PWM switching rate
  // Uses physics-based calculation (see calculateBootstrapCurrent_mA helper function)
  tmc9660::tmcl::BootstrapCurrentLimit bootstrapCurrent;
  float bootstrapCurrent_mA =
      calculateBootstrapCurrent_mA(profile.mosfet_gateCharge_nC, profile.pwmFrequency_Hz);

  if (bootstrapCurrent_mA <= 68.0f) {
    bootstrapCurrent = tmc9660::tmcl::BootstrapCurrentLimit::CUR_45_MA;
  } else if (bootstrapCurrent_mA <= 116.0f) {
    bootstrapCurrent = tmc9660::tmcl::BootstrapCurrentLimit::CUR_91_MA;
  } else if (bootstrapCurrent_mA <= 166.0f) {
    bootstrapCurrent = tmc9660::tmcl::BootstrapCurrentLimit::CUR_141_MA;
  } else if (bootstrapCurrent_mA <= 229.0f) {
    bootstrapCurrent = tmc9660::tmcl::BootstrapCurrentLimit::CUR_191_MA;
  } else if (bootstrapCurrent_mA <= 279.5f) {
    bootstrapCurrent = tmc9660::tmcl::BootstrapCurrentLimit::CUR_267_MA;
  } else if (bootstrapCurrent_mA <= 316.5f) {
    bootstrapCurrent = tmc9660::tmcl::BootstrapCurrentLimit::CUR_292_MA;
  } else if (bootstrapCurrent_mA <= 366.0f) {
    bootstrapCurrent = tmc9660::tmcl::BootstrapCurrentLimit::CUR_341_MA;
  } else {
    bootstrapCurrent = tmc9660::tmcl::BootstrapCurrentLimit::CUR_391_MA;
  }

  ok &= configureBootstrapCurrentLimit(bootstrapCurrent);

  // Step 1.6: Configure undervoltage protection (supply, VDRV, and bootstrap)
  // Supply level: Configured from profile (0=disabled, 1-16 map to HW levels 0-15)
  // VDRV protection: Enabled by default for safety
  // Bootstrap UVP: Enabled by default for safety (prevents gate drive failure from insufficient
  // bootstrap voltage)
  ok &= configureUndervoltageProtection(
      profile.supplyLevel,                         // Supply level from profile
      tmc9660::tmcl::UndervoltageEnable::ENABLED,  // VDRV protection (keep enabled)
      tmc9660::tmcl::UndervoltageEnable::ENABLED,  // Bootstrap UVW (keep enabled for safety)
      tmc9660::tmcl::UndervoltageEnable::ENABLED); // Bootstrap Y2 (keep enabled for safety)

  // ============================================================================
  // PART 2: PROTECTION PARAMETER CONFIGURATION
  // ============================================================================

  // Step 2.1: Estimate di/dt (current rise rate during switching)
  // di/dt ≈ V_bus / L_motor
  // Use worst-case low inductance for safety (higher di/dt → longer blanking needed)
  float di_dt_A_per_us;
  if (std::isnan(profile.motorInductance_uH) || profile.motorInductance_uH <= 0.0f) {
    // Conservative worst-case: assume low inductance (20 µH) for noisy motors
    // Note: Gear motors typically have higher inductance (50-100+ µH), but using
    // lower value ensures safer (longer) blanking time for worst-case scenarios
    constexpr float L_WORST_CASE_uH = 20.0f; // Fixed: was 50.0f, now matches comment
    di_dt_A_per_us = profile.busVoltage_V / L_WORST_CASE_uH;
  } else {
    di_dt_A_per_us = profile.busVoltage_V / profile.motorInductance_uH;
  }

  // Step 2.2: Select overcurrent blanking time based on di/dt
  // Higher di/dt → longer blanking needed to ignore switching spikes
  // Apply blankingMargin safety factor to the base time
  float baseBlankingTime_us;
  if (di_dt_A_per_us < 5.0f) {
    baseBlankingTime_us = 0.25f;
  } else if (di_dt_A_per_us < 15.0f) {
    baseBlankingTime_us = 0.5f;
  } else if (di_dt_A_per_us < 30.0f) {
    baseBlankingTime_us = 1.0f;
  } else if (di_dt_A_per_us < 60.0f) {
    baseBlankingTime_us = 2.0f;
  } else {
    // Very high di/dt - use longer blanking, but cap at PWM period limit
    float pwmPeriod_us = 1000000.0f / profile.pwmFrequency_Hz;
    // Cap blanking at 15% of PWM period for normal frequencies
    // For very high frequencies (>100kHz), use more conservative 10% cap
    float maxBlankingRatio = (profile.pwmFrequency_Hz > 100000.0f) ? 0.10f : 0.15f;
    baseBlankingTime_us = pwmPeriod_us * maxBlankingRatio;
    // Ensure minimum blanking time scales with PWM period but has absolute floor
    // At very high frequencies, use 5% of period or 0.25µs minimum (whichever is larger)
    float minBlanking_us = std::max(0.25f, pwmPeriod_us * 0.05f);
    if (baseBlankingTime_us < minBlanking_us) {
      baseBlankingTime_us = minBlanking_us;
    }
  }

  // Apply safety margin
  float targetBlankingTime_us = baseBlankingTime_us * profile.blankingMargin;

  // Map to discrete register values (select next higher value for safety)
  tmc9660::tmcl::OvercurrentTiming ocBlanking;
  if (targetBlankingTime_us <= 0.25f) {
    ocBlanking = tmc9660::tmcl::OvercurrentTiming::T_0_25_MICROSEC;
  } else if (targetBlankingTime_us <= 0.5f) {
    ocBlanking = tmc9660::tmcl::OvercurrentTiming::T_0_5_MICROSEC;
  } else if (targetBlankingTime_us <= 1.0f) {
    ocBlanking = tmc9660::tmcl::OvercurrentTiming::T_1_MICROSEC;
  } else if (targetBlankingTime_us <= 2.0f) {
    ocBlanking = tmc9660::tmcl::OvercurrentTiming::T_2_MICROSEC;
  } else if (targetBlankingTime_us <= 4.0f) {
    ocBlanking = tmc9660::tmcl::OvercurrentTiming::T_4_MICROSEC;
  } else if (targetBlankingTime_us <= 6.0f) {
    ocBlanking = tmc9660::tmcl::OvercurrentTiming::T_6_MICROSEC;
  } else {
    ocBlanking = tmc9660::tmcl::OvercurrentTiming::T_8_MICROSEC;
  }

  // Step 2.3: Select deglitch time based on gate charge (Qg)
  // Low Qg = fast switching = more noise = longer deglitch needed
  // High Qg = slow switching = less noise = shorter deglitch OK
  // Apply blankingMargin safety factor
  float baseDeglitchTime_us;
  if (profile.mosfet_gateCharge_nC < 15.0f) {
    // Fast FETs (GaN, low Qg) - need longer deglitch for noise filtering
    baseDeglitchTime_us = 2.0f;
  } else if (profile.mosfet_gateCharge_nC < 40.0f) {
    // Normal FETs - moderate deglitch
    baseDeglitchTime_us = 1.0f;
  } else {
    // Slow FETs (high Qg) - shorter deglitch sufficient
    baseDeglitchTime_us = 0.5f;
  }

  // Apply safety margin
  float targetDeglitchTime_us = baseDeglitchTime_us * profile.blankingMargin;

  // Map to discrete register values (select next higher value for safety)
  tmc9660::tmcl::OvercurrentTiming ocDeglitch;
  if (targetDeglitchTime_us <= 0.25f) {
    ocDeglitch = tmc9660::tmcl::OvercurrentTiming::T_0_25_MICROSEC;
  } else if (targetDeglitchTime_us <= 0.5f) {
    ocDeglitch = tmc9660::tmcl::OvercurrentTiming::T_0_5_MICROSEC;
  } else if (targetDeglitchTime_us <= 1.0f) {
    ocDeglitch = tmc9660::tmcl::OvercurrentTiming::T_1_MICROSEC;
  } else if (targetDeglitchTime_us <= 2.0f) {
    ocDeglitch = tmc9660::tmcl::OvercurrentTiming::T_2_MICROSEC;
  } else if (targetDeglitchTime_us <= 4.0f) {
    ocDeglitch = tmc9660::tmcl::OvercurrentTiming::T_4_MICROSEC;
  } else if (targetDeglitchTime_us <= 6.0f) {
    ocDeglitch = tmc9660::tmcl::OvercurrentTiming::T_6_MICROSEC;
  } else {
    ocDeglitch = tmc9660::tmcl::OvercurrentTiming::T_8_MICROSEC;
  }

  // Step 2.4: Select VGS short protection blanking/deglitch based on gate charge
  tmc9660::tmcl::VgsBlankingTime vgsBlanking;
  tmc9660::tmcl::VgsDeglitchTime vgsDeglitch;
  if (profile.mosfet_gateCharge_nC < 15.0f) {
    // Fast FETs - longer blanking/deglitch
    vgsBlanking = tmc9660::tmcl::VgsBlankingTime::T_0_5_MICROSEC;
    vgsDeglitch = tmc9660::tmcl::VgsDeglitchTime::T_2_MICROSEC;
  } else if (profile.mosfet_gateCharge_nC < 40.0f) {
    // Normal FETs
    vgsBlanking = tmc9660::tmcl::VgsBlankingTime::T_0_5_MICROSEC;
    vgsDeglitch = tmc9660::tmcl::VgsDeglitchTime::T_1_MICROSEC;
  } else {
    // Slow FETs - shorter times
    vgsBlanking = tmc9660::tmcl::VgsBlankingTime::T_0_25_MICROSEC;
    vgsDeglitch = tmc9660::tmcl::VgsDeglitchTime::T_0_5_MICROSEC;
  }

  // Step 2.5: Calculate overcurrent thresholds
  // TMC9660 uses dual sensing: RSHUNT for low-side (primary), VDS for high-side (always)
  // Low-side can optionally use VDS if Rds_on is low enough
  // Calculate separate thresholds for each sensing method

  // RSHUNT threshold (for low-side when VDS disabled)
  // Threshold voltage = I_peak * margin * R_shunt
  float rshuntThreshold_mV =
      profile.expectedPeakCurrent_A * profile.overcurrentMargin * profile.shuntResistance_mOhm;

  // VDS threshold (for high-side always, and low-side if VDS enabled)
  // Threshold voltage = I_peak * margin * Rds_on
  float vdsThreshold_mV =
      profile.expectedPeakCurrent_A * profile.overcurrentMargin * profile.mosfet_RdsOn_mOhm;

  // Helper function to select threshold register value
  auto selectThreshold = [](float voltage_mV, bool isVds) -> tmc9660::tmcl::OvercurrentThreshold {
    // For VDS: use VDS values (lower); For RSHUNT: use RSHUNT values (higher)
    // Select smallest threshold >= calculated value for safety
    if (isVds) {
      // VDS threshold values
      if (voltage_mV <= 63.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_80_OR_63_MILLIVOLT; // 63mV VDS
      } else if (voltage_mV <= 125.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_165_OR_125_MILLIVOLT; // 125mV VDS
      } else if (voltage_mV <= 187.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_250_OR_187_MILLIVOLT; // 187mV VDS
      } else if (voltage_mV <= 248.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_330_OR_248_MILLIVOLT; // 248mV VDS
      } else if (voltage_mV <= 312.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_415_OR_312_MILLIVOLT; // 312mV VDS
      } else if (voltage_mV <= 374.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_500_OR_374_MILLIVOLT; // 374mV VDS
      } else if (voltage_mV <= 434.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_582_OR_434_MILLIVOLT; // 434mV VDS
      } else if (voltage_mV <= 504.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_660_OR_504_MILLIVOLT; // 504mV VDS
      } else if (voltage_mV <= 705.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_125_OR_705_MILLIVOLT; // 705mV VDS
      } else if (voltage_mV <= 940.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_250_OR_940_MILLIVOLT; // 940mV VDS
      } else if (voltage_mV <= 1180.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_375_OR_1180_MILLIVOLT; // 1180mV VDS
      } else if (voltage_mV <= 1410.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_500_OR_1410_MILLIVOLT; // 1410mV VDS
      } else if (voltage_mV <= 1650.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_625_OR_1650_MILLIVOLT; // 1650mV VDS
      } else if (voltage_mV <= 1880.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_750_OR_1880_MILLIVOLT; // 1880mV VDS
      } else if (voltage_mV <= 2110.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_875_OR_2110_MILLIVOLT; // 2110mV VDS
      } else {
        return tmc9660::tmcl::OvercurrentThreshold::V_1000_OR_2350_MILLIVOLT; // 2350mV VDS (max)
      }
    } else {
      // RSHUNT threshold values (higher than VDS for same current)
      // Hardware automatically selects RSHUNT value when VDS is disabled for low-side
      if (voltage_mV <= 80.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_80_OR_63_MILLIVOLT; // 80mV RSHUNT
      } else if (voltage_mV <= 125.0f) {
        // Special case: V_125_OR_705 provides 125mV RSHUNT (maps to 705mV VDS for high-side)
        return tmc9660::tmcl::OvercurrentThreshold::V_125_OR_705_MILLIVOLT;
      } else if (voltage_mV <= 165.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_165_OR_125_MILLIVOLT; // 165mV RSHUNT
      } else if (voltage_mV <= 250.0f) {
        // V_250_OR_187 provides 250mV RSHUNT (normal case)
        return tmc9660::tmcl::OvercurrentThreshold::V_250_OR_187_MILLIVOLT;
      } else if (voltage_mV <= 330.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_330_OR_248_MILLIVOLT; // 330mV RSHUNT
      } else if (voltage_mV <= 375.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_375_OR_1180_MILLIVOLT; // 375mV RSHUNT
      } else if (voltage_mV <= 415.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_415_OR_312_MILLIVOLT; // 415mV RSHUNT
      } else if (voltage_mV <= 500.0f) {
        // V_500_OR_374 provides 500mV RSHUNT (normal case)
        return tmc9660::tmcl::OvercurrentThreshold::V_500_OR_374_MILLIVOLT;
      } else if (voltage_mV <= 582.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_582_OR_434_MILLIVOLT; // 582mV RSHUNT
      } else if (voltage_mV <= 625.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_625_OR_1650_MILLIVOLT; // 625mV RSHUNT
      } else if (voltage_mV <= 660.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_660_OR_504_MILLIVOLT; // 660mV RSHUNT
      } else if (voltage_mV <= 750.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_750_OR_1880_MILLIVOLT; // 750mV RSHUNT
      } else if (voltage_mV <= 875.0f) {
        return tmc9660::tmcl::OvercurrentThreshold::V_875_OR_2110_MILLIVOLT; // 875mV RSHUNT
      } else {
        return tmc9660::tmcl::OvercurrentThreshold::V_1000_OR_2350_MILLIVOLT; // 1000mV RSHUNT (max)
      }
    }
  };

  // Step 2.6: Determine if VDS should be used for low-side
  // VDS sensing is reliable when Rds_on is very low (< 10 mΩ) and for low bus voltages
  // For high-performance drives, RSHUNT is typically preferred for low-side
  bool useVdsLowSide = (profile.mosfet_RdsOn_mOhm < 10.0f);

  // Select appropriate thresholds
  tmc9660::tmcl::OvercurrentThreshold ocThresholdLowSide =
      useVdsLowSide ? selectThreshold(vdsThreshold_mV, true)      // Use VDS threshold
                    : selectThreshold(rshuntThreshold_mV, false); // Use RSHUNT threshold
  tmc9660::tmcl::OvercurrentThreshold ocThresholdHighSide =
      selectThreshold(vdsThreshold_mV, true); // Always VDS

  // Step 2.7: Configure all protection parameters
  // 2.7.1: Enable overcurrent protection (all phases, both sides)
  ok &= enableOvercurrentProtection(tmc9660::tmcl::OvercurrentEnable::ENABLED,  // UVW low side
                                    tmc9660::tmcl::OvercurrentEnable::ENABLED,  // UVW high side
                                    tmc9660::tmcl::OvercurrentEnable::ENABLED,  // Y2 low side
                                    tmc9660::tmcl::OvercurrentEnable::ENABLED); // Y2 high side

  // 2.7.2: Set overcurrent thresholds (separate for low-side and high-side)
  ok &= setOvercurrentThresholds(
      ocThresholdLowSide,   // UVW low side (RSHUNT or VDS based on useVdsLowSide)
      ocThresholdHighSide,  // UVW high side (always VDS)
      ocThresholdLowSide,   // Y2 low side (RSHUNT or VDS based on useVdsLowSide)
      ocThresholdHighSide); // Y2 high side (always VDS)

  // 2.7.3: Set overcurrent blanking times
  ok &= setOvercurrentBlanking(ocBlanking,  // UVW low side
                               ocBlanking,  // UVW high side
                               ocBlanking,  // Y2 low side
                               ocBlanking); // Y2 high side

  // 2.7.4: Set overcurrent deglitch times
  ok &= setOvercurrentDeglitch(ocDeglitch,  // UVW low side
                               ocDeglitch,  // UVW high side
                               ocDeglitch,  // Y2 low side
                               ocDeglitch); // Y2 high side

  // 2.7.5: Enable/disable VDS monitoring for low-side
  ok &= enableVdsMonitoringLow(
      useVdsLowSide ? tmc9660::tmcl::VdsUsage::ENABLED : tmc9660::tmcl::VdsUsage::DISABLED,  // UVW
      useVdsLowSide ? tmc9660::tmcl::VdsUsage::ENABLED : tmc9660::tmcl::VdsUsage::DISABLED); // Y2

  // 2.7.6: Enable VGS short protection (all transitions)
  ok &= configureVgsShortProtectionUVW(tmc9660::tmcl::VgsShortEnable::ENABLED,  // Low side ON
                                       tmc9660::tmcl::VgsShortEnable::ENABLED,  // Low side OFF
                                       tmc9660::tmcl::VgsShortEnable::ENABLED,  // High side ON
                                       tmc9660::tmcl::VgsShortEnable::ENABLED); // High side OFF

  ok &= configureVgsShortProtectionY2(tmc9660::tmcl::VgsShortEnable::ENABLED,  // Low side ON
                                      tmc9660::tmcl::VgsShortEnable::ENABLED,  // Low side OFF
                                      tmc9660::tmcl::VgsShortEnable::ENABLED,  // High side ON
                                      tmc9660::tmcl::VgsShortEnable::ENABLED); // High side OFF

  // 2.7.7: Set VGS short blanking and deglitch times
  ok &= setVgsShortBlankingTime(vgsBlanking, vgsBlanking); // UVW, Y2
  ok &= setVgsShortDeglitchTime(vgsDeglitch, vgsDeglitch); // UVW, Y2

  // ============================================================================
  // PART 3: FAULT HANDLING CONFIGURATION
  // ============================================================================

  // Step 3.1: Configure retry behavior after gate driver fault
  ok &= setRetryBehavior(profile.retryBehaviour);

  // Step 3.2: Configure drive fault behavior after all retries fail
  ok &= setDriveFaultBehavior(profile.faultBehaviour);

  // Step 3.3: Set maximum number of fault handler retries
  ok &= setFaultHandlerRetries(profile.faultHandlerRetries);

  return ok;
}

//***************************************************************************
//**                  SUBSYSTEM: Sensors                                  **//
//***************************************************************************

// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//  HALL sensors (digital Hall)
// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

bool TMC9660::FeedbackSense::configureHall(tmc9660::tmcl::HallSectorOffset sectorOffset,
                                           tmc9660::tmcl::Direction inverted,
                                           tmc9660::tmcl::EnableDisable enableExtrapolation,
                                           uint8_t filterLength) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::HALL_SECTOR_OFFSET,
                              static_cast<uint32_t>(sectorOffset));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::HALL_INVERT_DIRECTION,
                              static_cast<uint32_t>(inverted));
  ok &=
      driver.writeParameter(tmc9660::tmcl::Parameters::HALL_EXTRAPOLATION_ENABLE,
                            enableExtrapolation == tmc9660::tmcl::EnableDisable::ENABLED ? 1u : 0u);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::HALL_FILTER_LENGTH,
                              static_cast<uint32_t>(filterLength));
  return ok;
}

bool TMC9660::FeedbackSense::setHallPositionOffsets(int16_t offset0, int16_t offset60,
                                                    int16_t offset120, int16_t offset180,
                                                    int16_t offset240, int16_t offset300,
                                                    int16_t globalOffset) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::HALL_POSITION_0_OFFSET,
                              static_cast<uint32_t>(static_cast<int32_t>(offset0)));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::HALL_POSITION_60_OFFSET,
                              static_cast<uint32_t>(static_cast<int32_t>(offset60)));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::HALL_POSITION_120_OFFSET,
                              static_cast<uint32_t>(static_cast<int32_t>(offset120)));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::HALL_POSITION_180_OFFSET,
                              static_cast<uint32_t>(static_cast<int32_t>(offset180)));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::HALL_POSITION_240_OFFSET,
                              static_cast<uint32_t>(static_cast<int32_t>(offset240)));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::HALL_POSITION_300_OFFSET,
                              static_cast<uint32_t>(static_cast<int32_t>(offset300)));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::HALL_PHI_E_OFFSET,
                              static_cast<uint32_t>(static_cast<int32_t>(globalOffset)));
  return ok;
}

bool TMC9660::FeedbackSense::setHallPositionOffsetsDegrees(float offset0Deg, float offset60Deg,
                                                           float offset120Deg, float offset180Deg,
                                                           float offset240Deg, float offset300Deg,
                                                           float globalOffsetDeg) noexcept {
  return setHallPositionOffsets(
      degreesToHallOffset(offset0Deg), degreesToHallOffset(offset60Deg),
      degreesToHallOffset(offset120Deg), degreesToHallOffset(offset180Deg),
      degreesToHallOffset(offset240Deg), degreesToHallOffset(offset300Deg),
      degreesToHallOffset(globalOffsetDeg));
}

bool TMC9660::FeedbackSense::setHallPositionOffsetsRadians(float offset0Rad, float offset60Rad,
                                                           float offset120Rad, float offset180Rad,
                                                           float offset240Rad, float offset300Rad,
                                                           float globalOffsetRad) noexcept {
  return setHallPositionOffsets(
      radiansToHallOffset(offset0Rad), radiansToHallOffset(offset60Rad),
      radiansToHallOffset(offset120Rad), radiansToHallOffset(offset180Rad),
      radiansToHallOffset(offset240Rad), radiansToHallOffset(offset300Rad),
      radiansToHallOffset(globalOffsetRad));
}

int16_t TMC9660::FeedbackSense::degreesToHallOffset(float degrees) noexcept {
  // Convert degrees to 16-bit signed integer representation
  // Formula: value = (degrees * 65536) / 360
  // This maps 360° to the full 16-bit range (-32768 to 32767)
  constexpr float DEGREES_TO_UNITS = 65536.0f / 360.0f;
  float value = degrees * DEGREES_TO_UNITS;

  // Round to nearest integer
  int32_t intValue = static_cast<int32_t>(value + (value >= 0.0f ? 0.5f : -0.5f));

  // Wrap around to stay within 16-bit signed integer range [-32768, 32767]
  // This handles angles > 360° or < -360° by normalizing using modulo arithmetic
  constexpr int32_t RANGE = 65536;
  intValue %= RANGE;
  // Normalize negative modulo results to positive range [0, 65535]
  if (intValue < 0) {
    intValue += RANGE;
  }
  // Convert values in [32768, 65535] to negative range [-32768, -1]
  if (intValue > 32767) {
    intValue -= RANGE;
  }

  return static_cast<int16_t>(intValue);
}

int16_t TMC9660::FeedbackSense::radiansToHallOffset(float radians) noexcept {
  // Convert radians to 16-bit signed integer representation
  // Formula: value = (radians * 65536) / (2 * π)
  // This maps 2π radians to the full 16-bit range (-32768 to 32767)
  constexpr float RADIANS_TO_UNITS = 65536.0f / (2.0f * 3.14159265358979323846f);
  float value = radians * RADIANS_TO_UNITS;

  // Round to nearest integer
  int32_t intValue = static_cast<int32_t>(value + (value >= 0.0f ? 0.5f : -0.5f));

  // Wrap around to stay within 16-bit signed integer range [-32768, 32767]
  // This handles angles > 2π or < -2π by normalizing using modulo arithmetic
  constexpr int32_t RANGE = 65536;
  intValue %= RANGE;
  // Normalize negative modulo results to positive range [0, 65535]
  if (intValue < 0) {
    intValue += RANGE;
  }
  // Convert values in [32768, 65535] to negative range [-32768, -1]
  if (intValue > 32767) {
    intValue -= RANGE;
  }

  return static_cast<int16_t>(intValue);
}

bool TMC9660::FeedbackSense::getHallPhiE(int16_t& phiE) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::HALL_PHI_E, tmp))
    return false;
  phiE = static_cast<int16_t>(tmp);
  return true;
}

// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//  ABN encoders (ABN1, ABN2)
// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

bool TMC9660::FeedbackSense::configureABNEncoder(
    uint32_t countsPerRev, tmc9660::tmcl::Direction inverted,
    tmc9660::tmcl::EnableDisable nChannelInverted) noexcept {
  uint32_t steps = countsPerRev;
  if (steps > 0xFFFFFF)
    steps = 0xFFFFFF;
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ABN_1_STEPS, steps);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ABN_1_DIRECTION,
                              inverted == tmc9660::tmcl::Direction::INVERTED ? 1u : 0u);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ABN_1_N_CHANNEL_INVERTED,
                              nChannelInverted == tmc9660::tmcl::EnableDisable::ENABLED ? 1u : 0u);
  return ok;
}

bool TMC9660::FeedbackSense::configureABNInitialization(tmc9660::tmcl::AbnInitMethod initMethod,
                                                        uint16_t initDelay, int32_t initVelocity,
                                                        int16_t nChannelOffset) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ABN_1_INIT_METHOD,
                              static_cast<uint32_t>(initMethod));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ABN_1_INIT_DELAY, initDelay);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ABN_1_INIT_VELOCITY,
                              static_cast<uint32_t>(initVelocity));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ABN_1_N_CHANNEL_PHI_E_OFFSET,
                              static_cast<uint32_t>(static_cast<int32_t>(nChannelOffset)));
  return ok;
}

bool TMC9660::FeedbackSense::getABNInitializationState(
    tmc9660::tmcl::AbnInitState& state) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ABN_1_INIT_STATE, tmp))
    return false;
  state = static_cast<tmc9660::tmcl::AbnInitState>(tmp);
  return true;
}

bool TMC9660::FeedbackSense::getABNPhiE(int16_t& phiE) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ABN_1_PHI_E, tmp))
    return false;
  phiE = static_cast<int16_t>(tmp);
  return true;
}

bool TMC9660::FeedbackSense::getABNRawValue(uint32_t& value) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::ABN_1_VALUE, value);
}

bool TMC9660::FeedbackSense::configureABNNChannel(
    tmc9660::tmcl::AbnNChannelFiltering filterMode,
    tmc9660::tmcl::EnableDisable clearOnNextNull) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ABN_1_N_CHANNEL_FILTERING,
                              static_cast<uint32_t>(filterMode));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ABN_1_CLEAR_ON_NEXT_NULL,
                              clearOnNextNull == tmc9660::tmcl::EnableDisable::ENABLED ? 1u : 0u);
  return ok;
}

bool TMC9660::FeedbackSense::configureSecondaryABNEncoder(uint32_t countsPerRev,
                                                          tmc9660::tmcl::Direction inverted,
                                                          uint8_t gearRatio) noexcept {
  uint32_t steps = countsPerRev;
  if (steps > 0xFFFFFF)
    steps = 0xFFFFFF;
  if (gearRatio == 0)
    gearRatio = 1;

  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ABN_2_STEPS, steps);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ABN_2_DIRECTION,
                              inverted == tmc9660::tmcl::Direction::INVERTED ? 1u : 0u);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ABN_2_GEAR_RATIO, gearRatio);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ABN_2_ENABLE, 1u);
  return ok;
}

bool TMC9660::FeedbackSense::getSecondaryABNCountsPerRev(uint32_t& counts) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::ABN_2_STEPS, counts);
}

bool TMC9660::FeedbackSense::getSecondaryABNDirection(tmc9660::tmcl::Direction& dir) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ABN_2_DIRECTION, tmp))
    return false;
  dir = static_cast<tmc9660::tmcl::Direction>(tmp);
  return true;
}

bool TMC9660::FeedbackSense::getSecondaryABNGearRatio(uint8_t& ratio) noexcept {
  uint32_t tmp;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ABN_2_GEAR_RATIO, tmp))
    return false;
  ratio = static_cast<uint8_t>(tmp);
  return true;
}

bool TMC9660::FeedbackSense::setSecondaryABNEncoderEnabled(bool enable) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::ABN_2_ENABLE, enable ? 1u : 0u);
}

bool TMC9660::FeedbackSense::getSecondaryABNEncoderValue(uint32_t& value) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::ABN_2_VALUE, value);
}
// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//  SPI encoder timing & frame size
// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

bool TMC9660::FeedbackSense::configureSPIEncoder(uint8_t cmdSize, uint16_t csSettleTimeNs,
                                                 uint8_t csIdleTimeUs) noexcept {
  bool ok = true;
  ok &=
      driver.writeParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_MAIN_TRANSFER_CMD_SIZE, cmdSize);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_CS_SETTLE_DELAY_TIME,
                              csSettleTimeNs);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_CS_IDLE_DELAY_TIME,
                              csIdleTimeUs);
  return ok;
}

bool TMC9660::FeedbackSense::configureSPIEncoderDataFormat(
    uint32_t positionMask, uint8_t positionShift,
    tmc9660::tmcl::Direction invertDirection) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_POSITION_COUNTER_MASK,
                              positionMask);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_POSITION_COUNTER_SHIFT,
                              positionShift);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_DIRECTION,
                              static_cast<uint32_t>(invertDirection));
  return ok;
}

bool TMC9660::FeedbackSense::setSPIEncoderRequestData(const uint8_t* requestData,
                                                      uint8_t size) noexcept {
  if (size == 0 || size > 16)
    return false;
  uint32_t w0 = 0, w1 = 0, w2 = 0, w3 = 0;
  for (uint8_t i = 0; i < size && i < 16; ++i) {
    uint32_t val = static_cast<uint32_t>(requestData[i]) & 0xFFu;
    if (i < 4)
      w0 |= val << (8 * i);
    else if (i < 8)
      w1 |= val << (8 * (i - 4));
    else if (i < 12)
      w2 |= val << (8 * (i - 8));
    else
      w3 |= val << (8 * (i - 12));
  }
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_TRANSFER_DATA_3_0, w0);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_TRANSFER_DATA_7_4, w1);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_TRANSFER_DATA_11_8, w2);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_TRANSFER_DATA_15_12, w3);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_MAIN_TRANSFER_CMD_SIZE, size);
  return ok;
}

bool TMC9660::FeedbackSense::configureSPIEncoderInitialization(
    tmc9660::tmcl::SpiInitMethod initMethod, int16_t offset) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_INITIALIZATION_METHOD,
                              static_cast<uint32_t>(initMethod));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_OFFSET,
                              static_cast<uint32_t>(static_cast<int32_t>(offset)));
  return ok;
}

bool TMC9660::FeedbackSense::setSPIEncoderLUTCorrection(tmc9660::tmcl::EnableDisable enable,
                                                        int8_t shiftFactor) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_LUT_CORRECTION_ENABLE,
                              enable == tmc9660::tmcl::EnableDisable::ENABLED ? 1u : 0u);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_LUT_COMMON_SHIFT_FACTOR,
                              static_cast<uint32_t>(static_cast<int32_t>(shiftFactor)));
  return ok;
}

bool TMC9660::FeedbackSense::uploadSPIEncoderLUTEntry(uint8_t index, int8_t value) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_LUT_ADDRESS_SELECT, index);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SPI_LUT_DATA,
                              static_cast<uint32_t>(static_cast<int32_t>(value)));
  return ok;
}

bool TMC9660::FeedbackSense::getSPIEncoderCSSettleDelay(uint16_t& timeNs) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_CS_SETTLE_DELAY_TIME, v))
    return false;
  timeNs = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::FeedbackSense::getSPIEncoderCSIdleDelay(uint8_t& timeUs) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_CS_IDLE_DELAY_TIME, v))
    return false;
  timeUs = static_cast<uint8_t>(v);
  return true;
}

bool TMC9660::FeedbackSense::getSPIEncoderMainCmdSize(uint8_t& size) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_MAIN_TRANSFER_CMD_SIZE, v))
    return false;
  size = static_cast<uint8_t>(v);
  return true;
}

bool TMC9660::FeedbackSense::getSPIEncoderSecondaryCmdSize(uint8_t& size) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_SECONDARY_TRANSFER_CMD_SIZE, v))
    return false;
  size = static_cast<uint8_t>(v);
  return true;
}

bool TMC9660::FeedbackSense::getSPIEncoderPositionMask(uint32_t& mask) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_POSITION_COUNTER_MASK, mask);
}

bool TMC9660::FeedbackSense::getSPIEncoderPositionShift(uint8_t& shift) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_POSITION_COUNTER_SHIFT, v))
    return false;
  shift = static_cast<uint8_t>(v);
  return true;
}

bool TMC9660::FeedbackSense::getSPIEncoderPositionValue(uint32_t& value) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_POSITION_COUNTER_VALUE, value);
}

bool TMC9660::FeedbackSense::getSPIEncoderCommutationAngle(int16_t& angle) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_COMMUTATION_ANGLE, v))
    return false;
  angle = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::FeedbackSense::getSPIEncoderInitialization(tmc9660::tmcl::SpiInitMethod& method,
                                                         int16_t& offset) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_INITIALIZATION_METHOD, v))
    return false;
  method = static_cast<tmc9660::tmcl::SpiInitMethod>(v);
  if (!driver.readParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_OFFSET, v))
    return false;
  offset = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::FeedbackSense::getSPIEncoderDirection(tmc9660::tmcl::Direction& dir) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::SPI_ENCODER_DIRECTION, v))
    return false;
  dir = static_cast<tmc9660::tmcl::Direction>(v);
  return true;
}

bool TMC9660::FeedbackSense::getSPIEncoderLUTAddress(uint8_t& address) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::SPI_LUT_ADDRESS_SELECT, v))
    return false;
  address = static_cast<uint8_t>(v);
  return true;
}

bool TMC9660::FeedbackSense::getSPIEncoderLUTData(int8_t& data) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::SPI_LUT_DATA, v))
    return false;
  data = static_cast<int8_t>(v);
  return true;
}

bool TMC9660::FeedbackSense::getSPIEncoderLUTShiftFactor(int8_t& shiftFactor) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::SPI_LUT_COMMON_SHIFT_FACTOR, v))
    return false;
  shiftFactor = static_cast<int8_t>(v);
  return true;
}

// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
//  Auto-Configuration Functions
// –––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

bool TMC9660::FeedbackSense::configureAuto(const HallConfig& config) noexcept {
  bool ok = true;

  // Step 1: Configure basic Hall sensor settings
  ok &= configureHall(config.sectorOffset, config.direction, config.extrapolation,
                      config.filterLength);

  // Step 2: Set position offsets if provided
  if (config.offset0Deg.has_value() || config.offset60Deg.has_value() ||
      config.offset120Deg.has_value() || config.offset180Deg.has_value() ||
      config.offset240Deg.has_value() || config.offset300Deg.has_value() ||
      config.globalOffsetDeg.has_value()) {
    // Use provided values or defaults
    float offset0 = config.offset0Deg.value_or(0.0f);
    float offset60 = config.offset60Deg.value_or(60.0f);
    float offset120 = config.offset120Deg.value_or(120.0f);
    float offset180 = config.offset180Deg.value_or(180.0f);
    float offset240 = config.offset240Deg.value_or(240.0f);
    float offset300 = config.offset300Deg.value_or(300.0f);
    float globalOffset = config.globalOffsetDeg.value_or(0.0f);

    ok &= setHallPositionOffsetsDegrees(offset0, offset60, offset120, offset180, offset240,
                                        offset300, globalOffset);
  }

  return ok;
}

bool TMC9660::FeedbackSense::configureAuto(const AbnConfig& config) noexcept {
  bool ok = true;

  // Step 1: Configure ABN encoder basic settings
  ok &= configureABNEncoder(config.countsPerRev, config.direction, config.nChannelInverted);

  // Step 2: Configure ABN initialization method
  ok &= configureABNInitialization(config.initMethod, config.initDelay, config.initVelocity,
                                   config.nChannelOffset);

  // Step 3: Configure N-channel filtering
  ok &= configureABNNChannel(config.nChannelFiltering, config.clearOnNextNull);

  return ok;
}

bool TMC9660::FeedbackSense::configureAuto(const Abn2Config& config) noexcept {
  bool ok = true;

  // Configure secondary ABN encoder
  ok &= configureSecondaryABNEncoder(config.countsPerRev, config.direction, config.gearRatio);

  // Enable or disable the encoder
  ok &= setSecondaryABNEncoderEnabled(config.enable);

  return ok;
}

bool TMC9660::FeedbackSense::configureAuto(const SpiEncoderConfig& config) noexcept {
  bool ok = true;

  // Step 1: Configure SPI encoder timing and frame size
  ok &= configureSPIEncoder(config.cmdSize, config.csSettleTimeNs, config.csIdleTimeUs);

  // Step 2: Configure SPI encoder data format
  ok &= configureSPIEncoderDataFormat(config.positionMask, config.positionShift, config.direction);

  // Step 3: Set request data for continuous transfer mode if provided
  if (config.requestData.has_value()) {
    ok &= setSPIEncoderRequestData(config.requestData->data(), config.cmdSize);

    // Enable continuous transfer mode
    ok &= driver.writeParameter(
        tmc9660::tmcl::Parameters::SPI_ENCODER_TRANSFER,
        static_cast<uint32_t>(tmc9660::tmcl::SpiEncoderTransfer::CONTINUOUS_POSITION_COUNTER_READ));
  }

  // Step 4: Configure SPI encoder initialization
  ok &= configureSPIEncoderInitialization(config.initMethod, config.offset);

  // Step 5: Configure LUT correction if enabled
  if (config.lutCorrection == tmc9660::tmcl::EnableDisable::ENABLED) {
    ok &= setSPIEncoderLUTCorrection(config.lutCorrection, config.lutShiftFactor);
  }

  return ok;
}

//***************************************************************************
//**                  SUBSYSTEM: Torque/Flux Control (FOC)                **//
//***************************************************************************

bool TMC9660::TorqueFluxControl::stop() noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::MST, 0, 0, 0, nullptr);
}

bool TMC9660::TorqueFluxControl::setTargetTorque(int16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TARGET_TORQUE,
                               static_cast<uint32_t>(static_cast<int32_t>(milliamps)));
}

bool TMC9660::TorqueFluxControl::getActualTorque(int16_t& milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_TORQUE, v))
    return false;
  milliamps = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::setTargetFlux(int16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TARGET_FLUX,
                               static_cast<uint32_t>(static_cast<int32_t>(milliamps)));
}

bool TMC9660::TorqueFluxControl::getActualFlux(int32_t& milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_FLUX, v))
    return false;
  milliamps = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::setTorqueOffset(int16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TORQUE_OFFSET,
                               static_cast<uint32_t>(static_cast<int32_t>(milliamps)));
}

bool TMC9660::TorqueFluxControl::getTorqueOffset(int16_t& milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::TORQUE_OFFSET, v))
    return false;
  milliamps = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::setFluxOffset(int16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::FLUX_OFFSET,
                               static_cast<uint32_t>(static_cast<int32_t>(milliamps)));
}

bool TMC9660::TorqueFluxControl::getFluxOffset(int16_t& milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FLUX_OFFSET, v))
    return false;
  milliamps = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::setCurrentLoopGains(uint16_t p, uint16_t i, bool separate,
                                                     uint16_t fluxP, uint16_t fluxI) noexcept {
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

bool TMC9660::TorqueFluxControl::setTorqueFluxPiSeparation(
    tmc9660::tmcl::TorqueFluxPiSeparation sep) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::SEPARATE_TORQUE_FLUX_PI_PARAMETERS,
                               static_cast<uint32_t>(sep));
}

bool TMC9660::TorqueFluxControl::setCurrentNormalization(
    tmc9660::tmcl::CurrentPiNormalization pNorm,
    tmc9660::tmcl::CurrentPiNormalization iNorm) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::CURRENT_NORM_P,
                              static_cast<uint32_t>(pNorm));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::CURRENT_NORM_I,
                              static_cast<uint32_t>(iNorm));
  return ok;
}

bool TMC9660::TorqueFluxControl::getTorquePiError(int32_t& error) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::TORQUE_PI_ERROR, v))
    return false;
  error = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::getFluxPiError(int32_t& error) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FLUX_PI_ERROR, v))
    return false;
  error = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::getTorquePiIntegrator(int32_t& integrator) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::TORQUE_PI_INTEGRATOR, v))
    return false;
  integrator = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::getFluxPiIntegrator(int32_t& integrator) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FLUX_PI_INTEGRATOR, v))
    return false;
  integrator = static_cast<int32_t>(v);
  return true;
}

//-------------------------------------------------------------------------
// Auto-Configuration
//-------------------------------------------------------------------------

bool TMC9660::TorqueFluxControl::configureAuto(const TorqueFluxConfig& config) noexcept {
  bool ok = true;

  // Step 1: Calculate or use provided PI gains
  uint16_t torqueP, torqueI;
  uint16_t fluxP, fluxI;

  if (config.torqueP.has_value() && config.torqueI.has_value()) {
    // User provided direct PI gains - use them
    torqueP = config.torqueP.value();
    torqueI = config.torqueI.value();
  } else {
    // No direct gains provided - use defaults
    constexpr uint16_t DEFAULT_P = 50;
    constexpr uint16_t DEFAULT_I = 100;
    torqueP = DEFAULT_P;
    torqueI = DEFAULT_I;
  }

  // Flux gains: use provided values, or default to torque gains
  if (config.separateTorqueFluxLoops) {
    if (config.fluxP.has_value() && config.fluxI.has_value()) {
      fluxP = config.fluxP.value();
      fluxI = config.fluxI.value();
    } else {
      // Default flux gains to torque gains
      fluxP = torqueP;
      fluxI = torqueI;
    }
  } else {
    // Not used when loops are combined, but initialize anyway
    fluxP = torqueP;
    fluxI = torqueI;
  }

  // Step 2: Configure torque/flux loop separation and PI gains
  if (config.separateTorqueFluxLoops) {
    ok &=
        setTorqueFluxPiSeparation(tmc9660::tmcl::TorqueFluxPiSeparation::TORQUE_FLUX_PI_SEPARATED);
    ok &= setCurrentLoopGains(torqueP, torqueI, true, fluxP, fluxI);
  } else {
    ok &= setTorqueFluxPiSeparation(tmc9660::tmcl::TorqueFluxPiSeparation::TORQUE_FLUX_PI_COMBINED);
    ok &= setCurrentLoopGains(torqueP, torqueI, false);
  }

  // Step 3: Configure normalization (use provided values or defaults)
  // Default: SHIFT_8_BIT for P (good balance), SHIFT_16_BIT for I (better precision)
  // These defaults match the datasheet defaults and work well for most motors.
  tmc9660::tmcl::CurrentPiNormalization pNorm =
      config.pNormalization.value_or(tmc9660::tmcl::CurrentPiNormalization::SHIFT_8_BIT);
  tmc9660::tmcl::CurrentPiNormalization iNorm =
      config.iNormalization.value_or(tmc9660::tmcl::CurrentPiNormalization::SHIFT_16_BIT);
  ok &= setCurrentNormalization(pNorm, iNorm);

  // Step 4: Configure offsets
  ok &= setTorqueOffset(config.torqueOffset_mA);
  ok &= setFluxOffset(config.fluxOffset_mA);

  // Step 5: Configure field weakening if enabled
  if (config.enableFieldWeakening) {
    ok &= setFieldWeakeningI(config.fieldWeakeningI);

    // Convert percentage threshold to register value
    // Read current OUTPUT_VOLTAGE_LIMIT to calculate threshold
    uint16_t outputVoltageLimit;
    if (driver.motorConfig.getOutputVoltageLimit(outputVoltageLimit)) {
      // Calculate threshold as percentage of OUTPUT_VOLTAGE_LIMIT
      float thresholdValue =
          static_cast<float>(outputVoltageLimit) * config.fieldWeakeningVoltageThresholdPercent;
      uint16_t thresholdRegister = static_cast<uint16_t>(thresholdValue + 0.5f);

      // Clamp to valid range [0-32767]
      if (thresholdRegister > 32767)
        thresholdRegister = 32767;

      ok &= setFieldWeakeningVoltageThreshold(thresholdRegister);
    } else {
      // Fallback: use percentage of default OUTPUT_VOLTAGE_LIMIT (8000)
      constexpr uint16_t DEFAULT_OUTPUT_VOLTAGE_LIMIT = 8000;
      float thresholdValue = static_cast<float>(DEFAULT_OUTPUT_VOLTAGE_LIMIT) *
                             config.fieldWeakeningVoltageThresholdPercent;
      uint16_t thresholdRegister = static_cast<uint16_t>(thresholdValue + 0.5f);
      ok &= setFieldWeakeningVoltageThreshold(thresholdRegister);
    }
  }

  // Step 6: Configure torque biquad filter if enabled
  if (config.enableTorqueBiquadFilter) {
    ok &= setTargetTorqueBiquadFilterEnable(true);

    // Configure biquad filter coefficients if provided
    // Note: Coefficients are in Q4.20 format (24-bit, 4 integer + 20 fractional bits)
    // Default values if not provided: BCOEFF_0 = 1048576 (1.0), others = 0
    if (config.biquadACoeff1.has_value()) {
      ok &= setTargetTorqueBiquadFilterACoeff1(config.biquadACoeff1.value());
    }
    if (config.biquadACoeff2.has_value()) {
      ok &= setTargetTorqueBiquadFilterACoeff2(config.biquadACoeff2.value());
    }
    if (config.biquadBCoeff0.has_value()) {
      ok &= setTargetTorqueBiquadFilterBCoeff0(config.biquadBCoeff0.value());
    }
    if (config.biquadBCoeff1.has_value()) {
      ok &= setTargetTorqueBiquadFilterBCoeff1(config.biquadBCoeff1.value());
    }
    if (config.biquadBCoeff2.has_value()) {
      ok &= setTargetTorqueBiquadFilterBCoeff2(config.biquadBCoeff2.value());
    }
    // If coefficients are not provided, filter uses hardware defaults:
    // BCOEFF_0 = 1048576 (1.0 in Q4.20), all others = 0 (pass-through)
  }

  return ok;
}

//-------------------------------------------------------------------------
// Velocity control (123–139)
//-------------------------------------------------------------------------

//***************************************************************************
//**                  SUBSYSTEM: Velocity Control                           **//
//***************************************************************************

bool TMC9660::VelocityControl::stop() noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::MST, 0, 0, 0, nullptr);
}

bool TMC9660::VelocityControl::setVelocitySensor(
    tmc9660::tmcl::VelocitySensorSelection sel) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::VELOCITY_SENSOR_SELECTION,
                               static_cast<uint32_t>(sel));
}

bool TMC9660::VelocityControl::getVelocitySensor(
    tmc9660::tmcl::VelocitySensorSelection& sel) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::VELOCITY_SENSOR_SELECTION, v))
    return false;
  sel = static_cast<tmc9660::tmcl::VelocitySensorSelection>(v);
  return true;
}
bool TMC9660::VelocityControl::setTargetVelocity(int32_t velocity) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TARGET_VELOCITY,
                               static_cast<uint32_t>(velocity));
}

bool TMC9660::VelocityControl::getActualVelocity(int32_t& velocity) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY, v))
    return false;
  velocity = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::VelocityControl::setVelocityOffset(int32_t offset) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::VELOCITY_OFFSET,
                               static_cast<uint32_t>(offset));
}
bool TMC9660::VelocityControl::getVelocityOffset(int32_t& offset) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::VELOCITY_OFFSET, v))
    return false;
  offset = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::VelocityControl::setVelocityLoopGains(uint16_t p, uint16_t i) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::VELOCITY_P, p);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::VELOCITY_I, i);
  return ok;
}

bool TMC9660::VelocityControl::setVelocityNormalization(
    tmc9660::tmcl::VelocityPiNorm pNorm, tmc9660::tmcl::VelocityPiNorm iNorm) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::VELOCITY_NORM_P,
                              static_cast<uint32_t>(pNorm));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::VELOCITY_NORM_I,
                              static_cast<uint32_t>(iNorm));
  return ok;
}

bool TMC9660::VelocityControl::getVelocityPiIntegrator(int32_t& integrator) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::VELOCITY_PI_INTEGRATOR, v))
    return false;
  integrator = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::VelocityControl::getVelocityPiError(int32_t& error) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::VELOCITY_PI_ERROR, v))
    return false;
  error = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::VelocityControl::setVelocityScalingFactor(uint16_t factor) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::VELOCITY_SCALING_FACTOR, factor);
}

bool TMC9660::VelocityControl::getVelocityScalingFactor(uint16_t& factor) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::VELOCITY_SCALING_FACTOR, v))
    return false;
  factor = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::VelocityControl::setStopOnVelocityDeviation(uint32_t maxError,
                                                          bool softStop) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::STOP_ON_VELOCITY_DEVIATION, maxError);
  auto setting = softStop ? tmc9660::tmcl::EventStopSettings::STOP_ON_VEL_DEVIATION_SOFT_STOP
                          : tmc9660::tmcl::EventStopSettings::STOP_ON_VEL_DEVIATION;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::EVENT_STOP_SETTINGS,
                              static_cast<uint32_t>(setting));
  return ok;
}

bool TMC9660::VelocityControl::getStopOnVelocityDeviation(uint32_t& maxError,
                                                          bool& softStop) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::STOP_ON_VELOCITY_DEVIATION, v))
    return false;
  maxError = v;
  uint32_t mode;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::EVENT_STOP_SETTINGS, mode))
    return false;
  softStop = (mode == static_cast<uint32_t>(
                          tmc9660::tmcl::EventStopSettings::STOP_ON_VEL_DEVIATION_SOFT_STOP));
  return true;
}

bool TMC9660::VelocityControl::setVelocityLoopDownsampling(uint8_t divider) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::VELOCITY_LOOP_DOWNSAMPLING, divider);
}

bool TMC9660::VelocityControl::getVelocityLoopDownsampling(uint8_t& divider) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::VELOCITY_LOOP_DOWNSAMPLING, v))
    return false;
  divider = static_cast<uint8_t>(v);
  return true;
}

bool TMC9660::VelocityControl::setVelocityMeterSwitchThreshold(uint32_t threshold) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::VELOCITY_METER_SWITCH_THRESHOLD,
                               threshold);
}

bool TMC9660::VelocityControl::getVelocityMeterSwitchThreshold(uint32_t& threshold) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::VELOCITY_METER_SWITCH_THRESHOLD, v))
    return false;
  threshold = v;
  return true;
}

bool TMC9660::VelocityControl::setVelocityMeterSwitchHysteresis(uint16_t hysteresis) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::VELOCITY_METER_SWITCH_HYSTERESIS,
                               hysteresis);
}

bool TMC9660::VelocityControl::getVelocityMeterSwitchHysteresis(uint16_t& hysteresis) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::VELOCITY_METER_SWITCH_HYSTERESIS, v))
    return false;
  hysteresis = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::VelocityControl::getVelocityMeterMode(
    tmc9660::tmcl::VelocityMeterMode& mode) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::VELOCITY_METER_MODE, v))
    return false;
  mode = static_cast<tmc9660::tmcl::VelocityMeterMode>(v);
  return true;
}

//-------------------------------------------------------------------------
// Auto-Configuration
//-------------------------------------------------------------------------

bool TMC9660::VelocityControl::configureAuto(const VelocityConfig& config) noexcept {
  bool ok = true;

  // Step 1: Configure velocity sensor selection
  ok &= setVelocitySensor(config.sensorSelection);

  // Step 2: Configure PI gains (use provided values or defaults)
  constexpr uint16_t DEFAULT_P = 800; // Default P gain (typical for velocity control)
  constexpr uint16_t DEFAULT_I = 1;   // Default I gain (typical for velocity control)

  uint16_t velocityP = config.velocityP.value_or(DEFAULT_P);
  uint16_t velocityI = config.velocityI.value_or(DEFAULT_I);

  // Clamp to valid range [0-32767]
  if (velocityP > 32767)
    velocityP = 32767;
  if (velocityI > 32767)
    velocityI = 32767;
  if (velocityI < 1)
    velocityI = 1; // Minimum I gain

  ok &= setVelocityLoopGains(velocityP, velocityI);

  // Step 3: Configure normalization
  ok &= setVelocityNormalization(config.pNormalization, config.iNormalization);

  // Step 4: Configure velocity scaling factor
  // Formula from datasheet: k_RPM = (CPR × 2^24) / (40MHz × 60)
  // Note: It's recommended to leave scaling at 1 and handle conversion externally
  uint16_t velocityScalingFactor = 1; // Default: no internal scaling
  uint32_t cpr = 0; // Counts per mechanical revolution (used later for meter threshold)

  // Always calculate CPR (needed for meter threshold calculation even if scaling is provided)
  uint8_t polePairs = 1;
  if (config.motorPolePairs.has_value()) {
    polePairs = config.motorPolePairs.value();
  } else {
    // Read from MotorConfig
    uint32_t motorTypeVal, polePairsVal;
    if (driver.readParameter(tmc9660::tmcl::Parameters::MOTOR_TYPE, motorTypeVal) &&
        driver.readParameter(tmc9660::tmcl::Parameters::MOTOR_POLE_PAIRS, polePairsVal)) {
      polePairs = static_cast<uint8_t>(polePairsVal);
    }
  }

  // Calculate CPR based on sensor type
  switch (config.sensorSelection) {
  case tmc9660::tmcl::VelocitySensorSelection::SAME_AS_COMMUTATION:
    // CPR = 2^16 × motorPolePairs
    cpr = static_cast<uint32_t>(65536UL * polePairs);
    break;
  case tmc9660::tmcl::VelocitySensorSelection::DIGITAL_HALL:
    // CPR = 6 × motorPolePairs
    cpr = static_cast<uint32_t>(6 * polePairs);
    break;
  case tmc9660::tmcl::VelocitySensorSelection::ABN1_ENCODER:
  case tmc9660::tmcl::VelocitySensorSelection::ABN2_ENCODER:
  case tmc9660::tmcl::VelocitySensorSelection::SPI_ENCODER:
    // CPR = encoderCountsPerRev (from encoder datasheet)
    if (config.encoderCountsPerRev.has_value()) {
      cpr = config.encoderCountsPerRev.value();
    }
    // If not provided, cpr remains 0
    break;
  }

  // Calculate or use provided scaling factor
  if (config.velocityScalingFactor.has_value()) {
    // User provided explicit scaling factor
    velocityScalingFactor = config.velocityScalingFactor.value();
  } else if (cpr > 0) {
    // Auto-calculate scaling factor from CPR
    // Formula: k_RPM = (CPR × 2^24) / (40MHz × 60)
    constexpr float CONST_40MHZ_60 = 2400000000.0f; // 40MHz × 60
    constexpr float CONST_2_24 = 16777216.0f;       // 2^24

    float k_rpm = (static_cast<float>(cpr) * CONST_2_24) / CONST_40MHZ_60;
    velocityScalingFactor = static_cast<uint16_t>(k_rpm + 0.5f);

    // Clamp to valid range [1-2047]
    if (velocityScalingFactor < 1)
      velocityScalingFactor = 1;
    if (velocityScalingFactor > 2047)
      velocityScalingFactor = 2047;
  }
  // If cpr == 0 and no explicit scaling, use default scaling factor of 1

  ok &= setVelocityScalingFactor(velocityScalingFactor);

  // Step 5: Configure velocity loop downsampling
  ok &= setVelocityLoopDownsampling(config.loopDownsampling);

  // Step 6: Configure velocity reached threshold
  // Note: VELOCITY_REACHED_THRESHOLD is not a configurable parameter in TMC9660.
  // The VELOCITY_REACHED flag is set when both actual and target velocity are below
  // an internal threshold. The config.velocityReachedThreshold value is ignored.
  // (Parameter 134 is STOP_ON_VELOCITY_DEVIATION, not VELOCITY_REACHED_THRESHOLD)

  // Step 7: Configure stop on deviation if enabled
  if (config.stopOnDeviationMaxError.has_value()) {
    ok &= setStopOnVelocityDeviation(config.stopOnDeviationMaxError.value(),
                                     config.stopOnDeviationSoftStop);
  }

  // Step 8: Configure velocity meter switch threshold
  // Auto-calculate optimal threshold if not provided
  uint32_t meterThreshold = 2000; // Default value

  if (config.meterSwitchThreshold.has_value()) {
    meterThreshold = config.meterSwitchThreshold.value();
  } else if (cpr > 0) {
    // Auto-calculate optimal switchover point
    // Need: CPR (already calculated in Step 4), PWM frequency, loop downsampling, and velocity
    // scaling factor

    // Get PWM frequency
    uint32_t pwmFreq = 25000; // Default 25kHz
    if (config.pwmFrequency_Hz.has_value()) {
      pwmFreq = config.pwmFrequency_Hz.value();
    } else {
      uint32_t pwmFreqVal;
      if (driver.readParameter(tmc9660::tmcl::Parameters::MOTOR_PWM_FREQUENCY, pwmFreqVal)) {
        pwmFreq = pwmFreqVal;
      }
    }

    // Calculate velocity loop frequency
    // From datasheet: velocity loop frequency depends on PWM frequency and downsampling
    // f_Velo = f_PWM / (VELOCITY_LOOP_DOWNSAMPLING + 1)
    // Note: This is the actual execution frequency of the velocity control loop
    float f_velo = static_cast<float>(pwmFreq) / static_cast<float>(config.loopDownsampling + 1);

    // Calculate optimal switchover threshold if we have CPR
    if (cpr > 0) {
      // Formula 1: v_PerLim_RPM = 0.9 × (40MHz) / (CPR × 60 × 53)
      constexpr float CONST_40MHZ = 40000000.0f;
      float v_per_lim_rpm = 0.9f * CONST_40MHZ / (static_cast<float>(cpr) * 60.0f * 53.0f);

      // Formula 2: v_COP_RPM = 60 × (f_Velo + sqrt(f_Velo² + f_Velo × 40MHz × 8)) / (4 × CPR)
      float f_velo_sq = f_velo * f_velo;
      float sqrt_term = std::sqrt(f_velo_sq + f_velo * CONST_40MHZ * 8.0f);
      float v_cop_rpm = 60.0f * (f_velo + sqrt_term) / (4.0f * static_cast<float>(cpr));

      // Choose the smaller value
      float v_thr_rpm = std::min(v_per_lim_rpm, v_cop_rpm);

      // Convert to internal units: threshold = v_THR_RPM × k_RPM
      float threshold = v_thr_rpm * static_cast<float>(velocityScalingFactor);
      meterThreshold = static_cast<uint32_t>(threshold + 0.5f);

      // Clamp to valid range [0-134217727]
      if (meterThreshold > 134217727U)
        meterThreshold = 134217727U;
    }
    // If cpr == 0, use default threshold of 2000
  }

  ok &= setVelocityMeterSwitchThreshold(meterThreshold);
  ok &= setVelocityMeterSwitchHysteresis(config.meterHysteresis);

  // Step 9: Configure velocity offset
  ok &= setVelocityOffset(config.velocityOffset);

  // Step 10: Configure velocity biquad filter
  // Note: Velocity biquad filter is enabled by default in hardware for noise reduction
  if (config.enableVelocityBiquadFilter.has_value()) {
    ok &= setActualVelocityBiquadFilterEnable(config.enableVelocityBiquadFilter.value());
  }
  // If not explicitly set, filter remains in its default state (enabled)

  // Configure velocity biquad filter coefficients if provided
  // Note: Coefficients are in Q4.20 format (24-bit, 4 integer + 20 fractional bits)
  // Default values if not provided: Hardware-optimized defaults for velocity noise reduction
  if (config.velocityBiquadACoeff1.has_value()) {
    ok &= setActualVelocityBiquadFilterACoeff1(config.velocityBiquadACoeff1.value());
  }
  if (config.velocityBiquadACoeff2.has_value()) {
    ok &= setActualVelocityBiquadFilterACoeff2(config.velocityBiquadACoeff2.value());
  }
  if (config.velocityBiquadBCoeff0.has_value()) {
    ok &= setActualVelocityBiquadFilterBCoeff0(config.velocityBiquadBCoeff0.value());
  }
  if (config.velocityBiquadBCoeff1.has_value()) {
    ok &= setActualVelocityBiquadFilterBCoeff1(config.velocityBiquadBCoeff1.value());
  }
  if (config.velocityBiquadBCoeff2.has_value()) {
    ok &= setActualVelocityBiquadFilterBCoeff2(config.velocityBiquadBCoeff2.value());
  }

  return ok;
}

//-------------------------------------------------------------------------
// Position control (142–157)
//-------------------------------------------------------------------------

//***************************************************************************
//**                  SUBSYSTEM: Position Control                          **//
//***************************************************************************

bool TMC9660::PositionControl::stop() noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::MST, 0, 0, 0, nullptr);
}

bool TMC9660::PositionControl::setPositionSensor(
    tmc9660::tmcl::PositionSensorSelection sel) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::POSITION_SENSOR_SELECTION,
                               static_cast<uint32_t>(sel));
}

bool TMC9660::PositionControl::getPositionSensor(
    tmc9660::tmcl::PositionSensorSelection& sel) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::POSITION_SENSOR_SELECTION, v))
    return false;
  sel = static_cast<tmc9660::tmcl::PositionSensorSelection>(v);
  return true;
}

bool TMC9660::PositionControl::setTargetPosition(int32_t position) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TARGET_POSITION,
                               static_cast<uint32_t>(position));
}

bool TMC9660::PositionControl::getActualPosition(int32_t& position) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_POSITION, v))
    return false;
  position = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::PositionControl::setPositionScalingFactor(uint16_t factor) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::POSITION_SCALING_FACTOR, factor);
}

bool TMC9660::PositionControl::getPositionScalingFactor(uint16_t& factor) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::POSITION_SCALING_FACTOR, v))
    return false;
  factor = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::PositionControl::setPositionLoopGains(uint16_t p, uint16_t i) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::POSITION_P, p);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::POSITION_I, i);
  return ok;
}

bool TMC9660::PositionControl::setPositionNormalization(
    tmc9660::tmcl::VelocityPiNorm pNorm, tmc9660::tmcl::VelocityPiNorm iNorm) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::POSITION_NORM_P,
                              static_cast<uint32_t>(pNorm));
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::POSITION_NORM_I,
                              static_cast<uint32_t>(iNorm));
  return ok;
}

bool TMC9660::PositionControl::getPositionPiIntegrator(int32_t& integrator) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::POSITION_PI_INTEGRATOR, v))
    return false;
  integrator = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::PositionControl::getPositionPiError(int32_t& error) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::POSITION_PI_ERROR, v))
    return false;
  error = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::PositionControl::setStopOnPositionDeviation(uint32_t maxError,
                                                          bool softStop) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::STOP_ON_POSITION_DEVIATION, maxError);
  auto setting = softStop ? tmc9660::tmcl::EventStopSettings::STOP_ON_POS_VEL_DEVIATION_SOFT_STOP
                          : tmc9660::tmcl::EventStopSettings::STOP_ON_POS_VEL_DEVIATION;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::EVENT_STOP_SETTINGS,
                              static_cast<uint32_t>(setting));
  return ok;
}

bool TMC9660::PositionControl::getStopOnPositionDeviation(uint32_t& maxError,
                                                          bool& softStop) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::STOP_ON_POSITION_DEVIATION, v))
    return false;
  maxError = v;
  uint32_t mode;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::EVENT_STOP_SETTINGS, mode))
    return false;
  softStop = (mode == static_cast<uint32_t>(
                          tmc9660::tmcl::EventStopSettings::STOP_ON_POS_VEL_DEVIATION_SOFT_STOP));
  return true;
}

bool TMC9660::PositionControl::setPositionLoopDownsampling(uint8_t divider) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::POSITION_LOOP_DOWNSAMPLING, divider);
}

bool TMC9660::PositionControl::getPositionLoopDownsampling(uint8_t& divider) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::POSITION_LOOP_DOWNSAMPLING, v))
    return false;
  divider = static_cast<uint8_t>(v);
  return true;
}

bool TMC9660::PositionControl::setPositionLimitLow(int32_t limit) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::POSITION_LIMIT_LOW,
                               static_cast<uint32_t>(limit));
}

bool TMC9660::PositionControl::getPositionLimitLow(int32_t& limit) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::POSITION_LIMIT_LOW, v))
    return false;
  limit = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::PositionControl::setPositionLimitHigh(int32_t limit) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::POSITION_LIMIT_HIGH,
                               static_cast<uint32_t>(limit));
}

bool TMC9660::PositionControl::getPositionLimitHigh(int32_t& limit) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::POSITION_LIMIT_HIGH, v))
    return false;
  limit = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::PositionControl::setPositionReachedThreshold(uint32_t threshold) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::POSITION_REACHED_THRESHOLD, threshold);
}

bool TMC9660::PositionControl::getPositionReachedThreshold(uint32_t& threshold) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::POSITION_REACHED_THRESHOLD, v))
    return false;
  threshold = v;
  return true;
}

//-------------------------------------------------------------------------
// Open‐loop support (45–47)
//-------------------------------------------------------------------------

bool TMC9660::TorqueFluxControl::getOpenloopAngle(int16_t& angle) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::OPENLOOP_ANGLE, v))
    return false;
  angle = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::setOpenloopCurrent(uint16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::OPENLOOP_CURRENT, milliamps);
}

bool TMC9660::TorqueFluxControl::getOpenloopCurrent(uint16_t& milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::OPENLOOP_CURRENT, v))
    return false;
  milliamps = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::setOpenloopVoltage(uint16_t voltage) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::OPENLOOP_VOLTAGE, voltage);
}

bool TMC9660::TorqueFluxControl::getOpenloopVoltage(uint16_t& voltage) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::OPENLOOP_VOLTAGE, v))
    return false;
  voltage = static_cast<uint16_t>(v);
  return true;
}

//-------------------------------------------------------------------------
// Ref switch & stop-event (161–170)
//-------------------------------------------------------------------------

bool TMC9660::PositionControl::setReferenceSwitchEnable(
    tmc9660::tmcl::ReferenceSwitchEnable enable) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_ENABLE,
                               static_cast<uint32_t>(enable));
}

bool TMC9660::PositionControl::getReferenceSwitchEnable(
    tmc9660::tmcl::ReferenceSwitchEnable& enable) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_ENABLE, v))
    return false;
  enable = static_cast<tmc9660::tmcl::ReferenceSwitchEnable>(v);
  return true;
}

bool TMC9660::PositionControl::setReferenceSwitchPolaritySwap(
    tmc9660::tmcl::ReferenceSwitchPolaritySwap config) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_POLARITY_AND_SWAP,
                               static_cast<uint32_t>(config));
}

bool TMC9660::PositionControl::getReferenceSwitchPolaritySwap(
    tmc9660::tmcl::ReferenceSwitchPolaritySwap& config) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_POLARITY_AND_SWAP, v))
    return false;
  config = static_cast<tmc9660::tmcl::ReferenceSwitchPolaritySwap>(v);
  return true;
}

bool TMC9660::PositionControl::setReferenceSwitchLatchSettings(
    tmc9660::tmcl::ReferenceSwitchLatchSettings setting) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_LATCH_SETTINGS,
                               static_cast<uint32_t>(setting));
}

bool TMC9660::PositionControl::getReferenceSwitchLatchSettings(
    tmc9660::tmcl::ReferenceSwitchLatchSettings& setting) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_LATCH_SETTINGS, v))
    return false;
  setting = static_cast<tmc9660::tmcl::ReferenceSwitchLatchSettings>(v);
  return true;
}

bool TMC9660::PositionControl::setEventStopSettings(
    tmc9660::tmcl::EventStopSettings settings) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::EVENT_STOP_SETTINGS,
                               static_cast<uint32_t>(settings));
}

bool TMC9660::PositionControl::getEventStopSettings(
    tmc9660::tmcl::EventStopSettings& settings) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::EVENT_STOP_SETTINGS, v))
    return false;
  settings = static_cast<tmc9660::tmcl::EventStopSettings>(v);
  return true;
}

bool TMC9660::PositionControl::setReferenceSwitchSearchMode(
    tmc9660::tmcl::ReferenceSwitchSearchMode mode) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_SEARCH_MODE,
                               static_cast<uint32_t>(mode));
}

bool TMC9660::PositionControl::getReferenceSwitchSearchMode(
    tmc9660::tmcl::ReferenceSwitchSearchMode& mode) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_SEARCH_MODE, v))
    return false;
  mode = static_cast<tmc9660::tmcl::ReferenceSwitchSearchMode>(v);
  return true;
}

bool TMC9660::PositionControl::setReferenceSwitchSearchSpeed(int32_t speed) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_SEARCH_SPEED,
                               static_cast<uint32_t>(speed));
}

bool TMC9660::PositionControl::getReferenceSwitchSearchSpeed(int32_t& speed) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_SEARCH_SPEED, v))
    return false;
  speed = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::PositionControl::setReferenceSwitchSpeed(int32_t speed) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_SPEED,
                               static_cast<uint32_t>(speed));
}

bool TMC9660::PositionControl::getReferenceSwitchSpeed(int32_t& speed) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_SPEED, v))
    return false;
  speed = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::PositionControl::getRightLimitSwitchPosition(int32_t& position) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::RIGHT_LIMIT_SWITCH_POSITION, v))
    return false;
  position = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::PositionControl::getHomeSwitchPosition(int32_t& position) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::HOME_SWITCH_POSITION, v))
    return false;
  position = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::PositionControl::getLastReferencePosition(int32_t& position) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::LAST_REFERENCE_POSITION, v))
    return false;
  position = static_cast<int32_t>(v);
  return true;
}

//-------------------------------------------------------------------------
// Additional FOC telemetry and tuning parameters (305–334)
//-------------------------------------------------------------------------

bool TMC9660::PositionControl::getMccInputsRaw(uint16_t& inputs) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::MCC_INPUTS_RAW, v))
    return false;
  inputs = static_cast<uint16_t>(v);
  return true;
}

//-------------------------------------------------------------------------
// Auto-Configuration
//-------------------------------------------------------------------------

bool TMC9660::PositionControl::configureAuto(const PositionConfig& config) noexcept {
  bool ok = true;

  // Step 1: Configure position sensor selection
  ok &= setPositionSensor(config.sensorSelection);

  // Step 2: Configure PI gains (use provided values or defaults)
  constexpr uint16_t DEFAULT_P = 2000; // Default P gain (typical for position control)
  constexpr uint16_t DEFAULT_I = 100;  // Default I gain (typical for position control)

  uint16_t positionP = config.positionP.value_or(DEFAULT_P);
  uint16_t positionI = config.positionI.value_or(DEFAULT_I);

  // Clamp to valid range [0-32767]
  if (positionP > 32767)
    positionP = 32767;
  if (positionI > 32767)
    positionI = 32767;
  if (positionI < 1)
    positionI = 1; // Minimum I gain

  ok &= setPositionLoopGains(positionP, positionI);

  // Step 3: Configure normalization
  ok &= setPositionNormalization(config.pNormalization, config.iNormalization);

  // Step 4: Configure position scaling factor
  if (config.positionScalingFactor.has_value()) {
    ok &= setPositionScalingFactor(config.positionScalingFactor.value());
  } else if (config.encoderCountsPerRev.has_value()) {
    // Auto-calculate scaling factor from encoder CPR
    // Similar to velocity scaling, use simplified calculation
    uint32_t cpr = config.encoderCountsPerRev.value();
    if (cpr > 0) {
      // Simplified: assume 1:1 scaling for now (user should provide explicit scaling if needed)
      ok &= setPositionScalingFactor(1);
    }
  }

  // Step 5: Configure position loop downsampling
  ok &= setPositionLoopDownsampling(config.loopDownsampling);

  // Step 6: Configure position limits if provided
  if (config.positionLimitLow.has_value()) {
    ok &= setPositionLimitLow(config.positionLimitLow.value());
  }
  if (config.positionLimitHigh.has_value()) {
    ok &= setPositionLimitHigh(config.positionLimitHigh.value());
  }

  // Step 7: Configure position reached threshold
  ok &= setPositionReachedThreshold(config.positionReachedThreshold);

  // Step 8: Configure stop on deviation if enabled
  if (config.stopOnDeviationMaxError.has_value()) {
    ok &= setStopOnPositionDeviation(config.stopOnDeviationMaxError.value(),
                                     config.stopOnDeviationSoftStop);
  }

  return ok;
}

//-------------------------------------------------------------------------
// Additional FOC telemetry and tuning parameters (305–334)
//-------------------------------------------------------------------------

bool TMC9660::TorqueFluxControl::getFocVoltageUx(int16_t& voltage) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_VOLTAGE_UX, v))
    return false;
  voltage = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::getFocVoltageWy(int16_t& voltage) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_VOLTAGE_WY, v))
    return false;
  voltage = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::getFocVoltageV(int16_t& voltage) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_VOLTAGE_V, v))
    return false;
  voltage = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::getFocVoltageUq(int16_t& voltage) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_VOLTAGE_UQ, v))
    return false;
  voltage = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::setFieldWeakeningI(uint16_t milliamps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::FIELDWEAKENING_I, milliamps);
}

bool TMC9660::TorqueFluxControl::getFieldWeakeningI(uint16_t& milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FIELDWEAKENING_I, v))
    return false;
  milliamps = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::setFieldWeakeningVoltageThreshold(uint16_t voltage) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::FIELDWEAKENING_VOLTAGE_THRESHOLD,
                               voltage);
}

bool TMC9660::TorqueFluxControl::getFieldWeakeningVoltageThreshold(uint16_t& voltage) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FIELDWEAKENING_VOLTAGE_THRESHOLD, v))
    return false;
  voltage = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::getFocCurrentUx(int16_t& milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_CURRENT_UX, v))
    return false;
  milliamps = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::getFocCurrentV(int16_t& milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_CURRENT_V, v))
    return false;
  milliamps = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::getFocCurrentWy(int16_t& milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_CURRENT_WY, v))
    return false;
  milliamps = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::getFocCurrentIq(int16_t& milliamps) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::FOC_CURRENT_IQ, v))
    return false;
  milliamps = static_cast<int16_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::setTargetTorqueBiquadFilterEnable(bool enable) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_ENABLE,
                               enable);
}

bool TMC9660::TorqueFluxControl::getTargetTorqueBiquadFilterEnable(bool& enable) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_ENABLE, v))
    return false;
  enable = (v != 0);
  return true;
}

bool TMC9660::TorqueFluxControl::setTargetTorqueBiquadFilterACoeff1(int32_t coeff) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_1,
                               static_cast<uint32_t>(coeff));
}

bool TMC9660::TorqueFluxControl::getTargetTorqueBiquadFilterACoeff1(int32_t& coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_1, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::setTargetTorqueBiquadFilterACoeff2(int32_t coeff) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_2,
                               static_cast<uint32_t>(coeff));
}

bool TMC9660::TorqueFluxControl::getTargetTorqueBiquadFilterACoeff2(int32_t& coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_2, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::setTargetTorqueBiquadFilterBCoeff0(int32_t coeff) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_0,
                               static_cast<uint32_t>(coeff));
}

bool TMC9660::TorqueFluxControl::getTargetTorqueBiquadFilterBCoeff0(int32_t& coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_0, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::setTargetTorqueBiquadFilterBCoeff1(int32_t coeff) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_1,
                               static_cast<uint32_t>(coeff));
}

bool TMC9660::TorqueFluxControl::getTargetTorqueBiquadFilterBCoeff1(int32_t& coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_1, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::setTargetTorqueBiquadFilterBCoeff2(int32_t coeff) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_2,
                               static_cast<uint32_t>(coeff));
}

bool TMC9660::TorqueFluxControl::getTargetTorqueBiquadFilterBCoeff2(int32_t& coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_2, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::VelocityControl::setActualVelocityBiquadFilterEnable(bool enable) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_ENABLE,
                               enable);
}

bool TMC9660::VelocityControl::getActualVelocityBiquadFilterEnable(bool& enable) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_ENABLE, v))
    return false;
  enable = (v != 0);
  return true;
}

bool TMC9660::VelocityControl::setActualVelocityBiquadFilterACoeff1(int32_t coeff) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_1,
                               static_cast<uint32_t>(coeff));
}

bool TMC9660::VelocityControl::getActualVelocityBiquadFilterACoeff1(int32_t& coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_1, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::VelocityControl::setActualVelocityBiquadFilterACoeff2(int32_t coeff) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_2,
                               static_cast<uint32_t>(coeff));
}

bool TMC9660::VelocityControl::getActualVelocityBiquadFilterACoeff2(int32_t& coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_2, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::VelocityControl::setActualVelocityBiquadFilterBCoeff0(int32_t coeff) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_0,
                               static_cast<uint32_t>(coeff));
}

bool TMC9660::VelocityControl::getActualVelocityBiquadFilterBCoeff0(int32_t& coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_0, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::VelocityControl::setActualVelocityBiquadFilterBCoeff1(int32_t coeff) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_1,
                               static_cast<uint32_t>(coeff));
}

bool TMC9660::VelocityControl::getActualVelocityBiquadFilterBCoeff1(int32_t& coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_1, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::VelocityControl::setActualVelocityBiquadFilterBCoeff2(int32_t coeff) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_2,
                               static_cast<uint32_t>(coeff));
}

bool TMC9660::VelocityControl::getActualVelocityBiquadFilterBCoeff2(int32_t& coeff) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_2, v))
    return false;
  coeff = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::TorqueFluxControl::getTorqueFluxCombinedTargetValues(uint32_t& value) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::TORQUE_FLUX_COMBINED_TARGET_VALUES, value);
}

bool TMC9660::TorqueFluxControl::getTorqueFluxCombinedActualValues(uint32_t& value) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::TORQUE_FLUX_COMBINED_ACTUAL_VALUES, value);
}

bool TMC9660::TorqueFluxControl::getVoltageDqCombinedActualValues(uint32_t& value) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::VOLTAGE_D_Q_COMBINED_ACTUAL_VALUES, value);
}

bool TMC9660::TorqueFluxControl::getIntegratedActualTorqueValue(uint32_t& value) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::INTEGRATED_ACTUAL_TORQUE_VALUE, value);
}

bool TMC9660::VelocityControl::getIntegratedActualVelocityValue(uint32_t& value) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::INTEGRATED_ACTUAL_VELOCITY_VALUE, value);
}

//***************************************************************************
//**                  SUBSYSTEM: Motion Ramp                             **//
//***************************************************************************

bool TMC9660::Ramp::enable(bool on) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_ENABLE, on ? 1u : 0u);
}

bool TMC9660::Ramp::setAcceleration(uint32_t a1, uint32_t a2, uint32_t aMax) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_A1, a1);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_A2, a2);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_AMAX, aMax);
  return ok;
}

bool TMC9660::Ramp::setDeceleration(uint32_t d1, uint32_t d2, uint32_t dMax) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_D1, d1);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_D2, d2);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_DMAX, dMax);
  return ok;
}

bool TMC9660::Ramp::setVelocities(uint32_t vStart, uint32_t vStop, uint32_t v1, uint32_t v2,
                                  uint32_t vMax) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_VSTART, vStart);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_VSTOP, vStop);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_V1, v1);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_V2, v2);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_VMAX, vMax);
  return ok;
}

bool TMC9660::Ramp::setTiming(uint16_t tVmaxCycles, uint16_t tZeroWaitCycles) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_TVMAX, tVmaxCycles);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::RAMP_TZEROWAIT, tZeroWaitCycles);
  return ok;
}

bool TMC9660::Ramp::enableFeedForward(bool enableVelFF, bool enableAccelFF, uint16_t accelFFGain,
                                      tmc9660::tmcl::AccelerationFFShift accelFFShift) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::VELOCITY_FEEDFORWARD_ENABLE,
                              enableVelFF ? 1u : 0u);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ACCELERATION_FEEDFORWARD_ENABLE,
                              enableAccelFF ? 1u : 0u);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ACCELERATION_FF_GAIN, accelFFGain);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::ACCELERATION_FF_SHIFT,
                              static_cast<uint32_t>(accelFFShift));
  return ok;
}

bool TMC9660::Ramp::setDirectVelocityMode(bool enable) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::DIRECT_VELOCITY_MODE, enable ? 1u : 0u);
}

bool TMC9660::Ramp::getRampVelocity(int32_t& velocity) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::RAMP_VELOCITY, v))
    return false;
  velocity = static_cast<int32_t>(v);
  return true;
}

bool TMC9660::Ramp::getRampPosition(int32_t& position) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::RAMP_POSITION, v))
    return false;
  position = static_cast<int32_t>(v);
  return true;
}

//-------------------------------------------------------------------------
// Auto-Configuration
//-------------------------------------------------------------------------

bool TMC9660::Ramp::configureAuto(const RampConfig& config) noexcept {
  bool ok = true;

  // Step 1: Configure acceleration segments
  constexpr uint32_t DEFAULT_AMAX = 1000;
  constexpr uint32_t DEFAULT_A1 = 8000;
  constexpr uint32_t DEFAULT_A2 = 4000;

  uint32_t aMax = config.maxAcceleration.value_or(DEFAULT_AMAX);
  uint32_t a1 = config.acceleration1.value_or(DEFAULT_A1);
  uint32_t a2 = config.acceleration2.value_or(DEFAULT_A2);

  ok &= setAcceleration(a1, a2, aMax);

  // Step 2: Configure deceleration segments
  constexpr uint32_t DEFAULT_DMAX = 1000;
  constexpr uint32_t DEFAULT_D1 = 8000;
  constexpr uint32_t DEFAULT_D2 = 8000;

  uint32_t dMax = config.maxDeceleration.value_or(DEFAULT_DMAX);
  uint32_t d1 = config.deceleration1.value_or(DEFAULT_D1);
  uint32_t d2 = config.deceleration2.value_or(DEFAULT_D2);

  ok &= setDeceleration(d1, d2, dMax);

  // Step 3: Configure velocity thresholds and limits
  constexpr uint32_t DEFAULT_V1 = 0;
  constexpr uint32_t DEFAULT_V2 = 0;
  constexpr uint32_t DEFAULT_VSTART = 0;
  constexpr uint32_t DEFAULT_VSTOP = 1;

  uint32_t v1 = config.velocityThreshold1.value_or(DEFAULT_V1);
  uint32_t v2 = config.velocityThreshold2.value_or(DEFAULT_V2);
  uint32_t vStart = config.startVelocity.value_or(DEFAULT_VSTART);
  uint32_t vStop = config.stopVelocity.value_or(DEFAULT_VSTOP);
  uint32_t vMax = config.maxVelocity; // Required parameter, no default

  ok &= setVelocities(vStart, vStop, v1, v2, vMax);

  // Step 4: Configure timing constraints
  constexpr uint16_t DEFAULT_TVMAX = 0;
  constexpr uint16_t DEFAULT_TZEROWAIT = 0;

  uint16_t tVmax = config.timeAtVmax.value_or(DEFAULT_TVMAX);
  uint16_t tZeroWait = config.timeZeroWait.value_or(DEFAULT_TZEROWAIT);

  ok &= setTiming(tVmax, tZeroWait);

  // Step 5: Configure feedforward
  constexpr bool DEFAULT_VEL_FF = false;
  constexpr bool DEFAULT_ACCEL_FF = false;
  constexpr uint16_t DEFAULT_ACCEL_FF_GAIN = 8;
  constexpr tmc9660::tmcl::AccelerationFFShift DEFAULT_ACCEL_FF_SHIFT =
      tmc9660::tmcl::AccelerationFFShift::SHIFT_4_BIT;

  bool enableVelFF = config.enableVelocityFeedForward.value_or(DEFAULT_VEL_FF);
  bool enableAccelFF = config.enableAccelerationFeedForward.value_or(DEFAULT_ACCEL_FF);
  uint16_t accelFFGain = config.accelerationFeedForwardGain.value_or(DEFAULT_ACCEL_FF_GAIN);
  tmc9660::tmcl::AccelerationFFShift accelFFShift =
      config.accelerationFeedForwardShift.value_or(DEFAULT_ACCEL_FF_SHIFT);

  ok &= enableFeedForward(enableVelFF, enableAccelFF, accelFFGain, accelFFShift);

  // Step 6: Configure direct velocity mode
  constexpr bool DEFAULT_DIRECT_VEL_MODE = true;
  bool enableDirectVel = config.enableDirectVelocityMode.value_or(DEFAULT_DIRECT_VEL_MODE);

  ok &= setDirectVelocityMode(enableDirectVel);

  // Step 7: Enable/disable ramp generator (applied last)
  if (config.enableRamp.has_value()) {
    ok &= enable(config.enableRamp.value());
  }

  return ok;
}

//***************************************************************************
//**              SUBSYSTEM: Step/Dir Input Extrapolation                **//
//***************************************************************************

bool TMC9660::StepDir::setMicrostepResolution(
    tmc9660::tmcl::StepDirStepDividerShift µSteps) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::STEP_DIR_STEP_DIVIDER_SHIFT,
                               static_cast<uint32_t>(µSteps));
}

bool TMC9660::StepDir::enableInterface(bool on) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::STEP_DIR_ENABLE, on ? 1u : 0u);
}

bool TMC9660::StepDir::enableExtrapolation(bool enable) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::STEP_DIR_EXTRAPOLATION_ENABLE,
                               enable ? 1u : 0u);
}

bool TMC9660::StepDir::setSignalTimeout(uint16_t timeout_ms) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::STEP_DIR_STEP_SIGNAL_TIMEOUT_LIMIT,
                               timeout_ms);
}

bool TMC9660::StepDir::setMaxExtrapolationVelocity(uint32_t eRPM) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::STEP_DIR_MAXIMUM_EXTRAPOLATION_VELOCITY,
                               eRPM);
}

bool TMC9660::StepDir::enableVelocityFeedForward(bool enableVelFF) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::VELOCITY_FEEDFORWARD_ENABLE,
                               enableVelFF ? 1u : 0u);
}

//***********************************************************************
//**                    SUBSYSTEM: Reference Search                   **//
//***********************************************************************

bool TMC9660::ReferenceSearch::start() noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::RFS,
                            static_cast<uint16_t>(tmc9660::tmcl::ReferenceSearchCommand::START), 0,
                            0, nullptr);
}

bool TMC9660::ReferenceSearch::stop() noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::RFS,
                            static_cast<uint16_t>(tmc9660::tmcl::ReferenceSearchCommand::STOP), 0,
                            0, nullptr);
}

bool TMC9660::ReferenceSearch::getStatus(tmc9660::tmcl::ReferenceSearchStatus& status) noexcept {
  uint32_t val = 0;
  if (!driver.sendCommand(tmc9660::tmcl::Op::RFS,
                          static_cast<uint16_t>(tmc9660::tmcl::ReferenceSearchCommand::STATUS), 0,
                          0, &val))
    return false;
  status = static_cast<tmc9660::tmcl::ReferenceSearchStatus>(val & 0xFF);
  return true;
}

//***************************************************************************
//**                      SUBSYSTEM: Brake Chopper                       **//
//***************************************************************************

bool TMC9660::Brake::enableChopper(bool enable) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::BRAKE_CHOPPER_ENABLE, enable ? 1u : 0u);
}

bool TMC9660::Brake::setVoltageLimit(float voltage) noexcept {
  uint32_t raw = static_cast<uint32_t>(voltage * 10.0f + 0.5f);
  return driver.writeParameter(tmc9660::tmcl::Parameters::BRAKE_CHOPPER_VOLTAGE_LIMIT, raw);
}

bool TMC9660::Brake::setHysteresis(float voltage) noexcept {
  uint32_t raw = static_cast<uint32_t>(voltage * 10.0f + 0.5f);
  return driver.writeParameter(tmc9660::tmcl::Parameters::BRAKE_CHOPPER_HYSTERESIS, raw);
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
  return driver.writeParameter(tmc9660::tmcl::Parameters::BRAKE_RELEASING_DUTY_CYCLE, percent);
}

bool TMC9660::Brake::setHoldingDutyCycle(uint8_t percent) noexcept {
  if (percent > 99)
    percent = 99;
  return driver.writeParameter(tmc9660::tmcl::Parameters::BRAKE_HOLDING_DUTY_CYCLE, percent);
}

bool TMC9660::Brake::setReleasingDuration(uint16_t milliseconds) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::BRAKE_RELEASING_DURATION, milliseconds);
}

bool TMC9660::Brake::invertOutput(bool invert) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::INVERT_BRAKE_OUTPUT, invert ? 1u : 0u);
}

//-------------------------------------------------------------------------
// Auto-Configuration
//-------------------------------------------------------------------------

bool TMC9660::Brake::configureAuto(const BrakeConfig& config) noexcept {
  bool ok = true;

  // Step 1: Configure brake chopper (if enabled)
  if (config.enableChopper.has_value() && config.enableChopper.value()) {
    ok &= enableChopper(true);

    constexpr float DEFAULT_CHOPPER_VOLTAGE_V = 30.0f; // Default for 24V systems
    constexpr float DEFAULT_CHOPPER_HYSTERESIS_V = 2.0f;

    float chopperVoltage = config.chopperVoltageThreshold_V.value_or(DEFAULT_CHOPPER_VOLTAGE_V);
    float chopperHyst = config.chopperHysteresis_V.value_or(DEFAULT_CHOPPER_HYSTERESIS_V);

    // Clamp voltage to valid range [5.0-100.0V]
    if (chopperVoltage < 5.0f)
      chopperVoltage = 5.0f;
    if (chopperVoltage > 100.0f)
      chopperVoltage = 100.0f;

    // Clamp hysteresis to valid range [0.0-5.0V]
    if (chopperHyst < 0.0f)
      chopperHyst = 0.0f;
    if (chopperHyst > 5.0f)
      chopperHyst = 5.0f;

    ok &= setVoltageLimit(chopperVoltage);
    ok &= setHysteresis(chopperHyst);
  } else if (config.enableChopper.has_value() && !config.enableChopper.value()) {
    ok &= enableChopper(false);
  }
  // If enableChopper is not provided, leave chopper in current state

  // Step 2: Configure mechanical brake parameters (if provided)
  constexpr uint8_t DEFAULT_RELEASING_DUTY = 50;
  constexpr uint8_t DEFAULT_HOLDING_DUTY = 30;
  constexpr uint16_t DEFAULT_RELEASING_DURATION_MS = 100;
  constexpr bool DEFAULT_INVERT_OUTPUT = false;

  if (config.releasingDutyCycle.has_value()) {
    uint8_t duty = config.releasingDutyCycle.value();
    if (duty > 99)
      duty = 99; // Clamp to valid range
    ok &= setReleasingDutyCycle(duty);
  }

  if (config.holdingDutyCycle.has_value()) {
    uint8_t duty = config.holdingDutyCycle.value();
    if (duty > 99)
      duty = 99; // Clamp to valid range
    ok &= setHoldingDutyCycle(duty);
  }

  if (config.releasingDuration_ms.has_value()) {
    ok &= setReleasingDuration(config.releasingDuration_ms.value());
  }

  if (config.invertOutput.has_value()) {
    ok &= invertOutput(config.invertOutput.value());
  }

  return ok;
}

//***************************************************************************
//**                   SUBSYSTEM: I²t Overload Protection                **//
//***************************************************************************

bool TMC9660::IIT::configure(uint16_t timeConstant1_ms, float continuousCurrent1_A,
                             uint16_t timeConstant2_ms, float continuousCurrent2_A) noexcept {
  bool ok = true;
  ok &= setThermalWindingTimeConstant1(timeConstant1_ms);
  const uint32_t limit1 =
      static_cast<uint32_t>(continuousCurrent1_A * continuousCurrent1_A * timeConstant1_ms);
  ok &= setLimit1(limit1);
  ok &= setThermalWindingTimeConstant2(timeConstant2_ms);
  const uint32_t limit2 =
      static_cast<uint32_t>(continuousCurrent2_A * continuousCurrent2_A * timeConstant2_ms);
  ok &= setLimit2(limit2);
  return ok;
}

bool TMC9660::IIT::resetIntegralState() noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::RESET_IIT_SUMS, 0, 0);
}

bool TMC9660::IIT::setThermalWindingTimeConstant1(uint16_t ms) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::THERMAL_WINDING_TIME_CONSTANT_1, 0, ms);
}

bool TMC9660::IIT::getThermalWindingTimeConstant1(uint16_t& ms) noexcept {
  uint32_t v = 0;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::THERMAL_WINDING_TIME_CONSTANT_1, v))
    return false;
  ms = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::IIT::setLimit1(uint32_t limit) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::IIT_LIMIT_1, 0, limit);
}

bool TMC9660::IIT::getLimit1(uint32_t& limit) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::IIT_LIMIT_1, 0, limit);
}

bool TMC9660::IIT::setThermalWindingTimeConstant2(uint16_t ms) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::THERMAL_WINDING_TIME_CONSTANT_2, 0, ms);
}

bool TMC9660::IIT::getThermalWindingTimeConstant2(uint16_t& ms) noexcept {
  uint32_t v = 0;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::THERMAL_WINDING_TIME_CONSTANT_2, v))
    return false;
  ms = static_cast<uint16_t>(v);
  return true;
}

bool TMC9660::IIT::setLimit2(uint32_t limit) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::IIT_LIMIT_2, 0, limit);
}

bool TMC9660::IIT::getLimit2(uint32_t& limit) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::IIT_LIMIT_2, 0, limit);
}

bool TMC9660::IIT::getActualTotalMotorCurrent(uint32_t& current, uint8_t motorIndex) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_TOTAL_MOTOR_CURRENT, current,
                              motorIndex);
}

bool TMC9660::IIT::getSum1(uint32_t& sum) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::IIT_SUM_1, 0, sum);
}

bool TMC9660::IIT::getSum2(uint32_t& sum) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::IIT_SUM_2, 0, sum);
}

//===========================================================================
//==                  SUBSYSTEM: Telemetry & Status                       ==//
//===========================================================================

TMC9660::Telemetry::Telemetry(TMC9660& parent) noexcept : driver(parent) {}

bool TMC9660::Telemetry::getGeneralStatusFlags(uint32_t& flags) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::GENERAL_STATUS_FLAGS, flags);
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
  if (!driver.readParameter(tmc9660::tmcl::Parameters::ACTUAL_TOTAL_MOTOR_CURRENT, v))
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

bool TMC9660::Telemetry::getGeneralErrorFlags(uint32_t& flags) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::GENERAL_ERROR_FLAGS, flags);
}

bool TMC9660::Telemetry::getGateDriverErrorFlags(uint32_t& flags) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::GDRV_ERROR_FLAGS, flags);
}

bool TMC9660::Telemetry::clearGeneralErrorFlags(uint32_t mask) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::GENERAL_ERROR_FLAGS, mask);
}

bool TMC9660::Telemetry::clearGateDriverErrorFlags(uint32_t mask) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::GDRV_ERROR_FLAGS, mask);
}

bool TMC9660::Telemetry::getADCStatusFlags(uint32_t& flags) noexcept {
  return driver.readParameter(tmc9660::tmcl::Parameters::ADC_STATUS_FLAGS, flags);
}

bool TMC9660::Telemetry::clearADCStatusFlags(uint32_t mask) noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::ADC_STATUS_FLAGS, mask);
}

uint16_t TMC9660::Telemetry::getExternalTemperature() noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::EXTERNAL_TEMPERATURE, v))
    return 0;
  return static_cast<uint16_t>(v);
}

//***************************************************************************
//**                  SUBSYSTEM: Stop / Event                            **//
//***************************************************************************

bool TMC9660::StopEvents::enableDeviationStop(uint32_t maxVelError, uint32_t maxPosError,
                                              bool softStop) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::STOP_ON_VELOCITY_DEVIATION, maxVelError);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::STOP_ON_POSITION_DEVIATION, maxPosError);

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
    setting = softStop ? EventStopSettings::DO_SOFT_STOP : EventStopSettings::DO_HARD_STOP;

  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::EVENT_STOP_SETTINGS,
                              static_cast<uint32_t>(setting));
  return ok;
}

bool TMC9660::StopEvents::configureReferenceSwitches(uint8_t mask, bool invertL, bool invertR,
                                                     bool invertH, bool swapLR) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_ENABLE, mask);
  uint8_t cfg =
      (invertL ? 1u : 0u) | (invertR ? 2u : 0u) | (invertH ? 4u : 0u) | (swapLR ? 8u : 0u);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::REFERENCE_SWITCH_POLARITY_AND_SWAP, cfg);
  return ok;
}

bool TMC9660::StopEvents::getAndClearLatchedPosition(int32_t& pos) noexcept {
  uint32_t v;
  if (!driver.readParameter(tmc9660::tmcl::Parameters::LATCH_POSITION, v))
    return false;
  pos = static_cast<int32_t>(v);
  return driver.writeParameter(
      tmc9660::tmcl::Parameters::GENERAL_STATUS_FLAGS,
      static_cast<uint32_t>(tmc9660::tmcl::GeneralStatusFlags::RAMPER_LATCHED));
}

//-------------------------------------------------------------------------
// Auto-Configuration
//-------------------------------------------------------------------------

bool TMC9660::StopEvents::configureAuto(const StopEventsConfig& config) noexcept {
  bool ok = true;

  // Step 1: Configure deviation stop (if enabled)
  if (config.maxVelocityDeviation.has_value() || config.maxPositionDeviation.has_value()) {
    constexpr bool DEFAULT_SOFT_STOP = true;
    uint32_t maxVelError = config.maxVelocityDeviation.value_or(0);
    uint32_t maxPosError = config.maxPositionDeviation.value_or(0);
    bool softStop = config.deviationSoftStop.value_or(DEFAULT_SOFT_STOP);

    ok &= enableDeviationStop(maxVelError, maxPosError, softStop);
  }

  // Step 2: Configure reference switches (if enabled)
  if (config.referenceSwitchMask.has_value()) {
    constexpr bool DEFAULT_INVERT_L = false;
    constexpr bool DEFAULT_INVERT_R = false;
    constexpr bool DEFAULT_INVERT_H = false;
    constexpr bool DEFAULT_SWAP_LR = false;

    uint8_t mask = config.referenceSwitchMask.value();
    // Clamp mask to valid range [0-7]
    if (mask > 7)
      mask = 7;

    bool invertL = config.invertLeftSwitch.value_or(DEFAULT_INVERT_L);
    bool invertR = config.invertRightSwitch.value_or(DEFAULT_INVERT_R);
    bool invertH = config.invertHomeSwitch.value_or(DEFAULT_INVERT_H);
    bool swapLR = config.swapLeftRight.value_or(DEFAULT_SWAP_LR);

    ok &= configureReferenceSwitches(mask, invertL, invertR, invertH, swapLR);
  }

  return ok;
}

//===========================================================================
//==                  SUBSYSTEM: Protection                               ==//
//===========================================================================

bool TMC9660::Protection::configureVoltage(uint16_t overVoltThreshold,
                                           uint16_t underVoltThreshold) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SUPPLY_OVERVOLTAGE_WARNING_THRESHOLD, 0,
                              overVoltThreshold);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::SUPPLY_UNDERVOLTAGE_WARNING_THRESHOLD, 0,
                              underVoltThreshold);
  return ok;
}

bool TMC9660::Protection::configureTemperature(float warningDegC, float shutdownDegC) noexcept {
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
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::CHIP_TEMPERATURE_WARNING_THRESHOLD, 0,
                              warnRaw);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::CHIP_TEMPERATURE_SHUTDOWN_THRESHOLD, 0,
                              shutRaw);
  return ok;
}

bool TMC9660::Protection::setOvercurrentEnabled(bool enabled) noexcept {
  tmc9660::tmcl::OvercurrentEnable temp = enabled ? tmc9660::tmcl::OvercurrentEnable::ENABLED
                                                  : tmc9660::tmcl::OvercurrentEnable::DISABLED;
  return driver.gateDriver.enableOvercurrentProtection(temp, temp, temp, temp);
}

bool TMC9660::Protection::configureI2t(uint16_t timeConstant1_ms, float continuousCurrent1_A,
                                       uint16_t timeConstant2_ms,
                                       float continuousCurrent2_A) noexcept {
  bool ok = true;
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::THERMAL_WINDING_TIME_CONSTANT_1,
                              timeConstant1_ms);
  uint32_t limit1 =
      static_cast<uint32_t>(continuousCurrent1_A * continuousCurrent1_A * timeConstant1_ms);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::IIT_LIMIT_1, limit1);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::THERMAL_WINDING_TIME_CONSTANT_2,
                              timeConstant2_ms);
  uint32_t limit2 =
      static_cast<uint32_t>(continuousCurrent2_A * continuousCurrent2_A * timeConstant2_ms);
  ok &= driver.writeParameter(tmc9660::tmcl::Parameters::IIT_LIMIT_2, limit2);
  return ok;
}

bool TMC9660::Protection::resetI2tState() noexcept {
  return driver.writeParameter(tmc9660::tmcl::Parameters::RESET_IIT_SUMS, 1u);
}

//-------------------------------------------------------------------------
// Auto-Configuration
//-------------------------------------------------------------------------

bool TMC9660::Protection::configureAuto(const ProtectionConfig& config) noexcept {
  bool ok = true;

  // Step 1: Configure voltage protection
  constexpr float DEFAULT_OVERVOLTAGE_V = 28.0f;  // Default for 24V systems
  constexpr float DEFAULT_UNDERVOLTAGE_V = 20.0f; // Default for 24V systems

  float overVolt = config.overvoltageThreshold_V.value_or(DEFAULT_OVERVOLTAGE_V);
  float underVolt = config.undervoltageThreshold_V.value_or(DEFAULT_UNDERVOLTAGE_V);

  // Convert volts to 0.1V units (as required by configureVoltage)
  uint16_t overVoltThreshold = static_cast<uint16_t>(overVolt * 10.0f + 0.5f);
  uint16_t underVoltThreshold = static_cast<uint16_t>(underVolt * 10.0f + 0.5f);

  ok &= configureVoltage(overVoltThreshold, underVoltThreshold);

  // Step 2: Configure temperature protection
  constexpr float DEFAULT_TEMP_WARNING_C = 80.0f;
  constexpr float DEFAULT_TEMP_SHUTDOWN_C = 100.0f;

  float tempWarning = config.temperatureWarning_C.value_or(DEFAULT_TEMP_WARNING_C);
  float tempShutdown = config.temperatureShutdown_C.value_or(DEFAULT_TEMP_SHUTDOWN_C);

  ok &= configureTemperature(tempWarning, tempShutdown);

  // Step 3: Configure overcurrent protection
  constexpr bool DEFAULT_OVERCURRENT_ENABLED = true;
  bool enableOC = config.enableOvercurrent.value_or(DEFAULT_OVERCURRENT_ENABLED);

  ok &= setOvercurrentEnabled(enableOC);

  // Step 4: Configure I²t thermal protection
  constexpr uint16_t DEFAULT_I2T_TIME1_MS = 100;
  constexpr float DEFAULT_I2T_CURRENT1_A = 1.5f;
  constexpr uint16_t DEFAULT_I2T_TIME2_MS = 1000;
  constexpr float DEFAULT_I2T_CURRENT2_A = 1.25f;

  uint16_t time1 = config.i2tTimeConstant1_ms.value_or(DEFAULT_I2T_TIME1_MS);
  float current1 = config.i2tContinuousCurrent1_A.value_or(DEFAULT_I2T_CURRENT1_A);
  uint16_t time2 = config.i2tTimeConstant2_ms.value_or(DEFAULT_I2T_TIME2_MS);
  float current2 = config.i2tContinuousCurrent2_A.value_or(DEFAULT_I2T_CURRENT2_A);

  ok &= configureI2t(time1, current1, time2, current2);

  return ok;
}

//===========================================================================
//==                  SUBSYSTEM: Script                                   ==//
//===========================================================================

bool TMC9660::Script::upload(const std::vector<uint32_t>& scriptData) noexcept {
  if (!driver.sendCommand(tmc9660::tmcl::Op::DownloadStart, 0, 0, 0, nullptr))
    return false;

  // During download mode, scriptData contains packed TMCL operations
  // Each uint32_t represents one TMCL operation: [opcode, type, motor, value]
  // These operations will be stored to script memory instead of executed
  for (uint32_t packedOp : scriptData) {
    // Extract TMCL operation components from packed data
    uint8_t opcode = (packedOp >> 24) & 0xFF;
    uint8_t type = (packedOp >> 16) & 0xFF;
    uint8_t motor = (packedOp >> 8) & 0xFF;
    uint8_t value = packedOp & 0xFF;

    // Send the TMCL operation - it will be stored to script memory during download mode
    if (!driver.sendCommand(static_cast<tmc9660::tmcl::Op>(opcode), type, motor, value, nullptr))
      return false;
  }

  return driver.sendCommand(tmc9660::tmcl::Op::DownloadEnd, 0, 0, 0, nullptr);
}

bool TMC9660::Script::start(uint16_t address) noexcept {
  uint16_t type = (address == 0) ? 0 : 1;
  return driver.sendCommand(tmc9660::tmcl::Op::ApplRun, type, 0, address, nullptr);
}

bool TMC9660::Script::stop() noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::ApplStop, 0, 0, 0, nullptr);
}

bool TMC9660::Script::step() noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::ApplStep, 0, 0, 0, nullptr);
}

bool TMC9660::Script::reset() noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::ApplReset, 0, 0, 0, nullptr);
}

bool TMC9660::Script::getStatus(uint32_t& status) noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::GetStatusScript, 0, 0, 0, &status);
}

bool TMC9660::Script::readMemory(uint16_t address, uint32_t& value) noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::ReadMem, 0, 0, address, &value);
}

bool TMC9660::Script::addBreakpoint(uint16_t address) noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::Breakpoint, 0, 0, address, nullptr);
}

bool TMC9660::Script::removeBreakpoint(uint16_t address) noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::Breakpoint, 1, 0, address, nullptr);
}

bool TMC9660::Script::clearBreakpoints() noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::Breakpoint, 2, 0, 0, nullptr);
}

bool TMC9660::Script::getMaxBreakpointCount(uint32_t& count) noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::Breakpoint, 3, 0, 0, &count);
}

//***************************************************************************
//**                  SUBSYSTEM: RamDebug                                **//
//***************************************************************************

bool TMC9660::RamDebug::init(uint32_t sampleCount) noexcept {
  bool ok = true;
  // Initialize/reset RAM debug
  ok &= driver.sendCommand(tmc9660::tmcl::Op::RamDebug, 0, 0, 0, nullptr);
  // Set number of samples to capture
  ok &= driver.sendCommand(tmc9660::tmcl::Op::RamDebug, 1, 0, sampleCount, nullptr);
  return ok;
}

bool TMC9660::RamDebug::startCapture() noexcept {
  return driver.sendCommand(tmc9660::tmcl::Op::RamDebug, 6, 0, 0, nullptr);
}

bool TMC9660::RamDebug::readData(uint32_t index, uint32_t& data) noexcept {
  // Use ReadMem command to read memory at given index (assuming index is an
  // address in the debug buffer)
  return driver.sendCommand(tmc9660::tmcl::Op::ReadMem, 0, 0, index, &data);
}

bool TMC9660::RamDebug::getStatus(bool& isRunning) noexcept {
  uint32_t state = 0;
  if (!driver.sendCommand(tmc9660::tmcl::Op::RamDebug, 8, 0, 0, &state)) {
    return false;
  }
  // Interpret state: assume 0 = Idle, non-zero = Running
  isRunning = (state != 0);
  return true;
}

//===========================================================================
//**                SUBSYSTEM: FLASH STORAGE                             ==//
//===========================================================================
//-------------------------------------------------------------------------
// NvmStorage helpers
//-------------------------------------------------------------------------

bool TMC9660::NvmStorage::storeToFlash() noexcept {
  // Use STAP command with fixed fields as documented in the TMCL manual
  return driver.sendCommand(tmc9660::tmcl::Op::STAP, 0x0FFF, 0x0F, 0xFFFFFFFF, nullptr);
}

bool TMC9660::NvmStorage::recallFromFlash() noexcept {
  // Trigger a configuration reload from external memory using FactoryDefault
  if (!driver.sendCommand(tmc9660::tmcl::Op::FactoryDefault, 0, 0, 0, nullptr))
    return false;
  // Give the controller some time to process and then check status flag
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(50ms);
  uint32_t flags = 0;
  if (!driver.telemetry.getGeneralStatusFlags(flags))
    return false;
  constexpr uint32_t CONFIG_LOADED_MASK =
      static_cast<uint32_t>(tmc9660::tmcl::GeneralStatusFlags::CONFIG_LOADED);
  return (flags & CONFIG_LOADED_MASK) != 0;
}

bool TMC9660::NvmStorage::eraseFlashBank(uint8_t n) noexcept {
  // Erase specified flash bank via FactoryDefault with type field as bank index
  return driver.sendCommand(tmc9660::tmcl::Op::FactoryDefault, n, 0, 0, nullptr);
}

//===========================================================================
//==                SUBSYSTEM: Heartbeat (Watchdog)                       ==//
//===========================================================================

bool TMC9660::Heartbeat::configure(tmc9660::tmcl::HeartbeatMonitoringConfig mode,
                                   uint32_t timeout_ms) noexcept {
  using tmc9660::tmcl::HeartbeatMonitoringConfig;
  HeartbeatMonitoringConfig cfg = mode;
  return driver.globals.configureHeartbeat(cfg, timeout_ms);
}

//-------------------------------------------------------------------------
// Auto-Configuration
//-------------------------------------------------------------------------

bool TMC9660::Heartbeat::configureAuto(const HeartbeatConfig& config) noexcept {
  constexpr bool DEFAULT_ENABLE = false;
  constexpr uint32_t DEFAULT_TIMEOUT_MS = 1000;

  bool enable = config.enable.value_or(DEFAULT_ENABLE);
  uint32_t timeout = config.timeout_ms.value_or(DEFAULT_TIMEOUT_MS);

  // HeartbeatMonitoringConfig: DISABLED, TMCL_UART_INTERFACE, SPI_INTERFACE
  // For simple enable/disable, use TMCL_UART_INTERFACE when enabled (most common)
  tmc9660::tmcl::HeartbeatMonitoringConfig mode =
      enable ? tmc9660::tmcl::HeartbeatMonitoringConfig::TMCL_UART_INTERFACE
             : tmc9660::tmcl::HeartbeatMonitoringConfig::DISABLED;

  return configure(mode, timeout);
}

bool TMC9660::Power::enableWakePin(bool enable) noexcept {
  return driver.globals.writeBank0(tmc9660::tmcl::GlobalParamBank0::WAKE_PIN_CONTROL_ENABLE,
                                   enable ? 1u : 0u);
}

bool TMC9660::Power::enterPowerDown(tmc9660::tmcl::PowerDownTimeout period) noexcept {
  return driver.globals.writeBank0(tmc9660::tmcl::GlobalParamBank0::GO_TO_TIMEOUT_POWER_DOWN_STATE,
                                   static_cast<uint32_t>(period));
}

//-------------------------------------------------------------------------
// Auto-Configuration
//-------------------------------------------------------------------------

bool TMC9660::Power::configureAuto(const PowerConfig& config) noexcept {
  bool ok = true;

  // Step 1: Configure wake-up pin (if specified)
  if (config.enableWakePin.has_value()) {
    ok &= enableWakePin(config.enableWakePin.value());
  }

  // Step 2: Configure power-down timeout (if specified)
  if (config.powerDownTimeout.has_value()) {
    ok &= enterPowerDown(config.powerDownTimeout.value());
  }

  return ok;
}

//***************************************************************************
//**                   SUBSYSTEM: Global Parameter Access                **//
//***************************************************************************

bool TMC9660::Globals::writeBank0(tmc9660::tmcl::GlobalParamBank0 param, uint32_t value) noexcept {
  return driver.writeGlobalParameter(param, 0, value);
}

bool TMC9660::Globals::readBank0(tmc9660::tmcl::GlobalParamBank0 param, uint32_t& value) noexcept {
  return driver.readGlobalParameter(param, 0, value);
}

bool TMC9660::Globals::writeBank2(tmc9660::tmcl::GlobalParamBank2 param, int32_t value) noexcept {
  return driver.writeGlobalParameter(param, 2, static_cast<uint32_t>(value));
}

bool TMC9660::Globals::readBank2(tmc9660::tmcl::GlobalParamBank2 param, int32_t& value) noexcept {
  uint32_t tmp;
  if (!driver.readGlobalParameter(param, 2, tmp))
    return false;
  value = static_cast<int32_t>(tmp);
  return true;
}

bool TMC9660::Globals::writeBank3(tmc9660::tmcl::GlobalParamBank3 param, uint32_t value) noexcept {
  return driver.writeGlobalParameter(param, 3, value);
}

bool TMC9660::Globals::readBank3(tmc9660::tmcl::GlobalParamBank3 param, uint32_t& value) noexcept {
  return driver.readGlobalParameter(param, 3, value);
}

//-------------------------------------------------------------------------
// High level global parameter helpers
//-------------------------------------------------------------------------

bool TMC9660::Globals::setSerialAddress(uint8_t address) noexcept {
  return writeBank0(tmc9660::tmcl::GlobalParamBank0::SERIAL_ADDRESS, address);
}

bool TMC9660::Globals::getSerialAddress(uint8_t& address) noexcept {
  uint32_t tmp;
  if (!readBank0(tmc9660::tmcl::GlobalParamBank0::SERIAL_ADDRESS, tmp))
    return false;
  address = static_cast<uint8_t>(tmp);
  return true;
}

bool TMC9660::Globals::setHostAddress(uint8_t address) noexcept {
  return writeBank0(tmc9660::tmcl::GlobalParamBank0::SERIAL_HOST_ADDRESS, address);
}

bool TMC9660::Globals::getHostAddress(uint8_t& address) noexcept {
  uint32_t tmp;
  if (!readBank0(tmc9660::tmcl::GlobalParamBank0::SERIAL_HOST_ADDRESS, tmp))
    return false;
  address = static_cast<uint8_t>(tmp);
  return true;
}

bool TMC9660::Globals::configureHeartbeat(tmc9660::tmcl::HeartbeatMonitoringConfig iface,
                                          uint32_t timeout_ms) noexcept {
  bool ok = writeBank0(tmc9660::tmcl::GlobalParamBank0::HEARTBEAT_MONITORING_CONFIG,
                       static_cast<uint32_t>(iface));
  ok &= writeBank0(tmc9660::tmcl::GlobalParamBank0::HEARTBEAT_MONITORING_TIMEOUT, timeout_ms);
  return ok;
}

bool TMC9660::Globals::getHeartbeat(tmc9660::tmcl::HeartbeatMonitoringConfig& iface,
                                    uint32_t& timeout_ms) noexcept {
  uint32_t cfg;
  bool ok = readBank0(tmc9660::tmcl::GlobalParamBank0::HEARTBEAT_MONITORING_CONFIG, cfg);
  iface = static_cast<tmc9660::tmcl::HeartbeatMonitoringConfig>(cfg);
  ok &= readBank0(tmc9660::tmcl::GlobalParamBank0::HEARTBEAT_MONITORING_TIMEOUT, timeout_ms);
  return ok;
}

bool TMC9660::Globals::setIODirectionMask(uint32_t mask) noexcept {
  return writeBank0(tmc9660::tmcl::GlobalParamBank0::IO_DIRECTION_MASK, mask);
}

bool TMC9660::Globals::getIODirectionMask(uint32_t& mask) noexcept {
  return readBank0(tmc9660::tmcl::GlobalParamBank0::IO_DIRECTION_MASK, mask);
}

bool TMC9660::Globals::setPullEnableMask(uint32_t mask) noexcept {
  return writeBank0(tmc9660::tmcl::GlobalParamBank0::IO_INPUT_PULLUP_PULLDOWN_ENABLE_MASK, mask);
}

bool TMC9660::Globals::getPullEnableMask(uint32_t& mask) noexcept {
  return readBank0(tmc9660::tmcl::GlobalParamBank0::IO_INPUT_PULLUP_PULLDOWN_ENABLE_MASK, mask);
}

bool TMC9660::Globals::setPullDirectionMask(uint32_t mask) noexcept {
  return writeBank0(tmc9660::tmcl::GlobalParamBank0::IO_INPUT_PULLUP_PULLDOWN_DIRECTION_MASK, mask);
}

bool TMC9660::Globals::getPullDirectionMask(uint32_t& mask) noexcept {
  return readBank0(tmc9660::tmcl::GlobalParamBank0::IO_INPUT_PULLUP_PULLDOWN_DIRECTION_MASK, mask);
}

bool TMC9660::Globals::setAutoStart(bool enable) noexcept {
  return writeBank0(tmc9660::tmcl::GlobalParamBank0::AUTO_START_ENABLE, enable ? 1u : 0u);
}

bool TMC9660::Globals::getAutoStart(bool& enable) noexcept {
  uint32_t tmp;
  if (!readBank0(tmc9660::tmcl::GlobalParamBank0::AUTO_START_ENABLE, tmp))
    return false;
  enable = (tmp != 0);
  return true;
}

bool TMC9660::Globals::setClearUserVariables(bool clear) noexcept {
  return writeBank0(tmc9660::tmcl::GlobalParamBank0::CLEAR_USER_VARIABLES, clear ? 1u : 0u);
}

bool TMC9660::Globals::getClearUserVariables(bool& clear) noexcept {
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

bool TMC9660::Globals::getUserVariable(uint8_t index, int32_t& value) noexcept {
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

bool TMC9660::Globals::getTimerPeriod(uint8_t timer, uint32_t& period_ms) noexcept {
  if (timer > 2)
    return false;
  auto param = static_cast<tmc9660::tmcl::GlobalParamBank3>(timer);
  return readBank3(param, period_ms);
}

bool TMC9660::Globals::setInputTrigger(uint8_t index,
                                       tmc9660::tmcl::TriggerTransition transition) noexcept {
  if (index > 18)
    return false;
  auto param = static_cast<tmc9660::tmcl::GlobalParamBank3>(
      static_cast<uint16_t>(tmc9660::tmcl::GlobalParamBank3::INPUT_0_TRIGGER_TRANSITION) + index);
  return writeBank3(param, static_cast<uint32_t>(transition));
}

bool TMC9660::Globals::getInputTrigger(uint8_t index,
                                       tmc9660::tmcl::TriggerTransition& transition) noexcept {
  if (index > 18)
    return false;
  auto param = static_cast<tmc9660::tmcl::GlobalParamBank3>(
      static_cast<uint16_t>(tmc9660::tmcl::GlobalParamBank3::INPUT_0_TRIGGER_TRANSITION) + index);
  uint32_t tmp;
  if (!readBank3(param, tmp))
    return false;
  transition = static_cast<tmc9660::tmcl::TriggerTransition>(tmp);
  return true;
}

//***************************************************************************
//**        SUBSYSTEM: General-purpose GPIO (Digital/Analog I/O)          **//
//***************************************************************************

bool TMC9660::GPIO::setMode(uint8_t pin, bool output, bool pullEnable, bool pullUp) noexcept {
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
  return driver.sendCommand(tmc9660::tmcl::Op::SIO, pin, 0, value ? 1u : 0u, nullptr);
}

bool TMC9660::GPIO::readDigital(uint8_t pin, bool& value) noexcept {
  if (pin > 18)
    return false;
  uint32_t v;
  if (!driver.sendCommand(tmc9660::tmcl::Op::GIO, pin, 0, 0, &v))
    return false;
  value = (v != 0);
  return true;
}

bool TMC9660::GPIO::readAnalog(uint8_t pin, uint16_t& value) noexcept {
  if (pin > 18)
    return false;
  uint32_t v;
  if (!driver.sendCommand(tmc9660::tmcl::Op::GIO, pin, 1, 0, &v))
    return false;
  value = static_cast<uint16_t>(v);
  return true;
}

//***************************************************************************
//**        SUBSYSTEM: PRIVATE MEMBERS                                    **//
//***************************************************************************

//==================================================
// PRIVATE MEMBERS
//==================================================

// TMC9660.cpp - Implementation of TMC9660 motor controller interface
