#pragma once
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "Micros64.h"
#include "BallButlerConfig.h"

// Forward declaration for StateMachine
class StateMachine;

/**
 * CanInterface Teensy 4.0 (CAN1) helper for:
 *  ODrive CAN: set_input_pos / set_input_vel / limits / modes / gains / etc.
 *  Time Sync (ID 0x7DD)  shared wall clock
 *  Estimator ingest (cmd=0x09) and Iq feedback (cmd=0x14)
 *  Homing routine for the hand axis (end-stop current spike)
 *  Arbitrary parameter read/write via SDO (RxSdo/TxSdo)
 *  Ball Butler heartbeat publishing (ID 0x7D1)
 *
 * Notes
 *  - Uses CAN1.
 *  - vel_ff / torque_ff are scaled internally by 100.0 int16.
 */

// CAN IDs live in BallButlerConfig.h (namespace CanIds)

class CanInterface {
public:
  // ============================================================================
  // Constants (values from BallButlerConfig.h)
  // ============================================================================
  static constexpr uint8_t MAX_NODES = CanCfg::MAX_NODES;
  static constexpr uint32_t DEFAULT_HEARTBEAT_RATE_MS = CanCfg::HEARTBEAT_RATE_MS;

  // SDO Opcodes (canonical values in protocol_config.h)
  static constexpr uint8_t OPCODE_READ  = SDO::OPCODE_READ;
  static constexpr uint8_t OPCODE_WRITE = SDO::OPCODE_WRITE;

  // ============================================================================
  // Types and Enums
  // ============================================================================

  // Error codes — values derived from YAML-generated BallButlerErrorCode constants
  enum class BallButlerError : uint8_t {
    NONE          = BallButlerErrorCode::NONE,
    RELOAD_FAILED = BallButlerErrorCode::RELOAD_FAILED,
  };

  // ODrive command IDs — values derived from YAML-generated ODriveCmd constants
  enum class Cmd : uint8_t {
    heartbeat_message        = ODriveCmd::heartbeat_message,
    get_error                = ODriveCmd::get_error,
    RxSdo                    = ODriveCmd::RxSdo,
    TxSdo                    = ODriveCmd::TxSdo,
    set_requested_state      = ODriveCmd::set_requested_state,
    get_encoder_estimate     = ODriveCmd::get_encoder_estimate,
    set_controller_mode      = ODriveCmd::set_controller_mode,
    set_input_pos            = ODriveCmd::set_input_pos,
    set_input_vel            = ODriveCmd::set_input_vel,
    set_vel_curr_limits      = ODriveCmd::set_vel_curr_limits,
    set_traj_vel_limit       = ODriveCmd::set_traj_vel_limit,
    set_traj_acc_limits      = ODriveCmd::set_traj_acc_limits,
    get_iq                   = ODriveCmd::get_iq,
    get_temps                = ODriveCmd::get_temps,
    reboot_odrives           = ODriveCmd::reboot_odrives,
    get_bus_voltage_current  = ODriveCmd::get_bus_voltage_current,
    clear_errors             = ODriveCmd::clear_errors,
    set_absolute_position    = ODriveCmd::set_absolute_position,
    set_pos_gain             = ODriveCmd::set_pos_gain,
    set_vel_gains            = ODriveCmd::set_vel_gains,
  };

  enum class AxisHomeState : uint8_t {
    Unhomed = 0,
    Homing  = 1,
    Homed   = 2
  };

  // Non-blocking homing result (returned by updateHomeHand())
  enum class HomingStatus : uint8_t {
    IN_PROGRESS = 0,  // Still running, call again next loop
    DONE        = 1,  // Homing completed successfully
    FAILED      = 2,  // Homing failed (timeout, axis error, send failure)
  };

  struct CmdName { 
    Cmd cmd; 
    const char* name; 
  };

  // ODrive error masks (canonical values in protocol_config.h, namespace ODriveErrors)
  // Access via ODriveErrors::BRAKE_RESISTOR_DISARMED directly.

  // Endpoint IDs (canonical values in protocol_config.h, namespace EndpointId)
  struct EndpointIds {
    static constexpr uint16_t COMMUTATION_MAPPER_POS_ABS = EndpointId::commutation_mapper_pos_abs;
    static constexpr uint16_t GPIO_STATES                = EndpointId::GPIO_STATES;
  };

  struct ArbitraryParamResponse {
    uint16_t endpoint_id = 0;
    uint8_t  opcode = 0;
    union {
      float    f32;
      uint32_t u32;
      int32_t  i32;
    } value;
    uint64_t wall_us = 0;
    bool     valid = false;
  };

  struct HostThrowCmd {
    float yaw_rad = 0.f;
    float pitch_rad = 0.f;
    float speed_mps = 0.f;
    uint64_t throw_wall_us = 0;  // absolute wall-clock throw time (µs)
    uint64_t wall_us = 0;        // wall time when CAN message was received
    bool valid = false;
  };

  struct SyncStats {
    float mean_us = 0.f;
    float rms_us = 0.f;
    int32_t min_us = 0;
    int32_t max_us = 0;
    uint32_t frames = 0;
  };

  struct AxisHeartbeat {
    uint32_t axis_error = 0;
    uint8_t  axis_state = 0;
    uint8_t  procedure_result = 0;
    uint8_t  trajectory_done = 0;
    uint64_t wall_us = 0;
    bool     valid = false;
  };

  // Callback types
  typedef void (*ArbitraryParamCallback)(uint32_t node_id, const ArbitraryParamResponse& resp, void* user);

  // ============================================================================
  // Construction and Setup
  // ============================================================================

  CanInterface();

  void begin(uint32_t bitrate = CanCfg::BAUD_RATE);
  void loop();

  // ============================================================================
  // Debug Control
  // ============================================================================

  void setDebugStream(Stream* dbg);
  void setDebugFlags(bool timeSyncDebug, bool canDebug);

  // ============================================================================
  // Time Sync
  // ============================================================================

  uint64_t wallTimeUs() const;
  uint64_t localTimeUs() const;
  bool hasTimeSync() const { return have_offset_; }
  SyncStats getAndClearSyncStats();

  // ============================================================================
  // Generic CAN Send
  // ============================================================================

  bool sendRaw(uint32_t id, const uint8_t* data, uint8_t len);
  bool sendRTR(uint32_t id, uint8_t len);

  static inline uint32_t makeId(uint32_t node_id, Cmd cmd) {
    return (node_id << 5) | (uint8_t(cmd) & 0x1F);
  }

  // ============================================================================
  // ODrive Commands
  // ============================================================================

  bool setRequestedState(uint32_t node_id, uint32_t requested_state);
  bool setControllerMode(uint32_t node_id, uint32_t control_mode, uint32_t input_mode);
  bool setVelCurrLimits(uint32_t node_id, float current_limit, float vel_limit_rps);
  bool setTrajVelLimit(uint32_t node_id, float vel_limit_rps);
  bool setTrajAccLimits(uint32_t node_id, float accel_rps2, float decel_rps2);
  bool setPosGain(uint32_t node_id, float kp);
  bool setVelGains(uint32_t node_id, float kv, float ki);
  bool setAbsolutePosition(uint32_t node_id, float pos_rev);
  bool clearErrors(uint32_t node_id);
  bool reboot(uint32_t node_id);

  bool sendInputPos(uint32_t node_id, float pos_rev, float vel_ff_rev_per_s, float torque_ff);
  bool sendInputVel(uint32_t node_id, float vel_rps, float torque_ff);

  // ============================================================================
  // Arbitrary Parameter (SDO) Functions
  // ============================================================================

  bool sendArbitraryParameterFloat(uint32_t node_id, uint16_t endpoint_id, float value);
  bool sendArbitraryParameterU32(uint32_t node_id, uint16_t endpoint_id, uint32_t value);
  bool requestArbitraryParameter(uint32_t node_id, uint16_t endpoint_id);
  bool getLastArbitraryParamResponse(uint32_t node_id, ArbitraryParamResponse& out) const;
  void setArbitraryParamCallback(ArbitraryParamCallback cb, void* user = nullptr);
  /// BLOCKING — only call during setup(), never from loop() or update().
  bool isEncoderSearchComplete(uint32_t node_id, uint32_t timeout_ms = 100);
  /// BLOCKING — only call during setup(), never from loop() or update().
  bool readGpioStates(uint32_t node_id, uint32_t& states_out, uint32_t timeout_ms = 100);

  // ============================================================================
  // Axis State and Feedback
  // ============================================================================

  bool getAxisPV(uint32_t node_id, float& pos_rev_out, float& vel_rps_out, uint64_t& wall_us_out) const;
  bool getAxisIq(uint32_t node_id, float& iq_meas_out, float& iq_setpoint_out, uint64_t& wall_us_out) const;
  bool getAxisHeartbeat(uint32_t node_id, AxisHeartbeat& out) const;
  bool hasAxisError(uint32_t node_id, uint32_t mask) const;
  /// BLOCKING — only call during setup(), never from loop() or update().
  bool waitForAxisErrorClear(uint32_t node_id, uint32_t mask, uint32_t timeout_ms, uint16_t poll_ms = 5);
  bool isBallInHand() const { return ball_in_hand_; }

  // ============================================================================
  // Homing
  // ============================================================================

  void setHomeState(uint32_t node_id, AxisHomeState s);
  AxisHomeState getHomeState(uint32_t node_id) const;
  bool isAxisHomed(uint32_t node_id) const { return getHomeState(node_id) == AxisHomeState::Homed; }

  void requireHomeForAxis(uint32_t node_id, bool required);
  bool isHomeRequired(uint32_t node_id) const;
  void requireHomeOnlyFor(uint32_t node_id);
  void clearHomeRequirements();
  bool isAxisMotionAllowed(uint32_t node_id) const;

  // Non-blocking homing API: call startHomeHand() once, then poll
  // updateHomeHand() every loop() until it returns DONE or FAILED.
  bool startHomeHand(uint32_t node_id,
                     float homing_speed_rps,
                     float current_limit_A,
                     float current_headroom_A,
                     float settle_pos_rev,
                     float avg_weight = HandDefaults::HOMING_EMA_WEIGHT);

  bool startHomeHandStandard(uint32_t node_id,
                             int hand_direction = HandDefaults::HOMING_DIRECTION,
                             float base_speed_rps = HandDefaults::HOMING_SPEED_RPS,
                             float current_limit_A = HandDefaults::HOMING_CURRENT_A,
                             float current_headroom_A = HandDefaults::HOMING_HEADROOM_A,
                             float set_abs_pos_rev = HandDefaults::HOMING_ABS_POS_REV);

  HomingStatus updateHomeHand();

  // ============================================================================
  // Operating Configuration
  // ============================================================================

  bool restoreHandToOperatingConfig(uint32_t node_id,
                                    float vel_limit_rps = HandDefaults::OP_VEL_LIMIT_RPS,
                                    float current_limit_A = HandDefaults::OP_CURRENT_LIMIT_A);

  void setAutoClearBrakeResistor(bool enable, uint32_t min_interval_ms = CanCfg::AUTO_CLEAR_BRAKE_MS);
  void setEstimatorCmd(uint8_t cmd);

  // ============================================================================
  // Proprioception Mapping
  // ============================================================================

  void setHandAxisNode(uint8_t node_id)  { hand_node_id_  = node_id; }
  void setPitchAxisNode(uint8_t node_id) { pitch_node_id_ = node_id; }

  // ============================================================================
  // Host Command Interface
  // ============================================================================

  uint32_t getLastHostCmdMs() const { return last_host_cmd_ms_; }

  // ============================================================================
  // Ball Butler Heartbeat
  // ============================================================================

  void setStateMachine(StateMachine* sm) { state_machine_ = sm; }
  void setHeartbeatRate(uint32_t rate_ms) { heartbeat_rate_ms_ = rate_ms; }
  uint32_t getHeartbeatRate() const { return heartbeat_rate_ms_; }
  
  void setErrorCode(BallButlerError err) { current_error_code_ = err; }
  BallButlerError getErrorCode() const { return current_error_code_; }
  void clearErrorCode() { current_error_code_ = BallButlerError::NONE; }

  // ============================================================================
  // Static Utilities
  // ============================================================================

  static const char* errorCodeToString(BallButlerError err);
  static const CmdName* commandNameTable(size_t& count);

private:
  // ============================================================================
  // Internal Types
  // ============================================================================

  struct AxisStatePV { 
    volatile float pos_rev = 0.f;
    volatile float vel_rps = 0.f;
    volatile uint64_t wall_us = 0;
    volatile bool valid = false;
  };

  struct AxisStateIq { 
    volatile float iq_meas = 0.f;
    volatile float iq_setp = 0.f;
    volatile uint64_t wall_us = 0;
    volatile bool valid = false;
  };

  struct StatsAcc_ {
    int64_t sum = 0;
    uint64_t sum_sq = 0;
    int32_t minv = INT32_MAX;
    int32_t maxv = INT32_MIN;
    uint32_t n = 0;

    void add(int32_t x) {
      sum += x;
      sum_sq += uint64_t(int64_t(x) * int64_t(x));
      if (x < minv) minv = x;
      if (x > maxv) maxv = x;
      ++n;
    }
    void clear() { *this = {}; }
  };

  // ============================================================================
  // Constants
  // ============================================================================

  static constexpr uint8_t ALPHA_SHIFT_ = 3;
  static constexpr uint32_t PRINT_PERIOD_US_ = CanCfg::SYNC_STATS_PRINT_US;
  static constexpr float kVelScale_ = InputScale::hand_vel;
  static constexpr float kTorScale_ = InputScale::hand_tor;

  // ============================================================================
  // Static Members
  // ============================================================================

  static CanInterface* s_instance_;

  // ============================================================================
  // CAN Hardware
  // ============================================================================

  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1_;

  // ============================================================================
  // Configuration
  // ============================================================================

  // Debug
  Stream* dbg_ = nullptr;
  bool dbg_time_ = false;
  bool dbg_can_ = false;

  // Proprioception mapping
  uint8_t hand_node_id_  = 0xFF;
  uint8_t pitch_node_id_ = 0xFF;

  // Ball detection (non-blocking async state machine)
  enum class BallCheckPhase : uint8_t { IDLE, WAITING };
  uint8_t  ball_detect_gpio_pin_     = BallDetectCfg::GPIO_PIN;
  uint32_t ball_check_interval_ms_  = BallDetectCfg::CHECK_INTERVAL_MS;
  uint32_t last_ball_check_ms_      = 0;    // Last time we checked for ball in hand
  bool     ball_in_hand_            = true; // Assume we start with a ball in hand until we check
  uint8_t  ball_false_count_        = 0;    // Consecutive false readings so far
  uint8_t  max_ball_missing_samples_= BallDetectCfg::MAX_MISSING_SAMPLES;
  BallCheckPhase ball_check_phase_  = BallCheckPhase::IDLE;
  uint32_t ball_check_sent_ms_      = 0;    // When the SDO request was sent
  static constexpr uint32_t BALL_CHECK_TIMEOUT_MS = BallDetectCfg::CHECK_TIMEOUT_MS;

  // Auto-clear BRAKE_RESISTOR_DISARMED
  bool auto_clear_brake_res_ = true;
  uint32_t auto_clear_interval_ms_ = CanCfg::AUTO_CLEAR_BRAKE_MS;

  // Estimator command
  uint8_t estimatorCmd_ = uint8_t(Cmd::get_encoder_estimate);

  // ============================================================================
  // Time Sync State
  // ============================================================================

  volatile int64_t wall_offset_us_ = 0;
  volatile bool have_offset_ = false;
  StatsAcc_ stats_;
  uint64_t nextPrint_us_ = 0;

  // ============================================================================
  // Axis State Arrays
  // ============================================================================

  AxisStatePV axes_pv_[MAX_NODES];
  AxisStateIq axes_iq_[MAX_NODES];
  AxisHeartbeat hb_[MAX_NODES];
  AxisHomeState home_state_[MAX_NODES];
  uint64_t home_required_mask_ = 0;
  uint64_t last_brake_clear_us_[MAX_NODES] = {};

  // ============================================================================
  // Arbitrary Parameter State
  // ============================================================================

  ArbitraryParamResponse arb_param_resp_[MAX_NODES];
  ArbitraryParamCallback arb_param_cb_ = nullptr;
  void* arb_param_cb_user_ = nullptr;

  // ============================================================================
  // Host Command State
  // ============================================================================

  uint32_t last_host_cmd_ms_ = 0;  // Local millis() timestamp of last HOST_THROW_CMD

  // ============================================================================
  // Ball Butler Heartbeat State
  // ============================================================================

  StateMachine* state_machine_ = nullptr;
  uint32_t heartbeat_rate_ms_ = DEFAULT_HEARTBEAT_RATE_MS;
  uint32_t last_heartbeat_ms_ = 0;
  BallButlerError current_error_code_ = BallButlerError::NONE;

  // ============================================================================
  // Non-blocking Homing State
  // ============================================================================

  enum class HomingPhase : uint8_t {
    IDLE,           // Not homing
    CONFIGURE,      // Send clearErrors — advance immediately
    CLEAR_SETTLE,   // Wait for error clear to be processed before configuring axis
    SETTLE,         // Wait for control mode to take effect, verify CLOSED_LOOP
    SEND_VEL,       // Send velocity command — advance immediately
    MONITOR_IQ,     // Polling Iq readings, applying EMA filter, checking for spike
    STOP_SETTLE,    // Motor stopped, waiting before setting abs position
    FINALIZE,       // Set abs position, restore config, mark homed — advance immediately
  };

  HomingPhase homing_phase_      = HomingPhase::IDLE;
  uint32_t    homing_node_id_    = 0;
  float       homing_speed_rps_  = 0.f;
  float       homing_current_A_  = 0.f;
  float       homing_headroom_A_ = 0.f;
  float       homing_settle_rev_ = 0.f;
  float       homing_ema_weight_ = 0.f;
  float       homing_ema_        = 0.f;
  uint64_t    homing_last_iq_us_ = 0;
  uint32_t    homing_phase_ms_   = 0;     // millis() when current phase started
  uint32_t    homing_start_ms_   = 0;     // millis() when homing began (for timeout)

  // ============================================================================
  // Private Methods
  // ============================================================================

  // Time sync
  void handleTimeSync_(const CAN_message_t& msg);
  void maybePrintSyncStats_();

  // Receive handlers
  static void rxTrampoline_(const CAN_message_t& msg);
  void handleRx_(const CAN_message_t& msg);
  void handleTxSdo_(uint32_t node_id, const uint8_t* buf, uint8_t len);

  // Heartbeat
  void maybePublishHeartbeat_();
  void publishHeartbeat_();

  // Ball in hand check
  void maybeCheckBallInHand_();

  // Utilities
  static int16_t clampToI16_(float x);
};