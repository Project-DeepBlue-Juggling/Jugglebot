#pragma once
#include <Arduino.h>
#include <FlexCAN_T4.h>

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

// ============================================================================
// Centralized CAN ID Registry
// ============================================================================
/**
 * All CAN IDs used in the system are defined here in a single location.
 * This makes it easy to:
 *   - See all IDs at a glance and avoid collisions
 *   - Change IDs without hunting through the codebase
 *   - Add new IDs in an organized manner
 *
 * Storage: constexpr ensures these are compile-time constants with no runtime
 * overhead. The compiler will inline these values directly.
 */
namespace CanIds {
  // -------------------- Ball Butler Protocol IDs --------------------
  // These IDs are used for communication between the Ball Butler and host
  constexpr uint32_t HOST_THROW_CMD    = 0x7D0;  // Host -> BB throw command
  constexpr uint32_t HEARTBEAT_CMD     = 0x7D1;  // Ball Butler heartbeat (BB -> Host)
  constexpr uint32_t RELOAD_CMD        = 0x7D2;  // Host -> BB reload command
  constexpr uint32_t RESET_CMD         = 0x7D3;  // Host -> BB reset command
  constexpr uint32_t CALIBRATE_LOC_CMD = 0x7D4;  // Host -> BB calibrate location command

  // -------------------- Time Synchronization --------------------
  constexpr uint32_t TIME_SYNC_CMD     = 0x7DD;  // Wall clock sync from master
}

class CanInterface {
public:
  // ============================================================================
  // Constants
  // ============================================================================
  // Default heartbeat rate (ms). Set to 0 to disable.
  static constexpr uint32_t DEFAULT_HEARTBEAT_RATE_MS = 100;  // 20 Hz

  // SDO Opcodes for arbitrary parameters
  static constexpr uint8_t OPCODE_READ  = 0x00;
  static constexpr uint8_t OPCODE_WRITE = 0x01;

  // ============================================================================
  // Types and Enums
  // ============================================================================

  enum class BallButlerError : uint8_t {
    NONE          = 0,
    RELOAD_FAILED = 1,
  };

  enum class Cmd : uint8_t {
    heartbeat_message        = 0x01,
    get_error                = 0x03,
    RxSdo                    = 0x04,
    TxSdo                    = 0x05,
    set_requested_state      = 0x07,
    get_encoder_estimate     = 0x09,
    set_controller_mode      = 0x0B,
    set_input_pos            = 0x0C,
    set_input_vel            = 0x0D,
    set_vel_curr_limits      = 0x0F,
    set_traj_vel_limit       = 0x11,
    set_traj_acc_limits      = 0x12,
    get_iq                   = 0x14,
    get_temps                = 0x15,
    reboot_odrives           = 0x16,
    get_bus_voltage_current  = 0x17,
    clear_errors             = 0x18,
    set_absolute_position    = 0x19,
    set_pos_gain             = 0x1A,
    set_vel_gains            = 0x1B,
  };

  enum class AxisHomeState : uint8_t { 
    Unhomed = 0, 
    Homing  = 1, 
    Homed   = 2 
  };

  struct CmdName { 
    Cmd cmd; 
    const char* name; 
  };

  struct ODriveErrors {
    static constexpr uint32_t BRAKE_RESISTOR_DISARMED = 0x08000000u;
  };

  struct EndpointIds {
    static constexpr uint16_t COMMUTATION_MAPPER_POS_ABS = 488;  // NaN until encoder search is complete
    static constexpr uint16_t GPIO_STATES                = 700;
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
    float in_s = 0.f;
    uint64_t wall_us = 0;
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
  typedef void (*HostThrowCallback)(const HostThrowCmd& cmd, void* user);
  typedef void (*ArbitraryParamCallback)(uint32_t node_id, const ArbitraryParamResponse& resp, void* user);

  // ============================================================================
  // Construction and Setup
  // ============================================================================

  CanInterface();

  void begin(uint32_t bitrate = 1'000'000);
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
  bool isEncoderSearchComplete(uint32_t node_id, uint32_t timeout_ms = 100);
  bool readGpioStates(uint32_t node_id, uint32_t& states_out, uint32_t timeout_ms = 100);

  // ============================================================================
  // Axis State and Feedback
  // ============================================================================

  bool getAxisPV(uint32_t node_id, float& pos_rev_out, float& vel_rps_out, uint64_t& wall_us_out) const;
  bool getAxisIq(uint32_t node_id, float& iq_meas_out, float& iq_setpoint_out, uint64_t& wall_us_out) const;
  bool getAxisHeartbeat(uint32_t node_id, AxisHeartbeat& out) const;
  bool hasAxisError(uint32_t node_id, uint32_t mask) const;
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

  bool homeHand(uint32_t node_id,
                float homing_speed_rps,
                float current_limit_A,
                float current_headroom_A,
                float settle_pos_rev,
                float avg_weight = 0.7f,
                uint16_t iq_poll_period_ms = 10);

  bool homeHandStandard(uint32_t node_id,
                        int hand_direction = -1,
                        float base_speed_rps = 3.0f,
                        float current_limit_A = 5.0f,
                        float current_headroom_A = 3.0f,
                        float set_abs_pos_rev = -0.1f);

  // ============================================================================
  // Operating Configuration
  // ============================================================================

  bool restoreHandToOperatingConfig(uint32_t node_id,
                                    float vel_limit_rps = 1000.0f,
                                    float current_limit_A = 50.0f);

  void setAutoClearBrakeResistor(bool enable, uint32_t min_interval_ms = 500);
  void setEstimatorCmd(uint8_t cmd);

  // ============================================================================
  // Proprioception Mapping
  // ============================================================================

  void setHandAxisNode(uint8_t node_id)  { hand_node_id_  = node_id; }
  void setPitchAxisNode(uint8_t node_id) { pitch_node_id_ = node_id; }

  // ============================================================================
  // Host Command Interface
  // ============================================================================

  bool getHostThrowCmd(HostThrowCmd& out) const;
  void setHostThrowCallback(HostThrowCallback cb, void* user = nullptr);
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
  static constexpr uint32_t PRINT_PERIOD_US_ = 1'000'000;
  static constexpr float kVelScale_ = 100.0f;
  static constexpr float kTorScale_ = 100.0f;

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

  // Ball detection 
  uint8_t  ball_detect_gpio_pin    = 3;    // GPIO pin on hand ODrive for ball detection
  uint32_t ball_check_interval_ms_ = 200;  // How often to check for ball in hand
  uint32_t last_ball_check_ms_     = 0;    // Last time we checked for ball in hand
  bool     ball_in_hand_           = false;

  // Auto-clear BRAKE_RESISTOR_DISARMED
  bool auto_clear_brake_res_ = true;
  uint32_t auto_clear_interval_ms_ = 500;

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

  AxisStatePV axes_pv_[64];
  AxisStateIq axes_iq_[64];
  AxisHeartbeat hb_[64];
  AxisHomeState home_state_[64];
  uint64_t home_required_mask_ = 0;
  uint64_t last_brake_clear_us_[64] = {};

  // ============================================================================
  // Arbitrary Parameter State
  // ============================================================================

  ArbitraryParamResponse arb_param_resp_[64];
  ArbitraryParamCallback arb_param_cb_ = nullptr;
  void* arb_param_cb_user_ = nullptr;

  // ============================================================================
  // Host Command State
  // ============================================================================

  HostThrowCmd last_host_cmd_;
  uint32_t last_host_cmd_ms_ = 0;  // Local millis() timestamp of last HOST_THROW_CMD
  HostThrowCallback host_cb_ = nullptr;
  void* host_cb_user_ = nullptr;

  // ============================================================================
  // Ball Butler Heartbeat State
  // ============================================================================

  StateMachine* state_machine_ = nullptr;
  uint32_t heartbeat_rate_ms_ = DEFAULT_HEARTBEAT_RATE_MS;
  uint32_t last_heartbeat_ms_ = 0;
  BallButlerError current_error_code_ = BallButlerError::NONE;

  // ============================================================================
  // Private Methods
  // ============================================================================

  // Time sync
  static uint64_t micros64_();
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