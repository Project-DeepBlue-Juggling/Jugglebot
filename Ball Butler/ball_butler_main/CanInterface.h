#pragma once
#include <Arduino.h>
#include <FlexCAN_T4.h>

/**
 * CanInterface Teensy 4.0 (CAN1) helper for:
 *  ODrive CAN: set_input_pos / set_input_vel / limits / modes / gains / etc.
 *  Time Sync (ID 0x7DD default)  shared wall clock
 *  Estimator ingest (cmd=0x09) and Iq feedback (cmd=0x14)
 *  Homing routine for the hand axis (end-stop current spike)
 *  Arbitrary parameter read/write via SDO (RxSdo/TxSdo)
 *
 * Notes
 *  - Uses CAN1.
 *  - vel_ff / torque_ff are scaled internally by 100.0 int16.
 */

class CanInterface {
public:
  // ---------------------- ODrive command IDs -----------------------
  enum class Cmd : uint8_t {
    heartbeat_message        = 0x01,
    get_error                = 0x03,
    RxSdo                    = 0x04,  // Write to arbitrary parameter
    TxSdo                    = 0x05,  // Read from arbitrary parameter (response)
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

  struct CmdName { Cmd cmd; const char* name; };
  static const CmdName* commandNameTable(size_t& count);

  struct ODriveErrors {
    static constexpr uint32_t BRAKE_RESISTOR_DISARMED = 0x08000000u;
  };

  // ---------------------- SDO Opcodes for arbitrary parameters ----------------------
  static constexpr uint8_t OPCODE_READ  = 0x00;  // For reading arbitrary parameters
  static constexpr uint8_t OPCODE_WRITE = 0x01;  // For writing arbitrary parameters

  // ---------------------- Known endpoint IDs for arbitrary parameters ----------------------
  struct EndpointIds {
    static constexpr uint16_t COMMUTATION_MAPPER_POS_ABS = 488;  // commutation_mapper.pos_abs (NaN until encoder search complete)
    static constexpr uint16_t GPIO_STATES                = 700;  // get_gpio_states
  };

  // ---------------------- Arbitrary parameter response struct ----------------------
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

  // ---------- Host (Jetson) -> Teensy throw command ----------
  struct HostThrowCmd {
    float yaw_rad = 0.f;
    float pitch_rad = 0.f;
    float speed_mps = 0.f;
    float in_s = 0.f;
    uint64_t wall_us = 0;
    bool valid = false;
  };

  void setHostThrowCmdId(uint32_t id);
  bool getHostThrowCmd(HostThrowCmd& out) const;
  typedef void (*HostThrowCallback)(const HostThrowCmd& cmd, void* user);
  void setHostThrowCallback(HostThrowCallback cb, void* user = nullptr);

  // ---------------------- App-facing homing policy -----------------------
  enum class AxisHomeState : uint8_t { Unhomed = 0, Homing = 1, Homed = 2 };

  void setHomeState(uint32_t node_id, AxisHomeState s);
  AxisHomeState getHomeState(uint32_t node_id) const;
  bool isAxisHomed(uint32_t node_id) const { return getHomeState(node_id) == AxisHomeState::Homed; }

  void requireHomeForAxis(uint32_t node_id, bool required);
  bool isHomeRequired(uint32_t node_id) const;
  void requireHomeOnlyFor(uint32_t node_id);
  void clearHomeRequirements();
  bool isAxisMotionAllowed(uint32_t node_id) const;

  // ---------------------- Auto-clear BRAKE_RESISTOR ----------------------
  void setAutoClearBrakeResistor(bool enable, uint32_t min_interval_ms = 500);

public:
  CanInterface();

  // -------------------- Setup / service --------------------
  void begin(uint32_t bitrate = 1'000'000);
  void loop();

  // -------------------- Debug control ----------------------
  void setDebugStream(Stream* dbg);
  void setDebugFlags(bool timeSyncDebug, bool canDebug);

  // -------------------- Time Sync --------------------------
  void setTimeSyncId(uint32_t id);
  uint64_t wallTimeUs() const;
  uint64_t localTimeUs() const;
  struct SyncStats {
    float mean_us = 0.f; float rms_us = 0.f; int32_t min_us = 0; int32_t max_us = 0; uint32_t frames = 0;
  };
  SyncStats getAndClearSyncStats();

  // ---- Heartbeat snapshot ----
  struct AxisHeartbeat {
    uint32_t axis_error = 0;
    uint8_t  axis_state = 0;
    uint8_t  procedure_result = 0;
    uint8_t  trajectory_done = 0;
    uint64_t wall_us = 0;
    bool     valid = false;
  };
  bool getAxisHeartbeat(uint32_t node_id, AxisHeartbeat& out) const;

  bool hasAxisError(uint32_t node_id, uint32_t mask) const;
  bool waitForAxisErrorClear(uint32_t node_id, uint32_t mask,
                             uint32_t timeout_ms, uint16_t poll_ms = 5);

  // -------------------- Generic send helpers ----------------
  bool sendRaw(uint32_t id, const uint8_t* data, uint8_t len);
  bool sendRTR(uint32_t id, uint8_t len);  // (kept for generality, but not used for Iq anymore)

  static inline uint32_t makeId(uint32_t node_id, Cmd cmd) {
    return (node_id << 5) | (uint8_t(cmd) & 0x1F);
  }

  // -------------------- ODrive helpers ---------------------
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

  // -------------------- Arbitrary Parameter (SDO) Functions --------------------
  /**
   * Send an arbitrary parameter write command to an ODrive.
   * 
   * @param node_id     The axis/node ID (0-63)
   * @param endpoint_id The endpoint ID for the parameter (see EndpointIds struct)
   * @param value       The float value to write
   * @return true if the CAN message was sent successfully
   */
  bool sendArbitraryParameterFloat(uint32_t node_id, uint16_t endpoint_id, float value);

  /**
   * Send an arbitrary parameter write command with uint32 value.
   * 
   * @param node_id     The axis/node ID (0-63)
   * @param endpoint_id The endpoint ID for the parameter
   * @param value       The uint32 value to write
   * @return true if the CAN message was sent successfully
   */
  bool sendArbitraryParameterU32(uint32_t node_id, uint16_t endpoint_id, uint32_t value);

  /**
   * Request to read an arbitrary parameter from an ODrive.
   * The response will be received asynchronously via handleRx_ and stored internally.
   * Use getLastArbitraryParamResponse() to retrieve the response.
   * 
   * @param node_id     The axis/node ID (0-63)
   * @param endpoint_id The endpoint ID for the parameter to read
   * @return true if the CAN message was sent successfully
   */
  bool requestArbitraryParameter(uint32_t node_id, uint16_t endpoint_id);

  /**
   * Get the last received arbitrary parameter response for a given node.
   * 
   * @param node_id The axis/node ID
   * @param out     Output struct to store the response
   * @return true if a valid response exists for this node
   */
  bool getLastArbitraryParamResponse(uint32_t node_id, ArbitraryParamResponse& out) const;

  /**
   * Callback type for arbitrary parameter responses
   */
  typedef void (*ArbitraryParamCallback)(uint32_t node_id, const ArbitraryParamResponse& resp, void* user);
  
  /**
   * Set a callback to be invoked when an arbitrary parameter response is received.
   * 
   * @param cb   The callback function
   * @param user User data pointer passed to the callback
   */
  void setArbitraryParamCallback(ArbitraryParamCallback cb, void* user = nullptr);

  /**
   * Check if encoder search is complete on a given axis by reading commutation_mapper.pos_abs.
   * Returns true if the value is not NaN (i.e., encoder search is complete).
   * This is a blocking function that waits for the response.
   * 
   * @param node_id    The axis/node ID
   * @param timeout_ms Maximum time to wait for response
   * @return true if encoder search is complete, false if NaN or timeout
   */
  bool isEncoderSearchComplete(uint32_t node_id, uint32_t timeout_ms = 100);

  /**
   * Read GPIO states from an ODrive.
   * 
   * @param node_id    The axis/node ID
   * @param states_out Output: the GPIO states as a bitmask
   * @param timeout_ms Maximum time to wait for response
   * @return true if successfully read
   */
  bool readGpioStates(uint32_t node_id, uint32_t& states_out, uint32_t timeout_ms = 100);

  // Estimator and current feedback access
  bool getAxisPV(uint32_t node_id, float& pos_rev_out, float& vel_rps_out, uint64_t& wall_us_out) const;
  bool getAxisIq(uint32_t node_id, float& iq_meas_out, float& iq_setpoint_out, uint64_t& wall_us_out) const;

  // Typical config
  bool restoreHandToOperatingConfig(uint32_t node_id,
                       float vel_limit_rps = 1000.0f,
                       float current_limit_A = 50.0f);

  // -------------------- Homing (Hand) ----------------------
  bool homeHand(uint32_t node_id,
                float homing_speed_rps,
                float current_limit_A,
                float current_headroom_A,
                float settle_pos_rev,
                float avg_weight = 0.7f,
                uint16_t /*iq_poll_period_ms UNUSED now*/ = 10);

  bool homeHandStandard(uint32_t node_id,
                        int hand_direction = -1,
                        float base_speed_rps = 3.0f,
                        float current_limit_A = 5.0f,
                        float current_headroom_A = 3.0f,
                        float set_abs_pos_rev = -0.1f);

  // -------------------- Configs ----------------------------
  void setEstimatorCmd(uint8_t cmd);

  // -------------------- Proprioception mapping -------------
  // Tell CanInterface which nodes correspond to our proprioception channels.
  void setHandAxisNode(uint8_t node_id)  { hand_node_id_  = (node_id); }
  void setPitchAxisNode(uint8_t node_id) { pitch_node_id_ = (node_id);}

private:
  // ---------- processing Jetson commands ---------
  uint32_t host_throw_id_ = 0;           
  HostThrowCmd last_host_cmd_;           
  HostThrowCallback host_cb_ = nullptr;
  void* host_cb_user_ = nullptr;

  // ---------- internal types ----------
  struct AxisStatePV { volatile float pos_rev=0.f, vel_rps=0.f; volatile uint64_t wall_us=0; volatile bool valid=false; };
  struct AxisStateIq { volatile float iq_meas=0.f, iq_setp=0.f; volatile uint64_t wall_us=0; volatile bool valid=false; };

  AxisHomeState home_state_[64];
  AxisHeartbeat hb_[64];
  uint64_t home_required_mask_ = 0;

  // Auto-clear BRAKE_RESISTOR_DISARMED
  bool     auto_clear_brake_res_ = true;
  uint32_t auto_clear_interval_ms_ = 500;
  uint64_t last_brake_clear_us_[64] = {};

  // ---------- Arbitrary parameter response storage ----------
  ArbitraryParamResponse arb_param_resp_[64];
  ArbitraryParamCallback arb_param_cb_ = nullptr;
  void* arb_param_cb_user_ = nullptr;

  // ---------- time sync internals ----------
  static uint64_t micros64_();
  void handleTimeSync_(const CAN_message_t& msg);
  void maybePrintSyncStats_();

  // ---------- receive path ----------
  static void rxTrampoline_(const CAN_message_t &msg);
  void handleRx_(const CAN_message_t &msg);
  void handleTxSdo_(uint32_t node_id, const uint8_t* buf, uint8_t len);

  // ---------- helpers ----------
  static int16_t clampToI16_(float x);

private:
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1_;
  static CanInterface* s_instance_;
  Stream* dbg_ = nullptr;
  bool dbg_time_ = false, dbg_can_ = false;

  uint32_t timeSyncId_ = 0x7DD;
  volatile int64_t wall_offset_us_ = 0;
  volatile bool have_offset_ = false;
  static constexpr uint8_t ALPHA_SHIFT_ = 3;

  struct StatsAcc_ { int64_t sum=0; uint64_t sum_sq=0; int32_t minv=INT32_MAX; int32_t maxv=INT32_MIN; uint32_t n=0;
    void add(int32_t x){ sum+=x; sum_sq+=uint64_t(int64_t(x)*int64_t(x)); if(x<minv)minv=x; if(x>maxv)maxv=x; ++n; }
    void clear(){ *this={}; }
  } stats_;
  uint64_t nextPrint_us_ = 0;
  static constexpr uint32_t PRINT_PERIOD_US_ = 1'000'000;

  uint8_t estimatorCmd_ = uint8_t(Cmd::get_encoder_estimate);
  AxisStatePV axes_pv_[64];
  AxisStateIq axes_iq_[64];

  // Proprioception mapping
  uint8_t hand_node_id_  = 0xFF;
  uint8_t pitch_node_id_ = 0xFF;

  // Ball detection thresholds
  float ball_detect_iq_threshold  = -0.5f; // Iq > this means ball present

  // Scales
  static constexpr float kVelScale_ = 100.0f;
  static constexpr float kTorScale_ = 100.0f;
};