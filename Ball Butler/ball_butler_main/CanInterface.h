#pragma once
#include <Arduino.h>
#include <FlexCAN_T4.h>

/**
 * CanInterface — Teensy 4.0 (CAN1) helper for:
 *  • ODrive CAN: set_input_pos / set_input_vel / limits / modes / gains / etc.
 *  • Time Sync (ID 0x7DD default) — shared wall clock (µs)
 *  • Estimator ingest (cmd=0x09) and Iq feedback (cmd=0x14 via RTR)
 *  • Homing routine for the hand axis (end-stop current spike)
 *
 * Notes
 *  - Uses CAN1.
 *  - vel_ff / torque_ff are scaled internally by 100.0 → int16.
 *  - For Iq feedback, we issue periodic RTR requests on cmd 0x14 and ingest replies.
 */

class CanInterface {
public:
  // ---------------------- ODrive command IDs -----------------------
  // Typed, Arduino-native (fast) and easy to reference.
  enum class Cmd : uint8_t {
    heartbeat_message        = 0x01,
    get_error                = 0x03,
    RxSdo                    = 0x04, // write arbitrary param
    TxSdo                    = 0x05, // read arbitrary param
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

  // Optional: name lookup for debugging (PROGMEM strings)
  struct CmdName { Cmd cmd; const char* name; };
  static const CmdName* commandNameTable(size_t& count);

  // ---- ODrive axis error bitmasks (public, single source of truth) ----
  struct ODriveErrors {
    static constexpr uint32_t BRAKE_RESISTOR_DISARMED = 0x08000000u; // 134,217,728
  };

  // ---------- Host (Jetson) -> Teensy throw command (8 bytes) ----------
  struct HostThrowCmd {
    float yaw_rad = 0.f;       // [-pi, +pi]
    float pitch_rad = 0.f;     // [0, pi/2]
    float speed_mps = 0.f;     // [0..6.5535]
    float in_s = 0.f;          // [0..65.535], time-from-now
    uint64_t wall_us = 0;      // time received (synced wall clock)
    bool valid = false;
  };

  // Set which CAN ID carries the 8B host command (11-bit ID, std frame)
  void setHostThrowCmdId(uint32_t id);
  // Read last host command (returns false if never received)
  bool getHostThrowCmd(HostThrowCmd& out) const;

  // Optional: register a callback invoked on each valid host command
  typedef void (*HostThrowCallback)(const HostThrowCmd& cmd, void* user);
  void setHostThrowCallback(HostThrowCallback cb, void* user = nullptr);

  // ---------------------- Basic homing state -----------------------
  enum class AxisHomeState : uint8_t { Unhomed = 0, Homing = 1, Homed = 2 };

  void setHomeState(uint32_t node_id, AxisHomeState s);
  AxisHomeState getHomeState(uint32_t node_id) const;
  bool isAxisHomed(uint32_t node_id) const { return getHomeState(node_id) == AxisHomeState::Homed; }

  // ---- Per-axis "requires homing" policy (bitmask) ----
  // If an axis is NOT required, motion is always allowed.
  // If required, motion allowed only in Homing/Homed states.
  void requireHomeForAxis(uint32_t node_id, bool required);
  bool isHomeRequired(uint32_t node_id) const;
  void requireHomeOnlyFor(uint32_t node_id);     // convenience: exactly one axis required
  void clearHomeRequirements();                  // convenience: none required

  // Motion gate now considers the requirement mask
  bool isAxisMotionAllowed(uint32_t node_id) const;

  // ---------------------- Auto-clear Brake Resistor Disarmed -------------------------------
  // Enable/disable auto-clear and set the minimum interval between clear attempts per axis.
  // Default behavior is enabled with 500 ms interval.
  void setAutoClearBrakeResistor(bool enable, uint32_t min_interval_ms = 500);

public:
  CanInterface();

  // -------------------- Setup / service --------------------
  void begin(uint32_t bitrate = 1'000'000);
  void loop(); // call frequently

  // -------------------- Debug control ----------------------
  void setDebugStream(Stream* dbg);                 // nullptr = silent
  void setDebugFlags(bool timeSyncDebug, bool canDebug);

  // -------------------- Time Sync --------------------------
  void setTimeSyncId(uint32_t id);                  // default 0x7DD
  uint64_t wallTimeUs() const;                      // synced wall time (us)
  uint64_t localTimeUs() const;                     // local monotonic micros64()

  struct SyncStats {
    float mean_us = 0.f;
    float rms_us  = 0.f;
    int32_t min_us = 0;
    int32_t max_us = 0;
    uint32_t frames = 0;
  };
  SyncStats getAndClearSyncStats();                 // snapshot + clear

  // ---- Heartbeat snapshot (cached on receipt) ----
  struct AxisHeartbeat {
    uint32_t axis_error = 0;     // <axis>.active_errors | <axis>.disarm_reason
    uint8_t  axis_state = 0;     // <axis>.current_state
    uint8_t  procedure_result = 0;
    uint8_t  trajectory_done = 0;
    uint64_t wall_us = 0;        // time we received the heartbeat (synced clock)
    bool     valid = false;
  };

  bool getAxisHeartbeat(uint32_t node_id, AxisHeartbeat& out) const;

  // Convenience checks/waits so app code doesn’t touch bitmasks
  bool hasAxisError(uint32_t node_id, uint32_t mask) const;
  bool waitForAxisErrorClear(uint32_t node_id, uint32_t mask,
                            uint32_t timeout_ms, uint16_t poll_ms = 5);

  // -------------------- Generic send helpers ----------------
  bool sendRaw(uint32_t id, const uint8_t* data, uint8_t len);
  bool sendRTR(uint32_t id, uint8_t len);

  // Compose 11-bit arbitration id from node + cmd
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

  // Position + feed-forwards (scaled internally: 100.0)
  bool sendInputPos(uint32_t node_id, float pos_rev, float vel_ff_rev_per_s, float torque_ff);

  // Velocity command (scaled torque_ff only)
  bool sendInputVel(uint32_t node_id, float vel_rps, float torque_ff);

  // Estimator and current feedback access
  bool getAxisPV(uint32_t node_id, float& pos_rev_out, float& vel_rps_out, uint64_t& wall_us_out) const;
  bool getAxisIq(uint32_t node_id, float& iq_meas_out, float& iq_setpoint_out, uint64_t& wall_us_out) const;

  // Configure the hand to the normal operating conditions
  bool restoreHandToOperatingConfig(uint32_t node_id,
                       float vel_limit_rps = 1000.0f,
                       float current_limit_A = 50.0f);

  // -------------------- Homing (Hand) ----------------------
  // High-level "home the hand" routine (end-stop by Iq spike).
  // Returns true on success.
  //
  // Params:
  //  node_id: ODrive axis id for the hand (e.g., 6)
  //  homing_speed_rps: signed; move toward the hard stop
  //  current_limit_A: threshold for |Iq| avg to stop
  //  current_headroom_A: extra margin to set as axis Current_Limit during homing
  //  settle_pos_rev: position to set after hit (e.g., small offset away from stop)
  //  avg_weight: EMA factor (0..1), larger = slower response
  //  iq_poll_period_ms: how often to request Iq via RTR
  bool homeHand(uint32_t node_id,
                float homing_speed_rps,
                float current_limit_A,
                float current_headroom_A,
                float settle_pos_rev,
                float avg_weight = 0.7f,
                uint16_t iq_poll_period_ms = 10);

  // Convenience with the typical defaults:
  bool homeHandStandard(uint32_t node_id,
                        int hand_direction = -1,
                        float base_speed_rps = 3.0f,
                        float current_limit_A = 5.0f,
                        float current_headroom_A = 3.0f,
                        float set_abs_pos_rev = -0.1f);

  // -------------------- Configs ----------------------------
  // Change which CAN cmd id is treated as "Estimator" (default 0x09 per ODrive)
  void setEstimatorCmd(uint8_t cmd);

private:
  // ---------- processing Jetson commands ---------
  uint32_t host_throw_id_ = 0;           // 0 = disabled
  HostThrowCmd last_host_cmd_;           // cached
  HostThrowCallback host_cb_ = nullptr;
  void* host_cb_user_ = nullptr;

  // ---------- internal types ----------
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

  AxisHomeState home_state_[64];   // per-axis homing state
  AxisHeartbeat hb_[64];  // last heartbeat per axis
  uint64_t home_required_mask_ = 0;  // bit i => axis i requires homing

  // Auto-clear BRAKE_RESISTOR_DISARMED (0x08000000) when seen in Heartbeat axis_error
  bool     auto_clear_brake_res_ = true;
  uint32_t auto_clear_interval_ms_ = 500;
  uint64_t last_brake_clear_us_[64] = {};  // per-axis throttle timer

  // ---------- time sync internals ----------
  static uint64_t micros64_();                      // 64-bit local micros
  void handleTimeSync_(const CAN_message_t& msg);
  void maybePrintSyncStats_();

  // ---------- receive path ----------
  static void rxTrampoline_(const CAN_message_t &msg);
  void handleRx_(const CAN_message_t &msg);

  // ---------- helpers ----------
  static int16_t clampToI16_(float x);

private:
  // Teensy CAN1
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1_;

  // Singleton pointer for trampoline
  static CanInterface* s_instance_;

  // Debug
  Stream* dbg_ = nullptr;
  bool dbg_time_ = false;
  bool dbg_can_  = false;

  // Time sync
  uint32_t timeSyncId_ = 0x7DD;
  volatile int64_t wall_offset_us_ = 0; // master_us - local_us
  volatile bool have_offset_ = false;
  static constexpr uint8_t ALPHA_SHIFT_ = 3; // IIR smoothing

  // Stats
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
  } stats_;
  uint64_t nextPrint_us_ = 0;
  static constexpr uint32_t PRINT_PERIOD_US_ = 1'000'000; // 1 Hz

  // Estimator ingest (pos/vel) and Iq
  uint8_t estimatorCmd_ = uint8_t(Cmd::get_encoder_estimate);
  AxisStatePV axes_pv_[64]; // node_id 0..63
  AxisStateIq axes_iq_[64];

  // Scales — fixed at compile-time by spec (change only in firmware)
  static constexpr float kVelScale_ = 100.0f;
  static constexpr float kTorScale_ = 100.0f;
};
