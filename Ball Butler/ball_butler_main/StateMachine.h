#pragma once
/*
 * StateMachine.h - Robot state machine for Ball Butler
 * 
 * States:
 *   BOOT      - Homing all axes, checking for ball in hand
 *   IDLE      - Awaiting throw command (ball should be in hand)
 *   THROWING  - Executing throw trajectory
 *   RELOADING - Running reload sequence to grab next ball
 *   CALIBRATING - Calibrating location (yaw axis)
 *   ERROR     - Error state (manual reset required)
 * 
 * Transitions are event-based:
 *   BOOT -> IDLE:      After successful homing and ball detection
 *   BOOT -> ERROR:     Homing timeout or failure
 *   IDLE -> THROWING:  CAN or Serial command received
 *   THROWING -> RELOADING: Throw complete + delay elapsed
 *   RELOADING -> IDLE: Ball successfully grabbed
 *   RELOADING -> ERROR: Max reload attempts exceeded
 *   Any -> ERROR:      Critical failure
 *   ERROR -> BOOT:     Manual reset command
 * 
 * USAGE:
 *   1. Create StateMachine instance with references to hardware modules
 *   2. Call begin() in setup()
 *   3. Call update() in loop()
 *   4. Use requestThrow() to trigger a throw, and requestReload() to trigger a reload
 *   5. Use reset() to recover from ERROR state
 */

#include <Arduino.h>
#include <vector>
#include "BallButlerConfig.h"
#include "RobotState.h"
#include "TrajFrame.h"

// Forward declarations
class CanInterface;
class YawAxis;
class PitchAxis;
class HandTrajectoryStreamer;
class HandPathPlanner;

// Tracking command structure (for queueing tracking updates)
struct TrackingCmd {
  float yaw_deg = 0.f;
  float pitch_deg = 0.f;
  uint32_t received_ms = 0;
  bool valid = false;
};



// Convert state to string for debug output
const char* robotStateToString(RobotState s);

// --------------------------------------------------------------------
// StateMachine class
// --------------------------------------------------------------------
class StateMachine {
public:
  // ----------------------------------------------------------------
  // Configuration structure
  // ----------------------------------------------------------------
  // All defaults reference BallButlerConfig.h (SMDefaults::)
  struct Config {
    // Timeouts
    uint32_t homing_timeout_ms      = SMDefaults::HOMING_TIMEOUT_MS;
    uint32_t homing_retry_delay_ms  = SMDefaults::HOMING_RETRY_DELAY_MS;
    uint32_t reload_timeout_ms      = SMDefaults::RELOAD_TIMEOUT_MS;
    uint32_t post_throw_delay_ms    = SMDefaults::POST_THROW_DELAY_MS;
    uint32_t idle_no_cmd_timeout_ms = SMDefaults::IDLE_NO_CMD_TIMEOUT_MS;

    // Reload sequence positions and other parameters
    float reload_hand_top_rev     = SMDefaults::RELOAD_HAND_TOP_REV;
    float reload_pitch_ready_deg  = SMDefaults::RELOAD_PITCH_READY_DEG;

    float reload_yaw_angle_deg    = SMDefaults::RELOAD_YAW_ANGLE_DEG;
    float reload_pitch_grab_deg   = SMDefaults::RELOAD_PITCH_GRAB_DEG;

    float reload_hand_bottom_rev  = SMDefaults::RELOAD_HAND_BOTTOM_REV;
    float reload_hand_bottom_tolerance_rev = SMDefaults::RELOAD_HAND_BOTTOM_TOL_REV;
    uint32_t reload_hold_delay_ms = SMDefaults::RELOAD_HOLD_DELAY_MS;
    uint8_t reload_ball_check_samples = SMDefaults::RELOAD_BALL_CHECK_SAMPLES;

    uint32_t tracking_hand_pos_check_interval_ms = SMDefaults::TRACKING_HAND_CHECK_MS;

    // Rate-limiting intervals for tracking commands (prevents CAN bus flooding)
    uint32_t yaw_cmd_interval_ms   = SMDefaults::YAW_CMD_INTERVAL_MS;
    uint32_t pitch_cmd_interval_ms = SMDefaults::PITCH_CMD_INTERVAL_MS;

    // Ball-in-hand monitoring (CHECKING_BALL state)
    uint8_t check_ball_confirm_samples = SMDefaults::CHECK_BALL_CONFIRM_SAMPLES;
    float check_ball_disrupt_pitch_deg = SMDefaults::CHECK_BALL_DISRUPT_PITCH_DEG;

    // Positions to go to if reload fails or succeeds (home position)
    float hand_rev_home    = SMDefaults::HAND_REV_HOME;
    float pitch_deg_home   = SMDefaults::PITCH_DEG_HOME;
    float yaw_deg_home     = SMDefaults::YAW_DEG_HOME;

    // Calibration
    float calibrate_location_max_yaw_deg = SMDefaults::CALIB_MAX_YAW_DEG;
    float calibrate_location_min_yaw_deg = SMDefaults::CALIB_MIN_YAW_DEG;
    float yaw_pre_calib_accel = 0.0f; // Yaw acceleration before calibration (saved/restored at runtime)
    float yaw_pre_calib_decel = 0.0f; // Yaw deceleration before calibration (saved/restored at runtime)
    float yaw_calib_accel     = SMDefaults::CALIB_YAW_ACCEL;
    float yaw_calib_decel     = SMDefaults::CALIB_YAW_DECEL;
    uint32_t calibration_pause_ms = SMDefaults::CALIB_PAUSE_MS;

    // Retry limits
    uint8_t max_reload_attempts      = SMDefaults::MAX_RELOAD_ATTEMPTS;
    uint8_t max_homing_attempts      = SMDefaults::MAX_HOMING_ATTEMPTS;

    // Axis settle times (ms to wait after commanding position) and positions
    uint32_t pitch_settle_ms       = SMDefaults::PITCH_SETTLE_MS;
    uint32_t pitch_grab_settle_ms  = SMDefaults::PITCH_GRAB_SETTLE_MS;
    float yaw_angle_threshold_deg  = SMDefaults::YAW_ANGLE_THRESHOLD_DEG;

    // Axis limits
    float pitch_min_stow_angle_deg = SMDefaults::PITCH_MIN_STOW_DEG;
    float yaw_min_angle_deg        = SMDefaults::YAW_MIN_ANGLE_DEG;
    float yaw_max_angle_deg        = SMDefaults::YAW_MAX_ANGLE_DEG;

    // Node IDs (defaults from BallButlerConfig.h)
    uint8_t hand_node_id  = NodeId::HAND;
    uint8_t pitch_node_id = NodeId::PITCH;
  };

  // ----------------------------------------------------------------
  // Constructor
  // ----------------------------------------------------------------
  StateMachine(CanInterface& can, YawAxis& yaw, PitchAxis& pitch,
               HandTrajectoryStreamer& streamer, HandPathPlanner& planner);

  // ----------------------------------------------------------------
  // Lifecycle
  // ----------------------------------------------------------------
  // Initialize the state machine (call in setup())
  void begin();
  
  // Update the state machine (call in loop())
  void update();

  // ----------------------------------------------------------------
  // Configuration
  // ----------------------------------------------------------------
  void setConfig(const Config& cfg) { config_ = cfg; }
  Config& config() { return config_; }
  const Config& config() const { return config_; }

  // ----------------------------------------------------------------
  // Commands
  // ----------------------------------------------------------------
  // Request a throw or reload with given parameters
  // Returns true if command was accepted, false if rejected (wrong state, etc.)
  bool requestThrow(float yaw_deg, float pitch_deg, float speed_mps, uint64_t throw_wall_us);
  bool requestReload();
  bool requestCheckBall();
  bool requestCalibrateLocation();
  
  // Request tracking mode (called when HOST_THROW_CMD has speed=0)
  // Updates yaw/pitch targets while in TRACKING state
  bool requestTracking(float yaw_deg, float pitch_deg);

  // Request a smooth move of the hand axis to a target position (in revolutions)
  // Handles PV lookup, planning, buffer ownership, and streamer arming internally
  bool requestSmoothMove(float target_rev);
  
  // Manual reset from ERROR state -> restarts BOOT sequence
  void reset();
  
  // Force transition to ERROR state with message
  void triggerError(const char* reason);

  // ----------------------------------------------------------------
  // State queries
  // ----------------------------------------------------------------
  RobotState getState() const { return state_; }
  bool isIdle() const { return state_ == RobotState::IDLE; }
  bool isError() const { return state_ == RobotState::ERROR; }
  bool isBusy() const { return 
    state_ == RobotState::THROWING || 
    state_ == RobotState::RELOADING || 
    state_ == RobotState::BOOT || 
    state_ == RobotState::CALIBRATING ||
    state_ == RobotState::TRACKING;
  }
  
  // Get last error message (valid when in ERROR state)
  const char* getErrorMessage() const { return error_msg_; }

  // ----------------------------------------------------------------
  // Debug control
  // ----------------------------------------------------------------
  void setDebugStream(Stream* s) { dbg_ = s; }
  void setDebugEnabled(bool on) { dbg_enabled_ = on; }

private:
  // ----------------------------------------------------------------
  // State handlers
  // ----------------------------------------------------------------
  void enterState_(RobotState newState);
  void handleBoot_();
  void handleIdle_();
  void handleTracking_();
  void handleThrowing_();
  void handleReloading_();
  void handleCalibrating_();
  void handleCheckingBall_();
  void handleError_();

  // ----------------------------------------------------------------
  // Helper functions
  // ----------------------------------------------------------------
  bool homeHand_();

  bool moveHandToPosition_(float pos_rev);
  bool executeThrow_(float yaw_deg, float pitch_deg, float speed_mps);
  
  // Debug print helper
  void debugf_(const char* fmt, ...);

  // ----------------------------------------------------------------
  // Member variables
  // ----------------------------------------------------------------
  // Hardware references
  CanInterface&           can_;
  YawAxis&                yaw_;
  PitchAxis&              pitch_;
  HandTrajectoryStreamer& streamer_;
  HandPathPlanner&        planner_;
  
  // Configuration
  Config config_;
  
  // Current state
  RobotState state_ = RobotState::BOOT;
  
  // Timing
  uint32_t state_enter_ms_ = 0;       // When we entered current state
  uint32_t sub_state_ms_   = 0;       // For sub-state timing
  
  // State-specific variables
  uint8_t  homing_attempt_     = 0;
  uint8_t  homing_sub_state_   = 0;   // Sub-state within homing: 0=attempt, 1=waiting
  uint32_t homing_retry_ms_    = 0;   // Timestamp for retry delay
  uint8_t  reload_attempt_     = 0;
  uint8_t  reload_sub_state_   = 0;   // Sub-state within reload sequence
  uint8_t  ball_check_samples_collected_ = 0;  // Counter for ball check samples
  bool     ball_check_positive_   = false;     // True if any ball check was positive
  uint8_t  calibration_sub_state_ = 0;  // Sub-state within calibration
  uint32_t calibration_done_ms_   = 0;  // Timestamp when calibration pause started
  bool     throw_complete_        = false;
  uint32_t last_tracking_hand_pos_check_ms_ = 0; // Last time we checked the hand position during tracking
  
  // Ball-in-hand monitoring (CHECKING_BALL states)
  uint8_t  check_ball_sub_state_ = 0;        // Sub-state within CHECKING_BALL
  uint8_t  check_ball_samples_collected_ = 0; // Samples collected in CHECKING_BALL confirm phase
  
  bool reload_pending_ = false;

  // Pending throw request
  bool     throw_pending_          = false;
  float    pending_yaw_deg_        = 0;
  float    pending_pitch_deg_      = 0;
  float    pending_speed_mps_      = 0;
  uint64_t pending_throw_wall_us_  = 0;  // absolute wall-clock throw time (µs)
  
  // Tracking state
  TrackingCmd last_tracking_cmd_;
  uint32_t last_tracking_cmd_ms_ = 0;  // Time of last tracking command received
  uint32_t last_yaw_cmd_ms_   = 0;     // Rate-limiting: last yaw command forwarded
  uint32_t last_pitch_cmd_ms_ = 0;     // Rate-limiting: last pitch command forwarded
  
  // Trajectory buffer (owned here; used by executeThrow_, moveHandToPosition_, requestSmoothMove)
  std::vector<TrajFrame> traj_buffer_;

  // Error state
  char error_msg_[64] = {0};
  
  // Debug
  Stream* dbg_        = nullptr;
  bool    dbg_enabled_ = true;
};