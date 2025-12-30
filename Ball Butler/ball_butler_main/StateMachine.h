#pragma once
/*
 * StateMachine.h - Robot state machine for Ball Butler
 * 
 * States:
 *   BOOT      - Homing all axes, checking for ball in hand
 *   IDLE      - Awaiting throw command (ball should be in hand)
 *   THROWING  - Executing throw trajectory
 *   RELOADING - Running reload sequence to grab next ball
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

// Forward declarations
class CanInterface;
class YawAxis;
class PitchAxis;
class HandTrajectoryStreamer;
class HandPathPlanner;

// --------------------------------------------------------------------
// State enumeration
// --------------------------------------------------------------------
enum class RobotState : uint8_t {
  BOOT      = 0,  // Homing and initialization
  IDLE      = 1,  // Ready and waiting for command
  THROWING  = 2,  // Executing throw
  RELOADING = 3,  // Grabbing next ball
  ERROR     = 4   // Error state
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
  struct Config {
    // Timeouts
    uint32_t homing_timeout_ms     = 10000;  // 10 seconds for homing
    uint32_t homing_retry_delay_ms = 500;   // Delay between homing attempts
    uint32_t reload_timeout_ms     = 15000;  // 15 seconds for reload sequence
    uint32_t post_throw_delay_ms   = 1000;   // 1 second delay after throw before reload
    
    // Reload sequence positions
    float reload_hand_top_rev     = 8.7f;   // Hand position for reload
    float reload_hand_bottom_rev  = 0.0f;   // Hand position at bottom of stroke
    float reload_hand_bottom_tolerance_rev = 0.1f; // Tolerance for considering hand at bottom position
    float reload_pitch_ready_deg  = 70.0f;  // Pitch angle before grabbing ball
    float reload_pitch_grab_deg   = 90.0f;  // Pitch angle to grab ball
    float reload_yaw_angle_deg    = 180.0f; // Yaw angle for ball pickup
    float reload_yaw_home_deg     = 15.0f;  // Yaw home position
    uint32_t reload_hold_delay_ms = 500;    // Time to wait to check that the ball is in-hand

    // Positions to go to if reload fails
    float reload_fail_hand_rev    = 0.0f;   // Hand position on failure
    float reload_fail_pitch_deg   = 90.0f;  // Pitch angle on failure
    float reload_fail_yaw_deg     = 0.0f;   // Yaw angle on failure
    
    // Ball detection 
    uint8_t ball_detect_gpio_pin = 3;       // GPIO pin on hand ODrive for ball detection
    uint32_t ball_check_interval_ms = 200;  // How often to check for ball in hand
    
    // Retry limits
    uint8_t max_reload_attempts      = 3;      // Max attempts before ERROR
    uint8_t max_homing_attempts      = 3;      // Max homing attempts
    
    // Axis settle times (ms to wait after commanding position) and positions
    uint32_t pitch_settle_ms       = 500;
    uint32_t pitch_grab_settle_ms  = 1000; // Settle time when 'grabbing' the next ball
    float yaw_angle_threshold_deg  = 1.0f; // Threshold to consider yaw at target angle

    // Axis limits
    float pitch_min_stow_angle_deg = 80.0f; // Min pitch angle to consider stowed (for IDLE power saving)
    float yaw_min_angle_deg        = 5.0f;  // Min yaw angle
    float yaw_max_angle_deg        = 185.0f + yaw_min_angle_deg; // Max yaw angle

    // Node IDs
    uint8_t hand_node_id  = 8;
    uint8_t pitch_node_id = 7;
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
  bool requestThrow(float yaw_deg, float pitch_deg, float speed_mps, float in_s);
  bool requestReload();
  bool checkBallInHand_();
  
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
  bool isBusy() const { return state_ == RobotState::THROWING || state_ == RobotState::RELOADING; }
  
  // Get last error message (valid when in ERROR state)
  const char* getErrorMessage() const { return error_msg_; }
  
  // Check if ball is currently detected in hand
  bool isBallInHand() const { return ball_in_hand_; }

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
  void handleThrowing_();
  void handleReloading_();
  void handleError_();

  // ----------------------------------------------------------------
  // Helper functions
  // ----------------------------------------------------------------
  bool homeHand_();

  bool moveHandToPosition_(float pos_rev);
  bool executeThrow_(float yaw_deg, float pitch_deg, float speed_mps, float in_s);
  
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
  uint32_t last_ball_check_ms_ = 0;   // Last time we checked for ball in hand
  bool     ball_in_hand_       = false;
  bool     last_ball_in_hand_  = false;
  bool     throw_complete_     = false;
  
  bool reload_pending_ = false;

  // Pending throw request
  bool  throw_pending_    = false;
  float pending_yaw_deg_  = 0;
  float pending_pitch_deg_ = 0;
  float pending_speed_mps_ = 0;
  float pending_in_s_     = 0;
  
  // Error state
  char error_msg_[64] = {0};
  
  // Debug
  Stream* dbg_        = nullptr;
  bool    dbg_enabled_ = true;
};