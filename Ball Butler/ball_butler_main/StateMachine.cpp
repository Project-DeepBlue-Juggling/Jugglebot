/*
 * StateMachine.cpp - Robot state machine implementation
 * 
 * This module orchestrates all robot behavior through a finite state machine.
 * Each state has an entry action (enterState_) and an update handler.
 * 
 * DESIGN NOTES:
 *   - Non-blocking: update() must return quickly, no long delays
 *   - Uses millis() for all timing to keep loop() responsive
 *   - Throw requests are queued and executed when state permits
 *   - Reload sequence is broken into sub-states for non-blocking operation
 */

#include "StateMachine.h"
#include "CanInterface.h"
#include "YawAxis.h"
#include "PitchAxis.h"
#include "HandTrajectoryStreamer.h"
#include "HandPathPlanner.h"
#include "Proprioception.h"
#include "Trajectory.h"  // For HAND_MAX_SMOOTH_MOVE_POS
#include <vector>
#include <stdarg.h>

// External trajectory buffer (shared with main sketch)
extern std::vector<TrajFrame> g_traj_buffer;

// --------------------------------------------------------------------
// State name strings
// --------------------------------------------------------------------
const char* robotStateToString(RobotState s) {
  switch (s) {
    case RobotState::BOOT: return "BOOT";
    case RobotState::IDLE: return "IDLE";
    case RobotState::THROWING: return "THROWING";
    case RobotState::RELOADING: return "RELOADING";
    case RobotState::CALIBRATING: return "CALIBRATING";
    case RobotState::ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

// --------------------------------------------------------------------
// Constructor
// --------------------------------------------------------------------
StateMachine::StateMachine(CanInterface& can, YawAxis& yaw, PitchAxis& pitch,
                           HandTrajectoryStreamer& streamer, HandPathPlanner& planner)
  : can_(can), yaw_(yaw), pitch_(pitch), streamer_(streamer), planner_(planner) {}

// --------------------------------------------------------------------
// begin() - Initialize state machine
// --------------------------------------------------------------------
void StateMachine::begin() {
  state_ = RobotState::BOOT;
  state_enter_ms_ = millis();
  homing_attempt_ = 0;
  reload_attempt_ = 0;
  reload_pending_ = false;
  throw_pending_ = false;
  error_msg_[0]  = '\0';

  debugf_("[SM] State machine initialized, entering BOOT\n");
}

// --------------------------------------------------------------------
// update() - Main state machine update (call from loop())
// --------------------------------------------------------------------
void StateMachine::update() {
  switch (state_) {
    case RobotState::BOOT: handleBoot_(); break;
    case RobotState::IDLE: handleIdle_(); break;
    case RobotState::THROWING: handleThrowing_(); break;
    case RobotState::RELOADING: handleReloading_(); break;
    case RobotState::CALIBRATING: handleCalibrating_(); break;
    case RobotState::ERROR: handleError_(); break;
  }
}

// --------------------------------------------------------------------
// enterState_() - Common entry logic for state transitions
// --------------------------------------------------------------------
void StateMachine::enterState_(RobotState newState) {
  if (newState == state_) return;  // No change

  debugf_("[SM] %s -> %s\n\n", robotStateToString(state_), robotStateToString(newState));

  state_ = newState;
  state_enter_ms_ = millis();
  sub_state_ms_ = millis();

  // State-specific entry actions
  switch (newState) {
    case RobotState::BOOT:
      homing_attempt_ = 0;
      homing_sub_state_ = 0;
      homing_retry_ms_ = 0;
      break;

    case RobotState::IDLE:
      // Stow the pitch axis (90 degrees in IDLE mode)
      pitch_.setTargetDeg(90.0f);
      // Wait a brief moment to allow command to take effect
      delay(50);
      break;

    case RobotState::THROWING:
      throw_complete_ = false;
      // Put the pitch axis in CLOSED_LOOP_CONTROL mode
      can_.setRequestedState(config_.pitch_node_id, 8u);
      break;

    case RobotState::RELOADING:
      reload_attempt_ = 0;
      reload_sub_state_ = 0;
      // Put the pitch and hand axes in CLOSED_LOOP_CONTROL mode
      can_.setRequestedState(config_.pitch_node_id, 8u);
      can_.setRequestedState(config_.hand_node_id, 8u);
      break;

    case RobotState::CALIBRATING:
      // Only calibrate once the pitch axis is stowed
      pitch_.setTargetDeg(90.0f);
      // Wait a brief moment to allow command to take effect
      delay(50);
      
      // Get the current yaw axis accelerations
      yaw_.getAccel(config_.yaw_pre_calib_accel_, config_.yaw_pre_calib_decel_);
      // Set yaw accelerations to be slow for calibration
      yaw_.setAccel(config_.yaw_calib_accel_, config_.yaw_calib_decel_);

      // Move yaw to calibration angle
      yaw_.setTargetDeg(config_.calibrate_location_max_yaw_deg_);
      calibration_sub_state_ = 0;
      break;

    case RobotState::ERROR:
      // Error message should already be set by triggerError()
      break;
  }
}

// --------------------------------------------------------------------
// handleBoot_() - Homing and initialization state (non-blocking)
//
// Sub-states:
//   0: Ready to attempt homing
//   1: Waiting for retry delay after failed attempt
// --------------------------------------------------------------------
void StateMachine::handleBoot_() {
  const uint32_t elapsed = millis() - state_enter_ms_;

  // Check for overall timeout
  if (elapsed > config_.homing_timeout_ms) {
    triggerError("Homing timeout");
    return;
  }

  switch (homing_sub_state_) {
    case 0:  // Ready to attempt homing
      if (homing_attempt_ < config_.max_homing_attempts) {
        homing_attempt_++;
        debugf_("[SM] Homing attempt %d/%d\n", homing_attempt_, config_.max_homing_attempts);

        if (homeHand_()) {
          debugf_("[SM] Homing successful\n");

          // Configure yaw axis coordinate system:
          // - ZERO_OFFSET_DEG_ = encoder angle that corresponds to user 0°
          // - We want user range [0, 185] to be safely away from encoder 0/360
          // - Setting offset to 10° means: encoder 10° = user 0°, encoder 195° = user 185°
          // - Overshoot past user 0° gives negative user values (safe, no wrap)
          // - Overshoot past user 185° gives user values > 185° (safe, no wrap)
          // 
          // With soft limits [0, 185] and hard limit overshoot of 5°:
          // - Hard limit range is [-5°, 190°] in user coordinates
          // - In encoder space: [5°, 200°]
          // - The forbidden zone [200°, 5°] wraps around encoder 0/360 safely
          const float yaw_offset = config_.yaw_min_angle_deg;  // e.g., 5° or 10°
          yaw_.setZeroOffset(yaw_offset);
          yaw_.setSoftLimitsDeg(0.0f, config_.yaw_max_angle_deg - config_.yaw_min_angle_deg);
          
          debugf_("[SM] Yaw configured: offset=%.1f°, limits=[0, %.1f]°\n", 
                  yaw_offset, config_.yaw_max_angle_deg - config_.yaw_min_angle_deg);

          // Transition to IDLE
          enterState_(RobotState::IDLE);
          return;
        }

        // Homing failed - start retry delay
        debugf_("[SM] Homing attempt %d failed, waiting %lu ms before retry\n", 
                homing_attempt_, (unsigned long)config_.homing_retry_delay_ms);
        homing_sub_state_ = 1;
        homing_retry_ms_ = millis();
      } else {
        triggerError("Homing failed after max attempts");
      }
      break;

    case 1:  // Waiting for retry delay
      if (millis() - homing_retry_ms_ >= config_.homing_retry_delay_ms) {
        // Delay complete, go back to attempt state
        homing_sub_state_ = 0;
      }
      break;
  }
}

// --------------------------------------------------------------------
// handleIdle_() - Waiting for command
// --------------------------------------------------------------------
void StateMachine::handleIdle_() {
  // If there's no ball in hand, we should attempt to reload
  // if (!can_.isBallInHand()) {
  //   debugf_("[SM] No ball detected in hand while IDLE, initiating reload\n");
  //   enterState_(RobotState::RELOADING);
  //   return;
  // }
  
  // Check if we have a pending throw request
  if (throw_pending_) {
    throw_pending_ = false;

    // Validate we have a ball in hand before throwing
    if (!can_.isBallInHand()) {
      debugf_("[SM] WARNING: Throw requested but no ball detected. Exiting.\n");
      return;
    }

    // Execute the throw
    if (executeThrow_(pending_yaw_deg_, pending_pitch_deg_,
                      pending_speed_mps_, pending_in_s_)) {
      enterState_(RobotState::THROWING);
    } else {
      debugf_("[SM] Throw execution failed, staying in IDLE\n");
    }
  }

  if (reload_pending_) {
    reload_pending_ = false;

    // Begin the reloading process
    enterState_(RobotState::RELOADING);
    debugf_("[SM] Reloading...");
  }

  // If no throw command received for a while, stow the pitch axis
  // This handles the case where the host stops sending tracking commands
  const uint32_t last_cmd_ms = can_.getLastHostCmdMs();
  if (last_cmd_ms > 0 && (millis() - last_cmd_ms) > config_.idle_no_cmd_timeout_ms) {
    // Only send stow command if pitch is not already near stow position
    if (PRO.getPitchDeg() < config_.pitch_min_stow_angle_deg) {
      pitch_.setTargetDeg(90.0f);
    }
  }

  // If the pitch axis is at or above `config_.pitch_min_stow_angle_deg` (and at its target) and not currently IDLE, 
  // we can put it in IDLE mode to save power.
  // Any pitch angle lower than around 70 degrees will cause the axis to fight gravity, so we can only go IDLE when stowed
  CanInterface::AxisHeartbeat hb_pitch;
  if (can_.getAxisHeartbeat(config_.pitch_node_id, hb_pitch)) {
    if (hb_pitch.axis_state == 8u && hb_pitch.trajectory_done && PRO.getPitchDeg() >= config_.pitch_min_stow_angle_deg) {
      can_.setRequestedState(config_.pitch_node_id, 1u);  // IDLE
    } 

    // However, if the angle is ever LOWER than the stow threshold, we must ensure it's back in CLOSED_LOOP_CONTROL
    else if (PRO.getPitchDeg() < config_.pitch_min_stow_angle_deg && hb_pitch.axis_state != 8u) {
      can_.setRequestedState(config_.pitch_node_id, 8u);  // CLOSED_LOOP_CONTROL
    }
  }
}

// --------------------------------------------------------------------
// handleThrowing_() - Executing throw trajectory
// --------------------------------------------------------------------
void StateMachine::handleThrowing_() {
  // Check if throw is complete (streamer finished)
  if (!streamer_.isActive()) {
    if (!throw_complete_) {
      throw_complete_ = true;
      sub_state_ms_ = millis();  // Start post-throw delay timer
      debugf_("[SM] Throw complete, waiting %.1f sec before reload\n",
              config_.post_throw_delay_ms / 1000.0f);
    }

    // Wait for post-throw delay before transitioning to reload
    if (millis() - sub_state_ms_ >= config_.post_throw_delay_ms) {
      enterState_(RobotState::RELOADING);
    }
  }
}

// --------------------------------------------------------------------
// handleReloading_() - Reload sequence (non-blocking sub-state machine)
//
// Sub-states:
//   0: Move hand to top position
//   1: Move pitch to grab angle
//   2: Wait for pitch and hand to settle
//   3: Move yaw to pickup position
//   4: Wait for yaw settle
//   5: Move pitch to ready angle (grab ball)
//   6: Wait for pitch settle and check for ball in hand
//   7: Check for ball in hand and handle retries
//   8: Move yaw back to home
//   9: Wait for yaw settle
//  10: Wait for hand to reach bottom position
// --------------------------------------------------------------------
void StateMachine::handleReloading_() {
  const uint32_t elapsed = millis() - state_enter_ms_;
  const uint32_t sub_elapsed = millis() - sub_state_ms_;

  // Min duration of pitch/hand movements before checking done (can take a moment to start moving and update traj_done)
  const uint32_t min_duration_ms = 300; 

  CanInterface::AxisHeartbeat hb_hand;
  CanInterface::AxisHeartbeat hb_pitch;

  float current_yaw = PRO.getYawDeg();

  // Check for overall timeout
  if (elapsed > config_.reload_timeout_ms) {
    triggerError("Reload sequence timeout");
    return;
  }

  switch (reload_sub_state_) {
    case 0:  // Move hand to top position
      debugf_("[SM] Reload: Moving hand to top (%.2f rev)\n", config_.reload_hand_top_rev);
      moveHandToPosition_(config_.reload_hand_top_rev);
      reload_sub_state_ = 1;
      sub_state_ms_ = millis();
      break;

    case 1:  // Move pitch to grab angle
      debugf_("[SM] Reload: Pitching to grab angle (%.1f°)\n", config_.reload_pitch_ready_deg);
      pitch_.setTargetDeg(config_.reload_pitch_ready_deg);
      reload_sub_state_ = 2;
      sub_state_ms_ = millis();
      break;

    case 2:  // Wait for pitch and hand to settle
      if (sub_elapsed < min_duration_ms) {
        break;  // Ensure minimum duration
      }
      if (can_.getAxisHeartbeat(config_.pitch_node_id, hb_pitch) && can_.getAxisHeartbeat(config_.hand_node_id, hb_hand)) {
        // debugf_("[SM] Reload: Pitch traj_done=%d, Hand traj_done=%d\n",
        //         hb_pitch.trajectory_done, hb_hand.trajectory_done);
        if (hb_pitch.trajectory_done && hb_hand.trajectory_done) {
          // Put hand in CLOSED_LOOP_CONTROL to hold position
          can_.setRequestedState(config_.hand_node_id, 8u);
          reload_sub_state_ = 3;
          sub_state_ms_ = millis();
        }
      }
      break;

    case 3:  // Move yaw to pickup position
      debugf_("[SM] Reload: Yawing to pickup (%.1f°)\n", config_.reload_yaw_angle_deg);
      yaw_.setTargetDeg(config_.reload_yaw_angle_deg);
      reload_sub_state_ = 4;
      sub_state_ms_ = millis();
      break;

    case 4:  // Wait for yaw settle
      // Log the current yaw position for debugging
      // debugf_("[SM] Reload: Current yaw=%.2f°, Target yaw=%.2f°\n", current_yaw, config_.reload_yaw_angle_deg);
      if (abs(current_yaw - config_.reload_yaw_angle_deg) < config_.yaw_angle_threshold_deg) {
        reload_sub_state_ = 5;
        sub_state_ms_ = millis();
      }
      break;

    case 5:  // Move pitch to ready angle (should grab ball from hopper)
      debugf_("[SM] Reload: Pitching to grab position (%.1f°)\n", config_.reload_pitch_grab_deg);
      pitch_.setTargetDeg(config_.reload_pitch_grab_deg);
      reload_sub_state_ = 6;
      sub_state_ms_ = millis();
      break;

    case 6:  // Wait for pitch settle
      if (sub_elapsed < min_duration_ms) {
        break;  // Ensure minimum duration
      }
      if (can_.getAxisHeartbeat(config_.pitch_node_id, hb_pitch) && hb_pitch.trajectory_done) {
        // Pitch has settled! Move to next sub-state
        reload_sub_state_ = 7;
        sub_state_ms_ = millis();
      }
      break;

    case 7: // Check for ball in hand
      if (can_.isBallInHand()) {
        // Success! Continue with reload sequence
        reload_sub_state_ = 8;
      } else {
        // No ball - retry if attempts remaining
        reload_attempt_++;
        if (reload_attempt_ < config_.max_reload_attempts) {
          debugf_("[SM] Reload attempt %d/%d failed, retrying\n",
                  reload_attempt_, config_.max_reload_attempts);
          reload_sub_state_ = 0;  // Restart sequence
          sub_state_ms_ = millis();
          break;
        } else {
          // Reset positions and trigger error
          moveHandToPosition_(config_.hand_rev_home);
          pitch_.setTargetDeg(config_.pitch_deg_home);
          yaw_.setTargetDeg(config_.yaw_deg_home);

          triggerError("No ball after max reload attempts");
        }
      }

    case 8:  // Move hand back to home
      debugf_("[SM] Reload: Yawing to home (%.1f°)\n", config_.reload_yaw_home_deg);
      yaw_.setTargetDeg(config_.reload_yaw_home_deg);
      reload_sub_state_ = 9;
      sub_state_ms_ = millis();
      break;

    case 9:  // Wait for yaw settle and send hand to bottom
      if (abs(current_yaw - config_.reload_yaw_home_deg) < config_.yaw_angle_threshold_deg) {
        moveHandToPosition_(config_.reload_hand_bottom_rev);  // Move hand to safe position

        // Wait for hand to reach bottom before finishing
        reload_sub_state_ = 10;
        sub_state_ms_ = millis();
      }
      break;

    case 10:  // Wait for hand to reach bottom
      // Determine whether the hand is at the bottom by whether its in IDLE
      if (can_.getAxisHeartbeat(config_.hand_node_id, hb_hand)) {
        if (hb_hand.axis_state == 1u) {  // IDLE
          debugf_("[SM] Reload complete, ball in hand: %s\n", can_.isBallInHand() ? "YES" : "NO");
          enterState_(RobotState::IDLE);
        }
      }
      break;
  }
}

void StateMachine::handleCalibrating_() {
  // Calibration sequence:
  //   0: Wait for yaw to reach calibration position
  //   1: Move back to home position
  //   2: Wait for yaw to reach home, then pause
  //   3: Pause complete, restore settings and finish
  
  const uint32_t sub_elapsed = millis() - sub_state_ms_;
  const float current_yaw = PRO.getYawDeg();
  
  switch (calibration_sub_state_) {
    case 0:  // Wait for yaw to reach calibration position
      if (fabsf(current_yaw - config_.calibrate_location_max_yaw_deg_) < config_.yaw_angle_threshold_deg) {
        debugf_("[SM] Calibration: Reached calibration position (%.1f°), returning home...\n", current_yaw);
        yaw_.setTargetDeg(config_.calibrate_location_min_yaw_deg_);
        calibration_sub_state_ = 1;
      }
      break;
      
    case 1:  // Wait for yaw to reach home
      if (fabsf(current_yaw - config_.calibrate_location_min_yaw_deg_) < config_.yaw_angle_threshold_deg) {
        debugf_("[SM] Calibration: Reached home (%.1f°), pausing for %lu ms...\n", 
                current_yaw, (unsigned long)config_.calibration_pause_ms);
        sub_state_ms_ = millis();
        calibration_sub_state_ = 2;
      }
      break;
      
    case 2:  // Pause at home position
      if (sub_elapsed >= config_.calibration_pause_ms) {
        // Restore yaw axis accelerations
        yaw_.setAccel(config_.yaw_pre_calib_accel_, config_.yaw_pre_calib_decel_);
        debugf_("[SM] Calibration complete, returning to IDLE\n");
        enterState_(RobotState::IDLE);
      }
      break;
  }
}

// --------------------------------------------------------------------
// handleError_() - Error state (waiting for manual reset)
// --------------------------------------------------------------------
void StateMachine::handleError_() {
  // Just sit here until reset() is called
  // Could add periodic error message printing if desired
}

// --------------------------------------------------------------------
// State change requesters
// --------------------------------------------------------------------
// requestReload() - Queue a reload request
bool StateMachine::requestReload() {
  // Only accept reload commands when in IDLE state
  if (state_ != RobotState::IDLE) {
    debugf_("[SM] Reload rejected: not in IDLE state (current: %s)\n",
            robotStateToString(state_));
    return false;
  }

  // Updaing pending flag
  reload_pending_ = true;

  debugf_("[SM] Reload queued");

  return true;
}

// requestThrow() - Queue a throw request
bool StateMachine::requestThrow(float yaw_deg, float pitch_deg, float speed_mps, float in_s) {
  // Only accept throws when in IDLE state and if there is a ball in hand
  if (state_ != RobotState::IDLE) {
    debugf_("[SM] Throw rejected: not in IDLE state (current: %s)\n",
            robotStateToString(state_));
    return false;
  }
  if (!can_.isBallInHand()) {
    debugf_("[SM] Throw rejected: no ball in hand\n");
    return false;
  }

  // Store pending throw parameters
  throw_pending_ = true;
  pending_yaw_deg_ = yaw_deg;
  pending_pitch_deg_ = pitch_deg;
  pending_speed_mps_ = speed_mps;
  pending_in_s_ = in_s;

  debugf_("[SM] Throw queued: yaw=%.1f° pitch=%.1f° speed=%.2f m/s in=%.2f s\n",
          yaw_deg, pitch_deg, speed_mps, in_s);

  return true;
}

bool StateMachine::requestCalibrateLocation() {
  // Only accept calibration requests when in IDLE state
  if (state_ != RobotState::IDLE) {
    debugf_("[SM] Calibration rejected: not in IDLE state (current: %s)\n",
            robotStateToString(state_));
    return false;
  }

  debugf_("[SM] Calibration requested\n");

  enterState_(RobotState::CALIBRATING);
  return true;
}


// --------------------------------------------------------------------
// executeThrow_() - Actually execute the throw (called from handleIdle_)
// --------------------------------------------------------------------
bool StateMachine::executeThrow_(float yaw_deg, float pitch_deg,
                                 float speed_mps, float in_s) {
  // 1) Point yaw & pitch
  yaw_.setTargetDeg(yaw_deg);
  pitch_.setTargetDeg(pitch_deg);

  debugf_("[SM] Executing throw: yaw=%.1f° pitch=%.1f° speed=%.2f m/s\n",
          yaw_deg, pitch_deg, speed_mps);

  // 2) Check hand is homed
  if (!can_.isAxisHomed(config_.hand_node_id)) {
    debugf_("[SM] Throw rejected: hand not homed\n");
    return false;
  }

  // 3) Get fresh PV snapshot
  float pos_rev = 0, vel_rps = 0;
  uint64_t t_wall_us = 0;
  if (!can_.getAxisPV(config_.hand_node_id, pos_rev, vel_rps, t_wall_us)) {
    debugf_("[SM] Throw rejected: PV unavailable\n");
    return false;
  }

  // Check PV freshness (20ms max age)
  if (can_.wallTimeUs() - t_wall_us > 20000) {
    debugf_("[SM] Throw rejected: PV stale\n");
    return false;
  }

  // 4) Plan the throw
  auto plan = planner_.planThrowDecelZero(speed_mps, pos_rev, vel_rps,
                                          (uint32_t)(t_wall_us & 0xFFFFFFFFu));
  if (plan.trajectory.empty()) {
    debugf_("[SM] Throw rejected: planner returned empty trajectory\n");
    return false;
  }

  // 5) Check lead time
  float min_ts = 0.0f;
  for (const auto& f : plan.trajectory) {
    if (f.t_s < min_ts) min_ts = f.t_s;
  }
  const float required_lead_s = -min_ts + 0.1f;  // 100ms margin
  if (in_s < required_lead_s) {
    debugf_("[SM] Throw rejected: insufficient lead time (need %.2f s, got %.2f s)\n",
            required_lead_s, in_s);
    return false;
  }

  // 6) Arm the streamer
  g_traj_buffer = std::move(plan.trajectory);
  const float throw_wall_s = can_.wallTimeUs() * 1e-6f + in_s;

  bool ok = streamer_.arm(config_.hand_node_id, g_traj_buffer.data(),
                          g_traj_buffer.size(), throw_wall_s);

  debugf_("[SM] Throw armed: frames=%u, ready_time=%.2f s, %s\n",
          (unsigned)g_traj_buffer.size(), planner_.lastTimeToReadyS(),
          ok ? "OK" : "FAIL");

  return ok;
}


// --------------------------------------------------------------------
// reset() - Manual reset from ERROR state
// --------------------------------------------------------------------
void StateMachine::reset() {
  if (state_ != RobotState::ERROR) {
    debugf_("[SM] Reset ignored: not in ERROR state\n");
    return;
  }

  debugf_("[SM] Manual reset requested\n");
  error_msg_[0] = '\0';
  enterState_(RobotState::BOOT);
}

// --------------------------------------------------------------------
// triggerError() - Force transition to ERROR state
// --------------------------------------------------------------------
void StateMachine::triggerError(const char* reason) {
  strncpy(error_msg_, reason, sizeof(error_msg_) - 1);
  error_msg_[sizeof(error_msg_) - 1] = '\0';

  debugf_("[SM] ERROR: %s\n", error_msg_);

  enterState_(RobotState::ERROR);
}

// --------------------------------------------------------------------
// homeHand_() - Home hand axis (yaw/pitch don't need homing)
// --------------------------------------------------------------------
bool StateMachine::homeHand_() {
  // Hand axis needs homing (end-stop current spike detection)
  debugf_("[SM] Homing hand axis (node %d)...\n", config_.hand_node_id);

  bool ok = can_.homeHandStandard(config_.hand_node_id);

  if (!ok) {
    debugf_("[SM] Hand homing failed\n");
    return false;
  }

  debugf_("[SM] Hand homed successfully\n");
  return true;
}

// --------------------------------------------------------------------
// moveHandToPosition_() - Command hand to a position via smooth move
// --------------------------------------------------------------------
bool StateMachine::moveHandToPosition_(float target_rev) {
  // Clamp to valid range
  if (target_rev < 0) target_rev = 0;
  if (target_rev > HAND_MAX_SMOOTH_MOVE_POS) target_rev = HAND_MAX_SMOOTH_MOVE_POS;

  // Get current PV
  float pos_rev = 0, vel_rps = 0;
  uint64_t t_us = 0;
  if (!can_.getAxisPV(config_.hand_node_id, pos_rev, vel_rps, t_us)) {
    debugf_("[SM] moveHand: PV unavailable\n");
    return false;
  }

  // Plan smooth move
  auto plan = planner_.planSmoothTo(target_rev, pos_rev, vel_rps,
                                    (uint32_t)(t_us & 0xFFFFFFFFu));
  if (plan.trajectory.empty()) {
    // Already at target
    return true;
  }

  // Arm streamer
  g_traj_buffer = std::move(plan.trajectory);
  const float time_offset_s = can_.wallTimeUs() * 1e-6f;

  return streamer_.arm(config_.hand_node_id, g_traj_buffer.data(),
                       g_traj_buffer.size(), time_offset_s);
}

// --------------------------------------------------------------------
// debugf_() - Debug print helper (non-blocking safe version)
// --------------------------------------------------------------------
void StateMachine::debugf_(const char* fmt, ...) {
  if (!dbg_ || !dbg_enabled_) return;
  
  // Safety check: if this is Serial, do extensive checks to prevent blocking
  // This prevents hangs when USB is connected but no terminal is open
  if (dbg_ == &Serial) {
    // Check if we're in interrupt context (VECTACTIVE field, bits 0-8)
    uint32_t icsr = SCB_ICSR;
    if ((icsr & 0x1FF) != 0) {
      return;  // In ISR - never print
    }
    
    // Check USB enumeration status
    extern volatile uint8_t usb_configuration;
    if (!usb_configuration) {
      return;  // USB not enumerated
    }
    
    // Check Serial ready and buffer space
    if (!Serial || Serial.availableForWrite() < 64) {
      return;  // Skip output to prevent blocking
    }
  }

  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  dbg_->print(buf);
}