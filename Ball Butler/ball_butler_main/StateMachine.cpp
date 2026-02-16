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
#include "BallButlerConfig.h"
#include "CanInterface.h"
#include "YawAxis.h"
#include "PitchAxis.h"
#include "HandTrajectoryStreamer.h"
#include "HandPathPlanner.h"
#include "Proprioception.h"
#include "Trajectory.h"  // For HAND_MAX_SMOOTH_MOVE_POS
#include <stdarg.h>

// --------------------------------------------------------------------
// State name strings
// --------------------------------------------------------------------
const char* robotStateToString(RobotState s) {
  switch (s) {
    case RobotState::BOOT: return "BOOT";
    case RobotState::IDLE: return "IDLE";
    case RobotState::TRACKING: return "TRACKING";
    case RobotState::THROWING: return "THROWING";
    case RobotState::RELOADING: return "RELOADING";
    case RobotState::CALIBRATING: return "CALIBRATING";
    case RobotState::CHECKING_BALL: return "CHECKING_BALL";
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
  
  // Initialize tracking state
  last_tracking_cmd_ = TrackingCmd{};
  last_tracking_cmd_ms_ = 0;

  debugf_("[SM] State machine initialized, entering BOOT\n");
}

// --------------------------------------------------------------------
// update() - Main state machine update (call from loop())
// --------------------------------------------------------------------
void StateMachine::update() {
  switch (state_) {
    case RobotState::BOOT: handleBoot_(); break;
    case RobotState::IDLE: handleIdle_(); break;
    case RobotState::TRACKING: handleTracking_(); break;
    case RobotState::THROWING: handleThrowing_(); break;
    case RobotState::RELOADING: handleReloading_(); break;
    case RobotState::CALIBRATING: handleCalibrating_(); break;
    case RobotState::CHECKING_BALL: handleCheckingBall_(); break;
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
      // Stow the pitch axis
      pitch_.setTargetDeg(config_.pitch_deg_home);
      break;

    case RobotState::TRACKING:
      // Put the pitch axis in CLOSED_LOOP_CONTROL mode for tracking
      can_.setRequestedState(config_.pitch_node_id, ODriveState::CLOSED_LOOP);
      // Initialize tracking timestamp
      last_tracking_cmd_ms_ = millis();
      debugf_("[SM] Entering TRACKING mode\n");
      break;

    case RobotState::THROWING:
      throw_complete_ = false;
      // Put the pitch axis in CLOSED_LOOP_CONTROL mode
      can_.setRequestedState(config_.pitch_node_id, ODriveState::CLOSED_LOOP);
      break;

    case RobotState::RELOADING:
      reload_attempt_ = 0;
      reload_sub_state_ = 0;
      ball_check_samples_collected_ = 0;
      ball_check_positive_ = false;
      // Put the pitch and hand axes in CLOSED_LOOP_CONTROL mode
      can_.setRequestedState(config_.pitch_node_id, ODriveState::CLOSED_LOOP);
      can_.setRequestedState(config_.hand_node_id, ODriveState::CLOSED_LOOP);
      break;

    case RobotState::CALIBRATING:
      // Only calibrate once the pitch axis is stowed
      pitch_.setTargetDeg(config_.pitch_deg_home);

      // Get the current yaw axis accelerations
      yaw_.getAccel(config_.yaw_pre_calib_accel, config_.yaw_pre_calib_decel);
      // Set yaw accelerations to be slow for calibration
      yaw_.setAccel(config_.yaw_calib_accel, config_.yaw_calib_decel);

      // Move yaw to calibration angle
      yaw_.setTargetDeg(config_.calibrate_location_max_yaw_deg);
      calibration_sub_state_ = 0;
      break;

    case RobotState::CHECKING_BALL:
      // Enter CHECKING_BALL state to verify ball presence
      check_ball_sub_state_ = 0;
      check_ball_samples_collected_ = 0;
      // Put pitch in CLOSED_LOOP_CONTROL mode for movement
      can_.setRequestedState(config_.pitch_node_id, ODriveState::CLOSED_LOOP);
      debugf_("[SM] Entering CHECKING_BALL - disrupting to verify ball presence\n");
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
                      pending_speed_mps_)) {
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

  // If the pitch axis is at or above `config_.pitch_min_stow_angle_deg` (and at its target) and not currently IDLE,
  // we can put it in IDLE mode to save power.
  // Any pitch angle lower than around 70 degrees will cause the axis to fight gravity, so we can only go IDLE when stowed
  // Don't idle if a command was sent recently (prevents race condition with Serial/external commands)
  const bool recent_command = (millis() - pitch_.lastCommandMs()) < SMDefaults::PITCH_IDLE_DELAY_MS;
  
  CanInterface::AxisHeartbeat hb_pitch;
  if (can_.getAxisHeartbeat(config_.pitch_node_id, hb_pitch)) {
    if (!recent_command && hb_pitch.axis_state == ODriveState::CLOSED_LOOP && hb_pitch.trajectory_done && PRO.getPitchDeg() >= config_.pitch_min_stow_angle_deg) {
      can_.setRequestedState(config_.pitch_node_id, ODriveState::IDLE);
    }

    // However, if the angle is ever LOWER than the stow threshold, we must ensure it's back in CLOSED_LOOP_CONTROL
    else if (PRO.getPitchDeg() < config_.pitch_min_stow_angle_deg && hb_pitch.axis_state != ODriveState::CLOSED_LOOP) {
      can_.setRequestedState(config_.pitch_node_id, ODriveState::CLOSED_LOOP);
    }
  }
}

// --------------------------------------------------------------------
// handleTracking_() - Following target (speed=0 commands)
//
// In this state, the robot follows yaw/pitch targets sent by the host
// with throw_speed=0. This allows the host to aim the robot before
// actually throwing.
//
// Transitions:
//   - TRACKING -> IDLE: No tracking command received for idle_no_cmd_timeout_ms
//   - TRACKING -> THROWING: A throw command with speed > 0 is received
// --------------------------------------------------------------------
void StateMachine::handleTracking_() {
  // Check for timeout - return to IDLE if no tracking commands received
  const uint32_t elapsed_since_cmd = millis() - last_tracking_cmd_ms_;
  if (elapsed_since_cmd > config_.idle_no_cmd_timeout_ms) {
    debugf_("[SM] Tracking timeout (no cmd for %lu ms), returning to IDLE\n",
            (unsigned long)elapsed_since_cmd);
    enterState_(RobotState::IDLE);
    return;
  }

  // Periodically check the hand position. If it's outside of the threshold, command it to move back to the bottom position
  if (millis() - last_tracking_hand_pos_check_ms_ >= config_.tracking_hand_pos_check_interval_ms) {
    last_tracking_hand_pos_check_ms_ = millis();
    float hand_pos = PRO.getHandPosRev();
    if (hand_pos < config_.reload_hand_bottom_rev - config_.reload_hand_bottom_tolerance_rev ||
        hand_pos > config_.reload_hand_bottom_rev + config_.reload_hand_bottom_tolerance_rev) {
      debugf_("[SM] Tracking: Hand position %.2f rev outside of %.2f ± %.2f rev, moving back to bottom\n",
              hand_pos, config_.reload_hand_bottom_rev, config_.reload_hand_bottom_tolerance_rev);
      moveHandToPosition_(config_.reload_hand_bottom_rev);
    }
  }

  // Process pending throw request (if a throw with speed > 0 came in)
  if (throw_pending_) {
    throw_pending_ = false;

    // Validate we have a ball in hand before throwing
    if (!can_.isBallInHand()) {
      debugf_("[SM] WARNING: Throw requested but no ball detected. Returning to IDLE.\n");
      enterState_(RobotState::IDLE);
      return;
    }

    // Execute the throw
    if (executeThrow_(pending_yaw_deg_, pending_pitch_deg_,
                      pending_speed_mps_)) {
      enterState_(RobotState::THROWING);
    } else {
      debugf_("[SM] Throw execution failed, returning to IDLE\n");
      enterState_(RobotState::IDLE);
    }
    return;
  }

  // Apply the latest tracking command (yaw/pitch updates)
  if (last_tracking_cmd_.valid) {
    yaw_.setTargetDeg(last_tracking_cmd_.yaw_deg);
    pitch_.setTargetDeg(last_tracking_cmd_.pitch_deg);
    last_tracking_cmd_.valid = false;  // Mark as consumed
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
//   5: Move pitch to grab position (should grab ball from hopper)
//   6: Wait for pitch settle
//   7: Move pitch back to ready angle before checking for ball in hand
//   8: Await pitch arrival
//   9: Check for ball in hand (wait for multiple samples)
//   10: Move all axes to success positions (hand down, yaw home)
//   11: Wait for hand and pitch to reach final positions, then transition to IDLE
// --------------------------------------------------------------------
void StateMachine::handleReloading_() {
  const uint32_t elapsed = millis() - state_enter_ms_;
  const uint32_t sub_elapsed = millis() - sub_state_ms_;

  // Min duration of pitch/hand movements before checking done (can take a moment to start moving and update traj_done)
  const uint32_t min_duration_ms = SMDefaults::MIN_MOTION_DURATION_MS;

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
          can_.setRequestedState(config_.hand_node_id, ODriveState::CLOSED_LOOP);
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

    case 5:  // Move pitch to grab angle (should grab ball from hopper)
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

    case 7: // Move pitch back to ready angle before checking for ball in hand
      // This leads to more reliable detection of the ball in hand
      debugf_("[SM] Reload: Pitching back to ready angle (%.1f°)\n", config_.reload_pitch_ready_deg);
      pitch_.setTargetDeg(config_.reload_pitch_ready_deg);
      reload_sub_state_ = 8;
      sub_state_ms_ = millis();
      break;

    case 8: // Await pitch arrival
      // Await pitch arrival
      if (sub_elapsed < min_duration_ms) {
        break;  // Ensure minimum duration
      }

      if (can_.getAxisHeartbeat(config_.pitch_node_id, hb_pitch) && hb_pitch.trajectory_done) {
        // Pitch has settled, now we can check for ball in hand
        reload_sub_state_ = 9;
        sub_state_ms_ = millis();
      }
      break;

    case 9: // Check for ball in hand (wait for multiple samples)
      // Initialize sample collection on first entry to this sub-state
      if (ball_check_samples_collected_ == 0 && !ball_check_positive_) {
        // Fresh entry - reset tracking
        ball_check_positive_ = false;
        Serial.printf("[SM] Reload case 9: Checking for ball in hand, collecting %d samples...\n", config_.reload_ball_check_samples);
      }

      // Check current sample
      if (can_.isBallInHand()) {
        ball_check_positive_ = true;
      }
      ball_check_samples_collected_++;

      // If any sample was positive, success!
      if (ball_check_positive_) {
        debugf_("[SM] Ball detected after %d samples\n", ball_check_samples_collected_);
        ball_check_samples_collected_ = 0;
        ball_check_positive_ = false;
        reload_sub_state_ = 10;  // Move to next sub-state to return home
        sub_state_ms_ = millis();
        break;
      }

      // If we haven't collected enough samples yet, wait for more
      if (ball_check_samples_collected_ < config_.reload_ball_check_samples) {
        break;  // Stay in this sub-state, collect more samples
      }

      // All samples collected, none positive - retry if attempts remaining
      ball_check_samples_collected_ = 0;
      ball_check_positive_ = false;
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
      break;

    case 10:  // Move all axes to success positions (hand down, yaw home)
      debugf_("[SM] Reload: Yawing to home (%.1f°), moving hand to bottom (%.1f rev), pitching to home (%.1f°)\n", 
        config_.yaw_deg_home, config_.hand_rev_home, config_.pitch_deg_home);
      yaw_.setTargetDeg(config_.yaw_deg_home);
      moveHandToPosition_(config_.hand_rev_home);
      pitch_.setTargetDeg(config_.pitch_deg_home);
      reload_sub_state_ = 11;
      sub_state_ms_ = millis();
      break;

    case 11:  // Wait for hand and pitch to reach final positions. Don't require yaw to be home before accepting tracking/throwing targets
      // Determine whether the hand is at the bottom from the proprioception reading
      const float hand_pos = PRO.getHandPosRev();
      const bool hand_at_bottom = (hand_pos >= config_.reload_hand_bottom_rev - config_.reload_hand_bottom_tolerance_rev) &&
                                 (hand_pos <= config_.reload_hand_bottom_rev + config_.reload_hand_bottom_tolerance_rev);
      if (hand_at_bottom && can_.getAxisHeartbeat(config_.pitch_node_id, hb_pitch) && hb_pitch.trajectory_done) {
        enterState_(RobotState::IDLE);
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
      if (fabsf(current_yaw - config_.calibrate_location_max_yaw_deg) < config_.yaw_angle_threshold_deg) {
        debugf_("[SM] Calibration: Reached calibration position (%.1f°), returning home...\n", current_yaw);
        yaw_.setTargetDeg(config_.calibrate_location_min_yaw_deg);
        calibration_sub_state_ = 1;
      }
      break;
      
    case 1:  // Wait for yaw to reach home
      if (fabsf(current_yaw - config_.calibrate_location_min_yaw_deg) < config_.yaw_angle_threshold_deg) {
        debugf_("[SM] Calibration: Reached home (%.1f°), pausing for %lu ms...\n", 
                current_yaw, (unsigned long)config_.calibration_pause_ms);
        sub_state_ms_ = millis();
        calibration_sub_state_ = 2;
      }
      break;
      
    case 2:  // Pause at home position
      if (sub_elapsed >= config_.calibration_pause_ms) {
        // Restore yaw axis accelerations
        yaw_.setAccel(config_.yaw_pre_calib_accel, config_.yaw_pre_calib_decel);
        debugf_("[SM] Calibration complete, returning to IDLE\n");
        enterState_(RobotState::IDLE);
      }
      break;
  }
}

// --------------------------------------------------------------------
// handleCheckingBall_() - Verify ball presence after suspected removal
//
// Sub-states:
//   0: Move pitch to disrupt position
//   1: Wait for pitch to settle at disrupt position
//   2: Collect confirmation samples and either return to IDLE (ball detected) or confirm missing and go to RELOADING
// --------------------------------------------------------------------
void StateMachine::handleCheckingBall_() {
  const uint32_t sub_elapsed = millis() - sub_state_ms_;
  const uint32_t min_duration_ms = SMDefaults::MIN_MOTION_DURATION_MS;
  
  CanInterface::AxisHeartbeat hb_pitch;

  switch (check_ball_sub_state_) {
    case 0:  // Move pitch to disrupt position
      debugf_("[SM] CheckBall: Moving pitch to %.1f°\n", config_.check_ball_disrupt_pitch_deg);
      pitch_.setTargetDeg(config_.check_ball_disrupt_pitch_deg);
      check_ball_sub_state_ = 1;
      sub_state_ms_ = millis();
      break;

    case 1:  // Wait for pitch to settle at disrupt position
      if (sub_elapsed < min_duration_ms) {
        break;
      }
      if (can_.getAxisHeartbeat(config_.pitch_node_id, hb_pitch) && hb_pitch.trajectory_done) {
        debugf_("[SM] CheckBall: Reached disrupt position, checking if ball in hand...\n");
        check_ball_sub_state_ = 2;
        sub_state_ms_ = millis();
      }
      break;

    case 2:  // Collect confirmation samples
      // If ANY sample shows ball present, return to IDLE
      if (can_.isBallInHand()) {
        debugf_("[SM] CheckBall: Ball detected after %d samples, returning to IDLE\n",
                check_ball_samples_collected_ + 1);
        enterState_(RobotState::IDLE);
        return;
      }
      check_ball_samples_collected_++;

      // If all samples collected and all were false, ball is confirmed gone
      if (check_ball_samples_collected_ >= config_.check_ball_confirm_samples) {
        debugf_("[SM] CheckBall: Ball confirmed missing after %d samples, initiating RELOADING\n",
                check_ball_samples_collected_);
        enterState_(RobotState::RELOADING);
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
  // Only accept reload commands if in IDLE or TRACKING states
  if (state_ != RobotState::IDLE && state_ != RobotState::TRACKING) {
    debugf_("[SM] Reload rejected: currently in %s\n",
            robotStateToString(state_));
    return false;
  }

  // Updating pending flag
  reload_pending_ = true;

  debugf_("[SM] Reload queued\n");

  return true;
}

// requestThrow() - Queue a throw request
bool StateMachine::requestThrow(float yaw_deg, float pitch_deg, float speed_mps, uint64_t throw_wall_us) {
  // Accept throws when in IDLE or TRACKING state
  if (state_ != RobotState::IDLE && state_ != RobotState::TRACKING) {
    debugf_("[SM] Throw rejected: not in IDLE or TRACKING state (current: %s)\n",
            robotStateToString(state_));
    return false;
  }
  if (!can_.isBallInHand()) {
    debugf_("[SM] Throw rejected: no ball in hand\n");
    return false;
  }

  // Require time sync to be established for absolute throw times
  if (!can_.hasTimeSync()) {
    debugf_("[SM] Throw rejected: time sync not established\n");
    return false;
  }

  // Store pending throw parameters
  throw_pending_ = true;
  pending_yaw_deg_ = yaw_deg;
  pending_pitch_deg_ = pitch_deg;
  pending_speed_mps_ = speed_mps;
  pending_throw_wall_us_ = throw_wall_us;

  const float lead_ms = (float)((int64_t)throw_wall_us - (int64_t)can_.wallTimeUs()) / 1000.0f;
  debugf_("[SM] Throw queued: yaw=%.1f° pitch=%.1f° speed=%.2f m/s at wall+%.0f ms\n",
          yaw_deg, pitch_deg, speed_mps, (double)lead_ms);

  return true;
}

// requestCheckBall() - Queue a check ball presence request
bool StateMachine::requestCheckBall() {
  // Only accept check ball commands if in IDLE or TRACKING states
  if (state_ != RobotState::IDLE && state_ != RobotState::TRACKING) {
    debugf_("[SM] CheckBall rejected: currently in %s\n",
            robotStateToString(state_));
    return false;
  }

  enterState_(RobotState::CHECKING_BALL);
  return true;
}

// requestTracking() - Request tracking mode or update tracking targets
bool StateMachine::requestTracking(float yaw_deg, float pitch_deg) {
  // Accept tracking commands when in IDLE or already TRACKING
  if (state_ != RobotState::IDLE && state_ != RobotState::TRACKING) {
    // If we're in THROWING or RELOADING, don't print anything to the console
    if (state_ != RobotState::THROWING && state_ != RobotState::RELOADING) {
      debugf_("[SM] Tracking command rejected: currently in %s\n",
              robotStateToString(state_));
    }
    return false;
  }

  // Rate-limit yaw and pitch updates to avoid flooding the CAN bus.
  // Only mark the axis as needing an update if enough time has elapsed.
  const uint32_t now_ms = millis();
  bool yaw_due   = (now_ms - last_yaw_cmd_ms_)   >= config_.yaw_cmd_interval_ms;
  bool pitch_due = (now_ms - last_pitch_cmd_ms_) >= config_.pitch_cmd_interval_ms;

  if (yaw_due) {
    last_tracking_cmd_.yaw_deg = yaw_deg;
    last_yaw_cmd_ms_ = now_ms;
  }
  if (pitch_due) {
    last_tracking_cmd_.pitch_deg = pitch_deg;
    last_pitch_cmd_ms_ = now_ms;
  }

  // Mark command valid if either axis was updated
  if (yaw_due || pitch_due) {
    last_tracking_cmd_.received_ms = now_ms;
    last_tracking_cmd_.valid = true;
  }
  last_tracking_cmd_ms_ = now_ms;

  // If currently IDLE, transition to TRACKING
  if (state_ == RobotState::IDLE) {
    debugf_("[SM] Entering TRACKING: yaw=%.1f° pitch=%.1f°\n", yaw_deg, pitch_deg);
    enterState_(RobotState::TRACKING);
  }

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
// requestSmoothMove() - Public API for smooth hand moves (replaces handleSmoothCmd logic)
// --------------------------------------------------------------------
bool StateMachine::requestSmoothMove(float target_rev) {
  // Only accept smooth move commands when IDLE
  if (state_ != RobotState::IDLE) {
    debugf_("[SM] smoothMove rejected: currently in %s (must be IDLE)\n",
            robotStateToString(state_));
    return false;
  }

  // Clamp to valid range
  if (target_rev < 0) target_rev = 0;
  if (target_rev > HAND_MAX_SMOOTH_MOVE_POS) target_rev = HAND_MAX_SMOOTH_MOVE_POS;

  // Get current PV
  float pos_rev = 0, vel_rps = 0;
  uint64_t t_us = 0;
  if (!can_.getAxisPV(config_.hand_node_id, pos_rev, vel_rps, t_us)) {
    debugf_("[SM] smoothMove: PV unavailable\n");
    return false;
  }

  // Check PV freshness
  if (can_.wallTimeUs() - t_us > SMDefaults::PV_FRESHNESS_US) {
    debugf_("[SM] smoothMove: PV stale\n");
    return false;
  }

  // Plan smooth move
  auto plan = planner_.planSmoothTo(target_rev, pos_rev, vel_rps,
                                    (uint32_t)(t_us & 0xFFFFFFFFu));
  if (plan.trajectory.empty()) {
    debugf_("[SM] smoothMove: already at target\n");
    return true;  // Not an error — already there
  }

  // Own the buffer and arm the streamer
  traj_buffer_ = std::move(plan.trajectory);
  const uint64_t time_offset_us = can_.wallTimeUs();

  bool ok = streamer_.arm(config_.hand_node_id, traj_buffer_.data(),
                          traj_buffer_.size(), time_offset_us);

  debugf_("[SM] smoothMove to %.2f: %s\n", target_rev, ok ? "Armed" : "FAIL");
  return ok;
}

// --------------------------------------------------------------------
// executeThrow_() - Actually execute the throw (called from handleIdle_)
// --------------------------------------------------------------------
bool StateMachine::executeThrow_(float yaw_deg, float pitch_deg,
                                 float speed_mps) {
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

  // Check PV freshness
  if (can_.wallTimeUs() - t_wall_us > SMDefaults::PV_FRESHNESS_US) {
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

  // 5) Check lead time — compare absolute throw time against earliest trajectory frame
  float min_ts = 0.0f;
  for (const auto& f : plan.trajectory) {
    if (f.t_s < min_ts) min_ts = f.t_s;
  }
  const float required_lead_us = (-min_ts + OpCfg::SCHEDULE_MARGIN_S) * 1e6f;
  const int64_t actual_lead_us = (int64_t)pending_throw_wall_us_ - (int64_t)can_.wallTimeUs();
  if ((float)actual_lead_us < required_lead_us) {
    debugf_("[SM] Throw rejected: insufficient lead time (need %.0f ms, have %.0f ms)\n",
            (double)(required_lead_us / 1000.0f), (double)(actual_lead_us / 1000.0f));
    return false;
  }

  // 6) Arm the streamer — use uint64_t to avoid float precision loss at epoch scale
  traj_buffer_ = std::move(plan.trajectory);
  const uint64_t throw_wall_us = pending_throw_wall_us_;

  // Diagnostic: how far in the future is the throw?
  const float lead_ms = (float)((int64_t)throw_wall_us - (int64_t)can_.wallTimeUs()) / 1000.0f;

  bool ok = streamer_.arm(config_.hand_node_id, traj_buffer_.data(),
                          traj_buffer_.size(), throw_wall_us);

  debugf_("[SM] Throw armed: frames=%u, ready_time=%.2f s, lead=%.1f ms, %s\n",
          (unsigned)traj_buffer_.size(), planner_.lastTimeToReadyS(),
          (double)lead_ms, ok ? "OK" : "FAIL");

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
  traj_buffer_ = std::move(plan.trajectory);
  const uint64_t time_offset_us = can_.wallTimeUs();

  return streamer_.arm(config_.hand_node_id, traj_buffer_.data(),
                       traj_buffer_.size(), time_offset_us);
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