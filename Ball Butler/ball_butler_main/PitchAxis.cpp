// PitchAxis.cpp
#include "PitchAxis.h"
#include "CanInterface.h"

PitchAxis::PitchAxis(CanInterface& can, uint8_t node_id, Stream* log)
  : can_(can), node_(node_id), log_(log ? log : &Serial) {}

bool PitchAxis::begin() {
  // Use the struct’s default-initialized values
  return begin(Traj{});
}

bool PitchAxis::begin(const Traj& tcfg) {
  traj_ = tcfg;
  bool ok = true;
  ok &= configureController_();
  ok &= applyTraj_(traj_);
  ok &= enterClosedLoop_();
  configured_ = ok;
  if (log_) log_->printf("PITCH: begin() %s (node=%u)  vel=%.3f rps  acc=%.3f  dec=%.3f\n",
                         ok ? "OK" : "FAIL", (unsigned)node_,
                         traj_.vel_limit_rps, traj_.accel_rps2, traj_.decel_rps2);
  return ok;
}

bool PitchAxis::configureController_() {
  // POSITION_CONTROL + TRAPEZOIDAL_TRAJECTORY input mode
  return can_.setControllerMode(node_, CONTROL_MODE_POSITION, INPUT_MODE_TRAP_TRAJ);
}

bool PitchAxis::applyTraj_(const Traj& t) {
  bool ok = true;
  ok &= can_.setTrajVelLimit(node_, t.vel_limit_rps);
  ok &= can_.setTrajAccLimits(node_, t.accel_rps2, t.decel_rps2);
  return ok;
}

bool PitchAxis::enterClosedLoop_() {
  return can_.setRequestedState(node_, AXIS_STATE_CLOSED_LOOP);
}

bool PitchAxis::setTrajLimits(float vel_rps, float acc_rps2, float dec_rps2) {
  traj_.vel_limit_rps = vel_rps;
  traj_.accel_rps2    = acc_rps2;
  traj_.decel_rps2    = dec_rps2;
  const bool ok = applyTraj_(traj_);
  if (log_) log_->printf("PITCH: traj -> vel=%.3f rps  acc=%.3f  dec=%.3f  (%s)\n",
                         vel_rps, acc_rps2, dec_rps2, ok?"OK":"FAIL");
  return ok;
}

bool PitchAxis::setGains(float kp_pos, float kv_vel, float ki_vel) {
  bool ok = true;
  ok &= can_.setPosGain(node_, kp_pos);
  ok &= can_.setVelGains(node_, kv_vel, ki_vel);
  if (log_) log_->printf("PITCH: gains -> Kp=%.3f  Kv=%.3f  Ki=%.3f  (%s)\n",
                         kp_pos, kv_vel, ki_vel, ok?"OK":"FAIL");
  return ok;
}

bool PitchAxis::sendTargetRev_(float target_rev) {
  // Check whether the axis is ready to move (in CLOSED_LOOP_CONTROL)
  CanInterface::AxisHeartbeat hb_pitch;
  bool ok = can_.getAxisHeartbeat(node_, hb_pitch);
  if (!ok) {
    if (log_) log_->println("PITCH: sendTargetRev_ FAILED to get heartbeat.");
    return false;
  }
  if (hb_pitch.axis_state != AXIS_STATE_CLOSED_LOOP) {
    enterClosedLoop_();
  }

  return can_.sendInputPos(node_, target_rev, /*vel_ff*/0.0f, /*tor_ff*/0.0f);
}

void PitchAxis::reject_(const char* why, float value, float lo, float hi) const {
  if (!log_) return;
  log_->printf("PITCH: REJECTED — %s: %.3f (allowed %.3f .. %.3f)\n",
               why, value, lo, hi);
}

bool PitchAxis::setTargetDeg(float deg) {
  if (!configured_) {
    if (log_) {
      // Throttle the message to avoid spamming
      if (millis() - last_print_rejected_ms_ >= PRINT_PITCH_REJECTED_INTERVAL_MS) {
        last_print_rejected_ms_ = millis();
      }
      log_->println("PITCH: REJECTED — not configured (call begin() first).");
    }
    return false;
  }
  if (!(deg >= DEG_MIN && deg <= DEG_MAX)) {
    reject_("angle out of range (deg)", deg, DEG_MIN, DEG_MAX);
    return false;
  }
  const float target_rev = degToRev(deg); // (deg - 90)/360
  if (!(target_rev >= REV_MIN && target_rev <= REV_MAX)) {
    reject_("internal rev clamp", target_rev, REV_MIN, REV_MAX);
    return false;
  }
  const bool ok = sendTargetRev_(target_rev);
  if (log_) {
    if (ok) log_->printf("PITCH: target -> %.2f deg  (%.4f rev)\n", deg, target_rev);
    else    log_->printf("PITCH: send target FAILED (deg=%.2f)\n", deg);
  }
  return ok;
}

void PitchAxis::printStatusOnce() const {
  float pos_rev, vel_rps; uint64_t t_us;
  if (can_.getAxisPV(node_, pos_rev, vel_rps, t_us)) {
    const float pos_deg = revToDeg(pos_rev);
    if (log_) log_->printf("PITCH | pos=%.2f deg (%.4f rev)  vel=%.3f rps\n",
                           pos_deg, pos_rev, vel_rps);
  } else {
    if (log_) log_->println("PITCH | PV unavailable.");
  }
}
