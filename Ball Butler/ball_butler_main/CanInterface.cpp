#include "CanInterface.h"
#include <string.h>
#include <math.h>
#include "Proprioception.h"
#include "StateMachine.h"
#include "Trajectory.h"  // For LINEAR_GAIN
#include "RobotState.h"

// ---------------- Command-name table (optional; PROGMEM friendly) -------------
static const CanInterface::CmdName kCmdNames[] PROGMEM = {
  { CanInterface::Cmd::heartbeat_message, "heartbeat_message" },
  { CanInterface::Cmd::get_error, "get_error" },
  { CanInterface::Cmd::RxSdo, "RxSdo" },
  { CanInterface::Cmd::TxSdo, "TxSdo" },
  { CanInterface::Cmd::set_requested_state, "set_requested_state" },
  { CanInterface::Cmd::get_encoder_estimate, "get_encoder_estimate" },
  { CanInterface::Cmd::set_controller_mode, "set_controller_mode" },
  { CanInterface::Cmd::set_input_pos, "set_input_pos" },
  { CanInterface::Cmd::set_input_vel, "set_input_vel" },
  { CanInterface::Cmd::set_vel_curr_limits, "set_vel_curr_limits" },
  { CanInterface::Cmd::set_traj_vel_limit, "set_traj_vel_limit" },
  { CanInterface::Cmd::set_traj_acc_limits, "set_traj_acc_limits" },
  { CanInterface::Cmd::get_iq, "get_iq" },
  { CanInterface::Cmd::get_temps, "get_temps" },
  { CanInterface::Cmd::reboot_odrives, "reboot_odrives" },
  { CanInterface::Cmd::get_bus_voltage_current, "get_bus_voltage_current" },
  { CanInterface::Cmd::clear_errors, "clear_errors" },
  { CanInterface::Cmd::set_absolute_position, "set_absolute_position" },
  { CanInterface::Cmd::set_pos_gain, "set_pos_gain" },
  { CanInterface::Cmd::set_vel_gains, "set_vel_gains" },
};

const CanInterface::CmdName* CanInterface::commandNameTable(size_t& count) {
  count = sizeof(kCmdNames) / sizeof(kCmdNames[0]);
  return kCmdNames;
}

// ---------------- Error code name table ----------------
const char* CanInterface::errorCodeToString(BallButlerError err) {
  switch (err) {
    case BallButlerError::NONE:          return "NONE";
    case BallButlerError::RELOAD_FAILED: return "RELOAD_FAILED";
    default:                             return "UNKNOWN";
  }
}

// ---------------- static singleton ----------------
CanInterface* CanInterface::s_instance_ = nullptr;

// ---------------- ctor ----------------
CanInterface::CanInterface()
  : can1_() {}

// ---------------- 64-bit micros ----------------
uint64_t CanInterface::micros64_() {
  static uint32_t last_lo = ::micros();
  static uint64_t hi = 0;
  uint32_t now = ::micros();
  if (now < last_lo) hi += (uint64_t)1 << 32;
  last_lo = now;
  return (hi | now);
}

// ---------------- begin ----------------
void CanInterface::begin(uint32_t bitrate) {
  s_instance_ = this;

  can1_.begin();
  can1_.setBaudRate(bitrate);
  can1_.setMaxMB(16);
  can1_.enableFIFO();
  can1_.enableFIFOInterrupt();
  can1_.onReceive(rxTrampoline_);

  // Clear states
  last_host_cmd_ = HostThrowCmd{};  // clears .valid
  have_offset_ = false;
  wall_offset_us_ = 0;
  stats_.clear();
  nextPrint_us_ = micros64_() + PRINT_PERIOD_US_;
  home_required_mask_ = 0;

  // Initialise states
  for (int i = 0; i < 64; ++i) {
    axes_pv_[i].valid = false;
    axes_iq_[i].valid = false;
    home_state_[i] = AxisHomeState::Unhomed;
  }

  // Initialise last heartbeats arrays
  for (int i = 0; i < 64; ++i) {
    axes_pv_[i].valid = false;
    axes_iq_[i].valid = false;
    hb_[i].valid = false;
  }

  // Initialise BRAKE_RESISTOR_DISARMED auto-clearing stuff
  for (int i = 0; i < 64; ++i) {
    last_brake_clear_us_[i] = 0;
  }

  // Initialise arbitrary parameter response storage
  for (int i = 0; i < 64; ++i) {
    arb_param_resp_[i].valid = false;
  }

  // Initialise Ball Butler heartbeat state
  last_heartbeat_ms_ = 0;
  current_error_code_ = BallButlerError::NONE;
}

// ---------------- loop ----------------
void CanInterface::loop() {
  can1_.events();           // dispatch FIFO -> rxTrampoline_
  maybePrintSyncStats_();   // optional stats print
  maybePublishHeartbeat_(); // Ball Butler heartbeat publishing
}

// ---------------- debug ----------------
void CanInterface::setDebugStream(Stream* dbg) { dbg_ = dbg; }
void CanInterface::setDebugFlags(bool timeSyncDebug, bool canDebug) {
  dbg_time_ = timeSyncDebug;
  dbg_can_ = canDebug;
}

// ---------------- time functions ----------------
uint64_t CanInterface::localTimeUs() const { return micros64_(); }
uint64_t CanInterface::wallTimeUs() const {
  return uint64_t(int64_t(localTimeUs()) + wall_offset_us_);
}

CanInterface::SyncStats CanInterface::getAndClearSyncStats() {
  SyncStats out;
  if (stats_.n) {
    out.mean_us = float(stats_.sum) / float(stats_.n);
    out.rms_us = sqrtf(float(stats_.sum_sq) / float(stats_.n));
    out.min_us = stats_.minv;
    out.max_us = stats_.maxv;
    out.frames = stats_.n;
  }
  stats_.clear();
  return out;
}

// ---------------- generic send ----------------
bool CanInterface::sendRaw(uint32_t id, const uint8_t* data, uint8_t len) {
  CAN_message_t m;
  m.id = id;
  m.len = len;
  m.flags.extended = 0;
  m.flags.remote = 0;
  if (len && data) memcpy(m.buf, data, len);
  const bool ok = can1_.write(m);
  if (dbg_ && dbg_can_) dbg_->printf("[CAN->] id=0x%03lX len=%u %s\n", (unsigned long)id, (unsigned)len, ok ? "OK" : "FAIL");
  return ok;
}

bool CanInterface::sendRTR(uint32_t id, uint8_t len) {
  CAN_message_t m;
  m.id = id;
  m.len = len;
  m.flags.extended = 0;
  m.flags.remote = 1;
  const bool ok = can1_.write(m);
  if (dbg_can_ && dbg_) dbg_->printf("[CAN->RTR] id=0x%03lX len=%u %s\n", (unsigned long)id, (unsigned)len, ok ? "OK" : "FAIL");
  return ok;
}

// ---------------- Handling Jetson Commands ----------------
bool CanInterface::getHostThrowCmd(HostThrowCmd& out) const {
  if (!last_host_cmd_.valid) return false;
  out = last_host_cmd_;
  return true;
}
void CanInterface::setHostThrowCallback(HostThrowCallback cb, void* user) {
  host_cb_ = cb;
  host_cb_user_ = user;
}

// ---------------- ODrive helpers ----------------
static inline void wrFloatLE(uint8_t* b, float f) { memcpy(b, &f, 4); }
static inline void wrU32LE(uint8_t* b, uint32_t v) {
  b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; b[2] = (v >> 16) & 0xFF; b[3] = (v >> 24) & 0xFF;
}
static inline void wrU16LE(uint8_t* b, uint16_t v) { b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF; }

int16_t CanInterface::clampToI16_(float x) {
  if (x > 32767.f) return 32767;
  if (x < -32768.f) return -32768;
  return (int16_t)lrintf(x);
}

bool CanInterface::setRequestedState(uint32_t node_id, uint32_t requested_state) {
  uint8_t d[8] = { 0 };
  wrU32LE(&d[0], requested_state);
  return sendRaw(makeId(node_id, Cmd::set_requested_state), d, 8);
}

bool CanInterface::setControllerMode(uint32_t node_id, uint32_t control_mode, uint32_t input_mode) {
  uint8_t d[8] = { 0 };
  wrU32LE(&d[0], control_mode);
  wrU32LE(&d[4], input_mode);
  return sendRaw(makeId(node_id, Cmd::set_controller_mode), d, 8);
}

bool CanInterface::setVelCurrLimits(uint32_t node_id, float current_limit, float vel_limit_rps) {
  uint8_t d[8];
  wrFloatLE(&d[0], vel_limit_rps);
  wrFloatLE(&d[4], current_limit);
  return sendRaw(makeId(node_id, Cmd::set_vel_curr_limits), d, 8);
}

bool CanInterface::setTrajVelLimit(uint32_t node_id, float vel_limit_rps) {
  uint8_t d[8] = { 0 };
  wrFloatLE(&d[0], vel_limit_rps);
  return sendRaw(makeId(node_id, Cmd::set_traj_vel_limit), d, 8);
}

bool CanInterface::setTrajAccLimits(uint32_t node_id, float accel_rps2, float decel_rps2) {
  uint8_t d[8];
  wrFloatLE(&d[0], accel_rps2);
  wrFloatLE(&d[4], decel_rps2);
  return sendRaw(makeId(node_id, Cmd::set_traj_acc_limits), d, 8);
}

bool CanInterface::setPosGain(uint32_t node_id, float kp) {
  uint8_t d[8] = { 0 };
  wrFloatLE(&d[0], kp);
  return sendRaw(makeId(node_id, Cmd::set_pos_gain), d, 8);
}

bool CanInterface::setVelGains(uint32_t node_id, float kv, float ki) {
  uint8_t d[8];
  wrFloatLE(&d[0], kv);
  wrFloatLE(&d[4], ki);
  return sendRaw(makeId(node_id, Cmd::set_vel_gains), d, 8);
}

bool CanInterface::setAbsolutePosition(uint32_t node_id, float pos_rev) {
  uint8_t d[8] = { 0 };
  wrFloatLE(&d[0], pos_rev);
  return sendRaw(makeId(node_id, Cmd::set_absolute_position), d, 8);
}

bool CanInterface::clearErrors(uint32_t node_id) {
  uint8_t d[8] = { 0 };
  return sendRaw(makeId(node_id, Cmd::clear_errors), d, 8);
}

bool CanInterface::reboot(uint32_t node_id) {
  uint8_t d[8] = { 0 };
  return sendRaw(makeId(node_id, Cmd::reboot_odrives), d, 8);
}

bool CanInterface::sendInputPos(uint32_t node_id, float pos_rev, float vel_ff_rev_per_s, float torque_ff) {
  if (!isAxisMotionAllowed(node_id)) {
    Serial.printf("[Gate] Blocked set_input_pos to node %lu (not homed)\n", (unsigned long)node_id);
    return false;
  }
  uint8_t d[8];
  wrFloatLE(&d[0], pos_rev);
  const int16_t vel_i = clampToI16_(vel_ff_rev_per_s * kVelScale_);
  const int16_t tor_i = clampToI16_(torque_ff * kTorScale_);
  d[4] = uint8_t(vel_i & 0xFF);
  d[5] = uint8_t((vel_i >> 8) & 0xFF);
  d[6] = uint8_t(tor_i & 0xFF);
  d[7] = uint8_t((tor_i >> 8) & 0xFF);
  const bool ok = sendRaw(makeId(node_id, Cmd::set_input_pos), d, 8);
  if (dbg_can_ && dbg_) {
    dbg_->printf("[CAN->ODrive pos] node=%lu pos=%.4f vel_ff=%.3f tor_ff=%.3f ok=%d\n",
                 (unsigned long)node_id, (double)pos_rev, (double)vel_ff_rev_per_s, (double)torque_ff, (int)ok);
  }
  return ok;
}

bool CanInterface::sendInputVel(uint32_t node_id, float vel_rps, float torque_ff) {
  if (!isAxisMotionAllowed(node_id)) {
    Serial.printf("[Gate] Blocked set_input_vel to node %lu (not homed)\n", (unsigned long)node_id);
    return false;
  }
  uint8_t d[8];
  wrFloatLE(&d[0], vel_rps);
  const int16_t tor_i = clampToI16_(torque_ff * kTorScale_);
  d[4] = uint8_t(tor_i & 0xFF);
  d[5] = uint8_t((tor_i >> 8) & 0xFF);
  d[6] = 0;
  d[7] = 0;
  const bool ok = sendRaw(makeId(node_id, Cmd::set_input_vel), d, 8);
  if (dbg_can_ && dbg_) {
    dbg_->printf("[CAN->ODrive vel] node=%lu vel=%.4f tor_ff=%.3f ok=%d\n",
                 (unsigned long)node_id, (double)vel_rps, (double)torque_ff, (int)ok);
  }
  return ok;
}

// ================================================================================
// Arbitrary Parameter (SDO) Functions
// ================================================================================

bool CanInterface::sendArbitraryParameterFloat(uint32_t node_id, uint16_t endpoint_id, float value) {
  uint8_t d[8];
  d[0] = OPCODE_WRITE;
  wrU16LE(&d[1], endpoint_id);
  d[3] = 0;
  wrFloatLE(&d[4], value);
  const bool ok = sendRaw(makeId(node_id, Cmd::RxSdo), d, 8);
  if (dbg_can_ && dbg_) {
    dbg_->printf("[CAN->SDO Write] node=%lu endpoint=%u value=%.4f ok=%d\n",
                 (unsigned long)node_id, (unsigned)endpoint_id, (double)value, (int)ok);
  }
  return ok;
}

bool CanInterface::sendArbitraryParameterU32(uint32_t node_id, uint16_t endpoint_id, uint32_t value) {
  uint8_t d[8];
  d[0] = OPCODE_WRITE;
  wrU16LE(&d[1], endpoint_id);
  d[3] = 0;
  wrU32LE(&d[4], value);
  const bool ok = sendRaw(makeId(node_id, Cmd::RxSdo), d, 8);
  if (dbg_can_ && dbg_) {
    dbg_->printf("[CAN->SDO Write U32] node=%lu endpoint=%u value=%lu ok=%d\n",
                 (unsigned long)node_id, (unsigned)endpoint_id, (unsigned long)value, (int)ok);
  }
  return ok;
}

bool CanInterface::requestArbitraryParameter(uint32_t node_id, uint16_t endpoint_id) {
  uint8_t d[8] = { 0 };
  d[0] = OPCODE_WRITE;
  d[1] = endpoint_id & 0xFF;
  d[2] = (endpoint_id >> 8) & 0xFF;
  d[3] = 0;

  return sendRaw(makeId(node_id, Cmd::RxSdo), d, 8);
}

bool CanInterface::getLastArbitraryParamResponse(uint32_t node_id, ArbitraryParamResponse& out) const {
  if (node_id >= 64) return false;
  ArbitraryParamResponse snap;
  uint64_t t1, t2;
  do {
    t1 = arb_param_resp_[node_id].wall_us;
    snap = arb_param_resp_[node_id];
    t2 = arb_param_resp_[node_id].wall_us;
  } while (t1 != t2);
  if (!snap.valid) return false;
  out = snap;
  return true;
}

void CanInterface::setArbitraryParamCallback(ArbitraryParamCallback cb, void* user) {
  arb_param_cb_ = cb;
  arb_param_cb_user_ = user;
}

bool CanInterface::isEncoderSearchComplete(uint32_t node_id, uint32_t timeout_ms) {
  if (!requestArbitraryParameter(node_id, EndpointIds::COMMUTATION_MAPPER_POS_ABS)) return false;
  const uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    can1_.events();
    ArbitraryParamResponse resp;
    if (getLastArbitraryParamResponse(node_id, resp)) {
      if (resp.endpoint_id == EndpointIds::COMMUTATION_MAPPER_POS_ABS) {
        return !isnan(resp.value.f32);
      }
    }
    delay(1);
  }
  if (dbg_) dbg_->printf("[SDO] Timeout waiting for encoder search status on node %lu\n", (unsigned long)node_id);
  return false;
}

bool CanInterface::readGpioStates(uint32_t node_id, uint32_t& states_out, uint32_t timeout_ms) {
  if (!requestArbitraryParameter(node_id, EndpointIds::GPIO_STATES)) {
    if (dbg_) dbg_->printf("[SDO] Failed to send GPIO states request to node %lu\n", (unsigned long)node_id);
    return false;
  }
  const uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    can1_.events();
    ArbitraryParamResponse resp;
    if (getLastArbitraryParamResponse(node_id, resp)) {
      if (resp.endpoint_id == EndpointIds::GPIO_STATES) {
        states_out = resp.value.u32;
        return true;
      }
    }
    delay(1);
  }
  if (dbg_) dbg_->printf("[SDO] Timeout waiting for GPIO states on node %lu\n", (unsigned long)node_id);
  return false;
}

// ================================================================================
// Operating Config and Axis Access
// ================================================================================

bool CanInterface::restoreHandToOperatingConfig(uint32_t node_id, float vel_limit_rps, float current_limit_A) {
  constexpr uint32_t AXIS_STATE_IDLE = 1u;
  constexpr uint32_t CONTROL_MODE_POSITION = 3u;
  constexpr uint32_t INPUT_MODE_PASSTHROUGH = 1u;
  bool ok = true;
  ok &= setRequestedState(node_id, AXIS_STATE_IDLE);
  ok &= setControllerMode(node_id, CONTROL_MODE_POSITION, INPUT_MODE_PASSTHROUGH);
  ok &= setVelCurrLimits(node_id, current_limit_A, vel_limit_rps);
  return ok;
}

bool CanInterface::getAxisPV(uint32_t node_id, float& pos_out, float& vel_out, uint64_t& wall_us_out) const {
  if (node_id >= 64) return false;
  uint64_t t1, t2;
  float p, v;
  do {
    t1 = axes_pv_[node_id].wall_us;
    p = axes_pv_[node_id].pos_rev;
    v = axes_pv_[node_id].vel_rps;
    t2 = axes_pv_[node_id].wall_us;
  } while (t1 != t2);
  if (!axes_pv_[node_id].valid) return false;
  pos_out = p; vel_out = v; wall_us_out = t1;
  return true;
}

bool CanInterface::getAxisIq(uint32_t node_id, float& iq_meas_out, float& iq_setp_out, uint64_t& wall_us_out) const {
  if (node_id >= 64) return false;
  uint64_t t1, t2;
  float iqm, iqs;
  do {
    t1 = axes_iq_[node_id].wall_us;
    iqm = axes_iq_[node_id].iq_meas;
    iqs = axes_iq_[node_id].iq_setp;
    t2 = axes_iq_[node_id].wall_us;
  } while (t1 != t2);
  if (!axes_iq_[node_id].valid) return false;
  iq_meas_out = iqm; iq_setp_out = iqs; wall_us_out = t1;
  return true;
}

// ================================================================================
// Homing
// ================================================================================

bool CanInterface::homeHand(uint32_t node_id, float homing_speed_rps, float current_limit_A,
                            float current_headroom_A, float settle_pos_rev, float avg_weight, uint16_t) {
  setHomeState(node_id, AxisHomeState::Homing);
  if (!setRequestedState(node_id, 8u)) return false;
  if (!setControllerMode(node_id, 2u, 2u)) return false;
  const float vel_limit = fabsf(homing_speed_rps * 2.0f);
  if (!setVelCurrLimits(node_id, current_limit_A + current_headroom_A, vel_limit)) return false;
  delay(100);

  if (!sendInputVel(node_id, homing_speed_rps, 0.0f)) {
    setRequestedState(node_id, 1u);
    setHomeState(node_id, AxisHomeState::Unhomed);
    return false;
  }

  Serial.printf("[Home] node=%lu moving at %.3f rps; Iq limit=%.2f A\n",
                (unsigned long)node_id, (double)homing_speed_rps, (double)current_limit_A);

  float ema = 0.0f;
  uint64_t last_seen_us = 0;
  uint64_t start_time = millis();
  uint64_t timeout = 5000;

  for (;;) {
    can1_.events();
    float iq_meas, iq_setp;
    uint64_t t_us;
    if (getAxisIq(node_id, iq_meas, iq_setp, t_us)) {
      if (t_us != last_seen_us) {
        last_seen_us = t_us;
        const float alpha = constrain(avg_weight, 0.f, 0.9999f);
        ema = alpha * ema + (1.f - alpha) * iq_meas;

        if (hb_[hand_node_id_].axis_state == 1 || hb_[hand_node_id_].axis_error == 1) {
          if (millis() - start_time > timeout) {
            Serial.println("Timeout while homing hand... Exiting to try again.");
            return false;
          }
        }
        
        if (fabsf(ema) >= current_limit_A) {
          setRequestedState(node_id, 1u);
          delay(5);
          setAbsolutePosition(node_id, settle_pos_rev);
          Serial.printf("[Home] node=%lu homed. Set pos to %.3f rev.\n", (unsigned long)node_id, (double)settle_pos_rev);
          restoreHandToOperatingConfig(node_id);
          setHomeState(node_id, AxisHomeState::Homed);
          return true;
        }
      }
    }
    delay(1);
  }
}

bool CanInterface::homeHandStandard(uint32_t node_id, int hand_direction, float base_speed_rps,
                                    float current_limit_A, float current_headroom_A, float set_abs_pos_rev) {
  const float homing_speed = float(hand_direction) * base_speed_rps;
  return homeHand(node_id, homing_speed, current_limit_A, current_headroom_A, set_abs_pos_rev, 0.7f, 10);
}

// ================================================================================
// Home State Management
// ================================================================================

void CanInterface::setHomeState(uint32_t node_id, AxisHomeState s) {
  if (node_id < 64) home_state_[node_id] = s;
}

CanInterface::AxisHomeState CanInterface::getHomeState(uint32_t node_id) const {
  return (node_id < 64) ? home_state_[node_id] : AxisHomeState::Unhomed;
}

void CanInterface::requireHomeForAxis(uint32_t node_id, bool required) {
  if (node_id >= 64) return;
  const uint64_t bit = (1ULL << node_id);
  if (required) home_required_mask_ |= bit;
  else home_required_mask_ &= ~bit;
}

bool CanInterface::isHomeRequired(uint32_t node_id) const {
  if (node_id >= 64) return false;
  return (home_required_mask_ & (1ULL << node_id)) != 0;
}

void CanInterface::requireHomeOnlyFor(uint32_t node_id) {
  home_required_mask_ = (node_id < 64) ? (1ULL << node_id) : 0;
}

void CanInterface::clearHomeRequirements() { home_required_mask_ = 0; }

bool CanInterface::isAxisMotionAllowed(uint32_t node_id) const {
  if (node_id >= 64) return false;
  if (!isHomeRequired(node_id)) return true;
  AxisHomeState s = home_state_[node_id];
  return s == AxisHomeState::Homing || s == AxisHomeState::Homed;
}

// ================================================================================
// Axis Heartbeat
// ================================================================================

bool CanInterface::getAxisHeartbeat(uint32_t node_id, AxisHeartbeat& out) const {
  if (node_id >= 64) return false;
  AxisHeartbeat snap;
  uint64_t t1, t2;
  do {
    t1 = hb_[node_id].wall_us;
    snap = hb_[node_id];
    t2 = hb_[node_id].wall_us;
  } while (t1 != t2);
  if (!snap.valid) return false;
  out = snap;
  return true;
}

bool CanInterface::hasAxisError(uint32_t node_id, uint32_t mask) const {
  AxisHeartbeat hb;
  return getAxisHeartbeat(node_id, hb) && (hb.axis_error & mask);
}

bool CanInterface::waitForAxisErrorClear(uint32_t node_id, uint32_t mask, uint32_t timeout_ms, uint16_t poll_ms) {
  const uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    const_cast<CanInterface*>(this)->loop();
    AxisHeartbeat hb;
    if (getAxisHeartbeat(node_id, hb) && (hb.axis_error & mask) == 0u) return true;
    delay(poll_ms);
  }
  return false;
}

void CanInterface::setAutoClearBrakeResistor(bool enable, uint32_t min_interval_ms) {
  auto_clear_brake_res_ = enable;
  auto_clear_interval_ms_ = min_interval_ms ? min_interval_ms : 1;
}

void CanInterface::setEstimatorCmd(uint8_t cmd) { estimatorCmd_ = (cmd & 0x1F); }

// ================================================================================
// Ball Butler Heartbeat
// ================================================================================
void CanInterface::maybePublishHeartbeat_() {
  if (heartbeat_rate_ms_ == 0) return;
  if (!state_machine_) return;
  const uint32_t now_ms = millis();
  if (now_ms - last_heartbeat_ms_ < heartbeat_rate_ms_) return;
  last_heartbeat_ms_ = now_ms;
  publishHeartbeat_();
}

void CanInterface::publishHeartbeat_() {
  RobotState state = state_machine_->getState();
  bool ball_present = state_machine_->isBallInHand();
  
  // Map RobotState to heartbeat state values
  uint8_t state_val = robotStateToUint8(state);
  
  // Byte 0: State byte (bit 0 = ball_in_hand, bits 1-7 = state)
  uint8_t state_byte = (ball_present ? 0x01 : 0x00) | (state_val << 1);
  
  // Byte 1: State data (error_code for ERROR, 0 otherwise)
  uint8_t state_data = 0;
  if (state == RobotState::ERROR) {
    state_data = static_cast<uint8_t>(current_error_code_);
  }
  
  // Position feedback from Proprioception
  ProprioceptionData prop;
  PRO.snapshot(prop);
  
  float yaw_res = 0.01f;       // degrees
  float pitch_res = 0.002f;    // degrees
  float hand_res = 0.01f;      // mm

  // Yaw: resolution ~0.01° : 0-360° -> 0-65535 (uint16),
  uint16_t yaw_enc = 0;
  if (prop.isYawValid()) {
    float yaw_deg = fmodf(prop.yaw_deg, 360.0f);
    if (yaw_deg < 0) yaw_deg += 360.0f;
    yaw_enc = (uint16_t)(yaw_deg / yaw_res);
  }
  
  // Pitch:  resolution ~0.002° : 0-131.072° -> 0-65535 (uint16),
  uint16_t pitch_enc = 0;
  if (prop.isPitchValid()) {
    float pitch_deg = constrain(prop.pitch_deg, 0.0f, 90.0f);
    pitch_enc = (uint16_t)(pitch_deg / pitch_res);
  }
  
  // Hand: resolution 0.01mm : 0-655.36 mm -> 0-65535 (uint16), 
  uint16_t hand_enc = 0;
  if (prop.isHandPVValid()) {
    float hand_mm = prop.hand_pos_rev / LINEAR_GAIN * 1000.0f;
    hand_mm = constrain(hand_mm, 0.0f, 655.36f);
    hand_enc = (uint16_t)(hand_mm / hand_res);
  }
  
  // Assemble frame (little-endian for uint16 fields)
  uint8_t frame[8];
  frame[0] = state_byte;
  frame[1] = state_data;
  frame[2] = yaw_enc & 0xFF;
  frame[3] = (yaw_enc >> 8) & 0xFF;
  frame[4] = pitch_enc & 0xFF;
  frame[5] = (pitch_enc >> 8) & 0xFF;
  frame[6] = hand_enc & 0xFF;
  frame[7] = (hand_enc >> 8) & 0xFF;

  // Use centralized CAN ID from CanIds namespace
  sendRaw(CanIds::HEARTBEAT_CMD, frame, 8);
}

// ================================================================================
// RX Handlers
// ================================================================================

void CanInterface::rxTrampoline_(const CAN_message_t& msg) {
  if (s_instance_) s_instance_->handleRx_(msg);
}

void CanInterface::handleRx_(const CAN_message_t& msg) {
  // Time sync - uses instance variable (can be overridden from default)
  if (msg.id == CanIds::TIME_SYNC_CMD && msg.len == 8 && !msg.flags.remote) {
    handleTimeSync_(msg);
    return;
  }

  const uint32_t node = (msg.id >> 5);
  const uint8_t cmd = msg.id & 0x1F;

  // Host -> Teensy throw command
  if (msg.id == CanIds::HOST_THROW_CMD && msg.len == 8 && !msg.flags.remote) {
    const int16_t yaw_i = (int16_t)((uint16_t)msg.buf[0] | ((uint16_t)msg.buf[1] << 8));
    const uint16_t pit_u = (uint16_t)msg.buf[2] | ((uint16_t)msg.buf[3] << 8);
    const uint16_t sp_u = (uint16_t)msg.buf[4] | ((uint16_t)msg.buf[5] << 8);
    const uint16_t t_u = (uint16_t)msg.buf[6] | ((uint16_t)msg.buf[7] << 8);

    HostThrowCmd c;
    c.yaw_rad = float(yaw_i) * (float)M_PI / 32768.0f;
    c.pitch_rad = float(pit_u) * ((float)M_PI / 65536.0f);
    c.speed_mps = float(sp_u) * 0.0001f;
    c.in_s = float(t_u) * 0.001f;

    if (c.yaw_rad < -M_PI) c.yaw_rad = -M_PI;
    if (c.yaw_rad > M_PI) c.yaw_rad = M_PI;
    if (c.pitch_rad < 0.f) c.pitch_rad = 0.f;
    const float PI_2 = (float)M_PI * 0.5f;
    if (c.pitch_rad > PI_2) c.pitch_rad = PI_2;
    if (c.speed_mps < 0.f) c.speed_mps = 0.f;
    if (c.speed_mps > 6.5535f) c.speed_mps = 6.5535f;
    if (c.in_s < 0.f) c.in_s = 0.f;
    if (c.in_s > 65.535f) c.in_s = 65.535f;

    c.wall_us = wallTimeUs();
    c.valid = true;
    last_host_cmd_ = c;

    if (dbg_can_ && dbg_) {
      dbg_->printf("[HostCmd] yaw=%.3f rad pitch=%.3f rad speed=%.3f m/s in=%.3f s (id=0x%03lX)\n",
                   (double)c.yaw_rad, (double)c.pitch_rad, (double)c.speed_mps,
                   (double)c.in_s, (unsigned long)msg.id);
    }

    if (host_cb_) host_cb_(c, host_cb_user_);
    return;
  }

  // Host -> BB RELOAD command
  if (msg.id == CanIds::RELOAD_CMD && msg.len == 0 && !msg.flags.remote) {
    if (dbg_can_ && dbg_) {
      dbg_->printf("[CAN<-RELOAD] id=0x%03lX cmd=%u\n", (unsigned long)msg.id, (unsigned)msg.buf[0]);
    }
    if (state_machine_) {
      state_machine_->requestReload();
    }
    return;
  }

  // Host -> BB RESET command
  if (msg.id == CanIds::RESET_CMD && msg.len == 0 && !msg.flags.remote) {
    if (dbg_can_ && dbg_) {
      dbg_->printf("[CAN<-RESET] id=0x%03lX cmd=%u\n", (unsigned long)msg.id, (unsigned)msg.buf[0]);
    }
    if (state_machine_) {
      state_machine_->reset();
    }
    return;
  }

  // Host -> BB CALIBRATE LOCATION command
  if (msg.id == CanIds::CALIBRATE_LOC_CMD && msg.len == 0 && !msg.flags.remote) {
    if (dbg_can_ && dbg_) {
      dbg_->printf("[CAN<-CALIBRATE_LOC] id=0x%03lX cmd=%u\n", (unsigned long)msg.id, (unsigned)msg.buf[0]);
    }
    if (state_machine_) {
      state_machine_->requestCalibrateLocation();
    }
    return;
  }

  // ODrive heartbeat
  if (cmd == uint8_t(Cmd::heartbeat_message) && node < 64 && msg.len >= 7 && !msg.flags.remote) {
    uint32_t axis_error = (uint32_t)msg.buf[0] | ((uint32_t)msg.buf[1] << 8)
                        | ((uint32_t)msg.buf[2] << 16) | ((uint32_t)msg.buf[3] << 24);

    hb_[node].axis_error = axis_error;
    hb_[node].axis_state = msg.buf[4];
    hb_[node].procedure_result = msg.buf[5];
    hb_[node].trajectory_done = msg.buf[6];
    hb_[node].wall_us = wallTimeUs();
    hb_[node].valid = true;

    if (auto_clear_brake_res_ && (axis_error & ODriveErrors::BRAKE_RESISTOR_DISARMED)) {
      const uint64_t now_us = micros64_();
      const uint64_t min_gap_us = (uint64_t)auto_clear_interval_ms_ * 1000ULL;
      if (now_us - last_brake_clear_us_[node] >= min_gap_us) {
        last_brake_clear_us_[node] = now_us;
        const bool ok = clearErrors(node);
        if (dbg_) {
          dbg_->printf("[AutoClear] node=%lu axis_error=0x%08lX -> Clear_Errors %s\n",
                       (unsigned long)node, (unsigned long)axis_error, ok ? "OK" : "FAIL");
        }
      }
    }
    return;
  }

  // TxSdo (arbitrary parameter response)
  if (cmd == uint8_t(Cmd::TxSdo) && node < 64) {
    handleTxSdo_(node, msg.buf, msg.len);
    return;
  }

  // Estimator (pos, vel)
  if (cmd == estimatorCmd_ && msg.len == 8 && node < 64 && !msg.flags.remote) {
    float pos, vel;
    memcpy(&pos, &msg.buf[0], 4);
    memcpy(&vel, &msg.buf[4], 4);
    axes_pv_[node].pos_rev = pos;
    axes_pv_[node].vel_rps = vel;
    axes_pv_[node].wall_us = wallTimeUs();
    axes_pv_[node].valid = true;

    Proprioception& prop = PRO;
    const uint64_t t_us = axes_pv_[node].wall_us;

    if (node == hand_node_id_) {
      prop.setHandPV(pos, vel, t_us);
    }
    if (node == pitch_node_id_) {
      const float pitch_deg = 90.0 + pos * 360.0f;
      prop.setPitchDeg(pitch_deg, t_us);
    }

    if (dbg_can_ && dbg_) {
      dbg_->printf("[CAN<-PV] node=%lu pos=%.4f rev vel=%.4f rps id=0x%03lX\n",
                   (unsigned long)node, (double)pos, (double)vel, (unsigned long)msg.id);
    }
    return;
  }

  // Iq feedback
  if (cmd == uint8_t(Cmd::get_iq) && msg.len == 8 && node < 64 && !msg.flags.remote) {
    float iq_meas, iq_setp;
    memcpy(&iq_meas, &msg.buf[0], 4);
    memcpy(&iq_setp, &msg.buf[4], 4);
    axes_iq_[node].iq_meas = iq_meas;
    axes_iq_[node].iq_setp = iq_setp;
    axes_iq_[node].wall_us = wallTimeUs();
    axes_iq_[node].valid = true;

    if (node == hand_node_id_) {
      PRO.setHandIq(iq_meas, axes_iq_[node].wall_us);
    }

    if (dbg_can_ && dbg_) {
      dbg_->printf("[CAN<-Iq] node=%lu Iq=%.3fA set=%.3fA id=0x%03lX\n",
                   (unsigned long)node, (double)iq_meas, (double)iq_setp, (unsigned long)msg.id);
    }
    return;
  }

  if (dbg_can_ && dbg_) {
    dbg_->printf("[CAN] id=0x%03lX len=%d rtr=%d\n", (unsigned long)msg.id, msg.len, (int)msg.flags.remote);
  }
}

void CanInterface::handleTxSdo_(uint32_t node_id, const uint8_t* buf, uint8_t len) {
  if (len < 8) return;
  const uint8_t opcode = buf[0];
  const uint16_t endpoint_id = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);

  ArbitraryParamResponse resp;
  resp.opcode = opcode;
  resp.endpoint_id = endpoint_id;
  resp.wall_us = wallTimeUs();
  memcpy(&resp.value.f32, &buf[4], 4);
  resp.valid = true;
  arb_param_resp_[node_id] = resp;
  
  if (arb_param_cb_) {
    arb_param_cb_(node_id, resp, arb_param_cb_user_);
  }
}

void CanInterface::handleTimeSync_(const CAN_message_t& msg) {
  uint32_t sec = (uint32_t)msg.buf[0] | ((uint32_t)msg.buf[1] << 8) 
               | ((uint32_t)msg.buf[2] << 16) | ((uint32_t)msg.buf[3] << 24);
  uint32_t usec = (uint32_t)msg.buf[4] | ((uint32_t)msg.buf[5] << 8) 
                | ((uint32_t)msg.buf[6] << 16) | ((uint32_t)msg.buf[7] << 24);

  const uint64_t master_us = (uint64_t)sec * 1'000'000ULL + (uint64_t)usec;
  const uint64_t local_us = micros64_();
  const int64_t offset = (int64_t)master_us - (int64_t)local_us;

  if (!have_offset_) {
    wall_offset_us_ = offset;
    have_offset_ = true;
  } else {
    const int64_t diff = offset - wall_offset_us_;
    wall_offset_us_ += (diff >> ALPHA_SHIFT_);
  }

  const int32_t residual = int32_t(offset - wall_offset_us_);
  stats_.add(residual);
}

void CanInterface::maybePrintSyncStats_() {
  if (!dbg_time_ || !dbg_) return;
  const uint64_t now = micros64_();
  if (now < nextPrint_us_) return;
  nextPrint_us_ = now + PRINT_PERIOD_US_;
  if (!stats_.n) return;

  const float mean = float(stats_.sum) / float(stats_.n);
  const float rms = sqrtf(float(stats_.sum_sq) / float(stats_.n));
  dbg_->printf("[TimeSync] mean=%+.1f us  rms=%.1f us  min=%+d  max=%+d  n=%lu\n",
               (double)mean, (double)rms, stats_.minv, stats_.maxv, (unsigned long)stats_.n);
  stats_.clear();
}