#include "CanInterface.h"
#include <string.h>
#include <math.h>

// ---------------- Command-name table (optional; PROGMEM friendly) -------------
static const CanInterface::CmdName kCmdNames[] PROGMEM = {
  { CanInterface::Cmd::heartbeat_message,       "heartbeat_message" },
  { CanInterface::Cmd::get_error,               "get_error" },
  { CanInterface::Cmd::RxSdo,                   "RxSdo" },
  { CanInterface::Cmd::TxSdo,                   "TxSdo" },
  { CanInterface::Cmd::set_requested_state,     "set_requested_state" },
  { CanInterface::Cmd::get_encoder_estimate,    "get_encoder_estimate" },
  { CanInterface::Cmd::set_controller_mode,     "set_controller_mode" },
  { CanInterface::Cmd::set_input_pos,           "set_input_pos" },
  { CanInterface::Cmd::set_input_vel,           "set_input_vel" },
  { CanInterface::Cmd::set_vel_curr_limits,     "set_vel_curr_limits" },
  { CanInterface::Cmd::set_traj_vel_limit,      "set_traj_vel_limit" },
  { CanInterface::Cmd::set_traj_acc_limits,     "set_traj_acc_limits" },
  { CanInterface::Cmd::get_iq,                  "get_iq" },
  { CanInterface::Cmd::get_temps,               "get_temps" },
  { CanInterface::Cmd::reboot_odrives,          "reboot_odrives" },
  { CanInterface::Cmd::get_bus_voltage_current, "get_bus_voltage_current" },
  { CanInterface::Cmd::clear_errors,            "clear_errors" },
  { CanInterface::Cmd::set_absolute_position,   "set_absolute_position" },
  { CanInterface::Cmd::set_pos_gain,            "set_pos_gain" },
  { CanInterface::Cmd::set_vel_gains,           "set_vel_gains" },
};

const CanInterface::CmdName* CanInterface::commandNameTable(size_t& count) {
  count = sizeof(kCmdNames)/sizeof(kCmdNames[0]);
  return kCmdNames;
}

// ---------------- static singleton ----------------
CanInterface* CanInterface::s_instance_ = nullptr;

// ---------------- ctor ----------------
CanInterface::CanInterface() : can1_() {}

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
    home_state_[i]    = AxisHomeState::Unhomed;   // default: not homed
  }

  // Initialise last heartbeats arrays
  for (int i = 0; i < 64; ++i) {
    axes_pv_[i].valid = false;
    axes_iq_[i].valid = false;
    hb_[i].valid      = false;
  }

  // Initialise BRAKE_RESISTOR_DISARMED auto-clearing stuff
  for (int i = 0; i < 64; ++i) {
    last_brake_clear_us_[i] = 0;
  }
}

// ---------------- loop ----------------
void CanInterface::loop() {
  can1_.events();          // dispatch FIFO -> rxTrampoline_
  maybePrintSyncStats_();  // optional stats print
}

// ---------------- debug ----------------
void CanInterface::setDebugStream(Stream* dbg) { dbg_ = dbg; }
void CanInterface::setDebugFlags(bool timeSyncDebug, bool canDebug) {
  dbg_time_ = timeSyncDebug;
  dbg_can_  = canDebug;
}

// ---------------- time sync public ----------------
void CanInterface::setTimeSyncId(uint32_t id) { timeSyncId_ = id; }
uint64_t CanInterface::localTimeUs() const { return micros64_(); }
uint64_t CanInterface::wallTimeUs() const {
  return uint64_t(int64_t(localTimeUs()) + wall_offset_us_);
}

CanInterface::SyncStats CanInterface::getAndClearSyncStats() {
  SyncStats out;
  if (stats_.n) {
    out.mean_us = float(stats_.sum) / float(stats_.n);
    out.rms_us  = sqrtf(float(stats_.sum_sq) / float(stats_.n));
    out.min_us  = stats_.minv;
    out.max_us  = stats_.maxv;
    out.frames  = stats_.n;
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
  if (dbg_can_ && dbg_) dbg_->printf("[CAN->] id=0x%03lX len=%u %s\n", (unsigned long)id, (unsigned)len, ok ? "OK" : "FAIL");
  return ok;
}

bool CanInterface::sendRTR(uint32_t id, uint8_t len) {
  CAN_message_t m;
  m.id = id;
  m.len = len;
  m.flags.extended = 0;
  m.flags.remote = 1; // RTR
  const bool ok = can1_.write(m);
  if (dbg_can_ && dbg_) dbg_->printf("[CAN->RTR] id=0x%03lX len=%u %s\n", (unsigned long)id, (unsigned)len, ok ? "OK" : "FAIL");
  return ok;
}

// ---------------- Handling Jetson Commands ----------------
void CanInterface::setHostThrowCmdId(uint32_t id) { host_throw_id_ = (id & 0x7FFu); }
bool CanInterface::getHostThrowCmd(HostThrowCmd& out) const {
  if (!last_host_cmd_.valid) return false;
  out = last_host_cmd_;
  return true;
}
void CanInterface::setHostThrowCallback(HostThrowCallback cb, void* user) {
  host_cb_ = cb; host_cb_user_ = user;
}

// ---------------- ODrive helpers ----------------
static inline void wrFloatLE(uint8_t* b, float f) { memcpy(b, &f, 4); }
static inline void wrU32LE(uint8_t* b, uint32_t v) { b[0]=v&0xFF; b[1]=(v>>8)&0xFF; b[2]=(v>>16)&0xFF; b[3]=(v>>24)&0xFF; }

int16_t CanInterface::clampToI16_(float x) {
  if (x >  32767.f) return  32767;
  if (x < -32768.f) return -32768;
  return (int16_t)lrintf(x);
}

bool CanInterface::setRequestedState(uint32_t node_id, uint32_t requested_state) {
  uint8_t d[8] = {0};
  wrU32LE(&d[0], requested_state);
  return sendRaw(makeId(node_id, Cmd::set_requested_state), d, 8);
}

bool CanInterface::setControllerMode(uint32_t node_id, uint32_t control_mode, uint32_t input_mode) {
  uint8_t d[8] = {0};
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
  uint8_t d[8] = {0};
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
  uint8_t d[8] = {0};
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
  uint8_t d[8] = {0};
  wrFloatLE(&d[0], pos_rev);
  return sendRaw(makeId(node_id, Cmd::set_absolute_position), d, 8);
}

bool CanInterface::clearErrors(uint32_t node_id) {
  uint8_t d[8] = {0};
  return sendRaw(makeId(node_id, Cmd::clear_errors), d, 8);
}

bool CanInterface::reboot(uint32_t node_id) {
  uint8_t d[8] = {0};
  return sendRaw(makeId(node_id, Cmd::reboot_odrives), d, 8);
}

bool CanInterface::sendInputPos(uint32_t node_id,
                                float pos_rev,
                                float vel_ff_rev_per_s,
                                float torque_ff)
{
  if (!isAxisMotionAllowed(node_id)) {
    if (dbg_) dbg_->printf("[Gate] Blocked set_input_pos to node %lu (not homed)\n", (unsigned long)node_id);
    return false;
  }
  uint8_t d[8];
  wrFloatLE(&d[0], pos_rev);
  const int16_t vel_i = clampToI16_(vel_ff_rev_per_s * kVelScale_);
  const int16_t tor_i = clampToI16_(torque_ff * kTorScale_);
  d[4] = uint8_t( vel_i       & 0xFF);
  d[5] = uint8_t((vel_i >> 8) & 0xFF);
  d[6] = uint8_t( tor_i       & 0xFF);
  d[7] = uint8_t((tor_i >> 8) & 0xFF);
  const bool ok = sendRaw(makeId(node_id, Cmd::set_input_pos), d, 8);
  if (dbg_can_ && dbg_) {
    dbg_->printf("[CAN->ODrive pos] node=%lu pos=%.4f vel_ff=%.3f tor_ff=%.3f ok=%d\n",
                 (unsigned long)node_id, (double)pos_rev,
                 (double)vel_ff_rev_per_s, (double)torque_ff, (int)ok);
  }
  return ok;
}

bool CanInterface::sendInputVel(uint32_t node_id, float vel_rps, float torque_ff) {
  if (!isAxisMotionAllowed(node_id)) {
    if (dbg_) dbg_->printf("[Gate] Blocked set_input_vel to node %lu (not homed)\n", (unsigned long)node_id);
    return false;
  }
  uint8_t d[8];
  wrFloatLE(&d[0], vel_rps);
  const int16_t tor_i = clampToI16_(torque_ff * kTorScale_);
  d[4] = uint8_t( tor_i       & 0xFF);
  d[5] = uint8_t((tor_i >> 8) & 0xFF);
  d[6] = 0; d[7] = 0;
  const bool ok = sendRaw(makeId(node_id, Cmd::set_input_vel), d, 8);
  if (dbg_can_ && dbg_) {
    dbg_->printf("[CAN->ODrive vel] node=%lu vel=%.4f tor_ff=%.3f ok=%d\n",
                 (unsigned long)node_id, (double)vel_rps, (double)torque_ff, (int)ok);
  }
  return ok;
}

bool CanInterface::restoreHandToOperatingConfig(uint32_t node_id,
                                     float vel_limit_rps,
                                     float current_limit_A)
{
  constexpr uint32_t AXIS_STATE_IDLE        = 1u;
  constexpr uint32_t CONTROL_MODE_POSITION  = 3u;
  constexpr uint32_t INPUT_MODE_PASSTHROUGH = 1u;

  bool ok = true;
  ok &= setRequestedState(node_id, AXIS_STATE_IDLE);
  ok &= setControllerMode(node_id, CONTROL_MODE_POSITION, INPUT_MODE_PASSTHROUGH);
  ok &= setVelCurrLimits(node_id, current_limit_A, vel_limit_rps);
  return ok;
}

// ---------------- estimator & iq getters ----------------
bool CanInterface::getAxisPV(uint32_t node_id, float& pos_out, float& vel_out, uint64_t& wall_us_out) const {
  if (node_id >= 64) return false;
  uint64_t t1, t2;
  float p, v;
  do {
    t1 = axes_pv_[node_id].wall_us;
    p  = axes_pv_[node_id].pos_rev;
    v  = axes_pv_[node_id].vel_rps;
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

// ---------------- homing ----------------
bool CanInterface::homeHand(uint32_t node_id,
                            float homing_speed_rps,
                            float current_limit_A,
                            float current_headroom_A,
                            float settle_pos_rev,
                            float avg_weight,
                            uint16_t iq_poll_period_ms)
{
  // Update the state
  setHomeState(node_id, AxisHomeState::Homing);

  // 1) CLOSED_LOOP_CONTROL (8) — ODrive enum
  if (!setRequestedState(node_id, 8u)) return false;

  // 2) VELOCITY_CONTROL (2) + VEL_RAMP (2)
  if (!setControllerMode(node_id, 2u, 2u)) return false;

  // 3) Limits: Current_Limit = current_limit + headroom; Vel_Limit = |2x homing speed|
  const float vel_limit = fabsf(homing_speed_rps * 2.0f);
  if (!setVelCurrLimits(node_id, current_limit_A + current_headroom_A, vel_limit)) return false;

  delay(10);

  // 4) Start moving
  if (!sendInputVel(node_id, homing_speed_rps, 0.0f)) {
    // gate may have blocked (or bus error) → tidy up and revert to Unhomed
    setRequestedState(node_id, 1u); // IDLE
    setHomeState(node_id, AxisHomeState::Unhomed);
    return false;
  }
  if (dbg_) dbg_->printf("[Home] node=%lu moving at %.3f rps; Iq limit=%.2f A\n",
                         (unsigned long)node_id, (double)homing_speed_rps, (double)current_limit_A);

  // 5) EMA + polling Iq via RTR
  float ema = 0.0f;
  uint32_t last_poll = 0;
  for (;;) {
    // Poll Iq periodically (host → RTR; drive replies with 8 bytes: float meas, float setp)
    const uint32_t now_ms = millis();
    if ((now_ms - last_poll) >= iq_poll_period_ms) {
      last_poll = now_ms;
      sendRTR(makeId(node_id, Cmd::get_iq), 8);
    }

    // Service CAN and read available Iq
    can1_.events();

    float iq_meas, iq_setp; uint64_t t_us;
    if (getAxisIq(node_id, iq_meas, iq_setp, t_us)) {
      const float alpha = constrain(avg_weight, 0.f, 0.9999f);
      ema = alpha * ema + (1.f - alpha) * iq_meas;

      if (dbg_can_ && dbg_) {
        dbg_->printf("[Home] Iq_meas=%.3f A  EMA=%.3f A\n", (double)iq_meas, (double)ema);
      }

      if (fabsf(ema) >= current_limit_A) {
        // Hit the end stop → stop and set position
        setRequestedState(node_id, 1u); // IDLE (1)
        delay(5);
        setAbsolutePosition(node_id, settle_pos_rev);
        if (dbg_) dbg_->printf("[Home] node=%lu homed. Set pos to %.3f rev.\n",
                               (unsigned long)node_id, (double)settle_pos_rev);

        // Return the ODrive to its normal operating conditions
        restoreHandToOperatingConfig(node_id);  

        // Update the state
        setHomeState(node_id, AxisHomeState::Homed);
        return true;
      }
    }

    // brief yield
    delay(1);
  }

  // unreachable
  // return false;
}

bool CanInterface::homeHandStandard(uint32_t node_id,
                                    int hand_direction,
                                    float base_speed_rps,
                                    float current_limit_A,
                                    float current_headroom_A,
                                    float set_abs_pos_rev)
{
  // As in your Python: homing_speed = hand_direction * 3.0; but when "running downwards" you inverted.
  // Here we pass the speed directly — you choose sign when calling.
  // Your Python used set_abs_pos to hand_direction * -0.1; pass that via set_abs_pos_rev.
  const float homing_speed = float(hand_direction) * base_speed_rps;
  return homeHand(node_id,
                  /*homing_speed_rps=*/homing_speed,
                  /*current_limit_A=*/current_limit_A,
                  /*current_headroom_A=*/current_headroom_A,
                  /*settle_pos_rev=*/set_abs_pos_rev,
                  /*avg_weight=*/0.7f,
                  /*iq_poll_period_ms=*/10);
}

// ---------------- Home State ----------------
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
  else          home_required_mask_ &= ~bit;
}

bool CanInterface::isHomeRequired(uint32_t node_id) const {
  if (node_id >= 64) return false;
  return (home_required_mask_ & (1ULL << node_id)) != 0;
}

void CanInterface::requireHomeOnlyFor(uint32_t node_id) {
  if (node_id >= 64) { home_required_mask_ = 0; return; }
  home_required_mask_ = (1ULL << node_id);
}

void CanInterface::clearHomeRequirements() { home_required_mask_ = 0; }

bool CanInterface::isAxisMotionAllowed(uint32_t node_id) const {
  if (node_id >= 64) return false;
  // If this axis doesn't require homing, always allow motion.
  if (!isHomeRequired(node_id)) return true;

  // If it does require homing, allow only during Homing/Homed.
  AxisHomeState s = home_state_[node_id];
  return s == AxisHomeState::Homing || s == AxisHomeState::Homed;
}

// ---------------- Get Axis Heartbeat ----------------
bool CanInterface::getAxisHeartbeat(uint32_t node_id, AxisHeartbeat& out) const {
  if (node_id >= 64) return false;
  // atomic-ish copy
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

bool CanInterface::waitForAxisErrorClear(uint32_t node_id, uint32_t mask,
                                         uint32_t timeout_ms, uint16_t poll_ms) {
  const uint32_t start = millis();
  while ((millis() - start) < timeout_ms) {
    const_cast<CanInterface*>(this)->loop(); // pump CAN
    AxisHeartbeat hb;
    if (getAxisHeartbeat(node_id, hb) && (hb.axis_error & mask) == 0u) return true;
    delay(poll_ms);
  }
  return false;
}

// ---------------- Auto clear brake resistor disarmed error ----------------
void CanInterface::setAutoClearBrakeResistor(bool enable, uint32_t min_interval_ms) {
  auto_clear_brake_res_   = enable;
  auto_clear_interval_ms_ = min_interval_ms ? min_interval_ms : 1; // avoid 0
}

// ---------------- config ----------------
void CanInterface::setEstimatorCmd(uint8_t cmd) { estimatorCmd_ = (cmd & 0x1F); }

// ---------------- RX trampoline/handler ----------------
void CanInterface::rxTrampoline_(const CAN_message_t &msg) {
  if (s_instance_) s_instance_->handleRx_(msg);
}

void CanInterface::handleRx_(const CAN_message_t &msg) {
  // Time sync
  if (msg.id == timeSyncId_ && msg.len == 8 && !msg.flags.remote) {
    handleTimeSync_(msg);
    return;
  }

  // Decode node/cmd
  const uint32_t node = (msg.id >> 5) & 0x3F;
  const uint8_t  cmd  = msg.id & 0x1F;

  // Host -> Teensy throw command (8 bytes on a dedicated ID)
  if (host_throw_id_ && msg.id == host_throw_id_ && msg.len == 8 && !msg.flags.remote) {
    // Fields (LE):
    // 0-1: yaw_i16 scaled: yaw = yaw_i16 * pi / 32768   (±pi)
    // 2-3: pitch_u16 scaled: pitch = pitch_u16 * (pi / 65536)  (0..pi/2 used)
    // 4-5: speed_u16 scaled: speed = speed_u16 * 0.0001 m/s
    // 6-7: time_u16 scaled: in_s = time_u16 * 0.001 s
    const int16_t  yaw_i   =  (int16_t)( (uint16_t)msg.buf[0] | ((uint16_t)msg.buf[1] << 8) );
    const uint16_t pit_u   =  (uint16_t)msg.buf[2] | ((uint16_t)msg.buf[3] << 8);
    const uint16_t sp_u    =  (uint16_t)msg.buf[4] | ((uint16_t)msg.buf[5] << 8);
    const uint16_t t_u     =  (uint16_t)msg.buf[6] | ((uint16_t)msg.buf[7] << 8);

    HostThrowCmd c;
    c.yaw_rad   = float(yaw_i) * (float)M_PI / 32768.0f;    // clamp to [-pi, +pi]
    c.pitch_rad = float(pit_u) * ((float)M_PI / 65536.0f); // 0..(~pi/2 - 1 LSB)
    c.speed_mps = float(sp_u) * 0.0001f;                    // 0.1 mm/s per LSB
    c.in_s      = float(t_u) * 0.001f;                      // 1 ms per LSB

    // Range guard (host should validate, but be defensive)
    if (c.yaw_rad < -M_PI) c.yaw_rad = -M_PI;
    if (c.yaw_rad >  M_PI) c.yaw_rad =  M_PI;
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

  // Heartbeat: axis_error (u32) | state (u8) | proc_result (u8) | traj_done (u8)
  if (cmd == uint8_t(Cmd::heartbeat_message) && node < 64 && msg.len >= 7 && !msg.flags.remote) {
    uint32_t axis_error = (uint32_t)msg.buf[0]
                        | ((uint32_t)msg.buf[1] << 8)
                        | ((uint32_t)msg.buf[2] << 16)
                        | ((uint32_t)msg.buf[3] << 24);

    hb_[node].axis_error       = axis_error;
    hb_[node].axis_state       = msg.buf[4];
    hb_[node].procedure_result = msg.buf[5];
    hb_[node].trajectory_done  = msg.buf[6];
    hb_[node].wall_us          = wallTimeUs();
    hb_[node].valid            = true;

    // If spurious BRAKE_RESISTOR_DISARMED bit is present, auto-clear with throttle.
    if (auto_clear_brake_res_ && (axis_error & ODriveErrors::BRAKE_RESISTOR_DISARMED)) {
      const uint64_t now_us = micros64_();  // local clock is fine for throttling
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

    // (Optional) You can also inspect Axis_State (buf[4]), Procedure_Result (buf[5]), Traj_Done (buf[6]) here.

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
    axes_pv_[node].valid   = true;
    if (dbg_can_ && dbg_) {
      dbg_->printf("[CAN<-PV] node=%lu pos=%.4f rev vel=%.4f rps id=0x%03lX\n",
                   (unsigned long)node, (double)pos, (double)vel, (unsigned long)msg.id);
    }
    return;
  }

  // Iq reply (float meas, float setp)
  if (cmd == uint8_t(Cmd::get_iq) && msg.len == 8 && node < 64 && !msg.flags.remote) {
    float iq_meas, iq_setp;
    memcpy(&iq_meas, &msg.buf[0], 4);
    memcpy(&iq_setp, &msg.buf[4], 4);
    axes_iq_[node].iq_meas = iq_meas;
    axes_iq_[node].iq_setp = iq_setp;
    axes_iq_[node].wall_us = wallTimeUs();
    axes_iq_[node].valid   = true;
    if (dbg_can_ && dbg_) {
      dbg_->printf("[CAN<-Iq] node=%lu Iq=%.3fA set=%.3fA id=0x%03lX\n",
                   (unsigned long)node, (double)iq_meas, (double)iq_setp, (unsigned long)msg.id);
    }
    return;
  }

  // Optional general CAN debug
  if (dbg_can_ && dbg_) {
    dbg_->printf("[CAN] id=0x%03lX len=%d rtr=%d\n",
                 (unsigned long)msg.id, msg.len, (int)msg.flags.remote);
  }
}

// ---------------- Time sync handler ----------------
void CanInterface::handleTimeSync_(const CAN_message_t &msg) {
  // Payload: [uint32 sec][uint32 usec], little-endian
  uint32_t sec  = (uint32_t)msg.buf[0] |
                  ((uint32_t)msg.buf[1] << 8) |
                  ((uint32_t)msg.buf[2] << 16) |
                  ((uint32_t)msg.buf[3] << 24);
  uint32_t usec = (uint32_t)msg.buf[4] |
                  ((uint32_t)msg.buf[5] << 8) |
                  ((uint32_t)msg.buf[6] << 16) |
                  ((uint32_t)msg.buf[7] << 24);

  const uint64_t master_us = (uint64_t)sec * 1'000'000ULL + (uint64_t)usec;
  const uint64_t local_us  = micros64_();
  const int64_t  offset    = (int64_t)master_us - (int64_t)local_us;

  if (!have_offset_) {
    wall_offset_us_ = offset;
    have_offset_ = true;
  } else {
    const int64_t diff = offset - wall_offset_us_;
    wall_offset_us_ += (diff >> ALPHA_SHIFT_);  // IIR
  }

  const int32_t residual = int32_t(offset - wall_offset_us_);
  stats_.add(residual);
}

// ---------------- maybe print sync stats ----------------
void CanInterface::maybePrintSyncStats_() {
  if (!dbg_time_ || !dbg_) return;
  const uint64_t now = micros64_();
  if (now < nextPrint_us_) return;
  nextPrint_us_ = now + PRINT_PERIOD_US_;
  if (!stats_.n) return;

  const float mean = float(stats_.sum) / float(stats_.n);
  const float rms  = sqrtf(float(stats_.sum_sq) / float(stats_.n));
  dbg_->printf("[TimeSync] mean=%+.1f us  rms=%.1f us  min=%+d  max=%+d  n=%lu\n",
               (double)mean, (double)rms, stats_.minv, stats_.maxv, (unsigned long)stats_.n);
  stats_.clear();
}
