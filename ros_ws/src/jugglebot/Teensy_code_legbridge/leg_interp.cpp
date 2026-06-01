// =============================================================================
//  leg_interp.cpp — 500 Hz Hermite/Taylor interpolator (port of motor_guard)
// =============================================================================
//  Line-for-line transcription of the validated Python reference
//    tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py
//  (which the xref proves matches motor_guard to 0.0 rev). Keep the two in sync:
//  any change here must be mirrored there and re-validated by the xref.
// =============================================================================
#include "leg_interp.h"

#include <Arduino.h>          // IntervalTimer
#include <cstring>
#include "legbridge_config.h"
#include "udp_protocol.h"
#include "axis_state.h"
#include "odrive_protocol.h"
#include "can_buses.h"
#include "time_base.h"

namespace LegBridge {

// ── Ladder constants (mirror teensy_interp.py / motor_guard) ──────────────────
static constexpr float SEG_T   = SEGMENT_T_S;          // 0.025
static constexpr float MAXEXT  = MAX_EXTRAP_DT_S;       // 0.05
static constexpr float DECAY   = EXTRAP_DECAY_DT_S;     // 0.06
static constexpr float ALPHA   = JERK_EMA_ALPHA;        // 0.3
static constexpr float LEAD    = MAX_LEAD_REV;          // 0.15

// ── Active latched base state (read by the ISR) ───────────────────────────────
static float s_base_pos[NUM_LEGS];
static float s_base_vel[NUM_LEGS];
static float s_base_accel[NUM_LEGS];
static float s_base_torque[NUM_LEGS];
static float s_jerk[NUM_LEGS];
static float s_next_pos[NUM_LEGS];
static float s_next2_pos[NUM_LEGS];
static bool  s_has_next = false;
static bool  s_has_next2 = false;
static uint64_t s_base_ts_us = 0;
static bool  s_have_latched = false;

// Jerk history.
static float s_prev_accel[NUM_LEGS];
static bool  s_have_prev_accel = false;
static uint64_t s_prev_recv_us = 0;

// ── Staging (written by net task, consumed by ISR when s_pending) ─────────────
struct Staging {
  float u0[NUM_LEGS], u1[NUM_LEGS], u2[NUM_LEGS];
  float v0[NUM_LEGS], accel[NUM_LEGS], torque[NUM_LEGS];
  bool  has_u1, has_u2;
  uint64_t recv_us;
};
static Staging s_stage;
static volatile bool s_pending = false;

static volatile uint64_t s_last_setpoint_us = 0;
static volatile bool     s_output_enabled = false;   // Phase 8 owns this gate
// Deferred-stow descent state (set by the fault machine).
static volatile bool s_stow_active = false;
static volatile bool s_stow_complete = false;
static float s_stow_pos[NUM_LEGS];
static volatile uint32_t s_deadline_misses = 0;
static volatile uint32_t s_max_jitter_us = 0;
static uint64_t s_last_tick_us = 0;

static IntervalTimer s_timer;

// Deadline-miss threshold: a tick later than period + 25% is "missed".
static constexpr uint32_t JITTER_MISS_US = INTERP_PERIOD_US / 4;   // 500 us

// ── Setpoint handler (net task context) ───────────────────────────────────────
void interp_on_setpoint(uint16_t /*seq*/, const uint8_t* payload, uint16_t len) {
  if (len < JbUdp::SETPOINT_SIZE) return;
  JbUdp::SetpointPayload sp;
  memcpy(&sp, payload, sizeof(sp));

  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    s_stage.u0[i]     = sp.u0[i];
    s_stage.u1[i]     = sp.u1[i];
    s_stage.u2[i]     = sp.u2[i];
    s_stage.v0[i]     = sp.v0[i];
    s_stage.accel[i]  = sp.accel[i];
    s_stage.torque[i] = sp.torque_ff[i];
  }
  s_stage.has_u1 = (sp.flags & 0x1u) != 0;
  s_stage.has_u2 = (sp.flags & 0x2u) != 0;
  s_stage.recv_us = now_wall_us();

  s_last_setpoint_us = s_stage.recv_us;
  s_pending = true;          // publish last — staging is fully written above
}

// ── Latch (consume staging) — port of teensy_interp.latch_setpoint ────────────
static void latch_from_staging() {
  // Jerk EMA from consecutive accelerations.
  if (s_have_prev_accel) {
    const float dt_mpc = (float)((double)(s_stage.recv_us - s_prev_recv_us) * 1e-6);
    if (dt_mpc > 1e-6f) {
      for (uint8_t i = 0; i < NUM_LEGS; ++i) {
        const float raw = (s_stage.accel[i] - s_prev_accel[i]) / dt_mpc;
        s_jerk[i] = ALPHA * raw + (1.0f - ALPHA) * s_jerk[i];
      }
    }
  } else {
    for (uint8_t i = 0; i < NUM_LEGS; ++i) s_jerk[i] = 0.0f;
  }

  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    s_base_pos[i]    = s_stage.u0[i];
    s_base_vel[i]    = s_stage.v0[i];
    s_base_accel[i]  = s_stage.accel[i];
    s_base_torque[i] = s_stage.torque[i];
    s_next_pos[i]    = s_stage.u1[i];
    s_next2_pos[i]   = s_stage.u2[i];
    s_prev_accel[i]  = s_stage.accel[i];
  }
  s_has_next  = s_stage.has_u1;
  s_has_next2 = s_stage.has_u2 && s_stage.has_u1;
  s_base_ts_us = s_stage.recv_us;
  s_prev_recv_us = s_stage.recv_us;
  s_have_prev_accel = true;
  s_have_latched = true;
}

// ── 500 Hz ISR — port of teensy_interp.tick + CAN TX ──────────────────────────
static void interp_isr() {
  const uint64_t now_mono = micros64();
  // Jitter / deadline-miss accounting.
  if (s_last_tick_us != 0) {
    const uint32_t dt_tick = (uint32_t)(now_mono - s_last_tick_us);
    const uint32_t jit = (dt_tick > INTERP_PERIOD_US)
                         ? (dt_tick - INTERP_PERIOD_US) : (INTERP_PERIOD_US - dt_tick);
    if (jit > s_max_jitter_us) s_max_jitter_us = jit;
    if (dt_tick > INTERP_PERIOD_US + JITTER_MISS_US) s_deadline_misses++;
  }
  s_last_tick_us = now_mono;

  if (s_pending) { latch_from_staging(); s_pending = false; }

  // ── Deferred-stow descent (overrides the MPC ladder) ──
  // Velocity-limited per-leg descent to the off pose (stroke min), emitted on
  // CAN2. Runs only when output is enabled (the fault machine enables it on a
  // confirmed CAN2 reconnect). Mirrors _gently_move_to_setpoint(0.0).
  if (s_stow_active) {
    const float step = GENTLE_MOVE_VEL_LIMIT_RPS * (INTERP_PERIOD_US * 1e-6f);
    bool all_done = true;
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      const float target = STROKE_MIN_REV[i];      // off / fully-retracted pose
      float p = s_stow_pos[i];
      float v = 0.0f;
      if (p > target + step)      { p -= step; v = -GENTLE_MOVE_VEL_LIMIT_RPS; all_done = false; }
      else if (p < target - step) { p += step; v =  GENTLE_MOVE_VEL_LIMIT_RPS; all_done = false; }
      else                        { p = target; v = 0.0f; }
      s_stow_pos[i] = p;
      axes[i].target_pos_rev = p;
      axes[i].target_vel_rps = v;
      axes[i].target_torque_Nm = 0.0f;
      if (s_output_enabled) can2_send(ODrive::encode_leg_setpoint(i, p, v, 0.0f));
    }
    s_stow_complete = all_done;
    return;
  }

  if (!s_have_latched) return;

  const float dt = (float)((double)(now_wall_us() - s_base_ts_us) * 1e-6);
  float cmd_pos[NUM_LEGS], cmd_vel[NUM_LEGS], cmd_tor[NUM_LEGS];

  if (s_has_next) {
    // Mode 1: cubic Hermite between u0 and u1.
    float s = dt / SEG_T;
    if (s > 1.0f) s = 1.0f;
    const float s2 = s * s, s3 = s2 * s;
    const float h00 = 2.0f * s3 - 3.0f * s2 + 1.0f;
    const float h10 = s3 - 2.0f * s2 + s;
    const float h01 = -2.0f * s3 + 3.0f * s2;
    const float h11 = s3 - s2;
    const float invT = 1.0f / SEG_T;
    const float dh00 = (6.0f * s2 - 6.0f * s) * invT;
    const float dh10 = 3.0f * s2 - 4.0f * s + 1.0f;
    const float dh01 = (-6.0f * s2 + 6.0f * s) * invT;
    const float dh11 = 3.0f * s2 - 2.0f * s;
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      const float p0 = s_base_pos[i], p1 = s_next_pos[i], v0 = s_base_vel[i];
      const float v1 = s_has_next2 ? (s_next2_pos[i] - p1) / SEG_T : (p1 - p0) / SEG_T;
      cmd_pos[i] = h00 * p0 + h10 * (SEG_T * v0) + h01 * p1 + h11 * (SEG_T * v1);
      cmd_vel[i] = dh00 * p0 + dh10 * v0 + dh01 * p1 + dh11 * v1;
    }
  } else if (dt <= MAXEXT) {
    // Mode 2: cubic Taylor extrapolation.
    const float dt2 = dt * dt;
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      cmd_pos[i] = s_base_pos[i] + s_base_vel[i] * dt
                   + 0.5f * s_base_accel[i] * dt2
                   + (1.0f / 6.0f) * s_jerk[i] * (dt2 * dt);
      cmd_vel[i] = s_base_vel[i] + s_base_accel[i] * dt + 0.5f * s_jerk[i] * dt2;
    }
  } else {
    // Mode 3: velocity decay to zero.
    const float dt_b2 = MAXEXT * MAXEXT;
    const float dt_over = dt - MAXEXT;
    float decay = 1.0f - dt_over / DECAY;
    if (decay < 0.0f) decay = 0.0f;
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      const float vel_b = s_base_vel[i] + s_base_accel[i] * MAXEXT + 0.5f * s_jerk[i] * dt_b2;
      const float pos_b = s_base_pos[i] + s_base_vel[i] * MAXEXT
                          + 0.5f * s_base_accel[i] * dt_b2
                          + (1.0f / 6.0f) * s_jerk[i] * (dt_b2 * MAXEXT);
      const float extra = (dt_over >= DECAY)
          ? vel_b * (DECAY * 0.5f)
          : vel_b * dt_over * (1.0f - dt_over / (2.0f * DECAY));
      cmd_pos[i] = pos_b + extra;
      cmd_vel[i] = vel_b * decay;
    }
  }

  for (uint8_t i = 0; i < NUM_LEGS; ++i) cmd_tor[i] = s_base_torque[i];

  // Lead clamp (never run more than LEAD ahead of the encoder).
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    const float fb = axes[i].pos_rev;          // single-word atomic read
    const float pre = cmd_pos[i];
    float dev = cmd_pos[i] - fb;
    if (dev > LEAD) dev = LEAD; else if (dev < -LEAD) dev = -LEAD;
    cmd_pos[i] = fb + dev;
    if (cmd_pos[i] != pre) cmd_vel[i] = 0.0f;
  }
  // Stroke clamp (physical backstop).
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    const float pre = cmd_pos[i];
    if (cmd_pos[i] < STROKE_MIN_REV[i]) cmd_pos[i] = STROKE_MIN_REV[i];
    else if (cmd_pos[i] > STROKE_MAX_REV[i]) cmd_pos[i] = STROKE_MAX_REV[i];
    if (cmd_pos[i] != pre) { cmd_vel[i] = 0.0f; cmd_tor[i] = 0.0f; }
  }

  // Publish targets into the cache (for telemetry) and transmit to CAN2.
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    axes[i].target_pos_rev   = cmd_pos[i];
    axes[i].target_vel_rps   = cmd_vel[i];
    axes[i].target_torque_Nm = cmd_tor[i];
  }
  if (s_output_enabled) {
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      can2_send(ODrive::encode_leg_setpoint(i, cmd_pos[i], cmd_vel[i], cmd_tor[i]));
    }
  }
}

void leg_interp_init() {
  // Begin the 500 Hz tick. priority(): lower = more urgent on the Cortex-M NVIC.
  // We want the interp ISR ABOVE the FreeRTOS syscall ceiling so RTOS critical
  // sections never delay it — and the ISR therefore makes NO FreeRTOS calls.
  // The exact value vs configMAX_SYSCALL_INTERRUPT_PRIORITY must be confirmed on
  // the bench (handoff "Needs hardware validation").
  s_timer.begin(interp_isr, INTERP_PERIOD_US);
  s_timer.priority(32);
}

uint64_t interp_last_setpoint_us() { return s_last_setpoint_us; }
void interp_set_output_enabled(bool en) { s_output_enabled = en; }
bool interp_output_enabled() { return s_output_enabled; }
uint32_t interp_deadline_misses() { return s_deadline_misses; }
uint32_t interp_max_jitter_us() { return s_max_jitter_us; }
void interp_reset_jitter() { s_max_jitter_us = 0; }

void interp_begin_stow() {
  for (uint8_t i = 0; i < NUM_LEGS; ++i) s_stow_pos[i] = axes[i].pos_rev;  // start from actual
  s_stow_complete = false;
  s_stow_active = true;
}
void interp_end_stow() { s_stow_active = false; }
bool interp_stow_active() { return s_stow_active; }
bool interp_stow_complete() { return s_stow_complete; }

}  // namespace LegBridge
