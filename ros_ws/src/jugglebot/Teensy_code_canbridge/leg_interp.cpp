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
#include <cmath>              // std::isfinite (setpoint trust-boundary drop — Flash-A item 5)
#include "canbridge_config.h"
#include "udp_protocol.h"
#include "axis_state.h"
#include "odrive_protocol.h"
#include "can_buses.h"
#include "time_base.h"
#include "leg_homing.h"       // homing_active     (in-progress-move interlock — Flash-A item 1a)
#include "leg_activate.h"     // activate_active   (in-progress-move interlock — Flash-A item 1a)
#include "leg_deactivate.h"   // deactivate_active (in-progress-move interlock — Flash-A item 1a)

namespace CanBridge {

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

// Last-ACCEPTED SETPOINT sequence — a wrap-safe monotonic drop of stale/duplicate
// frames (Flash-A item 5). Touched ONLY in interp_on_setpoint (net-task context)
// and interp_reset (quiescent), never by the ISR, so no volatile/IRQ guard needed.
static uint16_t s_last_sp_seq = 0;
static bool     s_have_sp_seq = false;

static volatile uint64_t s_last_setpoint_us = 0;
static volatile bool     s_output_enabled = false;   // Phase 8 owns this gate
// Deferred-stow descent state (set by the fault machine).
static volatile bool s_stow_active = false;
static volatile bool s_stow_complete = false;
static float s_stow_pos[NUM_LEGS];
static float s_stow_speed = 0.0f;        // accel-ramped descent speed (rev/s)
static volatile uint32_t s_deadline_misses = 0;
static volatile uint32_t s_max_jitter_us = 0;
static uint64_t s_last_tick_us = 0;

static IntervalTimer s_timer;

// Deadline-miss threshold: a tick later than period + 25% is "missed".
static constexpr uint32_t JITTER_MISS_US = INTERP_PERIOD_US / 4;   // 500 us

// ── Setpoint handler (net task context) ───────────────────────────────────────
void interp_on_setpoint(uint16_t seq, const uint8_t* payload, uint16_t len) {
  if (len < JbUdp::SETPOINT_SIZE) return;

  // ── Session/stream-gap re-baseline (adversarial-review fix, before the seq guard) ─
  // The seq high-water (s_last_sp_seq) persists for the whole Teensy uptime
  // (interp_reset() has no runtime caller, and the Jetson-5V-powered can-bridge
  // OUTLIVES run_mpc). But the host resets its SHARED _tx_seq_stream counter to 0 on
  // every TeensyLinkClient construction (a routine bridge-node/ROS2 restart). Without
  // this reset, after ~50% of restarts the guard below would drop EVERY setpoint until
  // the counter climbed past the stale high-water — up to ~32767 frames (~13 min) — a
  // phantom, self-re-latching MPC_STALE E-STOP with the link showing up. If the last
  // ACCEPTED setpoint is already older than the staleness bound, the prior stream is
  // DEAD (a restart, or a >250 ms gap): there is no in-flight reordering left to
  // protect, so forget the high-water and re-baseline on the next frame. A live 40 Hz
  // stream (~25 ms gaps) never crosses MPC_CMD_STALENESS_US, so this never
  // false-resets within a session.
  if (s_have_sp_seq &&
      (micros64() - atomic_read_u64(&s_last_setpoint_us)) > MPC_CMD_STALENESS_US)
    s_have_sp_seq = false;

  // ── Wrap-safe monotonic-seq guard (Flash-A item 5) ──────────────────────────
  // Drop a stale or duplicate setpoint (out-of-order UDP delivery / a re-sent
  // frame). CRITICAL PARITY TRAP: SETPOINT and HEARTBEAT_J2T SHARE the host's
  // _tx_seq_stream counter (controller/teensy_link/client.py send_stream), so
  // setpoint seqs are strictly MONOTONIC but NOT contiguous — a heartbeat between
  // two setpoints consumes an intervening seq value. A `seq == last+1` guard would
  // therefore false-drop EVERY setpoint that follows a heartbeat. The test MUST be
  // strictly-greater via a SIGNED int16 wrap difference, tracking the last-ACCEPTED
  // setpoint seq only: (int16_t)(seq - s_last_sp_seq) <= 0 ⇒ stale/dup ⇒ drop.
  if (s_have_sp_seq && (int16_t)(seq - s_last_sp_seq) <= 0) return;

  JbUdp::SetpointPayload sp;
  memcpy(&sp, payload, sizeof(sp));

  // ── isfinite trust-boundary drop (Flash-A item 5) ───────────────────────────
  // A NaN/Inf field survives BOTH the lead clamp and the stroke clamp (a NaN fails
  // every comparison, so no clamp branch fires) and would reach encode_leg_setpoint
  // → the CAN wire. DROP the whole frame like a lost UDP frame — do NOT sanitize to
  // zero (a zeroed setpoint could trip MAX_DEVIATION or lurch a leg). CRITICAL: a
  // dropped frame does NOT advance s_last_setpoint_us below, so a stream of all-NaN
  // frames still eventually trips MPC_STALE (the staleness watchdog fires as if the
  // link went quiet), and does NOT advance s_last_sp_seq (only an accepted frame does).
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    if (!(std::isfinite(sp.u0[i]) && std::isfinite(sp.u1[i]) && std::isfinite(sp.u2[i]) &&
          std::isfinite(sp.v0[i]) && std::isfinite(sp.accel[i]) && std::isfinite(sp.torque_ff[i])))
      return;
  }

  const uint64_t recv = micros64();   // monotonic (item 14): feeds s_base_ts_us / s_last_setpoint_us / jerk dt,
                                      // all read against micros64() — a wall step must not corrupt the trajectory phase

  // Publish the staging atomically. The interp ISR runs ABOVE the FreeRTOS
  // syscall ceiling and can preempt this net-task write at any instruction; if
  // a second setpoint were written field-by-field while s_pending is still set
  // from a first, the ISR could latch a half-written mix. Disabling IRQs around
  // the fill + flag (PRIMASK save/restore) makes the publish a single atomic,
  // fully-ordered step (also serves as the compiler/memory barrier the seqlock
  // would otherwise provide). ~50 words copied → a couple of µs of IRQ-off.
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
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
  s_stage.recv_us = recv;
  s_pending = true;
  __set_PRIMASK(pm);

  // Only an ACCEPTED frame advances the seq high-water mark + the staleness clock.
  s_last_sp_seq = seq;
  s_have_sp_seq = true;
  atomic_write_u64(&s_last_setpoint_us, recv);   // 64-bit; read by the fault task
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

  // ── In-progress-move interlock (Flash-A item 1a) ────────────────────────────
  // A cold-start move (firmware homing / activate / deactivate) drives the SAME leg
  // ODrives this ISR streams to; if both TX at once they co-drive the legs (a fight
  // that can jerk the platform). The fault-task (FAULT_TASK_HZ = 10 Hz) is too slow
  // to gate this — up to 100 ms of co-driving — so suppress the leg TX HERE, at the
  // 500 Hz ISR (zero latency). These predicates make NO FreeRTOS calls (preserving the
  // ISR's above-syscall-ceiling contract): each reads a single-word file-static
  // (s_start_req volatile, s_phase a naturally-aligned enum → an atomic load on
  // Cortex-M7) via a CROSS-TU function call — declared in leg_*.h, defined in the
  // .cpp, so with the shipping -O2/no-LTO build the compiler cannot inline/hoist them
  // and the call itself is the reload barrier, so the ISR sees each phase transition
  // within one tick. (Were LTO ever enabled, s_phase should be marked volatile to
  // match s_start_req — see leg_homing/activate/deactivate.cpp.) The target cache
  // writes below stay UNCONDITIONAL (telemetry must keep flowing during a move).
  const bool coldstart = homing_active() || activate_active() || deactivate_active();

  // ── Deferred-stow descent (overrides the MPC ladder) ──
  // Velocity-limited per-leg descent to the off pose (0.0 rev = fully retracted),
  // emitted on CAN3 (the Jugglebot core bus). Runs only when output is enabled
  // (the fault machine enables it on a confirmed CAN3 reconnect). Mirrors
  // _gently_move_to_setpoint(0.0).
  if (s_stow_active) {
    const float dt_tick = INTERP_PERIOD_US * 1e-6f;
    // Accel-limit the descent speed (ramps 0 → limit) so there is no startup
    // velocity step — a trapezoidal-ish profile rather than a constant-velocity
    // jump. (The deceleration near the target is handled by the within-step band.)
    s_stow_speed += STOW_ACCEL_RPS2 * dt_tick;
    if (s_stow_speed > GENTLE_MOVE_VEL_LIMIT_RPS) s_stow_speed = GENTLE_MOVE_VEL_LIMIT_RPS;
    const float step = s_stow_speed * dt_tick;
    bool all_done = true;
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      const float target = STOW_OFF_POSE_REV;     // 0.0 — matches can_node _gently_move_to_setpoint(0.0)
      float p = s_stow_pos[i];
      float v = 0.0f;
      const float d = target - p;
      const float ad = (d < 0.0f) ? -d : d;
      if (ad > STOW_DONE_EPS_REV) {
        all_done = false;
        if (ad <= step) { p = target; }
        else            { p += (d > 0.0f ? step : -step); v = (d > 0.0f ? s_stow_speed : -s_stow_speed); }
      } else {
        p = target;
      }
      s_stow_pos[i] = p;
      axes[i].target_pos_rev = p;
      axes[i].target_vel_rps = v;
      axes[i].target_torque_Nm = 0.0f;
      // Present-axis gate (Phase 11). NOTE: the deferred-stow descent is NOT
      // `!coldstart`-gated (adversarial-review fix): a cold-start move can never
      // co-occur with a stow — the stow-BEGIN is guarded against active moves
      // (fault_machine.cpp evaluate/watchdog), and a HOME/ACTIVATE/DEACTIVATE
      // request is rejected while fault_stow_pending() (the symmetric guard). Gating
      // the descent TX on `!coldstart` while still marking s_stow_complete below
      // would let the watchdog IDLE the legs on a virtual (never-transmitted) descent
      // → an unarmed gravity drop. The mutual exclusion belongs on the MPC ladder TX
      // (below), not the self-contained safety stow. The target cache above still
      // updates for all legs (telemetry) regardless.
      if (s_output_enabled && leg_present(i)) can_jugglebot_send(ODrive::encode_leg_setpoint(i, p, v, 0.0f));
    }
    s_stow_complete = all_done;
    return;
  }

  if (!s_have_latched) return;

  // Trajectory-phase dt — THE most control-critical interval. micros64() (item 14):
  // s_base_ts_us was stamped with micros64() at recv; a wall STEP here would jump the
  // 500 Hz phase → a commanded-position discontinuity/jerk into the legs. Also makes
  // this consistent with the micros64() the jitter accounting already uses above.
  const float dt = (float)((double)(micros64() - s_base_ts_us) * 1e-6);
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
    // Defense-in-depth NaN/Inf sanitization (Flash-A item 5): a non-finite command
    // fails BOTH comparisons below (NaN compares false to everything), so it would
    // pass the clamp untouched and reach the wire. The interp_on_setpoint isfinite
    // drop is the PRIMARY guard; this is the backstop should the extrapolation math
    // itself ever produce a non-finite value. Hold the current encoder position (no
    // lurch); if even that is non-finite, fall back to the retracted stroke min.
    if (!std::isfinite(cmd_pos[i])) {
      const float fb = axes[i].pos_rev;
      cmd_pos[i] = std::isfinite(fb) ? fb : STROKE_MIN_REV[i];
    }
    if (cmd_pos[i] < STROKE_MIN_REV[i]) cmd_pos[i] = STROKE_MIN_REV[i];
    else if (cmd_pos[i] > STROKE_MAX_REV[i]) cmd_pos[i] = STROKE_MAX_REV[i];
    if (cmd_pos[i] != pre) { cmd_vel[i] = 0.0f; cmd_tor[i] = 0.0f; }
  }

  // Publish targets into the cache (for telemetry) and transmit to CAN3.
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    axes[i].target_pos_rev   = cmd_pos[i];
    axes[i].target_vel_rps   = cmd_vel[i];
    axes[i].target_torque_Nm = cmd_tor[i];
  }
  // Cold-start interlock (Flash-A item 1a): suppress the whole MPC leg TX while a
  // firmware homing/activate/deactivate move is driving the same ODrives (see the
  // interlock note above). The target cache was written for all legs regardless.
  if (s_output_enabled && !coldstart) {
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      // Present-axis gate (Phase 11): never stream a 500 Hz setpoint to a node
      // that isn't on the bus. No-op on the full robot (all six present); on the
      // single-leg bench rig this drops the 5 phantom frames/tick to absent
      // nodes 1-5. The target cache above is written for all legs regardless.
      if (leg_present(i)) can_jugglebot_send(ODrive::encode_leg_setpoint(i, cmd_pos[i], cmd_vel[i], cmd_tor[i]));
    }
  }
}

// Reset every interpolator file-static to its power-on value. Two callers:
//   (a) the native test harness, to isolate interp statics between cases (the
//       fault-machine test binary links leg_interp.o and must reset it too);
//   (b) on-target re-arm, to return the interp to a known state before a fresh
//       cold-start (analogous to fault_machine_init()).
// Call only when the interp is quiescent (output disabled / not mid-tick) — it is
// NOT ISR-safe (it races the 500 Hz interp_isr's reads/writes of these statics).
void interp_reset() {
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    s_base_pos[i] = s_base_vel[i] = s_base_accel[i] = s_base_torque[i] = 0.0f;
    s_jerk[i] = s_next_pos[i] = s_next2_pos[i] = 0.0f;
    s_prev_accel[i] = 0.0f;
    s_stow_pos[i] = 0.0f;
  }
  s_has_next = s_has_next2 = false;
  s_base_ts_us = 0;
  s_have_latched = false;
  s_have_prev_accel = false;
  s_prev_recv_us = 0;
  s_stage = Staging{};
  s_pending = false;
  s_last_sp_seq = 0;
  s_have_sp_seq = false;
  s_last_setpoint_us = 0;
  s_output_enabled = false;
  s_stow_active = false;
  s_stow_complete = false;
  s_stow_speed = 0.0f;
  s_deadline_misses = 0;
  s_max_jitter_us = 0;
  s_last_tick_us = 0;
}

void leg_interp_init() {
  // Begin the 500 Hz tick. priority(): lower value = more urgent on the Cortex-M7
  // NVIC (this core uses the top 4 of 8 priority bits → 16 levels, steps of 16).
  //
  // FreeRTOS enforces its syscall ceiling by raising BASEPRI to
  // configMAX_SYSCALL_INTERRUPT_PRIORITY inside every critical section. On this
  // stack that ceiling is 32 (0x20): configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=2
  // in .pio/libdeps/teensy41/freertos-teensy/src/FreeRTOSConfig.h, shifted by
  // (8 - configPRIO_BITS)=(8-4)=4 → 2<<4 = 32. BASEPRI masks any interrupt whose
  // priority VALUE is >= BASEPRI, so the old priority(32) sat EXACTLY AT the ceiling
  // → it was MASKED by every RTOS critical section and inherited their jitter (the
  // opposite of the comment's claim). priority(16) (0x10) is one NVIC level BELOW the
  // 0x20 ceiling (16 < 32), so the interp ISR runs ABOVE the syscall ceiling and no
  // RTOS critical section can delay it.
  //
  // HARD INVARIANT (audited, item 17): because interp_isr runs above the syscall
  // ceiling, interp_isr AND EVERYTHING IT CALLS make ZERO FreeRTOS API calls — any
  // FreeRTOS call from above the ceiling is undefined behaviour. The full call tree
  // is FreeRTOS-free: micros64() (PRIMASK-guarded + Arduino micros()),
  // latch_from_staging() (pure float math + file-statics), can_jugglebot_send()
  // (PRIMASK-guarded FlexCAN_T4::write mailbox register write, NOT a mutex),
  // homing_active()/activate_active()/deactivate_active()/leg_present() (single-word
  // reads via non-inline cross-TU calls under -O2/no-LTO — the call is the barrier),
  // and the ODrive encoders (pure). Keep it that way.
  s_timer.begin(interp_isr, INTERP_PERIOD_US);
  s_timer.priority(16);
}

uint64_t interp_last_setpoint_us() { return atomic_read_u64(&s_last_setpoint_us); }
float interp_base_pos(uint8_t i) { return (i < NUM_LEGS) ? s_base_pos[i] : 0.0f; }
bool  interp_have_latched() { return s_have_latched; }
void interp_set_output_enabled(bool en) { s_output_enabled = en; }
bool interp_output_enabled() { return s_output_enabled; }
uint32_t interp_deadline_misses() { return s_deadline_misses; }
uint32_t interp_max_jitter_us() { return s_max_jitter_us; }
void interp_reset_jitter() { s_max_jitter_us = 0; }

void interp_begin_stow() {
  // PRIMASK-publish (mirror interp_on_setpoint's exemplar above). The 500 Hz
  // interp_isr runs ABOVE the FreeRTOS syscall ceiling (item 17 raised it to
  // priority(16)) and can preempt this fault-task write at ANY instruction. A plain
  // volatile store of s_stow_active does NOT order the preceding NON-volatile
  // s_stow_pos[]/s_stow_speed/s_stow_complete stores on ARMv7-M, so the ISR could
  // observe s_stow_active==true with a stale/half-written s_stow_pos[] and descend
  // from a garbage base → a jerk on the physical legs at a CAN-reconnect. Disabling
  // IRQs around the payload fill + the flag makes the publish a single atomic,
  // fully-ordered step (also the compiler/memory barrier the seqlock would provide).
  // Save/restore PRIMASK (not a bare __enable_irq) so it nests safely if ever called
  // from an already-masked region. The axes[i].pos_rev reads are fast single-word
  // volatile loads — cheap inside the IRQ-off window.
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  for (uint8_t i = 0; i < NUM_LEGS; ++i) s_stow_pos[i] = axes[i].pos_rev;  // start from actual
  s_stow_speed = 0.0f;          // ramp the descent speed up from rest
  s_stow_complete = false;
  s_stow_active = true;
  __set_PRIMASK(pm);
}
void interp_end_stow() { s_stow_active = false; }
bool interp_stow_active() { return s_stow_active; }
bool interp_stow_complete() { return s_stow_complete; }

}  // namespace CanBridge
