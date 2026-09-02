// =============================================================================
//  leg_interp.cpp — 500 Hz Hermite/Taylor interpolator (port of motor_guard)
// =============================================================================
//  Line-for-line transcription of the validated Python reference
//    tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py
//  (which the xref proves matches motor_guard to 0.0 rev). Keep the two in sync:
//  any change here must be mirrored there and re-validated by the xref.
//
//  DELIBERATE DIVERGENCE (2026-07-10 MAX_DEVIATION-runaway forensics): the LEAD
//  CLAMP below no longer mirrors teensy_interp.py / motor_guard. Those still ZERO
//  vel_ff on clamp (the bang-bang that caused the stutter+latch — see the lead-clamp
//  comment). This firmware instead keeps the true interpolated vel_ff (capped) and
//  uses the lower MAX_LEAD_REV. The Jetson-side interp (motor_guard) carries the same
//  latent defect and needs the same fix under a separate, operator-gated change that
//  re-validates the xref; until then the two intentionally differ ONLY in the lead
//  clamp. Every other mode (Hermite/Taylor/decay + stroke clamp) stays a 1:1 port.
//
//  DELIBERATE DIVERGENCE (2026-07-11 clear-errors jolt forensics): the RE-ENABLE
//  RECOVERY SLEW below has NO analog in teensy_interp.py / motor_guard, and cannot:
//  the reference interpolator has no output-enable gate, so there is no false→true
//  edge to re-baseline from. It fires ONLY on that edge (a guard clear / arm / stow
//  resumption) to bound the transient when the streamed command has diverged from the
//  drifted encoder during suppression; normal streaming runs the unmodified ladder.
//  This is a Teensy-only safety layer around the guard gate, not a math change — the
//  xref (which drives the ladder with output implicitly always on) is unaffected.
//
//  HAND LANE (FW 17, unified-7dof Phase 3): axis 6 is the 7th interpolated
//  channel — the SAME Hermite/Taylor/decay math, but a SEPARATE block with its
//  OWN latched state, its OWN knot clock and its OWN guard constants
//  (MAX_LEAD_HAND_REV / HAND_VELFF_LIMIT_RPS / MAX_DEVIATION_HAND_REV — the leg
//  constants must NEVER touch axis 6; keeping the lanes in separate loops makes
//  that separation structural rather than a per-line index audit, and keeps the
//  leg path byte-identical to FW 16 for T-R3). The hand lane is NOT in the
//  motor_guard/teensy_interp.py xref scope (motor_guard is 6-lane by contract);
//  its parity anchor is the native harness's identical-knots test — hand lane
//  output == leg lane output for identical inputs, which transfers the leg
//  xref's trust to the shared math.
// =============================================================================
#include "leg_interp.h"

#include <Arduino.h>          // IntervalTimer
#include <cstring>
#include <cmath>              // std::isfinite (setpoint trust-boundary drop)
#include "canbridge_config.h"
#include "udp_protocol.h"
#include "axis_state.h"
#include "odrive_protocol.h"
#include "can_buses.h"
#include "time_base.h"
#include "leg_homing.h"       // homing_active     (in-progress-move interlock)
#include "leg_activate.h"     // activate_active   (in-progress-move interlock)
#include "leg_deactivate.h"   // deactivate_active (in-progress-move interlock)
#include "hand_source.h"      // hand_source_streamed (the § 2.4 mastery latch)

namespace CanBridge {

// ── Ladder constants (mirror teensy_interp.py / motor_guard) ──────────────────
static constexpr float SEG_T   = SEGMENT_T_S;          // 0.025 (0.010 under BENCH_SYSID_BUILD)
static constexpr float MAXEXT  = MAX_EXTRAP_DT_S;       // 0.05
static constexpr float DECAY   = EXTRAP_DECAY_DT_S;     // 0.06
static constexpr float ALPHA   = JERK_EMA_ALPHA;        // 0.3
static constexpr float LEAD    = MAX_LEAD_REV;          // 0.10 (2026-07-10 forensics)
static constexpr float VELFF_CAP = LEAD_CLAMP_VELFF_LIMIT_RPS;   // 3.5 rev/s vel_ff bound
// torque_ff ingest backstop (2026-07-14 gravity-FF firmware sitting). WIRE-Nm
// (ODrive-Nm, post Kt-prescale). Generated from hardware_config.yaml
// dynamics.torque_ff_firmware_clamp_wire_nm — see the three-layer clamp-chain
// comment there (SetpointPump ±0.145 wire binds first; this backstop catches a
// Jetson-side bug/bypass; the ODrive 10 A current clamp is the last resort).
static constexpr float TORQUE_CLAMP = Dynamics::TORQUE_FF_FIRMWARE_CLAMP_WIRE_NM;  // 0.25

// ── Active latched base state (read by the ISR) ───────────────────────────────
// Widened NUM_LEGS → NUM_AXES at FW 17: index 6 is the hand lane. The leg loops
// below still iterate NUM_LEGS — the hand lane is processed in its own block
// with its own flags/clock (declared after the staging), so index 6 of these
// arrays is written only by a HAS_HAND latch and read only by the hand block.
static float s_base_pos[NUM_AXES];
static float s_base_vel[NUM_AXES];
static float s_base_accel[NUM_AXES];
static float s_base_torque[NUM_AXES];
static float s_jerk[NUM_AXES];
static float s_next_pos[NUM_AXES];
static float s_next2_pos[NUM_AXES];
static float s_v1[NUM_AXES];        // transmitted exact u1-knot velocity (HAS_V1, v6 wire)
static bool  s_has_next = false;
static bool  s_has_next2 = false;
static bool  s_has_v1 = false;      // leg-frame-level: the last accepted frame carried v1
static uint64_t s_base_ts_us = 0;
static bool  s_have_latched = false;

// Jerk history (legs — the hand lane keeps its own, below).
static float s_prev_accel[NUM_LEGS];
static bool  s_have_prev_accel = false;
static uint64_t s_prev_recv_us = 0;

// ── Hand-lane latched state (FW 17) ──────────────────────────────────────────
// The hand has its OWN knot clock and flags: a hand-less frame latches the legs
// and LEAVES these untouched, so the hand's dt keeps growing from its last real
// knot — that aging dt is what drives the normative falling-edge DECAY (see the
// hand block in interp_isr). Written only by latch_from_staging / interp_reset
// (ISR / quiescent), read only by the ISR.
static bool     s_hand_active = false;      // a HAS_HAND frame latched while STREAMED
static bool     s_hand_has_next = false;
static bool     s_hand_has_next2 = false;
static bool     s_hand_has_v1 = false;
static uint64_t s_hand_ts_us = 0;           // recv of the last HAND-BEARING frame
static float    s_hand_prev_accel = 0.0f;   // hand jerk-EMA history
static bool     s_hand_have_prev_accel = false;
static uint64_t s_hand_prev_recv_us = 0;

// ── Staging (written by net task, consumed by ISR when s_pending) ─────────────
struct Staging {
  float u0[NUM_AXES], u1[NUM_AXES], u2[NUM_AXES];
  float v0[NUM_AXES], accel[NUM_AXES], torque[NUM_AXES];
  float v1[NUM_AXES];
  bool  has_u1, has_u2, has_hand, has_v1;
  uint64_t recv_us;
};
static Staging s_stage;
static volatile bool s_pending = false;

// Last-ACCEPTED SETPOINT sequence — a wrap-safe monotonic drop of stale/duplicate
// frames. Touched ONLY in interp_on_setpoint (net-task context)
// and interp_reset (quiescent), never by the ISR, so no volatile/IRQ guard needed.
static uint16_t s_last_sp_seq = 0;
static bool     s_have_sp_seq = false;

static volatile uint64_t s_last_setpoint_us = 0;
static volatile bool     s_output_enabled = false;   // the fault machine owns this gate
// Deferred-stow descent state (set by the fault machine).
static volatile bool s_stow_active = false;
static volatile bool s_stow_complete = false;
static float s_stow_pos[NUM_LEGS];
static float s_stow_speed = 0.0f;        // accel-ramped descent speed (rev/s)
static volatile uint32_t s_deadline_misses = 0;
static volatile uint32_t s_max_jitter_us = 0;
// ── 500 Hz ladder occupancy (FW 11, uplinked on CLOCK_DIAG 0x8F) ─────────────
// The last two telemetry gaps named in the Addendum to
// logbook/2026-07-18-teensy-uptime-tracking-degradation.md: neither the recovery
// slew's duty nor the Mode-2 extrapolation duty left the chip, so "the response
// shape also slows" could never be tested against "the ladder is falling back to
// extrapolation more often".
//
// CUMULATIVE SINCE BOOT and never cleared by a reader — the emitter differences
// two consecutive reads to get a window (profiling.cpp's s_prev_* idiom, and the
// CanErrors / BridgeTxDiag "the consumer differences them" discipline). That is
// what makes them RACE-FREE, and the choice is deliberate: a read-then-zero at
// the emit site would leave a window in which an ISR increment lands between the
// read and the store and is silently lost. At 500 Hz that window is hit often
// enough to matter, and a lost tick in a duty-cycle numerator is indistinguishable
// from a real change in occupancy.
// Each is a single naturally-aligned word, so the ISR's read-modify-write is
// uninterruptible-by-a-second-writer (the ISR is the ONLY writer) and the
// task-side load is atomic on Cortex-M7 — no PRIMASK guard needed, unlike the
// u64 s_last_tick_us above. `volatile` so the ISR cannot cache them in a register
// across ticks and the task cannot hoist its load out of a loop.
// u32 wraps after ~99 days of continuous ticking; the emitter's unsigned
// subtraction is wrap-correct, so the wrap costs a consumer nothing.
static volatile uint32_t s_tick_count         = 0;
static volatile uint32_t s_recover_slew_ticks = 0;
static volatile uint32_t s_extrap_ticks       = 0;
// Monotonic (us) stamp of the most recent 500 Hz tick. Written ONLY by interp_isr
// (and zeroed by interp_reset() when the interp is quiescent). `volatile` since
// 2026-08-09 because interp_last_tick_us() now reads it from TASK context for the
// hand-dispatch phase stamp — the ISR must not keep it in a register across ticks.
// Task-side reads go through atomic_read_u64 (a u64 is two 32-bit accesses on this
// core, so an unmasked read can tear across the ISR's store).
static volatile uint64_t s_last_tick_us = 0;
// Per-leg lead-clamp-engaged flag from the most recent computed tick (bit i = leg i).
// Diagnostic only (surfaced on the 10 Hz HeartbeatT2J): a single naturally-aligned
// byte, written once per tick by the ISR (atomic store on Cortex-M7), read by the
// heartbeat task. 2026-07-10 forensics telemetry.
static volatile uint8_t s_lead_clamp_mask = 0;
// Per-leg torque_ff-ingest-clamp flag from the most recent ACCEPTED setpoint frame
// (bit i = leg i). Diagnostic only (surfaced on the 10 Hz HeartbeatT2J flags,
// bits 8-13): a single naturally-aligned byte, written once per accepted frame by
// the net task (atomic store on Cortex-M7), read by the heartbeat task. Set when
// |torque_ff[i]| exceeded TORQUE_CLAMP at ingest; cleared by the next accepted
// frame whose torque_ff[i] is in bounds. 2026-07-14 gravity-FF observability.
static volatile uint8_t s_torque_clamp_mask = 0;

// ── Hand-lane counters + deviation-guard state (FW 17) ────────────────────────
// CONCURRENCY, matching this file's existing discipline. The counters are
// written ONLY by their single writer (the ISR for tick counters; the net task
// for the LEGACY discard) and read from task context — each a naturally-aligned
// u32/float, atomic on Cortex-M7, cumulative-since-boot (the census idiom: the
// consumer differences two reads, nothing is ever read-then-cleared).
// s_hand_dev_guard_armed is written from TASK context (the hand7 console
// handler) and read by the fault task — one volatile byte, the
// s_output_enabled idiom. The dev snapshot trio (last/max/cmd/fb) is
// ISR-written, task-read; a torn multi-field read is harmless for a 1 Hz
// console line and the 10 Hz fault poll keys off the COUNTER, never the floats.
static volatile uint32_t s_hand_sent             = 0;   // hand set_input_pos frames transmitted
static volatile uint32_t s_hand_unseen_skips     = 0;   // lane active but axis 6 never seen → skip, never guess
static volatile uint32_t s_hand_stale_holds      = 0;   // ticks whose clamp-anchor age hit the staleness cap
static volatile uint32_t s_hand_discard_legacy   = 0;   // Setpoint index 6 discarded while hand_source == LEGACY
static volatile uint32_t s_hand_lead_clamp_ticks = 0;   // THE lead-duty counter (nonzero in a throw ⇒ abort sitting)
static volatile uint32_t s_hand_dev_over_ticks   = 0;   // ticks with |residual| > MAX_DEVIATION_HAND_REV
static volatile float    s_hand_dev_last         = 0.0f;
static volatile float    s_hand_dev_max          = 0.0f;   // residual (signed) at the worst |residual| tick
static volatile float    s_hand_dev_snap_cmd     = 0.0f;   // raw command at that tick
static volatile float    s_hand_dev_snap_fb      = 0.0f;   // age-extrapolated fb at that tick
// Trip-dedicated snapshot (adversarial-review fix, 2026-09-02): the residual
// trio at the most recent EXCEED tick (|dev| > MAX_DEVIATION_HAND_REV). The
// fault machine reports THESE at an armed latch, never the boot-cumulative
// s_hand_dev_max above — an observe-block excursion must not be frozen into a
// later armed trip's /link_status snapshot (misattribution). Same single-writer
// discipline as the counters: ISR-written, task-read; a torn multi-field read
// is bounded and the 10 Hz latch keys off the COUNTER, never these floats.
static volatile float    s_hand_dev_trip_dev     = 0.0f;
static volatile float    s_hand_dev_trip_cmd     = 0.0f;
static volatile float    s_hand_dev_trip_fb      = 0.0f;
static volatile bool     s_hand_dev_guard_armed  = false;  // OBSERVE-FIRST boot default; `hand7 arm` flips it

// ── Re-enable recovery slew (2026-07-11 clear-errors jolt forensics) ──────────
// State for the output-enable-edge slew (see the file header + canbridge_config.h
// RECOVER_SLEW_*). All four are written AND read ONLY by the 500 Hz ISR, so no
// volatile/IRQ guard is needed (unlike the stow statics, which the fault task fills):
// s_output_enabled itself is the volatile the fault task owns, and the ISR only reads
// it. s_recover_pos holds the per-leg slewed command; s_recover_speed is the shared
// accel-ramped slew speed (same trapezoidal idiom as the stow descent).
static bool  s_recover_slewing      = false;   // LEG-group slew (adversarial-review fix: per-axis-group done)
static bool  s_output_enabled_prev  = false;   // ISR-local edge tracker for s_output_enabled
static float s_recover_pos[NUM_AXES];          // [6] = the hand lane (FW 17); captured at every
                                               // edge/pin regardless of lane state so a mid-slew
                                               // hand activation never slews from a stale base
static float s_recover_speed         = 0.0f;
// Hand-lane slew state (2026-09-02 per-axis-group fix): the hand converges on
// its OWN flag and its OWN accel-ramped speed, so a 2.0 rev hand excursion no
// longer holds the six legs in the FF-less slew (vel_ff/torque_ff zeroed) for
// up to ~2 s — the legs hand back the moment the LEG set converges. The speed
// cap stays the shared RECOVER_SLEW_VEL_RPS (bounded and safe for the hand's
// 2.0 rev band; a hand-specific speed constant is a later tunable, not needed
// for correctness). ISR-only state, like the leg slew statics above.
static bool  s_hand_recover_slewing  = false;
static float s_hand_recover_speed    = 0.0f;

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

  // ── Wrap-safe monotonic-seq guard ──────────────────────────────────────────
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

  const bool has_v1_flag   = (sp.flags & 0x8u) != 0;   // HAS_V1 (v6 wire, bit 3)
  bool       has_hand_flag = (sp.flags & 0x4u) != 0;   // HAS_HAND (v6 wire, bit 2)

  // ── hand_source interlock (§ 2.4): discard index 6 while LEGACY ─────────────
  // While the Platform Teensy stroke engine masters the hand, a v6 frame's hand
  // channel is DISCARDED — the frame itself (the legs) is still accepted, so a
  // producer that wrongly ships hand knots can never starve the leg stream into
  // MPC_STALE. Counted on a visible counter ([hand7] console): a climbing value
  // means the host is emitting hand knots the firmware refuses by mode.
  if (has_hand_flag && !hand_source_streamed()) {
    has_hand_flag = false;
    ++s_hand_discard_legacy;
  }

  // ── isfinite trust-boundary drop ────────────────────────────────────────────
  // A NaN/Inf field survives BOTH the lead clamp and the stroke clamp (a NaN fails
  // every comparison, so no clamp branch fires) and would reach encode_leg_setpoint
  // → the CAN wire. DROP the whole frame like a lost UDP frame — do NOT sanitize to
  // zero (a zeroed setpoint could trip MAX_DEVIATION or lurch a leg). CRITICAL: a
  // dropped frame does NOT advance s_last_setpoint_us below, so a stream of all-NaN
  // frames still eventually trips MPC_STALE (the staleness watchdog fires as if the
  // link went quiet), and does NOT advance s_last_sp_seq (only an accepted frame does).
  // Scope: legs always; v1 lanes only when HAS_V1; the hand lane's fields only
  // when HAS_HAND survives the interlock above — with it clear the firmware
  // IGNORES index 6 by contract, so garbage there must not kill the leg stream.
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    if (!(std::isfinite(sp.u0[i]) && std::isfinite(sp.u1[i]) && std::isfinite(sp.u2[i]) &&
          std::isfinite(sp.v0[i]) && std::isfinite(sp.accel[i]) && std::isfinite(sp.torque_ff[i])))
      return;
    if (has_v1_flag && !std::isfinite(sp.v1[i])) return;
  }
  if (has_hand_flag) {
    const uint8_t h = HAND_AXIS;
    if (!(std::isfinite(sp.u0[h]) && std::isfinite(sp.u1[h]) && std::isfinite(sp.u2[h]) &&
          std::isfinite(sp.v0[h]) && std::isfinite(sp.accel[h]) && std::isfinite(sp.torque_ff[h])))
      return;
    if (has_v1_flag && !std::isfinite(sp.v1[h])) return;
  }

  // ── torque_ff ingest clamp (2026-07-14 gravity-FF firmware backstop) ────────
  // Bound |torque_ff[i]| to TORQUE_CLAMP (wire-Nm) BEFORE staging. CLAMP, DON'T
  // REJECT: an oversized torque with valid pos/vel is a torque-path bug — dropping
  // the whole frame would starve the interp into the MPC_STALE E-STOP, converting
  // a torque bug into a position-control outage mid-motion. (A NaN torque still
  // drops the whole frame above — NaN means the frame is garbage.) Without this,
  // the only firmware bound is int16 saturation at ±3.2767 wire-Nm ≈ 59 A of
  // demand; the ODrive ADDS input_torque to the velocity loop's output BEFORE its
  // torque limit, so ≥ ~0.55 wire-Nm consumes the whole 10 A budget and the
  // position loop loses all authority. The engagement mask is published per leg
  // for the 10 Hz heartbeat (mirrors s_lead_clamp_mask).
  uint8_t tq_mask = 0;
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    if (sp.torque_ff[i] > TORQUE_CLAMP) {
      sp.torque_ff[i] = TORQUE_CLAMP;
      tq_mask |= (uint8_t)(1u << i);
    } else if (sp.torque_ff[i] < -TORQUE_CLAMP) {
      sp.torque_ff[i] = -TORQUE_CLAMP;
      tq_mask |= (uint8_t)(1u << i);
    }
  }

  const uint64_t recv = micros64();   // monotonic: feeds s_base_ts_us / s_last_setpoint_us / jerk dt,
                                      // all read against micros64() — a wall step must not corrupt the trajectory phase

  // Publish the staging atomically. The interp ISR runs ABOVE the FreeRTOS
  // syscall ceiling and can preempt this net-task write at any instruction; if
  // a second setpoint were written field-by-field while s_pending is still set
  // from a first, the ISR could latch a half-written mix. Disabling IRQs around
  // the fill + flag (PRIMASK save/restore) makes the publish a single atomic,
  // fully-ordered step (also serves as the compiler/memory barrier the seqlock
  // would otherwise provide). ~65 words copied (7 lanes + v1 since v6) → a few
  // µs of IRQ-off, same order as before the widening.
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    s_stage.u0[i]     = sp.u0[i];
    s_stage.u1[i]     = sp.u1[i];
    s_stage.u2[i]     = sp.u2[i];
    s_stage.v0[i]     = sp.v0[i];
    s_stage.accel[i]  = sp.accel[i];
    s_stage.torque[i] = sp.torque_ff[i];
    s_stage.v1[i]     = sp.v1[i];
  }
  s_stage.has_u1   = (sp.flags & 0x1u) != 0;
  s_stage.has_u2   = (sp.flags & 0x2u) != 0;
  s_stage.has_hand = has_hand_flag;   // post-interlock: LEGACY already discarded above
  s_stage.has_v1   = has_v1_flag;
  s_stage.recv_us = recv;
  s_pending = true;
  __set_PRIMASK(pm);

  // Only an ACCEPTED frame advances the seq high-water mark + the staleness clock
  // (and, likewise, publishes the torque-clamp mask — a dropped frame leaves it).
  s_torque_clamp_mask = tq_mask;   // single-store publish for telemetry
  s_last_sp_seq = seq;
  s_have_sp_seq = true;
  atomic_write_u64(&s_last_setpoint_us, recv);   // 64-bit; read by the fault task
}

// ── Latch (consume staging) — port of teensy_interp.latch_setpoint ────────────
static void latch_from_staging() {
  // Jerk EMA from consecutive accelerations (legs).
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
    s_v1[i]          = s_stage.v1[i];
    s_prev_accel[i]  = s_stage.accel[i];
  }
  s_has_next  = s_stage.has_u1;
  s_has_next2 = s_stage.has_u2 && s_stage.has_u1;
  s_has_v1    = s_stage.has_v1 && s_stage.has_u1;   // v1 is the u1-knot velocity — meaningless without u1
  s_base_ts_us = s_stage.recv_us;
  s_prev_recv_us = s_stage.recv_us;
  s_have_prev_accel = true;
  s_have_latched = true;

  // ── Hand lane (FW 17): latch ONLY on a hand-bearing frame ──────────────────
  // A hand-less frame leaves every [HAND_AXIS] slot and the hand knot clock
  // untouched, so the hand lane keeps aging from its last real knot. That aging
  // is load-bearing: it is what drives the hand block's Mode-1 → extrapolate →
  // DECAY continuation on a HAS_HAND falling edge (the normative rule), instead
  // of a fresh leg frame silently re-zeroing the hand's trajectory phase.
  if (s_stage.has_hand) {
    const uint8_t h = HAND_AXIS;
    if (s_hand_have_prev_accel) {
      const float dt_h = (float)((double)(s_stage.recv_us - s_hand_prev_recv_us) * 1e-6);
      if (dt_h > 1e-6f) {
        const float raw = (s_stage.accel[h] - s_hand_prev_accel) / dt_h;
        s_jerk[h] = ALPHA * raw + (1.0f - ALPHA) * s_jerk[h];
      }
    } else {
      s_jerk[h] = 0.0f;
    }
    s_base_pos[h]    = s_stage.u0[h];
    s_base_vel[h]    = s_stage.v0[h];
    s_base_accel[h]  = s_stage.accel[h];
    s_base_torque[h] = s_stage.torque[h];   // latched for completeness; the hand TX hard-zeros torque (see the hand block)
    s_next_pos[h]    = s_stage.u1[h];
    s_next2_pos[h]   = s_stage.u2[h];
    s_v1[h]          = s_stage.v1[h];
    s_hand_has_next  = s_stage.has_u1;
    s_hand_has_next2 = s_stage.has_u2 && s_stage.has_u1;
    s_hand_has_v1    = s_stage.has_v1 && s_stage.has_u1;
    s_hand_prev_accel = s_stage.accel[h];
    s_hand_have_prev_accel = true;
    s_hand_prev_recv_us = s_stage.recv_us;
    s_hand_ts_us     = s_stage.recv_us;
    s_hand_active    = true;
  }
}

// ── 500 Hz ISR — port of teensy_interp.tick + CAN TX ──────────────────────────
static void interp_isr() {
  const uint64_t now_mono = micros64();
  // Occupancy denominator, counted at the very TOP so it covers every entry
  // including the ones that return early below (deferred stow, or before the
  // first setpoint is latched). That makes it a true tick census — it should
  // equal 500 x window_seconds, and a shortfall is ISR starvation, which is a
  // second and independent read on the health interp_deadline_misses reports.
  ++s_tick_count;
  // Jitter / deadline-miss accounting.
  if (s_last_tick_us != 0) {
    const uint32_t dt_tick = (uint32_t)(now_mono - s_last_tick_us);
    const uint32_t jit = (dt_tick > INTERP_PERIOD_US)
                         ? (dt_tick - INTERP_PERIOD_US) : (INTERP_PERIOD_US - dt_tick);
    if (jit > s_max_jitter_us) s_max_jitter_us = jit;
    if (dt_tick > INTERP_PERIOD_US + JITTER_MISS_US) s_deadline_misses++;
  }
  s_last_tick_us = now_mono;

  // ── Hand-lane latch clear on the arm edge (adversarial-review fix, 2026-09-02) ─
  // interp_reset() has no runtime caller, so without this the hand-lane latch
  // (s_hand_active + its knot state) would persist across armed SESSIONS: a
  // later hand-less armed session whose source latch still reads STREAMED would
  // find the ancient latch alive, replay its decayed hold as a live 7th frame
  // (and drag the powered hand up to MAX_LEAD_HAND_REV toward a target no
  // producer commands via the recovery slew), or spuriously feed the deviation
  // guard. A fresh HAS_HAND latch is REQUIRED per armed session: clear the lane
  // on the s_output_enabled false→true edge, BEFORE the staging consume below —
  // so a hand-bearing frame already staged by the live stream-then-arm stream
  // re-latches the lane in this same tick (nothing is lost on the happy path).
  // ISR-context-safe: plain stores to ISR-owned statics; `out_en` is read ONCE
  // here and reused by the slew edge below (tasks cannot preempt this ISR, so
  // one read serves the whole tick). s_output_enabled_prev (the slew's edge
  // tracker) is deliberately NOT updated here — during a deferred-stow or
  // pre-first-latch early return the clear simply re-fires idempotently.
  const bool out_en = s_output_enabled;   // volatile read (fault-task owned)
  if (out_en && !s_output_enabled_prev) {
    s_hand_active = false;
    s_hand_has_next = s_hand_has_next2 = s_hand_has_v1 = false;
    s_hand_have_prev_accel = false;   // jerk history dies with the session
                                      // (a fresh latch then zeroes s_jerk[6])
  }

  if (s_pending) { latch_from_staging(); s_pending = false; }

  // ── In-progress-move interlock ──────────────────────────────────────────────
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
      // Present-axis gate. NOTE: the deferred-stow descent is NOT
      // `!coldstart`-gated (adversarial-review fix): a cold-start move can never
      // co-occur with a stow — the stow-BEGIN is guarded against active moves
      // (fault_machine.cpp evaluate/watchdog), and a HOME/ACTIVATE/DEACTIVATE
      // request is rejected while fault_stow_pending() (the symmetric guard). Gating
      // the descent TX on `!coldstart` while still marking s_stow_complete below
      // would let the watchdog IDLE the legs on a virtual (never-transmitted) descent
      // → an unarmed gravity drop. The mutual exclusion belongs on the MPC ladder TX
      // (below), not the self-contained safety stow. The target cache above still
      // updates for all legs (telemetry) regardless.
      // TxCls::LEGS (2026-08-24): a DEFERRED setpoint is SENT. The result is
      // deliberately discarded and NEVER retried — these are latest-wins
      // setpoints at 500 Hz, so re-sending a frame the queue already holds would
      // put a STALE setpoint on the wire behind a fresher one. The census counts
      // the deferral; the fault machine is not involved.
      if (s_output_enabled && leg_present(i)) can_jugglebot_tx(ODrive::encode_leg_setpoint(i, p, v, 0.0f), TxCls::LEGS);
    }
    s_stow_complete = all_done;
    return;
  }

  if (!s_have_latched) return;

  // Trajectory-phase dt — THE most control-critical interval. micros64():
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
      // u1-knot endpoint velocity: the TRANSMITTED v1 when the frame carried it
      // (HAS_V1, v6 wire — exact reconstruction for knot-aligned cubics, Phase 0
      // decision 2); else the flown (u2−u1)/SEG_T forward difference, with
      // (u1−u0)/SEG_T when no u2 lookahead exists either.
      const float v1 = s_has_v1   ? s_v1[i]
                     : s_has_next2 ? (s_next2_pos[i] - p1) / SEG_T
                                   : (p1 - p0) / SEG_T;
      cmd_pos[i] = h00 * p0 + h10 * (SEG_T * v0) + h01 * p1 + h11 * (SEG_T * v1);
      cmd_vel[i] = dh00 * p0 + dh10 * v0 + dh01 * p1 + dh11 * v1;
    }
  } else if (dt <= MAXEXT) {
    // Mode 2: cubic Taylor extrapolation.
    ++s_extrap_ticks;   // occupancy: this tick ran open-loop off the last knot
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
  // 2026-07-10 MAX_DEVIATION-runaway forensics — two changes break the stutter:
  //   * LEAD is now 0.10 (was 0.15) so pos_gain(40)*LEAD = 4.0 no longer exceeds the
  //     leg vel_limit — see canbridge_config.h MAX_LEAD_REV. (2026-07-16: vel_limit
  //     raised 4.0 → 6.0 → 12.0, so 4.0 sits well BELOW it and the clamp-engaged
  //     setpoint is bounded by 4.0 + VELFF_CAP instead; through 2026-07-15 it sat
  //     exactly AT the limit.)
  //   * Do NOT zero vel_ff when the clamp engages. Zeroing manufactured a
  //     discontinuous feedforward (and, with the old 0.15, a vel_limit-saturating
  //     P-term sprint) that seeded a ~6 Hz limit cycle. Instead pass the TRUE
  //     interpolated vel_ff through, bounded to ±VELFF_CAP (3.5 rev/s, kept below the
  //     drive vel_limit) so a runaway command cannot inject an over-limit feedforward.
  // (This deliberately diverges from teensy_interp.py/motor_guard — see the file
  //  header. The stroke clamp below still zeros vel_ff, correctly: a physical backstop
  //  hit means "hold at the limit", where feedforward is meaningless.)
  uint8_t clamp_mask = 0;
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    const float fb = axes[i].pos_rev;          // single-word atomic read
    float dev = cmd_pos[i] - fb;
    if (dev > LEAD)       { dev = LEAD;  clamp_mask |= (uint8_t)(1u << i); }
    else if (dev < -LEAD) { dev = -LEAD; clamp_mask |= (uint8_t)(1u << i); }
    cmd_pos[i] = fb + dev;
    // Bound the feedforward magnitude (applies whether or not the clamp engaged).
    if (cmd_vel[i] > VELFF_CAP)       cmd_vel[i] = VELFF_CAP;
    else if (cmd_vel[i] < -VELFF_CAP) cmd_vel[i] = -VELFF_CAP;
  }
  // (clamp_mask is published below, after the hand lane has had the chance to
  //  set bit 6 — one single-store publish per tick, as before.)
  // Stroke clamp (physical backstop).
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    const float pre = cmd_pos[i];
    // Defense-in-depth NaN/Inf sanitization: a non-finite command
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

  // ═══ Hand lane (axis 6) — unified-7dof FW 17 ═══════════════════════════════
  // A SEPARATE block, not a widened leg loop: the hand carries its own guard
  // constants (MAX_LEAD_HAND_REV 2.0 / HAND_VELFF_LIMIT_RPS 300 / clip
  // [0, HAND_MOTOR_MAX_POSITION]) and its own knot clock, and the leg constants
  // (MAX_LEAD_REV 0.10, LEAD_CLAMP_VELFF_LIMIT_RPS 3.5, STROKE_*_REV) must
  // NEVER touch axis 6 — separate loops make that structural. Active only while
  // a HAS_HAND frame has latched under hand_source == STREAMED.
  bool  hand_tx = false;              // computed + clamped this tick → publish/TX
  float h_pos = 0.0f, h_vel = 0.0f;
  // Gated on the LIVE latch as well as the latched lane state: a post-disarm
  // LEGACY switch must make a still-latched lane inert THAT tick — otherwise
  // the old knot state would keep emitting its decayed hold against the stroke
  // engine's frames at the next legacy dispatch (exactly the dual-mastery
  // window the latch exists to close). The switch gate (!mpc_active + settled)
  // means this can only flip while output is suppressed, so there is no
  // mid-stream transient to bound.
  if (s_hand_active && hand_source_streamed()) {
    const uint8_t h = HAND_AXIS;
    // The hand's own trajectory phase — dt from the last HAND-BEARING knot, not
    // from the last leg frame. This is what implements the NORMATIVE falling-
    // edge rule: when HAS_HAND falls while leg frames continue (host gap, a
    // backstop freeze to a hand-less HoldPlan, plan expiry), this dt keeps
    // growing, so the lane runs its Hermite segment to the u1 endpoint, then
    // Taylor-extrapolates ≤ MAX_EXTRAP_DT_S from the (u1, v1) endpoint state,
    // then DECAYS the velocity to zero over EXTRAP_DECAY_DT_S — exactly a leg's
    // Mode-2/3 wind-down. It must NEVER hold Mode 1's s = 1 endpoint forever:
    // that would keep commanding the endpoint with vel_ff = v1 — hold-at-last-
    // command from up to 200 rev/s, the precise hazard the rule forbids.
    const float hdt = (float)((double)(now_mono - s_hand_ts_us) * 1e-6);
    const float p0 = s_base_pos[h], v0 = s_base_vel[h];
    const float a0 = s_base_accel[h], jk = s_jerk[h];
    float endp = 0.0f, endv = 0.0f;   // state the post-segment extrapolation continues from
    float over;                       // time past the covered segment (< 0 ⇒ still inside Mode 1)
    if (s_hand_has_next) {
      const float p1 = s_next_pos[h];
      const float v1 = s_hand_has_v1   ? s_v1[h]
                     : s_hand_has_next2 ? (s_next2_pos[h] - p1) / SEG_T
                                        : (p1 - p0) / SEG_T;
      if (hdt <= SEG_T) {
        // Mode 1 — the same cubic Hermite as the legs (same basis, same v1 rule).
        const float s = hdt / SEG_T;
        const float s2 = s * s, s3 = s2 * s;
        const float invT = 1.0f / SEG_T;
        h_pos = (2.0f * s3 - 3.0f * s2 + 1.0f) * p0
              + (s3 - 2.0f * s2 + s) * (SEG_T * v0)
              + (-2.0f * s3 + 3.0f * s2) * p1
              + (s3 - s2) * (SEG_T * v1);
        h_vel = ((6.0f * s2 - 6.0f * s) * invT) * p0
              + (3.0f * s2 - 4.0f * s + 1.0f) * v0
              + ((-6.0f * s2 + 6.0f * s) * invT) * p1
              + (3.0f * s2 - 2.0f * s) * v1;
        over = -1.0f;
      } else {
        endp = p1; endv = v1; over = hdt - SEG_T;   // segment complete — continue from its endpoint
      }
    } else {
      endp = p0; endv = v0; over = hdt;             // no u1 knot — a leg's Mode-2 base
    }
    if (over >= 0.0f) {   // past the covered segment (or no u1 knot at all)
      if (over <= MAXEXT) {
        // Mode 2 — cubic Taylor from the endpoint state (continuous with Mode 1
        // at over = 0: pos = endp, vel = endv by construction).
        const float o2 = over * over;
        h_pos = endp + endv * over + 0.5f * a0 * o2 + (1.0f / 6.0f) * jk * (o2 * over);
        h_vel = endv + a0 * over + 0.5f * jk * o2;
      } else {
        // Mode 3 — velocity decay to zero (the leg formula, endpoint-based).
        const float dt_b2 = MAXEXT * MAXEXT;
        const float dt_over = over - MAXEXT;
        float decay = 1.0f - dt_over / DECAY;
        if (decay < 0.0f) decay = 0.0f;
        const float vel_b = endv + a0 * MAXEXT + 0.5f * jk * dt_b2;
        const float pos_b = endp + endv * MAXEXT + 0.5f * a0 * dt_b2
                            + (1.0f / 6.0f) * jk * (dt_b2 * MAXEXT);
        const float extra = (dt_over >= DECAY)
            ? vel_b * (DECAY * 0.5f)
            : vel_b * dt_over * (1.0f - dt_over / (2.0f * DECAY));
        h_pos = pos_b + extra;
        h_vel = vel_b * decay;
      }
    }

    // ── Feedback anchor: freshness-aware, never a guess ──────────────────────
    // Individual atomic loads, NOT snapshot_pos_vel: the seqlock's retry loop
    // spins while the writer (task_can_rx) is mid-update, and this ISR PREEMPTS
    // that writer — a retry here could spin forever. A torn pos/vel pair across
    // a 100 Hz update is a bounded, continuous-value error in a clamp ANCHOR,
    // which the clamp band dwarfs. (Same discipline as the leg lead clamp's
    // single-word fb read.)
    const uint64_t hts = atomic_read_u64(&hand_axis().pos_timestamp_us);
    if (hts == 0) {
      // Axis 6 has NEVER reported an encoder position: 0.0 rev is a real,
      // reachable, WRONG position, so commanding anything would be a guess.
      // Skip (no target publish, no TX) and say so — the bench7 unseen-skip
      // lesson, kept from the retired probe because it is the single most
      // likely way a sitting measures 6 frames while claiming 7.
      ++s_hand_unseen_skips;
    } else {
      // Age-extrapolated encoder (fb + vel·age) — the freshness-aware anchor
      // Phase 0 Decision 4 requires: a freshness-blind clamp either needs a
      // 5.0 rev band (46 % of stroke — not a guard) or re-creates the S1/S2
      // stale-anchor commanded-stop at 13× leg speed. Age is CAPPED at
      // MOTOR_FB_STALENESS_US and the cap is COUNTED (stale_holds): past the
      // cap the anchor stops following a possibly-dead extrapolation, the
      // frame still transmits, and the counter makes the telemetry gap loud.
      uint64_t age_us = (now_mono > hts) ? (now_mono - hts) : 0u;
      if (age_us > MOTOR_FB_STALENESS_US) { age_us = MOTOR_FB_STALENESS_US; ++s_hand_stale_holds; }
      const float fb    = hand_axis().pos_rev;   // single-word atomic reads
      const float fbv   = hand_axis().vel_rps;
      const float fb_ex = fb + fbv * (float)((double)age_us * 1e-6);

      // NaN/Inf backstop (mirror of the leg stroke-clamp backstop): a non-
      // finite command fails every clamp comparison and would reach the wire.
      if (!std::isfinite(h_pos)) {
        h_pos = std::isfinite(fb) ? fb : 0.0f;   // hold; 0.0 = retract as last resort
        h_vel = 0.0f;
      }

      // ── MAX_DEVIATION_HAND_REV residual — the 500 Hz tick's verdict ────────
      // Velocity-compensated BOTH sides: the RAW interpolated command (pre-
      // clamp) against the age-extrapolated encoder. The fault task differences
      // the exceed-tick counter at 10 Hz and (only when `hand7 arm`ed — the
      // guard ships OBSERVE-FIRST) latches the E-STOP off it; the max-residual
      // snapshot is the observe sitting's read.
      const float dev = h_pos - fb_ex;
      s_hand_dev_last = dev;
      const float adev = (dev < 0.0f) ? -dev : dev;
      const float amax = (s_hand_dev_max < 0.0f) ? -s_hand_dev_max : s_hand_dev_max;
      if (adev > amax) {
        s_hand_dev_max      = dev;
        s_hand_dev_snap_cmd = h_pos;
        s_hand_dev_snap_fb  = fb_ex;
      }
      if (adev > MAX_DEVIATION_HAND_REV) {
        ++s_hand_dev_over_ticks;
        // The trip's OWN excursion (most recent exceed tick) — what the fault
        // machine freezes into the MAX_DEVIATION latch snapshot. Multiple
        // exceed ticks inside one 10 Hz poll window are the same excursion;
        // last-writer-wins is the honest per-trip read (2026-09-02 fix).
        s_hand_dev_trip_dev = dev;
        s_hand_dev_trip_cmd = h_pos;
        s_hand_dev_trip_fb  = fb_ex;
      }

      // ── Hand lead clamp (MAX_LEAD_HAND_REV, against fb_ex) + lead-duty ─────
      float d = dev;
      if (d > MAX_LEAD_HAND_REV)       { d = MAX_LEAD_HAND_REV;  clamp_mask |= 0x40u; ++s_hand_lead_clamp_ticks; }
      else if (d < -MAX_LEAD_HAND_REV) { d = -MAX_LEAD_HAND_REV; clamp_mask |= 0x40u; ++s_hand_lead_clamp_ticks; }
      h_pos = fb_ex + d;
      // vel_ff bound: HAND_VELFF_LIMIT_RPS (300) — NEVER the legs' 3.5 (a 51× cut).
      if (h_vel > HAND_VELFF_LIMIT_RPS)       h_vel = HAND_VELFF_LIMIT_RPS;
      else if (h_vel < -HAND_VELFF_LIMIT_RPS) h_vel = -HAND_VELFF_LIMIT_RPS;
      // ── Stroke clip [0, HAND_MOTOR_MAX_POSITION] — the metal ───────────────
      // encode_leg_setpoint's clip_position() is the backstop; clipping HERE
      // too is what zeroes the feedforward at the stop (a backstop hit means
      // "hold at the limit", where feedforward is meaningless — the leg rule).
      const float pre = h_pos;
      if (h_pos < 0.0f)                       h_pos = 0.0f;
      else if (h_pos > HAND_MOTOR_MAX_POSITION) h_pos = HAND_MOTOR_MAX_POSITION;
      if (h_pos != pre) h_vel = 0.0f;
      hand_tx = true;
    }
  }
  s_lead_clamp_mask = clamp_mask;   // single-store publish (bits 0-5 legs, bit 6 hand)

  // ── Re-enable recovery slew (2026-07-11 clear-errors jolt forensics) ─────────
  // Detect the s_output_enabled false→true edge (a guard clear / arm / fb-stale or
  // stow resumption). On it, re-baseline the transmitted command to the LIVE ENCODER
  // and, while the slew is active, override cmd with a bounded velocity+accel ramp
  // toward the streamed (already lead+stroke-clamped) command in cmd_pos[]. Root
  // cause: during suppression the ODrive holds enc_freeze+lead while the leg drifts
  // onto the encoder, so at re-enable the streamed command can sit a full lead clamp
  // (0.10 rev) below the encoder — commanding it directly is a pos_gain × lead ≈
  // 4 rev/s kick to the current rail. The slew starts AT the encoder (dev 0, zero
  // P-kick) with vel_ff 0, so the first re-enabled frame is a no-op step, then ramps.
  // Because the streamed target is itself within ±lead of the encoder, the slewed
  // command stays inside the lead-clamp band the whole time (never runs the clamp).
  // (out_en was read once at the top of the ISR — same value, tasks cannot
  //  preempt an ISR mid-tick; the hand-lane latch clear above used it too.)
  if (out_en && !s_output_enabled_prev) {
    for (uint8_t i = 0; i < NUM_AXES; ++i) s_recover_pos[i] = axes[i].pos_rev;  // start at the live encoder (legs + hand)
    s_recover_speed = 0.0f;               // ramp the slew speed up from rest (no vel step at the edge)
    s_recover_slewing = true;
    // Per-axis-group slew (adversarial-review fix, 2026-09-02): the hand
    // converges on its OWN flag + clock, so the legs are not held FF-less for
    // the up-to-2 s a 2.0 rev hand excursion takes at the shared slew speed.
    s_hand_recover_speed = 0.0f;
    s_hand_recover_slewing = true;
  }
  s_output_enabled_prev = out_en;

  if (coldstart) {
    // Cold-start gate (2026-07-11 F2 fix). A firmware home/activate/deactivate move
    // drives the legs FASTER than RECOVER_SLEW_VEL_RPS (1.0 rev/s) while the MPC leg TX
    // is suppressed below. If the slew kept running here, s_recover_pos would slew
    // toward the lead-clamped command at only 1 rev/s while the ENCODER raced past it —
    // lagging by more than MAX_LEAD — and at move-end the un-re-clamped emit would kick
    // the leg. Instead HOLD s_recover_pos pinned to the LIVE ENCODER every tick
    // (mirroring the stow's mandatory output-off / clean-edge idiom) and skip the slew,
    // so when the move ends the slew resumes from a zero-deviation baseline on the
    // encoder. s_recover_slewing is intentionally left as-is: if it was armed, it
    // resumes (a benign one-tick no-op) when coldstart falls; if not, it stays off.
    for (uint8_t i = 0; i < NUM_AXES; ++i) s_recover_pos[i] = axes[i].pos_rev;
    s_recover_speed = 0.0f;
    s_hand_recover_speed = 0.0f;
  } else {
   if (s_recover_slewing) {
    // Occupancy counted HERE, at the override, not at the flag. During a
    // cold-start move (the branch above) the flag can stay armed while the ramp
    // is pinned to the live encoder and skipped — those ticks emit the
    // cold-start move's commands, not the ramp's, so counting them would
    // overstate the answer to the only question this counter is asked: how often
    // did the transmitted command come from the recovery ramp instead of the
    // trajectory. (Leg-group semantics kept after the 2026-09-02 per-axis-group
    // split: the counter answers for the LEG stream, its historical consumer.)
    ++s_recover_slew_ticks;
    const float dt_tick = INTERP_PERIOD_US * 1e-6f;
    s_recover_speed += RECOVER_SLEW_ACCEL_RPS2 * dt_tick;
    if (s_recover_speed > RECOVER_SLEW_VEL_RPS) s_recover_speed = RECOVER_SLEW_VEL_RPS;
    const float step = s_recover_speed * dt_tick;
    bool all_done = true;
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      const float tgt = cmd_pos[i];       // the normal streamed command for this tick
      float p = s_recover_pos[i];
      const float d = tgt - p;
      const float ad = (d < 0.0f) ? -d : d;
      float v = 0.0f;
      if (ad > RECOVER_SLEW_DONE_EPS_REV) {
        // Only present legs gate hand-back — an absent leg never streams, so its slew
        // state must not hold the whole robot in recovery (mirror the stow present-gate).
        if (leg_present(i)) all_done = false;
        if (ad <= step) { p = tgt; }      // final approach — snap onto the command, no overshoot
        else            { p += (d > 0.0f ? step : -step); v = (d > 0.0f ? s_recover_speed : -s_recover_speed); }
      } else {
        p = tgt;                          // already converged (the /recover happy path is a one-tick no-op)
      }
      // Re-run the lead clamp on the slewed command (2026-07-11 F2 fix). Normally p
      // stays within ±LEAD of the encoder (it starts on the encoder and tgt is itself
      // lead-clamped), so this is a no-op on the happy path. But it GUARANTEES the
      // EMITTED command can never exceed encoder±MAX_LEAD even if the slew state ever
      // lagged a fast-moving encoder — a defense-in-depth backstop mirroring the primary
      // lead clamp above. The clamped value is stored back into s_recover_pos so the
      // slew state itself can never run away either.
      const float fb = axes[i].pos_rev;   // single-word atomic read
      float dev = p - fb;
      if (dev > LEAD)       dev = LEAD;
      else if (dev < -LEAD) dev = -LEAD;
      p = fb + dev;
      s_recover_pos[i] = p;
      cmd_pos[i] = p;                     // transmit the slewed command in place of the streamed one
      cmd_vel[i] = v;                     // vel_ff = the slew velocity (0 at the edge, ≤ RECOVER_SLEW_VEL_RPS)
      cmd_tor[i] = 0.0f;                  // no torque feedforward during the recovery ramp
    }
    if (all_done) s_recover_slewing = false;   // LEG set converged → legs resume normal
                                               // streaming (full vel/torque FF) even while
                                               // the hand is still slewing (2026-09-02 fix)
   }
    // ── Hand lane slews on its OWN flag + clock (FW 17; per-axis-group 2026-09-02) ─
    // Same bounded ramp shape, but the re-clamp band is the hand's OWN
    // MAX_LEAD_HAND_REV — the legs' 0.10 rev here would pin a re-enabled hand
    // 2 s of slew away from a command it is allowed to be 2.0 rev from — and
    // the done-flag is the hand's own: pre-split, the hand shared the legs'
    // `all_done`, so every output-enable edge held ALL SIX LEGS at vel_ff =
    // torque_ff = 0 for the up-to-~2 s a 2.0 rev hand excursion takes at
    // 1 rev/s (was ~0.1 s legs-only). Runs only when the lane produced a
    // clamped command this tick (hand_tx); the slew base was captured for
    // axis 6 at the edge regardless, so a lane that activates mid-slew starts
    // from the real encoder, not a stale zero.
    if (s_hand_recover_slewing && hand_tx) {
      const uint8_t h = HAND_AXIS;
      const float dt_tick = INTERP_PERIOD_US * 1e-6f;
      s_hand_recover_speed += RECOVER_SLEW_ACCEL_RPS2 * dt_tick;
      if (s_hand_recover_speed > RECOVER_SLEW_VEL_RPS) s_hand_recover_speed = RECOVER_SLEW_VEL_RPS;
      const float hstep = s_hand_recover_speed * dt_tick;
      float p = s_recover_pos[h];
      const float d0 = h_pos - p;
      const float ad = (d0 < 0.0f) ? -d0 : d0;
      float v = 0.0f;
      bool hand_done = false;
      if (ad > RECOVER_SLEW_DONE_EPS_REV) {
        if (ad <= hstep) { p = h_pos; }   // final approach — snap on (done detected next tick,
                                          // AFTER the re-clamp below has had its say — leg parity)
        else { p += (d0 > 0.0f ? hstep : -hstep); v = (d0 > 0.0f ? s_hand_recover_speed : -s_hand_recover_speed); }
      } else {
        p = h_pos;
        hand_done = true;                 // already converged (happy path: one-tick no-op)
      }
      const float fb = hand_axis().pos_rev;   // single-word atomic read
      float dev = p - fb;
      if (dev > MAX_LEAD_HAND_REV)       dev = MAX_LEAD_HAND_REV;
      else if (dev < -MAX_LEAD_HAND_REV) dev = -MAX_LEAD_HAND_REV;
      p = fb + dev;
      s_recover_pos[h] = p;
      h_pos = p;
      h_vel = v;
      if (hand_done) s_hand_recover_slewing = false;   // hand converged on its own clock
    }
  }

  // Publish targets into the cache (for telemetry) and transmit to CAN3.
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    axes[i].target_pos_rev   = cmd_pos[i];
    axes[i].target_vel_rps   = cmd_vel[i];
    axes[i].target_torque_Nm = cmd_tor[i];
  }
  if (hand_tx) {
    // The hand target cache is also HAND_CMD_ECHO's source while STREAMED
    // (telemetry.cpp re-sources the echo from here — the bridge's own TX is
    // invisible to the CAN sniff under SRX_DIS). torque_ff is HARD ZERO on the
    // hand lane in FW 17: the v6 host always sends torque_ff[6] = 0, no hand
    // torque-FF clamp constant exists yet (Phase 0 signed none), and the leg
    // wire-Nm clamp is a leg number that must not touch axis 6 — so the lane
    // transmits no torque at all until a hand torque-FF chapter defines one.
    hand_axis().target_pos_rev   = h_pos;
    hand_axis().target_vel_rps   = h_vel;
    hand_axis().target_torque_Nm = 0.0f;
  }
  // Cold-start interlock: suppress the whole MPC leg TX while a
  // firmware homing/activate/deactivate move is driving the same ODrives (see the
  // interlock note above). The target cache was written for all legs regardless.
  if (s_output_enabled && !coldstart) {
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      // Present-axis gate: never stream a 500 Hz setpoint to a node
      // that isn't on the bus. No-op on the full robot (all six present); on the
      // single-leg bench rig this drops the 5 phantom frames/tick to absent
      // nodes 1-5. The target cache above is written for all legs regardless.
      if (leg_present(i)) can_jugglebot_tx(ODrive::encode_leg_setpoint(i, cmd_pos[i], cmd_vel[i], cmd_tor[i]), TxCls::LEGS);   // TxCls::LEGS — latest-wins, deferral counted, never retried
    }
    // ── The 7th frame — the hand lane's set_input_pos (FW 17) ─────────────────
    // Inside the SAME `s_output_enabled && !coldstart` gate as the legs, on the
    // same two counts the retired bench probe argued: safety (the cold-start
    // interlock exists because a firmware homing/activate/deactivate move
    // drives the same ODrives — axis 6 homes on that very machine — so a hand
    // frame outside the gate would be the one producer that ignores it) and
    // burst integrity (the 7-frame burst the bus was qualified on 2026-08-30 is
    // one co-resident burst per tick, one gate).
    //
    // encode_leg_setpoint(HAND_AXIS, …) is the verified axis-6 path: wire
    // scales 100/100, NO sign flip (leg_sign is identity for axis 6), and
    // clip_position() bounds the value to [0, HAND_MOTOR_MAX_POSITION] as the
    // final backstop behind the clip already applied above. torque hard 0 (see
    // the target-cache comment). TxCls::LEGS — this IS the seventh interpolated
    // axis of the streaming burst; TxCls::HAND means "a hand_ops dispatch" and
    // borrowing it would contaminate the ERR_TIMEOUT arc's counter.
    //
    // ISR COST: identical to the flown bench probe's frame (one PRIMASK-guarded
    // u64 read + single-word float reads + one can_jugglebot_tx) plus the lane
    // math above — measured 2026-08-30 at ≤ 2 µs added jitter for the burst.
    // No FreeRTOS call anywhere, so leg_interp_init()'s above-syscall-ceiling
    // contract still holds.
    if (hand_tx && leg_present(HAND_AXIS)) {
      can_jugglebot_tx(ODrive::encode_leg_setpoint(HAND_AXIS, h_pos, h_vel, 0.0f), TxCls::LEGS);
      ++s_hand_sent;
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
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    s_base_pos[i] = s_base_vel[i] = s_base_accel[i] = s_base_torque[i] = 0.0f;
    s_jerk[i] = s_next_pos[i] = s_next2_pos[i] = 0.0f;
    s_v1[i] = 0.0f;
    s_recover_pos[i] = 0.0f;
    if (i < NUM_LEGS) {
      s_prev_accel[i] = 0.0f;   // leg-only arrays (the hand keeps scalar history below)
      s_stow_pos[i] = 0.0f;
    }
  }
  s_has_next = s_has_next2 = s_has_v1 = false;
  s_hand_active = false;
  s_hand_has_next = s_hand_has_next2 = s_hand_has_v1 = false;
  s_hand_ts_us = 0;
  s_hand_prev_accel = 0.0f;
  s_hand_have_prev_accel = false;
  s_hand_prev_recv_us = 0;
  s_hand_sent = 0;
  s_hand_unseen_skips = 0;
  s_hand_stale_holds = 0;
  s_hand_discard_legacy = 0;
  s_hand_lead_clamp_ticks = 0;
  s_hand_dev_over_ticks = 0;
  s_hand_dev_last = s_hand_dev_max = 0.0f;
  s_hand_dev_snap_cmd = s_hand_dev_snap_fb = 0.0f;
  s_hand_dev_trip_dev = s_hand_dev_trip_cmd = s_hand_dev_trip_fb = 0.0f;
  // The observe/arm switch returns to its power-on value (OBSERVE) like every
  // other static here: a re-armed interp can never inherit an armed hand
  // deviation trip from a previous session without an explicit `hand7 arm`.
  s_hand_dev_guard_armed = false;
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
  s_recover_slewing = false;
  s_output_enabled_prev = false;
  s_recover_speed = 0.0f;
  s_hand_recover_slewing = false;
  s_hand_recover_speed = 0.0f;
  s_deadline_misses = 0;
  s_max_jitter_us = 0;
  s_tick_count = 0;
  s_recover_slew_ticks = 0;
  s_extrap_ticks = 0;
  s_last_tick_us = 0;
  s_lead_clamp_mask = 0;
  s_torque_clamp_mask = 0;
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
  // HARD INVARIANT (audited): because interp_isr runs above the syscall
  // ceiling, interp_isr AND EVERYTHING IT CALLS make ZERO FreeRTOS API calls — any
  // FreeRTOS call from above the ceiling is undefined behaviour. The full call tree
  // is FreeRTOS-free: micros64() (PRIMASK-guarded + Arduino micros()),
  // latch_from_staging() (pure float math + file-statics), can_jugglebot_tx()
  // (PRIMASK-guarded FlexCAN_T4::write mailbox register write, NOT a mutex),
  // homing_active()/activate_active()/deactivate_active()/leg_present() (single-word
  // reads via non-inline cross-TU calls under -O2/no-LTO — the call is the barrier),
  // hand_source_streamed() (FW 17 — the same single-volatile-byte cross-TU read
  // class, hand_source.cpp), and the ODrive encoders (pure). Keep it that way.
  s_timer.begin(interp_isr, INTERP_PERIOD_US);
  s_timer.priority(16);
}

uint64_t interp_last_setpoint_us() { return atomic_read_u64(&s_last_setpoint_us); }
// Torn-load guard is mandatory here: s_last_tick_us is written by interp_isr, and
// this accessor runs in TASK context (hand_ops, on task_net). Without the mask the
// two 32-bit loads can straddle the ISR's store and produce a nonsense u64 — which
// in the phase stamp would look like a wild phase value, i.e. exactly the kind of
// artefact that would falsely refute the phase-quantisation model.
uint64_t interp_last_tick_us() { return atomic_read_u64(&s_last_tick_us); }
float interp_base_pos(uint8_t i) { return (i < NUM_AXES) ? s_base_pos[i] : 0.0f; }
bool  interp_have_latched() { return s_have_latched; }
void interp_set_output_enabled(bool en) { s_output_enabled = en; }
bool interp_output_enabled() { return s_output_enabled; }
uint32_t interp_deadline_misses() { return s_deadline_misses; }
uint32_t interp_max_jitter_us() { return s_max_jitter_us; }
void interp_reset_jitter() { s_max_jitter_us = 0; }
// Cumulative-since-boot occupancy census. No PRIMASK guard: each is a single
// naturally-aligned 32-bit word, so the load is atomic on Cortex-M7 (the u64
// interp_last_tick_us above is the case that genuinely needs the mask).
uint32_t interp_tick_count() { return s_tick_count; }
uint32_t interp_recover_slew_ticks() { return s_recover_slew_ticks; }
uint32_t interp_extrap_ticks() { return s_extrap_ticks; }
uint8_t interp_lead_clamp_mask() { return s_lead_clamp_mask; }
uint8_t interp_torque_clamp_mask() { return s_torque_clamp_mask; }

void interp_begin_stow() {
  // PRIMASK-publish (mirror interp_on_setpoint's exemplar above). The 500 Hz
  // interp_isr runs ABOVE the FreeRTOS syscall ceiling (raised it to
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

// ── Hand-lane accessors (FW 17) — single-word atomic loads, census idiom ─────
bool     interp_hand_lane_active()      { return s_hand_active; }
uint32_t interp_hand_lead_clamp_ticks() { return s_hand_lead_clamp_ticks; }
uint32_t interp_hand_sent()             { return s_hand_sent; }
uint32_t interp_hand_unseen_skips()     { return s_hand_unseen_skips; }
uint32_t interp_hand_stale_holds()      { return s_hand_stale_holds; }
uint32_t interp_hand_discard_legacy()   { return s_hand_discard_legacy; }
uint32_t interp_hand_dev_over_ticks()   { return s_hand_dev_over_ticks; }
float    interp_hand_dev_last()         { return s_hand_dev_last; }
float    interp_hand_dev_max()          { return s_hand_dev_max; }
float    interp_hand_dev_snap_cmd()     { return s_hand_dev_snap_cmd; }
float    interp_hand_dev_snap_fb()      { return s_hand_dev_snap_fb; }
// Trip-dedicated snapshot (most recent EXCEED tick) — the fault machine's
// latch read; the boot-cumulative dev_max trio above stays the console's
// observe-first read (2026-09-02 fix).
float    interp_hand_dev_trip_dev()     { return s_hand_dev_trip_dev; }
float    interp_hand_dev_trip_cmd()     { return s_hand_dev_trip_cmd; }
float    interp_hand_dev_trip_fb()      { return s_hand_dev_trip_fb; }
bool     interp_hand_dev_guard_armed()  { return s_hand_dev_guard_armed; }
void     interp_set_hand_dev_guard_armed(bool armed) { s_hand_dev_guard_armed = armed; }

// ── hand7 console + 1 Hz status (task_diag) — the gpio_poll console idiom ────
// One status line shared by the handler and the 1 Hz step so a scrollback never
// carries two formats. Counters are BOOT-CUMULATIVE ([cantx] idiom): difference
// two reads across a block, never read an absolute as a block's value. This is
// the REQUIRED lead-duty read (lead=; non-zero during a throw ⇒ hard-abort the
// sitting) and the observe-first deviation read (dev_max=/dev_over=).
static void hand7_print_status() {
  Serial.printf("[hand7] src=%s guard=%s lane=%s sent=%lu discard_legacy=%lu unseen=%lu"
                " stale=%lu lead=%lu dev_over=%lu dev_last=%.4f dev_max=%.4f"
                " dev_cmd=%.4f dev_fb=%.4f\n",
                hand_source_streamed() ? "STREAMED" : "LEGACY",
                s_hand_dev_guard_armed ? "ARMED" : "observe",
                s_hand_active ? "active" : "idle",
                (unsigned long)s_hand_sent,
                (unsigned long)s_hand_discard_legacy,
                (unsigned long)s_hand_unseen_skips,
                (unsigned long)s_hand_stale_holds,
                (unsigned long)s_hand_lead_clamp_ticks,
                (unsigned long)s_hand_dev_over_ticks,
                (double)s_hand_dev_last,
                (double)s_hand_dev_max,
                (double)s_hand_dev_snap_cmd,
                (double)s_hand_dev_snap_fb);
}

bool interp_hand7_console(const char* line) {
  if (line == nullptr || strncmp(line, "hand7", 5) != 0) return false;
  const char* arg = line + 5;
  while (*arg == ' ') ++arg;
  if      (strcmp(arg, "arm")     == 0) s_hand_dev_guard_armed = true;   // the second sitting's explicit step
  else if (strcmp(arg, "observe") == 0) s_hand_dev_guard_armed = false;
  else if (*arg != '\0')                return false;   // not this command after all
  hand7_print_status();
  return true;
}

void interp_hand7_diag_step() { hand7_print_status(); }

}  // namespace CanBridge
