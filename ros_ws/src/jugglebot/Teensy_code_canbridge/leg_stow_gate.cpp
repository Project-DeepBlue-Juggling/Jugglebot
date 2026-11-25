// =============================================================================
//  leg_stow_gate.cpp — no Leg leaves Closed-loop above Stow (FW 53, I-FW-19)
// =============================================================================
//  See leg_stow_gate.h for the contract. Design: fw53_design.md 1.2 and 1.3.
//
//  Why a wrapper and not a frame filter inside can_jugglebot_tx: only the caller
//  knows the alternative to IDLE (freeze on a segment, stay silent, retry) and
//  the caller's intent (the homing Leg); a filter could only drop the frame and
//  the ladder would then report an IDLE it never got. can_jugglebot_tx is also
//  the 500 Hz IRQ-off hot path and is not compiled natively, so a check there
//  would be the one safety check in the system with no native test.
//
//  Concurrency. The wrappers run in several tasks (net/RPC, fault, the ladders'
//  task). The predicate reads only RX-task-written caches: per Leg it reads the
//  heartbeat STAMP first and the state after it. The RX decode writes axis_state
//  BEFORE last_heartbeat_us (can_buses.cpp heartbeat case), so a stamp read here
//  is never paired with an OLDER state than the one it stamped: a fresh stamp
//  cannot certify a stale IDLE. The HELD mask and the census are read-modify-
//  write from several tasks, so they change under a PRIMASK guard.
// =============================================================================

#include "leg_stow_gate.h"

#include <cmath>

#include "canbridge_config.h"
#include "protocol_config.h"    // ODriveState, ODriveCmd
#include "hardware_config.h"    // JBOp::TARGET_REACHED_*, ODriveDefaults::TRAP_DEC_LIMIT_RPS2
#include "axis_state.h"         // axes[], leg_present, snapshot_pos_vel
#include "odrive_protocol.h"    // the encoders this file alone may call
#include "can_buses.h"          // can_jugglebot_tx, tx_reached_the_wire, TxCls
#include "time_base.h"          // micros64, atomic_*_u64, rx_older_than
#include "leg_homing.h"         // homing_axis_in_flight
#include "fault_machine.h"      // fault_can_bus_down (the shared descent monitor)

namespace CanBridge {

// ── State ────────────────────────────────────────────────────────────────────
static volatile uint64_t s_cl_cmd_us[NUM_LEGS] = { 0 };   // atomic_*_u64 only
static volatile uint8_t  s_held_mask = 0;
static volatile uint32_t s_refusals  = 0;
// rig_is_bench witnesses. Written by task_can_rx only (rig_note_*), except the
// robot latch, which any reader may SET (never clear): a single byte, so a race
// can only latch robot sooner.
static volatile uint8_t  s_rig_heard_mask    = 0;     // bit a = axis a noted
static volatile uint64_t s_rig_first_axis_us = 0;     // first axis frame since boot; atomic_*_u64
static volatile uint64_t s_rig_tx_since_us   = 0;     // bus transmittable since; 0 = not at last sample
static volatile bool     s_rig_platform_seen = false;
static volatile bool     s_rig_robot         = false;

void leg_stow_gate_init() {
  for (uint8_t i = 0; i < NUM_LEGS; ++i) atomic_write_u64(&s_cl_cmd_us[i], 0);
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  s_held_mask = 0;
  s_refusals  = 0;
  s_rig_heard_mask    = 0;
  s_rig_platform_seen = false;
  s_rig_robot         = false;
  __set_PRIMASK(pm);
  atomic_write_u64(&s_rig_first_axis_us, 0);
  atomic_write_u64(&s_rig_tx_since_us, 0);
}

// ── Bench or robot ───────────────────────────────────────────────────────────
// 0 is the "never" sentinel of both stamps; micros64() is nonzero once a frame can
// arrive, and a 0 argument is bumped to 1 so a sentinel can never be written.
void rig_note_axis_frame(uint8_t axis, uint64_t now) {
  if (axis >= NUM_AXES) return;
  // Stamp before the mask bit: a reader that sees the bit always sees an anchor.
  // (A reader that sees the stamp without the bit counts zero axes: not a bench.)
  if (atomic_read_u64(&s_rig_first_axis_us) == 0) atomic_write_u64(&s_rig_first_axis_us, now ? now : 1);
  s_rig_heard_mask = (uint8_t)(s_rig_heard_mask | (uint8_t)(1u << axis));
}
void rig_note_platform_frame() { s_rig_platform_seen = true; }
void rig_note_bus(bool transmittable, uint64_t now) {
  if (!transmittable)                                 atomic_write_u64(&s_rig_tx_since_us, 0);
  else if (atomic_read_u64(&s_rig_tx_since_us) == 0) atomic_write_u64(&s_rig_tx_since_us, now ? now : 1);
}

// The union with heartbeat_seen keeps the count at least as large as the presence
// latch the rest of the firmware uses (leg_present), so no Leg the gate treats as
// present can be missing from the census.
uint8_t rig_axes_heard_mask() {
  uint8_t m = s_rig_heard_mask;
  for (uint8_t a = 0; a < NUM_AXES; ++a)
    if (axes[a].heartbeat_seen) m = (uint8_t)(m | (uint8_t)(1u << a));
  return m;
}
bool rig_platform_seen() { return s_rig_platform_seen; }
bool rig_robot_latched() { return s_rig_robot; }

bool rig_is_bench(uint64_t now) {
  if (s_rig_robot) return false;
  const uint8_t m = rig_axes_heard_mask();
  const bool two_or_more = (m & (uint8_t)(m - 1u)) != 0;
  if (two_or_more || s_rig_platform_seen) { s_rig_robot = true; return false; }
  if (m == 0) return false;                       // nothing heard: no evidence either way
  const uint64_t first = atomic_read_u64(&s_rig_first_axis_us);
  const uint64_t tx    = atomic_read_u64(&s_rig_tx_since_us);
  if (first == 0 || tx == 0) return false;        // no anchor, or the bus is not transmittable
  const uint64_t anchor = first > tx ? first : tx;
  // A stamp newer than `now` (caller captured now first) is not yet settled.
  return now >= anchor && now - anchor >= RIG_BENCH_SETTLE_US;
}

bool leg_idle_permitted(uint64_t now, uint8_t except_leg) {
  return rig_is_bench(now) || platform_at_stow(now, except_leg).ok;
}

// ── Per-Leg freshness ────────────────────────────────────────────────────────
// The heartbeat-derived state of a present Leg, or a refusal reason. Reads the
// stamp FIRST, then the state (see the concurrency note at the top).
static uint8_t fresh_state(uint8_t j, uint64_t now, uint8_t* state_out) {
  const uint64_t hb = atomic_read_u64(&axes[j].last_heartbeat_us);
  if (hb == 0 || rx_older_than(now, hb, CAN_HEARTBEAT_STALE_US)) return StowReason::STALE_HB;
  const uint64_t cl = atomic_read_u64(&s_cl_cmd_us[j]);
  // A heartbeat cached before (or just after) our last CLOSED_LOOP command may
  // still report IDLE for a Leg that is now holding. Direct comparison, no
  // subtraction: nothing here can wrap.
  if (cl != 0 && hb < cl + CL_ENGAGE_SETTLE_US) return StowReason::CL_UNCONFIRMED;
  *state_out = axes[j].axis_state;
  return StowReason::OK;
}

// Clause (b): a Closed-loop Leg at rest inside the Stow window.
static uint8_t closed_loop_at_stow(uint8_t j, uint64_t now) {
  if (axes[j].active_errors != 0) return StowReason::ERRORED;
  float pos = 0.0f, vel = 0.0f; uint64_t ts = 0;
  snapshot_pos_vel(axes[j], pos, vel, ts);
  if (ts == 0 || rx_older_than(now, ts, MOTOR_FB_STALENESS_US)) return StowReason::STALE_ENC;
  // Written so a NaN fails each test (every comparison with NaN is false).
  if (!(pos >= STOW_WINDOW_MIN_REV &&
        pos <= STOW_OFF_POSE_REV + JBOp::TARGET_REACHED_POS_TOL_REV)) return StowReason::NOT_AT_STOW;
  if (!(std::fabs(vel) <= JBOp::TARGET_REACHED_VEL_TOL_RPS)) return StowReason::MOVING;
  return StowReason::OK;
}

// One present Leg's verdict: OK (clause (a) or (b)) or its refusal reason.
static uint8_t leg_stow_reason(uint8_t j, uint64_t now) {
  uint8_t st = 0;
  const uint8_t r = fresh_state(j, now, &st);
  if (r != StowReason::OK) return r;
  if (st == ODriveState::IDLE || st == ODriveState::ENCODER_INDEX_SEARCH) return StowReason::OK;  // (a)
  return (st == ODriveState::CLOSED_LOOP) ? closed_loop_at_stow(j, now)                        // (b)
                                          : StowReason::UNKNOWN_STATE;
}

StowVerdict platform_at_stow(uint64_t now, uint8_t except_leg) {
  for (uint8_t j = 0; j < NUM_LEGS; ++j) {
    if (j == except_leg || !leg_present(j)) continue;
    const uint8_t r = leg_stow_reason(j, now);
    if (r != StowReason::OK) return StowVerdict{ false, j, r };
  }
  return StowVerdict{ true, 0xFF, StowReason::OK };
}

bool platform_blocked_only_by_cl_unconfirmed(uint64_t now) {
  bool any = false;
  for (uint8_t j = 0; j < NUM_LEGS; ++j) {
    if (!leg_present(j)) continue;
    const uint8_t r = leg_stow_reason(j, now);
    if (r == StowReason::OK) continue;
    if (r != StowReason::CL_UNCONFIRMED) return false;
    any = true;
  }
  return any;
}

AbortIdle leg_abort_idle_verdict(uint64_t now, uint64_t t_abort_us) {
  if (leg_idle_permitted(now)) return AbortIdle::SEND;
  // now >= t_abort_us first: an unsigned difference across a stamp newer than now
  // (BUG-93) would read as a huge age and end the wait, which is only the
  // conservative side, but the order keeps the arithmetic honest.
  if (now >= t_abort_us && now - t_abort_us < CL_CONFIRM_WAIT_US &&
      platform_blocked_only_by_cl_unconfirmed(now)) return AbortIdle::WAIT;
  return AbortIdle::HOLD;
}

bool all_present_legs_idle(uint64_t now) {
  for (uint8_t j = 0; j < NUM_LEGS; ++j) {
    if (!leg_present(j)) continue;
    uint8_t st = 0;
    if (fresh_state(j, now, &st) != StowReason::OK) return false;
    if (st != ODriveState::IDLE) return false;
  }
  return true;
}

// ── HELD mask + census ───────────────────────────────────────────────────────
static void set_held(uint8_t leg) {
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  s_held_mask = (uint8_t)(s_held_mask | (uint8_t)(1u << leg));
  __set_PRIMASK(pm);
}
uint8_t leg_held_mask() { return s_held_mask; }
void leg_held_clear(uint8_t leg) {
  if (leg >= NUM_LEGS) return;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  s_held_mask = (uint8_t)(s_held_mask & (uint8_t)~(1u << leg));
  __set_PRIMASK(pm);
}
void leg_held_clear_all() {
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  s_held_mask = 0;
  __set_PRIMASK(pm);
}
uint32_t leg_deenergise_refusals() { return s_refusals; }
uint64_t leg_cl_cmd_us(uint8_t leg) {
  return leg < NUM_LEGS ? atomic_read_u64(&s_cl_cmd_us[leg]) : 0;
}

static void note_refusal(uint8_t leg, Who who) {
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  s_refusals = s_refusals + 1;
  __set_PRIMASK(pm);
  if (who == Who::LADDER || who == Who::STOW) set_held(leg);
}

// ── The one gate in front of every Leg de-energise ───────────────────────────
// True = the request for `leg` may go out. On false the refusal is recorded.
static bool leg_deenergise_allowed(uint8_t leg, Who who) {
  if (leg_idle_permitted(micros64())) return true;
  note_refusal(leg, who);
  return false;
}

static DeResult send(const ODrive::CanFrame& f, uint8_t cls) {
  return tx_reached_the_wire(can_jugglebot_tx(f, cls)) ? DeResult::SENT : DeResult::TX_FAILED;
}

DeResult odrive_set_state_tx(uint8_t axis, uint32_t state, uint8_t cls, Who who) {
  if (axis >= NUM_AXES) return DeResult::REFUSED;
  if (axis == HAND_AXIS) return send(ODrive::encode_set_state(axis, state), cls);   // exempt
  if (state == ODriveState::CLOSED_LOOP) {
    atomic_write_u64(&s_cl_cmd_us[axis], micros64());   // stamp BEFORE the send
    return send(ODrive::encode_set_state(axis, state), cls);
  }
  const bool homing_owner = who == Who::HOMING_OWNER && state == ODriveState::IDLE &&
                            axis == homing_axis_in_flight();
  if (!homing_owner && !leg_deenergise_allowed(axis, who)) return DeResult::REFUSED;
  return send(ODrive::encode_set_state(axis, state), cls);
}

DeResult odrive_reboot_tx(uint8_t axis, uint8_t cls, Who who) {
  if (axis >= NUM_AXES) return DeResult::REFUSED;
  if (axis != HAND_AXIS && !leg_deenergise_allowed(axis, who)) return DeResult::REFUSED;
  return send(ODrive::encode_reboot(axis), cls);
}

DeResult odrive_sdo_write_tx(uint8_t axis, uint16_t endpoint_id, float value, uint8_t cls, Who who) {
  if (axis >= NUM_AXES) return DeResult::REFUSED;
  if (axis != HAND_AXIS && !leg_deenergise_allowed(axis, who)) return DeResult::REFUSED;
  return send(ODrive::encode_sdo_write(axis, endpoint_id, value), cls);
}

// ── The hold ─────────────────────────────────────────────────────────────────
float leg_freeze_target(float pos, float vel, float seg_lo, float seg_hi) {
  const float lo = seg_lo < seg_hi ? seg_lo : seg_hi;
  const float hi = seg_lo < seg_hi ? seg_hi : seg_lo;
  const float t  = pos + vel * std::fabs(vel) / (2.0f * ODriveDefaults::TRAP_DEC_LIMIT_RPS2);
  return t < lo ? lo : (t > hi ? hi : t);
}

bool leg_freeze_tx(uint8_t leg, float seg_lo, float seg_hi) {
  if (leg >= NUM_LEGS) return false;
  set_held(leg);
  float pos = 0.0f, vel = 0.0f; uint64_t ts = 0;
  const uint64_t now = micros64();
  snapshot_pos_vel(axes[leg], pos, vel, ts);
  if (ts == 0 || rx_older_than(now, ts, MOTOR_FB_STALENESS_US)) return false;
  if (!std::isfinite(pos) || !std::isfinite(vel) ||
      !std::isfinite(seg_lo) || !std::isfinite(seg_hi)) return false;
  const float target = leg_freeze_target(pos, vel, seg_lo, seg_hi);
  return tx_reached_the_wire(can_jugglebot_tx(
      ODrive::encode_set_input_pos(leg, ODrive::leg_sign(leg, target), 0, 0), TxCls::SAFETY));
}

// ── The gated descent to Stow (DEACTIVATE MONITOR, ACTIVATE RETURN) ──────────
bool leg_dropped_out(uint8_t leg, uint64_t now) {
  if (leg >= NUM_LEGS) return false;
  // Stamp read before state (the receive decode writes the state first).
  const uint64_t hb = atomic_read_u64(&axes[leg].last_heartbeat_us);
  if (hb == 0 || rx_older_than(now, hb, CAN_HEARTBEAT_STALE_US)) return false;   // no evidence
  if (hb < leg_cl_cmd_us(leg) + CL_ENGAGE_SETTLE_US) return false;               // pre-CL heartbeat
  return axes[leg].axis_state != ODriveState::CLOSED_LOOP;
}

static constexpr uint8_t LEG_BITS = (uint8_t)((1u << NUM_LEGS) - 1u);

// ── The ladders' Closed-loop confirmation before motion (FW 53 U8) ───────────
uint64_t legs_last_cl_cmd_us(uint8_t legs) {
  uint64_t t = 0;
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (legs & (uint8_t)(1u << i)) { const uint64_t c = leg_cl_cmd_us(i); if (c > t) t = c; }
  return t;
}

ClConfirm legs_cl_confirm(uint8_t legs, uint64_t now) {
  legs = (uint8_t)(legs & LEG_BITS);
  bool all = true;
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    if (!(legs & (uint8_t)(1u << i))) continue;
    if (axes[i].active_errors != 0) return ClConfirm::FAILED;
    // Stamp read before state (the receive decode writes the state first).
    const uint64_t cl = leg_cl_cmd_us(i);
    const uint64_t hb = atomic_read_u64(&axes[i].last_heartbeat_us);
    const bool post_cl = cl != 0 && hb != 0 && !rx_older_than(now, hb, CAN_HEARTBEAT_STALE_US) &&
                         hb >= cl + CL_ENGAGE_SETTLE_US;
    if (!post_cl) { all = false; continue; }                   // cached pre-CL, stale or none: no evidence
    if (axes[i].axis_state != ODriveState::CLOSED_LOOP) return ClConfirm::FAILED;
  }
  if (all) return ClConfirm::CONFIRMED;
  const uint64_t t0 = legs_last_cl_cmd_us(legs);
  // A window start newer than `now` (BUG-93 order) is still inside the window.
  if (t0 != 0 && (now < t0 || now - t0 < CL_CONFIRM_WAIT_US)) return ClConfirm::WAIT;
  return ClConfirm::EXPIRED;
}

static inline float min3(float a, float b, float c) { const float m = a < b ? a : b; return m < c ? m : c; }
static inline float max3(float a, float b, float c) { const float m = a > b ? a : b; return m > c ? m : c; }

// Freeze every live target except `skip` on the hull of its two ends and Stow.
static void freeze_live(uint8_t live, uint8_t skip, const float seg_a[], const float seg_b[]) {
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if ((live & (uint8_t)(1u << i)) && !(skip & (uint8_t)(1u << i)))
      (void)leg_freeze_tx(i, min3(seg_a[i], seg_b[i], STOW_OFF_POSE_REV),
                             max3(seg_a[i], seg_b[i], STOW_OFF_POSE_REV));
}

// Gated IDLE of every live target; a SENT Leg joins `done` and loses its HELD bit
// (a race refusal earlier in the same tick must not outlive the IDLE).
static void gated_idle_into_done(uint8_t live, uint8_t& done) {
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    if (!(live & (uint8_t)(1u << i))) continue;
    if (odrive_set_state_tx(i, ODriveState::IDLE, TxCls::OTHER, Who::LADDER) == DeResult::SENT) {
      done |= (uint8_t)(1u << i);
      leg_held_clear(i);
    }
  }
}

DescentStep stow_descent_step(uint8_t targets, uint8_t& done,
                              const float seg_a[], const float seg_b[],
                              bool timed_out, uint64_t now) {
  const uint8_t live = (uint8_t)(targets & ~done & LEG_BITS);
  // (1) Bus down: nothing can be sent. The ODrives finish the descent they hold;
  //     the fault machine's deferred stow (armed at the trip) takes over.
  if (fault_can_bus_down()) return DescentStep::BUS_DOWN;
  // (2) A target Leg disarmed itself: never lower the Platform with one Leg free.
  uint8_t dropped = 0;
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if ((live & (uint8_t)(1u << i)) && leg_dropped_out(i, now)) dropped |= (uint8_t)(1u << i);
  if (dropped) { freeze_live(live, dropped, seg_a, seg_b); return DescentStep::DROPPED; }
  // (3) Arrival: every live target at Stow AND settled, judged while still CL. The
  //     predicate runs once BEFORE the IDLE loop, so a refusal here (e.g. a stale
  //     heartbeat) is retried next tick without touching the census or HELD.
  bool all_arrived = true;
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    if (!(live & (uint8_t)(1u << i))) continue;
    const float pos = axes[i].pos_rev;   // Jugglebot conv (== STOW_OFF_POSE_REV conv)
    const float vel = axes[i].vel_rps;
    if (fabsf(pos - STOW_OFF_POSE_REV) > JBOp::TARGET_REACHED_POS_TOL_REV ||
        fabsf(vel) > JBOp::TARGET_REACHED_VEL_TOL_RPS) all_arrived = false;
  }
  if (all_arrived && leg_idle_permitted(now)) {
    gated_idle_into_done(live, done);
    if ((uint8_t)(targets & ~done & LEG_BITS) == 0) return DescentStep::ARRIVED;
  }
  // (4) Timeout: IDLE only where permitted (Stow, or the bench), else hold.
  if (timed_out) {
    const uint8_t still = (uint8_t)(targets & ~done & LEG_BITS);
    if (leg_idle_permitted(now)) gated_idle_into_done(still, done);
    else                         freeze_live(still, 0, seg_a, seg_b);
    return DescentStep::TIMED_OUT;
  }
  return DescentStep::RUNNING;
}

}  // namespace CanBridge
