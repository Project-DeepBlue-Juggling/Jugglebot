// =============================================================================
//  leg_activate.cpp — firmware activate (TRAP_TRAJ move to active pose)
// =============================================================================
//  See leg_activate.h for the design rationale. The ladder, per targeted leg:
//    SETUP   : refuse if any target leg has active_errors (legacy _gentle_move
//              parity) → seed input_pos(actual current, NON-clipped) → traj
//              vel/acc limits → POSITION/TRAP_TRAJ mode → CLOSED_LOOP
//    settle  : let CLOSED_LOOP engage + mode/limits take effect
//    COMMAND : set_input_pos(ACTIVATE_POSITION_REVS) → ODrive TRAP_TRAJ starts
//    MONITOR : stay active until every target leg reaches the active pose +
//              settles (|pos−target| ≤ tol AND |vel| ≤ tol) → mark OK, go idle.
//  The ODrive owns the trajectory; the Jetson observes completion independently
//  via telemetry. Staying active through MONITOR is what makes HOME/ACTIVATE
//  mutually exclusive for the WHOLE physical move (not just the ~10 ms fire) —
//  the firmware, not host serialization, is the safety boundary.
//
//  HAND_MOVE_TO (FW 26) runs THIS ladder for axis 6 alone with a caller target and
//  cruise (activate_target_rev / activate_traj_vel_rps read s_mode), answers its
//  request through a reply mailbox the net task drains, and retargets in flight
//  on a second request. See leg_activate.h for why each of those three exists.
//
//  HAND STALL GUARD (FW 36, BUG-56): every active tick, hand_stall_guard() drops a
//  hand that has sat at high current without moving for HAND_STALL_HOLD_US to IDLE
//  (leg_activate.h; thresholds + physics in canbridge_config.h).
//
//  FW 53 (I-FW-19, owner rule 2026-11-25: no Leg is made IDLE unless the Platform
//  is already at Stow; the Hand is exempt). Every Set_Axis_State goes through the
//  Stow gate (leg_stow_gate.h). The Hand is IDLEd on every abort, as before. The
//  Legs, by where the abort lands:
//    * bus down (any phase): nothing to the Legs, FAILED; the ODrives finish the
//      move they hold and the fault machine's deferred stow lowers the Platform
//      at reconnect (it starts only while !activate_active(), and the ladder has
//      ended);
//    * FW 53 U8: COMMAND sends no target until every target Leg has confirmed
//      CLOSED_LOOP by a heartbeat received after its own CL command (the Hand
//      is not in this wait); a Leg with an error, or no confirmation within
//      CL_CONFIRM_WAIT_US (400 ms) of the last CL command, takes the
//      pre-rise abort below, so no Leg rises with another free;
//    * before the rise is commanded (SETUP, COMMAND): ABORT_WAIT. The gated IDLE
//      goes out once the Platform is at Stow (the usual case after Homing: FW 52's
//      outcome), waiting at most CL_CONFIRM_WAIT_US while the only refusals are
//      unconfirmed CLOSED_LOOP sends; otherwise it is refused and the Legs hold
//      (configured ones at their measured seed), HELD;
//    * mid-rise (MONITOR): a target Leg that disarmed itself (a post-CL heartbeat
//      not CLOSED_LOOP, or a Get_Error with an error or disarm reason, checked
//      BEFORE the Soft E-stop that error raises) → freeze the others
//      on [seed, target], FAILED/HELD. A Soft E-stop (Hand ODrive error, the
//      HAND_STALL latch, overspeed), commands-not-allowed or the timeout → if
//      Idle is already permitted (the bench, or the Legs still at rest at Stow)
//      the gated IDLE; otherwise RETURN TO STOW (owner, 2026-11-25: "STOW is the
//      safe state"): every Leg is re-targeted to STOW_OFF_POSE_REV under the same
//      gentle TRAP_TRAJ profile, and the descent monitor shared with DEACTIVATE
//      (stow_descent_step) IDLEs it through the gate on encoder-confirmed
//      arrival. A Soft E-stop during the return does not interrupt it. The
//      ACTIVATE ends FAILED either way; HELD only if the return itself fails;
//    * the Hand's PASSTHROUGH hand-off refused after arrival: only the Hand is
//      IDLEd; the Legs hold the Active pose in TRAP_TRAJ, OK.
// =============================================================================
#include "leg_activate.h"

#include <Arduino.h>          // __get_PRIMASK / __disable_irq / __set_PRIMASK
#include <cmath>              // fabsf, std::isfinite
#include <cstring>            // memset
#include "canbridge_config.h"
#include "protocol_config.h"  // ODriveState, ODriveControlMode, ODriveInputMode
#include "hardware_config.h"  // JBOp::, ODriveDefaults::
#include "udp_protocol.h"     // JbUdp::RpcStatus / BusHealth / GuardMode / AXIS_ALL
#include "odrive_protocol.h"
#include "axis_state.h"
#include "can_buses.h"
#include "fault_machine.h"    // fault_can_bus_down / fault_guard_mode
#include "leg_homing.h"       // homing_active (no concurrent cold-start moves)
#include "leg_deactivate.h"   // deactivate_active (no concurrent cold-start moves)
#include "leg_stow_gate.h"    // odrive_set_state_tx, leg_idle_permitted, stow_descent_step (I-FW-19)
#include "time_base.h"        // micros64

namespace CanBridge {

// ── Sub-phase ladder ──────────────────────────────────────────────────────────
enum class APhase : uint8_t {
  IDLE,      // nothing running
  SETUP,     // error gate + seed + traj limits + TRAP_TRAJ mode + CLOSED_LOOP issued
  COMMAND,   // settle elapsed → command the active-pose target
  MONITOR,   // target commanded → wait for the leg(s) to reach + settle
  ABORT_WAIT,// pre-rise abort: wait (bounded) for CL confirmation, then the gated IDLE
  RETURN,    // mid-rise abort: the Legs descend to STOW, IDLE'd there through the gate
};

// The settle before COMMAND is CL_ENGAGE_SETTLE_US (canbridge_config.h, was this
// file's SETTLE_SETUP_US, same 10 ms): the Stow gate's CL-confirmation constant.
// Hard timeout on the WHOLE op (SETUP → COMMAND → physical move → settle). The
// ODrive runs the trajectory autonomously; MONITOR keeps the op "active" until it
// completes, so HOME is rejected for the whole move (not just the ~10 ms fire).
// The return to Stow (RETURN) gets its own clock of the same length.
static const uint64_t A_TIMEOUT_US = (uint64_t)(JBOp::GENTLE_MOVE_TIMEOUT_S * 1.0e6f);

// ── State (owned by the activate task; start-request handed off from net task) ─
static volatile bool    s_start_req  = false;   // net task → activate task
static volatile uint8_t s_start_axis = 0xFF;
// The Leg targets latched with the start (FW 51, BUG-130). ACTIVATE latches the
// constants JBOp::ACTIVATE_POSITION_REVS; ACTIVATE_TO latches the caller's six,
// already validated inside ACTIVATE_TO_BAND_REV by activate_to_request. Written by
// the net task only inside the same IRQ guard that sets s_start_req, copied to
// s_leg_target by the activate task inside the guard that consumes it, so the
// ladder never reads a half-written set.
static volatile float s_start_leg_target[HAND_AXIS];
static float          s_leg_target[HAND_AXIS];   // activate-task-owned: the move in flight

static APhase   s_phase   = APhase::IDLE;
static uint8_t  s_targets = 0;        // bitmask of legs being activated (bit i = leg i)
static uint8_t  s_setup_idx = 0;      // SETUP cursor — configure ONE leg per tick
static uint64_t s_t_start_us = 0;     // ladder start (overall timeout clock)
static uint64_t s_t_phase_us = 0;     // current sub-phase entry (settle clock)
static uint8_t  s_result[NUM_AXES] = { ACTIVATE_NONE };
// FW 53 U3 (the abort paths).
static constexpr uint8_t LEG_BITS = (uint8_t)((1u << NUM_LEGS) - 1u);
static float    s_seed[NUM_LEGS] = { 0.0f };  // measured pos at configure: one end of the freeze segment
static uint8_t  s_done = 0;           // RETURN: Legs IDLE'd at Stow (the gate SENT)
static uint8_t  s_ret_unsent = 0;     // RETURN: Legs whose Stow target has not reached the wire yet
static uint64_t s_t_abort_us = 0;     // ABORT_WAIT / RETURN entry (their own clocks)

// ── Hand stall guard (FW 36, BUG-56) — activate-task owned, s_stalled read by
//    the telemetry task (single-byte stores are atomic on Cortex-M7). ──────────
static bool          s_stall_timing = false;   // the stall predicate held last tick
static uint64_t      s_stall_t0_us  = 0;       // first tick of the current run
static volatile bool s_stalled[NUM_AXES] = { false };   // sticky, Diagnostic flags bit2
// ── Hand stall guard on the streamed lane and latched holds (FW 48, BUG-123) —
//    activate-task owned; s_lane_stalled read by the telemetry task. ───────────
static bool          s_lane_timing       = false;  // the stall predicate held at the last sample
static uint64_t      s_lane_t0_us        = 0;      // iq stamp of the run's first sample
static uint64_t      s_lane_last_iq_us   = 0;      // iq stamp of the last sample consumed
static bool          s_lane_idle_pending = false;  // a trip whose IDLE the TX path refused
static volatile bool s_lane_stalled      = false;  // sticky, Diagnostic flags bit3
// A new ladder run starts with a clean stall record for the axes it targets.
// BUG-131 (FW 49): a ladder that targets the Hand also drops a lane trip's
// still-refused IDLE. The ladder owns the Hand from here: SETUP issues its own
// CLOSED_LOOP and its own stall guard covers the move, so a late lane IDLE could
// only land mid-move. activate_step() runs the lane guard's retry BEFORE any
// ladder start, so the IDLE still gets its last try on this tick. The flag is
// deliberately NOT dropped on CLEAR_ERRORS: an IDLE that never went out may
// leave a stalled Hand in CLOSED_LOOP, and the operator's clear does not
// de-energise it, so the retry runs until the frame is queued or a ladder
// takes the Hand over.
static void stall_reset_for_targets() {
  s_stall_timing = false;
  for (uint8_t i = 0; i < NUM_AXES; ++i)
    if (s_targets & (uint8_t)(1u << i)) s_stalled[i] = false;
  if (s_targets & (uint8_t)(1u << HAND_AXIS)) { s_lane_stalled = false; s_lane_idle_pending = false; }
}

// ── HAND_MOVE_TO (FW 26) — the same ladder, a caller target (leg_activate.h) ──
// s_mode says which op owns the ladder; it is written BEFORE s_phase leaves IDLE
// (inside the same IRQ guard) so the net task never sees a running ladder with a
// stale owner.
enum class AMode : uint8_t { ACTIVATE, HAND_MOVE };
static AMode s_mode = AMode::ACTIVATE;
// net task → activate task: one latched start-or-retarget.
static volatile bool     s_hm_req = false;
static volatile float    s_hm_req_target = 0.0f;
static volatile float    s_hm_req_vel = 0.0f;
static volatile uint16_t s_hm_req_id = 0;
static volatile uint64_t s_hm_req_t_us = 0;
// activate-task-owned: the move in flight.
static float    s_hm_target = 0.0f;
static float    s_hm_vel = 0.0f;
static uint16_t s_hm_id = 0;
static uint64_t s_hm_t_accept_us = 0;
// activate task → net task reply mailbox. At most two replies per activate_step
// (a SUPERSEDED for the old request + a terminal one for the new), and the net
// task drains at 1 kHz against task_homing's HOMING_RATE_HZ, so 4 never fills;
// an overflow is counted (a lost reply is a host RpcTimeout, never a silent OK).
static constexpr uint8_t HM_MAILBOX_CAP = 4;
static HandMoveReply s_hm_box[HM_MAILBOX_CAP];
static volatile uint8_t  s_hm_box_head = 0;    // next pop
static volatile uint8_t  s_hm_box_count = 0;
static volatile uint32_t s_hm_box_drops = 0;

// ── Per-axis activate target (no second copy of any constant) ─────────────────
// Legs: the IK of [0,0,default_active_z,0,0,0]. Hand: JBOp::HAND_ACTIVATE_POSITION_REV
// (0.0 rev = the clip floor, one clean revolution-count above the homed hardstop
// at HAND_ABS_POS_REV = -0.10). Both are codegen'd — never a literal here.
static inline float activate_target_rev(uint8_t a) {
  if (a == HAND_AXIS)   // HAND_MOVE_TO: the caller's validated target, else the park
    return (s_mode == AMode::HAND_MOVE) ? s_hm_target : JBOp::HAND_ACTIVATE_POSITION_REV;
  return s_leg_target[a];   // the latched set: the constants (ACTIVATE) or the validated caller set (ACTIVATE_TO)
}
// TRAP_TRAJ cruise: the caller's (validated <= GENTLE_MOVE_VEL_LIMIT_RPS) for a
// HAND_MOVE_TO, the op's own gentle ceiling for ACTIVATE.
static inline float activate_traj_vel_rps() {
  return (s_mode == AMode::HAND_MOVE) ? s_hm_vel : JBOp::GENTLE_MOVE_VEL_LIMIT_RPS;
}

// Post one HAND_MOVE_TO reply for request `id` (activate-task context). The
// measured hand state rides along so the caller learns where the hand IS.
static void hm_post(uint16_t id, uint16_t status, uint8_t outcome, float target,
                    uint64_t t_accept_us) {
  HandMoveReply r;
  r.req_id = id;
  r.status = status;
  memset(&r.result, 0, sizeof(r.result));
  r.result.outcome    = outcome;
  r.result.pos_rev    = axes[HAND_AXIS].pos_rev;
  r.result.vel_rps    = axes[HAND_AXIS].vel_rps;
  r.result.target_rev = target;
  r.result.elapsed_ms = (uint32_t)((micros64() - t_accept_us) / 1000ull);
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  if (s_hm_box_count < HM_MAILBOX_CAP) {
    s_hm_box[(uint8_t)((s_hm_box_head + s_hm_box_count) % HM_MAILBOX_CAP)] = r;
    s_hm_box_count = (uint8_t)(s_hm_box_count + 1);
  } else {
    s_hm_box_drops = s_hm_box_drops + 1;
  }
  __set_PRIMASK(pm);
}
// Command every target's trajectory target (COMMAND, and a HAND_MOVE_TO retarget
// in MONITOR — both through this ONE loop, so the hand's setpoint has one cold-start
// wire path). Position-only target (vel_ff = tor_ff = 0). encode_leg_setpoint clips
// to the axis's [0,max] (defence against a bad target) + applies leg_sign
// (identity on axis 6). The ODrive TRAP_TRAJ planner trajectories from the
// seeded current pos (or, on a retarget, from the current state) to here. It is
// the SAME encoder the 500 Hz hand lane uses — no second hand codec.
static void command_targets() {
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
    if (!(s_targets & (uint8_t)(1u << i))) continue;
    can_jugglebot_send(
        ODrive::encode_leg_setpoint(i, activate_target_rev(i), 0.0f, 0.0f));
  }
}

// Terminal reply for the move in flight (only when a HAND_MOVE_TO owns the ladder).
static void hm_finish(uint16_t status, uint8_t outcome) {
  if (s_mode != AMode::HAND_MOVE) return;
  hm_post(s_hm_id, status, outcome, s_hm_target, s_hm_t_accept_us);
}

// ── Bus / fault gate (mirror leg_homing::homing_allowed). Never drive a
//    confirmed-dead bus or an E-STOP'd system. ───────────────────────────────
static bool activate_allowed() {
  // Bus term routed through the SHARED predicate (2026-07-29): this used to
  // re-derive it from can_buses_stats().jugglebot_health, which is the
  // INSTANTANEOUS report and would keep refusing on a transient error-passive
  // blip long after jugglebot_commands_allowed() stopped doing so. One
  // enforcement point — see classify_command_gate in can_buses.h. The fault /
  // E-STOP terms below are this gate's own and are unchanged.
  if (!jugglebot_commands_allowed()) return false;
  if (fault_can_bus_down()) return false;
  if (fault_guard_mode() == JbUdp::GuardMode::ESTOP) return false;
  return true;
}

// Build the target-axis bitmask: AXIS_ALL → every present axis (six legs AND the
// hand, bit HAND_AXIS); a single axis → that axis iff present. Returns 0 if there
// is no valid target. Absent-tolerant by construction (leg_present is the single
// presence predicate for all NUM_AXES): a hand-less bench still activates its legs,
// and a leg-less hand bench still parks the hand.
static uint8_t resolve_targets(uint8_t axis) {
  uint8_t mask = 0;
  if (axis == JbUdp::RpcArgs::AXIS_ALL) {
    for (uint8_t i = 0; i < NUM_AXES; ++i) if (leg_present(i)) mask |= (uint8_t)(1u << i);
  } else if (axis < NUM_AXES && leg_present(axis)) {
    mask = (uint8_t)(1u << axis);
  }
  return mask;
}

// ── The abort paths (FW 53 U3; see the header comment) ──────────────────────
// End the ladder, sending NOTHING: every axis still in s_targets gets `result`,
// and a HAND_MOVE_TO in flight is answered with `hm_status` / `hm_outcome`.
static void end_ladder(uint8_t result,
                       uint16_t hm_status = JbUdp::RpcStatus::ERR_REJECTED,
                       uint8_t hm_outcome = HAND_MOVE_ABORTED) {
  for (uint8_t i = 0; i < NUM_AXES; ++i)
    if (s_targets & (uint8_t)(1u << i)) s_result[i] = result;
  hm_finish(hm_status, hm_outcome);
  s_phase = APhase::IDLE;
  s_targets = 0;
  s_done = 0;
  s_ret_unsent = 0;
}

// The Hand is IDLEd on every abort (exempt from I-FW-19: the gate passes it
// through) and leaves the ladder FAILED. A refused frame is not retried, as in FW 52.
static void idle_hand_if_targeted() {
  constexpr uint8_t HB = (uint8_t)(1u << HAND_AXIS);
  if (!(s_targets & HB)) return;
  (void)odrive_set_state_tx(HAND_AXIS, ODriveState::IDLE, TxCls::OTHER, Who::LADDER);
  s_result[HAND_AXIS] = ACTIVATE_FAILED;
  s_targets = (uint8_t)(s_targets & (uint8_t)~HB);
}

// Ask the gate to IDLE every target Leg: sent only where Idle is permitted (the
// Platform at Stow, or the bench); a refusal sends nothing and sets HELD.
static void gated_idle_legs() {
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (s_targets & (uint8_t)(1u << i))
      (void)odrive_set_state_tx(i, ODriveState::IDLE, TxCls::OTHER, Who::LADDER);
}

// ABORT_WAIT, one tick: nothing has been commanded to move. Wait (bounded) while
// the only refusals are unconfirmed CLOSED_LOOP sends, then ask for the IDLEs.
static void abort_wait_step(uint64_t now) {
  if (fault_can_bus_down()) { end_ladder(ACTIVATE_FAILED); return; }   // nothing to a dead bus
  if (leg_abort_idle_verdict(now, s_t_abort_us) == AbortIdle::WAIT) return;
  gated_idle_legs();
  end_ladder(ACTIVATE_FAILED);
}

// RETURN: (re-)send the Stow target to every Leg whose frame has not reached the
// wire. The DEACTIVATE COMMAND frame byte for byte (position-only; vel_ff =
// tor_ff = 0; encode_leg_setpoint clips to [0, max] and applies leg_sign). The
// ODrive TRAP_TRAJ planner re-plans from its own setpoint (position AND velocity,
// so no step): a Leg rising at 2.5 rev/s decelerates through zero at
// TRAP_ACC_LIMIT_RPS2 (0.104 rev higher, 0.083 s later), then descends at the
// gentle cruise and decelerates into Stow. Because TRAP_ACC_LIMIT_RPS2 >=
// TRAP_DEC_LIMIT_RPS2, the reversal apex never passes the Active target the rise
// was already planned to stop at (logbook 2026-11-25-fw53-u3-activate-returns-to-stow).
static_assert(ODriveDefaults::TRAP_ACC_LIMIT_RPS2 >= ODriveDefaults::TRAP_DEC_LIMIT_RPS2,
              "RETURN apex bound: the reversal apex stays at or below the Active target "
              "only while TRAP_ACC_LIMIT_RPS2 >= TRAP_DEC_LIMIT_RPS2");
static void send_return_targets() {
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    const uint8_t bit = (uint8_t)(1u << i);
    if (!(s_ret_unsent & bit)) continue;
    if (can_jugglebot_send(ODrive::encode_leg_setpoint(i, STOW_OFF_POSE_REV, 0.0f, 0.0f)))
      s_ret_unsent = (uint8_t)(s_ret_unsent & (uint8_t)~bit);
  }
}

// RETURN, one tick: the descent monitor shared with DEACTIVATE (leg_stow_gate.h).
// The freeze segment (a timeout or a disarmed Leg during the return) is the hull of
// the Leg's seed, its Active target and Stow: every point the rise and the reversal
// can reach. The ACTIVATE is FAILED in every outcome; the IDLEs at Stow clear HELD.
static void return_step(uint64_t now) {
  const bool timed_out = now - s_t_abort_us > A_TIMEOUT_US;
  const DescentStep r = stow_descent_step((uint8_t)(s_targets & LEG_BITS), s_done,
                                          s_seed, s_leg_target, timed_out, now);
  if (r != DescentStep::RUNNING) { end_ladder(ACTIVATE_FAILED); return; }
  send_return_targets();
}

// A Soft E-stop, commands-not-allowed, a timeout, an errored target, a refused
// SETUP frame (anything but bus down and a disarmed Leg). Phase-dependent: see the
// header comment.
static void abort_ladder(uint64_t now,
                         uint16_t hm_status = JbUdp::RpcStatus::ERR_REJECTED,
                         uint8_t hm_outcome = HAND_MOVE_ABORTED) {
  idle_hand_if_targeted();
  if ((s_targets & LEG_BITS) == 0) { end_ladder(ACTIVATE_FAILED, hm_status, hm_outcome); return; }
  if (s_phase == APhase::MONITOR) {
    if (leg_idle_permitted(now)) { gated_idle_legs(); end_ladder(ACTIVATE_FAILED); return; }
    s_done = 0;
    s_ret_unsent = (uint8_t)(s_targets & LEG_BITS);
    s_t_abort_us = now;
    s_phase = APhase::RETURN;
    send_return_targets();
    return;
  }
  s_t_abort_us = now;
  s_phase = APhase::ABORT_WAIT;
  abort_wait_step(now);
}

// FW 53 U8: COMMAND's Closed-loop confirmation failed or expired (legs_cl_confirm):
// nothing has been commanded to move. The pre-rise abort (Hand IDLE, ABORT_WAIT),
// with ABORT_WAIT's clock at the last target Leg's CLOSED_LOOP command, the start
// of the window that just ended, so an expiry is not given a second window.
static void abort_unconfirmed(uint64_t now) {
  idle_hand_if_targeted();
  s_t_abort_us = legs_last_cl_cmd_us((uint8_t)(s_targets & LEG_BITS));
  s_phase = APhase::ABORT_WAIT;
  abort_wait_step(now);
}

// Bus down: the Hand IDLE is still asked (exempt; a confirmed-dead bus refuses it
// anyway), the Legs get nothing.
static void abort_bus_down(uint16_t hm_status, uint8_t hm_outcome) {
  idle_hand_if_targeted();
  end_ladder(ACTIVATE_FAILED, hm_status, hm_outcome);
}

// Target Legs that disarmed themselves mid-rise (CL-confirmed heartbeat, not CL).
static uint8_t dropped_legs(uint64_t now) {
  uint8_t m = 0;
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if ((s_targets & (uint8_t)(1u << i)) && leg_dropped_out(i, now)) m |= (uint8_t)(1u << i);
  return m;
}

// A Leg dropped out mid-rise: never move the Platform with one Leg free. Freeze
// every other target Leg at its stopping point on its own commanded segment
// [seed, target] (HELD); the dropped Leg is left alone.
static void abort_dropped(uint8_t dropped) {
  idle_hand_if_targeted();
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if ((s_targets & (uint8_t)(1u << i)) && !(dropped & (uint8_t)(1u << i)))
      (void)leg_freeze_tx(i, s_seed[i], s_leg_target[i]);
  end_ladder(ACTIVATE_FAILED);
}

void activate_init() {
  s_start_req = false;
  s_stall_timing = false;
  for (uint8_t i = 0; i < NUM_AXES; ++i) s_stalled[i] = false;
  s_lane_timing = false;
  s_lane_t0_us = 0;
  s_lane_last_iq_us = 0;
  s_lane_idle_pending = false;
  s_lane_stalled = false;
  s_hm_req = false;
  s_mode = AMode::ACTIVATE;
  s_hm_box_head = 0;
  s_hm_box_count = 0;
  s_hm_box_drops = 0;
  s_phase = APhase::IDLE;
  s_targets = 0;
  s_done = 0;
  s_ret_unsent = 0;
  s_t_abort_us = 0;
  for (uint8_t i = 0; i < NUM_LEGS; ++i) s_seed[i] = 0.0f;
  for (uint8_t i = 0; i < NUM_AXES; ++i) s_result[i] = ACTIVATE_NONE;
  for (uint8_t i = 0; i < HAND_AXIS; ++i) {
    s_start_leg_target[i] = JBOp::ACTIVATE_POSITION_REVS[i];
    s_leg_target[i]       = JBOp::ACTIVATE_POSITION_REVS[i];
  }
}

// The gates and the latch shared by ACTIVATE and ACTIVATE_TO (one copy, so the two
// can never diverge on when a rise may start). `leg_targets` is HAND_AXIS floats.
static uint16_t activate_gate_and_latch(uint8_t axis, const float* leg_targets) {
  using namespace JbUdp;
  if (!activate_allowed()) return RpcStatus::ERR_BUS_DOWN;
  if (homing_active() || deactivate_active()) return RpcStatus::ERR_REJECTED;  // no concurrent cold-start moves
  if (fault_stow_pending()) return RpcStatus::ERR_REJECTED;   // not during a deferred stow (adversarial-review fix: else the stow's virtual-complete IDLEs the legs)
  // MPC-stream interlock: reject while the MPC is actively driving
  // the legs (guard ENABLED on the Jetson) — a cold-start move must not co-drive the
  // same ODrives the 500 Hz stream commands (the interp ISR reciprocally suppresses
  // its TX during this move).
  if (fault_mpc_active()) return RpcStatus::ERR_REJECTED;
  if (resolve_targets(axis) == 0) return RpcStatus::ERR_BAD_ARGS;  // no present target leg
  // Reject if an activate is active or a start is already pending (idempotent).
  // IRQ-guarded so (busy-check + latch) is atomic vs the activate task that
  // consumes s_start_req at the top of activate_step.
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool busy = (s_phase != APhase::IDLE) || s_start_req || s_hm_req;
  if (!busy) {
    for (uint8_t i = 0; i < HAND_AXIS; ++i) s_start_leg_target[i] = leg_targets[i];
    s_start_axis = axis;
    s_start_req = true;
  }
  __set_PRIMASK(pm);
  if (busy) return RpcStatus::ERR_REJECTED;
  // Accepted: the targets' HELD bits restart from this request (fw53_design.md 3).
  const uint8_t targets = resolve_targets(axis);
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (targets & (uint8_t)(1u << i)) leg_held_clear(i);
  return RpcStatus::OK;
}

uint16_t activate_request(uint8_t axis) {
  return activate_gate_and_latch(axis, JBOp::ACTIVATE_POSITION_REVS);
}

bool activate_to_target_in_band(uint8_t leg, float target_rev) {
  if (leg >= HAND_AXIS || !std::isfinite(target_rev)) return false;
  return fabsf(target_rev - JBOp::ACTIVATE_POSITION_REVS[leg]) <= ACTIVATE_TO_BAND_REV;
}

uint16_t activate_to_request(const float leg_targets[HAND_AXIS]) {
  using namespace JbUdp;
  // Args first, pure validation independent of machine state (HAND_MOVE_TO's
  // order). A target outside the band is REFUSED, never clipped: the band is the
  // whole of what the host may ask, and a caller outside it has a bug a clip hides.
  for (uint8_t i = 0; i < HAND_AXIS; ++i)
    if (!activate_to_target_in_band(i, leg_targets[i])) return RpcStatus::ERR_BAD_ARGS;
  // All six Legs, always: a Levelled pose is a whole-Platform pose, and AXIS_ALL
  // on a board with a Leg absent would raise the others to a tilt nobody asked for.
  for (uint8_t i = 0; i < HAND_AXIS; ++i)
    if (!leg_present(i)) return RpcStatus::ERR_BAD_ARGS;
  return activate_gate_and_latch(RpcArgs::AXIS_ALL, leg_targets);
}

bool activate_active() { return s_phase != APhase::IDLE || s_start_req || s_hm_req; }

uint16_t hand_move_request(uint8_t axis, float target_rev, float vel_rps, uint16_t req_id) {
  using namespace JbUdp;
  // Args first — pure validation, independent of machine state. Axis 6 ONLY:
  // there is no per-leg move RPC and this must not become one (a coupled
  // platform moved one leg at a time tilts and binds). A bad target is REFUSED,
  // never clipped into range: a caller asking for an out-of-stroke position has
  // a bug that a silent clip would hide.
  if (axis != HAND_AXIS) return RpcStatus::ERR_BAD_ARGS;
  if (!std::isfinite(target_rev) || target_rev < 0.0f ||
      target_rev > HAND_MOTOR_MAX_POSITION) return RpcStatus::ERR_BAD_ARGS;
  if (!std::isfinite(vel_rps) || !(vel_rps > 0.0f) ||
      vel_rps > JBOp::GENTLE_MOVE_VEL_LIMIT_RPS) return RpcStatus::ERR_BAD_ARGS;
  if (!leg_present(HAND_AXIS)) return RpcStatus::ERR_BAD_ARGS;   // ACTIVATE parity (no present target)
  // The SAME gates as activate_request, in the same order: no motion on a dead
  // bus or under the latched E-STOP (ERR_BUS_DOWN), none beside another
  // cold-start move, a deferred stow or the live stream (ERR_REJECTED).
  if (!activate_allowed()) return RpcStatus::ERR_BUS_DOWN;
  if (homing_active() || deactivate_active()) return RpcStatus::ERR_REJECTED;
  if (fault_stow_pending()) return RpcStatus::ERR_REJECTED;
  if (fault_mpc_active()) return RpcStatus::ERR_REJECTED;
  // An errored hand is refused up front with no frame sent (ACTIVATE finds the
  // same in SETUP and IDLEs the axis; the recovery clears errors first, so an
  // error here is a caller sequencing bug worth a loud, motion-free answer).
  if (axes[HAND_AXIS].active_errors != 0) return RpcStatus::ERR_REJECTED;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool busy = s_hm_req || s_start_req ||
                    (s_phase != APhase::IDLE && s_mode != AMode::HAND_MOVE);
  if (!busy) {
    s_hm_req_target = target_rev;
    s_hm_req_vel    = vel_rps;
    s_hm_req_id     = req_id;
    s_hm_req_t_us   = micros64();
    s_hm_req        = true;
  }
  __set_PRIMASK(pm);
  return busy ? RpcStatus::ERR_REJECTED : RpcStatus::OK;
}

bool hand_move_active() {
  return s_hm_req || (s_phase != APhase::IDLE && s_mode == AMode::HAND_MOVE);
}

bool hand_move_pop_reply(HandMoveReply& out) {
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool have = s_hm_box_count > 0;
  if (have) {
    out = s_hm_box[s_hm_box_head];
    s_hm_box_head  = (uint8_t)((s_hm_box_head + 1) % HM_MAILBOX_CAP);
    s_hm_box_count = (uint8_t)(s_hm_box_count - 1);
  }
  __set_PRIMASK(pm);
  return have;
}

uint32_t hand_move_reply_drops() { return s_hm_box_drops; }

// Consume a latched HAND_MOVE_TO (activate-task context): a START from IDLE, or a
// RETARGET of the HAND_MOVE_TO in flight. Runs before the ACTIVATE consume so a
// start owns the ladder this tick.
static void hm_consume() {
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool     req   = s_hm_req;
  const float    tgt   = s_hm_req_target;
  const float    vel   = s_hm_req_vel;
  const uint16_t id    = s_hm_req_id;
  const uint64_t t_acc = s_hm_req_t_us;
  const bool     fresh = req && s_phase == APhase::IDLE;
  s_hm_req = false;
  if (fresh) { s_mode = AMode::HAND_MOVE; s_phase = APhase::SETUP; }  // owner before phase
  __set_PRIMASK(pm);
  if (!req) return;

  if (fresh) {
    s_hm_target = tgt; s_hm_vel = vel; s_hm_id = id; s_hm_t_accept_us = t_acc;
    s_targets = resolve_targets(HAND_AXIS);
    if (s_targets == 0) {   // hand vanished between request and consume
      hm_post(id, JbUdp::RpcStatus::ERR_BAD_ARGS, HAND_MOVE_ABORTED, tgt, t_acc);
      s_phase = APhase::IDLE;
      return;
    }
    s_result[HAND_AXIS] = ACTIVATE_RUNNING;
    stall_reset_for_targets();
    s_setup_idx = 0;
    s_t_start_us = micros64();
    s_t_phase_us = s_t_start_us;
    return;
  }
  if (s_mode != AMode::HAND_MOVE) {
    // Unreachable (hand_move_request refuses while an ACTIVATE owns the ladder);
    // never touch a running ACTIVATE — answer the request instead.
    hm_post(id, JbUdp::RpcStatus::ERR_REJECTED, HAND_MOVE_ABORTED, tgt, t_acc);
    return;
  }
  // RETARGET. Answer the superseded request first, then adopt the new one with a
  // fresh timeout clock. If the hand is already configured, re-send the cruise;
  // if the old target was already commanded, command the new one now — the
  // ODrive TRAP_TRAJ planner replans from the current pos/vel (profiled, no step).
  hm_post(s_hm_id, JbUdp::RpcStatus::OK, HAND_MOVE_SUPERSEDED, s_hm_target, s_hm_t_accept_us);
  s_hm_target = tgt; s_hm_vel = vel; s_hm_id = id; s_hm_t_accept_us = t_acc;
  s_t_start_us = micros64();
  const bool configured = !(s_phase == APhase::SETUP && s_setup_idx <= HAND_AXIS);
  if (configured &&
      !can_jugglebot_send(ODrive::encode_set_traj_vel_limit(HAND_AXIS, s_hm_vel))) {
    abort_ladder(micros64(), JbUdp::RpcStatus::ERR_TIMEOUT, HAND_MOVE_ABORTED);   // the Hand alone
    return;
  }
  if (s_phase == APhase::MONITOR) command_targets();   // s_targets == the hand alone
}

uint8_t activate_result(uint8_t axis) {
  return (axis < NUM_AXES) ? s_result[axis] : (uint8_t)ACTIVATE_NONE;
}

bool activate_stalled(uint8_t axis) { return axis < NUM_AXES && s_stalled[axis]; }

// The owner's rule (2026-11-02): no motion with high current for >= 300 ms aborts
// the move. Runs every active tick, in every phase, while the hand is a target;
// thresholds and their physics in canbridge_config.h. Only the HAND is guarded and
// only the hand is stopped: the six legs of an ACTIVATE are a loaded Platform, and
// IDLEing them for a Hand fault would drop it onto the stops (leg_activate.h).
// Returns true when the guard ended the ladder (the hand was its only target).
static bool hand_stall_guard(uint64_t now) {
  constexpr uint8_t HB = (uint8_t)(1u << HAND_AXIS);
  if (!(s_targets & HB)) { s_stall_timing = false; return false; }
  const float iq  = axes[HAND_AXIS].iq_measured;   // single-word atomic reads
  const float vel = axes[HAND_AXIS].vel_rps;
  // A NaN fails both comparisons, so it can never read as a stall.
  const bool stalled_now = fabsf(iq) >= HAND_STALL_IQ_A && fabsf(vel) <= HAND_STALL_VEL_RPS;
  if (!stalled_now) { s_stall_timing = false; return false; }
  if (!s_stall_timing) { s_stall_timing = true; s_stall_t0_us = now; return false; }
  if (now - s_stall_t0_us < HAND_STALL_HOLD_US) return false;
  // Window expired: stop pushing. A refused TX keeps the hand targeted and the
  // window expired, so the IDLE is re-sent next tick (the bus-down and timeout
  // aborts above still bound the ladder).
  if (odrive_set_state_tx(HAND_AXIS, ODriveState::IDLE, TxCls::OTHER, Who::LADDER) != DeResult::SENT)
    return false;
  s_result[HAND_AXIS]  = ACTIVATE_FAILED;
  s_stalled[HAND_AXIS] = true;
  s_stall_timing = false;
  s_targets = (uint8_t)(s_targets & (uint8_t)~HB);
  hm_finish(JbUdp::RpcStatus::ERR_REJECTED, HAND_MOVE_STALLED);   // HAND_MOVE_TO only
  if (s_targets == 0) { s_phase = APhase::IDLE; return true; }
  return false;                     // an ACTIVATE's legs carry on
}

bool hand_lane_stalled(uint8_t axis) { return axis == HAND_AXIS && s_lane_stalled; }

// The owner's rule (2026-11-20, BUG-123): 40 A / 1 rev/s / 150 ms, then IDLE.
// Runs every tick BEFORE the ladder; armed whenever the ladder does not target
// the Hand (the ladder's own guard above owns a Hand it moves). Only the Hand is
// stopped here: the Legs are held by the guard ESTOP the latch raises, exactly as
// for a Hand ODrive fault (ODRIVE_FATAL), never IDLEd (a loaded Platform).
// FW 49 (BUG-131 item 2): samples count only while the Hand may be in
// CLOSED_LOOP (see the predicate below).
static void hand_lane_stall_guard() {
  // A trip whose IDLE frame the TX path refused: re-send every tick until queued.
  if (s_lane_idle_pending &&
      odrive_set_state_tx(HAND_AXIS, ODriveState::IDLE, TxCls::OTHER, Who::LADDER) == DeResult::SENT)
    s_lane_idle_pending = false;
  constexpr uint8_t HB = (uint8_t)(1u << HAND_AXIS);
  if (s_phase != APhase::IDLE && (s_targets & HB)) { s_lane_timing = false; return; }
  // One evaluation per NEW sample, timed on the sample stamps: a tick with no new
  // sample neither advances the window nor restarts it, so stale data never trips.
  const uint64_t ts = atomic_read_u64(&axes[HAND_AXIS].iq_timestamp_us);
  if (ts == 0 || ts == s_lane_last_iq_us) return;
  const uint64_t prev = s_lane_last_iq_us;
  s_lane_last_iq_us = ts;
  // A hole in the sample stream (or the first sample after the ladder let go)
  // restarts the window: "continuously" is not known across it.
  if (prev == 0 || ts < prev || ts - prev > HAND_LANE_STALL_GAP_US) s_lane_timing = false;
  const float iq  = axes[HAND_AXIS].iq_measured;   // single-word atomic reads
  const float vel = axes[HAND_AXIS].vel_rps;
  const uint64_t vts = atomic_read_u64(&axes[HAND_AXIS].pos_timestamp_us);
  const bool vel_fresh = vts != 0 && (vts >= ts || ts - vts <= HAND_LANE_STALL_GAP_US);
  // BUG-131 item 2 (FW 49): only a Hand that may be in CLOSED_LOOP counts. An
  // IDLE ODrive keeps broadcasting the LAST Iq (the FOC loop that updates it is
  // stopped) with fresh frames, so after any IDLE taken at >= 40 A (this guard's
  // own trip, the ladder's, the E-stop button) "40 A and not moving" held forever
  // and the guard re-latched ~150 ms after every CLEAR_ERRORS (bench 2026-11-23).
  // A sample is excluded only when a FRESH heartbeat (no older than
  // CAN_HEARTBEAT_STALE_US at this sample) positively reports another state. A
  // Hand whose heartbeat was never seen or has gone stale is UNKNOWN and still
  // counts: a stalled Hand whose sparse 10 Hz heartbeat is lost is still pushing,
  // and FW 48's predicate is what applies to it.
  const uint64_t hbs = atomic_read_u64(&axes[HAND_AXIS].last_heartbeat_us);
  const bool hb_fresh = axes[HAND_AXIS].heartbeat_seen && hbs != 0 &&
                        (hbs >= ts || ts - hbs <= CAN_HEARTBEAT_STALE_US);
  const bool may_be_closed_loop =
      !hb_fresh || axes[HAND_AXIS].axis_state == (uint8_t)ODriveState::CLOSED_LOOP;
  // A NaN fails both comparisons, so it can never read as a stall.
  const bool stalled_now = may_be_closed_loop && vel_fresh &&
                           fabsf(iq) >= HAND_LANE_STALL_IQ_A &&
                           fabsf(vel) <= HAND_LANE_STALL_VEL_RPS;
  if (!stalled_now) { s_lane_timing = false; return; }
  if (!s_lane_timing) { s_lane_timing = true; s_lane_t0_us = ts; return; }
  if (ts - s_lane_t0_us < HAND_LANE_STALL_HOLD_US) return;
  // 150 ms of samples: stop pushing. Latch first, so the Legs are held and every
  // re-energising op is refused even if the IDLE frame has to wait for the bus.
  s_lane_timing  = false;
  s_lane_stalled = true;
  fault_notify_hand_stall();
  if (odrive_set_state_tx(HAND_AXIS, ODriveState::IDLE, TxCls::OTHER, Who::LADDER) != DeResult::SENT)
    s_lane_idle_pending = true;
}

void activate_step() {
  // ── FW 48 (BUG-123): the Hand stall guard outside the ladder. ──
  hand_lane_stall_guard();

  // ── HAND_MOVE_TO start / retarget (net task → activate task). ──
  hm_consume();

  // ── Consume a pending start request (net task → activate task). ──
  if (s_phase == APhase::IDLE) {
    const uint32_t pm = __get_PRIMASK(); __disable_irq();
    const bool    start = s_start_req;
    const uint8_t ax    = s_start_axis;
    s_start_req = false;
    if (start) {
      s_mode = AMode::ACTIVATE; s_phase = APhase::SETUP;
      for (uint8_t i = 0; i < HAND_AXIS; ++i) s_leg_target[i] = s_start_leg_target[i];
    }
    __set_PRIMASK(pm);
    if (!start) return;
    s_targets = resolve_targets(ax);
    if (s_targets == 0) { s_phase = APhase::IDLE; return; }  // legs vanished between request and consume
    for (uint8_t i = 0; i < NUM_AXES; ++i)
      if (s_targets & (uint8_t)(1u << i)) s_result[i] = ACTIVATE_RUNNING;
    stall_reset_for_targets();
    s_setup_idx = 0;   // SETUP configures one leg per tick starting here
    s_t_start_us = micros64();
    s_t_phase_us = s_t_start_us;
    // s_phase already set to SETUP inside the guard; fall through and run it.
  }

  const uint64_t now = micros64();

  // ── FW 53 U3: after an abort, its own phase owns the tick. ──
  if (s_phase == APhase::ABORT_WAIT) { abort_wait_step(now); return; }
  if (s_phase == APhase::RETURN)     { return_step(now); return; }

  // ── Aborts that apply in SETUP / COMMAND / MONITOR (checked before any drive). ──
  if (fault_can_bus_down()) {
    abort_bus_down(JbUdp::RpcStatus::ERR_BUS_DOWN, HAND_MOVE_ABORTED); return;
  }
  if (s_phase == APhase::MONITOR) {
    const uint8_t dropped = dropped_legs(now);
    if (dropped) { abort_dropped(dropped); return; }
  }
  if (!activate_allowed()) {
    abort_ladder(now, JbUdp::RpcStatus::ERR_BUS_DOWN, HAND_MOVE_ABORTED); return;
  }
  if (now - s_t_start_us > A_TIMEOUT_US) {
    abort_ladder(now, JbUdp::RpcStatus::ERR_TIMEOUT, HAND_MOVE_TIMEOUT); return;
  }
  if (hand_stall_guard(now)) return;   // FW 36: a stalled hand is IDLEd and dropped

  switch (s_phase) {
    case APhase::SETUP: {
      // Configure ONE leg per tick (s_setup_idx cursor). NEVER burst all legs'
      // config frames into a single tick: 5 frames × 6 legs = 30 overflowed the
      // 16-deep CAN3 TX buffer and most legs never entered CLOSED_LOOP (only the
      // last did) — 2026-06-26. One leg/tick mirrors the proven sequential homing
      // pattern (~few frames/tick). The COMMAND phase still fires all targets in
      // one tick AFTER every leg is configured + holding, so the physical move is
      // parallel (even platform rise).
      while (s_setup_idx < NUM_AXES && !(s_targets & (uint8_t)(1u << s_setup_idx)))
        ++s_setup_idx;
      if (s_setup_idx >= NUM_AXES) {
        // Every target leg configured + in CLOSED_LOOP/TRAP_TRAJ holding its seed.
        s_t_phase_us = now;          // start the settle clock for COMMAND
        s_phase = APhase::COMMAND;
        break;
      }
      const uint8_t i = s_setup_idx;
      // Synchronous per-leg error gate (legacy can_node._gentle_move_steps parity:
      // "refuse to enter CLOSED_LOOP if errors present"). ANY errored target fails
      // the whole op (a coupled platform must not activate a subset — that tilts it).
      if (axes[i].active_errors != 0) { abort_ladder(now); return; }
      // Seed the ACTUAL current position (NON-clipped) so CLOSED_LOOP holds in
      // place even at the sub-zero hardstop — the whole move is then TRAP_TRAJ-
      // profiled with no clip-snap off the hardstop. leg_sign applies the ODrive
      // convention (Jugglebot positive=extension → ODrive negative).
      const float cur = axes[i].pos_rev;  // single-word atomic read, Jugglebot conv
      if (i < NUM_LEGS) s_seed[i] = cur;  // one end of this leg's freeze segment
      bool ok = true;
      ok = can_jugglebot_send(
               ODrive::encode_set_input_pos(i, ODrive::leg_sign(i, cur), 0, 0)) && ok;
      // TRAP_TRAJ speed ceiling, axis-agnostic on purpose. JBOp::GENTLE_MOVE_VEL_LIMIT_RPS
      // is the op's OWN constant (not a LEG_* number), and it is the right one for
      // axis 6 too: 2.5 rev/s × Geometry::HAND_MM_PER_REV ≈ 81 mm/s of carriage travel,
      // so even a re-activate from catch-prime (9.96 rev) is a 4 s controlled walk
      // inside the 10 s GENTLE_MOVE_TIMEOUT_S rather than a blind lunge. The
      // streaming lane's 200-300 rev/s ceilings are for a planned, guarded throw;
      // a cold-start park must never inherit them. (There is no generated hand
      // trap-traj limit; if one is ever added this is its single consumer.)
      ok = can_jugglebot_send(
               ODrive::encode_set_traj_vel_limit(i, activate_traj_vel_rps())) && ok;
      ok = can_jugglebot_send(
               ODrive::encode_set_traj_acc_limits(
                   i, ODriveDefaults::TRAP_ACC_LIMIT_RPS2,
                   ODriveDefaults::TRAP_DEC_LIMIT_RPS2)) && ok;
      ok = can_jugglebot_send(
               ODrive::encode_set_controller_mode(
                   i, ODriveControlMode::POSITION, ODriveInputMode::TRAP_TRAJ)) && ok;
      // Through the gate (not gated: CLOSED_LOOP is no de-energise), which stamps
      // the Leg's CL-command time for the Stow predicate and leg_dropped_out.
      ok = (odrive_set_state_tx(i, ODriveState::CLOSED_LOOP, TxCls::OTHER, Who::LADDER)
                == DeResult::SENT) && ok;
      if (!ok) { abort_ladder(now, JbUdp::RpcStatus::ERR_TIMEOUT, HAND_MOVE_ABORTED); return; }
      axes[i].controller_mode = (uint8_t)ODriveControlMode::POSITION;   // commanded-mode record
      axes[i].input_mode      = (uint8_t)ODriveInputMode::TRAP_TRAJ;    // (axis_state.h)
      ++s_setup_idx;   // next target leg on the next tick
      break;
    }
    case APhase::COMMAND: {
      // The fixed settle stays: it is the Hand's only engage settle (the Hand is
      // configured last and is not in the confirmation) and the whole gate of a
      // HAND_MOVE_TO; for the Legs the confirmation below subsumes it.
      if (now - s_t_phase_us < CL_ENGAGE_SETTLE_US) break;
      // FW 53 U8 (owner: no Leg is raised while another is free): no target to ANY
      // axis until every target Leg's post-CL heartbeat reports CLOSED_LOOP with no
      // error. A Leg with an error, or no confirmation within
      // CL_CONFIRM_WAIT_US of the last CLOSED_LOOP command: the pre-rise abort.
      const ClConfirm cc = legs_cl_confirm((uint8_t)(s_targets & LEG_BITS), now);
      if (cc == ClConfirm::WAIT) break;
      if (cc != ClConfirm::CONFIRMED) { abort_unconfirmed(now); return; }
      command_targets();
      s_phase = APhase::MONITOR;  // stay active through the physical move
      break;
    }
    case APhase::MONITOR: {
      // Wait until every target leg reaches the active pose AND settles. The
      // start position is far from the target, so the position check prevents a
      // false "done" from the leg merely sitting still at the start. The leg ramps
      // up far from target (|pos−target| large), so it cannot satisfy both until
      // the move has actually completed. Timeout (above) bounds a stall.
      bool all_done = true;
      for (uint8_t i = 0; i < NUM_AXES; ++i) {
        if (!(s_targets & (uint8_t)(1u << i))) continue;
        const float pos = axes[i].pos_rev;  // Jugglebot conv (== activate_target_rev conv)
        const float vel = axes[i].vel_rps;
        if (fabsf(pos - activate_target_rev(i)) > JBOp::TARGET_REACHED_POS_TOL_REV ||
            fabsf(vel) > JBOp::TARGET_REACHED_VEL_TOL_RPS) {
          all_done = false;
        }
      }
      if (all_done) {
        // ── Hand hand-off to the streamed lane (skill-stack R1) ───────────────
        // The hand is the ONE axis whose next commander is the 2 ms interp tick,
        // and that lane commands in POSITION/PASSTHROUGH. Leaving axis 6 in
        // TRAP_TRAJ is the FW 18 fault class in a new dress: the ODrive would
        // treat every streamed set_input_pos as a fresh trajectory target and
        // the lane would move the hand late and softly with nothing in telemetry
        // saying why. So ACTIVATE hands over explicitly, here, after the park has
        // ARRIVED — input_pos is already the parked target, so the mode change is
        // motion-free (pos_setpoint == input_pos == HAND_ACTIVATE_POSITION_REV).
        // The legs stay in TRAP_TRAJ holding the active pose exactly as before;
        // the host's post-activate _run_configure is still their hand-off.
        if (s_targets & (uint8_t)(1u << HAND_AXIS)) {
          if (!can_jugglebot_send(ODrive::encode_set_controller_mode(
                  HAND_AXIS, ODriveControlMode::POSITION,
                  ODriveInputMode::PASSTHROUGH))) {
            // A hand left in TRAP_TRAJ is a silently-inert lane: fail it loudly.
            // FW 53: ONLY the Hand is IDLEd. The Legs are at their target in
            // TRAP_TRAJ holding the Active pose and get nothing; they are OK.
            idle_hand_if_targeted();
            for (uint8_t i = 0; i < NUM_LEGS; ++i)
              if (s_targets & (uint8_t)(1u << i)) s_result[i] = ACTIVATE_OK;
            hm_finish(JbUdp::RpcStatus::ERR_TIMEOUT, HAND_MOVE_ABORTED);
            s_phase = APhase::IDLE;
            s_targets = 0;
            return;
          }
          axes[HAND_AXIS].input_mode = (uint8_t)ODriveInputMode::PASSTHROUGH;
        }
        for (uint8_t i = 0; i < NUM_AXES; ++i)
          if (s_targets & (uint8_t)(1u << i)) s_result[i] = ACTIVATE_OK;
        hm_finish(JbUdp::RpcStatus::OK, HAND_MOVE_ARRIVED);   // HAND_MOVE_TO: arrived + handed off
        s_phase = APhase::IDLE;
        s_targets = 0;
      }
      break;
    }
    default:
      abort_ladder(now);
      break;
  }
}

}  // namespace CanBridge
