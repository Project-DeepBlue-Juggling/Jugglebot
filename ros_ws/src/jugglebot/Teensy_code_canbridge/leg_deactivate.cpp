// =============================================================================
//  leg_deactivate.cpp — firmware deactivate (TRAP_TRAJ → STOW → IDLE)
// =============================================================================
//  See leg_deactivate.h for the design rationale. The ladder, per targeted leg:
//    SETUP   : refuse if any target leg has active_errors (legacy _gentle_move
//              parity) → seed input_pos(actual current, NON-clipped) → traj
//              vel/acc limits → POSITION/TRAP_TRAJ mode → CLOSED_LOOP
//    settle  : let CLOSED_LOOP engage + mode/limits take effect
//    COMMAND : set_input_pos(STOW_OFF_POSE_REV) → ODrive TRAP_TRAJ descends
//    MONITOR : stay active until every target leg reaches STOW + settles
//              (|pos−stow| ≤ tol AND |vel| ≤ tol) → IDLE the leg → mark OK.
//  The ODrive owns the trajectory; the Jetson observes the CLOSED_LOOP → IDLE
//  completion via telemetry. The IDLE-on-arrival is the one difference from
//  ACTIVATE (which holds the active pose in CLOSED_LOOP) — deactivate safes the
//  robot, ending de-energised. Staying active through MONITOR is what makes
//  HOME/ACTIVATE/DEACTIVATE mutually exclusive for the WHOLE physical move.
//
//  FW 53 (I-FW-19, owner rule 2026-11-25: no Leg is made IDLE unless the
//  Platform is already at Stow). Every Leg state change goes through the Stow
//  gate (leg_stow_gate.h), and an abort HOLDS instead of IDLEing (fw53_design.md
//  3.1):
//    * before the descent is commanded (SETUP, COMMAND): bus down → end FAILED,
//      no frame; any other abort → ask the gate for IDLE per target Leg: sent
//      only if the Platform is already at Stow, else refused (nothing on CAN,
//      HELD bit set), so configured Legs hold their measured seed and the rest
//      hold their stream pose. FW 53 U3: while the only refusals are Legs whose
//      CLOSED_LOOP no heartbeat has confirmed yet, the abort waits (ABORT_WAIT,
//      at most CL_CONFIRM_WAIT_US, nothing sent) before asking, as ACTIVATE's does;
//    * during the descent (MONITOR, the shared stow_descent_step in
//      leg_stow_gate.cpp, which ACTIVATE's return to Stow also runs), in
//      precedence order: bus down → end FAILED,
//      no frame (the deferred stow takes over at reconnect); a target Leg whose
//      CL-confirmed heartbeat reports anything but CLOSED_LOOP → freeze the
//      other live targets, FAILED/HELD; arrival → gated IDLE (retried next tick
//      if refused); timeout → gated IDLE if at Stow, else freeze, FAILED/HELD.
//      A guard E-STOP or commands-not-allowed does NOT stop the descent
//      (DESCENT_CONTINUES_THROUGH_ESTOP below).
// =============================================================================
#include "leg_deactivate.h"

#include <Arduino.h>          // __get_PRIMASK / __disable_irq / __set_PRIMASK
#include <cmath>              // fabsf
#include "canbridge_config.h" // STOW_OFF_POSE_REV, GENTLE_MOVE_VEL_LIMIT_RPS, CL_ENGAGE_SETTLE_US
#include "protocol_config.h"  // ODriveState, ODriveControlMode, ODriveInputMode
#include "hardware_config.h"  // JBOp::, ODriveDefaults::
#include "udp_protocol.h"     // JbUdp::RpcStatus / BusHealth / GuardMode / AXIS_ALL
#include "odrive_protocol.h"
#include "axis_state.h"
#include "can_buses.h"
#include "fault_machine.h"    // fault_can_bus_down / fault_guard_mode
#include "leg_homing.h"       // homing_active (no concurrent cold-start moves)
#include "leg_activate.h"     // activate_active (no concurrent cold-start moves)
#include "leg_stow_gate.h"    // leg_idle_permitted (bench or Stow), odrive_set_state_tx, leg_freeze_tx (I-FW-19)
#include "time_base.h"        // micros64, atomic_read_u64, rx_older_than

namespace CanBridge {

// ── Sub-phase ladder (mirror leg_activate's APhase) ───────────────────────────
enum class DPhase : uint8_t {
  IDLE,      // nothing running
  SETUP,     // error gate + seed + traj limits + TRAP_TRAJ mode + CLOSED_LOOP issued
  COMMAND,   // settle elapsed → command the STOW target
  MONITOR,   // target commanded → wait for the leg(s) to reach + settle, then IDLE
  ABORT_WAIT,// pre-descent abort waiting (bounded) for CL confirmation before the gated IDLE
};

// The settle before COMMAND is CL_ENGAGE_SETTLE_US (canbridge_config.h, was this
// file's SETTLE_SETUP_US, same 10 ms): the Stow gate's CL-confirmation constant,
// so "CLOSED_LOOP has engaged" means one thing in the ladder and in the gate.
// Hard timeout on the WHOLE op (SETUP → COMMAND → physical descent → settle).
static const uint64_t D_TIMEOUT_US = (uint64_t)(JBOp::GENTLE_MOVE_TIMEOUT_S * 1.0e6f);

// OWNER DECISION 1 (fw53_design.md, open): what a guard E-STOP or
// commands-not-allowed does once the STOW target has been commanded (MONITOR).
//   true  (default): nothing. The descent continues to Stow. Once the target is
//         commanded, stopping is itself a new motion command and the target is
//         the safe state (precedent: the deferred stow in fault_machine.cpp).
//   false: freeze every live target on its commanded segment, FAILED/HELD.
// Flipping it is this one constant; test_leg_deactivate.cpp's "guard ESTOP after
// COMMAND" case pins the default and must be inverted with it.
static constexpr bool DESCENT_CONTINUES_THROUGH_ESTOP = true;

// ── State (owned by the deactivate task; start-request handed off from net task) ─
static volatile bool    s_start_req  = false;   // net task → deactivate task
static volatile uint8_t s_start_axis = 0xFF;

static DPhase   s_phase   = DPhase::IDLE;
static uint8_t  s_targets = 0;        // bitmask of legs being deactivated (bit i = leg i)
static uint8_t  s_done    = 0;        // target legs IDLE'd at arrival (the gate SENT)
static uint8_t  s_setup_idx = 0;      // SETUP cursor — configure ONE leg per tick
static uint64_t s_t_start_us = 0;     // ladder start (overall timeout clock)
static uint64_t s_t_phase_us = 0;     // current sub-phase entry (settle clock)
static uint64_t s_t_abort_us = 0;     // ABORT_WAIT entry (CL_CONFIRM_WAIT_US clock)
static float    s_seed[NUM_LEGS] = { 0.0f };   // measured pos at configure: freeze segment [seed, STOW]
static uint8_t  s_result[NUM_AXES] = { DEACTIVATE_NONE };

// ── Bus / fault gate (mirror leg_activate::activate_allowed). Never drive a
//    confirmed-dead bus or an E-STOP'd system. ───────────────────────────────
static bool deactivate_allowed() {
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
// is no valid target.
static uint8_t resolve_targets(uint8_t axis) {
  uint8_t mask = 0;
  if (axis == JbUdp::RpcArgs::AXIS_ALL) {
    for (uint8_t i = 0; i < NUM_AXES; ++i) if (leg_present(i)) mask |= (uint8_t)(1u << i);
  } else if (axis < NUM_AXES && leg_present(axis)) {
    mask = (uint8_t)(1u << axis);
  }
  return mask;
}

// ── The hand's deactivate: IDLE where it stands, immediately ──────────────────
// The hand does NOT descend with the legs. A leg carries the platform's weight and
// must be lowered under profile or it drops it; the hand's carriage hangs on a
// spool and gravity takes it to the bottom stop on its own the instant the axis
// de-energises — a TRAP_TRAJ descent to 0 rev would be a second, slower path to
// the same physical rest, with the axis powered the whole way down beside six legs
// that are also moving. So: de-energise it FIRST, before the legs start their
// descent, and drop it from the ladder's target mask. If the leg descent later
// aborts, the hand is already in its safe state. (The Hand is exempt from
// I-FW-19: the gate passes it straight through.)
// Returns the mask with HAND_AXIS cleared.
static uint8_t idle_hand_now(uint8_t mask) {
  if (!(mask & (uint8_t)(1u << HAND_AXIS))) return mask;
  (void)odrive_set_state_tx(HAND_AXIS, ODriveState::IDLE, TxCls::OTHER, Who::LADDER);
  s_result[HAND_AXIS] = DEACTIVATE_OK;
  return (uint8_t)(mask & (uint8_t)~(1u << HAND_AXIS));
}

static inline bool is_live(uint8_t i) {
  const uint8_t bit = (uint8_t)(1u << i);
  return (s_targets & bit) && !(s_done & bit);
}

// End the ladder, sending NOTHING. Every target leg not already IDLE'd at arrival
// gets `result` (the hand left s_targets in the first tick with its own result).
static void end_ladder(uint8_t result) {
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (is_live(i)) s_result[i] = result;
  s_phase = DPhase::IDLE;
  s_targets = 0;
  s_done = 0;
}

// Ask the Stow gate to IDLE every live target leg. Sent only while the Platform
// is at Stow; a refusal puts nothing on CAN and sets that leg's HELD bit.
static void gated_idle_live() {
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (is_live(i)) (void)odrive_set_state_tx(i, ODriveState::IDLE, TxCls::OTHER, Who::LADDER);
}

// One tick of the pre-descent abort (ABORT_WAIT). At Stow (e.g. DEACTIVATE of an
// already-stowed Platform) the legs IDLE as in FW 52; while the only refusals are
// Legs whose CLOSED_LOOP is not yet confirmed by a heartbeat, wait (bounded by
// CL_CONFIRM_WAIT_US, nothing sent); otherwise the gate refuses: nothing is sent,
// configured legs hold their measured seed in TRAP_TRAJ, the others hold their
// stream pose, HELD (fw53_design.md 3.1). Bus down ends at once with no frame.
static void abort_wait_step(uint64_t now) {
  if (fault_can_bus_down()) { end_ladder(DEACTIVATE_FAILED); return; }
  if (leg_abort_idle_verdict(now, s_t_abort_us) == AbortIdle::WAIT) return;
  gated_idle_live();
  end_ladder(DEACTIVATE_FAILED);
}

// Abort BEFORE the descent target is commanded (SETUP / COMMAND, not bus down).
static void abort_before_descent(uint64_t now) {
  s_t_abort_us = now;
  s_phase = DPhase::ABORT_WAIT;
  abort_wait_step(now);
}

// The hold DURING the descent, for the owner-decision-1 switch below: freeze every
// live leg on [seed, STOW] (leg_freeze_tx), then end FAILED.
static void freeze_live_and_end() {
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (is_live(i)) (void)leg_freeze_tx(i, s_seed[i], STOW_OFF_POSE_REV);
  end_ladder(DEACTIVATE_FAILED);
}

void deactivate_init() {
  s_start_req = false;
  s_phase = DPhase::IDLE;
  s_targets = 0;
  s_done = 0;
  for (uint8_t i = 0; i < NUM_LEGS; ++i) s_seed[i] = 0.0f;
  for (uint8_t i = 0; i < NUM_AXES; ++i) s_result[i] = DEACTIVATE_NONE;
}

uint16_t deactivate_request(uint8_t axis) {
  using namespace JbUdp;
  if (!deactivate_allowed()) return RpcStatus::ERR_BUS_DOWN;
  // No concurrent cold-start moves (symmetric with activate_request / homing).
  if (homing_active() || activate_active()) return RpcStatus::ERR_REJECTED;
  if (fault_stow_pending()) return RpcStatus::ERR_REJECTED;   // a deferred stow is already safing the platform (adversarial-review fix)
  // MPC-stream interlock: reject while the MPC is actively driving
  // the legs (guard ENABLED on the Jetson) — a cold-start move must not co-drive the
  // same ODrives the 500 Hz stream commands (the interp ISR reciprocally suppresses
  // its TX during this move).
  if (fault_mpc_active()) return RpcStatus::ERR_REJECTED;
  const uint8_t targets = resolve_targets(axis);
  if (targets == 0) return RpcStatus::ERR_BAD_ARGS;  // no present target leg
  // I-FW-19, single-Leg DEACTIVATE: lowering one Leg and IDLEing it under a posed
  // Platform breaks the owner rule (and tilts the Platform), so every OTHER present
  // Leg must already be at Stow. On the one-Leg bench the others are absent and it
  // passes (and rig_is_bench exempts the bench outright, inside leg_idle_permitted).
  // AXIS_ALL (the descent itself) and the Hand are not gated here.
  if (axis < NUM_LEGS && !leg_idle_permitted(micros64(), axis)) return RpcStatus::ERR_REJECTED;
  // Reject if a deactivate is active or a start is already pending (idempotent).
  // IRQ-guarded so (busy-check + latch) is atomic vs the deactivate task that
  // consumes s_start_req at the top of deactivate_step.
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  const bool busy = (s_phase != DPhase::IDLE) || s_start_req;
  if (!busy) { s_start_axis = axis; s_start_req = true; }
  __set_PRIMASK(pm);
  if (busy) return RpcStatus::ERR_REJECTED;
  // Accepted: the targets' HELD bits restart from this request (fw53_design.md 3).
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (targets & (uint8_t)(1u << i)) leg_held_clear(i);
  return RpcStatus::OK;
}

bool deactivate_active() { return s_phase != DPhase::IDLE || s_start_req; }

uint8_t deactivate_result(uint8_t axis) {
  return (axis < NUM_AXES) ? s_result[axis] : (uint8_t)DEACTIVATE_NONE;
}

void deactivate_step() {
  // ── Consume a pending start request (net task → deactivate task). ──
  if (s_phase == DPhase::IDLE) {
    const uint32_t pm = __get_PRIMASK(); __disable_irq();
    const bool    start = s_start_req;
    const uint8_t ax    = s_start_axis;
    s_start_req = false;
    if (start) s_phase = DPhase::SETUP;
    __set_PRIMASK(pm);
    if (!start) return;
    s_targets = resolve_targets(ax);
    s_done = 0;
    if (s_targets == 0) { s_phase = DPhase::IDLE; return; }  // axes vanished between request and consume
    for (uint8_t i = 0; i < NUM_AXES; ++i)
      if (s_targets & (uint8_t)(1u << i)) s_result[i] = DEACTIVATE_RUNNING;
    // The hand de-energises here and leaves the ladder (see idle_hand_now). A
    // hand-only DEACTIVATE (axis == HAND_AXIS) is therefore complete in this tick.
    s_targets = idle_hand_now(s_targets);
    if (s_targets == 0) { s_phase = DPhase::IDLE; return; }
    s_setup_idx = 0;   // SETUP configures one leg per tick starting here
    s_t_start_us = micros64();
    s_t_phase_us = s_t_start_us;
    // s_phase already set to SETUP inside the guard; fall through and run it.
  }

  const uint64_t now = micros64();
  if (s_phase == DPhase::ABORT_WAIT) { abort_wait_step(now); return; }
  const bool timed_out = now - s_t_start_us > D_TIMEOUT_US;

  // ── Aborts before the descent is commanded (checked before any drive). ──
  if (s_phase == DPhase::SETUP || s_phase == DPhase::COMMAND) {
    if (fault_can_bus_down())              { end_ladder(DEACTIVATE_FAILED); return; }   // nothing to a dead bus
    if (!deactivate_allowed() || timed_out) { abort_before_descent(now); return; }
  }

  switch (s_phase) {
    case DPhase::SETUP: {
      // Configure ONE leg per tick (s_setup_idx cursor) — same anti-burst pattern
      // as ACTIVATE (5 frames × 6 legs = 30 overflowed the CAN3 TX buffer, 2026-06-26).
      // The COMMAND phase still fires all targets in one tick AFTER every leg is
      // configured, so the physical descent is parallel (even platform lowering).
      while (s_setup_idx < NUM_LEGS && !(s_targets & (uint8_t)(1u << s_setup_idx)))
        ++s_setup_idx;   // NUM_LEGS, not NUM_AXES: the hand already left the mask
      if (s_setup_idx >= NUM_LEGS) {
        s_t_phase_us = now;          // start the settle clock for COMMAND
        s_phase = DPhase::COMMAND;
        break;
      }
      const uint8_t i = s_setup_idx;
      // Synchronous per-leg error gate (legacy _gentle_move "refuse to enter
      // CLOSED_LOOP if errors present"). ANY errored target fails the whole op —
      // a coupled platform must not lower a subset (that tilts it). The abort is
      // the pre-descent one: IDLE only from Stow, else hold (FW 53).
      if (axes[i].active_errors != 0) { abort_before_descent(now); return; }
      // Seed the ACTUAL current position (NON-clipped) so CLOSED_LOOP holds in
      // place, then the whole descent is TRAP_TRAJ-profiled with no clip-snap.
      // leg_sign applies the ODrive convention (Jugglebot positive=extension).
      const float cur = axes[i].pos_rev;  // single-word atomic read, Jugglebot conv
      s_seed[i] = cur;                    // one end of this leg's freeze segment
      bool ok = true;
      ok = can_jugglebot_send(
               ODrive::encode_set_input_pos(i, ODrive::leg_sign(i, cur), 0, 0)) && ok;
      ok = can_jugglebot_send(
               ODrive::encode_set_traj_vel_limit(i, JBOp::GENTLE_MOVE_VEL_LIMIT_RPS)) && ok;
      ok = can_jugglebot_send(
               ODrive::encode_set_traj_acc_limits(
                   i, ODriveDefaults::TRAP_ACC_LIMIT_RPS2,
                   ODriveDefaults::TRAP_DEC_LIMIT_RPS2)) && ok;
      ok = can_jugglebot_send(
               ODrive::encode_set_controller_mode(
                   i, ODriveControlMode::POSITION, ODriveInputMode::TRAP_TRAJ)) && ok;
      ok = (odrive_set_state_tx(i, ODriveState::CLOSED_LOOP, TxCls::OTHER, Who::LADDER)
                == DeResult::SENT) && ok;
      if (!ok) { abort_before_descent(now); return; }
      axes[i].controller_mode = (uint8_t)ODriveControlMode::POSITION;   // commanded-mode record
      axes[i].input_mode      = (uint8_t)ODriveInputMode::TRAP_TRAJ;    // (axis_state.h)
      ++s_setup_idx;   // next target leg on the next tick
      break;
    }
    case DPhase::COMMAND: {
      if (now - s_t_phase_us < CL_ENGAGE_SETTLE_US) break;  // let CLOSED_LOOP + mode/limits settle
      for (uint8_t i = 0; i < NUM_LEGS; ++i) {
        if (!(s_targets & (uint8_t)(1u << i))) continue;
        // Position-only target (vel_ff = tor_ff = 0). encode_leg_setpoint clips to
        // [0,max] + applies leg_sign. The ODrive TRAP_TRAJ planner trajectories
        // from the seeded current (active) pos DOWN to the STOW pose.
        can_jugglebot_send(
            ODrive::encode_leg_setpoint(i, STOW_OFF_POSE_REV, 0.0f, 0.0f));
      }
      s_phase = DPhase::MONITOR;  // stay active through the physical descent
      break;
    }
    case DPhase::MONITOR: {
      // The ODrives are executing TRAP_TRAJ to STOW. Owner decision 1: by default a
      // Soft E-stop / commands-not-allowed is not consulted after COMMAND (see
      // DESCENT_CONTINUES_THROUGH_ESTOP).
      if (!DESCENT_CONTINUES_THROUGH_ESTOP && !deactivate_allowed()) {
        freeze_live_and_end();
        return;
      }
      // The gated descent monitor shared with ACTIVATE's return to Stow
      // (leg_stow_gate.h stow_descent_step): bus down → end, no frame; a target leg
      // that disarmed itself → freeze the others; arrival → gated IDLE per leg
      // (retried next tick if refused); timeout → gated IDLE at Stow, else freeze.
      const uint8_t done_before = s_done;
      const DescentStep r = stow_descent_step(s_targets, s_done, s_seed, s_seed, timed_out, now);
      // A leg IDLE'd this tick: OK at arrival, FAILED when the timeout IDLE'd it.
      const uint8_t newly = (uint8_t)(s_done & ~done_before);
      for (uint8_t i = 0; i < NUM_LEGS; ++i)
        if (newly & (uint8_t)(1u << i))
          s_result[i] = (r == DescentStep::TIMED_OUT) ? DEACTIVATE_FAILED : DEACTIVATE_OK;
      if (r == DescentStep::RUNNING) break;
      if (r == DescentStep::ARRIVED) { s_phase = DPhase::IDLE; s_targets = 0; s_done = 0; return; }
      end_ladder(DEACTIVATE_FAILED);   // BUS_DOWN, DROPPED, TIMED_OUT: the live legs FAILED
      return;
    }
    default:
      abort_before_descent(now);
      break;
  }
}

}  // namespace CanBridge
