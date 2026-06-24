// =============================================================================
//  fault_machine.cpp — fault state machine + CAN3 watchdog + deferred stow
// =============================================================================
//  Single enforcement point for fault response on the Teensy. Ports
//  can_node.py:386-483 (_handle_error), :1443-1530 (_watchdog_check), and
//  :1098-1145 (_fault_response / _actuators_intact_and_holding), plus
//  motor_guard.py:843-864 (staleness + overspeed E-STOP).
//
//  The watchdog observes the leg ODrive heartbeats, which arrive on CAN3 (the
//  Jugglebot core bus, per ADR-0013); the logic is bus-agnostic — it reads the
//  per-axis cache, populated by the CAN3 RX decode.
//
//  Deferred-stow invariants (logbook 2026-05-19), preserved EXACTLY:
//    * CAN3 down ⇒ never command the dead bus (output gated off; ODrives hold).
//    * Arm the stow latch at CAN-loss DETECTION (the canonical §6 point).
//    * Execute the stow only on CONFIRMED reconnect (fresh leg heartbeats).
//    * Re-arm if the stow half-completes (bus re-drops mid-descent).
//    * Soft-reset attempt cap (= 1) exists because of a real bounce-loop
//      incident — do NOT simplify. One auto-clear, then fatal until an explicit
//      CLEAR_ERRORS resets the budget (mirrors can_node clear_error_flags).
// =============================================================================
#include "fault_machine.h"

#include <cmath>
#include "canbridge_config.h"
#include "protocol_config.h"     // ODriveState
#include "udp_protocol.h"        // FaultState, GuardMode
#include "axis_state.h"
#include "ball_butler_state.h"
#include "odrive_protocol.h"
#include "can_buses.h"
#include "leg_interp.h"
#include "leg_homing.h"         // homing_active (mutual exclusion with deferred stow)
#include "udp_link.h"            // udp_last_rx_us (Jetson link health)
#include "time_base.h"

namespace CanBridge {

static bool s_fatal_error = false;        // active error / disarm-while-CLOSED_LOOP
static bool s_undervoltage_error = false;
static bool s_fatal_can_error = false;    // CAN3 (Jugglebot core bus) confirmed down
static uint8_t s_soft_reset_attempts = 0;
static bool s_stow_pending = false;       // deferred-stow latch
static bool s_stowing = false;            // a deferred stow is in progress
static bool s_first_leg_hb_seen = false;
static volatile bool s_mpc_active = false;
static uint8_t s_guard_mode = JbUdp::GuardMode::DISABLED;
static uint8_t s_fault_state = JbUdp::FaultState::NONE;

void fault_set_mpc_active(bool a) { s_mpc_active = a; }

// ── Cache predicates over the leg axes ────────────────────────────────────────
static bool legs_all_closed_loop() {
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (axes[i].axis_state != ODriveState::CLOSED_LOOP) return false;
  return true;
}
static bool legs_clean() {
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (axes[i].active_errors != 0 || axes[i].disarm_reason != 0) return false;
  return true;
}
// _actuators_intact_and_holding: all legs CLOSED_LOOP and error/disarm-free.
static bool actuators_intact_and_holding() { return legs_all_closed_loop() && legs_clean(); }

static bool any_leg_heartbeat_stale() {
  const uint64_t now = now_wall_us();
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (axes[i].heartbeat_seen && (now - axes[i].last_heartbeat_us > CAN_HEARTBEAT_TIMEOUT_US))
      return true;
  return false;
}
// Phase 11: scoped to PRESENT legs (leg_present == heartbeat_seen). A present leg
// that has gone stale fails the predicate; absent legs (never seen) are skipped.
// This is what lets the deferred-stow reconnect fire on a subset-populated bus
// (the single-leg bench rig — odrv0 only): the prior all-six form dead-locks the
// reconnect branch (legs 1-5 never report fresh), so a stow armed at CAN-loss
// detection could never complete. No-op on the full robot (all six present, so it
// still requires all six fresh). Vacuously true only with zero present legs,
// which cannot co-occur with the fatal_can_error precondition of the sole caller
// (that needs s_first_leg_hb_seen — at least one leg already seen).
static bool all_present_legs_fresh() {
  const uint64_t now = now_wall_us();
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (leg_present(i) && (now - axes[i].last_heartbeat_us > CAN_HEARTBEAT_TIMEOUT_US))
      return false;
  return true;
}

// Reset the soft-reset budget + transient flags (mirror motors.clear_error_flags).
static void clear_error_flags() {
  s_fatal_error = false;
  s_undervoltage_error = false;
  s_soft_reset_attempts = 0;
}
// Send clear_errors to all legs (mirror can_node._clear_errors), only if the bus
// is up — never command a dead bus. Returns true iff it actually sent.
static bool clear_errors_can() {
  if (s_fatal_can_error) return false;
  for (uint8_t i = 0; i < NUM_LEGS; ++i) can_jugglebot_send(ODrive::encode_clear_errors(i));
  clear_error_flags();
  return true;
}

// External CLEAR_ERRORS RPC resets the auto-retry budget (mirrors the operator path).
void fault_notify_clear_errors() { clear_error_flags(); }

// ── Port of _handle_error determination ───────────────────────────────────────
static void evaluate_errors() {
  bool no_active = true, no_disarm = true, any_disarmed = false, any_cl = false;
  bool all_uv_only = true;
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    const uint32_t ae = axes[i].active_errors;
    const uint32_t dr = axes[i].disarm_reason;
    if (ae != 0) no_active = false;
    if (dr != 0) { no_disarm = false; any_disarmed = true; }
    if (axes[i].axis_state == ODriveState::CLOSED_LOOP) any_cl = true;
    if (!(dr == 0 || dr == ERR_DC_BUS_UNDER_VOLTAGE)) all_uv_only = false;
    if (ae & ERR_DC_BUS_UNDER_VOLTAGE) s_undervoltage_error = true;
  }

  // All axes fully clean — safe to clear everything (NB: does NOT reset the
  // soft-reset budget; only an explicit CLEAR_ERRORS does — bounce-loop guard).
  if (no_active && no_disarm) {
    s_fatal_error = false;
    s_undervoltage_error = false;
    return;
  }

  bool is_fatal = false;
  if (!no_active) is_fatal = true;                       // active errors → fatal
  if (any_disarmed && any_cl) is_fatal = true;           // disarmed while CLOSED_LOOP → fatal

  if (any_disarmed && !any_cl) {                         // disarmed, none CLOSED_LOOP → try one clear
    if (s_soft_reset_attempts < MAX_SOFT_RESET_ATTEMPTS) {
      // Only consume the budget if the clear actually went out (bus up).
      // clear_errors_can resets the budget to 0 on success; the +1 then makes it
      // exactly one auto-retry. On a down bus it is a no-op and the budget is
      // preserved (the CAN-down watchdog owns that case).
      if (clear_errors_can()) s_soft_reset_attempts += 1;
    } else {
      is_fatal = true;                                   // budget exhausted → fatal
    }
  }

  // Undervoltage recovery: every disarm is 0-or-undervoltage and there WAS a
  // disarm ⇒ the bus dropped and recovered; clear undervoltage (+ auto-clear if
  // nothing else is fatal).
  if (no_active && all_uv_only && any_disarmed) {
    s_undervoltage_error = false;
    if (!is_fatal) { clear_errors_can(); return; }
  }

  if (is_fatal) s_fatal_error = true;
}

// ── CAN3 watchdog + deferred stow ─────────────────────────────────────────────
static void watchdog_and_stow() {
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (axes[i].heartbeat_seen) { s_first_leg_hb_seen = true; break; }

  // Mark per-axis staleness for telemetry.
  const uint64_t now = now_wall_us();
  for (uint8_t i = 0; i < NUM_AXES; ++i)
    axes[i].heartbeat_stale = axes[i].heartbeat_seen &&
                              (now - axes[i].last_heartbeat_us > CAN_HEARTBEAT_TIMEOUT_US);

  // Ball Butler heartbeat staleness (CAN1, BB_HEARTBEAT_TIMEOUT_US = 0.5 s).
  // Information-only — no fault response is triggered by BB silence (the CAN3
  // watchdog below is the safety-relevant one); this just keeps bb_state.
  // heartbeat_stale in sync for the [bb] diag print and upstream HeartbeatT2J.
  bb_state.heartbeat_stale = bb_state.heartbeat_seen &&
      (now - bb_state.last_heartbeat_us > BB_HEARTBEAT_TIMEOUT_US);

  // Detection: CAN3 leg heartbeats went stale → fatal_can_error + arm the stow.
  if (s_first_leg_hb_seen && any_leg_heartbeat_stale() && !s_fatal_can_error) {
    s_fatal_can_error = true;
    s_stow_pending = true;            // canonical arm point (§6) — independent of any host path
    if (s_stowing) { interp_end_stow(); s_stowing = false; }  // bus re-dropped mid-descent → re-arm
  }

  // Confirmed reconnect: leg heartbeats fresh again → clear fatal, run the stow.
  // Mutual exclusion with Phase 9b homing: the homing task drives its own
  // velocity frames on CAN3 (and aborts to IDLE the instant fault_can_bus_down
  // latched, well before this reconnect branch can fire), so in practice homing
  // is already done by here. The !homing_active() guard makes that invariant
  // explicit — never let the position-streamed stow and a homing move co-drive a
  // leg; the stow stays pending and runs the next cycle once homing finishes.
  if (s_fatal_can_error && all_present_legs_fresh()) {
    s_fatal_can_error = false;
    if (s_stow_pending && !s_stowing && !homing_active()) {
      interp_begin_stow();            // profiled velocity-limited descent to the off pose
      s_stowing = true;
    }
  }

  // Stow progress. Completion → IDLE all legs (deactivating) and clear the latch.
  if (s_stowing) {
    if (interp_stow_complete()) {
      for (uint8_t i = 0; i < NUM_LEGS; ++i) can_jugglebot_send(ODrive::encode_set_state(i, ODriveState::IDLE));
      interp_end_stow();
      s_stowing = false;
      s_stow_pending = false;
    }
  }
}

// ── Guard mode + output gate (port of motor_guard staleness/overspeed) ────────
static bool jetson_link_up() {
  const uint64_t last = udp_last_rx_us();
  if (last == 0) return false;
  return (now_wall_us() - last) <= JETSON_LINK_TIMEOUT_US;
}

static void evaluate_guard() {
  // E-STOP conditions (motor_guard._check_safety).
  bool estop = false;
  uint8_t state = JbUdp::FaultState::NONE;

  // Motor overspeed.
  for (uint8_t i = 0; i < NUM_LEGS; ++i) {
    float v = axes[i].vel_rps; if (v < 0) v = -v;
    if (v > MAX_MOTOR_VEL_RPS) { estop = true; state = JbUdp::FaultState::MOTOR_OVERSPEED; break; }
  }
  // MPC command staleness (the hard link-fault trigger).
  const uint64_t age = now_wall_us() - interp_last_setpoint_us();
  const bool ever_cmd = interp_last_setpoint_us() != 0;
  if (!estop && ever_cmd && s_mpc_active && age > MPC_CMD_STALENESS_US) {
    estop = true; state = JbUdp::FaultState::MPC_STALE;
  }
  // Max deviation: the incoming MPC command (interp base = u0) diverged too far
  // from the encoder — catches stale zeros / sign errors / runaway command
  // sources (motor_guard.py:539-551, checked at command-arrival not per-tick).
  if (!estop && s_mpc_active && interp_have_latched()) {
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      // Phase 11: only check present legs. An absent leg reads pos_rev=0; a
      // nonzero u0 broadcast to it (the production MPC sends 6 leg targets) would
      // false-trip the E-STOP. No-op on the full robot.
      if (!leg_present(i)) continue;
      if (fabsf(interp_base_pos(i) - axes[i].pos_rev) > MAX_DEVIATION_REV) {
        estop = true; state = JbUdp::FaultState::MAX_DEVIATION; break;
      }
    }
  }

  // Motor feedback staleness (port of motor_guard MOTOR_FB_STALENESS_S=0.15,
  // motor_guard.py:884-891). Suppress output if a PRESENT leg's encoder feedback
  // has gone stale — the fast guard for a frozen feedback loop. MAX_DEVIATION above
  // only catches a freeze while the command is moving AWAY from the frozen encoder;
  // a HOLD freeze (cmd ≈ frozen enc) would otherwise go unnoticed and fly blind.
  // Recoverable, NOT a latched E-STOP: output re-enables when feedback returns,
  // mirroring motor_guard's command suppression. Present-scoped + mpc_active/latched
  // gated exactly like MAX_DEVIATION (so it never false-trips pre-arm, when
  // pos_timestamp_us is still 0). pos_timestamp_us is stamped now_wall_us() on each
  // encoder RX (can_buses.cpp), so this clock comparison is consistent.
  bool fb_stale = false;
  if (s_mpc_active && interp_have_latched()) {
    const uint64_t now = now_wall_us();
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      if (leg_present(i) && (now - axes[i].pos_timestamp_us > MOTOR_FB_STALENESS_US)) {
        fb_stale = true; break;
      }
    }
  }

  // Fault-state reporting priority (highest-severity active condition wins).
  if (s_fatal_can_error)        s_fault_state = JbUdp::FaultState::CAN_BUS_DOWN;
  else if (s_fatal_error)       s_fault_state = JbUdp::FaultState::ODRIVE_FATAL;
  else if (estop)               s_fault_state = state;
  else if (fb_stale)            s_fault_state = JbUdp::FaultState::MOTOR_FB_STALE;
  else if (!jetson_link_up())   s_fault_state = JbUdp::FaultState::LINK_LOST;
  else                          s_fault_state = JbUdp::FaultState::NONE;

  // Guard mode.
  if (estop || s_fatal_error)                       s_guard_mode = JbUdp::GuardMode::ESTOP;
  else if (s_mpc_active && jetson_link_up() && !s_fatal_can_error)
                                                    s_guard_mode = JbUdp::GuardMode::ENABLED;
  else                                              s_guard_mode = JbUdp::GuardMode::DISABLED;

  // Output gate. NEVER command a dead bus. During a deferred stow, output stays
  // on so the descent reaches CAN3 (stow only runs post-reconnect, so the bus is
  // confirmed up). Otherwise output requires the guard ENABLED and no fatal.
  bool allow = false;
  if (s_stowing) {
    allow = true;   // stow descent integrates its own position; not encoder-gated
  } else {
    allow = (s_guard_mode == JbUdp::GuardMode::ENABLED) && !s_fatal_can_error
            && !s_fatal_error && !estop && !fb_stale;
  }
  interp_set_output_enabled(allow);
}

void fault_machine_init() {
  s_fatal_error = s_undervoltage_error = s_fatal_can_error = false;
  s_soft_reset_attempts = 0;
  s_stow_pending = s_stowing = s_first_leg_hb_seen = false;
  s_guard_mode = JbUdp::GuardMode::DISABLED;
  s_fault_state = JbUdp::FaultState::NONE;
  interp_set_output_enabled(false);
}

void fault_step() {
  evaluate_errors();
  watchdog_and_stow();
  evaluate_guard();
}

uint8_t fault_state()        { return s_fault_state; }
uint8_t fault_guard_mode()   { return s_guard_mode; }
bool    fault_can_bus_down() { return s_fatal_can_error; }
bool    fault_stow_pending() { return s_stow_pending; }

}  // namespace CanBridge
