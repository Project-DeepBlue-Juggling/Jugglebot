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
//    * Arm the stow latch at CAN-loss DETECTION (the canonical arm point).
//    * Execute the stow only on CONFIRMED reconnect (fresh leg heartbeats).
//    * Re-arm if the stow half-completes (bus re-drops mid-descent).
//    * Soft-reset attempt cap (= 1) exists because of a real bounce-loop
//      incident — do NOT simplify. One auto-clear, then fatal until an explicit
//      CLEAR_ERRORS resets the budget (mirrors can_node clear_error_flags).
//
//  DELIBERATE per-path axis-scope split — NOT an oversight:
//    * ERROR-EVAL + CLEAR paths iterate NUM_AXES (legs 0-5 + HAND axis 6):
//      evaluate_errors, clear_errors_can's CAN send, clear_disarm_reasons. This is
//      can_node parity — _handle_error evaluates the hand too, so a hand active-error
//      or hand disarm-while-CLOSED_LOOP ESTOPs the whole platform, and a CLEAR clears
//      the hand's caches. (OBSERVABLE BEHAVIOUR CHANGE: a hand fault now E-STOPs the
//      legs; call it out at the next powered re-validation sitting.)
//    * CAN3 WATCHDOG + DEFERRED STOW stay legs-only (NUM_LEGS): any_leg_heartbeat_stale,
//      all_present_legs_fresh, s_first_leg_hb_seen, the stow-complete IDLE fan-out. The
//      stow is a LEG-only physical retraction to the off pose; the hand is NEVER stowed,
//      so a stale hand heartbeat must NOT arm the leg watchdog/stow. NOTE this is
//      DELIBERATELY TIGHTER than can_node, whose any_heartbeat_stale scans JUGGLEBOT_AXES
//      (legs + hand): for the CAN3 bus-loss event this watchdog is named for the two are
//      behaviourally equivalent (all leg heartbeats go stale together on a bus drop); the
//      only divergence is a silent HAND-only ODrive drop, which can_node would stow but we
//      deliberately do not (there is nothing to stow on the hand).
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
#include "leg_activate.h"       // activate_active (mutual exclusion with deferred stow)
#include "leg_deactivate.h"     // deactivate_active (mutual exclusion with deferred stow)
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

// ── Guard E-STOP latch (motor_guard sticky-self.mode semantics) ──
// The three guard E-STOP conditions (MOTOR_OVERSPEED / MPC_STALE / MAX_DEVIATION)
// LATCH once tripped: guard_mode stays ESTOP and 500 Hz output stays gated off until
// an EXPLICIT operator CLEAR_ERRORS (fault_notify_clear_errors) — mirroring
// motor_guard._trigger_estop's sticky self.mode (motor_guard.py:1137-1142), whose
// only exit is an operator disable/enable. PRE-LATCH these auto-cleared the instant
// the transient cleared (vel dropped, a fresh setpoint arrived, deviation shrank) and
// silently re-enabled leg streaming with NO operator ack. Set by the fault task,
// cleared by the RPC task → volatile (matches s_mpc_active / s_reboot_in_progress).
// First-trigger-wins freezes the reported reason. fb_stale is deliberately EXCLUDED
// (motor-feedback staleness is recoverable, not a latched E-STOP).
static volatile bool s_estop_latched = false;
static uint8_t       s_estop_state = JbUdp::FaultState::NONE;  // reason frozen at latch

// ── MAX_DEVIATION latch-event snapshot (2026-07-10 forensics) ──
// Frozen at the instant a MAX_DEVIATION guard E-STOP first LATCHES: which leg
// crossed, its deviation (u0 - encoder), the raw commanded base u0, and the
// encoder at the crossing. The 2026-07-10 runaway could not be pinned because
// the 10 Hz link_status straddled the crossing and no per-leg deviation was
// recorded; this snapshot surfaces the exact trip on HeartbeatT2J → /link_status.
// Persists across a CLEAR_ERRORS (post-mortem value) — a genuine re-latch refreshes
// it; only fault_machine_init() clears it. leg == 0xFF ⇒ no MAX_DEVIATION latch yet.
static uint8_t s_max_dev_latch_leg = 0xFF;
static float   s_max_dev_latch_dev = 0.0f;
static float   s_max_dev_latch_u0  = 0.0f;
static float   s_max_dev_latch_enc = 0.0f;

// ── Reboot-in-progress watchdog-suppression latch ──────────────────
// Armed ONLY by fault_notify_reboot_started() (the REBOOT_ODRIVES RPC), so a
// spontaneous CAN loss never sets it and the deferred-stow inversion is preserved.
static volatile bool s_reboot_in_progress = false;   // suppression latch armed
static uint64_t      s_reboot_deadline_us = 0;        // 64-bit; access via atomic_*_u64
static bool          s_reboot_saw_stale   = false;    // legs went stale since arming

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
  const uint64_t now = micros64();   // interval clock — never the steppable wall
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (axes[i].heartbeat_seen && (now - atomic_read_u64(&axes[i].last_heartbeat_us) > CAN_HEARTBEAT_TIMEOUT_US))
      return true;
  return false;
}
// Scoped to PRESENT legs (leg_present == heartbeat_seen). A present leg
// that has gone stale fails the predicate; absent legs (never seen) are skipped.
// This is what lets the deferred-stow reconnect fire on a subset-populated bus
// (the single-leg bench rig — odrv0 only): the prior all-six form dead-locks the
// reconnect branch (legs 1-5 never report fresh), so a stow armed at CAN-loss
// detection could never complete. No-op on the full robot (all six present, so it
// still requires all six fresh). Vacuously true only with zero present legs,
// which cannot co-occur with the fatal_can_error precondition of the sole caller
// (that needs s_first_leg_hb_seen — at least one leg already seen).
static bool all_present_legs_fresh() {
  const uint64_t now = micros64();   // interval clock — never the steppable wall
  for (uint8_t i = 0; i < NUM_LEGS; ++i)
    if (leg_present(i) && (now - atomic_read_u64(&axes[i].last_heartbeat_us) > CAN_HEARTBEAT_TIMEOUT_US))
      return false;
  return true;
}

// Reset the soft-reset budget + transient flags (mirror motors.clear_error_flags).
static void clear_error_flags() {
  s_fatal_error = false;
  s_undervoltage_error = false;
  s_soft_reset_attempts = 0;
}
// Zero the per-axis disarm_reason cache (mirror can_node._clear_errors →
// motor_state.clear_disarm_reasons). The disarm_reason cache is written ONLY by the
// Get_Error RX decode, so after a CLEAR a stale nonzero disarm would otherwise persist
// until the next Get_Error frame; during that window evaluate_errors would read the
// stale disarm (any_disarmed=true) and re-enter the soft-reset/fatal branch, re-faulting
// the state the operator just cleared. Kept a SEPARATE function from clear_error_flags()
// to preserve the 1:1 structural mirror of the Python (which calls clear_error_flags()
// and clear_disarm_reasons() as distinct steps). Scoped to NUM_AXES (hand
// parity with can_node, which clears the hand too). Mirrors ONLY disarm_reason, NOT
// active_errors: active_errors self-heals on the next Get_Error, and zeroing it could
// mask a still-active error.
static void clear_disarm_reasons() {
  for (uint8_t i = 0; i < NUM_AXES; ++i) axes[i].disarm_reason = 0;
}
// Send clear_errors to all axes incl. the hand (mirror can_node
// ._clear_errors), only if the bus is up — never command a dead bus. Returns true
// iff it actually sent.
static bool clear_errors_can() {
  if (s_fatal_can_error) return false;
  for (uint8_t i = 0; i < NUM_AXES; ++i) can_jugglebot_send(ODrive::encode_clear_errors(i));  // NUM_AXES incl. hand (aligns rpc.cpp AXIS_ALL)
  clear_error_flags();
  clear_disarm_reasons();   // mirror can_node._clear_errors → clear_disarm_reasons
  return true;
}

// External CLEAR_ERRORS RPC resets the auto-retry budget (mirrors the operator path)
// AND is the SOLE release path for the guard E-STOP latch. The latch
// reset must live HERE, not in clear_error_flags() — clear_error_flags is also called
// by the internal soft-reset / UV-recovery auto-retry (clear_errors_can), and letting
// those internal paths release the guard E-STOP would defeat the latch (e.g. an ODrive
// UV bounce auto-clearing a motor-overspeed E-STOP with no operator ack).
void fault_notify_clear_errors() {
  clear_error_flags();
  clear_disarm_reasons();   // mirror can_node._clear_errors → clear_disarm_reasons (else the
                            // stale disarm cache re-faults until the next Get_Error frame)
  s_estop_latched = false;
  s_estop_state = JbUdp::FaultState::NONE;
}

// External REBOOT_ODRIVES RPC arms the bounded watchdog-suppression latch:
// the deliberate reboot silence must not be read as a CAN loss. Bounded by
// s_reboot_deadline_us (the blind-spot backstop). Called from the RPC/UDP-RX task; the
// 64-bit deadline is written atomically (read by the fault task's watchdog_and_stow).
// s_reboot_saw_stale is reset here so a fresh reboot cannot inherit a stale "saw" from
// a prior one (which could release the new latch early). s_reboot_in_progress is
// published LAST so the fault task never observes an armed latch with a stale deadline.
void fault_notify_reboot_started() {
  atomic_write_u64(&s_reboot_deadline_us, micros64() + REBOOT_WATCHDOG_SUPPRESS_US);  // interval deadline
  s_reboot_saw_stale = false;
  s_reboot_in_progress = true;
}

// ── Port of _handle_error determination ───────────────────────────────────────
static void evaluate_errors() {
  bool no_active = true, no_disarm = true, any_disarmed = false, any_cl = false;
  bool all_uv_only = true;
  // iterate NUM_AXES (legs 0-5 + HAND axis 6), NOT NUM_LEGS. can_node
  // _handle_error evaluates over states[:len(JUGGLEBOT_AXES)] = 7 (incl. the hand),
  // so a HAND active-error or hand disarm-while-CLOSED_LOOP must set s_fatal_error /
  // ESTOP here too (else the firmware diverges from can_node — the standing hand-gap
  // note). See the DELIBERATE per-path scope split documented at the top of this file.
  for (uint8_t i = 0; i < NUM_AXES; ++i) {
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
  const uint64_t now = micros64();   // interval clock — never the steppable wall
  for (uint8_t i = 0; i < NUM_AXES; ++i)
    axes[i].heartbeat_stale = axes[i].heartbeat_seen &&
                              (now - atomic_read_u64(&axes[i].last_heartbeat_us) > CAN_HEARTBEAT_TIMEOUT_US);

  // Ball Butler heartbeat staleness (CAN1, BB_HEARTBEAT_TIMEOUT_US = 0.5 s).
  // Information-only — no fault response is triggered by BB silence (the CAN3
  // watchdog below is the safety-relevant one); this just keeps bb_state.
  // heartbeat_stale in sync for the [bb] diag print and upstream HeartbeatT2J.
  bb_state.heartbeat_stale = bb_state.heartbeat_seen &&
      (now - atomic_read_u64(&bb_state.last_heartbeat_us) > BB_HEARTBEAT_TIMEOUT_US);
      // ^ atomic (review fix): the last bare last_heartbeat_us read the torn-u64 hardening sweep
      //   missed — the CAN3-decode writer (task_can_rx, prio 5) can preempt this
      //   fault-task (prio 3) read between its two 32-bit loads → a torn u64. Info-only
      //   (BB silence drives no fault/stow), but keep it torn-free like its siblings.

  // Reboot-in-progress latch. A REBOOT_ODRIVES RPC armed a bounded
  // suppression of the CAN-loss detector so the deliberate reboot silence is not read
  // as a real loss. Release on fresh-leg-heartbeats-AFTER-going-stale (the ODrives
  // came back — the common case, far tighter than the deadline) OR at the bounded
  // deadline (the failure-case backstop: a real loss coinciding with a reboot is
  // caught here, at most REBOOT_WATCHDOG_SUPPRESS_US late). The s_reboot_saw_stale
  // gate is essential: a latch armed while the legs are STILL fresh (the reboot
  // frames have not yet silenced them) must not release immediately on that same
  // freshness — only a fresh reading AFTER the legs have actually gone stale proves
  // the reboot completed. Runs BEFORE detection so the AND below sees the released state.
  if (s_reboot_in_progress) {
    if (any_leg_heartbeat_stale()) s_reboot_saw_stale = true;
    if ((s_reboot_saw_stale && all_present_legs_fresh()) ||
        now >= atomic_read_u64(&s_reboot_deadline_us)) {
      s_reboot_in_progress = false;
    }
  }

  // Detection: CAN3 leg heartbeats went stale → fatal_can_error + arm the stow.
  // ANDs !s_reboot_in_progress so a deliberate reboot's silence does not false-trip
  // a spontaneous loss never arms that latch, so it detects as before.
  if (s_first_leg_hb_seen && any_leg_heartbeat_stale() && !s_fatal_can_error
      && !s_reboot_in_progress) {
    s_fatal_can_error = true;
    s_stow_pending = true;            // canonical arm point — independent of any host path
    if (s_stowing) { interp_end_stow(); s_stowing = false; }  // bus re-dropped mid-descent → re-arm
  }

  // Confirmed reconnect: leg heartbeats fresh again → clear fatal, run the stow.
  // Mutual exclusion with homing: the homing task drives its own
  // velocity frames on CAN3 (and aborts to IDLE the instant fault_can_bus_down
  // latched, well before this reconnect branch can fire), so in practice homing
  // is already done by here. The !homing_active() guard makes that invariant
  // explicit — never let the position-streamed stow and a homing move co-drive a
  // leg; the stow stays pending and runs the next cycle once homing finishes.
  if (s_fatal_can_error && all_present_legs_fresh()) {
    s_fatal_can_error = false;
    if (s_stow_pending && !s_stowing && !homing_active() && !activate_active() && !deactivate_active()) {
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
  return (micros64() - last) <= JETSON_LINK_TIMEOUT_US;   // interval: udp_last_rx_us is mono
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
  const uint64_t age = micros64() - interp_last_setpoint_us();   // interval: setpoint stamped mono
  const bool ever_cmd = interp_last_setpoint_us() != 0;
  if (!estop && ever_cmd && s_mpc_active && age > MPC_CMD_STALENESS_US) {
    estop = true; state = JbUdp::FaultState::MPC_STALE;
  }
  // Max deviation: the incoming MPC command (interp base = u0) diverged too far
  // from the encoder — catches stale zeros / sign errors / runaway command
  // sources (motor_guard.py:539-551, checked at command-arrival not per-tick).
  // Capture the crossing (leg/dev/u0/enc) locally; it is frozen into the latch
  // snapshot below ONLY on the first trip (so a later tick can't overwrite it).
  uint8_t md_leg = 0xFF;
  float   md_dev = 0.0f, md_u0 = 0.0f, md_enc = 0.0f;
  if (!estop && s_mpc_active && interp_have_latched()) {
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      // Only check present legs. An absent leg reads pos_rev=0; a
      // nonzero u0 broadcast to it (the production MPC sends 6 leg targets) would
      // false-trip the E-STOP. No-op on the full robot.
      if (!leg_present(i)) continue;
      const float u0  = interp_base_pos(i);
      const float enc = axes[i].pos_rev;
      const float dev = u0 - enc;
      if (fabsf(dev) > MAX_DEVIATION_REV) {
        estop = true; state = JbUdp::FaultState::MAX_DEVIATION;
        md_leg = i; md_dev = dev; md_u0 = u0; md_enc = enc;
        break;
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
  // pos_timestamp_us is still 0). pos_timestamp_us is stamped micros64() on each
  // encoder RX (can_buses.cpp), so this monotonic clock comparison is consistent.
  bool fb_stale = false;
  if (s_mpc_active && interp_have_latched()) {
    const uint64_t now = micros64();   // interval clock — never the steppable wall
    for (uint8_t i = 0; i < NUM_LEGS; ++i) {
      // atomic_read_u64: a naked u64 load is two 32-bit reads on Cortex-M7, and
      // task_can_rx (the writer, higher priority) can preempt between them — a
      // torn timestamp here makes the staleness guard flap in BOTH directions.
      // now > ts guards the ordering half of the hazard: the prio-5 writer can
      // stamp AFTER our `now` capture, and an unclamped subtraction would wrap
      // past the threshold — a false one-tick MOTOR_FB_STALE (~100 ms output
      // suppression) on the live leg path.
      const uint64_t ts = atomic_read_u64(&axes[i].pos_timestamp_us);
      if (leg_present(i) && now > ts && (now - ts > MOTOR_FB_STALENESS_US)) {
        fb_stale = true; break;
      }
    }
  }

  // Latch the guard E-STOP: once any guard condition trips, hold it
  // (sticky guard_mode==ESTOP + output gated off) until fault_notify_clear_errors().
  // The `&& !s_estop_latched` freezes the reported reason at the FIRST trip (mirrors
  // motor_guard._check_safety's early-return pinning self._fault_state). fb_stale is
  // NOT latched — it is deliberately recoverable.
  if (estop && !s_estop_latched) {
    s_estop_latched = true; s_estop_state = state;
    // Freeze the MAX_DEVIATION snapshot at the crossing (only when that is the
    // latching reason; md_leg==0xFF for the overspeed/stale paths).
    if (state == JbUdp::FaultState::MAX_DEVIATION) {
      s_max_dev_latch_leg = md_leg;
      s_max_dev_latch_dev = md_dev;
      s_max_dev_latch_u0  = md_u0;
      s_max_dev_latch_enc = md_enc;
    }
  }

  // Fault-state reporting priority (highest-severity active condition wins).
  if (s_fatal_can_error)        s_fault_state = JbUdp::FaultState::CAN_BUS_DOWN;
  else if (s_fatal_error)       s_fault_state = JbUdp::FaultState::ODRIVE_FATAL;
  else if (s_estop_latched)     s_fault_state = s_estop_state;
  else if (fb_stale)            s_fault_state = JbUdp::FaultState::MOTOR_FB_STALE;
  else if (!jetson_link_up())   s_fault_state = JbUdp::FaultState::LINK_LOST;
  else                          s_fault_state = JbUdp::FaultState::NONE;

  // Guard mode. The latched E-STOP holds ESTOP across ticks (not the instantaneous
  // `estop`) until an operator clear.
  if (s_estop_latched || s_fatal_error)             s_guard_mode = JbUdp::GuardMode::ESTOP;
  else if (s_mpc_active && jetson_link_up() && !s_fatal_can_error)
                                                    s_guard_mode = JbUdp::GuardMode::ENABLED;
  else                                              s_guard_mode = JbUdp::GuardMode::DISABLED;

  // Output gate. NEVER command a dead bus. During a deferred stow, output stays
  // on so the descent reaches CAN3 (stow only runs post-reconnect, so the bus is
  // confirmed up). Otherwise output requires the guard ENABLED and no fatal.
  //
  // Stow vs fatal/E-STOP (DELIBERATE, not an oversight): an in-progress
  // stow is allowed to complete even if s_estop_latched / s_fatal_error trips
  // mid-descent. Aborting the stow (gating output off) would disarm the raised legs
  // → gravity drop — strictly worse than finishing the controlled, velocity-limited
  // descent to the off pose, which IS the safety response. (The velocity limit
  // bounds any overspeed during the descent.) The one condition that legitimately
  // interrupts a stow is a bus RE-DROP, which re-arms it (watchdog_and_stow sets
  // s_stowing=false and re-latches s_stow_pending). A guard E-STOP still latches and
  // holds AFTER the stow completes — it simply does not pre-empt the descent.
  bool allow = false;
  if (s_stowing) {
    allow = true;   // stow descent integrates its own position; not encoder-gated
  } else {
    allow = (s_guard_mode == JbUdp::GuardMode::ENABLED) && !s_fatal_can_error
            && !s_fatal_error && !s_estop_latched && !fb_stale;
  }
  interp_set_output_enabled(allow);
}

void fault_machine_init() {
  s_fatal_error = s_undervoltage_error = s_fatal_can_error = false;
  s_soft_reset_attempts = 0;
  s_stow_pending = s_stowing = s_first_leg_hb_seen = false;
  s_estop_latched = false;
  s_estop_state = JbUdp::FaultState::NONE;
  s_max_dev_latch_leg = 0xFF;
  s_max_dev_latch_dev = s_max_dev_latch_u0 = s_max_dev_latch_enc = 0.0f;
  s_reboot_in_progress = s_reboot_saw_stale = false;
  atomic_write_u64(&s_reboot_deadline_us, 0);
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
bool    fault_mpc_active()   { return s_mpc_active; }

// MAX_DEVIATION latch-event snapshot (frozen at the crossing; 0xFF leg ⇒ none).
uint8_t fault_max_dev_leg()   { return s_max_dev_latch_leg; }
float   fault_max_dev_value() { return s_max_dev_latch_dev; }
float   fault_max_dev_u0()    { return s_max_dev_latch_u0; }
float   fault_max_dev_enc()   { return s_max_dev_latch_enc; }

}  // namespace CanBridge
