#pragma once
// =============================================================================
//  fault_machine.h — per-axis fault state machine + CAN3 watchdog + deferred stow
// =============================================================================
//  Ports the Jetson fault logic onto the Teensy:
//    * Per-axis error evaluation + fatal determination + soft-reset attempt
//      limiter + undervoltage gating — can_node.py:386-483 (_handle_error).
//    * CAN3 heartbeat watchdog + deferred-stow latch — can_node.py:1443-1530
//      (_watchdog_check) and 1098-1145 (_fault_response /
//      _actuators_intact_and_holding). The leg heartbeats arrive on CAN3 (the
//      Jugglebot core bus, per ADR-0013).
//    * MPC-staleness + motor-overspeed + max-deviation E-STOP — motor_guard.py:
//      843-864. This E-STOP now LATCHES (Fable-5 [13]): once any of the three guard
//      conditions trips, guard_mode stays ESTOP and interp output stays gated off
//      until an EXPLICIT operator CLEAR_ERRORS (fault_notify_clear_errors) — mirroring
//      motor_guard's sticky self.mode (only an operator disable/enable clears it).
//      The prior per-tick recompute auto-cleared the instant the transient cleared
//      and silently re-enabled 500 Hz leg streaming with no operator ack — unsafe.
//      (motor_guard needs disable-then-enable; the can-bridge re-enables once the
//      latch clears AND s_mpc_active/link/no-fatal-can still hold — a deliberate
//      one-step vs two-step difference, mpc_active being the Jetson's guard-ENABLE.)
//
//  The hard-won deferred-stow safety inversion from
//  logbook/2026-05-19-can-loss-fault-response-safety-inversion.md is preserved
//  EXACTLY (see fault_machine.cpp for the invariant comments):
//    1. CAN3 down  ⇒ NEVER command the dead bus (interp output gated off; the
//       leg ODrives autonomously hold their last CLOSED_LOOP setpoint).
//    2. On CAN3 loss detection, ARM the deferred-stow latch.
//    3. Execute the stow ONLY once the bus is CONFIRMED restored (fresh leg
//       heartbeats) — a profiled velocity-limited descent to the off pose.
//    4. If the stow half-completes (bus re-drops), RE-ARM the latch.
//    5. If the bus never returns, the terminal fail-safe is leg IDLE.
//
//  This is the single enforcement point for fault-stow on the Teensy — both the
//  CAN-loss watchdog and any explicit fault route through here.
// =============================================================================

#include <cstdint>

namespace CanBridge {

void fault_machine_init();
// Drive at FAULT_TASK_HZ (10 Hz). Evaluates errors + watchdogs, sets the guard
// mode / interp output gate / stow state, and updates the reported fault_state.
void fault_step();

// Set by the Jetson-heartbeat handler: is the MPC commanding (guard ENABLED on
// the Jetson)? Gates whether the interp output may be enabled.
void fault_set_mpc_active(bool active);

// An explicit operator CLEAR_ERRORS (RPC) resets the soft-reset auto-retry
// budget — mirrors can_node clear_error_flags (the one-shot auto-clear refills) —
// AND is the SOLE release path for the guard E-STOP latch ([13]). The internal
// soft-reset/UV auto-retry does NOT release the latch (only clear_error_flags,
// not this hook), so an ODrive bounce can't silently clear a guard E-STOP.
void fault_notify_clear_errors();

// A REBOOT_ODRIVES RPC arms a BOUNDED watchdog-suppression latch (Phase 6): the
// ~2–4 s heartbeat silence of a deliberate ODrive reboot must not be read as a CAN
// loss (which would falsely arm the 2026-05-19 deferred stow on reconnect). Armed
// ONLY here, so a SPONTANEOUS CAN loss (not preceded by a reboot) is completely
// unaffected — the deferred-stow inversion is fully preserved. Released on
// fresh-leg-heartbeats-AFTER-stale (the common case — the ODrives came back) OR at
// the bounded REBOOT_WATCHDOG_SUPPRESS_US deadline (the failure-case backstop, so a
// real loss coinciding with a reboot is still caught, at most one window late).
void fault_notify_reboot_started();

// Reported up via HeartbeatT2J.
uint8_t  fault_state();                  // JbUdp::FaultState
uint8_t  fault_guard_mode();             // JbUdp::GuardMode (ESTOP ⟺ estop || fatal_error)
bool     fault_can_bus_down();           // fatal_can_error (CAN3)
bool     fault_stow_pending();           // deferred-stow latch armed
bool     fault_mpc_active();             // s_mpc_active (J→T heartbeat bit0 — bench diag)

}  // namespace CanBridge
