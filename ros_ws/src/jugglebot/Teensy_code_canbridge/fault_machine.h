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
//    * MPC-staleness + motor-overspeed E-STOP — motor_guard.py:843-864.
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
// budget — mirrors can_node clear_error_flags (the one-shot auto-clear refills).
void fault_notify_clear_errors();

// Reported up via HeartbeatT2J.
uint8_t  fault_state();                  // JbUdp::FaultState
bool     fault_can_bus_down();           // fatal_can_error (CAN3)
bool     fault_stow_pending();           // deferred-stow latch armed

}  // namespace CanBridge
