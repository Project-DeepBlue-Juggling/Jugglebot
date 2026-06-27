#pragma once
// =============================================================================
//  leg_deactivate.h — Phase 11 U5 firmware deactivate (TRAP_TRAJ move to STOW,
//  then IDLE)
// =============================================================================
//  The controlled inverse of ACTIVATE: lowers the legs from the active pose
//  (≈ 2.19 rev) back to the STOW pose (CanBridge::STOW_OFF_POSE_REV = 0.0 rev —
//  the same off-pose can_node deactivated to) using the ODrive's onboard
//  TRAP_TRAJ planner, then drops every leg to IDLE once it has arrived and
//  settled. This is the β-path analogue of can_node `deactivate`
//  (`_gentle_move_steps(0.0, deactivating=True)` → IDLE), but it delegates the
//  trajectory to the ODrive (POSITION/TRAP_TRAJ) instead of streaming a software
//  trapezoid over PASSTHROUGH — ODrive-autonomous, no host streaming, exactly as
//  ACTIVATE does (the offload philosophy).
//
//  Why a bounded firmware op (not a generic SET_INPUT_POS RPC): identical to
//  ACTIVATE — the β architecture has no per-leg "move" RPC and only the gated
//  40 Hz setpoint stream moves a leg under command (D10); a controlled lower is a
//  bounded, single-purpose firmware op so we never re-introduce a second,
//  unbounded leg-motion path. The target is the firmware constant STOW_OFF_POSE_REV,
//  so the Jetson cannot command "deactivate to anywhere".
//
//  Parallel even-descent: ALL present legs fire together so the platform lowers
//  straight down (no tilt/binding), mirroring ACTIVATE's even-rise.
//
//  The IDLE on arrival is the one behavioural difference from ACTIVATE (which
//  leaves the legs in CLOSED_LOOP holding the active pose). Deactivate's whole
//  purpose is to safe the robot: a controlled descent ending de-energised. The
//  Jetson observes completion via the CLOSED_LOOP → IDLE transition
//  (controller/teensy_link/deactivate.py DeactivateMonitor), NOT the post-IDLE
//  resting position — the homed/stowed leg relaxes into the foam stop by a
//  variable amount the instant it IDLEs, so post-IDLE position is unreliable (the
//  exact lesson from the 2026-06-26 homing-observer bug). The firmware MONITOR
//  owns the position+velocity arrival check while still in CLOSED_LOOP; the
//  Jetson trusts the firmware's clean IDLE end-state.
//
//  Precondition: the legs already have usable gains/limits (a prior /configure)
//  and are holding the active pose in CLOSED_LOOP. Deactivate sets the traj
//  limits, TRAP_TRAJ mode, re-asserts CLOSED_LOOP, seeds the actual current
//  position (so the descent is fully TRAP_TRAJ-profiled, no clip-snap), then
//  commands the STOW target.
//
//  Fire-and-monitor: the DEACTIVATE RPC validates + latches a start and returns
//  OK immediately; `deactivate_step()` runs the SETUP → settle → COMMAND →
//  MONITOR → IDLE ladder in a task; the Jetson observes the descent via telemetry.
//
//  Determinacy / safety: no unbounded loops, no blocking, no ISR work. Bounded by
//  a hard timeout; any abort (bus down / E-STOP / timeout) leaves the targeted
//  legs in IDLE — the move is NEVER left driving. Re-running DEACTIVATE while one
//  is active is rejected (idempotent); a concurrent HOME or ACTIVATE is rejected
//  too (the cold-start moves are mutually exclusive).
// =============================================================================

#include <cstdint>

namespace CanBridge {

// Per-axis deactivate outcome — firmware-internal status (the Jetson infers the
// same from telemetry; this is for the diag/`[deactivate]` print).
enum DeactivateResult : uint8_t {
  DEACTIVATE_NONE    = 0,   // never deactivated this boot
  DEACTIVATE_RUNNING = 1,   // the descent ladder is in progress for this axis
  DEACTIVATE_OK      = 2,   // reached STOW + settled + IDLE'd
  DEACTIVATE_FAILED  = 3,   // aborted (timeout / bus down / E-STOP)
};

void deactivate_init();

// RPC entry (net-task context). `axis` == AXIS_ALL deactivates every PRESENT leg
// (parallel even-descent); a single leg index deactivates just that leg iff
// present. Validates bus health + targets, rejects a concurrent
// DEACTIVATE/ACTIVATE/HOME, then latches a non-blocking start. Returns a
// JbUdp::RpcStatus (OK = accepted).
uint16_t deactivate_request(uint8_t axis);

// Drive the deactivate state machine one tick. Call at the cold-start task rate
// (HOMING_RATE_HZ, shared task_homing) from a task (never an ISR). No-op when
// idle with no pending request.
void deactivate_step();

bool    deactivate_active();             // a deactivate is pending or running
uint8_t deactivate_result(uint8_t axis); // last DeactivateResult for a leg axis (0..5)

}  // namespace CanBridge
