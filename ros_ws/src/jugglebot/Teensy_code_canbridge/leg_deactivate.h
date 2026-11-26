#pragma once
// =============================================================================
//  leg_deactivate.h — firmware deactivate (TRAP_TRAJ move to STOW,
//  then IDLE)
// =============================================================================
//  The controlled inverse of ACTIVATE: lowers the legs from the active pose
//  (≈ 2.19 rev) back to the STOW pose (CanBridge::STOW_OFF_POSE_REV = 0.0 rev —
//  the same off-pose can_node deactivated to) using the ODrive's onboard
//  TRAP_TRAJ planner, then drops every leg to IDLE once it has arrived and
//  settled. This is the Teensy-side analogue of can_node `deactivate`
//  (`_gentle_move_steps(0.0, deactivating=True)` → IDLE), but it delegates the
//  trajectory to the ODrive (POSITION/TRAP_TRAJ) instead of streaming a software
//  trapezoid over PASSTHROUGH — ODrive-autonomous, no host streaming, exactly as
//  ACTIVATE does (the offload philosophy).
//
//  Why a bounded firmware op (not a generic SET_INPUT_POS RPC): identical to
//  ACTIVATE — the Teensy-side architecture has no per-leg "move" RPC and only the gated
//  40 Hz setpoint stream moves a leg under command; a controlled lower is a
//  bounded, single-purpose firmware op so we never re-introduce a second,
//  unbounded leg-motion path. The target is the firmware constant STOW_OFF_POSE_REV,
//  so the Jetson cannot command "deactivate to anywhere".
//
//  Parallel even-descent: ALL present legs fire together so the platform lowers
//  straight down (no tilt/binding), mirroring ACTIVATE's even-rise.
//
//  THE HAND (axis 6) DE-ENERGISES, IT DOES NOT DESCEND (skill-stack R1). ACTIVATE
//  energises axis 6 and parks it at 0 rev, so DEACTIVATE is what puts it back —
//  but the choreography is not symmetric, because the physics is not. A leg
//  carries the platform and must be lowered under profile; the hand's carriage
//  hangs on a spool and gravity takes it to the bottom stop the moment the axis
//  IDLEs. So the hand is IDLE'd in the very first tick of the op, BEFORE the legs
//  begin their descent, and then leaves the target mask entirely — it is never in
//  SETUP, COMMAND or MONITOR, and a later leg abort finds it already safe. A
//  hand-only DEACTIVATE (axis == HAND_AXIS) therefore completes in one tick.
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
//  a hard timeout. FW 53 (I-FW-19): no Leg is made IDLE unless the Platform is
//  already at Stow, so an abort HOLDS the legs instead of dropping them: before
//  the descent is commanded nothing is sent (each leg holds where it is), during
//  it the live legs are frozen at their stopping point on their own commanded
//  segment (leg_freeze_tx); a dead bus gets no frame at all; a guard E-STOP after
//  the target is commanded lets the descent finish. The leg IDLEs (arrival, or an
//  abort from an already-stowed Platform) go through the Stow gate. A single-leg
//  DEACTIVATE is refused while any other present leg is not at Stow. See
//  leg_deactivate.cpp's header and fw53_design.md 3.1. Re-running DEACTIVATE while one
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
  DEACTIVATE_FAILED  = 3,   // aborted (timeout / bus down / E-STOP / errored or dropped leg): held, or IDLE only from Stow
};

void deactivate_init();

// RPC entry (net-task context). `axis` == AXIS_ALL deactivates every PRESENT axis
// — the hand IDLEs immediately, the legs lower in parallel (even descent); a
// single axis index (0..6) deactivates just that axis iff present. Validates bus
// health + targets, rejects a concurrent
// DEACTIVATE/ACTIVATE/HOME, rejects a single-leg request (ERR_REJECTED) unless
// every OTHER present leg is at Stow (I-FW-19), clears the targets' HELD bits on
// accept, then latches a non-blocking start. Returns a
// JbUdp::RpcStatus (OK = accepted).
uint16_t deactivate_request(uint8_t axis);

// Drive the deactivate state machine one tick. Call at the cold-start task rate
// (HOMING_RATE_HZ, shared task_homing) from a task (never an ISR). No-op when
// idle with no pending request.
void deactivate_step();

bool    deactivate_active();             // a deactivate is pending or running
uint8_t deactivate_result(uint8_t axis); // last DeactivateResult for an axis (0..6, hand included)

}  // namespace CanBridge
