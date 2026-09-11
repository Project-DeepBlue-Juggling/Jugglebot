#pragma once
// =============================================================================
//  leg_activate.h — firmware activate (TRAP_TRAJ move to active pose)
// =============================================================================
//  Moves every PRESENT axis from its homed hardstop (≈ −0.10 rev, below the
//  workspace) to its active pose using the ODrive's onboard TRAP_TRAJ planner:
//  the six legs to JBOp::ACTIVATE_POSITION_REVS ≈ 2.19 rev (the IK of
//  [0,0,default_active_z,0,0,0]) and the HAND (axis 6) to
//  JBOp::HAND_ACTIVATE_POSITION_REV = 0.0 rev — the clip floor, ~3.3 mm of
//  carriage travel off the homed stop at Homing::HAND_ABS_POS_REV = −0.10.
//
//  ACTIVATE ENERGISES THE HAND (skill-stack R1, owner decision 2026-09-11). Before
//  R1 the operator energised axis 6 by hand (`--close-loop`) and the hand_source
//  latch decided who owned it; both are gone. There is now exactly one hand master
//  — the streamed lane in leg_interp.cpp — and ACTIVATE is its handover: it parks
//  the hand at 0 rev and leaves axis 6 in POSITION/PASSTHROUGH, the mode that lane
//  commands in. The park is a ONE-SHOT second writer of an axis-6 setpoint, and it
//  can never overlap the streamed one: leg_interp's `coldstart` interlock
//  (homing_active() || activate_active() || deactivate_active()) suppresses the
//  whole 7-frame burst, hand included, for the entire duration of this op.
//  The Teensy-side architecture has no per-leg "move" RPC and only the gated 40 Hz setpoint
//  stream moves a leg under command; like HOME, the activation move is a
//  bounded, single-purpose firmware op rather than a general SET_INPUT_POS RPC
//  (which would re-introduce a second, unbounded leg-motion path). The target is
//  a firmware constant (codegen'd JBOp::ACTIVATE_POSITION_REVS), so the Jetson
//  cannot command "activate to anywhere".
//
//  This is the Teensy-side analogue of can_node `_gentle_move_steps`, but it delegates
//  the trajectory to the ODrive (POSITION/TRAP_TRAJ) instead of streaming a
//  software trapezoid over PASSTHROUGH — ODrive-autonomous, no host streaming,
//  consistent with the offload philosophy.
//
//  Parallel even-rise: ALL present legs fire together so the platform rises
//  straight up (no tilt/binding) — unlike HOME, which is sequential. Gentler than
//  the legacy clip-snap: the seed before CLOSED_LOOP is the ACTUAL sub-zero
//  hardstop position (non-clipped), so the whole move is TRAP_TRAJ-profiled with
//  no ~0.1 rev snap off the hardstop.
//
//  Precondition (mirrors HOME's "inherits gains from prior _setup_odrives"): the
//  legs must already have usable position/velocity gains + vel/curr limits, set
//  by a prior /configure (teensy_bridge_node._run_configure). Activate sets only
//  the traj limits, TRAP_TRAJ mode, CLOSED_LOOP, and the target.
//
//  Fire-and-monitor: the ACTIVATE RPC validates + latches a start and returns OK
//  immediately; `activate_step()` runs the SETUP → settle → COMMAND ladder in a
//  task; the Jetson observes the physical move via telemetry (pos → active,
//  vel → 0) with controller/teensy_link/activate.py ActivateMonitor — no firmware
//  status field, exactly as HOME/encoder-search are observed.
//
//  Determinacy / safety: no unbounded loops, no blocking, no ISR work. Bounded by
//  a hard timeout; any abort (bus down / E-STOP / timeout) leaves the targeted
//  legs in IDLE — the move is NEVER left driving. Re-running ACTIVATE while one is
//  active is rejected (idempotent); a concurrent HOME is rejected too.
// =============================================================================

#include <cstdint>

namespace CanBridge {

// Per-axis activate outcome — firmware-internal status (the Jetson infers the
// same from telemetry; this is for the diag/`[activate]` print).
enum ActivateResult : uint8_t {
  ACTIVATE_NONE    = 0,   // never activated this boot
  ACTIVATE_RUNNING = 1,   // the fire ladder is in progress for this axis
  ACTIVATE_OK      = 2,   // TRAP_TRAJ move commanded (the ODrive runs it)
  ACTIVATE_FAILED  = 3,   // aborted (timeout / bus down / E-STOP)
};

void activate_init();

// RPC entry (net-task context). `axis` == AXIS_ALL activates every PRESENT axis
// — the six legs in parallel (even platform rise) AND the hand; a single axis
// index (0..6) activates just that axis iff present.
// Validates bus health + targets, rejects a concurrent ACTIVATE/HOME, then
// latches a non-blocking start. Returns a JbUdp::RpcStatus (OK = accepted).
uint16_t activate_request(uint8_t axis);

// Drive the activate state machine one tick. Call at the cold-start task rate
// (HOMING_RATE_HZ, shared task_homing) from a task (never an ISR). No-op when
// idle with no pending request.
void activate_step();

bool    activate_active();             // an activate is pending or running
uint8_t activate_result(uint8_t axis); // last ActivateResult for an axis (0..6, hand included)

}  // namespace CanBridge
