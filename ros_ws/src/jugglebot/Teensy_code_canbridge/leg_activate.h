#pragma once
// =============================================================================
//  leg_activate.h — Phase 11 U5 firmware activate (TRAP_TRAJ move to active pose)
// =============================================================================
//  Moves the legs from the homed hardstop (≈ −0.10 rev, below the workspace) to
//  the active pose (JBOp::ACTIVATE_POSITION_REVS ≈ 2.19 rev — the IK of
//  [0,0,default_active_z,0,0,0]) using the ODrive's onboard TRAP_TRAJ planner.
//  The β architecture has no per-leg "move" RPC and only the gated 40 Hz setpoint
//  stream moves a leg under command (D10); like HOME, the activation move is a
//  bounded, single-purpose firmware op rather than a general SET_INPUT_POS RPC
//  (which would re-introduce a second, unbounded leg-motion path). The target is
//  a firmware constant (codegen'd JBOp::ACTIVATE_POSITION_REVS), so the Jetson
//  cannot command "activate to anywhere".
//
//  This is the β-path analogue of can_node `_gentle_move_steps`, but it delegates
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

// RPC entry (net-task context). `axis` == AXIS_ALL activates every PRESENT leg
// (parallel even-rise); a single leg index activates just that leg iff present.
// Validates bus health + targets, rejects a concurrent ACTIVATE/HOME, then
// latches a non-blocking start. Returns a JbUdp::RpcStatus (OK = accepted).
uint16_t activate_request(uint8_t axis);

// Drive the activate state machine one tick. Call at the cold-start task rate
// (HOMING_RATE_HZ, shared task_homing) from a task (never an ISR). No-op when
// idle with no pending request.
void activate_step();

bool    activate_active();             // an activate is pending or running
uint8_t activate_result(uint8_t axis); // last ActivateResult for a leg axis (0..5)

}  // namespace CanBridge
