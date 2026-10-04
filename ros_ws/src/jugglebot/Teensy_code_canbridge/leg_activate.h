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
#include "udp_protocol.h"   // JbUdp::RpcArgs::ResultHandMoveTo

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

// ── HAND_MOVE_TO (FW 26, additive RPC 0x61) ──────────────────────────────────
//  ACTIVATE's axis-6 ladder with a CALLER target and cruise: SETUP (error gate,
//  seed input_pos at the measured position, traj vel = the caller's cruise, traj
//  acc/dec, POSITION/TRAP_TRAJ, CLOSED_LOOP) -> 10 ms settle -> COMMAND
//  encode_leg_setpoint(target) -> MONITOR until |pos-target| <= tol and
//  |vel| <= tol -> the same POSITION/PASSTHROUGH hand-off ACTIVATE performs. It
//  SHARES ACTIVATE's state machine, so it is mutually exclusive with ACTIVATE,
//  HOME and DEACTIVATE by construction (activate_active() is true throughout,
//  which also holds leg_interp's coldstart interlock) and inherits every abort:
//  bus down / E-STOP / 10 s timeout -> the hand is commanded IDLE.
//
//  WHY it exists: the hand-jam recovery must raise a hand that is pinching a ball
//  and later lower it, and nothing else can put the hand at a chosen position
//  outside a streamed pattern (the relay allow-table carries no hand position op
//  by design, ACTIVATE's target is the constant 0.0). It never touches the current
//  limit: the caller sets that first with SET_VEL_CURR_LIMITS, so a recovery runs
//  the whole move at its relief current.
//
//  WHY the reply is deferred (the net task never waits): the caller needs to know
//  where the move ENDED, and the move takes seconds. hand_move_request() only
//  validates + latches; the activate task posts exactly one reply per accepted
//  request into a small mailbox on whichever terminal path the move takes, and
//  the NET task drains it (Rpc::rpc_service) and sends the RPC_RESPONSE. The
//  activate task never touches the socket: task_homing has a 1 KB stack sized
//  for CAN sends, and lwIP's send path belongs on task_net's 4 KB one.
//
//  WHY a second request RETARGETS instead of being refused: the recovery's stall
//  response (raise again while a lower is stalled on the ball) must not wait out
//  the 10 s timeout, whose IDLE would drop the hand onto the ball it is meant to
//  free. The ODrive TRAP_TRAJ planner replans from the current state, so a
//  retarget is profiled, never a step. The superseded request is answered
//  (status OK, outcome SUPERSEDED) and the timeout clock restarts for the new
//  target. A request arriving while another is still latched (not yet consumed
//  by the task, <= one task tick) is refused ERR_REJECTED so no request can go
//  unanswered.
enum HandMoveOutcome : uint8_t {
  HAND_MOVE_ARRIVED    = 0,   // OK: at target, settled, handed off to PASSTHROUGH
  HAND_MOVE_SUPERSEDED = 1,   // OK: a newer HAND_MOVE_TO retargeted this move
  HAND_MOVE_TIMEOUT    = 2,   // ERR_TIMEOUT: 10 s elapsed, hand commanded IDLE
  HAND_MOVE_ABORTED    = 3,   // ERR_BUS_DOWN / ERR_REJECTED / ERR_TIMEOUT (TX): hand IDLE
};

// One deferred HAND_MOVE_TO reply (req_id echoed from the request).
struct HandMoveReply {
  uint16_t req_id;
  uint16_t status;                          // JbUdp::RpcStatus
  JbUdp::RpcArgs::ResultHandMoveTo result;
};
// Net-task side of the mailbox: pop the oldest pending reply (false = none).
bool     hand_move_pop_reply(HandMoveReply& out);
uint32_t hand_move_reply_drops();         // mailbox overflows since boot (must stay 0)

// RPC entry (net-task context). Returns OK when the request was LATCHED (the
// activate task answers it later through the mailbox) or the refusal status (answer it now):
//   ERR_BAD_ARGS  axis != HAND_AXIS, hand absent, target not finite or outside
//                 [0, HAND_MOTOR_MAX_POSITION], vel not in (0, GENTLE_MOVE_VEL_LIMIT_RPS]
//   ERR_BUS_DOWN  ACTIVATE's activate_allowed() gate (bus / CAN-down / E-STOP latch)
//   ERR_REJECTED  HOME / DEACTIVATE / stow / MPC-stream active, an ACTIVATE pending
//                 or running, another HAND_MOVE_TO still latched, hand active_errors
uint16_t hand_move_request(uint8_t axis, float target_rev, float vel_rps, uint16_t req_id);
bool     hand_move_active();          // a HAND_MOVE_TO is latched or running

}  // namespace CanBridge
