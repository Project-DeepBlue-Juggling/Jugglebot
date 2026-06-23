#pragma once
// =============================================================================
//  leg_homing.h — Phase 9b firmware homing (velocity-limited move to hardstop)
// =============================================================================
//  Line-for-line port of can_node.py `_home_motor_steps` (~:1387): drive a leg
//  in VELOCITY/VEL_RAMP at a fixed speed until the current (Iq) EMA crosses the
//  homing limit (the motor stalls against the mechanical hardstop), then IDLE and
//  `set_absolute_position` to define that stop as the home reference. The proven
//  recipe detects the stop by CURRENT SPIKE — NOT a known position — so it does
//  NOT (and cannot) reuse the interp's position-streamed `interp_begin_stow`
//  descent (that targets a known pose and the stroke-clamp would block reaching a
//  hardstop). See plans/active/teensy-can-offload.md "Phase 9b".
//
//  Why this lives in firmware: there is no per-leg "move" RPC — the only path
//  that moves a leg is the 40 Hz setpoint stream (D10). Homing-to-hardstop drives
//  the leg, so the move belongs in the can-bridge HOME handler (Revised
//  sequencing item 3).
//
//  Fire-and-monitor: the HOME RPC validates + latches a start and returns OK
//  immediately (the net task must not block for the ~seconds a homing takes).
//  `homing_step()` runs the state machine in its own FreeRTOS task at
//  HOMING_RATE_HZ (≈ the 100 Hz Iq update rate, so the Iq EMA matches can_node's
//  per-reading cadence). The Jetson observes completion via the telemetry +
//  diagnostic streams (axis_state → IDLE, pos → LEG_ABS_POS_REV), exactly as
//  Phase 9a observed encoder search — no protocol change.
//
//  Determinacy / safety: no unbounded loops, no blocking, no ISR work. Every
//  active phase is bounded by a hard timeout (Homing::MOTOR_TIMEOUT_S); any
//  abort (bus down / ODrive fatal / timeout) leaves the axis in IDLE — the move
//  is NEVER left driving. Re-running HOME while one is active is rejected
//  (idempotent); single-axis scope (legs 0..5; rejects the hand and AXIS_ALL).
// =============================================================================

#include <cstdint>

namespace CanBridge {

// Per-axis homing outcome — firmware-internal status (the Jetson infers the same
// from telemetry; this is for the diag/`[home]` print and possible future uplink).
enum HomingResult : uint8_t {
  HOMING_NONE    = 0,   // never homed this boot
  HOMING_RUNNING = 1,   // a homing is in progress on this axis
  HOMING_OK      = 2,   // hardstop found + reference set
  HOMING_FAILED  = 3,   // aborted (timeout / bus down / ODrive fatal)
};

void homing_init();

// RPC entry (net-task context). Validates `axis` (leg 0..5 only) + bus health,
// then latches a non-blocking start request. Returns a JbUdp::RpcStatus
// (OK = accepted/started). ERR_REJECTED if a homing is already active.
uint16_t homing_request(uint8_t axis);

// Drive the homing state machine one tick. Call at HOMING_RATE_HZ from a task
// (never an ISR). No-op when idle with no pending request.
void homing_step();

bool    homing_active();             // a homing is pending or running
uint8_t homing_result(uint8_t axis); // last HomingResult for a leg axis (0..5)

}  // namespace CanBridge
