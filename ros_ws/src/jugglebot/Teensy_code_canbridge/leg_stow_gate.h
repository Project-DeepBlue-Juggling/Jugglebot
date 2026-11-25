#pragma once
// =============================================================================
//  leg_stow_gate.h — no Leg leaves Closed-loop above Stow (FW 53, I-FW-19)
// =============================================================================
//  Owner rule (2026-11-25): "A Leg should never be made IDLE unless the Platform
//  is already at the Stow position. Once the Platform is at Stow, then any/all
//  Legs can freely enter IDLE." The Hand is exempt.
//
//  This module is the rule's single enforcement point. It holds:
//    * the predicate platform_at_stow() and its HOME twin all_present_legs_idle();
//    * the three wrappers that are (from FW 53 on) the ONLY code allowed to build
//      a Set_Axis_State, a Reboot or an SDO write for ANY axis, the Hand
//      included, so tests/firmware/test_leg_deenergise_lint.py can forbid the
//      raw encoders everywhere else without a branch-aware exception;
//    * the hold (leg_freeze_tx), the per-Leg HELD mask and the refusal census.
//
//  The predicate (normative text: fw53_design.md 1.1, to land as I-FW-19 in
//  motion/skills/INVARIANTS.md). The Platform is at Stow iff every PRESENT Leg
//  (heartbeat seen since boot, leg_present()) other than `except_leg` has
//    - a heartbeat stamp != 0 and no older than CAN_HEARTBEAT_STALE_US, received
//      at least CL_ENGAGE_SETTLE_US after this gate's last CLOSED_LOOP send to
//      that Leg, and that heartbeat reports
//    - (a) IDLE or ENCODER_INDEX_SEARCH (not holding load), or
//    - (b) CLOSED_LOOP with active_errors == 0, an encoder sample != 0 and no
//      older than MOTOR_FB_STALENESS_US, position in
//      [STOW_WINDOW_MIN_REV, STOW_OFF_POSE_REV + TARGET_REACHED_POS_TOL_REV]
//      and |vel| <= TARGET_REACHED_VEL_TOL_RPS.
//  Anything else refuses. Zero present Legs is vacuously at Stow.
//
//  U1 of the FW 53 arc: this module is compiled and linked into the image but no
//  call site uses it yet (they migrate in U2..U5), so FW_VERSION stays 52 and
//  the board's behaviour is unchanged.
// =============================================================================

#include <cstdint>

namespace CanBridge {

// Why platform_at_stow() refused (StowVerdict::reason). OK only when ok == true.
namespace StowReason {
  constexpr uint8_t OK             = 0;
  constexpr uint8_t STALE_HB       = 1;   // heartbeat stamp 0 or older than CAN_HEARTBEAT_STALE_US
  constexpr uint8_t STALE_ENC      = 2;   // CL Leg: encoder stamp 0 or older than MOTOR_FB_STALENESS_US
  constexpr uint8_t NOT_AT_STOW    = 3;   // CL Leg: position outside the Stow window (or non-finite)
  constexpr uint8_t MOVING         = 4;   // CL Leg: |vel| > TARGET_REACHED_VEL_TOL_RPS (or non-finite)
  constexpr uint8_t ERRORED        = 5;   // CL Leg: active_errors != 0
  constexpr uint8_t UNKNOWN_STATE  = 6;   // a state that is neither IDLE, ENCODER_INDEX_SEARCH nor CL
  constexpr uint8_t CL_UNCONFIRMED = 7;   // heartbeat older than the last CL send + CL_ENGAGE_SETTLE_US
}

struct StowVerdict {
  bool    ok;
  uint8_t blocking_leg;   // first Leg (ascending index) that refused; 0xFF when ok
  uint8_t reason;         // StowReason
};

// The predicate. `now` is micros64() captured by the caller BEFORE the stamps are
// read (rx_older_than reads a stamp newer than `now` as age zero, BUG-93).
// `except_leg` (default none) is skipped as if absent: DEACTIVATE(single Leg)
// asks whether every OTHER present Leg is at Stow.
StowVerdict platform_at_stow(uint64_t now, uint8_t except_leg = 0xFF);

// The firmware twin of ARMING_CONTRACT A6, for HOME(Leg): every present Leg has
// a fresh, CL-confirmed heartbeat reporting IDLE (strictly; ENCODER_INDEX_SEARCH
// and an at-Stow Closed-loop Leg both refuse).
bool all_present_legs_idle(uint64_t now);

// Who is asking. Decides only the HOMING_OWNER carve-out and whether a Leg
// refusal sets that Leg's HELD bit.
enum class Who : uint8_t { LADDER, STOW, RPC, HOMING_OWNER };
enum class DeResult : uint8_t { SENT, REFUSED, TX_FAILED };

// The three wrappers. By case:
//   * the Hand (HAND_AXIS): passes straight through (exempt);
//   * odrive_set_state_tx(Leg, CLOSED_LOOP): stamps the Leg's CL-command time
//     (before the send, so a failed send still delays the confirmation: the
//     conservative side) and sends; not gated;
//   * odrive_set_state_tx(Leg, IDLE, HOMING_OWNER) for the Leg the homing task is
//     driving (homing_axis_in_flight()): sent without the predicate (the A6 twin
//     established every Leg IDLE when the HOME was accepted). Any other
//     HOMING_OWNER request falls through to the predicate like any caller;
//   * anything else for a Leg: platform_at_stow(micros64()). On refusal nothing
//     goes on CAN, the census counts it, and LADDER/STOW callers set the Leg's
//     HELD bit (an RPC refusal is the host's answer, not a held Leg);
//   * axis >= NUM_AXES: REFUSED, not counted.
// TX_FAILED = the frame did not reach the wire (tx_reached_the_wire false).
DeResult odrive_set_state_tx(uint8_t axis, uint32_t state, uint8_t cls, Who who);
DeResult odrive_reboot_tx(uint8_t axis, uint8_t cls, Who who);
// Every SDO write to a Leg is gated (endpoint ids are unverified per ODrive FW,
// and one of them is requested_state). SDO READS are not de-energises and stay
// on ODrive::encode_sdo_read.
DeResult odrive_sdo_write_tx(uint8_t axis, uint16_t endpoint_id, float value, uint8_t cls, Who who);

// The hold, for a Closed-loop POSITION/TRAP_TRAJ Leg (fw53_design.md 3):
// target = clamp(pos + v*|v| / (2 * TRAP_DEC_LIMIT_RPS2), [min(lo,hi), max(lo,hi)]),
// sent as set_input_pos(leg, leg_sign(target), 0, 0) on TxCls::SAFETY. Stopping
// at the stopping point avoids TRAP_TRAJ's decelerate-past-and-return; clamping to
// the Leg's own commanded segment means a glitched encoder can never command
// travel outside the path already commanded. Sends NOTHING (returns false) when
// the encoder stamp is 0 or older than MOTOR_FB_STALENESS_US, or any input is
// non-finite: the ODrive then finishes its existing, in-stroke target. Always
// sets the Leg's HELD bit (the caller has decided to hold it). Hand / out of
// range: false, no frame, no bit. True iff the frame reached the wire.
bool  leg_freeze_tx(uint8_t leg, float seg_lo, float seg_hi);
// The pure target arithmetic behind leg_freeze_tx (exposed for the native test).
float leg_freeze_target(float pos, float vel, float seg_lo, float seg_hi);

// Per-Leg HELD bits (bit i = Leg i), sticky until cleared by the next accepted
// ACTIVATE/DEACTIVATE/HOME targeting that Leg or CLEAR_ERRORS(AXIS_ALL) (wired in
// U2..U5; the Diagnostic's DIAG_FLAG_LEG_HELD lands in U5).
uint8_t  leg_held_mask();
void     leg_held_clear(uint8_t leg);
void     leg_held_clear_all();
// Leg de-energise requests refused since boot (all callers). Console/diag only.
uint32_t leg_deenergise_refusals();
// micros64() of this gate's last CLOSED_LOOP send to a Leg (0 = none since boot).
uint64_t leg_cl_cmd_us(uint8_t leg);

// Zero every file-static (boot, and each native TEST_CASE).
void leg_stow_gate_init();

}  // namespace CanBridge
