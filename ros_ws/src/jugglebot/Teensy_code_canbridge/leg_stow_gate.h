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
//  FW 53 (U1..U5): every Leg de-energise call site goes through the wrappers
//  below; FW_VERSION 53 is the first image in which they all do.
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

// ── The bench exemption (owner, 2026-11-25) ──────────────────────────────────
// "The bench leg can be put into IDLE at any position, without any negative
// consequence ... the bench setup will only ever have one ODrive on CAN2, while the
// robot has 7 (and the Platform Teensy)." The same board drives both, so the rig is
// inferred from what the Jugglebot bus has carried since boot, and every doubt
// reads ROBOT (gate on). rig_is_bench(now) is true iff ALL of:
//   * the robot latch is clear;
//   * exactly ONE ODrive axis, Leg or Hand (node ids 0..6), has been heard since
//     boot: a well-formed frame noted by rig_note_axis_frame, or heartbeat_seen;
//   * no Platform Teensy frame since boot (rig_note_platform_frame);
//   * the bus was transmittable at the last 1 kHz sample, and
//     now - max(first axis frame, bus last became transmittable) >= RIG_BENCH_SETTLE_US.
// A second axis or any Platform Teensy frame sets the robot latch (sticky until the
// next bridge boot), even if those nodes later fall silent. A bench misread as a
// robot is harmless (its Leg must then be at Stow to idle); the reverse is the
// failure this predicate is built against (leg_stow_gate.cpp, logbook
// 2026-11-25-fw53-u1b-bench-exemption).
bool rig_is_bench(uint64_t now);

// The one question every Leg de-energise asks (the wrappers, DEACTIVATE):
// rig_is_bench(now) || platform_at_stow(now, except_leg).ok.
bool leg_idle_permitted(uint64_t now, uint8_t except_leg = 0xFF);

// The witnesses, written by the Jugglebot-bus RX/service path (can_buses.cpp,
// task_can_rx): a well-formed frame from ODrive axis `axis`; any Platform Teensy
// frame (is_platform_teensy_tx_id); the live ESR1.SYNCH sample.
void rig_note_axis_frame(uint8_t axis, uint64_t now);
void rig_note_platform_frame();
void rig_note_bus(bool transmittable, uint64_t now);

// Console view (the [rig] serial line): bit a = axis a heard since boot; the
// Platform witness; the sticky robot latch.
uint8_t rig_axes_heard_mask();
bool    rig_platform_seen();
bool    rig_robot_latched();

// The firmware twin of ARMING_CONTRACT A6, for HOME(Leg): every present Leg has
// a fresh, CL-confirmed heartbeat reporting IDLE (strictly; ENCODER_INDEX_SEARCH
// and an at-Stow Closed-loop Leg both refuse).
bool all_present_legs_idle(uint64_t now);

// Who is asking. Decides only the HOMING_OWNER carve-out and whether a Leg
// refusal sets that Leg's HELD bit and counts in the census.
// ARRIVAL_RETRY (FW 53 audit follow-up): an arrival IDLE whose caller has just
// established leg_idle_permitted(now) this tick and asks again next tick
// (stow_descent_step's arrival loop, the deferred-stow CONFIRM loop). A refusal
// there can only be a frame landing between the caller's check and the wrapper's;
// it is refused silently (nothing on CAN, no HELD, no census), because HELD reaches
// the host at once and its ACTIVATE/DEACTIVATE monitors fail on the first HELD.
enum class Who : uint8_t { LADDER, STOW, RPC, HOMING_OWNER, ARRIVAL_RETRY };
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
//   * anything else for a Leg: leg_idle_permitted(micros64()), i.e. the bench
//     exemption or platform_at_stow(). On refusal nothing
//     goes on CAN, the census counts it, and LADDER/STOW callers set the Leg's
//     HELD bit (an RPC refusal is the host's answer, not a held Leg); an
//     ARRIVAL_RETRY refusal is silent (no census, no HELD);
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
// stopping_distance = false (FW 53 audit follow-up, the deferred-stow CONFIRM
// timeout): target = clamp(pos) only. For a Leg that may be in PASSTHROUGH, where
// set_input_pos is an unprofiled step, the measured position is the one target that
// commands no motion; in TRAP_TRAJ it is still a profiled move.
bool  leg_freeze_tx(uint8_t leg, float seg_lo, float seg_hi, bool stopping_distance = true);
// The pure target arithmetic behind leg_freeze_tx (exposed for the native test).
float leg_freeze_target(float pos, float vel, float seg_lo, float seg_hi);

// ── Shared by the ACTIVATE and DEACTIVATE ladders (FW 53 U3) ─────────────────
// True iff at least one present Leg refuses the Stow predicate and EVERY refusing
// Leg refuses only CL_UNCONFIRMED (its last heartbeat predates this gate's
// CLOSED_LOOP send + CL_ENGAGE_SETTLE_US). platform_at_stow() names only the first
// refusing Leg, so a posed Leg behind a configured one would otherwise read as
// "just unconfirmed".
bool platform_blocked_only_by_cl_unconfirmed(uint64_t now);

// How long the ladders wait for Closed-loop confirmation after the last CLOSED_LOOP
// command: COMMAND's confirmation before any motion (legs_cl_confirm, FW 53 U8) and
// the pre-motion abort's wait before it asks for the IDLEs (ABORT_WAIT), one
// constant because both wait for the same event. A Leg whose drive takes E ms to
// report CLOSED_LOOP (the engage latency, unmeasured) has its first CLOSED_LOOP
// heartbeat within E + 99 ms (heartbeat period), E + 198 ms with one lost, and the
// ladder sees it one 10 ms tick later: E + 208 ms must fit. 250 ms (U3) tolerated
// only E < 42 ms; 400 ms tolerates E < 192 ms. Cost: a Leg that never engages
// holds the others (stationary, nothing commanded) up to 400 ms before the abort.
constexpr uint64_t CL_CONFIRM_WAIT_US = 400000ull;

// The pre-motion abort's per-tick verdict. SEND: leg_idle_permitted(now), ask the
// gate for the IDLEs now. WAIT: refused only for CL_UNCONFIRMED and fewer than
// CL_CONFIRM_WAIT_US since `t_abort_us`; send nothing, ask again next tick.
// HOLD: anything else; ask for the IDLEs (the gate refuses them, sets HELD).
enum class AbortIdle : uint8_t { SEND, WAIT, HOLD };
AbortIdle leg_abort_idle_verdict(uint64_t now, uint64_t t_abort_us);

// A target Leg has left CLOSED_LOOP on its own: its heartbeat is fresh, was
// received at least CL_ENGAGE_SETTLE_US after this gate's last CLOSED_LOOP send to
// it (a heartbeat cached before that command still reads IDLE for a Leg that is
// now holding: ACTIVATE starts from IDLE Legs), and reports another state; OR
// (FW 53 audit follow-up) its cached active_errors or disarm_reason is non-zero,
// whatever the heartbeat says: the drive's Get_Error can precede its next
// heartbeat, and the Legs must not move in that gap. Every caller runs after
// legs_cl_confirm required both 0, so a non-zero value is news from this move.
bool leg_dropped_out(uint8_t leg, uint64_t now);

// FW 53 U8 (owner: no Leg is lowered or raised while another is free): the
// ladders' Closed-loop confirmation, asked by ACTIVATE's and DEACTIVATE's COMMAND
// before any motion target goes to any Leg. For the Leg bits of `legs`:
//   FAILED    some target Leg has an error: active_errors != 0 or a cached
//             disarm_reason != 0 (both from Get_Error);
//   CONFIRMED every target Leg has a post-CL heartbeat (fresh, received >= its
//             leg_cl_cmd_us + CL_ENGAGE_SETTLE_US) reporting CLOSED_LOOP (and no
//             Leg bits at all: HAND_MOVE_TO, the Hand alone);
//   WAIT      otherwise, while fewer than CL_CONFIRM_WAIT_US have passed since the
//             latest target Leg's CL command (legs_last_cl_cmd_us);
//   EXPIRED   otherwise.
// A heartbeat cached before the CL command (an ACTIVATE after Homing: IDLE) is
// neither a confirmation nor a failure; nor is a post-CL one reporting another
// state with no error (a drive still engaging: the 400 ms bound ends a Leg that
// never engages). A Leg with no CL stamp never confirms.
enum class ClConfirm : uint8_t { CONFIRMED, WAIT, FAILED, EXPIRED };
ClConfirm legs_cl_confirm(uint8_t legs, uint64_t now);
// The latest leg_cl_cmd_us over the Leg bits of `legs` (0 if none): the start of
// the confirmation window, and the clock of the pre-motion abort it ends in.
uint64_t legs_last_cl_cmd_us(uint8_t legs);

// One tick of a gated descent to Stow, for Legs that are Closed-loop POSITION/
// TRAP_TRAJ and already commanded to STOW_OFF_POSE_REV: DEACTIVATE's MONITOR and
// ACTIVATE's RETURN (the owner's return to Stow after a mid-rise abort). The
// caller owns the state: `targets` (Leg bits), `done` (Legs IDLE'd at arrival,
// updated here), the two freeze-segment ends per Leg (the freeze segment is the
// hull of seg_a[i], seg_b[i] and STOW_OFF_POSE_REV) and `timed_out`. Precedence:
//   BUS_DOWN  fault_can_bus_down(): nothing sent;
//   DROPPED   a live target leg_dropped_out(): every OTHER live target frozen
//             (leg_freeze_tx, HELD); the dropped Leg is left alone;
//   ARRIVED   every live target in the arrival band and leg_idle_permitted(now):
//             a gated IDLE per live Leg (Who::ARRIVAL_RETRY); SENT sets its done
//             bit and clears its HELD bit. ARRIVED only once every target is done;
//             a refusal inside the loop (a frame landing between the check and
//             the wrapper) is silent, keeps RUNNING and is retried next tick;
//   TIMED_OUT `timed_out`: gated IDLE of the live Legs if leg_idle_permitted(now)
//             (SENT -> done bit), else every live Leg frozen (HELD);
//   RUNNING   otherwise.
// A Soft E-stop is deliberately not an input: once the Stow target is commanded,
// the descent is the safe motion (owner decision 1).
enum class DescentStep : uint8_t { RUNNING, ARRIVED, BUS_DOWN, DROPPED, TIMED_OUT };
DescentStep stow_descent_step(uint8_t targets, uint8_t& done,
                              const float seg_a[], const float seg_b[],
                              bool timed_out, uint64_t now);

// Diagnostic flags bits (FW 53, additive in the existing flags byte: no struct or
// MsgType change, PROTOCOL_VERSION stays 14; the host masks unknown bits). Next free
// bits after DIAG_FLAG_ACTIVATE_STALL 0x04 and DIAG_FLAG_LANE_STALL 0x08 (leg_activate.h).
// LEG_HELD: this Leg's bit in leg_held_mask() (a Leg held Closed-loop above Stow).
// RIG_BENCH: rig_is_bench() is true (set on every axis' Diagnostic).
// Host mirrors: teensy_link/activate.py, pinned by tests/firmware/test_stow_gate_xref.py.
constexpr uint8_t DIAG_FLAG_LEG_HELD  = 0x10u;
constexpr uint8_t DIAG_FLAG_RIG_BENCH = 0x20u;

// Per-Leg HELD bits (bit i = Leg i), sticky until cleared by the next accepted
// ACTIVATE/DEACTIVATE/HOME targeting that Leg or CLEAR_ERRORS(AXIS_ALL) (all wired; the
// Diagnostic carries it as DIAG_FLAG_LEG_HELD).
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
