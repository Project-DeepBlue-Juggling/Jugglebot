---
id: BUG-105
kind: bug
title: Hand keeps its last velocity feedforward on a Soft E-stop, disarm or link loss and stays in Closed-loop
status: in_progress
created: '2026-11-14'
updated: '2026-11-23T00:21:22Z'
owners:
- night-shift
priority: high
effort: s
hardware: bench-hand
owner: hands
radius: subsystem
diagnosis: hypothesis
area: hand
blocked_by: []
links:
- bug-motor-fb-stale-suppression-leaves-the-odrive-holding-its
- bug-can-bridge-fatal-can-silence-check-wraps-when-a-frame-is
filed_by: claude
---

On every software stop except MOTOR_FB_STALE, the last frame the Hand ODrive holds keeps its velocity feedforward, and the Hand stays in Closed-loop. Found by the 2026-11-13 maintainer review and checked on 2026-11-14 against the owner's objection that a power cut clears the terms. Related to BUG-19, whose cure (the FW 35 hold flush) was scoped to MOTOR_FB_STALE.

## Findings

- `leg_interp.cpp:1903-1918`: the vel_ff is zeroed only when `hold_flush` is set, and `s_hold_flush_ok` is set for MOTOR_FB_STALE alone (`:151-166`).
- `leg_interp.cpp:1920-1936`: the torque drain then re-sends the last frame with `hand_emit_vel = s_hand_tx_vel` and zero torque, so the stale vel_ff is actively refreshed.
- On a Soft E-stop, a disarm or a link loss nothing commands the Hand to Idle (`fault_machine.cpp:602-660`; `docs/can_bridge/safety.md:42-44` says to let the ODrive hold). The Hand's own CAN Watchdog is off.
- BUG-19's bench result on a Leg: the axis settles at `input_pos + vel_ff / pos_gain` (0.05 rev at vel_ff 2.0, gain 40). It was never measured on the Hand.

## Size on the Hand (arithmetic, not measured)

| Stroke speed at the stop | Offset at gain 35 |
| --- | --- |
| 150 rev/s | about 4.3 rev |
| 300 rev/s (the vel_ff cap) | about 8.6 rev |

The Hand deviation Guard is 2.5 rev, but it only latches the Soft E-stop, which is the state already in force. The stroke is about 10.5 rev.

## The owner's objection

A power cut does clear the terms: they are volatile, and the axis comes back Idle. That covers the E-stop button if it cuts ODrive power; no wiring document in the repo says what it cuts. It does not cover a Soft E-stop, a disarm or a link loss, where the axis never leaves Closed-loop. Whether entering Closed-loop clears vel_ff on ODrive Pro 0.6.11 is undocumented, and the open 0.5.6 source does not clear it.

## Wanted

1. A bench-Hand measurement: stop the stream mid-stroke by a disarm and record where the Hand settles. Get the bench top stop first; the bench Hand has the shorter stroke.
2. If it matches the arithmetic, widen the hold flush to every suppress edge for the Hand, or send one frame with zero vel_ff on those edges. The drain already transmits on a Soft E-stop, so the rule "transmit nothing a stop forbids" is not what blocks it.
3. Record what the E-stop button cuts. Answered 2026-11-22 (owner, bench Hand): it cuts the ODrive bus power. The robot's button is not covered by that answer.

## Already known in the tree (2026-11-14)

The property is not new: `tests/hardware/hand_stream_bench.py:150` and the 2026-11-03 logbook entry on the Hand stall-stop options both state that the bridge does not flush vel_ff on a disarm and that a disarm while moving leaves the Hand at `input_pos + vel_ff/35`. The bench driver is built to disarm only from a hold for that reason. What is missing is a measurement on the Hand and a decision on a cure.

## Sitting

- setup: bridge-cycled, armed
- why: the cure passed on 2026-11-23 (three cuts, offsets within 0.005 rev). The branch has since gained BUG-131's guard fix, so one cut on the final firmware 49 build confirms the cure before the branch is merged.
- do:
  - The cure is firmware 49 on the branch below. Launch down. Check the branch out, flash it to the bridge, and flash main's build back before the robot is launched again.
  - Bench Hand on the bridge, Hand homed and parked between 0.3 and 0.7 rev, Cup empty, E-stop in hand. The bench top stop is 10.4 rev.
  - From the repo root in the venv: `python tests/hardware/hand_stream_bench.py --stage triangle --tri-span 4 --tri-speed 10 --tri-cut up --tri-cut-disarm now --top-stop 10.4 --close-loop`. The stage sends its disarm at the cut itself and saves its tri-cut lines to a `_tricut.txt` file beside the CSV.
  - Read the line `disarm landed on a live feedforward`. On `no` or `unknown`, recover and run the cut again, up to three cuts in all.
  - Recover after each cut with `python tests/hardware/hand_stream_bench.py --clear-errors`, re-home if needed.
- pass: on a cut whose output says `disarm landed on a live feedforward: yes`, the Hand settles within 0.05 rev of the firmware's last commanded position and holds without drifting.
- record: nothing to paste: the stage names the `_tricut.txt` file it wrote. Any Guard or latch, and anything odd about the stop.
- minutes: 10
- branch: night-shift/2026-11-23-BUG-105

## Sitting result (2026-11-20)

- verdict: inconclusive
- notes: Same as Step 13; no runs were accepted. (sheet 2026-11-20-bench-hand, step 14) [derived: Not flown. Every triangle run (17 CSVs, temp/logs/hand_stream_triangle_20261120_23*.csv) was aborted by the tool's own 0.5 rev deviation belt within 50 ms of the ramp start, before any cut; no offset was measured. A tool defect, not a Hand result.]

## Sitting result (2026-11-22 22:23)

- verdict: inconclusive
- notes: owner: done; the pasted output of the 20 rev/s upward cut is in the sheet's results file. (sheet 2026-11-22-bench-hand, step 9)
- derived: Two of three cuts flown. 10 rev/s up (temp/logs/hand_stream_triangle_20261122_220628.csv, owner's paste on step 8): settled +0.2844 rev above the firmware's last command against vel_ff/35 = +0.2857, drift 0.0007 rev, fault NONE: the law holds.
- derived, continued: 20 rev/s up (hand_stream_triangle_20261122_220713.csv): the disarm landed after the lane's feedforward had decayed to 0 (echo vel_ff +0.00, the Hand 2.10 rev past the last knot), so the offset was -0.0016 against an expected 0: within the criterion but no test of the law at this speed; drift 0.0287 rev. The 20 rev/s downward cut and the E-stop button check were not flown.

## Sitting result (2026-11-22 22:49)

- verdict: pass
- notes: owner: both 20 rev/s cuts flown; the Hand was put in IDLE with odrivetool to lower it between them. The pasted stage output is in the sheet's results file. (sheet 2026-11-22-bench-hand-2, step 8)
- derived: 20 rev/s down (temp/logs/hand_stream_triangle_20261122_223621.csv): settled +0.0047 rev from the firmware's last command, echoed vel_ff 0.00, drift 0.0032 rev.
- derived, continued: 20 rev/s up (hand_stream_triangle_20261122_223818.csv): +0.0009 rev, vel_ff 0.00, drift 0.0134 rev.
- derived, continued: Both disarms landed after the lane had decayed its feedforward to 0 (the Hand ran on 2.1 rev), so the only cut with a live feedforward is the 10 rev/s one of the earlier Sitting (0.284 against 0.286). No fault or latch. The E-stop button check was not recorded. Owner at the confirmation: the E-stop button cuts the ODrive bus power on the bench Hand.

## Sitting result (2026-11-23 11:01)

- verdict: pass
- notes: owner: ran three cuts; all had 'disarm landed on a live feedforward: yes'. (sheet 2026-11-23-bench-hand, step 5)
- derived: Owner supplied the terminal lines at the confirmation, which flips the derived inconclusive to pass.
- derived, continued: Three 10 rev/s upward cuts on firmware 49 with the disarm sent at the cut (temp/logs/hand_stream_triangle_20261123_104633.csv, _104746.csv, _104809.csv), all 'disarm landed on a live feedforward: yes'.
- derived, continued: Settled offset against the firmware's last command: -0.0042, -0.0008 and -0.0003 rev, where main's build would have left +0.2766, +0.2857 and +0.2857. Owner: a clean, sharper stop.

## Progress

- 2026-11-21: Not flown at the bench Sitting of 2026-11-20: every triangle run aborted on the tool's 0.5 rev belt within 50 ms. Cause: the firmware's arm-edge recovery slew (1 rev/s cap) cannot follow a ramp that starts at the arm. Fix held on night-shift/2026-11-21-BUG-105 (0bcea663); it also widens the belt to 0.5 + speed/35, which waits for the owner.
- 2026-11-21: 2026-11-21: the triangle fix is on main (fcdb3ecc). Owner ruled the belt: 0.5 + speed/35 rev (0.79 at 10 rev/s, 1.07 at 20). The row is ready for the next bench Sitting; the sheet commands are unchanged.
- 2026-11-22: Bench Sitting 2026-11-22: the 10 rev/s upward cut settled 0.284 rev above the firmware's last command against 0.286 expected, drift 0.0007 rev. The 20 rev/s upward cut tested nothing: the disarm landed after the feedforward had decayed to 0 (the Hand ran on 2.1 rev). The downward cut was not flown; it is on sheet 2026-11-22-bench-hand-2.
- 2026-11-22: Bench Sitting 2026-11-22 (second sheet): both 20 rev/s cuts settled within 0.005 rev of the firmware's last command, but each disarm landed after the lane had decayed its feedforward to 0. The law stands on the 10 rev/s cut (0.284 against 0.286). The bench E-stop button cuts ODrive bus power. Next: design the cure, zeroing the feedforward on a disarm that lands inside the decay.
- 2026-11-23: Night Shift 2026-11-23: cure built as can-bridge FW 49, held on night-shift/2026-11-23-BUG-105: (b) 2 test gaps, (c), (f) firmware; unblocked by a day Session flash, then the bench-Hand cut
- 2026-11-23: Day Session 2026-11-23: the held branch (c212f086) gained the two tests the night's audit asked for (a Hand stream through a real Soft E-stop and HAND_STALL latch; the lane's feedforward is 0 before it idles), BUG-131's stale-flag fix, and a stage flag that disarms at the cut. Gate green on the branch. Ready for the bench.
- 2026-11-23: Passed at the Sitting of 2026-11-23: three 10 rev/s cuts with a live feedforward settled -0.004, -0.001 and -0.000 rev from the firmware's last command (0.28 on main's build); a clean, sharper stop. Not merged: the branch now also carries BUG-131's guard fix, which is unflown. One confirming cut rides the next bench sheet, then the branch merges by hand.
