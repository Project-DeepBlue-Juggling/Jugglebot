---
id: BUG-55
kind: bug
title: 'Hand stall stop near a release cannot fit the stroke: the 20 ms grace plus the C2 stop carry the Hand past the clip'
status: in_progress
created: '2026-11-02'
updated: '2026-11-07T06:39:19Z'
owners:
- jugglebot-50
priority: medium
effort: m
hardware: bench-hand
owner: decision
radius: normative
diagnosis: confirmed
area: can-bridge
blocked_by: []
links:
- logbook/2026-11-04-sitting-debrief.md
filed_by: jugglebot-3d
---

If the scheduled stream runs dry near a Throw's release, the can-bridge cannot stop the Hand inside its stroke. The lane continues the last span's cubic for SCHED_GRACE_S (20 ms, about 3 rev at 150 rev/s) and only then runs the C2 stop (3500 rev/s^2, 350k rev/s^3), which also cannot start below its entry acceleration.

## Evidence

Modelled with tools/probes/hand_stall_stop.py (production self-throw, release 860): a cover ending one knot before the release stops at 15.5 rev (Apex 1.05) and 20.0 rev (Apex 1.42) with the continuous release; with the release step (FEAT-36) the worst knot is the release itself, 14.6 and 18.5 rev. The clip is 10.501, the hard stop 10.701.

Pre-existing (not caused by FEAT-36, which is slightly better). It needs at least three missed frames (over 70 ms; the worst measured emit gap is 47 ms, BUG-4). A clamped entry acceleration plus a 4500 stop still leaves 12.8 rev: the grace is the main term, and the planned brake already uses the stroke above the release.

A frame that loses its scheduled knot set (`HAS_SCHED` cleared by `setpoint_pump.py`) falls back to the legacy Mode 1/2/3 ladder, whose Mode-2 extrapolation from the frame accel and a jerk estimate reverses velocity when cut at a stepped release (seen in `tests/sim/test_skills_gate.py`; that test now pins the continuous shape).

## Options

To weigh (firmware, flash): drop the grace for the Hand while it moves upward fast, start the Hand stop at the cover end with a C1 entry, stop at the plan's own brake authority; the IDEA-17 bumper is the physical backstop. Logbook 2026-11-02-hand-release-step.

## Options (2026-11-03)

Options 1 and 2 change the can-bridge firmware (`ros_ws/src/jugglebot/Teensy_code_canbridge/`) and only matter once flashed. Option 4 runs a probe script the night may not run. Each is work for the owner's Session. The night drafted them from the body and from BUG-58, which shows the trigger is real: three forward gaps over 70 ms on 2026-11-02, one of them ending in Hand MAX_DEVIATION +9.9 rev.

1. No grace for the Hand while it moves upward faster than a threshold (say 50 rev/s): the C2 stop starts at the cover end. This removes the main term, about 3 rev at 150 rev/s.
2. Option 1 plus a stop that enters from the span's current acceleration (C1 entry) and brakes at the plan's own brake authority (4500 rev/s^2, the stream STOP since 8ffeba66). This is the shortest stop the drive can give.
3. Fix the starvation instead (BUG-58: probe, then a CPU partition or a 4-knot frame), and leave the stop as is. The IDEA-17 bumper is the physical backstop.
4. Model first: extend `tools/probes/hand_stall_stop.py` to options 1 and 2, and pick on the stop height against the 10.501 clip.

Recommendation: 4 then 2. The probe exists, it settles whether option 1 alone fits under the clip, and option 2 is one firmware change with one native test. BUG-58 still matters, but a stop that cannot fit the stroke is a hazard on its own.

## Sitting 2026-11-03 findings

The FW 36 stall guard never fired: `diag_flags` bit2 (value 4) was never set in 31,389 capture rows, and the launch log holds no STALL or HAND_MOVE_STALLED line, so there was no stall stop today.

## Sitting

- setup: bridge-cycled, armed
- why: FW 37 (owner call TKT-31, option 2; grace 6 ms at FW 37, 14 ms at FW 39) stops a fast upward Hand inside the stroke when the stream dies; it has never run on hardware, and the stroke_cut bench stage exercises exactly that path with no ball.
- do:
  - Launch down. Confirm the bridge identity reads FW 40 (proto 12). Bench Hand on the bridge, bridge rebooted after re-cabling, Hand homed and parked at or below 0.9 rev, Cup empty, E-stop in hand.
  - Measure where the bench top stop is in rev. The defaults need it at or above 9.7 rev (stroke top 8.5, worst stop 8.66, plus the stage's 1.0 rev margin), and the stage only checks the production 10.501 clip. If it is lower, stop: this row needs a shorter stroke plan first.
  - From the repo root in the venv: `python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --repeat 3` (upstroke, 3 runs at the defaults: peak 70 rev/s, entry near 63 rev/s at the 14 ms grace, modelled hold 3.18 rev from a 0.2 rev park and 3.88 from 0.9, and the stage prints its own; FW 37 at 6 ms was 56.8 rev/s and 2.62 rev).
  - Only if those runs settle comfortably: `python tests/hardware/hand_stream_bench.py --stage stroke_cut --cut-at-vel 64 --close-loop` (C1 entry about 73 rev/s, modelled hold 5.06 rev from a 0.2 rev park and 5.76 from 0.9; 6 ms was 4.45 rev).
  - Controls: `--cut-mode downstroke --close-loop` (C2 stop, hold about 4.27 rev) and `--cut-mode slow --close-loop` (full 20 ms grace then C2, hold about 2.54 rev).
  - Never Ctrl-C mid-stroke: a disarm while moving leaves the Hand up to 2 rev above the last command. Recovery is `--clear-errors`.
- pass: every upstroke run scores S1-S5 in the stage's verdict line: commanded peak at most the modelled hold + 0.1 rev and under 10.0 rev, sched_hand_c1_stops +1 and sched_stops +1 per run, no fault and no sched_refused, encoder settled within 0.05 rev of the hold within 150 ms; both controls show C1 count +0 with their hold within 0.1 rev of the model.
- record: each run's verdict line (cut frame, entry speed, modelled and measured hold, encoder overshoot, settle time, counter deltas), the peak Iq at the stop entry, any guard or latch event, the temp/logs CSV paths, and the bench top stop in rev. The bench Hand's current cap is the same as production (owner, 2026-11-07), so the stage's 10 A assumption is void and its drive may follow the 4500 rev/s^2 stop more closely than modelled.
- minutes: 30

## Progress

- 2026-11-03: Owner call 2026-11-03 (TKT-31): probe extended, FW 37 built (no grace for a fast upward Hand becomes a 6 ms grace then a C1 stop at 4500; +0.71 rev margin at Apex 1.42), FW 38 adds the counters, stroke_cut bench stage written; flash and the bench-hand Sitting row below are owed (detail in logbook 2026-11-03-jb145-hand-stall-stop-options)
- 2026-11-04: 2026-11-03 Sitting (22:29, P4 ramp): the FW 37 C1 stop FIRED on the robot twice (goal 9 THROW launch brake 117 -> ~4 rps in 60 ms at -24 A, rest 8.71 rev; goal 15 launch start 19 -> 0 rps in 40 ms at -23 A, rest 1.69 rev; clip 10.501; sched_hand_c1_stops +1 each), stopping the Hand inside the stroke with no overshoot. The bench S1-S5 pass is still owed.
- 2026-11-05: C1 stop fired on the robot 3x 2026-11-05, all mid-upstroke on host forward gaps 40-44 ms; fit the stroke (hold 4.4-5.0 rev) but each firing ended the Attempt. hand_stall_stop.py option 2, release-knot stop end vs 10.501 clip by grace: 6 ms ok to Apex 1.42; 10 ms Apex 1.35 at 0.00 margin; 14 ms Apex 1.00 +0.47, 1.10 +0.08, 1.35 -0.93; 18 ms Apex 1.00 -0.25. FW 39 (C1 grace 14 ms).
- 2026-11-07: 2026-11-07: owner: bench Hand matches production except a shorter stroke, same current cap. The row now asks for the bench top stop first: its defaults need 9.7 rev or more, and the stage only checks the production clip.
