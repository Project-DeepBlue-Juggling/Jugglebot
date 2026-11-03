---
id: JB-145
kind: bug
title: 'Hand stall stop near a release cannot fit the stroke: the 20 ms grace plus the C2 stop carry the Hand past the clip'
status: in_progress
created: '2026-11-02'
updated: '2026-11-03T06:24:23Z'
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
links: []
filed_by: jugglebot-3d
---

If the scheduled stream runs dry near a Throw's release, the can-bridge cannot stop the Hand inside its stroke. The lane continues the last span's cubic for SCHED_GRACE_S (20 ms, about 3 rev at 150 rev/s) and only then runs the C2 stop (3500 rev/s^2, 350k rev/s^3), which also cannot start below its entry acceleration.

## Evidence

Modelled with tools/probes/hand_stall_stop.py (production self-throw, release 860): a cover ending one knot before the release stops at 15.5 rev (Apex 1.05) and 20.0 rev (Apex 1.42) with the continuous release; with the release step (JB-144) the worst knot is the release itself, 14.6 and 18.5 rev. The clip is 10.501, the hard stop 10.701.

Pre-existing (not caused by JB-144, which is slightly better). It needs at least three missed frames (over 70 ms; the worst measured emit gap is 47 ms, JB-23). A clamped entry acceleration plus a 4500 stop still leaves 12.8 rev: the grace is the main term, and the planned brake already uses the stroke above the release.

A frame that loses its scheduled knot set (`HAS_SCHED` cleared by `setpoint_pump.py`) falls back to the legacy Mode 1/2/3 ladder, whose Mode-2 extrapolation from the frame accel and a jerk estimate reverses velocity when cut at a stepped release (seen in `tests/sim/test_skills_gate.py`; that test now pins the continuous shape).

## Options

To weigh (firmware, flash): drop the grace for the Hand while it moves upward fast, start the Hand stop at the cover end with a C1 entry, stop at the plan's own brake authority; the JB-114 bumper is the physical backstop. Logbook 2026-11-02-hand-release-step.

## Options (2026-11-03)

Options 1 and 2 change the can-bridge firmware (`ros_ws/src/jugglebot/Teensy_code_canbridge/`) and only matter once flashed. Option 4 runs a probe script the night may not run. Each is work for the owner's Session. The night drafted them from the body and from JB-149, which shows the trigger is real: three forward gaps over 70 ms on 2026-11-02, one of them ending in Hand MAX_DEVIATION +9.9 rev.

1. No grace for the Hand while it moves upward faster than a threshold (say 50 rev/s): the C2 stop starts at the cover end. This removes the main term, about 3 rev at 150 rev/s.
2. Option 1 plus a stop that enters from the span's current acceleration (C1 entry) and brakes at the plan's own brake authority (4500 rev/s^2, the stream STOP since 8ffeba66). This is the shortest stop the drive can give.
3. Fix the starvation instead (JB-149: probe, then a CPU partition or a 4-knot frame), and leave the stop as is. The JB-114 bumper is the physical backstop.
4. Model first: extend `tools/probes/hand_stall_stop.py` to options 1 and 2, and pick on the stop height against the 10.501 clip.

Recommendation: 4 then 2. The probe exists, it settles whether option 1 alone fits under the clip, and option 2 is one firmware change with one native test. JB-149 still matters, but a stop that cannot fit the stroke is a hazard on its own.

## Sitting 2026-11-03 findings

The FW 36 stall guard never fired: `diag_flags` bit2 (value 4) was never set in 31,389 capture rows, and the launch log holds no STALL or HAND_MOVE_STALLED line, so there was no stall stop today.

## Sitting

- setup: bridge-cycled, armed
- why: FW 37 (owner call JB-154, option 2 with a 6 ms grace) stops a fast upward Hand inside the stroke when the stream dies; it has never run on hardware, and the stroke_cut bench stage exercises exactly that path with no ball.
- do:
  - Launch down. Confirm the bridge identity reads FW 38 (proto 11). Bench Hand on the bridge, bridge rebooted after re-cabling, Hand homed and parked at or below 0.9 rev, Cup empty, E-stop in hand.
  - From the repo root in the venv: `python tests/hardware/hand_stream_bench.py --stage stroke_cut --close-loop --repeat 3` (upstroke, 3 runs at the defaults: peak 70 rev/s, cut near 57 rev/s, modelled hold 2.62 rev).
  - Only if those runs settle comfortably: `python tests/hardware/hand_stream_bench.py --stage stroke_cut --cut-at-vel 64 --close-loop` (C1 entry about 71 rev/s, modelled hold 4.45 rev).
  - Controls: `--cut-mode downstroke --close-loop` (C2 stop, hold about 4.27 rev) and `--cut-mode slow --close-loop` (full 20 ms grace then C2, hold about 2.54 rev).
  - Never Ctrl-C mid-stroke: a disarm while moving leaves the Hand up to 2 rev above the last command. Recovery is `--clear-errors`.
- pass: every upstroke run scores S1-S5 in the stage's verdict line: commanded peak at most the modelled hold + 0.1 rev and under 10.0 rev, sched_hand_c1_stops +1 and sched_stops +1 per run, no fault and no sched_refused, encoder settled within 0.05 rev of the hold within 150 ms; both controls show C1 count +0 with their hold within 0.1 rev of the model.
- record: each run's verdict line (cut frame, entry speed, modelled and measured hold, encoder overshoot, settle time, counter deltas), the peak Iq at the stop entry, any guard or latch event, the temp/logs CSV paths, and the bench Hand rig's actual current cap (not written down anywhere; the stage assumes 10 A).
- minutes: 30

## Progress

- 2026-11-03: Owner call 2026-11-03 (JB-154): probe extended, FW 37 built (no grace for a fast upward Hand becomes a 6 ms grace then a C1 stop at 4500; +0.71 rev margin at Apex 1.42), FW 38 adds the counters, stroke_cut bench stage written; flash and the bench-hand Sitting row below are owed (detail in logbook 2026-11-03-jb145-hand-stall-stop-options)
