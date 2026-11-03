---
id: JB-145
kind: bug
title: 'Hand stall stop near a release cannot fit the stroke: the 20 ms grace plus the C2 stop carry the Hand past the clip'
status: inbox
created: '2026-11-02'
updated: '2026-11-02T18:15:06Z'
owners: []
priority: medium
effort: m
hardware: none
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

