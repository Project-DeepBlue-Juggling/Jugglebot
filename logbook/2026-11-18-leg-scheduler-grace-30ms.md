---
title: "can-bridge FW 44: the scheduler grace becomes per lane group, legs 20 to 30 ms (cover + grace 80 ms), the Hand unchanged at 20 ms (owner decision 2026-11-18); a global 30 ms was withdrawn for the Hand's floor clip"
type: investigation
date: 2026-11-18
status: complete
phase: "Sitting 2026-11-18 follow-up"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - teensy_link/gap_monitor.py
  - teensy_link/rpc_args.py
  - tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py
  - tests/firmware/native/test_leg_interp.cpp
  - tests/firmware/test_sched_c2_twin.py
subsystem: [firmware, can-bridge]
tags: [scheduler, grace, fw-44]
---

# can-bridge FW 44: Leg scheduler grace 20 to 30 ms, the Hand stays at 20 ms

## What

`SCHED_GRACE_S` (one constant read by both lane groups) splits into
`SCHED_GRACE_LEG_S = 0.030f` and `SCHED_GRACE_HAND_S = 0.020f`
(`canbridge_config.h`). In `leg_interp.cpp` every read of the grace goes
through `sched_exhaust_us(first_or_axis)` / `sched_s_max(first_or_axis)`:
cover expiry and the C2 stop entry (`sched_advance`), the promotion handover
instant and the incoming frame's evaluation (`sched_apply`), and the playing
curve (`sched_eval_axis` -> `play_eval`, which now takes the group's `s_max`).
The leg group runs out 2T + 30 = 80 ms after the playing frame's stamp; the
Hand group still at 70 ms, and its fast-upstroke C1 stop
(`HAND_C1_STOP_GRACE_US` 14 ms, decision at 64 ms) is untouched.
FW 43 to 44, NO WIRE CHANGE (PROTOCOL_VERSION stays 14);
`EXPECTED_BRIDGE_FW_VERSION` 44.

Mirrors: the twin (`teensy_interp.py`: both constants, the same two helpers,
`play_eval(..., s_max, now)`), `teensy_link/gap_monitor.py`
(`SCHED_COVER_LEG_MS` 80, `SCHED_COVER_HAND_MS` 70, `SCHED_COVER_MS` = the
earlier of the two = 70, which the WARN latch and the STOP verdicts judge
against), the bridge node's STOP verdict text (names both covers), the
trajectory node's emit-gap WARN text, `stream_tuning`, `stop_record`,
`lane_latch_rehearsal.py` (leg cover 80), and symbol-only renames in
`hand_stall_stop.py` and `tests/hardware/hand_stroke_cut.py` (Hand constants,
values unchanged).

Static asserts: `HAND_C1_STOP_GRACE_US < SCHED_GRACE_HAND_S`;
`SCHED_GRACE_HAND_S <= SCHED_GRACE_LEG_S` (the gap monitor and verdicts rely on
the Hand's cover being the earlier); the HeartbeatT2J rx_age bucket assert now
pins the 70 ms edge to `SCHED_EXHAUST_HAND_US == 70000` with
`SCHED_EXHAUST_LEG_US >= SCHED_EXHAUST_HAND_US`. The edge keeps its meaning
("past a group's cover + grace at arrival": the Hand's, the earlier group);
an arrival at 70-80 ms counts as > 70 although the legs can still promote it.
The wire names `rx_age_le70` / `rx_age_gt70` and the `generate_udp_protocol.py`
field text ("past cover + grace") are unchanged and still true of the Hand.

## Why

Owner decision 2026-11-18, after the first analysis: "Let's keep the hand at
20 ms and bring the legs to 30 ms". At that day's Sitting two Attempts ended on
plain lateness with a forwarded lag of 21-23 ms against the 20 ms grace; the
host-GC and slow-bridge-loop ends overran by a further 2-14 ms and are not all
rescued.

## Discussion

**A global 30 ms was built first and withdrawn.** The first change moved the
single `SCHED_GRACE_S` to 30 ms. The Hand reads that grace whenever its
continued velocity at cover end + 14 ms is not upward above 50 rev/s, so the
change reached every slow-upward and every downward Hand cover end:
- the bench stroke-cut tool (`tests/hardware/hand_stroke_cut.py`, which models
  stream death with the twin) refused at 30 ms the rung it accepted at 20 ms
  (park 0.4855, top 8.75, 100 rev/s): the stream-death envelope reached
  -2.084 rev on the down leg against the 0 rev floor clip, and the default
  stroke was refused below the floor too. A Hand moving down at 70-100 rev/s
  near the bottom is carried past the floor clip before its stop starts;
- the twin's in-grace promotion invariant failed for the Hand: a frame
  promoted 25-30 ms into the grace stepped 0.076 rev against the 0.05 rev Hand
  resume tolerance;
- the live Patterns' fastest downward Hand speed near the bottom had not been
  checked.
The owner's own words had named the legs ("having the legs run for an extra
10 ms is fine"), and the legs carry neither the floor-clip risk nor the C1 path,
so the grace was made per group instead of re-deriving the bench rungs.

**The mixed case.** One frame carries all lanes, and the groups now run out at
different times. A frame arriving 70-80 ms after the playing frame's stamp is
promoted by the legs (still in their grace) and met by the Hand's stop (begun at
70 ms). The Hand's handling is exactly FW 43's: compared with the resume rule,
refused while armed if outside it (the Hand latched, `sched_refused` rises, the
host's `LANE_REFUSED` END fires, since that counter is shared by both groups,
and the node installs its hand-less hold). The legs, instead of a C2 stop from
the extrapolated state followed by a refusal and a latched hold of their own
(FW 43: legs moving faster than 0.3 rev/s are refused), keep playing the real,
admissible plan until that hold arrives. The split is not new: FW 37's C1 stop
already stops the Hand at 64 ms while the legs ride to 70, and R5 sitting 7
flew the opposite split (Hand resumed, legs latched). Judged no worse than both
stopping: the Hand's state is identical, and the legs follow a planned,
guard-checked trajectory instead of an open-loop extrapolation plus stop.

**The leg cost.** Extra travel before resting (previous probe,
`grace_probe2.py`, leg stop 250 rev/s^2 / 25000 rev/s^3): 2.8 / 3.2 / 3.5 mm
at a constant 275 / 320 / 350 mm/s, 10.0-11.8 mm with a = +5000 mm/s^2 and
j = +200000 mm/s^3 extrapolated through the grace (the session acceleration
limit is 8000 mm/s^2, so that corner can be larger). A same-plan leg frame
promoted 25-30 ms into the grace: on the twin's leg case (the Hand plan x 0.01)
the step is 0.0008 rev / 0.09 rev/s / 7 rev/s^2, inside the leg resume
tolerance (0.005 / 0.3 / 60); at the sitting jerk bound (jerk flipping +-J at a
knot, dj = 5660 rev/s^3) it is 0.015-0.026 rev / 1.8-2.5 rev/s / 140-170
rev/s^2 (1.0-1.8 mm, 125-180 mm/s), against 0.0076 rev / 1.13 rev/s / 113
rev/s^2 at FW 43's 20 ms. A promotion inside the grace is not gated by the
resume tolerance (it is counted in `promo_over`), and at FW 43 the same frame
would have met a stop, a refusal and the end of the Attempt.

## Verification

(2026-11-18, `flock /tmp/jugglebot-tests.lock python -m pytest tests/firmware tests/teensy_link tests/motion/test_hand_stream_bench_stroke_cut.py tests/motion/test_lane_latch_rehearsal.py tests/ros/test_stream_tuning.py tests/ros/test_teensy_bridge_node_bridge_diag.py -q`, 925 passed, 4 skipped in 79.78 s, run 2026-11-18; this includes the native doctest rewritten so the Hand now stops 10 ms before the Legs and its STOP record comes first, each record asserted against its own entry time).
Hand unchanged: every Hand expectation in the twin, native and bench tests
passes with its FW 43 numbers (the bench stroke-cut tests unedited; the edits
are symbol renames plus one twin test whose stop count now excludes the legs),
and the new `test_the_hand_lane_is_bit_identical_to_fw43` compares the Hand
ladder, C1 entries, phase, latch and promotion maxima with a twin carrying the
FW 43 leg grace over randomised lateness and drops. New pins:
`test_the_grace_is_per_group_legs_30_ms_hand_20_ms` and
`test_group_covers_mirror_the_header_graces_and_the_twin`.
Compile: SUCCESS in 40 s, 2026-11-18. Not flashed. Before flashing: `./run_tests.sh --full`
(CLAUDE.md case (a)), the lockstep host checkout at EXPECTED_BRIDGE_FW_VERSION
44, and BUG-93's held branch renumbered to FW 45 when it lands.
