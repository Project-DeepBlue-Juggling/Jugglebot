---
title: "Measure first — the loop is not a 40 ms loop, and 52 ms of the seat edge was never the release"
type: feature
date: 2026-08-26
status: resolved
phase: "toss-pipelined-preamble — Phase B0"
related_plan: toss-pipelined-preamble.md
files_changed:
  - tools/probes/toss_loop_census.py
  - tools/probes/seat_edge_decomposition.py
  - tools/probes/cadence_rung_check.py
  - tools/probes/README.md
  - tests/motion/test_cadence_rung_check.py
subsystem:
  - ros
tags:
  - cadence
  - measurement
  - instrument
  - toss
---

# Measure first — the loop is not a 40 ms loop, and 52 ms of the seat edge was never the release

## Summary

Phase B0 of `plans/active/toss-pipelined-preamble.md`: three offline
measurements, no production code, taken before B1–B5 design against any number.
Two of them contradict a premise the plan was commissioned on.

**P1 — the loop census, first read** (`tools/probes/toss_loop_census.py`, new).
`NODE_LOOP_PERIOD_S = 0.040` is **not a bound**: over the 2026-08-26 evening
sitting, 53 of 73 chained cycles exceed it (p50 0.0447, p90 0.0519, max 0.0626),
and `loop_n_over_pre` is non-zero on 48 of 66 **successful** cycles — the early
warning the census was built for, firing on its first real read. `body`
dominates (40 cycles vs 32 sleep, 1 obs; p50 0.0233 against obs's 0.0026), so
B5's cheap-observation lever is the *smallest* of its three. The one cycle that
commanded a positioning move spent **0.3022 s** in a single iteration (0.2774 of
it in `body`) and died `ABORTED_CANT_MAKE_RELEASE`. Sizing, ceil-to-10 ms: the
chained loop wants **0.070 s**, not 0.040.

**P2 — the seat-edge decomposition** (`tools/probes/seat_edge_decomposition.py`,
new). The +183.9 ms splits **(a) release −1.6 / (b) flight-model +102.1 /
(c) cup seating +85.9 ms**, closing per row to 0.0001 ms. The plan's *"about
52.3 ms of it is the release itself running late"* does not survive: the mocap
back-cast puts the ball airborne within 2 ms of the announced release, and the
~48 ms is the ball still occluding the beam on its way out of the cup.

**P3 — the pipelined floor model** (`tools/probes/cadence_rung_check.py`,
`--pipeline`). Reproduces § 2.7 exactly (0.4170 at h = 1.0, 0.3941 at h = 1.3),
zero grid violations at the new floors, 196 with the loop period removed from
the commit budget. Band-floor frontier 50.6 → **56.3 throws/min**.

**What P1 does to the plan.** At the *measured* 0.070 s the h = 1.0 pipelined
floor is 0.4470 against a 0.4349 milestone — **short by 12.1 ms**, and rung P5
is refused outright. B5 is therefore not "load-bearing for the margin, not for
the milestone" (§ 3): at the loop the machine actually runs, it is load-bearing
for half the milestone.

## Implementation

* `toss_loop_census.py` — reads the ten `loop_*` record fields, **split by
  preamble class** (`position_code`), because `min_throw_delay_s` is charged at
  the chained one. Carries a provenance filter: 312 of the 659 records in
  `temp/logs/` are suite-written (`tests/ros/conftest.py::_isolate_toss_record_sinks`
  leaks), and a synthetic 0.030 s period pooled into a bound argument is a bound
  argument about pytest.
* `seat_edge_decomposition.py` — bridges bag→ROS through `t_land_bag` and
  cross-checks that bridge against the recorded `release_time_err_ms` (agrees to
  0.0002 ms) rather than assuming it. Refuses rows it cannot decompose; exits 2
  with the re-mine recipe when the corpus has no arc fits. **The four bags had
  to be re-mined without `--sensor-only`**, which returns before the mocap pass.
* `cadence_rung_check.py` — `commit_budget_s` (dispatch + ONE loop period +
  slack), `PIPELINED_LADDER` (§ 6.2 P0–P5), a modelled COMMIT tick with the
  hand-park and cup-seat evidence instants, `--pipeline` / `--loop-period` /
  `--seat-edge`. ⚠ `commit_budget_s` **does not exist in `ros_ws/`**; B4 lands
  it and must reconcile the two — the obligation is in the function's docstring.
  Two bugs found and fixed while writing it: a `loop_period_s = 0` counter-check
  spun forever (now a `ValueError`, pinned by a test), and the at-the-floor rows
  slipped a whole tick on a 5.6e-17 s rounding residual (now covered by the same
  `FLOOR_REPRESENTATION_SLACK_S` the delay floor carries).

## Verification

Probe P1, run **2026-08-27**: `python tools/probes/toss_loop_census.py --csv --json`
— 74 census-bearing records over 1 sitting, chained max **0.0626 s**, exit 1
(the corpus says 0.040 is not a bound).

Probe P2, run **2026-08-27**: `python tools/probes/seat_edge_decomposition.py --csv --json`
— reproduces the published n = 33 / +183.9 ms headline to −0.0 ms, three-way
split on 25 rows, sum-of-medians +186.4 vs +183.9 published (Δ +2.5 ms) and vs
+193.3 matched (Δ −6.9 ms), both inside the plan's 10 ms acceptance, exit **0**.

Probe P3, run **2026-08-27**:
`python tools/probes/cadence_rung_check.py --solve --frontier --pipeline --grid`
— **0 violations** serial, **0** pipelined, **196** on the counter-check, all six
§ 6.2 rungs COMMIT, exit **0**, 0.88 s.

Scoped gate, run **2026-08-27**: `pytest tests/motion/test_cadence_rung_check.py -q`
— **13 passed in 0.79 s** (4 pre-existing + 9 new).

Not run: the full suite. The orchestrator holds that gate.

## Outcome

Three things the next phases have to answer, none of them B0's to decide:

1. **B5 is promoted, or the h = 1.0 milestone is re-cut.** At 0.070 s the floor
   misses by 12.1 ms; at 0.025 s it clears by 32.9 ms, which is § 2.7's own
   number. B5 is now a prerequisite for P4/P5, not a margin improvement.
2. **Q-2's answer is "almost none".** The correctable term is the flight-time
   model at +102 ms (53 % of the seat edge), not the release. Populating
   `JB_OP_TOSS_RELEASE_LATENCY_MS` with ~52 ms would make the ball leave *early*
   by that much and buy no seat-edge correction.
3. **The record belt leaks into the operator's corpus.** 312 of 659 records in
   `temp/logs/` are suite-written despite the isolation fixture. Not fixed here
   (`tests/ros/` is another session's tree this week); both new probes filter,
   and every other reader of that directory is currently blind to it.
