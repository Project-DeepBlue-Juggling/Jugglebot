---
title: "Handoff-catch SPLICE_TOO_LATE: handoff lead 13 -> 17 knots (350 ms of solve budget, 113 ms over the worst recorded reply) and the catch re-aim fences re-keyed on the real splice instant so the pinned re-aim carries the correction the earlier dispatch costs; the PINNED-BUDGET floor is the raw path's own minimum budget, not the solve p90"
type: feature
date: 2026-10-16
status: in-progress
phase: "handoff-solve-chaining — P1 (implemented, not yet flown)"
related_plan: handoff-solve-chaining.md
related:
  - 2026-10-15-handoff-replay-baseline.md
  - 2026-10-14-two-ball-columns-sessions-why-attempts-end.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_schedule.py
  - tests/ros/test_install_segment.py
  - tests/ros/test_skills_plan_bench.py
  - tools/probes/handoff_replay.py
  - plans/active/handoff-solve-chaining.md
  - logbook/2026-10-16-handoff-lead-8-knots-and-snap-aware-resend-fences.md
  - logbook/INDEX.md
subsystem:
  - skill-stack
  - motion
tags:
  - timing
  - catching
  - re-aim
---

## Summary

P1a + P1b of `handoff-solve-chaining.md`, one commit. Landed, NOT yet flown.
(1) `schedule.HANDOFF_LEAD_EXTRA_KNOTS` 4 -> 8: a CATCH that follows a release is dispatched 17 knots = 0.425 s before the pinned release knot, 350 ms of solve budget (14 knots past the 3-knot wire read). `LEAD_S`, `CATCH_FREEZE_S` and `SOLVE_BUDGET_KNOTS` are untouched; `trajectory_node` is unchanged (its deadline for a pinned splice already follows `splice_cliff_s`, Fix 1).
(2) The catch re-aim fences in `SkillExecutor._resend_live_catch` are re-keyed on the instant the re-send actually splices, plus a new non-terminal PINNED-BUDGET pre-decline.

## Motivation

The 2026-10-15 replay (previous entry): Fix 1's 250 ms left 13-28 ms on the 12 recorded attempt-ending refusals. 350 ms leaves 113 ms (min) / 128 ms (median); the accepted-install minimum is 150 ms. The price: tracker-fit availability at dispatch falls from 61 % recorded to 5 % at 8 knots (live fits converge 0.2-0.3 s after release), so nearly every handoff catch now goes out on the learned prior / schedule and the tracker correction rides the re-aim. The re-aim fences therefore have to be right. At 8 knots the old ones tested `t + skill.lead_s` (= HANDOFF_LEAD_S for a handoff catch), which is neither where a snapped re-send splices nor what the node's raw knot uses; they would have closed the re-aim window ~40 ms after a p50 first install and never after a slow one.

## Discussion

Withdrawn 1: "first aims are prior-based". 74 % / 61 % of recorded first aims were tracker-aimed; the prior share is a consequence of the longer lead, not the baseline.

Withdrawn 2: "worker chaining is needed for 8 knots". The skill_node tick blocks on each install, so a due CATCH queues behind its predecessor: at most 31 ms of budget lost at the 0.200 s minimum gap, no stale read. Chaining stays the plan's conditional § 2.C.

Withdrawn 3 (audit, 2026-10-16): the PINNED-BUDGET floor was first set to the tracker-aim solve p90 (0.190 s). Declining a pinned re-send hands the catch to the RAW path, whose budget is only LEAD_S - WIRE_READ_KNOTS*dt .. + dt = 0.150-0.175 s and whose refusal is equally terminal (2026-09-20 rule). A 0.190 floor therefore discarded pinned re-sends holding 0.150-0.190 s in favour of a path with less. The rule is "never decline onto a path with no more budget"; the floor (`RESEND_PINNED_BUDGET_FLOOR_S`) is the raw path's own minimum, 0.150 s, derived rather than tuned.

Audit finding 5: the snap band's -dt/2 guard errs toward "raw". That is safe for the window fences (raw fences are later and tighter) but unsafe for the budget decline: in the half-knot where the node still snaps a re-send would go out with ~0.11 s and be refused terminally. Hence `maybe_snapped` (band + dt/2) is used for the budget check only; the cost is one tick of delay at worst.

Tradeoff accepted (owner, 2026-10-15): with the fence fix ~20 % of catches end on the prior vs ~39 % today (and ~95 % at 8 knots without it). The longer lead buys first-install margin by spending first-aim quality.

Not changed: the 10 mm / 10 ms re-send tolerance, 25 ms minimum interval, cap 2, converged-fit requirement, the 2026-09-20 terminal-refusal rule, every node refusal (SPLICE_TOO_LATE, transit, limits) and the firmware MAX_DEVIATION guard. The executor's fences only decide whether to ASK.

## Design / Implementation

- New helper `_resend_splice_instant` returns `(t_s, snapped, t_rel, maybe_snapped)`. `t_rel` = last release before the landing among the schedule's earlier skills (equals `skill.t_abs_s - skill.window_s` for a handoff catch). snapped iff `t + LEAD_S <= t_rel + n_detach*dt - dt/2` (module `LEAD_S`, never `skill.lead_s`); `t_s = t_rel` when snapped else `t + LEAD_S`.
- MIN_WINDOW (terminal) and CONTACT-WINDOW (non-terminal) now test `t_s`.
- PINNED-BUDGET (non-terminal): when `maybe_snapped` and `cliff_est - t < RESEND_PINNED_BUDGET_FLOOR_S`, `cliff_est = t_rel - WIRE_READ_KNOTS*dt - dt/2`. No cap count, `_live_catch` kept, logged once per catch.
- `INVARIANTS.md` gains row I-CATCH-4. Probe `tools/probes/handoff_replay.py` gained the fence model (columns `reaim_firable_new_fences_L*`, summary § 3b).
- Tests: `test_a_pinned_resend_window_opens_at_first_install_end_and_closes_at_the_floor[4|6|8]`, `test_the_pinned_budget_decline_is_not_terminal`, `test_a_resend_after_the_snap_band_splices_raw`, `test_the_fences_use_LEAD_S_not_the_handoff_lead`, `test_the_ambiguous_half_knot_declines_pinned_budget`, `test_the_handoff_min_gap_is_400_ms_less_a_knot_per_extra_knot` (EXTRA 2..10, abs 1e-6 because the knot dt is float32). `test_install_segment` 255/262 ms cases re-pinned to 355/362 ms (cliff-minus-dispatch 360 ms in that fixture); `test_skills_plan_bench` budget 350 ms; one existing test renamed `test_the_handoff_budget_is_the_lead_less_the_wire_read_and_the_knot_past_it_refuses` (0.340 s installs, 0.360 s refused).

Control cycle at 40 Hz: nothing on the wire changes until an install; the emitter keeps sampling the installed record through every solve. The first install splices at the same release knot from the same `release_state_at_knot` seed, only earlier relative to its cliff; a pinned re-send re-solves the same window from that same seed, as a first tracker-aimed install would. Feedforward per-knot from the installed CyclePlan is untouched; a refusal installs nothing and the rest-terminal tail runs out. New timing facts: dispatch 100 ms earlier; minimum dispatch gap 0.200 s; the tick may block up to ~31 ms longer at that gap.

## Verification

- 2026-10-16, `python tools/probes/handoff_replay.py` with the S1/S2/S8 log+bag pairs from its docstring, lead 0.425 rows: 0 negative margins on 299 accepted handoff installs, minima S1 163 / S2 159 / S8 150 ms. § 3b fence model over 216 late-fit catches: 165 pinned-firable (76 %), 49 raw path, 2 PINNED-BUDGET declined.
- 2026-10-16, `pytest tests/motion/test_skills_schedule.py tests/ros/test_install_segment.py tests/ros/test_skills_plan_bench.py -q -p no:cacheprovider`: 280 passed.
- 2026-10-16, `pytest tests/motion/test_skills_executor.py tests/motion/test_skills_schedule.py tests/sim/test_plans_index.py -q -p no:cacheprovider`: 470 passed, 2 skipped in 8.95 s.
- The full gate is recorded in the commit message.

## Outcome

Implemented, not flown. Open: the 38 ms cliff bias (derived cliff ~D+0.238 vs nominal D+0.200) stays open for the next sitting's `_refusal_timing` lines. P2 = rehearsal row on the loaded Jetson, then colcon build + launch restart.
