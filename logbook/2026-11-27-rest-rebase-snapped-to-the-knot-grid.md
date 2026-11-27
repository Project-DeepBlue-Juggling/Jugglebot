---
title: "BUG-140 (TKT-96 option 2): an out-of-band fresh REST REBASED from a seed at rest now starts on the first Knot at or after t_inst + the wire read, so the emitter samples it on its Knots; no cap, nothing that flies today is refused, and the install line reports the slide in whole Knots"
type: bugfix
date: 2026-11-27
status: complete
phase: "BUG-140, held for a Sitting"
related_issues: [BUG-140, TKT-96]
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tests/motion/test_skills_rebase_continuity.py
  - tests/motion/test_skills_executor.py
  - tests/ros/test_install_segment.py
---

# Out-of-band REST rebase snapped to the Knot grid (BUG-140)

**What.** `executor._rebase_fresh_rest` now puts the origin of every REBASED fresh REST at `t0 = t0_before_solve + k*dt`, `k = ceil((t_inst + WIRE_READ_KNOTS*dt - t0_before_solve)/dt)` (new `executor.rebase_slide_knots`, the one spelling of that arithmetic, read by both seed branches). The in-motion branch already used exactly this `k` for its head cut; the at-rest branch used `t0 = t_inst + WIRE_READ_KNOTS*dt` itself, off the grid. No cap (owner, TKT-96: option 2, the cap is decided later from the Sittings' slide table). The `REBASED` install line keeps its text and gains, for a non-band REST, `; slide N knots on the knot grid, snapped +X s past the wire read`; the band REST keeps its own `slide N of at most 8 knots` clause.

## Discussion

**Which grid.** The emitter's grid is ABSOLUTE: `trajectory_node._ceil_to_grid` (C2FF spec, 2026-09-14) puts the emitter's ticks at whole multiples of `dt` in `perf_counter()` time (G = 0), and every hand-carrying install origin is meant to sit on a node of it, else every frame samples the plan between Knots (the C2FF measurement: 299-679 rev/s^2 Hand acceleration steps). It is not the running record's grid as such: the running record is on the absolute grid only if its own origin was (a pre-BUG-140 at-rest REBASE was not). So the reference is `t0_before_solve`, not the running record: for every non-band fresh install the node calls `_snap_t0_to_grid(t_now)` before the solve (`_svc_install_segment`), so `t0_before_solve` is an absolute grid node, and `t0_before_solve + k*dt` is one too. That is also the grid the in-motion branch already cut on, so the two branches now share one origin rule and differ only in whether the head is dropped. (Snapping to `executor._ceil_to_knot_grid` directly was the alternative; on the live path it gives the same instant, and off it, where `t_now` is not snapped, the in-motion cut can only be on the solve's grid, so the solve's grid is the one rule that holds for both branches.) The band REST is untouched: it passes `t0_wire = anchor + slide*dt`, already whole knots, and `rebase_slide_knots` returns that slide exactly (the 1e-9 guard), so its origin is bit-identical.

**One rebase, walked.** Solve outruns `FRESH_REST_REBASE_S`; `t_inst` is read in the worker; `t0_wire = t_inst + WIRE_READ_KNOTS*dt` (75 ms at emit lead 0) is the floor the wire read needs. Seed at rest: `k` as above, `t0 = t_now + k*dt`, the plan is the whole solve (`k_cut = 0`), knot 0 is checked against the held seed (unchanged check, unchanged plan, so unchanged verdict). Between install and `t0` the emitter samples `state_at(tau < 0)`, which `CyclePlan._locate` clamps to knot 0 = the held seed, so the snap adds 0 to 25 ms more of the held rest: no step, no velocity or acceleration change. From `t0` on, every tick is `t0 + j*dt`, so `tau = j*dt` and the emitter reads knot j itself, never an interpolation (the new test asserts this tick by tick). Seed in motion: `k_cut = k`, unchanged from before, so the cut index, the reference `plan.state_at(k*dt)` and the I-PLAN-12 refusal are unchanged. What changes on the wire: an at-rest REBASED REST starts up to one Knot later than it did; nothing else.

**Other readers of the origin.** `InstallResult.t0_s` and `PlanRecord.t0_s` carry the snapped origin; `event_t_s` is on the plan's own clock (a REST has none) and is unchanged. The REST's end (`record.end_s`) moves later by the same < 1 Knot; a REBASED at-rest REST already ended late by its whole slide (up to +1.015 s logged), so every reader of its end already copes with a late end, and 25 ms more is inside that class. The next Segment either splices into this record (its `k_s` is relative to `record.t0_s`, now on the grid, the case the splice arithmetic was written for) or is a band REST / fresh install anchored at `_ceil_to_knot_grid(record.end_s)`, now an exact grid node with no rounding left. The node's fresh-REST stall fence (`_reply_outran_wire`) reads only the worker's clock, not `t0`, and the snap only adds headroom over the first frame. `REST_RETURN_ONSET_S` bounds the gap from a non-rebased origin to the first sampled knot; a later origin only shortens it, assuming the firmware's handover budget is measured from the origin and not from the install instant (not checked here: read the Hand's resume timing at the first REBASED at-rest REST in the Sitting's log).

**No cap, and the edge case.** The only REBASE refusal that depends on the slide is the in-motion `ORIGIN_TOO_LATE` "rebased past its own end"; its `k` is the same ceil as before, so its boundary does not move (pinned by a test that passes on old and new code). The at-rest branch has no slide-dependent refusal at all. So nothing accepted today is refused; the brief's "REST accepted with slack < dt" case does not exist on either branch.

## Fix

- `executor.rebase_slide_knots` (new, public) and `_rebase_fresh_rest` (both branches read it); the REBASED message's appended slide clause; docstrings in `install_segment`, `schedule.py` (two comments) and `trajectory_node.py` (two comments) updated to say the origin is snapped.
- `INVARIANTS.md` K1 and I-PLAN-12: the sentences stating `t0 = t_inst + WIRE_READ_KNOTS*dt` AMENDED 2026-11-27 with the new tests.
- Tests (`tests/motion/test_skills_rebase_continuity.py`): `test_an_at_rest_rebase_outside_the_band_starts_on_the_knot_grid` (5 phases), `test_an_in_motion_rebase_cuts_on_a_knot_and_reports_its_slide` (5 phases), `test_a_one_second_slide_is_accepted_and_reported_in_whole_knots`, `test_the_in_motion_past_its_own_end_boundary_is_unchanged`; the two at-rest tests that pinned the old off-grid origin now pin `t_now + k*dt`. `tests/ros/test_install_segment.py::test_no_accepted_install_changes_a_knot_already_on_the_wire` restores the old off-grid origin through `rebase_slide_knots` (monkeypatch) so it keeps testing a splice into an off-grid record, the harder case it was written for.

## Verification

- Fail-before (2026-11-27, executor.py reverted to HEAD, `night_shift_nonet.sh env -C <worktree> timeout 900 python -m pytest tests/motion/test_skills_rebase_continuity.py -q -p no:cacheprovider -k "..."`): 15 failed, 1 passed (the edge-case pin, by design); the 5 in-motion cases fail on the missing helper and the slide clause only.
- Scoped run (2026-11-27, `night_shift_nonet.sh env -C <worktree> timeout 900 python -m pytest tests/motion/test_skills_rebase_continuity.py tests/motion/test_skills_install_origin.py tests/motion/test_skills_executor.py tests/motion/test_skills_schedule.py tests/motion/test_emit_lead_derivation.py tests/motion/test_magazine_throw.py tests/ros/test_install_segment.py -q -p no:cacheprovider -n 4 --dist loadfile`): **671 passed, 1 skipped in 687.24 s**. The default gate (`./run_tests.sh`) was not run (Night Shift unit rule); `executor.py` and `trajectory_node.py` are tier 1, so the gate is owed before merge.
- Not flown. Held for a Sitting (BUG-140's `## Sitting` section).

## Audit fixes (2026-11-28)

Owner answer TKT-99, option 1: apply both behaviour-affecting audit findings, re-audit, keep held for a Sitting. This also corrects the Discussion's claim that the band REST's origin is "bit-identical" through the 1e-9 guard: that held only at small clock values.

1. **Float rounding in the slide at a large clock.** `rebase_slide_knots` guarded `ceil` with an absolute 1e-9 knot, but the error of `(t0_wire - t0)/dt` is a few ulps of the `perf_counter` magnitude (about 2e-9 knot at 1e6 s, more beyond), so after days of uptime an exact whole-knot gap ceiled one knot late. For the band REST that put the origin one Knot past the slide `BAND_REST_SLIDE_CAP_KNOTS` had just checked. Fix: (a) `install_segment` passes the band REST's already-computed integer slide to `_rebase_fresh_rest(..., slide_knots=...)`, which uses it verbatim (no recomputation from times, so the origin is exactly the capped slide); (b) for the non-band path `rebase_slide_knots` uses a relative guard, `max(1e-9, 4*eps*max(|t0_wire|, |t0|)/dt)`, documented in its body. At small clocks both give the old values.
   **Confirmed recipe** (probe `/tmp/probe_bug140.py`, 2026-11-28, old formula `ceil((e + k*dt - e)/dt - 1e-9)`, anchors `e` = clock + a 0.0001237 s sweep, dt 25 ms): k+1 returned for k = 2, 7 at 1e6 s; k = 1, 2, 6, 7, 11 at 1e7 s; every anchor at k = 8 (the cap) for clocks 3e6 to 8e6 s, a slide of 9, one past the cap, after the cap check passed 8. Tests: `tests/motion/test_skills_executor.py::test_a_band_rest_at_a_large_clock_starts_exactly_the_slide_the_cap_checked` ((t0, slide) = (1e6, 7), (3e6, 8), (1e7, 7)), and `tests/motion/test_skills_rebase_continuity.py::test_the_slide_guard_scales_with_the_clock_magnitude` (k = 0..12 at 1e6, 3e6, 1e7 s).
2. **Float `k` in the I-WIRE-7 harness patch.** `test_no_accepted_install_changes_a_knot_already_on_the_wire` monkeypatched `rebase_slide_knots` to a float-returning lambda; an in-motion trial would have sliced with a float `k_cut`. The patch is gone: the real int helper stays and the old off-grid origin is restored by wrapping `_rebase_fresh_rest` (held, `k_cut == 0` only; each trial now asserts `'from rest'`, so the seed holds still). The loop moved to `_assert_no_wire_knot_rewritten`, and a sibling, `test_no_accepted_splice_after_an_on_grid_rebased_rest_changes_a_knot_on_the_wire`, runs the same trials WITHOUT any patch (on-grid REBASED record).

INVARIANTS.md not touched: its wording (`executor.rebase_slide_knots`, "the one spelling for both branches") still names the two seed branches of the non-band REBASE, which is true.

### Verification

- Fail-before (2026-11-28, executor.py temporarily restored to HEAD, `night_shift_nonet.sh python -m pytest tests/motion/test_skills_executor.py -q -k large_clock`): 3 failed (slides 8, 9, 8 knots for expected 7, 8, 7; the 3e6 case is the cap + 1); with the fix: 3 passed. The guard-scaling test (`-k scales_with`, 39 passed) and the sibling harness test (`-k wire`, 13 passed) pass with the fix; the guard test was not run on HEAD.
- Scoped run (2026-11-28, `night_shift_nonet.sh python -m pytest tests/motion/test_skills_rebase_continuity.py tests/ros/test_install_segment.py tests/motion/test_skills_executor.py tests/motion/test_magazine_throw.py tests/motion/test_skills_install_origin.py -q`): see the unit report for the result. The default gate was not run (Night Shift unit rule); still owed before merge.
