---
title: "BUG-149: a CATCH install's CPU grew about 1 ms per 100 record Knots because splice re-measured every mark's twin; only the marks near the seam are re-measured now (splice_at 24.1 -> 12.2 ms at 1279 Knots, no emitted byte changed)"
type: investigation
date: 2026-11-28
status: complete
phase: "Night Shift"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - tests/motion/test_splice_twin_remeasure.py
  - logbook/2026-11-28-catch-install-cpu-growth.md
  - logbook/INDEX.md
subsystem: [skills, planner]
---

## Summary

Hypothesis from the 2026-11-27 debrief (the splice copies the whole record head) is **refuted in its letter, confirmed in its place**: the growing step is inside `splice_at` (the install-only step), but it is not the array copy (numpy slices and `vstack` of 1300 rows are microseconds). `extend` re-measured the `stroke_clear_s` of every release and the `arm_lead_s` of every catch on the joined plan, each a hand-velocity scan plus a ~40-step bisection through `CyclePlan.hand_at`. Marks accumulate with the Attempt (one release and one catch per ~46 Knots), so the cost is linear in record length: the profile at 677 Knots showed 31 `_zero_crossing_s` calls (1250 `hand_at` calls) inside one `splice_at`.

## Measurement (offline, Python 3.8 venv, `OMP_NUM_THREADS=1`, `time.process_time`, cross-site CATCH+throw chain of ovals, driver `/tmp/probe_bug149c.py`, uncommitted)

| record Knots | `plan_segment` CPU before | `splice_at` CPU before | `splice_at` CPU after |
|---|---|---|---|
| 29 | 52.8 ms | 11.1 ms | 10.7 ms |
| 584 | 53.9 ms | 18.3 ms | 11.3 ms |
| 1001 | 53.3 ms | 21.7 ms | 11.6 ms (at 955) |
| 1279 / 1325 | 51.3 ms | 24.1 ms | 12.2 ms (at 1325) |

Everything else in `install_segment` stayed at about 0.5 ms and `plan_segment` stayed flat, matching the Sitting (growth only in an install-only step). The 2000 to 3000 Knot range was not reached: under box load (load average ~38) the multithreaded BLAS made the solves unusable for timing; the linear trend above is what the fix removes. The residual ~1 ms rise to 1325 Knots is `np.vstack`/`CyclePlan` finiteness scans over the whole joined arrays (O(n) numpy, not Python loops).

## Discussion

The debrief hypothesis ("the splice copies the whole record head") was refuted in mechanism. Ruled out: the array copy (numpy slices and `vstack` of ~1300 rows cost microseconds) and `plan_segment` (flat at about 50 ms across record length, see the table). What grew is the twin re-measure inside `extend`, which `splice_at` calls: one hand-velocity scan plus a ~40-step bisection through `CyclePlan.hand_at` per accumulated mark. Bounding the re-measure was chosen over trimming the record because trimming changes what the joined plan and every downstream reader see (a behaviour change on the emitter path), whereas the bounded re-measure leaves every emitted byte identical and is pinned by an exact comparison against the old full re-measure.

## Fix

`extend` (`unified_cycle.py`) now re-measures a HEAD mark only when its measurement can read a knot at or past the seam (`_release_twin_needs_remeasure`, `_catch_twin_needs_remeasure`): the forward scan of `plan_stroke_clear_s` is decided by knots at or below its stop knot, the backward scan of `plan_arm_lead_s` by knots at or below the catch knot, and the joined plan is bit-identical to the head below the seam knot. A release whose old twin is `None` (no stop inside the head) is always re-measured, as are all marks of the new window. One knot of float slack each. Only meta diagnostics (`stroke_clear_s`, `arm_lead_s`) are involved; the readers are `magazine_throw.py` (its own call), the `CycleMeta.arm_lead_s` property (`unified_cycle.py` ~1125, reads `catches[0]`), the meta-level `stroke_clear_s` carried unchanged by `extend`, and tests; the last two are unaffected. No plan array, Knot time, splice knot or refusal is touched.

Control implications: none on the emitter path. Plans, splice point, seam velocity re-derivation and the bounded gate are unchanged; the SPLICE_TOO_LATE check reads the solve time, so this lowers the install CPU (12 ms at 1279 Knots, measured offline: 24.1 -> 12.2 ms; the +12 to +15 ms recovered at Throw 32 is an extrapolation from that table, not measured at Throw 32) and can only turn late refusals into accepts.

## Verification (2026-11-28)

- `OMP_NUM_THREADS=1 python -m pytest tests/motion/test_splice_twin_remeasure.py -q`: 3 passed in 7.17 s (before the 2026-11-29 additions; the file now has 8 tests). The determinism tests recompute the old path (full re-measure on the joined plan) as the reference after each of 14 installs (over 100 marks) and compare exactly; the scaling test counts twin measurements per install.
- Fail-before (source change stashed, same command): scaling test fails, `[9, 11, 13, 15, 17, 19, ...]` measurements per install, max 35 > 8; determinism tests pass on both (they pin that no value changed).
- Scoped regression, `OMP_NUM_THREADS=1 python -m pytest tests/motion/test_unified_cycle.py tests/motion/test_unified_cycle_splice.py tests/motion/test_splice_twin_remeasure.py tests/motion/test_skills_bump.py tests/ros/test_install_segment.py tests/motion/test_skills_rebase_continuity.py tests/motion/test_splice_tilt_rate_seam.py -q -m "not nightly and not serial"`: 261 passed in 122.18 s.
- Audit findings of 2026-11-28 (two test-strength items, closed 2026-11-29, see below): the seam assertion in `test_the_streamed_head_is_bit_identical_below_the_seam` is vacuous and compares only pose and hand_rev, and the two helpers have no direct boundary tests.
- `./run_tests.sh` was NOT run (Night Shift unit rule); the gate decides the merge.

## Night Shift 2026-11-29: test strength

Applied the audit's two test-strength findings, test code only (`tests/motion/test_splice_twin_remeasure.py`; `unified_cycle.py` untouched). (1) `test_the_streamed_head_is_bit_identical_below_the_seam` now takes the real splice knot from `InstallResult.splice_k` (the chain fixture returns it), asserts the head really was truncated (`k_s < n_old - 1` on every install), and compares pose, pose velocity, hand position and hand velocity bit for bit below `k_s`, plus the pinned position of the seam knot. (2) Direct boundary tests of `_release_twin_needs_remeasure` (None always re-measured; a release at the seam; stops at n_a-3 skip, n_a-2, n_a-1, n_a re-measured) and `_catch_twin_needs_remeasure` (int(t/dt) = n_a-1, n_a-2 re-measured; n_a-3 and below skip), and a seeded (50-case) property check that whenever a helper says skip, the carried twin equals a full re-measure on the new joined plan.

Planted-defect checks (each restored afterwards): release boundary off by one (`(n_a-3)` to `(n_a-2)`) fails the two release boundary tests; release helper returning False for None and catch helper skipping up to n_a+5 fails the determinism test, the None test, the catch boundary test and the property test (4 failed, 4 passed).

Verification (2026-11-29): `night_shift_nonet.sh pytest -q tests/motion/test_splice_twin_remeasure.py tests/motion/test_unified_cycle.py`: 95 passed, 1 failed in 799 s on a loaded box (24 concurrent pytest processes). The one failure, `test_unified_cycle.py::test_the_qp_and_the_gate_stay_within_an_order_of_magnitude`, is a wall-clock ratio test; `unified_cycle.py` is byte-identical to the branch head (restored after the plants), so it is load, not this change. All 8 tests in `test_splice_twin_remeasure.py` pass. `./run_tests.sh` was NOT run.
