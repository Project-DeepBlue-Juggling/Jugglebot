---
title: "Planner wire horizon derives from the emit lead (P1a): WIRE_READ_KNOTS = 3 + emit_lead_knots, with the fresh-REST rebase threshold and stall fence deliberately NOT following it; configured lead stays 0, bit-identical"
type: feature
date: 2026-11-25
status: in-progress
phase: "emit-lead P1a"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/bump.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - config/hardware_config.yaml
  - tests/motion/test_emit_lead_derivation.py
  - tests/conftest.py
  - tests/motion/test_emit_lead_one_tier.py
  - tests/ros/test_install_segment.py
  - tools/probes/skill_constants_dump.py
  - logbook/2026-11-25-emit-lead-planner-derivation.md
subsystem: [motion, ros, config]
tags: [safety, testing, docs]
---

## Summary

Emit lead 1 was half implemented: `trajectory_node._emit_once` already stamps and samples `t_k + L*dt`, and FW 22+ plays a frame on its stamp, but the planner's wire horizon `schedule.WIRE_READ_KNOTS` was the literal 3 and nothing read `trajectory_op.emit_lead_knots`. Flipping the YAML to 1 would have let a splice rewrite knots already on the wire. P1a makes the horizon derive from the configured lead and leaves the configured value at **0**, so nothing changes on the robot. P1b (test literals, lead-1 tier, contract test), P2 (probes), P3 (seed-at-the-wire-horizon) and P4 (the flip) are NOT in this change.

## Changes

- `schedule.py`: `EMIT_LEAD_KNOTS = int(hw.JB_TRAJ_EMIT_LEAD_KNOTS)` (import refuses anything but 0 or 1), `WIRE_READ_BASE_KNOTS = 3`, `WIRE_READ_KNOTS = BASE + EMIT_LEAD_KNOTS`; next to `REST_RETURN_ONSET_S` (unchanged, 0.100): `FRESH_REST_REBASE_S` and `FRESH_REST_STALL_FENCE_S`. Doc strings that quoted "9 / 13 / 17 knots", "75 ms", "225 ms" now state the formula.
- `executor.py`: re-exports the new names; the fresh-REST REBASE threshold test reads `FRESH_REST_REBASE_S`, while the rebase target and staleness keep `t_inst + WIRE_READ_KNOTS*dt` (one variable, `wire_margin_s`, had been used for both). The REBASED message prints the threshold.
- `trajectory_node.py`: the emitter reads `sk_exec.EMIT_LEAD_KNOTS` (one spelling); the fresh-REST stall fence reads `FRESH_REST_STALL_FENCE_S`; both nodes log one `wire budget: ...` INFO line at startup.
- `INVARIANTS.md`: new row I-WIRE-7; K1, I-PLAN-12, I-CATCH-4, I-CATCH-5, I-INSTALL-1 amended (BUG-95's amendments kept).
- `hardware_config.yaml`: `emit_lead_knots` comment replaced (value 0). `generate_config.py` produced no diff in the in-repo generated files (comments do not reach them); BallButler was not found beside the worktree, so it was untouched.

## Discussion

Under the naive rule "everything follows `WIRE_READ_KNOTS`" two numbers would be wrong at lead 1; the code at lead 0 cannot show why, because both derivations coincide with the naive one there.

1. **The fresh-REST REBASE threshold is 50 ms at lead 1, not 100 ms.** A fresh REST from rest samples its first knot at `tau` in `(solve + L*dt, solve + (L+1)*dt]` after its origin, so the worst gap from origin to first sampled knot is `threshold + (L+1)*dt`. That gap is pinned by `REST_RETURN_ONSET_S` = 0.100, the hand-copied budget that keeps the opening REST's Hand handover inside the firmware resume tolerance (`SCHED_RESUME_TOL_POS_HAND_REV` 0.05 rev; `REST_RETURN_ONSET_BUDGET_REV` 0.025 rev uses the onset model `_ONSET_KP*peak_a*T^2`). With a 100 ms threshold at lead 1 the onset is 150 ms and the model's displacement grows 2.25x: the 2026-09-27/28 refused-resume / E-stop class. So `FRESH_REST_REBASE_S = REST_RETURN_ONSET_S - (L+1)*dt` (0.075 at lead 0, 0.050 at lead 1). Cost, accepted: at lead 1 fresh RESTs whose solve is 50-75 ms are rebased (start up to 125 ms later); how many is unmeasured (P2).
2. **The node's fresh-REST stall fence stays 37.5 ms.** A rebased origin `t_inst + WIRE_READ_KNOTS*dt` has headroom `(WIRE_READ_KNOTS - L - 1)*dt = 2*dt` over the first sampled frame at EVERY lead; the old fence `0.5*WIRE_READ_KNOTS*dt` would be 50 ms at lead 1, i.e. zero margin. `FRESH_REST_STALL_FENCE_S = 0.5*WIRE_READ_BASE_KNOTS*dt`.

The rebase TARGET and staleness keep `WIRE_READ_KNOTS*dt` (the wire does read one knot more at lead 1); only the threshold test splits off. Rejected alternative: leave `wire_margin_s` for both and compensate at the call site (would re-create the FEAT-56 class of hand-copied stale numbers).

BUG-95's band REST needed no edit for the lead: its wire test (`k_wire = floor((t_inst - E)/dt) + WIRE_READ_KNOTS`) already reads the constant, and its slide cap (`BAND_REST_SLIDE_CAP_KNOTS` = 8) is a bound against measured slides, not a lead-derived number, so it is unchanged. The band REST branch never tests a solve against the rebase threshold (it slides by whole knots against the wire read).

## Control-cycle walk (dt 25 ms, lead 0 as configured)

One fresh REST: dispatch `t_d`, `t0 = t_d`, solve 90 ms, `t_inst = t_d + 0.090`. Old: `solve - 3*dt = 0.015 > 0` rebase to `t_inst + 0.075`. New: `solve - FRESH_REST_REBASE_S = 0.090 - 0.075` identical (the two floats are equal: `3*0.025 == 0.100 - 1*0.025`). Splice: `k_wire = floor((t_inst - t0)/dt) + WIRE_READ_KNOTS` with `WIRE_READ_KNOTS` = 3 + 0. Emitter: `lead = EMIT_LEAD_KNOTS*dt` = 0, `tau = now - t0`, unchanged. Stall fence compare `stall_s > 0.0375` (same float as `0.5*3*dt`). At lead 1 the emitter would sample u2 one knot further ahead and every splice knot would move one knot later through `WIRE_READ_KNOTS`; that is the derivation, not exercised here.

## Verification

- 2026-11-25, bit-identity: `python $S/out/emit-lead-p1a/dump.py` before and after the change (every public numeric/str constant of `schedule`, `executor`, `bump`, 159 names before): 0 values changed, 0 removed, 8 new names (`EMIT_LEAD_KNOTS`, `WIRE_READ_BASE_KNOTS`, `FRESH_REST_REBASE_S`, `FRESH_REST_STALL_FENCE_S`, each in schedule and executor).
- 2026-11-25, `pytest tests/motion/test_emit_lead_derivation.py -q`: 3 passed in 1.7 s. Fails without the change: with `WIRE_READ_KNOTS = 3` restored at lead 1 the lead-1 case fails; with `FRESH_REST_REBASE_S = WIRE_READ_KNOTS*dt` the lead-1 case fails (lead 0 passes in both mutations, as it must).
- Scoped suites, 2026-11-25, `pytest -q -n 3 -m "not nightly"` over `tests/motion/test_skills_schedule.py`, `test_skills_executor.py`, `test_skills_install_origin.py`, `test_skills_rebase_continuity.py`, `test_skills_bump.py`, `test_magazine_throw.py`, `test_emit_lead_derivation.py`, `tests/ros/test_install_segment.py`, `test_trajectory_node.py`, `test_skill_node.py` and the config and logbook readers: **1321 passed, 1 skipped in 3617 s** (run outside the shared test lock, box load about 18). The default gate on the integrated branch is cited in the commit that merges this entry.
- Default gate on the integrated branch, 2026-11-25 05:07 (bc8ada5d: main dd2328f6 + FEAT-50 + emit-lead P1a), `flock /tmp/jugglebot-tests.lock ./run_tests.sh`: **11213 passed, 59 skipped, 2 xfailed in 738.72 s**, serial tail **7 passed in 29.67 s**; box load average 4 falling to 3.

## 2026-11-25, P1b: test literals, the lead-1 tier, the contract test, P1a audit fixes

Configured `emit_lead_knots` is still **0**; nothing changes on the robot at the next launch.

**Audit fixes (no-ops at lead 0).**
- `executor.install_segment` fresh gate: `rebase_threshold_s = FRESH_REST_REBASE_S if kind == REST else wire_margin_s`. The 50 ms-at-lead-1 threshold protects the Hand's opening-REST handover only; a non-reserved fresh THROW/CATCH (sim/offline path) has no handover and keeps the wire-read margin for its `ORIGIN_TOO_LATE` refusal. I-WIRE-7 says so.
- Narrative: the `ORIGIN_TOO_LATE` docstring bullet and the *Fresh origin* paragraph name the REST threshold; the REBASED message prints the threshold only when it is the criterion (a band REST rebases because the wire read outran the running record's end); the I-INSTALL-1 seed-at-the-wire-horizon sentence is marked PLANNED (P3, not landed); this entry's "filled in below" line removed.
- `tools/probes/skill_constants_dump.py` (the bit-identity script) is committed; README row added.

**Control-cycle walk for the gate change.** Fresh REST, lead 1: solve 60 ms, `t_inst = t_d + 0.060`. Threshold 0.050: `0.060 - 0.050 > 0`, REBASE to `t_inst + 4*dt`. Fresh CATCH, same solve: threshold `wire_margin_s = 0.100`, `0.060 - 0.100 < 0`, installs at `t_now` (before the fix: refused). At lead 0 both thresholds are 0.075 (the same float), so every branch is bit-identical.

**Tests.** `tests/conftest.py` env hook `JUGGLEBOT_TEST_EMIT_LEAD_KNOTS` (sets the constant on `jugglebot.hardware_config` and `hardware_config`; unset: the `if` is false, no import, no assignment). Literals rewritten to derive (all pass at lead 0 and 1): `test_skills_bump.py` (freeze approx, windows placed relative to `BUMP_FREEZE_LEAD_S`/`CATCH_FREEZE_S`), `test_skills_schedule.py` (13/17 knots, REST lead 0.225), `test_skills_install_origin.py` (REST threshold vs wire margin, solve constants), `test_skills_rebase_continuity.py` (k 7/8, +0.160, fast solve), `test_skills_executor.py` (splice_k 25, resend window, band REST slides shifted by `EMIT_LEAD_KNOTS*dt`, +0.475), `tests/ros/test_install_segment.py` (0.325, `rest_end - 0.225`, single spelling), `tests/ros/test_trajectory_node.py` (single spelling). `test_cup_capture.py`'s `- 0.225` is the RECORDED dispatch lead of a lead-0 sitting's replay fixture and passes at lead 1: left as is, with a comment. New: consumer tests (`test_a_60_ms_solve_rebases_a_fresh_rest_only_past_its_own_threshold`, `test_the_fresh_rest_stall_fence_is_37_5_ms_at_every_emit_lead`, `test_the_emitter_stamps_each_frame_emit_lead_knots_ahead_of_its_tick`) run at lead 1 by `test_emit_lead_derivation.py::test_the_lead_one_consumers_run_the_real_code`; contract test `test_no_accepted_install_changes_a_knot_already_on_the_wire`; nightly `tests/motion/test_emit_lead_one_tier.py`. Probes `handoff_replay.py` and `throw_fresh_lead_replay.py` take `--wire-read-knots` (default 3).

**Fails without the change (mutate once, restore).** Gate reverted to one shared threshold: consumer subprocess fails (CATCH refused at lead 1). Stall fence `0.5*WIRE_READ_KNOTS*dt`: fails at the 40 ms probe. Emitter `lead = 0.0`: fails on the stamp. Contract test with `WIRE_READ_KNOTS = 3` at lead 1: fails (a `hand_next2_rev` of an already-published frame rewritten by 9.5e-9 rev); passes at lead 1 as derived and at lead 0. It needs an OFF-grid origin (a REBASED REST) plus a tick racing the install, otherwise the one-knot slack hides the revert.

## Verification (P1b)

- 2026-11-25, `python tools/probes/skill_constants_dump.py` before the first edit and after the last: 167 names, **0 changed, 0 removed, 0 new**.
- 2026-11-25, nightly tier, `flock /tmp/jugglebot-tests.lock python -m pytest tests/motion/test_emit_lead_one_tier.py -q -m nightly` (inner: `-n 3 -m "not nightly"` over the 12 files with the hook at 1; box load about 16): **974 passed, 1 failed in 653.7 s**. The one failure, `test_a_long_rest_in_the_bug95_band_installs_fresh_at_the_running_records_end`, asserted `t0 == rest_end` but the origin is rounded up onto the grid, so it flaked on the real clock's phase at EVERY lead (1 in 3 reruns at lead 0 before the fix); the assertion is now `rest_end <= t0 < rest_end + dt`, 12/12 reruns pass (6 at each lead). No non-test lead-1 defect found. The tier was not re-run end to end after that one-line fix.
- 2026-11-25, `flock /tmp/jugglebot-tests.lock python -m pytest -q -n 3 -m "not nightly" tests/motion/test_emit_lead_derivation.py test_skills_schedule.py test_skills_executor.py test_skills_install_origin.py test_skills_rebase_continuity.py test_skills_bump.py test_magazine_throw.py test_cup_capture.py tests/ros/test_install_segment.py tests/ros/test_trajectory_node.py tests/sim/test_docs_links.py` (lead 0, no env): **868 passed in 435.5 s**.
- Default gate for P1b on the integrated branch, 2026-11-25 (6278784a: main 61064fab + P1b), `flock /tmp/jugglebot-tests.lock ./run_tests.sh`: **11230 passed, 59 skipped, 2 xfailed in 770.88 s**, serial tail **7 passed in 29.28 s**.
