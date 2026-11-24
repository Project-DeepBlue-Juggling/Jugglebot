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
- The scoped suites and the config/docs tests: see the commit message and the final report of the work session (filled in below).

- Scoped suites, 2026-11-25, `pytest -q -n 3 -m "not nightly"` over `tests/motion/test_skills_schedule.py`, `test_skills_executor.py`, `test_skills_install_origin.py`, `test_skills_rebase_continuity.py`, `test_skills_bump.py`, `test_magazine_throw.py`, `test_emit_lead_derivation.py`, `tests/ros/test_install_segment.py`, `test_trajectory_node.py`, `test_skill_node.py` and the config and logbook readers: **1321 passed, 1 skipped in 3617 s** (run outside the shared test lock, box load about 18). The default gate on the integrated branch is cited in the commit that merges this entry.
- Default gate on the integrated branch, 2026-11-25 05:07 (bc8ada5d: main dd2328f6 + FEAT-50 + emit-lead P1a), `flock /tmp/jugglebot-tests.lock ./run_tests.sh`: **11213 passed, 59 skipped, 2 xfailed in 738.72 s**, serial tail **7 passed in 29.67 s**; box load average 4 falling to 3.
