---
title: "A plan seeded from rest now opens at the seed's zero rate: knot 0's finite-differenced rates are the seed's, not a one-sided stencil (BUG-152, I-PLAN-17)"
type: investigation
date: 2026-11-28
status: in-progress
phase: "Offline 2026-11-28"
related_plan: ""
related_issues: [BUG-152]
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/motion/test_seed_rest_knot0_rate.py
subsystem: [motion]
tags:
  - safety
  - planner
---

A window planned from rest (`CycleState.at_rest`: the hop entry's opening THROW, a Feed THROW, a fresh REST) carried a non-zero knot-0 rate made by `cup_realize._knot_derivative`'s one-sided stencil, and `CyclePlan.state_at(t <= 0)` streamed it beside a held position before the start time. `decompose` now takes `seed_at_rest`, and `unified_cycle._realize` passes it for an exactly-at-rest seed: knot 0's tilt, cup-axis and z-float rates are the seed's zero, every other knot is bit-identical. New invariant row I-PLAN-17 (wording pending owner approval). Hardware unflown.

## Symptoms

BUG-152 (2026-11-27, JP-11 sim gate): the fed Columns hop entry's opening THROW at Sep. 125 / Apex 0.95 had knot 0 `pose_vel` x -1.1305 mm/s, ry -0.01803 rad/s; before t0 the emitter sent equal positions with that rate, the firmware's cubic Hermite bulged each Leg by up to 8.6 um and +3.6 / -1.8 mm/s at 40 Hz for about 0.3 s, and the sim gate's Leg mirror read 1.20e-4 rev against `MIRROR_TOL_LEG_REV` 1.0e-4.

## Diagnosis

`decompose` computes the tilt rate, the cup-axis rate (inside the `arm·axis_dot` term of the xy and slider rates) and the z-float rate with `_knot_derivative`, whose knot-0 value is the second-order one-sided stencil `(-3a0 + 4a1 - a2)/(2dt)`; the rest seed's rate never reaches knot 0. The class, measured with `probe_seed.py` (wraps `unified_cycle._realize`, records `|plan.pose_vel[0] - state.pose_vel|` and the Hand rate per kind and seed type) on origin/main 47f85f31 (before) and this branch (after), 2026-11-28, scratchpad `out/bug152/`:

| Seed | Where (sim run) | knot-0 rate minus seed rate, before | after |
|---|---|---|---|
| rest: hop entry opening THROW (LAUNCH) | learner fed Columns, Sep. 125, Apex 0.95, 6 Throws | x 1.1305 mm/s, ry 0.01803 rad/s | 0 |
| rest: same, Apex 0.90 (6 Attempts, each re-enters by hop) | same | x 1.1883, ry 0.019 | 0 |
| rest: hop entry, 25-Throw learner window | learner fed Columns | x 1.1305, ry 0.01803 | 0 |
| rest: one-ball fed opening THROW (5 Attempts) | learner `--one-ball-fed` | x 0.6172, tilt 0.00984 | 0 |
| rest: fresh REST (settle) | `--reload` gate | xy 0.0708, tilt 0.00115, Hand 0.00046 rev/s | 0 |
| rest: plain t0 Columns LAUNCH, plain fed gate LAUNCH and REST | plain gate, plain fed gate | 0 (on-site start, flat first knots) | 0 |
| rest: cross-site LAUNCH (unit fixture) | `probe_min.py` | ry 0.0087..0.0214 rad/s, x up to 0.039 mm/s | 0 |
| moving: splice windows (LAUNCH, LANDING, SETTLE) | `--reload`, plain gate | up to xy 1.761 mm/s, tilt 0.0170 rad/s (seed tilt rate up to 0.0115) | unchanged |
| post-release: STEADY, SETTLE, LANDING | every gate | up to xy 2.97 mm/s, tilt 0.0266 rad/s | unchanged |

Rest seeds carry it whenever the first knots are not flat; moving and post-release seeds carry a larger stencil-versus-seed gap, but their window's knot 0 is dropped at the join (`_concat_plans` keeps the head's seam knot, re-derived centrally by `_seam_velocity` on a splice), so it is never emitted: see Discussion. The magazine THROW builds a closed-form `CyclePlan` with analytic knot rates and never calls `decompose` (outside the class). Not measured: the human-catch wait (no sim path run).

## Discussion

**Why the stencil is wrong at a rest seed, not merely imprecise.** Every tilt branch leaves its seed C1 (I-PLAN-11: the banking schedule's `start_rate`, `rest_slew`'s carried quintic, the pose planner's quintic bridge), so from rest the series starts like `c·t³`. The second-order one-sided stencil over knots 0, 1, 2 of `c·t³` is `(4·c·dt³ - 8·c·dt³)/(2·dt) = -2·c·dt²`: a rate of the OPPOSITE sign to the motion that follows. The cross-site LAUNCHes above show it (tilt series 0, -0.00024, -0.00156 rad; stencil ry +0.0116 rad/s). The fed hop entry's series 0, -0.0002, +0.00012 is not cubic-shaped but the same stencil answers for a seed whose rate is zero by construction. The value is not a better estimate of anything the machine does: the seed is at rest, and the emitter streams knot 0 before t0 as a hold.

**Approaches considered.**
1. *Fix it in the emitter or `CyclePlan.state_at(t <= 0)`* (stream a zero rate before t0): ruled out; `state_at`'s pre-start contract (knot 0's pose AND rate) is pinned by `tests/motion/test_cycle_plan.py::test_before_start_returns_the_first_knot_boundary_conditions` and the brief forbids changing it, and it would leave the plan, the gate and the wire disagreeing about knot 0 (plan == emitted == gated is the `_realize` contract).
2. *Change `_knot_derivative`'s end stencil* (e.g. first-order, or central with a ghost knot): ruled out; it is the same estimate at a moving start, and at a RELEASE or rest-TERMINAL end the one-sided stencil is the value that answers the boundary condition exactly (`_seam_velocity`'s docstring, `HELD_SLEW_LANDS_BEFORE_RELEASE_KNOTS`); changing it moves every plan's last knot.
3. *Write the seed's rate onto knot 0 for EVERY non-post-release seed* (the general form, "knot 0's rate is the seed's"): measured, not taken. On the sim gates the moving seeds are splices (the `--reload` gate's `launch moving` / `landing moving` / `settle moving`, the Columns `settle moving`), and on a splice or extend `_concat_plans` DROPS the window's knot 0; the seam knot is the head's, or `_seam_velocity`'s central re-derivation (I-PLAN-11's enforcement). So a moving seed's knot 0 is never emitted, and moving it would only change what the window's own `validate_cycle` sees at an unemitted knot, with the chance of flipping a splice accept or refuse for no wire benefit. Left as a named follow-up, not done.
4. *Taken: `decompose(seed_at_rest=True)` for an EXACTLY-at-rest seed*, decided once in `_realize`. The finite-differenced terms at knot 0 (tilt rate, the cup-axis rate inside `arm·axis_dot`, the z-float rate) are the seed's zero; the analytic cup velocity (the QP's own rest seed) stays. Exact zero rather than a tolerance, the same discipline as I-PLAN-11's `rest_slew` dispatch: `CycleState.at_rest` builds exact zeros for every fresh origin, and a splice onto a rest-terminal knot reads exact zero too (the stencil over three equal knots), where writing zero changes nothing because that knot 0 is dropped anyway.

**Tradeoff accepted.** The first interval's Hermite now starts at zero rate against knot 1's central rate; that interval's interpolated motion moves by at most `4/27·|Δv|·dt` (x 0.004 mm, ry 6.7e-5 rad for the BUG-152 numbers) and is gated as before by `validate_cycle` (numbers in Verification).

## Fix

- `cup_realize.decompose(..., seed_at_rest=False)`: when True, `axis_dot[0]`, `dz_dot[0]` and `tilt_dot[0]` are zero before the rates are assembled; every other knot and every caller passing nothing is bit-identical.
- `unified_cycle._seed_is_at_rest(state)` (not post-release, exactly zero pose, Hand and cup rate) and `_realize` passing it: the one caller.
- INVARIANTS.md: new row I-PLAN-17 after I-PLAN-16 (wording pending owner approval).
- `tests/motion/test_seed_rest_knot0_rate.py`: 7 tests.

**One control cycle (40 Hz, dt 25 ms), the hop entry's opening THROW at Sep. 125 / Apex 0.95** (`probe_walk.py`, 2026-11-28; gate limits 350 mm/s, 5000 mm/s², 200k mm/s³):
1. *Before t0.* `state_at(t <= 0)` streams knot 0: pose (-62.5, 0, 170, 0, 0, 0) with rate 0 (was x -1.1305 mm/s, ry -0.01803 rad/s). Equal positions with equal zero tangents: the firmware's cubic Hermite is exactly the held pose; the 8.6 um / +3.6 / -1.8 mm/s Leg ripple is gone. Leg velocity feedforward before t0 is 0 instead of oscillating; the legs carry no acceleration feedforward.
2. *Knot 0 to knot 1.* Hermite from (x -62.5, v 0) to (x -62.626, v -13.51 mm/s); ry from (0, 0) to (-0.0002, +0.0024 rad/s). Changing only the start tangent moves the interpolated position by at most `4/27·|Δv|·dt`: x 0.0042 mm, ry 6.7e-5 rad. Gated peaks over knots 0..2 (the first two intervals): Leg velocity 16.94 mm/s (unchanged), acceleration 1283.5 mm/s² (unchanged, 74 % margin), jerk 117,737 -> 129,638 mm/s³ (41 % -> 35 % margin), below the whole plan's own peak of 131,461 (unchanged). Hand unchanged (0.4547 rev/s, 18.13 rev/s²).
3. *Knot 1 onwards.* Bit-identical positions and rates; the plan's whole-window gate peaks are unchanged.

## Verification

Probes and outputs: scratchpad `out/bug152/` of the session that made this change (`probe_seed.py`, `probe_walk.py`, `probe_min.py`, and BUG-152's `out/jp11diag/probe_c.py` for the Leg mirror); "before" = a detached origin/main 47f85f31 worktree, "after" = this branch, both 2026-11-28.

- Leg mirror on the hop entry path (`probe_c.py --learn --pattern columns --feed-angle-deg 11.9 --feed-speed-mmps 5600 --separation-mm 125 --apex-m 0.95 --seeds 0 --target-throws 6`): opening plan 1.196e-4 -> 1.152e-7 rev, Attempt max 1.196e-4 -> 1.094e-5 (the later plans' 1.094e-5 is unchanged). Apex 0.90: 1.257e-4 -> 1.201e-5 (it had two hop entries over 6 Attempts). Band `MIRROR_TOL_LEG_REV` 1.0e-4, untouched.
- Gate verdict lines (verdict, bands, attempts, drops, makes, release error) identical before and after on: learner fed Columns Apex 0.95 and 0.90 (6 Throws), learner fed Columns 25 Throws (FAIL on `mono_apex` both before and after, not this change), learner one-ball fed (PASS, 30 makes), `--reload` (PASS), plain fed Columns gate (PASS, mirror 1.04e-5), plain `python sim/skills_gate.py --seeds 0` (PASS, 20/20, mirror 9.16e-6, the probe table identical). Catches and Landings: the makes, drops and band verdicts do not move; per-Landing positions were not diffed (unchecked).
- JP-11 rotating Columns (branch `offline/jp11-rotating` 81f81b75 with this commit cherry-picked into a throwaway worktree, 2026-11-28): `probe_d.py 10 0.95` mirror 1.204e-4 (BUG-152's `a_step10.txt`) -> 1.042e-5, trial PASS; `python sim/skills_gate.py --step-deg 10 --separation-mm 125 --apex-m 0.90 --seeds 0 --n-throws 4` mirror 1.27e-4 -> 5.84e-5 but FAIL both before and after on installs 3/4, makes 3 (a refused install, a separate JP-11 finding, not this fix).
- New tests (2026-11-28, `python -m pytest -q tests/motion/test_seed_rest_knot0_rate.py`): 7 passed on this branch; 6 failed, 1 passed (the on-site LAUNCH, already zero) with the file copied onto the origin/main worktree.
- Scoped readers (2026-11-28, `flock /tmp/jugglebot-tests.lock python -m pytest -q -n 2 --dist loadfile` over tests/motion/ test_cup_realize, test_unified_cycle, test_unified_cycle_splice, test_splice_tilt_rate_seam, test_validate_cycle, test_skills_segments, test_skills_install_origin, test_skills_rebase_continuity, test_pose_planner, test_skills_bump, test_magazine_throw, test_cycle_plan, test_seed_rest_knot0_rate, test_release_step, test_cup_contact_contract, test_skills_executor; tests/sim/ test_skills_gate, test_no_legacy_item_ids, test_logbook_front_matter, test_docs_links; tests/ros/test_install_segment): **956 passed, 2 failed in 539.40 s** at load average ~10; the two (`test_unified_cycle.py::test_the_qp_and_the_gate_stay_within_an_order_of_magnitude`, QP stage 3.35 s wall, and `test_install_segment.py::test_TI1_seven_channel_frames_reach_the_wire_and_the_flags_fall`, a frame-timing loopback) re-run alone (2026-11-28, same flock, `python -m pytest -q` on the two node IDs): **2 passed in 1.97 s**: load flakes.
- No pinned plan bytes moved: the Columns Schedule golden is a Schedule (unaffected); no golden pins a realised knot-0 rate; the one test that pins pre-start `state_at` (`test_cycle_plan.py`) uses a synthetic fixture.
- Not run here: `./run_tests.sh --full` (tier 1, orchestrator at integration), `test_logbook_index.py` (needs `python -m mapkit gen` for this entry, orchestrator).
