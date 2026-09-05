---
title: "A cycle window planned from rest was handed free fall, and the install guard could not have seen it — the boundary-condition fix, and the velocity term that closes the class"
type: bugfix
date: 2026-09-05
status: resolved
phase: "unified-7dof-planner — Phase 5 (prep)"
related_plan: unified-7dof-planner.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - tests/ros/test_unified_cycle_integration.py
  - tests/motion/test_unified_cycle.py
subsystem:
  - ros
  - motion
tags:
  - safety
  - testing
---

# A cycle window planned from rest was handed free fall

## Summary

`trajectory/plan_cycle` at `MODE_NEW` with a post-release kind (`KIND_SETTLE`,
`KIND_LANDING`) planned from a stationary machine handed the cup QP a boundary
condition describing no machine at all. Three defects, one class:

1. **The `g` fallback.** `CycleState.to_cup_state` defaults a post-release
   window's cup acceleration to `g` — right for a chain from a state measured
   just after a release, false off a terminal hold. MEASURED (2026-09-05, the
   60 mm lateral `KIND_SETTLE` at `SETTLE_CUP_Z_MM` = 689.6 mm, 1.4 s, banking
   on, session limits 250/3000/150000): knot 0 carried cup `a_z` = **−9.806
   m/s²** — apparent gravity in the cup exactly zero for one 25 ms knot, the
   seated ball weightless — knot 1 reversed to **+19.612 m/s²**, and the cup
   arced **75.42 mm upward** across a purely lateral move (hand 0.3161 →
   2.7007 rev, **2.3845 rev** of slider spent on nothing).
2. **The rest predicate that could not fire.** The fix for (1) is scoped by
   `_cycle_start_state`'s `at_rest`, and that predicate's platform bound was
   `max_step_rev / mm_to_rev / knot_dt` = **841.04 mm/s** — a *pump safety
   ceiling* read as a speed, against the **250 mm/s** a unified sitting
   commands. It was therefore true at every platform speed the machine can
   reach, so a window planned during a live move was handed EXACT zero cup
   acceleration alongside a non-zero forward-mapped velocity. It also had no
   angular term at all, and a banked carry is largely rotation.
3. **The detach-cone rows on a window that follows no release.** `plan_cycle`
   forced `post_release` from `_KIND_SHAPE`, so a SETTLE off a hold still
   assembled the equalities that pin the acceleration DIRECTION at knots
   `1..n_detach` for a ball that does not exist. MEASURED the same day: cup
   `acc_x` at knots 0..3 came out `[0, 0, 0, 0.2013]` m/s² with the rows against
   `[0, 0.1870, 0.1801, 0.1732]` without them — 50 ms of forbidden lateral
   acceleration at the head of a lateral carry, which is exactly what
   `cup_cycle.CupState`'s own docstring says the rows must not be used for.

`validate_cycle` accepts every one of those shapes, and the install-continuity
guard compared POSITIONS only, so nothing downstream refused any of them. The
first caller is UH-3, whose entire pass criterion is *"no visible ball
disturbance"*.

## Discussion

### Why the fix is three changes and not one

(1) alone was found by the Phase 5 carry probe and is recorded in
[`2026-09-05-unified-7dof-phase5-prep.md`](2026-09-05-unified-7dof-phase5-prep.md).
The audit of that change found the other two, and they are the reason the first
fix was not yet safe: (1) is *scoped by* (2), so a bound that never binds makes
the fix fire on a moving machine as well; and (3) is the same category error
one layer down — a property of the BALL being read off the window's KIND.

`post_release` is not a property of the kind. It is the claim "a ball left this
cup at the window start", and off a terminal hold that claim is false. So
`plan_cycle`'s agreement check became one-way: a state may still not claim a
release the kind cannot have had (a LAUNCH with `post_release=True` is a caller
that believes something about the ball the planner does not), but a post-release
KIND may be planned from a rest state. `_cycle_start_state` now derives
`post_release` from the same predicate it already trusted for the acceleration,
which is what finally makes its own comment — *"it stops the two branches
describing the same stationary robot two different ways"* — true.

### The velocity term is the enforcement point, and it closes a class

The tempting fix was to stop at (1)–(3). But the reason nothing caught any of
them is structural: `_install_continuity_ok` compared leg and hand POSITIONS,
and a position matches to the micron across a velocity step — which is precisely
what the emitter's v0/v1 channels then ship to the pump. Defect (2) was one
instance of that class; there was no reason to think it was the last.

So the guard grew the rate twin of each term it already had, at the same
`0.25 × STEP_BOUND_MARGIN` fraction of the per-knot bound the pump implies:
legs `0.25 × 0.80 × MAX_POSITION_STEP_REV / knot_dt` = **2.4 rev/s**, hand
`0.25 × 0.80 × hand_vel_limit_rps` = **40 rev/s**. That is a contract in the
project's sense — an invariant, one enforcement point, and a test that fails if
it is violated — rather than a fourth patch.

**It is only safe if the paths that must pass do pass.** MEASURED (2026-09-05,
`/tmp/probe_a3_replan.py`): NEW-from-rest and the EXTEND at its live plan time
both measure **0.000000 rev/s** of leg drift; the two REPLANs of the shipped
install measure 0.000000 rev/s on the legs and **7.55 / 7.47 rev/s** on the hand
— 19 % of that term's bound, and it is the solve's own duration against the
hand's acceleration rather than a discontinuity. The hand rate term is charged
only when the LIVE plan also carries a hand track: otherwise the reference is
the measured axis-6 velocity, i.e. encoder noise on a held slider, and the first
cycle install of every session would be refused for a sensor reason.

The refusal now names the term it failed on (`leg velocity drift 4.8000 rev/s >
2.4000`), because "commanded state moved during planning" told an operator
nothing about which channel moved.

## Fix

- `trajectory_node._cycle_start_state` — the rest predicate is scaled off the
  LIVE session limits, a thousandth of the commanded velocity ceiling on each of
  three channels: **0.25 mm/s** linear, **0.00114 rad/s** angular (that
  thousandth carried to the rim through `GEOM_PLAT_RADIUS_MM` = 219.075 mm) and
  **0.20 rev/s** on the hand. `post_release` follows the same predicate. One
  INFO line per seed records which branch was used and the three speeds.
- `unified_cycle.plan_cycle` — the kind/state `post_release` agreement check is
  one-way. `cup_cycle.py` is untouched.
- `trajectory_node._install_continuity_ok` — leg and hand VELOCITY terms, and
  `_continuity_detail` so five refusal messages name the failing term.
- `unified_cycle.CycleState` — docstring notes that `pose_accel` is carried but
  never read by `to_cup_state`.

## Verification

- (2026-09-05) `python -m pytest tests/ros/test_unified_cycle_integration.py
  tests/motion/test_unified_cycle.py -q` — **130 passed in 17.16 s**.
- (2026-09-05) `python sim/unified_gate.py --no-viewer` — **PASS**: SET 1
  single-toss `core_clean 26/26` (threshold 24), SET 2 two-pose beat **EXACT**
  (worst dev 4.441e-16 s), invariants pump/mirror/flags/decay all OK, 0 leg and
  0 hand lead-clamp ticks. Unchanged from before the fix.
- (2026-09-05) `python sim/cycle_gate.py` — **PASS**, binding-band PASS, parity
  EXACT, banking-beats-level 11/11, worst capture **0.000 mm**. Unchanged.
- Probes, all 2026-09-05 under the venv, offline against the production planner:
  `/tmp/probe_a2_detach.py` (the detach-row `acc_x` table above),
  `/tmp/probe_a6_freefall.py` (the free-fall table at both 750.0 and 689.6 mm),
  `/tmp/probe_a1_a3.py` and `/tmp/probe_a3_replan.py` (the bounds and the
  measured drifts).

## Outcome

The three defects are fixed and the class they came from has an enforcement
point. Six tests pin it: the rest predicate on all four channels
(`test_the_rest_predicate_is_scaled_off_the_LIVE_session_LIMITS`), the detach
rows in both directions
(`test_a_post_release_kind_from_a_rest_state_drops_the_detach_cone`), and the
velocity term's refusal and its same-origin passes
(`test_the_install_guard_charges_VELOCITY_as_well_as_position`,
`test_the_velocity_term_passes_the_same_origin_re_installs`). Nothing has been
flown; Phase 5's ladder is the first caller.
