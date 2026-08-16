---
title: BB-led two-ball juggle — sim port-readiness (BB position, catch_vel_ratio, deterministic scatter)
type: refactor
date: 2026-06-24
status: resolved
phase: "Phase 3 → Phase 4 prep"
related_plan: "bb-led-two-ball-juggle-demo.md"
files_changed:
  - sim/juggle_demo.py
  - sim/hand/trajectory.py
  - sim/ball_butler/sim.py
  - tests/sim/test_ball_butler_sim.py
  - plans/archived/bb-led-two-ball-juggle-demo.md
commits:
  - 7c922cb
subsystem:
  - sim
tags:
  - testing
  - kinematics
  - ballistics
---

# BB-led two-ball juggle — sim port-readiness

## Summary

Resumed the BB-led two-ball one-hand juggle demo (sim is at Phase 3:
33 captures / 0 drops on the 30 s run). Closed the three sim-side gaps
the plan flagged before hardware bring-up: (1) updated the Ball Butler
placement to the measured post-cutover throw position, (2) reconciled the
sim hand `catch_vel_ratio` to the hardware config value, and (3) made BB
landing scatter reproducible under a seed. All three are sim-only,
additive, and leave the MPC path untouched.

## Motivation

The simulation already juggles two balls in one hand (BB primes one ball,
Jugglebot sustains the oval pattern), but three tracked open items in
`plans/archived/bb-led-two-ball-juggle-demo.md` §6 blocked it from being a
faithful predictor of hardware:

- The BB world-frame placement was a nominal on-axis guess `(0,−1500,1500)`;
  the real, measured throw position (post BB-cutover) is off-axis.
- `sim/hand/trajectory.py` used `CATCH_VEL_RATIO = 0.9`, but
  `hardware_config.yaml` (and the Phase 1 feasibility math) use `0.6` for
  the platform/Jugglebot hand — a sim-vs-hardware fidelity gap.
- BB scatter drew from a fresh unseeded `np.random.default_rng()`, so
  `bb_scatter_mm > 0` runs were not reproducible — blocking the planned
  T-I7 catch-tolerance test.

## Changes

1. **BB placement** (`sim/juggle_demo.py`, `JuggleDemoConfig`):
   `bb_position_mm` default `(0,−1500,1500) → (−872,−630,1430)` mm (world
   frame, origin at Jugglebot's base centroid, +z up). `bb_yaw_offset_rad`
   default `π/2 → atan2(630, 872) ≈ 0.626` rad so BB's local +x points at
   Jugglebot. The number is BB's yaw-axis origin (the sim's native param);
   the ball-release point sits ~`release_l_position_mm` (150 mm) out along
   the arm, which the BB aim solver already accounts for. BB time-of-flight,
   aim (yaw/pitch/speed), and `bb_lead` all auto-recompute from this.

2. **`catch_vel_ratio`** (`sim/hand/trajectory.py`):
   `CATCH_VEL_RATIO = 0.9 → 0.6`, matching
   `hardware_config.yaml` `teensy_trajectory.catch_vel_ratio`.

3. **Deterministic scatter** (`sim/ball_butler/sim.py`,
   `sim/juggle_demo.py`): `BallButlerSim.throw_at_jugglebot` gains an
   optional `rng: np.random.Generator` param; the runner builds
   `self._rng = np.random.default_rng(cfg.seed)` and threads it through. No
   `rng` → fresh unseeded generator (legacy behaviour preserved).

## Discussion

**Why patch `catch_vel_ratio` as a constant rather than make the hand
module config-driven.** The robust-contract instinct is to delete the
hardcoded constant and have `sim/hand/trajectory.py` read
`catch_vel_ratio` from `hardware_config.yaml`, closing the drift class
permanently. I deliberately did *not* do that here. `trajectory.py` is a
standalone Python port of `Trajectory.h` with ~10 constants that mirror
the config (`INERTIA_RATIO`, `HAND_STROKE_M`, `CATCH_VEL_HOLD_PCT`, …).
Making only `catch_vel_ratio` config-driven while the rest stay hardcoded
is a half-measure that's *more* confusing than uniform hardcoding — a
reader can't tell which constants are authoritative. The full
config-drive of the hand module is genuinely owned by the
hand-trajectory-generator-overhaul side-quest, which is rewriting that
file anyway. The right scope here is: correct the one wrong value, cite
the config line as source of truth in a comment, and leave the contract
work to the plan that owns the rewrite. (The user confirmed 0.6 is
hardware-validated as reliable and that they'll tune it over time.)

**Why the `catch_vel_ratio` change didn't move the catch rate.** Changing
0.9 → 0.6 lengthens the catch *stroke duration* (the hand descends more
slowly relative to the ball), which I expected might desync the pattern.
It didn't: the catch *target* is computed from
`HandCatchTrajectory.sample(0.0)` — the slider position at the velocity-
hold midpoint — which is ~198 mm independent of the velocity ratio (the
ratio scales the stroke's *speed*, not the geometric midpoint). So the BB
aim point and catch-pose geometry are unchanged; only the timeline's catch
stroke timing shifts, which the schedule absorbs. Empirically the demo
holds 33/0 before and after.

**BB position as yaw-axis origin vs release point.** `(−872,−630,1430)` is
treated as BB's yaw-axis origin because that is `BallButlerSim`'s native
placement parameter. The ~150 mm arm offset to the actual release point
is within the "~" of the measurement and is auto-compensated by the aim
solver (which targets the catch cup regardless of standoff). If a future
mocap calibration gives the exact yaw-axis location, it drops straight in.

## Verification

- **Demo (headless, 30 s, `python sim/juggle_demo.py --duration 30
  --no-log`, run 2026-06-24):** 33 captures, 0 drops — unchanged from the
  pre-change baseline (also 33/0), confirming the off-axis BB and slower
  catch stroke don't break the pattern.
- **Changes are live (not cached/stale):** `CATCH_VEL_RATIO == 0.6`; the
  new BB position yields TOF 0.815 s and an off-axis release point
  `(−790,−698,1586)` vs the old `(103,−1460,1648)` / 0.828 s.
- **Determinism:** same seed → byte-identical release positions; different
  seed → different; no `rng` → varies (legacy preserved). Two end-to-end
  `--seed 7 --scatter-mm 20` runs both give 21/0. New regression test
  `tests/sim/test_ball_butler_sim.py::TestConvenience::
  test_scatter_reproducible_with_seeded_rng`.
- **Suite (`pytest tests/sim/ -q`, run 2026-06-24):** 944 passed, 1 failed,
  4 skipped, 1 xfailed in 593.16 s. The single failure
  (`test_hot_loop_allocation_contract`) is the known order-dependent
  `tracemalloc`-baseline allocation flake — it concerns the MPC hot loop
  (untouched by this change) and passes 3/3 in isolation. Not introduced
  here.

## Outcome

The two §6 open items (`catch_vel_ratio` discrepancy; BB scatter
non-determinism) are RESOLVED and the BB placement matches the measured
hardware throw position. The sim demo is now a faithful predictor for
Phase 4 hardware bring-up. The hardware-orchestrator-form decision
(ROS2 node vs standalone) remains the one open §6 item before Phase 4.

## Open Questions

- Exact BB yaw-axis origin from mocap (current value is the measured
  throw position treated as the yaw-axis origin; ~150 mm arm offset is
  auto-compensated by the aim solver).
- `catch_vel_ratio` may be tuned further once hardware catches are
  observed (user flagged 0.6 as a reliable starting point, not final).
