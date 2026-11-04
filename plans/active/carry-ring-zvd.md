---
title: Carry ring suppression — zero-residual-vibration rows on the pose carry so the Platform is still at release
created: 2026-11-04
status: active
owner: Harrison
last_updated: 2026-11-04
pathway: two-ball-oval
scope: >-
  The one-ball oval's carried Throws land +76 mm (dwell 0.40) and +122 mm (0.35) long because the pose STEADY
  window's minimum-energy carry decelerates into the release knot and the Platform's ~6 Hz lateral mode is
  still swinging forward at separation (JB-102; logbook/2026-11-04-sitting-debrief.md § Root cause). Identify
  the mode from the bags already on disk, add zero-residual-vibration (ZVD) equality rows on the modelled
  Platform residual at the release knot in cup_cycle._pose_augment behind a config flag, re-sweep the box with
  the flag on, and fly it A/B on the oval. Closes when carried and from-rest Throws land within 10 mm of each
  other at dwell 0.40 and the oval holds 30 consecutive Catches there.
steps:
  - {id: P0, title: Identify the Platform mode from existing bags, kind: phase, status: not_started, effort: s, hardware: none}
  - {id: P1, title: The ZVD contract and rows behind a flag, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: P2, title: Box re-sweep with the rows on, kind: phase, status: not_started, effort: s, hardware: none}
  - {id: P3, title: A/B Sitting on the oval, kind: phase, status: not_started, effort: s, hardware: robot}
related_logbook:
  - 2026-11-04-sitting-debrief.md
  - 2026-10-31-hold-ladder-sitting-the-hold-model-was-wrong.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py::_pose_augment
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py::_assemble
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - config/hardware_config.yaml
  - tools/admissible_sweep_run.py
---

# Carry ring suppression (ZVD on the pose carry)

Map Item: JB-102 (`map/feature-pre-release-hold-rung-release-from-a-moving-platform-hold-0.md`), under the
two-ball-oval Pathway. This plan is the root-cause fix ranked first by the 2026-11-04 offline investigation;
it sits beside two-ball-oval P4 (whose 30-Catch gate it unblocks) and ahead of P5 (two balls need dwell
0.30-0.35, where the ring is largest).

## 1. The problem, in one paragraph

`compile_one_ball` sets `t_release = t_catch + dwell_s`, so the 150 mm carry from the +75 catch to the -75
release fits inside the Dwell. The folded STEADY window is one QP (`cup_cycle._assemble`: mean-square cup
acceleration; `_pose_augment`: leg jerk on the composed pose) with the release triple pinned. Its
minimum-energy carry uses the whole Dwell and decelerates into the release knot: commanded Platform v_x is
+0.1 mm/s at release at every Dwell, with Platform x deceleration 2.6 / 4.4 / 5.9 m/s^2 at Dwell 0.50 /
0.40 / 0.35. The legs carry no acceleration feedforward, so the Platform lags during the deceleration and
overshoots forward when it ends; at separation (+20..+40 ms) it runs +39 / +79 mm/s (encoder) faster than
commanded at 0.40 / 0.35, and the ball's launch velocity error matches it about 1:1. Nothing in the program
knows the mode exists. It is not a defect; the plan meets "still at release" in the command and not in the
plant.

**Measured (2026-11-03, 75 caught Throws):** raw landing +47.8 +/- 15.1 mm from rest, +76.2 +/- 17.5 mm
carried. **Model A** (6 Hz, damping 0.17, 15 ms delay, driven by commanded acceleration) puts the ring's
share at +24 / +51 / +79 mm at 0.50 / 0.40 / 0.35 and reproduces two of three 2026-10-31 hold-ladder
differences within ~15 %. A ~+22 mm common offset, also on Throws from rest, is NOT the ring and is out of
scope here (§ 6).

## 2. Decisions this plan takes (owner can overturn any)

1. **Fix the cause in the planner, not the aim.** Catch outward authority 30 mm was declined as a band-aid
   (owner 2026-11-04); learner authority 60 mm and a carried flag in the learner context only move the mean,
   not the scatter the ring adds to separation jitter. `pre_release_hold_s` 0.050 is phase compensation and
   stays an experiment, not the fix.
2. **Hard equality rows, with a pre-registered soft fallback.** The rows say the modelled residual at release
   is zero; a window that cannot meet them is refused, honestly, and the box shows which cells that costs.
   **Fallback, decided now:** if P2 loses the oval cell at dwell 0.40 / Apex 1.00 or leaves it under 20 mm
   of x room, switch to a quadratic penalty on the same mode state (weight chosen so the 0.40 residual is
   under 10 mm/s) and re-sweep. The penalty never refuses, so it is the weaker contract.
3. **Mode from bags first; a Sitting only if the bags disagree.** The 2026-10-31 hold rungs command the
   Platform still for 50-100 ms before release, which is a free-decay window; 2026-11-03 adds 103 carried
   releases. If the fits agree (P0's gate), no identification Sitting is needed.
4. **Flag off by default, one switch.** `hardware_config.yaml` `trajectory_op.carry_zvd` false until P3. The
   edit to `cup_cycle.py` re-hashes the box whatever the flag says, so P2 sweeps with the flag ON (the state
   that will fly). P2 also decides whether the box stamp records the flag, so that a flag-off launch refuses
   on the stamp mismatch instead of flying cells swept for the other setting.
5. **Every pose STEADY window, not only the oval.** The mode does not know the pattern. Columns pose windows
   get the rows too; P2's sweep and P3's Columns canary check that nothing regressed.

## 3. Phases

### P0: Identify the Platform mode from existing bags

- New probe `tools/probes/platform_mode_fit.py` (committed: it is reused at every re-identification). Input a
  bag; output per axis (x, y), per source (encoder FK of `/robot_state`, mocap `Platform` body): natural
  frequency, damping, input delay, and the fit residual.
- Two estimators, both reported: (a) free decay in the hold window (2026-10-31 rungs: hold 0.100 and 0.075 at
  Dwell 0.45, hold 0.050 at 0.50); (b) forced response, commanded acceleration (FK of `/leg_cmd_executed`) to
  measured Platform error, over the carry (2026-11-03, 103 carried releases). The debrief's median-profile fit
  came out over-damped (5.1 Hz, damping 0.76) because the median smears the ring; fit per release and report
  the distribution.
- Bags: `~/Desktop/rosbags/2026-10-31_13-06-17`, `_13-13-19`, `_13-14-00`, `~/Desktop/rosbags/2026-11-03_22-29-00`.
  Reuse the debrief's probes in `/tmp/claude-1000/.../scratchpad/debrief/oval/` (fk.py, fk2.py) and
  `.../scratchpad/ring/` (fitmode.py) as starting points; they are not committed and rot at reboot, so the
  committed probe absorbs what it needs.
- **Gate:** the two estimators agree on frequency within +/-10 % and the encoder fit predicts the 2026-11-03
  carried excess at 0.40 and 0.35 within 15 mm/s. **If not:** fly the identification row (dwell 0.40, hold 0
  then hold 0.050, 30 Throws each, mocap and encoders) as JB-102's `## Sitting` section before P1 lands.
- Also report the mocap / encoder excess ratio (~1.8 on 2026-11-03): if the structural part (compliance
  beyond the motors) has a different frequency from the servo part, the rows need two modes (§ 5 risk 1).
- Output: the values for `trajectory_op.platform_mode_*` and a short logbook entry.

### P1: The ZVD contract and rows behind a flag

Contract first (CLAUDE.md "contracts over patches"):

- **INVARIANTS.md, new I-PLAN-16** (normative text, then code): *With `carry_zvd` on, a pose STEADY window's
  modelled Platform residual at the release knot is zero per lateral axis: the mode state (e, e') of
  `e'' + 2*zeta*w*e' + w^2*e = a_cmd(t - delay)`, driven by the window's composed Platform acceleration and
  started from the mode state its seed carries in, together with its derivative in w (the ZVD robustness
  rows). A window that cannot meet the rows is refused `RING`, never clipped.* Enforcement point:
  `cup_cycle._pose_augment`. Test: `tests/motion/test_cup_zvd.py`.
- **Rows.** In pose mode the tilt series is prescribed, so Platform x = cup_x - (cup_z - c) * axis_x(t) is
  affine in the jerk variables (the same composition the leg-jerk objective already builds). The mode state
  at the release knot is a fixed linear functional of the Platform acceleration samples (a convolution with
  the mode's impulse response, discretised at the 25 ms knots with the 500 Hz Hermite's sub-steps), so the
  rows are four dense equality rows per lateral axis (e, e', de/dw, de'/dw), eight in all, appended after
  block 4 of `_pose_augment`.
- **The seed's mode state.** The window does not start at rest: the catch and the previous window's motion
  leave a ring. The rows' right-hand side carries the mode state propagated from the commanded acceleration
  history before knot 0 (the installed plan's own samples, the same source I-PLAN-11's splice seed uses).
  The catch impact (ball momentum) is not commanded and is not modelled; P3 measures whether it matters.
- **Config.** `trajectory_op`: `carry_zvd` (bool, default false), `platform_mode_hz` and
  `platform_mode_zeta` (per axis, from P0), `platform_mode_delay_s`. Regenerate with
  `config/generate_config.py`; `hardware_config.py` is gated.
- **Tests** (`tests/motion/test_cup_zvd.py`, all per-commit, none `nightly`):
  - the rows' residual: simulate the model on the planned acceleration, residual at release under 1 mm/s at
    the nominal mode and under 15 mm/s across frequency x0.85 and x1.15, at Dwell 0.50 / 0.40 / 0.35;
  - flag off: plans byte-identical to today's (golden from the pre-change planner);
  - the release triple, the catch equality, the C-CUP-4 rows and the release seed unchanged (I-PLAN-11's C1
    seam test re-run with the flag on);
  - a window that cannot meet the rows refuses `RING` with the time it would need;
  - leg peaks at 0.40 and 0.35 against 600 / 8000 / 200000 recorded (not asserted beyond the limits).
- Gate: `./run_tests.sh --full` (a `controller/`-adjacent planner change; CLAUDE.md tier 1 case (c)).
- Logbook: full form (the hard-vs-soft choice and the seed-state choice are Discussion triggers).

### P2: Box re-sweep with the rows on

- `tools/admissible_sweep_run.py`, committed family set, one round plus the 3-family spot check, BLAS one
  thread, no gated-file edit from sweep start to commit.
- **Decision gate (pre-registered):** the oval pose cells at Dwell 0.40 keep at least 20 mm x room at
  Apex 1.00, and no Columns pose cell the Runbook flies loses admissibility. Pass: commit the box. Fail:
  decision 2's soft-penalty fallback, re-sweep, same gate.
- Record per cell: room before / after, the `RING` refusal count, leg peaks. Expected cost from the 1-D
  prototype: acceleration x1.2 and jerk x1.8 at 0.40 (inside the limits); at 0.35 leg jerk ~175k, close to
  200000, so 0.35 may lose cells. That is an honest result, not a reason to relax the rows.
- Decide whether the box stamp records `carry_zvd` (so a flag-off launch refuses loudly on a stamp mismatch)
  or the flag is removed once P3 passes. Recommendation: stamp it.

### P3: A/B Sitting on the oval

JB-102's `## Sitting` section, written when P2 commits. Outline:
- Setup armed, bb-fed, bridge cycled; the P4 working point; plant_id `zvd-oval-<date>`.
- Oval dwell 0.40, Apex 1.00: one Attempt with `carry_zvd` off (learner on a fresh plant), one with it on.
  Then 0.35 with it on, if 0.40 holds.
- Columns canary, two Attempts, flag on: nothing regressed.
- **Pass:** at Dwell 0.40 with the flag on, carried minus from-rest raw bias within +/-10 mm, carried
  per-Throw sd under 17.5 mm, and 30 consecutive Catches in one Attempt (this also meets two-ball-oval P4's
  gate). No `CUP_SEATING_CONE` or leg LIMIT end.
- **Record:** raw x per Throw (landed minus u) split from-rest / carried; encoder and mocap excess at
  separation; `RING` refusals; install-line leg peaks; the mode fit re-run on this bag.

## 4. Control-system walk-through (CLAUDE.md rule)

- **Feedforward path.** The planner's `pose` / `pose_vel` per 25 ms knot go to the can-bridge's 500 Hz
  Hermite. Only knots strictly inside the window change; the catch equality, the soft catch velocity match,
  the release triple and the release seed are untouched, so `release_state_from_meta`, the SETTLE tail, the
  `extend` seams and the splices see the same terminal states. The jerk-parametrised QP keeps C2 at splices.
- **One 40 Hz cycle.** Eight dense rows on a ~150-variable QP: negligible beside the solve, but measure it
  (the serial budget tests in `tests/motion/test_unified_cycle_budget.py` must stay green). The seed's mode
  state is computed once per window from samples the executor already holds; no new I/O in the loop.
- **Carry shape.** The ZV signature: a front-loaded carry with a harder mid-carry deceleration and a slow
  crawl over the last ~75 ms (at 0.40: -117 -> -81 -> -44 -> 0 mm/s at the last knots) instead of a hard
  deceleration into the release. Peak Platform |v| 656 -> 727 mm/s at 0.40.
- **The ball.** The harder deceleration happens with the ball in the cup, after touch-down. The C-CUP-4 cone
  rows bound it in the program and the seating check stays the authority on the realised plan. This is the
  main risk; P2 counts the cone refusals.
- **The Hand.** Hand arrays are not in the rows, but the leg-jerk objective couples z through the composed
  pose; P2 watches the HAND_* gates.
- **Safety authority** is unchanged: the Teensy `MAX_DEVIATION` guard and live `validate_cycle` on every
  segment.

## 5. Risks

1. **Two modes.** Mocap excess is ~1.8x encoder excess, so part of the motion is structural compliance
   beyond the motors. If P0 finds a second frequency, the rows grow to two modes (eight more rows) or the
   design falls back to the servo mode plus leg acceleration feedforward (JB-102 option 4, owner decision:
   it reopens the superseded accel-FF arc).
2. **Mode drift.** Load, temperature or a re-tune moves the frequency; ZVD tolerates about +/-15 %. P3's
   record re-fits the mode on every Sitting bag that flies the oval, which keeps a running check.
3. **0.35 loses cells.** Expected near the jerk limit. Two balls need 0.30-0.35; if the rows price 0.35 out,
   that is the P5 conversation (a longer Dwell, a higher Apex, or the jerk limit), not a reason to drop the rows.
4. **The common +22 mm offset** stays, absorbed by the learner within its 30 mm authority.

## 6. Out of scope

- The ~+22 mm offset seen on Throws from rest too (tilt error times v_z, the Hand stroke's reaction on the
  Platform, the composition at the level offset).
- JB-161's two-ball Columns first-Throw outward landing: P0's probe will be pointed at those releases
  afterwards, as a separate check.
- Learner-side compensation (JB-166's carried flag, the 60 mm authority, on hold).

## 7. Status

| Step | Status | Note |
|---|---|---|
| P0 | not started | |
| P1 | not started | |
| P2 | not started | |
| P3 | not started | |
