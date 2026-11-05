---
title: Carry ring suppression — zero-residual-vibration rows on the pose carry so the Platform is still at release
created: 2026-11-04
status: active
owner: Harrison
last_updated: 2026-11-05
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
  - {id: P0, title: Identify the Platform mode from existing bags, kind: phase, status: complete, effort: s, hardware: none}
  - {id: P1, title: The ZVD contract and rows behind a flag, kind: phase, status: complete, effort: m, hardware: none}
  - {id: P2, title: Box re-sweep with the rows on, kind: phase, status: complete, effort: s, hardware: none}
  - {id: P3, title: A/B Sitting on the oval, kind: phase, status: in_progress, effort: s, hardware: robot}
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

**P0 result (2026-11-04, `tools/probes/platform_mode_fit.py`, bags 2026-10-31 and 2026-11-03; logbook/2026-11-04-carry-ring-zvd-p0-p1.md): the gate FAILED.**

| source | f_n Hz | zeta | delay ms | n good / n |
|---|---|---|---|---|
| encoder (forced fit, both days) | 5.40 / 5.42 | 0.28 / 0.14 | 21 / 22 | 96/109, 99/233 |
| mocap (forced fit, both days) | 4.48 / 4.41 | 0.22 / 0.20 | 17 / 18 | 66/109, 41/233 |

- The two estimators do not agree: free decay is not evaluable (the composed command is never acceleration-quiet through a hold; 2 and 4 usable windows), and the encoder and mocap frequencies differ by ~20 % on both days with non-overlapping IQRs. That is the § 5 risk-1 signature (structure rings below the motors), but one mode versus two is not settled.
- The encoder fit over-predicts the 2026-11-03 separation excess by +69.5 (0.40) and +58.3 mm/s (0.35); Model A by +66 / +54. Model A matches the mocap excess (+127 vs +122 at 0.35), so the debrief's landing-bias validation was a mocap-like match. y is unidentified (carry acceleration 345 mm/s^2 rms against ~3000 in x).
- **Decision (orchestrator, owner can overturn):** proceed with P1 and P2 at the provisional 4.9 Hz / zeta 0.20 / 20 ms now in the yaml, and fly the identification row at the P3 Sitting before the A/B. Reason: the ZVD rows' +/-15 % band centred at 4.9 Hz spans 4.2-5.6 Hz, covering both the mocap (4.45) and encoder (5.4) frequencies, whereas the old 6.0 Hz sits outside the band for the mode the ball sees; and nothing in P1 or P2 is wasted by a later re-fit (values are config, the box is swept under the flag).

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

**P1 result (2026-11-04, uncommitted; lands with the P2 box in one commit):**
- I-PLAN-16 (INVARIANTS.md) and eight ZVD equality rows (4 per lateral axis: e, e', de/dw, de'/dw) in `cup_cycle._pose_augment`, behind `trajectory_op.carry_zvd` (default false). The mode math stays in the gated `cup_cycle.py`, as a list of modes per axis so a second mode is config.
- **Hard rows with a RING re-solve:** when the QP refuses and ring rows were assembled, the same program is re-solved without them; if that solves, the window is refused `RING` naming the residual it would have had, else the original reason stands.
- **Seed:** `CycleState.mode_state` carries the mode state from the head's realised Hermite acceleration (head assumed at rest at knot 0); flag off leaves `mode_state` and `mode0` None.
- **Live switch (mirrors I-PLAN-13 (d)):** `carry_zvd` is a skill_node parameter on every InstallSegment request (new srv field, colcon build needed), the response reports what was solved, and the box carries a `carry_zvd` stamp (level cells false); the sweep has 55 families.
- Measured at 4.9 Hz / 0.20 / 20 ms (x axis e' at release, mm/s): flag off -54.7 at 0.40 and -84.8 at 0.35; on, ~0 at nominal and in the realised Hermite plan. Leg peaks stay inside 600 / 8000 / 200000 (0.35: acc 5096, jerk 123041). Core solve 47.6 ms of 50 at dwell 0.65. Flag-off plans are byte-identical to the pre-change golden.
- **Open owner decision:** the +/-15 % bound (< 15 mm/s) is not met at the short Dwell: 0.35 x1.15 +18.8 mm/s at the old 6 Hz mode (pinned `xfail(strict)`), and at 4.9 Hz the off-nominal frequencies leave 16.0 (0.40 x1.15) and 28.3 (0.35 x1.15) mm/s. A second-derivative (ZVDD) row set was probed and only moves the excess to the other side. It is the short carry against the 200 ms mode period; accept the worst case or change the bound.

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

**P2 result (2026-11-05; sweep `python tools/admissible_sweep_run.py --out temp/sweeps/zvd_2026-11-04`, started 2026-11-04 20:08, finished ~23:55; logbook/2026-11-04-carry-ring-zvd-p0-p1.md): gate 1 PASSED, gate 2 FAILED.**
- Box `gate_hash e8d303ae93bc`, 55 families (level 11, pose 22, pose + `carry_zvd` 22), limits 600 / 8000 / 200000 / 4500, 195 cells (level 31, pose 82 flag off, 82 flag on of which 25 EMPTY). Round b (ovalx040_pose, columnsx1_pose_zvd, columns_pose) identical, `CMP_STATUS 0`. All 113 old cells are present flag off and value-equal; zero flag-off pose cells are empty.
- **Gate 1 (oval Dwell 0.40, Apex 1.00, rows on, at least 20 mm x room): PASS.** 30 mm x and y room, same as flag off, and every oval band at 0.40 / 0.45 / 0.50 keeps 30 / 30. Dwell 0.35 at Apex 1.00 also keeps 30 / 30; 0.35 at Apex 1.10 drops 30 to 10 mm x; the three oval 0.30 flag-on cells are empty (at 0.30 flag off they were already thin: 1 / 10 / 25 mm x).
- **Gate 2 (no Columns pose cell the Runbook flies loses admissibility): FAIL.** The Columns canary cell (Sep 125, Apex 0.95, Dwell 0.27) is present flag off at 30 x 30 and EMPTY with the rows on, at P1/P1 and P2/P2: `RING` refused 18522 of 18522 probes in columns_pose_zvd (and turned90; 6174 in turned45). Columns Apex 0.85 / 0.90, the turned cells 0.85-0.97 and columns_highlow Apex 1.00 are empty too; Columns Apex 1.00 keeps 10 mm x, highlow 1.05 keeps 8 mm x / 25 mm y. Hop, self_toss and oval at 0.40 and above are identical flag on and off.
- Zero `CUP_SEATING_CONE` and zero `POSE_WINDOW` refusals anywhere; at 0.35 only 375 `HAND_LIMIT_ACC` probes (ovalx035) and 23 `MARGIN` (oval035).
- Reading: Columns has no lateral carry, so the only lateral Platform excitation is the prescribed tilt sweep's lever-arm term. A Dwell 0.27 window (11 knots, most pinned by the catch / release triples and the holds) cannot shape it around a 204 ms mode period, so the eight hard rows plus the jerk box are infeasible while the ring-less program solves (that is `RING` by construction).
- **Decision (orchestrator 2026-11-05 00:20, owner asked and not yet answered):** ship the box as swept. An empty flag-on Columns cell means a flag-on Columns launch is refused by the box, loudly; nothing is silently clipped. Fly the P3 A/B on the oval with the rows on and the Columns canary with the flag OFF. The pre-registered soft-penalty fallback is deferred to the owner (see Questions) instead of a second 3.5 h sweep that would collide with the 04:45 Night Shift.

### P3: A/B Sitting on the oval

JB-102's `## Sitting` section, written when P2 commits. Outline:
- Setup armed, bb-fed, bridge cycled; the P4 working point; plant_id `zvd-oval-<date>`.
- Oval dwell 0.40, Apex 1.00: one Attempt with `carry_zvd` off (learner on a fresh plant), one with it on.
  Then 0.35 with it on, if 0.40 holds.
- Columns canary, two Attempts, flag OFF (the box has no Columns cell with the rows on at Dwell 0.27): the flag-off planner is unchanged.
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
3. **0.35 loses cells.** Measured (P2): only the Apex 1.10 band drops (30 to 10 mm x); 0.35 at Apex 1.00 keeps its room; 0.30 is empty with the rows on. The Columns window (Dwell 0.27) is the larger loss (gate 2). Expected near the jerk limit. Two balls need 0.30-0.35; if the rows price 0.35 out,
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
| P0 | done (gate FAILED) | Mode 5.4 Hz encoder / 4.45 Hz mocap, zeta 0.14-0.28, delay 17-22 ms; yaml 4.9 / 0.20 / 0.020 provisional; identification row scheduled at the P3 Sitting |
| P1 | done | Landed uncommitted with the P2 box: flag-off byte-identical, residual ~0 nominal, budget 47.6 ms core of 50; 0.35 x1.15 bound is an open owner decision |
| P2 | done (gate 1 PASS, gate 2 FAIL) | Box e8d303ae93bc, 195 cells; oval 0.40 / Apex 1.00 keeps 30 mm; Columns canary cell empty with the rows on (RING 18522 / 18522); shipped as swept, fallback is an owner decision (Questions) |
| P3 | in progress | 2026-11-05 22:46 Sitting: A arm flag-off only (9/9, 3/3, 0/0, each ended by a firmware STOP: 2 Hand C1 stops, 1 bridge GC pause); B arm never set, no A/B data. Re-fly under FW 39 (Hand C1 grace raised). Identification step dropped from the row: 20 mm profiled steps excite no ring; whole-bag forced fit x 5.47 enc / 4.68 mocap Hz, y 4.06 / 4.24 Hz, zeta x 0.26-0.36, y 0.14-0.17, delay 15-21 ms; provisional 4.9 / 0.20 / 0.020 kept. Gains the identification row first (JB-102 `## Sitting`); Columns canary flies with the flag off |

## 8. Questions

Owner decision needed (asked 2026-11-05, unanswered). Gate 2 failed: with the rows on the Columns window at Dwell 0.27 is infeasible (RING), and the pre-registered fallback (decision 2) was written for a gate-1 failure, not this one. Shipped meanwhile: the box as swept, P3 oval A/B with the rows on, Columns canary flag off.

- (a) **Accept:** Columns under Dwell about 0.30 flies flag off; the rows are an oval / carry tool. This reverses decision 5 in effect.
- (b) **Soft-penalty fallback** (decision 2) and a second re-sweep (about 3.5 h; must avoid the 04:45 Night Shift).
- (c) **A principled scope rule:** the rows are assembled only when the window can hold them (for example carry knots at least one damped mode period); `RING` is replaced by "rows not assembled", with the stamp recording it.
- (d) Something else.

2026-11-05 22:46 Sitting note for the owner: the A/B gave no data on (a)-(d) (B arm never flown), and y now has a measured 4.1-4.2 Hz, below the provisional 4.9 Hz.

Recommendation: (a) for P3, then decide between (b) and (c) with the A/B data in hand, because the two-ball patterns at Dwell 0.30-0.35 face the same window-length limit and the A/B shows whether hard rows are worth it.
