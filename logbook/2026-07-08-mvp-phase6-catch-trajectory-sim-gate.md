---
title: MVP Phase 6 — Catch trajectory + sim reload gate (landed light; vel-match deferred)
type: feature
date: 2026-07-08
status: resolved
phase: "6"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/tilt_geometry.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/ballistics_bc.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/planner.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/__init__.py
  - sim/juggle_noise.py
  - sim/reload_gate.py
  - tests/motion/test_trajectory_tilt_geometry.py
  - tests/motion/test_trajectory_ballistics_bc.py
  - tests/motion/test_trajectory_planner_catch.py
  - tests/sim/test_reload_gate.py
commits:
  - 12c7ad1
  - b490dfd   # docs (logbook + plan + INDEX)
subsystem: motion
tags: [mvp-trajectory-bringup, catch, reload, tilt, ballistics, sim-gate, mujoco, light-scope]
---

## Summary

Phase 6 lands the reload **catch trajectory** (`planner.build_catch`), its supporting
geometry (`tilt_geometry.tilt_to_receive` + `ballistics_bc` touchdown/launch BCs),
and a headless seeded **production-in-the-loop reload-gate harness**
(`sim/reload_gate.py`) that drives the *actual* `build_catch` planner + `KnotEmitter`
+ a real `SetpointPump` into the MuJoCo plant, throws a ballistic ball under the §3
juggle noise, and arms the arm-and-forget Teensy hand mirror
(`sim.hand.trajectory.HandCatchSequence`).

The gate **CORE-PASSES**: across the nominal 20-run grid (5 landing offsets within
±60 mm × 4 arrival speeds 2.5–4.0 m/s) **and** all four robustness sweeps
(arm-time ±30 ms, event_vel ±10 %) it catches every ball, holds it, keeps the
hold-phase platform quiescent to **≤ 0.02 mm / ≤ 0.02°**, records **zero**
feasibility violations in accepted runs and **zero** pump rejects. The one
Reload-gate criterion it does **not** meet — hand-contact velocity-match
(|v_hand − v_ball| ≤ 15 % at first contact, floors ~0.26) — is a **light-scope
deferral** (operator-approved): its root cause is fully diagnosed and it is
inconsistent with the hardware-validated hand design (see Open Questions). The
required leg limits for Phase 4 are published: **vel ≈ 156 mm/s, acc ≈ 660 mm/s²,
jerk ≈ 10 331 mm/s³** (already 1.15× headroom-inflated).

## Motivation

Phase 6 is the sim-gated bridge to the hardware reload (Phase 7). Its deliverables:
port the Jugglebot-bb catch **geometry** (tilt-to-receive, the cup lever arm — Phase 4
already ported the lever arm into `shaping.py`), add the ballistic boundary
conditions, add the `build_catch` platform trajectory (reach → tilt-through-seat →
quiescent hold), and build a production-in-the-loop harness that proves a sim pass
would transfer to hardware. The plan's inherited premise is load-bearing: **hardware
already catches smoothly** with the arm-and-forget Platform-Teensy hand; the old
Jugglebot-bb sims (continuous sub-tick velocity-matched hand commands that never
existed on hardware) were the janky side. So the job is to make the sim *reproduce*
the hardware-proven behaviour, not to tune hardware.

## Design

### `tilt_geometry.py` — catch tilt, single lever-arm source

`tilt_to_receive(v_arrival)` ports the bb "Kai collinear catch": the cup up-axis is
set anti-parallel to the arrival velocity (ball drops straight down the cup axis,
zero lateral in the cup frame), clamped to `MAX_TILT_DEG = 12°`. `cup_axis(rx, ry)`
is the inverse (verified anti-parallel to −v to 1e-3). The cup **lever arm**
(`CUP_TILT_CENTER_Z_MM = 744.3`, `LEVER_ARM_MM_PER_DEG = 1.66`, cup-height-derived)
was already ported into `shaping.py` in Phase 4; to keep a **single source of truth**
(the "protect the contracts" rule — two copies drift), `tilt_geometry` **re-exports**
those symbols rather than redefining them, and the geometry-regression test pins them
against the bb reference here.

### `ballistics_bc.py` — touchdown quadratic + launch/arrival BCs

Copied (not imported — `motion/` stays free of `controller/` path dependence, as
`quintic.py` copies `hermite.py`) the identical `launch_velocity` /
`arrival_velocity` forms from `controller/ballistics.py`, and **adds** the touchdown
quadratic the catch path needs (`controller/ballistics` only has the apex-constrained
`flight_time_from_apex`; the catch knows the release state, not the apex, so it solves
the descending root of `z(t) = z_catch` directly). Overlap documented + regression-
pinned against `controller.ballistics`.

### `planner.build_catch` — reach / tilt-through-seat / quiescent hold

Assembles, gated as one plan: (1) a **reach** from the seed to `catch_pose` over the
FIXED arrival lead, with **translational velocity at arrival forced to zero** (a
baked-in safety invariant: velocity matching is the hand's job, so the platform is
translationally still at contact); (2) a **tilt-through-seat decay** — the reach
arrives at the seat tilt with a *small* residual tilt rate (`0.07 rad/s`, tilt-
increasing direction) so the rim is still moving through the seat angle at contact
(a parked tilted rim deflects the ball), decayed to rest over 0.15 s; (3) a **literal
quiescent hold** (zero twist, `settle_hold_s`); (4) optional slow **return** to
neutral. Fixed-lead like `build_timed` — a too-tight lead is loudly rejected
`TOO_FAST` with the minimal feasible lead; a spatial `WORKSPACE`/`UNREACHABLE` is
re-raised.

### `sim/reload_gate.py` — the production-in-the-loop harness

Per trial: synthesise the ball's spawn state a short lead before arrival and apply
the §3 BB throw noise **there** (the bb apex/descent-spawn model — see Discussion);
observe a descent window with tracking noise and fit a known-gravity arc
(`BallisticEstimator`, not a raw finite difference); build `catch_pose` from the
observed landing + arrival velocity (receive tilt + lever-arm-compensated reach xy +
the cup-height-consistent platform z); **plan via `build_catch`** (the single gate);
then stream 40 Hz emitter knots through a **real `SetpointPump`** into the MuJoCo
plant while sampling the arm-and-forget `HandCatchSequence` at the **physics-substep
rate** (the Teensy runs the hand high-rate — see Discussion), spawn the ball, and
measure catch/hold/quiescence/separation/vel-match. Emits a JSON report to
`temp/reports/` and publishes the required leg limits.

## Implementation

New pure modules `tilt_geometry.py`, `ballistics_bc.py`; `planner.build_catch` +
`_min_feasible_catch`; `__init__` exports. Ported `sim/juggle_noise.py` (verbatim —
pure numpy, MuJoCo-free). New `sim/reload_gate.py` harness. Tests: 34 pure-motion
(`test_trajectory_tilt_geometry` 15, `test_trajectory_ballistics_bc` 7,
`test_trajectory_planner_catch` 12) + 9 harness (`test_reload_gate`). **No ROS
interface / config / launch change** (Phase-6 scope is the planner + sim harness; the
`build_catch` node wiring is Phase 7), so **no colcon or codegen gate this phase**.

## Verification

- Full suite: `pytest tests/ -q` (2026-07-08) = **2222 passed, 1 xfailed in 526.04 s**
  (baseline 2179 passed / 1 xfailed at Phase-5 audit-fix `22ed9cf`; net **+43** = the
  new tests only: 15 tilt + 7 ballistics + 12 catch + 9 harness; no regressions).
- New-tests scoped (2026-07-08): `pytest tests/motion/test_trajectory_tilt_geometry.py
  tests/motion/test_trajectory_ballistics_bc.py tests/motion/test_trajectory_planner_catch.py
  tests/sim/test_reload_gate.py -q` = **43 passed in 6.11 s**.
- **Reload gate (`python sim/reload_gate.py --trials 20`, 2026-07-08):** nominal
  **CORE PASS** — core_clean **18/20** (threshold 18), caught **20/20**, held
  **20/20**, feasibility violations **0**, pump rejects **0**, worst hold travel
  **0.01 mm**, worst hold tilt **0.01°**, worst separation **0.0 ms**. Robustness
  sweeps all CORE PASS: arm +30 ms **20/20**, arm −30 ms **20/20**, event_vel +10 %
  **20/20**, event_vel −10 % **20/20** (seeds 100/200/300/400). The two nominal
  non-core-clean trials are still CAUGHT — they exceed the ≤80 mm reach envelope flag
  (offset 40 mm + lever shift + noise → 89/92 mm), not a catch failure.
- **Required leg limits published to Phase 4** (max measured leg peak across accepted
  runs × 1.15 headroom): **vel ≈ 156 mm/s, acc ≈ 660 mm/s², jerk ≈ 10 331 mm/s³**
  (at `lead_s = 0.7`, ≤ 80 mm reach, ≤ 12° tilt). Modest — within the YAML ceilings
  (280 / 4000 / 200 000) and near the Phase-1 defaults; only the jerk is meaningfully
  above the 8 000 default.
- Production-in-the-loop invariant re-asserted inside the gate: every emitted catch
  knot accepted by a real `SetpointPump` (`test_every_emitted_knot_pump_accepted`;
  0 rejects across all runs).

## Discussion

### Why the gate is production-in-the-loop on the MAIN sim plant (not the bb port)

The plan's "Production-in-the-loop rule" requires the sim gate to drive the *actual*
`motion.trajectory` planner + emitter → sim plant with the arm-and-forget hand, and
every emitted knot to pass a real `SetpointPump`. A probe (2026-07-08) confirmed the
**main** `MuJoCoPlant` + `sim/model/jugglebot.xml` + `sim/ball/manager.py` already
support a single-ball MuJoCo catch: `spawn_ball`, `command_hand`, contact-based
`poll_capture` → kinematic hold. So the harness runs on the unchanged main sim files
— no bb port required for the reload catch (see the six-file audit). This is the
faithful gate the plan wants: a sim pass transfers because every command flows through
the same planner/gate/pump the robot uses.

### Six-file diff-audit — the bb divergences are MULTI-BALL, not catch-critical

The plan's required six-file audit (`sim/plant/mujoco_plant.py`,
`sim/model/jugglebot.xml`, `sim/ball/manager.py` + `__init__.py`,
`sim/ball_butler/sim.py`, `sim/hand/trajectory.py`, `sim/main.py`) resolved cleanly:
the bb divergences are **two-ball juggling** features (Phases 8/9), NOT single-ball
reload-catch requirements. Specifically: `jugglebot.xml` adds a second ball `ball2` +
a ball-ball collision `exclude`; `ball/manager.py` (611 lines diverged) adds a `Ball`
class for multi-ball lifecycle; `ball/__init__.py` exports it; `ball_butler/sim.py`
and `main.py` are near-identical (8 / 17 lines, runner plumbing). **None are needed
for the reload catch**, so nothing was ported except `juggle_noise.py` (the §3 noise
model, verbatim). The multi-ball files are the Phase-8/9 port.

**One catch-relevant finding surfaced (documented, deliberately NOT applied):**
`sim/hand/trajectory.py` diverges on `CATCH_VEL_RATIO` — **main = 0.9**, but the bb
branch changed it to **0.6** with the note *"matches hardware_config.yaml
teensy_trajectory.catch_vel_ratio (the platform hand). Hardware-validated as reliable
… (Was 0.9 — a stale port value.)"* Confirmed against ground truth: main
`config/hardware_config.yaml` says `teensy_trajectory.catch_vel_ratio: 0.6`, so the
main sim mirror's hardcoded **0.9 is stale vs the config source of truth**. This bears
directly on the vel-match deferral (Open Questions) and is fixable, but I did NOT
change it this session — see the decide+document fork below.

### Fork (mine, decide+document) — did NOT change `CATCH_VEL_RATIO` 0.9→0.6

Concrete failure modes weighed: (a) `sim/hand/trajectory.py` is a **shared** sim file;
flipping 0.9→0.6 alters the hand catch profile used by *unaudited* existing tests
(`sim/test_hand_stroke.py`, `sweep_speed_ratio.py`, the hand coordinator tests) —
risking a red suite for a change orthogonal to Phase 6's deliverables and violating
rollback granularity; (b) `CATCH_VEL_RATIO` is a hand-behavior parameter the plan
explicitly gates behind *hardware evidence / stop-and-decide* ("hardware-side changes …
are last resort"); (c) fixing it would **not change the light-scope outcome** — at 0.6
the *designed* first-contact mismatch is ~40 % (the hand decelerates the ball over the
stroke, not velocity-matches at contact), so the ≤ 15 % criterion becomes *more*
clearly unmeetable, not less. So: documented as a HIGH-priority Open Question with the
exact fix, deferred to operator/next-session review with proper test reconciliation.
The CORE gate result is independent of 0.6 vs 0.9 (the ball is caught and held either
way).

### Fork (mine, decide+document) — `build_catch` gate is `validate_follow`

The task framed this as a fork I own ("per-announcement planning can afford the
analytic gate; pre-freeze supersedes arrive at announcement rate, not 40 Hz"). I chose
the **fast `validate_follow`**, matching the Phase-5 `catch/dynamic_target` path
(`build_timed`). Concrete reasons over the analytic `validate`: (1) consistency — the
catch IS the CATCH-mode reach, and the Phase-5 catch path already uses the fast gate,
so a supersede installs a C2 replan shortly after its seed; (2) the catch is *never*
lean-shaped, so `validate_follow`'s shaping-blindness is a guard, not a limitation;
(3) its knot-step bound is bit-identical to `validate`, so pump-acceptance is
preserved exactly; (4) the analytic gate's only advantage — measuring a lean
superposition — is irrelevant to an unshaped catch. The analytic gate's ~377 ms would
only add install latency for no benefit.

### Fork (mine, decide+document) — the tilt-through-seat residual rate is small + fixed

The plan wants the tilt to "ramp through the seat with a small residual tilt rate at
t_arrival decayed to zero over 0.15 s". The magnitude is a physical-tuning parameter
with no config constant. I chose a conservative fixed default `0.07 rad/s` (~4°/s),
tilt-increasing direction, capped so the ~0.3° overshoot stays well inside MAX_TILT
(12°) and the hold-quiescence tilt budget (<1°). Ruled out: (a) zero rate (a parked
rim — the very failure the plan calls out); (b) a large rate (would blow the tilt
ceiling / hold-quiescence and induce needless leg motion). The value is exposed as a
`build_catch` parameter for the operator to tune on hardware (Phase 7). The plan is
always rest-terminating (the decay + hold end at rest), so this is safe regardless.

### The vel-match diagnosis arc (>2 documented attempts → light-scope)

The one unmet criterion, |v_hand − v_ball| ≤ 15 % at first contact, was chased through
several documented attempts before invoking the operator-approved light-scope rule:

1. **Throw synthesis bug** — the release back-integration used `v_arrival` instead of
   `v_release` (inverting the gravity term), launching the ball to a 6 m apex. Fixed.
2. **Observation amplification** — fitting a full-flight release and propagating over a
   ~1.6 s arc amplified velocity error; and a steep ground-launched throw made the 2 %
   BB noise scatter landings ~150 mm (uncatchable). Fixed by the **bb apex/descent-spawn
   model**: synthesise the ball's spawn state a short lead before arrival and apply the
   noise there (what `juggle_bb_catch` does), and observe a descent window near arrival.
   → catch reliability went to **20/20** and reach mostly ≤ 80 mm.
3. **Hand-rate fidelity** — the hand was commanded once per 25 ms platform knot, but the
   Teensy runs its arm-and-forget catch profile high-rate; the ~14 ms velocity-hold was
   unresolved. Fixed by sampling `HandCatchSequence` every physics substep. → vel-match
   went from 0.44–0.99 to a consistent ~0.30.
4. **Contact-timing fixed point** — capture fires ~+45 ms past centre-arrival (the ball
   descends into the co-moving cup before contacting the collision geom), and this is
   *coupled* (moving the hand later raises the cup and shifts capture earlier). Swept the
   `capture_offset_s` fixed point; the minimum lands vel-match at **~0.26**, a hard
   floor.
5. **Decomposition** — at capture the hand velocity is a **uniform ~0.74×** the ball
   velocity in the SAME direction (lateral and axial mismatch scale identically → the
   cup axis IS correctly aligned; the tilt geometry is right). The floor is *magnitude*:
   the Teensy velocity-hold window (~14 ms) is narrower than the achievable capture-
   timing alignment (~±20 ms) in the contact→instant-hold model, so the hand is rarely
   at its −0.9·v hold speed at the single contact instant.

The floor is a genuine **sim-contact-model artifact**, reinforced by the
`CATCH_VEL_RATIO` finding: the real hand catches at 0.6·v_ball (a *designed* 40 %
first-contact mismatch) and absorbs the ball over the full stroke — "velocity at first
contact" is not the physical quantity the hardware hand optimises. Per the operator-
approved light-scope rule, everything except this contact criterion is landed, with the
criterion documented precisely below. The two evidence artefacts the plan actually asks
for (contact relative-velocity and hold-travel) are produced by the harness's JSON
report: hold-travel is decisively resolved (**≤ 0.02 mm**, resolving the "prior sim
smoothness was not real" caveat on the *platform* side), and the contact relative-
velocity is characterised (~0.26, root-caused).

## Open Questions

1. **Hand-contact vel-match ≤ 15 % (light-scope deferral).** Not met (floors ~0.26 at
   best timing). Root cause fully diagnosed (Discussion #4–5): the Teensy ~14 ms
   velocity-hold is narrower than the achievable ball-cup capture-timing alignment in
   MuJoCo's contact→instant-hold model. **Reframed by the `CATCH_VEL_RATIO` finding:**
   the hardware hand catches at 0.6·v_ball (designed ~40 % first-contact mismatch,
   absorbing over the stroke), so the ≤ 15 % *first-contact* criterion is inconsistent
   with the hardware-validated hand — the criterion itself likely needs revisiting
   (measure over the seat stroke, or match to the 0.6 design) before Phase 7. The
   hardware premise (catches are already smooth) means this is a sim-metric gap, not a
   trajectory defect.
2. **`sim/hand/trajectory.py` `CATCH_VEL_RATIO = 0.9` is stale vs config (0.6).** HIGH
   priority. The main sim hand mirror disagrees with `hardware_config.yaml`
   `teensy_trajectory.catch_vel_ratio: 0.6` (the bb branch already fixed this). Fix:
   port 0.9→0.6, then reconcile the existing hand-catch tests (`sim/test_hand_stroke.py`,
   `sweep_speed_ratio.py`) — deferred out of Phase-6 scope (hand-behavior change,
   stop-and-decide) to keep the suite green and rollback granular.
3. **Reach envelope vs ±60 mm offsets under noise.** Two nominal trials (offset 40 mm)
   reach 89/92 mm — offset + lever-arm shift + noise can exceed the ≤ 80 mm reliable
   envelope even from a ≤ 60 mm intended offset. Still caught; flagged not fatal. Phase 7
   should either tighten the offset spec or widen the envelope with hardware evidence.
4. **Catch z-convention (inherited from Phase 5).** The harness uses the cup-height-
   consistent platform z derived from the sim geometry (probed); the MPC-offset →
   STOW-relative (+170) mapping remains hardware-UNVERIFIED (Phase-7a task). The gate
   rejects an out-of-stroke z loudly meanwhile.

## Related

- Plan: `plans/active/mvp-trajectory-bringup.md` § Phase 6, § Hand-catch smoothness,
  § Sim strategy, § Reload sequence.
- Predecessor: `logbook/2026-07-08-mvp-phase5-timed-targets.md` (the `build_timed`
  catch reach + reach-freeze this phase's `build_catch` extends; the fast
  `validate_follow` gate reused).
- Reference (archaeological): `~/Desktop/Jugglebot-bb` `demo/bb-led-two-ball-juggle`
  (`sim/juggle_tilt.py`, `sim/juggle_noise.py`, `sim/juggle_bb_catch.py`) — the
  sim-validated catch geometry ported here.
- Gate reports: `temp/reports/reload_gate_nominal.json` + the four sweeps (regenerate
  with `python sim/reload_gate.py --trials 20`).
