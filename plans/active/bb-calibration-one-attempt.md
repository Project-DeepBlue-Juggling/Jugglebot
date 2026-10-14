---
title: Ball Butler calibration is a one-attempt process, every time
status: active
owner: harrison
created: 2026-10-14
last_updated: 2026-10-14
related_logbook:
  - 2026-10-14-bb-calibration-one-attempt-investigation.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/bb_base_frame.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/jugglebot/mocap_interface.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py
---

# Ball Butler calibration is a one-attempt process, every time

## 0. Goal and acceptance

Owner intent, verbatim: **"BB calibration is a one-attempt process, every
time."**

Since the last calibration code change (b37faf8f, 2026-10-11 00:59) there were
42 sweep starts, 17 accepted and 25 refused. A BB power cycle and a QTM
re-alignment are the only events in the window. Nothing physical on BB, its
shelf or its markers was touched. Every history-gate refusal was a false
positive. The evidence is in the related logbook entry.

Acceptance, in order:

1. **Offline replay.** The replay harness re-solves every recorded sweep from
   2026-10-10 to 2026-10-14 under the new gates. The 25 refused sweeps come
   out accepted. The two intrinsic-quality refusals (CONSTELLATION_TOO_FEW_MOVING
   and the 7.9 s circle-centres sweep) are judged on their data, not forced.
2. **Hardware.** The first sweep of each of ten consecutive sittings is
   accepted with no override, and its kappa and yaw lie inside the band
   characterised in Phase 0.

## 1. Root causes being closed

Evidence for each is in the entry. Labels match its census.

- **C1.** Kappa (BB yaw offset in the shelf base frame) stepped +0.25 deg
  between 2026-10-13 00:18 and 22:00. The cause is solver-side and
  pose-dependent QTM reconstruction at the shelf, not BB and not the encoder.
- **C2.** The kappa gate limit sat at its 0.09 deg floor. It was built from a
  per-sweep sigma measured in one sitting. Cross-sitting repeatability was
  never characterised.
- **C3.** The only remedy offered, `bb_moved`, destroys the pool (34 records to
  1, no backup). Under 5 records, every later sweep drops onto the world-frame
  path and the 0.50 mm template gate.
- **C4.** The world-frame gate compares against the last accepted sweep in QTM
  coordinates. Moving Jugglebot and re-aligning QTM moves BB in world by
  design. The gate refuses the first sweep after every re-alignment, forcing
  `bb_moved` again. This is a positive-feedback loop.
- **C5.** TEMPLATE_RESIDUAL at 0.50 mm sits inside the daily reconstruction
  scatter. A sweep passes or fails by which path it took, not by its data.
- **C6.** Design premise: the pooled kappa is the output and the sweep only
  checks it. In the window the gates caught zero real moves and refused 23
  good sweeps.

## 2. Decisions already taken (owner, 2026-10-14)

- **Measure fresh each time.** The sweep is the output. History is advisory
  at most.
- **Measure the robot frame in software.** No manual QTM re-alignment to
  Jugglebot's base. No assumption that mocap origin equals robot base.
- **`bb_moved` is retired as a gate input.**

Also fixed by the owner: BB's yaw encoder is absolute, and BB's shelf is
room-fixed while Jugglebot is not.

## 3. Phases

Order: **0 then 1 then (2 in parallel with 3).** Design-bearing units, where
the CLAUDE.md model-tier rule allows Opus (one line of justification in the
brief): **Phase 3** (frame math, safety-adjacent aim and catch sites) and the
**band-setting step of Phase 0** (every later threshold derives from it).
Everything else is Sonnet work. Scope each agent to under about 80 tool
calls.

### Phase 0 - Characterise

Deliverable:

- A replay harness in `tools/probes/` that re-solves every sweep in the
  2026-10-10 to 2026-10-14 bags (about 60 sweeps; the solver is pure Python).
  It prints a per-sweep verdict under the current gates and under the new
  ones.
- Cross-sitting distributions of kappa, yaw, axis point, yaw-source lag and
  template residual. These set every advisory band used in Phases 1 and 2.
  No band is taken from one sitting's sigma.
- **Hardware pin of the kappa step (P7).** In one launch, alternate sweeps
  to the old end (about 126 deg) and the new end (about 121 deg), three
  each. Then do three BB power cycles, each followed by a park reading and a
  sweep. Log the sweep end angle and fitted lag in the attempt message (not
  recorded today). Decision criterion: if kappa follows the sweep range, the
  band is widened to cover it and the range becomes a recorded field. If it
  does not, the band stays as characterised and the mechanism stays "QTM
  systematics" in the entry.

Files likely touched: new `tools/probes/bb_calibration_replay.py`;
`bb_calibration.py` (log end angle and lag only); `tests/ros/test_bb_*.py`
(replay fixture).

Pinned by: replay probe output committed under `temp/probes/` plus a test
that loads a small recorded-sweep fixture and asserts the distribution
summary code.

Gate: bands published in the logbook with the (date, command, result)
triple; owner reviews the band before Phase 1 consumes it.

- **Cross-check against throws.** Correlate each launch's accepted calibration (yaw, position, pose source, pool n) with that launch's feed landing bias from the columns analysis (`temp/probes/u4_feeds.csv`, parallel session 2026-10-14: median x -17 / +6 / -7 mm across three launches with an identical BB command). If the bias tracks the calibration, the characterised band gains a throw-side meaning (mm at the catch per degree of kappa); if it does not, the landing bias has another owner and this plan does not claim it.

### Phase 1 - Gate inversion, non-destructive reference, operator surface (P1, P3, P6)

Deliverable:

- Hard refusals only for intrinsic sweep quality, decided without history:
  CONSTELLATION_MIN_FRAMES, TOO_FEW_MOVING, LAG_AT_EDGE, MOCAP_CLOCK, axis
  consensus, CONSTELLATION_RESIDUAL at 1.0 mm.
- Kappa vs pool, yaw vs last accepted and template residual vs 0.50 mm
  become advisory lines.
- The pool is kept in epochs and never discarded, with a file backup on
  every write. With fewer than 5 records the base path still outputs the
  sweep's kappa and skips the comparison. There is no mode switch on pool
  size.
- One outcome line per attempt reports every advisory at once, in plain
  language. A refusal names an operator action only when that action is the
  remedy.
- The console prints startup state: pool n, `base_ref`, template path,
  pose and yaw source.
- `bb_moved` stops gating. It remains only as an explicit "start a new
  epoch" command if the owner wants one.
- A rehearsal row for the calibration rung is added to the `tests/hardware`
  runsheet (dress-rehearse rule).

Files likely touched: `bb_calibration.py`, `bb_base_frame.py`,
`mocap_node.py`, `tests/ros/test_bb_*.py`, `tests/ros/test_mocap_node_*.py`,
the `tests/hardware` runsheet.

Pinned by: the replay test, where the 25 refused sweeps are accepted and a
synthetic bumped-BB sweep (shifted markers) still yields an advisory. Also
tests that no code path resets the pool without a backup.

Gate: `./run_tests.sh --full`, plus the replay verdict table in the logbook.

### Phase 2 - Template residual (P5)

Deliverable:

- One path and one statistic for the template residual. It is advisory at
  the Phase 0 band and capped hard at 1.0 mm.
- A labelling and coverage investigation, as a logbook entry with a
  recommendation: why only 5 of 7 BB markers label at rest (QTM3 in 4 bags,
  QTM5 never), why rest pair separations vary 0.5 to 4 mm bag to bag, and
  whether a QTM rigid-body definition for BB makes labelling robust. Any
  QTM-side change is the owner's to make.

Files likely touched: `bb_calibration.py`, `resources/bb_marker_template.json`
(only if the investigation justifies re-fitting; pose-sampling coverage
matters), `tests/ros/test_bb_*.py`.

Pinned by: a test that the same data gives the same residual verdict on any
pool size, and the replay.

Gate: replay still accepts the 25; the 0.52 to 0.76 mm sweeps appear as
advisories, not refusals.

### Phase 3 - Robot frame measured in software (P4)

Deliverable:

- `mocap_node` fits the Jugglebot Base rigid body (the labelled
  "Base - 1..7" markers) at session start and tracks it. It publishes
  T_world to base.
- Consumers are re-expressed in the base frame: BB aim xy (`skill_node.py`),
  stow plane z 578.2 (`mocap_interface.py`, static TF in `mocap_node.py`),
  landing plane z 830 (`ball_tracker_node.py`), the Platform offset as a full
  rigid transform, and `columns_feed_bb_bias_mm`.
- The world-frame consistency gate is retired. The Base alignment check
  (2000 mm / 180 deg thresholds, never fires) is replaced by a real
  rigid-body fit-residual check.
- **Artefact migration, once:** `bb_calibration_last_accepted.json`;
  `bb_base_frame_state.json` (`base_ref`, per-record world yaw);
  `resources/bb_base_frame.json` (`base_pose_at_build`); the template gauge
  pin (landing-derived); `throw_affine_correction.json` (disabled);
  `columns_feed_bb_bias_mm`. Learner memory is release-relative and needs no
  migration.
- The manual QTM re-alignment step is removed from the runsheets.

Control analysis required before code (CLAUDE.md): which setpoints change
frame, whether the Platform offset step at session start can produce a
discontinuity at the 40 Hz planner, and a walk through one cycle with the
transform applied. When the rigid transform is identity (frame already
aligned), every output must equal today's bit for bit.

Files likely touched: `mocap_node.py`, `mocap_interface.py`, `skill_node.py`,
`ball_tracker_node.py`, `bb_calibration.py`, `bb_base_frame.py`, the
resources JSONs, `tests/ros/test_bb_*.py`, `tests/ros/test_mocap_node_*.py`,
`tests/sim/test_skills_gate.py`.

Pinned by:

- Identity-transform test: outputs equal today's.
- Rotated and translated QTM frame test: every aim, catch site and plane is
  invariant.
- A bag replay across the 2026-10-10 23:32 frame re-definition with no
  override.

Gate: `./run_tests.sh --full`; dress rehearsal on the loaded Jetson against
the live `/robot_state` before any powered sitting. Hardware check: move
Jugglebot, do not re-align QTM, and the first sweep is accepted and a catch
column still lands.

## 4. Risks and what this plan does NOT do

- No change to BB firmware. No claim about the kappa-step mechanism beyond
  the entry; Phase 0 pins it on hardware but the plan does not depend on the
  answer.
- Removing history gates also removes protection against a genuinely bumped
  BB. Mitigation: the advisory line still prints, and the throw learner
  absorbs a 0.25 deg error (about 4 mm at 1 m). The R5 gate was met under
  this error.
- Phase 3 touches aim and catch geometry. A frame error there moves where the
  robot goes. Hence the identity test, the rotated-frame invariance test and
  the rehearsal gate.
- The labelling investigation may conclude the fix is QTM-side. That is a
  recommendation, not a code change here.
- Out of scope: ball-tracker, learner and skill-stack changes not forced by
  the frame change; the manual wand calibration of QTM itself.

## 5. Status

| Phase | Status | Date | Note |
|-------|--------|------|------|
| 0 Characterise | not started | - | design-bearing (bands) |
| 1 Gate inversion and operator surface | not started | - | needs Phase 0 bands |
| 2 Template residual | not started | - | parallel with 3 after Phase 1 |
| 3 Robot frame in software | not started | - | design-bearing; after Phase 1 |
