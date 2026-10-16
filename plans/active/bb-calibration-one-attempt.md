---
title: Ball Butler calibration is a one-attempt process, every time
status: active
owner: harrison
created: 2026-10-14
last_updated: 2026-10-16
related_logbook:
  - 2026-10-14-bb-calibration-one-attempt-investigation.md
  - 2026-10-16-bb-calibration-phase1-gate-inversion.md
  - 2026-10-16-bb-calibration-phase2-template-residual.md
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
42 sweep starts, 17 accepted and 25 refused. A BB power cycle, a QTM
re-alignment and a BB firmware flash (8 to 9, 2026-10-13 20:09, CANCEL_CMD
only; the diff touches no yaw, sweep or reporting code, checked 2026-10-15)
are the only events in the window, and QTM's wand calibration was 562 h old across the window (re-wanded 2026-10-16). Nothing physical on BB, its
shelf or its markers was touched. Every history-gate refusal was a false
positive. The evidence is in the related logbook entry.

Acceptance, in order:

1. **Offline replay.** The replay harness re-solves every recorded sweep from
   2026-10-10 to 2026-10-14 under the new gates. The 25 refused sweeps come
   out accepted. The two intrinsic-quality refusals (CONSTELLATION_TOO_FEW_MOVING
   and the 7.9 s circle-centres sweep) are judged on their data, not forced.
2. **Hardware.** The first sweep of each of ten consecutive sittings is
   accepted with no override, and its kappa and yaw lie inside the band
   characterised in Phase 0 (owner decision pending 2026-10-16: reword to
   'accepted with no override and no operator action; kappa inside its
   epoch's band; any KAPPA_EXCURSION recorded with its cause' - the
   2026-10-16 first sweep legitimately opened a new epoch).

## 1. Root causes being closed

Evidence for each is in the entry. Labels match its census.

- **C1.** Kappa (BB yaw offset in the shelf base frame) stepped about +0.3 deg
  (+0.25 in the console census, +0.31 in the replay) between 2026-10-13 00:18 and 22:00. Leading candidate
  cause (2026-10-16): two-state kappa, +0.31 deg on 10-13 and -0.33 deg on
  10-16, both between sittings, QTM reconstruction and the wand calibration
  EXCLUDED 2026-10-16 (the 10:51 and 15:08 sweeps share one calibration and
  one frame). **Decided by Phase 2 (2026-10-16 evening): at BB.** Every marker
  subset, leave-one-out and label-free refit keeps the step on 75 sweeps of 22
  sittings, both sweep legs carry it, the axis point moved under 0.5 mm, and
  the BB firmware reads the encoder absolutely with a constant zero (nothing
  stored or homed). Next: the owner's BB-only power-cycle A/B (Phase 2 entry).
  Also NOT the sweep pose mix (clip experiment, Phase 0 entry § D), not the
  encoder's firmware, not the shelf (tilt in world flat).
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
- **`bb_moved` is deleted** (owner, 2026-10-15: "no particular desire to keep
  it around"). Not kept as an epoch command. The parameter, its arming
  message, the override branches in both gates and the pool reset go in
  Phase 1, with the launch-file and runsheet references.
- **The hardware pin of the kappa step stays in Phase 0** (owner, 2026-10-15).

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
- **Pin of the kappa step (P7), redesigned 2026-10-15.** The sweep end is
  not commandable: BB firmware drives the yaw to a compile-time 120 deg
  target and reverses at the first sample past 119 deg, so the recorded
  maximum is overshoot (hardware_config.h CALIB_MAX_YAW_DEG, StateMachine
  handleCalibrating_). From 2026-10-13 22:00 the overshoot shrank from 4 to 8
  deg to 0 to 3 deg at the top and from about 1 deg to none at the return,
  with the start yaw unchanged and the peak speed about 2 deg/s lower: the
  sweep dynamics changed at the power cycle, and with them the pose range
  the solver samples. The pin is therefore in two parts.
  1. **Offline, Phase 0 (this unit).** The replay harness re-solves every
     pre-step sweep with its frames clipped to the post-step yaw range
     (about 0 to 119 deg) and reports kappa(full) minus kappa(clipped) per
     sweep. Criterion: if clipping moves kappa by about +0.25 deg (the
     observed step, SD 0.03), the mechanism is pose-mix dependence of the
     fit (QTM reconstruction systematics at the shelf), the band covers it
     and the sweep range becomes a recorded field. If clipping moves kappa
     by less than 0.1 deg, the pose mix is not the mechanism and "not BB"
     in the entry is downgraded to "unresolved": a physical change at BB
     coincident with the power cycle (encoder magnet seat, yaw mount,
     supply state changing the drive dynamics) becomes the leading
     candidate.
  2. **Hardware: re-scoped 2026-10-16 (owner).** The three-power-cycle
     sitting is dropped: every session boundary is already a power cycle (BB
     is on the main motor PSU) and kappa survived all but one of them, and
     Phase 1's acceptance (first sweep of ten consecutive sittings) is ten
     more. Replaced by (i) the owner's QTM check, done (wand calibration 562 h
     old, re-wanded 2026-10-16; QTM's BB residual 0.30 -> 0.23 mm) and (ii)
     the first Phase 1 sweep after the re-wand (read 2026-10-16: see the
     Phase 1 entry; the re-wand was not the cause): BB template residual back
     near 0.26 mm with kappa moving off the 179.59 deg level (Phase 0's K2;
     the node labels it K0 on the migrated live state) says
     reconstruction owned the step; kappa staying at 179.59 with the residual
     down says something else at the shelf. The decision criterion stays
     0.20 deg (the within-epoch band). A runtime sweep-end parameter in BB
     firmware remains the owner's call in the BB repo and is not needed.

Files likely touched: new `tools/probes/bb_calibration_replay.py`;
`bb_calibration.py` (log end angle and lag only); `tests/ros/test_bb_*.py`
(replay fixture).

Pinned by: the replay's fidelity and summary tables in the Phase 0 logbook
entry (run output stays under gitignored `temp/probes/`), a committed
one-sweep fixture in `tools/probes/data/` with its `--emit-fixture` recipe,
and a test that re-solves the fixture and asserts the distribution summary
code.

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
  every write. The sweep's own yaw offset and axis point are always the
  output; the base frame only yields kappa for the epoch advisory. There is
  no mode switch on pool size, and the `bb_pose_source` and
  `bb_base_min_sweeps` parameters are deleted with `bb_moved`.
- Kappa epochs: a sweep joins the nearest epoch whose median is within 0.20
  deg of its kappa (the current epoch first), else opens a new one; nothing
  is ever discarded and the state file is backed up on every write.
- Advisory bands are the Phase 0 table (Delta kappa 0.20 deg, Delta yaw 0.20
  deg within a world epoch, Delta p_b 1.0 mm, template residual 0.90 mm with
  the 1.0 mm hard cap kept, |lag| 13 ms; world axis point information only).
- One outcome line per attempt reports every advisory at once, in plain
  language. A refusal names an operator action only when that action is the
  remedy.
- The console prints startup state: pool n, `base_ref`, template path,
  pose and yaw source.
- `bb_moved` is deleted outright: parameter, arming log line, both gate
  override branches, the pool reset, and every launch-file, GUI and runsheet
  reference (grep first, count to zero after).
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

- Controlled-pair measurement first: the 2026-10-16 10:51 and 15:08 bags
  (one wand calibration, one world frame, kappa 179.57 vs 179.26): which of
  the 7 BB markers are labelled, the per-marker residuals, and kappa refitted
  on the marker subset common to all sittings and with label-free marker
  pickup (all visible markers near BB matched to the template, labelled or
  not). Kappa collapsing to one value = labelling/template artefact, Phase 2
  fixes it in software; kappa keeping two levels = BB, owner hardware check
  of the encoder and yaw mount. Then the same across every bag.
- One path and one statistic for the template residual. Reference level
  0.35-0.46 mm post-re-wand (not 0.26); advisory band 0.90 and hard cap
  1.0 mm unchanged; no remedy named until one is established.
- Why the shelf-marker residual against the as-built template went
  0.004 -> 0.30-0.35 mm and did not recover after the re-wand (Phase 3
  builds the robot frame on those markers).
- A labelling and coverage investigation, as a logbook entry with a
  recommendation: why only 5 of 7 BB markers label at rest (QTM3 in 4 bags,
  QTM5 never), why rest pair separations vary 0.5 to 4 mm bag to bag, and
  whether label-free pickup on our side makes labelling robust (owner
  question 2026-10-16). Any QTM-side change is the owner's to make.

Files likely touched: `bb_calibration.py`, `resources/bb_marker_template.json`
(only if the investigation justifies re-fitting; pose-sampling coverage
matters), `tests/ros/test_bb_*.py`.

Pinned by: a test that the same data gives the same residual verdict on any
pool size, and the replay.

Gate: replay still accepts the 25; the 0.52 to 0.76 mm sweeps appear as
advisories, not refusals; the controlled-pair result, with the subset-kappa
table, in the Phase 2 entry.

**Outcome (2026-10-16 evening): done.** Entry
`logbook/2026-10-16-bb-calibration-phase2-template-residual.md`. The step is
at BB: every marker subset, leave-one-out and label-free refit keeps it on 75
sweeps of 22 sittings, both sweep legs carry it, the axis point moved under
0.5 mm, and the BB firmware reads the encoder absolutely with a constant zero
(read for the owner). Label-free pickup is a null result (pair: 0 of ~80k
near-axis unlabelled points are BB markers; all bags 46 of 587k, kappa effect
at most 0.014 deg), so nothing lands in the node; no template re-fit (the
per-marker body-frame residuals flip sign between sittings); the residual
stays one statistic (pose-binned max over pairs, band 0.90 / cap 1.0
unchanged) with reference level 0.18-0.46 mm under one wand calibration and
a pool-size pin. No production behaviour changes: replay-harness columns,
tests, comments and docs only. Tests: baseline `./run_tests.sh` (2026-10-16,
log `temp/logs/run_tests_baseline_phase2.log`) 6666 passed, 11 skipped +
serial 4 passed; `./run_tests.sh --full` (2026-10-16, log
`temp/logs/run_tests_full_phase2.log`) 6672 passed, 1 failed, 11 skipped in
321.62 s + serial 7 passed in 28.40 s, the failure the known GUI replay load
flake (`pytest tests/ros/test_replay_overview.py -q` alone, 2026-10-16: 11
passed in 22.06 s); +7 test cases. Replay (`python
tools/probes/bb_calibration_replay.py --subset 1,2,4,6,7 -v`, 2026-10-16, run
`temp/probes/bb_calibration_replay/20261016-181127`): 90 sweeps, 82 accepted,
23 of 24 logged refusals accepted, the circle-centres sweep refused; the
subset-kappa table is the entry's § B. Owner decisions carried forward
unchanged: acceptance 2 wording; whether BB was powered off between 10:58
and 15:05 on 10-16. Next for the mechanism: the owner's BB-only power-cycle
A/B within one launch (entry § Open Questions). Phase 3 cleared to start, in
parallel with the acceptance-2 tally (3 of 10).

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
  the entries; the 2026-10-16 first sweep excluded reconstruction drift and
  Phase 2 excluded labelling, the template, timing and firmware (the step is
  at BB, mechanism open: owner's power-cycle A/B), and the plan does not
  depend on the answer.
- QTM wand-calibration age is hygiene (562 h old across the window); it is
  NOT a kappa fix (the 2026-10-16 re-wand did not move kappa) and the
  advisory names no remedy.
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
| 0 Characterise | done | 2026-10-16 | bands published in the Phase 0 entry; pose mix excluded; the power-cycle pin dropped (every session is one), replaced by the 2026-10-16 QTM re-wand + the first Phase 1 sweep |
| 1 Gate inversion and operator surface | code live, R1-R4 passed, tally 3 of 10 (15:08, 16:10, 16:23; pending acceptance rewording) | 2026-10-16 | merged c115743d; mechanism: at BB (Phase 2) |
| 2 Template residual | done | 2026-10-16 | controlled pair + 75 sweeps of 22 sittings: the step is at BB (every subset, leave-one-out and label-free refit keeps it; firmware excluded); no pickup, no re-fit, one statistic kept (reference 0.18-0.46 mm, pool-size pin); owner's power-cycle A/B next |
| 3 Robot frame in software | not started | - | design-bearing; after Phase 1 |
