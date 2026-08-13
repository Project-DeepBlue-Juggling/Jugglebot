---
title: "Critical-point ILC — E-1 implemented (whole-arc estimators, aim channels live) and Phase 2 shipped dormant (the layer-3 artifact and its seam)"
type: feature
date: 2026-08-13
status: resolved
phase: "critical-point-ilc — E-1 implementation + Phase 2 (post-Gate-1)"
related_plan: critical-point-ilc.md
files_changed:
  - plans/active/critical-point-ilc.md
  - logbook/2026-08-13-critical-point-ilc-e1-impl-and-phase2.md
  - logbook/INDEX.md
  - tools/probes/toss_record_miner.py
  - tools/probes/mocap_parity_bias.py
  - tools/probes/README.md
  - ros_ws/src/jugglebot/jugglebot/toss_record.py
  - ros_ws/src/jugglebot/jugglebot/motion/toss_ilc.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - config/hardware_config.yaml
  - tests/hardware/ilc_fit_lib.py
  - tests/hardware/ilc_fit.py
  - tests/motion/test_toss_ilc.py
  - tests/ros/test_toss_ilc_node.py
  - tests/motion/test_ilc_fit.py
  - tests/motion/test_toss_record.py
  - tests/ros/test_toss_record_miner.py
  - config/generated/hardware_config.py (+ the five sibling generated headers)
subsystem:
  - motion
  - tracking
  - ros
  - config
  - tools
tags:
  - testing
  - kinematics
---

# Critical-point ILC — E-1 implemented, Phase 2 shipped dormant

The Gate-1-approved unit, run as two sequential Opus agents (sequential
deliberately — the second consumes the first's re-mined corpus and both
touch `toss_record.py`/`ilc_fit.py`). Detailed results live in the plan's
§ E-1 and § Phase-2 results blocks (written this session); this entry is
the arc and the verification record.

**E-1 half.** The miner's lateral velocities at both plane crossings now
come from a whole-arc fit (bias-immune by parity; vertical stays
per-branch), with `coverage_asym_s` + `usable_for_lateral_fit` and the
parity diagnostic promoted to `tools/probes/mocap_parity_bias.py` with
committed provenance (it reproduces the load-bearing E-1 numbers exactly;
the position-lock rows are re-based per-group and the aero bound
recomputed — 3.9 m/s of wind at nominal drag, 1.5 at the implied bound). Re-mined: arrival and release directions now agree
(one shared lateral velocity), the arrival-y channel moved +10.3 mrad (the
predicted phantom was 10.9), and the lateral flight term collapsed
~58 → ~6 mm — lateral landing error is release-side, measured. The fit
lib's default mask is full-size (aim channels LIVE; `E1_MASK` historical)
and `ILC_SPEED_AUTHORITY = 0.15` landed per the Gate-1 owner decision.
The 3-DOF fit is consistent across all three goal cells (|aim|
0.0091–0.0105 rad, 52–60 % of the D7 clamp; `event_vel_trim` −0.1076
unchanged to six decimals). V1–V4 all PASS on the re-mined corpus.

**Phase-2 half.** `jugglebot/motion/toss_ilc.py` (the layer-3 per-goal
artifact: all-or-nothing parse, provenance dormancy on the toss_cal D3
pattern, Gate-1 key quantisation, exact-zero miss) + one apply seam beside
the aim-map lookup (`clamp_total_aim(map + trim + ilc)`, the existing
clamp final; a clamp hit refuses the ILC contribution whole), the
`validate_event_vel`-gated speed trim, applied values recorded per toss,
and `jugglebot_operational.toss_ilc_enabled: false` + codegen with the
tripwire test. The artifact writer accumulates `u` across fits
(`--from-artifact`) and re-validates the accumulated vector — Phase 3's
k ≤ 3 loop made real — and refuses unprovable provenance (a mined-only
corpus needs an explicit `--declare-toss-cal`).

## Discussion

The load-bearing choices and their whys are recorded in the plan next to
their numbers (the estimator-not-window E-1 resolution; the config-flag
decision — a learned correction's arming must be answerable from git; the
refuse-not-truncate clamp composition; the writer's provenance refusal).
Two demotions from the implementation kept the record honest:
`coverage_asym_s` is a gross-truncation guard, not a leak predictor (a
0.0003 s row still leaks 6 mm/s; ~1.4 mrad standing lateral uncertainty),
and the E-1 "indoor wind" bound weakens to 1.5 m/s at the data-implied
drag bound — the draft refutation rests on the two-sitting position-lock
and the blob census. The absolute-bias caveat became numerically visible:
`land_err` (plane-position, carries b_y absolutely) and `arrival_dir`
(whole-arc, bias-immune) disagree by a systematic +18 mm in y — the
measured b_y span — so the pooled aim fit is a compromise between two
channels that disagree about the world; the static fixtured-ball capture
is the discriminator.

**Pre-commit audit (Opus, 2026-08-13): consolidated 4 BLOCKING /
14 WARNING — all applied same-day** (code by an Opus fix agent, every fix
mutation-verified; narrative by the orchestrator). The runtime shipped
clean — the auditor independently verified OFF-path byte-identity on the
branch the old test did not cover, and that only lateral channels moved in
the miner; every BLOCKING bites at Phase 3, in the artifact writer. What
the fixes closed: `--from-artifact` accumulated onto an UNCHECKED prior
(the seed's provenance now goes through the production
`provenance_mismatch` gate and each cell's seeded u through
`admit_command` — well-formedness never implied fitted-on-this-plant, and
`admit_command` is not monotone in u); the pooled write validated every
cell against the MODAL cell's geometry (now each cell's own goal —
mutation test showed the old code writing a cell for a goal outside the
sequencer band); the byte-identical-OFF test was VACUOUS (its fixture had
no map/trim so `_toss_aim_for_goal` early-returned before the rewritten
composition arm — replaced with an aimed fixture, a reflection fingerprint
over every field, and a pre-Phase-2 arithmetic ORACLE arm, the only
structure that can catch a composition rewrite); an unguarded
`provenance_mismatch` on the goal-build path could kill `_build_toss_cycle`
on codegen skew instead of failing closed (now zero-correction + WARN, and
the same guard extended to the status snapshot on the Trigger path); the
COVERAGE_ASYM sizing docstrings were false against their own corpus
(restated with named populations); and the phantom-aim headline was 44 mm,
not 54 (production landing gain, verified at 4006.7 mm/rad — corrected at
every site including two committed entries, dated).

Codegen side effect: the generator also rewrote
`../BallButler/ball_butler_main/hardware_config.h` (+12/−4) in the
separate BallButler repo — left for the owner to commit there, and it is a
**drift catch-up, not a Phase-2 change**: only `TOSS_ILC_ENABLED` is this
change-set's; the other eleven lines are accumulated Jugglebot-side drift
being flushed for the first time, several behavioural for BB firmware
(`CATCH_VEL_SCALE_DEFAULT` 0.8→0.9, `TOSS_TIER` "8b"→"8a",
`TOSS_REQUIRE_BALL_EVIDENCE` false→true, `LEVELLING_SETTLE_S` 0.5→1.0,
plus four new `TOSS_SESSION_*` constants) — review, don't rubber-stamp.

## Verification

- E-1 half (run 2026-08-13): miner `--self-check` **56/56**;
  `mocap_parity_bias.py --self-check` **23/23** (reproduces the E-1
  evidence number-for-number); `pytest tests/motion/test_ilc_fit.py
  tests/ros/test_toss_record_miner.py tests/motion/test_toss_record.py
  tests/ros/test_ilc_measurement_probes.py -q -p no:cacheprovider`:
  **164 passed in 57.63 s**; CLI `--validate` on the re-mined corpus:
  V1–V4 PASS.
- Phase-2 half (run 2026-08-13, pre-audit): `pytest tests/motion tests/ros
  -q -p no:cacheprovider -n 4 --dist loadfile`: **3757 passed in 195.57 s**
  (post-audit-fix rerun of the same sweep, serial: **3768 passed in
  550.47 s**).
- The 11-file toss-stack battery, post-audit-fix (`pytest
  tests/motion/test_toss_ilc.py tests/motion/test_ilc_fit.py
  tests/motion/test_toss_record.py tests/motion/test_toss_trim.py
  tests/ros/test_toss_ilc_node.py tests/ros/test_toss_record_miner.py
  tests/ros/test_ilc_measurement_probes.py tests/ros/test_toss_calibration.py
  tests/ros/test_toss_record_publisher.py tests/ros/test_toss_sequencer.py
  tests/ros/test_toss_coordinator.py -q -p no:cacheprovider`, run
  2026-08-13): **677 passed in 103.07 s** (666 pre-fix).
- Pre-audit full gate (`./run_tests.sh -q`, run 2026-08-13): **PASS — 5329
  passed, 5 skipped, 0 failed in 244 s** (a concurrent audit-side serial
  rerun hit one load-flake on `test_publish_reads_cache_only_no_round_trip`;
  the gate run itself was green).
- `/audit --unstaged` (Opus): consolidated **4 BLOCKING / 14 WARNING** —
  all applied same-day, dispositions below. Post-audit-fix full gate: the
  triple follows in this entry's final bullet.
- Post-audit-fix full gate on the final tree (`./run_tests.sh -q`, run
  2026-08-13): **PASS — 5340 passed, 5 skipped, 0 failed** (parallel 236 s
  rc=0, serial 8 s rc=0, total 244 s; counts tallied from the captured run
  log).
- `colcon build --packages-select jugglebot` is REQUIRED before the next
  launch (node + config changes); not run desk-side.
