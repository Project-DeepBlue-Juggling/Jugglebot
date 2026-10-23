---
title: Two-ball oval P3 - the pose-level carry planner lands behind `planner_mode` (level | pose), C-CUP-4 becomes QP rows, the z band is measured and pinned, the box is stamped with the planner
type: feature
date: 2026-10-23
status: resolved
phase: "two-ball-oval — P3"
related_plan: two-ball-oval.md
related:
  - 2026-10-18-two-ball-oval-p1-oval-pattern-dispatchable.md
  - 2026-10-23-oval-p2-banking-leak-not-reproduced.md
  - 2026-10-23-oval-rung2-flown-dwell-050.md
files_changed:
  - "config: config/hardware_config.yaml (trajectory_op.planner_mode, pose_*) + the regenerated hardware_config.py/.h"
  - "planner: ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py (PoseConfig, PoseSpec, _assemble(pose=), _pose_augment, plan_window(pose=), CupCyclePlan.platform_dz)"
  - "planner: ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py (prescribed_tilt_fn, _quintic_bridge, decompose reads platform_dz)"
  - "planner: ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py (PLANNER_MODES, POSE_WINDOW, CycleGoals.planner_mode/pose_cfg, CycleMeta.planner_mode, _pose_spec_for, _seed_tilt_pins, _receive_pin_for, _throw_tilt_from_velocity)"
  - "planner: ros_ws/src/jugglebot/jugglebot/motion/skills/segments.py (SegmentConfig.planner_mode, _planner_mode)"
  - "box: ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py (planner_mode stamp; select/describe_miss require it)"
  - "box: tools/admissible_sweep.py (--planner-mode), config/generated/admissible_box.yaml (re-swept: every level cell value-identical, three pose oval cells added)"
  - "nodes: ros_ws/src/jugglebot/jugglebot/skill_node.py (planner_mode parameter, box select, request field), trajectory_node.py (request mode -> SegmentConfig), motion/skills/executor.py (planner_mode attribute)"
  - "wire: ros_ws/src/jugglebot_interfaces/srv/InstallSegment.srv (planner_mode request + response)"
  - "sim: sim/skills_gate.py (planner_mode gate knob)"
  - "probes: tools/probes/oval_dwell_sweep.py (--planner-mode)"
  - "tests (new): tests/motion/test_pose_planner.py"
  - "tests: tests/motion/test_unified_cycle_budget.py (pose-mode case), tests/motion/test_cup_cycle.py, tests/motion/test_skills_admissible.py, tests/ros/test_install_segment.py, tests/ros/test_skill_node.py, tests/sim/test_skills_gate.py"
  - "docs: ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md, plans/active/two-ball-oval.md, tests/hardware/session_oval_rung1.md, logbook/INDEX.md"
subsystem:
  - skill-stack
  - motion-planner
tags:
  - oval
  - c-cup-4
  - planner
  - safety
  - testing
---

# Two-ball oval P3 - the pose-level carry planner

## Summary

The folded STEADY window (catch at C, carry, release at T) can now be solved with a second
formulation, the **pose-level carry planner**, behind `SegmentConfig.planner_mode` (`level` |
`pose`, default `level`). In `pose` mode the tilt is PRESCRIBED before the QP (seam pin ->
receive pin -> throw pin, quintic sweeps, the throw pin held 2 knots), the objective adds the
LEG jerk of the composed pose through one fixed Jacobian, and the C-CUP-4 seating rule
becomes QP rows (20 deg cone about the prescribed axis, 0.5 g floor) instead of only a
post-solve check. Measured through the production segment path with the real holds (the
prototype never saw them): at the rung-2 baseline cell (150 mm, apex 0.95, dwell 0.50) the
carry's window jerk falls **111.5k -> 44.3k mm/s^3**, leg acceleration -42 %, the realised
seating residual **32.1 -> 18.6 deg**; the 0.45 cell becomes admissible at today's limits;
and at dwell 0.40 the first refusal moves from the cone (49.1 deg) to **leg velocity**
(357 > 350 mm/s), which is P4's working-point raise. Decision 24's offline gate (residual <=
20 deg on every interior knot at 150 mm AND peak leg jerk <= the level baseline) passes at
every box cell. Three owner decisions re-shaped the plan's design on the measurements: the free
centroid-z band SHIPS AT 0 (it spends leg velocity and buys no jerk), the touch-down skid the
leg-jerk objective introduces is ACCEPTED as the logged diagnostic it already was, and the
receive-pin holds go to 0 / 0. The admissible box is stamped with the planner it was swept
under and `select` refuses a mismatch by name; three pose-swept oval cells coexist with the
level cells. The default stays `level`; the hardware gate (rung 1 re-flown as the swept
column, 30 attempts, catch rate >= as-is with leg current not higher) is the next sitting.

## What landed

- **`cup_cycle.py`** — `PoseConfig` (the named knobs, defaults from `trajectory_op.pose_*`),
  `PoseSpec` (tilt callable, Jacobian, seed dz state), `_assemble(pose=None)`: the level
  program is assembled UNCHANGED and `_pose_augment` appends its blocks after every existing
  column; `pose=None` adds not one byte (the T-U2 parity fixtures and the box's value-equal
  re-sweep pin it). `plan_window(pose=)`; `CupCyclePlan.platform_dz` (trailing, `None`
  unless the z band is on). The cone is written about the PRESCRIBED cup axis (`R_k^T (a - g)`
  in the cup frame), not about vertical as the ticket-09 prototype did.
- **`cup_realize.py`** — `prescribed_tilt_fn(...)`: the piecewise series as a callable of
  time (hold seam / `_quintic_bridge` seam->receive, C1 at the seed per I-PLAN-11 / hold
  receive / smoothstep receive->throw / hold throw), refusing an eaten window by naming the
  hold; `decompose` reads `platform_dz` (pose z = active + dz, slider -= dz, analytic rates),
  guarded so the `None` path is byte-identical.
- **`unified_cycle.py`** — `PLANNER_MODES`, `POSE_WINDOW`; `CycleGoals.planner_mode` /
  `pose_cfg`; `_pose_spec_for` builds the spec (pins, catch knot computed exactly as
  `_assemble` does, Jacobian at mid-site / active z / level, seed dz) only for a STEADY with
  both events and no held axis; `_realize` takes the prescribed series instead of calling
  `tilt_schedule`; `CycleMeta.planner_mode` names what was SOLVED (a pose session's LAUNCH /
  LANDING / SETTLE report `level`). `_seed_tilt_pins` / `_receive_pin_for` /
  `_throw_tilt_from_velocity` are the pin computations lifted out of `_realize` so the spec
  builder and the realisation share them.
- **`segments.py`** — `SegmentConfig.planner_mode` (default `hardware_config`), read only by
  the folded CATCH+then_throw; `_planner_mode` refuses an unknown value by name.
- **`admissible.py` / `tools/admissible_sweep.py`** — the `planner_mode` stamp (unit A, see
  the test recipes).
- **Nodes and wire** — skill_node's `planner_mode` parameter is the ONE live switch: it selects
  the box cell and rides on every `InstallSegment` request; trajectory_node plans that install
  with it (its own `SegmentConfig.planner_mode` is the default for a request carrying none);
  the response reports the mode solved (unit B).
- **Probes** — `tools/probes/oval_dwell_sweep.py --planner-mode`: the production ladder in
  either mode (the design probe that produced the tables below was a scratchpad monkeypatch,
  not committed; the production tool reproduces its jerk and residual to the integer and its leg velocity / acceleration within one unit).

## The ladder, through the production path (150 mm, apex 0.95, 350/5000/200000, hand 3900)

Window = the folded STEADY's extended segment report; `pose` = z band 0, h_pre 2, h_catch 0/0,
w 10, cone 20 deg / 0.5 g. Measured 2026-10-23 (`tools/probes/oval_dwell_sweep.py --mode asis
--planner-mode {level,pose}`, HEAD d36ef389 + this phase's working tree).

| dwell | level: jerk / vel / acc / residual / skid | pose: jerk / vel / acc / residual / skid | pose QP ms |
|---|---|---|---|
| 0.65 | 58,742 / 168 / 1666 / 17.8 deg / 14 | 44,329 / 168 / 970 / 12.0 deg / 225 | 23 |
| 0.60 | 71,428 / 183 / 1937 / 21.5 deg / - | 44,329 / 185 / 1104 / 16.9 deg / - | - |
| 0.55 | 88,313 / 203 / 2291 / 26.2 deg / 12 | 44,329 / 208 / 1303 / 18.6 deg / 379 | 16 |
| 0.50 | 111,510 / 227 / 2769 / 32.1 deg / 33 | 44,329 / 242 / 1591 / 18.6 deg / 494 | 17 |
| 0.45 | 143,763 / 259 / 3509 / 39.4 deg / 65 | 56,479 / 294 / 1989 / 18.6 deg / 645 | 18 |
| 0.40 | REFUSED `CUP_SEATING_CONE` 49.1 deg | REFUSED `LIMIT_VEL` 357 mm/s (18.7 deg) | 17 |
| 0.35 | REFUSED `LIMIT_VEL` 387 (56.3 deg) | REFUSED `LIMIT_VEL` 431 (18.7 deg) | - |

The pose arm's window-jerk floor (44,329 at 0.65 / 0.60 / 0.55 / 0.50) is not the carry's:
`validate_cycle` places it at leg 5, 150 ms AFTER the release — the SETTLE tail the extended
segment carries (the post-release hand retract on a stationary platform), identical in both
modes and the same number the level arm's tail would read if its carry did not exceed it
(level's peak sits before the release, leg 2, 100 ms out: the carry). So on these cells the
pose planner has taken the carry's jerk BELOW the tail's; what the knob arms moved was the
carry, read where the window peak left the tail (0.45 and below: leg 3, before the release).

## Design decisions, measured (owner, 2026-10-23, `AskUserQuestion`)

The design probe ran the formulation of ticket 11 as a monkeypatch on the PRODUCTION chain
(`_hop_throw_cell` + `_folded_catch_cell`, `SegmentConfig()` defaults: 0.100 s pre-release
hold, 0.050 s post-release hold) across dwell 0.65..0.30 and 20 knob arms. Four decisions:

1. **Free centroid z: pinned by default, knob kept** (`pose_z_band_mm: 0`). A +-30 mm band
   SATURATED on every accepted cell and cost +57 / +62 / +61 mm/s of leg velocity and
   +270 / +320 / +320 mm/s^2 of leg acceleration at 0.65 / 0.55 / 0.50 for the SAME window
   jerk (44.3k / 44.3k / 48.3k vs 44.3k / 44.3k / 49.8k pinned); it refused the 0.45 cell on
   the sweep margin (347 mm/s) where pinned z passes (297, 65.0k, 18.6 deg); the QP ran 25-37
   ms against 17-23. Mechanism: leg velocity is not in the objective, so a z channel is spent
   riding the band to shave jerk that is not the window's peak. Decision 33 (the seam-dz lever,
   decision 58's ablation) closes on this null: with no dz block there is no seam dz; a free
   seam could only add platform z motion on top of a channel that already costs velocity.
   The owner's first reading of the table was the reverse ("restricting z generates higher
   vel/acc?"); the corrected reading was put back as a table and the decision taken on it.
2. **Skid: accepted, logged** (decision 28b stands). Under the leg-jerk objective the cup
   arrives at C moving across the receive axis: 225 / 379 / 494 mm/s at 0.65 / 0.55 / 0.50
   (level 14 / 12 / 33; the ball arrives at ~4 m/s with ~170 mm/s lateral; the BB feed catch
   tolerates 1.3 m/s of ball lateral into a parked cup). The two levers were measured and
   declined: the lateral catch-velocity weight x100 reads 10 / 48 / 105 mm/s but costs +18 %
   window jerk at 0.50 and refuses 0.40 on `HAND_LIMIT_ACC` 4308; a hard lateral match at
   touch-down reads 55 mm/s (the soft z match's residual) at +28 % jerk / +67 % leg acc and
   refuses 0.45 on the hand. The hardware A/B column (seat-bit continuity, raw dropouts vs the
   rung-2 69 % at 0.50) judges it; the lateral weight is the recorded first lever.
3. **Knobs: h_pre 2, w 10, h_catch_pre 0, h_catch_post 0.** h_pre 0 -> hand 4166 rev/s^2 at
   0.40 (the release-seam braking spike ticket 11 named); h_pre 4 -> +35 % carry jerk at 0.50.
   w 1 -> leg velocity 427 vs 401 at 0.40; w 30 -> `HAND_LIMIT_ACC` at 0.45. The receive-pin
   hold 2 / 1 cost 11 / 13 / 10 % of window jerk at 0.50 / 0.45 / 0.40 (49.8k -> 44.3k, 65.0k
   -> 56.5k, 85.9k -> 77.5k) and changed nothing else: the quintic already arrives at the pin
   with zero tilt rate AND acceleration, so "the platform is not rotating while the ball
   lands" holds without it. All four stay named config knobs.
4. **The mode switch**: `SegmentConfig.planner_mode` + ONE live node parameter + the box stamp.
   Single-owner refinement: the parameter lives on skill_node (it selects the box) and rides
   on every `InstallSegment` request; trajectory_node plans with the request's value. Two
   parameters would have been the unenforced twin `dwell_s` closed.

Also measured and kept: cone 30 deg instead of 20 buys no jerk (44.3k / 48.7k vs 48.3k) so the
plan's 20 deg target is free; floor 0.3 g instead of 0.5 likewise; the cone rows do ALL the
seating work (without them the pose arm reads 37-76 deg); the leg-jerk objective is what makes
the dz block usable at all (cone + dz without it: 878k mm/s^3).

## Discussion

### Why this approach over alternatives

*Prescribed tilt, not a tilt variable.* `shift = (cup_z - C) * axis_xy(rx, ry)` is bilinear in
(cup_z, tilt) and trigonometric; the Goldfarb-Idnani dual cannot take it. The pins depend only
on the event velocities, all known before the QP, so with them fixed the composition is affine
in the jerk variables and the program stays a convex QP solved by the same solver. The
realisation samples the SAME callable the QP differentiated: one series by construction, and
`tilt_schedule` is not consulted (a second, disagreeing attitude series is exactly what the
prescription removes).

*Rows appended after every level column, `pose=None` a no-op.* The level program's column
order is what the T-U2 parity fixtures, the warm-start keys and the admissible box's pinned
values all depend on; the pose blocks are appended (the discipline the runway row, the acc
boxes and the C-CUP-2 floor already follow), so level mode is the pre-P3 planner byte for byte
and the re-sweep's level cells are value-identical.

*The cone about the PRESCRIBED axis, not vertical.* The check (`_cup_seating_check`) judges the
residual against the prescribed cup axis; a shaper about vertical would hold a different angle.
With the series known before the QP the cup-frame rotation is a constant per knot and the rows
stay linear.

*Measure through the production holds first.* P1 found the prototype's hold-less chain
mis-read the rung-1 cell by 13 deg. The design probe therefore ran the prototype's formulation
on the production chain BEFORE any production code was written, and every knob default above
is a production-path number; the prototype's h_catch 2 / 1 and band 30 did not survive it.

*One owner for the mode.* The box selection (skill_node) and the solve (trajectory_node) had
to agree; the `dwell_s` precedent (a node parameter against a swept value) was the model, and
carrying the value on the request is the only way one parameter reaches both.

### What was ruled out

- **The z band as shipped behaviour (ticket 11's "the z band STAYS")** — withdrawn on the
  measurement above (trigger a). The prototype compared pose-with-band against the LEVEL
  planner (leg acc -43 %) and never isolated the band against pose-with-z-pinned; on the
  production path pinned z has the lower velocity and acceleration at every cell and the same
  peak jerk. The knob stays for a hardware A/B.
- **The receive-pin holds (plan decision 32's 2 / 1)** — measured cost 10-13 % of carry jerk,
  no measurable benefit; the plan allowed zero once the cost was known.
- **Lateral catch weight / hard touch-down match for the skid** — see decision 2; both trade
  jerk and hand acceleration for a number decision 28b made a diagnostic.
- **The seam-dz ablation as a code experiment** — moot with no dz block; closed as a null with
  the band measurement rather than built.
- **Two `planner_mode` parameters (one per node)** — the unenforced twin; replaced by the
  request field.
- **Per-knot Python loops in the assembly** — the probe's assembly cost 26-35 ms per window
  (more than its solve); production vectorises the leg-jerk rows (one `einsum`) and the cone
  rows (one broadcast). The budget test's pose case exists to keep that so.

### Tradeoffs accepted

- **Skid 225-494 mm/s at touch-down** for the carry's -50..-60 % jerk and the 18.6 deg
  residual; judged on hardware, lever recorded.
- **Leg velocity +0..+7 %** at the box cells under pose (168 / 208 / 242 vs 168 / 203 / 227)
  — the carry is quicker where it is free to be; 0.40 is refused on it at today's 350 mm/s.
- **The planner core budget is tighter** in pose mode on the longest oval cell: 43.0 ms core (plan minus
  gate; total 50.4 ms) against level's 40.4 on the same 61-knot window and the owner's 50 ms — the pose
  solve is 37 ms vs 25 (54 vs 49 active-set iterations, the cone rows) but it skips the 10 ms banking
  schedule. Measured 2026-10-24 on the idle Jetson, 30 solves. The 1.4 s reference cycle of 2026-09-12
  read 18.4 ms; the folded oval window with its holds is simply a heavier solve in BOTH modes.
- **Pose applies to the folded STEADY only.** LAUNCH / LANDING / SETTLE and the held feed
  catch keep the planner every rung has flown; a pose session reports `level` on them. The
  whole-chain peak in the one-ball cells sits in those windows (ticket 11), so the pose
  planner lowers the CARRY's jerk and leaves the chain's peak where it was.
- **The default stays `level`** until the hardware gate; the box carries both planners' oval
  cells, so a sitting flips one parameter.

### Open questions

- The 44,329 mm/s^3 window-jerk floor inside the pre-release hold: whether a release-hold
  shaped for the pose planner (the throw pin's lever with the stroke) lowers it is a P4 item
  if the chain's peak ever binds there.
- The raw seat-bit dropout rate (69 % of carries at 0.50 on the level planner) is the physical
  watch item the swept column reads against; the skid is the first suspect if it rises.

## Test recipes

| test id | driver | fixture | fail-before evidence |
|---|---|---|---|
| `test_pose_planner.py::test_pose_passes_decision_24_offline_gate_at_the_box_cells[0.65/0.55/0.50]`, `::test_pose_rung2_baseline_cell_reads_the_measured_values`, `::test_pose_through_plan_segment_matches_plan_steady`, `::test_pose_accepts_dwell_045_where_level_reads_39_deg` | production segment path: `plan_segment(THROW)` from rest at T, then `plan_steady` / `plan_segment(CATCH + then_throw)` with `planner_mode` | `columns_sites(150)`, apex 0.95, 350/5000/200000, hand 3900 | the feature did not exist (`CycleGoals` had no `planner_mode`; `TypeError`); the asserted numbers are the probe's, reproduced to the integer by the production port on 2026-10-23 |
| `::test_pose_cone_rows_bind_the_realised_residual`, `::test_pose_floor_rows_bind_the_realised_seating_force` | `PoseConfig(cone_deg=30)` / `(floor_g=0.3)` against the defaults | dwell 0.50 | the realised residual / minimum `a_z + g` FOLLOW the row parameters (28.1 deg at 30; 0.54 g at 0.3) — the rows act; the no-cone ablation of the probe read 45 deg at 0.50 |
| `::test_pose_free_z_band_assembles_the_dz_block_and_rests_at_the_release`, `::test_pose_seam_refuses_an_off_height_seed_with_the_band_at_zero` | `PoseConfig(z_band_mm=30)`; a seed 10 mm off the active height | dwell 0.50 | band on: `platform_dz` inside +-30 mm, at rest at the release, leg velocity ABOVE pinned (the shipped-default evidence); band 0 + off-height seed: `POSE_SEAM`; band 30 + the same seed: accepted with `dz0` = 10 mm |
| `::test_pose_window_refusal_names_the_hold`, `::test_an_unknown_planner_mode_is_refused_by_name` | `PoseConfig(h_pre_knots=200)`; `planner_mode='banana'` on goals and on `SegmentConfig` | - | `POSE_WINDOW` naming `h_pre`; `ValueError` naming `planner_mode` |
| `::test_pose_mode_leaves_launch_and_settle_as_the_level_program`, `::test_pose_spec_is_none_for_a_held_axis_steady_and_for_a_landing` | `plan_launch` / `plan_settle` in pose mode vs level; `_pose_spec_for` with `hold_tilt` / LANDING | - | arrays `np.array_equal`; meta reports `level`; spec `None` |
| `::test_prescribed_tilt_fn_pins_holds_and_is_at_rest_at_every_pin`, `::test_prescribed_tilt_fn_honours_the_seed_rate_and_refuses_an_eaten_window` | the callable alone | n 60 / 40 | pins exact, rate and acceleration zero at every pin (1e-6 / 1e-3), seed rate honoured to 1e-4, holds widen the flat region, eaten windows and a pin outside the cone raise naming the field |
| `::test_decompose_applies_platform_dz_analytically_and_leaves_none_untouched`, `::test_pose_spec_validates_its_inputs` | hand-built flat cup plan with / without `platform_dz` | n 20 | `None` and all-zero dz realise `np.array_equal`; a sine dz moves pose z, slider and slider rate analytically; wrong shapes refused |
| `test_unified_cycle_budget.py::test_pose_mode_steady_fits_the_same_budget` | `plan_steady` in pose mode on the 61-knot oval cell, 30 solves | 150 / 0.95 / 0.65 | failed on the loaded box (core 62-75 ms with the sweep and an agent's tests running), passes idle: core 43.0 ms; the prototype's per-knot assembly (26-35 ms alone) would have failed it idle, which is what it guards |
| `test_cup_cycle.py::test_cup_cycle_plan_mirrors_the_sim_dataclass_fields` | the trailing `platform_dz` field | - | red after the field landed; updated |
| `test_skills_admissible.py::test_planner_modes_vocabulary_equals_unified_cycles`, `::test_round_trip_dump_then_load_preserves_planner_mode`, `::test_round_trip_dump_then_load_tolerates_an_unstamped_planner_mode`, `::test_box_rejects_a_planner_mode_outside_the_vocabulary`, `::test_select_refuses_a_cell_swept_under_the_other_planner_and_names_it`, `::test_a_planner_mode_miss_is_reported_before_a_dwell_miss`, `::test_pose_and_level_cells_coexist_at_one_pattern_pair_dwell_and_band`, `::test_overlap_refusal_keys_on_planner_mode`, `::test_select_treats_an_unstamped_box_as_matching_no_planner_mode`, `::test_select_and_describe_miss_require_planner_mode` (unit A, Sonnet) | the stamp mirrored on the `dwell_s` tests | hand-built boxes | written first, run against the unedited module: 9 failed / 1 passed (`AttributeError ... no attribute 'PLANNER_MODES'`; `AdmissibleBox.__init__() got an unexpected keyword argument 'planner_mode'`; the `TypeError` test passed trivially before the keyword existed) |
| `test_skills_admissible.py::test_p3_resweep_changed_only_the_stamp_and_added_the_pose_cells` (+ the pre-oval snapshot test widened to drop the stamp) | committed YAML vs `tests/fixtures/admissible_box_pre_p3.yaml` (HEAD's box) | the 22 pre-P3 cells | red until the re-sweep landed (unstamped, stale gate hash) |
| `test_install_segment.py` (4 new: a `pose` request plans under pose, `''` uses the node default, `'banana'` is refused `PLANNER_MODE` with nothing solved, the node default) (unit B, Sonnet) | mock-ROS `InstallSegment` request through `_svc_install_segment` | the file's folded-catch fixtures | before the srv/mock/node change: `AttributeError: 'Response' object has no attribute 'planner_mode'`; after the srv + mock change but before the node change: 3 of 4 failed (pose and `''` planned level, `'banana'` accepted) |
| `test_skill_node.py` (4 new: default `level` on every request, parameter `pose` on every request, a level box refused by name under `pose`, a pose box selected) | the node's request builder and accept path | boxes stamped per test | written after the node edit (the mock request had no field before it; recorded as such) |
| `test_pose_planner.py::test_a_pose_segment_spliced_onto_a_level_record_reports_its_own_mode` | `uc.splice_at` of the folded pose segment onto the level THROW record at its release knot | the oval cell at 0.50 | audit finding: `splice_at` inherited the running record's `level` so every pose install replied `level`; red before the one-line `splice_at` fix, green after (both modes asserted) |
| `test_skill_node.py::test_planner_mode_cannot_change_while_a_goal_is_active` | `set_parameters` during an active columns goal | the file's columns fixtures | audit finding: a mid-goal `planner_mode` flip diverged the request from the executor's frozen box selection; refused by name (`a goal is active`), the same value re-set allowed, applies between attempts |
| `test_skills_gate.py` (2 new: config validation / threading; one short `oval_1ball` run under `pose` on the committed pose cells) | the sim gate with `planner_mode='pose'` | gate cfg | the run first failed with an overlap refusal because it re-stamped the level oval cells `pose` while the re-swept box already carried pose cells (duplicate key) — now it reads the committed pose cells; 28 s under load, default tier like the file's other learner runs |

## Audit (one pass, phase end: `/audit --unstaged`, 2026-10-24)

No BLOCKING findings. Two behaviour findings, both fixed in the same session with a regression test each: (1)
`unified_cycle.splice_at` built the spliced meta from the running record's `planner_mode`, so a pose CATCH spliced onto
a level THROW record (the normal install) replied `level` and would have fired skill_node's MISMATCH warning on every
folded catch of a pose sitting — the spliced meta now carries the installed segment's own mode (`extend` already
carried the head's for the folded join, found by unit B); (2) `ros2 param set /skill_node planner_mode` was accepted
mid-goal while the executor's box selection is frozen at accept — refused by name while a goal is active, the guard the
debug parameter already had. Narrative findings fixed: the YAML comment quoted the retired 2/1-hold number (49.8k) for the
shipped defaults (44.3k); the plan carried a literal SHA placeholder (replaced by the `git log --grep Logbook-Entry`
reference, the canonical link); the design probe's velocity / acceleration figures differed from the production tool's
by one unit (356 vs 357 mm/s, 1590 vs 1591 mm/s², 167 / 207 vs 168 / 208) — the tool's numbers now stand everywhere and
"to the integer" is claimed for jerk and residual only; the band-cost range read +57..+61 where the 0.55 cell measured
+62; an unused `rotvec_to_rot_matrix` import in `cup_cycle`; the `POSE_SEAM` tolerance was 1 nm (now 1 µm / 1 µm/s,
inert today). Four of those edits touched gated files, so the box was re-swept a third time (two rounds) after them —
the Verification bullet above records the final stamp.

## Xfail accounting

None.

## Verification

- Final full gate: `./run_tests.sh --full`, run 2026-10-24 02:40 on the final tree (after the audit fixes and the third re-sweep; idle box): **parallel 7140 passed / 4 skipped in 328.54 s; serial 8 passed in 29.54 s; RESULT PASS, exit 0** (`temp/logs/p3_final_full2.log`) — +42 parallel tests and +1 serial over the 7098 / 7 baseline (the pre-audit tree read 7138 / 8 at 01:20, `temp/logs/p3_final_full.log`).
- `pytest tests/motion/test_pose_planner.py -q`, run 2026-10-24 on the final tree: **19 passed**
  (38 s under the sweep's load on the first run, 2.3 s idle).
- Serial budget tests (`pytest tests/motion/test_unified_cycle_budget.py tests/motion/test_validate_cycle_budget.py
  tests/motion/test_solver_worker_lateness.py -q`, run 2026-10-24 01:17 on the idle box): **5 passed in 10.65 s**
  (the pose case included). The two wall-clock tests that failed under the sweep's load re-run clean idle:
  `test_unified_cycle.py::test_the_qp_and_the_gate_stay_within_an_order_of_magnitude` (its own docstring's
  starvation signature) 1 passed; `test_install_segment.py -k real_worker_process` 1 passed.
- Hypothesis depth: `pytest tests/motion/test_unified_cycle.py -q --hypothesis-profile=ci-deep
  --hypothesis-seed=0`, run 2026-10-23 under the sweep's load: **86 passed, 2 skipped, 1 failed in 527 s** —
  the one failure is the ratio test above (load), green idle; no new hypothesis test was added this phase.
- Box-dependent suites after the merge (`pytest tests/motion/test_skills_admissible.py
  tests/motion/test_skills_executor.py tests/ros/test_skill_node.py tests/sim/test_skills_gate.py -q`, run
  2026-10-24 00:50-01:22 under the round-c sweep): **762 passed, 2 failed** — unit B's extra per-goal INFO
  line against the one-start-line operator contract (line removed) and its sim pose test's duplicate
  stamping (fixed to read the committed pose cells); both green on re-run (9 passed).

Baseline: `./run_tests.sh --full`, run 2026-10-23 on d36ef389 before any edit: **parallel 7098
passed / 4 skipped in 319.57 s; serial 7 passed in 27.82 s; exit 0**
(`temp/logs/p3_baseline_full.log`).

- `colcon build` (the `InstallSegment.srv` change rebuilds the interfaces): `cd ros_ws && colcon build
  --packages-select jugglebot_interfaces jugglebot`, run 2026-10-23 after the last node edit: **2 packages
  finished in 8 min 25 s (jugglebot_interfaces 8 min 21 s, jugglebot 2.93 s), exit 0**
  (`temp/logs/colcon_p3.log`). A FULL launch restart is owed before the A/B sitting (new interfaces; GUI
  hard refresh).
- Box re-sweep (every family: the five gated planner files changed): `bash temp/sweeps/sweep_P3.sh` (nine
  invocations per round, `--leg-vel 350 --leg-acc 5000 --leg-jerk 200000 --hand-acc 3900`, BLAS one
  thread: single / hop / columns at dwell 0.27 under `--planner-mode level`; oval 150 mm / apex 0.95 ±
  0.025 at dwell 0.65 / 0.55 / 0.50 under `level` AND under `pose`), round a 2026-10-23 23:21 → 2026-10-24
  00:11, round b 00:11 → 00:36 (first round killed and restarted 23:20 after the `extend` fix below moved
  the gate hash). The seven files that exist in both rounds are **byte-identical after line 1** (line 1
  is `swept_at`: round a straddled midnight). **Round a's single and hop files do not exist**: those two
  ladders sweep several apexes per process and stamped each box's `swept_at` at its own write time, so the
  midnight straddle put two dates in one file and `admissible.dump` refused it (correctly). Fixed in the
  tool the same night (one date per invocation, passed into every apex's sweep call; the stub test asserts
  it); the single and hop second run (round c, `temp/sweeps/sweep_P3_round_c.sh`, 00:50 → 01:16) is
  **byte-identical to round b for both families, line 1 included**, so every one of the nine families
  is two-run identical; that merge (gate_hash f8076a0a6780) was then superseded by the audit's four
  gated-file edits, and the WHOLE sweep ran a third time (`bash temp/sweeps/sweep_P3.sh`, 01:44 → 02:34,
  both rounds after midnight: all nine families `cmp` IDENTICAL, line 1 included). Final merge
  (`scratchpad/merge_p3_box.py`, round b files in the committed order single → hop → columns → oval 0.65 /
  0.55 / 0.50 level → the three pose cells): **25 cells, gate_hash e3eb288fc787 == the live gate, swept_at
  2026-10-24, every one of the 22 level cells value-identical to `tests/fixtures/admissible_box_pre_p3.yaml`
  (HEAD's box), merged-file md5 4f5cd25f5177b177a009a308bafc4aae**. The three pose cells read the same admitted set as their level
  twins (landing ±20 mm, apex band [0.608, 1.047]) — the pose planner admits every offset the level one
  admits at these cells.
- Production ladder, both planners (`python tools/probes/oval_dwell_sweep.py --mode asis --planner-mode
  {level,pose} --dwells 0.65,0.60,0.55,0.50,0.45,0.40,0.35`, run 2026-10-23, outputs
  `temp/probes/oval_dwell_sweep_p3_{level,pose}.{md,json}`): the table above; level reproduces the P2
  entry's 58,742 / 71,428 / 88,313 / 111,510 / 143,763 exactly; pose 44,329 x4 / 56,479 / `LIMIT_VEL`
  357 at 0.40 / 431 at 0.35.

## Outcome

P3 is code complete: the pose-level carry planner is a production formulation of the folded STEADY
window behind `planner_mode`, default `level`, with the C-CUP-4 shaper rows, the prescribed tilt
series and the leg-jerk objective, measured on the production path at every oval box cell and
passing decision 24's offline gate there (residual 12.0 / 18.6 / 18.6 deg, window jerk 44.3k against
58.7k / 88.3k / 111.5k at 0.65 / 0.55 / 0.50). The free centroid-z band, the skid and the catch holds
were decided on the measurements (band pinned, skid logged, holds 0 / 0); the box carries both planners'
oval cells, stamped and refused by name on a mismatch; one parameter (`ros2 param set /skill_node
planner_mode pose`) flips a sitting. Owed before the next sitting: a FULL launch restart (new
`InstallSegment` interface; GUI hard refresh). Next: the A/B sitting of `tests/hardware/session_oval_rung1.md`
§ 8 — the as-is column and the swept column at dwell 0.50, 30 attempts each, leg current not higher, seat
bit read against the rung-2 69 % raw-dropout baseline; a GO flips the default with every family re-swept
under `pose`. P4 (the working-point raise; the shaper has already moved the 0.40 refusal onto leg
velocity) is cleared to start after that sitting's outcome is logged.
