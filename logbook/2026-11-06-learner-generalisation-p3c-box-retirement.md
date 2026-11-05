---
title: "Learner generalisation P3c: the offline Admissible box is retired - the box API, the sweep tool, the committed YAML and the executor's box path are deleted; the planner cells the sweep owned live on as a probe library"
type: refactor
date: 2026-11-06
status: in-progress
phase: "learner-generalisation P3c (box retirement) - code complete on branch p3-online-admissibility, held for the owner's merge"
related_plan: learner-generalisation.md
files_changed:
  - config/generated/admissible_box.yaml
  - docs/motion_planner/index.md
  - docs/patterns.md
  - docs/project-map.md
  - docs/runbook.md
  - logbook/2026-11-06-learner-generalisation-p3c-box-retirement.md
  - logbook/INDEX.md
  - map/feature-p4-ramp-sitting-columns-canary-at-600-8000-and-20-a-then-the.md
  - map/feature-pre-release-hold-rung-release-from-a-moving-platform-hold-0.md
  - map/idea-jp-10-turned-columns-the-columns-pair-at-fixed-angles-about.md
  - map/idea-jp-9-high-low-columns-ball-1-at-apex-1-05-m-ball-2-at-0-95-m.md
  - map/pathway-learner-generalisation.md
  - map/ticket-what-are-today-s-pathways-their-destinations-and-which-plans.md
  - plans/active/INDEX.md
  - plans/active/carry-ring-zvd.md
  - plans/active/learner-generalisation.md
  - plans/active/pattern-progression.md
  - plans/active/two-ball-oval.md
  - plans/archived/INDEX.md
  - plans/archived/box-sweep-same-site-fidelity.md
  - ros_ws/docs/catch_reach_envelope.md
  - ros_ws/gui/js/juggle-panel.js
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/learner.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/magazine_throw.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/nominate.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/segments.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/sites.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/shaping.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/tracking/matcher.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - sim/skills_gate.py
  - tests/fixtures/admissible_box_pre_h4500.yaml
  - tests/fixtures/admissible_box_pre_oval.yaml
  - tests/fixtures/admissible_box_pre_p3.yaml
  - tests/fixtures/admissible_box_pre_p4.yaml
  - tests/fixtures/admissible_box_pre_release_step.yaml
  - tests/hardware/session_oval_rung1.md
  - tests/hardware/skills_plan_bench.py
  - tests/motion/test_admissible_sweep_run.py
  - tests/motion/test_check_segment_budget.py
  - tests/motion/test_cup_contact_contract.py
  - tests/motion/test_pose_planner.py
  - tests/motion/test_skills_admissible.py
  - tests/motion/test_skills_admissible_online.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_executor_online.py
  - tests/motion/test_skills_install_origin.py
  - tests/motion/test_skills_oval.py
  - tests/motion/test_skills_schedule.py
  - tests/motion/test_turned_columns_box.py
  - tests/motion/test_validate_cycle.py
  - tests/ros/test_gui_juggle_defaults.py
  - tests/ros/test_install_segment.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skill_node_nominate.py
  - tests/ros/test_skill_node_online.py
  - tests/ros/test_skill_node_reload_only.py
  - tests/ros/test_skills_plan_bench.py
  - tests/ros/test_trajectory_node.py
  - tests/sim/test_skills_gate.py
  - tests/sim/test_skills_gate_online.py
  - tools/admissible_sweep.py
  - tools/admissible_sweep_run.py
  - tools/probes/README.md
  - tools/probes/admissible_obviation/README.md
  - tools/probes/admissible_obviation/affine_check.py
  - tools/probes/admissible_obviation/boundary_replay.py
  - tools/probes/admissible_obviation/family_run.py
  - tools/probes/admissible_obviation/loaded_budget.py
  - tools/probes/admissible_obviation/margin_rays.py
  - tools/probes/admissible_obviation/online_check_replay.py
  - tools/probes/admissible_obviation/semantic_gate_hash.py
  - tools/probes/oval_dwell_sweep.py
  - tools/probes/p4_headroom_cells.py
  - tools/probes/planned_release_motion.py
  - tools/probes/planner_cells.py
  - tools/probes/release_step_shape.py
subsystem:
  - motion
  - ros
  - sim
  - tools
tags:
  - refactor
  - testing
  - docs
---

# Learner generalisation P3c: box retirement

## What changed

Nothing at runtime read the offline Admissible box after P3a (the per-Throw online check) and P3b (goal
acceptance by nomination), so it is deleted per the plan's ledger
(`plans/active/learner-generalisation.md` § 6.4):

| Retired | Where it was |
|---|---|
| `AdmissibleBox`, `clip`, `select`, `describe_miss`, `check_limits`, `check_site_workspace`, `dump`, `load`, `gate_hash` and its eight gated files, `LimitsMismatch`, `PATTERNS`, the dwell / planner / ZVD stamps | `motion/skills/admissible.py` (1034 -> about 215 lines; it keeps `MARGIN_FRAC`, `margin`, `CommandVerdict`, `Resolution`, `resolve_command`, `AdmissibleError`) |
| the `boxes` argument, `_forward_clip_xy`, `BOX_FORWARD_GAIN_FLOOR`, `_box_dwell_s`, the box branches of `_command_u` and `_hop_command`, the `BOX-*` notes | `motion/skills/executor.py` |
| `_ADMISSIBLE_BOX_PATH` and every `boxes` hand-off | `skill_node.py` |
| `--box`, `_load_admissible_boxes`, the `box` config field | `sim/skills_gate.py` |
| `tools/admissible_sweep.py`, `tools/admissible_sweep_run.py`; the probes that studied the sweep (`family_run.py`, `boundary_replay.py`, `semantic_gate_hash.py`) | tools/ |
| `config/generated/admissible_box.yaml`; `tests/fixtures/admissible_box_pre_*.yaml` (five) | config/, tests/fixtures/ |

Three things follow from the deletion and are deliberate:

- **The planner cells survive as a library.** The sweep's cell builders (`throw_cell`,
  `columns_catch_throw_cell`, `chained_catch_cell`, `hop_*`, `folded_catch_cell`, `oval_window`,
  `within_margin`) are what eight probes and several tests plan with. They moved, bodies unchanged, to
  `tools/probes/planner_cells.py`; `rest_state` and `sitting_limits` are imported from
  `motion/skills/nominate.py`, not copied. `margin_rays.py` is the kept design-time headroom report (owner
  decision D10).
- **With no checker the executor commands the Learner's authority-clipped candidate**, unclipped and
  unrefused: what `boxes=None` did before, which the sim runs without a Learner check and many tests rely
  on. With a checker (the node always wires one) nothing changed in this phase.
- **Editing the eight planner files no longer costs a two-hour re-sweep.** `gate_hash` is gone with the box
  it stamped.

The companion plan `plans/archived/box-sweep-same-site-fidelity.md` is archived as superseded (none of its
work landed; its concern, a cup fly-back the sweep planned and the executor never dispatches, does not
exist in the stand-in or the nomination, which both release a same-site fold at the caught xy).

## Census (`grep -rn`, logbook/ and plans/archived/ excluded)

Hits per symbol in production code / tests / tools, sim and config / living documents:

| Symbol | Start (e9df3272, 07:25) | End (08:50) |
|---|---|---|
| `AdmissibleBox` | 22 / 19 / 19 / 10 | 1 / 1 / 1 / 10 |
| `check_limits` | 10 / 26 / 1 / 7 | 4 / 3 / 1 / 7 |
| `gate_hash` | 23 / 65 / 34 / 26 | 1 / 15 / 5 / 21 |
| `admissible_box` (the YAML and its fixtures) | 10 / 61 / 15 / 7 | 4 / 20 / 1 / 7 |
| `describe_miss` | 6 / 15 / 0 / 2 | 1 / 2 / 0 / 2 |
| `check_site_workspace` | 4 / 12 / 4 / 0 | 1 / 1 / 0 / 0 |
| `_forward_clip_xy`, `BOX-*` notes | 25 / 17 / 5 / 6 | 3 / 7 / 1 / 6 |
| `admissible_sweep` (the tool) | 23 / 24 / 33 / 23 | 10 / 13 / 7 / 26 |
| `LimitsMismatch` | 5 / 13 / 1 / 3 | 0 / 10 / 0 / 3 |
| `_ADMISSIBLE_BOX_PATH` | 1 / 73 / 0 / 0 | 0 / 0 / 0 / 0 |
| `boxes=` | 14 / 29 / 9 / 6 | 0 / 0 / 0 / 6 |
| `_GATED_FILES` | 6 / 2 / 1 / 0 | 0 / 1 / 0 / 0 |

No living use of a retired symbol remains. What the end counts are:

- **production**: `skill_node.py`'s local variable named `check_limits` (unrelated), and dated
  history comments that name the retired tool or API in `admissible.py`, `nominate.py`, `segments.py`,
  `sim/skills_gate.py` and three comment lines of `config/hardware_config.yaml` (the history of the working
  point, left as written);
- **tests**: the list of retired names in the test that asserts they are gone
  (`tests/motion/test_skills_admissible_online.py`), history comments in `tests/motion/` and `tests/sim/`, an
  unrelated `--box` option of the tilt-calibration grid, and the closed `tests/hardware/session_*.md`
  runsheets (flown `gate_hash` rows are history; `LimitsMismatch` appears only there);
- **tools**: `planner_cells.py`'s docstring, the READMEs' recorded results, and an unrelated `--box` in
  `displaced_reach_frontier.py`;
- **living documents**: other Sessions' active plans (bannered, not rewritten), map Items' history,
  `docs/night-shift/` reports, this plan's own text about the retirement, and the glossary term.

Script and raw counts are session artefacts; the commands are `grep -rnE "<symbol>" ros_ws/src | tests |
tools sim config | docs map plans/active plans/parked CLAUDE.md ...` over `*.py, *.md, *.yaml, *.js, *.sh,
*.html`, excluding `logbook/` and `plans/archived/`.

## Tests

| File | Before -> after | What happened |
|---|---|---|
| `tests/motion/test_skills_admissible.py` | 111 -> deleted | box and sweep mechanics; two tests of the surviving cell timing moved to `test_skills_admissible_online.py` |
| `tests/motion/test_turned_columns_box.py`, `test_admissible_sweep_run.py` | deleted | tested the sweep tool |
| `tests/motion/test_skills_executor.py` | 285 -> 268 | 17 box-path tests deleted: four have named online twins, the nine forward-clip tests are covered by the authority, predicted-landing and checked-once twins or were box arithmetic, four were box mechanics |
| `tests/motion/test_skills_executor_online.py` | 41 -> 44 | the no-checker test now pins "the candidate is commanded as is"; a no-checker lateral-authority test; a no-checker Learner-error refusal (audit) |
| `tests/motion/test_skills_admissible_online.py` | 19 -> 41 | T-R3: every retired name absent from `admissible`, `__all__` exactly the online API, no `boxes` parameter on `SkillExecutor`, the two tools and the YAML gone; the two moved cell-timing tests |
| `tests/motion/test_skills_oval.py` | 26 -> 25 | one box-key test deleted |
| `tests/ros/test_skill_node.py` | 392 -> 390 | 70 box-path patches and four box fixtures removed as setup; two box-only tests deleted; `test_check_needs_no_box_file` now asserts the one informational oval refusal |
| `tests/ros/test_gui_juggle_defaults.py` | 9 -> 9 | re-pinned: every GUI pre-fill must NOMINATE viable at the working point (three box-pinning tests replaced by two) |
| `tests/ros/test_skills_plan_bench.py`, `tests/hardware/skills_plan_bench.py` | one test deleted | the bench no longer compares its jerk constant with the swept box; nothing ties `SELF_TOSS_LEG_JERK_MMPS3` to the live limits now (offline bench only) |
| `tests/sim/test_skills_gate.py`, `test_skills_gate_online.py` | 48 -> 43 | the three forward-clip tests (their online twins stay) and the two `--box` tests deleted |

No surviving test was weakened. Counts are the units' own scoped runs (2026-11-06, venv python, one BLAS
thread, `pytest -q -p no:cacheprovider`) and the gate below.

## Verification

- **Baseline**: the previous phase's full tier (2026-11-06 07:23, `./run_tests.sh --full` at what became
  e9df3272): 8459 passed, 51 skipped + 10 serial.
- **Sim gate, online path the only path** (2026-11-06 08:51-08:55, `OPENBLAS_NUM_THREADS=1 python
  sim/skills_gate.py --no-viewer [...]`, four runs side by side): plain exit 0, PASS (70.1 s, 5 seeds);
  `--learn --pattern self_toss` exit 0, PASS (203.6 s); `--learn --pattern hop` exit 0, PASS (209.8 s);
  `--learn --pattern columns` exit 0, PASS (222.1 s). Logs `temp/logs/skills_gate_p3c_*.log`.
- **Full tier, first run** (2026-11-06 08:55-09:03, `./run_tests.sh --full`): 8324 passed, 1 failed, 51
  skipped + 10 serial. The failure was `tests/ros/test_skill_node.py::test_check_needs_no_box_file`, left by
  the cleanup asserting that `skills/check` never prints REFUSED, while the oval at the default 250 mm
  separation parameter is an informational refusal; the assertion now says exactly that.
- **Full tier on the final tree** (2026-11-06 09:09-09:17, `./run_tests.sh --full`): **8326 passed, 51
  skipped** (409.56 s) and **10 passed** serial (37.19 s); RESULT PASS. 133 fewer than the baseline's 8459:
  the deleted box and sweep tests net of this phase's new ones. No hypothesis test was added or changed.
- **Audit** (one read-only pass over the whole diff, 2026-11-06 08:55): no blocking finding; the deletion
  changed no behaviour on either executor path (HEAD's `_command_u` with no box and no checker compared with
  the new one), no import of a deleted name remains, and no generator or manifest names a deleted file.
  Applied: two test names in INVARIANTS, a retirement note in `ros_ws/docs/catch_reach_envelope.md`, a
  re-pointed path in a map ticket, and a new test for the checker-less Learner-error refusal, whose box-era
  pin had gone with the box. Recorded, not fixed: nothing ties the offline bench's
  `SELF_TOSS_LEG_JERK_MMPS3` to the live limits any more.
- **Runbook**: `docs/runbook.md` regenerated (`python -m mapkit gen`); it carries no box or sweep instruction.

## Left for the owner

- **"Admissible box" is still a GLOSSARY term** (and so in CLAUDE.md's generated term block). The glossary
  changes only through the grilling and domain-modeling skills.
- **Items JB-120 and JB-150** (the box-gate bug; sweeps on the Win10 desktop) are held by a peer Session's
  claim, so the retirement note was not written on them and nothing was forced. Both look closable once
  this merges. No Item was closed by this phase: closing rides with the merge to main.
- **Other Sessions' active plans** (`carry-ring-zvd.md`, `two-ball-oval.md`, `pattern-progression.md`) carry a
  dated banner; their sweep sections are left as written. Four in-progress Items (JB-162, JB-102, JB-107,
  JB-108) lost the box-hash step of their `## Sitting` sections; expect merge conflicts in `map/` if those
  Items moved on main meanwhile.
- **Closed runsheets under `tests/hardware/`** keep their flown box and `gate_hash` rows as history; only
  `session_oval_rung1.md`, the one still to be flown, was changed.
