---
title: "JB-3: the `session limits` identifiers and prose renamed to Sitting limits outside history; admissible box re-swept value-identical"
type: refactor
date: 2026-10-28
status: resolved
phase: "inbox-wall - JB-3"
files_changed:
  - config/generated/admissible_box.yaml
  - config/hardware_config.yaml
  - docs/agents/domain.md
  - map/feature-rename-session-limits-to-sitting-limits.md
  - plans/active/leg-gain-tuning-methodology.md
  - ros_ws/docs/can-node-teensy-parity.md
  - ros_ws/docs/catch_reach_envelope.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/bump.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/magazine_throw.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/chase.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/limits.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - sim/analysis/diagnose.py
  - sim/skills_gate.py
  - tests/hardware/mvp_bench_runbook.md
  - tests/hardware/session_anomaly_fixes.md
  - tests/hardware/session_gain_retune.md
  - tests/hardware/session_phase4_ramp.md
  - tests/hardware/session_skill_stack_r1_flash.md
  - tests/hardware/session_skills_r3.md
  - tests/hardware/session_skills_r4.md
  - tests/hardware/session_skills_r5.md
  - tests/hardware/session_skills_r5_sitting2.md
  - tests/hardware/session_skills_r5_sitting3.md
  - tests/hardware/session_skills_r5_sitting4.md
  - tests/hardware/session_torque_ff.md
  - tests/hardware/session_unified7_bus_headroom.md
  - tests/hardware/session_unified7_cycle_ladder.md
  - tests/hardware/session_unified7_hand_bringup.md
  - tests/hardware/skills_plan_bench.py
  - tests/hardware/traj_ramp_battery.py
  - tests/motion/test_cup_capture.py
  - tests/motion/test_cup_contact_contract.py
  - tests/motion/test_cup_realize.py
  - tests/motion/test_follower_chase.py
  - tests/motion/test_magazine_throw.py
  - tests/motion/test_pose_planner.py
  - tests/motion/test_retime.py
  - tests/motion/test_shaped_batch.py
  - tests/motion/test_skills_admissible.py
  - tests/motion/test_skills_bump.py
  - tests/motion/test_skills_capture_replay.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_install_origin.py
  - tests/motion/test_skills_rebase_continuity.py
  - tests/motion/test_skills_segments.py
  - tests/motion/test_solver_worker.py
  - tests/motion/test_solver_worker_lateness.py
  - tests/motion/test_splice_tilt_rate_seam.py
  - tests/motion/test_trajectory_follower.py
  - tests/motion/test_trajectory_planner_catch.py
  - tests/motion/test_unified_cycle.py
  - tests/motion/test_unified_cycle_budget.py
  - tests/motion/test_unified_cycle_splice.py
  - tests/motion/test_validate_cycle.py
  - tests/motion/test_validate_cycle_budget.py
  - tests/motion/test_validate_cycle_vectorised.py
  - tests/ros/test_install_segment.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skills_plan_bench.py
  - tests/ros/test_trajectory_node.py
  - tests/ros/test_unified_cycle_levelling.py
  - tests/sim/test_diagnose_trajectory.py
  - tests/sim/test_skills_gate.py
  - tools/admissible_sweep.py
  - tools/probes/README.md
  - tools/probes/displaced_reach_frontier.py
  - tools/probes/late_reaim/bump_replay.py
  - tools/probes/late_reaim/capture_records.py
  - tools/probes/planned_release_motion.py
  - tools/probes/skills_segment_sweep.py
  - tools/probes/skills_sizing_sweep.py
subsystem:
  - motion
  - docs
tags:
  - glossary
  - rename
  - admissible-box
---

## Summary

JB-3: the live velocity / acceleration / jerk limits are now called by their glossary term, Sitting limits, everywhere outside history. `TrajectoryLimits.with_session_limits` became `with_sitting_limits` (60 call sites), the `session_limits` test fixture and four test names follow, and prose says "sitting limit(s)". The probe `displaced_reach_frontier.py` now writes a `sitting_limits` JSON key. The rename was done by one case-preserving regex, including 7 hits split across a line break. Three hits were rewritten by hand. `docs/agents/domain.md` used the rename as its example of a glossary term ahead of the code, and that example is now past tense. In `ros_ws/docs/can-node-teensy-parity.md` the two "(session limit)" notes meant a Claude usage limit, so they now read "Claude Session usage limit". The line "sitting limits at session start" in `unified_cycle.py` and its test twin became "Sitting start". The YAML comments changed and the generated `hardware_config` artefacts did not, because the generator carries no comments.

Grep count (`grep -rn -i -E 'session[ _-]limits?'` over every file, excluding `.git logbook attic temp build install log archived`). Before: **250 lines in 85 files**. After: **9 lines, all deliberate**. They are the `_Avoid_` lines in `GLOSSARY.md` and `docs/glossary.md`, the avoid-word seed in `tests/sim/avoid_words_checked.txt` and `test_glossary_avoid_words.py`, the JB-3 title and its dated 2026-09-21 survey line, the generated JB-3 row in `docs/project-map.md`, and two quotations in map tickets (the seed list and #17's title). A multi-line scan for the wrapped form also finds zero.

The edits touched three gated files (`feasibility.py`, `cup_realize.py`, `unified_cycle.py`, comments and docstrings only), so the admissible box was re-swept on the frozen tree. The sweep used the 2026-10-24 `sweep_P3b.sh` recipe: ten invocations at 350/5000/200000, hand 3900, BLAS one thread, run as one round. Agreement with the committed box is the determinism check.

## Verification

- Re-sweep (2026-10-28 11:30-12:20 AEDT, ten `tools/admissible_sweep.py` invocations, log `temp/logs/jb3_resweep.log`): **26 cells, boxes + limits value-identical to the committed box**. Only `swept_at` and `gate_hash` changed. The new `gate_hash` is `f8a9e188b2cf`, equal to `admissible.gate_hash()` on the final tree. Box md5 `0d4da3616399e6d68e91de17504d5065`.
- Full gate (2026-10-28, `./run_tests.sh --full` on the final tree, log `temp/logs/jb3_gate.log`): **parallel 7361 passed / 13 skipped in 364.54 s + serial 8 passed in 29.90 s, RESULT PASS**. The only edit after it was this triple; `pytest -q tests/sim/test_glossary_avoid_words.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py` re-run on it.
- NOT FLOWN: no control behaviour changed. Only comments, identifiers, the box stamp, the admissible LimitsMismatch message text and one probe JSON key differ.
