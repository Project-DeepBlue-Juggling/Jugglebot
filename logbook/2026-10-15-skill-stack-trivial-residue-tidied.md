---
title: Skill-stack trivial residue tidied - transit entry retired, dead interfaces deleted, two renames, hand-clock load-flake fixed
type: refactor
date: 2026-10-15
status: resolved
phase: "two-ball-skill-stack — residue (post-R6)"
related_plan: skill-stack-dress-rehearsals-and-residue.md
related:
  - 2026-10-14-plans-board-cleared.md
  - 2026-10-14-skill-stack-possession-precondition-at-accept.md
  - 2026-10-14-mpc-residue-retired.md
  - 2026-10-14-two-ball-columns-sessions-why-attempts-end.md
files_changed:
  - sim/skills_gate.py
  - sim/ball/noise.py
  - sim/juggle_tilt.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  - ros_ws/src/jugglebot_interfaces/msg/ (7 msg files deleted)
  - ros_ws/src/jugglebot_interfaces/srv/SendBallButlerCommand.srv
  - ros_ws/docs/can-node-teensy-parity.md
  - tests/ros/conftest.py
  - tests/ros/test_skill_node.py
  - tests/motion/test_skills_executor.py
  - tests/sim/test_skills_gate.py
  - tests/sim/test_ball_noise.py
  - tests/sim/test_sim_import_style.py
  - tests/hardware/session_skills_r5_sitting6.md
  - tests/hardware/session_skills_r5_sitting7.md
  - tests/hardware/session_skills_r5_sitting8.md
  - tools/probes/README.md
  - tools/probes/archived/ (six probes moved)
  - tools/probes/throw_outcome_bag_probe.py
subsystem:
  - ros
  - sim
  - tools
  - docs
tags:
  - testing
---

# Skill-stack trivial residue tidied

## Summary

Owner, 2026-10-14/15: "carry out item 5, tidy the small leftovers" = the (c) rows of the archived residue plan's § 2. **The interfaces package changed: run `colcon build --packages-select jugglebot_interfaces jugglebot` and restart the launch** (the orchestrator runs the build after the commit; the restart is the owner's).

- **Transit entry retired.** The `transit` entry mode / `--entry` knob in `sim/skills_gate.py` is gone (`hop_entry` follows `feed_mode`; `compile_columns(hop_entry=bool)`); transit-only tests deleted, five `test_skill_node.py` tests and the `_columns_fed` helper re-pinned to hop; four transit-only tests deleted (the uncatchable-feed test now drives the compile-refusal tail deterministically with a patched `compile_columns`, plus a -250 mm geometry test; the earlier -80 mm "refuses LIMIT_VEL" pin was measured under the silently-wrong parked entry and plans clean under the hop entry); runsheets sitting 6/7/8 annotated. The `schedule.py`, `test_skills_executor.py`, `test_skills_schedule.py` and `test_skills_gate.py` hunks of the retirement were committed in 7f590d41 (the peer session's `2026-10-14-two-ball-columns-sessions-why-attempts-end`) by agreement between the two sessions; the rest lands with this unit.
- **Defect found by the phase audit and fixed before commit:** the retirement had dropped `entry=` from `_install_columns_schedule`'s `compile_columns` call, and the committed default `hop_entry=False` is the parked entry, so THROW 0 of the live fed columns schedule lost `entry_hop` — the node tests re-pinned to hop could not see it (they assert the bridge site and aim, which come from the choreography, not the schedule flag). Fixed with `hop_entry=True` and pinned by `test_the_installed_fed_columns_schedule_marks_throw_0_as_the_hop_entry`.
- **Eight dead `jugglebot_interfaces` types deleted** (`ThrowDebug`, `BallStateMulti`, `BallStateSingle`, `TargetArray`, `SetTrapTrajLimitsMessage`, `LegsTargetReachedMessage`, `CanTrafficReportMessage`, `SendBallButlerCommand`) with their CMakeLists lines and `tests/ros/conftest.py` mocks. Census after: 0 live references (`attic/` and dated records only); `ros_ws/docs/can-node-teensy-parity.md` annotated.
- **Six probes archived** to `tools/probes/archived/` (`teensy_can3_telem`, `skill_node_response_starvation_probe`, `juggle_cup_bandlimit`, `juggle_tilt_bandlimit`, `hand_poll_cadence_census`, `hand_cascade_ff_model`), README updated.
- **Two renames.** `sim/juggle_noise.py` -> `sim/ball/noise.py` (test -> `test_ball_noise.py`, import-style allow-list entry dropped, five live importers updated). `HAND_LANE_REFUSED` -> `LANE_REFUSED` (value too; it covers hand and legs; history comment in `executor.py`, INVARIANTS row says "was HAND_LANE_REFUSED until 2026-10-15").
- **Hand-clock load-flake fixed.** `skill_node` read `_hand_telemetry_mono` against a wall clock the tests could not freeze, so a slow run aged the fixture sample past the 500 ms accept check (probe BEFORE: `REJECTED_BALL_UNKNOWN ... /hand_telemetry stale (age 601 ms >= 500 ms)`; AFTER: accepted). Fix: `self._hand_clock = time.perf_counter` beside every read; tests freeze it (`_HAND_CLOCK_FROZEN_S`) plus a regression test.
- **Deliberately not done:** `sim/juggle_tilt.py` keeps its name (two box-gated files cite it; a gated-file edit means a box re-sweep; its docstring says so).

## Why

The owner's ask above. The transit condition: transit was kept only until hop met the R5 gate, which it did at sitting 8 (2026-10-13, 32 consecutive Ball-Butler-fed columns catches), so the fallback had no remaining purpose.

## Verification

All 2026-10-15.
- Transit: `pytest -q -p no:cacheprovider tests/ros/test_skill_node*.py` 311 passed; `tests/sim/test_skills_gate.py` 30 passed in 189.46 s; `tests/motion/test_skills_schedule.py` 152 passed.
- Renames: `python -c "import sim.ball.noise, sim.skills_gate"` OK; `pytest -q -p no:cacheprovider` on the seven touched files 653 passed in 32.59 s; `tests/sim/test_skills_gate.py` 30 passed in 115.73 s.
- Hand clock: `pytest -q -p no:cacheprovider tests/ros/test_skill_node.py test_skill_node_reload_only.py test_skill_node_magazine.py test_skill_node_resend_param.py` 312 passed in 16.07 s; resend_param looped x5, 5 passed each time.
- Interfaces/conftest: `pytest -q -p no:cacheprovider tests/ros/ -x --ignore=tests/ros/test_skill_node.py --ignore=tests/ros/test_skill_node_reload_only.py` 2765 passed, 2 skipped in 456.97 s.
- Docs: `pytest -q -p no:cacheprovider tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_plans_index.py` 108 passed, 2 skipped in 0.86 s.
- Full gate: ./run_tests.sh --full, run 2026-10-15 01:06-01:11 on this tree (temp/logs/gate_tidy_final_20261015.log): 6630 passed, 11 skipped, 1 failed in 292.78 s (parallel), serial 7 passed in 27.33 s; the one failure, tests/ros/test_replay_overview.py::test_a_pass_still_exiting_for_an_older_version_neither_reports_computing_nor_blocks_the_requeue (GUI replay arc, untouched tonight, timing-dependent), passed 3/3 scoped and 11/11 with its file at load 1.3 and was green in the 00:56 full run of the same tree minus the hop_entry fix - a load flake, not a regression

## Open

The four archived probes that import the old `sim.juggle_noise` path (`juggle_catch_offset`, `juggle_fastcatch`, `juggle_throw_accuracy`, `juggle_allkinematic_loop`) are records and stay broken. Nothing else.
