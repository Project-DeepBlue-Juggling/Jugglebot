---
title: "The Catch re-aim log lines carry the ask's signed Robot-frame direction (dx, dy), so the BUG-75 offline replay no longer tries four directions"
type: feature
date: 2026-11-26
status: complete
phase: "BUG-75 re-aim observability"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - tools/probes/late_reaim/parse_resends.py
  - tools/probes/late_reaim/label_rows.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_late_reaim_replay.py
subsystem: [skills, tooling]
---

## What and why

`SkillExecutor._resend_live_catch` logged a Catch re-aim's size (mm, max over axes) and time shift but not its direction, so the BUG-75 replay (`tools/probes/late_reaim/`) had to try four directions. Every RESEND line from that function (`RESEND` accepted, `RESEND-REFUSED <code>`, `RESEND-SKIPPED <reason>`) now ends in ` dx=+12.3 dy=-4.5`: new landing minus the installed landing, Robot frame, mm, signed, one decimal. Not covered: the `BUMP` / `BUMP-DECLINED` lines of `catch_bump` (they carry `delta_xy_mm` in the decision, a separate follow-up).

Observability only. The difference was already computed for `moved_mm` (`np.max(np.abs(delta))` is unchanged); the new fields are two floats from it, formatted into the existing string. Control walk-through: no planning, admission, fence or timing input changes; the string is built on the same tick, once per logged line (declines already once per catch and reason).

Readers checked (`grep -rn "the fit moved the landing\|RESEND"` over tools/, tests/, ros_ws/, .claude/, sim/): label_rows, parse_resends, selftoss_landing_decomposition, handoff_replay, feed_catch_bag_probe, emit_lead_admissibility, limit_refusal_tally, capture_move_replay_extract and the executor tests. All use unanchored `search` or substring tests, so the appended tail does not break them. parse_resends and label_rows now add `dx`/`dy` to each row (None on older logs). `bump_replay.py` still loops its four directions: using the logged direction there is more than a few lines and is left for the replay session.

## Verification

- 2026-11-26, `pytest tests/motion/test_skills_executor.py tests/motion/test_skills_executor_human_catch.py tests/motion/test_skills_capture_replay.py tests/motion/test_late_reaim_replay.py tests/sim/test_sim_import_style.py -q -p no:xdist` (BLAS pinned): 365 passed in 27.45 s. Also `tests/motion/test_feed_catch_bag_probe_regex.py tests/motion/test_cup_capture.py`: 21 passed.
- Both new tests (`test_re_aim_lines_carry_the_signed_robot_frame_direction_of_the_ask`, `test_label_rows_and_parse_resends_read_old_and_new_lines`) fail with the executor and tools changes stashed (2 failed), pass with them.
