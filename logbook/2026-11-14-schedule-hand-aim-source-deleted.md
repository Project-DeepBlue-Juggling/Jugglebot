---
title: "The schedule_hand Catch aim source is deleted (BUG-106)"
type: refactor
date: 2026-11-14
status: complete
phase: "maintainer review"
subsystem: skill-stack
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/hand_launch.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - sim/skills_gate.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_executor_online.py
  - tests/motion/test_skills_hand_launch.py
  - tests/motion/test_skills_oval.py
  - tests/ros/test_skill_node.py
  - tests/hardware/session_skills_r3_apex_ladder.md
  - logbook/INDEX.md
---

## What

`catch_aim_source` had three values: `tracker` (the default, what flies), `schedule`
(open loop, still used for A/B sheets) and `schedule_hand` (`schedule` plus one re-aim of
the committed Catch from the measured launch speed of the Hand). The third is deleted
with everything reachable only from it: `_hand_ratio_landing`,
`_hand_corrected_landing`, `_resend_hand_corrected_catch`, `_resend_too_late`, the
`_hand_corrected` set, the `launch_ratio` constructor argument, `hand_launch.py` and the
feed for it in `skill_node`. About 490 lines of code and 550 of tests. A launch that
still passes `schedule_hand` logs the existing unknown-value warning and flies
`tracker`.

## Why

Owner decision, 2026-11-14, on BUG-106. The re-send path fenced with `skill.lead_s`,
which I-CATCH-4 forbids; it was not ported when `_resend_live_catch` was. A search of
`temp/logs` and `~/.ros/log` for 2026-08-14 to 2026-11-14 found no line showing it
flown, and no sheet or entry sets it. Porting a path nobody flies would have kept about
200 lines the contract no longer describes.

Default behaviour is unchanged. In `_catch_aim` the `tracker` branch returns before the
changed lines, and the `schedule` arm returns the same `_prior_landing` as before. The
early-command prior under `tracker` and `schedule` was already `_prior_landing`; only
the deleted arm used `_predicted_landing`. `_hand_corrected` was written only on the
deleted paths, so the two places that read it always saw it empty.

One row of `INVARIANTS.md` changed: C-CATCH-2 no longer lists the hand-corrected re-send
among the callers of the clamp.

## Verification

- 2026-11-14, `./run_tests.sh --full` on this change together with the Ball Butler
  firmware row: 9787 passed, 77 skipped, 1 xfailed (parallel, 635 s), 10 passed
  (serial); RESULT PASS.
- 2026-11-14, one audit pass over the commit against the pre-image: no blocking
  finding; every removed branch reachable only under the deleted value.
- `grep -rn "schedule_hand\|AIM_SCHEDULE_HAND"` outside `logbook/`, `plans/archived/`
  and `map/`: 37 before, 1 after (`docs/night-shift/2026-11-02.md`, a dated report).
