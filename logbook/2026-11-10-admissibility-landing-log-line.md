---
title: "ADMISSIBILITY-LANDING: the online check logs the Catch landing it planned against"
type: feature
date: 2026-11-10
status: complete
phase: "BUG-87 evidence"
subsystem: skills
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - tests/motion/test_skills_executor_online.py
---

# ADMISSIBILITY-LANDING log line

## What changed

BUG-87 (the live online check refused skill 2's Catch of Ball 1 at m=-0.208 while the nomination read -0.049) cannot be diagnosed from a Sitting, because the launch log never said which landing the check planned that Catch to. `_resolve_catch_command` now queues one note per skill (`_note_once`, key `ADMISSIBILITY-LANDING`), built where the check already runs, before the check resolves:

`ADMISSIBILITY-LANDING skill <idx>: catch landing (x, y, z) mm at t=<abs> s (<+dt> s ahead), source=<aim>`

The landing is the clamped one the check uses; `dt` is its time minus the dispatch instant; `source` is `_catch_aim`'s label (`tracker`, `schedule`, `schedule_hand`, `schedule:learned`), carried in a new write-only attribute `_dispatch_aim_label` set at the dispatch. One consequence: `dispatch`'s `_NoAdmissibleCommand` branch now returns the notes queued before the refusal ahead of its END line (previously they stayed queued), so the line is visible on exactly the refusal BUG-87 is about. The existing `ADMISSIBILITY` line is unchanged. Not emitted where the check is skipped for want of a release (no landing is known there). Behaviour is unchanged: no value, decision or timing moved; nothing is built on the 40 Hz path beyond the dispatch.

## Verification

- 2026-11-10, `tools/night_shift_nonet.sh pytest -q tests/motion/test_skills_executor_online.py tests/motion/test_skills_executor.py tests/sim/test_skills_gate_online.py`: 349 passed in 57.32 s.
- Both new tests (pass and refusal) fail with the executor change reverted (2 failed, 43 passed).
