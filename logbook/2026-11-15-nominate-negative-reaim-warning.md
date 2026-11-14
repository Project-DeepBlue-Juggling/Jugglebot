---
title: "Nominate flags a negative Catch re-aim margin in capitals and red (BUG-91, TKT-70 option 2)"
type: bugfix
date: 2026-11-15
status: complete
phase: night-shift
subsystem: skill-stack/gui
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/nominate.py
  - ros_ws/gui/js/juggle-panel.js
  - ros_ws/gui/css/juggle-panel.css
  - tests/motion/test_skills_nominate.py
  - tests/ros/test_gui_juggle_nomination.py
  - tests/ros/js/juggle_nomination_harness.js
  - logbook/INDEX.md
tags: [night-shift, BUG-91]
related_items: [BUG-91]
---

# Nominate flags a negative Catch re-aim margin in capitals and red

## What changed

`/skills/nominate` accepted `columns_turned` (90 deg, Apex 0.90, tilt-back 3 deg) with `success: True` while its own line printed a negative re-aim margin (-0.080); all five Attempts of the 2026-11-13 Sitting then ended at the first Catch after the Feed. The owner chose TKT-70 option 2, "flag in capitals": no refusal, no limit change.

- **`nominate.py`:** new `AIM_WARNING_TEXT = 'WARNING: RE-AIM MARGIN NEGATIVE'`; `summary()` appends it after the existing `AIM_NEGATIVE_TEXT` sentence, so the first reply line (the one the GUI shows) now ENDS with it.
- **GUI:** `runNominate` gives an accepted reply whose first line ends with the warning the state `warn`; `.jp-nom-sum.warn` is `--accent-red`, same as a refusal.

## Why display only

The re-aim figure is walked on the identity command (the BUG-87 class), so it is not a verdict: `rep.ok`, `success`, refusals and every gate are untouched, and a test pins that.

## Verification

- 2026-11-15, `tools/night_shift_nonet.sh python -m pytest tests/motion/test_skills_nominate.py tests/ros/test_gui_juggle_nomination.py tests/ros/test_skill_node_nominate.py -q -p no:cacheprovider` (the base's wrapper): **1 failed, 108 passed in 259.89 s**. The one failure, `test_columns_at_p4_plans_every_segment_and_matches_the_p3a_probe_cell` (worst rung `flight_frac` 0.1 against the pinned -0.2), is not this change's: the same test fails identically on `origin/main`'s code under `-k` (run 2026-11-15 on the BUG-93 tree, which leaves `nominate.py` untouched) and passes inside a full suite there (`./run_tests.sh --full` on night-shift/2026-11-15-BUG-93, 2026-11-15). It is order-dependent, not a regression.
- Fail without the change (2026-11-15): with only `nominate.py` restored to HEAD, `tools/night_shift_nonet.sh python -m pytest tests/motion/test_skills_nominate.py -q -p no:cacheprovider -k reaim` gave **2 failed, 6 passed**; with the change, **8 passed**. The GUI harness case was not run against a reverted panel.
- The unit that wrote this entry was cut off (API credit) before it ran its checks; the orchestrator ran the two above.
