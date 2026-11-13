---
title: "Night Shift 2026-11-14: 0 merged, 2 held (FEAT-50, carried IDEA-22), 5 questions (TKT-68 to TKT-72)"
type: investigation
date: 2026-11-14
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-14.md
  - logbook/2026-11-14-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
---

# Night Shift 2026-11-14

**Counts.** 0 merged, 2 held (FEAT-50 built tonight, IDEA-22 carried), 5 new question tickets (TKT-68 to TKT-72), 112 skipped. The six Items the 2026-11-13 debrief filed were rated; four owner decisions got options and a ticket. Report: `docs/night-shift/2026-11-14.md`.

**Nightly.** CLAIMED GREEN 9538/9645 passed, 0 failed, 0 errored, 1 xfailed, 106 skipped 2026-11-14T04:02:27+11:00.

**Cost.** 1 Sonnet builder, 1 Sonnet audit; 2 full gate runs. The runner adds the figure to the report.

**What a future night should copy.**

- A change that adds a ROS subscription or publisher must regenerate `ros_ws/docs/choreography.md` (`python tools/gen_choreography_map.py`); `tests/ros/test_choreography_map.py` is not in a unit's scoped run, so FEAT-50's first full gate failed on it. Name it in any brief that touches a node's topics.
- Map writes on `main` while a unit works on a branch in the same clone can go through a `git worktree` of `main`, but remove it before the full gate: it sits under `temp/` inside the clone.
- A branch cut before tonight's own `map:` pushes must merge `origin/main` before the audit; the gate refuses a branch that does not contain the base. Only `docs/project-map.md` conflicted, resolved by `python -m mapkit gen`.

## Verification

- 2026-11-14, `tools/night_shift_nonet.sh python -m pytest -q tests/sim/test_night_shift_report.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py`: 53 passed in 3.92 s.
