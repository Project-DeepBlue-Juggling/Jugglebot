---
title: "A row that already holds its own fit no longer logs the false 'no evidence or row' refusal line (JB-176)"
type: bugfix
date: 2026-11-06
status: complete
phase: ""
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - tests/motion/test_skills_executor.py
  - logbook/2026-11-06-jb176-false-refusal-line.md
  - logbook/INDEX.md
subsystem:
  - skill-stack
tags:
  - tracker-identity
  - log-accuracy
---

# JB-176: the refusal line that said "no row" after the row was appended

**What.** On 2026-11-05 Throws 4, 5, 6 and 8 logged `TRACKER-IDENTITY-REFUSED throw N ... -- not this release's flight; no evidence or row from it` 2-29 ms AFTER `memory row appended`. Once a row's own fit sits in `best_landing`, the Tracker moves to the same ball's next flight; `_release_landing` correctly refuses that fit, but "no row" was false and a debrief read five lost rows.

**Fix.** In `_advance_outcomes` only: `best_landing` is set solely by `_consider_landing` from a `from_fit` landing (checked: the `from_fit` test precedes the assignment) and is frozen at the ball's next release, so "set" implies "own fit". For such a row the gate still runs and still refuses (the refused landing never reaches `_consider_landing`, so no row gains another flight's landing, the 2026-10-02 contamination), but its line goes to a scratch sink, the once-per-throw key is left as it was, and `rejected_apex_m`, `rejected_reason`, `n_rejected` are not touched. A row with no fit of its own keeps the line and the bookkeeping. Also fixed the docstring: the freeze is at the next release, not the crossing.

**Readers checked.** `n_rejected`, `rejected_reason` and `rejected_apex_m` are read only in `_finalise_outcome`, and only when `best_landing is None` (the OUTCOME `no row:` line and `_report`'s `no_row_reason`); `report.py` and `skill_node.py` do not read them. A row that held its own fit now shows 0 refused, no reason, no apex there; no motion path reads any of it.

**Not this.** Throw 3's missing row has a different cause (no fit of its own, the Tracker never confirmed its flight): JB-181.

**Verification.** 2026-11-06. The new `test_a_row_with_its_own_fit_logs_no_refusal_for_the_next_flights_fit` failed with the executor hunk temporarily reverted (copy saved, two lines sed-edited back, `pytest -q tests/motion/test_skills_executor.py -k "own_fit or no_fit_still"`: 1 failed, 1 passed) and passes with it restored. Scoped run: `pytest -q tests/motion/test_skills_executor.py tests/motion/test_skills_report.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py`, **327 passed in 9.29 s** (scoped run, not the full gate).
