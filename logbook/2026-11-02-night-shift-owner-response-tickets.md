---
title: "Night Shift: every Item waiting on the owner gets a question ticket and a Respond in cell; an answered hold resumes its branch"
type: feature
date: 2026-11-02
status: complete
phase: "night-shift (owner response)"
related_plan:
  - night-shift.md
files_changed:
  - .claude/commands/night-shift.md
  - docs/night-shift/README.md
  - mapkit/night_shift_report.py
  - tests/sim/test_night_shift_report.py
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
  - report
  - question-tickets
---

# Night Shift: a response cell for every Item that needs the owner

**Why.** The 2026-11-02 report's Held rows (JB-135, JB-32) each ended "Your call on ..." with no way to answer. Owner ask: every Item that needs their input gets a mechanism to respond, not just the Questions table.

**What.** Reuse the question-ticket machinery (owner's choice) instead of a new section kind.
- Protocol (`.claude/commands/night-shift.md`): 2h now files a ticket for every Held row whose unblock is the owner's call and every `owner decision` Skipped row without an open ticket; Sitting-only holds, `out of reach` and `not reached` get none. Hold tickets are filed on `main`, never on the branch (the app writes the answer to `main`; a branch copy would stay open and trip the gate's condition (e) at the re-gate). The Held row gains a last column `Respond in` (bare `JB-m` or `none`). New 2l RESUME: an answered hold ticket checks out `night-shift/<old-date>-JB-n`, merges `origin/main` (the gate requires the base inside the branch), then applies / merges as is / drops / re-holds under a new ticket. Two rows of the dispatch table sharpened.
- Layout (`docs/night-shift/README.md`) and checker (`mapkit/night_shift_report.py`): a Held table with a `Respond in` header must fill it with a bare `JB-m` or `none`; Held tables without the column (committed history) still pass.
- `tools/night_shift_gate.py` NOT touched: it already releases on a closed (`complete`/`dropped`) question ticket and reads the branch's copy, which hold tickets never have; avoiding it keeps this off the full-gate tier.
- Map app (separate repo, same branch name): the serve-time join also adds `Your answer` to the Held table from its `Respond in` cell.

**Verification.**
- 2026-11-02, `python -m pytest tests/sim/test_night_shift_report.py tests/sim/test_night_shift_gate.py tests/sim/test_night_shift_runner.py tests/sim/test_night_shift_candidate.py tests/sim/test_sitting_debrief_protocol.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_question_tickets.py tests/sim/test_glossary_avoid_words.py -q`: **185 passed in 116.36 s**.
- Red without the change: `test_held_respond_in_must_be_a_bare_id_or_none` fails with `mapkit/night_shift_report.py` stashed (2026-11-02).
- Map app, 2026-11-02, `MAPKIT_SRC=<this worktree> python -m pytest tests/test_nightshift.py -q`: 10 passed; the new Held test fails with `server/nightshift.py` stashed.
