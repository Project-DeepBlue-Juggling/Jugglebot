---
title: "Night Shift 2026-11-05: 0 merged, 1 held (JB-163, the CONSOLE window line and pinned FW_CHECK lines, suite green, 3 audit findings open), 4 question tickets (JB-170 to JB-173); JB-159's reopened overlap did not reproduce headless"
type: investigation
date: 2026-11-05
status: complete
phase: "night-shift (fifth live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-05.md
  - logbook/2026-11-05-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
  - report
---

# Night Shift 2026-11-05 (fifth live night)

**Counts.** There were 74 candidates. Merged 0, held 1 (JB-163), questioned 3 (JB-159, JB-167, JB-169), 4 question tickets (JB-170 to JB-173), skipped 70. Report: `docs/night-shift/2026-11-05.md`.

**Nightly.** `CLAIMED GREEN 8170/8274 passed, 0 failed, 0 errored, 0 xfailed, 104 skipped 2026-11-05T04:03:26+11:00`.

**What the night found.**
- JB-159 was reopened by the owner. Headless Chromium (snap `chromium --headless --dump-dom`, served over a local `http.server`) ran the real chart modules, live and replay. The pills did not overlap, and the installed GUI unit serves the live checkout with `no-cache`. So the night asked for a screenshot instead of guessing a fix.
- Side note: the repo copy `tools/systemd/jugglebot-gui.service` still names the retired `Jugglebot-skills` worktree, while the installed unit names `~/Desktop/Jugglebot`. That is the drift its README warns about.
- JB-169: the bag shows why the final CATCH is judged at all, but not whether Ball 1 was in the cup. Four rule options went to the owner.
- JB-163 was built as answered. The gate held it on the audit's label findings.
- Units ran in isolated worktrees in parallel, and their diffs were applied to the branch in the clone.

**Verification.** Full gate (`./run_tests.sh --full`, 2026-11-05) on JB-163 d3821ed8: 8215 passed + 9 serial, `EXIT=0`. `origin/main`'s `tools/night_shift_gate.py` (run with `PYTHONPATH=.`) said `HOLD [b] behaviour-affecting findings open: 3`. The map-write check (`pytest -q tests/sim/test_map_items.py tests/sim/test_map_items_rated.py tests/sim/test_glossary_avoid_words.py tests/sim/test_mapkit_docs.py`) was green before each `map:` push: 153, 154, then 156 passed. It caught one avoid-word in JB-170 before the push.

**Cost.** API until the credit ran out at launch, then the Claude subscription (opus). Units: 4 Sonnet. Wall clock: 04:45 to 05:10 AEDT.
