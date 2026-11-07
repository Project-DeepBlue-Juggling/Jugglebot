---
title: "Next Sitting Runbook no longer says to latch hand_source: the latch was removed on 2026-09-11"
type: bugfix
date: 2026-11-07
status: complete
phase: night-shift
related_plan:
  - night-shift.md
files_changed:
  - mapkit/sitting.py
  - docs/runbook.md
subsystem:
  - night-shift
tags:
  - runbook
---

# Runbook drops the hand_source step

**What.** Removed "Latch `hand_source` before arming" from Setup 0 of the Next Sitting Runbook, at its source in `mapkit/sitting.py`, and regenerated `docs/runbook.md`. Owner's request.

**Why.** The latch no longer exists. The firmware dropped it at FW 21 and the host at PROTOCOL 7 (commit 1e2c0c91, 2026-09-11): the can-bridge streamed lane is now the one Hand master. The step could not be carried out, and it sat in the list the owner reads before arming. Found while checking the memory index's safety rules against the code. No motion, wire or firmware change.

## Verification

Scoped (2026-11-07, `python -m pytest tests/sim/test_sitting_sections.py tests/sim/test_sitting_debrief_protocol.py tests/sim/test_mapkit_docs.py tests/sim/test_mapkit_no_ros.py tests/sim/test_map_items.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py -q`): 104 passed. Readers grepped: `mapkit/sitting.py` is read by `mapkit/gen_docs.py`, `tools/night_shift_candidate.sh`, their tests and the map app, and only the generated page changes. The grep also named `tests/sim/test_night_shift_candidate.py`, which the first run missed; run after the push (2026-11-07, `python -m pytest tests/sim/test_night_shift_candidate.py -q`): 5 passed.
