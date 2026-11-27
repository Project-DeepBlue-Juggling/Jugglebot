---
date: 2026-11-27
title: "Night Shift 2026-11-27: merged BUG-147 and BUG-148, held BUG-139, BUG-140 and FEAT-65 (the three answered tickets built), BUG-146 diagnosed, 3 questions"
type: process
status: complete
phase: "Night Shift"
related_issues: [BUG-147, BUG-148, BUG-139, BUG-140, FEAT-65, BUG-146]
files_changed:
  - docs/night-shift/2026-11-27.md
---

# Night Shift 2026-11-27

Unattended run over the 16-Item delta (no full walk); the API credit ran out before the first call, so the whole night ran on the Claude subscription (Opus orchestrator). Report: [docs/night-shift/2026-11-27.md](../docs/night-shift/2026-11-27.md).

- **Nightly:** CLAIMED GREEN 11543/11595 passed, 0 failed, 0 errored, 2 xfailed, 50 skipped 2026-11-27T02:01:27+11:00.
- **Merged 2:** BUG-147 (flasher BEGIN and COMMIT filter on opcode), BUG-148 (the nominated worst rung was named by float noise, not by load).
- **Held 3 tonight:** BUG-139 (process noise on b, TKT-95 option 1; day Session), BUG-140 (REST rebase snapped to the Knot grid, TKT-96 option 2; 2 audit findings open, then a Sitting), FEAT-65 (x and y shrinkage, TKT-97 option 1; misses the gate on the 5455-row Memory). 6 carried holds.
- **Questions 3:** TKT-98 (BUG-146 fix choice), TKT-99 (BUG-140 audit findings), TKT-100 (FEAT-65 next step).
- **Diagnosed:** BUG-146's HAND_LIMIT_C2 refusal is the finite-difference knot rate of the Hand's tilt-drop term, read by the C2 gate's Hermite rebuild (about 0.116·θ² rev/s²); Findings and Options on the Item.
- **Skipped 11** with reasons on the Items; 146 Items unchanged.
- **Process note:** two map pushes to main mid-night moved the base under running gates: BUG-148 held at [pre] once, and BUG-147's first MERGE could not land (generated-page conflict) and was re-gated. Next time, batch map writes on main until the night's gates are done.

## Verification

2026-11-27, gate runs (`python D/gate-base/tools/night_shift_gate.py ... --record`): BUG-147 MERGE (suite exit 0 in 1326 s, 11600 collected against 11597), BUG-148 MERGE (suite exit 0 in 2764 s, 11597 against 11596). Report and logbook, 2026-11-27, `tools/night_shift_nonet.sh timeout 600 python -m pytest tests/sim/test_night_shift_report.py tests/sim/test_logbook_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py -q -p no:cacheprovider`: **69 passed in 2.82 s**.
