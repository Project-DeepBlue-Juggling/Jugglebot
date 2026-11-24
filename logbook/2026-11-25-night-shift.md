---
title: Night Shift 2026-11-25 - 1 merged (BUG-137), 0 held, 2 questions, 5 skipped
type: investigation
date: 2026-11-25
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
---

# Night Shift 2026-11-25

Delta of 6 Items (not a full walk, 153 unchanged). Report: [docs/night-shift/2026-11-25.md](../docs/night-shift/2026-11-25.md).

- **Merged:** BUG-137. The plant model's at-prior decisions read a no-forgetting covariance, so a long single-Site Memory no longer applies an invented slope at a far Site. Entry: [2026-11-25-plant-model-far-site-guard-unforgotten-information](2026-11-25-plant-model-far-site-guard-unforgotten-information.md).
- **Filed:** BUG-139, the residue of BUG-137. The slope's value is still corrupted, so one row at a second Site opens the guard onto a line up to 59 mm off, as before the fix. The fix needs an owner decision on the forgetting law.
- **Questions:** TKT-93 (BUG-138, how to separate release kinds in the plant model) and TKT-94 (FEAT-65, the E2 gate and who merges).
- **Findings:** IDEA-29 was classified from two 2026-11-20 bags. Most high-jerk intervals are real steps once per Throw cycle at the Segment boundary; 15 to 18 % are dropped-knot echo artefacts. Feasibility written.
- **Skipped:** BUG-129 (waiting-trigger) and FEAT-1 (owner-session), both unchanged.
- **Held:** none new; 5 carried (FEAT-59, BUG-95, IDEA-23, FEAT-50, BUG-122).
- **Nightly:** CLAIMED GREEN 11071/11129 passed, 0 failed, 0 errored, 1 xfailed, 57 skipped 2026-11-25T02:01:57+11:00.
- **Cost:** 4 units (1 Opus, 3 Sonnet), 1 gate run. The API run stopped on the credit balance before touching anything. The rest ran on the Claude subscription (opus), and the Opus unit was cut off by its usage limit at its logbook step.

## Verification

- 2026-11-25, `python D/gate-base/tools/night_shift_gate.py ... --item BUG-137`: **MERGE**, suite exit 0 in 1369 s (`./run_tests.sh --full` on 6dfc7a73), collected base 11129, tip 11137.
- 2026-11-25, `python -m mapkit item skip --batch temp/reports/night-shift/skips-2026-11-25.json --commit --no-push`: 3 of 3 Items changed, map-write check clean (090fbd0c).
