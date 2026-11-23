---
title: Night Shift 2026-11-24 - 0 merged, 0 held, 0 questions, 11 skipped
type: investigation
date: 2026-11-24
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
---

# Night Shift 2026-11-24

Delta of 12 Items (not a full walk, 147 unchanged). Report: [docs/night-shift/2026-11-24.md](../docs/night-shift/2026-11-24.md).

- **Answered:** TKT-92, option 4: ACTIVATE lands on the levelled neutral. That is a firmware RPC change and a flash, so BUG-130 was re-rated (effort m, hardware robot) and skipped `frozen-path` for a day Session.
- **Skipped:** BUG-132 and IDEA-27 (sitting); BUG-135, BUG-136 and FEAT-64 (frozen-path, firmware); BUG-133 (out-of-reach, BallButler repo); BUG-129 (waiting-trigger, the shm_clean launches); IDEA-25 (owner-session); IDEA-26 and IDEA-28 (low-priority, Feasibility written).
- **Findings:** BUG-136's offline check is weaker than its body said. `cone/catch_event` does not publish the bridge arrival time, so the recorded arrival adds the 0 to 10 ms drain, the size of the step it looks for.
- **Held:** none new; 5 carried (FEAT-59, BUG-95, IDEA-23, FEAT-50, BUG-122).
- **Nightly:** CLAIMED GREEN 11068/11125 passed, 0 failed, 0 errored, 1 xfailed, 56 skipped 2026-11-24T02:04:01+11:00.
- **Cost:** no units and no gate runs. The API run stopped on the credit balance before touching anything; the night ran on the Claude subscription (opus).

## Verification

- 2026-11-24, `python -m mapkit item skip --batch temp/reports/night-shift/skips-2026-11-24.json --commit --no-push`: 11 of 11 Items changed, map-write check clean (2a7c3f60).
- 2026-11-24, `python -m mapkit check` before the hand-made notes commit 6446397c: clean.
