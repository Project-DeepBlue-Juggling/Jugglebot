---
title: Night Shift 2026-11-23 - 1 merged, 1 held, 1 question, 5 skipped
type: investigation
date: 2026-11-23
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
---

# Night Shift 2026-11-23

Delta of 8 Items (not a full walk, 146 unchanged). Report: [docs/night-shift/2026-11-23.md](../docs/night-shift/2026-11-23.md).

- **Merged:** FEAT-63. Map search lists the groups of ongoing exact-number matches first (jugglebot-map:19fa7c9).
- **Held:** BUG-105 on `night-shift/2026-11-23-BUG-105`. The Hand's vel_ff is zeroed on every suppress edge, as can-bridge FW 49. The gate held it at (b) for two test gaps, at (c) for bench-hand and at (f) for firmware. 5 holds were carried from earlier nights.
- **Questioned:** BUG-130 through TKT-92: leave the held Active pose as designed, add a GUI not-Level indicator, add an automatic go_home, or land ACTIVATE on Level.
- **Skipped:** BUG-131 (sitting), BUG-129 (waiting-trigger), FEAT-62 (too-large), IDEA-20 and IDEA-25 (owner-session).
- **Nightly:** CLAIMED GREEN 11011/11064 passed, 0 failed, 0 errored, 1 xfailed, 52 skipped 2026-11-23T02:00:27+11:00.
- **Cost:** 5 units (1 Opus, 4 Sonnet) and 4 gate runs. The first BUG-105 run held at [pre] because tonight's map commits had moved `origin/main`. The second ran the suite red: the stall-bench test's fake bridge still reported FW 48, and 231f5179 fixed that. The night ran on API credit until the balance ran out, then on the Claude subscription (opus).

## Verification

- 2026-11-23, `python tools/night_shift_gate.py ... --branch night-shift/2026-11-23-FEAT-63` (the app gate): MERGE.
- 2026-11-23, `python tools/night_shift_gate.py ... --branch night-shift/2026-11-23-BUG-105`: HOLD (b, c, f); suite exit 0 in 1218 s, collected base 11065, tip 11065.
