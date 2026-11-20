---
title: "Night Shift 2026-11-20: 3 merged (BUG-115, BUG-96, BUG-90 closed), 6 held (BUG-120, FEAT-59, BUG-93 renumbered to FW 46, 3 carried), 1 new question; parallel gates starved each other's test count"
type: investigation
date: 2026-11-20
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-20.md
  - logbook/2026-11-20-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
---

# Night Shift 2026-11-20

Report: [docs/night-shift/2026-11-20.md](../docs/night-shift/2026-11-20.md).

- **Counts.** 3 merged: BUG-115 and BUG-96 through the gate, and BUG-90 closed by the owner's TKT-85 answer. 6 held: BUG-120 (TKT-87), FEAT-59 (gate (i) by design), BUG-93 (renumbered to can-bridge FW 46) and 3 carried. 1 new question (TKT-87), 0 earlier. 0 skipped; 144 Items unchanged.
- **Nightly.** `CLAIMED GREEN 10608/10661 passed, 0 failed, 0 errored, 1 xfailed, 52 skipped 2026-11-20T02:00:27+11:00`.
- **Gate verdicts (2026-11-20, `python D/gate-base/tools/night_shift_gate.py --repo . --base origin/main --branch <b> --item <ID> --audit D/audit-<ID>.md --record $NIGHT_SHIFT_VERDICTS`).**
  - BUG-115: MERGE, suite exit 0 in 2173 s, collected 10661 to 10680.
  - BUG-96, first run: HOLD [i], suite exit 0 in 1199 s.
  - BUG-96, re-gate: MERGE, suite exit 0 in 1114 s, 10680 to 10692.
  - BUG-120: HOLD [b] 2 open, suite exit 0 in 1108 s, 10692 to 10692.
- **Lesson: run gates one at a time.** BUG-96's first gate held at (i) because the base's test collection timed out after 1800 s. BUG-115's gate had been queued at the same time, and it took the box lock between BUG-96's suite and BUG-96's collection. A third queued gate (BUG-120) was stopped, and from then on gates ran one at a time. All three later gates got a verdict on their first run.
- **Merges move main.** Every commit to `main` (the map writes and the BUG-90 closure) meant merging `main` into each open branch again before its gate. Tonight's own map writes were therefore held until after the last gate.
- **Cost.** 7 Sonnet units, no Opus. The API credit ran out before the first turn, so the night ran on the Claude subscription (opus). Wall clock 02:46 to 04:45.
