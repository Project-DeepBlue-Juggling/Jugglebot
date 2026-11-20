---
title: "Night Shift 2026-11-21: 1 merged (BUG-120 per TKT-87), 3 map-app Items held on two audit findings each (BUG-121, BUG-127, FEAT-61), 4 new questions, 16 skipped; an app vitest flakes under a gate suite (BUG-128)"
type: investigation
date: 2026-11-21
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-21.md
  - logbook/2026-11-21-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
---

# Night Shift 2026-11-21

Report: [docs/night-shift/2026-11-21.md](../docs/night-shift/2026-11-21.md).

- **Counts.** 1 merged: BUG-120, resumed on the owner's TKT-87 answer. 3 held tonight: BUG-121, BUG-127 and FEAT-61, all map app, each at (b) with 2 findings open (FEAT-61 also at (a)). 7 carried holds. 4 new questions (TKT-88 to TKT-91), 0 earlier. 16 skipped; 130 Items unchanged.
- **Nightly.** `CLAIMED GREEN 10746/10798 passed, 0 failed, 0 errored, 1 xfailed, 51 skipped 2026-11-21T02:01:27+11:00`.
- **Gate verdicts (2026-11-21, `python D/gate-base/tools/night_shift_gate.py --repo . [--app-repo <app>] --base origin/main --branch <b> --item <ID> --audit <audit> [--app-log <log>] --record $NIGHT_SHIFT_VERDICTS`).**
  - BUG-120: MERGE, suite exit 0 in 1178 s, collected 10798 at base and tip.
  - BUG-121 (app): HOLD [b] 2 open; app suite EXIT=0.
  - BUG-127 (app): HOLD [b] 2 open; app suite EXIT=0.
  - FEAT-61 (app): HOLD [a] app suite EXIT=1, [b] 2 open.
- **App test flake.** FEAT-61's app suite failed on "an optimistic status rolls back when the server refuses (422)". The test holds the mocked 422 reply for only 30 ms, and the app suite ran while BUG-120's full gate suite loaded the box. The test passes alone on the same tip, and the unit saw it fail on the unchanged source. Filed as BUG-128: an app suite that runs beside a gate suite can hold a clean app branch at (a).
- **IDEA-25's trigger fired.** Rung P4 passed on 2026-11-20, so the Item now has options and question TKT-88. The recommended option is the debrief's Jetson-only hole-fill change.
- **Command guard.** One `grep` that named the probe directory was refused by the guard, as designed.
- **Cost.** 7 Sonnet units, no Opus. The API credit ran out before the first turn, so the night ran on the Claude subscription (opus). Wall clock 02:45 to about 03:20.
