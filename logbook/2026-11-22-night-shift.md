---
title: "Night Shift 2026-11-22: 4 map-app Items merged (FEAT-61, BUG-121, BUG-127 on TKT-89 to TKT-91; BUG-128 the app test flake), 0 held tonight, 6 carried, Sunday full walk with 71 skips rewritten; FEAT-62 split off BUG-127"
type: investigation
date: 2026-11-22
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-22.md
  - logbook/2026-11-22-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
---

# Night Shift 2026-11-22

Report: [docs/night-shift/2026-11-22.md](../docs/night-shift/2026-11-22.md).

- **Counts.** 4 merged, all map app: BUG-128 first, so the three answered holds re-gated on a stable app suite, then FEAT-61 (TKT-91 merge as is), BUG-121 (TKT-89 apply both fixes) and BUG-127 (TKT-90: a Problem Step counts as done, apply the test fixes). 0 held tonight, 6 carried holds. 0 new questions, 0 earlier. 71 skipped on the Sunday full walk (73 skip reasons rewritten, BUG-105's and FEAT-32's stale notes corrected); 69 Items unchanged. FEAT-62 filed: the owner wants the sheet builder to derive Step dependencies. IDEA-25 pinned as owner-session per TKT-88.
- **Nightly.** `CLAIMED GREEN 10875/10925 passed, 0 failed, 0 errored, 1 xfailed, 49 skipped 2026-11-22T02:01:31+11:00`.
- **Gate verdicts (2026-11-22, `python D/gate-base/tools/night_shift_gate.py --repo . --app-repo <app> --base origin/main --branch <b> --item <ID> --app-log <log> --audit <audit> --record $NIGHT_SHIFT_VERDICTS`).** BUG-128 MERGE (tip 87a59f5), FEAT-61 MERGE (f88694a), BUG-121 MERGE (5ef81a1), BUG-127 MERGE (4e6271f), each on an app suite EXIT=0 (`night_shift_nonet.sh tools/night_shift_app_suite.sh --app <app> --mapkit .`; last run, BUG-127: server 143 passed, 3 skipped; frontend lint, typecheck, vitest, build and contract check rc=0).
- **Merge order.** All three held branches edit `SheetView.tsx`; merging `origin/main` into each in turn (FEAT-61, then BUG-121, then BUG-127) auto-merged without conflicts.
- **Cost.** 6 Sonnet units, no Opus. The API credit ran out before the first turn, so the night ran on the Claude subscription (opus). Wall clock 02:45 to about 03:25.
