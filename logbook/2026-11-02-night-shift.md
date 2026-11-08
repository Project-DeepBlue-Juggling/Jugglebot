---
title: "Night Shift 2026-11-02, the second live night: 0 merged, 2 held for one-line calls (FEAT-35 console hover marker, FEAT-7 biased sim plant), 2 question tickets (TKT-27, TKT-28), BUG-52 findings and Sitting section, BUG-53 filed; the run ended on an exhausted API credit balance while writing its report"
type: investigation
date: 2026-11-02
status: complete
phase: "night-shift (second live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-02.md
  - logbook/INDEX.md
subsystem:
  - night-shift
tags:
  - night-shift
  - report
---

# Night Shift 2026-11-02 (second live night)

**Counts.** Candidates 55 (preflight: 7 added, 18 changed, 11 removed since the last night); 14 Items touched. Merged 0. Held 2: FEAT-35 on `night-shift/2026-11-02-FEAT-35` (one audit note open: the hover marker can stick when the CONSOLE view is hidden by keyboard), FEAT-7 on `night-shift/2026-11-02-FEAT-7` (three audit findings open on the unit's own tests and CLI; the columns slice is a second unit). Questioned 2: TKT-27 on FEAT-1, TKT-28 on IDEA-17. BUG-52 got findings and a Sitting section; BUG-53 filed; BUG-50, BUG-51, BUG-32, BUG-35, FEAT-35 and BUG-52 rated; 45 skipped, each with its reason. Units: 3 Sonnet implementation or research units and 2 Sonnet audit-reporter passes. Cost USD 28.82. Report: `docs/night-shift/2026-11-02.md`.

**Nightly.** CLAIMED RED 7880/8001 passed, 29 failed, 0 errored, 0 xfailed, 92 skipped 2026-11-02T04:03:01+11:00. All 29 come from the live checkout's uncommitted HAND_LIMIT_ACC raise to 4500 against a committed box swept at 3900 (`LimitsMismatch`); HEAD is green on the box test.

**How the night ended.** The orchestrator's API call failed with HTTP 400 "Credit balance is too low" at 05:48, after the report was written but before the Skipped rows were filled, the logbook entry written and the commit made. Everything else had been pushed before then (9 `map:` commits on `main`, both held branches at their gated tips), so the Night Shift's invariant held: nothing was left half-written on `main`. A daytime Session finished the report from the orchestrator's drafts (`temp/reports/night-shift/report-draft-skipped.md`) on 2026-11-02 and committed it. The runner's `last_run.json` records `CLAUDE_FAILED` and its `detail` field quotes the first line of `claude.err`, which is the connectors warning and not the cause; the cause is the `result` field of `2026-11-02.claude.json`.

## Verification

- Held tips and their gate runs (`./run_tests.sh --full`, 2026-11-02, clone): FEAT-35 31f6fc46 7926 passed + 8 serial `EXIT=0`; FEAT-7 0d414162 7928 passed + 8 serial `EXIT=0`; both branch heads equal `origin/` and the gated `HEAD=`.
- Report layout (`python -c "import mapkit.night_shift_report as r; print(r.report_errors(open('docs/night-shift/2026-11-02.md').read()))"`, 2026-11-02): `[]`.
