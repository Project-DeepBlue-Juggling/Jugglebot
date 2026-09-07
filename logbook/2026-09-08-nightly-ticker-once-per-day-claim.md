---
title: "Nightly ticker: the 04:00 result is claimed once per day — the first session reads it, every later session sees LOWERED and reads nothing; the token-budget agent rules land in CLAUDE.md"
type: feature
date: 2026-09-08
status: resolved
phase: "tooling — nightly runner / session workflow"
files_changed:
  - tools/nightly_ticker.sh
  - tools/nightly_suite.sh
  - tools/systemd/README.md
  - CLAUDE.md
subsystem:
  - tooling
  - docs
---

# Nightly ticker: once-per-day claim on the 04:00 result; token-budget rules in CLAUDE.md

**What.** Every session read `temp/reports/nightly/status` at start, so with several
sessions a day the same RED was read and re-diagnosed N times — and this week's two REDs
(09-05, 09-07) were one session's own 04:00 work-in-progress, since the nightly runs
against the working tree. `tools/nightly_suite.sh` now calls `tools/nightly_ticker.sh raise`
after every `status` write (DEFERRED exit and final render/fallback). A session runs
`tools/nightly_ticker.sh check --who "<session>"`: the first claims `ticker.raised` by an
atomic rename to a per-PID file (rename(2) guarantees one winner), writes `ticker.lowered`
with run date, ack time and label, and prints `CLAIMED <status>`, so it reads `latest.md`
and surfaces the result; every later session prints `LOWERED … by <who>`
and reads nothing. `STALE` (>2 days), `NEVER`, `UNRAISED` (status but no ticker: read once)
keep the runner-health cases visible. Subagents never run the check.

**Also in this change.** CLAUDE.md gains the owner-approved rules from the 2026-08-30..09-07
retrospective (recommendations 1–7; 8 declined): a top Workflow-Rules bullet "Token budget
governs agent use" (cap agents at ~80 calls with a handoff, Sonnet default, phase excerpts
not the plan, one audit per phase, compact after commit, logs by path), a pre-sitting
dress-rehearsal rule, "the nightly measures the working tree", and end-of-day commit. The
same rules are in `~/.claude/…/memory/feedback_*.md` and the MEMORY.md index.

**Verification.**
- 2026-09-08, `NIGHTLY_REPORTS_DIR=<scratch>` functional run, 11 cases: NEVER / refused
  raise without status / UNRAISED / CLAIMED then LOWERED / **six concurrent checks → exactly
  one CLAIMED** / STALE at 3 days / DEFERRED raises and claims. `bash -n` clean on both scripts.
- 2026-09-08, live seed (`raise` + `check` on `temp/reports/nightly/`): CLAIMED
  `GREEN 6968/6974 … 2026-09-08T04:01:34+10:00`, second check LOWERED. That run predates the
  wiring, so today's flag was raised by hand; the runner raises tomorrow's.
- 2026-09-08, `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py -q`
  (the tests that read the changed paths; no `*.py`/`*.yaml` touched): **34 passed in 0.53s**.
