---
title: "Probe archival convention started + completed session prompt deleted"
type: refactor
date: 2026-08-09
status: resolved
phase: "housekeeping (post ERR_TIMEOUT closure)"
files_changed:
  - logbook/2026-08-09-probe-archival-and-prompt-cleanup.md
  - logbook/INDEX.md
  - plans/active/INDEX.md
  - plans/active/PROMPT-err-timeout-hand-path.md
  - tests/hardware/session_err_timeout_bench.md
  - tools/probes/README.md
  - tools/probes/archived/hand_dispatch_ladder.py
  - tools/probes/archived/link_status_flash_control.py
subsystem:
  - tooling
tags:
  - housekeeping
  - probes
---

# Probe archival convention started + completed session prompt deleted

Two owner-directed housekeeping moves after the ERR_TIMEOUT closure
(`2026-08-02-err-timeout-attribution-instrumentation`, CLOSED 2026-08-09):

1. **`tools/probes/archived/` convention started.** Probes whose motivating
   investigation is CLOSED move there — committed and runnable (a validated
   instrument for future regression checks), but out of the "what would I
   reach for today" directory. Each keeps a ledger row in
   `tools/probes/README.md § Archived probes`. First occupant:
   `hand_dispatch_ladder.py` (the instrument behind the 15/40→0/40
   discriminator and the 120/120 FW 10 validation). Second occupant, same
   day: `link_status_flash_control.py` (flash-comparison control for the
   CAN3 drive-path arc, CLOSED 2026-07-31). `link_status_health_scan.py`
   stays LIVE deliberately — it is the generic per-session `/link_status`
   scanner that later arcs (the ERR_TIMEOUT recount included) reused; the
   convention archives by investigation status, not by age. Live references
   updated (bench runbook, memory, README cross-reference, each probe's own
   usage docstring); historical logbook text deliberately untouched —
   it records where the file lived when its commits were made. Grep before
   and after the move: 0 live references to the old path remain.
2. **`plans/active/PROMPT-err-timeout-hand-path.md` DELETED** (owner
   convention, first instance): a completed prompt's arc lives entirely in
   the logbook, so the file adds nothing once its Done-means list is
   satisfied — and a stale prompt with a live-looking fence is exactly the
   "stale plan documents actively mislead" class (H-G,
   `2026-07-29-can3-bus-health-flap-hand-sensor-poller`). The convention is
   recorded in `plans/active/INDEX.md § Orchestration prompts`.

## Verification

- Reference sweep before the move (grep, run 2026-08-09): live pointers in
  `tests/hardware/session_err_timeout_bench.md` and memory only; both
  updated; post-move sweep: 0 hits outside the closed entry's historical
  text. The pre-commit audit caught one more the sweep pattern missed: the
  probe's OWN usage docstring still named its old path — fixed (the
  runnability promise beats byte-identical-rename purity; the move still
  registers as a rename at ~99 % similarity). The same docstring fix was
  applied pre-emptively to `link_status_flash_control.py` (three usage
  lines).
- `./run_tests.sh`, run 2026-08-09 (twice — once per archival commit; latest cited): **4226 passed, RESULT PASS** (187.51 s
  parallel; serial phase empty). A first run FAILED on
  `test_index_mentions_no_plan_that_left_plans_active` — the plans-index
  guard correctly caught the deletion note naming the deleted file; the note
  was reworded to honour the guard rather than weakening it, and the guard's
  behaviour is now documented in the note itself.
