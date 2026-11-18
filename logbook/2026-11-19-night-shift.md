---
title: "Night Shift 2026-11-19: 3 merged (BUG-104, BUG-108, FEAT-60), 6 held (BUG-96 tonight, 5 carried), 4 new questions; serial tests failed under a day Session's load until 04:45"
type: investigation
date: 2026-11-19
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
files_changed:
  - docs/night-shift/2026-11-19.md
  - logbook/2026-11-19-night-shift.md
  - logbook/INDEX.md
subsystem:
  - night-shift
---

# Night Shift 2026-11-19

Report: [docs/night-shift/2026-11-19.md](../docs/night-shift/2026-11-19.md).

**Counts:** merged 3 (BUG-104 and BUG-108 through the gate; FEAT-60 closed as a passed check), held 6 (BUG-96 tonight, 5 carried), new question tickets 4 (TKT-83 to TKT-86), skipped 22 of the 29-Item delta (the 4 answered tickets are not rows), unchanged 120. Filed BUG-120 for a load flake in `test_install_segment.py`.

**Nightly:** `CLAIMED GREEN 10345/10399 passed, 0 failed, 0 errored, 1 xfailed, 53 skipped 2026-11-19T02:01:58+11:00`.

**What made the night slow:** from about 03:00 to 04:55 a day Session's pytest workers held the box at load 14 to 21. Under that load the serial wall-clock tests (`test_loop_timing`, `test_ipc_latency`, the motor_guard Hermite tolerance, the check-segment budget) failed in three of five suites. After 04:45 both re-run suites passed: BUG-104 (`flock /tmp/jugglebot-tests.lock night_shift_nonet.sh ./run_tests.sh --full`, 2026-11-19: 10338 passed + 10 serial, PASS) and BUG-108 (same command, 2026-11-19: 10331 passed + 10 serial, PASS). The first BUG-104 run was cut off at 44% by the orchestrator's 10-minute default background-command limit. Map writes pushed mid-night moved `origin/main` under two running suites, and the gate held both at `[pre]`. Those branches were merged up and re-run, and the night's remaining `main` writes were kept local until the last gate.

**Cost:** 5 Sonnet agents (1 implementation unit, 4 audit-reporter runs); wall clock 02:46 to 05:05; auth: API until the credit balance ran out, then the Claude subscription (opus).
