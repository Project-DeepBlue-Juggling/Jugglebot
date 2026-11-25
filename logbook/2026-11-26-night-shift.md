---
title: Night Shift 2026-11-26 - 3 merged (BUG-145, FEAT-66, BUG-143), 3 held, 3 questions, 18 skipped
type: investigation
date: 2026-11-26
status: complete
phase: "night-shift (live night)"
related_plan:
  - night-shift.md
---

# Night Shift 2026-11-26

Delta of 26 Items (not a full walk, 131 unchanged), plus BUG-145, which was filed and merged tonight. Report: [docs/night-shift/2026-11-26.md](../docs/night-shift/2026-11-26.md). Nightly: `CLAIMED RED 11381/11435 passed, 1 failed, 0 errored, 2 xfailed, 51 skipped 2026-11-26T02:03:27+11:00`.

- **Merged:**
  - BUG-145: the nightly's one failure. The real-rclpy BUG-47 test's fake peer gave no Leg state, so the new host Leg IDLE gate refused its Home. Entry: [2026-11-26-rclpy-smoke-home-test-leg-diagnostics](2026-11-26-rclpy-smoke-home-test-leg-diagnostics.md).
  - FEAT-66: the IMU charts button sits with the other show/hide buttons. Entry: [2026-11-26-gui-imu-charts-button-with-odrive-toggle](2026-11-26-gui-imu-charts-button-with-odrive-toggle.md).
  - BUG-143: a race in the stall watchdog GIL test. Entry: [2026-11-26-stall-watchdog-gil-test-names-before-stall](2026-11-26-stall-watchdog-gil-test-names-before-stall.md).
- **Held:**
  - BUG-138 at (f): built as the owner's TKT-93 option 3, with I-LEARN-1 amended.
  - FEAT-65 by the owner's TKT-94 order: the replay misses the Apex gate. TKT-97 asks what to do.
  - BUG-144 at (c): the bag record flag needs a Sitting's bag.
  - Four carried holds; three are superseded by day Session work on main.
- **Questions:** TKT-95 (BUG-139), TKT-96 (BUG-140), TKT-97 (FEAT-65).
- **Lesson:** the first BUG-145 gate held on eight wall-clock tests. Two units' scoped tests were running beside it (load average about 15). The re-run on a quiet box passed. Never run a unit's tests while a gate suite runs. Hold `main` pushes until the night's merges are done: each push makes every waiting branch re-merge `origin/main` before its gate.

## Verification

Gate records under `$NIGHT_SHIFT_VERDICTS`, 2026-11-26, `python D/gate-base/tools/night_shift_gate.py --repo . --base origin/main --branch night-shift/2026-11-26-<ID> ...`:

- BUG-145 at 58df3e10: HOLD, suite exit 1 (load).
- BUG-145 at 85911d25: **MERGE**, suite exit 0 in 1268 s.
- FEAT-66 at 2d92a491: **MERGE**, 1287 s.
- BUG-143 at 279c933c: **MERGE**, 1249 s.
