---
title: "The real-rclpy Home-in-flight exit test's fake peer now sends six fresh Idle Leg DIAGNOSTIC frames, so the host Leg IDLE rule lets its Home start again (BUG-145)"
type: bugfix
date: 2026-11-26
status: complete
phase: "Night Shift 2026-11-26"
related_plan: ""
related_issues: [BUG-145]
files_changed:
  - tests/rclpy_smoke/test_rclpy_smoke.py
subsystem: [tests, ros]
tags:
  - night-shift
---

# Real-rclpy Home-in-flight test: Leg DIAGNOSTIC frames from the fake peer

**What.** `tests/rclpy_smoke/test_rclpy_smoke.py::test_bridge_node_exit_with_home_action_in_flight_bug47` failed in the 2026-11-26 nightly and in every run since `45f9545f` (2026-11-25). That commit made the host Leg IDLE rule (ARMING_CONTRACT A7) read six unknown Legs when no Leg has heartbeated. The test's fake peer sent no DIAGNOSTIC frames, so `/home_motors` was refused before Homing began. The test's premise, a Home callback in flight at SIGINT, never happened, and its `'homing: aborted on axis'` assertion failed.

**Fix.** In `answer_home` mode `_fake_peer` now pumps one DIAGNOSTIC per Leg at 5 Hz (HB_SEEN, axis state Idle) once the bridge has spoken. That is the state a lowered Platform at Stow reports. The node's gate is unchanged. The address-learning loop is shared with the `unpowered` pump.

**Why the test, not the gate.** The gate is right: a Home with Legs of unknown state must be refused. The test had been relying on the gate's old pass-when-empty behaviour, which `45f9545f` closed on purpose.

## Verification

- Before (2026-11-26, `tools/night_shift_nonet.sh timeout 300 python -m pytest tests/rclpy_smoke/test_rclpy_smoke.py -q -k bug47 -p no:cacheprovider`): **1 failed, 1 passed**, the same refusal as the nightly.
- After (2026-11-26, `tools/night_shift_nonet.sh timeout 400 python -m pytest tests/rclpy_smoke/test_rclpy_smoke.py -q -p no:cacheprovider`): **6 passed in 20.04 s**.
