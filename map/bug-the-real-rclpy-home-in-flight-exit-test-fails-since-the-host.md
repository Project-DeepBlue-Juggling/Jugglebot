---
id: BUG-145
kind: bug
title: 'The real-rclpy Home-in-flight exit test fails since the host Leg IDLE gate: its fake peer sends no Leg DIAGNOSTIC frames'
status: inbox
created: '2026-11-26'
updated: '2026-11-25T15:47:33Z'
owners: []
priority: high
effort: xs
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links: []
filed_by: claude
---

The nightly of 2026-11-26 went RED on one test: `tests/rclpy_smoke/test_rclpy_smoke.py::test_bridge_node_exit_with_home_action_in_flight_bug47`. It reproduces in the Night Shift clone every run (2026-11-26, 1 failed in 11.8 s).

## Evidence

- The test's fake peer ACKs HOME but sends no DIAGNOSTIC frames, so no Leg shows HB_SEEN.
- Since `45f9545f` (2026-11-25, the host Leg IDLE rule A7), `_present_leg_states` reads six unknown Legs when none has heartbeated, and `/home_motors` refuses before Homing starts: "home REFUSED: Leg 0 state unknown ... (not Idle)".
- So the test never reaches its premise (a Home callback in flight at SIGINT), and `'homing: aborted on axis'` never appears in the log. The node itself exits promptly.

## Consequence

The BUG-47 regression pin is not exercising its path, and every `./run_tests.sh --full` run (the nightly and every Night Shift gate) is red until the test is updated.

## Wanted

The fake peer, in its `answer_home` mode, sends fresh DIAGNOSTIC frames at 1 Hz or faster for all six Legs with HB_SEEN and axis state Idle, so the A7 gate passes and the Home reaches its monitor loop. Any other Homing precondition added on 2026-11-25 (Homing only from a lowered Platform) is satisfied the same way, by the fake peer, never by relaxing the node's gate.
