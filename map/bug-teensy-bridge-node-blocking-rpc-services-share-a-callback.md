---
id: BUG-98
kind: bug
title: teensy_bridge_node blocking RPC services share a callback group with the 100 Hz publishers and the link-loss check
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:08:57Z'
owners: []
priority: low
diagnosis: hypothesis
area: can-bridge
blocked_by: []
links: []
filed_by: claude
---

Several `teensy_bridge_node` services make blocking RPC calls from the default callback group, which also holds the 100 Hz robot-state publisher and the 1 Hz link-loss health check. Found by the 2026-11-13 maintainer review; the group membership was read by the reviewer, the executor behaviour was inferred.

## Findings

- `bb/reload`, `bb/reset`, `bb/calibrate`, `set_hand_state`, `set_hand_gains`, `reboot_odrives` and `odrive_command` are created with no `callback_group` (`teensy_bridge_node.py:2310`, `:2378-2390`).
- `_version_check_poll` (`:4511-4535`) calls `_call_rpc` and its own comment says it shares the group of `_publish_robot_state`.
- The default RPC budget is 0.5 s times three attempts (`teensy_link/rpc.py:140`). Against a dead bridge one such call stalls the telemetry publishers and the link-loss latch for up to 1.5 s.
- The firmware owns the real safety, so this is an observability and timing gap, not a hazard.

## Wanted

Put the blocking services in their own callback group, the pattern `skill_node` already uses for waited service calls, and add a test that a slow RPC does not delay the health check.
