---
id: BUG-98
kind: bug
title: teensy_bridge_node blocking RPC services share a callback group with the 100 Hz publishers and the link-loss check
status: complete
created: '2026-11-14'
updated: '2026-11-18T03:09:09Z'
owners:
- night-shift
priority: low
effort: s
hardware: robot
owner: none
radius: subsystem
diagnosis: hypothesis
area: can-bridge
blocked_by: []
links:
- logbook/2026-11-18-sitting-debrief.md
- logbook/2026-11-16-bridge-blocking-rpc-callback-group.md
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

## Sitting

- setup: level, bb-fed
- why: The branch moves the bridge node's six blocking services and both firmware-version polls into their own callback group, so a slow call no longer stalls the 100 Hz robot state and the link-loss check; the node's threading changed, so its normal startup is flown once before it is trusted.
- do:
  - Launch on the branch. In a second terminal, `ros2 topic hz /robot_state` and leave it running.
  - Boot, Home, Level as usual, with Ball Butler connected and calibrated (its calibrate and Feed services are among those moved).
  - Watch the hz terminal while the Ball Butler calibrate runs, and note any service that answers differently from the last Sitting (refused, slow, or a timeout).
- pass: startup and the Ball Butler calibrate behave as on the last Sitting, and `/robot_state` holds near 100 Hz throughout the calibrate.
- record: the lowest `/robot_state` rate seen during the calibrate, any service that failed or hung, and the launch log path.
- minutes: 10
- branch: night-shift/2026-11-16-BUG-98

## Sitting result (2026-11-18 13:05)

- verdict: pass
- notes: /robot_state held at almost exactly 98 Hz throughout the calibrate; basically unchanged since launch (sheet 2026-11-18-robot, step 32) [derived: Bag 2026-11-18_11-31-37, calibrate window 11:37:30.3-11:37:51.9: /robot_state mean 98.6 Hz, lowest 1 s bin 95 Hz, longest gap 44.5 ms; main the same day 98.3 and 98.8 Hz. No service failed or hung. No log line shows the bridge calibrate RPC itself being called.]

## Progress

- 2026-11-16: Night Shift 2026-11-16: held on night-shift/2026-11-16-BUG-98: blocking services in their own callback group, suite and audit clean; unblocked by a Sitting
- 2026-11-18: Passed at the Sitting of 2026-11-18: /robot_state held 98.6 Hz (lowest second 95) through the Ball Butler calibrate; landed on main as a5037ab3 through tools/sitting_merge.sh. Audit: 0 behaviour-affecting, 3 narrative findings recorded in the debrief entry.
