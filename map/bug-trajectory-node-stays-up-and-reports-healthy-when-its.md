---
id: BUG-96
kind: bug
title: trajectory_node stays up and reports healthy when its emitter cannot bind the command socket
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:08:54Z'
owners: []
priority: medium
diagnosis: confirmed
area: skill-stack
blocked_by: []
links: []
filed_by: claude
---

If `trajectory_node` cannot bind the :5557 command socket it logs one error and its emitter thread returns, but the node stays up and keeps accepting installs. Found by the 2026-11-13 maintainer review; the bind path was confirmed by reading.

## Findings

- `trajectory_node.py:1442-1453`: on a bind failure `_emitter_loop` logs "The emitter will NOT start" and returns.
- The reviewer found no liveness check on the thread before shutdown and no emitter-alive field in `_publish_status` (`:5501`), which reports `max_emit_gap_ms` only. That part was not re-read.
- The likely cause is a stale `trajectory_node` still holding the address. The firmware's setpoint staleness Watchdog then trips, which is the safe direction, but the operator and the GUI see a healthy planner.

## Wanted

Either exit the node on a bind failure so the launch shows it, or publish an emitter-alive flag and refuse installs while it is false. Root cause for preferring the exit: a node that cannot emit has no purpose, and a refusal path is one more state to test.
