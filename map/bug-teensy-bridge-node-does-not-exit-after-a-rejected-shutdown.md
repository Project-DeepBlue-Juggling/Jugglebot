---
id: JB-133
kind: bug
title: teensy_bridge_node does not exit after a rejected shutdown Stow; launch SIGTERM-killed it 4.8 s later (2026-10-31 13:06)
status: inbox
created: '2026-11-01'
updated: '2026-11-01T01:57:47Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
diagnosis: unknown
area: can-bridge
blocked_by: []
links: []
filed_by: claude
---

On 2026-10-31 13:06 (launch log `~/.ros/log/2026-10-31-13-06-17-735496-jetson-1119080/launch.log`) the shutdown Stow was rejected at once (`DEACTIVATE: ERR_REJECTED`, leg 0 latched ODRIVE_FATAL after a SPINOUT_DETECTED), yet `teensy_bridge_node` did not exit; launch SIGTERM-killed it 4.8 s later (exit -15). A rejected Stow should leave nothing to wait for, so something in the teardown after it blocks for seconds.

Reproduce offline first: a FakeTeensy that rejects DEACTIVATE, then `on_shutdown()` and the node's `main()` finally, timed. Candidates: the transport stop joining the RX or heartbeat thread, or an executor or service teardown.

A second observation for the same look: at 13:13 the Stow was still unconfirmed 4.8 s after it began (normal 1.5 s). With the 8 s window from JB-82 the next such shutdown logs its outcome.
