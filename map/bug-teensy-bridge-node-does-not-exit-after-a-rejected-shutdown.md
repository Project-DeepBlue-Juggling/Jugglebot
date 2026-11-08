---
id: BUG-47
kind: bug
title: teensy_bridge_node does not exit after a rejected shutdown Stow; launch SIGTERM-killed it 4.8 s later (2026-10-31 13:06)
status: backlog
created: '2026-11-01'
updated: '2026-11-07T19:00:06Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
diagnosis: hypothesis
area: can-bridge
blocked_by: []
links: []
filed_by: claude
---

On 2026-10-31 13:06 (launch log `~/.ros/log/2026-10-31-13-06-17-735496-jetson-1119080/launch.log`) the shutdown Stow was rejected at once (`DEACTIVATE: ERR_REJECTED`, leg 0 latched ODRIVE_FATAL after a SPINOUT_DETECTED), yet `teensy_bridge_node` did not exit; launch SIGTERM-killed it 4.8 s later (exit -15). A rejected Stow should leave nothing to wait for, so something in the teardown after it blocks for seconds.

Reproduce offline first: a FakeTeensy that rejects DEACTIVATE, then `on_shutdown()` and the node's `main()` finally, timed. Candidates: the transport stop joining the RX or heartbeat thread, or an executor or service teardown.

A second observation for the same look: at 13:13 the Stow was still unconfirmed 4.8 s after it began (normal 1.5 s). With the 8 s window from BUG-26 the next such shutdown logs its outcome.

## Findings (2026-11-08)

Not reproduced offline; no code changed. A throwaway test used the node harness with a FakeTeensy that rejects DEACTIVATE, on a latched fault, with the node stopping its own client.

- `on_shutdown()` returned in 0.9 s against the 4.8 s seen. Per phase: wire disarm 0.0 s, `_shutdown_stow` 0.001 s, `client.stop()` 0.1 s, the fixed settle 0.2 s. About 0.6 s is unaccounted for, nowhere near 4.8 s.
- A rejected Stow returns at once, before the monitor loop, so BUG-26's 8 s window is never entered.
- Ruled out as the delay: the transport join (capped at 1 s per thread), the Stow confirmation wait, and the RPC and ToD closes.

### Where the time probably went (hypothesis)

`main()`'s `finally` runs `executor.shutdown()`, and offline tests cannot reach it because it needs real rclpy. The 13:06 launch log fits a callback still running there:

- A homing goal started on the bridge at 784.81 (launch.log line 2110). No later line in the log mentions it, and no goal result appears.
- SIGINT at 792.35, Stow rejected at 792.55, SIGTERM at 797.36. All other nodes had exited by 792.91.

So the executor may be waiting on the homing goal's callback. This is not proven: homing may log nothing when it ends, and rclpy's source was not read.

### Next

- Read `executor.shutdown()` in a Session, then test with real rclpy: a long-running action callback, then SIGINT.
- A fix would touch how homing is told to stop, so it is not for an unattended night. The cost today is 4.8 s and a SIGTERM at shutdown.
- The 13:13 Stow, unconfirmed at 4.8 s, is a hardware-log question: the FakeTeensy never completes a descent.

## Progress

- 2026-11-08: Night Shift 2026-11-08: not reproduced offline; on_shutdown takes 0.9 s. Hypothesis: executor.shutdown waits on a homing callback, which needs real rclpy (detail below)
