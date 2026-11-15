---
id: TKT-77
kind: ticket
title: 'BUG-96 held on night-shift/2026-11-16-BUG-96: apply the audit''s clean-exit fix and merge, merge as is, or drop?'
status: open
created: '2026-11-16'
updated: '2026-11-15T19:02:06Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-trajectory-node-stays-up-and-reports-healthy-when-its
type: question
---

BUG-96 is built on night-shift/2026-11-16-BUG-96: when the emitter cannot bind :5557, trajectory_node sets a flag, shuts rclpy down and `main()` exits 1, so the launch shows a dead node instead of a healthy-looking one. The audit found that on real Foxy the bind usually fails before `main()` builds the executor, so the executor constructor raises on the dead context, `on_shutdown()` never runs, and the exit code is 1 only because of a traceback.

## Options

1. **Apply the audit's fixes and merge:** right after constructing the node, `main()` checks the flag (or `rclpy.ok()`), runs `on_shutdown()` and `destroy_node()`, and exits 1; a test fakes an executor whose constructor raises after shutdown; the logbook is reworded. The next night builds it, re-gates and merges on MERGE.
2. **Merge as is:** the user-visible fix holds (the node dies, exit non-zero), but by traceback and without closing the planner worker cleanly.
3. **Drop the branch.**

## What each answer unblocks

- 1 or 2: the next night re-gates and merges.
- 3: BUG-96 goes back to open.

Recommendation: 1. It is a few lines, and it makes the exit deterministic and closes the segment-solver worker.
