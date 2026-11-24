---
id: BUG-141
kind: bug
title: SIGINT during the bridge node's boot-state read skips its shutdown handler
status: inbox
created: '2026-11-25'
updated: '2026-11-24T17:39:47Z'
owners: []
priority: low
blocked_by: []
links: []
filed_by: claude
---

A SIGINT during `teensy_bridge_node`'s constructor boot-state read lands outside `main()`'s `try`. The node dies with a KeyboardInterrupt traceback (exit -2) and skips `on_shutdown`. Found by the real-rclpy smoke harness (FEAT-55, 2026-11-24).

## Findings

- The boot-state read takes about 5 s on a dead link, so the window is wide at a launch with the bridge unreachable.
- The harness waits for the "TeensyBridgeNode up" line before it sends SIGINT, to stay out of this window.
- Related: BUG-47 (slow exit with a homing goal in flight).

## Wanted

- Construct the node inside the guarded block, so an interrupt during the boot read runs the same shutdown path.
