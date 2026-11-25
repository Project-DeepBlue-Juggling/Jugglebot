---
id: BUG-141
kind: bug
title: SIGINT during the bridge node's boot-state read skips its shutdown handler
status: complete
created: '2026-11-25'
updated: '2026-11-25T03:58:10Z'
owners: []
priority: low
effort: s
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: can-bridge
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

## Progress

- 2026-11-25: fixed with BUG-47: a SIGINT during the boot-state read is caught and main() runs the normal shutdown. Residual: an interrupt earlier in the constructor (a milliseconds window) skips it. Entry 2026-11-25-bridge-sigint-exit-bug47.
