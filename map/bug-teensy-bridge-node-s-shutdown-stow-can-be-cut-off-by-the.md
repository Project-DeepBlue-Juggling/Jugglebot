---
id: BUG-26
kind: bug
title: teensy_bridge_node's shutdown Stow can be cut off by the launch's 5 s SIGTERM; teensy_link has no single-owner interlock
status: complete
created: '2026-10-28'
updated: '2026-11-01T01:57:22Z'
owners: []
priority: medium
effort: s
hardware: none
owner: decision
radius: subsystem
diagnosis: confirmed
area: can-bridge
blocked_by: []
links: []
filed_by: claude
---

Two halves of one hazard: who owns the bridge link while the node shuts down.

- `teensy_bridge_node`'s on_shutdown Stow takes about 8 s. The launch escalates SIGINT to SIGTERM after 5 s and no `sigterm_timeout` is set in the launch files, so the Stow can be cut off; the firmware deferred-stow is the only backstop.
- `TeensyLinkClient` has no single-owner interlock (no flock or fcntl anywhere in `teensy_link/`); a second process on the same UDP link is a standing bench hazard.
- If the owner trusts the firmware deferred-stow, file only the interlock half at low priority.

Source entries: [logbook/2026-07-12-cone-sync-udp-link-contention.md](logbook/2026-07-12-cone-sync-udp-link-contention.md)

## Options (2026-11-01)

1. Raise `sigterm_timeout` (about 12 s) in both launch files so the Stow completes; interlock deferred. Smallest change, reversible, no firmware.
2. Trust the firmware deferred-stow: drop the Stow-cutoff half, keep only the interlock half as a low-priority Item.
3. Both: the launch timeout now, and a `fcntl.flock` on a lock file keyed by the bridge host:port in `TeensyLinkClient` so a second process on the same link fails loudly.
4. Interlock only (option 3's second half) and leave the timeout alone, if the owner would rather the launch stays quick to kill.

Recommendation: 3, timeout first (one line per launch file; a cut-off Stow leaves the Platform wherever SIGTERM found it, which is the hazard the Stow exists to prevent), interlock as its own unit.

A question ticket carries this to the owner; the answer under its `## Answer` re-opens this Item.

## Progress

- 2026-11-01: Owner chose the split (TKT-19): link single-owner (no SO_REUSEADDR, LinkBusyError) and an 8 s SIGTERM window for the bridge node; the post-rejection teardown hang is filed separately.
