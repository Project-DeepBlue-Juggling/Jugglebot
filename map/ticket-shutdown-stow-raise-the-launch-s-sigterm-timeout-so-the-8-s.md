---
id: TKT-19
kind: ticket
title: 'Shutdown Stow: raise the launch''s sigterm_timeout so the 8 s Stow completes, or trust the firmware deferred-stow and keep only the UDP single-owner interlock?'
status: complete
created: '2026-11-01'
updated: '2026-11-01T00:35:15Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: can-bridge
blocked_by: []
links: []
parent: bug-teensy-bridge-node-s-shutdown-stow-can-be-cut-off-by-the
type: question
---

BUG-26 names two halves of one hazard: `teensy_bridge_node`'s on_shutdown Stow takes about 8 s while the launch escalates SIGINT to SIGTERM after 5 s (no `sigterm_timeout` in either launch file), and `teensy_link/` has no single-owner interlock on the UDP link. The body says the firmware deferred-stow is the only backstop and asks whether the owner trusts it. Night Shift 2026-11-01 drafted the options below into the Item's `## Options (2026-11-01)` section and cannot pick one without the owner.

## Options

1. Raise `sigterm_timeout` (about 12 s) in both launch files so the Stow completes; interlock deferred. Smallest change, reversible, no firmware.
2. Trust the firmware deferred-stow: drop the Stow-cutoff half, keep only the interlock half as a low-priority Item.
3. Both: the launch timeout now, and a `fcntl.flock` on a lock file keyed by the bridge host:port in `TeensyLinkClient` so a second process on the same link fails loudly.
4. Interlock only (option 3's second half) and leave the timeout alone, if the owner would rather the launch stays quick to kill.

Recommendation: 3, timeout first (one line per launch file; a cut-off Stow leaves the Platform wherever SIGTERM found it, which is the hazard the Stow exists to prevent), interlock as its own unit.

## What each answer unblocks

- 1 or 3: a Night Shift unit on the launch files (hardware none, radius subsystem) that the gate can merge.
- 2 or 4: the Item is re-rated (radius local, priority low) and the interlock becomes the only slice.

## Answer

I lean Option 1, but partially because I don't full understand Option 3. I'd like to discuss this in more depth
