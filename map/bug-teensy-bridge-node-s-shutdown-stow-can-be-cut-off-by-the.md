---
id: JB-82
kind: bug
title: teensy_bridge_node's shutdown Stow can be cut off by the launch's 5 s SIGTERM; teensy_link has no single-owner interlock
status: inbox
created: '2026-10-28'
updated: '2026-10-28T07:26:31Z'
owners: []
priority: medium
blocked_by: []
links: []
filed_by: claude
---

Two halves of one hazard: who owns the bridge link while the node shuts down.

- `teensy_bridge_node`'s on_shutdown Stow takes about 8 s. The launch escalates SIGINT to SIGTERM after 5 s and no `sigterm_timeout` is set in the launch files, so the Stow can be cut off; the firmware deferred-stow is the only backstop.
- `TeensyLinkClient` has no single-owner interlock (no flock or fcntl anywhere in `teensy_link/`); a second process on the same UDP link is a standing bench hazard.
- If the owner trusts the firmware deferred-stow, file only the interlock half at low priority.

Source entries: [logbook/2026-07-12-cone-sync-udp-link-contention.md](logbook/2026-07-12-cone-sync-udp-link-contention.md)
