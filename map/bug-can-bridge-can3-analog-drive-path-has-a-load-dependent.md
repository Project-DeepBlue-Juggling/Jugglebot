---
id: JB-83
kind: bug
title: can-bridge CAN3 analog drive path has a load-dependent hardware fault (the Jugglebot loom stays on CAN2)
status: inbox
created: '2026-10-28'
updated: '2026-10-28T07:26:42Z'
owners: []
priority: low
blocked_by: []
links: []
filed_by: claude
---

The can-bridge CAN3 analog drive path has a load-dependent hardware fault that was never repaired. The Jugglebot loom stays on CAN2; the firmware still runs the bus swap until the CAN3 path is repaired.

- CAN3 now carries the Cone and, since 2026-10-25, the wave camera, so the faulty bus gained load.
- Unchecked whether JB-52 (CLIP_MISSING on first start) relates.
- Never plug the Jugglebot loom back into CAN3 until repaired.

Source entries: [logbook/2026-07-31-can3-drive-path-fault-jugglebot-to-can2.md](logbook/2026-07-31-can3-drive-path-fault-jugglebot-to-can2.md), [logbook/2026-06-30-canbridge-phase6-reboot-latch.md](logbook/2026-06-30-canbridge-phase6-reboot-latch.md), [logbook/2026-07-05-canhub-marginal-can3-diagnosis.md](logbook/2026-07-05-canhub-marginal-can3-diagnosis.md)
