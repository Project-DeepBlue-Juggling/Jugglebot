---
id: BUG-7
kind: bug
title: Platform Teensy truncates the persisted levelling offset to int16 milliradians
status: inbox
created: '2026-10-24'
updated: '2026-11-01T06:48:15Z'
owners: []
priority: medium
effort: s
hardware: robot
owner: hands
radius: normative
diagnosis: confirmed
area: platform-teensy
blocked_by: []
links:
- plans/archived/levelling-frame-contract.md
filed_by: claude
---

Measured 0.0264 deg, worst case 0.0573 deg per axis, against the +-0.05 deg band; LVL-3 is blind to it. Address it at the next levelling touch; it has no other owner.
