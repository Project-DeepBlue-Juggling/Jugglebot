---
id: BUG-7
kind: bug
title: Platform Teensy truncates the persisted levelling offset to int16 milliradians
status: in_progress
created: '2026-10-24'
updated: '2026-11-09T22:24:44Z'
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

## Sitting

- setup: level
- why: a relaunch must restore the same levelling offset a fresh `level` flew, now that it persists in 0.1 mrad counts.
- do:
  - Note the offset the orchestrator logs after `level` ("Gravity offset published").
  - Relaunch without re-levelling.
  - Read `pose_offset_rad` from `/robot_state`.
- pass: each axis of the read-back within 0.0001 rad of the logged offset.
- record: both offsets, in rad.
- minutes: 3

## Progress

- 2026-11-10: Fixed and flashed 2026-11-10: can-bridge FW 43, Platform FW 10, PROTOCOL 14. The offset persists in rounded 0.1 mrad counts. The robot read-back in the Sitting section closes it.
