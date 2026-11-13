---
id: BUG-7
kind: bug
title: Platform Teensy truncates the persisted levelling offset to int16 milliradians
status: complete
created: '2026-10-24'
updated: '2026-11-10T08:13:13Z'
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
- logbook/2026-11-10-levelling-offset-0p1-mrad-counts.md
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
- 2026-11-10: Robot check passed 2026-11-10: the 19:01 level logged (-0.0018433, +0.0057132) rad; relaunches at 19:07 and 19:09 restored (-0.0018, +0.0057), errors 0.043 and 0.013 mrad against the 0.1 mrad criterion (the old truncation would have been 0.84 and 0.71 mrad). PLATFORM_FW_CHECK v10 and BRIDGE_FW_CHECK v43 OK, no faults.
