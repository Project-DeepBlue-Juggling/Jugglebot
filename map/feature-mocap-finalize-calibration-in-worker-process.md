---
id: FEAT-5
kind: feature
title: mocap _finalize_calibration in a worker process
status: inbox
created: '2026-10-24'
updated: '2026-10-24T13:05:00Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: subsystem
area: tracking
blocked_by: []
links:
- plans/archived/skill-stack-dress-rehearsals-and-residue.md
filed_by: claude
night_skip: waiting-trigger
night_skip_on: '2026-11-15'
night_skip_note: Its trigger (a calibration that stalls the node) has not fired.
---

Each calibration sweep stalls mocap for 10-13 s because `_finalize_calibration` runs in the node's interpreter, so never calibrate during a goal. Moving it to a worker process removes the stall (bb-self-cancel Discussion 5). Trigger: any calibration needed near a goal.
