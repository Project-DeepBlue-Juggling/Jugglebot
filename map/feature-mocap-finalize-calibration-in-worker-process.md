---
id: JB-30
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
---

Each calibration sweep stalls mocap for 10-13 s because `_finalize_calibration` runs in the node's interpreter, so never calibrate during a goal. Moving it to a worker process removes the stall (bb-self-cancel Discussion 5). Trigger: any calibration needed near a goal.
