---
id: JB-177
kind: feature
title: Add a data-only verdict to the Sitting result box (pass / fail / pending / data)
status: inbox
created: '2026-11-05'
updated: '2026-11-05T12:31:18Z'
owners: []
priority: low
effort: s
hardware: none
owner: none
radius: subsystem
area: map-app
blocked_by: []
links: []
parent: map-improvements
filed_by: claude
---

JB-149's 2026-11-05 22:54 result was a pure report of observations with no pass criterion, and `pending` means the pass needs the logs, so the owner had no honest option.

## Wanted
- Add `data` to `mapkit.sitting.VERDICTS` (mapkit/sitting.py), `parse_sitting_results`, and the app's result box (jugglebot-map fork).
- `/sitting-debrief` treats `data` as neither merge-eligible nor failing.
- Update the verdict list in `docs/agents/map-writes.md` and the tests that pin VERDICTS.
