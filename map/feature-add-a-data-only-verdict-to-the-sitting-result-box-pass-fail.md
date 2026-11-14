---
id: FEAT-42
kind: feature
title: Add a data-only verdict to the Sitting result box (pass / fail / pending / data)
status: complete
created: '2026-11-05'
updated: '2026-11-14T12:01:36Z'
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

BUG-58's 2026-11-05 22:54 result was a pure report of observations with no pass criterion, and `pending` means the pass needs the logs, so the owner had no honest option.

## Wanted
- Add `data` to `mapkit.sitting.VERDICTS` (mapkit/sitting.py), `parse_sitting_results`, and the app's result box (jugglebot-map fork).
- `/sitting-debrief` treats `data` as neither merge-eligible nor failing.
- Update the verdict list in `docs/agents/map-writes.md` and the tests that pin VERDICTS.

## Progress

- 2026-11-14: Landed with close-out C9 on 2026-11-14: data and inconclusive are in mapkit.sitting.VERDICTS, the debrief derives them and merges only a confirmed pass, and the Sitting sheet records notes only so no verdict button exists to extend. The All Items page's own result box still offers pass, fail and pending.
