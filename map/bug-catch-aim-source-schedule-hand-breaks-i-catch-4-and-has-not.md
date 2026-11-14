---
id: BUG-106
kind: bug
title: 'Catch aim source schedule_hand breaks I-CATCH-4 and has not been flown in three months: delete or port'
status: complete
created: '2026-11-14'
updated: '2026-11-14T02:13:34Z'
owners: []
priority: low
effort: s
hardware: none
owner: decision
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-14-schedule-hand-aim-source-deleted.md
filed_by: claude
---

The Catch aim source `schedule_hand` has a re-send path that breaks invariant I-CATCH-4, and nothing shows it has been flown in three months. Found by the 2026-11-13 maintainer review; the usage search was done on 2026-11-14.

## What it is

`catch_aim_source` is a `skill_node` parameter with three values (`executor.py:350-363`). `tracker` is the default: the Tracker's Fit aims the Catch. `schedule` is open loop: the commanded Landing of that ball's previous Throw aims it. `schedule_hand` is `schedule` plus one re-aim of the committed Catch from the Hand's measured launch speed, read from `/hand_telemetry`, with no mocap.

## Findings

- `executor.py:7372` and `_resend_too_late` (`:7414`) fence the re-send with `skill.lead_s`. I-CATCH-4 says the fences use the module `LEAD_S` and model the snap; only `_resend_live_catch` was moved onto `_resend_splice_instant`. Re-aims are declined about 200 ms early and snapped ones are refused.
- The path is reachable: `skill_node.py:1314` declares the parameter and passes it to the executor at four call sites.
- Usage, 2026-08-14 to 2026-11-14: no `CATCH-AIM ... source=schedule_hand` line in `temp/logs/` or `~/.ros/log`, and no sheet, Runbook or logbook entry sets it. `schedule` was set deliberately on nine days, last on 2026-10-19. Rosbags and parameter events were not searched, so this is no evidence of use, not proof of none.

## Decision for the owner

Delete `schedule_hand` (about 200 lines: `executor.py:4613-4750`, `:7299-7416`, the early-command branch, `hand_launch.py` if nothing else reads it), or port its fences to I-CATCH-4. Deleting is the smaller change and removes a path the contract no longer describes.

## Progress

- 2026-11-14: Deleted by owner decision 2026-11-14; tracker and schedule unchanged, full suite 9797 passed
