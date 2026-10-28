---
id: JB-84
kind: bug
title: Ball Butler over-CAN flash calls the retired Jugglebot-skills worktree
status: inbox
created: '2026-10-28'
updated: '2026-10-28T07:26:45Z'
owners: []
priority: medium
blocked_by: []
links: []
parent: ball-butler
filed_by: claude
---

The Ball Butler over-CAN flash hook and link check call `~/Desktop/Jugglebot-skills`, a worktree retired 2026-10-23 that no longer exists. The next over-CAN Ball Butler flash fails. The same class was fixed once by BallButler commit 587e732 (2026-10-10) and came back.

- `BallButler/ball_butler_main/platformio.ini:94` (`upload_command`) and line 44.
- `scripts/bb_link_check.py:25` and line 13.
- The fix is in the BallButler repo, not this one: point them at `~/Desktop/Jugglebot`.
- F3, F4, yaw-homing and the aim refit stay rows of the archived residue plan (plans/archived/skill-stack-dress-rehearsals-and-residue.md section 2).

Source entries: [logbook/2026-09-30-skill-stack-r5-sitting-1.md](logbook/2026-09-30-skill-stack-r5-sitting-1.md), [logbook/2026-10-13-skill-stack-r5-bb-throw-self-cancel-and-phantom-catch.md](logbook/2026-10-13-skill-stack-r5-bb-throw-self-cancel-and-phantom-catch.md)
