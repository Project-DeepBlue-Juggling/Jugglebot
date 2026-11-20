---
id: BUG-124
kind: bug
title: Hand Homing can set its zero a revolution high with nothing checking the stroke
status: inbox
created: '2026-11-20'
updated: '2026-11-20T05:22:15Z'
owners: []
priority: medium
effort: s
hardware: none
owner: decision
radius: subsystem
diagnosis: hypothesis
area: hand
blocked_by: []
links:
- logbook/2026-11-20-sitting-debrief.md
filed_by: claude
night_skip: waiting-trigger
night_skip_on: '2026-11-20'
night_skip_note: 'Owner 2026-11-20: no fix; reopen only if a Homing sets the zero wrong again.'
---

Sitting of 2026-11-20, launch 1. The 12:49:37 Homing set the Hand's zero about 1.0 rev (33 mm) above the bottom stop: it travelled 3.0 rev and tripped at a peak of 9.1 A; the 13:07:19 re-home found the stop 1.0-1.1 rev lower. In Columns Attempts 1-6 the Hand pressed its top stop at 9.78-9.80 rev against a 9.94-10.30 rev command at every Feed Catch (24-39 A, zero speed, under 0.2 s each) until the operator noticed and re-homed;

Attempts 7-10 reach 10.17-10.30 rev with no press. Landing errors and completions did not shift, so the misses that day are not explained by it.

Why the first Homing tripped early is unknown (obstruction, a Ball under the Cup, friction); the trapped-Ball scan sees no marker at 12:49:36 (cannot see).

Owner options: (a) after Homing, a gentle probe up to the top stop that checks the stroke is 10.70 rev within a tolerance; (b) compare the Homing travel end with the previous home; (c) an operator check only.

## Progress

- 2026-11-20: Owner, 2026-11-20: no fix for now; keep this record in case a false Homing happens again.
