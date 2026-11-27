---
id: TKT-103
kind: ticket
title: 'BUG-75: with the re-aim directions logged, BUMP second re-aims, send none, raise the launch bound, or gate them by size?'
status: open
created: '2026-11-28'
updated: '2026-11-27T15:52:26Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-catch-re-aim-refused-infeasible-on-8-4-of-throws-at-the-p2
type: question
---

BUG-75: your 2026-11-26 decision was to read the logged re-aim direction at the next Sitting, then choose the second re-aim policy. The 2026-11-27 Sitting logged it (638 lines): on the oval a second re-aim reversed the first in 54 of 75 Catches, taking back about 22 % of it; Columns 21 of 37. Second re-aims partly undo noise in the first rather than chase fresh noise.

Earlier: 57 of 72 turned-pair refusals were second re-aims, 0.1 to 0.3 s after an accepted first; flown second re-aims are small (median 12 mm, p90 about 20 mm).

## Options

1. Enable the BUMP for second re-aims: the replay accepts 41 to 57 % diagonal and 79 to 83 % axis, short of the 94.7 % axis bar.
2. Send no second re-aim: removes most of the refusals; the cost is the 22 % take-back, which mostly undoes noise.
3. Raise the BUMP-LAUNCH bound (the 80 mm/s what-if took one 90 deg diagonal group from 56.5 to 81.5 %; one Columns miss on 2026-11-27 was declined at 61.4 against 50 mm/s).
4. A second re-aim only when it is larger than about 20 mm (the flown p90) and does not reverse the first.

## What each answer unblocks

Any of 1 to 4: the next night builds it on a branch with its replay numbers and a Sitting section, and holds it for a Sitting (it changes flown re-aims). 3 also touches a safety bound, so it is held for your Session rather than a Sitting alone.

Recommendation: 2, because the logged directions show second re-aims mostly take back noise, and dropping them removes most refusals without loosening any bound.
