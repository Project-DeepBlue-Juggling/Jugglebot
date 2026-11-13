---
id: TKT-71
kind: ticket
title: 'BUG-92 Ball Butler frame phantoms: the static-cell rule on by default, the same rule off by default, a Ball Butler zone in config, or leave it?'
status: open
created: '2026-11-14'
updated: '2026-11-13T17:50:59Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: tracking
blocked_by: []
links: []
parent: bug-static-ball-butler-frame-markers-are-tracked-as-human-throw
type: question
---

With `detect_human_throws` on, static Ball Butler frame markers start human_throw tracks: 12 of 124 live tracks and 26 of 30 bench phantoms on 2026-11-13 started on fixed cells. The label filter cannot name them, because QTM labels the flying ball `Ball Butler - 1`.

## Options

1. **The proposed static-cell rule, on by default:** drop a candidate whose first sample is within 40 mm of a 10 mm cell occupied over 90 % of the last 30 to 120 s; a parameter turns it off.
2. **The same rule, off by default:** a Sitting A/Bs it.
3. **A Ball Butler zone in config** (x -730 to -988, z 1665 to 1861 mm): simpler, breaks when Ball Butler moves.
4. **Leave it:** phantoms only appear while `detect_human_throws` is on.

## What each answer unblocks

- 1, 2 or 3: the next night builds it on a branch (offline tests on synthetic markers) and holds it for a human-Throw Sitting window.
- 4: BUG-92 closes.

Recommendation: 1. On the launch 3 slice it removed 26 of 30 phantoms and 0 of 114 true Throws, with no label or zone list.
