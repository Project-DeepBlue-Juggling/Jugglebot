---
id: JB-173
kind: ticket
title: 'JB-163 held on night-shift/2026-11-05-JB-163: apply the audit''s 3 label fixes and merge, apply only the ring-full fix, merge as is, or drop?'
status: complete
created: '2026-11-05'
updated: '2026-11-04T21:23:17Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: gui
blocked_by: []
links: []
parent: bug-console-search-missing-some-entries
type: question
---

JB-163 is built on night-shift/2026-11-05-JB-163 as you answered in JB-164: a window line above the CONSOLE ("lines since HH:MM:SS (page subscribed), N dropped") and the FW_CHECK startup lines pinned against ring eviction. The full gate passed (`./run_tests.sh --full`, 2026-11-05: 8215 passed, EXIT=0), but the audit left three findings on the window label open, so the gate held it.

## Options
1. Apply all three and merge: count ring and pinned drops separately so "(ring full)" shows only when the ring evicted; add "startup lines pinned" to the label once a pinned line is older than the window; add the label and tie tests.
2. Apply only the first (the wrong "(ring full)" path, which needs over 50 FW_CHECK lines, about 25 bridge restarts with the page open) and merge.
3. Merge as is.
4. Drop the branch.

Recommendation: 1. It is about ten lines, and it fixes exactly the label this Item exists to make honest.

## What each answer unblocks
- 1 or 2: the next night applies the fixes, re-gates and merges.
- 3: the next night re-gates and merges.
- 4: JB-163 goes back to open.

## Answer

Agreed
