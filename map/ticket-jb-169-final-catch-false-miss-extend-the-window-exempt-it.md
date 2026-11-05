---
id: JB-172
kind: ticket
title: 'JB-169 final-Catch false miss: extend the window, exempt it, judge it by the Tracker, book it unverifiable, or test the cup sensor first?'
status: complete
created: '2026-11-05'
updated: '2026-11-04T21:20:22Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-full-length-columns-goal-ends-missed-catch-on-its-final
type: question
---

JB-169: a full-length Columns goal ends MISSED_CATCH on its final Catch. The Night Shift traced why the final CATCH is judged at all: no exemption covers it, and the cup read EMPTY until the Stop seated, 0.17 s after the window closed. Whether Ball 1 was really in the cup is not settled by the bag (Findings (2026-11-05) on JB-169).

## Options
1. Extend the final CATCH's window past the Stop's landing so the Stop's SEATED read vetoes the miss. This also hides a real final miss, because the Stop seats in an empty cup too.
2. Exempt the final CATCH of a goal that ends in a Stop, as the carried-Throw case already is. The same loss of detection, but explicit and narrow.
3. Judge that one Catch by the Tracker (last Fit inside the cup radius, ball lost at cup height) instead of the cup alone. Most faithful to the physics; needs a new rule and thresholds.
4. Book the end as "ended on a Stop, final Catch unverifiable": keep the OUTCOME rows and pass or fail nothing on that Catch.
5. First settle the sensor question at a Sitting: rest one ball off-centre in the cup and read the cup state.

Recommendation: 4 now (an honest record, no lost detection), and 5 when a Sitting has a spare minute.

## What each answer unblocks
- 1, 2 or 4: a night unit in `executor.py` with a test that ends a full-length Columns goal on a Stop.
- 3: a larger unit that needs thresholds from you.
- 5: a Sitting row.

## Answer

4 now. The real final solution is 3, but we don't need to go to that much effort right now. 4 is a good temporary solution
