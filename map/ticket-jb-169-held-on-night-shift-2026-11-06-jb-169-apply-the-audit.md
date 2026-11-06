---
id: JB-180
kind: ticket
title: 'JB-169 held on night-shift/2026-11-06-JB-169: apply the audit''s 2 fixes and merge, also count the final Catch in the records, merge as is, or drop?'
status: complete
created: '2026-11-06'
updated: '2026-11-05T22:53:51Z'
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

JB-169 is built on night-shift/2026-11-06-JB-169 as you answered in JB-172: a Columns goal's final Catch on a Stop is booked unverifiable, not `MISSED_CATCH`, with no motion change. The audit left three findings open, so the merge gate held it.

## The open findings

1. **A false CAUGHT.** A SEATED read after the verdict turns the row CAUGHT and counted, while the end line still says "unverifiable". On the flown 2026-11-04 goal the Stop's seat (480.203) falls inside Ball 1's row window (ends 480.227), so this is the expected case, and that seat is not Ball 1's evidence.
2. **A wrong label.** A final Catch after an unrelated END (`LANE_REFUSED`, a mode change) is also booked "ended on a Stop", and its rows are kept. The old path dropped them.
3. **The records count.** The ledger reads N-1/N-1 for that goal, so the same run scores differently depending on the cup read. It matches "pass or fail nothing", but it is your call.

## Options

1. Apply 1 and 2, keep 3 as it is, and merge: a booked row stays unverifiable whatever the cup reads later, and the rule runs only when no other END has fired.
2. Apply 1 and 2 and count the final Catch in the records, so the ledger reads N/N.
3. Merge as is. Not advised: finding 1 defeats the point on the very goal that raised this.
4. Drop the branch.

Recommendation: 1.

## What each answer unblocks

- 1 or 2: the next night applies them, re-gates and merges.
- 3: the next night re-gates and merges.
- 4: JB-169 goes back to open.

## Answer

Option 2 is the desired outcome
