---
id: TKT-55
kind: ticket
title: 'FEAT-47 drops counter: live only, live plus a base figure from the launch logs, or live plus the full history, and which edge cases count as a drop?'
status: complete
created: '2026-11-09'
updated: '2026-11-08T21:40:39Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: gui
blocked_by: []
links: []
parent: feature-add-total-drops-to-date-counter
type: question
---

The live count looks easy: the records ledger already has Throws and Catches for each Attempt. The body's definition still leaves cases only you can settle, and the backfill is large (about 1200 launch logs and 570 bags since January, in several log formats).

## Questions

- **An Attempt that ends with a ball in the air** (a Fault, a refused Segment, a Stop): does that ball count as a drop? Recommendation: yes if it was thrown and not caught, and the Stop Throw itself never counts.
- **A Ball Butler Feed that misses the Cup:** yes by your definition. Recommendation: count it.
- **A final Catch booked unverifiable:** Recommendation: not a drop, matching the ledger.

## Options for the backfill

1. Live counter only, from the first Attempt after it lands. No backfill.
2. Live counter plus a one-off base figure from Attempt end lines in the launch logs. Probably reaches back only as far as those lines exist (to be checked).
3. Live counter plus the full history, reading older logs and bags with agents. The most complete and by far the most tokens.

Recommendation: 2. It gets a defensible number without a large agent bill, and the page can state the date it counts from.

## What each answer unblocks

- 1: the next night builds the counter on a branch.
- 2: the next night builds it and runs the count.
- 3: a day Session plans the backfill.

## Answer

Answering the questions: the first two are drops. A final Catch booked unverifiable is not a drop.

On the 'Options': I want option 3. I want as complete a record as possible. We can use Sonnet agents for this, which will be cheap.
