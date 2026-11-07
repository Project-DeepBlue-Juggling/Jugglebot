---
id: JB-199
kind: ticket
title: 'JB-188 late second re-aim refusals: close as designed, decline them in the pre-solve fence, or read why the turned pair''s first ask is later?'
status: complete
created: '2026-11-07'
updated: '2026-11-06T22:53:36Z'
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

JB-188 is diagnosed: every one of the 29 INFEASIBLE refusals is a late SECOND re-aim of a Catch whose first re-aim was already accepted, inside 0.11 to 0.18 s of the landing. The Hand stays on the first aim as designed, and the Throws after a refusal were 22 of 22 caught. Full table in `## Findings (2026-11-07)` on JB-188.

## Why the turned pair shows it most

Its first ask arrives about 0.13 s later (median window 0.324 s against 0.459 s), so its second ask falls in that band: 15 of 17 refused. Why it is later is an untested hypothesis (Fit convergence on the turned geometry).

## Options

1. Close it as designed: a true infeasibility, harmless, no change.
2. Decline those asks in the pre-solve fence, so they spend no solve and no re-aim cap count. A small executor change, and the log line changes.
3. Look into why the turned pair's first ask is later: a read of the bag, no code. A later first ask costs Catch authority, whatever the refusals do.

Recommendation: 3 if you plan to fly the turned pair again, otherwise 1. Option 2 only tidies a count.

## What each answer unblocks

- 1: the next night closes JB-188.
- 2: an offline unit with a replay of one refused re-aim, which also names the binding row.
- 3: a Night Shift unit that reads the 15:53 bag and writes findings.

## Answer

Agreed. Investigate why the turned pair's first ask is later.
