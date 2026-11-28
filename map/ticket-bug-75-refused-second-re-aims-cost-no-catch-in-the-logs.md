---
id: TKT-110
kind: ticket
title: 'BUG-75: refused second re-aims cost no Catch in the logs; close it as accepted, decline them before the solve, or measure the wasted solve first?'
status: open
created: '2026-11-29'
updated: '2026-11-28T16:26:10Z'
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

Your TKT-103 answer asked whether a refused second re-aim has a tangible downside. Keeping the system as it is was your default if it has none. The logs say it has no measured downside on Catches. Each refusal costs one wasted solve on the worker and one log line.

## What the logs show

- Every INFEASIBLE refusal (29 of 29 at the P2 Sitting, 57 of 72 on the turned pair later) is a second re-aim of a Catch whose first re-aim was accepted. The Hand stays on the first re-aim.
- Throws after a refusal: 22 of 22 caught, landing radius median 15.3 mm. Other Throws: 62 of 62 caught, median 19.5 mm (2026-11-07 findings; attribution by log order, so indicative).
- The cost is a QP solve that cannot succeed, started 0.11 to 0.18 s before landing. Its CPU on the busy worker near a Catch was never measured.
- Your point stands: a second re-aim that is accepted still takes back about 22 % of the first's noise, so dropping all of them is not free.

## Options

1. Close BUG-75 as accepted: keep the system as it is, the refusals are expected and harmless.
2. Keep second re-aims, but decline the ones that cannot succeed in the pre-solve fence (splice to landing under about 0.18 s). Same Catches, no wasted solve, no refusal line. The next night builds it with a replay of the logged asks and holds it for a Sitting.
3. Leave it open until a Sitting measures the refused solve's CPU on the worker.

## What each answer unblocks

1: the next night closes the Item with this evidence. 2: a branch held for a Sitting. 3: the Item waits on a Sitting with a `## Sitting` section the night writes.

Recommendation: 1, because no Catch has been lost to a refusal. Option 2 is worth building only if the worker's CPU near a Catch becomes the bottleneck (FEAT-31, BUG-149).
