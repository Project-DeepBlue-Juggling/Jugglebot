---
id: JB-210
kind: ticket
title: 'JB-184 reopened by two more Leg separations on 2026-11-07: keep no bound and retire Dwell 0.30 and 0.32, bench-test a joint first, bound planned Leg acceleration, or hold the joints mechanically?'
status: complete
created: '2026-11-08'
updated: '2026-11-08T00:20:40Z'
owners: []
priority: high
effort: xs
hardware: none
owner: decision
radius: local
area: legs
blocked_by: []
links: []
parent: bug-legs-1-and-2-left-their-magnetic-base-joints-on-columns-at
type: question
---

On 2026-11-06 you set no bound on transit aggressiveness, and said a second separation reopens that. It has happened twice: on 2026-11-07, Columns at Apex 0.93, Sep. 125 pulled Legs 1 and 2 out at Dwell 0.32 and again at Dwell 0.30. JB-192's row had called those goals inside what has flown.

## What the log shows

| Attempt | Planned Leg peak | Joints |
|---|---|---|
| 2026-11-06, Sep. 170, Dwell 0.27 | 486 mm/s, 6296 mm/s^2 | let go |
| 2026-11-07, Dwell 0.30 | 406 mm/s, 5968 mm/s^2 | let go at +5.4 s |
| 2026-11-07, Dwell 0.32 | 404 mm/s, 5872 mm/s^2 | let go at +5.96 s |
| 2026-11-06, Sep. 120 | 371 mm/s, 5488 mm/s^2 | held |
| 2026-11-07, Dwell 0.27 | 384 mm/s, 5120 mm/s^2 | held |

Both new separations began about 0.1 s after the third transit ended near rest, with shorter transits than Sep. 170. So a peak may not be the right measure. Full tables are on JB-184.

## Options

1. Keep no bound, and take Dwell 0.30 and 0.32 off the Runbook. Dwell 0.27 held.
2. Bench pull-off test of one magnetic joint before choosing anything.
3. A bound on planned Leg acceleration at goal accept, inside the 5488 to 5872 gap: 7 % wide, on three cases, cause not shown. It would refuse both goals.
4. Hold the joints mechanically.

Recommendation: 1 and 2 together, with no limit value yet.

## What each answer unblocks

- 1: the next night retires those Dwells from JB-192's row and writes the bench row if you also chose 2.
- 2: the next night writes the bench row.
- 3: a safety-bearing plan for your Session, with the value you choose.
- 4: a hardware step for you.

## Answer

I'm attempting Option 4 now, so we will see if this problem persists once I've updated the joints. Fingers crossed!
