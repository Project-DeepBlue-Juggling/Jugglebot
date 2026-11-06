---
id: JB-183
kind: feature
title: 'First Sitting on the merged learner-generalisation stack: live admissibility check, nomination and the first learned Throws'
status: complete
created: '2026-11-06'
updated: '2026-11-06T03:04:55Z'
owners: []
priority: high
effort: s
hardware: robot
owner: hands
radius: subsystem
area: learner
blocked_by: []
links:
- plans/active/learner-generalisation.md
- logbook/2026-11-06-sitting-debrief.md
parent: learner-generalisation
filed_by: claude
---

The no-motion phases of plan `learner-generalisation.md` (P3a, P3b, P3c, P1, P4) were merged to main on 2026-11-06 and have never run on the powered robot. Every other row of the Runbook now flies on this stack, so a fault here confounds all of them.

## Evidence
- The full tier and the sim gate pass on the merge (logbook entries `2026-11-06-learner-generalisation-p3a` to `-p4`). Nothing was flown, and `skills/check` and `skills/nominate` were never rehearsed on the loaded box with the new code.
- Replay over the 3,276 committed rows: the first Throw at a Site with no rows misses its Apex by 12.9 mm (mean) against 74.5 mm for the identity prior.
- The memory holds rows at Site x of 50, 60, 75 and 125 mm either side of centre, so Columns at Sep. 170 (Sites at 85 mm) is a pair of Sites it has never seen.
- Offline nomination at 600 / 8000 / 200000, Hand 4500 (2026-11-06, `nominate.nominate`): Columns 1.00 / 0.27 is viable at Sep. 100 (margin 0.33) and Sep. 170 (0.21). Sep. 200 passes at 0.08 only and Sep. 250 is refused, so the row stays at 170.
- Migrated on 2026-11-06: `temp/learn/jugglebot/memory_v2.csv` holds 3365 rows and the Learner accepts 3334 of them (K 1.077).

## Wanted
- P3a and P3b of the plan closed (their status is in progress for exactly this rehearsal), and its hardware tests T-H2 (a first learned Throw at a new Site) and T-H3 (Columns re-flown on the new stack) on record.

## Sitting
- setup: level, armed, bb-fed
- first: every other row flies on the merged stack, so no other result is readable until this row has passed
- why: first powered run of the live admissibility check, goal acceptance by nomination and the generalising Learner; it closes P3a and P3b and flies T-H2 and T-H3.
- do:
  - Before power, launch up with the bag recording and the GUI open: `ros2 service call /skills/check std_srvs/srv/Trigger`, then `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns,apex_m=1.00,separation_mm=100,dwell_s=0.27,num_cycles=21'}"`. Nothing moves. The nominate reply must read viable.
  - Armed and level. GUI: Juggle panel `Columns`, Apex 1.00, Sep. 100, Dwell 0.27, Throws 21. One Attempt. At Start the launch log prints `learner: ...; rows N (accepted, rejected, unusable)`: N must be 3365 or more, which shows the migrated memory loaded.
  - New Sites. GUI: `Columns`, Apex 1.00, Sep. 170, Dwell 0.27, Throws 21. One Attempt. If Start is refused, the message names the nearest viable configuration: fly that instead and write it down.
  - If an Attempt ends on an `ADMISSIBILITY` refusal or a refused goal names `memory_v2.csv`, stop this row and note the line; do not fly the other rows on a guess.
- pass: both service calls answer and nominate reads viable; the Sep. 100 Attempt reads `COMPLETED (20/20 caught)` with no Attempt ended by an `ADMISSIBILITY` refusal (end `NO_ADMISSIBLE_COMMAND`); at Sep. 170 each ball's first Throw has a Tracker Apex within 40 mm of the commanded Apex (the identity prior missed by about 75 mm).
- record: the launch log and bag paths, N from the `learner:` line, the nominate reply and its time, the count of `ADMISSIBILITY-EARLY` lines ending `the early resolution is sent` against `re-checked at dispatch`, each ball's first five Tracker Apexes at Sep. 170, caught of thrown per Attempt.
- minutes: 25

## Sitting result (2026-11-06 13:33)

- verdict: pass
- notes (the owner's box, split into bullets):
  - Great result!
  - Row 2 ended prematurely a few times due to ball mid-air collisions (unsurprising given the narrow separation gap).
  - Row 3 was terminated with an E-stop because the aggressive lateral movements ripped Legs 1 and 2 out of their (magnetic) base joints.
  - Thankfully no actual damage was incurred.
  - In the same Sitting, after fixing the legs and re-levelling, I tried sep. 125 and 135, both passing with the only drops being mid-air collisions or otherwise messy throws.
  - No attempt ended with a software hangup.

## Progress

- 2026-11-06: Passed at the Sitting of 2026-11-06 (owner): the merged stack caused no Attempt end; 100 of 100 early admissibility checks reused, no refusal; first Throws at new Sites within 15 mm of the commanded Apex (Sep. 135) and 5 mm (Sep. 170, 2 Throws). The Sep. 100 clause (20/20) was not met: its Attempts ended on ball collisions. Sep. 170 separated Legs 1 and 2 from their base joints.
