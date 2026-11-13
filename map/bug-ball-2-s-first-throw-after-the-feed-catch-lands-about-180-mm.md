---
id: BUG-63
kind: bug
title: Ball 2's first throw after the feed catch lands about 180 mm outward along the separation axis
status: inbox
created: '2026-11-03'
updated: '2026-11-13T01:10:12Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
radius: subsystem
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links:
- map/idea-jp-9-high-low-columns-ball-1-at-apex-1-05-m-ball-2-at-0-95-m.md
- map/idea-jp-10-turned-columns-the-columns-pair-at-fixed-angles-about.md
- map/bug-ball-2-learner-command-pinned-at-the-20-20-mm-clamp-corner.md
- logbook/2026-11-04-sitting-debrief.md
- logbook/2026-11-06-sitting-debrief-2.md
- logbook/2026-11-10-sitting-debrief.md
filed_by: sitting-debrief 2026-11-03
---

On 2026-11-03 Ball 2's first Throw after the feed catch landed about 180 mm outward along the separation axis in 5 of 11 high-low Attempts and in 5 of 7 turned-90-degree Attempts. When it happens the Attempt ends within about 3 s at 0/0 or 1/1.

## Sitting

- setup: level, armed, bb-fed
- why: the hypothesis is that a feed catch whose lateral aim was clamped seats the ball off-centre and the first Throw carries a lateral kick. A fresh plant_id aims the feed catch by the identity prior, so the clamp never engages: if the outward first throw disappears the clamped aim is implicated, if it persists the clamp is cleared. Either outcome is data.
- do:
  - Fly this right after the IDEA-10 high-low Attempts, same settings. Before the goal: `ros2 param set /skill_node plant_id bug63-fresh-<date>` (a name no memory file has; refused while a goal runs) and `ros2 param set /skill_node apex_m_ball2 0.90`.
  - GUI: Juggle panel `Columns - high-low`, Apex 1.00, Apex 2 0.90, Sep. 100, Dwell 0.27, Throws 21. One Attempt; a second only if the first ended before Ball 2's first Throw for an unrelated reason.
  - Afterwards restore the plant_id the IDEA-10 Attempts used (read it from their goal start line, `plant_id=...`) and `ros2 param set /skill_node apex_m_ball2 0.0`.
- pass: Ball 2's first Throw after the feed catch lands within 60 mm of its Site along the separation axis (the 2026-11-03 misses were about 180 mm outward: landings x -223 to -240 against aims near -50).
- record: the plant_id, Ball 2's first-throw landing (x, y) mm and its aim from the install line, the feed-catch aim line (clamped or not), how the Attempt ended, the bag path.
- minutes: 10

## Findings (Sitting 2026-11-03, launch.log lines as cited)

- High-low Attempts 5, 8, 10 and 11 (ends at L9881, L10944, L11442, L11610) ended 0/0 or 1/1 on that Throw, with first Ball 2 landings (-230,-17), (-223,-97), (-228,-117) and (-240,+16) mm against aims near x -50.
- Attempt 4 (L9713) is a 37 mm cousin: Ball 2 landed (-14,-17) against (-52,0).
- The along-axis error is about 180 mm every time; the perpendicular part varies from -117 to +16 mm.
- At 90 deg the Ball 2 misses landed 37-50 mm outward of the aim (y -89, -96, -93, -95, -104 against -53) while the catch aim was clamped to 10 mm outward (`AIM-LATERAL-CLAMPED`).
- Not timing (tracker landing time minus schedule -46..+17 ms, release error -8..+17 ms), not apex (normal apart from one 1.23 m), not a saturated launch (launch iq 22-36 A, speeds as scheduled).
- The logged feed-catch numbers do not separate good from bad: settle 133-276 ms and seat +0.12..+0.31 s in both.
- The re-aim refusals (INFEASIBLE x4, UNVERIFIED x1, LIMIT_JERK x1) sit on these misses but a 180 mm miss is outside any re-aim authority (20 mm lateral, 10 mm outward), so they are consequences.

## Hypothesis (not tested)

A feed catch whose lateral aim was clamped (`CATCH-AIM skill 1 ... y -20.0`) seats the ball off-centre in the Cup, so the first Throw off it carries a lateral kick: 0.2 m/s over 0.88 s is 180 mm, about 2.9 degrees. A collision of Ball 1 with Ball 2 is not the natural reading because Ball 2 goes away from Ball 1's column.

## Next

A bag probe (`tools/probes/`) of the ball's seat position in the Cup at the first Ball 2 release against the outcome. No change on the Throw path until it names a cause. Related: IDEA-10, IDEA-11, BUG-52.

## Sitting result (2026-11-13 12:10)

- verdict: pending
- notes: Ran about 5 attempts, none of which reached the 21 throws. The pattern seemed generally cleaner than for Step 14, but there was always at least one messy throw that ended the pattern. (sheet 2026-11-13-robot, step 15)

## Progress

- 2026-11-04: 2026-11-03 Sitting, one-ball oval: the clamped-feed-catch mechanism is REFUTED for the oval (first Throw after the feed catch was the BEST throw, caught 19/19, +47.8 mm raw; 7 clamped feed catches each followed by a normal Throw 1). The oval's long throws follow the CARRY (FEAT-32 mechanism).
  - Next: check whether Columns Ball 2's first Throw follows a transit whose deceleration ends close to release (fk.py restricted to those releases).
- 2026-11-06: 2026-11-06 endurance Sittings (30 plain Columns goals that threw): the first three Throws after the Feed all landed within 64 mm of the Site; one candidate only (14:55 goal 9, Feed aim clamped +92 to +30 mm in y, also a collision goal). The 180 mm outward first Throw was not reproduced on plain Columns.
- 2026-11-08: Not flown at the Sitting of 2026-11-07. As data: Ball 2's first Throw landed within 45 mm in all 32 Attempts; with the Feed aim clamped (n 25) +4.2 mm outward, unclamped (n 7) +9.6; no 180 mm case. No support for the clamped-aim reading here; the fresh-plant row is still unflown.
- 2026-11-10: Sitting of 2026-11-10: the first Catch after the Feed ended 7 of 11 two-ball fed Attempts. Ball 2's first-throw landing outward along the separation axis: high-low (sep 100) 157, 23 (after a 246 mm identity refusal) and 37 mm; straight Columns 196 mm; turned 90 deg 47, 26 and 14 mm.
