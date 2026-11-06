---
id: JB-187
kind: feature
title: 'P2 characterisation Sitting: Apex sweep, off-centre Sites, ball swap and a turned pair, to pin the Learner''s plant model'
status: inbox
created: '2026-11-06'
updated: '2026-11-06T03:35:07Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
radius: subsystem
area: learner
blocked_by: []
links:
- plans/active/learner-generalisation.md
- tests/hardware/session_learner_characterisation.md
parent: learner-generalisation
filed_by: claude
---

P2 of plan `learner-generalisation.md`: the Sitting that pins the structure of the Learner's plant model. Every row so far was flown on the x axis with two balls, so three questions cannot be answered from the data. Runbook: `tests/hardware/session_learner_characterisation.md`.

## Evidence
- The plan's identifiability table (§ 2.7): aim angle against fixed offset, a radial centre pull, and a frame-fixed against Pattern-fixed lateral offset are not identifiable from the corpus; the per-ball offset is confounded with the Site.
- 2026-11-06, offline `nominate.nominate` at 600 / 8000 / 200000, Hand 4500: the one-ball rungs nominate at margin 1.000 with no Leg motion (Apex 0.60 to 1.20, Sites 60 mm off centre on x and y); the turned pair at Sep. 120 asks the Legs for 264 mm/s and 3011 mm/s^2, inside what flew clean that day (361 mm/s, 4672 mm/s^2, 156 mm transits).

## Wanted
- The three decisions of the Runbook's table, made by its pre-set rules at the debrief; the plant model's structure pinned in the plan's § 2.2; the frame for a per-beat rotation of the Pattern chosen.

## Sitting
- setup: level, armed, bb-fed
- why: pins the plant model's structure (fixed offset or aim angle, radial centre pull, per-ball offset) and the frame for turning a Pattern, which the rest of the Learner generalisation Pathway waits on.
- do:
  - Follow `tests/hardware/session_learner_characterisation.md`, steps 1 to 17. Steps 1 and 2 are before power and move nothing; step 14 is a nomination that needs Ball Butler up.
  - Blocks A to C are one-ball `Self-toss` goals (Throws 20) with the ball placed in the cup by hand: five Apexes at the centre Site, four Sites 60 mm off centre, then two more balls. The Site is set with `ros2 param set /skill_node site_x_mm` and `site_y_mm`.
  - Block D is `Columns — turned` at Angle 90, Sep. 120, Apex 1.00, then plain `Columns` at Sep. 120. Ball Butler is needed only here.
  - Apex never above 1.05. Stop on any `ADMISSIBILITY` refusal, a Hand current sample at 55.0 A, or anything wrong at the base joints.
- pass: every one-ball step has 20 Throws flown (one Attempt, or two shorter ones), steps 15 and 16 have about 20 caught Throws per Site, and no Attempt ended on an `ADMISSIBILITY` refusal. Type the result as pending: the three decisions are computed at the debrief.
- record: the launch log and bag paths, the clock time and step number of each Start, which ball (A, B or C) flew in each Attempt, any Attempt repeated and why, anything seen at the base joints.
- minutes: 40
