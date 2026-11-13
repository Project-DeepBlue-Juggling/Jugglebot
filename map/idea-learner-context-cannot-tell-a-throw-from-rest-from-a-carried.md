---
id: IDEA-18
kind: idea
title: Learner context cannot tell a Throw from rest from a carried Throw
status: inbox
created: '2026-11-04'
updated: '2026-11-13T05:58:39Z'
owners: []
priority: low
effort: m
hardware: robot
owner: none
radius: subsystem
area: learner
blocked_by: []
links:
- logbook/2026-11-04-sitting-debrief.md
- logbook/2026-11-13-sitting-debrief.md
filed_by: claude
---

Filed by the 2026-11-04 debrief (2026-11-03 Sitting, one-ball oval). The oval's context x = [-0.075, 0, ...] is the same for Throw 1 (from rest, +48 mm raw long) and carried Throws 2+ (+76 mm), so the learner's y_hat is biased low on carried throws (+32..+52 vs ~+45 measured). Immaterial while the command is pinned at its authority; matters once the authority or the carry fix frees it. Option: a carried/from-rest flag or the seat velocity in the context.

## Feasibility (2026-11-05)

- What it takes: the learner state `x` is 4-D today (release-site xy, measured seat offset xy; `executor.py` ~L2560 and `_seat_offset_m`). The oval's carried Throws have seat offset 0 like a Throw from rest, so the two share one neighbourhood.
- Add a fifth component (a carried flag, or the seat velocity at release) with its own bandwidth in `LearnerConfig`, and migrate the Memory: old rows need a value, and a wrong default (0 = from rest) would mislabel every carried row already flown.
- Effort m (state vector, Memory schema migration, learner tests). Hardware robot: only flown Throws show whether `y_hat` stops reading low on carried Throws. Owner none.
- Risk: a new state dimension splits existing support, so the first Sitting after it starts colder (fewer than `k_min` neighbours means the identity prior) until carried rows accumulate.
- Not built tonight: rated m and robot.

## Progress

- 2026-11-13: Sitting of 2026-11-13: the split holds on a second plant at Dwell 0.40: landing minus command +27 mm from rest against +71 +/- 18 mm carried (launch 1: +32 vs +57; +27 vs +111 at 0.35). One pair at the identical command: +23 vs +141 mm. Priority left as filed.
