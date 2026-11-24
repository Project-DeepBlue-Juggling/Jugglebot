---
id: IDEA-29
kind: idea
title: 'Classify the 5x jerk intervals in the oval: real acceleration steps at re-aim splices or a dropped-echo artefact'
status: inbox
created: '2026-11-24'
updated: '2026-11-24T11:28:54Z'
owners: []
priority: medium
effort: s
hardware: none
area: skill-stack
blocked_by: []
links: []
filed_by: claude
---

About 15 % of the intervals in a one-ball oval (42 of 273, 2026-11-20) show roughly 5 times the nominal jerk in `/leg_setpoint_echo`. Either they are real acceleration steps at re-aim Splices, or the echo dropped a message. Classify them. Found in the PWAY-2 review, `logbook/2026-11-24-learner-efficacy-review-and-scatter-decomposition.md`.

## Method

- Decode `/leg_cmd_executed` from the same bag (a bag decode, no hardware needed) and compare the same intervals.
- If the executed commands show the step, it is a real acceleration step: find which Splice and why.
- If they do not, it is an echo artefact and nothing is wrong with the motion.

## Context

- Oval: Platform 2.2 to 2.8 m/s squared; Hand acceleration p90 at 73 to 84 % of its 4500 rev/s squared limit.
- Not a Learner matter: landing offset explains 10 % or less of peak acceleration or jerk.
- Four Leg separations (BUG-72) followed executed Leg commands of 8,974 to 14,450 mm/s squared, so roughness is worth understanding.
- The sample is small (one oval run).
