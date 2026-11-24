---
id: IDEA-29
kind: idea
title: 'Classify the 5x jerk intervals in the oval: real acceleration steps at re-aim splices or a dropped-echo artefact'
status: inbox
created: '2026-11-24'
updated: '2026-11-24T16:53:19Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
area: skill-stack
blocked_by: []
links: []
filed_by: claude
night_skip: low-priority
night_skip_on: '2026-11-25'
night_skip_note: 'Classified 2026-11-25: mostly real per-cycle Segment-boundary steps, 15-18 % dropped-knot echo artefacts; naming the Splice is a launch-log read.'
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

## Feasibility (2026-11-25)

Night Shift 2026-11-25 classified the high-jerk intervals offline from two 2026-11-20 oval bags. The verdict is MIXED, mostly REAL. The 42 of 273 count was not reproduced, because the study's script is not in the repo; the definition below is the night's own.

- **Method.** The `oval_carry_bag_metrics.py` recipe: the third difference of `/leg_setpoint_echo` at 40 Hz, abs-max over the six Legs, high = over 5x the median active jerk. `/leg_cmd_executed` was put on the echo knots and run through the same stencil.
- **Counts.** Bag 2026-11-20_22-53-34: 60 clusters, 49 REAL, 11 ARTEFACT. Bag 2026-11-20_13-48-32: 198 clusters, 168 REAL, 30 ARTEFACT.
- **REAL (82 to 85 % of clusters).** The executed commands show the same 5 to 10x step. It comes once per Throw cycle, and 98 to 99 % of them fall within 0.3 s of a Throw or Landing announcement: a real acceleration step at the cycle's Segment boundary.
- **ARTEFACT (15 to 18 %).** An isolated 1:2:1 spike, the signature of one dropped knot. Each follows a 47 to 65 ms message gap and a catch-up. The executed commands show only 0.06 to 0.4x of it: an Emit-gap echo artefact.
- **Not done.** The bag carries no Splice markers, so which Splice makes the per-cycle step is open. The executed peaks were not compared in mm/s squared with BUG-72's separations.
- **Next.** A launch-log read for the Splice that fires 0.04 to 0.12 s from each Throw or Landing. Effort s, hardware none, offline.

Findings file: `temp/reports/night-shift/findings-IDEA-29.md` in the Night Shift clone (gitignored).
