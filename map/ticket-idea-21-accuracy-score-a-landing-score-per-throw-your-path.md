---
id: TKT-60
kind: ticket
title: 'IDEA-21 accuracy score: a landing score per Throw, your path score over the flight, or both, and do Feeds count?'
status: open
created: '2026-11-10'
updated: '2026-11-09T17:52:35Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: gui
blocked_by: []
links: []
parent: idea-create-and-display-an-accuracy-tidyness-score
type: question
---

IDEA-21 asks for a score of how closely a Pattern matches its target. It is now under the new PWAY-10 (GUI Improvements), whose Destination is a draft for you to replace. Its Feasibility (2026-11-10) finds the per-Throw data already booked by the learner, so a live score adds no measurable load.

## Options

1. **Landing score:** distance from each Throw's Fit landing to its target Site, RMS per Attempt, live in the GUI.
2. **Path score, your formula:** time-averaged squared distance between the Tracker's samples and the planned flight, mean per Attempt.
3. Both: landing live, path after the Sitting from the bag.

Recommendation: 1 first, in millimetres you already read as Catch radius; add 2 if it misses errors you see.

## Also wanted in your answer

- Should Feeds count, or only the robot's own Throws?
- A Throw with no Fit: leave it out and count it, or score it as a miss?

## What each answer unblocks

- 1: the next night builds the score module and its tests; the GUI readout follows on a branch.
- 2 or 3: the same, plus the planned path stored per Throw.
