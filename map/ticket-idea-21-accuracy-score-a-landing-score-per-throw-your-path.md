---
id: TKT-60
kind: ticket
title: 'IDEA-21 accuracy score: a landing score per Throw, your path score over the flight, or both, and do Feeds count?'
status: complete
created: '2026-11-10'
updated: '2026-11-09T21:44:39Z'
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

## Answer

I like the idea of Landing score, but I worry that it might be vulnerable to mocap noise. Path score would also be, but I'd think that noise would be averaged out over the whole path. At the end of the day, I'd expect both to be functionally identical (assuming the Path score was calculated based on the actual throw state, not the planned one) and I can see how Landing score is faster/easier to compute. Could you try both on some logs to see how they differ?

Feeds should be scored as well, but separately. I'm thinking something like this:
- After a juggling Attempt ends, as part of the message in the GUI stating how many throws/catches were made, the 'average accuracy' score is given.
- After a ball is fed to Jugglebot, that throw's accuracy score is given.

These scores should be appended to a list of scores for Jugglebot and Ball Butler. It might make sense to separate Jugglebot's scores into the different Patterns, though I'm open to discussion here. I'm just thinking this could be a useful source of data as we aim to improve the accuracy score; this will give us a tangible comparison metric to use over time to gauge the robots' performance.
