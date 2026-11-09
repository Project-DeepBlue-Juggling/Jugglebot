---
id: IDEA-21
kind: idea
title: Create and display an 'accuracy'/'tidyness' score
status: backlog
created: '2026-11-09'
updated: '2026-11-09T17:52:35Z'
owners: []
priority: medium
effort: m
hardware: none
owner: decision
radius: subsystem
area: gui
blocked_by: []
links: []
filed_by: harrison
parent: pathway-gui-improvements
---

A metric should be devised that 'scores' juggling patterns based on how close the real pattern matches the target pattern. I'm thinking something like `sum of ( error^2 / sample rate )` for each throw, as well as an average for the whole pattern. 

If this would add excessive burden to the Jetson, this could be calculated after each session as long as we're able to buffer the relevant data for long enough.

## Feasibility (2026-11-10)

- **Data already there.** Every Throw's learner row (`motion/skills/memory.py`) holds its target Site (`site_xy_m`) and the Fit's observed landing (`y`). A landing error per Throw is one subtraction at Throw end, on the orchestrator thread that already writes the row. No new load on the Jetson.
- **Your formula** (squared error summed over samples, divided by the sample rate) is an integral of squared error over the flight. It needs the planned ballistic path of each Throw beside the Tracker's samples. Both exist at Throw end; it costs a few hundred samples per Throw, or it can run after the Sitting from the bag.
- **Effort** m: the score in a pure module with its tests, a field on the end-of-Attempt summary, a GUI readout. Hardware none to build; judged on recorded Sittings first.
- **Risk:** a score that mixes what the robot controls (the Throw) with what it does not (Ball Butler's Feed, Tracker gaps) reads worse for reasons no tuning fixes. Feeds and Throws with no Fit should be left out, and counted.

## Options (2026-11-10)

1. **Landing score:** per Throw, the distance from the Fit's landing to the target Site; per Attempt, the RMS. Live in the GUI.
2. **Path score (your formula):** per Throw, the time-averaged squared distance between the Tracker's samples and the planned flight; per Attempt, the mean. Catches apex and timing errors the landing hides.
3. Both: the landing score live, the path score after the Sitting from the bag.

Recommendation: 1 first. Its data is already booked per Throw, and it reads in millimetres you already use (Catch radius). Add 2 if the landing score misses errors you see.

## Progress

- 2026-11-10: Night Shift 2026-11-10: under PWAY-10 as asked; feasibility and options written; awaiting the owner's choice in TKT-60
