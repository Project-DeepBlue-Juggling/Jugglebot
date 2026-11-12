---
id: IDEA-21
kind: idea
title: Create and display an 'accuracy'/'tidyness' score
status: complete
created: '2026-11-09'
updated: '2026-11-11T19:00:09Z'
owners: []
priority: medium
effort: m
hardware: none
owner: none
radius: subsystem
area: gui
blocked_by: []
links:
- logbook/2026-11-12-landing-score.md
filed_by: harrison
parent: gui-improvements
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

## Findings (2026-11-11): landing score vs path score on recorded Sittings

From TKT-60: you asked to try both scores on logs. Four bags were read offline: 360 Throws and 20 Feeds, over self_toss, columns, hop and columns_turned. Each Throw's target Site and planned flight come from `/throw_announcements`, and the Fit and samples from `/balls` and `/mocap_data`. All figures are mm.

| Flights | Landing median / RMS | Path vs plan median | Path vs Fit median | Rank agreement (rho) |
|---|---|---|---|---|
| all Throws (20 without a Fit) | 24.8 / 51.5 | 80 | 2.5 | 0.30 |
| all Feeds | 22.0 / 22.2 | 318 | 3.2 | -0.14 |
| hop, 2026-11-10 19:00 | 14.0 / 18.8 | 59 | 1.6 | 0.33 |
| columns_turned, 2026-11-10 20:11 | 26.7 / 70.2 | 88 | 2.3 | 0.08 |

- **Mocap noise barely moves the landing score.** The noise is 0.2 to 1.2 mm per sample. Refitting the noisy samples moves a landing by 0.14 to 0.35 mm, against landing errors of 14 to 32 mm.
- **The scores are not the same when the path is the planned flight.** Path vs plan is 3 to 15 times the landing error, and it ranks flights differently. The plan is the nominal release, but the flown ball leaves earlier or higher. One Feed flew an apex of 2518 mm against 2236 mm planned, yet landed 22 mm from its target.
- **Path vs the Fit's own parabola** (the "actual throw state" variant) is 1.6 to 3.2 mm. It measures mocap noise and Fit quality, not accuracy, and does not track landing error (r = 0.03).
- **RMS is dominated by a few misses.** One Columns window reads median 31 mm but RMS 95 mm. A per-Attempt median plus a miss count reads truer.
- **Cost:** the landing score is a lookup of the Fit the Tracker already makes (about 13 us per flight). The path score is 5 to 14 ms per flight.
- **Caveats:** 4 to 5.5 minute windows per bag; only one bag has turned Columns. The target is the announced landing point, not the learner's `site_xy_m`. A parabola fit cannot see a systematic mocap offset.

Recommendation: the landing score, with Feeds apart and Throws per Pattern, and a count of Throws without a Fit. Keep path vs Fit only as a mocap-health number.

## Progress

- 2026-11-10: Night Shift 2026-11-10: under PWAY-10 as asked; feasibility and options written; awaiting the owner's choice in TKT-60
- 2026-11-11: Night Shift 2026-11-11: both scores tried on four bags (Findings); the landing score wins; awaiting the build choice in the question ticket
- 2026-11-12: Night Shift 2026-11-12: held on night-shift/2026-11-12-IDEA-21: landing score built, 2 audit findings open (Feed target, empty-Cup count); unblocked by your answer
