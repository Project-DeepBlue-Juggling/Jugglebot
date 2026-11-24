---
title: "Learner efficacy E0: the unsettled-ball account tested on 3,950 Throws from existing bags - verdict BETWEEN, leaning against. The ball's centre does not move against the Cup before release (3 mm/s against a launch term of 17 mm/s); the launch term appears in the 35 ms after the announced release"
type: investigation
date: 2026-11-25
status: resolved
phase: "learner-efficacy E0 (PWAY-2): offline test of the unsettled-ball account against a criterion fixed beforehand; no production code changed"
related_plan: learner-efficacy.md
related_entries:
  - 2026-11-24-learner-efficacy-review-and-scatter-decomposition.md
files_changed:
  - tools/probes/release_settle_bag_probe.py
  - tools/probes/README.md
  - logbook/2026-11-25-learner-efficacy-e0-unsettled-ball-test.md
  - logbook/INDEX.md
  - plans/active/learner-efficacy.md
  - plans/active/INDEX.md
  - docs/project-map.md
subsystem:
  - motion
  - tracking
tags:
  - testing
  - performance
---

# Learner efficacy E0: the unsettled-ball test from existing bags

## Summary

**Asked (plan `learner-efficacy.md`, Phase E0):** the 2026-11-23 review found a launch term g (the ball's lateral launch velocity minus the commanded Cup velocity and the measured Platform error, about 17 mm/s per axis, random from Throw to Throw). The owner's account is that a ball that has not settled in the Cup, and circles in it, is the main cause. Does any measure of settling, taken from the bags we already have, explain g or the Throws that land 60 mm or more off?

**Verdict against the pre-registered criterion: BETWEEN, leaning NOT SUPPORTED.** 28 settle measures were tried on 3,950 Throws (3,125 with g; 1,154 carried, 1,971 at Dwell 0.32 s or less), with Sitting 2026-11-18 held out untouched until the measures were frozen.

- **No measure reaches SUPPORTED** (30 % of g's variance, or AUC 0.75 for the 60 mm tail). The best on the held-out Sitting: 4 % (Memory seat offset), a 10 to 12 % variance drop (Catch entry radius), AUC 0.72 for all scalar measures together on 9 far Throws (0.61 on the tuning Sittings, 25 far Throws).
- **Every velocity and offset measure is at zero.** The ball's velocity and offset against the Cup over the last 35, 100, 200 and 500 ms explain between -0.6 % and +2 % of g; Memory's seat offset, the one measure with a direction that shows anything, reaches 4 % on the held-out Sitting. In the last 35 ms the ball's centre moves against the Cup at 2.6 / 3.6 mm/s (robust sd, x / y), which is the noise floor of a ball that has rested for over a second (2.0 / 3.8 mm/s), against a g of 18.0 / 17.1 mm/s. A ball whose centre is circling at release is ruled out as the source of g.
- **It is not NOT SUPPORTED by the letter**, because three results sit above the 10 % and AUC 0.6 floors: Throws made from rest (ball seated 1 s or more) show 27 % less g variance than running Throws of the same configuration (95 % CI 2 to 46 %, n=81 against 293; the median |g| is the same, ratio 0.95, and in the two Sittings where the comparison can be made inside one Sitting the from-rest Throws have MORE variance, finding 3); the Catch entry radius shows a mild dose-response (g rms 16.2 mm/s in the most on-axis fifth, 18 to 20 in the upper three fifths); the combined-index AUC is 0.61 to 0.72 on few far Throws.
- **Where g does appear (a separate result, not a settle measure):** in the 35 ms after the announced release the ball moves against the Cup at 13.5 / 13.1 mm/s, and that velocity explains 20 % of g on the tuning Sittings and 22 % on the held-out one (r = +0.42 on each axis, n=2,947). g is born at separation, not before it.

Per the plan, BETWEEN goes to E1. This entry recommends E1 be aimed at the separation (seat sensitivity, spin, the release contact) and not at circling time.

## What was asked

1. Does a settle measure explain 30 % or more of g's variance held out, or separate the 60 mm tail at AUC 0.75 or more (SUPPORTED)?
2. Or do all of them stay under 10 % and AUC 0.6 on 300 or more Throws including carried Throws and Dwell 0.32 s or less (NOT SUPPORTED)?
3. Anything else goes to E1.

## Method

**Data.** All 21 Throw-bearing bags of Sittings 2026-11-06, 11-07, 11-10, 11-13, 11-18 and 11-20 (17 GB): 3,950 Memory rows matched to an announcement. 2026-11-08 has no Throws. Owner decisions at the checkpoint (2026-11-25): all Sittings in, 11-18 held out (1,321 Throws, 606 carried with g); 11-07 (Platform left tilted after a re-seat) and the 11-20 22:53 bag (after the Hand motor overheat) kept and flagged.

**g, re-derived first.** On the review's own 2026-11-06 15:53 series g came out at 17.7 / 17.0 mm/s and explains 56 % / 54 % of the lateral landing error held out (n=317; the review had 50 % / 69 % on n=205, having dropped Throws without a full-flight fit). The launch velocities match the review's to the last digit on the 205 shared Throws. Lag-1 r of g is 0.03. The headline reproduces.

**Two deviations from the review's definition, both tested.**
- *Commanded Cup velocity.* The review differentiated the Cup position built from Hand telemetry positions on paired stamps; with the Hand moving at about 3.7 m/s along a Banked axis, a few ms of stamp error puts about 20 mm/s of noise in the lateral velocity. This entry takes the Hand term from the commanded Hand velocity instead (`vcup*_a` in the probe). The cleaner velocity correlates better with the launch velocity (r 0.75 against 0.50 on x) but g barely changes (17.1 / 16.5 mm/s), because the commanded Cup position already carries the same information.
- *Regression strata.* g's regressions are fitted per Sitting day and release kind (60 Throws or more), within Attempt-by-Site-by-kind groups of 5 or more. For the from-rest comparison a second g is centred on the configuration (bag, Site, kind, Dwell), since a first Throw often has no five same-kind neighbours in its Attempt.

**Release reference.** The announcement's `throw_time` sits on the 25 ms tick grid, and Memory's `t_abs_s` is up to 12.5 ms away from it. On the 2026-11-06 15:53 Sitting the ball's own peak upward velocity lands 2.7 ms before the announced time (sd 6.5 ms, most of it a late cluster, finding 5) and scatters 9.8 ms against Memory's time; over all Sittings the median is 2.3 ms before, with 86 % of Throws within 3 ms of it, so the announcement is the executed release and is used throughout, as the review did.

**Settle measures** (`tools/probes/release_settle_bag_probe.py`). The ball marker is expressed in the Platform's Mocap frame per frame (Kabsch on the five Platform markers), reduced to a lateral offset from the Hand axis (a line fitted per bag), and turned to world axes at release. Timing goes through `_bag_time.py` for every stream. The ball is visible in the Cup in 90 to 100 % of frames (median coverage 1.0 in the last 35 and 100 ms), which is far better than the review's 35 ms check suggested: that check worked in world axes and never removed the Platform's own motion.

**The occlusion artefact, and what was done about it.** The raw offset steps by 3 to 8 mm at particular Hand heights, the same way on every Throw: the cameras lose and regain parts of the ball behind the rim and the Platform, and the marker centroid shifts (the taped-ball bias). The measures are therefore taken on the residual from the median trace of the other Throws that share a trajectory (bag, Site, kind, Dwell), indexed by time before release, leaving the Throw itself out; grid points where the Throws disagree most (the step zones) are dropped. A ball at rest with the Hand still needs no template.

**The 28 measures.** Nine with a direction: velocity against the Cup over the last 35 ms, -100 to -35 ms, 100, 200 and 500 ms; mean offset over -100 to -35 ms and over 200 ms; the Catch entry offset (where the ball came in against the Hand axis, measured above the rim); Memory's seat offset. Eighteen without: time since the Catch, Dwell, wiggle and path speed over 100 and 200 ms, the magnitudes of three velocities and of the offset, Catch entry radius, |Memory seat offset|, axial lift (maximum and rms), and four Hand ball sensor measures (time held, raw flips, raw-low fraction, the lag from the Mocap seat to the sensor's HELD). Plus from rest (seated 1 s or more), which Mocap and the sensor agree on for 99 of 101 Throws.

**Scoring** (owner, 2026-11-25). A measure with a direction: held-out R-squared of g, folds being whole Attempts, and a fit on the tuning Sittings scored on 11-18. A measure without one: the drop in g's variance in the most-settled third, thirds taken within Sitting day and release kind. From rest: the variance ratio against running Throws of the same configuration, bootstrapped by Attempt; this comparison was fixed as primary before 11-18 was opened. The tail: landing minus command, 60 mm or more from the configuration's median (49 Throws, 34 with g). Seed 0 throughout; the final run was made twice and the two outputs are byte-identical.

## Findings

### 1. Measures with a direction (held-out R-squared of g, x and y pooled)

| Measure | Validated? | Tuning Sittings (n) | Held-out 11-18 (n) | Fit on tuning, scored on 11-18 |
|---|---|---|---|---|
| Velocity, last 35 ms | yes: noise floor 2.0 / 3.8 mm/s on from-rest Throws | +0.001 (1901) | -0.004 (1048) | -0.000 |
| Velocity, -100 to -35 ms | partly (artefact residue, sd 6.3 / 12.3 mm/s) | -0.001 (1852) | -0.001 (1048) | -0.002 |
| Velocity, 100 ms | partly | +0.000 (1888) | -0.003 (1060) | -0.003 |
| Velocity, 200 ms | partly | -0.001 (1739) | -0.002 (994) | -0.002 |
| Velocity, 500 ms (carried only in practice) | partly | -0.004 (615) | +0.020 (596) | +0.015 |
| Offset, -100 to -35 ms | partly | +0.002 (1852) | -0.001 (1048) | +0.001 |
| Offset, 200 ms | partly | -0.002 (1739) | -0.006 (994) | -0.003 |
| Catch entry offset | no: uncorrelated with Memory's seat offset (r 0.06 / 0.03) | -0.001 (1849) | +0.003 (1057) | -0.001 |
| Memory seat offset | not validated here | +0.010 (2014) | +0.040 (1111) | +0.040 |
| All nine together | | +0.004 (2014) | +0.032 (1111) | +0.041 |
| *Velocity 0 to +35 ms AFTER release (separate)* | | *+0.195 (1889)* | *+0.223 (1058)* | *+0.220* |

The same table restricted to Throws with adequate coverage (80 % or more in the 35 and 200 ms windows; 1,967 Throws) and split by release kind changes nothing: the largest single-measure pre-release figure is +0.071 (Memory seat offset, carried, tuning) and +0.048 on 11-18; all nine together reach +0.090 on the tuning Sittings' carried Throws and +0.027 on 11-18's.

### 2. Measures without a direction

Drop in g's variance in the most-settled third; AUC for the 60 mm tail (far Throws in brackets).

| Measure | Validated? | Drop, tuning | Drop, 11-18 | AUC, tuning | AUC, 11-18 | AUC, all |
|---|---|---|---|---|---|---|
| Time since the Catch | yes (by construction) | +18 % | +3 % | 0.59 (39) | 0.54 (10) | 0.60 (49) |
| Dwell | not scorable in thirds: constant within a stratum; see finding 4 | | | 0.53 | 0.54 | 0.55 |
| Wiggle 100 ms | no: does not separate rest from running (AUC 0.44, 15 rest Throws) | +5 % | +7 % | 0.55 | 0.56 | 0.57 |
| Wiggle 200 ms | yes: rest 0.13 mm, running 0.53 mm (AUC 0.85); decays with time since the Catch (rho -0.56) | 0 % | +5 % | 0.50 | 0.47 | 0.50 |
| Path speed 100 ms | no | +6 % | +4 % | 0.56 | 0.54 | 0.57 |
| Path speed 200 ms | yes (AUC 0.80, rho -0.59) | -3 % | +2 % | 0.49 | 0.44 | 0.49 |
| Speed, last 35 ms | no (at the noise floor) | -3 % | +3 % | 0.52 | 0.68 (9) | 0.57 |
| Speed, -100 to -35 ms | no | +20 % | 0 % | 0.53 | 0.50 | 0.53 |
| Speed, 100 ms | no | +22 % | 0 % | 0.58 | 0.53 | 0.58 |
| Offset size | no | +20 % | +4 % | 0.47 | 0.44 | 0.48 |
| Catch entry radius | partly: tracks the sensor's seat lag (rho 0.34) and raw-low fraction (0.24) | +15 % | +10 % | 0.60 | 0.63 (9) | 0.60 |
| Memory seat offset size | no | +9 % | -7 % | 0.51 | 0.44 | 0.48 |
| Axial lift, maximum | no (Hand telemetry stamp slack dominates) | +18 % | +4 % | 0.52 | 0.44 | 0.50 |
| Axial rms | no (same) | +19 % | -4 % | 0.48 | 0.60 (8) | 0.51 |
| Sensor: time held | yes (by construction) | +11 % | +2 % | 0.54 | 0.44 | 0.53 |
| Sensor: raw flips, 500 ms | separates rest from running (AUC 0.96) but that is the arrival edge | -68 % | -42 % | 0.50 | 0.62 | 0.52 |
| Sensor: raw-low fraction, 200 ms | same caveat (AUC 0.83) | +13 % | +6 % | 0.55 | 0.50 | 0.55 |
| Sensor: seat lag | partly (rho 0.34 with entry radius) | +4 % | -5 % | 0.43 | 0.26 | 0.39 |
| All eighteen plus from rest, one index | | +19 % (10-fold) | +5 % (fit on tuning) | 0.61 (25) | 0.72 (9) | |

The tuning-Sitting drops of 18 to 22 % did not survive on 11-18 (0 to 4 %), which is what a best-of-many selection looks like. The one figure that holds on both is the Catch entry radius (15 % and 10 %).

### 3. From rest (settled by construction)

| Comparison | From rest | Running | Variance ratio | Drop |
|---|---|---|---|---|
| Same configuration, tuning (primary) | n=81, rms 14.5 mm/s | n=293, rms 17.0 | 0.73 (95 % CI 0.54 to 0.98) | 27 % |
| Same configuration, all Sittings | n=95, rms 14.4 | n=293, rms 17.0 | 0.72 (0.55 to 0.95) | 28 % |
| Against every Throw, tuning | n=81 | n=2,286, rms 19.4 / 18.8 | 0.59 (0.43 to 0.76) | 41 % |
| Against every Throw, 11-18 | n=14 | n=1,223 | 0.78 (0.44 to 1.11) | 22 % |

The median |g| of from-rest Throws is 0.95 of the running median. In the only two Sittings that hold both kinds of Throw in one configuration the variance ratio is above 1 (2026-11-06: 1.20, n=12 from rest; 2026-11-20: 2.21, n=9): there the from-rest Throws scatter more. The pooled 0.73 therefore comes from setting the 57 from-rest Throws of 2026-11-13 (a low-g Sitting) beside running Throws that are mostly 2026-11-06's (254 of 293), so it is partly a difference between Sittings and the 27 % is an upper reading, not a within-configuration result. 11-18 has no configuration with both kinds of Throw. No from-rest Throw landed 60 mm off (0 of 95; 1.1 expected at the running rate).

### 4. Dose-response

g rms (both axes pooled) by time since the Catch, all Sittings: under 0.32 s 19.0 mm/s (n=2,134); 0.32 to 0.40 s 18.4 (222); 0.40 to 0.50 s 14.3 (156); 0.50 to 0.70 s 15.6 (937); from rest 14.4 (95). That looks like a step at 0.4 s, but it is mostly a Sitting effect: at Dwell 0.27 s the Sittings read 19.4 (11-06), 28.4 (11-07, the tilted Platform), 13.2 (11-10), 17.8 (11-13), 16.3 (11-18), 18.6 (11-20), against 15.3 to 16.4 for carried Throws at Dwell 0.50 s. Without 11-07 the short-Dwell Throws exceed the carried ones by 1 to 3 mm/s in four Sittings and fall below them in one (11-10). By Catch entry radius: 16.2, 16.4, 18.4, 19.7, 18.4 mm/s across fifths, and the far-Throw rate rises from 0.5 % to 1.8 %. By wiggle, seat lag and axial lift there is no trend.

### 5. Release timing (not a settle measure)

In 4.2 % of Throws (152 of 3,628) the ball's peak upward velocity comes more than 8 ms after the announced release. Their g rms is 19.3 mm/s against 15.9 for the rest; their far-Throw rate is 1.3 % against 1.1 %.

## Discussion

### Why these measures, and what each rests on

The owner described the unsettled ball at the checkpoint: its centre circles at mm scale, it can also spin or rock in place, it is worse after an off-axis Catch, and it bounces axially sometimes, mostly after an off-axis landing; an on-axis ball seats almost at once. The measures follow that description. Velocity and offset against the Cup see a circling centre directly. Wiggle and path speed see it without needing a direction. The Catch entry offset and radius, and the sensor's seat lag, stand for the off-axis Catch. Axial lift stands for the bounce. From rest and time since the Catch stand for "settled" with no reliance on the marker at all.

Nothing here sees a ball spinning or rocking in place with its centre still. That part of the account is untested by the marker measures and is reached only through from rest and time since the Catch.

### What the result shows about the owner's account

It splits the account in two.

*A circling centre is not the source of g.* If g were the ball's sideways motion in the Cup carried into flight, the ball would be moving against the Cup at about 17 mm/s just before release. It is moving at 3 mm/s, the same as a ball that has sat for over a second, and that 3 mm/s has no relation to g (R-squared 0.001 on 1,901 Throws, 0.000 scored on the held-out Sitting). The measurement is validated in the one way open to it: from-rest Throws give its noise floor, and the floor is a fifth of g.

*Settling as a whole is not excluded.* Pooled, Throws from rest carry about a quarter less g variance than running Throws of matching configuration, with an interval that nearly reaches zero, an unchanged median, and the opposite sign in the two Sittings where the match is exact; off-axis Catches are followed by slightly larger g. Both are in the direction the owner predicted and both are small: if every Throw were as settled as a from-rest Throw, g would fall from about 17 to about 14.5 mm/s per axis. A from-rest Throw also differs from a running one in ways that are not the ball (no Catch stroke before it, a Platform that has not just moved), so even the quarter is an upper bound on what settling buys.

So the account's strong form ("primary cause") does not fit, and a weaker form (settling contributes a minority share, through something other than a moving centre) is still open.

### g is born at separation

The post-release window was added at the checkpoint, reported apart from the criterion. In the 35 ms before the announced release the ball does not move against the Cup; in the 35 ms after, it moves at 13 mm/s and that motion already holds 20 to 22 % of g. The remainder is plausibly still being acquired while the ball clears the Cup (the window ends before it has fully left) and measurement noise in a 35 ms fit. This places the term in the tens of ms around separation: the release contact between ball and Cup as the Hand decelerates, with spin and a rim touch as the candidates the review already named. It is the result E4 most needs, and it argues against a settle check before release or a Catch that damps circling as the lead candidates.

### What limits the claim

- **Occlusion.** The template removes the repeatable artefact but its step zones jitter from Throw to Throw, so wiggle, path speed and the velocities over 100 ms and longer carry artefact residue (sd 6 to 12 mm/s in the -100 to -35 ms velocity). They are marked "partly" validated and a null from them is weaker than the null from the last 35 ms, where the artefact is smooth and the floor is measured.
- **The template removes what is common.** A ball that always rolls the same way in a given configuration is subtracted with the artefact. g is the random part by definition, so this does not hide a cause of g, but it would hide a systematic bias.
- **The number of measures.** 28 were tried. The thresholds are effect sizes, so no correction was applied to them; the protection is the held-out Sitting, on which the tuning Sittings' best scalar figures fell from about 20 % to about 0 to 4 %.
- **Few far Throws.** 49 in all, 34 with g, 10 on the held-out Sitting. An AUC of 0.72 on 9 Throws has an interval that covers both 0.6 and 0.75; the tail question is not settled by these bags.
- **From rest is thin and unevenly spread.** 95 Throws with g, 57 of them from one Sitting (11-13), none comparable on 11-18, and the pooled comparison mixes Sittings (finding 3). The 27 % is one of three results keeping the verdict off NOT SUPPORTED; the other two (Catch entry radius 10 to 12 %, index AUC 0.61 to 0.72) do not depend on it.
- **The axial measures failed validation**: Hand telemetry is 100 Hz with up to 14 ms of stamp and encoder age, which at Hand speeds is tens of mm. The bounce the owner describes is untested.
- **g itself.** It is a regression residual within groups; anything the Platform markers and the command do not capture is in it, including Tracker-independent Mocap noise in a 165 ms fit.

### What was ruled out, and what would overturn this

Ruled out: a ball whose centre is moving against the Cup at release as the source of g; rim occlusion as a reason the bags cannot answer the question (coverage is 90 to 100 %); the review's 35 ms check as evidence either way (it mixed Platform motion into the ball's). Not ruled out: spin or rocking in place; a seat that is off-axis by less than the artefact residue (about 0.5 mm) and sets the direction of the release contact; the release contact itself.

What would overturn the lean: a bench measurement (E1) in which a deliberately unsettled ball launches with several times the lateral scatter of a hand-placed one, or in which launch direction follows a seat offset of a few tenths of a mm. What would confirm it: bench launch scatter that is the same for settled and unsettled balls and present for a hand-placed ball.

### Tradeoffs accepted

- The extraction ran as one background script from the orchestrating session, not as several agents (owner, 2026-11-25): the same output for far fewer tokens.
- The probe is a run-on-demand harness with no committed fixture and no test. A fixture cut from 17 GB of bags that pins measures still marked "partly validated" would freeze the wrong thing; if E1 or E4 comes to depend on one of them, that is the time.
- The analysis script is one-off and uncommitted (below); only the extractor is promoted.

## What changes

- **`tools/probes/release_settle_bag_probe.py`** (new) with its row in `tools/probes/README.md`: per-Throw release state (the ingredients of g) and the in-Cup settle measures from a bag.
- **`plans/active/learner-efficacy.md`**: step PE0 complete, with the Outcome and what it means for E1 and E4.
- No production code changed. No map Item is closed. BUG-79 and IDEA-18 are not touched by this result: the carried and from-rest Throws show the same g (15 to 16 against 16 to 19 mm/s), and E0 measured scatter, not the bias those Items describe.

## Verification

- **Baseline:** `git log -1` at the start, 2026-11-24: `bc101eaf plans: archive learner-generalisation (superseded in part); narrow the PWAY-2 Destination; learner-efficacy owner answers`.
- **g re-derived**, 2026-11-24, `python rederive.py` on the review's `b1106.npz` (session scratchpad): n=317, g sd 17.7 / 17.0 mm/s, held-out R-squared of landing error on g 0.56 / 0.54; launch velocity equal to the review's on the 205 shared Throws (sd of difference 0.0).
- **Extraction**, 2026-11-25, `release_settle_bag_probe.py extract <bag>` over 21 bags, log `temp/logs/e0-extract-2026-11-24.log`: all 21 completed. `features` over the 21 series, log `temp/logs/e0-features-2026-11-25.log`: 3,950 Throws, 3,666 with a launch fit.
- **Analysis**, 2026-11-25, `python analysis.py tune` then `python analysis.py final` in `temp/analysis/learner-efficacy-e0/` (local, gitignored; `analysis.py`, `e0lib.py`, `plots.py`, outputs `out_tune.txt`, `out_final.txt`, `throws_final.csv`): the tables above. `final` run twice: outputs byte-identical (md5 `56a1d93cfce55338b788a231ca4f4e57`).
- **The committed probe reproduces the analysed records**, 2026-11-25, `python tools/probes/release_settle_bag_probe.py features temp/probes/release_settle/2026-11-10_19-00-33.npz --out <scratch>/a.csv`, then `cmp` against the analysed CSV: identical.
- **Plots:** `temp/probes/release_settle/validate_2026-11-06_15-53-05.png`, `validate_2026-11-13_11-42-08.png` (settled against running traces), `g_vs_measures.png`.
- **Tests:** named in the commit message with the date, command and result (the probe has no importer and no test reads it by path; the documentation tests that read the changed files were run).

## Open Questions

- Is the release contact repeatable for a hand-placed ball on the bench Hand, and how does launch direction respond to a seat offset of a few tenths of a mm? (E1.)
- Does spin at release link g to the in-flight curvature the review measured (5 to 7 mm of landing)? Not separable in these bags.
- Why does 2026-11-10 show a g of 12 to 13 mm/s at the same Dwell where 2026-11-06 shows 18 to 19?
- The 4.2 % of Throws whose peak velocity is late: a real late separation or a flat-topped velocity profile?
