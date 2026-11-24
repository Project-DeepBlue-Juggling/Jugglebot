---
title: "PWAY-2 review: the Learner as built cannot deliver the efficacy wanted; Throw scatter is dominated by a random launch term that no landing-space estimator can reach, and the Pathway is re-scoped to a new plan (learner-efficacy) - desk investigation from existing Memory and bags, no code change"
type: investigation
date: 2026-11-24
status: resolved
phase: "learner-generalisation review (PWAY-2): eight read-only studies, owner re-scope 2026-11-24; the old P5 retired in favour of plans/active/learner-efficacy.md"
related_plan: learner-efficacy.md
files_changed:
  - logbook/2026-11-24-learner-efficacy-review-and-scatter-decomposition.md
  - logbook/INDEX.md
  - map/pathway-learner-generalisation.md
  - map/bug-learner-far-site-guard-fails-on-long-single-site-memories.md
  - map/bug-plant-model-pools-carried-and-from-rest-rows-so-oval-rows.md
  - map/feature-learner-a-noise-aware-shrinkage-estimator-in-place-of-the.md
  - map/idea-classify-the-5x-jerk-intervals-in-the-oval-real-acceleration.md
  - plans/active/learner-efficacy.md
  - plans/active/learner-generalisation.md
  - plans/active/INDEX.md
  - docs/project-map.md
subsystem:
  - motion
  - tracking
tags:
  - testing
  - docs
  - performance
---

# PWAY-2 review and the scatter decomposition (desk investigation, no code change)

## Summary

**Asked (owner, 2026-11-23):** the front page says PWAY-2 "Learner generalisation" is at "P5 Close-out". Is that true to the Pathway's heart?

**Answer:** formally true (P3a, P3b, P3c, P1, P2 and P4 complete 2026-11-06; P5 not started), substantively misleading. P5 held most of the Destination's hardware proof, and the Learner as built cannot deliver the efficacy the owner wants. Eight read-only studies on 2026-11-23 found that the landing error is mostly white (sd 19.9 / 19.3 / 17.2 mm on x / y / Apex against 21.7 / 20.3 / 18.5 mm for the whole model-only error in the same study; its source is the ball's launch state, with a random launch term g that is invisible to everything the Learner and the Platform sensors see), and that the learnable part has a ceiling of 2 to 4 % of rms on x and nothing on y or Apex. The Learner also has three real flaws (Defects A, B and C below) that are worth fixing for quietness and correctness, not for scatter. The owner re-scoped (2026-11-24): same Pathway, a new plan `plans/active/learner-efficacy.md`, old P5 retired with its five items mapped.

No learner or plant-model code changed, and no plan was edited between 2026-11-08 and this entry.

## What was asked

1. Is "P5 Close-out" an honest status for PWAY-2?
2. If the Learner does not give the efficacy wanted, why not, and where does the scatter come from?
3. What should the Pathway's next plan contain?

## Method

Eight read-only studies, run 2026-11-23 by agents under one orchestrating session: a code review of the Learner (Opus), a Memory-data study, a predictability bound with cross-validation, a sequential replay of five command laws, a release-state decomposition from bags (Opus), an Attempt-end census, a jerk study, and an audit of the old plan's P5 and the Destination wording. Scripts and outputs are local and gitignored: `temp/analysis/learner-review-2026-11-23/` (subfolders `predict/`, `release/`, `jerk/`, plus `a.py b.py c.py e.py` and the probe scripts at the top level) in the checkout `~/Desktop/Jugglebot`.

**What was NOT re-verified.** Every number below is the studies' own. Nobody re-ran them independently, and the file:line references from the code review were not re-read when this entry was written. Treat the numbers as measured once.

## Findings

### 1. What is delivered (stands)

- Box retirement is real: zero live references to AdmissibleBox, `admissible_box.yaml` or `gate_hash`; a test asserts the old files are absent; online admissibility is the only path (hundreds of live checks flown, 0 refusals at P2).
- The global plant model f0 works: K 1.077 to 1.088 over 7 flying days, mean lateral bias about 0, no per-ball difference. The first Throw at a never-seen (Site, Apex) landed 26.7 mm lateral (n=18) against 26.8 mm steady state (n=1896); in the migrated rows it was 35.9 mm (n=45) against 25.9.
- The polygon walk (Site changes every Throw, one ball, empty Memory) caught 25 of 25 on 2026-11-20.
- No Attempt end in any debrief since the merge is attributed to the Learner.

### 2. Code review of the Learner

- Live path verified: `executor._command_u` to the context to `Learner.command` (learner.py:296): u_m = f0 inverse of y_d, plus alpha times r-hat over the 16 most recent support rows, clipped to c(1 - e^(-sum w / 2)) about u_m. Cap 30 mm lateral, 20 mm Apex. K and b forget at lambda 0.99.
- The key matches exactly on ball_id and release_kind (learner.py:261); kernel h_p 0.05 m, h_v 0.15 m/s; Dwell is inert (h_d infinite); yaw appears nowhere. A residual transfers about 0.1 m in Apex and about 50 mm in Site (sum w 16 at the point, 5.9 at Site +50 mm, 0.3 at +100 mm, 1.4 to 1.8 at Apex +-0.1 m).
- **Defect A: most-recent-k, not nearest-k** (learner.py:270). On the corpus 49 % of queries get less than half the evidence nearest-k would give; 25 % get sum w below 4 although 12 or more near-exact rows exist.
- **Defect B: the far-Site guard fails on long single-Site memories.** b forgets and s does not, so the sd of s falls from 0.245 to 0.013 with no excitation. Past about 150 single-Site rows an invented slope is applied; a real oval-only Memory (bug89) applies -18.7 mm at x = +0.125 with no rows there. The test at `test_plant_model.py:206` checks only at 60 rows. The plan's statement that a Site far from the data gets no offset is false in that regime.
- **Defect C: f0 pools carried and from-rest rows.** Adding 607 oval rows to the main Memory moves the applied offset at the +62.5 mm Site from -3.3 to +4.8 mm. The old P5 item 4 criterion ("theta is shared" between Columns and the oval) would contaminate Columns as written. The p4-oval fit gives b_x = +179 mm and s = +1.23 per m, outside the priors; only the sum means anything.
- The proportional law with alpha at most 0.889 leaves 11 % of any residual uncorrected.
- Memories are siloed per plant_id (23 under `temp/learn`, mostly single-Pattern), so cross-Pattern sharing has never happened on hardware. A fresh plant's first-Throw Apex ratio is 1.077 +- 0.015 (+51 to +92 mm), IDEA-23.
- Missing tests: transfer (learn at A, first-Throw error at B), an out-of-family plant, a long single-Site memory against the guard, leakage between release kinds, a closed-loop mid-Pattern change.

### 3. Memory data (2,090 live rows, 2026-11-06 to 11-20, `temp/learn/jugglebot/memory_v2.csv`)

- Landing error sd is x 21.1 mm and y 20.3 mm; lateral rms 29.2 mm live against 29.5 mm migrated. Apex robust sd 15 mm. It is flat by Throw index (Throw 1: 27.0 mm, n=89; Throw 21 and later: 27.1 mm, n=831). Lag-1 autocorrelation 0.05 / 0.04 / 0.15.
- Counterfactual without the residual correction: lateral rms 29.7 mm against 29.2 mm actual; Apex 18.5 against 19.3 (worse with the correction).
- The target y_d is not stored and was inferred (x = 0, y = 0, Apex = round(u2 * 1.081, 0.05)); a replay of the real Learner matched the stored u to 1.6 / 1.6 / 2.2 mm. No Attempt id is stored, so 120 s gaps were used.
- About 70 % of rows are one configuration (Sites +-62.5 mm, Apex 0.95 or 1.0, Dwell 0.27); 4 carried rows.

### 4. Predictability bound (2,051 rows, cross-validation grouped by Attempt and by day)

- Best held-out R-squared of the model-only error from every pre-Throw column plus history features: x +0.044 (two-ball +0.073), y -0.016, Apex -0.028. Ceiling: 2 to 4 % of rms on x, nothing on y or Apex.
- Variance by source (mm sd, x / y / Apex): between-day 1.5 / 2.0 / 0.4; between-Attempt 1.8 / 1.4 / 3.6; between-configuration 6.9 / 3.8 / 4.4; slow within-Attempt about 0; white 19.9 / 19.3 / 17.2.
- Transit (Site minus previous Site) is the largest single predictable piece: R-squared on x +0.056, about 1 mm rms. It is not shown to be BUG-79's mechanism specifically.

Sequential replay, a prediction-quality test and not a closed-loop simulation (rms in mm, x / y / Apex; command jitter x / y / Apex):

| Law | rms x / y / Apex | command jitter x / y / Apex |
|---|---|---|
| Model only | 21.71 / 20.26 / 18.54 | 0 |
| Production | 20.97 / 20.28 / 18.14 | 8.5 / 5.7 / 4.7 |
| Nearest-16 | 21.12 / 20.44 / 18.46 | 9.7 / 7.1 / 4.3 |
| Shrinkage (kernel mean over all past rows, gain n_eff / (n_eff + sigma_noise squared / sigma_prior squared), sigma_prior 10 mm) | 20.91 / 20.07 / 18.48 | 6.2 / 5.5 / 0.8 |
| Shrinkage plus leaky integrator (0.95 / 0.05) | 20.76 / 19.95 / 18.10 | 7.6 / 5.6 / 2.7 |

Differences between laws are 0.1 to 0.3 mm with no confidence interval: ties, except against model-only. A shuffled-error control shows that noise alone reproduces about 70 % of the production correction's median magnitude and about half its power.

### 5. Release-state decomposition from bags

Sample: n=205 Self-throw 2026-11-06 15:53 and n=129 Columns 2026-11-20 (Opus). Held-out R-squared of lateral landing error (x / y), within Attempt-by-Site groups:

| Covariate set | 11-06 | 11-20 |
|---|---|---|
| Mocap full-flight landing to Memory landing (Tracker check) | 0.96 / 0.95 | 0.99 / 0.98 |
| Ball launch state (release position + launch velocity) | 0.82 / 0.78 | 0.85 / 0.87 |
| Commanded Cup position + velocity at release | 0.10 / -0.04 | 0.02 / -0.06 |
| Measured Platform error (Mocap velocity, tilt, tilt rate vs command) | 0.16 / 0.05 | 0.34 / 0.26 |
| Hand velocity and position error; release dz; transit; seat | 0.03 or less | 0 or less |
| g: ball launch velocity minus (commanded Cup + measured Platform error) | 0.50 / 0.69 | 0.35 / 0.42 |
| Command + Platform error + g + release position | 0.93 / 0.88 | 0.93 / 0.92 |

- g is about 16 mm/s per axis (about 14 mm of landing), random (lag-1 |r| at most 0.11), and invisible in the Platform's measured motion.
- The Tracker is cleared: it lands within 2 to 4 mm of the full-flight Mocap fit, with Fit residual 0.31 mm rms.
- Platform tilt error is 0.06 degrees (about 3.7 mm of landing). The earlier "tilt at release correlates with scatter, r 0.78" is the commanded Banking (commanded tilt explains 93 to 94 % of the measured Platform normal), not a tracking error.
- Platform lateral velocity error: 6.5 / 7.2 mm/s Self-throw (about 6 mm), 16.9 / 12.6 mm/s Columns (about 11 to 15 mm); upper bounds, since Mocap derivative noise is 3 to 10 mm/s. The one partly learnable term is the Columns Platform x-velocity error (R-squared 0.27).
- In-flight curvature varies (lateral acceleration sd 21 to 24 mm/s squared), about 5 to 7 mm of landing; spin would link it to g (inferred).
- Apex: the Fit's vertical launch speed sd is 30.4 mm/s (0.72 %), about 13 mm of Apex; Hand telemetry at 100 Hz cannot resolve it. Platform vertical velocity at release was not examined.
- The ball marker in the Cup over the last 35 ms correlates with g at r 0.09 to 0.48, confounded by rim occlusion: inconclusive.
- Not covered: Sittings 11-07, 11-08, 11-10, 11-13, 11-18; the IMU; carried Throws (none in the sample).

### 6. Attempt-end census (149 ends: four 2026-11-06 sets, 11-07, 11-18)

| Class | n | Share |
|---|---:|---:|
| Throw accuracy: landing far or beyond the Catch clamp | 28 | 19 % |
| Throw accuracy: mid-air collision | 24 | 16 % |
| Smooth miss, accuracy or Catch unresolved | 16 | 11 % |
| On-target ball bounced out or lost at the Catch | 27 | 18 % |
| Planner, lane stops, admissibility refusals, Leg separation | 32 | 21 % |
| Drive fault | 8 | 5 % |
| Ball Butler | 1 | 1 % |
| Unknown | 13 | 9 % |

- Throw accuracy accounts for 35 % strict and 46 % at the ceiling. Tails kill: colliding flights are 60 to 78 mm off a straight line; on 11-18 nine lost flights were 100 mm or more off (7 outward at the Ball 2 Site). Outward Catch authority is 10 mm (35 mm inward); a 15 mm test is owner-decided and not flown.
- Bounce-outs: 6 of 11 diagnosed entered 3 to 15 mm from the Cup axis; 10 of 11 were Ball 2. Two far Throws followed a late-seating Catch.
- Roughness shows as hardware stress, not Attempt ends: four Leg separations (BUG-72) followed executed Leg commands of 8,974 to 14,450 mm/s squared.

### 7. Jerk study (one Columns Attempt and one one-ball oval, 2026-11-20; small clean samples)

- Landing offset explains 10 % or less of peak acceleration or jerk; halving scatter would cut the p90 peak about 8 %.
- Columns: the Platform crosses about 110 mm in 0.27 s (fixed by flight time) at about 8.6 m/s squared lateral, matching the closed form 5.77 * Delta / T squared. 0.27 s is near one period of the 4.3 Hz ring (inferred).
- Oval: Platform 2.2 to 2.8 m/s squared; Hand acceleration p90 at 73 to 84 % of its 4500 rev/s squared limit.
- About 15 % of oval intervals (42 of 273) show roughly 5 times nominal jerk in `/leg_setpoint_echo`: either real acceleration steps at re-aim Splices or a dropped-echo artefact. `/leg_cmd_executed` would settle it. Unclassified.

### 8. Audit of the old plan's P5 and the Destination

- "Any Site" is not commandable: only `separation_mm` of a pair symmetric about the origin plus `columns_angle_deg`; there is no per-Site xy in `Juggle.action` or the GUI. The old P5 did not list this.
- `SkillExecutor.request_beat_change` (executor.py:5981) has one non-test caller, `sim/skills_gate.py:1890`; `skill_node.py` has none.
- Only the Columns family uses `compile_beats`; `compile_oval` does not exist; carried events are refused by name (schedule.py:1687-1689).
- `BEAT_YAW_NOT_SUPPORTED` at schedule.py:822 and :1507; no yaw in learner.py or plant_model.py.
- INVARIANTS has no D3 lateral-authority row, and I-BEAT-1 still says "until the plan's P2".
- Stale text: the body of `map/pathway-learner-generalisation.md` says "P2 and P5 not started"; the `Juggle.action` dwell_s comment still describes the box; `juggle-panel.js` has a `box:` label key.
- BUG-79's Dwell A/B check failed 2026-11-07 (Leg separation, BUG-72); `carry-ring-zvd.md` was parked 2026-11-23 with IDEA-27 as its unpark test.

## Discussion

### The withdrawn hypothesis and its decision rule

Before the release study the orchestrating session proposed that the scatter is "the Platform still settling at release after the lateral move": a loop of scatter, then transit, then a disturbed release, then more scatter. The decision rule was registered before looking: if measured release state explained most of the variance, a two-stage learner (command to release state to landing) would lead the plan. It explains 5 to 34 %, and transit explains nothing measurable. The jerk study kills the second half of the same hypothesis (scatter drives roughness: at most 10 %). The hypothesis does not survive, and the plan does not lead with a two-stage learner.

### "The correction is mostly noise" held only in part

The session's first reading of the counterfactual (29.7 against 29.2 mm) was that the residual correction is mostly noise. The shuffled-error control gives noise alone about 70 % of the production correction's median magnitude and about half its power. So the claim is partly right: much of what the Learner applies is chasing white error. But "about half" is not "all", and the replay shows a small real gain over model-only (20.97 against 21.71 mm on x). The honest statement is that the correction is mostly noise by magnitude and slightly useful by effect, and that the estimator could be quieter at no cost, which is what E2(a) is for.

### Why a better landing-space estimator cannot deliver efficacy

The predictability bound is the argument. Every pre-Throw column plus history features, cross-validated by Attempt and by day, explains at most 4.4 % of x and nothing of y or Apex. Between-day, between-Attempt and slow within-Attempt variance together are a few mm sd against a white 17 to 20 mm sd. Any law whose only input is the landing history and the command context is bounded by that ceiling, which is why four different laws tie to within 0.1 to 0.3 mm. The one measurable difference between them is command jitter (production 8.5 mm against shrinkage 6.2 mm on x, Apex 4.7 against 0.8): a quieter command, not a smaller error. E2 is therefore justified as correctness and quietness, and the plan says in its first lines that it is not expected to reduce scatter.

### The owner's unsettled-ball account, and why it fits

The owner (2026-11-24): there is no lateral play in the Hand carriage; the Cup is a rigid cone; a ball that has not settled can circle around inside it, usually for no more than about a second, long enough to matter in some Patterns. He strongly suspects this is the primary cause of Throws leaving in an unexpected direction.

It fits the data on four counts. The dominant term g is random from Throw to Throw (lag-1 |r| at most 0.11), as a circling ball's phase at release would be. It is invisible in the Platform's measured motion (the Platform is rigid and measured; the ball is not). It is not explained by commanded Cup state (R-squared 0.10 or less), by Hand velocity or position error, by release dz or seat (0.03 or less), and the launch state of the ball explains 78 to 87 % of the landing, so the error is born at the ball. And the two worst tails, colliding flights 60 to 78 mm off line and nine flights 100 mm or more off on 11-18, are consistent with a rare large circling radius (not tested).

It is an account that fits, not a measurement. What would refute it: a settle measure (ball motion relative to the Cup over the last 100 to 500 ms before release, Hand ball sensor state and settle time, time since the Catch) explaining under 10 % of g with AUC under 0.6 on 300 or more Throws, including carried Throws and Dwell 0.32 or less. What would support it: 30 % of g held out, or AUC 0.75 for the 60 mm tail. These are the pre-registered thresholds of step E0, with the in-between case sent to E1, a direct bench measurement. Two alternatives the data does not exclude: spin (inferred from the curvature variation, linked to g) and a rim contact at release; neither is separable from circling with these sensors.

### Why the Platform IMU cannot see the dominant term

The IMU is on the Platform, and the Platform's measured motion already explains only 5 to 34 % and none of g. The dominant term is the ball moving relative to a rigid Cup, which no Platform-frame sensor observes. Markers cannot easily go on the robot's Cup and an IMU cannot go on it; markers can go on the bench Hand, which is why E1 is a bench study. The IMU keeps a smaller, real job (E6): Platform vertical velocity at release against the Apex scatter and the carried overshoot (+34 / +57 / +111 mm at Dwell 0.50 / 0.40 / 0.35, unexamined, an inference), and the Columns Platform x-velocity error, the one partly learnable Platform term.

### What was ruled out

- **The Tracker as a scatter source:** it lands within 2 to 4 mm of the full-flight Mocap fit; Fit residual 0.31 mm rms; Memory landing and Mocap landing agree at R-squared 0.96 to 0.99.
- **Tilt error:** 0.06 degrees, about 3.7 mm of landing; the 0.78 correlation was the commanded Banking.
- **Release timing:** Hand velocity and position error and release dz explain 0.03 or less.
- **Transit as a scatter source:** the largest predictable piece, but about 1 mm rms; it explains nothing of the scatter, and it is not shown to be BUG-79's mechanism. It stays in the plan (E3) as a bias term for two-ball and for configuration differences.
- **A two-stage learner as the lead item:** the decision rule did not trigger.
- **Scatter as the cause of roughness:** at most 10 % of peak acceleration or jerk.

### Tradeoffs accepted

- The plan leads with finding the launch term (E0, E1) rather than with the Learner, although the Learner is the Pathway's name. The cost is that the Pathway's headline work (E2) is justified as correctness, not efficacy, and the plan states it up front.
- The old plan is not archived and its P5 is retired, not deleted: the owner's 2026-11-06 decision that the plan stays open until these have flown has not been revisited, and archiving is offered separately.
- A new plan beside the old one costs a second document; editing the old plan in place would have buried a verdict that reverses its premise ("P5 is the hardware proof").
- The Destination wording is wider than the new plan (a mid-Pattern operator channel and per-Site xy in the goal are not built and not planned). The plan says so plainly; narrowing the Destination is an owner decision not yet taken.

### Limits of the evidence

- Two Sittings carry the release decomposition (n=205 and n=129); five other Sittings, the IMU and carried Throws are not covered.
- Targets y_d are inferred, and Attempt boundaries are inferred from 120 s gaps.
- The jerk samples are small (one Columns Attempt, one oval run).
- The replay is a prediction-quality test with no confidence intervals, not a closed-loop simulation.
- Platform velocity errors are upper bounds (Mocap derivative noise 3 to 10 mm/s).
- Every number is an agent's, not independently re-verified.

## Owner decisions (2026-11-23 and 11-24)

- The heart is now learner efficacy: "the heart is getting the learner working better"; attempt at once and change mid-Pattern "are nice and I'm glad we have them".
- The scatter is "very largely plant, and it could (/should) be improved".
- A fresh plant per experiment was not deliberate; "happy to stop doing this".
- An arrival term: "wouldn't be a bad idea to add a term for this to our learner".
- Re-scope: keep the same Pathway (PWAY-2), written up as a new plan. Write-up and commit approved.

## What changes

- **New plan `plans/active/learner-efficacy.md`** (pathway learner-generalisation): E0 the unsettled-ball test from existing bags; E1 a bench Hand launch study; E2 a quiet, correct Learner (shrinkage estimator, release-kind split, far-Site guard, one Memory for the robot, the missing tests); E3 the transit term and yaw; E4 acting on the launch term (a decision gate, not a build); E5 Catch-side robustness (scope decision for the owner); E6 the Platform IMU's part; E7 close-out.
- **Old P5 retired** in `plans/active/learner-generalisation.md`: items 1 and 2 map to E3, item 3 (a mid-Pattern operator channel on the robot) is not pursued, items 4 and 5 map to E7, with item 4 now conditional on E2(b).
- **Items filed** for the three Learner flaws and the oval jerk question. Items filed: BUG-137 (far-Site guard, Defect B), BUG-138 (pooled plant model, Defect C), FEAT-65 (shrinkage estimator, Defect A), IDEA-29 (oval jerk intervals)
- The jerk finding is recorded as an Item, not a step: it is not a Learner matter.

## Withdrawn claims

- [2026-11-23] Claimed the scatter is the Platform still settling at release after the lateral move (scatter, transit, disturbed release, scatter).
  WITHDRAWN: measured release state explains 5 to 34 % of the variance, transit explains nothing measurable, and scatter explains at most 10 % of peak acceleration or jerk. The pre-registered rule (release state explaining most of the variance would put a two-stage learner first) did not trigger.
  Superseded by: findings 4 and 5 and the Discussion above.
- [2026-11-23] Claimed "the correction is mostly noise".
  PARTLY WITHDRAWN: the shuffled-error control gives noise 70 % of the correction's median magnitude and half its power. The correction is small and partly noise, not entirely noise.
  Superseded by: finding 4 and the Discussion above.

## Verification

This is an analysis entry with no code change, so there is no test result to cite for a behaviour. The numbers are the eight studies' own, produced 2026-11-23, and were not independently re-verified. The scripts and their outputs are local and gitignored: `temp/analysis/learner-review-2026-11-23/` in `~/Desktop/Jugglebot` (subfolders `predict/`, `release/`, `jerk/`, plus `a.py b.py c.py e.py` and the probe scripts), so a fresh clone does not hold them; E0 promotes the reusable extractors from `release/` to `tools/probes/`.

Documentation checks, 2026-11-24, `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py -q`: result recorded in the commit message. `tests/sim/test_logbook_index.py` is expected to fail until `python -m mapkit gen` regenerates `logbook/INDEX.md`, which the committing session does.

## Open Questions

- Does a settle measure explain g? E0 answers it from existing bags; E1 only if E0 lands between the thresholds.
- Which of the candidate actions for the launch term (the owner's Cup hardware change, a settle check before release, a Catch that damps circling) E4 should build, after E0 and E1.
- Whether E5 (Catch-side robustness, including the 15 mm outward authority test) belongs in this plan or in its own Items.
- Whether to narrow the Pathway's Destination wording, and whether to archive `learner-generalisation.md`.
- Whether the E2 gate as proposed (sequential replay rms no worse than production on every axis and command jitter at least 30 % lower) is the owner's gate.
