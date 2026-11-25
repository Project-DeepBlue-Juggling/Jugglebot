---
title: Learner efficacy - find the launch term and fix it at the Hand; Learner correctness; Catch robustness
created: 2026-11-24
status: active
owner: Harrison
last_updated: 2026-11-25
pathway: learner-generalisation
branch: main
scope: >-
  Fewer Attempts lost to Throw accuracy. Re-scoped 2026-11-25 after E0 (owner): the launch term is a
  property of the ball leaving the Cup, so the plan measures it on the bench Hand and fixes it in the Hand
  itself, judged against that bench number. The Learner keeps only its two correctness fixes; the transit
  term, yaw, the shrinkage estimator and the IMU study are retired from this plan and stay as Items.
steps:
  - {id: PE0, title: Unsettled-ball test from existing bags, kind: phase, status: complete, completed: 2026-11-25, effort: m, hardware: none}
  - {id: PE1, title: Bench Hand launch baseline, kind: phase, status: not_started, effort: s, hardware: bench-hand}
  - {id: PE2, title: Learner correctness fixes, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: PE3, title: The transit term and yaw, kind: phase, status: retired, date: 2026-11-25, effort: l, hardware: robot}
  - {id: PE4, title: Hand change against the bench baseline, kind: phase, status: not_started, effort: m, hardware: bench-hand}
  - {id: PE5, title: Catch-side robustness, kind: phase, status: not_started, effort: m, hardware: robot}
  - {id: PE6, title: The Platform IMU's part, kind: phase, status: retired, date: 2026-11-25, effort: m, hardware: robot}
  - {id: PE7, title: Close-out, kind: phase, status: not_started, effort: s, hardware: none}
related_logbook:
  - 2026-11-24-learner-efficacy-review-and-scatter-decomposition.md
  - 2026-11-25-learner-efficacy-e0-unsettled-ball-test.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/{learner,plant_model,memory,executor,schedule}.py (Learner.command at learner.py:296, the most-recent-k selection at learner.py:270, the exact-match key at learner.py:261, SkillExecutor._command_u, the yaw refusal BEAT_YAW_NOT_SUPPORTED at schedule.py:822 and :1507)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md (I-LEARN-1, I-BEAT-1)
  - tests/motion/test_plant_model.py (line 206 is the only far-Site guard test)
  - tools/probes/release_settle_bag_probe.py (E0's release-state and settle-measure extractor), temp/learn/jugglebot/memory_v2.csv
---

# Learner efficacy

Branch `main`. Follow-on to `plans/archived/learner-generalisation.md`, whose step P5 it replaces. Same
Pathway (PWAY-2, `map/pathway-learner-generalisation.md`). The narrative of the review that produced this
plan (eight read-only studies, 2026-11-23, one hypothesis withdrawn) is in
`logbook/2026-11-24-learner-efficacy-review-and-scatter-decomposition.md`; this file keeps decisions,
interfaces and status. Every number below is a study's own and was not independently re-verified.

## 1. Context

The question (owner, 2026-11-23): the front page said PWAY-2 was at "P5 Close-out". It was formally true
and substantively misleading: P5 held most of the Destination's hardware proof, and the Learner as built
cannot deliver the efficacy the owner wants. What stands (box retirement, the global plant model f0 with K
1.077-1.088 over 7 flying days, polygon_walk 25/25 on 2026-11-20) is not reopened.

What the review found:

- **The scatter is not predictable from what the Learner sees.** Over 2,051 live Memory rows (CV grouped by
  Attempt and by day) the best held-out R2 of the model-only error from every pre-Throw column plus history
  is x +0.044, y -0.016, Apex -0.028: a ceiling of 2-4 % rms on x and nothing on y or Apex. Landing error sd
  is x 21.1 / y 20.3 mm, Apex robust sd 15 mm, flat by Throw index (27.0 mm at Throw 1, 27.1 mm from Throw 21).
  Without the residual correction lateral rms is 29.7 mm against 29.2 mm actual.
- **Variance by source** (mm sd, x / y / Apex): between-day 1.5 / 2.0 / 0.4; between-Attempt 1.8 / 1.4 / 3.6;
  between-configuration 6.9 / 3.8 / 4.4; slow within-Attempt about 0; white 19.9 / 19.3 / 17.2.
- **Release-state decomposition from bags** (held-out R2 of lateral landing error x / y within Attempt by
  Site groups; n=205 Self-throw 2026-11-06, n=129 Columns 2026-11-20):

| Covariate set | 11-06 | 11-20 |
|---|---|---|
| Mocap full-flight landing to Memory landing (Tracker check) | 0.96 / 0.95 | 0.99 / 0.98 |
| Ball launch state (release position + launch velocity) | 0.82 / 0.78 | 0.85 / 0.87 |
| Commanded Cup position + velocity at release | 0.10 / -0.04 | 0.02 / -0.06 |
| Measured Platform error (Mocap velocity, tilt, tilt rate vs command) | 0.16 / 0.05 | 0.34 / 0.26 |
| Hand velocity/position error; release dz; transit; seat | <= 0.03 | <= 0 |
| g: ball launch velocity minus (commanded Cup + measured Platform error) | 0.50 / 0.69 | 0.35 / 0.42 |
| Command + Platform error + g + release position | 0.93 / 0.88 | 0.93 / 0.92 |

  g is about 16 mm/s per axis (about 14 mm of landing), random (lag-1 |r| <= 0.11) and invisible in the
  Platform's measured motion. The Tracker is cleared (2-4 mm of the full-flight fit). The hypothesis that the
  Platform is still settling at release after the lateral move was withdrawn: release state explained 5-34 %,
  transit nothing measurable, and the pre-registered rule (if release state explained most of the variance a
  two-stage learner would lead) did not fire.
- **Limits of the evidence.** Two Sittings carry the release decomposition (n=205, n=129); five others, the IMU
  and carried Throws are not covered; targets y_d are inferred and Attempt boundaries come from 120 s gaps; the
  replay is a prediction-quality test with no confidence intervals; the Platform velocity errors are upper bounds.
- **Owner's physical account (2026-11-24).** No lateral play in the Hand carriage; the Cup is a rigid cone; a
  ball that has not settled can circle inside it, usually for no more than about a second. The owner strongly
  suspects this is the primary cause of Throws leaving in an unexpected direction, and has Cup hardware ideas
  (not specified). Markers cannot easily go on the robot's Cup and an IMU cannot go on the Cup; markers can go
  on the bench Hand. This is a hypothesis, untested: E0 and E1 test it.
- **Attempt-end census** (149 ends: four 2026-11-06 sets, 11-07, 11-18): Throw accuracy 35 % strict, 46 %
  ceiling (landing far or beyond the Catch clamp 19 %, mid-air collision 16 %, smooth miss unresolved 11 %);
  on-target ball bounced out or lost at the Catch 18 %; planner, lane stops, admissibility refusals and Leg
  separation 21 %; drive fault 5 %; unknown 9 %; Ball Butler 1 %. Tails kill: colliding flights 60-78 mm off a
  straight line, nine lost flights >= 100 mm off on 11-18.
- **Learner defects** (code review, file:line in the logbook entry): A, the 16 most recent rows rather than
  the nearest 16 (49 % of queries get under half the evidence nearest-k would give); B, the far-Site guard
  fails on long single-Site memories (b forgets, s does not; past about 150 rows an invented slope is
  applied, -18.7 mm at x = +0.125 on a real oval-only memory); C, f0 pools carried and free_cup rows (607
  oval rows move the applied offset at the +62.5 mm Site from -3.3 to +4.8 mm). Memories are siloed per plant
  (23 under `temp/learn`), so cross-Pattern sharing has never happened on hardware.
- **Jerk is not a Learner matter**: landing offset explains <= 10 % of peak acceleration or jerk. Recorded as
  an Item (IDEA-29), not a step.

## 2. Owner decisions 2026-11-23/24

- The heart is now learner efficacy: "the heart is getting the learner working better"; attempt at once and
  change mid-Pattern "are nice and I'm glad we have them".
- The scatter is "very largely plant, and it could (/should) be improved".
- A fresh plant per experiment was not deliberate; "happy to stop doing this".
- An arrival term: "wouldn't be a bad idea to add a term for this to our learner".
- Re-scope: keep the same Pathway (PWAY-2), written up as a new plan. Write-up and commit approved.

**2026-11-25, after E0.** "It might not be worth going too deep down this rabbit hole ... the true solution
here would be a fix to the hand itself." The plan is cut to what can move repeatability: a bench baseline
(E1), a Hand change judged against it (E4), the Catch (E5), and the Learner's two correctness bugs (E2).
The owner's account stands as his prediction for E1: the scatter comes mostly from the ball not seating
properly during the Dwell, so a hand-placed, settled ball should show little of it. E0's reading predicts the
opposite (the term arises at separation, and Throws from rest on the robot carry about the same g). E1 is
built to tell the two apart.

## 3. What this plan does not do

- **A mid-Pattern operator channel on the robot** and **per-Site xy**: unbuilt by owner priority (2026-11-23).
- **The transit term, yaw, the shrinkage estimator and the IMU study** (retired 2026-11-25, see E3, E6 and
  E2): none can be felt while the scatter is about 20 mm sd. They stay as Items (BUG-79, IDEA-12, IDEA-18,
  FEAT-65, IDEA-27) and are worth reopening once the launch scatter is down.
- **The Pathway's Destination is now wider than this plan.** It still names bias "for carried Throws, new
  configurations and yaw"; with E3 retired, completing this plan does not complete it. Whether to narrow
  the Destination again or leave those as its remaining Items is an owner decision (§ 7).
- Reducing scatter by a better landing-space law: the ceiling is 2-4 %.
- Jerk and Leg-separation roughness (BUG-72). (The 15 mm outward-authority trial is in scope, under E5.)

## 4. Phase summary

Step ids: the frontmatter ids are `PE0` to `PE7` (a constraint of the plan tooling); prose, the logbook entry and the Items say E0 to E7 for the same steps.

| Phase | Scope | Hardware | Status | Decides |
|---|---|---|---|---|
| E0 | Unsettled-ball test from existing bags | none | COMPLETE 2026-11-25: BETWEEN, leaning against | what E1 asks |
| E1 | Bench Hand launch baseline (three arms) | bench-hand | NOT STARTED, next | seating or separation; E4's acceptance number |
| E2 | Learner correctness fixes (BUG-137, BUG-138, one Memory) | none | NOT STARTED | nothing downstream; independent |
| E3 | The transit term and yaw | robot | RETIRED 2026-11-25 | - |
| E4 | Hand change against the bench baseline | bench-hand, then robot | NOT STARTED | whether the launch term is cut |
| E5 | Catch-side robustness | robot | NOT STARTED | whether the 15 mm authority and the bounce-out causes need Items |
| E6 | The Platform IMU's part | robot | RETIRED 2026-11-25 | - |
| E7 | Close-out | none | NOT STARTED | archive of this plan |

Order: E1, then E4. E2 needs no hardware and can run at any time. E5 can follow E1 at any point.

## 5. Phases in detail

### Phase E0: Unsettled-ball test from existing bags — COMPLETE 2026-11-25

**Purpose.** Without it E1 and E4 are built on the owner's strong suspicion alone. It prevents a Cup
hardware change aimed at a term that the existing data would have shown to be elsewhere.

**Preconditions.** None. Bags and scripts: `temp/analysis/learner-review-2026-11-23/release/` in the
checkout `~/Desktop/Jugglebot` (local, gitignored).

**Work.** Test whether ball motion relative to the Cup before release (Mocap ball marker in the
Platform/Cup frame over the last 100-500 ms, Hand ball sensor state and settle time, time since the Catch,
Dwell) predicts g and the >= 60 mm tails. Promote the reusable extractors to `tools/probes/` per
`tools/probes/README.md`. Known limit: rim occlusion of the marker in the Cup (the 2026-11-23 check of the
last 35 ms gave r 0.09-0.48 with g, inconclusive).

**Criterion (pre-registered in the review).** SUPPORTED if a settle measure explains >= 30 % of g's variance
held out, or separates the >= 60 mm tail from the rest at AUC >= 0.75. NOT SUPPORTED if < 10 % and
AUC < 0.6 on >= 300 Throws including carried Throws and Dwell <= 0.32. Between the two: go to E1 for a direct
measurement.

**Hardware and safety.** None; read-only. The 2026-11-23 sample had no carried Throws and did not cover
Sittings 11-07, 11-08, 11-10, 11-13, 11-18, which is why the >= 300 Throws including carried is required.

**Decides.** SUPPORTED: E1 measures the size, E4 is aimed at settling. NOT SUPPORTED: g stays unexplained,
E1 is dropped or becomes a pure seat-sensitivity study, E4 reconsiders (in-flight curvature, spin).

**Outcome (2026-11-25): BETWEEN, leaning NOT SUPPORTED.** 28 settle measures on 3,950 Throws from 21 bags
(3,125 with g; 1,154 carried, 1,971 at Dwell <= 0.32), Sitting 2026-11-18 held out
(`logbook/2026-11-25-learner-efficacy-e0-unsettled-ball-test.md`). No measure reaches SUPPORTED: best on the
held-out Sitting 4 % of g (Memory seat offset), a 10-12 % variance drop (Catch entry radius), AUC 0.72 on 9 far
Throws. Every velocity and offset measure is at zero (-0.6 to +2 % of g): in the last 35 ms the ball's centre moves against the Cup at
2.6 / 3.6 mm/s, the noise floor of a rested ball, against a g of 18.0 / 17.1 mm/s, so a circling centre is not the
source of g. Not NOT SUPPORTED by the letter: Throws from rest show 27 % less g variance than running Throws of
matching configuration (95 % CI 2-46 %, n=81, median unchanged; pooled across Sittings, and reversed in the two
Sittings that hold both kinds in one configuration), and off-axis Catches are followed by slightly
larger g. Separate result: g appears in the 35 ms after the announced release (the ball's velocity against the
Cup there explains 20-22 % of g; before release, 0 %). **Phase E1 cleared to start**, re-scoped
2026-11-25 as a three-arm bench baseline that tests the owner's seating account against this reading. On
the bags a perfect settle fix is bounded at about a quarter of g's variance (17 to about 14.5 mm/s).

### Phase E1: Bench Hand launch baseline

**Purpose.** A number for the launch scatter of the current Cup, and a test that tells the owner's account
(poor seating during the Dwell) from E0's reading (the release contact at separation). Without it a Hand
change cannot be judged: Attempt length is too noisy to show whether a Cup change worked.

**Preconditions.** The bench Hand in the Mocap cameras' view (owner, 2026-11-24). **No markers on the Cup are
needed**: the bench Hand does not move sideways, so the ball's lateral launch velocity from Mocap is the
launch term directly. The measure is the one E0 used (linear fit 0.035 to 0.2 s after release;
`tools/probes/release_settle_bag_probe.py` holds the fit).

**Work.** Three arms at one Apex, about 30 Throws each, the ball's lateral launch velocity sd per axis:
- (a) hand-placed and settled, thrown after at least 2 s;
- (b) dropped in from about Catch height and thrown after a Dwell of about 0.27 s;
- (c) placed a few mm off-axis (or dropped in off-axis) and thrown after about 0.27 s.

**Criterion (pre-registered 2026-11-25, before any bench Throw).** On the robot g is about 17 mm/s per axis.
- **Seating (the owner's account):** arm (a) at 6 mm/s or less AND arm (b) or (c) at twice arm (a)'s
  variance or more. E4 then targets seating: a Cup that seats the ball faster, or a Catch that lands it
  on-axis.
- **Separation (E0's reading):** arm (a) at 12 mm/s or more, with (b) and (c) within 1.5 times its
  variance. E4 then targets the release: Cup geometry and surface at the contact, Hand deceleration.
- **Neither:** arm (a) at 6 mm/s or less and (b), (c) no worse means the term needs the robot (Platform
  motion, Banking, the real Catch); a bench Cup change alone is then the wrong bet and E4 moves to the
  robot. Anything between the thresholds is reported as mixed, with both shares.
With 30 Throws an sd is known to about 13 %, so 6 against 12 mm/s is resolved; a variance ratio of 2 is
marginal at 30 and needs about 60 per arm if it lands near the threshold.

**Hardware and safety.** **The bench Hand's stroke is SHORTER than the robot's with the same current cap
(`stroke_cut` checks only the production clip): get the top stop first**, before any Throw. One bridge board
serves bench and robot; run `live_session` (`tools/lib/live_session.sh`) first and never flash the
bench-sysid firmware to the robot. Single-owner UDP on the bench. Run it as a Sitting with a `## Sitting`
section on the filed Item.

**Decides.** What E4 changes, and the number each Hand iteration is judged against. A bench result is a
transfer claim to the robot until E4 confirms it there.

### Phase E2: Learner correctness fixes

**Purpose.** Two measured faults make the Learner wrong in ways that do not depend on scatter: the far-Site
guard can apply a slope with no rows behind it (-18.7 mm at x = +0.125 on a real oval-only Memory), and oval
rows move Columns commands (the applied offset at the +62.5 mm Site moves from -3.3 to +4.8 mm).

**Preconditions.** None. **Safety-bearing: this changes commanded u.**

**Work.**
- (a) BUG-137. Fix the far-Site guard and pin it with a long single-Site test, at 60 rows and past 150.
- (b) BUG-138. Split or term the plant model by release kind: theta must not be pooled across carried and
  free_cup rows.
- (c) One Memory for the robot: plant `jugglebot` always, a new plant only when hardware physically changes
  (ball type, Hand motor, recalibration, **a new Cup from E4**); experiments load the main Memory and write
  to a scratch copy.
- (d) Tests for the two fixes: a long single-Site memory against the guard, leakage between release kinds.

Retired from this step 2026-11-25: the shrinkage estimator (FEAT-65, stays an Item; it buys a quieter
command, not accuracy) and the transfer, out-of-family and closed-loop tests that went with it.

**Control-implication walk-through (required before the code lands).** One Throw cycle step by step:
`executor._command_u` builds the context, `Learner.command` returns u_m = f0^-1(y_d) plus the residual
clipped about u_m (cap 30 mm lateral, 20 mm Apex), the executor installs u in the Segment's release. Both
fixes change u_m only. They must keep (i) identity at an empty Memory, (ii) the cap about the MODEL command
(I-LEARN-1), (iii) install-time `validate_cycle` and the Teensy `MAX_DEVIATION` guard on the path. Failure
mode to test: a step in u between consecutive Throws when the guard switches. Nothing in the 40 Hz tick
changes; the Learner runs at command time.

**Criterion (proposed, owner to confirm).** Both new tests pass; sequential replay rms no worse than
production on every axis; sim gate green; first flown as a normal Columns Attempt.

**Hardware and safety.** None to build. `./run_tests.sh --full` before the commit, the sim gate
(`sim/skills_gate.py`) green, and a dress rehearsal against the live `/robot_state` on the loaded Jetson
before any Sitting that flies it. Wrap every run in `flock /tmp/jugglebot-tests.lock`.

**Decides.** Nothing downstream; E7's I-LEARN-1 row describes the result.

### Phase E3: The transit term and yaw — RETIRED 2026-11-25

Retired by the re-scope (owner, 2026-11-25). The expected gain is a bias of 9-15 mm on two-ball Throws under
about 20 mm sd of scatter, which barely moves the rms. BUG-79, IDEA-12 and IDEA-18 stay open as Items;
`BEAT_YAW_NOT_SUPPORTED` (schedule.py:822, :1507) stays. **What would reopen it:** E4 cutting the launch
scatter enough that the bias is a visible share of the landing error.

### Phase E4: Hand change against the bench baseline

**Purpose.** The only step in this plan that can cut the launch scatter, which is 35-70 % of lateral landing
variance: removing it would take about 20 mm sd to 11-16 mm.

**Preconditions.** E1's result and number.

**Work.** The owner's Hand or Cup change (design his; E1 says whether it should target seating or the
release). Each iteration repeats the E1 arm that showed the scatter, on the bench, and is kept or dropped on
that number. A kept change then flies on the robot as a normal Columns Attempt, and
`release_settle_bag_probe.py` on that bag gives g for comparison with E0's 17 mm/s.

**Criterion.** To be pre-registered here after E1 and before the first iteration (a proposed form: the
bench launch sd at least halved, and g on the robot down by at least a third).

**Hardware and safety.** Bench Hand first (the E1 cautions apply), then the robot. A new Cup changes the
ball's seat height and release point: re-check the Hand top stop and the Catch plane before the first robot
Throw, and start a new plant Memory (E2(c)).

**Decides.** Whether the launch term is cut; whether E3 is worth reopening.

### Phase E5: Catch-side robustness

**Purpose.** Scatter that is random has to be absorbed at the Catch, or it keeps ending Attempts: outward
Catch authority is 10 mm (35 mm inward) against about 20 mm sd, and on-target bounce-outs are 18 % of ends
(6 of 11 diagnosed entered 3-15 mm from the Cup axis; 10 of 11 were Ball 2; two far Throws followed a
late-seating Catch). If E1 supports the owner's seating account this step also feeds E4: an on-axis Catch
is then a cause of a clean Throw, not only of a held ball.

**Preconditions.** None beyond E1's result.

**Work.** The 15 mm outward-authority trial (owner-decided, not flown) and the bounce-out causes.

**Hardware and safety.** Robot; the Catch clamp is a safety-adjacent bound, so a change is analysed against
`validate_cycle` and the Teensy `MAX_DEVIATION` guard first.

**Decides.** Whether these become Items.

### Phase E6: The Platform IMU's part — RETIRED 2026-11-25

Retired by the re-scope (owner, 2026-11-25): the IMU cannot see the ball against the Cup, and E0 placed the
launch term there. IDEA-27 (the ring at Dwell 0.40) stays on `plans/parked/carry-ring-zvd.md`. **What would
reopen it:** an Apex-scatter question, which this plan no longer carries.

### Phase E7: Close-out

**Purpose.** Leave the normative rows true to what was built.

**Preconditions.** E2 landed; E4 decided.

**Work.** The INVARIANTS rows: the D3 lateral-authority row (the authority is a Learner bound, never an
actuator bound; widening c_max is safe only while install-time `validate_cycle` and the Teensy
`MAX_DEVIATION` guard stay on the path), the stale I-BEAT-1 yaw clause re-pointed to IDEA-12, an I-LEARN-1
update for E2's fixes and its "verified at the plan's P5" clause. The oval on the shared Memory (old P5
item 4) is not flown under this plan: it needs `compile_oval` on `compile_beats`, which does not exist, and
goes to an Item with E2(b) as its precondition.

**Hardware and safety.** None; the INVARIANTS edits are normative-doc edits and go through
`/audit --unstaged`.

**Decides.** Archive of this plan.

## 6. Mapping from the old plan's P5

| Old P5 item | Disposition |
|---|---|
| 1 Arrival pull (BUG-79) | Retired with E3 (2026-11-25); the Item stays open |
| 2 Per-beat yaw (IDEA-12) | Retired with E3 (2026-11-25); the Item stays open |
| 3 Mid-Pattern change on the robot | Not pursued (owner priority 2026-11-23); see § 3 |
| 4 The oval on the shared Memory | To an Item at E7, conditional on E2(b) |
| 5 INVARIANTS rows | E7 |

The old plan's stance "stays open until these have flown" (owner, 2026-11-06) was reversed by the owner on
2026-11-24: the old plan is archived as superseded in part by this one.

## 7. Open questions

1. The Destination of PWAY-2 still names bias for carried Throws, new configurations and yaw, which this plan
   no longer delivers (§ 3). Narrow it, or leave those as the Pathway's remaining Items?
2. E4's criterion (proposed: bench launch sd at least halved, g on the robot down by at least a third) is
   to be fixed after E1.
3. E2's replay gate (rms no worse on every axis) comes from a prediction-quality replay, not a closed-loop
   simulation; owner to confirm.

Answered 2026-11-24 (owner): Mocap will cover the bench Hand (E1); E5 stays in this plan;
`learner-generalisation.md` is archived. Answered 2026-11-25 (owner): the re-scope of § 2.

## 8. Residue inherited from `plans/archived/learner-generalisation.md`

Open at that plan's archival (2026-11-24) and carried by no step above. Each is a candidate Item, not
scheduled work.

1. **D9, a measured overrun in place of the fixed 10 % margin.** `MARGIN_FRAC` 0.9 is the online acceptance
   criterion; replacing it with a per-limit-class tracking overrun measured from bags was left as a follow-up
   Item and never filed.
2. **P3a slack.** Whether the early start (`admissibility_early_command`, on by default) or the literal slot
   flies first; what buys slack when a check spills past the dispatch; a `--kind check` run of
   `loaded_budget.py` under a launch.
3. **The carried oval's release contact.** Whether the carried release shows the release-contact
   discontinuity, and whether the plant model needs release-velocity terms for it. E0 found the same g for
   carried and free_cup Throws (15-16 against 16-19 mm/s).
4. **Cosmetic:** the `box:` label key in `ros_ws/gui/js/juggle-panel.js` and the `dwell_s` comment in
   `Juggle.action` that still describes the box.

