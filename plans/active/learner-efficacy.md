---
title: Learner efficacy - find the launch term, quiet the Learner, carry the old close-out
created: 2026-11-24
status: active
owner: Harrison
last_updated: 2026-11-24
pathway: learner-generalisation
branch: main
scope: >-
  Fewer Attempts lost to Throw accuracy. Find the launch term (the ball's motion against the Cup at release)
  at its source, make the Learner quiet and correct at what a landing-space learner can do (bias in unusual
  contexts), and carry the items of learner-generalisation.md's retired P5 that still matter (the transit
  term, yaw, the oval on the shared Memory, the INVARIANTS rows). Follows the 2026-11-23/24 review, which
  found the scatter is mostly not predictable from anything the Learner sees.
steps:
  - {id: PE0, title: Unsettled-ball test from existing bags, kind: phase, status: not_started, effort: m, hardware: none}
  - {id: PE1, title: Bench Hand launch study, kind: phase, status: not_started, effort: m, hardware: bench-hand}
  - {id: PE2, title: A quiet and correct Learner, kind: phase, status: not_started, effort: l, hardware: none}
  - {id: PE3, title: The transit term and yaw, kind: phase, status: not_started, effort: l, hardware: robot}
  - {id: PE4, title: Act on the launch term, kind: phase, status: not_started, effort: m, hardware: robot}
  - {id: PE5, title: Catch-side robustness, kind: phase, status: not_started, effort: m, hardware: robot}
  - {id: PE6, title: The Platform IMU's part, kind: phase, status: not_started, effort: m, hardware: robot}
  - {id: PE7, title: Close-out, kind: phase, status: not_started, effort: s, hardware: robot}
related_logbook:
  - 2026-11-24-learner-efficacy-review-and-scatter-decomposition.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/{learner,plant_model,memory,executor,schedule}.py (Learner.command at learner.py:296, the most-recent-k selection at learner.py:270, the exact-match key at learner.py:261, SkillExecutor._command_u, the yaw refusal BEAT_YAW_NOT_SUPPORTED at schedule.py:822 and :1507)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md (I-LEARN-1, I-BEAT-1)
  - tests/motion/test_plant_model.py (line 206 is the only far-Site guard test)
  - tools/probes/ (E0 promotes the release-state extractors here), temp/learn/jugglebot/memory_v2.csv
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

## 3. What this plan does not do

- **A mid-Pattern operator channel on the robot** (old P5 item 3) and **per-Site xy** ("any Site" is not
  commandable today: only `separation_mm` of a pair symmetric about the origin plus `columns_angle_deg`).
  Both stay unbuilt by owner priority (2026-11-23).
- **The Pathway's Destination was narrowed to this plan's scope on 2026-11-24 (owner).** It no longer names
  "any Site" or a change mid-Pattern, so completing this plan completes the Pathway's Destination.
- Reducing scatter by a better landing-space law: the ceiling is 2-4 %. E2 buys quiet and correctness, not
  accuracy.
- Jerk and Leg-separation roughness (BUG-72). (The 15 mm outward-authority trial is in scope, under E5.)

## 4. Phase summary

Step ids: the frontmatter ids are `PE0` to `PE7` (a constraint of the plan tooling); prose, the logbook entry and the Items say E0 to E7 for the same steps.

| Phase | Scope | Hardware | Status | Decides |
|---|---|---|---|---|
| E0 | Unsettled-ball test from existing bags | none | NOT STARTED | whether E1 is needed; what E4 targets |
| E1 | Bench Hand launch study | bench-hand | NOT STARTED | the seat sensitivity and circling time; E4's candidates |
| E2 | A quiet and correct Learner (defects A, B, C; one Memory; tests) | none | NOT STARTED | the estimator E3 extends |
| E3 | The transit term and yaw (old P5 items 1, 2) | robot | NOT STARTED | E7's oval row |
| E4 | Act on the launch term (decision gate) | robot | NOT STARTED | design after E0/E1 |
| E5 | Catch-side robustness | robot | NOT STARTED | whether the 15 mm authority and the bounce-out causes need Items |
| E6 | The Platform IMU's part | robot | NOT STARTED | IDEA-27, carried overshoot |
| E7 | Close-out (old P5 items 4, 5) | robot | NOT STARTED | archive of this plan |

Recommended order is E0 to E7. E0 and E2 need no hardware and can run in parallel; E2 must not wait on
E0/E1 because defects A to C are measured Learner faults independent of the launch term.

## 5. Phases in detail

### Phase E0: Unsettled-ball test from existing bags

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

### Phase E1: Bench Hand launch study

**Purpose.** Rim occlusion limits what bags can show about a ball inside the Cup. A rigid-body marker set on
the bench Hand's Cup measures the ball-versus-Cup term directly, without the occlusion problem.

**Preconditions.** E0 result of "between" or SUPPORTED (a NOT SUPPORTED result may still justify a short
seat-sensitivity run, owner's call). Mocap coverage (owner, 2026-11-24): when the markers go on, the bench
Hand is moved into the Mocap cameras' view.

**Work.** Markers on the bench Hand's Cup as a rigid body; bench Throws with the ball (a) hand-placed and
settled, (b) placed a few mm off-axis, (c) released while circling. Measures: launch-direction sd settled
versus unsettled, the seat sensitivity (mm/s of lateral launch velocity per mm of offset), how long
circling lasts.

**Hardware and safety.** **The bench Hand's stroke is SHORTER than the robot's with the same current cap
(`stroke_cut` checks only the production clip): get the top stop first**, before any Throw. One bridge board
serves bench and robot; run `live_session` (`tools/lib/live_session.sh`) first and never flash the
bench-sysid firmware to the robot. Single-owner UDP on the bench. Run it as a Sitting with a `## Sitting`
section on the filed Item.

**Decides.** The seat sensitivity and circling time set which E4 candidate is worth building; a bench
result is a transfer claim to the robot until E4 confirms it there.

### Phase E2: A quiet and correct Learner

**Purpose.** Three measured faults make the Learner noisier and less correct than it needs to be, none of
which reduces scatter: A makes the command jitter (8.5 / 5.7 / 4.7 mm production command jitter x / y / Apex
against 6.2 / 5.5 / 0.8 for shrinkage), B can apply a slope with no rows behind it, C lets oval rows move
Columns commands.

**Preconditions.** None to start. **Safety-bearing: this changes every commanded u.**

**Work.**
- (a) FEAT-65. Replace the 16-most-recent mean with a noise-aware shrinkage estimator over all in-kernel rows (kernel
  mean, gain n_eff / (n_eff + sigma_noise^2 / sigma_prior^2), sigma_prior 10 mm in the replay) (closes defect A).
- (b) BUG-138. Split or term the plant model by release kind (defect C): theta must not be pooled across carried and
  free_cup rows.
- (c) BUG-137. Fix the far-Site guard and pin it with a long single-Site test, at 60 rows and past 150 (defect B).
- (d) One Memory for the robot: plant `jugglebot` always, a new plant only when hardware physically changes
  (ball type, Hand motor, recalibration); experiments load the main Memory and write to a scratch copy.
- (e) The missing tests: transfer (learn at A, first-Throw error at B), an out-of-family plant, long
  single-Site memory against the guard, leakage between release kinds, a closed-loop mid-Pattern change.

**Control-implication walk-through (required before the code lands).** One Throw cycle step by step:
`executor._command_u` builds the context, `Learner.command` returns u_m = f0^-1(y_d) plus the residual
clipped about u_m to c(1 - exp(-sum w / 2)) (cap 30 mm lateral, 20 mm Apex), the executor installs u in the
Segment's release. The estimator changes only the residual's value; it must keep (i) identity at an empty
Memory, (ii) the cap about the MODEL command (I-LEARN-1), (iii) install-time `validate_cycle` and the Teensy
`MAX_DEVIATION` guard on the path. Failure modes to test: a step in u between consecutive Throws when a row
enters or leaves the kernel (the shrinkage gain must vary continuously), and Throw 1 at an empty Memory.
Nothing in the 40 Hz tick changes; the Learner runs at command time.

**Criterion (proposed, owner to confirm).** Sequential replay rms no worse than production on every axis and
command jitter at least 30 % lower; sim gate green; first flown as a normal Columns Attempt. Not expected to
reduce scatter (ceiling 2-4 %); the replay differences between laws are 0.1-0.3 mm with no confidence
interval, so do not promise more than quiet. The gate numbers themselves are an open question (§ 7).

**Hardware and safety.** None to build. `./run_tests.sh --full` is required before the commit (the
`nightly` tier covers `controller/` and `sim/` paths and the default gate cannot see it), the sim gate
(`sim/skills_gate.py`) green, and a dress rehearsal against the live `/robot_state` on the loaded Jetson
before any Sitting that flies it. Wrap every run in `flock /tmp/jugglebot-tests.lock`.

**Decides.** The estimator E3 extends with a transit term and E7's INVARIANTS row for I-LEARN-1 describes.

### Phase E3: The transit term and yaw

**Purpose.** Two-ball Throws land pulled toward the partner Site (BUG-79). A term in a frame that turns with
the Pattern is the only form that can learn it on a pair whose angle changes each Beat, and yaw is refused by
name until it exists. Old P5 items 1 and 2.

**Preconditions.** E2 landed (the term extends the new estimator, not the one being replaced). The
arrival pull's Dwell A/B check failed 2026-11-07 (Leg separation, BUG-72); `carry-ring-zvd.md` is parked
with IDEA-27 as its unpark test, so do not plan a flight that repeats the refused row.

**Work.** Add the arrival pull as a transit vector (actual previous Catch or rest position to release
position, with its duration), so it turns with the Pattern and from-rest is the zero-transit case; it covers
IDEA-18's carried-versus-from-rest gap in the same term if the data supports it. Memory schema v3
(`arrived_from`; stored rows reconstructed, never defaulted to zero). Then lift `BEAT_YAW_NOT_SUPPORTED` (schedule.py:822, :1507) and re-point I-BEAT-1.

**Criterion (carried from the old P5, proposed, owner to confirm).** On two-ball Columns the pull toward the
partner, fitted against the real release, is within 2 SE of the one-ball fit; a Columns goal with a
per-Beat yaw is nominated and flown and its lateral landing error is not distinguishable from the same pair
flown unturned in the same Sitting.

**Hardware and safety.** Robot. Expected gain is a bias (about 9-15 mm two-ball, about 7 mm sd between
configurations on x), not scatter. Changes commanded u: same gate as E2. Dress rehearsal first, every refusal
reported at once.

**Decides.** Whether the oval can share the Memory (E7).

### Phase E4: Act on the launch term

**Purpose.** A decision gate, not a build: it prevents building a settle check or a Cup change for a term E0
and E1 did not establish.

**Preconditions.** E0 and E1 results. Design TBD.

**Work.** Candidates, none chosen: the owner's Cup hardware change (unspecified); a settle check before
release (Hand ball sensor) that holds or flags an unsettled Throw; a Catch that damps circling. Each is
costed against the E1 measurements and the census (Throw accuracy is 35-46 % of ends).

**Criterion.** TBD after E0/E1; to be pre-registered in this section before any hardware flight.

**Hardware and safety.** Robot. A hold before release changes Dwell timing on the shared CAN wall clock and
any held Throw must stay rest-terminal; analyse the timing effect before building.

**Decides.** The plan's next build, or that the launch term is not addressable here.

### Phase E5: Catch-side robustness

**Purpose.** Scatter that is random has to be absorbed at the Catch, or it keeps ending Attempts: outward
Catch authority is 10 mm (35 mm inward) against about 20 mm sd, and on-target bounce-outs are 18 % of ends
(6 of 11 diagnosed entered 3-15 mm from the Cup axis; 10 of 11 were Ball 2; two far Throws followed a
late-seating Catch).

**Preconditions.** None beyond E0's result. Scope (owner, 2026-11-24): E5 stays in this plan.

**Work.** The 15 mm outward-authority trial (owner-decided, not flown) and the bounce-out causes.

**Hardware and safety.** Robot; the Catch clamp is a safety-adjacent bound, so a change is analysed against
`validate_cycle` and the Teensy `MAX_DEVIATION` guard first.

**Decides.** Whether these become Items.

### Phase E6: The Platform IMU's part

**Purpose.** The IMU cannot see the dominant term (ball against Cup); this phase bounds what it can see so it
is not oversold.

**Preconditions.** Platform FW 15 captures only (BUG-132: earlier captures stamped sync wrongly).

**Work.** Measure Platform vertical velocity at release against Apex scatter and the carried overshoot
(+34 / +57 / +111 mm at Dwell 0.50 / 0.40 / 0.35; unexamined, an inference); sharpen the Columns Platform
x-velocity error (the one partly learnable Platform term, R2 0.27); IDEA-27 (ring at Dwell 0.40).

**Hardware and safety.** Robot, Armed-upload slowdown applies.

**Decides.** Whether an IMU-based feedforward or term is worth building.

### Phase E7: Close-out

**Purpose.** Retire the old plan's remaining promises so the Pathway is not left claiming proof it lacks.

**Preconditions.** E2 landed; E3 decided.

**Work.** (1) The oval on the shared Memory (old P5 item 4): Columns and the oval fly on one Memory
on hardware; rows stay separated by `release_kind` (no carried row in a `free_cup` query's support). The
old criterion "theta is shared" is now CONDITIONAL on E2(b): pooled as written it contaminates Columns
(defect C). Precondition: `compile_oval` targets `compile_beats` (does not exist today). Discharges I-LEARN-1's
"verified at the plan's P5" clause. (2) The INVARIANTS rows (old item 5): the D3 lateral-authority row (the
authority is a Learner bound, never an actuator bound; widening c_max is safe only while install-time
`validate_cycle` and the Teensy `MAX_DEVIATION` guard stay on the path), the stale I-BEAT-1 yaw clause
re-pointed, an I-LEARN-1 update for the new estimator.

**Hardware and safety.** Robot for the oval flight; the INVARIANTS edits are normative-doc edits and go
through `/audit --unstaged`.

**Decides.** Archive of this plan.

## 6. Mapping from the old plan's P5

| Old P5 item | Disposition |
|---|---|
| 1 Arrival pull (BUG-79) | E3 |
| 2 Per-beat yaw (IDEA-12) | E3 |
| 3 Mid-Pattern change on the robot | Not pursued (owner priority 2026-11-23); see § 3 |
| 4 The oval on the shared Memory | E7, now conditional on E2(b) |
| 5 INVARIANTS rows | E7 |

The old plan's stance "stays open until these have flown" (owner, 2026-11-06) was reversed by the owner on
2026-11-24: the old plan is archived as superseded in part by this one.

## 7. Open questions

1. The E2 gate numbers (30 % command-jitter reduction, rms no worse on every axis) are proposed, from a
   prediction-quality replay and not a closed-loop simulation; owner to confirm.

Answered 2026-11-24 (owner): Mocap will cover the bench Hand once markers are on it (E1); E5 stays in this
plan; `learner-generalisation.md` is archived (`plans/archived/learner-generalisation.md`); the Destination
is narrowed to this plan's scope, so a mid-Pattern change on the robot and per-Site xy are not scheduled
under this Pathway.

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
   discontinuity, and whether the plant model needs release-velocity terms for it. E0 and E1 bear on this.
4. **Cosmetic:** the `box:` label key in `ros_ws/gui/js/juggle-panel.js` and the `dwell_s` comment in
   `Juggle.action` that still describes the box.

