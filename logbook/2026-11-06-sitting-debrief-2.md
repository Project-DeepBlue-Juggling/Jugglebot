---
title: "Sittings 2026-11-06 (afternoon) debrief: the P2 characterisation Sitting passed (JB-187); the two-ball pull toward the partner Site is an arrival transient and not a Site property, so the arrival term is not added; the Hand fault was JB-137's mechanism again; four endurance launches ended Columns runs at a mean of 22 Throws"
type: investigation
date: 2026-11-06
status: complete
phase: "learner-generalisation P2 characterisation Sitting"
related_plan: learner-generalisation.md
related_issues: []
files_changed:
  - logbook/2026-11-06-sitting-debrief-2.md
  - logbook/2026-11-06-nomination-reaim-figure.md
  - logbook/2026-11-06-hand-feedback-watch.md
  - logbook/INDEX.md
  - plans/active/learner-generalisation.md
  - map/feature-p2-characterisation-sitting-apex-sweep-off-centre-sites-ball.md
  - map/bug-nomination-margin-is-higher-than-the-live-plan-s-it-plans-at.md
  - map/bug-two-ball-throws-land-about-9-mm-toward-the-partner-site-an.md
  - map/bug-the-flyback-over-corrects-a-catch-offset-by-1-2-to-1-4-times.md
  - docs/project-map.md
  - docs/runbook.md
  - .gitignore
subsystem: [motion, ros, tracking]
tags: [safety, dynamics, testing]
---

# Sittings 2026-11-06 (afternoon) debrief

Launch log of the P2 Sitting: `~/.ros/log/2026-11-06-15-53-05-334499-jetson-1580225/launch.log`, bag
`~/Desktop/rosbags/2026-11-06_15-53-05`. The four endurance launches before it (14:55 to 15:46) are listed
with their log directories and bags in `temp/probes/sitting_debrief_2026-11-06b/context.md`. The seven unit
reports behind every number here (`unit_A_C.md`, `unit_B_D.md`, `unit_transit.md`, `unit_hand.md`,
`unit_stack.md`, `unit_endurance.md`, and `context.md`) are kept in that directory, which is gitignored.

## Results

| Row | Verdict | Evidence |
|---|---|---|
| JB-187 (P2 characterisation Sitting) | pass (owner, typed 16:10; confirmed at the debrief) | 19 goals flown at 15:53. 11 of 12 one-ball goals read 20/20; the twelfth ended on a Hand fault (below) and its repeat read 20/20. 346 live `ADMISSIBILITY` checks, 0 refusals. Base joints stayed attached (owner). Every clause of the Runbook is met, two with a departure noted below. |
| The four P2 decisions | taken, all "keep today's structure" | See Decisions. |
| JB-185 (nomination margin) | closed by option C | `logbook/2026-11-06-nomination-reaim-figure.md`. |

Goals in brief (launch-log goal lines in `context.md`): goals 1 to 5 were the one-ball Apex sweep (0.90, 1.00,
0.80, 1.05, 0.70; 20 Throws each, all 20/20); 6 to 9 the off-centre Sites; 10 to 12 the ball swap (goal 10 ended
on the Hand fault after 3 Throws, goal 11 repeated it); 13 to 15 the `columns_turned` pair at 90 degrees (20/20,
then 35 and 14 Throws ending in MISSED_CATCH); 16 to 19 plain Columns at Sep. 120 (2, 5, 38 and 7 Throws, the
last ending ABORTED_NO_RELEASE). Of the five MISSED_CATCH ends in goals 14 to 18, two were collisions and three
smooth misses (see What ends Columns runs).

## Symptoms

- Goal 10: the Hand ODrive reported CURRENT_LIMIT_VIOLATION after the second Throw (launch.log 5867), the guard
  latched ODRIVE_FATAL, the Attempt ended ABORTED_MODE_CHANGED at 3/3 caught. The Throw reached Apex 0.32 m against
  the 1.00 m commanded. Owner: "I am not sure why this was triggered, as all balls are functionally identical."
- Seven one-ball goals flew at Site x = -50 mm and not at x = 0 as the Runbook states (below).
- The endurance launches: 35 Columns goals, none reached its 101 Throws.

## Diagnosis

**The stack.** 346 per-Throw `ADMISSIBILITY` and 346 `ADMISSIBILITY-EARLY` lines, no refusal. All 346 early
checks sent the early resolution; 343 left the candidate unmoved. Lowest live margin 0.280 (goal 18, launch.log
10290). The live minimum sat 0.04 to 0.07 below the nomination's worst margin in every two-ball goal; in one-ball
goals the nomination reads 1.000 and the live minimum 0.45 to 0.66. Planned Leg peaks in the two-ball goals were
320 to 371 mm/s and 3656 to 5488 mm/s^2: above the previous clean envelope (361 / 4672) in goal 17 (speed) and in
goals 15 and 17 (acceleration), far under the separating transit of Sep. 170 (486 / 6296, JB-184). The Runbook's
offline prediction (264 / 3011) read low because it did not include the Catch re-aim. `RESEND-REFUSED` read
0.197 per Throw (68 of 346); 29 of them INFEASIBLE (8.4 % against 2.3 % at the 13:19 Sitting); 15 of those on the
69 turned-pair Throws. Refused Throws did not land worse. Filed as JB-188.

**The Site departure.** Seven one-ball goals (1 to 7) flew at Site x = -50 mm, the launch default. The Runbook's
step 2 (two `ros2 param set` lines before power) was not applied, and step 8 set only y. The Memory rows carry the
Site (`site=[-0.05, 0.0]` for goals 1 to 5, `[-0.05, +0.06]` and `[-0.05, -0.06]` for goals 6 and 7); the console
prints no Site, so it was invisible until the first row. Cause on the Runbook side: this Sitting's author put a
state-changing step among the before-power checks, and its blocks did not each set both coordinates. The design
survived: y varies at x = -50, x varies at y = 0, and the centre Site flew with balls B and C only (goals 10 to
12). Filed as JB-190 (print the Site in the started line).

**The Hand fault (goal 10).** Not the ball and not a planned current. The Hand's encoder frames fell to about
10 Hz for 0.6 s (six frozen runs of 83 to 94 ms, 1254.765 to 1255.356) while its current frames kept arriving.
The last freeze started at the stroke floor (0.063 rev, -2.0 rev/s), so the firmware's Hand lead clamp anchored
there and held the transmitted command at the anchor plus 2.0 rev (the bag echo reads 1.914 and 1.894 rev; the
clamp formula gives 1.913 to 1.883). The Hand followed the clamped command, then met a 1.2 rev step as the clamp
released: iq setpoint 69.9 A (the 70 A soft limit), measured 81.3 A, over the 80 A hard maximum. The drive
tripped CURRENT_LIMIT_VIOLATION; the unpowered Hand coasted to 10.734 rev, past the 10.701 rev stop, at 8 rev/s
or less. `lead_clamp_mask` read 64 (bit 6) in the heartbeat at 1255.439. This is the JB-137 mechanism a second
time, ending through the drive's hard limit instead of MAX_DEVIATION: every firmware guard sat outside the 30 to
150 ms gap by design (MAX_DEVIATION counts only anchors 30 ms or fresher, MOTOR_FB_STALE needs 150 ms, the fresh
residual of 1.3 rev was under the 2.5 rev guard). Normal Throws run at 23 A median and 32.5 A maximum at launch
(140 Apex 1.00 Throws), under half of 70 A. The stream was clean (lost 0, forward gap at most 40.8 ms). The
cause of the encoder frame loss is not known: the loss was selective by message (iq frames kept coming), and Leg 0
showed the same cadence. A per-node, per-message RX census at the bridge would settle it.

**What the Learner did.** K held 1.0765 to 1.0795. The first Throw at the Apexes it had not flown (0.70 and 0.80,
two and two prior rows) was within 0.004 m of the command (-0.002 and +0.004). The worst first-Throw Apex error
was Apex 1.05 at -0.049 m. First Throws at the off-centre Sites landed 20, 45, 30 and 54 mm out against 5 to
16 mm at the centre Site.

## Discussion

**A premise the plan carried was wrong.** The plan called `s` a "centre pull", a property of the Site: a
one-ball offset proportional to Site x (-0.074 per m on the corpus). The Sitting does not support that.

- One-ball Throws show no significant pull: `s_x` = -0.073 +- 0.061 and `s_y` = +0.067 +- 0.059 per m. The
  leave-one-Site-out error is 6.2 mm for x only, 7.3 for radial and 7.0 for separate x and y, so the x-only
  structure stays (decision B below), but the data does not call it a centre pull either.
- Two-ball Throws land pulled toward the PARTNER Site by -0.18 +- 0.02 per m of Site coordinate (about 11 mm at
  Sep. 120), over 25 Attempts along x, and -0.19 +- 0.04 over 3 Attempts turned to 90 degrees (along y). In Columns along x the partner direction and the sign of Site
  x coincide, so a Site-x term cannot tell them apart (inference).
- The transit analysis (`unit_transit.md`) then showed it is an arrival effect. The first Throw of an Attempt,
  with the Platform at rest, has no pull (it lands +8.9 +- 1.8 mm outward of later Throws, 117 Attempts). A
  departure term is null; the departure transit starts about 70 ms after release. In the P2 bag the pull is half
  position and half velocity. Position: the commanded Carry creeps 5 to 6 mm toward the partner between touch-down
  and release, and the recorded release does not hold it, so the flyback never returns it. Velocity: the Platform
  still moves about 7 mm/s toward the partner at release beyond its command, the ring left by the arrival (about
  5 Hz here; the carry-ring work measured about 6 Hz). The cause of the Carry creep is not identified.
- An arrival term `c * (Site - arrived_from) / 2` with c = -0.150 +- 0.010 beats today's model on 382 held-out
  Attempts by 6.5 +- 2.8 mm^2. But the pull grows faster than linearly with Sep. and vanishes at Apex 1.05 and
  above, so a scalar c would over-correct by about 6 mm at Apex 1.10. Whether c depends on Apex (ballistic) or on
  the time from arrival to release (transient) cannot be told from any flight so far: they moved together.

**Ruled out.** A radial Site term (block B); a departure effect; a ballistic-velocity term (the Apex test);
Platform tilt as the cause (the tilt at release is commanded and points outward in Columns, the wrong sign for an
inward pull).

**Decision taken: the arrival term is NOT added to the plant model now.** Reasons: (1) about half of it (the Carry
creep) may be a planner defect that should be fixed at its source, not learned; (2) the other half is the ring
that the carry-ring-zvd plan already targets; (3) a scalar term is known to be wrong at Apex 1.05 and above; (4)
it needs a Memory column (`arrived_from`, schema v3), and the 2203 stored two-ball rows would default to zero:
their residual memory would keep the -9 mm pull while f0 applied it again, a double count of about 9 mm at every
Columns Site. Cost of waiting (inference): the residual memory keeps absorbing the pull at the Sites it has
flown, and a new pair geometry would start about 9 mm off. The question is recorded as the open question that gates
`yaw_rad` and is filed as JB-192.

**The `yaw_rad` frame.** The Sites rotate about the Platform origin in the Robot frame (as `sites.py` does for
`columns_angle_deg`). `b` and the one-ball `s` stay in the Robot frame, unrotated. The arrival pull is along the
transit and turns with the Pattern; it is not a term of f0, and the residual memory is keyed on Robot-frame Site
xy, so a pair whose angle changes each beat cannot learn it. `yaw_rad` stays refused (`BEAT_YAW_NOT_SUPPORTED`)
until the pull is fixed at its source or carried by a term.

**The four decisions the plan's rules covered.** All four keep today's structure.

- **A, lateral offset is a fixed distance.** Slope of (landing - command) against Apex: -22 +- 17 mm/m (x) and
  +8 +- 16 mm/m (y), under 2 SE. The apparent x trend is time drift (+7 mm per 100 s). The aim model differs from
  the fixed one by 1.9 mm over the whole Apex range, below the level SEs, so by the pre-set rule (ship the model
  with fewer parameters) it stays a distance. K is one number over 0.70 to 1.00 (1.084 to 1.090). At 1.05 it
  drops to 1.067 +- 0.004 with the Hand's peak speed 4 % under its command: a Hand effect (JB-147 family), not a
  model term.
- **B, the centre pull stays on Robot x only.** See the error figures above; the one-ball `s_y` has the wrong sign
  for a radial pull.
- **C, the per-ball offset stays OFF.** Balls B and C at the same Site differ by 10 mm (x) and 12 mm (y), under
  2 SE and under 0.6 of the per-Throw scatter (17 to 22 mm); a common time trend absorbs the step. Apex gain per
  ball agrees within 0.007. Ball A flew at a different Site and is not a clean reference. The owner's "all balls
  are functionally identical" is supported on this evidence.
- **D, the frame of the common offset `b` is not identified.** It wanders 8 mm per Attempt on each axis and the
  held-out verdict flips with the training set, so by the rule it stays in the Robot frame.

**The Throw-to-Throw coupling.** In the one-ball goals the Cup releases from where it caught (commanded release
against the previous landing: slope 0.87, r +0.93) and the planner Banks to fly the offset back. The flyback
returns 1.19 mm per mm, against 0.87 mm of release offset, so each landing error returns at -0.3 to -0.45 of itself
on the next Throw (lag-1 slope -0.30 measured, corpus -0.45 x and -0.24 y). It damps. It is a flyback gain about
1.2 to 1.4 times too strong, a planner matter and not a plant-model term; filed as JB-193. The lateral scatter is
set by the Platform tilt at release (ball lateral velocity against tilt r 0.78 x, 0.62 y). Release-time error (+9 ms
mean, sd 7.5 ms) has no effect on Apex (r 0.00).

**Owner decisions the Hand fault raises (none applied).**

1. The Hand lead-clamp anchor. Today it is `fb + vel * age`; stale at the stroke floor it stays at the floor while
   the plan accelerates at about 2400 rev/s^2, then releases a 1.2 to 2.6 rev step. The Legs got a dead-reckoned
   anchor at FW 31. Options, each needing an offline replay first: port that anchor to the Hand (continuity, but
   the Hand then follows the plan blind for up to 150 ms); stop the Hand with a C1-style stop when a launch goes
   blind; slew back to the plan when the clamp releases; or keep today's behaviour with the ODrive trip as the
   backstop.
2. The 70 / 80 A gap. Against: this event overshot the clipped setpoint by 11.3 A (JB-138: 3.3 A at 50 A). For:
   normal Throws never come within 37 A of the soft limit, the trip did what a hard maximum is for, and the cause
   sits upstream in the clamp step. Whether a powered Hand would have avoided the top stop cannot be told without
   a replay of the ODrive loop. The live hard maximum is not readable from the log (the JSON backup says 80 A).

Visibility was the third item and was done without a limit change: `logbook/2026-11-06-hand-feedback-watch.md`.

**What was not changed.** No limit, margin or guard (standing scope: those come back as owner decisions with the
numbers). No code in this entry's scope; the plant model, `yaw_rad` and the clamp are as before.

## What ends Columns runs

Survey of the 35 endurance goals (all `columns`, Dwell 0.27, 101 Throws asked, Apex 0.95 to 1.10, Sep. 125 to 133;
`unit_endurance.md`). Median 15 Throws, mean of the 30 that threw 22.2, best 73, 62, 54, 45, 35. About 30 ends over
667 Throws judged: 4.5 ends per 100 Throws, a mean run of about 22 Throws.

| End class | Goals | Throws at end |
|---|---|---|
| Smooth missed Catch (no collision) | 13 | 4 to 33 |
| Mid-air ball collision, confirmed | 8 | 1 to 29, median about 17 |
| Ambiguous (one-sided kink or a 30 mm pair) | 2 | 13, 43 |
| Hand or Leg drive fault (SPINOUT, HAND_JAM) | 5 | 1 to 52 |
| Lane refused after a stream gap (GC pause) | 2 | 73, 62 |
| Refused or aborted before the first Throw | 5 | 0 |

- **JB-186 is confirmed at a larger sample, but collisions are a quarter of the ends, not half.** The eight
  confirmed collisions show a closest approach of 70 to 77 mm (ball 74 mm) with both tracks kinking 170 to 800 mm/s;
  Sep. 125, 130 and 133 alike. The wider gap did not help (Sep. 125: 4.8 ends per 100 Throws; Sep. 133: 8.4; small
  n, the 133 runs follow a power cut). At the P2 Sitting 2 of 5 two-ball ends were collisions; 5 of 7 collision-like
  events did not end the Attempt.
- **Smooth missed Catches of Ball 2 are the largest class** (new Item JB-191): 21 of the 23 MISSED_CATCH first ENDs
  are the Catch of Ball 2, though both balls' Throws land equally well (lateral RMS 29.6 and 31.9 mm). In 9 of the
  13 smooth ends the last fit was within 40 mm of the aim and the cup read EMPTY: a bounce-out, a false EMPTY or a
  late fit, not resolved (it needs `/hand_telemetry`). At the P2 Sitting three of five ends were this class, each
  with the Catch aim clamped at the lateral authority (10 mm outward three times, 35 mm inward twice).
- **The two longest runs ended on a garbage-collection pause**: 73 and 62 Throws, both LANE_REFUSED after a 58 to
  64 ms generation-2 pause in `trajectory_node` (forward gap 72 to 78 ms, bridge loop gap 25 to 28 ms: the stream
  was late, the bridge was not starved). `gc_freeze` is default off and the launch lines do not say whether it was
  passed (JB-149).
- Fault ends carry a mode name: four of five read WRONG_MODE, ABORTED_MODE_CHANGED or GUARD_LATCHED (JB-175). Two
  bus power cuts (the owner's E-stop button, inferred) interrupted the afternoon.
- **The Learner over long runs.** 593 new rows. No drift (slopes within +-0.2 mm per Throw over runs of 52 to 70
  rows); Apex error RMS settles from 28.4 mm (Throws 1 to 5) to 14.7 mm (Throws 31 to 80); the lateral error is a
  floor of about 30 mm RMS (x 20, y 22) that a mean-shift correction cannot lower. K from the rows: 1.074 to 1.082
  over Apex 0.95 to 1.10.

Two other ABORTED_NO_RELEASE ends were not release failures: P2 goal 19 (launch.log 11050: the cup read empty for
the whole stroke, the ball was lost at the Catch before; the rising Cup may have struck it, not told apart at
100 Hz) and endurance 14:55 goal 9 (the release was cancelled by an earlier END). JB-97 carries both.

## Fix

No change to the plant model, the planner, the firmware or any limit. Code that did land, each with its own entry:
the nomination's worst-corner figure (`logbook/2026-11-06-nomination-reaim-figure.md`, closes JB-185) and the host
Hand feedback watch (`logbook/2026-11-06-hand-feedback-watch.md`, a diagnostic for JB-137). Paperwork: JB-187 and
JB-185 closed; JB-188 to JB-193 filed (below); plan `learner-generalisation.md` updated (P2 and P3b complete,
section 2.2 pinned, the `yaw_rad` frame, section 7); progress notes written to JB-137, JB-186, JB-166, JB-97,
JB-161, JB-115, JB-184.

**Memory is now tracked.** `temp/learn/jugglebot/memory_v2.csv` is tracked in git and committed at each debrief
(owner decision 2026-11-06); `.gitignore` un-ignores it. `memory.csv` stays frozen as the migration test's fixture
(3365 rows), and neither file is edited by hand.

## Notes for Items held by other owners

Five Progress notes could not be written to their Items because other owners hold them. Texts as intended:

- **JB-149.** "2026-11-06 endurance Sittings: the two longest Columns runs (73 and 62 Throws) both ended
  LANE_REFUSED after a 58 to 64 ms generation-2 garbage collection in `trajectory_node` (forward gap 72 to 78 ms,
  bridge loop gap 25 to 28 ms: the stream was late, the bridge was not starved). 14:55 log L3519-L3527 and
  L11162-L11174. `gc_freeze` is opt-in and default off; whether those launches passed it is not in the log."
- **JB-175.** "2026-11-06: fault ends carry a mode name instead of the fault. Four of five drive-fault ends in the
  endurance Sittings read WRONG_MODE, ABORTED_MODE_CHANGED or GUARD_LATCHED, and the P2 Hand
  CURRENT_LIMIT_VIOLATION read ABORTED_MODE_CHANGED with 3/3 caught. A spurious MISSED-BALL line follows the fault
  in 14:55 goal 4 (L6528). The MISSED_CATCH mislabel itself was not seen."
- **JB-147.** "2026-11-06: the coast signature reproduced from a known cause (P2 goal 10: drive disarmed on
  CURRENT_LIMIT_VIOLATION 25 to 37 ms after release, Hand coasted to 10.734 rev, past the 10.701 stop).
  Endurance: two Hand SPINOUT latches (14:55 L6493, L7014, the second within 10 ms of a C1 stop) and three
  HAND_JAM stalls at +2.38 to +2.52 rev at iq -65 to -71 A. At Apex 1.05 the Hand's peak speed ran 4 % under its
  command (K 1.067 against 1.087)."
- **JB-108.** "2026-11-06 P2 Sitting: `columns_turned` at 90 degrees, Apex 1.00, Sep. 120 flown three times under
  the merged Learner: 20/20 (completed), 35/35 then a collision, 14/14 then a collision. Live admissibility
  margin 0.31 to 0.34 against 0.378 nominated. The two-ball pull toward the partner Site turns with the pair
  (-0.19 per m along y)."
- **JB-169.** "2026-11-06 endurance Sittings: no goal reached its final Catch, so not tested. Related: 9 mid-run
  MISSED_CATCH ends have the last fit within 40 mm of the aim and the cup reading EMPTY (14:55 to 15:46 goals 7,
  8, 16, 17, 18, 24, 26, 28, 33); the cup trace in `/hand_telemetry` was not read."

## Items filed

| Item | Priority | What |
|---|---|---|
| JB-188 | medium | Catch re-aim refused INFEASIBLE on 8.4 % of Throws at the P2 Sitting, 15 of 69 on the turned pair |
| JB-189 | low | A Throw from a faulted Hand stroke is written to the Memory as caught |
| JB-190 | low | A goal entered by a Feed prints `(reload) ENDED` and no `done` line; the Site is not printed at the start |
| JB-191 | high | Two-ball Columns runs end on a missed Catch of Ball 2 with no collision: 13 of 23 ends, mean run 22 Throws |
| JB-192 | high | Two-ball Throws land about 9 mm toward the partner Site: an arrival transient the plant model has no term for (gates `yaw_rad`) |
| JB-193 | medium | The flyback over-corrects a Catch offset by 1.2 to 1.4 times: each landing error returns at -0.3 to -0.45 on the next Throw |

## Verification

The numbers come from the launch logs and bags named at the top; each unit report states its scripts and greps.
Gate on the debrief's tree:

Full gate, run 2026-11-06 17:20 to 17:28 on the complete change set of this debrief (`./run_tests.sh --full`): **8792 passed, 52 skipped, 2 failed** in 453 s, plus the serial tail **10 passed**. Both failures were document currency tests, not code (`tests/sim/test_map_items_rated.py`: Item JB-190 moved to in_progress without its `owner` and `radius` ratings; `tests/sim/test_plans_index.py::test_index_is_current[active]`: the plan's `scope:` line was edited after `python -m mapkit gen`). Both were fixed (rated, regenerated) with no code edit after the gate, and the document tests were re-run: (2026-11-06 17:30, `python -m pytest tests/sim/test_map_items_rated.py tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py tests/sim/test_sitting_debrief_protocol.py tests/sim/test_sitting_sections.py tests/sim/test_night_shift_gate.py -q`: **304 passed, 48 skipped**; `python -m pytest tests/sim -k "mapkit or map_items" -q`: **310 passed, 48 skipped**).

## Outcome

The P2 Sitting answered its questions: the Learner's structure holds at every Apex and Site flown, the ball is not
a model term, and the stack caused no Attempt end (346 checks, 0 refusals). What it found is outside the structure
the plan fixed rules for: a two-ball arrival transient that the plant model cannot hold and a flyback gain that is
too high. Columns runs end at a mean of 22 Throws, mostly on missed Catches of Ball 2 and collisions, not on the
Learner.

## Open Questions

- Fix the arrival pull at its source or carry it with a term, or both. Proposed order: probe the planner's Carry
  creep offline (`_carried_release_mm` against the streamed Cup position at the release knot, in a sim replay),
  then one hardware check (Columns Sep. 125, Apex 0.93, alternating Dwell 0.27 and 0.40, three Attempts each,
  bag recorded; the check is a proposal, not on the Runbook). The learn option needs the `arrived_from` column
  and a migration that reconstructs it for stored rows and never defaults it to zero.
- Why do Ball 2's Catches end runs more than Ball 1's (JB-191)? The cup trace from `/hand_telemetry` for the nine
  EMPTY-with-a-good-fit Catches has not been read.
- Why is the turned pair's re-aim QP infeasible more often (JB-188)?
- What causes the Hand encoder frame loss (selective by message; Leg 0 showed the same cadence)?
- The Hand anchor and the 70 / 80 A gap (owner decisions above).
- Per-beat rotation of a Pattern and a live operator channel for a mid-Pattern change wait on the arrival pull;
  the oval is separate.

## Follow-up, same day

The position half of the arrival pull recorded above (a commanded Carry creep that might be a planner defect) was withdrawn by an offline probe the same evening: the release follows the caught position by design, and against the release that flew the pull is almost all velocity. The same probe found the Memory's `rel` records the dispatch aim, and the Hand dropouts were traced to a clock beat. See `logbook/2026-11-06-arrival-pull-and-hand-dropout-probes.md`.
