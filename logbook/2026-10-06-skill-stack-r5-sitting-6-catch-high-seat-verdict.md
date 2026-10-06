---
title: "R5 sitting 6, second attempt (2026-10-06 evening): the hand is not slow, the catch is taken high by design — and the cup sensor's seat now lands around the re-release, where the caught verdict's window had closed; the SEATED evidence closes 0.12 s past the release (the miss rule's own bound) and reads the sample-complete history"
type: investigation
date: 2026-10-06
status: in-progress
phase: "two-ball-skill-stack — R5"
related_plan: two-ball-skill-stack.md
related_entries:
  - 2026-10-06-skill-stack-r5-sitting-6-start-state-refusals.md
  - 2026-10-05-skill-stack-r5-sitting-5-afternoon-parked-feed-and-catch-high.md
  - 2026-10-04-skill-stack-r5-sitting-4.md
  - 2026-09-17-late-catches-are-a-late-tracker.md
files_changed:
  - plans/active/columns-tracking-ff-and-shaping.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/motion/test_skills_executor.py
---

# R5 sitting 6, second attempt: catch high, a late seat, and a verdict that raced it

## Summary

The owner re-flew `tests/hardware/session_skills_r5_sitting6.md` on the evening of 2026-10-06
(launch 20:19:40, bag `~/Desktop/rosbags/2026-10-06_20-19-40`, log
`~/.ros/log/2026-10-06-20-19-40-338838-jetson-420583/launch.log`) after the morning's fixes
(`2026-10-06-skill-stack-r5-sitting-6-start-state-refusals.md`) and reported a poor outcome:
the Step 2 regression (`columns_1ball`, reload ticked) "nominally passed more than one attempt,
but even in those attempts the hand seemed like it was moving far too slowly". Between the
sittings every leg ODrive had been put through `FULL_CALIBRATION_SEQUENCE`; mid-sitting the hand
ODrive was too, and its reboot latched the guard, FAULTed the machine, and left the platform
holding at the P2 site until the next activate moved it to the home pose in one TRAP_TRAJ move.

Four attempts, all `columns_1ball` with reload (ball A fed by Ball Butler, three real throws and
three phantom strokes per attempt):

| attempt | start | end | reported | what actually happened |
|---|---|---|---|---|
| 1 | 20:23:58 | `LIMIT_JERK` at skill 7 (A's second CATCH), 220 792 > 200 000 mm/s³ | 1/2 caught | throw 1 caught; the pattern ended before A's second catch, the ball landed 11 mm from P2 on a platform parked at P1 |
| 2 | 20:24:21 | done | 2/3 caught | **3/3 caught** — throw 1 was in the cup and was re-thrown; the verdict read MISSED |
| 3 | 20:24:34 | `LIMIT_VEL` at skill 6 (the phantom CATCH), 353.6 > 350 mm/s | 0/2 caught | throw 1 caught and re-thrown by the running fold; the pattern had ended, the re-throw landed on the platform |
| 4 | 20:26:19 | done | 2/3 caught | **3/3 caught** — throw 2 read MISSED the same way |

Three findings, in the order the owner asked:

1. **The hand is not slow and the hardware did not change.** Every throw stroke tonight peaked at
   126–135 rev/s commanded and 122–137 rev/s measured (meas/cmd 0.94–1.05, command delay 7–11 ms,
   `tools/probes/hand_overspeed_bag_probe.py`), the same as sittings 4 and 5 (124–131 / 117–134
   rev/s, 5–14 ms), and every ball reached its apex (0.936–0.974 m against 0.95 commanded). What
   the owner saw is **catch high** (`sites.CATCH_CUP_Z_MM` 830 → 930, owner decision 2026-10-05,
   first flown with real throws tonight): the cup no longer dives at −1.6 m/s to meet the ball
   at 830 mm; it brakes to a stop near the top of its stroke after the release, dwells there, and
   meets the ball at −0.96 m/s, 68 mm into its descent, then carries it down through the whole
   237 mm stroke. The hand's velocity at the catch instant is −20 rev/s where it was −50 at 830.
   That is the design, and it is visibly a slower, waiting hand.

2. **The cup sensor's seat moved past the caught verdict's window.** The raw possession bit does
   not read SEATED when the ball lands — the cup is accelerating downward at ~0.4 g and the ball
   rides in it under ~0.6 of its own weight — but when the hand bottoms out and reverses into the
   next stroke. At 830 that reversal came +0.12..+0.20 s after the landing; at 930, with 100 mm
   more with-ball stroke to carry down, it comes at **+0.227..+0.288 s**, against a re-release at
   +0.27 s. The executor's verdict window closed at the re-release less 10 ms and sampled the
   observer once per 40 Hz tick, so its own latch fell 1–31 ms inside the close on the catches it
   scored and past it on the ones it did not. Three of five chained catches were reported MISSED
   with the ball in the cup. The learner is unaffected (its fit never reads the `caught` column),
   the MISSED_CATCH stop rule is unaffected (it already reads a sample-complete window closing
   0.12 s after the release — sitting 4 measured that bound on real feed catches), the operator
   line and the `caught` column were wrong. **Fix: the caught verdict's SEATED evidence now closes
   at the same bound the miss rule uses, and the verdict reads the same history.**

3. **Two of four attempts still ended on the known 10 % margin class** (sitting 4 § "Why the
   planner refuses what the box certified"): attempt 3's phantom CATCH at P1 was refused
   `LIMIT_VEL` 353.6 after the preceding catch had been re-aimed to (82.5, −20.0) — the full 20 mm
   authority in both axes, away from P1 — so the site-to-site transit was 146 mm instead of 125;
   attempt 1's `LIMIT_JERK` at A's second catch is the late-landing signature (release +14 ms,
   apex scatter ±0.02 m ≈ ±9 ms of flight). Not fixed here; owner decision, options below.

Also measured, for the owner's two asides: leg 4 is no longer on the 10 A clamp (its
95th-percentile current fell from 9.85 A on 2026-10-05 to 2.67 A tonight, legs 0/3/5 unchanged),
so the leg `FULL_CALIBRATION_SEQUENCE` fixed it and the torque-feedforward plan's "leg 4 outlier"
premise is retired; the hand's own recalibration cut its throw-stroke current by ~20 % for the
same motion (peak |iq| 28–33 A loaded / 16–22 A empty after, 31–37 / 24–28 before). The FAULT
that did not stow, and the activate "snap", are in their own section.

## Measured

### Hand tracking is unchanged

`tools/probes/hand_overspeed_bag_probe.py --min-peak-rps 60` on three bags (primary columns only):

| bag | strokes | cmd peak (rev/s) | meas peak (rev/s) | meas/cmd | delay (ms) | peak |iq| loaded / empty (A) |
|---|---|---|---|---|---|---|
| 2026-10-04_19-52-08 (sitting 4, catch 830) | 23 | 119.8–130.6 | 117.4–128.8 | 0.93–1.03 | 5–9 | 25–34 / 19–28 |
| 2026-10-05_16-42-57 (sitting 5) | 23 | 123.7–131.7 | 119.4–133.5 | 0.96–1.04 | 1–14 | 21–32 / 14–28 |
| 2026-10-06_20-19-40 before the hand recal (att. 1–3) | 13 | 125.9–134.6 | 122.0–136.5 | 0.94–1.05 | 7–8 | 31–37 / 24–28 |
| 2026-10-06_20-19-40 after the hand recal (att. 4) | 6 | 129.2–135.2 | 128.1–134.7 | 0.99–1.03 | 10–11 | 28–33 / 16–22 |

Position error through a 130 rev/s stroke is ±25 mm (the 7–10 ms delay at 4.2 m/s), identical in
the 10-04 and 10-06 zooms (`scratchpad/probe_hand_profile.py`, one cycle each). The 100 mm higher
catch shows directly in the velocity trace: the pre-throw dip that is the catch sits at −20 rev/s
tonight and at −50 rev/s on 10-04.

### Where the seat lands against the window

`scratchpad/probe_seat_race.py` over the bag: the first raw-bit 0→1 edge after each scheduled
landing (`/hand_telemetry` stamp), its arrival (bag log time), the window close the executor used
(re-release − 0.010 s), and the hand at the edge:

| catch | raw edge − scheduled landing | hand at the edge | margin to the close | reported |
|---|---|---|---|---|
| att. 1, throw 1 | +0.242 s | 0.84 rev, +40.6 rev/s (rising) | +16 ms | CAUGHT (latch at +0.285 vs the row's landing) |
| att. 2, throw 1 | +0.227 s | 0.51 rev, +21.9 rev/s | +31 ms | **MISSED** |
| att. 2, throw 2 | +0.230 s | 0.82 rev, +41.1 rev/s | +30 ms | CAUGHT (+0.284) |
| att. 2, throw 3 | +0.259 s | 1.08 rev, −11.3 rev/s (the stop REST) | final catch, no bound | CAUGHT (+0.309) |
| att. 3, throw 1 | +0.288 s | 6.37 rev, +128.5 rev/s (mid-stroke) | −29 ms | **MISSED** (the pattern had ended; the ball was re-thrown) |
| att. 4, throw 1 | +0.234 s | 0.89 rev, +46.0 rev/s | +24 ms | CAUGHT (+0.287) |
| att. 4, throw 2 | +0.255 s | 3.38 rev, +119.9 rev/s | +1 ms | **MISSED** |
| att. 4, throw 3 | +0.277 s | 0.38 rev, −6.5 rev/s | final catch | CAUGHT (+0.335) |

Every edge is the hand at or just past the bottom reversal, and on the two MISSED chained
catches the ball that "was not caught" was thrown again 40–60 ms later and tracked to a clean
apex. The row's own landing (`t_release + flight(u)`, the learner's commanded apex 0.879 m) sits
33 ms before the schedule's, which is why the executor's `seat` phase reads 0.28–0.29 s for a
+0.23–0.24 s raw edge: the close is at +0.293 on the row's clock and the latch made it by
≤ 10 ms on every scored catch.

Earlier sittings for the same quantity (`seat +x.xx s` on the operator lines): 10-04 19:52 (catch
830, 91 catches) mode +0.17 s, 10-05 16:42 +0.10..+0.19 s; tonight +0.28..+0.34 s.

### Leg currents: leg 4 before and after the leg recalibration

`scratchpad/probe_leg_iq.py` over `/robot_state` (100 Hz, `iq_measured`), one columns run each:

| leg | 10-05 16:42 (30 s hop run): peak / p95 / rms A, fraction ≥ 9.5 A | 10-06 20:24 (16.5 s, attempts 2–3): same |
|---|---|---|
| 0 | 8.20 / 3.24 / 1.66, 0.000 | 7.11 / 2.67 / 1.44, 0.000 |
| 1 | 10.36 / 6.13 / 2.44, 0.008 | 9.92 / 6.65 / 2.59, 0.010 |
| 2 | 10.62 / 5.91 / 2.68, 0.008 | 10.33 / 6.33 / 3.14, 0.006 |
| 3 | 6.10 / 2.64 / 1.10, 0.000 | 5.90 / 2.83 / 1.40, 0.000 |
| **4** | **10.97 / 9.85 / 4.49, 0.069** | **6.92 / 2.67 / 1.39, 0.000** |
| 5 | 6.48 / 3.44 / 2.25, 0.000 | 6.86 / 3.34 / 1.90, 0.000 |

Legs 1 and 2 remain the working pair in the lateral transits (p95 6–7 A, on the clamp ~1 % of
the time); leg 4 now looks like legs 0/3/5.

### The FAULT and the activate move

See § "The FAULT that did not stow" below for the timeline and the bag numbers.

## Discussion

**"Too slow" was the design, not a defect, and the data had to say so before anything else.** The
owner's instinct was right that something about the hand was different tonight: it was the first
time catch high flew with real throws (the 10-05 night attempts never threw). The probe
separated the two candidates cleanly — the stroke peaks, delay and overspeed ratio are the same
three sittings running, so the plant did not slow; the commanded catch profile did, by exactly
the amount `sites.py`'s own table predicts (cup −0.96 m/s at contact, 68 mm empty drop). The
question that remains is the owner's: whether a cup that waits at the top and carries the ball
down is what they want to watch, against the gentler contact it buys. Nothing in this entry
changes the catch plane.

**Why the verdict raced, and why it was not visible before.** `_outcome_window` closed the
SEATED latch at `t_next_release − 0.010` on the reasoning (2026-09-16) that the ball is in the
cup for the whole interval before the re-release and the cup is empty by construction after it.
The first half holds; the second describes the ball, not the sensor. The raw bit needs the cup to
press on the ball, which happens at the bottom reversal, and lags a departing ball on the way
out. Sitting 4 had already measured this on real feed catches — latest seat `R + 0.028` s,
n = 11 — and set the MISSED_CATCH rule's close at `R + 0.12`, but the caught verdict kept its
own older close, so from 2026-10-04 the two rules read the same cup sensor against different
windows. At 830 the reversal came early enough (+0.12..+0.20 s) that the older close never bit;
catch high moved the reversal 100 ms later and exposed the gap on the first evening it flew.

**Why the fix is the miss rule's bound and history, not a wider epsilon.** Two defects were
stacked: the close itself, and one observer sample per 40 Hz tick against a 100 Hz sensor (a
seat inside the window's last tick is seen or missed by phase; tonight's latch margins were 1–31
ms). Moving the close alone would have left the phase race; the sample-complete
`seat_window` history the node already keeps for the miss rule removes it. Reusing
`MISSED_CATCH_CLOSE_AFTER_RELEASE_S` and `MISSED_CATCH_OTHER_LANDING_GUARD_S` rather than minting
new numbers means one cup sensor, one window, two verdicts that cannot disagree — the "timing
twin" the plan's § 0 forbids is exactly what the two rules had become. The tracker's own freeze
(`_consider_landing`, `_landing_instant`) stays at the release: it answers which estimate is this
flight's, a different question, and the 2026-09-16 contamination it guards is untouched. A row
with a history wired finalises one `MISSED_CATCH_DECIDE_LAG_S` (30 ms) later than before so the
window's last sample has arrived; the next throw of the same ball is commanded at its own
dispatch (`L + 0.045` s in columns), before any window could close, so the learner's feedback
lag is unchanged.

**Ruled out:** the hand's mid-sitting recalibration (attempt 4 behaves like attempts 1–3 on every
timing column); the REST-RETRY fix from the morning (it fired on every attempt and installed on
the second or third try, as designed); a stale or late tracker (arrival −1..+4 ms on every
scored catch).

**What is still open — the 10 % margin.** Attempt 3 is the clearest case yet of the lateral
authority eating the transit margin: the previous catch moved 20 mm away from P1 in x and 20 mm
in y, the next transit grew 17 %, and the leg velocity came out 1 % over. Attempt 1's jerk
refusal is the late-landing case sitting 4 reproduced in the virtual loop. The box was swept at
the nominal sites and the nominal landing time; neither perturbation is certified. Levers, for
the owner: (a) a landing-TIME authority beside the lateral one (plan the catch at the tracker's
time only within ±N ms of the schedule's; a cup up to 20 ms mistimed meets the ball 80 mm higher
or lower on its 4 m/s descent, inside the stroke), (b) an asymmetric lateral authority that
clamps the away-from-the-other-site direction tighter than the toward direction, (c) a box sweep
with ±25 ms landing jitter and ±20 mm aim dimensions so the chosen geometry is certified against
what actually refuses, (d) the leg acc/jerk ceilings, which are YAML ceilings today and a
hardware decision. None of (a)–(c) is gated; (d) is.

## Fix

`executor.py`:

- `SEAT_AFTER_RELEASE_S = MISSED_CATCH_CLOSE_AFTER_RELEASE_S` (0.12 s), with the sitting's
  numbers in its docstring.
- `_PendingOutcome.t_seat_close_s`, filled at registration by the new pure helper
  `_seat_close(schedule, ball_id, t_land_scheduled_s, t_next_release_s)`: the next release plus
  `SEAT_AFTER_RELEASE_S`, pulled back `MISSED_CATCH_OTHER_LANDING_GUARD_S` before the other ball's
  next landing, `None` for a final catch.
- `_bound_by_next_release` bounds `finalise_at` by `t_seat_close_s` instead of
  `t_next_release_s − OUTCOME_NEXT_RELEASE_EPS_S`; the latter still bounds the tracker freeze and
  `_landing_instant`, unchanged.
- `_advance_outcomes` finalises `MISSED_CATCH_DECIDE_LAG_S` after the close when a `seat_window`
  is wired, and calls the new `_latch_from_history(pend, t_open, t_close)` first: a single
  raw-SEATED sample in the window sets `caught_seen` (and `t_seat_s` from the history's first
  seated stamp) when the per-tick latch has not; `None` is blind and leaves the latch alone.
- `SeatWindow.t_first_seated_s` (optional, default `None`).

`skill_node.py::_seat_window` records the first raw-seated stamp.

No gated file changed; the admissible box and gate `3c9533225417` stand.

## Verification

- `tests/motion/test_skills_executor.py`: the window test rewritten for the new close
  (`test_the_window_closes_at_the_seat_close_past_this_balls_next_release`), `_seat_close`'s
  bounds pinned (`test_the_seat_close_mirrors_the_missed_catch_rules_bounds`), and a new section
  replaying the sitting: seats at −0.043, −0.005, +0.018 and +0.10 s from the re-release all read
  CAUGHT; a seat past `R + 0.12` and an empty cup throughout both read MISSED; the registered row
  carries the computed seat close; a between-tick pulse is carried by the history with the seat
  phase from its stamp; a blind history invents nothing; the decide lag applies only with a
  history wired.
- Mutation check (2026-10-06, the bound reverted to `release − eps` in a scratch copy, scoped
  `-k 'rerelease or seat_close or sample_complete or history_is_blind or decide_lag or
  empty_cup_through'`): 7 failed, 5 passed — the −0.043 s case still passes with the old bound,
  as its docstring says it should.
- Scoped (`python -m pytest tests/motion/test_skills_executor.py tests/ros/test_skill_node.py
  tests/ros/test_ball_possession.py tests/sim/test_skills_gate.py -q -n 4 --dist loadfile`, run
  2026-10-06, log `temp/logs/scoped_s6b_verdict_20261006.log`): **612 passed in 310.13 s**.
- Full gate (`./run_tests.sh --full`, run 2026-10-06, log
  `temp/logs/gate_full_r5_sitting6_seat_verdict_20261006.log`): **parallel 6111 passed, 9
  skipped, 1 xfailed in 307.19 s; serial 6 passed in 20.36 s; RESULT PASS, exit 0.** The only
  edit after that run is this Verification paragraph and the agent numbers below;
  `tests/sim/test_logbook_front_matter.py` + `test_logbook_search.py` + `test_plans_index.py`
  re-run after it (2026-10-06): 110 passed.

## The FAULT that did not stow, and the activate "snap"

Timeline from the launch log (ROS epoch, 1791278xxx):

- 718.60 the can-bridge guard latched `MOTOR_FB_STALE` on axis 6 — the hand ODrive's feedback
  stopped when the owner rebooted it after its `FULL_CALIBRATION_SEQUENCE` (the FW 27 staleness
  guard from sitting 4 working as designed). `trajectory_node` eased the command onto the
  measured pose (62.0, −0.1, 170.0) mm over 0.20 s and held; the orchestrator forced
  ACTIVE → FAULT (guard-only: no ODrive error yet, streaming mode preserved).
- 720.41 the hand reported `INITIALIZING` → `ODRIVE_FATAL`; the fault was promoted to a real
  fault (control mode `ERROR`, setpoint output disarmed). The legs stayed in CLOSED_LOOP holding
  the ODrive setpoints they had.
- 721.00 the guard cleared itself once the hand came back; 721.10 FAULT → BOOT → IDLE (already
  homed). **No deactivate was ever requested**: `ActiveHandler.on_exit` does request one, but
  `orchestrator_node._cancel_pending_operations` discards it on the forced ACTIVE → FAULT path
  (its docstring: "correct for ACTIVE→IDLE but wrong for ACTIVE→FAULT"), and the real-fault exit
  FAULT → BOOT → IDLE has no stow of its own. IDLE's invariant (platform stowed, legs idle) was
  violated with the legs holding an ACTIVE pose 62 mm off centre.
- 734.60 the owner's activate from IDLE ran the firmware ACTIVATE: six independent ODrive
  TRAP_TRAJ moves from wherever each leg was to `ACTIVATE_POSITION_REVS`, cruise
  `GENTLE_MOVE_VEL_LIMIT_RPS` 2.5 rev/s ≈ 178 mm/s per leg, accel/decel `TRAP_ACC_LIMIT_RPS2` 30
  rev/s² ≈ 2130 mm/s² (`Teensy_code_canbridge/hardware_config.h`, `leg_activate.cpp`), done in
  0.6 s. That is the "snap": roughly half the streamed lane's velocity ceiling and 40 % of its
  acceleration ceiling, with no jerk limit and no platform-level coordination.

Bag numbers for that move against the juggle transits (Sonnet agent over `/robot_state` 100 Hz
and the mocap Platform body; `scratchpad/agent_activate_snap.md`, `activate_snap.png`):

| window | leg peak \|vel\| (mm/s) | leg peak \|iq\| (A) | platform peak (mm/s) |
|---|---|---|---|
| the FAULT hold 718.0–721.5 | 3 | 0.2–2.2, flat | 4 |
| the snap 734.4–735.6 (legs moved 9–28 mm) | 111–268; legs 1 and 2 overshot the 2.5 rev/s cruise to 3.77 and 3.05 rev/s and rang for ~0.3 s | **10.25 / 10.27 on legs 1 / 2** (the clamp), 4–5 elsewhere | 707 |
| a routine activate from STOW 565.6–567.3 (150–167 mm per leg) | 195–208, a clean trapezoid at 2.5 rev/s | 3.3–5.1 | 271 |
| columns attempt 2, 664.5–670.5 | 173–429 peak, 115–295 at p95 | 5.3–10.3 peak, 3.2–8.3 at p95 | 1428 |

Reading: the snap was not gentle. Six independent trapezoids at 30 rev/s² (an acceleration STEP,
no jerk limit) on an off-centre start put legs 1 and 2 — the pair that carries every lateral
transit — on the 10 A clamp for a 24–28 mm move and rang them past their own velocity cap, while
the four lightly-loaded legs tracked the triangle cleanly. The juggle transits reach the same
10 A peak on the same two legs at 5000 mm/s², but under a 200 000 mm/s³ jerk limit the
acceleration ramps over 25 ms rather than stepping. The agent's platform position-residual
metric (15 mm snap vs 8 mm juggle, 0.1 s moving average) is not a fair smoothness measure on a
0.15 s move and is not relied on. What transfers to the juggle question: the limiting element
is legs 1/2's current at the start of every lateral move, and a bare acceleration step is enough
to saturate and ring them — which argues for shaping/jerk, not for a higher acceleration
ceiling, and is consistent with the torque-FF plan's sizing (legs 1/2 need the most).

Proposed fix (not applied — state-machine behaviour, owner decision): on the real-fault exit,
when the fault was entered from ACTIVE with the wire armed and the legs are alive and homed
(`is_homed`, no leg ODrive error), request `deactivate` and hold FAULT until it resolves before
returning BOOT — the bridge refuses a deactivate only while the guard is latched, and the exit
condition already requires the guard clear. The guard-only path (resume to ACTIVE without
re-arming) is untouched.

## Open questions

- Whether to keep the 930 mm catch plane now that its visible cost (a waiting hand, a seat that
  the sensor only confirms at the bottom reversal) has been seen against its benefit (−0.96 m/s
  contact, 68 mm empty drop). Owner call; the verdict fix stands either way.
- The 10 % margin class (above): which of (a)–(d), and in what order.
- Whether a `caught=False` row from before this fix should be corrected in the memory. The fit
  never reads the column; the rows' `(u, y)` pairs are real throws. Left as recorded.
- Ball Butler's feed landed 44–72 mm off in x and 0.151–0.155 s early (outside the ±0.150 s
  identity band, so the catch kept the schedule) on every attempt tonight and was still caught
  each time; the calibration work is in another session's hands
  (`2026-10-06-bb-calibration-consensus-outcast.md`).
