---
title: "Learn the apex, aim the catch from the tracker — a time from a commanded instant can't be made clean"
type: investigation
date: 2026-09-18
status: done
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - plans/active/two-ball-skill-stack.md (§ 0 item 5 pins the new u/y with both measured biases; § 2.2/2.5 updated; § 2.7 gets the 2026-09-18 ordered-aim paragraph)
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py (_ball_to_msg copies landing_from_fit)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md (ABORTED_NO_RELEASE row notes the deleted observed-flight<=0 drop; NO_LANDING sentence follows the new ordered-aim rule)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py (AdmissibleBox.flight_s -> apex_m; clip/dump in apex; load converts a pre-2026-09-18 flight_s key through the exact inverse)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py (Landing.from_fit refuses a row with no converged fit; _command_u/_throw_terminal/_catch_terminal convert apex<->flight once; _catch_aim ordered rule — from_fit tracker landing, then schedule prior, then unfitted tracker; _resend_live_catch gets from_fit/tolerance/cap fences)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/learner.py (h_y third entry 0.2 s -> 0.10 m)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/memory.py (APEX_RATIO_BAND=(0.25,2.56) replaces FLIGHT_RATIO_BAND; CSV header apex_m; _check_header refuses the old header rather than reinterpreting it)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py (Skill.y_d / ThenThrow.y_d carry apex_m; new apex_from_vz)
  - ros_ws/src/jugglebot/jugglebot/skill_node.py (Landing.from_fit copied directly from BallState, not getattr; catch_aim_source default -> AIM_TRACKER)
  - ros_ws/src/jugglebot/jugglebot/tracking/ball.py (Ball.landing_from_fit)
  - ros_ws/src/jugglebot/jugglebot/tracking/matcher.py (_update_landing_prediction sets landing_from_fit True on the fit branch, False on the KF fallback)
  - ros_ws/src/jugglebot_interfaces/msg/BallState.msg (+ bool landing_from_fit)
  - sim/skills_gate.py (APEX_BAND_MM=42.0 replaces FLIGHT_BAND_S; sim tracker's Landing carries from_fit=True; --catch-aim-source help text)
  - tests/hardware/session_skills_r3.md (seat= phase note; NO_LANDING refusal row rewritten)
  - tests/hardware/session_skills_r3_apex_ladder.md (aim-source table re-marked)
  - tests/motion/test_learner.py
  - tests/motion/test_skills_admissible.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_schedule.py
  - tests/ros/test_skill_node.py
  - tests/sim/test_skills_gate.py
  - tools/admissible_sweep.py (still sweeps a flight grid, converts the resulting band to apex when it builds the box)
  - tools/probes/README.md (derives its band as sqrt(APEX_RATIO_BAND))
  - tools/probes/outcome_landing_replay.py (re-answers the 2026-09-16 question in apex terms)
  - tools/probes/tracker_bag_replay.py (dl_* header: the deadline is an upper bound on what the tracker knew, not the aim instant)
subsystem:
  - motion
  - tracking
  - ros
  - sim
  - tools
tags:
  - performance
  - testing
  - docs
---

## Symptom

The 2026-09-17 23:49 sitting — after that day's `FlightFit` tracker fix and `CAUGHT_WINDOW_S`
widening had landed — flew four throws at the identity command, then let the learner adjust on
the same hand stroke. Identity throws seated **+0.104 s** after the scheduled landing, cup parked
at the bottom moving 0.3 m/s: the operator's "smoothest" catch. The learner's very next throws
seated **+0.015 s** (cup still at 8.6 rev near the top, diving at −2.3 m/s while the ball fell at
3.4 m/s) and **+0.338 s** (cup already travelling back up at +29 rev/s) — the operator's "hits the
hand, bounces off, then the hand moves down and the ball follows". `memory.csv` read on-target the
whole time. The learner had converged its own metric while the catch got worse.

## Diagnosis

Read against Lee et al. (RAI, arXiv:2608.26800v2) side by side with `learner.py`/`executor.py`/
`memory.py` (full table in the paper-review report; page numbers are the paper's printed pages).

### 1 — the catch stroke has ~20 ms of margin, and the learner walked the plant across it

`late_catch_bag3.txt`: nominally the ball passes the cup's parked height ~0.11 s before touch-down
and the dive starts ~0.13 s before it — a ~20 ms gap against a ±40–60 ms disturbance. A ball that
arrives *late* (identity command on a 20 %-fast plant) meets the cup near the bottom, where it is
slow — smooth. A ball that arrives on time or early meets it at the top, where the cup then
accelerates away at >g. The first attempt was not luckier; it was on the safe side of a cliff. The
learner did not cause this — it removed the timing error that had been hiding it.

### 2 — the outcome is a time measured from an instant the ball does not obey

`y[2] = crossing − scheduled release` (`_finalise_outcome` (the computation this entry removes)). Two independent lags inflate it: the
09-17 ballistic fit still lags the true crossing by **+0.026..+0.053 s** (n=6, `flight_truth3.txt`
`y_rec − y_true`), and the true release itself lags the scheduled knot by **0.019..0.137 s**
knot-to-release at the SAME hand stroke (apex heights repeating to ±5 mm — the launch speed was
never the problem). Driving `y[2] → y_d` therefore drives the *airborne* time short: ground truth
at convergence is a commanded apex of 0.42 m against a desired 0.60 m, arrival speed 3.05 m/s
against the desired 3.43 m/s — the learner settled the machine ~20 % low in apex while its own
metric read on target. The paper cannot have this failure: `u` and `y` are the same physical
quantity, a position at a fixed horizon (p13–14); neither the release instant nor filter lag ever
enters it.

### 3 — the measurement noise is 2–4× the physical variation, so the fit is mostly noise

At fixed `u`, `y[2]` spreads ±0.045 s while the physical apex/speed scatter is only ±0.02 s
equivalent. Running `learner.command` on today's 22-row memory (2026-09-18): the fitted local gain
is `D[2,2] = 0.944`, against a real apex gain of **1.196 ± 0.072** — the learner's model of the one
channel it controls is wrong by 25 %. `h_y`'s flight bandwidth of 0.2 s (`learner.py:51`) is 1.5×
the *entire* explored command range (0.569–0.6996 s), so every row is a neighbour and the "local"
fit is a global fit over noise.

Two more data points, not separate root causes: one memory row (line 13) recorded
`u[2] = 0.584780`, `y[2] = 0.584823` — 43 µs apart, exactly `t_release + u_flight` — while that
flight's own ballistic fit says 0.667 s (83 ms off): the KF fallback echoed the announcement
instead of measuring anything. And the standing +55 mm lateral bias was checked as a possible
contributor to the flight-time error and **refuted**: neutralising the lateral outcome moves
`u[2]` by 1.9 ms only (`D`'s off-diagonals fit to −0.0014/+0.0108) — still a real aim defect, just
not this one.

## Discussion

**(a) The 2026-09-17 hypothesis was true but not the root.** "The tracker's landing estimate is
late" is correct and stays correct — but Diagnosis #2 shows the deeper problem survives a perfect
tracker: the learned quantity is a TIME measured from a COMMANDED instant (the scheduled release),
and no estimator can make that clean, because the physical release itself lags the knot by
0.02–0.14 s. Fixing tracker lag alone would still leave the release-lag term in `y[2]`. The
09-17 entry is not wrong; it stopped one layer short of the arrangement error underneath it.

**Why apex, not flight.** An apex read off the converged ballistic fit is a POSITION at a fixed
plane, exactly the paper's `u`/`y` (eq. 12, p13) — invariant to *when* release happened and to
*when* the tracker's estimate settled, because "how high did it go" does not care which instant you
measured the release from. A flight-time metric is invariant to neither. The identity prior is
preserved (commanded apex → observed apex, same units, same physical meaning) and the SNR
improves by roughly 4× (apex fit rms 4–7 mm ⇒ ±0.4 %, against ±0.045 s flight noise on a
0.57–0.70 s command range). The paper's own note (p14) is that a reparameterisation is only
equivalent to the original if the metric transforms with it — a time from a commanded instant is
not a transform of a position, it *adds* the release lag as a new error term. Apex is the
transform that keeps the identity-prior/same-units property; "fix the bias" was rejected because
the bias (release lag) is not constant — 0.019–0.137 s across four throws on the same stroke — so
a corrected constant would already be wrong for the next throw.

**Why landing xy, not apex xy.** The owner's ruling (2026-09-18): outcome = (landing x, landing y,
apex height). Apex xy and landing xy carry the same information up to a fixed offset — apex xy is
release position plus half the landing displacement along the flight — so using both would give
the learner two collinear columns and split any real lateral correction arbitrarily between them
instead of assigning it. Landing xy is kept because it is the quantity the catch actually has to
hit.

**(b) The non-obvious tradeoff: the schedule's flight time stops being learnable, and the catch
re-couples to QTM.** Once `u`/`y` no longer carry a flight-time term, nothing in the learner can
correct `schedule.py`'s `t_f` — that channel is now open-loop by construction, matching the paper's
own arrangement (`t_f` prescribed and fixed, Table S2/S3, pS4/pS9). The catch can no longer lean on
a learned flight time either, so `catch_aim_source` moves to `AIM_TRACKER` by default: a
converged, `from_fit` tracker landing outranks the schedule's prior, which outranks an unfitted
tracker landing, with no step waiting on a tracker call. This matches the paper's own arrangement
(catch continuously replanned from the latest ball estimate until 0.1 s before the scheduled
catch, p6) rather than inventing a new one. The coverage risk: mocap sees the ball in only
25–55 % of frames (typically 40 %), unlabelled, so a converged fit is not guaranteed every throw —
the schedule prior is kept as the second rung specifically for that gap, and an unfitted tracker
landing is kept as a third rung below the prior for the case where even the prior is absent (a
standalone catch with no previous release in this schedule). `_resend_live_catch` is gated the
same way: a re-send only fires on a `from_fit` landing outside tolerance (10 mm / 10 ms, tightened
from 1.0 mm/2 ms because that used to be inside the tracker's own noise) and capped at 2 per catch,
so an unconverged or noisy tracker cannot spend the re-solve budget the paper's continuous replan
would otherwise burn every tick.

**(c) Why this beats "fix the bias" for reasons the code alone won't show.** Fixing the release-lag
bias directly (subtract a measured constant from `y[2]`) was the fast path and was rejected because
the bias is not one number — it is a range (0.019–0.137 s) that moves with the stroke's cold/warm
state and with which side of the ~20 ms margin cliff (Diagnosis #1) the throw lands on. A constant
correction tuned to today's data would silently mis-tune the next sitting's data, and nothing in
the code would flag it — the metric would still read "converged" the way it did before this
change. Apex removes the term rather than estimating it, which is the only fix that does not need
re-validating every time the plant's timing shifts.

**What unit 1 deleted and kept, and why the freeze machinery survives an apex metric.**
`_finalise_outcome`'s `observed flight <= 0` branch and `sim/skills_gate.py`'s `t_f_nominal` are
gone — redundant once the reference is `apex_m` itself, and `_consider_landing`'s existing "must
be at or after release" test already covers the case the deleted branch existed for. Kept
deliberately: `_landing_instant`, `_bound_by_next_release`, and the freeze rule. They answer "WHICH
flight is this estimate about?" — a question an apex reading does not settle any more than a
flight-time reading did, because a held ball's "lands now" estimate is not bounded away from the
commanded apex either. `FLIGHT_RATIO_BAND` becomes `APEX_RATIO_BAND = (0.25, 2.56)`, written down
as the exact SQUARE of the retired band `(0.5, 1.6)` (flight ratio = release-speed ratio, apex goes
as its square) — no new physical judgement was made, the old one was re-expressed.

**Hyperparameter re-pin — left open.** `γ = 1e-2` (paper 0.001) and `η = 0.2` (paper's own bolder
0.3) were tuned against a flight-time metric with 1.5×-of-range bandwidth; they are not re-derived
here. `h_y[2]` moved 0.2 s → 0.10 m (≈3× the measured ±0.02–0.04 m apex scatter — the same margin
the old value gave the flight-time channel, applied to the new units), but `γ`/`η` themselves are
carried to a `probe_learner.py` re-run on real apex data before they are touched, per this
codebase's rule against tuning noisy constants independently.

## Fix

1. **`memory.py`** — `APEX_RATIO_BAND` replaces `FLIGHT_RATIO_BAND`; CSV header is
   self-describing (`u2_apex_m`/`y2_apex_m`); `_check_header` raises rather than reinterpreting an
   old-format file (the numbers would silently parse as apex meters instead of flight seconds).
2. **`schedule.py`** — `Skill.y_d`/`ThenThrow.y_d` carry `(zeros(2), apex_m)`; new
   `apex_from_vz(vz_m_s)` shares the module's one gravity constant with `flight_s`/`apex_m`.
3. **`executor.py`** — `Landing.from_fit: bool`; `_consider_landing` refuses a row with
   `from_fit=False` ("no converged ballistic fit for this flight"), the catch AIM path is
   untouched by that refusal; `_command_u`/`_throw_terminal`/`_catch_terminal` convert
   apex↔flight exactly once each; `_catch_aim` is the ordered rule (from_fit tracker landing →
   schedule prior → unfitted tracker landing, no step waits); `_resend_live_catch` adds the
   `from_fit`-only / 10 mm-10 ms tolerance / 2-per-catch fences, each refusal logged once per
   (catch, reason).
4. **`admissible.py`** — `AdmissibleBox` stores/clips/dumps `apex_m`; `load` accepts a
   pre-2026-09-18 `flight_s` key and converts it through the exact inverse of `schedule.flight_s`.
5. **Tracker path** — `BallState.msg` gains `bool landing_from_fit`; `matcher.py` sets it True on
   the fit branch, False on the KF fallback; `ball_tracker_node.py` copies it through;
   `skill_node.py` reads it directly (not `getattr`) so a stale `jugglebot_interfaces` build fails
   loudly instead of silently starving the learner of rows. `colcon build --packages-select
   jugglebot_interfaces jugglebot` (2026-09-18): 2 packages finished, 3 min 43 s.
6. **`sim/skills_gate.py`** — `APEX_BAND_MM = 42.0` replaces `FLIGHT_BAND_S = 0.020` (same
   physical tolerance: `dh/dt_f = g·t_f/4 = 2.10 m/s` at 0.9 m, so 20 ms ≈ 42 mm); the sim
   tracker's `Landing` carries `from_fit=True` (its estimator is already a batch parabola).
7. **`skill_node.py`** — `catch_aim_source` default → `AIM_TRACKER`; the typo-fallback path is
   now the live default rather than a fallback arm of an A/B, since flying the other arm would
   silently diverge from what the runsheet names.
8. **Docs** — plan § 0 item 5 / § 2.2 / § 2.5 / § 2.7 updated in place (dated 2026-09-18
   paragraphs, not new sections); `INVARIANTS.md`, the two hardware-session runsheets, and
   `tools/probes/README.md` / `outcome_landing_replay.py` / `tracker_bag_replay.py` updated to
   match the new metric and the new aim rule.

## Verification

| what | date, command | result |
|---|---|---|
| interfaces + package build | 2026-09-18, `colcon build --packages-select jugglebot_interfaces jugglebot` | **2 packages finished, 3 min 43 s** |
| skill_node + sim gate (apex) | 2026-09-18, `pytest tests/ros/test_skill_node.py tests/sim/test_skills_gate.py -q -p no:cacheprovider` | **74 passed, 1 xfailed in 35.72 s** |
| executor (apex, unit 1) | 2026-09-18, `pytest tests/motion/test_skills_executor.py -q -p no:cacheprovider` | **101 passed in 137.39 s** |
| motion + skill_node + sim (unit 1, merged) | 2026-09-18, `pytest tests/motion tests/ros/test_skill_node.py tests/sim -q -p no:cacheprovider` | **2741 passed, 4 skipped, 2 xfailed in 852.92 s** (earlier identical run before the added test: 2739 passed, 4 skipped, 2 xfailed in 977.38 s) |
| executor + skill_node + sim (unit 2, ordered-aim, merged on unit 1) | 2026-09-18, `pytest tests/motion/test_skills_executor.py tests/ros/test_skill_node.py tests/sim -q -p no:cacheprovider` | **982 passed, 4 skipped, 2 xfailed in 471.68 s** (`test_skills_executor.py` alone: 107 passed in 16.4 s) |
| sim learner gate, policy A | 2026-09-18, `python sim/skills_gate.py --learn --policy A --seeds 0 1` | **FAIL (exit 1), 103.6 s** — apex band entered at throw 4 on both seeds (criterion ≤5), monotone in xy and apex, 0 drops, 22 makes/seed; FAIL only because `band_xy` is never entered (pre-existing, plan R3 item (k), not moved by this work) |

(2026-09-18, `./run_tests.sh --full`, log `temp/logs/apex_units_full_20260918.log`, the units' tree before one fix): **1 failed, 6267 passed, 9 skipped, 2 xfailed in 306.70 s; serial 6 passed** — the failure was `tests/ros/test_skills_plan_bench.py::test_self_toss_rehearsal_grows_memory_and_the_learner_changes_the_command`: the plan bench's analytic tracker built `Landing` without `from_fit`, so every row was refused and the memory never grew; fixed by marking its two landings `from_fit=True` (an analytic tracker is a converged fit, as `sim/skills_gate.py` already says). (2026-09-18, `./run_tests.sh`, log `temp/logs/apex_units_gate2_20260918.log`, the committed tree): **PASS — 6230 passed, 9 skipped, 1 xfailed in 219.23 s; serial 3 passed in 7.77 s.**

## Carried

* **Cup downward-acceleration contract + dive re-phase** — bound `|a_cup,down| ≤ κg` (`κ<1`) in
  `feasibility.validate_cycle`, the mirror of the paper's `a_throw = g` release constraint, for the
  measured ~15 m/s² > g cup dive; then re-phase the catch stroke onto the condition that already
  worked (contact near the bottom of the dive, ~0.10 s after the scheduled landing) or park the cup
  low enough the ball cannot reach it before the dive starts. Owner input needed — this is Diagnosis
  #1's root, not yet touched.
* **The +55 mm lateral trim.** Possibly the same root cause as the cup-accel item: with
  `a_cup,z → g` the apparent gravity's vertical part vanishes and `tilt_to_receive` needs a huge
  angle for any lateral aim — the 12° saturation from
  [2026-09-16-banking-saturates-on-small-lateral-offsets](2026-09-16-banking-saturates-on-small-lateral-offsets.md).
  Worth checking before any other fix; if it holds, the cup-accel fix unpins lateral authority as a
  side effect. Pre-registered fallback: ship the measured static −55 mm offset and leave
  `learner_lateral_authority_mm = 0`.
* **γ/η/h_y re-pin on real apex data** — re-run `probe_learner.py` against apex-metric rows before
  touching `γ = 1e-2` / `η = 0.2` independently; if deferred, raise `η` to the paper's 0.3 as an
  interim.
* **`config/generated/admissible_box.yaml` still carries `flight_s` keys**, loaded through
  `admissible.py`'s conversion by design. The next `tools/admissible_sweep.py` run rewrites it with
  `apex_m` directly, after which the `flight_s` legacy-load branch has no live caller — retire it
  then, not before.
* **Mocap coverage (25–55 %, typically 40 %)** is the live risk under the new `AIM_TRACKER`
  default: a throw with no converged fit falls through to the schedule prior, which is exactly the
  time-from-commanded-instant metric this entry moved away from for the OUTCOME row (the catch AIM
  path is a separate concern from the learned outcome and is allowed to use the prior). Watch the
  `RESEND-SKIPPED NO-CONVERGED-FIT` rate at the next sitting.
* **`sim/skills_gate.py`'s xy-band xfail** (plan R3 item (k), the sim's own +8.5 mrad aim error
  with lateral authority pinned to 0) is untouched — the `band_xy` FAIL above is that same item,
  not a new one.
* **`colcon build` is owed on the live tree before any sitting** — `ros_ws/install`/`build` still
  hold the `schedule`-default `skill_node` and the pre-ordered-aim `executor` until rebuilt.
