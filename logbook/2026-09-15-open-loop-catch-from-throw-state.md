---
title: "Open-loop catch from the throw state — the catch no longer depends on the tracker"
type: feature
date: 2026-09-15
status: done
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/hand_launch.py (NEW — HandLaunchMonitor: the measured hand launch-speed ratio r = v_meas/v_cmd over the throw stroke)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py (AIM_SCHEDULE/AIM_SCHEDULE_HAND/AIM_TRACKER; _catch_aim; _hand_ratio_landing/_hand_corrected_landing; _resend_hand_corrected_catch; one CATCH-AIM log line per catch)
  - ros_ws/src/jugglebot/jugglebot/skill_node.py (catch_aim_source parameter defaulting to schedule; HandLaunchMonitor fed from /hand_telemetry; _launch_ratio handed to both executors)
  - sim/skills_gate.py (catch_aim_source on both gate configs + --catch-aim-source)
  - tests/motion/test_skills_hand_launch.py (NEW — 13 tests)
  - tests/motion/test_skills_executor.py (7 aim-source tests)
  - tests/ros/test_skill_node.py (4 parameter/telemetry-plumbing tests)
  - tests/hardware/session_skills_r3_apex_ladder.md (step 26 + the "catch is open loop" section)
  - plans/active/two-ball-skill-stack.md (§ 2.7 dated decision paragraph)
---

## Symptom

At the 2026-09-15 R3 sitting **every** self-toss ended the same way:

```
END NO_LANDING: the tracker has no landing for ball 0 by the deadline
```

13 throws, 13 identical ends. Mocap never produced a marker for the flying
ball, so the CATCH was never aimed at all: the executor waited until
`skill.t_abs_s - CATCH_DEADLINE_WINDOW_S - lead_s`, gave up, and the hand
went back to rest while the ball fell past it. The throw itself was fine —
and independently measured ~8–9 % fast (apex 1.38 m for a commanded 0.9 m
before the torque-FF work, r ≈ 1.05–1.09 across the 0.5–0.9 m ladder,
`temp/probes/hand_overspeed_2026-09-15_18-51-37.csv`).

Two separate facts were killing the catch, and only one of them is a
perception problem: the catch **could not be aimed** (no tracker landing),
and even a correctly aimed catch was **mistimed** (the plant's real flight is
~r× the commanded one — at 0.9 m and r = 1.086 that is ~74 ms).

## Discussion

### Why the aim comes from the SCHEDULE, not from perception

The owner's decision (2026-09-15): *"pivot to open-loop catches based on the
throw state. I'm inclined to say we should prefer the theoretical throw
state, but if we can reliably adjust that towards the measured throw state,
that would be even better. We shouldn't use QTM data for catch prediction
(for the time being)."*

The root cause that decides this is not "mocap was mis-configured today" but
**where the information actually is**. The schedule already knows, before the
ball leaves the cup, the landing the throw was *commanded* to achieve: the
plan's `y_d` and the release site, closed-form through `ballistics_bc`. That
is the same construction the memory-based learner treats as its command
(plan § 2.5, identity prior), so aiming the catch at it makes the catch
consistent with the throw *by construction* — there is no second estimate
that can disagree. The tracker, by contrast, is an estimate that must (a)
correlate the right ball, (b) fit a ballistic arc from a handful of frames,
and (c) do both inside the transit window. Today it produced nothing at all,
and the failure mode of "produced nothing" was an attempt that ended after
exactly one throw. A dependency whose absence is fatal, on a signal this
machine has repeatedly failed to deliver, is the wrong dependency for a
motion that is fully determined before the ball is even released.

Three alternatives were on the table and were ruled out:

* **Fix the tracker first.** The mocap gap is real and worth fixing, but it
  is a *separate* investigation (marker on a moving matte ball, `/balls`
  correlation, the cone/BB reflector masking) and it blocks every R3 rung
  behind it. The catch does not need it.
* **Wait longer for the tracker, then fall back.** This is what the
  catch-with-throw path already does ("at release, then refine",
  2026-09-13) and it is exactly what must NOT be extended to a standalone
  catch: waiting dispatches the catch *after* its own ball's release, which
  splices into the launch THROW's settle tail — measured 262 743 mm/s³ of leg
  jerk against a 150 000 limit, on every attempt — where the release-snap
  dispatch accepts at 78 326. The wait is not free; it is a refusal with
  extra steps.
* **Aim from the hand's measured state alone** (no schedule term). Rejected:
  the measured hand state is one scalar about the launch, not a landing. It
  cannot place the ball in x/y, and if the hand telemetry is silent there is
  nothing left. The schedule term is the one that always exists.

### Why the measured correction is the hand, and why it is a ratio of peaks

`schedule_hand` is the "adjust it towards the measured throw state" half of
the decision, and the constraint is that it may not reintroduce a perception
dependency. The hand's own telemetry is the only *independent* measurement of
the launch this machine has: `/hand_telemetry` carries `vel_ff_cmd` and
`vel_meas` at ~100 Hz from the hand ODrive, and the whole of the ~25 % apex
error is a launch-speed error, so one scalar captures it:

```
r = peak |v_meas| / peak |v_cmd|   over the stroke ending at the release
```

`r` scales the launch velocity, and for a vertical self-toss that is purely a
*timing* correction (`T' ≈ r·T`) — which is precisely the observed symptom
("the catch is not timed"). The general case is not approximated: the scaled
launch velocity is re-flown to the commanded landing **plane** with
`ballistics_bc.arrival_state_at_z`, so a scaled throw that also drifts
horizontally gets the drifted arrival position too.

Peaks, not a sample at release, for three reasons worth writing down because
the obvious implementation is the single sample:

1. At release the hand's velocity slope is thousands of rev/s², so a 5 ms
   stamp skew is a several-percent error in exactly the quantity being
   estimated. A peak is a stationary point of the same signal — the
   skew-insensitive feature (pinned by
   `test_the_ratio_survives_a_sampling_skew_between_the_two_channels`: a
   whole-sample lag moves `r` < 2 %, where a release-instant sample of the
   same stroke is ~10 % low).
2. A ratio is unit-free, so no `hand_mm_per_rev` gain enters and no second
   copy of it can drift out of step with the one in `skill_node`.
3. Both peaks are taken along the *commanded* peak's sign, so a hand wired
   the other way round reports the same `r`, never `1/r`.

Every refusal path in the monitor returns `None` and the caller keeps the
theoretical aim: fewer than 4 samples in the window, a commanded peak below
the 5 rev/s throw floor (the window holds no throw), a non-finite ratio, or a
ratio outside [0.75, 1.35] (that is not a 9 %-class plant error, it is a
broken window). The committed catch is always the theoretical one, so a
declined correction costs nothing — this is the "fail towards the aim that
already exists" shape, not a fail-open.

### The correction goes into the DISPATCH when it can

The first implementation re-aimed the catch one tick after committing it, and
the tests immediately showed the re-aim firing in the *same* tick as the
dispatch: a catch dispatches after its own ball's release, so the stroke is
usually already measured by then. That is two ~25 ms solves on the
orchestrator thread for one landing that was knowable at the first. So
`_catch_aim` applies the correction at dispatch time whenever the ratio
exists, and `_resend_hand_corrected_catch` exists only for the case the
dispatch cannot serve (the ratio not yet measurable). Either way the
correction happens **once** — `r` describes a stroke that has already
happened, so a second application would re-install the same landing.

### What the sim gate says, and the one number that did NOT improve

The columns gate in `schedule` mode is **20/20 makes, 0 drops, 22/22 installs
accepted, PASS on all five seeds** — where the same gate in `tracker` mode is
**14/20 makes and 14/15 installs, FAIL** (the known install-15 refusal
carried into R4). So the open-loop aim is not merely adequate in sim, it
removes a standing gate failure: the tracker's landing drifts, the re-aim
re-solves, and one of those re-solves is refused.

The R3 learner run (`--learn --policy A`) is the one that needs reading
carefully. In `schedule` mode it catches everything (makes 25, drops 0, every
seed) but **FAILs its band criterion the same way the tracker baseline does**
(`band_xy = None`, on all five seeds in both modes) — see Verification for
both runs side by side. The
learner's y still comes from the *tracker* (outcome capture is deliberately
unchanged), so the aim source cannot fix a band criterion; what this run
shows is that open-loop aiming does not *degrade* the learner run. On
hardware, with mocap blind, no outcome row finalises at all and the learner
simply stays at its identity prior — which is the correct behaviour for "no
observation, no row" and is why this unit left outcome capture alone.

## Fix

`motion/skills/executor.py` gains an aim-source vocabulary
(`AIM_SCHEDULE` / `AIM_SCHEDULE_HAND` / `AIM_TRACKER`, validated in the
constructor) and one decision point, `_catch_aim`:

* `schedule` — every catch whose ball's previous release is in this schedule
  is aimed at `_predicted_landing` and dispatched at its scheduled instant.
  No tracker call, and `_resend_live_catch` is not run at all. The one catch
  this cannot aim is columns' very first, of a ball released before `t0`; it
  keeps the wait-then-`NO_LANDING` behaviour, because nothing else is
  possible.
* `schedule_hand` — the same, plus one correction from the measured `r`,
  applied at dispatch when available and by a single fenced re-send
  otherwise (freeze `CATCH_FREEZE_S`, window floor `MIN_WINDOW_S`, both
  checked against the corrected landing as well as the committed one).
* `tracker` — the pre-2026-09-15 path, byte-for-byte: tracker first, gated by
  `_valid_tracked_landing`, predicted landing as a catch-with-throw's
  fallback, `_resend_live_catch` refining.

`skill_node` declares `catch_aim_source` (**default `schedule`** — the live
default), validates it (an unknown value logs an error and falls back to
`schedule`, never to the mode that failed today), feeds a
`HandLaunchMonitor` from `/hand_telemetry` and hands both the aim source and
the `launch_ratio` callable to every executor it builds. The executor's own
constructor default stays `tracker` so the sim gate and the R2/R3 tests keep
exercising the refine path; the two defaults are pinned apart by
`test_the_live_catch_aim_default_is_the_schedules_own_throw_state`.

Every catch now logs one line naming its aim source and the landing it used
(`CATCH-AIM skill 2: source=schedule landing=(…) t_land=…`), with `r` and
`Δt` on the `schedule_hand` path.

## Verification

* **Sim gate, columns, open loop** — `python sim/skills_gate.py --seeds
  0 1 2 3 4 --catch-aim-source schedule` (2026-09-15): **PASS**, 20 scheduled
  / **20 makes** / 0 drops / 22-of-22 installs accepted on every seed, wall
  51.6 s.
* **Sim gate, columns, tracker baseline** — `python sim/skills_gate.py
  --seeds 0 1 2 3 4 --catch-aim-source tracker` (2026-09-15): **FAIL**, 20
  scheduled / **14 makes** / 0 drops / 14-of-15 installs on every seed, wall
  48.5 s (the pre-existing install-15 refusal, carried to R4).
* **R3 learner run, open loop** — `python sim/skills_gate.py --learn --policy
  A --catch-aim-source schedule` (2026-09-15): **FAIL** on `band_xy = None`
  with **makes 25, drops 0** and `mono_xy`/`mono_flt` True on all five seeds,
  19–25 attempts, wall 737.1 s.
* **R3 learner run, tracker baseline** — `python sim/skills_gate.py --learn
  --policy A --catch-aim-source tracker` (2026-09-15): **FAIL** on
  `band_xy = None` too, with makes 24-25, drops 0 and `mono_xy`/`mono_flt`
  True on all five seeds, 5-25 attempts, wall 641.5 s. The band criterion
  fails IDENTICALLY in both modes, which is what makes it a pre-existing
  learner finding rather than a cost of the aim change.
* **Executor + helper** — `pytest tests/motion/test_skills_executor.py
  tests/motion/test_skills_hand_launch.py -q` (2026-09-15): **91 passed in
  50.42 s**.
* **Node plumbing** — `pytest tests/ros/test_skill_node.py -q`
  (2026-09-15): **54 passed in 1.60 s**.
* **Sim-gate tests** — `pytest tests/sim/test_skills_gate.py -q`
  (2026-09-15): **11 passed, 1 xfailed in 62.99 s**.
* **Scoped closing run** — `pytest tests/motion/test_skills_hand_launch.py
  tests/motion/test_skills_executor.py tests/ros/test_skill_node.py
  tests/ros/test_skills_plan_bench.py tests/sim/test_skills_gate.py
  tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py -q`
  (2026-09-15): **301 passed, 1 xfailed in 228.69 s**.

Not run here: `./run_tests.sh` (the main session's gate) and no hardware —
the operator flies this at the next sitting per the updated runsheet.

(2026-09-15, `./run_tests.sh --full`, log `temp/logs/r3_followups_full2_20260915.log`, the combined tree of the four same-day units): **PASS — parallel 6146 passed, 9 skipped, 2 xfailed in 286.47 s; serial 6 passed in 18.79 s.**

## Open questions

* The mocap gap itself is untouched: with no tracker landing, no memory row
  finalises, so the learner stays at the identity prior and `schedule_hand`
  is the only correction in the loop. Worth its own investigation before the
  learner is expected to converge on hardware.
* `r` is measured per throw and applied to that throw's own catch only —
  nothing carries it forward into the *command* (that is the learner's job,
  and the learner needs outcome rows). If mocap stays blind, a hand-measured
  outcome (apex from `r`) is the obvious candidate source for a memory row.
* `schedule_hand` has not been flown. The apex ladder's K=0 vs K=0.7 A/B is
  the natural place to also A/B `schedule` vs `schedule_hand`.
