---
title: "Skill-stack R3 — a memory-based learner at one site: the sim gate MET, the catch-with-throw handoff traced through three withdrawn hypotheses to a release-snap install, and a columns regression a correct tracker anchor exposed"
type: feature
date: 2026-09-13
status: in-progress
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/{learner,memory}.py (new)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/{executor,schedule}.py, INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py (_TILT_BLEND_MIN_KNOTS floor)
  - ros_ws/src/jugglebot/jugglebot/skill_node.py, ball_tracker_node.py
  - sim/skills_gate.py (--learn harness)
  - config/generated/admissible_box.yaml, tools/admissible_sweep.py
  - tools/probes/{skills_single_site_sweep,throw_outcome_bag_probe}.py (new)
  - tests/hardware/skills_plan_bench.py, session_skills_r3.md (new)
  - tests/motion/test_learner.py (new), test_skills_{admissible,executor,schedule,segments}.py
  - tests/ros/test_skill_node.py, test_skills_plan_bench.py
  - tests/sim/test_skills_gate.py
subsystem:
  - motion
  - ros
  - sim
  - tools
tags:
  - safety
  - testing
---

# R3 — a memory-based learner at one site

## Outcome

The R3 sim criterion is **MET**: `python sim/skills_gate.py --learn --policy A`
run twice at seeds 0–4 gave identical results (`temp/reports/skills_gate_learn_A_run{1,2}.json`,
204.7 / 205.9 s, 2026-09-13) — all seeds **PASS**, `band_xy` 3, `band_flt` 3,
both monotone checks T/T, 3 attempts, 25 makes, 0 drops. Policy B, the
back-to-back chained form (`skills_gate_learn_B_run1.json`, 191.4 s): **PASS**
on band 5/5, 1 attempt (the full 25-throw chain), 0 drops. Getting there took
the whole rung: a schedule/executor precondition ladder, a memory-based
learner with a persisted per-plant memory, a traced planner root cause behind
an aim-box collapse, and three rounds of bisection on why a catch-with-throw
handoff refused after the box was already certified feasible.

The **hardware gate is the one outstanding item** — no powered sitting this
rung. Its runsheet, `tests/hardware/session_skills_r3.md`, is written (sim
triple filled, Findings A and B resolved) but not yet flown.

Separately, this rung's own fixes exposed that R2's sim gate (20/20 × 5 seeds,
logged 2026-09-12) was **partly propped up by a tracker-anchor drift bug**:
with the drift bug the columns pattern (`python sim/skills_gate.py --seeds 0`)
still passes 20/20; with the correct anchor landed this rung it fails
`LIMIT_JERK` at install 15. This is filed, not fixed, at R3 (§ Discussion 5f,
§ Open Questions) — the owner ruled it orthogonal to the single-site learner
and not safety-critical (refused before motion), and carried it to R4.

The learning-stack deletion (`toss_ilc.py`, `toss_trim.py`, `toss_cal*.py`,
`toss_record.py` and the miner/parity/decomposition probes it feeds, per
`census_learning_stack.md`) is scoped and owner-approved to run **after** the
chained sim passed, which it now has; **the deletion has landed in this
worktree** (`git grep` hits for the learning-stack names, 2026-09-13: 2535 before → 1069 after the re-home and out-of-node deletion → 698 after the FSM-node removal, of which 91 are outside `*.md` and every one is a dated retirement note, provenance prose, or a live FSM module's unrelated `toss_*` name (`scratchpad/deletion_counts_{before,after}.md`); `reload_coordinator_node.py` 12 911 → 10 524 lines).

## Discussion

Six things here are why this entry takes the full investigation form: a
withdrawn-and-corrected hypothesis chain on the handoff splice (§5), a
non-obvious tradeoff on how a chained throw's command is fixed (§6), and an
approach (release-snap install refined by re-sends) that beat the plan's
original wait-for-landing rule for reasons the code alone won't tell a future
reader (§4).

### 1. The learner hyperparameters: the plan's § 2.5 start point never learns, and γ depends on units

The owner's decision (2026-09-13) fixed the learner at `k16 kmin2 hx0.01
hy(.05,.05,.2) γ1e-2 η0.2` with a new (P1,P1) box at 150k. That is not the
plan's § 2.5 starting point: the learner probe
(`probe_learner.py` + `probe2_out/`, scratchpad, 2026-09-13) found the § 2.5
start point (`h_y 0.02`) **never learns** — against the ~37 mm / 93 ms
cold-start error the kernel weights (~1e-11) are negligible next to `γ`, so the
fit collapses onto the prior and the command never moves. A fully underflowed
weight set (Σw = 0) is a separate hazard: unit a's learner (`learner.py`, 167
lines) needed a hard guard (`ValueError` at the weight sum) added by the main
session because it otherwise produced a NaN command —
listed under "Defects found this rung" below. The adopted bandwidth,
`hy(.05,.05,.2)`, is one bandwidth per outcome component — landing x (m),
landing y (m), flight (s); `γ` is only meaningful in those SI units (it
competes with Σw·(command spread)²), which is why the probe tuned bandwidth
and `γ` together rather than inheriting the plan's numbers unchanged.

### 2. The outcome definition: the tracker plane moved to 830, the probe's candidate rule was fixed, and release lag is left for the sitting

`ball_tracker_node`'s `landing_z` moved to 830 (`sites.CATCH_CUP_Z_MM`); `y` is
the native `/balls` prediction; flight is the tracker's crossing time minus
the schedule's release time. This is also the fix to an R2 defect: the CATCH
skill had been aiming at the tracker's raw 809.08 mm plane instead of the
830 mm sites constant (§ Fix). The FSM's own catch plane moves with it —
accepted, not treated as a separate migration.

The outcome probe (units 1 and 1b, `throw_outcome_bag_probe.py`, 2026-09-13)
tried several candidate landing-detection rules against nine hand-checked
09-07 bag landings and measured ground truth at RMS 4.7 mm. The rule adopted,
candidate e — take the last tracker update above 880 mm and register the
crossing at the 830 mm plane rather than trusting whatever sample happens to
land nearest it — reads 15.7 mm / 3.6 ms against that ground truth. Only 10 of
27 candidate crossings were usable (17 excluded); that 10/27 usable rate is
the perception blind rate carried to Open Questions below, and
`OUTCOME_GUARD_S = 0.012 s` discards tracker estimates sampled within 12 ms of
their own predicted crossing — about 50 mm above the plane at the ~4.2 m/s
arrival, the same cut as candidate e's last-update-above-880-mm rule.

Separately, old bags show 156–371 ms of commanded-to-physical release lag,
unexplained. Decision 10 leaves the flight definition unchanged and defers the
measurement to the sitting; the risk flagged at decision time was explicit —
if the lag is real, the sim's flight band could fail on hardware and the box's
0.750 s flight floor (§3) would refuse the corrected throw rather than admit
it, which is exactly the tradeoff in §6.

### 3. The single-site cycle and the box: zero leg jerk with one ball, a launch-gate collapse, and the traced pin-blend root cause

A single-site probe (`skills_single_site_sweep.py`, 2026-09-13) is the
baseline the rest of the rung is read against: one ball, repeated
catch-and-throw at the same site, gives **zero leg jerk in both the launch and
chained forms** — hand peaks 3097 / 3341 rev/s², reach ~30 mm, and an
un-prelevelled run hits 174 772 mm/s³ (over the 150k ceiling) against 19 773
prelevelled — confirming the R1 pre-level porting (§ Fix) is load-bearing, not
cosmetic. A cold-start first catch lands 38 mm off-plan at 74 876 mm/s³.
Against that baseline, R2's earlier 188 000 mm/s³ single-site-looking number
is now understood as an artefact of the two-ball splice seed (the catch was
seeded at the *other* ball's release) — not a real single-site figure.

Adding a nonzero aim, though, collapsed the admissible box: unit h2's launch
gate (the box now also gates launch-from-rest, not just chained catches) left
the (P1,P1) cell's xy admissible set at **(0, 0)** — no nonzero aim was
plannable at all, only the flight window survived (0.750–0.857 s). Decision
11 pre-registered a fallback in case this couldn't be traced in one unit
(fly the launch THROW at identity; let the learner command only
catch-carried throws, in a ±40/±30 mm chained box) rather than let an
untraced planner defect block the rung.

It didn't come to that. Unit i (an Opus investigation, per the token-budget
policy — one planner-math unit) traced the mechanism:
`cup_realize._accel_bounded_schedule`'s pin-blend width `L` is sized from the
pin gap, which produces a 2-knot corner at the release seam (knot 1 plus a
one-sided finite-difference slope break) for *any* nonzero aim — independent
of aim magnitude, matching the ~125k aim-jerk figure logged as an open defect
before the trace. The owner adopted a floor with a contract, not a one-off
relaxation: `_TILT_BLEND_MIN_KNOTS = 8.0` plus
`test_launch_leg_jerk_scales_with_the_aim_offset` (which fails at floor 1.0,
confirmed by an in-process patch) — 398 planner+skills tests pass (not
serial, 2026-09-13). Banking saturation (hand deceleration exceeding g forces
a ~12° bank, the mechanism behind R2's 188k number) is a related but distinct
effect, logged for R4 rather than folded into this floor.

With the floor landed, the box was regenerated at 150k with the launch gate
in place: (P1,P1) xy **[−40, 40] × [−30, 30] mm**, flight 0.750–0.857 s,
27.8 s wall (`tools/admissible_sweep.py`, 2026-09-13) — the full range h2 had
collapsed to zero. The regeneration also carries R3's shared
`limits.leg_jerk_mmps3` change (R2's 200 000 → 150 000) into the columns
boxes: (P1,P2) landing_xy_m x narrowed [−0.02, 0.02] → [−0.02, 0.01]; (P2,P1)
landing_xy_m y narrowed [−0.02, 0.02] → [−0.01, 0.02]
(`config/generated/admissible_box.yaml`) — any columns caller of
`check_limits` at R2's 200k now refuses.

### 4. Catch timing: wait-for-landing, to the settle-tail hypothesis, to install-at-release-then-refine — and a stale sim tracker

Decision 9 (start of rung) was: a single-site CATCH with no landing yet at
dispatch time *waits* for the first landing, refusing `NO_LANDING` only at the
deadline `t_land − 0.278 − lead`. Unit h1 hit this early:
`compile_self_toss` was dispatching the catch before its own release, so
every attempt refused `NO_LANDING` — fixed inside h1 alongside raising
`FLOOR_LIFT_S` from 1.0 to 1.5 s (1.0 s refused `LIMIT_JERK` at 177 241 mm/s³
from the centred park).

Once the aim-jerk floor (§3) landed and the chained sim could actually run,
every attempt died at the **first** catch-with-throw handoff, refusing
`LIMIT_JERK`. The working hypothesis (main session) was that wait-for-landing
moves the splice past the detach cone into the just-completed throw's settle
tail — the same split-form infeasibility R2 had found and fixed by making a
same-site catch-and-throw one continuous window — and that the box cell only
certifies the *snap* seed, not a deferred one. Three rounds of bisection
(§5) eventually converged on a different, more specific mechanism than
originally guessed: a deferred-dispatch splice lands inside the *launch's*
settle tail specifically, and only splicing exactly at the release snap
avoids it (probe_handoff_capture_v2, 2026-09-13: baseline 262 743 mm/s³
refused, every other candidate input still refused, only the release-snap
splice flips to 78 326 mm/s³ accepted).

The owner's decision 12 (2026-09-13, after a plain-language re-explanation)
supersedes decision 9 for this case only: **a catch-with-throw installs at
the scheduled release (the snap), aimed at the predicted landing** — the
ballistic arrival computed from the ball's own previous release, not a
tracked observation — **and is refined afterward by tracker re-sends**;
standalone catches keep wait-for-landing, and a cold start is single-throw
attempts throughout. Unit l landed `_previous_release` and
`_predicted_landing` and moved catch-with-throw dispatch to the scheduled
instant (`test_an_end_to_end_self_toss_schedule_fails_today_without_the_
scheduled_dispatch_fix`: old code 3/7 `LIMIT_JERK`, new code 7/7; 208 tests
pass). `--learn --policy A --seeds 0` then passed 14 attempts, but warm
(post-first-throw) attempts still ended `WINDOW_TOO_SHORT` / `LIMIT_JERK` at
the **second** catch-with-throw — one handoff further than before, not fixed.

The third capture (`probe_handoff_capture_v3`/`flip_v3.py`, 2026-09-13) found
why: the *sim tracker* (`sim/skills_gate.py`) returns the **frozen landing of
the flight that just ended** whenever a ball's landing estimator hasn't reset
yet (it only resets at the next release), and the executor was using that
stale value instead of `_predicted_landing`. Catch 1's eleven re-sends were
all refused (nine `LIMIT_JERK`, two `LIMIT_ACC`) chasing a landing that was
already in the past. Patching the tracker to return `None` while a ball isn't
airborne took the run from 25 attempts down to 3, **25/25 makes, 0 drops —
the chain runs.** Unit n made this a first-class executor invariant rather
than an artefact of the sim tracker's reset timing: `_valid_tracked_landing`
(applied at both dispatch and re-send) accepts a tracked landing for a catch
only if its `t_land` is after that ball's previous release instant. On
hardware this staleness class is not exposed today, because
`skill_node._maybe_announce` replaces `_correlation[ball]` on every carried
release, so a stale correlation naturally can't answer — *(superseded 2026-09-16: that side effect was the contamination path and is gone; correlation is per release now, see `2026-09-16-tracker-correlation-follows-the-flight-in-progress.md`)* — but the guard is
adopted anyway so the executor enforces the invariant once, centrally,
instead of relying on that node-level side effect.

### 5. What was ruled out

Six hypotheses were tested and killed (one was later corrected) before decision
12 and unit n's fix converged. Each is recorded so a future session doesn't
re-run the same probe:

- **Settle-tail cause** (the splice lands in the throw's own settle tail).
  `probe_steady_seed.py` (2026-09-13) measured seed placement A (snap), B
  (deferred +0.05/+0.15/+0.30 s), and C (refine from A with the carried
  release fixed): A accepts at 0 jerk (hand 3393), B accepts at 0 jerk at
  every offset tested. **Dead** as originally framed — but see the correction
  below.
- **Lateral-velocity noise cause.** Bisection unit k (`probe_handoff_capture/
  bisect.py`, 2026-09-13) found a refusal at 235 887 mm/s³ with an n=3
  estimator reading landing velocity (50.7, 87.3, −4603) mm/s; forcing the
  lateral component to zero made it *worse* (306 052 mm/s³), not better.
  **Dead.**
- **"Seed placement is not it"** (the B-probe's initial conclusion above).
  **Corrected**: `bisect_v2.py` (2026-09-13) found that of every candidate
  input tried — including feeding the learner's command as pure identity —
  only splicing exactly at the release snap flips the result from a 262 743
  mm/s³ refusal to a 78 326 mm/s³ accept. The earlier "not it" verdict had
  only tested identity-style inputs, not the release-snap splice itself;
  seed placement **is** the converged cause, just not in the form first
  guessed (a deferred splice lands in the *launch's* settle tail, not the
  catch's).
- **Unlearned landing error at a fixed carried release.** C's surviving
  explanation after the settle-tail and lateral-velocity kills: j's probe run
  had the xy box collapsed (§3, before the floor landed), so the learner
  could never correct the aim, and an unlearned error was the leading
  suspect. Once the box was regenerated and the chained sim re-run
  (`skills_gate_learn_{A,B}_2026091318{5306,5406}.json`, 2026-09-13), the
  learner *did* converge (err_xy 37.1 → 0.1 mm, flight 89 → 0.1 ms over the
  run) — yet policy B still ended 25/25 attempts at `LIMIT_JERK` on the
  first catch-with-throw even fully converged. **Dead**, pre-registered
  before the re-run.
- **`_previous_release` reads the wrong release** (finds the initial launch
  instead of the carried release). Read by the main session after unit l
  landed: it reads correctly. **Dead.**
- **The floor (§3) caused the columns regression.** An open regression
  surfaced mid-rung: `python sim/skills_gate.py --seeds 0` (the columns
  pattern) went from 20/20 passing after unit d to failing 14/15 installs.
  Agent l had called this "pre-existing"; that call was wrong — it was
  introduced this rung. In-process bisection
  (`temp/logs/r3_columns_floor{8,1}.log`, 2026-09-13) found floor 8 vs floor
  1.0 gave an *identical* failure, and so did `_predicted_landing → None` and
  `CATCH_DEADLINE_WINDOW_S = inf`, alone and combined — ruling out both the
  aim-jerk floor and the new catch-dispatch machinery. **Dead** as the cause;
  the true mechanism (unit m, §6 of Fix / Outcome) was the tracker-anchor fix
  from R3-j removing a drift bug that had been silently propping up R2's
  columns gate.

### 6. Tradeoffs accepted

- **Chained lag accepted.** A catch-with-throw's `then_throw` command `u` is
  computed once, at CATCH dispatch; any re-sends the catch triggers reuse
  that same `u` rather than recomputing it against a fresher landing
  estimate. This is why decision 12's release-snap install is paired with
  "refined by tracker re-sends" for the *landing aim*, not the throw command
  itself — the throw command is deliberately allowed to lag one catch behind
  the freshest observation.
- **The FSM's catch plane moves too.** Moving `landing_z` to 830 (§2) is not
  scoped to the skill path; the FSM's own catch geometry now reads the same
  constant. Accepted rather than forked, to avoid two live definitions of
  the cup plane.
- **The 830 mm plane also moves the matcher's association floor.** `tracking/
  matcher.py` ignores markers below `landing_z + min_height_above_landing_mm`
  (50 mm), so that floor moves 859 → 880 mm for the FSM too (`ball_tracker_
  node.py:44`, `matcher.py:336`) — tracks stop associating 21 mm earlier.
  Accepted as part of the same landing-plane move, not treated separately.
- **`FLOOR_LIFT_S` 1.5 s.** Raised from the plan's 1.0 s because 1.0 s refuses
  `LIMIT_JERK` at 177 241 mm/s³ from the centred park (unit h1); the extra
  0.5 s is dwell budget the session no longer has for anything else at the
  operating point.
- **The box can't lengthen a flight.** The admissible box's flight window is
  a fixed geometric constraint (0.750–0.857 s at the adopted cell, §3), not
  something the learner can push against per-throw. If the sitting confirms
  the 156–371 ms release lag is real (§2, decision 10's flagged risk), a
  systematically shifted flight time fails the band and the box refuses the
  corrected throw outright rather than admitting a longer one — the fix for
  that, if needed, is a box or definition change, not something the learner
  can absorb on its own.

## Fix

- **`motion/skills/learner.py`** (new, 167 lines): the memory-based learner —
  k16, kmin2, hx0.01, hy(.05,.05,.2), γ1e-2, η0.2 (§1) — plus a `ValueError`
  guard at the weight sum for the underflow case the probe exposed (a
  Defect, below).
- **`motion/skills/memory.py`** (new, 182 lines): per-plant persistence at
  `temp/learn/<plant_id>/memory.csv`; `plant_id` is a `skill_node` parameter,
  default `'jugglebot'`.
- **`motion/skills/schedule.py`**: `compile_self_toss` plus an opening REST at
  the site (the floor lift) (unit b1).
- **`motion/skills/executor.py`** (now 1186 lines, `wc -l`): the learner hook,
  `NO_ADMISSIBLE_COMMAND`, and outcome capture on `done` (unit b1); the precondition
  ladder (`Observations`, `precondition_refusals`, `MODE_CHANGED`,
  `NO_RELEASE` — unit b2); wait-for-landing plus the raised `FLOOR_LIFT_S`
  (unit h1); the launch-from-rest box gate (unit h2); `_previous_release` +
  `_predicted_landing` and the scheduled-dispatch install for
  catch-with-throw (unit l, decision 12); the `_valid_tracked_landing` guard
  at both dispatch and re-send (unit n). Plan § 0 module-size line: 593 → 1186
  lines, past the ~500 guideline, because the executor is the ONE orchestrator
  the sim gate and `skill_node` share (§ 0 Coherence) — every addition here
  answers a measured defect or a PORT@R3 row, and splitting the ladder or the
  catch dispatch into a sibling would give the sim and the robot two call paths
  to keep in step. A split along those seams is an R4 option, not an R3 need.
- **`motion/skills/admissible.py` / `tools/admissible_sweep.py` +
  `config/generated/admissible_box.yaml`**: the single-site chained cell at
  150k (unit c), then regenerated with the launch gate and the aim-jerk floor
  in place (§3): (P1,P1) xy [−40, 40] × [−30, 30] mm, flight 0.750–0.857 s.
- **`motion/trajectory/cup_realize.py`**: `_TILT_BLEND_MIN_KNOTS = 8.0`, the
  contract floor for the pin-blend width traced in unit i (§3).
- **`skill_node.py`**: the threading fix (a Defect, below), the tracker plane
  (830), announce + latch, learner/memory/box wiring, and the
  `skills/start_self_toss` service (unit e1); observations, pre-level,
  `skills/check`, and `stop` keeping finalising work correctly (unit e2, 83
  tests pass).
- **`ball_tracker_node.py`**: `landing_z` → 830 (`sites.CATCH_CUP_Z_MM`), the
  other half of the tracker-plane defect fix.
- **`sim/skills_gate.py`** (`--learn` harness; measured 1219 lines, `wc -l` —
  628 → 1189 at unit d's landing per the R3 checklist, plus j's tracker-anchor
  fix and n's None-while-not-airborne patch since): one stream loop with two
  callers, the park-rev epsilon fix (unit d); `t_land` anchored to the
  tracker's last sample rather than "now" (unit j, D2 — the sim tracker had
  been drifting after capture); the tracker returns `None` while a ball is
  not airborne rather than a frozen prior landing (unit n, D1's actual fix).
- **`tests/hardware/skills_plan_bench.py`** + **`session_skills_r3.md`**
  (new): `--pattern self-toss` and the R3 runsheet (unit g; the sim triple,
  the resolved Findings A/B, the box, decision 12 and policy A filled in by
  unit g2); its results table stays empty for the sitting.
- **Probes**: `tools/probes/throw_outcome_bag_probe.py` (new, units 1/1b, §2)
  and `tools/probes/skills_single_site_sweep.py` (new, unit 4, §3).
- **`INVARIANTS.md`**: 8 rows moved PORT → KEEP (unit b2).

### R2 defects found and fixed this rung

- **`skill_node` single-threaded spin.** `rclpy.spin` plus a blocking install
  wait meant every live install would time out — nothing R2's own gate could
  see, since it never drove the real node. Fixed in unit e1.
- **Catch aimed at the wrong tracker plane.** CATCH was aiming at the raw
  809.08 mm tracker plane instead of 830 mm (`sites.CATCH_CUP_Z_MM`). Fixed
  by moving the tracker's `landing_z` to 830 (§2, e1).

### The learning-stack deletion

Owner-approved (2026-09-13, `census_learning_stack.md`): delete
`toss_ilc.py`, `toss_trim.py`, `motion/toss_cal.py`, `toss_record.py` and the
fit/grid/analysis modules and probes it feeds, once the chained sim passes.
It now has. **(`git grep` hits for the learning-stack names, 2026-09-13: 2535 before → 1069 after the re-home and out-of-node deletion → 698 after the FSM-node removal, of which 91 are outside `*.md` and every one is a dated retirement note, provenance prose, or a live FSM module's unrelated `toss_*` name (`scratchpad/deletion_counts_{before,after}.md`); `reload_coordinator_node.py` 12 911 → 10 524 lines)** — this rung's
worktree diff (`git status`) shows both deletions landed; f1 (re-home the live
`toss_record.py` symbols into `ball_possession.py`) and f2 (the FSM node
surgery) are both done.

## Verification

All runs below are 2026-09-13 unless noted.

- **Baseline**: `./run_tests.sh --full` @ `0778ca5`: **6552 passed, 9
  skipped, 1 xfailed + 6 serial, PASS.**
- **Single-site probe** (`tools/probes/skills_single_site_sweep.py`): 0 leg
  jerk both forms; hand 3097/3341 rev/s²; reach ~30 mm; park refuses
  `HAND_STROKE`; un-prelevelled 174 772 > 150k, prelevelled 19 773;
  cold-start F1 catch 38 mm off-plan at 74 876 mm/s³.
- **Learner probe** (scratchpad `probe_learner.py` + `probe2_out/`): the § 2.5
  start point (`h_y 0.02`) never learns.
- **Outcome probe** (`tools/probes/throw_outcome_bag_probe.py`, referencing
  09-07 bags): ground truth RMS 4.7 mm; candidate e 15.7 mm / 3.6 ms (n=9);
  10/27 candidate crossings usable; `OUTCOME_GUARD_S` 0.012 s; release lag
  156–371 ms.
- **`pytest tests/motion/test_learner.py -q`**: 31 passed (plus the
  non-finite-weight fix from the main session).
- **Admissible sweep tests** (unit c, single-site chained cell):
  `pytest tests/motion/test_skills_admissible.py -q` (2026-09-13): 25 passed
  in 1.14 s.
- **`compile_self_toss` + ladder** (unit b1/b2): landed; INVARIANTS 8 rows
  KEEP.
- **`skill_node` e2**: `pytest tests/ros/test_skill_node.py
  tests/motion/test_skills_executor.py -q` (2026-09-13): 83 passed in 1.90 s.
- **`sim/skills_gate.py --learn` harness** (unit d, discrete form only):
  learner converged by throw 3 on 5/5 seeds, bit-identical twice; chained
  form not yet run at this point (1 throw/attempt dispatch defect, fixed
  later).
- **h1 landed** (wait-for-landing + `FLOOR_LIFT_S` 1.5): passing, 1 expected
  fail (the sim learner test with the xy box still collapsed at that point):
  `pytest tests/motion/test_skills_executor.py tests/motion/test_skills_schedule.py
  tests/ros/test_skill_node.py tests/ros/test_skills_plan_bench.py
  tests/sim/test_skills_gate.py -q` (2026-09-13): 202 passed, 1 failed.
- **h2 landed** (launch-from-rest box gate):
  `pytest tests/motion/test_skills_admissible.py -q` (2026-09-13): 26 passed
  in 1.78 s.
- **Aim-jerk floor** (`_TILT_BLEND_MIN_KNOTS = 8.0` +
  `test_launch_leg_jerk_scales_with_the_aim_offset`, which fails with the floor
  patched to 1.0): `pytest tests/motion/test_{cup_realize,cup_cycle,unified_cycle,
  unified_cycle_splice,validate_cycle,validate_cycle_vectorised,skills_segments,
  skills_admissible,skills_schedule,skills_executor}.py -q -m 'not serial'`
  (2026-09-13): 398 passed in 22.47 s. On the still-flying FSM / unified-ring
  plans (audit finding 8, `/tmp/probe_floor_fsm.py`, 2026-09-13, floor 1.0 vs
  8.0 through the real `unified_cycle` entry points): no verdict flips;
  LAUNCH+SETTLE, the floor-lift SETTLE, the LAUNCH+STEADY chain (jerk 39 949)
  and the tilted-ring replan_tail (113 845) are bit-identical; the one active
  case, a displaced 60 mm carry, moves away from its limits (jerk
  108 593 → 34 939 mm/s³, tilt accel 2.781 → 1.276 rad/s²).
- **Box regeneration** (`python tools/admissible_sweep.py`): (P1,P1) xy
  [−40, 40] × [−30, 30] mm, flight 0.750–0.857 s, 27.8 s. Also carries R3's
  shared `limits.leg_jerk_mmps3` change into the columns boxes: (P1,P2)
  landing_xy_m x narrowed [−0.02, 0.02] → [−0.02, 0.01]; (P2,P1) landing_xy_m
  y narrowed [−0.02, 0.02] → [−0.01, 0.02].
- **Chained mechanics, seed 0** (`sim/skills_gate.py --learn --policy B
  --seeds 0`, `temp/reports/skills_gate_learn_B_20260913T182807.json`): 25
  throws / 14 attempts, 25 makes, 0 drops; flight band at throw 4, xy band
  never reached (box); each attempt ends `WINDOW_TOO_SHORT` ×11 /
  `LIMIT_JERK` ×2 after 2 throws.
- **j, D2 fix** (tracker `t_land` anchored to last sample):
  `test_tracker_landing_time_is_anchored_to_the_last_sample_not_now`:
  `pytest tests/motion/test_skills_executor.py tests/motion/test_skills_schedule.py
  tests/sim/test_skills_gate.py -q` (2026-09-13): 105 passed.
- **`probe_steady_seed.py`** (settle-tail / seed-placement withdrawal, §5): A
  snap accept jerk 0 (hand 3393); B deferred +0.05/+0.15/+0.30 s accept jerk
  0; C refine from A with the release fixed accepts only at t_land − 0.40 s
  (109k, +16 mm/+45 ms), refuses `LIMIT_JERK` at −0.40/−0.30/−0.25 s
  (259k–467k, +38 mm/+95 ms), and a zero-error re-send at −0.25 s refuses
  `WINDOW_TOO_SHORT`.
- **Chained seed 0 re-run on current tree**
  (`skills_gate_learn_{A,B}_2026091318{5306,5406}.json`): learner converges
  err_xy 37.1, 37.1, 5.1, 1.9, 1.1 … 0.1 mm; flight 89, 89, 7.5, 4.6, 3.3 …
  0.1 ms; PASS both; 10 sim tests pass; policy B still ends 25/25 attempts
  `LIMIT_JERK` at the first catch-with-throw.
- **k, attempt-1 bisection** (`probe_handoff_capture/bisect.py`): refusal
  235 887 mm/s³ at splice k=29, n=3 estimator, landing (−2.5, 81.7) mm,
  velocity (50.7, 87.3, −4603) mm/s, t_land +84 ms vs nominal, release
  pinned, dwell 0.22 s; flip (c) t_land → nominal accepts at 116 929; (a)
  lateral vel → 0 is worse (306 052); (f) snap is a near miss (150 812).
  Estimator lateral-vel error: n=3 mean 88 mm/s, n=20 5.0 mm/s, n=40
  1.8 mm/s; HW tracker confirmed at n=3 matches
  (`hardware_config.py:272`).
- **k follow-up** (`probe_handoff_capture_v2/bisect_v2.py`): converged
  attempts 10/20 land on time (−14/+13 ms), learned u (−25.25 mm y,
  0.777 s) inside the box; baseline 262 743 mm/s³ refused; every input
  including u→identity still refuses; only (f) release-snap splice flips to
  78 326 mm/s³ accepted.
- **l landed** (`_previous_release` + `_predicted_landing`, scheduled-dispatch
  install):
  `test_an_end_to_end_self_toss_schedule_fails_today_without_the_scheduled_
  dispatch_fix` — old code 3/7 `LIMIT_JERK`, new code 7/7:
  `pytest tests/motion/test_skills_executor.py tests/motion/test_skills_schedule.py
  tests/ros/test_skill_node.py tests/ros/test_skills_plan_bench.py
  tests/sim/test_skills_gate.py -q` (2026-09-13): 208 passed in 20.80 s.
  `--learn --policy A --seeds 0`: PASS, 14 attempts; warm attempts end
  `WINDOW_TOO_SHORT`/`LIMIT_JERK` at the second catch-with-throw.
- **k, third capture** (`probe_handoff_capture_v3/flip_v3.py --learn --policy
  A --seeds 0`): attempt 10's second catch-with-throw refuses
  `WINDOW_TOO_SHORT` (window −0.272 s); catch 1's 11 re-sends all refused (9
  `LIMIT_JERK`, 2 `LIMIT_ACC`); patched (tracker → None while not airborne):
  25 attempts → 3, 25/25 makes, 0 drops.
- **n landed** (`_valid_tracked_landing` guard + sim tracker None-while-not-
  airborne): 4 new tests (2 fail without the fix); the same five-file command
  (2026-09-13): 212 passed in 20.11 s. **Sim criterion MET**: `python sim/skills_gate.py --learn --policy A` seeds 0–4,
  run twice (`skills_gate_learn_A_run{1,2}.json`, 204.7 / 205.9 s): all PASS,
  band_xy 3, band_flt 3, both monotone checks T/T, 3 attempts, 25 makes, 0
  drops. Policy B (`skills_gate_learn_B_run1.json`, 191.4 s): all PASS, band
  5/5, 1 attempt (25-throw chain), 0 drops. Columns seed 0: FAIL,
  `LIMIT_JERK` at install 15 — filed (§5, §Handoff), not fixed at R3.
- **m, columns bisection** (`probe_columns_resend_trace/install_trace/
  old_anchor.py`): old drifting tracker anchor → PASS 20/20, 83/142 installs
  (the R2 numbers); correct anchor (j's fix) → 0 re-sends, 985 fence checks
  all within 1 mm / 2 ms, 13 back-to-back catch+throw splices creep to
  install 15 = 204 534 > 200 000, `LIMIT_JERK`.
- **Final gate**: `./run_tests.sh --full` on the final tree, after the
  deletion and every audit fix (2026-09-13): **5943 passed, 6 skipped, 1
  xfailed** in 282.78 s (parallel) + **6 passed** in 18.29 s (serial);
  `RESULT: PASS`. Below the 6552-passed baseline because the deletion removed
  its ~650 learning-stack tests. The first final run, before the audit
  fixes, failed 6 (four unified-launch speed-trim / record tests of deleted
  mechanisms, removed; the committed `ros_ws/docs/choreography.md`,
  regenerated with `python tools/gen_choreography_map.py`).
- **Split-commit check**: the learner/memory and blend-floor commits verified on
  their own snapshot in a detached worktree at that commit (2026-09-13):
  `pytest tests/motion/test_learner.py tests/motion/test_{cup_realize,cup_cycle,
  unified_cycle,unified_cycle_splice,validate_cycle,validate_cycle_vectorised,
  skills_segments,skills_admissible,skills_schedule,skills_executor}.py -q -m
  "not serial"`: 383 passed in 21.91 s.
- **Offline rehearsal**: `python tests/hardware/skills_plan_bench.py --rehearse
  --pattern self-toss --arm A --attempts 3` on the idle Jetson (2026-09-13):
  `blas threads: 1`; G1 PASS, G2 PASS, G4 PASS (0/3 attempts ended early), G3 /
  G5 SKIP offline; the memory grew 1 → 2 → 3 rows and the third command moved
  off the identity prior (−30.8 mm, 0.780 s), landing error 37.3 mm / 93 ms →
  6.9 mm / 8 ms.

## Handoff

- **The R3 hardware sitting is next**: `tests/hardware/session_skills_r3.md`
  is written and up to date with this entry; it needs to be flown before R3
  can close, and it is where the 156–371 ms release lag
  (§2) gets measured against the flight band and the 0.750 s box floor (§6).
- **Deletion (f1, f2) landed**: f1 re-homed `toss_record.py`'s live
  symbols into `ball_possession.py` and deleted the rest of the learning
  stack per `census_learning_stack.md`; f2 was the FSM node surgery, FSM test
  updates, and the identity-release pin. Both are done in this worktree.
- **Carried to R4** (do not fix at R3, per the owner's orthogonality ruling
  on the columns regression):
  1. The columns-pattern jerk creep the correct tracker anchor exposed
     (§5f, §m): R2's sim gate was partly propped up by the drift bug; the
     true columns behaviour needs its own fix.
  2. Banking saturation when hand deceleration exceeds g (the ~12° bank
     behind R2's 188k figure, §3).
  3. `admissible.gate_hash` covers `feasibility.py` + `segments.py` only — a
     `cup_realize.py` edit (like this rung's floor) leaves a stale box
     undetected. Filed as a follow-up, not fixed here.
  4. The columns opening REST, re-carried from R3's own assumption list
     (decision 11's note: "Columns opening REST re-carried to R4").
- **Still open from this rung, not R4-specific**: the release lag (§2) and
  the perception blind rate (10/27, §2) — both are sitting-measured, not
  code fixes.
- **Documentation not yet updated**: plan § 3 Status + R3 Outcome (must stay
  NOT DONE until the hardware gate is flown); § 2.5 hyperparameters and a
  centring note; § 2.7 for the tracker-plane change. `/audit --unstaged` and
  the commit sequence (learner+memory; outcome; sim; preconditions; deletion;
  runsheet+docs) are also still pending, as is the `project_two_ball_skill_
  stack.md` memory pointer update.
