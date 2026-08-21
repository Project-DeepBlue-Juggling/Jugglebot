---
title: Velocity-continuous smooth-move prelude — the unread extern, and the two bounds that decide when it is affordable
type: bugfix
date: 2026-07-27
status: resolved
phase: "Self-toss anomaly fixes — hand-command-continuity Phase 4"
related_plan: "hand-command-continuity.md"
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code/Trajectory.h
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/hand_stroke.py
  - ros_ws/docs/hand_command_continuity.md
  - sim/hand/trajectory.py
  - tools/probes/hand_stroke_timeline.py
  - tests/firmware/test_hand_smooth_move_xref.py
  - tests/sim/test_hand_trajectory.py
  - tests/motion/test_hand_stroke.py
  - tests/ros/test_catch_coordinator_node.py
  - tests/hardware/session_anomaly_fixes.md
  - plans/archived/hand-command-continuity.md
commits:
  - 5369fc2
subsystem:
  - motion
  - can
  - sim
  - config
tags:
  - safety
  - testing
  - kinematics
  - docs
---

# Velocity-continuous smooth-move prelude — the unread extern, and the two bounds that decide when it is affordable

## Summary

`Trajectory.h::makeSmoothMove` seeded every hand profile `v = a = 0` from
`current_hand_position` while `current_hand_velocity` sat declared `extern
volatile` two lines above and was **never read** — so any hand command landing
while the hand moved commanded a velocity STEP. That unread `extern` is the
2026-07-25 post-throw dip (10.7–55.3 mm below the stroke end). The quintic now
goes `(x0, v0, a = 0) → (target, 0, 0)`, with two cannot-fit tests deciding when
that is affordable: the overshoot must fit inside the stroke, and the arrest must
not take longer than the longest rest-to-rest move the stroke admits. When either
fails it falls back to today's exact rest-to-rest profile.

**Not flashed.** `Teensy_code.ino` is byte-untouched; the deliverable is the
header it compiles. The Platform Teensy carries no `FW_VERSION`, so an un-flashed
board is indistinguishable from a fixed one at the bench — runbook row H4.0 is the
only guard.

> **SUPERSEDED on the `FW_VERSION` point (2026-07-27, Phase 6 —
> [2026-07-27-platform-teensy-fw-version](2026-07-27-platform-teensy-fw-version.md)).**
> The board now declares `FW_VERSION` and reports it in bytes 5-6 of the 0x6E0
> RobotState reply, so an un-flashed board IS detectable: runbook row **FW-1**
> (`grep PLATFORM_FW_CHECK`) replaces the four-link inference chain H4.0a-c.
> The rest of this paragraph stands — this entry's own flash is still required,
> and FW-1 is what confirms it happened. Contract:
> `ros_ws/docs/platform_fw_version.md`.

## Problem

Every hand command — a kind-3 prime / retract / `SAFE_ABORT`, and the prelude
ahead of every kind-0/1/2 stroke — is prepended by `makeSmoothMove`, which
rebuilds the Teensy's single packed queue from the live hand position. It seeded
the replacement profile at `v = 0`. Measured 2026-07-25: a catch arm landing 8–18
ms after ball release froze the setpoint at the live encoder value (6.20–7.78 rev)
with the hand travelling through it at ~120 rev/s; the position loop coasted to
10.17–10.32 rev — as little as 0.775 rev from the 11.1 rev overextension guard —
and yanked the hand 0.34–1.75 rev below the stroke end.

## Root Cause

The velocity was available and unread. `current_hand_velocity` is written at
`Teensy_code.ino:439-441` from the ODrive axis-6 0x009 frame — the same
`Pos_Estimate`/`Vel_Estimate` pair `hand_telemetry` publishes — and
`makeSmoothMove` read only the position half of it.

## Discussion

### Why a fallback, rejecting both options the plan offered

Plan step 2 said to "refuse loudly, or brake to the limit" when the overshoot
will not fit. Both cause a concrete failure:

* **Refuse.** For a kind-3 that means *not clobbering*, and a kind-3 retract
  clobbering an armed kind-0 is the only un-arm mechanism the Teensy offers — a
  pre-release `SAFE_ABORT` depends on it. Refusing via an *empty* trajectory is
  worse still: `Teensy_code.ino:472-475` returns from the kind-3 handler **before**
  `packedMsgs.clear()`, so an armed stroke would survive the abort.
* **Brake to the limit.** The required acceleration is bounded by nothing the
  firmware declares. Near a target the bulge has no room to be absorbed by the
  s-shape's own travel: an abort dispatched with the hand essentially *at*
  `hand_retract_rev = 0.0` and descending at the measured −60 rev/s needs
  ~28 000 rev/s², 280× the declared limit. Declaring a second, higher limit
  changes what the machine can physically do at the bench — an operator decision.
  (Far from the target the same descent is served fine — 5.0 → 0.0 rev at
  −7.5 rev/s is honoured — so that figure bounds the near-target corner, not every
  retract. The first draft of this reasoning over-generalised it.)

The fallback **is** today's profile, so it adds no commanded magnitude the
firmware could not already produce, is never empty, and is observable as the
probe's from-rest `seeds` row that the bench already aborts on.

### The floor was on the hard stop, and the reason given for it was wrong

This is the finding two independent review lenses reached from different starting
points, and it is the one that mattered most.

The excursion clamp shipped with its lower bound at `Homing::HAND_ABS_POS_REV =
−0.1` rev, 0.1 rev below the plan's own stated `[0, 11.1]` bound, justified as:
*a margin "would put the shipped SAFE_ABORT target outside the band, making every
abort retract infeasible by construction."*

That justification is false, and the code refutes it two lines away: the bound is
relaxed to `min(FLOOR, start, target)`, so the retract target is one of the
endpoints and is admitted at **any** floor value. Re-running the branch logic with
the floor at 0.0 confirmed it — every retract case still lands exactly on 0.0; the
only effect is that the two undershooting cases route to the rest-to-rest
fallback, which is today's behaviour and is *shorter*, not "T toward zero".

What the −0.1 floor actually bought was permission to command the hand onto the
bottom hard stop. `−0.1` rev **is** the stop (the axis homes downward into it at
−3 rev/s), and it is below the floor the host already declares for this axis —
`teensy_bridge_node` rejects a smooth-move target below 0 and `can/odrive.py`
clips a hand setpoint below 0 and warns, so the firmware and the host would have
disagreed about the legal range. Two review lenses reported this independently
from compiled float32 sweeps; I re-derived it in the mirror before acting, and the
numbers agree — the −0.1 floor admitted commanded troughs to **−0.09841 rev** on a
retract re-dispatched mid-descent (9.99 rev at −22.36 rev/s), **−0.09942 rev** on a
catch-descent prelude near the bottom (1.23 → 6.1267 rev at −11.96 rev/s), and a
**prime** dispatched at the mid-point of a live retract (4.98 rev, −24.63 rev/s)
dived to **−0.05794 rev** — 5 rev *below its own start*, for a move whose target is
the top of the stroke. Sweeping the reachable grid, **2570 profiles that fit inside
the duration cap were honoured under the −0.1 floor and are rejected under 0.0**,
worst trough −0.10006 rev = 3.17 mm below encoder zero — and the worst of them is
an ordinary retract (0.65 rev → 0.0 at −7.15 rev/s), not a corner case. The ceiling
keeps 0.5 rev of margin for exactly the position-loop undershoot (+0.186 rev
measured) that the floor had zero allowance for.

Fixed to `JBOp::HAND_RETRACT_REV = 0.0`. **Tradeoff accepted:** the floor still
carries no *margin* — the 0.1 rev down to the stop is the whole allowance — where
the ceiling carries 0.5. That asymmetry is now deliberate and documented rather
than derived, because a margin below zero would start refusing legitimate retract
profiles for no measured reason.

A float32 slack of 1e-4 rev (3.2 µm) had to come with the tighter floor: the
excursion's interior turning point is evaluated in `float`, so a profile that is
monotone in exact arithmetic can report a trough a few ulps *below its own
endpoint* (5.0 → 0.0 rev at −7.5 rev/s measures −2.4e-6 rev). Without the slack,
endpoint rounding alone would have sent those moves to the fallback and the fix
would have been a no-op on the retract path — the tighter bound would have
silently disabled the feature it was protecting.

### Why the excursion clamp was not enough: a duration bound too

Arresting `v0` costs **time** as well as travel, and the two scale differently —
the duration grows linearly in `|v0|` while the excursion grows as `v0²`. So
mid-stroke, where there is room for a big bulge, the excursion clamp alone lets a
prelude run arbitrarily long.

The case that made it concrete: `catch_coordinator_node._PRIME_INFLIGHT_S = 1.2 s`
suppresses a re-prime while a prime ascent could still be running — a kind-3
re-dispatch mid-ascent rebuilds the profile from the live position and yanks the
moving hand backwards, which is the 2026-07-23 stutter (5/12 ascents stalled 60–70
ms with velocity reversals to −4 rev/s). Its pinning test's own comment says it
exists so that "a later prime raise **or a Phase-4 duration change** cannot outgrow
it silently" — and it evaluated only the `v0 = 0` branch. A prime dispatched into
a live retract (which never stamps `_prime_dispatch_mono`, so it is not
suppressed) at the retract's own peak descent speed — start ≈ 5.04 rev at −24.6
rev/s, which is exactly where the retract quintic peaks — solves to **1.2057 s**
(and 1.2084 s at the exact peak, 4.98 rev / −24.63 rev/s). Past the window
outright, not merely past its 1.5× headroom.

Two ways to close it, and the choice matters:

* *Raise `_PRIME_INFLIGHT_S`.* The invariant needs 1.81 s to keep its 1.5×
  headroom. That is a host timing change on the catch path, on the strength of a
  case never observed at the bench.
* *Bound the firmware's honoured duration.* Chosen. The bound is the longest
  **rest-to-rest** smooth move the stroke admits (full travel, 0 → 11.1 rev =
  0.8005 s), derived from constants already in the header. An honoured prelude can
  then never take longer than a profile this firmware **already emitted before the
  change** — which is the same argument the fallback rests on, extended from
  magnitude to duration, and it keeps *every* host window sized on a commanded
  hand move valid without any of them moving.

Cost: it clips the top of the mid-stroke continuity band, from ~20.9 to ~20.3
rev/s. The phase's actual target — the settle tail, measured at ≤ 0.25 rev/s — is
three orders of magnitude below that, so nothing the fix exists for is affected.

Honest limit that survives both bounds: `PRELUDE_ALLOWANCE_S = 76 ms` still does
not model a continuous prelude (0.24 s at the dead-band edge). That one is
instrumented, not fixed — see contract limit F.1 and runbook row H4.7.

### The bench instrument would have scored a working fix as a failure

The brief for this phase requires a two-sided instrument check, and it earned its
keep. `tools/probes/hand_stroke_timeline.py` separates a `makeSmoothMove` BRAKE
from the armed catch descent by how far below `x3` the commanded position travels,
with the threshold at 0.5 rev — justified in the source by the brakes visible in
the seven observed tosses (0.206–0.365 rev). Every one of those was the
settle-from-above case.

Phase 4 kills that premise. An honoured brake seeded from a *downward* velocity
dives `0.00778·v0²` rev below `x3`: past 0.5 rev at just **8.05 rev/s**, and
honoured out to **3.213 rev** where the new duration cap takes over. Demonstrated
by building a synthetic capture carrying a brake at exactly the clamp's reach and
running the shipped probe against it, with the old separator restored:

| reading | old separator (0.5 rev) | fixed separator |
|---|---|---|
| `catch_desc` | 2.05 s — **the brake** | 3.03 s — the real descent |
| `peak_pos_rev` | 9.9602 (under-reports) | 9.9794 |
| `dip_below_x3_rev` | **0.0** | 3.2117 |

The last row is the one that matters: the instrument reported **zero dip on a
capture where the hand dove 3.21 rev = 102 mm below the stroke end**, because
`catch_desc` caps the dip/peak window and had been placed on the brake. That is a
false PASS on the row guarding the 11.1 rev end stop (bench H4.5 is a hard abort
with E-STOP). Separator re-anchored on the catch region — the descent must reach
`x5 + 0.33` rev, which no honoured brake can — and the deep-brake case is now a
permanent gate case. It is biased deep on purpose: too shallow fails *silently* on
the end-stop row, too deep leaves `catch_desc` unset and floods the report with
spurious truncations, and a loud failure is the safer side of that trade.

### The velocity and torque feedforward streams were unvalidated

The phase rewrote the firmware's velocity and acceleration samplers, and nothing
in the repository compared the emitted `tr.v[]` or `tr.tor[]` against the mirror —
the compiled-header driver printed `tr.v.front()` and stopped. Mutation-verified:
changing `30.0f*t4` to `3.0f*t4` in `quinticS1` leaves the full-stroke prime
commanding **−354 rev/s** of velocity feedforward at its final sample and a
finite-differenced 1863 rev/s², and every test in the file passed. A sign flip in
`quinticH1` (which preserves `h1(0) = 1`, so `tr.v[0]` still reads `v0`) and any
mutation of `quinticS2`/`quinticH2` (which reach only `tr.tor`) were likewise
invisible. Those are the streams the ODrive receives as feedforward alongside the
setpoint, on the one file whose other validation is a hand flash. The driver now
emits every sample and all four mutations are caught.

### A test that asserted the code's own predicate back at itself

The excursion sweep asserted `trough >= min(HAND_HOME_ABS_POS_REV, start,
target)` — the exact expression `plan_smooth_move` used to *accept* the plan.
Tautological on the floor side: move the floor constant to −1.0 and it still
passes. That is precisely why the floor defect above survived the implementer's
own 60-case sweep, compounded by a grid whose only near-bottom cases were monotone
or fallbacks, so no case ever landed in the accepted-with-undershoot region. The
assertions are now written against the *physical* limits — the overextension
guard, and encoder zero as a bare `0.0` — with the code's own band checked
separately, and two states taken from shipped profiles were added to the grid.

### What I checked and could not break

* The **peak-acceleration bound is sound**, which is the direction that matters:
  it is the triangle-inequality bound on `|δ·s'' + u·h''|`, and the peak is
  monotone decreasing in `T`, so the 0.05 s floor can only reduce it.
* The **torque feedforward has no step at the seam**: `s''(0) = h''(0) = 0`, so
  `acc(0) = 0` on every branch — the property the old profile had, deliberately
  preserved.
* **Rest-to-rest is unchanged.** The `v0 == 0` branch carries the historical
  expression verbatim, and `u·h` is exactly `+0.0f` at `u = 0`, so every existing
  prime, retract and catch prelude emits bit-identical commanded positions.
* `Teensy_code.ino` is byte-untouched, so the kind-3 clobber path is unchanged
  except that the empty branch is strictly **narrower** (empty now also requires
  the hand to be at rest) — the un-arm mechanism fires in strictly more cases.

### Withdrawn mid-finalize

I set out to fix the `smooth_move_v0_deadband_rps = 6.0` value as well, since all
three lenses flagged its rationale. I did not, and the reason is in the evidence:
the two available measurements **disagree**. The 5.39 rev/s anchor is a top-park
p99 from the 2026-07-24 reload sitting (maximum never published); an independent
re-read of the three 2026-07-25 traces filtered for *stationarity* (position
spread < 0.02 rev over ±0.3 s) rather than for a position band gives p99
0.134–0.144 and max 0.96 rev/s, suggesting 5.39 is an artefact of a filter that
swallowed transit samples. If that is right the dead-band is ~6× above the real
noise floor — conservative, and safe to leave — but moving it is a park-band
change on an anchor I cannot re-measure from a trace that does not park at the
top. The *arithmetic* claims attached to it were wrong and are fixed (the quoted
0.2802 rev is the branch's infimum, not a bound on it; the H3.5 margin argument
compares a commanded excursion against a measured-position band and does not hold
above the dead-band edge). The value itself is handed to the operator.

## Fix

* `Trajectory.h`: seed from `current_hand_velocity` behind a dead-band; exact
  `s`/`h` decomposition in the samplers; closed-form accel-limited duration;
  closed-form excursion; two cannot-fit tests (end stops, and
  `smoothMoveMaxDuration()`), both falling back to the rest-to-rest profile. Floor
  is `JBOp::HAND_RETRACT_REV`; comparisons carry a documented 1e-4 rev float32
  slack.
* `sim/hand/trajectory.py`: the branch-accurate mirror, plus
  `smooth_move_max_duration_s()`. `smooth_move_duration_s` docstrings (mirror and
  host) now say they return the **accel-limited** duration and that the firmware
  substitutes the rest-to-rest one on the fallback branch.
* `tools/probes/hand_stroke_timeline.py`: `_CATCH_DESC_ABOVE_X5_REV` replaces
  `_CATCH_DESC_BELOW_X3_REV`; a `deep-brake` synthetic gate case.
* `catch_coordinator_node.py`: comment only, recording why `_PRIME_INFLIGHT_S`
  survives Phase 4.
* Four new codegen constants (`quintic_h_max`, `quintic_h2_max`,
  `smooth_move_v0_deadband_rps`, `smooth_move_excursion_margin_rev`) in the
  existing `teensy_trajectory` section — no `HW_SECTIONS` row needed.

## Verification

Full suite: `pytest tests/ -q`, run 2026-07-27 on this Jetson at HEAD `854df28`
with this phase's work uncommitted — **3943 passed, 3 xfailed, 198 warnings in
1399.35 s (23:19)**, exit 0. Baseline at the same HEAD was 3574 passed, 3 xfailed
in 1382.00 s: **xfail unchanged at 3**, and no test was weakened, skipped, deleted
or xfailed.

The +369 delta is accounted for exactly, by collecting each changed test file
against a detached worktree at `854df28`:

| file | baseline | now | delta |
|---|---|---|---|
| `tests/firmware/test_hand_smooth_move_xref.py` | 0 (new) | 173 | +173 |
| `tests/sim/test_hand_trajectory.py` | 33 | 224 | +191 |
| `tests/motion/test_hand_stroke.py` | 17 | 22 | +5 |
| `tests/ros/test_catch_coordinator_node.py` | 75 | 75 | 0 (a body was extended, no new case) |
| | | | **+369** |

Neither known-flaky allocation test (`test_hot_loop_allocation_contract`,
`test_t3b_h4_on_post_solve_allocates_within_budget`) failed in this run, so no
isolated re-run was needed.

Mutation evidence, all run 2026-07-27 with the tree restored and re-verified by
md5 afterwards:

* `quinticS1` / `quinticH1` / `quinticS2` / `quinticH2`, one token each — **all
  four now caught**, each by exactly one test (the compiled-header xref).
  *Attribution:* that they were previously invisible is established two ways —
  the panel's contract lens mutation-ran `quinticS1` and `quinticH1` against the
  pre-fix file and got 15/15 green, and I confirmed by inspection that the driver
  read `tr.v.front()` and never `tr.tor` at all, so no test could observe the
  torque stream. I did not re-run the pre-fix mutations myself.
* the probe separator reverted to `x3 − 0.5` rev — `--gate` FAILs the `deep-brake`
  case on `catch_desc` and `peak_pos_rev`.

Numbers I re-derived myself rather than inheriting from the reviews (the
candidate honoured profile, mirror in float64):

| case (start → target @ v0) | duration | commanded trough |
|---|---|---|
| 9.99 → 0.0 @ −22.36 rev/s | 1.3185 s | **−0.09841 rev** |
| 1.23 → 6.1267 @ −11.96 rev/s | 0.8172 s | **−0.09942 rev** |
| 4.98 → 9.9594 @ −24.63 rev/s (prime into a live retract) | **1.2084 s** | −0.05794 rev |
| 5.04 → 9.9594 @ −24.62 rev/s | **1.2057 s** | −0.05 rev |

All four sat *above* the old −0.1 rev floor, i.e. all four were honoured and
commanded onto the stop; the third and fourth are also the cases that outgrew
`_PRIME_INFLIGHT_S = 1.2 s`.

**The floor fix is load-bearing independently of the duration cap**, which I
checked because the four cases above happen to violate both bounds. Sweeping
start ∈ [0.05, 11.0] × target ∈ {0.0, 6.1267, 9.9594} × v0 ∈ (−40, −0.05] at 0.05
resolution, restricted to profiles *inside* the duration cap: **2570 profiles were
honoured under the old −0.1 rev floor and are rejected under the new 0.0 rev
floor**, worst commanded trough **−0.10006 rev = 3.17 mm below encoder zero**. The
worst one is not an exotic corner — `start 0.65 rev → target 0.0 @ −7.15 rev/s,
T = 0.3804 s` is an ordinary retract from near the bottom at a gentle descent
speed.

Instrument self-checks: `tools/probes/hand_stroke_timeline.py --gate` → `GATE PASS
— 25/25 rows` **and** `GATE PASS — fixed-shape branch` (five cases), exit 0.

Codegen: `python config/generate_config.py` run twice after the YAML edit; modified
file set unchanged, no new files.

Instrument self-checks: `tools/probes/hand_stroke_timeline.py --gate` → `GATE PASS
— 25/25 rows` **and** `GATE PASS — fixed-shape branch` (five cases), exit 0.

Codegen: `python config/generate_config.py` run twice after the YAML edit; modified
file set unchanged, no new files.

**Not verified on hardware.** Nothing in this entry has run on the robot. The
Platform Teensy has not been flashed.

## Related

- Plan: `plans/archived/hand-command-continuity.md` § Phase 4 — Outcome
- Contract: `ros_ws/docs/hand_command_continuity.md` (obligation F, limits F.1–F.6)
- Bench: `tests/hardware/session_anomaly_fixes.md` § CHECK HAND-4
- Phases 0–3: `logbook/2026-07-26-hand-prime-rev-derived.md` and its predecessors
## Close-out — 2026-08-21

**Status `in-progress` → `resolved`, and the "Not verified on hardware" line
above is superseded.** It was true when written. The Platform Teensy was flashed
before the 2026-07-27 sitting and `FW-1`/`H4.0d` read
`PLATFORM_FW_CHECK: OK — v1` on **all six launches** of it, so the Phase-4 bench
rows mean something — and this phase's own branch fired:

* **`dip_below_x3` 0.000–0.026 rev on 15 of 17 tosses**, against a pre-fix
  0.339–1.748 rev (10.7–55.3 mm) — a 40–70× reduction;
* **the velocity-continuous branch fired on hardware for the first time** on 4 of
  17 tosses (`v0` = −8.44 / −6.90 / −6.98 / −7.55 rev/s), max commanded
  **10.2259 rev**, 0.374 rev under the clamp. The runbook's claim that "no row
  provokes it" was wrong: a fast throw provokes it every time;
* `H4.9`, `H4.10` and `H4.0b` PASS.

Verdicts: `logbook/2026-07-28-anomaly-fixes-validation-sitting.md`.

**Two of this entry's numbers moved on 2026-08-18** with the hand end-stop
correction (`logbook/2026-08-18-hand-end-stop-corrected.md`), recorded rather
than edited in:

* `smoothMoveMaxDuration()` **0.80054 → 0.78964 s** (the stroke got shorter), so
  a prelude whose honoured duration lands in (0.78964, 0.80054] s — `|v0|` in
  (20.04, 20.32] rev/s — now takes the rest-to-rest fallback where it was
  honoured before. Conservative, but a behaviour change, and it is why the board
  went to **FW 3** and why bench row H4.10 reads a board still emitting up to
  0.8005 s as UNFLASHED;
* the affordable continuity band mid-stroke is **~19.96 rev/s**, not ~20.3, and
  the binding bound FLIPPED — the excursion clamp now binds ahead of the duration
  cap's ~20.04 rev/s. The stroke-top figure (~9.1 rev/s) is unchanged, because
  the 10.60 rev ceiling held (the margin moved 0.5 → 0.2 with the base).

The excursion clamp itself is bit-identical across that correction.
