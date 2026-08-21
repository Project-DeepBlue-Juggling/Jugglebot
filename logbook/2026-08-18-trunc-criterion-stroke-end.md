---
title: The truncation criterion goes profile-relative on both halves — the stroke end, then the collapse floor
type: bugfix
date: 2026-08-18
status: resolved
phase: "5"
related_plan: hand-command-continuity.md
files_changed:
  - tools/probes/hand_stroke_timeline.py
  - tests/motion/test_hand_stroke_timeline_probe.py
  - tests/hardware/session_anomaly_fixes.md
  - tests/firmware/test_hand_throw_decel_xref.py
  - tools/probes/README.md
  - plans/archived/hand-command-continuity.md
subsystem:
  - tools
tags:
  - testing
  - safety
---

## Summary

`hand_stroke_timeline.py`'s truncation detector fired on **Phase 1's catch-arm
gate working correctly**. Bench rows 1 / 2 / `H2.2` / `H4.6` therefore ABORTed on
a healthy capture, and the 2026-07-27 validation sitting had to adjudicate them
PASS by hand — the exact situation a criterion exists to prevent.

The scan is now bounded on the **commanded profile reaching `x3`** instead of on
a 50 ms wall-clock margin past the *modelled* stroke end. The two events it
confused now differ in kind rather than in delay. The command that used to be
mis-scored is not hidden: it is reported in a new `post_stroke_cmd` row.

Two-sided, mutation-verified, and validated against the whole evidence base:
**69 post-fix throws now read `trunc = -` / `seeds = 0`, both real pre-fix
truncations still fire.**

Incidental find, and the more urgent one: **`--gate` — the runbook's mandatory
instrument self-check — was RED on the shipped tree**, and nothing in the test
suite ran it. Both halves fixed.

**Addendum, 2026-08-20 — the same defect twice.** A *second* instance surfaced
during the work and, on the owner's decision, was fixed in the same session: the
other half of the predicate, which localises *where* the command froze, used an
absolute `_COLLAPSE_VEL_REV_S = 10` rev/s against a ramp whose velocity scales
with the throw. Same root cause as the 50 ms wall — **an absolute constant
judging a profile-relative event** — and the same fix shape. "Collapse" is now
the modelled stroke's own commanded velocity at the stroke-end band edge.

Both halves of the truncation verdict are now profile-relative, and the only
constant either derives from is `_X3_SHORT_REV = 0.05` rev — the instrument's
declared position resolution, which is also exactly where it is blind. No fixed
velocity or wall-clock threshold decides `trunc` any more. (Two absolute
velocities survive *outside* the verdict: `_ASCENT_VEL_REV_S`/`_REST_VEL_REV_S`
locate the stroke start, so at a throw slow enough to peak under 20 rev/s —
0.633 m/s, far below the C-HAND-3 admission floor — the probe reports
`no-throw-stroke` and rows 1/2 are not scored at all. `_TRUNC_SCAN_MARGIN_S`
survives too, demoted to a fallback bound that no admissible capture reaches.)

## Problem

### Defect 1 (2026-08-18): the truncation scan's time wall

`plans/archived/hand-command-continuity.md` § Phase 5 recorded, from the 2026-07-27
sitting:

> Rows 1/2/`H2.2`/`H4.6` carry a **criterion defect** (they fire on the gated
> arm's own prelude landing within `_TRUNC_SCAN_MARGIN_S = 0.050` of the
> *modelled* stroke end) — adjudicated PASS; the criterion still needs fixing.

Rows 1/2 exist to catch the 2026-07-25 clobber class: a command landing *inside*
the throw stroke, clearing the packed queue and re-preluding from the live
encoder, so the ball departs on the position loop's reaction to a frozen
setpoint. But Phase 1's arm gate withholds the catch arm until the stroke
completes and then dispatches on the next balls tick — so **the gate working**
lands a from-rest prelude just past the stroke end, seeded at a live position
that has sagged 0.06–0.17 rev under `x3` (25 measured re-seeds).

That prelude satisfies both halves of the old truncation predicate
(`vel_ff_cmd` below `_COLLAPSE_VEL_REV_S`, `pos_cmd` short of `x3`) whenever it
fell inside the 50 ms window. The 2026-07-27 sitting straddled it: **6** tosses
reported a truncation across its six bags and **3** across its three traces, and
on `2026-07-27_15-39-3x/5x`, recorded *both* ways, the same physical tosses read
**4** through the bag against **3** through the trace: the bag puts toss 22's arm
**49.2 ms** past the modelled stroke end and the trace puts the *same physical
arm* at **54.7 ms** — either side of the 50 ms wall. The verdict depended on the
recording path.

Counts throughout this entry are **per recording**. Two of the sitting's sessions
are in the evidence set twice (bag `15-37-50` / trace `15-37-55`, bag `15-39-38` /
trace `15-39-50`), so those physical tosses are counted twice in the 71 / 69 / 44
populations. That is deliberate — the whole point is that the same toss can score
differently through two recording paths — but it means the figures are throw-
recordings, not distinct throws.

### Defect 2 (2026-08-20): the collapse threshold, same class

Bounding the scan on stroke completion fixed *where* the scan may look. It left
the other half of the predicate untouched: **how** the freeze is localised inside
that range. That was `vel_ff_cmd < _COLLAPSE_VEL_REV_S`, an absolute 10 rev/s.

The firmware's throw profile decelerates from the commanded release speed to rest
over `t_dec`, so the ramp's velocity at any given distance short of `x3` scales
with the throw. At the slow end of the band it drops under 10 rev/s *while still
short of `x3`* — and then the ramp satisfies both halves of the predicate and the
probe reports a truncation on a perfectly clean stroke.

Whether it actually fired depended on where a ~100 Hz telemetry sample happened
to land inside a window a couple of milliseconds wide. Same signature as defect
1: **a verdict that is a property of the recording, not of the robot.**

## Root Cause

### Defect 1

`analyse_throw` bounded the truncation scan in **time**:

```python
scan_end = t_release_obs + model.t_dec + _TRUNC_SCAN_MARGIN_S
```

`t_release_obs` is derived from the *observed* stroke start plus the *modelled*
`t_acc + t_vel`, so the bound is a model instant plus a fixed 50 ms. Nothing in
that expression is about whether the stroke actually finished. A truncation and
a post-stroke command are the same shape to it; only their delay differs, and
the delay is not a property of either event.

Measured (see Verification for the command), relative to the instant `pos_cmd`
first reached `x3`:

| | population | offset |
|---|---|---|
| post-stroke command (gated arm's prelude, or a Phase-4 brake) | 44, over 69 post-fix throws | **+30.6 … +128.3 ms** |
| real truncation | 2 pre-fix throws | **−208.1 / −299.0 ms** |

Sign, not size. The prelude is *always* after, because the gate is what put it
there; the truncation is *always* before, because a truncated command only
reaches `x3` later and as part of the **replacement** quintic.

### Defect 2

`_COLLAPSE_VEL_REV_S = 10.0` — one number, applied to every throw. The admissible
range of "the ramp's own velocity where it is 0.05 rev short of `x3`" **straddles
it**: 8.58 rev/s at the C-HAND-3 admission floor, 15.31 rev/s at its ceiling. A
constant inside that range is necessarily wrong at both ends — it fires on the
ramp at the bottom and is slack against a real freeze at the top.

Measured width of the window in which the ramp satisfies both predicates, walked
over the closed form at 0.01 ms steps:

| release speed | window | phases that fire (of 100) |
|---|---|---|
| **2.440 m/s** (C-HAND-3 admission floor) | **1.94 ms** | **20** |
| 2.550 m/s | 1.29 ms | 13 |
| 2.6971 m/s (the retired `FLIGHT_TIME_MIN_S` floor) | 0.58 ms | 6 |
| 2.800 m/s | 0.16 ms | 2 |
| 2.845 m/s and above | 0.00 ms | 0 |

The "phases that fire" column is end-to-end through `analyse_throw` on clean
synthetics, sweeping the sampling phase across a whole 10 ms telemetry period —
the free parameter the defect turned on.

**The exposure had tripled under us.** It was first characterised at 2.6971 m/s
(6/100). The C-HAND-3 derived throw envelope landed on 2026-08-18 and moved the
admission floor *down* to 2.440 m/s, where it is 20/100. A hardcoded "worst case
2.6971" would have gone stale in the safe-looking direction, which is why the
tests import the band floor rather than quoting it.

## Discussion

### Why not move the margin

The obvious repair is to move `_TRUNC_SCAN_MARGIN_S`. Note first that the
direction is **narrowing**, not widening: widening the scan makes the detector
look *further* past the stroke end and catch *more* arm preludes. To suppress
the false positive the wall has to come in below the arm's arrival — the sitting
measured arrivals at 36.7–127.9 ms past the modelled stroke end, so the margin
would have to drop under 36.7 ms. (The sitting's own write-up quotes 28.0–61.6 ms
for "the margin"; that was measured by hand from the sample where `pos_cmd` hit
`x3` *exactly*, over a smaller set of tosses, and is not the same quantity as
either the probe's `stroke_end_hold_ms` — 30.6–128.3 ms over 44 commands — or the
offset past the *modelled* end that the wall actually compares against. All three
are cited by name from here on.)
(The task brief that commissioned this fix said "widen"; the substantive point —
that moving the wall trades a false positive for a blind spot — is right, and
the direction is the opposite of the word. Recorded so a future reader is not
confused by the plan-row wording.)

Narrowing fails for a reason that is not about the number chosen:

* **The false-positive edge is a scheduling quantity with no lower bound.** The
  arm's arrival delay is the balls-tick phase plus the dispatch shift, and the
  gate is *designed* to dispatch on the first tick after the stroke completes —
  an arm at +5 ms is the gate working, not a fault. The shift is not stable
  either: it grew from `+12.8…+21.9 ms` to `+54…+63 ms` between the 2026-07-25
  and 2026-07-27 sittings, tracking can-bridge Teensy uptime
  (`logbook/2026-07-28-anomaly-fixes-validation-sitting.md`). What was measured
  is only what the tick phase happened to give: **+36.7 ms** minimum past the
  modelled stroke end over the sitting's 44 post-stroke commands.
* **The true-positive edge is fixed, and it is `_X3_SHORT_REV`, not the velocity
  threshold.** A freeze is visible only while `pos_cmd` is more than 0.05 rev
  short of `x3`; the ramp is inside that band for its last **7.2 ms** at
  3.93 m/s (8.3 ms at 3.44, 10.6 ms at 2.70, walked over the closed form at 1 µs
  steps). So the true-positive edge sits ~7–11 ms *before* the modelled end.

One edge is pinned, the other is a coincidence of today's tick phase. A wall at,
say, +20 ms would have worked on this sitting's data with ~17 ms either side —
and would fail the first time a tick lands early, buying a blind spot on late
truncations, the clobber class the row exists to catch, at its worst (the last
millimetres of the ramp, where the ball has already left and the stroke end is
what sets the hand's excursion toward the 10.8 rev stop).

### Why the stroke end is the right boundary

A truncation *is* "the stroke was stopped before it finished". That is a
statement about the commanded profile, not about a clock. Bounding the scan at
the first sample whose `pos_cmd` is no longer short of `x3` makes the criterion
say exactly what the row means, and it has no wall to place:

* an arm landing **after** the command reached `x3` cannot be a truncation —
  there is no stroke left to truncate;
* an arm landing **before** it is a genuine Phase-1 gate failure, and still
  aborts row 1.

It also introduces no new threshold. "Reached" uses the same `_X3_SHORT_REV`
band the truncation predicate already used for "short of", so the two are exact
complements: the scan runs over the maximal prefix in which the command has
never come within `_X3_SHORT_REV` of the stroke end. Measured margins on that
band are large in both directions — the post-fix commanded plateau sits within
**0.0002 rev** of `x3` (250× inside the band), the two real truncations freeze
**2.26 / 2.83 rev** short (45–57× outside it).

### Why the prelude is still reported

Narrowing a criterion earns the objection "what did you just stop seeing?".
Answer: nothing, deliberately. The command that used to be scored as a
truncation now prints as `post_stroke_cmd`, with the instant, the commanded
position, the hold since the stroke end, and `|cmd − pos_meas|` — the
`makeSmoothMove` re-seed fingerprint. Runbook rows 8 and 9 are REPORT rows over
it. The fingerprint turned out to carry more than expected: over the 44 real
post-stroke commands it splits 0.000000–0.000452 rev (from-rest re-seed at the
live encoder) from 0.0588–0.1677 rev (a Phase-4 velocity-continuous brake
diving below `x3`) with no overlap, agreeing with row 7's brake annotation on
all 44.

That closes a gap that predates this fix, too. `H4.6` told the operator to
distinguish "a seed at or above `x3` with `trunc = -`" as `makeSmoothMove`'s
cannot-fit fallback — but the probe could never emit that. Seeds are only
collected once `trunc` has fired, and the standalone step rule
(`_REPACK_STEP_REV = 0.5` rev) cannot see the ~0.156 rev step a stroke-top
re-seed actually makes. `post_stroke_cmd` is now where that case shows.

### The gate was already red

While prototyping, `--gate` failed on `headroom_to_limit_rev`: expected 0.930,
got 0.626. The expectation was the literal `11.1 − 10.174`, left behind by the
2026-08-18 hard-stop correction (`3760daa`, 11.1 → the measured 10.8 rev). The
runbook makes `--gate` a **mandatory** pre-analysis self-check whose `GATE FAIL`
ABORTs the whole HAND analysis — so an operator would have discovered it
mid-sitting and correctly refused to score any HAND row.

Root cause of *that*: no test ran the gate. `tools/probes/` had probe tests for
`levelling_tilt_bag_check` and `catch_reach_replay`, but not for this one, and
`--gate` was only ever invoked by hand. Both halves are fixed — the expectation
is derived from the same generated constant the probe computes against, and the
gate now runs in the pytest suite.

### Why the collapse floor is the profile's own velocity, and why that ends the class

The instinct after defect 1 is to pick a better number — scale 10 rev/s down, or
key it to the tier. That is the same mistake one level in: any constant is a
guess about a quantity the model already knows exactly.

The ramp's velocity is a known function of commanded position. So the loosest
threshold that can *never* fire on an intact stroke is the ramp's own velocity at
the **last position that still counts as short of `x3`** — that is,
`x3 - _X3_SHORT_REV`. Two properties make it the right answer rather than a
better guess:

1. **Monotonicity turns it into a proof, not a margin.** `Trajectory.h`'s decel
   segment is *constant* deceleration (`x2 + v·tau + 0.5·throwD·tau²`, `throwD`
   constant), so commanded velocity falls monotonically with commanded position.
   Every sample of an intact stroke still short of `x3` therefore sits at or
   above the floor **by construction**, at every speed. The self-trigger window
   is empty, not narrow. Contrast defect 1's wall, which could only ever be
   "wide enough for the data we have".
2. **It introduces no constant.** With `v = 0` at `x3`,
   `v² = -2·throwD·(x3 - x)`, so the floor is
   `sqrt(-2·throwD·delta)` with `delta = _X3_SHORT_REV` — the instrument's
   already-declared position resolution, read through the shipped stroke model.
   `throwD` comes from the commanded event velocity. Verified against a bisection
   on `pos_rev` to better than 1e-3 rev/s across the whole wire band
   `v ∈ [0.3, 7.0]` m/s.

The floor scales **linearly** with the commanded event velocity — `throwD` goes
as `v²`, since `throwA = v/t_acc` and `t_acc ∝ 1/v` — and the ratio is the
sharpest form of the whole argument:

```
floor / v_cmd = sqrt(delta_m·(ir+1)/(accel_st·ir)) = 0.111171
```

No `v` on the right-hand side. **The floor is always 11.1 % of the commanded peak
velocity**, which is also why it can never reach the velocity-hold plateau the
scan range contains. 8.58 rev/s at the admission floor, 13.82 at 3.93 m/s, 15.31
at the ceiling.

(This paragraph read "scales as `sqrt(v)`" until the audit caught it — the quoted
values were always the linear law, so the prose contradicted its own numbers.)

**Does raising the floor at speed blunt the detector?** No, and the margin is not
close. A frozen setpoint reads ~0 rev/s: the two real 2026-07-25 truncations
freeze at **0.010 rev/s** against a 13.82 rev/s floor — a **1382×** margin. The
failure mode a higher floor would create is firing *too early* on the ramp, and
monotonicity is exactly what rules that out.

**Where the criterion is now blind, stated plainly.** A freeze inside
`_X3_SHORT_REV` of `x3` is invisible — but that is by definition, not by
threshold: at 0.05 rev (1.6 mm) from the stroke end the ball has long gone and
there is no stroke left to truncate. The blind spot is the instrument's declared
position resolution and nothing more, and it is the same one both halves of the
criterion share.

## Fix

### Defect 1

**One enforcement point**, `analyse_throw` in `tools/probes/hand_stroke_timeline.py`:

```python
i_end = next((i for i in range(i_peak, len(win))
              if win[i].pos_cmd >= model.x3_rev - _X3_SHORT_REV), None)
...
i_stop = len(win) if i_end is None else i_end
i_tr = next((i for i in range(i_peak, i_stop) ...), None)
```

Everything else follows from `i_end`:

* `stroke_end_reached`, `stroke_end_hold_ms`, `post_stroke_cmd`,
  `post_stroke_cmd_pos_rev`, `post_stroke_cmd_vs_meas_rev` — new reported (never
  gated) fields and two new printed rows. `post_stroke_cmd` is bounded by
  `catch_desc`, which is itself a commanded departure from `x3` and would
  otherwise always claim the row.
* `_TRUNC_SCAN_MARGIN_S` is **kept, unchanged at 0.050**, demoted in its comment
  to the fallback bound for the pathological capture where the command never
  reaches `x3` and no catch descent is found.
* `_GATE_EXPECT`'s `headroom_to_limit_rev` is now
  `hw.GEOM_HAND_MOTOR_HARD_STOP_REVS - 10.174`.
* `_synth_fixed_session` gained the arm-prelude shape (`arm_hold_s`,
  `arm_sag_rev = 0.156`, `arm_over_rev = 0.046`), both defaults inside the
  2026-07-27 spread over 25 measured re-seeds — sag 0.056–0.172 rev, settle
  0.046–0.222 rev above `x3`; the settle default is the *smallest* measured,
  which is the tightest case for the peak/dip window. The commanded velocity is
  computed so it is never differenced *across* the re-seed step: the firmware's
  `vel_ff_cmd` is the profile's own velocity and a from-rest quintic opens at 0,
  so the real captures read 0.00 to +0.44 rev/s at the seed sample, never
  negative. A centred difference would have invented a −150 rev/s spike and
  handed the synthetic a downward command the real telemetry does not have.
* `--gate`'s fixed-shape branch gained three cases: `arm-prelude+28ms`,
  `arm-prelude+62ms` (either side of the retired wall — the criterion may not
  depend on the delay *at all*) and `late-trunc` (a real clobber 0.30 rev short
  of `x3`, the blind spot a narrowed wall would have bought).

### Defect 2

**One enforcement point** — `_collapse_floor_rps(model)` in the same
file, replacing the module-level `_COLLAPSE_VEL_REV_S`:

```python
delta_m = _X3_SHORT_REV / LINEAR_GAIN_REV_PER_M
return math.sqrt(-2.0 * model.throwD * delta_m) * LINEAR_GAIN_REV_PER_M
```

The constant is **deleted**, not shadowed, so nothing can read it by accident;
`model.throwD >= 0` raises rather than returning a complex number. The predicate
becomes `win[i].vel_ff_cmd < floor_rps` with `floor_rps` computed once per throw.
`--gate` gains a `band-floor` case that sweeps the **sampling phase** — the free
parameter the defect turned on — across a whole telemetry period at four slow
speeds, and prints the C-HAND-3 admission speed it tested against (imported
lazily, so the probe's offline import surface is unchanged and a missing envelope
module degrades to literals with a printed note rather than failing).

Operator-facing: runbook rows 1, 2, `H2.2`, `H4.6` rewritten to score the stroke
end; new REPORT rows 8 (`stroke_end`) and 9 (`post_stroke_cmd`); a prose block
explaining the correction and, explicitly, **"do not score `post_stroke_cmd` as
row 1"**. The plan's Phase 5 row and `tools/probes/README.md` follow.

## Verification

### Defect 1 (2026-08-18)

**Empirical basis, prototyped before any test was written** (CLAUDE.md's
empirical-probe rule). `/tmp/probe_trunc_criterion.py` (uncommitted) against the
whole evidence base — the six 2026-07-27 session bags plus six
`toss_trace_*.jsonl` recordings, **71 analysable throws**, run 2026-08-18:

* before: 11 throws reported `trunc`, of which **9 were post-fix captures**
  (false positives) and 2 were the real 2026-07-25 truncations;
* after: **69 read `trunc = -`, `seeds = 0`**; the two real truncations still
  read `ok` at `7.1245` and `7.7004` rev with their seed counts unchanged;
* 44 post-fix throws had a command land between the stroke end and the catch
  descent; all are now reported as `post_stroke_cmd`, and its `|cmd − pos_meas|`
  figure splits them **cleanly and in agreement with row 7 on 44/44**:
  **0.000000–0.000452 rev** on the 25 tosses with no brake (a from-rest re-seed
  at the live encoder — the gated arm's prelude) against **0.0588–0.1677 rev**
  on the 19 with row 7 annotated (a Phase-4 velocity-continuous brake diving
  below `x3`). A 130× separation, and an unplanned second discriminator: the
  row distinguishes the two kinds of post-stroke command on its own.

**Mutation verification** — five mutations, each re-running
`pytest tests/motion/test_hand_stroke_timeline_probe.py -q`, run 2026-08-18:

| # | mutation | result (against `11 passed, 1 xfailed`) |
|---|---|---|
| A | revert the bound (`i_stop = len(win)`) | **2 failed** — `…is_not_a_truncation[0.028]` and the gate self-check. `[0.062]` still passes, which *is* the luck-dependence, reproduced |
| B | drop the `post_stroke_cmd` reporting | **3 failed** — both `…reported_rather_than_hidden` cases and the gate |
| C | restore the literal `0.93` headroom expectation | **2 failed** — the gate and `…anchored_on_the_shipped_hard_stop` |
| D | over-narrow (`i_stop = i_peak`, scan disabled) | **4 failed** — `…inside_the_stroke_is_still_a_truncation`, the committed-fixture test, the gate, **and the band-floor `xfail` XPASSing** (deleting the detector "fixes" that defect too, and `strict=True` says so) |
| E | loosen the "reached `x3`" band to 0.5 rev | **3 failed** — `…inside_the_stroke_is_still_a_truncation`, the gate, and the same strict XPASS |

Both directions are covered: too lax (A, B, E) and too strict (D). D and E also
demonstrate that the `xfail` is live rather than decorative.

**Probe self-check** (`python tools/probes/hand_stroke_timeline.py --gate`, run
2026-08-18): exit 0, `GATE PASS — 25/25 rows within tolerance` and
`GATE PASS — fixed-shape branch`. Before this change the same command exited
non-zero with `GATE FAIL — 24/25`.

**Full suite.** At the end of Defect 1 (`./run_tests.sh`, run 2026-08-18):
**5294 passed, 1 xfailed in 228.43 s** (parallel, rc=0) plus
`5730 deselected in 5.78 s` (serial, rc=0), total 239 s. RESULT: PASS. Re-run
after Defect 2 and the audit fixes (`./run_tests.sh`, run 2026-08-20):
**5306 passed in 228.59 s** (parallel, rc=0) plus `5741 deselected in 5.70 s`
(serial, rc=0), total 239 s. RESULT: PASS. Note the xfail count going
**1 → 0**: the marker was removed, not flipped to XPASS. The default gate is the right one here — nothing under
`controller/` or `sim/` is touched. Note the working tree also carried a
concurrent session's unrelated changes (`config/`, `toss_sequencer.py`,
`motion/trajectory/throw_envelope.py`) at the time of the run; an earlier run
caught that session mid-edit
(`test_toss_goal_rejections_via_execute[flight_band-…]`), which passed on its own
immediately after and is green in the run quoted here.

### Defect 2 (2026-08-20)

**Empirical, same evidence base.** Re-ran the whole set — six 2026-07-27 bags plus
six `toss_trace_*.jsonl`, 71 analysable throws, 2026-08-20: **2 truncated / 69
clean, byte-for-byte the same verdicts as before the collapse change**, and both
real truncations fire with a 1382× margin (0.010 rev/s freeze against a
13.82 rev/s floor). The band-floor case the new criterion must *not* fire on was
swept end-to-end: **0 of 100 sampling phases** at each of 2.440 / 2.550 / 2.6971 /
2.800 m/s (was 20 / 13 / 6 / 2), and **0 spurious fires over 960 clean captures**
spanning `v ∈ [0.70, 7.00] m/s` (64 speeds × 15 sampling phases).

**The margin that matters is on the real telemetry, not the synthetics.** Over
every in-scan sample of all 69 clean throws in the evidence base — 489 samples —
the minimum `vel_ff_cmd − floor` is **+0.4288 rev/s** (2026-08-20): `vel_ff_cmd`
12.5200 against a 12.0912 floor at 3.44 m/s, on a sample sitting 0.0036 rev short
of the band edge. That sample also measures the premise the by-construction
argument rests on — its commanded velocity matches the model's 12.518 to
0.002 rev/s, i.e. `pos_cmd` and `vel_ff_cmd` really do come from one consistent
frame. Positive by construction rather than by luck, which is what monotonicity
buys.

**Mutation verification** — five more, each re-running
`pytest tests/motion/test_hand_stroke_timeline_probe.py -q` (2026-08-20) against
a green **17 passed**:

| # | mutation | result |
|---|---|---|
| F | revert to the absolute `floor_rps = 10.0` | **3 failed** — the gate, and `…no_sampling_phase…[2.44]` / `[2.6971]` |
| G | floor computed but does **not scale** (fixed reference model) — the subtle form of the same defect | **4 failed** — the gate and three `…no_sampling_phase…` speeds |
| H | `floor_rps = 0.0` (detector blind) | **4 failed** — `…inside_the_stroke_is_still_a_truncation`, the committed fixture, the gate, `…a_freeze_at_the_band_floor_still_fires` |
| I | drop the `sqrt` (algebra) | **2 failed** — `…can_never_report_itself…`, `…scales_with_the_commanded_throw` |
| J | `delta` in rev instead of metres (unit error) | **11 failed** |

Both directions again: too lax (F, G, I, J) and too strict (H). G is the one
worth keeping: a floor that is *derived* but not *per-throw* still reintroduces
the defect, and only the phase-sweep tests catch it.

**The `xfail` is gone, not flipped.** `test_the_decel_ramp_at_the_band_floor_is_not_a_truncation`
and its companion were deleted and replaced by four unconditional tests; the file
carries no `xfail` **marker** and no **code** reference to
`_COLLAPSE_VEL_REV_S`. Verified 2026-08-20:
`grep -c "pytest.mark.xfail" tests/motion/test_hand_stroke_timeline_probe.py`
= **0**, and `grep -rn _COLLAPSE_VEL_REV_S` over `*.py` returns only docstring
prose that names the retired constant deliberately (test `:41`/`:55`, probe
`:113`/`:674`). An earlier draft of this line claimed `grep -c = 0` for both,
which does not reproduce — the substance was right, the cited command was not.

**Probe self-check** (`python tools/probes/hand_stroke_timeline.py --gate`, run
2026-08-20): exit 0, both `GATE PASS` lines, `band-floor` reporting
`0/40 fire` at each of four speeds and printing `C-HAND-3 admits from 2.4400 m/s
(0.4949 s flight)`.

**Full suite**: the triple is at the end of this section, re-run after every
change in this entry.

**Two stale live safety numbers fixed at the same time** (flagged on 2026-08-18,
released for edit on 2026-08-20). The runbook's pre-fix baseline table carried a
`headroom to 11.1` column and "0.775 rev = 24.5 mm of headroom to the 11.1 rev
end-stop"; re-anchored on the measured 10.8 rev stop, the true worst case is
**0.475 rev = 15.0 mm** (ball 17, `peak` 10.3248 — the `peak` measurements are
unchanged, only the anchor was wrong, one-sidedly optimistic by 0.3 rev / 9.5 mm).
Verified independently with `rev_to_mm` against `GEOM_HAND_MOTOR_HARD_STOP_REVS`
rather than taken on trust. And `tests/firmware/test_hand_throw_decel_xref.py`
quoted "28.0-61.6 ms" as the window the latched `Vel_FF` residual is held for;
that was the sitting's hand measurement, and the probe's `stroke_end_hold_ms`
measures **30.6-128.3 ms** with **25 of 69** clean throws seeing no command at all
before the catch descent. The docstring now rests the conclusion on the magnitude
bound (1.8-6.5 mm) rather than on the window being short, which is the argument
that actually holds.

## Open Questions

1. **The post-stroke sag is still invisible to `dip_below_x3`.** Item 10 of the
   2026-07-28 sitting's instrument-defects list: the dip window opens at the
   *coasting peak*, which on these captures comes after the prelude, so the
   `0.151–0.159 rev (4.8–5.0 mm)` sag between stroke end and prelude is excluded
   and the gated row reads `0.000`. That is tracking droop rather than a
   commanded yank, and 2–11× smaller than the pre-fix dip — but the row would not
   catch it if it grew. Out of scope here; deliberately left alone.
2. **`stroke_end_reached` on a truncated throw belongs to the replacement move,
   not to the stroke.** The row prints an annotation saying so. A cleaner model
   would report the two separately; not worth the field today.
3. **`H4.6`'s cannot-fit fallback has a report row but no bench evidence.** It
   now shows as `post_stroke_cmd` with row 7 annotated, but the branch has never
   been observed at the stroke top. The first capture that carries one should be
   checked against this row rather than assumed.
4. **CLOSED 2026-08-20 — the collapse threshold.** Surfaced by this work, pinned
   `xfail(strict=True)`, and fixed two days later on the owner's decision. It was
   the same defect class as the one above (an absolute constant judging a
   profile-relative event), which is the argument that justified closing it
   rather than a preference for tidiness. Kept in this list rather than deleted
   so the arc reads in order: the defect, the deliberate deferral, the decision,
   the fix. Substance is now in § Problem / § Root Cause / § Discussion /
   § Fix / § Verification under *Defect 2*.

5. **The detector's blind region is now exactly `_X3_SHORT_REV`, and a
   post-Phase-4 clobber can still slip through above the floor.** Surfaced while
   proving Defect 2's fix safe; pre-existing, not introduced, and not verifiable
   from any capture in the evidence base — by design, since Phase 1 prevents the
   shape that would produce one.

   Since Phase 4, `makeSmoothMove` seeds a clobber's replacement prelude at the
   **live** velocity once that exceeds `SMOOTH_MOVE_V0_DEADBAND_RPS = 6.0` rev/s,
   so the command does *not* fall to ~0 and neither `trunc` nor `seeds` (which
   needs `|vel_ff_cmd| < _SEED_VEL_REV_S = 2.0`) is guaranteed to fire. The
   detectable band for such a clobber is `6.0 rev/s <= v0 < collapse floor`:

   | release speed | detectable v0 window, old absolute 10 rev/s | new profile-relative floor |
   |---|---|---|
   | 2.440 m/s | 6.0 – 10.0 | 6.0 – **8.58** |
   | 3.9308 m/s | 6.0 – 10.0 | 6.0 – **13.82** |
   | 4.3568 m/s | 6.0 – 10.0 | 6.0 – **15.31** |

   So the change is **more** sensitive to this shape at the fast throws where a
   clobber matters most (the coast toward the end stop grows as v²) and
   marginally less over a 1.4 rev/s sliver at the admission floor. And by
   monotonicity, a `v0` below the floor implies the command was within
   `_X3_SHORT_REV` of `x3` — i.e. the residual blind region is *exactly* the
   instrument's declared position resolution, not a speed-dependent accident as
   it was under the absolute constant.

   What still catches such a clobber: `pos_cmd` steps from the ramp to the live
   *measured* position, so `dip_below_x3`, `peak` and `pullback` see the
   consequences even when `trunc` does not. Not scheduled for work — closing it
   would need a firmware-side "queue cleared" observable, which is the protocol
   change § CHECK HAND-2 already records as a follow-up.
