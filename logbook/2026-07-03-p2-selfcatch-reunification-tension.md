---
title: P2 self-catch re-unification — the smooth-motion cycle and the fragile MAKE are in DIRECT tension; the robust seat's runway IS the ceiling slam, so re-unifying on plan_cup_cycle without co-designing a gentler throw breaks the MAKE (0/12). STOPPED with evidence, no code landed.
type: investigation
date: 2026-07-03
status: blocked
phase: "Online-juggle tilt re-architecture — motion-quality course-correction (P2 re-unification attempted, blocked by MAKE fragility)"
related_plan: "bb-online-juggle-tilt-rearchitecture.md"
files_changed: []
commits:
  - 394d9bd
subsystem:
  - sim
tags:
  - control
  - review
  - blocked
---

# P2: re-unify the self-catch on plan_cup_cycle — attempted, blocked by the MAKE

> **Superseding follow-up (2026-07-03, same day).** This entry's conclusion — "the
> smoothing needs a gentler throw + apex-rendezvous catch" — was **refined and largely
> overturned** by `2026-07-03-catch-control-formulation-design-basis.md`. The "runway =
> ceiling slam" tension below is real *only for a catch that commands the hand as a
> CONSTANT position per 40 Hz tick* (jump-and-settle into an ultra-stiff actuator). The
> sim already runs the hand at 1 kHz (`plant.step` calls `hand_cmd_fn` every substep); a
> **continuous velocity-matched sub-tick command** makes the cup co-move at −2.7 m/s
> smoothly (measured −2700 mm/s vs the constant command's −35) and catch the fast
> arrival on a *descending* cup over a short runway — **no gentler throw needed.** The
> hand is not acceleration-limited (tens of g available). Read the design-basis entry
> for the real root cause + the proven fix; the attempts below all kept the
> constant-per-tick command, which is why they forced the gentle-throw conclusion.

## Summary

P2 asked to re-architect `SelfCatchRunner` (`sim/juggle_selfcatch.py`) onto the
codebase's own whole-cycle minimum-acceleration planner
(`controller/demo/juggle_planner.py::plan_cup_cycle`), deleting the
reposition/hold/carry-half/recovery stitch and the `carry_vel=[0,0,-2]` step, so
the toss/catch is SMOOTH (path ~1026 -> ~550-650 mm/cycle, **zero** ceiling
clamp, contact on a **descending** cup, no command step) while preserving the
MAKE (`SelfCatchConfig(oscillate=True, release="kinematic", dip_m=0.10)`,
sustained >= 10/12 all seeds).

**Outcome: I could not achieve both after extensive honest iteration (~35 probe
runs across the architectures below). No code was landed — the module is
unchanged and the MAKE is intact. The blocker is a real, physical tension, not a
tuning miss.** The smooth-motion cup cycle and the fragile MAKE's robust seat are
in *direct* conflict: the seat gets its downward runway from the very
ceiling-overshoot that "zero clamp + descending contact" forbids.

## The core tension (the load-bearing finding)

The seat is confirmed only when the ball is within `SEAT_RADIUS_M` (40 mm) of the
cup opening AND co-moving with the cup (`rel_vel <= SEAT_REL_VEL_MPS = 0.60 m/s`)
for `SEAT_PERSIST_SUBSTEPS = 15` substeps (`sim/ball/manager.py::_confirm_seat`).
The validated MAKE throws the ball to an apex ~370 mm above the release (v_take_vz
~= +2.69 m/s), so it **arrives at catch_z descending at ~-2.7 m/s**. To seat it,
the cup must be moving DOWN at ~-2.7 m/s (within 0.6 m/s) when they meet — i.e.
the cup needs ~60-100 mm of *downward runway* built up before the intercept.

The current (working) catch (`_catch`, the `kin` branch) gets that runway by
**overshooting UP to the hardware ceiling and falling back onto the ball**: its
co-moving z-tracker targets a clamped position (`z_t = min(pos_e+SEAT_DEPTH,
ready_z+SEAT_DEPTH) = 0.944 m`) but with the ball's full descent velocity
(`vz_t = vel_e[2] ~ -2.7`); the quintic then *overshoots above the clamp to
cup_z ~= 1.01 m (slider 355)* to "arrive at the clamp moving fast down." That
overshoot IS the review's 93 ceiling ticks, the "up-moving cup at contact", and
the "ball seats ~145 mm above catch_z on a ceiling-parked cup". The ball then
falls onto the ceiling-parked cup and seats. **The ceiling slam is the runway.**

Remove the overshoot (any variant that keeps the cup below the ceiling / receives
descending) and the runway vanishes: the cup, waiting low, cannot accelerate to
-2.7 m/s in the ~1 tick the fast ball takes to cross the seat zone, so **the ball
punches through and is not caught.** Verified repeatedly.

## Per-phase decomposition of the 1026 mm / 93 ceiling ticks (baseline, seed 0)

Instrumented the current runner by phase (kinematic osc MAKE, 3 cycles ÷ 3):

| phase             | z-path/cycle | ceiling ticks |
|-------------------|-------------:|--------------:|
| `_reposition_to`  | ~176 mm      | 0 |
| `_carry_up`       | ~230 mm      | 0 |
| `_release_recover`| ~297 mm      | 0 |
| `_catch`          | ~318 mm      | **93 (all)** |

So the stitch (reposition+carry+recover ~700 mm) is easy to remove, BUT **all 93
ceiling ticks live in `_catch`** — the catch itself is the slam, and the slam is
load-bearing for the seat (above).

## What I tried (and why each failed)

1. **One continuous `plan_cup_cycle` throw->catch->throw, played through** (the
   literal P2 ask). At the MAKE's 40 mm A<->B separation the cup sits under the
   ball's vertical path, and the min-accel plan *winds up above the throw* (to
   ~950 mm) then descends THROUGH the descending ball, colliding at ~858 mm
   (fast, un-matched) and batting it. The wind-up is the min-accel objective's
   response to the throw's `cvel[:,-1] == v_takeoff` (+2.69) constraint + the
   entry velocity — the plan always rides the full stroke `[665, 1010]`.
2. **Gentle plan-throw (decouple the kinematic release).** With a kinematic
   release the ball leaves at v_take regardless of cup velocity, so the plan's
   throw-velocity constraint is unnecessary; I faked `throw_target` to give the
   plan a small end velocity (`throw_vel_scale`). This shrank the oval and killed
   the wind-up, but the plan's open-loop velocity-match still never co-moves with
   the *actual* (noisy, estimated) ball for 15 substeps, so **it does not seat**
   (contacts empty across seeds).
3. **Tight planner z-bounds `[685, 985]` (margin inside the stroke).** Gives a
   clean path ~600 mm/cycle with zero HW clamp — but the cup then parks at the
   soft ceiling and the ball hits it un-matched; still no seat.
4. **Closed-loop co-moving tracker + low wait clamp + descending gate** (port of
   the working seat, ceiling removed). Zero ceiling ticks, no step, contact
   descending — but the ball punches through (0-2/12): the low wait starves the
   runway (the tension, directly).
5. **Catch near the apex** (ball momentarily slow -> trivial match, cup stays
   below ceiling). Two sub-variants: (a) pure plan — cup opening driven to
   `ball_apex + SEAT_DEPTH`, catch_vel ~0 — still open-loop, does not seat; (b)
   cup rising to meet the apex — **bats the ball up** (cup rising at contact ->
   rel_vel too high). Best 2/6.
6. **Config-only gentler throw on the UNCHANGED module** (lower apex via
   `throw_target_z`/`flight_s`, keep `_catch` verbatim). **Breaks the MAKE (0/12)**
   — the equilibrium is tuned to the specific fast throw.
7. **Most-conservative stitch removal**: replace only reposition+carry with a
   clean min-accel plan carry, keep `_release_recover` AND `_catch` *verbatim*.
   **Still 0/12** — the seat is fragile to the exact cup state it is handed;
   changing the pre-throw trajectory alone breaks it.

## Discussion

**Why this is a genuine tension, not a tuning miss.** The review's thesis — "the
non-smoothness is architectural; re-unify on `plan_cup_cycle`" — is right that the
*stitch* over-travel is architectural and removable. But it under-counted the
catch: the robust seat of a **-2.7 m/s** arrival intrinsically needs downward
runway, and on a 355 mm slider that runway can only come from (a) a ceiling
overshoot (what the code does — the slam), or (b) catching where the ball is slow
(near apex), which either doesn't seat (open-loop plan) or bats the ball (rising
cup). "Zero ceiling clamp + descending contact + preserve *this* fast-throw MAKE"
is over-determined: the same 40 mm-separation / fast-throw geometry that makes the
MAKE also forces the slam.

**Why every restructuring breaks the MAKE.** The 12/12 headline is a fragile
equilibrium tuned end-to-end to the stitched trajectory — the reposition dip, the
carry, the recover, and the ceiling-overshoot catch together shape the ball's
whole-cycle path. Changing *any* leg (even keeping the catch byte-identical and
only cleaning the carry hand-off; even just lowering the apex by config) drops it
to 0/12. This matches the plan-author's warning that "a naive one-line change to
the seat already dropped seed 5 to 8/12" — the sensitivity is worse than that: it
is whole-cycle, not seat-local.

**Why I stopped rather than ship a partial.** Two partial states exist, neither
acceptable: motion *can* be made clean (zero clamp, no step, descending) but then
the ball punches through (0-2/12); the MAKE holds only with the slam. Shipping the
clean-but-broken loop trades a documented-honest slam for an undocumented drop;
shipping a half-clean loop that still slams the ceiling misses the gate metrics
the review defined. Per the acceptance rule ("both, or stop; do not commit a
regression"), the disciplined move is to land the *evidence* and hand the parent a
scoped next step, not thrash.

## The real next step (co-design, not re-plumb)

The smoothing is **not separable from the throw**. A version that meets both would
have to co-design a NEW equilibrium: a **gentler throw whose apex sits below the
ceiling** (so the arrival is slow enough to seat with modest, sub-ceiling runway),
paired with a **closed-loop co-moving apex-rendezvous catch** (the cup pre-rises
to `apex + SEAT_DEPTH`, decelerating to ~0 there so it is *not rising* at contact,
then the tracker rides the actual ball down and arrests via the P3 momentum budget
`d = v²/(2·a)`), then re-tune sustained >= 10/12 from scratch on the new geometry.
That is a larger piece of work than a re-plumb onto `plan_cup_cycle` and should be
scoped as its own rung, with the MAKE gate re-established on the new throw.

## Verification

- Baseline (unchanged module), `SelfCatchConfig(oscillate=True, n_cycles=12,
  release="kinematic", dip_m=0.10)`, seeds 0-5 (run 2026-07-03): **12/12 all
  seeds**; motion-quality probe seed 0: **1026 mm/cycle, 93 ceiling ticks, contact
  cup vz +9..+11 (UP), 60 mm/tick step** (reproduces the review).
- Per-phase instrumentation (run 2026-07-03): table above; all 93 ceiling ticks
  in `_catch`.
- Restructuring attempts 1-7 (run 2026-07-03): best MAKE-preserving-with-clean-
  motion = 0/12; best clean-motion (zero clamp, descending) = 0-2/12 (punch-
  through). No configuration achieved both.
- Working tree left clean (`git status` clean); `sim/juggle_selfcatch.py` and
  `tests/sim/test_juggle_selfcatch.py` unchanged — the MAKE is intact.

## Related

- The review this follows: `logbook/2026-07-03-motion-quality-review.md`.
- The MAKE mechanism: `logbook/2026-07-01-rung2b-kinematic-release.md` (its
  "co-moving descent seat" narrative is the seat this entry finds is runway-fed
  by the ceiling overshoot — the earlier entry's follow-up note is still owed).
- The planner: `controller/demo/juggle_planner.py::plan_cup_cycle`; the
  continuous-cycle reference: `sim/juggle_online.py::_plan_cycle` (catches with
  the SAME planner because its 100 mm separation + 150 mm-lower catch keep the cup
  out of the ball's vertical path and give a natural down-stroke — neither holds
  at the MAKE's 40 mm / near-equal-height geometry).
