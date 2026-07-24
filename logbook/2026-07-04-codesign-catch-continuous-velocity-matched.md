---
title: Co-design catch — the continuous velocity-matched sub-tick command makes the single-ball self-catch SMOOTH (cup path 1026→322 mm/cycle, ZERO slider clamps, contact received DOWN at −0.67 m/s) WITHOUT a gentler throw, and the same insight retires the vestigial v_takeoff-overshoot carry; the Rung-2b MAKE is regenerated 12/12 on all 6 seeds and is tighter than before (in-cup offset 0.6→0.3 mm)
type: feature
date: 2026-07-04
status: resolved
phase: "Online-juggle tilt re-architecture — motion-quality course-correction (the co-design catch rung: implementing the 2026-07-03 design basis)"
related_plan: "bb-online-juggle-tilt-rearchitecture.md"
files_changed:
  - sim/juggle_catch.py
  - sim/juggle_selfcatch.py
  - tests/sim/test_juggle_selfcatch.py
  - tools/probes/juggle_motion_quality.py
  - tools/probes/README.md
  - plans/active/bb-online-juggle-tilt-rearchitecture.md
  - logbook/INDEX.md
commits:
  - 3126c6d
subsystem:
  - sim
tags:
  - control
  - planning
  - contact
---

# Co-design catch — continuous velocity-matched sub-tick command

## Summary

Implements the 2026-07-03 catch control-formulation design basis
(`2026-07-03-catch-control-formulation-design-basis.md`). The single-ball
self-catch is now **smooth**: the cup receives the ball on a **descending** cup
by tracking a **continuous velocity-matched trajectory at the sub-tick (1 kHz)
rate**, instead of snapping to a constant per-40 Hz-tick position and settling.
This resolves the non-smooth catch **without a gentler throw** — the P2 entry's
"the runway *is* the ceiling slam" tension dissolves once the command is
continuous — and the **same insight** (the kinematic release imposes the take-off
velocity regardless of the cup's velocity) retires the vestigial `v_takeoff`
carry that made the cup overshoot ~120 mm upward after every throw.

**Motion quality (self-catch MAKE, kinematic oscillation, seed 0), before → after:**

| metric | before (constant per-tick) | after (continuous) | target |
|---|---:|---:|---|
| cup z path / cycle | **1026 mm** | **322 mm** | ~550–650 (halved) |
| slider-clamp ticks | **93 ceiling** | **0** | 0 |
| cup vz at ball contact | **+9…+11 mm/s (UP)** | **−669 mm/s (DOWN)** | DOWN (receive) |
| max per-tick z displacement | 60.5 mm | 34.7 mm | (velocity-match signature) |
| disengaged travel | 45 % | 26 % | small |
| stroke span used | 350 / 355 mm | 184 / 355 mm | a fraction |

**MAKE regenerated (and tighter):** the A↔B oscillation sustains **12/12 cycles on
all 6 seeds** (release="kinematic", dip_m=0.10), every cycle genuinely **separated**
and **physically seated** (transient seat offset ~27 mm → settles centred), landing
flat at ~3.4 mm and rejecting the disturbance to a **flat 0.3 mm** in-cup offset —
*tighter* than the pre-fix ceiling-slam MAKE's 0.5–0.6 mm.

## Root cause (recap of the design basis)

The hand is a MuJoCo `position` actuator (kp=100000, no force limit) that moves
200 mm in one 25 ms tick — tens of g of authority, **not** acceleration-limited.
`MuJoCoPlant.step` already loops 25 substeps and samples `hand_cmd_fn(t)` every
substep. The old catch passed `hand_cmd_fn = lambda _t: <constant slider>` — the
40 Hz value frozen across the whole tick — so the ultra-stiff actuator **jumped to
the setpoint and settled within the tick** (near-zero velocity at the sample
instant). To get *any* downward velocity at the crossing it targeted the ball's
descent velocity toward a clamped position, and the quintic **overshot to the
stroke ceiling** to "arrive at the clamp moving fast down" — the 93 ceiling ticks,
the +9 mm/s UP contact, and the ball seating ~145 mm above `catch_z` on a parked
cup that the motion-quality review found.

## The fix

**1. Continuous velocity-matched command (the design-basis mechanism).** A new
`_ContinuousQuintic` (`sim/juggle_catch.py`) keeps the quintic coefficients so it
can be *sampled at any intra-tick time*; the catch builds one per tick for xy / z
/ tilt and hands `plant.step` a `hand_cmd_fn` / `plat_cmd_fn` that evaluate it at
each substep's sim time. The cup then genuinely descends at the ball's velocity.
Isolated A/B (reproducing the design basis, `/tmp/probe_command_continuity.py`):
at a −2.7 m/s target the cup moves **−2689…−2700 mm/s** continuous vs **−37…−33
mm/s** constant — same per-tick displacement, opposite velocity content.

**2. Momentum-budget arrest.** Once the true ball reaches `catch_z` the co-moving
cup is decelerated to rest over `d = v²/(2·a)` at `CATCH_ARREST_ACCEL_G = 6 g`
(~60 mm for the arrival) — a **min-jerk** quintic re-solved every tick (the
receding-horizon re-solve + the `3·dt` horizon floor mean only its low-jerk onset
executes, so no mid-stroke deceleration peak ejects the ball — the anti-ejection
property the old constant-decel seat guaranteed, now via a smooth bounded-jerk
onset). The budget sets the endpoint/travel, which is *bounded*, so a fast arrival
can't slam the slider floor once the cup honestly tracks the command (a
fixed-*time* decel would).

**3. The enabled corollary — retire the `v_takeoff` carry (kinematic only).**
`ballistic_release` imposes `v_takeoff` on the ball **independent of the cup's
velocity**, so the carry need not build `v_takeoff`. The kinematic carry now lifts
the cup to the throw point ending at **~rest**, and the recover (`_release_recover_
kinematic`) just eases the near-stationary cup up to the catch ready height — no
~120 mm post-release upward overshoot to claw back. The ball is ballistic +
contact-disabled after release, so its landing is **decoupled** from the cup's
recover motion (verified: identical landing/loop across dip and recover variants).
This is what takes the path from 871 mm (continuous catch alone) to **322 mm**.

All three changes are gated on `cfg.release == "kinematic"`; the `detach` path (and
`SingleCatchRunner` for the shipped Rung-1 catch, made continuous the same way) keep
tilt=0 / Rung-1 / Rung-2a byte-identical.

## Discussion

**Why this dissolves the P2 "runway = slam" tension (hypothesis reframed).** The
P2 re-unification entry (`2026-07-03-p2-selfcatch-reunification-tension.md`)
concluded, after ~35 probe runs, that a −2.7 m/s arrival *intrinsically* needs a
downward runway that only a ceiling overshoot (or a gentler throw) can supply — a
genuine physical tension. That conclusion was **load-bearing but conditional**:
every one of those runs kept the *constant-per-tick* command, under which the cup
cannot carry velocity across the sample instant, so the only way to have a
moving cup at the crossing was to overshoot. Free the command to be continuous and
the cup co-moves over the **same displacement** with **real** velocity — the
runway is built by *velocity continuity*, not by stroke overshoot. The design
basis narrowed P2's "smoothness vs the fast throw" tension to "smoothness vs a
*constant-per-tick* catch," and this entry confirms the narrowing empirically: the
fast throw is caught on a descending cup with zero clamp and **no gentler throw**.
This is the project's recurring pattern — the confidence of P2's explanation was
not evidence for its sufficiency; the next tool (continuous command) didn't fit the
"needs overshoot" hypothesis, so the hypothesis was reframed rather than rescued.

**Why the carry redesign belongs here (not scope creep).** The operator's original
complaint was two-part: the hand (a) "moves up and down its stroke significantly
more than it needs to" and (b) "doesn't accelerate to *receive* the ball." The
continuous catch fixes (b). Part (a) is dominated by the `v_takeoff` carry: the cup
was driven up to +2.7 m/s at release purely because the *emergent contact* throw
imparts the cup's velocity to the ball — but the kinematic release does not, so
that acceleration (and the overshoot + recover it forces) is a workaround from a
throw mode we no longer use, *exactly analogous* to the ceiling overshoot being a
workaround for the constant command. Removing it is the same "the hand moves only
to interact with the ball" principle applied to the throw side. It is a larger
diff than the literal "continuous catch," but it is what delivers "path ~halved"
(322 vs 871), and it is what makes the change honest to the operator's principle
rather than to the letter of one phase.

**The metric that lies, and the one that doesn't.** The review's `max_cmd_step`
metric measures per-tick z *displacement*. For a velocity-matched descent that is
`v·dt` **by construction** (67 mm/tick at −2.7 m/s), so a large "step" there is the
co-move signature, **not** a discontinuity — the old constant-command catch and a
perfect continuous catch both read ~60 mm for opposite reasons. The probe now also
reports `max |Δcup vz|/tick` (command continuity) and its guidance says to judge
command smoothness by the velocity step, not the displacement. The decisive,
unambiguous signals remain **zero clamp**, **contact moving DOWN**, and the
**disengaged fraction** — all three move the right way (93→0, +10→−669, 45→26 %).

**What was ruled out.** (a) A gentler throw (P2's proposed co-design) — unnecessary;
the kinematic velocity release is unchanged. (b) A hand-acceleration setting — the
sim hand applies none; it was never the lever (design basis). (c) A deeper dip cut:
`dip_m=0.06` was MAKE-safe and trims a further ~65 mm (322→256), but `dip_m=0.10`
is the established MAKE value and 322 is already well past halved, so the dip is
left unchanged to minimise config churn (recorded here as a cheap future lever).

**Tradeoff accepted.** The kinematic catch/carry/recover are now a materially
different trajectory shape from the detach path they branch from. The gate is the
strict `test_detach_mode_is_byte_identical_default` plus the detach column/oscillation
BREAK tests, which pin that divergence to the kinematic branch only. The fidelity
caveat from the kinematic release (`2026-07-01-rung2b-kinematic-release.md`) is
unchanged and still owed to hardware bring-up: we do not resolve the final contact
millisecond of the throw, and we now also assume the cup can release near-rest and
still deliver a clean ballistic launch — a sim idealisation to re-check on hardware.

## Verification

- **Continuity mechanism (design-basis A/B, `/tmp/probe_command_continuity.py`,
  run 2026-07-04):** cup vz at a −2.7 m/s target — **continuous −2689/−2700 mm/s vs
  constant −37/−33 mm/s** (reproduces the design-basis table).
- **MAKE, real module (direct `run_self_catch`, run 2026-07-04):** seeds 0–5,
  `SelfCatchConfig(oscillate=True, n_cycles=12, release="kinematic", dip_m=0.10)` —
  **12/12 every seed**, all cycles separated, all real seats (seat_offset > 0),
  landing flat ~3.4 mm, in-cup offset flat **0.3 mm**.
- **Motion quality (`python tools/probes/juggle_motion_quality.py`, run 2026-07-04):**
  self-catch MAKE **322 mm/cycle, ceil 0 / floor 0, contact −669 mm/s DOWN, max cmd
  step 34.7 mm/tick (the velocity-match signature), disengaged 26 %**.
- **Rung-1 catch (`pytest tests/sim/test_juggle_catch.py -q`, run 2026-07-04):**
  **8 passed in 31.54 s** — the continuous `SingleCatchRunner` still seats the gentle
  and fast (real-BB ~4.9 m/s) arrivals within the characterised offset.
- **BB-catch tool (`pytest tests/sim/test_juggle_bb_catch.py -q`, run 2026-07-04):**
  **6 passed in 14.63 s.** The continuous `SingleCatchRunner` shifted one FAR-REACH
  edge-case seat (seed 2's §3 noise scatters the landing to ~153 mm reach, at the
  150 mm workspace clip) from ~20 → **22.3 mm** in-cup — still a clean, held catch,
  well inside the 40 mm seat radius. The pinned clean-seat bound (which encoded the
  old jump-and-settle catch) was consciously re-baselined 20 → 25 mm, with the
  measured value recorded in the test docstring (audit Finding 1, operator-approved).
- **Self-catch MAKE / byte-identity / motion-quality tests:** all green within the
  full suite below. The MAKE is additionally verified directly (12/12 all seeds,
  above); byte-identity by `test_detach_mode_is_byte_identical_default` + the detach
  column/oscillation BREAK tests (the retained column-degenerate `xfail(strict=True)`
  is intact); smoothness by the new `test_kinematic_catch_motion_is_smooth` (asserts
  ZERO slider clamps, contact moving DOWN, cup path < 550 mm/cycle).
- **Full suite (`pytest tests/ -q`, run 2026-07-04):** **1602 passed, 4 skipped,
  3 xfailed in 1020.39 s (0:17:00), exit 0 — no failures.** The mandatory pre-commit
  gate. This change adds one test (`test_kinematic_catch_motion_is_smooth`); the
  3 xfailed (incl. the retained column-degenerate BREAK) and 4 skipped are unchanged.

## Related

- The design basis this implements: `2026-07-03-catch-control-formulation-design-basis.md`.
- The chain it closes: `2026-07-03-motion-quality-review.md` (P0 + the non-smoothness)
  → `2026-07-03-p2-selfcatch-reunification-tension.md` (the "runway = slam" tension,
  now dissolved by the continuous command) → this entry (the co-design catch MAKE).
- The MAKE it regenerates: `2026-07-01-rung2b-kinematic-release.md`. Its "co-moving
  descent seat" — which that entry's 2026-07-03 follow-up note found was *aspirational*
  (the ceiling-slam MAKE was a static-cup catch) — now **genuinely executes**: with the
  continuous command the cup receives the ball on a descending, co-moving stroke.
- Key code: `sim/juggle_catch.py` (`_ContinuousQuintic`, `CATCH_ARREST_ACCEL_G`, the
  continuous `SingleCatchRunner` catch); `sim/juggle_selfcatch.py`
  (`_catch_continuous`, `_release_recover_kinematic`, the kinematic carry);
  `tools/probes/juggle_motion_quality.py` (the gate + the command-continuity metric).
