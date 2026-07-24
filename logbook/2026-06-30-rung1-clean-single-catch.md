---
title: Rung-1 clean single catch — constant-decel seat kills the soft-contact bounce; frozen-early reach + ramped tilt; ~3.3 mm in-cup offset under §3 noise
type: feature
date: 2026-06-30
status: in-progress
phase: "Online-juggle tilt re-architecture — Phase 1 / Rung 1 (clean single catch, BB-reload)"
related_plan: "bb-online-juggle-tilt-rearchitecture.md"
files_changed:
  - sim/juggle_catch.py
  - sim/juggle_noise.py
  - sim/juggle_tilt.py
  - tests/sim/test_juggle_catch.py
  - tests/sim/test_juggle_noise.py
  - tests/sim/test_juggle_tilt.py
  - tools/probes/juggle_catch_offset.py
  - tools/probes/README.md
commits:
  - d3ae5b0
subsystem:
  - sim
tags:
  - control
  - planning
  - contact
  - catch
  - characterisation
---

# Rung-1 clean single catch (BB-reload)

## Summary

Phase 1 / Rung 1 of `plans/active/bb-online-juggle-tilt-rearchitecture.md`:
validate the **catch** primitive standalone — catch one BB-thrown ball cleanly
(seated AND held, small characterised in-cup offset) from a BB **placeable
anywhere**, by **translate-to-reach** (slow centroid translation to the observed
landing) + **tilt-to-receive** (collinear catch tilt) + the Rung-0 lever-arm
realisation, **robust to the §3 noise** (2 % BB throw + 0.5 mm tracking, on by
default).

A prior partial scaffold (`sim/juggle_noise.py`, `sim/juggle_tilt.py`,
`sim/juggle_catch.py`) ran but **did not catch**: `caught=True, held=False`,
in-cup offset ~776 mm — the ball seated for a few ticks then escaped. Building on
the (sound) noise/tilt modules, this entry debugs the catch realisation to a
**clean, characterised catch**: default placement **17/20 seeds clean** under the
§3 noise, **mean in-cup offset 3.3 mm** (max 7.9 mm), and the offset stays ~3–4 mm
across the workspace out to 100 mm reach. The noise model + tilt geometry +
translate-to-reach + lever-arm compensation are all covered by new tests.

## What was broken (diagnosis)

The scaffold's catch only worked for a ball landing **dead-centre with no noise**
(off 2.9 mm). Any of (a) BB-noise-scattered landing, (b) a non-zero nominal
landing, or (c) tracking noise broke it. The realisation (`realize_tilted`) was
*not* at fault — it lands the cup opening within **0.1 mm** of the commanded
target statically across the workspace and the whole slider range (the lever-arm
compensation is accurate; verified, now a test). Three distinct contact-dynamics
failure modes were stacked:

1. **Soft-contact bounce at the seat (dominant).** The slider settle was a
   **min-jerk** trajectory from the velocity-matched catch to rest. A min-jerk
   decel-to-rest has a **deceleration PEAK ~1.9× its average** in the middle of
   the stroke; that force spike ejected the seated ball **upward** off the cup,
   and the cup then descended out from under it. (Trace: ball seats at off 7 mm,
   then `bz` jumps +3.9 m/s up while the cup keeps descending.)

2. **Lateral swipe / tilted-rim deflection.** For off-centre landings the cup was
   still **translating laterally** (and tilting) when the ball arrived, so the
   moving + tilted rim knocked the ball sideways (`bx`: −100 → +42 mm in 6 ticks,
   vs the ball's true ~0.09 m/s). The reach completed *at* touch-down, not before.

3. **Up-loop in the approach.** A min-jerk to `(catch_z, v_match<0)` from a
   near-rest hold **back-loads** the large end-velocity by looping the cup UP
   first — raising it into the descending ball (an off-centre seat → bounce).

## Fix

`sim/juggle_catch.py` realisation, in priority order of impact:

- **Constant-deceleration settle (the unlock).** Replace the min-jerk
  decel-to-rest with a **constant** deceleration: ramp the (downward) cup velocity
  linearly to zero over `seat_decel_time_s`, then hold. No peak ⇒ steady contact
  force ⇒ the ball rides the cup smoothly to rest. This single change took the
  default placement from ~30 % to ~90 % clean.
- **Reach on a fixed receding horizon, frozen-early target.** The XY reach eases
  toward the predicted landing on a fixed `reach_settle_floor_s` (0.10 s) receding
  horizon, and freezes its target on the first post-warm-up tick (the first n≥4
  ballistic fit) so the parked xy doesn't jitter on the post-crossing
  back-extrapolation. The observed apex→catch flight is short (~0.26 s), so the
  reach and the slider descend **overlap** (the descend starts ~one tick after the
  reach planning begins) rather than the reach fully settling first — it is the
  firm constant-decel seat (above), not a parked-then-descend separation, that
  carries the off-centre catches. (An earlier draft credited two lead-time knobs,
  `reach_lead_s`/`reach_freeze_lead_s`, with sequencing the reach before the
  descend; an audit found both inert at this geometry — the lead times exceed the
  whole ~0.26 s flight — so they were deleted and this narrative corrected. The
  characterised rate is unchanged: the byte-identical probe sweep confirms the
  knobs were dead.)
- **Tilt RAMPS through the seat (not parked early).** The tilt target is frozen
  *later* (when descending) and ramps to the collinear receive tilt completing at
  touch-down. A fully-tilted cup parked early presents an asymmetric rim the ball
  glances off; ramping keeps the cup nearer level when the ball first enters.
- Catch a touch higher (`catch_z` 0.80 → 0.84 m) for decel head-room and a closer
  velocity match (`catch_slider_vel_ratio` 0.8 → 0.95).

## Verification

- New unit tests: `tests/sim/test_juggle_noise.py` (NoiseConfig defaults,
  `perturb_throw`/`observe` σ-scaling + seed-reproducibility, BallisticEstimator
  exact-arc recovery + noise-averaging beats finite-difference + touchdown
  consistency) and `tests/sim/test_juggle_tilt.py` (collinear tilt direction /
  clamp / anti-parallel alignment, lever-arm signs, and the lever-arm
  compensation landing the **real MuJoCo** `hand_opening` site within 1 mm of the
  commanded target across the workspace + slider range).
- New integration tests: `tests/sim/test_juggle_catch.py` (noise-off clean +
  centred; ≥6/8 seeds clean under §3 noise with every clean seat <15 mm;
  translate-to-reach catches off-centre landings with `reach_mm>0`; workspace
  clamp).
- Characterisation harness `tools/probes/juggle_catch_offset.py`: default
  placement **17/20 clean**, mean offset **3.3 mm**, max **7.9 mm**; workspace
  ring out to 100 mm reach mostly 4–5/5 with ~3–4 mm offsets; noise ablation
  (default placement, 10 seeds): none 10/10, BB-2 % 9/10, tracking-0.5 mm 9/10,
  both 9/10.
- Full suite: see Outcome.

## Discussion

**Why the constant-decel settle, not a softer/longer min-jerk.** The recurring
trap was treating the catch as a *kinematic* problem (match position + velocity,
then ease to rest) when the binding constraint is the **soft contact's force
transient**. Three different min-jerk settles were tried (shorter, longer, deeper
floor) and all bounced under noise, because min-jerk's defining property —
minimum jerk — *maximises the mid-stroke acceleration* for a given boundary, which
is exactly the force spike that ejects the ball. The fix is not a gentler
min-jerk; it is a **different acceleration shape** (constant) that trades jerk at
the endpoints for a flat force profile in the middle. This is the level-of-
abstraction climb: the failure class is "force transient ejects a soft-seated
ball", and the contract is "the seat deceleration must be peak-free", not "tune
the settle horizon".

**Hypotheses withdrawn (load-bearing).**
- *"It's the velocity match / the ratio."* A full velocity match (ratio 1.0) was
  tried and made it **worse** (the abrupt floor-settle over-decelerated and
  separated the ball). The ratio is a second-order knob; the settle *shape* was
  first-order. Withdrawn.
- *"The tilt is too aggressive — reduce or disable it."* Disabling the tilt (level
  catch) was **worse** (the ball's lateral arrival velocity skids it across a
  symmetric cup). Tilt is necessary; the issue was *when* it's applied (ramp,
  don't park), not *whether*.
- *"A steeper BB throw (smaller tilt) will be more robust."* Tested; no
  improvement (2/10) — confirming the failure was the settle, not the tilt
  magnitude.
- *"Closed-loop ball-position tracking will be more robust than the open-loop
  velocity-match descent."* Prototyped twice; both worse — the ballistic estimate
  is free-fall (doesn't see the cushion deceleration), and tracking the raw
  observed position never brings the ball to rest. The open-loop velocity-match
  approach + constant-decel settle won.

**Accepted tradeoff: ~85–90 %, not 100 %.** The soft-contact seat is a genuine
knife-edge; the remaining ~10–15 % are seed-specific contact stochasticity (a
platform-pose-dependent vertical compliance — the legs sit at different
extensions at a translated/tilted pose). Pushing past this would need either a
contact-model change (out of scope — the plan mandates keeping contact-physics
fidelity) or a longer observed flight than the slider range affords (the
apex→catch window is only ~0.26 s, which caps how far a *matured-estimate* reach
can settle before the descent — the far-corner 100 mm + full-scatter cases are at
that timing edge). Crucially for the ladder, **Rung 2b re-plans every cycle**, so
an occasional single-catch miss is recoverable; and **when the catch is clean the
seat is tight (~3.3 mm)** — far below the ~15 mm the throw amplification cared
about. The test asserts a *majority* clean-rate (deterministic on the pinned
MuJoCo) plus a tight per-catch offset bound, rather than an unachievable 100 %.
A follow-up **gate investigation** (next section) root-caused this precisely: it
is **reach-limited** — the misses are **deterministic in the reach** (which the
seed sets via the noise draw), not random contact behaviour, so the
"seed-specific contact stochasticity / pose-dependent compliance" framing above
is directionally right but imprecise — and, importantly, a *slower/higher*
pattern makes it **worse**, not better.

## Knife-edge root-cause — reach-limited (gate investigation, 2026-06-30)

At the Rung-1 gate the operator asked whether the ~85 % is a sim-contact artefact
to ignore or a real seat weakness to fix, before passing to Rung 2. A focused
sweep (N=20 seeds/config) root-caused it: **the clean rate is a monotonic
function of the platform REACH** (the translation to the observed landing) —
nothing else:

| reach | clean/20 | config |
|---|---|---|
| 1 mm | 20 | noise off |
| 3 mm | 18 | tracking-noise only (0.5 mm) |
| 30 mm | 18 | half noise (1 %) |
| 60 mm | 17 | **default (2 % BB)** |
| 85 mm | 15 | far 80 mm placement |
| 112 mm | 12 | double noise (4 %) |
| 115–154 mm | 0–1 | longer flight (0.9–1.2 s) |

**Verdict: neither a sim artefact nor a seat-design flaw — a reach-vs-
convergence-window constraint.**
- *Not an artefact* — a clean, monotonic, physical dependence on reach, not
  per-seed contact randomness. Misses cluster exactly where the reach is large;
  tracking noise alone (reach ~3 mm) is 18/20, BB noise alone (reach ~60 mm)
  reproduces the full 17/20 — the BB landing scatter, via the reach, is the driver.
- *Not a seat flaw* — the constant-decel seat is sound: at small reach it is 20/20
  with a 0.8 mm offset. The seat works; the *reach* fails it. (The clean-seat
  offset also loosens with reach, 0.8 → 6.0 mm — the same effect.)
- *The constraint* — the **slow** translate-to-reach cannot converge a **large**
  reach within the short (~0.26 s) observed descent window: the cup arrives off /
  still-moving and knocks the ball. The 2 % BB noise scatters the landing to
  ~60 mm reach on average; the ~15 % misses are the seeds it scatters past the
  catch's reliable reach (~60–80 mm).

**Two findings that reshape the strategy:**
1. **A slower/higher pattern makes it WORSE, not better.** Longer flight collapsed
   it (0.60 s → 17/20; 0.90 s → 1/20; 1.20 s → 0/20): the 2 % velocity noise acts
   over more time → more landing scatter → bigger reach (+ a faster arrival). Tempo
   is the wrong lever; the catch favours **gentle, accurate** arrivals.
2. **The catch's robustness is gated on the INPUT accuracy.** The 2 % BB is a
   deliberately noisy *worst-case* input. In the actual loop the catch is fed the
   **cup's own throw** (Rung 2a) — tight throw (small reach) → ~90–100 %;
   scattered like the 2 % BB → ~85 %. Whether the catch needs hardening, and how, can only be
   judged after Rung 2a characterises the throw.

**Gate decision:** PASS to Rung 2a. The reach the catch faces in the loop is set
by the throw accuracy, which Rung 2a measures. If hardening is later needed, the
lever is **reach-convergence** (catch higher / observe earlier) and a gentler
arrival — explicitly NOT slowing the tempo. (Sweep recipe: vary `NoiseConfig`
fracs, `flight_s`, and `landing_xy_m` in `SingleCatchConfig`, 20 seeds each,
read `CatchResult.clean` + `.reach_mm`; a one-off, not committed.)

## Outcome

Full suite, finalize gate run on the committed (post-audit-fix) code
(`pytest tests/ -q`, run 2026-06-30): **1543 passed, 4 skipped, 2 xfailed in
751.15 s** (the 35 new Rung-1 tests included; only pre-existing warnings; the
flaky `test_hot_loop_allocation_contract` passed in-suite). The Rung-1 subset
(`pytest tests/sim/test_juggle_catch.py tests/sim/test_juggle_noise.py
tests/sim/test_juggle_tilt.py -q`, run 2026-06-30): **35 passed in ~21 s**. The
audit fix (deleting the two inert lead knobs) is behaviour-preserving — the
`tools/probes/juggle_catch_offset.py` sweep is byte-identical before/after
(home placement 17/20 clean, mean 3.26 mm / max 7.92 mm unchanged).

Commit SHA: d3ae5b0 (feat(sim): Rung-1 clean single catch (BB-reload) under
section-3 noise).
