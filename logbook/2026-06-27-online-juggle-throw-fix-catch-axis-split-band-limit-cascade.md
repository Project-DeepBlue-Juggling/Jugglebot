---
title: Online juggle integration — throw-separation slam fix + catch axis-split (0→5 catches, beats the offline demo's 2) + the band-limit-cascade wall that blocks sustained 2-ball
type: feature
date: 2026-06-27
status: in-progress
phase: "Sim2Real fidelity — concern 1: online runner integration (sim/juggle_online.py). Throw fixed, catch axis-split landed (0→5 catches), sustained 2-ball blocked by the throw-amplifies-catch-error cascade — characterised, fixes ruled out, path forward identified"
related_plan: "bb-led-two-ball-juggle-demo.md"
files_changed:
  - sim/juggle_online.py
  - controller/demo/juggle_planner.py
commits:
  - 7cb81de
subsystem:
  - controller
  - sim
tags:
  - control
  - planning
  - tracking
  - kinematics
  - contact
---

# Online juggle integration — throw fix + catch axis-split + the band-limit cascade

## Summary

Continues `2026-06-27-online-replanning-architecture-and-cup-bandlimit` (the
architecture + planner) into the **runner** (`sim/juggle_online.py`). Took it
from **0 catches** at handoff to **5 catches / 0 drops** (seed 0) under full
contact physics with per-throw online re-planning — already **better than the
old offline demo's 2** (whose ≥30 headline test, `test_full_sim_juggle_
reaches_target_catches`, is `xfail(strict=True)` "pending the band-limit cascade
fix"). Two real fixes landed; then the **same band-limit cascade** that xfails
the old demo was characterised as the wall blocking sustained 2-ball, and a wide
space of candidate fixes was ruled out empirically. The fix that would actually
break the wall (a less position-sensitive throw, or a self-centring cup) is
scoped for a focused follow-up.

## Fix 1 — throw-separation slam (cycle-0 re-plan)

**Symptom**: the runner caught 0/0; the held ball was either cohesively dragged
back down (switched contact) or launched at **16.8 m/s** (≫ the planned 5,
stiff-always).

**Diagnosis** (`tools/probes/juggle_online_debug.py` + `/tmp/probe_plan_shape`):
not a contact problem — a **pre-roll → main-loop handoff discontinuity**. The
carry pre-roll's steep final z-rise to the throw (665→850 mm in the last 3 knots,
slider accelerating 0→5 m/s at ~135 m/s²) lags the slider, so the pre-roll ends
with the cup at **783 mm / +4.6 m/s** — but the main loop's `t_rel=0` then
commands the plan's *throw state* (**850 mm / +5.05 m/s**), a **+67 mm position
jump**. The slider over-corrects (overshoots to 1010 mm) and that velocity spike
slams the ball. Crucially, **every later cycle already avoids this** — it
re-plans from the *achieved* cup state, so it's continuous. Only cycle 0 handed a
lagging cup to a plan that assumed perfect placement.

**Fix**: re-plan cycle 0 from the achieved pre-roll cup state too (the online
architecture's own self-correction, applied to cycle 0). Result: clean
`a_cup < −g` separation at ~v_take (~4.5 m/s, the operator's
decelerate-and-separate model exactly), no slam. (An earlier prompt's
"bounded-slider can't clamp" framing was already known-wrong; this confirms
separation needs no clamp.)

## Fix 2 — catch axis-split (slider descends *with* the ball, moderately)

**Symptom**: the catch seated unreliably; the ball "sank below" the cup.

**Diagnosis**: the ball arrives at **−5.33 m/s** but the uniform soft
velocity-match (weight 200) let the smoothness cost win on the **vertical**, so
the cup **near-stopped** at the bottom of the oval (**−2.55 m/s**) while the ball
punched past it and sank below the 40 mm seat radius. The original field comment
*claimed* "the slider satisfies the vertical component exactly" — it did not.

**Fix**: split the catch velocity-match **by morphology axis** (the
level-platform decoupling applied to the catch, mirroring the old demo's separate
`catch_slider_vel_ratio`): a **strong** vertical (slider) match so the cup
descends *with* the ball, a **soft** lateral (platform) match. But a *full*
vertical match overshoots — it drives the cup at the ball's full −5.3 m/s, which
on the short slider **slams the floor** (only ~40 mm below the catch) and ejects
the ball (captures → 0). The **seating sweet spot** is a **moderate** descent,
cup ~**−3.4 m/s** (`catch_slider_vel_ratio=0.7`, `weight=600`):

| cup catch vz | captures (10 s sweep, `/tmp/probe_grid`) |
|---|---|
| −2.55 (near-stop, uniform soft) | 1 |
| **−3.43 (moderate)** | **5** |
| −4.73 (strong) | 2 |
| −5.34 (full match, floor slam) | 0 |

Result: **0 → 5 catches**. This is the one lever that worked, because it improves
seating **without distorting the throw**.

## The wall — the throw amplifies the band-limited catch error

5 catches, not sustained: the pattern **collapses to 1-ball within 1–2 cycles**.
A pre-seated (perfectly centred) ball **grooves indefinitely** (caught at
7–12 mm every cycle); any **caught-then-thrown** ball **diverges**. Measured
(`/tmp/probe_throwchar`, `/tmp/probe_arrival`):

1. The catch seats the ball **~15 mm off-centre** in the cup — the platform's
   **band-limited lateral tracking error** at the catch, irreducible on this
   actuator.
2. The contact-carry throw **amplifies** that: an off-centre ball launches with a
   large lateral velocity (vx **+282 mm/s** vs the intended −94) and **lands
   >150 mm off — just beyond the platform's ±150 mm reach** → the catch can't
   reach it → lost.
3. **Loop gain > 1** outside a ~10 mm basin. The centred ball stays centred; the
   off-centre ball walks off (~+319 mm/cycle once diverged).

This is **exactly the "band-limit cascade"** the old demo's xfail names — a real,
deep, previously-unsolved problem in this project, not a tuning miss. The online
runner (5) already beats the old offline demo (2); both hit the same wall.

## Discussion — what was ruled out, and why

The catch axis-split worked because it improves seating without touching the
throw. **Every other lever failed**, and the failures triangulate the wall:

- **Catch dwell** (port of the old demo's reach-hold-dwell into the NLP — the
  user's explicit choice): a window/weight sweep found **no-dwell (5) beats every
  dwell setting (1–4)**. The dwell fixes the caught ball's co-motion but
  **distorts the throw** (the NLP redistributes the trajectory to satisfy the
  xy-hold), and it merely **swaps which ball is lost**. Kept as a disabled opt-in
  knob (the settle-don't-swipe lesson is sound; a future pattern with more
  catch→throw dwell time may benefit) — see `PlannerConfig.catch_dwell_*`.
- **Throw coast** (the operator's decelerate-and-separate model — press the ball
  in so friction settles its lateral kick before separation): the **hard** coast
  (`acc==0` at the throw) is **infeasible** — it keeps the cup rising into the
  slider ceiling within the tight apex-1.3 vertical budget. The **soft** coast
  (penalty toward zero post-throw vertical accel) **hurts** (5→1–2), same as the
  dwell — it distorts the throw. Kept as a disabled opt-in knob
  (`PlannerConfig.throw_coast_*`). The deeper reason it can't work as hoped: the
  kick happens during the **carry up-stroke**, before separation, not at the
  throw.
- **Symmetric startup** (both balls enter via catches; no pre-seated ball thrown
  from the imperfect pre-roll): **worse (2)**. It removed the *only* clean throw
  (the pre-seated, centred ball), so neither ball grooves — which **confirmed**
  the binding constraint is throw consistency, not the startup asymmetry.
- **Separation** (100 mm) is already optimal; **75/85/120 mm** score 1–3.
- **Lateral smoothness** (higher `lateral_accel_weight`, lower `max_jerk_xy`): no
  help.
- **Workspace/reach**: only "helps" (6) at an **unphysical 300 mm** reach (the
  platform does ~150 mm), so it isn't real — and with loop gain >1 a wider basin
  only delays divergence.
- **Lower apex** (gentler pattern): **worse** (0.7/0.9/1.1 m → 1; 1.3 m → 5) with
  the fixed throw/catch heights — and apex ≤0.5 m goes infeasible. A genuinely
  gentler pattern would need throw/catch heights re-tuned to match; not a
  drop-in.

The unifying finding: **anything that distorts the NLP trajectory worsens the
throw**, because throw consistency — not catch detection — is the binding
constraint, and **apex 1.3 makes the whole pattern tight** (fast tempo fills the
slider, the catch tracks imperfectly → the ~15 mm error, the throw amplifies it).

Two hypotheses were withdrawn under evidence (per the engineering philosophy):
(a) "raise the catch off the slider floor" — raising catch_z made it *worse*;
(b) "the lateral dwell will fix the divergence" — it swapped which ball survives,
revealing the throw (not the catch) as the wall.

## Path forward (scoped for a focused follow-up)

The actual fix must break the **position → velocity amplification** in the throw:

1. **A less position-sensitive throw** — e.g. a small controlled-velocity nudge
   toward the planned v_take during the stiff-throw window, blending contact
   fidelity with a stability correction. Most promising; stays mostly faithful.
2. **A self-centring cup** (model change) — deeper/steeper cup or lower friction
   so an off-centre ball rolls to centre during the carry before the up-stroke.
3. **Re-tune the whole pattern gentler** (lower apex *with* matched throw/catch
   heights), trading demo height for a slower tempo that the band-limited
   platform tracks within the basin.

## Verification

- `python sim/juggle_online.py` (committed defaults: `duration_s=20`, seed 0;
  run 2026-06-27): **5 captures / 0 drops** under full contact + online
  re-planning. All 5 land by t≈4.9 s, then the pattern collapses to 1-ball (the
  wall below) — so the count is stable across 10/12/20 s windows.
- `pytest tests/sim/test_demo_juggle_planner.py -q` (run 2026-06-27): **7
  passed** — the catch axis-split + opt-in dwell/coast knobs leave the planner's
  hard throw/catch position constraints and jerk bounds intact (the 7 tests pin
  those; none asserts the soft catch velocity).
- Full suite: `pytest tests/ -q` (run 2026-06-27) — result cited in the commit.
- Old-demo calibration: `test_full_sim_juggle_reaches_target_catches` is
  `xfail(strict=True)` at 2 catches (band-limit cascade); this online runner
  reaches 5 against the same wall.
