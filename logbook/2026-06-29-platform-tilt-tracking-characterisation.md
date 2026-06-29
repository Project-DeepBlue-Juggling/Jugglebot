---
title: Platform-tilt (orientation) tracking characterisation — tilt is static-exact, ~1.66 mm/deg lever arm, generous band & leg headroom; viable for throw-aiming
type: feature
date: 2026-06-29
status: in-progress
phase: "Online-juggle tilt re-architecture — Phase 0 (characterise tilt/orientation tracking before re-introducing tilt in the planner)"
related_plan: "bb-online-juggle-tilt-rearchitecture.md"
files_changed:
  - tools/probes/juggle_tilt_bandlimit.py
  - tools/probes/README.md
commits:
  - 3bb19a5
subsystem:
  - sim
tags:
  - control
  - planning
  - tracking
  - kinematics
  - characterisation
---

# Platform-tilt tracking characterisation (Phase 0)

## Summary

The online-juggle tilt re-architecture (`plans/active/bb-online-juggle-tilt-rearchitecture.md`)
aims each throw by **tilting the cup** and detaching the ball along the tilted
axis: the lateral take-off velocity becomes `slider_speed × sin(tilt)`, delivered
by the fast/accurate slider along a roughly-constant tilt, instead of by the
band-limited platform *translation* that wrecked the level-decoupled design (the
catch's ~15 mm offset got amplified beyond the ±150 mm reach → divergence, logbook
`2026-06-27-online-juggle-throw-fix-catch-axis-split-band-limit-cascade`). Phase 0
asks: does the Stewart platform actually *track tilt* well enough to bet the
re-architecture on it?

**Verdict: yes, comfortably.** Tilt is held **static-exact** (no droop), the cup's
lateral lever arm is a clean **~1.66 mm/deg** with only second-order vertical
cross-coupling, the legs keep **>45 mm of stroke margin even at 24°** (tilt is not
stroke-limited at z=170 — the z=0 leg-saturation artefact does not recur), and
tilt tracks **better dynamically than lateral translation** (~0.99 / 3° at the
1.6 Hz cycle freq vs translation's 0.95 / 17°). A modest tilt held through the
throw and re-aimed cycle-to-cycle sits well inside the band.

New committed probe: `tools/probes/juggle_tilt_bandlimit.py` (mirrors
`juggle_cup_bandlimit.py`; headless MuJoCo, side-effect-free, stdout only).

## Measurements (`tools/probes/juggle_tilt_bandlimit.py`, run 2026-06-29)

Operating point: centroid z = 170 mm (STOW-relative), slider = 180 mm, cup
(`hand_opening` site) ~840 mm world at level. Both tilt axes characterised because
the Stewart leg layout is not x/y-symmetric.

### 1. Static hold + lever arm (sweep 0/3/6/9/12°)

| axis | cmd° | achieved° | droop° | cup lateral Δ (mm) | mm/° | cup vertical Δ (mm) |
|------|------|-----------|--------|--------------------|------|---------------------|
| ry (aim +x) | 3 | 3.00 | 0.000 | +4.99 | 1.664 | −0.13 |
| ry | 6 | 6.00 | −0.000 | +9.97 | 1.661 | −0.52 |
| ry | 9 | 9.00 | −0.001 | +14.92 | 1.658 | −1.17 |
| ry | 12 | 12.00 | −0.001 | +19.83 | 1.652 | −2.08 |
| rx (aim −y) | 3 | 3.00 | −0.003 | −4.99 | −1.663 | −0.13 |
| rx | 12 | 12.00 | −0.004 | −19.82 | −1.652 | −2.09 |

- **Static-hold accuracy:** achieved = commanded to ≤0.004° across the whole
  sweep on both axes. No measurable droop under gravity (stiff position
  actuators, kp = 2e5). Tilt is a "set it and it holds" DOF.
- **Lever arm:** **~1.66 mm of lateral cup shift per degree of tilt**, essentially
  constant across the range (1.664 → 1.652 mm/° as the small-angle term softens).
  This is pure rigid-body geometry — the cup sits ~95 mm above the centroid
  (1.66 × 180/π ≈ 95 mm), and `Δlateral = h·sin θ`. Identical magnitude on both
  axes (the lever arm is the cup height, axis-independent).
- **Vertical cross-coupling:** second-order and small — `Δz = h·(cos θ − 1)`,
  i.e. −0.13 mm at 3°, −2.08 mm at 12°. Phase 1 should compensate the *lateral*
  shift in `centroid_xy`; the vertical term is small enough to fold into the
  slider offset if needed.

### 2. Leg headroom at high tilt (slide range [−5, 285] mm)

| axis | cmd° | leg[min,max] (mm) | margin_lo | margin_hi |
|------|------|-------------------|-----------|-----------|
| ry | 12 | [117.6, 193.6] | 122.6 | 91.4 |
| ry | 24 | [84.5, 233.2] | 89.5 | 51.8 |
| rx | 12 | [112.8, 179.2] | 117.8 | 105.8 |
| rx | 24 | [74.2, 204.8] | 79.2 | 80.2 |

At z = 170 the legs sit at ~154 mm with the platform level; a ry tilt spreads
them faster than rx (the asymmetric leg layout), but **even at 24° both axes keep
>45 mm of margin** to the nearest slide limit. Tilt is **not** stroke-limited at
the operating height — the z = 0 leg-saturation artefact (a leg pinned at the
−5 mm floor, logbook `2026-06-27-online-replanning-architecture-and-cup-bandlimit`)
does not recur at z = 170.

### 3. Dynamic tilt Bode (amp = 6°, both axes ~identical)

| freq (Hz) | amp_ratio | lag (°) | achieved peak rate (°/s) |
|-----------|-----------|---------|--------------------------|
| 0.8 | 0.999 | 1.6 | 30 |
| 1.0 | 0.998 | 2.0 | 38 |
| **1.6 (cycle)** | **0.994** | **3.1** | **60** |
| 2.0 | 0.991 | 3.8 | 75 |
| 3.0 | 0.977 | 5.6 | 110 |
| 5.0 | 0.949 | 8.8 | 179 |
| 8.0 | 0.887 | 12.0 | 268 |

At the **1.6 Hz juggle cycle frequency** tilt tracks at **0.994 amplitude /
3.1° lag** — markedly better than lateral *translation* at the same frequency
(0.95 / 17°, from `juggle_cup_bandlimit.py`). Tilt stays ≥0.95 out to 5 Hz and
only reaches −1 dB (0.887) at 8 Hz.

## Recommendation for Phase 1

- **Max usable tilt magnitude:** **~12° design max** (≈0.21 rad), giving
  `5 m/s × sin(12°) ≈ 1.04 m/s` lateral take-off — covering the plan's ~0.95 m/s
  target (11°) with margin. Nothing *forces* a hard cap below ~24° (legs keep
  >45 mm margin and the hold is exact), but 12° keeps the vertical cross-coupling
  ≤2 mm and the lateral lever-arm offset ≤20 mm — small, easily compensated.
- **Max usable tilt rate:** keep the tilt **rate** demand inside the ~unity band,
  i.e. tilt changes shaped at ≲2 Hz (amp_ratio ≥0.99, lag ≤4°). Operationally:
  **hold tilt ~constant through the throw and re-aim between throws** (the
  operator's instinct — a fast tilt *change* swings the cup through its lever arm
  and re-enters the band limit). A full-amplitude tilt oscillation at the 1.6 Hz
  cycle rate already tracks at 0.994, so cycle-to-cycle re-aiming is comfortably
  inside the band; there is no need to whip the tilt.
- **Lever-arm compensation (Phase 1):** offset `centroid_xy` by
  −1.66 mm × tilt° along the aim axis so the *cup* lands where the plan wants it;
  the vertical −0.0145 mm/°² term is negligible until ~12° (−2 mm) and can fold
  into the slider offset.

## Discussion

**Why tilt tracks better than translation (the load-bearing physical point).**
The level-decoupled design put the lateral throw component on platform
*translation* — moving the whole platform + hand mass sideways, against six legs
and the connect constraints, which low-pass-filters fast lateral transients
(−3 dB ~5 Hz). Tilt instead rotates the platform about its own centroid: the cup
moves laterally through a *small* lever arm (~95 mm), so a 6° tilt is only ~10 mm
of cup motion, and the rotational inertia about the centroid is far smaller than
the translational inertia being shoved sideways. The actuators barely move
differentially. That is why the tilt Bode is nearly flat where the translation
Bode has already rolled off — and it's exactly the lever the re-architecture
wants: **aim** (a held tilt, on the well-tracked DOF) is decoupled from **speed**
(the slider, already characterised near-perfect to >4 m/s).

**Why this de-risks the re-architecture.** The divergence root cause was a *loop
gain > 1*: a band-limited platform delivered the lateral throw velocity, so a
small in-cup catch offset got amplified by the throw. Tilt breaks that loop —
the throw velocity is now set by a *constant tilt × the perfect slider*, nearly
independent of small in-cup offsets, and the tilt itself is held static-exact.
Phase 0 confirms the mechanism the plan bets on is real on our morphology: the
aiming DOF is accurate and has generous headroom. (Phase 2 remains the
make-or-break: that the tilted detach actually imparts the lateral velocity
*cleanly under full contact* and kills the divergence — Phase 0 only establishes
that the platform can *present* the tilt, not yet that the ball leaves along it.)

**What was ruled out / checked, not assumed.** (a) *Droop* — a gravity-loaded
hold could have sagged; it does not (≤0.004°). (b) *Leg saturation* — the z = 0
lateral sweep's "0.68 static gain" was pure leg-pinning; we explicitly swept tilt
to 24° to confirm >45 mm margin remains at z = 170, so the artefact is height-
specific and absent here. (c) *Axis asymmetry* — characterised **both** rx and ry
because the leg layout isn't x/y-symmetric; the lever arm and Bode are identical
(geometry: lever arm = cup height), only the leg-spread rate differs (ry spreads
faster), and both keep ample margin. (d) The dynamic Bode used a *small* 6°
amplitude (≈10 mm cup), so it is not an apples-to-apples cup-displacement
comparison with translation's 80 mm sweep — but that is precisely the point: tilt
needs only small cup motion to aim, and even at the cycle rate it tracks ~unity.

## Verification

- `python tools/probes/juggle_tilt_bandlimit.py` (run 2026-06-29): produced all
  numbers above; runs headless on the Jetson venv in ~33 s, side-effect-free
  (stdout only, no files written, no hardware).
- `pytest tests/ -q` (run 2026-06-29): see the Phase 0 commit message for the
  (date, command, result) triple — Phase 0 adds only a new standalone probe +
  docs (no production code touched), so the suite is unaffected.
