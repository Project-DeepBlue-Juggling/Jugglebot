---
title: Rung-2a single-ball tilt-aimed throw — tilt-aim is exact (lateral = slider·sin θ), open-loop landing ≤33 mm across the column + 50 mm ring; a directional separation/aim asymmetry caps the clean box at ~±70 mm
type: feature
date: 2026-06-30
status: in-progress
phase: "Online-juggle tilt re-architecture — Phase 2 / Rung 2a (single-ball throw to arbitrary targets, open-loop accuracy)"
related_plan: "bb-online-juggle-tilt-rearchitecture.md"
files_changed:
  - controller/demo/juggle_planner.py
  - sim/juggle_tilt.py
  - sim/juggle_online.py
  - sim/juggle_throw.py
  - tests/sim/test_demo_juggle_planner.py
  - tests/sim/test_juggle_tilt.py
  - tests/sim/test_juggle_throw.py
  - tools/probes/juggle_throw_accuracy.py
  - tools/probes/README.md
commits:
  - a907528
subsystem:
  - sim
tags:
  - control
  - planning
  - contact
  - throw
  - characterisation
---

# Rung-2a single-ball tilt-aimed throw (open-loop accuracy)

## Summary

Phase 2 / Rung 2a of `plans/active/bb-online-juggle-tilt-rearchitecture.md`:
validate the **throw** primitive standalone — throw one ball **tilt-aimed** to a
scoped workspace target, accurately and at settable cadence, measured
**open-loop** (no catch), under the §3 tracking noise. This is where tilt enters
the *throw*: instead of putting the lateral take-off on band-limited platform
translation (the level-decoupling's wall), the cup is **tilted so its symmetry
axis points along the ballistic take-off velocity**, and the fast slider drives
the cup up that axis — so `lateral take-off = slider_speed × sin(tilt)`.

Three pieces landed: (1) the planner (`plan_cup_cycle`) grows a **tilted-axis
detach** — Kai's `cross(cup_acc − g, axis) == 0` over the detach knots,
generalising the level `cup_acc_xy == 0` so `tilt = 0` is byte-for-byte
unchanged; (2) the realisation (`juggle_online.realize`) **re-introduces
orientation**, unified onto `sim.juggle_tilt.realize_tilted` (lever-arm
compensation), byte-identical at zero tilt; (3) a single-throw harness
(`sim/juggle_throw.py`) carries the seated ball up the tilted axis, detaches it
under real contact, and measures the landing.

**Result.** The tilt-aim is **exact**: the planned take-off velocity is collinear
with the tilted cup axis, the lateral take-off equals `|v|·sin(tilt)` to machine
precision, and under full contact the cup reaches the commanded take-off velocity
to **~1 %** in every direction. Open-loop landing accuracy on the **reliable box**
— the column toss + the full 50 mm-radius ring (all 8 directions) — is **8.3 mm
(column) to ≤33 mm (50 mm ring)**, *all inside the catch's ~60-80 mm reliable
reach* (Rung 1); the §3 tracking noise adds only ~2-3 mm of scatter to the landing
the catch would observe. A genuine **directional separation/aim asymmetry**
(rooted in the cup-contact + non-y-symmetric leg layout) caps the *clean* box at
~±70 mm: beyond ~100 mm the pure +y/−y throws fail to separate ("glue") and the
−x / +x−y throws over-shoot. The column toss — what Rung 2b's self-catch actually
needs — is well inside the catch reach, so the gate question ("accurate enough to
close a loop on?") is **yes for the self-catch**; the full ±100 mm oval needs the
asymmetry resolved first (Rung 3 concern).

## Approach (control/physics walked through first)

A clean tilted throw is a chain of three things, each validated separately:

1. **Aim (planner).** The ballistic take-off velocity to hit the target,
   `v_takeoff = (target − throw_pos)/T − ½gT`, sets BOTH the speed and the
   direction. The cup is tilted so its axis points along `v_takeoff`
   (`tilt_to_throw`, the throw mirror of the catch's `tilt_to_receive`:
   `tilt_to_throw(v) == tilt_to_receive(−v)`). The detach constraint
   generalises from "no lateral cup acceleration" (flat cup, world +z) to "the
   net applied acceleration `cup_acc − g` is collinear with the tilted axis"
   (`cross(cup_acc − g, axis) == 0`) — physically, the cup-contact force on the
   ball is purely **axial**, so the ball leaves up the axis with no sideways
   shove. At `axis = [0,0,1]` the cross reduces *exactly* to `cup_acc_xy == 0`,
   so the level plan is unchanged (kept as the literal original branch — the
   `tilt=0` regression guarantee, asserted byte-for-byte).

2. **Realisation (lever arm).** The cup opening must end up where the plan wants
   it under tilt; `realize_tilted` offsets the centroid by the Rung-0 height-aware
   lever arm. Crucially, for a FIXED tilt a pure cup motion *along the axis* moves
   the centroid only `ds·sinθ·(1−cosθ)` (sub-mm over a 150 mm stroke) — so the
   lateral take-off is delivered by the **slider through the tilt**, NOT by
   platform translation. This is the whole point of the re-architecture, and it
   holds: `juggle_online.realize` now delegates to `realize_tilted`, unifying the
   two (the duplication the tilt module flagged for Rung 2a).

3. **Detach under contact (the hard part).** Carry the seated ball up the tilted
   axis on a `plan_cup_cycle` trajectory (tilt ramped in during the carry, held
   constant through the release per Rung 0), release by physics
   (`begin_physics_throw` + stiff contact), and re-plan the recovery from the
   achieved cup state with the tilted detach. The ball detaches when the cup
   decelerates past gravity along the contact axis.

## What was explored / ruled out (the separation is the binding constraint)

The aim (step 1-2) was correct on the first try — verified numerically and
confirmed under contact (cup reaches `v_takeoff` to ~1 % in **all** directions).
The binding constraint is the clean **separation** (step 3): whether the ball
leaves the cup cleanly, and with the planned velocity, before the post-throw cup
motion re-contacts it. A sequence of open-loop-throw realisations was probed
(`/tmp/probe_*` — one-offs, not committed):

- **Zero-order-hold position commands → stop-and-go (withdrawn immediately).** A
  per-tick position setpoint that jumps each tick makes the cup *settle to rest at
  each setpoint within the tick*, so the cup has ~0 instantaneous velocity at the
  tick boundary even while averaging 1 m/s — the ball detaches at ~0 velocity and
  falls. **Fix:** sample the trajectory at the **sub-tick** time in the plant's
  `hand_cmd_fn`/`plat_cmd_fn` (what `juggle_online` already does) so the setpoint
  moves continuously and the cup acquires real velocity. With this the cup
  reaches `v_takeoff` cleanly.

- **Pure-slider axial retract recovery (worse).** Driving the cup straight back
  down its own axis (slider only) after release gave a *perfect* take-off velocity
  but separated only the +x-ish directions; an aggressive retract glued even +x.
  The contact separation is acutely sensitive to the exact recovery shape.

- **Explicit "pull-away" down −axis (worse).** Forcing the cup down the axis
  shoved the still-seated ball, adding spurious lateral velocity (−x overshot to
  −217 mm). Ruled out.

- **Descend-and-laterally-away recovery (worst).** Moving the cup laterally away
  carried the seated ball with it — *everything* glued except the column. Ruled
  out.

- **`plan_cup_cycle` carry + `plan_cup_cycle` recovery (the juggle_online-faithful
  path — kept).** Re-planning the recovery as a natural carry oval (the proven
  runner's own machinery) gave the best and most consistent separation: the
  column + the favorable directions separate cleanly. This is the production
  harness.

So the **separation under this contact model is a genuine knife-edge** — the same
*class* of finding as Rung 1's reach-limited catch: the tilt-aim mechanism is
sound and exact, but the contact realisation has a narrow operating envelope that
this rung's job is to **characterise honestly**, not paper over.

## Characterisation (`tools/probes/juggle_throw_accuracy.py`, run 2026-06-30)

Open-loop landing error vs target, flight 0.60 s, §3 tracking noise on (σ 0.5 mm),
2 seeds/target. (true error = noise-free landing; the catch reaches for the noisy
"observed" landing, within ~2-3 mm of true.)

| target (mm) | tilt° | sep | landing err (mm) | reach (mm) |
|---|---|---|---|---|
| (0, 0) column | 0.0 | 2/2 | **8.3** | 8 |
| (+50, 0) | 1.8 | 2/2 | 14.0 | 51 |
| (+35,+35) | 1.8 | 2/2 | 22.4 | 72 |
| (0,+50) | 1.8 | 2/2 | 18.3 | 32 |
| (−35,+35) | 1.8 | 2/2 | 29.8 | 43 |
| (−50, 0) | 1.8 | 2/2 | 32.4 | 68 |
| (−35,−35) | 1.8 | 2/2 | 4.9 | 51 |
| (0,−50) | 1.8 | 2/2 | 23.6 | 27 |
| (+35,−35) | 1.8 | 2/2 | 9.5 | 56 |
| (+100, 0) | 3.5 | 2/2 | 21.5 | 79 |
| (−71,+71) | 3.5 | 2/2 | 18.6 | 84 |
| (+71,+71) | 3.5 | 2/2 | 69.3 | 120 |
| (−71,−71) | 3.5 | 2/2 | 49.8 | 150 |
| (+71,−71) | 3.5 | 2/2 | **125.0** | 220 |
| (−100, 0) | 3.5 | 2/2 | **134.2** | 227 |
| (0,±100) | 3.5 | **0/2** | — (glued) | — |

**The reliable box** (column + the 50 mm ring, all 8 directions): separates
10/10, lands **4.9-32.4 mm**, every direction **inside the catch's ~60-80 mm
reliable reach**. Adding +x and −x+y out to 100 mm stays good (21.5 / 18.6 mm).
**The asymmetry** appears at ~100 mm: pure ±y **fails to separate**, and −x /
+x−y **over-shoot** (~125-134 mm). Cadence sweep (column): flight 0.55-0.85 s all
separate; the sweet spot is ~0.55-0.60 s (8-21 mm), faster (0.45 s) degrades
(38 mm — the cup under-runs `v_takeoff` at low speed). Across the whole sweep
(34 targets): 30/34 separate, and **87 % of separated throws land within the
catch reach**; the reliable box is 10/10.

## Discussion

**Why tilt-aim, and why it works (the load-bearing point).** The level design put
the lateral throw component on platform *translation* (−3 dB ~5 Hz); a fast
lateral transient is exactly what that band-limit kills, which is why a ~15 mm
catch offset got amplified past the reach (the 2-ball divergence). Tilt moves the
lateral component onto the **slider** (perfect to >4 m/s) projected through a
**roughly-constant tilt** (Rung 0: 0.994/3° at the cycle freq). Rung 2a confirms
this is real under contact, not just kinematically: the planned take-off is
collinear with the tilted axis by construction, and the *achieved* cup velocity
matches `v_takeoff` to ~1 % in every direction — i.e. the aim is decoupled from
the slow DOF, exactly as the re-architecture bet.

**The throw scatter vs the catch reach (the cross-rung gate).** Rung 1's
knife-edge investigation established the catch is **reach-limited** (reliable to
~60-80 mm reach; the BB's 2 % noise scatters the landing to ~60 mm and that's the
~85 % case), and — critically — "the throw's landing scatter *is* the reach the
catch faces in the loop." Rung 2a measures that scatter directly. For the
**column toss** (target = throw xy, tilt ≈ 0 — what Rung 2b's self-catch is): the
ball lands 8 mm from the target, so the self-catch faces an **8 mm reach** — deep
inside the reliable region, and far below the ~15 mm the throw amplification cared
about. The §3 tracking noise adds only ~2 mm (the `BallisticEstimator` averages it
down). So the self-catch loop is fed an *accurate, tight* throw — the precondition
Rung 1 said makes the loop sustainable. For modest lateral patterns (≤50 mm ring),
all directions stay within the catch reach. **This is the answer to the gate: the
throw is accurate enough to close the Rung-2b self-catch loop.**

**The directional asymmetry — a real finding, not a harness bug.** Beyond ~100 mm,
the throw's clean envelope is direction-dependent: pure ±y glues, −x / +x−y
over-shoots. The aim (cup velocity) is symmetric and exact; the **separation/
detach under contact** is what breaks. Two morphology facts drive it: (a) the cup
collision mesh + the non-y-symmetric Stewart leg layout (Rung 0 already noted ry
spreads the legs faster than rx; here the *sign* of the tilt about x matters too),
and (b) the soft→stiff contact at release plus the post-throw cup path — a
free-falling or laterally-moving cup re-contacts the slow-separating ball. This is
the *same class* of result as Rung 1's reach-limit: the primitive's mechanism is
sound, but the contact realisation has a characterised operating envelope. It is
**load-bearing for Rung 3** (the full ±100 mm oval), where it must be resolved —
the lever is the recovery/release shape and the seat-centring through the throw,
explicitly NOT the aim (which is already exact). It is **not** load-bearing for
Rung 2b's column self-catch.

**What was deliberately NOT done.** No contact-model change (the plan mandates
keeping contact fidelity), no kinematic/controlled-velocity throw shortcut (also
mandated), and the asymmetry is not "tuned away" with a per-direction fudge — it
is reported as the real envelope so the operator's gate decision and Rung 3 see
the true picture. The §3 *BB-throw* noise does not apply to a cup throw (the cup,
not the Ball Butler, is the thrower); only the *tracking* noise applies, on the
post-throw landing observation, which is where it would enter the loop.

## Verification

- New planner unit tests (`tests/sim/test_demo_juggle_planner.py`): tilted-axis
  detach collinear with the axis (`cross(cup_acc − g, axis) == 0`), `detach_axis`
  None vs `[0,0,1]` byte-for-byte identical (the `tilt=0` guarantee), tilt-aim
  numeric (`v_takeoff` ∥ axis, lateral = `|v|·sinθ`), throw boundary still exact.
- New tilt-geometry tests (`tests/sim/test_juggle_tilt.py`): `tilt_to_throw`
  (parallel-to-take-off, mirror of `tilt_to_receive`, direction signs, clamp);
  `juggle_online.realize` level byte-identical + delegates to `realize_tilted`.
- New integration tests (`tests/sim/test_juggle_throw.py`): column toss separates
  + lands <15 mm; 100 mm lateral separates + within the catch reach; tilt-aim
  under contact (lateral = `speed·sinθ`, cup reaches `v_takeoff` to 5 %); the
  reliable small-lateral box lands within reach; the true error is deterministic
  and §3 noise adds <12 mm scatter; noise-off matches the true landing.
- Characterisation harness `tools/probes/juggle_throw_accuracy.py` (committed):
  the sweep above.
- Full suite: see Outcome.

## Outcome

Rung-2a subset (`pytest tests/sim/test_demo_juggle_planner.py
tests/sim/test_juggle_tilt.py tests/sim/test_juggle_throw.py -q`, run 2026-06-30):
**41 passed in 24.38 s**. Full suite (`pytest tests/ -q`, run 2026-06-30):
**1559 passed, 4 skipped, 2 xfailed in 764.35 s** (+16 new Rung-2a tests over the
1543 baseline; no regressions, no new skips/xfails; only pre-existing warnings).

Commit SHA: a907528 (this rung's feature commit).
