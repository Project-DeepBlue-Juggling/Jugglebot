---
title: Rung-2b single-ball self-catch — MAKE-OR-BREAK BREAK; the pure column diverges (loop gain > 1) for column-specific reasons, and the column is a degenerate case where tilt is inactive
type: feature
date: 2026-07-01
status: in-progress
phase: "Online-juggle tilt re-architecture — Phase 3 / Rung 2b (throw-and-self-catch loop, MAKE-OR-BREAK gate)"
related_plan: "bb-online-juggle-tilt-rearchitecture.md"
files_changed:
  - sim/juggle_selfcatch.py
  - tools/probes/juggle_selfcatch_loopgain.py
  - tools/probes/README.md
  - tests/sim/test_juggle_selfcatch.py
  - plans/active/bb-online-juggle-tilt-rearchitecture.md
commits:
  - f745f29
subsystem:
  - sim
tags:
  - control
  - planning
  - contact
  - throw
  - catch
  - make-or-break
---

# Rung-2b single-ball self-catch — the make-or-break gate

## Summary

Phase 3 / Rung 2b of `plans/active/bb-online-juggle-tilt-rearchitecture.md`: the
**make-or-break** rung. Compose the Rung-2a tilt-aimed **throw** with the Rung-1
**catch** into the minimal closed loop — one ball, thrown up and caught by the
**same** cup, cycle after cycle, re-planned each cycle from the achieved cup state
+ the noisy observed ball, under the §3 tracking noise. The gate: does tilt make
the throw→catch→throw loop **stable** (loop gain < 1, ≥ 10 sustained cycles), or
does the throw still amplify (loop gain > 1, the pre-tilt divergence)?

**Verdict: BREAK.** The pure **column** single-ball self-catch does **not**
sustain. Across all seeds and all three recover/geometry variants it diverges
within **0–3 cycles**: the catch reach amplifies each cycle (~8 → ~50 → ~110 →
~210 mm, `tools/probes/juggle_selfcatch_loopgain.py`, run 2026-07-01) past the
catch's ~60–80 mm reliable reach → drop. Loop gain > 1, head-to-head with the
pre-tilt divergence (which also had loop gain > 1).

**But the load-bearing finding is *why*, and it reframes the gate:** the column
diverges for **column-specific** reasons that tilt was never designed to address,
and — critically — **for a column the commanded tilt is ~0, so the tilt mechanism
(decoupling lateral aim from the band-limited platform) never *engages*.** The
column is a degenerate case where tilt is inactive, so it does not test the tilt
hypothesis. This is a **plan-framing problem**, surfaced for the operator: the
minimal stationary self-catch is necessarily a column (a lateral throw walks), and
the column is exactly where tilt does nothing. The right make-or-break test needs
a configuration where tilt engages — but that is a travelling / two-ball pattern
(Rung 3). See the Discussion for the recommended re-plan.

## What was built

`sim/juggle_selfcatch.py` — `run_self_catch(SelfCatchConfig)`, the single-ball
self-catch loop that composes the proven primitives on one persistent plant
carrying one ball through N throw→catch cycles:

- **carry** the seated ball up the (near-vertical, tilt≈0) column axis on the
  validated Rung-2a `plan_cup_cycle` carry half (tilt ramped in);
- **release** under stiff contact + **recover** with the tilted-axis detach
  (`plan_cup_cycle` recover — the Rung-2a-faithful path; an ad-hoc axial-retract
  recover is also exposed as `recover="retract"`);
- **catch** the in-flight ball with the Rung-1 machinery (observe → translate-to-
  reach → tilt-to-receive → constant-decel seat), re-planned from the noisy
  observation; and
- between cycles, for the **stationary** column, smoothly reposition the held ball
  back to a fixed origin (a `drift` variant throws in-place from the caught xy,
  isolating the reposition's contribution).

Per-cycle metrics (`SelfCatchCycle`): separated / caught / held, in-cup offset,
**reach**, seat offset, landing. `SelfCatchResult.sustained` = consecutive
caught+held cycles from the start; `.reach_trend_mm`; `.diverged`.

## The measurements (`tools/probes/juggle_selfcatch_loopgain.py`, run 2026-07-01)

Sustained cycle count (gate needs ≥ 10) and the per-cycle **reach trend** (mm),
6 seeds × 12 cycles, §3 tracking noise on:

| variant | sustained (seeds 0–5) | representative reach trend |
|---|---|---|
| faithful (plan recover, stationary column) | 1,1,1,2,2,3 | seed 3: **7 → 107 → 130** |
| drift (plan recover, throw in-place) | 0,0,0,0,0,0 | seed 0: 12 (drops at the seat) |
| retract (ad-hoc axial retract, stationary) | 2,1,2,1,… | seed 2: **19 → 57 → 156** |

**No configuration sustains anywhere near 10 cycles.** The faithful composition's
first throw is tight (~8 mm reach, matching Rung-2a's column) and is cleanly
caught — the primitives compose for **one** cycle — but the reach then amplifies
several-fold per cycle and blows past the catch's reliable reach within 1–3
cycles.

## Root causes (two compounding, column-specific failure modes)

Walking one throw→catch→throw cycle through the contact physics (per CLAUDE.md's
control-rigor rule) surfaced two distinct problems, neither of which is the
pre-tilt band-limit cascade:

### 1. The column separation singularity (a caught ball does not cleanly detach)

The detach constraint is **free-fall**: `cup_acc == g` at release (Kai's clean
detach). Under free-fall the ball and the cup share the **same** acceleration. For
a **column** the ball and cup also sit on the **same vertical line**, so after
release they ride up **together** — they do not separate. Traced directly
(`contacts_hand == True` through the entire recover for a caught ball; the ball
tracks the cup to within ~5 mm and gets carried back down instead of flying free).

Rung-2a's column throw *did* separate — but only because a freshly **spawned**
ball (`spawn_in_hand`) carried incidental spin (`[-1.7, -0.5, -0.3]` rad/s from
the carry) that broke the contact. A Rung-1-**caught** ball arrives straight down
a column and seats **spinless** (`~[0,0,0]`), so it glues. **The primitives do not
compose on a column**: Rung-2a's separation relied on a property (spin) that
Rung-1's catch does not supply.

Forcing separation with an **axial slider retract** (decelerate the cup faster
than g so it drops away from the ball) works, but the hard deceleration + the cup
mesh / non-y-symmetric leg asymmetry (the same morphology asymmetry Rung-2a flagged
at ±100 mm) imparts a **lateral velocity kick** to the exiting ball — a knife-edge
that feeds mode 2.

### 2. Loop gain > 1 via the reach (the throw amplifies the cup's lateral state)

When the ball *does* separate, the in-cup **position** offset that the catch
achieves stays small (~0.2–4 mm — the catch centres it well). The amplified
quantity is not the in-cup offset (contra the plan's framing) but the cup's
**lateral state / the landing reach**: the column throw's separation is
**chaotically sensitive** to the ball's residual lateral velocity at release, so
the small lateral motion introduced by the catch reach and the stationary
reposition is amplified into a large landing offset → the catch must reach further
→ the reposition is larger → more residual lateral motion → the reach amplifies
(measured ~5–6× cycle-over-cycle from the tight first throw). Loop gain > 1.

The `drift` variant (no reposition-to-origin) removes that amplification path and
**still** fails — at cycle 0, at the seat (the ball escapes a ~35 mm-transient
seat). So the column catch-seat is *itself* a knife-edge; the divergence is
**column-intrinsic**, not only a reposition artefact.

## Discussion

**Why this is a BREAK, and why the *reason* matters more than the verdict.** The
plan's Rung-2b hypothesis is that tilt makes the loop stable because tilt moves the
lateral throw component off the band-limited platform (−3 dB ~5 Hz) onto the fast
slider projected through a roughly-constant tilt. That mechanism is real and was
validated open-loop in Rung 2a. **But it only helps when there is a lateral throw
demand for tilt to carry.** A single-ball *stationary* self-catch is necessarily a
**column** (a ball thrown with lateral velocity lands laterally and never returns
to its start — only a straight-up throw comes back), and a column has **zero**
lateral demand, so the commanded tilt is ~0 and the tilt mechanism is **inactive**.
The column therefore does not — cannot — test "did tilt kill the divergence." It
diverges for reasons (the free-fall detach singularity on the vertical line; the
chaotic column separation) that tilt was never meant to touch.

**The plan-framing problem, stated plainly (this is load-bearing for the operator).**
Rung 2b was chosen as the make-or-break gate on the premise that the column is the
*easiest* case (lowest lateral demand → sustains → then ramp to the oval). The
physics inverts that premise: the column is not the easiest case, it is a
**degenerate/singular** case — the one geometry where (a) the free-fall detach
cannot separate a centred ball and (b) tilt is inactive. The pre-tilt divergence
the whole arc is trying to beat was a **lateral** (two-ball) throw where the
band-limit bit; the column self-catch is a different animal. So the gate as
written does not discriminate the hypothesis: a column BREAK does **not** imply
tilt failed, and a hypothetical column MAKE would **not** have implied tilt
succeeded.

**Recommended re-plan (surfaced, not decided).** Two options for the operator:

1. **Test the tilt hypothesis where tilt engages.** Replace the degenerate column
   self-catch with a **two-point single-ball oscillation** (throw from A up-and-
   over to B, catch at B, throw back to A — the cup shuttles A↔B). This keeps the
   single-ball simplicity but gives a real lateral demand, so tilt engages and the
   cup's lateral recover motion breaks the column separation singularity (Rung-2a's
   favourable lateral directions separated cleanly, 2/2). This is a *faithful*
   make-or-break test of the tilt hypothesis. It walks/shuttles rather than being
   spatially fixed, but it is still a single-ball self-catch.

2. **Skip Rung 2b as a distinct gate.** If the two-ball oval (Rung 3) is where tilt
   must prove itself anyway, the column self-catch may not be a meaningful
   intermediate gate. The operator could fold the tilt make-or-break into a
   *minimal lateral* two-ball columns test.

Either way, the harness + probe here are the reusable apparatus: `SelfCatchConfig`
already parameterises the throw/catch geometry, the recover strategy, and
stationary-vs-drift, so a two-point oscillation is a small extension.

**What was explored and ruled out (so the next session does not re-tread it).**
- *Quintic carry from the achieved seat* (instead of the `plan_cup_cycle` carry):
  pre-detaches the ball at low velocity (the intermediate deceleration exceeds g),
  so the ball falls out. The validated `plan_cup_cycle` carry is required — kept.
- *Longer settle-hold* before the carry (to damp the held ball's lateral velocity
  and break the reach→velocity path): marginal; the loop still diverged. The
  residual is not cleanly a settle-able velocity.
- *Axial retract recover* (to force separation of a centred caught ball): separates
  but imparts a chaotic lateral kick that amplifies — a knife-edge, exactly the
  class Rung-2a warned about ("aggressive retract glued even +x"). Exposed as a
  non-default `recover` option, not the primary.
- *No-reach catch* (cup fixed at origin, seat off-centre by the landing): fails
  immediately — the Rung-1 catch needs the reach even for a ~7 mm offset
  (consistent with Rung 1). So the reach is not optional, and with the reach the
  reposition amplification bites.
- *qpos-injected in-cup offsets* to measure the throw's loop gain directly: the
  cup **rolls the ball back to centre** during the carry (concave cup), and the
  landing came out chaotic (0–120 mm for injected offsets that all recentred) —
  confirming the column separation is a contact knife-edge, but the teleport
  injection is itself artefact-prone, so the *faithful full loop* is the sound
  measurement (and it diverges).

**On not tuning it away.** The gate explicitly forbids papering over a diverging
loop or tuning endlessly. Three independent mechanizations (faithful plan recover,
axial retract, in-place drift) all diverge, with a clear physical root cause, so
the honest move is to STOP and surface — not to keep hunting for a recover shape
that happens to sustain a column, which would be tuning a degenerate case that
does not test the hypothesis anyway.

## Verification

- New harness `sim/juggle_selfcatch.py` (`run_self_catch`) composes the Rung-2a
  throw + Rung-1 catch; deterministic per seed (the §3 tracking noise is the only
  RNG).
- New probe `tools/probes/juggle_selfcatch_loopgain.py` (committed): the sweep
  above; writes `temp/probes/juggle_selfcatch_loopgain.csv`.
- New tests `tests/sim/test_juggle_selfcatch.py`: the primitives compose for one
  cycle (first throw separates + caught + held, reach < 25 mm within the catch
  reach); the headline capability (`sustains ≥ 10`) is `xfail(strict=True)` (the
  BREAK); the divergence is characterised across 6 seeds (`sustained < 5`,
  `diverged`); the reach-amplification loop-gain signature; the drift variant also
  fails; seed-determinism.
- Full suite: see Outcome.

## Outcome

Rung-2b subset (`pytest tests/sim/test_juggle_selfcatch.py -q`, run 2026-07-01):
**10 passed, 1 xfailed in 45.34 s**. Full suite (`pytest tests/ -q`, run
2026-07-01): **1569 passed, 4 skipped, 3 xfailed in 806.34 s** (+10 passed +1
xfailed over the 1559 baseline for the new Rung-2b tests; no regressions, no new
skips; only pre-existing warnings). The flaky `test_hot_loop_allocation_contract`
passed in-suite.

**Gate: BREAK — STOP and re-plan with the operator.** The pure column single-ball
self-catch diverges (loop gain > 1); the column is a degenerate case where tilt is
inactive, so it does not test the tilt hypothesis. Recommend re-plan option 1 (a
two-point single-ball oscillation where tilt engages) before proceeding to Rung 3.

Commit SHA: f745f29 (this rung's feature commit).
