---
title: Rung-2b OPTION-1 re-plan — the two-point A↔B oscillation is the HONEST tilt test (tilt ENGAGED), and it STILL diverges; loop gain > 1 via the throw's chaotic sensitivity to the throw-ORIGIN pose — a GENUINE tilt-hypothesis BREAK on a non-degenerate geometry
type: feature
date: 2026-07-01
status: in-progress
phase: "Online-juggle tilt re-architecture — Phase 3 / Rung 2b (throw-and-self-catch loop, MAKE-OR-BREAK gate, OPTION-1 re-plan)"
related_plan: "bb-online-juggle-tilt-rearchitecture.md"
files_changed:
  - sim/juggle_selfcatch.py
  - tools/probes/juggle_selfcatch_loopgain.py
  - tools/probes/README.md
  - tests/sim/test_juggle_selfcatch.py
  - plans/active/bb-online-juggle-tilt-rearchitecture.md
commits:
  - 6308689
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

# Rung-2b OPTION-1 re-plan — the two-point A↔B oscillation (tilt engaged) STILL diverges

## Summary

Phase 3 / Rung 2b of `plans/active/bb-online-juggle-tilt-rearchitecture.md`, the
**make-or-break** gate. The prior Rung-2b run (`f745f29`) was a pure single-ball
**column** self-catch; it diverged, but the load-bearing finding was that a column
commands **~0 tilt**, so the tilt mechanism never *engages* — the column is a
**degenerate** test of the tilt hypothesis (logbook
`2026-07-01-rung2b-selfcatch-column-divergence.md`). The operator chose **OPTION 1**:
re-plan Rung 2b as a **two-point single-ball A↔B oscillation** (throw A→B, catch at
B, throw B→A, catch at A, … — the cup shuttles A↔B) so every throw is **lateral**
and the commanded tilt is **non-zero** — i.e. tilt actually does the lateral-aim
work. This is the **honest** make-or-break test of the tilt hypothesis on a
**non-degenerate** geometry.

**Verdict: BREAK — the oscillation does not sustain either, WITH tilt engaged.**
Across every separation (20–70 mm) and axis (x / y / diagonal) the A↔B loop
diverges within **1–4 cycles**; no configuration sustains anywhere near the gate's
10 (max sustained **4** of ≥ 10, at x-20 — and even there the landing is already
amplifying). The commanded tilt is genuinely non-zero (default x-40: **1.42°**, vs the
column's exactly **0.0°**), so this is **not** a degenerate-case artefact — it is a
genuine failure of the tilt hypothesis: *tilt engaging is not sufficient to make
the throw→catch→throw loop stable.*

**The load-bearing distinction from the column, and the root cause.** The in-cup
**seat offset stays small** (~0.5–2 mm — tilt *does* keep the ball centred in the
cup), but the **landing amplifies** (default x-40 seed 1: landing error
**3.7 → 89 → 728 mm** in three cycles). The amplified quantity is the **landing**,
not the seat. Root cause (measured, deterministic): the tilt-aimed throw's landing
is **chaotically sensitive to the throw-ORIGIN pose** — a 10 mm shift of the throw
point swings the landing **~40 mm** (`dLanding/dOrigin` ≈ 4, and up to ~11 with
*sign changes* at a real 1.8° tilt). This is the **contact-detach knife-edge**
Rung 2a flagged (the non-y-symmetric Stewart leg layout + the soft→stiff release),
which Rung 2a only characterised **from the origin** and deferred; the oscillation
throws from **off-origin** points A/B, where the pose-chaos is **loop-fatal**.
**Tilt fixes the band-limit — the pre-tilt divergence's mechanism — but NOT this
pose-chaos, which is the binding amplification off-origin. So the tilt
re-architecture, by itself, does not close the loop.**

## What was built

`sim/juggle_selfcatch.py` — the existing self-catch apparatus grows an
`oscillate=True` mode (the column path is **behaviorally unchanged** — its throw/
target computation is identical, re-indented under a new `else:` when the
`oscillate` branch was added, and its tests still pass unchanged):

- `SelfCatchConfig` gains `oscillate` + `osc_point_a_m` / `osc_point_b_m` (default
  A=(−20, 0) mm ↔ B=(+20, 0) mm — 40 mm on x, tilt ~1.4°). The A↔B axis/separation
  live inside what Rung 2a called its reliable box *from the origin*; part of this
  entry's finding is that the box does **not** transfer to off-origin throws.
- `run()` alternates the throw **target** A↔B by cycle parity (cyc 0 → B, cyc 1 →
  A, …) and re-plans the throw **origin** from the **achieved** caught xy (the
  closed-loop self-correction — exactly as the column loop does), except cyc 0
  (freshly seeded at A). The validated `plan_cup_cycle` carry + tilted-axis recover
  and the Rung-1 catch machinery are reused unchanged.
- `SelfCatchCycle` gains `target_xy_mm`, `landing_err_mm`, `tilt_deg`;
  `SelfCatchResult` gains `in_off_trend_mm` + `landing_err_trend_mm` — the
  oscillation loop-gain trends (the column's `reach_trend_mm` is not the right
  signal for a two-point pattern, where the landing sits near B, not the origin).

## The measurements (`tools/probes/juggle_selfcatch_loopgain.py`, run 2026-07-01)

Oscillation sustained-cycle count (gate needs ≥ 10) + the per-cycle in-cup-offset
and landing-error trends, 6 seeds × 12 cycles, §3 tracking noise on. The committed
probe sweeps the x axis across the **full 20–70 mm range** plus y-40 and diag-50 in
one run, so the "every separation (20–70 mm) and axis (x/y/diagonal)" verdict is
reproducible from the committed artifact alone:

| geometry | tilt° | sustained, seeds 0–5 (max) | representative trend |
|---|---|---|---|
| x-20 (A=−10↔B=+10) | 0.71 | 1,1,2,1,1,4 (**4**) | seed 5 landing_err **15 → 15 → 13 → 86 → 694 mm**; seed 2 → 1074 |
| x-30 (A=−15↔B=+15) | 1.06 | 2,2,1,1,2,1 (**2**) | seed 1 landing_err **16.7 → 108 → 1136 mm** |
| x-40 (A=−20↔B=+20) | 1.42 | 2,2,1,1,1,0 (**2**) | seed 1 landing_err **3.7 → 89 → 728 mm** |
| x-50 (A=−25↔B=+25) | 1.77 | 0,0,0,0,0,0 (**0**) | cyc 0 lands 43.9 mm off B (off-origin asymmetry), seat fails |
| x-60 (A=−30↔B=+30) | 2.13 | 1,1,1,1,1,2 (**2**) | cyc 0 lands 22.8 mm off; seed 5 landing_err → 103 → 623 |
| x-70 (A=−35↔B=+35) | 2.48 | 0,0,0,0,0,0 (**0**) | cyc 0 lands 11.7 mm off but the catch does not hold the seat |
| y-40 (A=0,−20↔0,+20) | 1.42 | 0,0,0,1,0,0 (**1**) | cyc 0 lands 28.9 mm off |
| diag-50 | 1.81 | 1,0,0,0,1,1 (**1**) | seed 0 landing_err **20.7 → 366 mm**; in_off 2.5 → 712 |

**No configuration sustains** (max **4** of the required ≥ 10, at x-20 — and even
there the landing is amplifying: 15 → 86 → 694 mm, so it is diverging, just one
cycle slower to drop). The default **x-40** composes cyc 0 cleanly (lands **3.7 mm**
from B, deep inside the catch reach — the primitives *do* compose the first lateral
cycle with tilt on) for 5/6 seeds, then diverges by cyc 2–3. The cyc-0 landing error
is **non-monotonic in separation** — 43.9 mm at x-50, 22.8 at x-60, 11.7 at x-70 —
itself a signature of the **sign-changing** off-origin asymmetry (it is *not* simply
"worse with size"); but no separation holds past a couple of cycles (x-50/x-70 fail
the seat at cyc 0, x-60 diverges by cyc 2).

**The pose-sensitivity (the loop-gain root cause), deterministic, noise off:**

| throw origin | aim | landing (mm) |
|---|---|---|
| (0, 0) | (0, 0) | (−2.5, −7.9) |
| (−10, 0) | (0, 0) | (−2.3, **+32.9**) |

A 10 mm shift of the throw origin (same target) swings the landing **40.8 mm** — a
`dLanding/dOrigin` of ~4. At a real 1.8° tilt (throw ~50 mm toward B, origin swept
±10 mm about A) the landing jumps **non-monotonically** with *sign changes*
(gain up to ~11). The throw map is a contact-detach knife-edge in the cup pose.

## Root cause — the throw's chaotic sensitivity to the throw-origin pose

Walking one A→B→A cycle through the contact physics (per CLAUDE.md's control-rigor
rule) isolates the amplification to the **throw map**, not the catch seat:

1. **The catch seats the ball tight (~0.3–1 mm) when it holds.** Tilt-to-receive +
   the constant-decel seat centre the ball — the in-cup offset does **not** grow.
   Whatever tilt was supposed to fix about *seating*, it fixes.
2. **The throw then flings a well-centred ball off-target.** A ball centred to
   ~0.5 mm, thrown from an off-origin point, lands 15–90 mm from the nominal
   target — because the **detach** imparts a lateral-velocity kick that depends
   chaotically on the exact platform pose (position + tilt sign). The kick is a
   deterministic function of the pose (MuJoCo is deterministic — the same origin
   gives the same landing to machine precision), but a *steep, sign-changing*
   function, so it behaves like chaos under the loop's few-mm cup-position drift.
3. **Closed-loop, this is loop gain > 1.** The catch lands the cup at the (off)
   landing; the next throw is from that drifted origin; the drifted origin lands
   even further off (gain > 1); the reach the catch faces grows past ~60–80 mm →
   drop. Divergence in 1–4 cycles, matching the observed sustained counts.

This is the **same class** as Rung-2a's "the separation under this contact model is
a genuine knife-edge" and the column BREAK's mode-2 ("chaotically sensitive to the
ball's residual lateral state"). The oscillation shows the knife-edge is a function
of the **throw-origin pose** and is **loop-fatal off-origin** — the regime Rung 2a
never characterised (it threw only from the origin).

## Discussion

**Why the oscillation is the honest tilt test (and the column was not).** The tilt
hypothesis (plan §1) is that tilt makes the loop stable because it moves the
lateral throw component off the **band-limited platform translation** (−3 dB ~5 Hz)
onto the fast slider projected through a roughly-constant tilt. That mechanism only
*engages* when there is a lateral throw demand. The column has **zero** lateral
demand → tilt ~0 → the mechanism is inactive → the column cannot test the
hypothesis (its BREAK was degenerate). The A↔B oscillation supplies a real lateral
demand each cycle (throw A→B is lateral by construction), so the commanded tilt is
non-zero (1.4° at 40 mm) and tilt genuinely does the aim work: `lateral take-off =
slider_speed × sin(tilt)`, delivered by the perfectly-tracked slider, **not** by
platform translation. Confirmed numerically: `cycles[0].tilt_deg` = 1.42° for the
oscillation vs **0.0°** for the column. This is a faithful, non-degenerate test.

**Head-to-head — all three diverge (loop gain > 1), and *why* differs:**

| | mechanism of divergence | tilt state | amplified quantity |
|---|---|---|---|
| pre-tilt (2026-06-27) | **band-limited** level throw can't aim a fast lateral transient; ~15 mm catch offset amplifies | tilt absent (level) | landing → >150 mm/cycle (walked ~+319/cyc) |
| column (2026-07-01) | free-fall detach glues a spinless caught ball on the vertical line; chaotic separation | tilt ~0 (degenerate) | reach 8 → 210 mm |
| **oscillation (this)** | throw's **chaotic sensitivity to the throw-origin pose** (contact-detach knife-edge) | **tilt ~1.4° (engaged)** | landing 3.7 → 728 mm (seat offset stays ~0.5 mm) |

The head-to-head is the point. Tilt was introduced specifically to beat the
**pre-tilt** mechanism (the band-limit), and it *does*: the aim is now delivered by
the slider, exact to ~1 % (Rung 2a). But the loop **still** diverges, because a
**different** amplification — the contact-detach pose-chaos — dominates once the
throws happen off-origin. Tilt is a band-limit fix; it does nothing about the
detach knife-edge. So the honest reading is: **the tilt re-architecture is
necessary but not sufficient to close the throw→catch→throw loop. The binding
constraint has moved from the band-limit (solved) to the contact-detach's
pose-sensitivity (unsolved).**

**Why this A↔B direction/separation (and why it doesn't rescue the verdict).** The
default is the **x-axis at 40 mm** because (a) it gives the *cleanest* first throw of
the sweep (lands 3.7 mm from B — the cyc-0 landing error is non-monotonic in
separation, 43.9/22.8/11.7 mm at 50/60/70, so no larger separation composes as
tightly, and none sustains), and (b) it clearly engages tilt (1.4°). It gives the
oscillation the *fairest* shot: a clean start, then observe whether the loop holds.
It does not — and neither does any other axis/separation swept (x, y, diagonal;
20–70 mm). The x-axis carries the *worst* cross-axis contamination (a +x
throw picks up large −/+y), the y-axis the largest along-axis overshoot; both give
loop gain > 1. There is no favourable direction because the pose-chaos is
sign-changing in **every** direction I measured.

**The plan-framing correction (load-bearing for the operator).** OPTION 1's premise
was that the oscillation "stays inside the Rung-2a reliable box (column + 50 mm
ring, ≤ 33 mm), so it tests tilt, not the deferred ±100 mm asymmetry." That premise
is **partly wrong**: Rung 2a's reliable box is a **from-origin** property (it threw
only from (0,0)); the oscillation necessarily throws from **off-origin** points
A/B, and the throw asymmetry is **origin-dependent**, not just displacement-
dependent — so the "reliable box" does **not** transfer. The same displacement
(+50 mm on x) that lands 14 mm off from the origin lands **45 mm off** from
(−25, 0), with a flipped-sign contamination. The oscillation thus **exposes the
Rung-2a asymmetry at modest separations**, not only at ±100 mm. The asymmetry Rung 2a
deferred to Rung 3 is, in fact, the thing that breaks Rung 2b.

**What was explored and ruled out (so the next session does not re-tread it).**
- *Every axis and separation 20–70 mm* (x swept at 20/30/40/50/60/70 mm, plus y-40
  and diag-50): all diverge, max sustained **4** of the required ≥ 10
  (`tools/probes/juggle_selfcatch_loopgain.py` — the committed sweep now covers the
  full range in one run). No favourable direction — the pose-chaos is sign-changing
  everywhere.
- *Small separations (20–30 mm, tilt ~0.7–1.1°)* to shrink the catch reach: still
  don't sustain (max **4**, at x-20 — and the landing is amplifying even there:
  15 → 86 → 694 mm). The catch reach is not the sole binding issue; the throw
  pose-chaos amplifies even a small offset. So this is not merely a reach-limited
  catch — the throw map itself has gain > 1.
- *Larger separations (≥ 50 mm, tilt ~1.8–2.5°)* to engage tilt harder: also don't
  sustain (max **2**). The cyc-0 landing error is **non-monotonic** in separation
  (43.9 mm at 50, 22.8 at 60, 11.7 at 70) — itself the sign-changing pose asymmetry,
  *not* simply "worse with size" — but no separation holds past a couple of cycles.
- *Deterministic re-check with noise off*: the divergence backbone is
  deterministic (the pose-chaos), not a noise artefact; §3 noise only jitters which
  cycle drops.
- *Tuning the recover/release shape* was **not** pursued: Rung 2a already swept
  recover shapes (pure-slider retract, pull-away, descend-and-away, plan_cup_cycle)
  and settled on `plan_cup_cycle` as the most consistent; the pose-chaos is large
  (gain ~4–11) and sign-changing across all directions, so a recover tweak is not a
  credible path to gain < 1, and chasing one would be exactly the "tune endlessly"
  the gate forbids.

**On not tuning it away.** The gate explicitly forbids papering over a diverging
loop. The divergence is deterministic, holds across all axes/separations with tilt
engaged, and has a clear physical root cause (the contact-detach pose-chaos), so
the honest move is to STOP and surface the BREAK with its head-to-head — not to
hunt for a geometry that happens to sustain a few extra cycles.

## Verification

- `sim/juggle_selfcatch.py::run_self_catch(oscillate=True)` composes the Rung-2a
  throw + Rung-1 catch into the A↔B loop; deterministic per seed (§3 tracking noise
  the only RNG). The column path is behaviorally unchanged (its tests unchanged).
- `tools/probes/juggle_selfcatch_loopgain.py` (committed) sweeps both the COLUMN
  variants and the OSCILLATION geometries; writes
  `temp/probes/juggle_selfcatch_loopgain.csv`.
- `tests/sim/test_juggle_selfcatch.py` adds the oscillation tests: tilt is engaged
  (1.4° vs the column's 0.0°); the first lateral cycle composes; the headline
  `sustains ≥ 10` is `xfail(strict=True)` (the BREAK); does-not-sustain across 6
  seeds; the landing-amplification loop-gain signature (with the seat offset staying
  small); the pose-sensitivity root cause (deterministic ~40 mm swing per 10 mm
  origin shift); seed-determinism.
- Full suite: see Outcome.

## Outcome

Rung-2b subset (`pytest tests/sim/test_juggle_selfcatch.py -q`, run 2026-07-01):
**21 passed, 2 xfailed in 94.92 s** (the 2 xfails = the column *and* the oscillation
`sustains ≥ 10` headlines — both BREAKs). Finalize full-suite gate
(`pytest tests/ -q`, run 2026-07-01): **1580 passed, 4 skipped, 4 xfailed in
853.98 s** (exit 0) — no regressions; the known-flaky allocation tests passed
in-suite. Loop-gain probe re-run (`python tools/probes/juggle_selfcatch_loopgain.py
--seeds 6 --cycles 12`, run 2026-07-01) swept the full 20–70 mm range and reproduced
the BREAK at all eight geometries (max sustained 4 of ≥ 10, at x-20).

**Gate: BREAK — STOP and surface to the operator (go/no-go).** The two-point A↔B
oscillation, WITH tilt engaged (~1.4°), does **not** sustain — it diverges within
1–4 cycles (loop gain > 1) at every separation/axis. This is a **genuine**
tilt-hypothesis failure on a non-degenerate geometry, **not** a degenerate-case
artefact: tilt fixes the band-limit (the pre-tilt divergence's mechanism) but not
the contact-detach's chaotic sensitivity to the throw-origin pose, which is the
binding amplification off-origin. The tilt re-architecture is necessary but **not
sufficient** to close the loop; the next lever is the **contact-detach knife-edge**
(the throw-origin pose-sensitivity / the non-y-symmetric leg asymmetry Rung 2a
deferred), explicitly **not** the aim (already exact) and **not** the tempo (Rung 1
showed slower is worse).

Commit SHA: 6308689 (feat(sim): Rung-2b OPTION-1 re-plan — two-point A↔B
oscillation, tilt engaged, still a BREAK).
