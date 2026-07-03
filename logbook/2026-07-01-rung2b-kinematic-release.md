---
title: Rung-2b kinematic-release — the MAKE. The throw→catch→throw loop closes (12/12 cycles, all seeds) once the throw cuts the ball free at the planned take-off velocity instead of letting it emerge from the contact push; contact-parameter tuning ruled out first, then a clean contact-carry ↔ kinematic hand-off rejects the real ~0.6 mm seat offset (loop gain ≪ 1)
type: feature
date: 2026-07-01
status: resolved
phase: "Online-juggle tilt re-architecture — Phase 3 / Rung 2b (throw-and-self-catch loop, MAKE-OR-BREAK gate — the kinematic-release re-architecture; MAKE)"
related_plan: "bb-online-juggle-tilt-rearchitecture.md"
files_changed:
  - sim/ball/manager.py
  - sim/plant/mujoco_plant.py
  - sim/juggle_selfcatch.py
  - tests/sim/test_ball.py
  - tests/sim/test_juggle_selfcatch.py
  - tools/probes/juggle_kinematic_release.py
  - tools/probes/juggle_allkinematic_loop.py
  - tools/probes/juggle_selfcatch_loopgain.py
  - tools/probes/README.md
  - plans/active/bb-online-juggle-tilt-rearchitecture.md
commits:
  - 37a47e5
subsystem:
  - sim
tags:
  - control
  - planning
  - contact
---

# Rung-2b kinematic-release — the make-or-break MAKE

> **Follow-up correction (2026-07-03).** The **loop closure is real** (12/12 × 6
> seeds), but the *catch mechanism* described below ("co-moving descent seat …
> tracks the ball descent then arrests to a co-moving stop") does **not execute as
> written**. Instrumentation (`tools/probes/juggle_motion_quality.py`) shows the
> ball seats **~145 mm ABOVE catch_z on a cup pegged at the stroke ceiling, moving
> UP at contact** — the kinematic seat overshoots to the ceiling to build the
> downward runway a −2.7 m/s arrival needs, then the ball falls onto the parked
> cup. The MAKE is a **static-cup catch** produced by an unintended controller; the
> "co-moving seat" is aspirational. The motion is not smooth (~1026 mm cup
> path/cycle, ceiling slammed every cycle). See `2026-07-03-motion-quality-review.md`
> and `2026-07-03-p2-selfcatch-reunification-tension.md` (which found the slam IS
> the seat's runway, so smoothing the catch requires co-designing a gentler throw —
> a larger rung). The loop-gain numbers stand; the seat narrative is corrected.

## Summary

The Rung-2b single-ball throw→catch→throw self-catch loop — the plan's designated
make-or-break gate — **now closes.** With a **kinematic velocity release** on the
throw, the two-point A↔B oscillation sustains **12/12 cycles on all 6 seeds** with a
**flat ~0.6 mm in-cup offset** (loop gain ≪ 1), head-to-head against the shipped
contact-detach throw which diverges within 1–3 cycles (0–2 sustained, mean 1.2/12).

This resolves the two prior Rung-2b BREAKs
(`2026-07-01-rung2b-selfcatch-column-divergence.md`,
`2026-07-01-rung2b-oscillation-tilt-engaged-diverges.md`). The oscillation BREAK had
already isolated the root cause deterministically: the **contact-detach throw is a
knife-edge** — a 10 mm throw-origin shift swings the landing ~40 mm
(`dLanding/dOrigin` ≈ 2.7 induced-2-norm, ~6.6 worst one-sided), so loop gain > 1 and
no catch scheme can stabilise it. **Tilt fixed the band-limit (the original 2026-06-27
divergence's mechanism) but is *necessary-but-not-sufficient* — a second, independent
mechanism (contact-detach pose-sensitivity) kept loop gain > 1.** The fix removes that
mechanism at the source.

## Investigation arc (Path A)

The operator steered this after reviewing the oscillation BREAK, and the arc was run as
a sequence of small empirical probes before any harness code was written.

### 1. Contact-parameter tuning — ruled out

The knife-edge lives in the throw's *detach*, and the detach happens under the "stiff
release" contact regime (`CONTACT_STIFF_SOLREF/SOLIMP` in `sim/ball/manager.py`). The
obvious first hypothesis: is it a *contact-stiffness* artefact that softer/graduated
contact parameters fix? A colleague's fixed-contact params (`solref 0.01 1.5`,
`solimp 0.9 0.95 0.001`, `friction 1 0.005 0.001`) were the specific prompt.

- **A/B measurement** (`/tmp/probe_contact_params.py`, 2026-07-01, noise off,
  `dLanding/dOrigin` at target (100,0) mm, origin ±10 mm): baseline stiff release →
  **gain 2.74**, separates cleanly, err@origin 21.5 mm. The colleague's params (applied
  to *our* 35 mm ball) → the ball **fails to separate** (`separated=False`, rides the
  cup out, lands ~110 mm off). Their softer/graduated contact (10 ms time-const,
  `dmin 0.9`) triggers exactly the **soft-contact drag-back** our stiff 4 ms release
  exists to prevent.
- **Frontier scan** (`/tmp/probe_contact_sweep.py`, 2026-07-01): swept release
  `solref` time-const 4–10 ms × `solimp` `dmin` 0.90–0.99. **Every** configuration that
  separates cleanly has gain **2.4–16** (all ≫ 1); softening either glues the ball
  (no separation) or wrecks accuracy. **There is no sweet spot.**

Conclusion: contact stiffness is **not** the lever. The pose-sensitivity is *intrinsic
to the contact-carry detach* — the take-off velocity is a contact push whose magnitude
and direction depend on where the ball sits in the cup, so an origin/seat offset →
different push → amplified landing. This holds at **zero noise**, so it is a
deterministic plant property, not observation noise. (It also independently answered
the operator's standing question — "is the §3 noise requirement making this harder?" —
**no**: every measurement here is noise-free and the gain is unchanged.)

### 2. Kinematic velocity release — the fix, validated open-loop

Instead of letting the take-off velocity **emerge** from the contact push, **cut the
ball free** at the release instant with a **set velocity**. The planner already computes
`v_takeoff(throw_origin → target)`, which *compensates* an origin offset by design; the
contact-carry detach then *corrupts* that compensation with a pose-dependent push (the
2.7 gain). A kinematic release delivers the planned/achieved velocity cleanly, so the
compensation survives.

- **Open-loop measurement** — the reproducible committed probe
  (`tools/probes/juggle_kinematic_release.py`, 2026-07-02 `python
  tools/probes/juggle_kinematic_release.py`, noise off, target (100,0) mm, origin ±10 mm):
  **detach `dLanding/dOrigin` = 3.53** (BREAK, err 101.5 mm off-origin) vs **kinematic =
  0.04** (MAKE, err 3.6 mm) — an ~**88×** reduction, and the make-or-break gate evidence.
  The initial `/tmp` exploration (`/tmp/probe_kinematic_release.py`, not committed)
  measured the same effect *from the origin*: contact-detach **2.74** (induced-2-norm) /
  err 21.5 mm, kinematic **0.01** at the achieved cup velocity (err 1.2 mm) and **0.05**
  at the planned `v_takeoff` (err 5.1 mm). The harness releases at `v_takeoff` (see
  below); all kinematic figures are ≪ 1.

**Why this is the clean-separation limit, not a cheat.** A ball that leaves the cup with
no residual contact push departs at the cup's velocity (Newton). The contact-carry
detach adds a push *on top of* that ideal; the kinematic release *is* that ideal. It
models a clean separation instead of a messy one — the fidelity cost is that we stop
resolving the last millisecond of detach contact, which is precisely the millisecond
that carries the knife-edge.

### 3. Closed-loop confirmation — all-kinematic loop

Before touching the real harness, an **all-kinematic** single-ball A↔B loop (kinematic
capture + kinematic release, one consistent contact model, no hand-off) isolated the
question "does a kinematic release *close the loop*?" from the integration.

- `/tmp/probe_allkinematic_loop.py` → promoted to
  `tools/probes/juggle_allkinematic_loop.py` (2026-07-01, §3 tracking noise on):
  **12/12 cycles on all 6 seeds** with a **flat ~2.4 mm** landing error (loop gain
  ≪ 1), vs the contact-carry baseline's mean 1.2/12. Clean positive signal.

### 4. The integration challenge — and the hand-off contract

A quick monkeypatch that swapped **only** the release in the real `SelfCatchRunner`
(`/tmp/probe_hybrid_loop.py`, 2026-07-01) did **not** sustain, because the throw and the
contact-carry **catch share the contact machinery**, and each naive swap fails one way:

- **Contact left enabled** + overwrite the ball velocity → the catch is undisturbed, but
  the ongoing contact push **re-corrupts** the imposed velocity (cyc-1 landing jumped to
  **101 mm**). Forcing the velocity through more ticks disrupts the near-cup dynamics and
  the catch fails.
- **Contact disabled** via `release_ball` → clean flight (the open-loop recipe), but the
  catch **fails to seat** (`held=False`, ball sloshes out): `release_ball` is the
  *kinematic-hold* ejector, and its guard flags + `contype` toggle disrupt the
  *contact-carry* seat state machine.

The resolution is a **new primitive designed as a contract**, not ad-hoc flag-poking:
`Ball.ballistic_release(velocity_mms)` (+ the `MuJoCoPlant` delegate). Its documented
state-in/state-out:

- **In:** a contact-carried held ball (`_held` True, ball geom `contype=3`), co-moving
  with the cup.
- **Out:** the ball is free (`_held` False), its linear velocity is exactly the imposed
  value, ball-hand contact is broken for the separation (`contype=1`, ground-only) so
  **no residual push corrupts the launch**, and the re-capture guard is armed. Because
  the manager stays in `contact_carry` mode, `check_capture` re-enables hand contact
  once the ball clears the cup and routes the returning ball to the **seat** metric — so
  the **real contact-carry catch re-seats it normally**.

This is the clean hand-off between a kinematic throw and the contact-carry catch: it
defeats both failure modes with one enforcement point. The harness exposes it as
`SelfCatchConfig.release="kinematic"` (default `"detach"`, so tilt=0 / Rung-1 / Rung-2a
/ the detach loop are **byte-identical**). The kinematic mode also uses a co-moving
seat regime robust to the clean release and a shorter carry dip (0.10 m vs 0.16 m) that
avoids a between-cycle reposition artefact.

**The throw releases at `v_takeoff`, not the achieved cup velocity.** `v_takeoff` is
exactly the ballistic velocity the catch's estimator predicts the landing from; the
achieved cup velocity overshoots it (~61 mm/s in the carry), which would bias the
landing. Both are ≪-1-gain open-loop (0.05 vs 0.01); `v_takeoff` is chosen for
loop-consistency with the catch, not for the marginally lower open-loop gain.

## Result — the MAKE (head-to-head)

Hybrid = **contact-carry catch** (Rung-1, KEPT — its cushioned constant-decel seat
produces the real ~0.6 mm in-cup offset that is the disturbance to reject) +
**kinematic-release throw**. Default A↔B oscillation (x-40, tilt ~1.4°), 12 cycles,
§3 tracking noise, seeds 0–5:

| seed | detach sustained | kinematic sustained | kinematic in-cup offset trend (mm) |
|-----:|:----------------:|:-------------------:|:-----------------------------------|
| 0 | 2 | **12** | 0.5 0.6 0.6 0.5 0.6 0.6 0.6 0.6 0.6 0.8 0.6 0.5 |
| 1 | 2 | **12** | 0.5 0.6 0.5 0.6 0.6 0.6 0.6 0.6 0.6 0.6 0.6 0.6 |
| 2 | 1 | **12** | 0.6 0.6 0.6 0.6 0.5 0.6 0.6 0.6 0.6 0.5 0.5 0.6 |
| 3 | 1 | **12** | 0.6 0.6 0.5 0.6 0.5 0.5 0.6 0.6 0.6 0.6 0.5 0.6 |
| 4 | 1 | **12** | 0.6 0.5 0.6 0.7 0.7 0.6 0.6 0.5 0.6 0.6 0.5 0.6 |
| 5 | 0 | **12** | 0.6 0.6 0.6 0.6 0.6 0.6 0.6 0.7 0.6 0.6 0.6 0.6 |

(2026-07-02, direct `SelfCatchRunner(...).run()` head-to-head, same geometry/seed:
detach mean **1.2/12**, kinematic **12/12** every seed, in-cup offset **dead flat**
~0.5–0.6 mm → loop gain ≪ 1, the real seat offset REJECTED.) The kinematic in-cup
offset does not grow across 12 cycles — the disturbance the catch introduces each cycle
is rejected rather than amplified, which is the loop-gain-<-1 signature the gate asks
for.

**Gate verdict: MAKE.** The tilt + kinematic-release architecture closes the
single-ball self-catch loop under contact + §3 noise. → proceed to Rung 3 (two-ball).

## Verification

- Self-catch suite (2026-07-02, `pytest tests/sim/test_juggle_selfcatch.py -q`):
  **33 passed, 1 xfailed in 290.21s**. The xfail is the retained column-degenerate
  BREAK (kept as documentation). New tests assert the MAKE (`sustained ≥ 10` on all
  seeds), the head-to-head (`kin.sustained > 2·detach.sustained`), real-seat-offset
  rejection, determinism-per-seed, and `release="detach"` byte-identity.
- Full suite (2026-07-02, `pytest tests/ -q`): **1594 passed, 4 skipped, 3 xfailed in
  1071.87s**, exit 0 — no failures (+14 passed / −1 xfailed vs the 1580/4/4 baseline: the
  new hybrid tests, plus the oscillation-BREAK strict-xfail resolved by the MAKE). A
  first gate run surfaced 2 failures in the new `test_ball.py::TestBallisticRelease`
  cases — a **test bug, not a primitive bug**: `get_ball_state` reads MuJoCo `sensordata`
  (refreshed only by `mj_step`/`mj_forward`), and the tests read it immediately after
  `ballistic_release` (which writes `qpos`/`qvel` directly), so they saw the stale
  parked z (98237 mm). Confirmed empirically (a single `mj_forward` yields the seated
  z=489.6 mm + the imposed velocity [250,0,2500] exactly) and fixed by forwarding once
  before the read (matching the real loop, where a `plant.step` always follows the
  release). The primitive is proven correct by the direct `data.qvel` assertion (which
  passed) and the 12/12 loop.
- `Ball.ballistic_release` has unit coverage in `tests/sim/test_ball.py`
  (`test_imposes_velocity_and_breaks_contact`, `test_no_residual_push_flies_ballistic`).

## Discussion

**Why the fix is a contract, not a patch (the Engineering-Philosophy call).** The naive
monkeypatch would have "worked" only by threading residual-push compensation and
guard-flag resets through the runner — a patch that re-breaks the moment the catch or
carry changes. Instead the ball's throw→flight→re-seat transition is a single
primitive with a stated pre/post-condition and one enforcement point, living next to the
two pre-existing detach paths (`begin_physics_throw`, `release`) it complements. Future
throw modes compose against the contract, not against the runner's internal state.

**The reframe that unlocked it (hypothesis withdrawn, load-bearing).** The whole
re-architecture's bet was "tilt is the engine that kills the divergence." The
oscillation BREAK forced that to be *narrowed*: tilt kills the **band-limit** mechanism
(the 2026-06-27 divergence) but a **second, independent** mechanism — the contact-detach
knife-edge — kept loop gain > 1. Confidence in the tilt hypothesis was not evidence for
its sufficiency; the next data point (oscillation still diverges *with* tilt engaged)
didn't fit, so the hypothesis was narrowed rather than rescued. The fix targets the
mechanism the narrowing exposed.

**What was ruled out, and why that matters.** The frontier scan (§1) is the load-bearing
negative result: it converts "we couldn't find a contact tuning that works" into "no
separating contact tuning *can* work" (gain 2.4–16 across the whole grid). Without it,
the kinematic release looks like one option among many; with it, the kinematic release
is the *only* lever that removes the knife-edge at its source. The scan cost ~15 minutes
and prevented an open-ended tuning rabbit-hole.

**Fidelity tradeoff, stated honestly.** The kinematic release stops resolving the final
contact millisecond of the throw — the same millisecond that carries the knife-edge on
real hardware. Two mitigations frame the risk: (a) it is the *clean-separation limit*,
which is the physically correct idealisation of a cup that lets go without pulling; (b)
the catch is kept fully contact-physical (contact-carry seat, real ~0.6 mm offset), so
the loop still exercises a genuine disturbance-rejection problem. What we have **not**
proven is that a real cup separates this cleanly; the residual-push physics is a
sim-2-real question flagged for the hardware bring-up, not resolved here. The all-
kinematic vs hybrid agreement (both 12/12) says the result is not an artefact of the
catch idealisation.

**Why keep the detach path.** `release="detach"` remains the default and byte-identical
so tilt=0, Rung-1, Rung-2a, and the two documented BREAK configs stay reproducible —
the divergence evidence is kept runnable, not overwritten. The column self-catch test
stays `xfail(strict=True)` as the documented degenerate case.

**Carried forward to Rung 3.** The two-ball composition inherits the same throw
primitive; the kinematic release should carry, but two-ball adds inter-ball timing and a
second seat offset. The knife-edge is closed at the throw level (a property of the
primitive, not the ball count), which is the reason to expect Rung 3 to compose — but
the make-or-break discipline applies there too.

## Related

- Prior BREAKs: `2026-07-01-rung2b-oscillation-tilt-engaged-diverges.md` (the root-cause
  measurement), `2026-07-01-rung2b-selfcatch-column-divergence.md` (the degenerate case).
- Primitives: `2026-06-30-rung2a-single-ball-tilt-throw.md` (throw, where the knife-edge
  was first characterised), `2026-06-30-rung1-clean-single-catch.md` (catch, the seat
  offset).
- Pre-tilt divergence (the original band-limit cascade, head-to-head baseline):
  `2026-06-27-online-juggle-throw-fix-catch-axis-split-band-limit-cascade.md`.
- Reusable probes: `tools/probes/juggle_kinematic_release.py` (open-loop knife-edge
  gain), `tools/probes/juggle_allkinematic_loop.py` (all-kinematic loop de-risk),
  `tools/probes/juggle_selfcatch_loopgain.py` (the full detach/oscillation/kinematic
  sweep). One-off session probes (`/tmp/probe_contact_params.py`,
  `/tmp/probe_contact_sweep.py`, `/tmp/probe_hybrid_loop.py`) were the throwaway A/B and
  integration-failure probes; not committed.
