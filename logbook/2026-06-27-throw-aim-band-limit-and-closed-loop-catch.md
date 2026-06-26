---
title: Throw-aim root cause is a platform BAND LIMIT (not angular tracking) + closed-loop catch (1→2 catches)
type: investigation
date: 2026-06-27
status: in-progress
phase: "Sim2Real fidelity — concern 1 (contact) throw-aim: band-limit diagnosis; closed-loop catch WIP"
related_plan: "bb-led-two-ball-juggle-demo.md"
files_changed:
  - controller/demo/juggle_optimizer.py
  - sim/juggle_demo.py
  - sim/ball/manager.py
  - tests/sim/test_demo_juggle_sim.py
commits:
  - 82ec983
subsystem:
  - sim
  - controller
tags:
  - dynamics
  - kinematics
  - control
  - tracking
  - testing
---

# Throw-aim root cause is a platform BAND LIMIT (not angular tracking)

## Summary

The BB-led juggle demo's emergent contact throw lands ~100–140 mm from the
planned catch, so the open-loop scheduled catch misses and the pattern does not
close (1 catch). This session **diagnosed the throw-aim to root and refuted the
2026-06-26 "angular tracking" headline**: a same-instant cup-velocity
decomposition shows the ω×r term tracks fine (contributes < 0.04 m/s to the aim
error), while the dominant miss is a **platform band limit** — the heavy
connect-constraint Stewart platform low-pass-filters every fast maneuver
(~30 mm+ pose-tracking error at the throw AND the catch), so the *achieved*
cup velocity carries ~0.1 m/s of unplanned sweep momentum that no offline plan
predicts. The old demo only ever hit 30 catches because the kinematic
velocity-override **bypassed the platform** (it *set* the ball velocity).

Three mitigations landed, taking the demo **1 → 2 catches** and flipping the
short two-ball headline test to passing (un-xfailed):
1. an optimiser **throw-knot twist penalty** (throw error 249 → 137 mm/s);
2. **`THROW_SPEED_CALIB` 1.08 → 1.0** (the stiffness fix made contact faithful,
   so the old over-throw compensation now over-shoots);
3. a **closed-loop catch** — the runner observes the in-flight ball, ballistically
   predicts its landing, and reaches the platform xy to meet it.

The full ≥30-catch pattern remains blocked by a **band-limit cascade** and is
left as `xfail`; development is paused here to reconsider the planning
architecture (see Path forward).

## Motivation

Follow-up A from `2026-06-26-contact-mechanics-integration.md`: close the throw
aim so the pattern catches ≥30/0 drops again under faithful contact. That entry
hypothesised the residual was platform *angular* tracking (achieved ω ≠ planned
ω at the throw, via the ω×r cup-offset term). The first task was to re-measure
cleanly before committing to a fix — which overturned the hypothesis.

## Investigation

**Clean same-instant decomposition (refutes angular tracking).** Instrumenting
the runner at substep resolution and sampling at the *exact* ball-separation
instant (t_rel ≈ +0.006 s — the throw fires right at the throw knot, so timing
is not the issue), the ball launch velocity decomposes as:

| x-term            | planned (knot 0) | achieved (sep) | error  |
|-------------------|-----------------:|---------------:|-------:|
| centroid linear   | +0.80            | +0.99          | +0.19  |
| ω×r               | −0.12            | −0.13          | −0.01  |
| slider (bank)     | −0.85            | −0.81          | +0.04  |
| **ball launch**   | **−0.17**        | **+0.05**      | +0.22  |

(Error = achieved(sep) − planned(knot 0), i.e. each term's contribution to the
aim error; the column sums to the ball-launch error +0.22. The same-instant
sep-vs-sep centroid residual is slightly larger, ~+0.25, because the planned
centroid is already falling at sep — either way the centroid dominates and ω×r
is negligible.)

The yaw rate *is* mis-tracked (achieved over-rotates by ~1.3 rad/s and lags
> 2 rad/s through the maneuver), but `Δω × r_hand = [−0.006, −0.035, −0.001]`
m/s — **negligible**, because r_hand (0.13 m, almost pure +z) is nearly
parallel to the yaw axis, so even a large yaw-rate error produces almost no
lateral cup velocity. The 2026-06-26 premise ("ω×r ≈ 0.2 m/s lateral error")
does not survive the decomposition. The lateral miss is dominated by the
**centroid linear** term.

**Why the centroid term is the lever, and why it's a band limit.** The throw's
lateral velocity is a *small difference of two large terms*: a large planned
centroid velocity (+0.80) cancelled by the banked slider (−0.85), netting the
small required −0.17. The optimiser leaves the centroid velocity large because
the bank cancels it "for free" (nothing in the leg-jerk² objective penalises
it). But a large, fast-reversing centroid velocity is exactly what the heavy
platform phase-lags. A full-window trace shows the platform tracks the planned
pose like a **low-pass filter**: at the velocity peak (where accel ≈ 0) the
achieved still under-shoots by ~0.21 m/s — a magnitude attenuation, not just a
phase delay. The achieved cup velocity therefore carries ~0.1–0.25 m/s of
sweep momentum the plan cannot predict.

**Bound-twist mitigation and its floor.** A soft penalty on the throw-knot
centroid velocity (windowed over knots N-1,0,1) cuts the planned centroid to
~0 and the throw error 249 → 137 mm/s. But it *floors* there: with the plan
commanding ~0, the platform still *achieves* +0.107 m/s — it sails through the
commanded-zero point carrying its sweep momentum. The tilt/bank (0.98) and
position (±3 mm) track fine; only the inertia-limited centroid motion does not.
The actuators are already stiff (kp=2e5, ~130 Hz leg bandwidth) and the connect
constraints are tight (5 ms), so the ~6 Hz *platform pose* bandwidth is
dominated by platform inertia — a real limit, not gains/compliance.

**Closed-loop catch (the chosen direction).** Since the band limit is
fundamental and deterministic, the catch was made to *reach* for where the ball
actually goes: observe the in-flight ball, ballistically predict its xy at the
catch instant (exact in sim), and blend a platform-xy correction (ramp-HOLD-ramp
smootherstep, peaking with zero correction-velocity to preserve the
velocity-matched seat, zero at the throws). This caught the first emergent hand
throw (cup to 16 mm, within the seat) — demo 1 → 2 catches, short test passes.

**Seat radius 30 → 40 mm.** The reach brings the cup to ~16–33 mm of the ball —
right at the original 30 mm `SEAT_RADIUS_M` boundary (the reach is itself
band-limited, so a few mm of slop remains). The seat radius was widened to 40 mm
(still physical — the ball radius is 35 mm, so 40 mm centre-to-opening is mostly
overlapping) to seat the reached catch. The handoff explicitly sanctions
re-tuning the seat thresholds against the closed pattern; the change is contained
to the demo (the seat metric only runs in `contact_carry` mode).

## Verification

- Throw-aim decomposition probe (`/tmp/probe_throw_aim.py`, 2026-06-27): aim
  error [+0.22, −0.11, +0.39] m/s at baseline; ω×r tracking-error contribution
  [−0.006, −0.035, −0.001] m/s (negligible). Bound-twist (lin=3e5) → 137 mm/s.
- Demo (`python sim/juggle_demo.py --duration 30 --no-log --seed 0`, run
  2026-06-27): **2 captures / 0 drops** (was 1) with the closed-loop catch.
- `tests/sim/test_demo_juggle_sim.py::test_short_run_...` (un-xfailed):
  **passes** — both balls caught. `::test_full_...` stays `xfail` (2 < 30).
- Full suite: `pytest tests/ -q` — (date, command, result) triple in the
  landing commit message.

## Discussion

This was a long diagnosis with **several hypotheses raised and withdrawn on
evidence** — recorded so a future reader (especially the next planning
re-architecture) does not re-walk them.

**"It's angular tracking / the ω×r term" — WITHDRAWN.** The handoff's framing.
The clean same-instant decomposition shows ω×r tracks to 0.01 m/s and, because
r_hand ∥ the yaw axis, even the *full* yaw-rate error contributes < 0.04 m/s.
The earlier "x-component badly off" reading was a measurement-timing artifact
(planned and achieved sampled ~12 ms apart on a fast-swinging ω). Lesson:
decompose the actual quantity (cup velocity), don't reason from a proxy (ω).

**"Bound the throw-knot centroid velocity to zero" — PARTIAL, then FLOORED.**
Penalising the *plan*'s throw-knot velocity drove the planned centroid to ~0
but the *achieved* stayed ~0.1 m/s (sweep momentum). A single-knot penalty even
made it worse (a sharp velocity notch the low-pass platform over-shoots);
widening to a 3-knot window helped (249→137) but the floor is the platform, not
the plan.

**"The platform under-banks / the tilt mis-tracks" — WITHDRAWN.** Measured: tilt
2.54° planned vs 2.49° achieved, bank lateral attenuation 0.98, position ±3 mm.
The slow/steady DOFs (tilt, position) track fine; only the *fast* centroid
velocity reversal is attenuated. Confirms the low-pass character.

**The closed-loop catch hit the SAME band limit, three times.** (a) The reach is
itself a fast maneuver the platform over-shoots — the cup overshoots the
commanded reach by ~1.2×, needing a 0.83 overshoot-calibration to land on the
ball. (b) A peak-and-leave bump injects reach-velocity that breaks the
velocity-matched seat (the cup passes the ball while still moving); a
ramp-HOLD-ramp that dwells co-moving was required. (c) In the two-ball pattern,
ball 0 (the BB ball) re-seats *loosely* after the fast velocity-matched catch
and its throw stroke then launches it **at the stroke start** (carrying the
platform's instantaneous lateral sweep velocity) → a 3 m/s sideways throw; the
resulting lost ball commands an absurd reach that flails the platform (hence a
reach clamp). Each is a distinct consequence of the same root cause.

**Why this matters for the architecture.** The demo is fully open-loop
(scheduled throws/catches, no ball feedback) driven by an *offline* periodic
trajectory that assumes perfect tracking. On a band-limited platform that
assumption is false, and the emergent throw exposes it. The kinematic override
hid it by cheating. Closing the loop on the catch helps but fights the band
limit at every fast maneuver. The cleaner long-term fix is likely to treat the
planner as a *higher-level* online planner (re-planning against the achieved
state) rather than a one-shot offline trajectory — see Path forward.

## Path forward

Development is **paused** here to reconsider the planning architecture. Options
on the table:
- **Online / higher-level planning.** Use the MPC (or a kinematic toss-juggling
  planner) as a higher-level planner that re-plans against the *achieved*
  platform/ball state each cycle, instead of replaying a one-shot offline
  trajectory that assumes perfect tracking. Reference to review:
  `github.com/kploeger/kinematic_planning_for_nball_toss_juggling` (Kai
  Ploeger's kinematic n-ball toss-juggling planner).
- **Deterministic throw pre-compensation.** Keep the physical contact throw but
  bias the platform command at the throw for its known, repeatable sweep
  momentum so the emergent ball lands near the planned catch (open-loop catch
  then works). Smaller change; partial walk-back from "pure emergent".
- **Harden the closed-loop catch.** Fix ball-0's loose re-seat (settle before
  the throw), make the reach collision-aware vs the other in-flight ball, and
  stabilise across cycles. Most faithful, largest effort.

Goal restated by the user: the simulation should be **high-fidelity and entirely
physics-driven**.

## Artifacts (one-off probes, not committed; /tmp)

- `probe_throw_aim.py` — substep throw decomposition (planned vs achieved cup
  velocity terms; full-window ω + centroid tracking).
- `probe_tune_twist.py` — throw-twist weight sweep (centroid speed, yaw, leg
  limits).
- `probe_aim_sweep.py` — achieved aim/landing error vs weight combos.
- `probe_tilt.py` — bank/tilt tracking at the throw.
- `probe_reach_debug.py`, `probe_catch1.py`, `probe_ball0.py` — closed-loop
  catch reach + ball-0 re-throw diagnostics.
