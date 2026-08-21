---
title: INERTIA_RATIO does not reconcile with the measured masses — what it actually is, and whether it matters
status: parked   # 2026-08-21 — throws work across the whole envelope; parked on priority, not on a blocker
created: 2026-08-21
related_logbook:
  - 2026-08-18-derived-throw-envelope.md
  - 2026-07-29-hand-post-release-decel.md
related_code:
  - config/hardware_config.yaml
  - ros_ws/src/jugglebot/Teensy_code_platform/Trajectory.h
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/throw_envelope.py
  - ros_ws/docs/hand_decel_feedforward.md
---

# Plan — reconcile `INERTIA_RATIO`

**Branch:** `mvp-trajectory-bringup`
**Status:** PARKED 2026-08-21. Nothing here is blocked; the throw profile is
empirically validated at the current value across the whole C-HAND-3 envelope,
including a session to 5.608 m/s on 2026-08-21. This exists so the discrepancy
is not lost, not because anything is broken.

## The finding

`config/hardware_config.yaml` declares `inertia_ratio: 0.747` in **both** the
`teensy_trajectory` and `ball_butler_trajectory` sections, commented only as
"Inertia ratio for throw dynamics". `throw_envelope.py:153` states the identity
explicitly:

> `INERTIA_RATIO = m_hand/(m_hand + m_ball)` — the same identity that makes
> `throwD = -throwA/IR` a constant-motor-torque design.

On 2026-08-21 the owner measured both masses directly:

* **moving linear mass = 64.3 g** — the hand plus the linear slider it is
  fastened to. Explicitly NOT including the motor, which spins and contributes
  through the hand spool.
* **ball = 71 g, 70 mm diameter** (now `physics.juggling_ball_mass_kg` /
  `juggling_ball_radius_mm`).

Neither reading of the identity produces 0.747:

| reading | value | vs declared |
|---|---|---|
| translating mass, `0.0643/(0.0643+0.071)` | **0.4752** | far below |
| reflected inertia, `J_ff = 9.5e-6` | **0.8409** | above |
| reflected inertia, `J_true >= 1.0126e-5` | **0.8492** | above |

Supporting numbers (spool radius 5.032 mm, from `LINEAR_GAIN` = 31.6284 rev/m):
ball reflected inertia **1.798e-6 kg·m²**, hand-alone reflected inertia
**1.628e-6 kg·m²**. For 0.747 to be a reflected-inertia ratio the hand-side
inertia would have to be **5.308e-6 kg·m²** — roughly half the declared `J_ff`
and well under the measured decel-side bound `J >= 1.0126e-5`.

**The owner's clarification points at the reflected reading.** The rotor spins
through the spool, so it belongs in the hand-side term: with `J_ff = 9.5e-6`
the rotor accounts for `9.5e-6 - 1.628e-6 = 7.87e-6`, i.e. **~83 % of the
hand-side inertia**. That makes 0.8409 the physically meaningful figure and
0.4752 a red herring — but it does not explain 0.747.

## Why it is not urgent

`INERTIA_RATIO` is not inert — it sets `throwD = -throwA/IR` and feeds
`calcThrow`'s `x2` (ball release) and `x5` (catch point). But the resulting
profile is validated on hardware: throws land, catches succeed, and the
2026-08-21 session flew 4.436 → 5.608 m/s with coast 0.200 → 0.250 rev and every
throw accepted. Whatever 0.747 is, the machine works with it.

So there are two very different situations and the repo cannot distinguish them:

1. **The comment is wrong and 0.747 is a correctly-tuned coefficient** that
   happens not to be the mass ratio the docstring claims. Then the fix is
   documentation only.
2. **The value is wrong** and the tuned profile has absorbed the error into `x2`
   and `x5`. Then correcting it moves the release and catch points, and needs a
   re-validation round exactly like the `hand_stroke_m` question
   (`config/hardware_config.yaml`, that key's comment).

## What would unpark it

Any of:

* a throw-profile change that touches `x2`/`x5` for another reason — do this
  reconciliation in the same pass rather than tuning on top of an unexplained
  constant;
* `plans/parked/hand-trajectory-generator-overhaul.md` resuming, since it
  rewrites the generator that consumes `INERTIA_RATIO`;
* the accel-FF or learned-FF arcs resuming, both of which model hand inertia and
  would inherit the discrepancy;
* a measured rotor inertia becoming available, which would settle reading (1) vs
  (2) directly.

## Phase 1 — archaeology (desk, ~1 h)

Reconstruct what 0.747 was derived from. `git log -S "0.747"` over
`config/hardware_config.yaml` and the firmware; check whether it predates the
sensorised hand and the 2026-07-31 hand-ODrive swap. Establish whether it was
ever a mass ratio or was always a tuning constant. **Deliverable: a one-line
answer to "situation 1 or situation 2".**

## Phase 2 — only if situation 2 (bench)

Re-derive from the reflected identity with a measured rotor inertia, then treat
it as a profile change: `x2`/`x5` move, so the tilt map, `toss_calibration.yaml`
and the catch tuning all need re-validation. Not a config edit.

## Constraints

* **Do not change `inertia_ratio` without a re-validation round.** It is
  load-bearing for the release point, and the current value is the one every
  hardware result in the logbook was measured at.
* `throw_envelope.BALL_MASS_KG` derives from it and feeds `ACCEL_AUTHORITY`. The
  current error is **conservative** (~5 % tight, established 2026-08-21), so
  there is no safety pressure to move quickly.
* The two `inertia_ratio` keys (Jugglebot and BallButler) are separate values
  that happen to be equal. Decide deliberately whether a correction applies to
  both — BallButler's hand is a different mechanism (`hand_stroke_m` 0.28).
