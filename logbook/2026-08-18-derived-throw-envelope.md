---
title: Throw height is now derived from the machine's limits, not two hand-picked constants — and the derived ceiling is 0.887 s, not 1.10
type: feature
date: 2026-08-18
status: resolved
phase: "single-ball-toss / hand-command-continuity follow-on"
related_plan: single-ball-toss.md
files_changed:  # sources of truth + the docs that carried the old band; the
                # commit carries the full set (generated copies, 6 test files)
  - config/hardware_config.yaml
  - config/generate_config.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/throw_envelope.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/docs/hand_throw_envelope.md
  - ros_ws/docs/hand_decel_feedforward.md
  - sim/toss_gate.py
  - tests/motion/test_throw_envelope.py
  - tests/hardware/session_anomaly_fixes.md
  - tests/hardware/session_phase8_toss_hardware.md
  - tests/hardware/toss_trace_recorder.py
  - plans/active/single-ball-toss.md
subsystem:
  - motion
  - config
tags:
  - safety
  - kinematics
---

# The derived throw-admission envelope (contract C-HAND-3)

## Summary

`toss_sequencer` gated throw height on two literals — `FLIGHT_TIME_MIN_S = 0.55`
("plan sweep floor") and `FLIGHT_TIME_MAX_S = 1.10` ("plan sweep ceiling"). The
owner asked for those to become **derived** from the machine's configured
accel / velocity / torque / timing limits, at one enforcement point, with every
input named.

They are. Contract **C-HAND-3** (`ros_ws/docs/hand_throw_envelope.md`) states the
invariant; `jugglebot/motion/trajectory/throw_envelope.py::evaluate` is the
single enforcement point; `tests/motion/test_throw_envelope.py` (35 tests) fails
if it is violated. Six bounds are evaluated per throw — `END_STOP`,
`DECEL_AUTHORITY`, `ACCEL_AUTHORITY`, `REGEN`, `WIRE_BAND`, `ARM_WINDOW` — and a
refusal names the binding one and quotes the computed envelope:
`REJECTED_THROW_ENVELOPE(END_STOP:modelled peak 10.660 rev at 4.436 m/s exceeds
10.600 rev …)`.

| | shipped (hand-picked) | derived | change |
|---|---|---|---|
| flight time | 0.55 – 1.10 s | **0.4949 – 1.1485 s** | floor −55 ms, ceiling **+49 ms** |
| apex (`throw_height_m`) | 0.371 – 1.483 m | **0.300 – 1.617 m** | ceiling **+134 mm** |
| release speed | 2.709 – 5.399 m/s | **2.440 – 5.637 m/s** | ceiling **+0.24 m/s** |

**The binding bound is `DECEL_FF_HEADROOM`** at 5.637 m/s — the decel
feedforward may draw at most 85 % of `hand_curr_limit_a`, C-HAND-2's own
requirement — just ahead of `DECEL_AUTHORITY` at 5.816. `END_STOP` does not bind
until 7.468 m/s. That is the outcome the owner's directive described: the
machine's limits are accel/velocity/torque, not an end stop.

**This entry covers two passes, and the second overturned the first.** The
2026-08-18 pass modelled coast from the only data that then existed — the
2026-07-27 pre-fix ladder — and got a ceiling of **0.887 s** that *refused the
1.0 m working point the machine flies every session*. It said explicitly that
one capture would settle it. The capture happened on **2026-08-20** and did:
measured coast at 4.436 m/s is **0.226 rev, not the 0.700 the pre-fix model
extrapolated**, and the working point sits 19.4 mm from metal, not 4.4.

Status is **in-progress**: the envelope is measured rather than
extrapolated-from-superseded-data, but the ceiling sits **1.27× past the fastest
speed ever flown on this plant**, and two items need owner sign-off
(§ Open questions).

## Motivation

The 2026-07-27 sitting's five off-run-sheet ~1.2 m tosses made light physical
contact with the hand's end stop. C-HAND-2 corrected the braking feedforward that
caused it. What nothing corrected was the **admission gate**: the band that
decided a 1.2 m toss was legal in the first place.

That band's ceiling had exactly one physical justification, in its own comment —
"≈ 5.4 m/s < 7.0 Teensy ceiling", i.e. it is below the builder clamp. C-HAND-2
later justified it better (decel authority), but both justifications bound the
**commanded** profile. The commanded profile never leaves `x3` = 9.9594 rev.
What reaches for the stop is the **uncommanded** ballistic coast past `x3`, and
nothing bounded that at all.

## Design

### The shape of the answer

The gate takes `(flight_time_s, release_speed_mps)` — the *commanded* release
speed, not one re-derived from the flight time. That distinction is load-bearing:
a Tier-8b displaced throw is aimed, so it releases faster than its flight time
alone implies, and a T-only band silently under-bounds it. `FLIGHT_TIME_MIN_S`
and `FLIGHT_TIME_MAX_S` survive as module names but are now the **reported
projection** of the speed envelope onto Tier-8a co-located flights; the sim
sweep, the runbook and several tests read them, and none of them is the gate.

Full derivation of each bound, with numbers, is in the contract. In brief:

| bound | binds at | made of |
|---|---|---|
| **`DECEL_FF_HEADROOM`** | **5.637 m/s** ← the ceiling | `decel_ff_current_headroom_frac` 0.85, `hand_curr_limit_a` 50, `Kt` 0.0055133, `J_ff` 9.5e-6 |
| `DECEL_AUTHORITY` | 5.816 m/s | as above but `J` 1.050e-5, `torque_soft_limit` 0.7 |
| `ACCEL_AUTHORITY` | 5.945 m/s | + the ball (0.0952 kg, from `INERTIA_RATIO`) and gravity |
| `REGEN` | 6.574 m/s | `dc_max_negative_current` 8 A × `dc_bus_nominal_v` **45 V** = 360 W **burst** |
| `WIRE_BAND` | 7.0 m/s | `max_event_vel_mps` |
| `END_STOP` | 7.468 m/s | `hand_motor_hard_stop_revs` 10.8, `end_stop_margin_rev` 0.2, the measured coast ladder |
| `ARM_WINDOW` | **T ≥ 0.4949 s** | `arm_window_margin_s` 0.05 + `hand_stroke`'s whole timing model |

**The torque bounds bind and `END_STOP` does not** — which is the owner's
directive satisfied, and the opposite of what the 2026-08-18 pass concluded from
pre-fix data (`END_STOP` at 4.357 m/s with torque holding 1.46 m/s of slack).
`tests/sim/test_hand_throw_decel_ff.py::test_the_derived_ceiling_is_torque_bound_and_this_contract_sets_it`
is the tripwire if it inverts again.

The floor **loosens** (0.55 → 0.4949 s), because 0.55 was never derived from
anything: the catch-arm window it protects does not close until **0.4542 s**.

## Discussion

### 2026-08-20 — the capture happened, and it overturned the 2026-08-18 pass

The 2026-08-18 pass refused to credit the flashed FW-2 plant and modelled coast
from the 2026-07-27 pre-fix ladder, on the grounds that the correction was
unmeasured. It listed four reasons, and reason 1 was "it has never been
measured". It also said the way out was one capture on a working point the
machine already flies. **That is exactly what resolved it**, which is the best
possible outcome for that argument: the conservative call held the line for two
days and cost nothing but a refused goal.

**What the capture shows.** Bag `2026-08-20_21-51-39`, six throws, two each at
0.5 / 0.7 / 1.0 m, all caught, on the flashed plant (Platform FW 3, symmetric
±0.7 N·m torque limits, `iq_meas` reaching −17.32 A — well past the old −10.00 A
floor, so the clamp is provably gone):

| release | apex | n | coast max | η |
|---|---|---|---|---|
| 3.142 | 0.5 m | 2 | 0.1342 | 0.968 |
| 3.714 | 0.7 m | 2 | 0.2119 | 0.954 |
| 4.436 | 1.0 m | 14 | **0.2260** | 0.951 |

Against the pre-fix model's extrapolated **0.700 rev** at 4.436, measured is
**0.226**. The 2026-08-18 draft put that working point 4.4 mm from metal; it is
**19.4 mm**.

**The clamp was the mechanism, and there is a within-session A/B for it.** Bag
`2026-08-18_18-42-19` changes the braking clamp mid-session — I verified this
from the raw `iq_meas` rather than taking it on report: the braking floor never
passes **−8.87 A** through t = 0–100 s, then reaches −13.71 / −16.58 / −17.39 A
from t = 100 s. Its **first** throw, at t = 84.9 s, coasted **+0.763 rev**;
every other throw in the same bag, at the *identical* commanded 4.436 m/s,
coasts 0.18–0.23. Same command, same firmware, **3.8× the coast**. That single
bag answers the question C-HAND-2 flagged as open and the 2026-08-18 pass could
only argue about: **the clamp was live, it *was* binding the decel ramp, and
removing it is what fixed the overshoot.** That throw is excluded from the
ladder as pre-fix data.

### The extrapolation law had to change too, not just the numbers

This is the part that would have been easy to get wrong by swapping rungs and
leaving the code alone. The old law above the top rung was **hold the achieved
deceleration constant** (`coast = v_rev²/(2·a_top) − d_dec`). That is the
*authority-saturated* shape, and it fitted the pre-fix plant, whose tracking
fraction was collapsing (η 0.982 → 0.799 as the clamp truncated the brake).

The post-fix plant is **tracking-limited**: η is nearly flat at 0.951–0.968 and
the axis is using ~1.8× less braking authority than it has. Applying the old law
to it predicts **3.30 rev** of coast at the ceiling where the fitted family
predicts 0.39 — a factor of eight, and in the direction that would have kept
`END_STOP` binding for no physical reason.

So the law is now `coast_top·(v/v_top)^p`. Fitted exponents on this data:
log-log OLS on per-speed **maxima 1.50**, on means 1.31, on all 18 points 1.12;
minimax best 1.4 / 1.2 / 0.8. **Shipped p = 2.0 — above every one of them**, and
the ballistic exponent besides. The exponent is weakly identified (within-speed
scatter ±0.023 rev is a third of the whole slope across the measured span), which
is itself the argument for the pessimistic end rather than the fitted one.

**Choosing the conservative exponent was free, and that is why it was chosen.**
At p = 2.0 `END_STOP` binds at 7.468 m/s; on a strict `C·v²` envelope forced
above *every* individual measured point it binds at 6.458. Both are above the
5.637 that actually binds, so the ceiling is unchanged either way.

### The extrapolation caveat, stated plainly

The data spans **3.142–4.436 m/s**. The shipped ceiling is **5.637 m/s — 1.27×
past the fastest speed ever flown on this plant.** Two things make that
tolerable, and neither is "the fit looks good":

1. **The extrapolation does not set the ceiling.** A configured torque limit
   does. The extrapolation is load-bearing only for the *claim* that `END_STOP`
   does not bind — and that claim survives the entire fitted family (p = 0.8
   through 2.5 all clear it), including a strict upper envelope through every
   point.
2. **Extending the ladder is cheap and does not require a special session.**
   Any traced session that naturally flies higher adds a rung.

If either of those stops being true — a plant regression that makes `END_STOP`
competitive again, say — the extrapolation becomes load-bearing and must be
replaced by measurement before the ceiling is trusted.

### `DECEL_FF_HEADROOM`: a bound the first pass missed because nothing reached it

Moving the ceiling out to `DECEL_AUTHORITY`'s 5.816 m/s immediately turned
`tests/sim/test_hand_throw_decel_ff.py` red: at that speed the decel feedforward
alone wants **45.2 A of 50 — 90 %** — against C-HAND-2's requirement that it stay
under 85 %, because a saturated feedforward leaves the position/velocity loop no
authority to correct on top of it.

That was a genuine contradiction between two contracts about the same throw, and
the fix is not to relax the test. The 0.85 was a **test literal**; it is now
`hand_throw_envelope.decel_ff_current_headroom_frac` and an envelope bound, and
it is what binds. The first pass never hit it because its ceiling (4.357 m/s)
was nowhere near — a good illustration of why an envelope should enumerate
bounds rather than stop at the one that happens to be tightest today.

### Regen: the burst/steady-state distinction is the whole bound

The owner corrected two numbers on 2026-08-20: the bus is **45 V**, not the 48
I had inferred from the overvoltage trip, and the HV rail's regen capacity is
**300 W — steady-state**.

Fenced naively on instantaneous power against 300 W, `REGEN` would bind at
**5.478 m/s** and become the ceiling. That would be wrong. Braking is a burst:
a 50–90 ms decel ramp against the 3.5 s `MIN_TOSS_THROW_DELAY_S` cadence floor
is a **~2 % duty**, so the steady-state average is a few watts even at peak
instantaneous regen. The owner confirmed 360 W bursts are within the rail, so
`dc_max_negative_current = −8.0 A` is a design point, not a misconfiguration —
and the fence is the drive's 360 W burst, which binds at 6.574 m/s and does not
reach the front. Measured peak instantaneous regen at the working point is
**79.3 W**.

I also could not reproduce the 5.68 m/s figure that came with the correction; it
computes to **5.478** on the current-limit basis. The 5.68 appears to scale the
*commanded* speed by a power ratio computed at the *measured* peak speed, which
double-counts the ~3.4 % tracking shortfall. Moot now that the bound is fenced
on the burst, but recorded because the same slip would matter if it ever binds.

### The ladder is per-speed MAXIMA now, which halves what the margin pays for

The 2026-08-18 pass shipped per-tier **means** (that was all the pre-fix probe
emitted), so the modelled peak was central and the margin had to cover the
within-tier upper deviation — 0.083 rev of the 0.200, leaving **2.8 mm** of true
one-sided headroom. The rungs are now per-speed **maxima**, so the only scatter
term left is telemetry aliasing (0.029 rev), and the residual is **0.171 rev =
5.4 mm**. Academic, since `END_STOP` no longer binds, but it is why
`end_stop_margin_rev` staying at 0.20 (owner decision, 2026-08-20) is
comfortable rather than tight.

Caveat on `n`: the two lower rungs are the max of **two** throws each, which
under-states a population max. The rung that anchors the extrapolation has
n = 14.

**The 70-clean-tosses inference from the 2026-08-18 pass is now moot** — and
worth a line because it was inconclusive by construction. It computed that the
tosses were either strong (≈98 %) or negligible (≈48 %) evidence against the
pre-fix model depending on an unmeasured scatter-scaling law, and said so. The
measurement settled it directly: the pre-fix model over-stated coast at that
speed by 3.1×. Calculating the inference was still right — it was the strongest
statement the existing data could make — but it never could have replaced the
capture.


### Why the coast model is an interpolated ladder and not a fitted curve

I tried the parametric form first, because a config table of measurements is an
unusual thing to put in `hardware_config.yaml`. The physical model is
`a_ach = (J_ff/J_true)·a_cmd + a_loop`, two parameters, and it fits the four
tiers with slope 0.700 and intercept 350 rev/s² (C-HAND-2's own regression).

**It is unusable as a bound.** A least-squares fit is *optimistic* at the top
tier (2391 vs 2330 measured) — precisely where it binds. Forcing it under all
four points instead (intercept 260.8) makes it wildly conservative in the middle:
it predicts 0.805 rev of coast at 3.969 m/s where **0.347 was measured**, a 2.3×
over-prediction, and puts the derived ceiling at **0.731 s — below the shipped
default toss.** Four points do not identify a two-parameter plant model, which
C-HAND-2 says in as many words ("the plant is **not identifiable** from the
available capture"). So the contract does not pretend they do: it interpolates
the measurement.

The interpolant choice turned out not to be load-bearing, which is the reassuring
part. Interpolating the measured *overshoot*, the *achieved deceleration*, or the
*tracking efficiency* η gives ceilings of **4.357 / 4.366 / 4.390 m/s** — a 0.03
m/s spread, 7 ms of flight. The shipped model interpolates the directly measured
overshoot, with no derived intermediate. (A running-max monotonisation handles
the one non-monotone pair, 0.074 at 2.742 m/s vs 0.063 at 3.440 m/s.)

Above the top rung the model holds the **achieved deceleration** constant at its
top-rung value. That is conservative: across the four rungs achieved decel
*grows* (912 → 1440 → 1793 → 2329 rev/s²), so assuming it stops growing is the
pessimistic reading, and it makes coast go as `v²` up there — the shape the
measurement itself has.

### A bound I deliberately did NOT enforce

C-HAND-2's honest open-loop over-brake condition is
`J_ff ≤ J_true − τ_grav/(2π·a_cmd)`, and it fails below `a_cmd ≈ 2100 rev/s²`.
Enforced literally, that is a **floor** at `v ≥ 4.125 m/s` ⇒ `T ≥ 0.845 s`,
which against the 0.887 s ceiling leaves a **42 ms band** — absurd, and it would
refuse the shipped default toss.

It is not enforced because it is the wrong requirement, for the reason C-HAND-2
already gives: the consequence of over-braking is the hand stopping *short* of
`x3` and being dragged back up — a dip, **away from the stop**. It is a stroke-
quality defect, gated at the bench by row H7.4 (`dip_below_x3 ≤ 0.100 rev`)
against a predicted 0.013–0.025 rev, not a machine-damage bound. Recording it
here because a naive "derive everything" pass produces that 42 ms slit and it
looks authoritative.

### Two declarations of one physical quantity, at two values, on purpose

The hand's reflected inertia is now declared twice:
`teensy_trajectory.throw_decel_reflected_inertia_kgm2` = **9.5e-6** and
`hand_throw_envelope.measured_reflected_inertia_kgm2` = **1.050e-5**. That looks
exactly like the drift bug this codebase keeps closing, so it is worth stating
why it is not.

The first sizes a **feedforward**, where under-declaring means under-braking, so
it must sit *below* the measurement (C-HAND-2's one-sided-safety clause). The
second sizes an **authority ceiling** `a_max = I·Kt/(J·2π)`, where over-declaring
means crediting less authority, so it must sit *at or above* it. Each is
conservative for its own use and neither is conservative for the other's. A
future tidy-up that unified them would silently make one of the two unsafe;
`test_the_declared_inertias_bracket_the_measurement_in_opposite_directions`
is what stops it, and it pins the ordering as well as the values.

### The one inferred number is now owner-stated

`dc_bus_nominal_v` was **48.0, inferred** from the drive's 50 V overvoltage trip
— flagged in the 2026-08-18 pass as the only guessed input in the contract. The
owner corrected it to **45.0** on 2026-08-20, along with the rail's 300 W
steady-state capacity. Every input to the envelope is now either a transcription
of flashed config, an owner-stated fact, or a measurement with a bag id.

### What the refusal says, and why that mattered enough to add a code

`REJECTED_THROW_ENVELOPE` is a **new** outcome rather than a reuse of
`REJECTED_FLIGHT_TIME`, and the payload carries `BOUND:numbers`
(the `REJECTED_POSITION(NO_RESPONSE)` shape). The reason is the one the
`REJECTED_TILT_CLAMP` comment already records for a sibling gate: a fail-closed
refusal that names the *wrong subsystem* routes the operator to the wrong knob.
An `END_STOP` refusal is about the hand's coast; naming the clock would send the
operator to the flight time. And the two bounds an operator actually hits want
*opposite* corrections — throw **lower** for `END_STOP`, **higher** for
`ARM_WINDOW` — so "outside the band" is worse than useless.

`REJECTED_FLIGHT_TIME` survives, narrowed to "not a positive finite number",
which is what lets `__post_init__`'s never-coerce belt still report a sign typo
as a sign typo.

## Implementation

* **`config/hardware_config.yaml`** — new section `hand_throw_envelope` (11 keys),
  registered in `HW_SECTIONS` as `HAND_ENV_` / `HandEnv`. Includes three plant
  facts that had lived only in prose and test literals until now: the hand
  ODrive's `torque_constant`, its `torque_soft_limit`, and the measured
  ball-free gravity hold current.
* **`throw_envelope.py`** — the enforcement point. Imports only `math`, the
  generated config, `hand_stroke` and `ballistics_bc`; no ROS. Reads `a_cmd`
  off the shipped stroke model rather than re-deriving `123.55·v²`. Malformed
  or empty coast ladder ⇒ `ValueError` **at import**, so a bad regenerate is
  loud at node start rather than at the first throw.
* **`toss_sequencer.py`** — one call, placed immediately after the wire-band
  check (the first point at which `event_vel_mps` is trustworthy: an 8b aim that
  hit the tilt ceiling leaves the meaningless 8a fallback behind, and refusing
  *that* against an end-stop model would name the hand for an aiming fault).
* **`sim/toss_gate.py`** — the sweep's outer flight rungs now read from the
  envelope, and the 8b advisory long-flight spots moved 0.95 → the derived
  ceiling. A sweep point the shipped FSM refuses measures nothing.
* **Docs** — new `ros_ws/docs/hand_throw_envelope.md`; C-HAND-2 gains a sibling
  cross-reference and a correction box (its "0.67 rev = 21 mm to the ~11.06 rev
  physical stop" inference assumed the old end stop and is withdrawn in place).
* **Runbook § CHECK HAND-7** — a read-first box: R0–R2 are inside the envelope
  and still climbable; **R3, R4 and R5 are not**, and must not be unlocked by
  widening the envelope to fit them. A new top rung at the envelope ceiling
  (0.887 s) replaces R5.

## Verification

See § Outcome for the full-tier gate's (date, command, result) triple.

Scoped, run 2026-08-20 (venv, `-q -p no:randomly`):

* `pytest tests/ros/ tests/motion/ tests/sim/test_toss_gate.py
  tests/sim/test_hand_throw_decel_ff.py -m "not nightly"` — **3716 passed,
  3 deselected in 533.98 s**.
* `pytest tests/motion/test_throw_envelope.py` — **38 passed in 0.55 s**.

Coast re-derived from the bags rather than taken on report:
`python tools/probes/hand_stroke_timeline.py --bag ~/Desktop/rosbags/2026-08-20_21-51-39
--json`, run 2026-08-20 → six rows, `peak_over_x3_rev` **0.1342 / 0.1315 /
0.1803 / 0.2119 / 0.1880 / 0.2153**. The clamp-change exclusion was verified
from the raw `/hand_telemetry` `iq_meas` in `2026-08-18_18-42-19` (20 s windows:
−8.87 A floor through 0–100 s, −13.71 / −16.58 / −17.39 A after).

**Not deployed.** `python config/generate_config.py` has been run and the
regenerated artifacts are in the tree; deploying needs
`colcon build --packages-select jugglebot` + relaunch. **No flash** — no firmware
reads a `HandEnv::` constant.

## Outcome

**`./run_tests.sh --full`** (every tier, `nightly` included), run **2026-08-20**:
**parallel phase 488 s rc=0, serial phase 9 passed in 40.96 s, total 532 s;
5741 tests; RESULT: PASS.**

The 2026-08-19 run of the same command on the first pass was also PASS
(5719 passed + 4 xfailed parallel, 9 passed serial, 530 s). An earlier
2026-08-18 run reported 1 failure —
`tests/motion/test_motor_guard.py::test_normal_interpolation_unchanged` — which
was a wall-clock load flake (an audit subagent was driving pytest on the same box
alongside the 4-worker phase), the same mechanism as the known
`test_decay_boundary_continuity` flake and in the same file. Verified unrelated
and green scoped: **48 passed, 2 deselected in 12.13 s**; `motor_guard` contains
zero references to `throw_envelope`, `toss_sequencer` or any `HAND_ENV_*`
constant.

**Reviewed by an independent adversarial audit on the first pass** (read-only,
~40 numbers re-derived from source). No arithmetic error, no blocking code
defect, fifteen real issues — all fixed. The four that changed substance:
the coast ladder was per-tier MEANS (now moot: it is per-speed maxima); the
margin equality pin blocked its own documented safe direction and would have
forced a flash (changed to `host >= firmware`); `toss_trim.speed_gain()`'s
docstring carried a safety argument this contract refutes (*"x3 is
velocity-independent, so a speed trim cannot move the hand toward the end
stop"* — the coast past x3 is strongly velocity-dependent, and the trim is
applied *after* the gate); and the 8b sim spots were still refused after being
"fixed", because an aimed goal releases faster than its flight time implies.

**Not deployed.** `python config/generate_config.py` has been run and the
regenerated artifacts are in the tree; deploying needs
`colcon build --packages-select jugglebot` + relaunch. **No flash** — no firmware
reads a `HandEnv::` constant, and Platform FW 3 / can-bridge FW 15 are current.

## Open questions

1. **The ceiling is 1.27× past the fastest speed ever flown on this plant**
   (5.637 m/s vs a measured span topping out at 4.436). The extrapolation is not
   what sets it — a configured torque limit is — and the `END_STOP` claim
   survives the whole fitted exponent family. Extending the ladder when a session
   naturally goes higher is the cheap way to close it. **Owner sign-off wanted on
   flying above 4.436 m/s at all**, since nothing has.
2. **`dc_max_negative_current` (360 W at 45 V) sits above the rail's 300 W
   continuous rating.** Owner-confirmed as a burst design point (2026-08-20) and
   the ~2 % duty clears it by ~60×. Recorded because a cadence-floor change or a
   continuous-throw mode would make it live.
3. **Raising the ceiling further is a drive decision, not an implementer's** —
   it needs `hand_curr_limit_a` above 50 A or
   `decel_ff_current_headroom_frac` above 0.85, and the second moves C-HAND-2
   too.
4. **The two lower coast rungs are the max of two throws each.** The rung that
   anchors the extrapolation has n = 14; the others do not.
5. **`catch_vel_scale` is not an envelope input**, and moving the floor made that
   corner worse: the closing knob value went 0.45 → 0.659 against a shipped
   default of 0.9 and an operator floor of 0.3 (safety ratio 1.78× → 1.36×).
   Closing it means plumbing the goal's `catch_vel_scale` into the FSM — a
   `reload_coordinator_node` change, out of scope here.
