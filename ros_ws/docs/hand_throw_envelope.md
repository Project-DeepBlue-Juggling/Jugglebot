# C-HAND-3 — the derived throw-admission envelope

**Status:** normative. Landed 2026-08-18, replacing the hand-picked
`FLIGHT_TIME_MIN_S = 0.55` / `FLIGHT_TIME_MAX_S = 1.10` literals in
`toss_sequencer.py`.
**Sibling contracts:** `ros_ws/docs/hand_command_continuity.md` (**C-HAND-1**),
which governs *when* a hand command may be dispatched, and
`ros_ws/docs/hand_decel_feedforward.md` (**C-HAND-2**), which governs *how hard
the throw brakes once it has been*. This one governs **which throws may be
dispatched at all**.

## The contract

> **C-HAND-3.** A throw shall be admitted only if, at the release speed the hand
> will actually be commanded to, **every** one of the following holds: the
> modelled post-release peak hand position clears the mechanical end stop by the
> declared margin; the commanded deceleration and the commanded ball-carrying
> acceleration are both within the drive's torque authority; the decel
> feedforward alone leaves the control loop its declared current headroom; the
> peak burst braking power at release is within the drive's regen budget; the release speed is inside the
> bridge's acceptance band; and the catch-arm window is still open by the
> declared margin. Every quantity in that test shall be a configured value or an
> algebraic consequence of one — **no hand-picked flight-time or throw-height
> boundary may exist anywhere in the admission path.** A refusal shall name the
> binding bound and quote the computed envelope.

**One enforcement point:** `jugglebot/motion/trajectory/throw_envelope.py`
`evaluate(flight_time_s, release_speed_mps)`, called exactly once — from
`toss_sequencer.TossSequencer.step`'s CHECKING block, immediately after the
bridge-band check.
**One declared input set:** `config/hardware_config.yaml` §
`hand_throw_envelope` (plus `jugglebot_geometry.hand_motor_hard_stop_revs`,
`jugglebot_odrive_defaults.hand_curr_limit_a` and the `teensy_trajectory`
stroke constants it derives the profile from).
**Tests that fail without it:** `tests/motion/test_throw_envelope.py`, plus the
band rows in `tests/ros/test_toss_sequencer.py`,
`tests/ros/test_toss_coordinator.py`, `tests/motion/test_hand_stroke.py`,
`tests/sim/test_hand_throw_decel_ff.py` and `tests/sim/test_toss_gate.py`.
**Deployment:** `python config/generate_config.py` +
`colcon build --packages-select jugglebot` + relaunch. **No flash** — no firmware
reads any of these keys; the generated `HandEnv::` C++ namespace is inert.

## The root cause it closes

`toss_sequencer` shipped two literals:

```python
FLIGHT_TIME_MIN_S = 0.55   # plan sweep floor (throw speed ≈ 2.7 m/s)
FLIGHT_TIME_MAX_S = 1.10   # plan sweep ceiling (≈ 5.4 m/s < 7.0 Teensy ceiling)
```

The ceiling's own comment names its justification: it is below the Teensy's
`MAX_EVENT_VEL_MPS` builder clamp. C-HAND-2 sized it more carefully than that —
at 1.10 s the commanded deceleration is 3597 rev/s², **83–86 %** of the axis's
4178–4333 rev/s² ceiling — but that is a bound on the **commanded** profile. It
says nothing about the **uncommanded** part.

**The commanded profile never leaves `x3` = 9.9594 rev.** What reaches for the
end stop is the ballistic coast past `x3`, driven by the fraction of the
commanded deceleration the plant fails to achieve, and it grows steeply with
release speed. Measured 2026-07-27:

| commanded release | coast past `x3` | measured peak |
|---|---|---|
| 2.742 m/s | +0.074 rev | 10.033 rev |
| 3.440 m/s | +0.063 rev | 10.022 rev |
| 3.969 m/s | +0.347 rev | 10.306 rev |
| 4.858 m/s | **+1.020 rev** | **10.980 rev** ← light physical contact |

Nothing bounded that, because **the end stop's declared location was wrong**.
`hand_motor_hard_stop_revs` read **11.1** — 0.3 rev *past* metal — from before
the sensorised hand was fitted until 2026-08-18, when the operator measured
metal contact at **10.8** (`logbook/2026-08-18-hand-end-stop-corrected.md`,
commit `3760daa`). Against 11.1 the 4.858 m/s tier read as headroom. Against
10.8 it is 0.18 rev **past** the stop, and it is the contact the operator felt.

So the failure class is not "the ceiling was slightly too high". It is: **an
admission gate whose only physical input was one of the several limits that
actually bind, sitting downstream of a geometric constant that was wrong in the
unsafe direction.** The fix has to enumerate the limits and derive the gate from
all of them, or the next wrong constant produces the same class of failure
somewhere else.

## The bounds

Notation: `v` = commanded release speed (m/s); `v_rev = v · G` with
`G = LINEAR_GAIN_REV_PER_M` = 31.6172 rev/m; `T` = flight time (s).

### The stroke landmarks all three bounds hang off

Both are **velocity-independent**, which is a property of `calcThrow`'s algebra,
not a coincidence:

| landmark | value | why velocity-independent |
|---|---|---|
| `x2` (ball release) | **5.9138 rev** | `x1 = accelSt/(1+IR)` and `v·t_vel = velHold`; neither carries `v` |
| `x3` (stroke top) | **9.9594 rev** | `x3 = totalStroke` identically |
| `d_dec = x3 − x2` | **4.0456 rev** | `IR·accelSt/(1+IR)` |

`d_dec` is the whole reason a tracking shortfall becomes end-stop travel:
`calcThrow` allocates the *ideal* stopping distance and ends exactly on the top
of the usable stroke, with **zero** allowance for error.

And one identity the contract leans on throughout:

```
v_rev² / (2 · a_cmd(v))  =  d_dec       exactly, at every v
```

because `a_cmd(v) = G(1+IR)/(2·accelSt·IR) · v²` = 123.55·v² rev/s². So
`peak = x2 + v_rev²/(2·a_ach)` and `over = d_dec·(a_cmd/a_ach − 1)` are the same
statement, and both are exact restatements of the measurement.

### B1 — `END_STOP`. The bound the old constants missed.

```
x3 + coast(v)  ≤  hand_motor_hard_stop_revs − end_stop_margin_rev
             i.e.  peak(v) ≤ 10.8 − 0.2 = 10.60 rev
```

`10.60` coincides today with two other lines — the firmware's
`smooth_move_excursion_margin_rev` clamp ceiling, and the runbook's
`peak > 10.60 + E-STOP` rows in § CHECK HAND-4 and § CHECK HAND-7 — but that is
a **coincidence, not a design**, and this contract does not claim otherwise.
`3760daa` moved the firmware key 0.5 → 0.2 for the sole purpose of holding 10.60
fixed when the stop moved 11.1 → 10.8; the firmware's 0.2 is a residual. The two
keys guard genuinely different things — the firmware constant bounds a
**computed** smooth-move excursion the Teensy checks exactly, this one bounds a
**stochastic** ballistic excursion nothing checks in flight — so the pinned
invariant is the *relation*, `host ≥ firmware`, not equality
(`test_the_host_coast_margin_is_never_looser_than_the_firmware_clamp`). Equality
would have been a trap: both this document and the YAML say raising the host
margin is the safe direction, and an equality pin makes that edit red — with the
obvious way to green it (raise the firmware key too) requiring a **flash**.

#### The coast ladder — measured on the flashed plant (2026-08-20)

Each rung is that speed's **MAXIMUM** observed coast, not its mean:

| release | apex | n | coast max (rev) | source |
|---|---|---|---|---|
| 3.142 | 0.5 m | 2 | **0.1342** | bag `2026-08-20_21-51-39` |
| 3.714 | 0.7 m | 2 | **0.2119** | bag `2026-08-20_21-51-39` |
| 4.436 | 1.0 m | 14 | **0.2260** (min 0.1796, mean 0.2079) | + bag `2026-08-18_18-42-19` |

Re-derive any row with
`python tools/probes/hand_stroke_timeline.py --bag ~/Desktop/rosbags/<id> --json`.

**One throw is excluded, and it is the most informative datum in the set.**
Bag `2026-08-18_18-42-19` contains a **clamp change mid-session**: braking
`iq_meas` never passes −8.87 A through t = 0–100 s (the −10.00 A
`torque_soft_min` still live), then reaches −13.71 / −16.58 / −17.39 A from
t = 100 s on. Its **first** throw, at t = 84.9 s, coasted **+0.763 rev** (peak
10.7222); every other throw in the same bag, at the **identical** commanded
4.436 m/s, coasts 0.18–0.23. Same command, same firmware, **3.8× the coast**.
That within-session A/B is the direct evidence that **the clamp was the coast
mechanism**. That throw is pre-fix data and must never be readmitted.

#### What the margin now buys

With maxima rather than means, the only scatter term left is telemetry aliasing:

| term | rev | mm |
|---|---|---|
| telemetry aliasing (`½·a_top·(5 ms)²`, `a_top` = 2302 rev/s²) | 0.029 | 0.9 |
| **residual headroom** | **0.171** | **5.4** |
| `end_stop_margin_rev` (**frozen at 0.20 by owner decision, 2026-08-20**) | 0.200 | 6.3 |

Up from 2.8 mm under the old mean-based ladder — and academic, because
`END_STOP` no longer binds. Caveat on `n`: the two lower rungs are the max of
**two** throws each, which under-states a population max; the rung that anchors
the extrapolation has n = 14.

#### Above the top rung: an extrapolation, stated plainly

`coast(v) = coast_top · (v/v_top)^p` with **p = 2.0**.

> ⚠ ~~**The data spans 3.142–4.436 m/s (1.41×). The shipped ceiling is 5.637 m/s
> — 1.27× past the fastest speed ever measured on this plant.**~~
>
> ✅ **DISCHARGED BY MEASUREMENT 2026-08-21.** The operator flew the envelope to
> its ceiling: `2026-08-21_10-11-42`, throws at 4.436 / 4.858 / 5.246 / 5.608 ×2
> m/s — the last **0.029 m/s under the 5.637 bound**. Every throw was ACCEPTED by
> the gate and none was truncated. Measured coast:
>
> | v_cmd | coast past x3 | headroom to the 10.8 rev stop |
> |---|---|---|
> | 4.436 | 0.200 rev | 0.641 rev |
> | 4.858 | 0.212 | 0.629 |
> | 5.246 | 0.213 | 0.628 |
> | **5.608** | **0.215 / 0.250** | **0.590 rev = 18.7 mm** |
>
> **The model is conservative by ~40 % at the ceiling**: p = 2.0 predicts 0.361 rev
> of coast at 5.608 m/s; measured is 0.215–0.250. Coast is close to FLAT across the
> whole 3.142 → 5.608 m/s span (0.134 → 0.250, ratio 1.87 against a speed ratio of
> 1.78, i.e. **p ≈ 1.0**), which is what a tracking-limited plant with 1.8× unused
> braking authority should do.
>
> **p = 2.0 is deliberately RETAINED** — it is now measured-conservative rather than
> assumed-conservative, and lowering it would buy ceiling the machine cannot use
> (`DECEL_FF_HEADROOM` binds first regardless). Do not "correct" it to the fit.
>
> The only drops that session were the 1.6 m throws, whose balls struck the ROOM's
> ceiling — a trajectory disturbance outside the machine, not an envelope failure.

Fitted exponents (computed 2026-08-20): log-log OLS on per-speed **maxima 1.50**,
on means 1.31, on all 18 individual points 1.12; minimax best 1.4 / 1.2 / 0.8.
**2.0 is above every one of them.** The exponent is weakly identified —
within-speed scatter (±0.023 rev at 4.436) is a third of the whole slope across
the measured span (0.134 → 0.226) — which is itself the reason to take the
pessimistic end rather than the fitted one. 2.0 is also the ballistic exponent:
what coast would do if the tracking shortfall were a fixed fraction of a `v²`
commanded decel.

**And it costs nothing, which is what makes the extrapolation tolerable.** At
p = 2.0 `END_STOP` first binds at **7.468 m/s**, and on a strict `C·v²` envelope
forced above *every* measured point (C = 0.015362, set by the 3.714 outlier) at
**6.458 m/s** — both above the 5.637 m/s that actually binds. **So the
extrapolation does not set the ceiling; a configured torque limit does.** It is
load-bearing only for the *claim* that `END_STOP` does not bind, and that claim
survives the whole fitted family (p = 0.8 → 2.5 all clear it).

#### Why not the old "hold achieved deceleration constant" law

That law (`coast = v_rev²/(2·a_top) − d_dec`) is the **authority-saturated**
shape, and it fitted the pre-fix plant, whose tracking fraction was collapsing
(η 0.982 → 0.799). The post-fix plant is **tracking-limited** with η nearly flat
(**0.968 / 0.954 / 0.951** across the measured span) and ~1.8× the braking
authority it is using. Asserting that its achieved deceleration saturates at the
measured 2302 rev/s² would be physically wrong, and wildly so: it predicts
**3.30 rev** of coast at the ceiling where the fitted family predicts 0.39.

**Result: `v ≤ 7.468 m/s` — non-binding.**

### B2 — `DECEL_AUTHORITY`

```
a_cmd(v) = 123.55·v²  ≤  min(I_max·Kt, τ_soft) / (J · 2π)
         = min(50.0 × 0.0055133, 0.7) / (1.050e-5 × 2π)
         = 0.27567 N·m / 6.5973e-5
         = 4178 rev/s²
```

The **current** limit binds; `torque_soft_limit` (±0.7 N·m since 2026-08-18) is
2.5× looser and is carried so that a regression back to the old asymmetric
`torque_soft_min = −0.055133 N·m` (= exactly −10.00 A) shows up as a 2.5×-tighter
envelope instead of silently truncating the brake again. Gravity is **excluded**:
on an upward deceleration it brakes in the same direction, so ignoring it
under-states authority — the conservative direction.

`J = 1.050e-5` is the **larger** of C-HAND-2's two identifications (the
regression; the decel-side torque balance bounds it at ≥ 1.0126e-5). Larger `J`
means *less* authority, so the larger value is conservative here — the opposite
of `throw_decel_reflected_inertia_kgm2`, where the *smaller* value is
conservative because it sizes a feedforward. **The same physical quantity is
declared twice, at two different values, on purpose**; the two keys' comments
each say why, and
`test_the_declared_inertias_bracket_the_measurement_in_opposite_directions` pins
the ordering.

**Result: `v ≤ 5.816 m/s`.** Not the binding bound — `DECEL_FF_HEADROOM` is.

### B2b — `DECEL_FF_HEADROOM`. **What actually binds.**

The feedforward is sized on the *declared* `J_ff` = 9.5e-6, so it draws
`J_ff·2π·a_cmd/Kt` amps. C-HAND-2 requires that to leave the loop room — a
saturated feedforward has no authority left to correct anything on top of
itself, and raising `hand_curr_limit_a` is an operator decision:

```
J_ff·2π·a_cmd/Kt  ≤  decel_ff_current_headroom_frac · I_max  =  0.85 × 50 = 42.5 A
⇒ a_cmd ≤ 3925 rev/s²
```

**Result: `v ≤ 5.637 m/s`** ⇒ `T ≤ 1.1485 s` ⇒ apex `≤ 1.617 m`. **This is the
ceiling.**

The 0.85 was a *test literal* in `tests/sim/test_hand_throw_decel_ff.py` until
2026-08-20. It had to become a config key and an envelope bound the moment the
measured coast ladder moved the ceiling out to 5.816 m/s, because the
feedforward there wants **45.2 A of 50 — 90 %** — i.e. the envelope would
otherwise have admitted a throw C-HAND-2 forbids. Two contracts disagreeing
about the same throw is the failure this key closes.

### B3 — `ACCEL_AUTHORITY`

The ascent carries the ball and fights gravity:

```
a_acc(v) = IR · a_cmd(v) ≤ (I_max·Kt − τ_grav/IR) / (J_ascent · 2π)
J_ascent = J + m_ball/(2π·G)²  = 1.050e-5 + 2.412e-6 = 1.291e-5 kg·m²
m_ball   = m_hand·(1/IR − 1)   = 0.0952 kg     (IR ≡ m_hand/(m_hand+m_ball))
τ_grav   = 1.50 A × Kt / IR    = 0.01107 N·m   (ball-free hold, mass-corrected)
```

**Result: `v ≤ 5.945 m/s`** — above B2, so the decel side always binds at the
top. That is not a coincidence to be assumed: the ascent torque is
`IR·(J_ascent/J)` = 0.747 × 1.2297 = **0.919** of the decel torque at the same
speed, plus a constant 0.0111 N·m of gravity, so the two cross at 4.09 m/s and
above that the decel side is always the larger demand. Below 4.09 m/s the ascent
is the larger demand but both are far inside the limit (24.7 A at the crossing).
Evaluated rather than assumed, because the crossing moves if the ball mass or
`INERTIA_RATIO` ever changes.

### B4 — `REGEN`

Braking power peaks at release, where `ω` is largest:

```
τ_brake · ω(v)  ≤  |dc_max_negative_current| · V_bus  =  8.0 A × 45.0 V = 360 W
```

**Result: `v ≤ 6.574 m/s` — non-binding.**

**The fence is the BURST, not the rail's steady-state rating.** The HV rail's
capacity is **300 W steady-state** (owner, 2026-08-20), and 360 W bursts are
explicitly within it — so `dc_max_negative_current = −8.0 A` is a design point,
not a misconfiguration. Braking is a burst: the decel ramp is 50–90 ms, once per
cycle. Fencing the instantaneous peak against a continuous rating would tighten
the envelope on a duty the machine never runs.

> ⚠ **The duty cycle moved on 2026-08-22 and this paragraph was re-derived.** It
> read "against the 3.5 s `MIN_TOSS_THROW_DELAY_S` cadence floor, a **~2 % duty**,
> so the steady-state average is a few watts". That floor is retired (census A1)
> and the cadence ladder's operating point is a **0.985 s cycle period** (dwell
> 0.49 s at flight 0.4949 s, ~61 throws/min), so the duty is **5.1–9.2 %** and the
> average at the 360 W fence is **18–33 W** against the rail's 300 W. It still
> clears, by ~9–16× instead of ~60×. **This is the one number the cadence work
> makes monotonically worse — re-check it before any rung faster than R5-prime.**

Measured at the 1.0 m working point: peak instantaneous regen **79.3 W**
(132.1 rev/s × −17.32 A × Kt, bag `2026-08-20_21-51-39`) — 26 % of the
steady-state rating, as an *instantaneous* figure.

`max_regen_current` is 0.0 (no brake resistor), so the rail is where regen goes.
The bus voltage is **45 V, owner-stated**; it read an inferred 48.0 until
2026-08-20.

### B5 — `WIRE_BAND`

`teensy_bridge_node._svc_set_hand_traj` raises outside
`[0.3, 7.0] m/s` before any CAN frame exists. `toss_sequencer` gates on it one
line earlier so the operator gets `REJECTED_EVENT_VEL` and routes at the wire;
the copy inside `evaluate` exists so any *future* caller inherits the whole
envelope.

### B6 — `ARM_WINDOW`. What actually sizes the floor.

```
window(T, v) = [T − max(0.3, required_arm_lead(v_armed))]
             − [throw_decel(v) + ARM_SUPPRESS_MARGIN_S]
             ≥ arm_window_margin_s
```

Both edges are `hand_stroke`'s: the left is when the throw stroke has provably
cleared, the right is the Teensy's `:533` budget check. The window narrows
monotonically toward short flights and **closes entirely at T = 0.4542 s** —
below that the catch arm cannot be placed at all and the ball flies uncatchable.

`arm_window_margin_s = 0.050` is the **same number the bench gates every withheld
arm on** (runbook row H1.5, `slack > 0.050 s`) — one number, two surfaces. That
is its *only* job, and the consequence has to be stated: since the floor is
defined as the flight where the window equals this margin, **a goal admitted at
the floor sits exactly on H1.5's gate**. It is a bench boundary, not a
comfortable operating point.

An earlier draft of this section also claimed the margin "covers the ~23 ms
announcement-to-release transit that `required_arm_lead_s` documents as
excluded". **It cannot do both jobs.** Subtract that transit from a 50 ms window
and the real slack at the floor is ~27 ms, which fails H1.5 outright. The
honest reading is the first one: the margin equals the bench gate, the transit is
still excluded, and short flights near the floor are where that bites.

`v_armed` is the flight's **vertical** arrival speed times
`catch_vel_scale_default`, so it is identical for a displaced Tier-8b throw of
the same flight time: a horizontal launch component changes neither the vertical
launch component nor the fall.

**Result: `T ≥ 0.4949 s`** ⇒ apex `≥ 0.300 m`.

## What the envelope comes out as

| | shipped (hand-picked) | derived (C-HAND-3) | change |
|---|---|---|---|
| flight time | 0.55 – 1.10 s | **0.4949 – 1.1485 s** | floor −55 ms, ceiling **+49 ms** |
| apex height (`throw_height_m`) | 0.371 – 1.483 m | **0.300 – 1.617 m** | floor −71 mm, ceiling **+134 mm** |
| release speed | 2.709 – 5.399 m/s | **2.440 – 5.637 m/s** | floor −0.27, ceiling **+0.24 m/s** |

**Both edges now sit outside the old hand-picked band, and the binding bound is
`DECEL_FF_HEADROOM`** at 5.637 m/s, just ahead of `DECEL_AUTHORITY`'s 5.816.
`END_STOP` does not bind until 7.468 m/s.

Binding order, which is the contract's whole claim:

| bound | binds at | |
|---|---|---|
| **`DECEL_FF_HEADROOM`** | **5.637 m/s** | ← the ceiling |
| `DECEL_AUTHORITY` | 5.816 m/s | |
| `ACCEL_AUTHORITY` | 5.945 m/s | |
| `REGEN` | 6.574 m/s | burst fence |
| `WIRE_BAND` | 7.000 m/s | |
| `END_STOP` | 7.468 m/s | 6.458 on a strict upper envelope |

| flight | apex | release | modelled peak | vs 10.8 rev stop |
|---|---|---|---|---|
| 0.4949 s | 0.30 m | 2.440 m/s | 10.094 rev | 22.3 mm clear ← floor |
| 0.80 s (default) | 0.78 m | 3.931 m/s | 10.137 rev | 21.0 mm clear |
| 0.9032 s | 1.00 m | 4.436 m/s | 10.185 rev | 19.4 mm clear ← working point, n = 14 |
| 1.10 s (old ceiling) | 1.48 m | 5.399 m/s | 10.294 rev | 16.0 mm clear |
| **1.1485 s** | 1.617 m | 5.637 m/s | **10.324 rev** | **15.0 mm clear** ← the ceiling |

The 2026-08-18 draft of this contract put the ceiling at **0.887 s** and refused
the 1.0 m working point, because it modelled coast from the pre-fix ladder. The
measurement replaced that: coast at 4.436 m/s is **0.226 rev, not the 0.700 the
pre-fix model extrapolated**.

## What changed on 2026-08-20, and what the measurement settled

The 2026-08-18 draft of this contract shipped a ceiling of **0.887 s** and
**refused the 1.0 m working point the machine flies every session**. It argued
at length that the pre-fix coast ladder should not be replaced by a *model* of
the flashed plant, however plausible, because the model was unmeasured. That
argument was right, and the answer was always going to be one capture.

The capture happened (2026-08-20, six throws, all caught) and it settled three
things:

1. **The clamp was the coast mechanism.** The within-session A/B in
   `2026-08-18_18-42-19` — 0.763 rev clamped vs 0.18–0.23 unclamped at the
   identical commanded speed — is direct, not inferential.
2. **The post-fix plant is tracking-limited, not authority-limited.** η is
   nearly flat at 0.951–0.968 where pre-fix it collapsed 0.982 → 0.799. That
   invalidated the old extrapolation law, not just the old numbers.
3. **`END_STOP` stops binding**, and the ceiling returns to torque — which is
   what the owner's directive asked for: *the limits of the system should be
   determined from the configured accel/velocity/torque/timing limits*.

The 2026-08-18 draft also reasoned about whether the 70 clean tosses at the 1.0 m
working point were evidence the pre-fix model over-stated coast, and concluded it
depended on an unmeasured scatter-scaling law (98 % vs 48 % chance of a contact).
**It did over-state**: measured coast there is 0.226 rev, not 0.700, and the
peak is 19.4 mm from metal rather than 4.4. The inference was inconclusive and
correctly labelled as such; the measurement was decisive.

### The superseded ladder, kept so nobody reinstates it

| release | pre-fix coast (2026-07-27 group **means**) |
|---|---|
| 2.742 | 0.074 |
| 3.440 | 0.063 |
| 3.969 | 0.347 |
| 4.858 | **1.020** ← touched metal |

Captured with the legacy decel feedforward **and** the −10.00 A
`torque_soft_min` live. On that ladder a 1.10 s toss modelled to **12.17 rev**
against a 10.8 rev stop, and `END_STOP` set the entire envelope. **Do not mix
rungs from the two ladders** — `test_the_coast_ladder_is_the_post_fix_measurement`
fails if a pre-fix rung is readmitted.

## What would move this envelope

The ceiling is now `DECEL_FF_HEADROOM`, so the levers are different from before:

* **Raising it needs an operator decision on the drive**, not a fresh capture:
  either `hand_curr_limit_a` above 50 A, or `decel_ff_current_headroom_frac`
  above 0.85 — and the second is C-HAND-2's own requirement, so it moves that
  contract too. Neither is an implementer's call.
* **Extending the coast ladder past 4.436 m/s** would shrink the extrapolation
  (currently 1.27×) but would *not* move the ceiling, because `END_STOP` is not
  what binds. It is worth doing when a session naturally flies higher — the
  ladder is where the evidence lives — but it buys confidence, not height.
* **Lowering `end_stop_margin_rev`** is not available: frozen at 0.20 by owner
  decision, 2026-08-20.

## Scope — what this contract does NOT touch

| surface | state | why |
|---|---|---|
| any firmware | **untouched** | no firmware reads a `HandEnv::` constant; the emitted namespace is inert. Deploying is codegen + `colcon build` + relaunch |
| ~~`MIN_TOSS_THROW_DELAY_S = 3.5`~~ | **retired 2026-08-22**, outside this contract | it was out of scope here and stayed so: C-HAND-3 bounds *how big* a throw may be, the cadence work bounds *when* the next one may fire. The retirement (census A1) replaced it with a 0.10 s goal-storm debounce plus the derived `hand_stroke.min_throw_event_delay_s`, and it did not move any bound on this page |
| `throw_decel_reflected_inertia_kgm2` (9.5e-6) | untouched | it sizes the FIRMWARE feedforward and is deliberately LOW; this contract's `measured_reflected_inertia_kgm2` (1.050e-5) sizes a HOST authority ceiling and is deliberately HIGH. Both are conservative *for their own use*, and the pair is pinned ordered |
| the catch-side `catch_vel_scale` knob | **not an envelope input** | the floor is sized against `catch_vel_scale_default`. A knob at its 0.3 floor can still close the arm window at an admitted flight — the runbook's H1.4 corner, unchanged. Closing it means plumbing the goal's `catch_vel_scale` into the FSM, which is a coordinator change |
| Tier-8b displacement gates | untouched | orthogonal — they bound *where*, this bounds *how hard*. **But note the interaction:** the reported flight band is the Tier-8a CO-LOCATED projection, and an 8b goal is AIMED, so it releases faster than its flight time alone implies. At the 8a ceiling a 50 mm displacement already lifts the commanded release from 4.35683 to 4.35742 m/s and is refused. **Tier 8b's usable ceiling is therefore strictly below the reported band**, by an amount that grows with displacement — which is precisely why `evaluate` takes the commanded speed and not the flight time |
| commanded positions/velocities/torques | **bit-identical** | this is an admission gate. A throw it admits is byte-for-byte the throw the machine made before |
| `REJECTED_FLIGHT_TIME` | **narrowed, not removed** | it now means "the flight time is not a positive finite number". Keeping it is what lets a sign typo still report as a sign typo instead of as an end-stop fault |

## Failure vocabulary

`REJECTED_THROW_ENVELOPE(<BOUND>:<numbers>)`, where `<BOUND>` is one of
`END_STOP`, `DECEL_AUTHORITY`, `DECEL_FF_HEADROOM`, `ACCEL_AUTHORITY`,
`REGEN`, `WIRE_BAND`, `ARM_WINDOW`, `INPUT`. Bounds are evaluated machine-damage-first, so when several
fail the operator hears about the one that breaks metal.

The detail string always carries the offending quantity, the derived limit, and
the units — never a bare "too high", because "too high" sends an operator to the
wrong knob (throw *lower* for `END_STOP`, *higher* for `ARM_WINDOW`).

## Open questions

* **The ceiling is 1.27× past the fastest speed ever measured on this plant.**
  The extrapolation is not what sets it (a configured torque limit is), and the
  `END_STOP` claim survives the whole fitted exponent family — but nothing has
  been flown above 4.436 m/s post-fix. Extending the ladder when a session
  naturally goes higher is the cheap way to close it.
* **`dc_max_negative_current` is configured above the rail's continuous rating**
  (360 W vs 300 W). Owner-confirmed as a burst design point, 2026-08-20, and the
  ~2 % duty cycle clears it by ~60×. Recorded because a future change to the
  cadence floor or to a continuous-throw mode would make it live.
* **The two lower coast rungs are the max of two throws each.** The rung that
  anchors the extrapolation has n = 14; the others do not.
* **`catch_vel_scale` is not an envelope input.** The floor is sized against
  `catch_vel_scale_default`, so a knob at its 0.3 floor can still close the arm
  window at an admitted flight (runbook H1.4). Closing it means plumbing the
  goal's `catch_vel_scale` into the FSM — a coordinator change.
