# C-HAND-2 — the hand's post-release deceleration feedforward

**Status:** normative. Landed 2026-07-29 (plan
`plans/archived/hand-command-continuity.md` Phase 7; follow-on item D of
the anomaly-fixes orchestration prompt — completed and deleted 2026-08-15 per the
prompt-deletion convention; the item's narrative lives in
`logbook/2026-07-29-hand-post-release-decel.md`).
**Sibling contracts:** `ros_ws/docs/hand_command_continuity.md` (**C-HAND-1**),
which governs *when* a hand command may be dispatched, and
`ros_ws/docs/hand_throw_envelope.md` (**C-HAND-3**, 2026-08-18), which governs
*which throws may be dispatched at all*. This one governs *how hard the throw
stroke brakes once it has been*.

> ### ⚠ Four things in this document moved after it was written
>
> 0. **Two rows of the founding measurement table were measuring a different
>    event**, and the re-derivation the 2026-08-18 clamp removal ordered has now
>    been done. Read § *The re-derivation on the UNCLAMPED drive* before using
>    any number below: the declared 9.5e-6 is no longer known to sit *under* the
>    measured inertia, and the evidence points the other way. Nothing about the
>    contract's *statement* changes; what changed is which side of it the plant
>    is on.
>
> 1. **`FLIGHT_TIME_MAX_S` is no longer 1.10 s.** Every "band ceiling" below was
>    written at 1.10 s and is left at that value, because each is a statement
>    about a *speed* and re-reading it at a moved band edge would silently
>    restate it about a different machine. The shipped ceiling is now DERIVED
>    (C-HAND-3) and is **0.887 s**.
> 2. **The negative torque clamp is GONE.** § *The negative torque clamp —
>    unresolved* below describes `torque_soft_min = −0.055133 N·m` (= −10.00 A)
>    against a `torque_soft_max` of +0.5 as the live configuration. Since
>    2026-08-18 they are a symmetric **±0.7 N·m**, saved to the drive — runbook
>    row H7.0c is now a regression check, not an open question. **And whether it
>    was ever *binding* on the achieved deceleration is no longer open either —
>    it was** (2026-08-23). The A/B is two sittings running the SAME corrected
>    feedforward at the SAME 4.436 m/s tier: with the clamp live the throw's coast
>    peaked **+0.719 rev past `x3`** (31/31 tosses), with it gone **−0.279 rev**
>    (3/3). The kinematic argument in that section — that a hard clamp would force
>    one achieved decel per tier — is the same argument the probe retracted on
>    2026-08-18, and it is wrong for the same reason.
> 3. **The end stop is 10.8 rev, not the ~11.06 this document infers.** It was
>    declared 11.1 — 0.3 rev *past* metal — until the operator measured metal
>    contact on 2026-08-18 (`logbook/2026-08-18-hand-end-stop-corrected.md`).
>    Every margin quoted below against "the physical stop" is therefore **0.26
>    rev (8 mm) more optimistic than the truth**; the margins quoted against the
>    **10.60 rev** runbook line are unaffected, because that line did not move.
>
> Neither correction changes this contract's own claim — the feedforward sizing
> is unaltered — but both change what may be concluded from it about how high
> this machine may throw. That question moved to C-HAND-3.

## The contract

> **C-HAND-2.** The torque feedforward of a throw stroke's post-release
> deceleration segment shall be sized from the hand axis's **total reflected
> inertia at the motor** (rotor + cable-driven load), and that declared inertia
> shall be **less than or equal to** the measured reflected inertia. Two
> obligations follow: the deceleration the profile commands is a deceleration
> the drive is actually asked to produce, and the feedforward alone can never
> brake harder than the commanded profile — so the hand can never be commanded
> to stop short of `x3` and then be dragged back up to it.

**One enforcement point:** `Teensy_code_platform/Trajectory.h::throwDecelToTorque`,
consumed by exactly one caller — `buildThrow`'s `torA[2]`.
**One declared value:** `config/hardware_config.yaml`
`teensy_trajectory.throw_decel_reflected_inertia_kgm2`.
**Tests that fail without it:** `tests/sim/test_hand_throw_decel_ff.py` (**14
test functions, 22 collected cases** — `pytest tests/sim/test_hand_throw_decel_ff.py
--collect-only -q`, run 2026-08-18; the 14th arrived with C-HAND-3) and
`tests/firmware/test_hand_throw_decel_xref.py` (7 cases against the shipped
`Trajectory.h`, compiled and run with `g++ -Wall -Wextra -Werror`).
Empirical probe behind every number here:
`tools/probes/hand_decel_authority.py`.

## The root cause it closes

`accelToTorque(a) = a · INERTIA_HAND_ONLY_KG · HAND_SPOOL_RADIUS_M` models the
hand axis as a **pure translating mass on a spool**. Two things are missing:

1. **the motor's own rotor inertia**, entirely; and
2. the **effective** radius. One motor revolution moves
   `2π·HAND_SPOOL_RADIUS_M / LINEAR_GAIN_FACTOR` = 31.628 mm of cable, so the
   effective radius is 5.0337 mm, not the 5.21 mm the expression uses.

The reflected inertia the historical expression implies is therefore

```
J_ff = m·r / (2π·LINEAR_GAIN) = 0.281 × 0.00521 / 198.6588 = 7.3695e-6 kg·m²
```

against a **measured 1.0126e-5 – 1.050e-5 kg·m²**. The braking feedforward was
~70 % of the torque the commanded deceleration physically requires, and the
30 % shortfall has to be closed by a position/velocity loop that cannot do it in
time: `hand_pos_gain = 35` rev/s per rev, `hand_vel_gain = 0.007` Nm/(rev/s)
(= 1.27 A/(rev/s) at the hand's `torque_constant` 0.0055133), and
`hand_vel_int_gain = 0.07`, i.e. an integrator unwind constant of
`vel_gain/vel_int_gain = 0.100 s` — **1.1 to 2.1× the entire 47.4–93.3 ms
deceleration ramp**.

### Why the geometry gives the shortfall a 4.046 rev lever arm

`calcThrow` allocates the decel `0.5·v·t_dec` metres of travel, and with
`t_dec = INERTIA_RATIO·t_acc` and `t_acc = 2·accelSt/((1+IR)·v)` that reduces to
`0.5·IR·accelSt·2/(1+IR)` — **velocity-independent, 4.046 rev at every speed**,
ending exactly on `x3 = totalStroke`, the top of the usable stroke. So the
profile allocates precisely the ideal stopping distance with **zero** allowance
for tracking error, and

```
peak = x3 + 4.046 · (1/η − 1)          η ≡ achieved decel / commanded decel
```

Measured on the 2026-07-27 sitting (`logbook/2026-07-28-anomaly-fixes-validation-sitting.md`):

| commanded release | commanded decel | measured peak | over `x3` | implied η | ⚠ |
|---|---|---|---|---|---|
| 2.742 m/s | 929 rev/s² | 10.033 rev | +0.074 | 0.982 | **the arm, not the throw** |
| 3.440 m/s | 1462 rev/s² | 10.022 rev | +0.063 | 0.985 | **the arm, not the throw** |
| 3.969 m/s | 1947 rev/s² | 10.306 rev | +0.347 | 0.921 | the throw (5/5) |
| 4.858 m/s | 2916 rev/s² | 10.980 rev | **+1.020** | **0.799** | the throw on 2/5 |

The last row is the tier that made **light physical contact with the end stop**
(operator testimony, 2026-07-28). η degrades with speed because the loop's
contribution is roughly a fixed torque while the feedforward deficit scales with
the commanded decel.

> **⚠ The two low rows do not measure this throw's deceleration** (found
> 2026-08-23, `tools/probes/hand_decel_authority.py` § *Why the coast window
> exists*). `peak` was the maximum `pos_meas` over the ~400 ms after the
> commanded stroke end, and C-HAND-1's gated catch arm dispatches its prelude
> 37-183 ms into that window and climbs back past `x3`, overshooting it by
> 0.046-0.222 rev. On **10 of the sitting's 17 tosses** — both low tiers on
> every toss — the reported peak is that arm's excursion. Re-scored with the
> window bounded at the first post-stroke command, the two low tiers' throws
> topped out at **−0.051 and −0.092 rev**, i.e. *below* `x3`, not above it.
>
> So the claim "the plant already tracks to η = 0.98 at the bottom of the band",
> which this contract leans on twice — once to reject a computed undershoot and
> once to argue the gravity over-brake is absorbed — **was never measured**. The
> two upper rows are unaffected in kind: 3.969 is the throw on 5/5 and 4.858 on
> 2/5, and the end-stop contact that motivated the whole phase is real. What
> moved is the *bottom* of the band, and it moved toward over-braking.

## Why a feedforward correction and not a steeper ramp

**Steepening is not available at the top of the shipped band.** With the
identified reflected inertia and the shipped `hand_curr_limit_a = 50.0`, the
axis's absolute deceleration ceiling is `50 × 0.0055133 / (J·2π)` =
**4178–4333 rev/s²**. The commanded decel at `FLIGHT_TIME_MAX_S = 1.10 s`
(v = 5.396 m/s) is already **3597 rev/s² — 83–86 % of it**. The steepest
commandable ramp shortens the decel distance from 4.046 to ~3.48 rev, which
reduces the overshoot by at most ~14 %: 1.02 → 0.88 rev on the tier that already
touched the stop. It also moves the release point `x2` up the stroke, changing
the ball's release height and re-calibrating every throw.

**A computed undershoot is the same lever** (it shortens the commanded decel
distance) and inherits the same ceiling, while additionally requiring the
overshoot to be *predicted*: the measured overshoot on the tier that touched the
stop scatters over 0.901–1.103 rev (a 0.202 rev spread on five throws), so no chosen undershoot both helps at the top of the band
and avoids driving the hand **below** `x3` at the bottom, where the plant already
tracks to η = 0.98. Driving the hand below `x3` re-creates exactly the
operator-visible dip Phases 1–4 removed, on the *commanded* profile, where
C-HAND-1's `dip_below_x3 ≤ 0.10 rev` gate cannot tell it from the defect.

**Raising an ODrive gain or current limit** is an operator decision and is not
needed: the corrected feedforward peaks at 31.6 A on the tier that touched and
38.9 A at the band ceiling, inside the shipped 50 A limit with 22 % headroom.

## The declared inertia, and why it is deliberately low

All off `temp/logs/toss_trace_2026-07-27_15-39-50.jsonl`, re-derivable with
`python tools/probes/hand_decel_authority.py --trace <that file>`.

| method | phase measured | value |
|---|---|---|
| **decel-side torque balance, per tier** — `J ≥ (τ_FF_wire + τ_grav)/(2π·a_achieved)`, taking the largest of the four tier bounds | **post-release** | **≥ 1.0126e-5 kg·m²** |
| regression of achieved-vs-commanded decel across the four flown tiers (slope 0.702 = `J_ff/J_true`) | post-release | 1.050e-5 kg·m² |
| geometric load-only floor `m_hand·(1/(2π·LINEAR_GAIN))²` — a hard lower bound | — | 7.120e-6 kg·m² |

The **decel-side torque balance is the anchor**, and it is a genuine bound
rather than a fit. Through the entire overshoot the hand is *ahead* of
`pos_cmd`, so the position loop is braking too (`τ_loop ≥ 0`), and friction only
adds; hence `J·2π·a_achieved ≥ τ_FF + τ_grav`. A lower bound is a lower bound,
so the largest across tiers binds — **1.0126e-5**, from the 4.858 m/s tier, the
one that touched the stop. It uses **no `iq` measurement at all**, which is why
the telemetry aliasing (below) cannot corrupt it.

> ### An identification that was withdrawn — do not reinstate it
>
> An earlier draft of this contract anchored the safety clause on an
> **accel-phase** torque balance (peak `iq` 25.67 A × Kt, less the 1.50 A
> gravity hold, against 147.29 rev/s over the commanded 70.5 ms accel →
> 1.015e-5 kg·m²). **That number is not a post-release inertia.** Release
> happens at `x2`, the *end* of the velocity hold, so the ball is in the cup
> throughout the ascent. `INERTIA_RATIO = 0.747` is `m_hand/(m_hand+m_ball)` —
> that is exactly what makes `throwD = −throwA/IR` a constant-motor-torque
> design — so `m_ball = 0.0952 kg` and its reflected inertia is
> `0.0952·(1/(2π·LINEAR_GAIN))² = 2.412e-6 kg·m²`, **24 % of the quoted value**.
> Ball-corrected, that method yields **7.74e-6** for the post-release axis,
> which is *below* the declared 9.5e-6 — i.e. read as a decel identification it
> would say the shipped feedforward over-brakes by 23 %.
>
> It is also refuted as a decel identification by the decel data itself: at
> `J_decel = 7.74e-6` the legacy feedforward would have delivered
> `7.3696/7.74 = 95 %` of the commanded decel at every tier, so η would be ~0.95
> flat. Measured η is **0.799** at 4.858 m/s. So 7.74e-6 is wrong for the decel
> too — the accel-phase method simply is not measuring this axis's post-release
> inertia, in either direction.
>
> The old "the residual 3.2e-6 between the load floor and the identifications is
> the rotor" line was wrong for the same reason: it differenced a *ball-inclusive*
> number against a *hand-only* floor and called the ball part of the rotor. On the
> decel-side bound the rotor residual is `1.0126e-5 − 7.120e-6 = 2.9e-6 kg·m²`.

> **Superseded as a statement about the CURRENT plant** (2026-08-23). Both
> numbers in the table above were measured through the −10.00 A clamp and with
> the contaminated window; the re-derivation on the restored drive is
> § *The re-derivation on the UNCLAMPED drive* below, and it puts `J_true` at or
> **below** the declared value rather than above it. The rows are kept because
> the *methods* are still the two this contract uses, and because the 2026-08-10
> A/B against them is what proves the clamp was binding.

**The shipped value is 9.5e-6 — 6–10 % below the decel-side evidence, on
purpose.** The feedforward alone produces a deceleration of `a_cmd · J_ff/J_true`,
so *considered alone* it cannot over-brake while `J_ff ≤ J_true`. That is the
second obligation of the contract. Raising the value is a bench decision backed
by measurement, not a desk one — and the enforcement is
`tests/sim/test_hand_throw_decel_ff.py::test_declared_inertia_cannot_over_brake`,
which pins the declared value under the **1.0126e-5 decel-side bound**.

### The re-derivation on the UNCLAMPED drive (2026-08-23) — and why no new value landed

`plans/active/catch-robustness.md`'s conditional Open row fired on 2026-08-18:
every capture in the repo predated the removal of the hand ODrive's −10.00 A
torque clamp, so the decel-side bound above was measured through it. The
operator flew a HAND-7 ladder on the restored drive on 2026-08-23 —
`~/Desktop/rosbags/2026-08-23_19-14-54`, **15 throws, 5 tiers × 3, at 2.706 /
3.440 / 3.920 / 4.436 / 4.858 m/s**, can-bridge FW **15** (proto 5), Platform FW
**3**, `torque_ff_enabled = 1`, bridge uptime 116.7 h. Score it with

```
python tools/probes/hand_decel_authority.py --bag ~/Desktop/rosbags/2026-08-23_19-14-54
```

**The clamp removal is confirmed in both channels**, which is what the
discriminator was for:

| | 2026-08-10 (clamp LIVE, Platform FW 2) | 2026-08-23 (clamp GONE, FW 3) |
|---|---|---|
| tier | 4.436 m/s, 31 tosses | 4.436 m/s, 3 tosses |
| commanded decel feedforward | 27.2 A | 27.2 A |
| worst braking `iq` per toss | min **−10.52**, median −7.05 A | min **−14.40**, median −13.95 A |
| whole-session `iq` floor | **−11.42 A** | **−17.77 A** |
| coast peak vs `x3` | **+0.719 rev** (31/31 the throw's own) | **−0.279 rev** (3/3) |

Braking current now tracks the command monotonically across the whole ladder —
9.1 A commanded → −3.5 measured, 16.3 → −7.3, 20.0 → −9.2, 27.2 → −14.0, 30.8 →
−17.7 (per-tier medians of the worst fresh sample per toss) — and it exceeds the
old −10 A ceiling on every toss of the top three tiers, which it structurally
could not do before.

**But achieved deceleration did NOT become tier-independent.** η reads **1.018 /
1.044 / 1.053 / 1.074 / 1.076** up the ladder — a monotone trend, and *above*
1 at every tier, which by the definition above means the hand **stopped short of
`x3` on all 15 throws**. Its coast topped out 0.058-0.292 rev under the stroke
top and then sagged a further 0.034-0.183 rev under the latched terminal torque,
ending **0.113-0.468 rev below `x3`** — over row H7.4's 0.100 rev band on 15 of
15. (Both shipped instruments reported `dip_below_x3 = 0.000` on all 15; that is
the same window defect as the founding table's, and it is now marked rather than
silent — `hand_stroke_timeline.py`'s `coast_below_x3` row.)

Re-running this contract's own two methods with the coast window and with the
wire feedforward read off the capture:

| method | result on the unclamped drive |
|---|---|
| **decel-side torque balance, per tier** | every tier's coast finished BELOW `x3`, so the hand never caught `pos_cmd`, `τ_loop` pushed **up** through the whole excursion, and the SAME expression is an **upper** bound: `J_true ≤ 1.004e-5 / 1.025e-5 / 9.41e-6 / 9.63e-6 / **9.04e-6**` kg·m² by tier |
| regression of achieved-vs-commanded decel | slope 0.9415 (R² 0.9999) → `J_ff/slope` = 1.009e-5; on the measured wire torque, `1/(2π·slope)` = 1.027e-5 (R² 0.9915, intercept 0.84 A against the 1.50 A ± 50 % gravity hold) |

Both readings are computed **at the commanded release velocity**, and that is
where the capture stops being decisive:

* the hand's own encoder puts its peak velocity **−5.5 % to +2.3 %** of
  commanded, and its ~100 Hz sampling of a 3-6 ms velocity apex is biased low by
  `a · 2.5 ms` = **−4.1 %** at the top tier — so de-biased, the encoder says the
  release is *on target*;
* the **ball** says it is not. A ballistic fit `z = z₀ + v·t − ½g·t²` over the
  rise, on the mocap marker with static reflectors filtered, gives **+15.5 /
  +11.8 / +11.3 / +9.9 / +10.6 %** by tier (n = 3 each, within-tier spread under
  2 points), and the fitted `z₀` of 794-832 mm agrees with the announced release
  height 802.3 mm to ±30 mm — so the fit does not rest on that assumption.
  Independently corroborated: the ILC corpus measures this hand **+11 % fast**
  (`logbook/2026-08-21-ilc-primary-foldin.md`).

**The two channels agree on the SIGN and disagree on the MAGNITUDE, and the sign
is what this contract turns on.** `J` scales as `v⁻²`, so the encoder channel
gives `J_true ≤ 9.04e-6` and the ball channel `≤ ~7.5e-6` — **both below the
declared 9.5e-6.** Together with the direct kinematic observation (the hand
stops short on 15/15), the one-sided-safety clause now reads **violated in the
over-braking direction**, and this contract's documented response to that is to
*lower* `throw_decel_reflected_inertia_kgm2`, never raise it.

**So the package that went looking for a higher value found the opposite, and
still did not land one.** The direction is settled; the magnitude is not — the
two channels' upper bounds differ by 20 %, and closing the gap by flashing a
guessed value costs a Platform Teensy flash (Arduino IDE; `pio` is CAN-mute and
suspended) plus a re-validation ladder. Landing 9.5e-6 → 9.0e-6 → 7.5e-6 in
sequence would be three flashes to converge on a number one measurement settles.

**What settles it**, in order:

1. **The rev→mm gain, measured statically.** The two channels reconcile exactly
   if one motor revolution moves ~10 % more hand than the
   `2π·HAND_SPOOL_RADIUS_M / LINEAR_GAIN_FACTOR` = 31.628 mm assumed above —
   the encoder is right in *rev* space and every m/s in the hand path is 10 %
   low. That also re-bases `J`, which scales as gain⁻². Measure it with the hand
   at rest: command a known rev displacement and measure the cup's travel.
   *(Attempted from this bag by regressing the seated ball's mocap `z` on
   `pos_meas` through the ascent — **inconclusive**: the ball is occluded in the
   cup for most of the climb, only 1 of 15 throws yielded ≥ 15 paired samples,
   and that fit read −21 % at R² 0.81. It is not evidence in either direction.)*
2. **Then re-fly R0-R5** and re-read `cst_pk`, `under` and the per-tier sense
   line. Until step 1 lands, a ladder measures the same ambiguity again.

### The one-sided-safety clause, stated honestly: gravity is in it too

The inequality above is about the **feedforward alone**, and the feedforward is
not the only open-loop braking torque. On an *upward* deceleration **gravity
brakes in the same direction**. The measured ball-free hold torque is
`1.50 A × Kt = 0.00827 N·m`, which is equivalent to an extra
`τ_grav/(2π·a_cmd)` of feedforward inertia — **speed-dependent, and largest at
the bottom of the band**:

| flight time | `a_cmd` | gravity as ΔJ | open-loop `J_eff` | vs `J_true ≥ 1.0126e-5` |
|---|---|---|---|---|
| `FLIGHT_TIME_MIN_S` = 0.55 s | 899 rev/s² | +1.464e-6 | 1.0964e-5 | **exceeds it** — open-loop over-brake |
| 0.80 s | 1902 rev/s² | +0.692e-6 | 1.0192e-5 | marginal |
| `FLIGHT_TIME_MAX_S` = 1.10 s | 3597 rev/s² | +0.366e-6 | 9.866e-6 | under — no over-brake |

So the blunt claim "while `J_ff ≤ J_true` it can **never** over-brake" is **false
below roughly `a_cmd ≈ 1900 rev/s²`**, and the honest open-loop condition is
`J_ff ≤ J_true − τ_grav/(2π·a_cmd)`. Three things bound the consequence, and all
three are why 9.5e-6 still ships:

1. **No declared value satisfies both requirements.** Meeting the gravity-inclusive
   inequality at the band floor needs `J_ff ≤ 8.69e-6`, which puts the pessimistic
   peak at `9.9594 + 4.0456·(1.050e-5/8.69e-6 − 1)` = **10.80 rev** — past the
   10.60 hard-abort line this phase exists to get under. The two requirements are
   incompatible, which means the open-loop inequality is the **wrong** requirement:
   what matters is the *closed-loop measured* dip, not an open-loop bound.
2. **The over-brake is worst exactly where the loop is strongest.** The gravity
   term dominates at low `a_cmd`, which is the same place the decel ramp is
   longest (93.3 ms at 2.742 m/s vs 52.7 ms at 4.858 m/s). The measured loop
   attenuation at that tier is `0.0739/1.7185 = 4.3 %` of the open-loop excursion.
   Applied to the 0.300 rev open-loop undershoot (0.580 rev with worst-case wire
   rounding) that predicts a **physical dip of 0.013–0.025 rev**, against the
   0.100 rev gate.
3. **The gate is the enforcement, not the algebra.** Bench row **H7.4**
   (`dip_below_x3 ≤ 0.100 rev`) measures the real thing on the real plant, and its
   documented response — *lower* `throw_decel_reflected_inertia_kgm2`, never raise
   it — is the fail-safe direction. The ladder gains a rung **R0 at the band
   floor** precisely because that is where over-braking bites first, and it is
   below every rung the ladder previously had.

### The negative torque clamp — unresolved, and a bench pre-flight

`config/ODrive config Files/odrive_pro_hand_config.json` declares
`axis0.config.torque_soft_min = −0.055133331567049026 N·m`, which at this Kt is
**exactly −10.00 A**, against a `torque_soft_max` of +0.5 N·m (+90.7 A, so the
50 A current limit is what binds on the motoring side). The asymmetry is stark
and the value looks like a paste of a `torque_constant`, not a design point.

If that clamp is live on the flashed drive it truncates the decel feedforward —
**legacy and corrected alike** — at every tier above ~0.49 m, and this whole
phase is a no-op that reads as a physics result.

**Counter-evidence, which is why this is flagged rather than treated as proven:**
a hard clamp would force one single achieved deceleration at every tier, and the
measured `a_achieved` grows 911 → 1440 → 1794 → 2330 rev/s² across the band.
Reconciling a live clamp with that requires a negative damping coefficient. So
it is very probably not binding. Against that: with the decel window corrected
(below), the largest braking current ever sampled in a decel ramp across all 17
tosses is **−9.91 A**, and the whole-session floor is −11.4 A, while the
kinematics require ~26 A of braking at the top tier. That gap is unexplained;
the aliasing is the likely cause, but it has not been shown.

> **RESOLVED 2026-08-23 — it was binding, and the current gap was the tell, not
> the aliasing.** On the restored drive the whole-session braking floor moves
> **−11.42 → −17.77 A** and exceeds the old −10 A clamp on every toss of the top
> three tiers, which it structurally could not do before; braking `iq` now
> scales monotonically with the commanded feedforward across the ladder (9.1 A
> commanded → −3.5 measured, 16.3 → −7.3, 20.0 → −9.2, 27.2 → −14.0, 30.8 →
> −17.7). The measured current still falls well short of the commanded
> feedforward, so the aliasing caveat above stands on its own — it was simply
> never the whole explanation. Keep the paragraph: it is the shape of a
> counter-argument that felt strong and was not, and § *The re-derivation on the
> UNCLAMPED drive* is where the consequences land.

**Bench pre-flight H7.0c** settles it in 30 seconds: read
`axis0.config.torque_soft_min` off the live hand axis *before* the flash. And
row **H7.3** is the in-band discriminator — under a live clamp the overshoot
stays speed-dependent and the flatness row fails.

### Predicted effect, stated as a bracket rather than a number

The plant is **not identifiable** from the available capture to the accuracy a
point prediction would need: `hand_telemetry` is a ~100 Hz snapshot of a 500 Hz
stream, `iq_meas` has a median repeat run of 4 samples (effective refresh
13–25 Hz), and a cascade simulation using the shipped gains under-predicts the
measured 4.86 m/s peak by 0.87 rev while over-predicting the current by 40 %.
So the effect is bracketed:

* **Pessimistic (feedforward is the *only* braking, loop contributes nothing):**
  `over = 4.046·(J_true/J_ff − 1)` — **velocity-independent**, 0.267 rev at
  `J_true = 1.0126e-5` and 0.426 rev at `J_true = 1.050e-5`. Peak ≤ **10.39 rev**
  at every speed in the band.
* **Optimistic (loop keeps the ~348 rev/s² it contributes today):** peak
  ≈ 10.15 rev at the band ceiling.

Against the runbook's `10.60` hard-abort line that is **0.215 rev = 6.8 mm** of
margin in the pessimistic bracket. *(This paragraph originally continued "and
0.67 rev = 21 mm to the ~11.06 rev physical stop the operator's contact
implies". **That inference was wrong**: it assumed the declared 11.1 rev stop.
The stop is 10.8 rev, measured 2026-08-18, so the pessimistic bracket's real
clearance to metal is **0.41 rev = 13 mm**, and at the +1.9 % release-speed
error measured on five throws it is **0.24 rev = 7.7 mm**. That thinness is a
principal reason C-HAND-3's end-stop bound does not credit this bracket.)* The velocity-independence of the
pessimistic bracket is the property that matters most: it is what removes the
superlinear growth that made the band ceiling unflyable.

**That 0.215 rev is quoted at the COMMANDED release speed, and the plant does not
hit it exactly.** The overshoot scales as `v²`, so

```
over = d_dec · [ (1+ε)² · J_true/J_ff − 1 ]        ε ≡ release-speed error
```

Measured `v_pk` against the commanded 153.4 rev/s at the ~1.2 m tier: +1.8 %,
−7.1 %, +1.9 %, −0.3 %, −4.0 %. At ε = +1.9 % the pessimistic peak is
**10.56 rev** and at ε = +2.6 % it reaches the 10.60 line — so **R5 can land in
the DEBRIEF band, or trip the hard abort, on release-speed scatter alone while
the fix is working exactly as modelled.** The measured peaks are sampled maxima
of an aliased stream, so the true scatter is at least this. Runbook rung **R5**
carries the cross-check: read the achieved `v_pk` before concluding anything
about the feedforward from a DEBRIEF-band peak.

**Nothing here is validated on hardware.** `tests/hardware/session_anomaly_fixes.md`
§ CHECK HAND-7 is a per-tier ladder that must be climbed in order, and the band
ceiling must not be flown until the lower tiers confirm the bracket.

## Scope — what this contract does NOT touch

| surface | state | why |
|---|---|---|
| the throw's **accel** and velocity-hold torque | unchanged (`accelToTorque`) | correcting the ascent feedforward makes the hand track the ascent better and so **raises the achieved release velocity**, re-calibrating every throw height the machine has flown — on a machine whose hand already reaches its end stop. Operator decision |
| commanded **position** and **velocity** streams, every kind | bit-identical | so C-HAND-1's stroke-busy window, the throw's event-delay floor, `_PRIME_INFLIGHT_S`, and `tools/probes/hand_stroke_timeline.py`'s whole verdict model stay valid **without moving**. (That floor was the constant `MIN_THROW_EVENT_DELAY_S = 1.0 s` when this row was written; on 2026-08-22 it became `hand_stroke.min_throw_event_delay_s(v_throw)`, derived from the same stroke geometry, and on 2026-08-23 the ACCEPT-time gate that fronts it gained the pre-dispatch sequence on top. The row's claim is unaffected — the streams this change leaves bit-identical are exactly the ones that function reads. `trajectory/commanded_pose`, added the same day, is the same sampled plan state with its orientation, so it inherits the claim rather than widening it.) |
| kind-1 `makeCatch` | untouched | `buildCatch` passes `accelToTorque` explicitly |
| `makeSmoothMove` (kind-3 prime/retract/SAFE_ABORT and every prelude) | untouched | the kind-3 clobber is the only un-arm mechanism the Teensy offers |
| kind-2 `makeFull` | untouched, and **carries two known defects** | no live host dispatches kind 2. Its `accelToTorque(acc * LINEAR_GAIN)` feeds a rev/s² quantity into an m/s² conversion (31.6× too large), and it does not carry the corrected decel feedforward. Recorded in `buildCommand`'s comment; fix together, with a bench validation, if kind 2 is ever revived |
| ODrive gains and current limit | untouched | operator decision |
| the **terminal latched torque** | **changes by 1.289×**, even though the streams do not | `buildSegment` stops one 500 Hz sample short of `t3`, so the last commanded frame carries the FULL decel feedforward and `Set_Input_Pos` latches it until the next hand command. That standing value goes 24.5 → 31.6 A at the 4.858 m/s tier and 30.2 → 38.9 A at the band ceiling. Steady-state position is unaffected (the velocity integrator absorbs a constant torque offset), and on the shipped toss path the C-HAND-1-gated catch arm overwrites it 28–62 ms later — but it is the one magnitude this phase moves at the latch, so `hand_decel_authority.py` reports `tor_hold_max` and bench row **H7.6** records it. **Not** analysed for an IDLE→CLOSED_LOOP transition that resets the integrator while the torque stays latched; see the open questions |
| release speeds above the flight band | **out of contract** | the feedforward is sized only over the flight band, which was `[0.55, 1.10]` s when this was written and is `[0.495, 0.887]` s since 2026-08-18 (**C-HAND-3**) — i.e. the sized range now *contains* the admitted range with room to spare, which is the safe direction. `MAX_EVENT_VEL_MPS = 7.0` is a *builder* clamp, not an operating point: at 7.0 m/s `a_dec` is 6054 rev/s² and the corrected feedforward alone wants **65.5 A of a 50 A limit** (the legacy one already wanted 50.8 A). What keeps callers inside is the host's admission gate — until 2026-08-18 a hand-picked flight-time band, now `throw_envelope.evaluate`; any caller reaching the 7.0 m/s clamp is out of contract. Pinned by `test_the_feedforward_is_only_sized_over_the_flight_band` |

## Deployment

**A Platform Teensy flash.** `FW_VERSION` went **1 → 2** for this phase; it is
**3** in the tree since the 2026-08-18 end-stop correction, and
`rpc_args.PLATFORM_FW_VERSION_EXPECTED` is 3 to match. The host's
`PLATFORM_FW_VERSION_EXPECTED` moves with it, so a board still on 1
reads `PLATFORM_FW_CHECK: FAIL` on `link_status/platform_fw_version` (contract
C-PLATFW-1, `ros_ws/docs/platform_fw_version.md`). It **warns and never
refuses** — a v1 board is not unsafe, it simply still coasts — but every
§ CHECK HAND-7 row is meaningless on one.

Also `colcon build --packages-select jugglebot` **+ relaunch**: the regenerated
`hardware_config.py` ships inside the ROS package and the launch runs the
*installed* copy. No `jugglebot_interfaces` change. No codegen beyond
`python config/generate_config.py`, already run.
