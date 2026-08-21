---
title: The hand's post-release deceleration feedforward — stop flirting with the end stop
type: bugfix
date: 2026-07-29
status: tuned
phase: "Self-toss anomaly fixes — Phase 7 (post-release decel)"
related_plan: "hand-command-continuity.md"
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code/Trajectory.h
  - ros_ws/src/jugglebot/Teensy_code/Teensy_code.ino
  - controller/teensy_link/rpc_args.py
  - sim/hand/trajectory.py
  - ros_ws/docs/hand_decel_feedforward.md
  - ros_ws/docs/hand_command_continuity.md
  - tests/sim/test_hand_throw_decel_ff.py
  - tests/firmware/test_hand_throw_decel_xref.py
  - tools/probes/hand_decel_authority.py
  - tools/probes/README.md
  - tests/hardware/session_anomaly_fixes.md
  - tests/hardware/session_phase8_toss_hardware.md
  - plans/archived/hand-command-continuity.md
  - plans/active/PROMPT-anomaly-fixes-orchestration.md
commits:
  - f920087
subsystem:
  - motion
  - config
tags:
  - safety
  - dynamics
  - testing
  - docs
---

# The hand's post-release deceleration feedforward — stop flirting with the end stop

## Summary

On 2026-07-27 the hand made light physical contact with its mechanical end stop
on ~1.2 m self-tosses. Nothing commanded was at fault — `pos_cmd` never left
`x3 = 9.9594` rev. The braking **torque feedforward**, the only term that commands
braking open-loop, was sized from a hand-mass-on-a-spool model that omits the
motor's rotor and uses the raw spool radius, implying a reflected inertia of
`7.3695e-6` kg·m² against a measured `≥1.0126e-5`. It delivered ~70 % of what the
commanded deceleration physically needs, and the shortfall fell to a loop whose
integrator unwind constant (0.100 s) is 1.1–2.1× the whole 47–93 ms ramp. Because
`calcThrow` allocates the decel **4.046 rev of travel at every speed**, ending
exactly on `x3` with zero allowance for tracking error, that shortfall converts
1:1 into end-stop travel. Fixed at one enforcement point (`throwDecelToTorque`,
one caller) with the inertia declared in config; commanded position and velocity
are bit-identical on every kind. `FW_VERSION` 1 → 2. **Not flashed** — the
operator owns deployment and the R0→R5 bench ladder.

## Symptom

Measured on the sitting's own capture,
`temp/logs/toss_trace_2026-07-27_15-39-50.jsonl` (17 self-tosses):

| commanded release | commanded decel | measured peak | over `x3` | implied η |
|---|---|---|---|---|
| 2.742 m/s | 929 rev/s² | 10.033 rev | +0.074 | 0.982 |
| 3.440 m/s | 1462 rev/s² | 10.022 rev | +0.063 | 0.985 |
| 3.969 m/s | 1947 rev/s² | 10.306 rev | +0.347 | 0.921 |
| 4.858 m/s | 2916 rev/s² | 10.980 rev | **+1.020** | **0.799** |

η degrades with speed because the loop's contribution is roughly a fixed torque
while the feedforward deficit scales with the commanded decel. The last row is
the tier that touched.

## Discussion

*(Written before the Fix section, per the project's rule — the reasoning below
drove the code, not the other way round.)*

### Why a feedforward correction and not a steeper ramp or a computed undershoot

The brief offered three levers and framed the question as *authority-limited vs
tracking-limited*. **The measurement says both answers are needed**, and that is
what selected the fork. The drive is not current-saturated (whole-session max
|iq| = 25.67 A of 50, with a smooth tail and no clipping; achieved decel keeps
*growing*, 911 → 2330 rev/s²). But steepening is still unavailable, because the
**commanded** decel at `FLIGHT_TIME_MAX_S = 1.10 s` already sits at 3597 rev/s² =
**83–86 %** of the axis's own 4178–4333 rev/s² ceiling at `hand_curr_limit_a = 50`.
The steepest commandable ramp shortens the decel distance 4.046 → ~3.48 rev and
buys at most ~14 % of the overshoot. Concrete failure mode: a profile asking for
more decel than the drive can deliver produces the *same plant motion as today*
while making the operator believe the fix landed — an instrument reading "fixed"
on an unfixed machine. It also moves `x2` up the stroke, re-calibrating every
throw height.

The computed-undershoot fork is mathematically the same lever (it shortens the
commanded decel distance), so it inherits the same ceiling, and it additionally
requires the overshoot to be *predicted* — against a measured 0.202 rev scatter
on the tier that touched. No fixed Δ both helps at the band top and avoids
driving the hand *below* `x3` at the floor where the plant already tracks to
η = 0.98, and a commanded excursion below `x3` re-creates the operator-visible
dip Phases 1–4 removed, on the commanded profile, where C-HAND-1's
`dip_below_x3` gate scores the hand's *position*, not its cause, and so cannot
tell the two apart.

### Why the ascent is deliberately left wrong

`accelToTorque` carries the same missing-rotor error on the ascent, and a second
one: the profile is a constant-motor-torque design (`INERTIA_RATIO = 0.747 =
m_hand/(m_hand+m_ball)`, which is exactly what makes `throwD = −throwA/IR`), yet
it uses `INERTIA_HAND_ONLY` for the ascent where the ball is still in the cup.
Correcting it would make the hand track the ascent better and so **raise the
achieved release velocity** — measured peak is 147.29 rev/s against a commanded
153.66, 4 % low — lengthening every flight. A silent 4 % height increase on a
machine whose hand already reaches its end stop is the opposite of what this
phase is for. The decel is also the only segment where the inertia is
unambiguous: after release the ball is gone, so the axis is exactly rotor + hand.

### Two hypotheses withdrawn

**(1) The settled post-stroke offset is the terminal residual Vel_FF.** Refuted
by the trace: the *median* `vel_ff_cmd` through those windows is 0.00–0.02 rev/s,
predicting 0.0006 rev against a measured offset up to +0.067 — two orders of
magnitude too small. Recorded as an open question, not shipped as a finding.

**(2) A cascade simulation can predict the peak.** Built (ODrive cascade, 500 Hz
ZOH setpoints, 8 kHz inner loop) and discarded as unsound: with any physically
plausible `J` it under-predicts the measured 4.86 m/s peak by 0.87 rev while
over-predicting the current by ~40 %. That failure is itself the finding —
`hand_telemetry` is a ~100 Hz snapshot of a 500 Hz stream and `iq_meas` has a
median repeat-run of 4 samples (effective 13–25 Hz), so the plant is not
identifiable to the accuracy a point prediction needs. Hence the effect ships as
a **bracket**, and the bench ladder is what turns it into a measurement.

### What the adversarial review changed — and one thing it did not

Three reviewers ran independently. Full dispositions below; four findings changed
what the bench will do, and two of those had **two lenses converging from
different starting points**, which is the strongest signal the panel produces.

**The safety clause was anchored on a measurement of the wrong thing
(converged: physics + regression lenses).** The contract cited an *accel-phase*
torque balance → 1.015e-5 kg·m² as one of two identifications bounding the
declared inertia. But release is at `x2`, the end of the velocity hold, so the
ball is in the cup for the entire ascent; `INERTIA_RATIO = 0.747` makes
`m_ball = 0.0952 kg`, reflecting `2.412e-6 kg·m²` — **24 % of the quoted value**.
Ball-corrected, that method yields **7.74e-6**, *below* the declared 9.5e-6, so
the claim "9.5e-6 sits 7–10 % below BOTH identifications" was false. The number
happened to land near the right answer because two errors cancelled.

I did not simply subtract the ball and move on, because 7.74e-6 is *also* refuted
as a decel inertia: at that value the legacy feedforward would have delivered
`7.3696/7.74 = 95 %` of the commanded decel at every tier, so η would be flat
~0.95, where 0.799 was measured. The accel method is not measuring this quantity
in either direction. So the anchor was **re-derived from the decel phase**, as a
genuine bound rather than a fit:

```
J ≥ (τ_FF_wire + τ_grav) / (2π · a_achieved)
```

valid because through the entire overshoot the hand is *ahead* of `pos_cmd`, so
the position loop is braking too (`τ_loop ≥ 0`) and friction only adds. Per tier
this gives 8.43 / 8.65 / 8.72 / **1.0126e-5**; the largest binds. It uses **no
`iq` measurement at all**, which is why the telemetry aliasing that killed the
cascade model cannot corrupt it. The test constant moved 1.015e-5 → 1.0126e-5,
i.e. *tighter*. The old "the residual 3.2e-6 is the rotor" line was wrong for the
same reason — it differenced a ball-inclusive number against a hand-only floor —
and is now 2.9e-6.

**The one-sided-safety proof omitted gravity, and this one has no clean fix.** On
an *upward* deceleration gravity brakes in the **same direction** as the
feedforward. The measured ball-free hold torque, 1.50 A × Kt = 0.00827 N·m, is
worth `τ_grav/(2π·a_cmd)` of extra effective inertia — speed-dependent, and
largest at the bottom of the band: +1.46e-6 at `FLIGHT_TIME_MIN_S` against
+0.37e-6 at the ceiling. So open-loop `J_eff = 1.0964e-5` at the floor, *above*
`J_true`, and the flat claim "while `J_ff ≤ J_true` it can never over-brake" is
**false below `a_cmd ≈ 1900 rev/s²`**.

The obvious response — lower the declared inertia until the gravity-inclusive
inequality holds — **does not exist**. Meeting it at the band floor needs
`J_ff ≤ 8.69e-6`, which puts the pessimistic peak at
`9.9594 + 4.0456·(1.050e-5/8.69e-6 − 1) = 10.80 rev`, past the 10.60 hard-abort
line this phase exists to get under. The two requirements are incompatible,
which is itself the finding: **the open-loop inequality is the wrong
requirement.** What matters is the closed-loop *measured* dip, and three things
bound it — (a) the over-brake is worst exactly where the decel ramp is longest
(93.3 ms at 2.742 m/s vs 52.7 ms at 4.858) and the loop most effective, measured
attenuation 4.3 % of the open-loop excursion at that tier, predicting a physical
dip of 0.013–0.025 rev against a 0.100 gate; (b) H7.4 measures the real thing and
its documented response is *lower* the declared inertia, the fail-safe direction;
(c) the ladder gained a **band-floor rung R0**, because that is where
over-braking bites first and the ladder previously started at 0.60 m — while
routine reload/catch work throws at ~0.38 m every day.

The tradeoff accepted, stated plainly: **the shipped configuration commands a
small open-loop over-brake at the bottom of the flight band, in exchange for
clearing the end stop at the top.** That is a deliberate choice between two
failure modes of very different severity — a ~0.02 rev dip the loop absorbs
versus physical contact with a hard stop — and it is now written where the next
reader will find it instead of being hidden behind an inequality that looked
airtight.

**The verdict instrument was reading the wrong phase of the stroke.** This is the
finding that would have cost a powered sitting. `iq_ramp`'s window was
`[throw_time, throw_time + t_dec]`, anchored on the announcement. Measured: the
commanded stroke end lands **120.5–166.9 ms** after `throw_time` while `t_dec` is
only 52.7–93.3 ms, so the window had **zero overlap with the real deceleration on
13 of 17 tosses** and max overlap 0.21 on the rest. It was reporting the *ascent*
current — 10–23 A, where the true decel braking current is 3.6–9.9 A. Bench row
H7.5 gates the highest-energy rung of the ladder on that number. Re-anchored on
the commanded profile (`stop`, which the probe already computes), with
`t_anchor` and a fresh-`iq`-sample count now printed so the contamination is
visible rather than silent.

That finding arrived together with its sibling: **the instrument had no
self-check**, alone among the instruments this runbook cites (INST-1 through
INST-5 all have one; INST-2's acceptance is explicitly two-sided). It now has
`--self-check`, and the design point is that it scores **both shapes** — a
synthetic pre-fix capture that must be FLAGGED and a synthetic post-fix capture
that must be ACCEPTED — through the same `analyse()` the bench calls. An
instrument validated only against the broken case scores a working fix as a
failure, routes correct work back for rework, and burns the sitting; that is
worse than having no instrument. It also had to exercise the real code path
rather than reimplement the window inline, or it would have passed on the very
bug it exists to catch. Mutation-verified: restoring the old window makes the
self-check FAIL on the window assertion and PASS everything else.

**An unexamined drive-config clamp could make the entire phase a no-op.**
`odrive_pro_hand_config.json` declares `axis0.config.torque_soft_min =
−0.055133331567049026 N·m` — **exactly −10.00 A** at the hand's Kt — against a
`torque_soft_max` of +0.5 N·m (+90.7 A). The asymmetry is stark and the value
looks like a paste of a `torque_constant`, not a design point. If live it
truncates the decel feedforward, *legacy and corrected alike*, at every tier
above ~0.49 m.

I could not settle it from the desk and did not pretend to. **Counter-evidence:**
a hard clamp would force one single achieved deceleration at every tier, and the
measured `a_achieved` grows 911 → 1440 → 1794 → 2330 rev/s²; reconciling that
with a live clamp needs a negative damping coefficient. So it is very probably
not binding. **Against that:** with the window corrected, the largest braking
current ever sampled in a decel ramp across all 17 tosses is −9.91 A and the
whole-session floor is −11.4 A, while the kinematics require ~26 A of braking at
the top tier. That gap is unexplained; aliasing is the likely cause (0–1 fresh
`iq` samples per ramp on 8 of 17 tosses) but has not been shown. It became
pre-flight **H7.0c** — a 30-second `odrivetool` read *before* the flash — plus a
named failure-table row, because without it the ladder would read pre-fix numbers
and the existing table would route the operator to "the declared inertia is still
an under-estimate": a physics conclusion about a plant that never saw the new
command. H7.3 is documented as the in-band discriminator.

**What review did NOT change: the declared value stays 9.5e-6.** Two reviewers
implied re-sizing. I traced both and declined, because the arithmetic above shows
no value satisfies the gravity-inclusive inequality *and* the peak requirement,
and because the correction's direction and mechanism are independently confirmed.
Re-sizing on desk arithmetic would have traded a measured, bounded, loop-absorbed
dip at the band floor for a peak the ladder cannot clear at the ceiling. That is
a bench decision with an instrument now capable of making it.

### One instrument threshold was mis-derived in the conservative direction

H7.3's flatness gate (0.25 rev) came from the bracket's velocity-independence,
not from a model that fits the data. The *achieved* overshoot is
`loop attenuation × open-loop bracket`, and the attenuation is strongly
speed-dependent — measured 4.3 / 3.7 / 20.2 / 59.4 % of the 1.7185 rev pre-fix
bracket. Applying those ratios to the 0.4259 rev post-fix bracket predicts
per-tier `over_x3` of 0.018 / 0.016 / 0.086 / 0.253 — a **spread of 0.237 rev for
a fix behaving exactly as modelled**, i.e. 95 % of its own gate. A coin flip that
would have blocked R5 and handed back "the feedforward did not become the
dominant braking term" on a working fix. Raised to 0.35 (1.5× headroom over the
prediction, still failing the pre-fix shape by 2.7×), with the derivation written
into the row so the next person can judge it rather than apply it.

## Fix

**One enforcement point.** `Trajectory.h::throwDecelToTorque`, consumed by
exactly one caller — `buildThrow`'s `torA[2]`:

```cpp
constexpr float THROW_DECEL_TORQUE_K =
    TeensyTraj::THROW_DECEL_REFLECTED_INERTIA_KGM2 * 2.f * (float)M_PI * LINEAR_GAIN;
inline float throwDecelToTorque(float a) { return a * THROW_DECEL_TORQUE_K; }
```

`buildSegment` gained an explicit per-segment `torA[4]` rather than deriving
torque from `aA` inside the loop — so `buildCatch`'s "kind-1 is untouched" is a
line of code at the call site rather than an assumption about a hidden branch.
The declared inertia is a config key
(`teensy_trajectory.throw_decel_reflected_inertia_kgm2 = 9.5e-6`) rather than a
dimensionless boost factor, because a boost factor is a tuning knob with no
physical check while a declared inertia is checkable against the geometric load
floor and the decel-side bound — which is exactly what
`test_declared_inertia_cannot_over_brake` does.

Net effect on the wire: the decel segment's torque goes from `−m·r·|throwD|` to
`−J_ff·2π·LINEAR_GAIN·|throwD|`, a **1.2891×** increase — 24.5 → 31.6 A at the
tier that touched, 30.2 → 38.9 A at the band ceiling, against a 50 A limit. No
discontinuity is introduced: the torque stream was already a step at `t2`, and
the step simply gets larger; the commanded acceleration profile it feeds forward
is unchanged. The change is pure feedforward, added outside the feedback path, so
it cannot move a pole — it changes the plant's forced response only, and
monotonically in the braking direction.

**Deliberately not fixed, and recorded in place:** `buildCommand` (kind 2,
`makeFull`) computes `accelToTorque(acc * LINEAR_GAIN)`, feeding a rev/s²
quantity into an m/s² conversion, so its torque feedforward is 31.6× too large.
No live host dispatches kind 2 (grep: only `archived/`), and fixing it is a
commanded-magnitude change on a path this phase is not chartered for. Documented
in `buildCommand`'s own comment and in C-HAND-2's scope table so the next reader
does not "fix the inconsistency" the wrong way. Likewise the terminal residual
Vel_FF that `buildSegment` leaves on every stroke: diagnosis complete, but it is
shared with `buildCatch`, so removing it changes the commanded terminal frame of
the catch descent that meets the ball — an operator decision. Pinned with its
exact bound instead, and the terminal *torque* latch (which this phase multiplies
by 1.289×) is now reported by the probe and by bench row H7.6.

## Findings adjudication

Every finding from all three reviewers, with disposition:

| # | lens | severity | claim | disposition |
|---|---|---|---|---|
| 1 | physics + regression (**converged**) | HIGH/MED | identification (a) is ball-inclusive; "7–10 % below both" is false | **VERIFIED + FIXED** — `J_ball = 2.412e-6` confirmed from `INERTIA_RATIO`; re-anchored on the decel-side bound 1.0126e-5, test constant tightened, rotor residual corrected to 2.9e-6 |
| 2 | contract | HIGH | `iq_ramp` window has ~zero overlap with the decel | **VERIFIED + FIXED** — measured: zero overlap on 13/17, max 0.21. Window re-anchored on the commanded profile; `t_anchor` and fresh-sample count now printed; H7.5 reference numbers restated |
| 3 | physics | HIGH | the one-sided-safety proof omits gravity | **VERIFIED + FIXED (documentation + test + ladder)** — confirmed by my own arithmetic; no declared value satisfies both requirements, so the enforcement moved to H7.4 and a new band-floor rung R0. Declared value deliberately unchanged; see Discussion |
| 4 | physics | HIGH (NOT-PROVEN) | `torque_soft_min = −0.0551 N·m` may clamp the feedforward | **VERIFIED as a config fact + DEFERRED to the bench** — I read the value myself. Not settleable from the desk (counter-evidence: achieved decel grows 2.6× across tiers). Became pre-flight H7.0c, a failure-table row, and a probe warning |
| 5 | contract | MED | the probe has no self-check and no INST row | **VERIFIED + FIXED** — `--self-check` added, two-sided, exercising the real `analyse()`; mutation-verified against the finding-2 bug. INST-6 row added |
| 6 | contract | MED | "6055 rev/s² at the band top" stale at 3 sites | **VERIFIED + FIXED** — grep confirmed; corrected in all three with a note. Re-derived `|throwD| = 123.55·v²` → 3597 at `FLIGHT_TIME_MAX_S` |
| 7 | contract | MED | two sites still call `platform_fw_version = 1` the flashed reading | **VERIFIED + FIXED** — `session_phase8_toss_hardware.md:26` updated to 2 with the stale-flash reading; C-HAND-1 row F now points at C-PLATFW-1 instead of restating a number that will drift again |
| 8 | physics + regression (**converged**) | MED/LOW | H7.3's 0.25 rev gate is mis-derived; a working fix predicts 0.237 | **VERIFIED + FIXED** — reproduced the attenuation model; raised to 0.35 with the derivation written into the row |
| 9 | contract + regression (**converged**) | LOW/MED | `settle_offset` range excludes the 4.858 m/s tier; a third range appears nowhere in the data | **VERIFIED + FIXED** — probe output confirms −0.0043…+0.0671 overall; restated **per tier** in all four places, `vff_hold` maxima given their tier labels |
| 10 | regression | MED | the terminal latched *torque* grows 1.289× and nothing measures it | **VERIFIED + PARTIALLY FIXED** — confirmed the last frame carries the full decel torque. Added `tor_hold_max` to the probe and to H7.6; firmware unchanged (shared with `buildCatch` — operator territory). IDLE→CLOSED_LOOP scenario recorded as an open question, not asserted |
| 11 | regression | MED | no zero-skips desk row for the new xref in HAND-7 | **VERIFIED + FIXED** — H7.0a added, worded like H4.0b; the xref also added to INST-4 |
| 12 | physics | MED | release-velocity scatter consumes the bracket's 0.215 rev margin | **VERIFIED + FIXED (documentation)** — arithmetic reproduced (ε = +2.6 % reaches 10.60). Carried into C-HAND-2, the bracket test's docstring, and an R5 `v_pk` cross-check. Deliberately NOT folded into the assertion: ε is a plant property, that test guards the config |
| 13 | physics | LOW | the current test parametrises only the flight band; `MAX_EVENT_VEL_MPS = 7.0` wants 65.5 A | **VERIFIED + FIXED** — confirmed 65.5 A of a 50 A limit. New `test_the_feedforward_is_only_sized_over_the_flight_band` asserts both halves; C-HAND-2 scope table declares the clamp out of contract |
| 14 | contract | — | `vff_hold_max = 2.02` rev/s at 2.742 m/s exceeds the pinned `|a_dec|·dt` ceiling | **NOT FIXED — carried as an open question.** The reviewer explicitly did not raise it as a finding and could not prove it; the compiled firmware satisfies the bound when tested directly. Recorded below because if the announcement's `initial_velocity` is not the speed the Teensy was commanded with, every `a_cmd`/`t_dec`/η the probe prints inherits the error |
| 15 | regression | — | plan says "0.21 rev = 6.7 mm" where the contract says "0.215 = 6.8 mm"; test `_GRAVITY = 9.81` vs firmware 9.806 | **REFUTED as findings** — the reviewer itself judged them non-findings (true value 6.79 mm; the constant only picks a test speed). No failure scenario. Left alone |

## Verification

**Full suite** (`pytest tests/ -q`, run 2026-07-29 on the Jetson in the project
venv): **4096 passed, 3 xfailed in 1428.61 s (0:23:48)**, exit 0.

Against the baseline at `ac74c1a` (`pytest tests/ -q`, run 2026-07-28: **4068
passed, 3 xfailed in 1422.44 s**) that is **+28**, accounted for exactly: this
phase's two new files contribute 21 (`tests/sim/test_hand_throw_decel_ff.py`) and
7 (`tests/firmware/test_hand_throw_decel_xref.py`). The **xfail count is
unchanged at 3**, and `git status --porcelain tests/` shows no existing test file
modified — the only tracked changes under `tests/` are the two new files and two
markdown runbooks, which pytest never collects.

**Scoped** (`python -m pytest tests/sim/test_hand_throw_decel_ff.py
tests/firmware/test_hand_throw_decel_xref.py -q`, run 2026-07-29): **28 passed in
1.35 s**.

**Instrument self-check** (`python tools/probes/hand_decel_authority.py
--self-check`, run 2026-07-29): `SELF-CHECK: PASS`, exit 0 — synthetic pre-fix
capture scores FLAG (spread 0.9575 rev, top peak 10.9798), synthetic post-fix
scores ACCEPT (spread 0.0000, top peak 10.3854).

**Mutation verification**, all reverted afterwards and the tree confirmed clean:

* restore the old announcement-anchored `iq_ramp` window → `--self-check` FAILs
  on the window assertion, both sides, and passes everything else;
* raise the declared inertia to 1.014e-5 or 1.0e-5 → **5 tests FAIL** across both
  files, including the new gravity-inclusive band-ceiling claim;
* (from the implementer) revert `throwDecelToTorque` → `accelToTorque` in
  `buildThrow` → 3 xref tests FAIL; apply the corrected conversion to the accel
  segment too → 2 xref tests FAIL.

**Codegen determinism**: `python config/generate_config.py` run twice; the second
run left `config/generated/hardware_config.h` byte-identical.

**Not run here, by standing rule**: anything robot-actuating. No flash, no
launch, no `odrivetool`.

## Deployment

**Both, and skipping either makes § CHECK HAND-7 meaningless.**

1. **Platform Teensy flash** of `Teensy_code/Teensy_code.ino` — `Trajectory.h`
   and the regenerated `Teensy_code/hardware_config.h` changed, `FW_VERSION` goes
   1 → 2. Not the can-bridge, not the CatchingCone.
2. **`colcon build --packages-select jugglebot` + source + relaunch** — the
   regenerated `hardware_config.py` ships inside the ROS package and the launch
   runs the *installed* copy; `rpc_args.PLATFORM_FW_VERSION_EXPECTED` (1 → 2) is
   imported by `teensy_bridge_node`. No `jugglebot_interfaces` change. Codegen
   already run and its artefacts are in the tree.

Then `tests/hardware/session_anomaly_fixes.md` § CHECK HAND-7 / § THE RUN SHEET
stage 8: three desk pre-flights (**H7.0a** xref with zero skips, **H7.0b/INST-6**
probe self-check, **H7.0c** read `torque_soft_min` off the live drive), then the
gated **R0 → R5** ladder.

## Open questions

1. **Is `axis0.config.torque_soft_min` live on the flashed hand drive?** If it is
   −0.0551 N·m the corrected feedforward is truncated above ~0.49 m and this
   phase is a no-op. Probably not binding (achieved decel grows 2.6× across the
   band, which a hard clamp forbids), but unresolved. Pre-flight H7.0c.
2. **Where is the hand's physical end stop, actually?** Three sources disagree —
   `hardware_config.yaml` says "true max ~11.4 rev", geometry gives 11.224, the
   runbook's H4.5 says 11.124. The operator's contact at a measured peak of 11.06
   suggests the stop may sit *below* the declared 11.1 rev guard, i.e. the guard
   is not protective. Every HAND-7 band is at or below 10.60 rev so the ladder is
   safe under all three candidates, but the anchor is still open — as is whether
   what was contacted is a compliant bumper or a hard limit.
3. **The settled post-stroke offset.** Shrinks with speed (+0.067 rev at
   2.742 m/s → ~0 at 4.858) and is unattributed; the Vel_FF hypothesis is
   refuted. Candidates: velocity-integrator residual, or friction holding the
   hand above its setpoint.
4. **Does `vff_hold_max = 2.02` rev/s at 2.742 m/s really exceed `|a_dec|·dt`?**
   One reviewer traced it to the throw's own last frame with a `t3 − t_last` of
   2.18 ms against `dt = 2.00 ms`, consistent independently of `a_dec` — which
   would mean the announcement's `initial_velocity` is not exactly the speed the
   Teensy was commanded with, and every `a_cmd`/`t_dec`/η the probe prints
   inherits that error. The compiled firmware satisfies the bound when tested
   directly. Worth one bench capture with the commanded speed logged alongside.
5. **Should `buildSegment` emit a terminal sample at `t3`?** Today the last frame
   carries up to 7.19 rev/s of Vel_FF *and* the full decel torque, both latched.
   Fixing it changes `buildCatch`'s terminal frame too, hence operator territory.
   Note `test_hand_smooth_move_xref.py` asserts the *opposite* property for
   `makeSmoothMove`, so the two builders currently disagree on a stated hazard.
6. **Should the ascent feedforward also be corrected?** Same missing-rotor error
   plus the ball-in-cup one. Correcting it raises the achieved release velocity
   ~4 % and re-calibrates every throw height — squarely an operator decision.
7. **`buildCommand` (kind 2) is 31.6× too large.** No live host dispatches it,
   but `teensy_bridge_node` still validates and accepts `traj_type 2`, so a future
   caller would inherit it.

## Related

* Contract: `ros_ws/docs/hand_decel_feedforward.md` (**C-HAND-2**); sibling
  `ros_ws/docs/hand_command_continuity.md` (**C-HAND-1**),
  `ros_ws/docs/platform_fw_version.md` (**C-PLATFW-1**)
* Plan: `plans/archived/hand-command-continuity.md` § Phase 7
* Sitting that produced the symptom:
  `logbook/2026-07-28-anomaly-fixes-validation-sitting.md`
* Probe: `tools/probes/hand_decel_authority.py`
* Bench: `tests/hardware/session_anomaly_fixes.md` § CHECK HAND-7, stage 8
## Close-out — 2026-08-21

**Status `in-progress` → `tuned`.** The deployment this entry was waiting on
happened, and the feedforward is confirmed *commanding*: the 2026-08-10
diagnosis measured `tor_ff_cmd` = **−0.1500 N·m on 54/54 strokes**, which is
`throwDecelToTorque` at the declared `J_ff` = 9.5e-6 — the FW-1 spool model would
read −0.11 and the two are not confusable. Platform FW 2 is aboard (FW 3 since
the 2026-08-18 end-stop correction, which does not touch this path).

The phase's own physical claim was then validated at the top of the envelope on
**2026-08-21**: the machine flew to within **0.029 m/s** of the C-HAND-3 ceiling
with coast **0.215 / 0.250 rev** and **18.7 mm of headroom** to the 10.8 rev
stop, and coast measured **flat** in speed (p ≈ 1.0) rather than quadratic —
which is the velocity-independence this feedforward exists to produce.
`logbook/2026-08-21-envelope-flown-to-ceiling.md`.

**`tuned`, not `resolved`, and the reason is finding 4 of this entry's own
review.** The unexamined drive clamp was real: `torque_soft_min` **was** live at
−0.0551 N·m = exactly −10.00 A, so braking `iq_meas` floored at ≈ −11.8 A against
the −27.2 A the feedforward asks for. Diagnosed 2026-08-10
(`logbook/2026-08-10-hand-drive-braking-clamp-diagnosis.md`), and the clamp is
now a symmetric ±0.7 N·m with `save_configuration()` run — so bench row **H7.0c**
is a regression check rather than an open question. The *sibling* investigation
that opened (catch-seat harshness, and how much of the pre-fix behaviour was the
clamp rather than the feedforward) is owned by `plans/active/catch-robustness.md`,
not by this entry.

**Also open, unchanged**: the settled `pos_meas − pos_cmd` offset whose first
explanation this entry withdrew, and the two kind-2 defects
(`buildCommand`'s 31.6× torque unit error, `buildSegment`'s residual velocity
feedforward) which are pinned rather than fixed because no live host dispatches
kind 2.
