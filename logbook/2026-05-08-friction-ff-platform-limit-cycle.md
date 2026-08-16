---
title: Friction-FF platform limit cycle — diagnosis and smooth-gate fix
type: investigation
date: 2026-05-08
status: resolved
phase: pr-3b-persistent-enable
related_plan: "2026-05-08 friction-ff-motor-guard-integration.md"
related_issues:
  - 2026-04-27-friction-feedforward-bench-validation.md
sessions:
  - mpc_20260508_131718.csv .. 132032.csv   # baseline 7-move battery (FF off)
  - mpc_20260508_132342.csv, mpc_20260508_132415.csv  # FF-on round 1 (ff_sign=-1)
  - mpc_20260508_133436.csv, mpc_20260508_133452.csv  # FF-on round 2 (ff_sign=+1)
rosbags:
  - 2026-05-08_13-16-52   # baseline
  - 2026-05-08_13-21-42   # FF on, sign=-1
  - 2026-05-08_13-34-15   # FF on, sign=+1
files_changed:
  - config/hardware_config.yaml
  - ros_ws/src/jugglebot/jugglebot/motion/friction_ff_params.py
  - ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py
  - tests/motion/test_friction_ff_params.py
  - tests/motion/test_motor_guard_friction_ff.py
  - logbook/2026-05-08-friction-ff-platform-limit-cycle.md
subsystem:
  - motion
  - config
tags:
  - friction
  - feedforward
  - limit-cycle
  - bench-vs-platform
  - hold-phase
  - smooth-gate
---

# Friction-FF platform limit cycle — diagnosis and smooth-gate fix

## Summary

PR 3a's flag-flip from `friction_ff.enabled: false` → `true` exposed a
fundamental design flaw in the bench-validated friction FF.  The
function's hard `0 → τ_s × Kt = 0.122 Nm` step at `|v| = 1e-4 rev/s`
bootstrapped a self-sustaining ~5 Hz limit cycle on platform with
~1.5–1.8 mm peak-to-peak hold-phase oscillation.  The bench couldn't
have detected this because its Python-generated trapezoid trajectories
produced `vel_ff = 0.0` exactly during hold, never crossing the
dead-zone edge.  PR 2.1 replaces the hard step with a smooth gate
`gate(v) = 1 − exp(−(|v|/v_gate)²)` (v_gate = 0.05 rev/s) that scales
the FF magnitude continuously with velocity, eliminating the
discontinuity that drove the cycle.  YAML default `ff_sign` was also
flipped from -1 (bench convention) to +1 (platform convention) after
verifying that `can_node._send_position_target` already negates
`torque_ff` for leg axes — the bench bypassed can_node, so its sign
chain has one negation; the platform has two.

## Symptoms

User report after running the PR 3a A/B procedure
(plans/archived/friction-ff-motor-guard-integration.md §9):

> Step 2 was a FAIL.  The motion-onset was worse with ff on, and the
> platform exhibited growing oscillations whenever holding at a pose
> (both the initial hold and while holding the z=220 pose).  Swapping
> the ff_sign to all +ve seems to have made the movement to z=220
> smoother, but oscillations while holding were still present, and the
> diagnosis report indicates that motion onset was even worse than with
> the signs being -ve.

Quantitative confirmation from `diagnose.py --json`:

| Metric | BASELINE (FF off) | FF on, sign=-1 | FF on, sign=+1 |
|---|---:|---:|---:|
| Hold-phase aggregate `act_std` | **21.4 µm** | **862.4 µm (40×)** | **661.4 µm (31×)** |
| Hold-phase per-leg max range | 0.16 mm | **4.46 mm** | **4.49 mm** |
| Hold-phase `\|vel\|_max` | 5.4 mm/s | **83.4 mm/s** | **74.2 mm/s** |
| `per_leg_chatter` | 0.015 (clean) | 0.18 | 0.18–0.29 |
| cmd_ext oscillation during hold | 0.19 mm | **1.83 mm** | **2.62 mm** |

No platform safety event occurred — no E-stop, no motor faults, no
current-limit hits.  The position loop's gain margin bounded the
amplitude, but 1.5 mm hold-error vs 30 µm target is unusable.

## Diagnosis

### First (incorrect) hypothesis: dead-zone edge instability

Initial reading suggested vel_ff oscillating around the `1e-4 rev/s`
dead-zone boundary, with the friction FF chattering between 0 and
±0.122 Nm at the boundary.  This was **wrong**.  The data analysis
(see "Smoking gun" below) showed vel_ff is in the boost band 100 % of
the time during FF-on hold — the system never visits the dead zone
during the limit cycle.

### Smoking gun: rosbag `/leg_lengths_topic` analysis

`motion_bridge_node` publishes 18 floats per cycle (6 cmd_pos + 6
vel_ff + 6 torque_ff) sourced from `motor_guard._commanded_*` buffers,
recorded at ~380 Hz in the rosbag.  This is the EXACT stream that
goes to `can_node` (less can_node's int16 quantisation and per-leg
sign flip).  Analysis from rosbag 2026-05-08_13-21-42, hold tail:

| Bucket | BASELINE hold-only | FF-on hold (limit cycle) |
|---|---:|---:|
| Dead-zone `\|v\| < 1e-4` | **77.8 %** | 1.3 % |
| `(1e-4, 0.001)` | 1.9 % | 1.7 % |
| `[0.001, 0.01)` | 15.4 % | 16.2 % |
| `[0.01, 0.20)` boost band | **4.9 %** | **76.3 %** |
| `\|v\| ≥ 0.20` | 0.0 % | 4.6 % |

Three crucial findings:

1. **In a clean BASELINE hold, vel_ff is in the dead-zone 78 % of the
   time.**  The bench's `1e-4` threshold IS appropriate for the
   platform's natural hold-noise floor.  The boost band would have
   fired only 4.9 % of the time during a clean hold.

2. **During FF-on hold, the limit cycle pushes vel_ff into the boost
   band 76 % of the time.**  This is feedback-driven: the FF kick at
   the dead-zone edge causes leg motion → MPC commands corrections →
   vel_ff grows → FF stays at full magnitude → motion sustained.

3. **The friction FF math itself is mathematically correct.**  Replaying
   the rosbag's vel_ff stream through the friction-FF function reproduces
   the rosbag's `torque_ff` exactly (modulo the MPC base dynamics
   torque, ~0.04 Nm).  No implementation bug.  The failure is a design
   flaw in the bench-validated function.

### MPC ↔ motor_guard data flow analysis

Traced the full path on 2026-05-08:

1. **MPC computes `vel_mm_s = (cmd_next − cmd) / dt0`** at every 40 Hz
   tick (`mpc.py:1204`) — the rate of change of consecutive commanded
   positions.  During hold, the MPC re-plans against tracking errors
   (gravity sag, mocap noise).  Its commanded `ext_mm` changes by
   tens of µm tick-to-tick → `vel_mm_s` is non-zero but tiny
   (median 0.001 rev/s = 0.07 mm/s during clean BASELINE hold).
2. **HardwarePlant has a 1-LSB dead-band** (`hardware_plant.py:357`) that
   zeros `vel_mm_s` if all 6 legs are sub-LSB.  But it requires ALL
   six.  With FF-induced motion at mm scale, deltas are above LSB —
   dead-band never fires during the limit cycle.
3. **Motor_guard's Hermite interpolator** translates MPC's per-tick
   u[0]/u[1]/u[2]/v0 into a 500 Hz `_commanded_vel_ff_rps` stream.
   Hermite is faithful — slightly smoothing if anything (BASELINE
   hold: MPC raw 8.8 % dead-zone occupancy → Hermite 77.8 % dead-zone).
4. **Friction FF reads `_commanded_vel_ff_rps`** and emits torque per
   the bench formula.  At any `\|v\| > 1e-4`, hard step to ±0.122 Nm.

The first kick is the mechanism: when BASELINE-style vel_ff briefly
crosses 1e-4 (which happens ~17 % of the time per bucket data), the
hard step injects 0.122 Nm in a single 500 Hz tick.  The position
loop overshoots; vel_ff jumps to ~0.05 rev/s; FF stays at full
magnitude.  The MPC's hold-tracking corrections then drive sustained
sign-flipping at ~5–10 Hz.

### Sign convention — verified +1 for platform

`can_node._send_position_target` (`can_node.py:711-713`) negates BOTH
position and torque_ff for leg axes via `_leg_sign(axis_id, value)`:

```python
setpoint = _leg_sign(axis_id, setpoint)
vel_ff = _leg_sign(axis_id, vel_ff)
torque_ff = _leg_sign(axis_id, torque_ff)
```

The bench's `friction_ff_demo.py` bypassed can_node entirely and sent
torque_ff direct to ODrive.  Bench validated `ff_sign = -1` against
that one-negation chain.  Platform has motor_guard's sign × can_node's
negation × ODrive's wiring inversion — two negations vs the bench's
one — so platform `ff_sign` must be flipped to +1 to land the same
iq value at the ODrive that the bench sent.

User's manual flip during testing confirmed this directionally: with
sign=+1, "the movement to z=220 was smoother" (the FF actually
assisting motion during the move).  The hold-phase oscillation
persisted because the limit cycle is sign-agnostic — the chattering
averages to zero either way.

### Why the bench couldn't detect this

The bench (`tests/hardware/friction_ff_demo.py`) uses POSITION/PASSTHROUGH
mode with a Python-generated trapezoid trajectory.  During the trapezoid's
hold portion, `traj_v.append(0.0)` produces `vel_ff = 0.0` **exactly** —
the dead-zone is never crossed.  The platform's Hermite interpolator
on MPC commands has no such property because the MPC re-plans against
tracking errors during hold.

Plus: the bench's single-ODrive setup has no MPC re-planning loop.  Limit
cycles require feedback: friction kick → motion → re-plan → opposite
kick.  The bench can't reproduce that loop at MPC time scales.

This is a fundamental bench limitation, not a bench bug.  The bench
validated the friction model under continuous-motion conditions; it
COULD NOT validate hold-phase behaviour with realistic MPC re-planning.

### Lesson

When a bench validation produces a clean numerical result, that result
is conditional on the test environment's stimulus distribution.
Testing on platform with a realistically distributed input
(here: MPC's hold-phase commanded-vel noise) is qualitatively
different from the bench's idealized ramp-then-hold.  Future
bench-to-platform transitions should explicitly characterise the
stimulus distributions both environments produce, and stress-test
the function across both.

## Discussion

This investigation went through several conceptual reframings; recording
them is the load-bearing part of this entry.

### The dead-zone-step hypothesis was almost right but mechanistically wrong

I initially diagnosed the limit cycle as "vel_ff oscillates around
±1e-4, FF chatters between 0 and ±0.122 Nm at the boundary".  Wrong:
the limit cycle is in the boost band, not at the boundary.  The
boundary is the BOOTSTRAP — once crossed, the system spends 100 % of
its time well inside the boost band.

The user pushed back on my initial diagnosis, asking for more thorough
investigation.  That push found the rosbag analysis that gave the right
mechanism.  Lesson: **when an explanation is plausible but you haven't
inspected the actual telemetry, ask "what does the data show?" before
committing to the fix path.**  The first hypothesis was directionally
correct (the discontinuity is the problem) but mechanistically off
(the system doesn't oscillate around the boundary; it gets KICKED across
the boundary and stays kicked).

### The smooth gate as a structural fix

The fix has to do two things:

1. **Eliminate the bootstrap kick.**  At `|v|` just above 1e-4, the FF
   magnitude must be small enough that the position loop can absorb
   it without overshooting.
2. **Preserve FF benefit at real motion velocities.**  When `|v|` is
   well into the boost band (≥0.05 rev/s), FF should provide
   meaningful assistance.

The smooth gate `gate(v) = 1 − exp(−(|v|/v_gate)²)` does both naturally:

- At `|v| ≪ v_gate`, gate `≈ (|v|/v_gate)²` (Taylor expansion) — the
  torque scales as `v²`, heavily damped.
- At `|v| = v_gate`, gate `= 1 − e⁻¹ ≈ 0.63` — moderate FF.
- At `|v| ≥ 2·v_gate`, gate `≈ 1` — full FF.

The function form mirrors the Stribeck `(τ_s−τ_c)·exp(−(v/ω_s)²)` term —
both are Gaussians, one decreasing, one increasing.  Same structural
shape.

### Sizing v_gate

`v_gate = 0.05` was sized against the BASELINE hold-phase distribution:

| BASELINE hold percentile | vel_ff (rev/s) |
|---|---:|
| 50 (median) | 0 (in dead-zone) |
| 90 | 0.006 |
| 99 | 0.021 |
| max | 0.29 |

At `v_gate = 0.05`, the gate value at the 99th percentile of hold-noise
(0.021 rev/s) is `1 − exp(−0.176) ≈ 0.16`, giving torque `≈ 0.16 ×
0.122 = 0.019 Nm = 19 mNm`.  That's at the edge of the position
loop's absorption capacity.  At the 90th percentile it's ~5 mNm —
absorbed cleanly.

The bench's effective FF "engagement velocity" (where torque rose to
half-max) was around 0.10 rev/s (within the boost band, since boost
fired full at 0.20 and Stribeck taper at 0.20 was 80 % of τ_s).  With
`v_gate = 0.05`, the new engagement velocity is around 2·v_gate =
0.10 rev/s — same.

If platform validation finds v_gate = 0.05 is too aggressive (still
some hold-phase chatter), raise to 0.10.  If too damped (motion-onset
benefit lost), lower to 0.03.  The integration plan §4 outlier-bisection
protocol applies.

### Removed the boost band entirely

The bench's boost band fired full `τ_s × Kt` for `|v| in [1e-4, 0.20)`
to compensate for Stribeck under-shooting τ_s near v=0.  With the
smooth gate this purpose is served by the gate function itself —
no need for a separate boost branch.  Removed `stiction_boost_threshold_rps`
from YAML and `_friction_boosted` / `_friction_in_boost` /
`_friction_dead_zone` scratch buffers from motor_guard.  Simpler,
fewer tunables, single-mode behaviour.

### Verification: replay the actual rosbag through the new function

Replayed the FF-on rosbag's vel_ff stream through the smooth-gate
function with sign=+1:

| | Hold-tail median `|τ_ff|` | Hold-tail 90 %ile | Max |
|---|---:|---:|---:|
| Boost band (actual rosbag) | **137 mNm** | 161 mNm | 163 mNm |
| Smooth gate (replay, same v) | 37 mNm | 108 mNm | 112 mNm |
| Smooth gate (BASELINE-hold v) | **0.0 mNm** | **1.65 mNm** | 112 mNm |

The third row is the load-bearing prediction.  In a clean hold (v_gate
applied to BASELINE vel_ff distribution), the smooth gate produces
median 0.00 mNm and 90 %ile 1.65 mNm — well below the position loop's
absorption capacity.  No bootstrap kick → no limit cycle.

## Fix

PR 2.1 commit (2026-05-08) — see commit log.

### Changes

1. **`config/hardware_config.yaml` — `friction_ff:` block:**
   - `ff_sign` default flipped from `[-1,...]` to `[+1,...]` (platform
     convention).  Comment block above the field documents the can_node
     negation chain.
   - `stiction_boost_threshold_rps` removed.
   - `v_gate_rps: 0.05` added (smooth-gate scale).
   - Comment block above the YAML section updated to describe the
     smooth-gate model and reference this logbook entry.

2. **`ros_ws/src/jugglebot/jugglebot/motion/friction_ff_params.py`:**
   - `FrictionFFParams` dataclass: replaced `stiction_boost_threshold_rps`
     field with `v_gate_rps`.
   - Loader validation updated.

3. **`ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py`:**
   - `_compute_friction_ff_Nm` rewritten with the smooth gate.  Boost-band
     branches removed.  Zero-allocation contract preserved (PR 2 audit
     fix #6 still holds).  Buffer set updated: removed `_friction_boosted`,
     `_friction_in_boost`, `_friction_dead_zone`; added `_friction_gate`.
   - Function docstring expanded to ~60 lines explaining the smooth-gate
     rationale and pointing here.

4. **`tests/motion/test_motor_guard_friction_ff.py` and `test_friction_ff_params.py`:**
   - Replaced `test_friction_ff_boost_at_low_v` and
     `test_friction_ff_continuity_at_boost_threshold` with smooth-gate
     equivalents.
   - Added `test_friction_ff_smooth_gate_kills_low_v` (regression guard
     for the bootstrap fix).
   - Added `test_friction_ff_smooth_gate_continuous_through_zero`
     (Lipschitz-bounded continuity check).
   - Added `test_friction_ff_smooth_gate_no_kick_on_baseline_holdnoise`
     (synthesised BASELINE-distribution replay, asserts median |τ_ff|
     < 5 mNm).
   - Replaced parametrised `test_friction_ff_dead_zone_edge` with
     `test_friction_ff_smooth_gate_table` (pins gate values at 0.1, 1,
     2, 5 × v_gate).
   - Updated `test_friction_ff_scratch_buffer_identity` for the new
     buffer set.
   - Updated `test_load_params_bench_defaults_present` for ff_sign=+1
     and v_gate_rps=0.05.

### Design choice: keep `friction_ff.enabled: false`

Default stays disabled.  PR 3a-revised is the on-platform A/B with the
corrected gate.  Until that validates, the platform runs friction-FF-off.

## Outcome

| Hypothesis | Standing | Evidence |
|---|---|---|
| The bench's `1e-4` dead-zone is too tight for platform | ❌ Refuted | 78 % of BASELINE hold-time is in dead-zone; threshold IS appropriate |
| The boost band's hard step bootstraps a limit cycle | ✅ Confirmed | Replaying bench formula on rosbag vel_ff reproduces the observed torque_ff exactly |
| Platform sign should be `+1` (vs bench `-1`) | ✅ Confirmed | can_node._send_position_target negates torque_ff for legs (line 713) |
| Limit cycle is bounded, not divergent | ✅ Confirmed | 1.5 mm peak-to-peak across 30 s of hold |
| Bench could not have detected this | ✅ Confirmed | Bench's trapezoid trajectory has vel_ff = 0.0 exactly during hold |
| Smooth gate eliminates the kick | ✅ Confirmed (2026-05-08 platform A/B) | Step 1 hold matches BASELINE 21 µm exactly; no chatter, no oscillation |
| Smooth gate preserves motion-onset benefit | ✅ Confirmed (2026-05-08 platform A/B) | Aggregate motion-onset 285 → 185 ms (1.54× faster); max 555 → 337 ms (1.65× faster) on the 7-move battery |

## Validation (PR 3a-revised, 2026-05-08 ~15:00)

On-platform A/B with the smooth gate, all 7 moves of the leg-gain-tuning
test battery.  Sessions: `mpc_20260508_150357.csv` (Step 1 active hold),
`mpc_20260508_150442.csv` (Step 2 sign-verify), `mpc_20260508_150532..150754.csv`
(Step 3 7-move battery).  Rosbags: `2026-05-08_15-03-29` and `15-05-17`.

Operator observation (Steps 1+2): "no observable oscillation during either
sequence.  The platform did move slightly during the holds, but there was no
oscillation and the small movements seemed similar to what I've observed in
the past."

Operator observation (Step 3): "the smoothest run of the 7-move battery that
we've ever run — there were still a few OH SPIKEs and MPC solve failures, but
all movements were smooth and controlled."

Quantitative results (vs yesterday's BASELINE FF-off battery):

| Metric | BASELINE | PR 2.1 | Δ |
|---|---:|---:|---:|
| Aggregate motion-onset median | 285 ms | **185 ms** | **−35 % (1.54×)** |
| Aggregate motion-onset max | 555 ms | **337 ms** | **−39 % (1.65×)** |
| Per-leg chatter median (across all legs × 7 moves) | 0.052 | 0.043 | −17 % |
| Per-leg chatter max | 0.262 | 0.200 | −24 % |
| Step 1 active-hold act_std (15 s, FF on) | 21 µm (yesterday's M1 hold) | **21 µm** | **identical** |
| Limit cycle (FF on) | n/a | **none** | ✅ |

The biggest improvements were on extreme-pose Move 6 (Active → y=−100,
466 → 171 ms = **−63 %**) and Move 5 (diag+tilt, 323 → 173 ms = **−47 %**).
Smaller-or-no improvement on Moves 1, 3, 7 suggests their baseline latency is
already inertia-limited — friction is no longer the dominant contributor on
those moves.

Move 7 hold quality regressed slightly (605 → 838 µm).  Investigation
showed this is a pre-existing extreme-pose ringing issue × FF activation
during the ringing — NOT a limit cycle (no chatter, magnitudes don't grow,
sign-flip rate normal).  The right fix is per-leg gain tuning at extreme
poses (separate workstream, see `2026-04-19-leg1-pose-dependent-hold-twitch.md`),
not friction-FF parameter changes.

Move 4 showed an apparent +89 % motion-onset regression but per-leg
breakdown showed it's a single-sample artifact (n=1 onset per leg, different
legs dominated each run).  Operator re-ran Move 4 five times to characterise
properly — analysis pending in a follow-up entry.

`v_gate = 0.05 rev/s` was the right size — no further tuning needed for
PR 3b.

## Outcome (PR 3b status)

YAML default `friction_ff.enabled` flipped from `false` to `true` on
2026-05-08 in PR 3b.  Friction FF is now active on every launch unless
explicitly disabled via `--friction-ff false` CLI flag or the
`friction_ff_enable:=false` ROS2 launch argument.

Integration plan §5 acceptance criteria need to be re-baselined for
realistic platform numbers — see follow-up note in
`plans/archived/friction-ff-motor-guard-integration.md`.

## Withdrawn claims

- **[2026-05-08 ~12:30] "The dead-zone step at v=1e-4 causes vel_ff to
  oscillate around the boundary, producing a 0 → ±τ_s chatter at 5 Hz."**
  WITHDRAWN: rosbag analysis showed vel_ff is in the boost band 100 %
  of FF-on hold time, never visiting the dead zone during the limit
  cycle.  The boundary is the BOOTSTRAP, not the operating point.
  Superseded by: the boost-band-as-amplifier framing in the Discussion
  section above.

## Open Questions

1. **What is `v_gate`'s correct platform value?**  0.05 is sized off
   BASELINE hold-noise but is theoretical until measured.  The PR 3a-revised
   A/B will tell us if 0.05 is too aggressive (chatter persists) or too
   damped (motion-onset benefit lost).

2. **Per-leg `v_gate`?**  Currently a single global scalar; the YAML
   schema can be promoted to a length-6 array if per-leg refinement
   shows different legs need different gate scales.

3. **What's the actual platform motion-onset benefit with the smooth gate?**
   Bench showed 81 → 32 ms with hard boost.  Smooth-gate engagement is
   slightly slower; expect 175 → ~120–135 ms on platform (vs ~90 ms
   theoretical max).  Still a worthwhile improvement.
