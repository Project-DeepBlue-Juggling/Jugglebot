---
title: "BUG-57: hand current limit 60 -> 70 A, stream STOP 3500 -> 4500 (FW 36), REGEN on bus power less copper loss, and C-HAND-6, a top-of-stroke margin for every rising seed (K 4.75e-5, the largest satisfiable fit; the owner's 200 rev/s fit has no solution)"
type: bugfix
date: 2026-11-02
status: open
phase: "BUG-57 (owner decisions, 2026-11-02)"
subsystem: motion
related_entries:
  - 2026-11-02-hand-release-step.md
  - 2026-11-02-hand-odrive-limits-backup.md
  - 2026-11-01-hand-limit-4500-and-stroke-floor.md
  - 2026-11-02-jb146-hand-stall-guard.md
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/throw_envelope.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/docs/hand_throw_envelope.md
  - tools/probes/release_step_shape.py
  - tools/probes/hand_stall_stop.py
  - tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py
  - tests/motion/test_release_step.py
  - tests/motion/test_throw_envelope.py
  - tests/motion/test_magazine_throw.py
  - tests/firmware/native/test_leg_interp.cpp
  - tests/ros/test_teensy_bridge_node_hand_jam.py
  - sim/skills_gate.py
---

# BUG-57: 70 A, a 4500 STOP, REGEN with copper loss, and a top margin

**Summary.** At the first Apex 1.35 Columns goal (Sitting 2026-11-02 17:00),
one launch overshot. The stepped brake (C-HAND-5) saturated at -60.4 A and the
Hand struck the top stop at 10.83 rev (SPINOUT). The owner decided:

1. `hand_curr_limit_a` 60 → 70 A, the drive's own soft max.
2. The brake plans to a box top lowered by a margin that grows with speed
   (C-HAND-6), for ANY window whose seed cup is rising, not only the
   post-release ones.
3. `stream_stop_hand_accel_rps2` 3500 → 4500 rev/s², so the firmware's stall
   stop is as hard as the planned brake. It rides in can-bridge FW 36 with
   BUG-56 (one bump, one flash).
4. The REGEN envelope bound models the motor's copper loss.
5. The margin is ONE constant `TOP_MARGIN_K_REV_S2`, fitted at 200 rev/s.

The box is not re-swept and nothing is flown.

## What the bag says

Source: the incident bag, `/hand_telemetry` at 100 Hz, Jetson-stamped. Times are
log time minus 1793599800 s.

| t (s) | pos_cmd | vel_ff | pos_meas | vel_meas | iq (A) |
|---|---|---|---|---|---|
| 1.533 | 5.484 | 152.2 | 4.325 | 136.5 | 26.7 |
| 1.552 | 7.955 | 96.8 | 7.441 | 168.3 | 4.4 |
| 1.561 | 8.790 | 70.6 | 8.843 | 109.1 | -57.1 |
| 1.570 | 9.380 | 48.3 | 9.742 | 79.7 | -57.1 |
| 1.580 | 9.769 | 30.3 | 10.505 | 77.7 | -60.4 |
| 1.590 | 10.000 | 16.7 | 10.832 | -44.6 | -60.4 (frozen) |

- At the release the Hand was **1.16 rev behind its command**: about 8 ms of
  lag at 140 rev/s. The launch started from a Catch that dived to 0.03 rev.
- Across the step, the position loop was still pushing the Hand forward to
  close that gap. At t = 1.552 the feedforward asked for -0.124 N·m, but the
  measured iq was +4.4 A. The brake did not start until the Hand passed its
  command, around 8.8 rev, and by then it was 71 rev/s faster than the
  command.
- The "0.67 rev past the planned top" in BUG-57 is truncated by the hard stop.
  At 10.505 rev the Hand was still moving at about 76 rev/s under -60 A. At
  the 60 A decel-FF bound (4711 rev/s²), or the 3700 rev/s² seen in the
  position steps, it would have stopped at **11.1-11.3 rev**. That is an
  overrun of about **1.05 rev** past the planned top 10.16.

## Discussion

### The margin model

**Headroom model.** The Hand reaches the step with a velocity error `Δv` above
its command. Feedback can remove that error only with the drive's deceleration
headroom above the planned brake:

    a_head = a_sat(I) − a_b(v, m)
    O      = Δv² / (2 · a_head)            overrun past the commanded top

- `a_sat(I)` is the decel-FF bound `0.85 · I · Kt / (2π J_ff)`: 4710.6 rev/s²
  at 60 A and 5495.7 rev/s² at 70 A.
- `a_b` is the planned brake: the seed `1.10 · v² / (2 (D0 − m))`, clamped at
  the stream STOP (now 4500).
- `D0` = planned top − release = 10.16 − 5.48 = 4.68 rev.
- A margin is admissible only if `T0 − m + O(v, m) ≤ clip − cushion`
  (10.501 − 0.10 = 10.401) AND the plan itself can stop inside `D0 − m` at the
  4500 hand limit, `v² / (2 (D0 − m)) ≤ 4500`.

**Calibration (one incident).** v = 152 rev/s at 60 A with m = 0: a_b = 2715,
a_head = 1995, and the overrun O = 1.05 rev needs Δv = 64.7 rev/s, so
**Δv = 0.426·v** (the bag's Δv at t = 1.552 is 71 rev/s). The seed was not
clamped there, so the 4500 STOP leaves the calibration unchanged.

**The 4500 STOP changes the shape of the answer.** Under the 3500 clamp the
seed stopped rising once it hit 3500, so a larger margin bought stroke without
spending headroom. With 4500 the seed follows the margin up, and above about
155 rev/s each extra rev of margin raises `a_b` enough to cost more overrun
than it saves. Required margin `m_req` (smallest m meeting the cushion), 70 A:

| v (rev/s) | m_req, STOP 3500 | m_req, STOP 4500 | best stop with STOP 4500 (rev) |
|---|---|---|---|
| 132 | 0.232 | 0.232 | |
| 139 | 0.333 | 0.333 | |
| 149 | 0.552 | 0.552 | |
| 152 | 0.655 | 0.655 | |
| 155 | 0.802 | 0.802 | |
| 157 | | 0.978 | |
| 158 | 0.893 | **none** | 10.419 |
| 161 | 0.937 | **none** | 10.52 (m 2.0, plan-infeasible) |
| 170 | | **none** | 11.198 |
| 200 | | **none** | 13.567 (m ≤ 0.236 for a feasible plan) |

**The owner's fit at 200 rev/s has no solution.** At 200 rev/s a plan that
brakes at ≤ 4500 needs 4.44 of the 4.68 rev above the release, so m ≤ 0.236,
and the model's best stop is 13.57 rev. The largest speed with a solution is
**157.7 rev/s, m_req 1.180 rev**, and K is fitted there:
**`TOP_MARGIN_K_REV_S2 = 4.75e-5`**, with `TOP_MARGIN_MAX_REV` raised 1.0 →
**1.9** so the clamp does not cut K·v² below 200 rev/s.

| v (rev/s) | margin (rev) | model stop (rev) | seed a_b |
|---|---|---|---|
| 132 | 0.828 | 9.858 | 2487 |
| 139 | 0.918 | 9.899 | 2824 |
| 149 | 1.055 | 10.052 | 3367 |
| 158 | 1.186 | 10.420 | 3928 |
| 161 | 1.231 | 10.654 | 4132 |
| 175 | 1.455 | 11.50 | 4500 |
| 192 | 1.751 | 11.77 | 4500 |
| 200 | 1.900 | 11.91 | 4500 |

The fit sits on the steep edge of the curve (m_req goes 0.80 → 1.18 between
155 and 157.7 rev/s), so it over-covers everything below it: at 139 rev/s it
lowers the top 0.92 rev where the model asks 0.33. Above 157.7 nothing
satisfies the model. For comparison, K 3.6e-5 (the 161 rev/s fit under the old
3500 clamp) gives 0.70 rev at 139 and stops the model at 10.617 at 161, about
the same as this K's 10.654 there.

**Assumptions to check:**

1. The velocity error scales with v (a lag in time).
2. The current-rise lag and the feedback fight before the Hand passes its
   command are folded into the single 0.426 factor.
3. The decel-FF bound is the deceleration the saturated drive actually gets.
4. The planned brake follows its seed. The QP finishes the stop above the
   seed when the seed is clamped, so above the clamp the model is optimistic.
5. One incident calibrates everything. The other ~190 launches that Sitting
   topped out at 10.16-10.23 with at most -36 A.

### Any rising seed (owner decision)

`post_release_cup_config` now lowers the QP box top for every seed with
`v_z > 0`, sized from the seed's own `|v|`. A post-release seed always rises,
so those windows are bit-identical to the post-release-only rule. A seed at
rest or moving down keeps the very config object. An interior seed mid-brake
or mid-launch gets the margin of the speed it still has, which is never more
than the release's, so the plan it replaces stays inside the box.

In practice `splice_at` refuses a splice inside the 2-knot detach cone, and at
Apex 1.05 the brake is over in about 2.3 knots (139 → 73 → 20 → -14 rev/s), so
the stream has no mid-brake splice knot today. The new test plans from the
interior seed one knot into the brake (73 rev/s, `post_release=False`) and
checks that the QP got the lowered top.

### REGEN with the copper loss (owner decision)

The old fence was `I_lim · Kt · ω ≤ 360 W` (the drive's `dc_max_negative_current`
-8 A × 45 V; no brake resistor). At 70 A that refused every throw over
4.835 m/s (Apex 1.19). The bus sees the mechanical power less the winding's
loss, `P_bus(I) = Kt·I·ω − c·R·I²`, with R = 0.0400 Ω (the drive's phase
resistance, new YAML key `hand_phase_resistance_ohm`). The fence takes the
worst case over 0 ≤ I ≤ I_lim: an interior maximum `(Kt·ω)²/(4cR)` at
`I* = Kt·ω/(2cR)` while I* is under the limit.

- **Convention.** c = 1.5 for amplitude-invariant dq currents, 1.0 for
  power-invariant. The open ODrive v3 FOC uses the amplitude-invariant Clarke
  form (`I_alpha = I_a`), but the ODrive Pro firmware is closed and was not
  checked here. The code takes **c = 1.0**, the smaller loss, as the brief
  asks.
- **Where it binds.** 7.467 m/s at c = 1.0 (8.783 at c = 1.5), outside the
  7.0 m/s wire band. It binds nowhere in the band. The envelope ceiling is
  `DECEL_FF_HEADROOM` at 6.768 m/s (flight 1.379 s, Apex 2.33 m), far above
  the box's Apex 1.40. At 4.835 m/s the bus sees at most 165 W, not 360.
- The 300 W steady-state check needs no loss term of its own: it bounds the
  average by the burst fence times the duty, and the fence now bounds bus
  power with the loss already out.

### The 4500 STOP (owner decision)

The firmware's stream-cover-exhaustion stop decelerates the Hand at up to
`max(4500, |a_entry|)`. The jerk bound stays 350000 rev/s³ (about 1.9× the
knot jerk at 4500), so a stop whose jerk binds is longer, never harder. The
brake seed's clamp is `min(hand limit, STOP)` = 4500 from both sides. The
C-HAND-6 brakes now peak above 3500 from Apex 1.20, so the old 3500 STOP would
have handed a stalled brake a gentler stop than the plan.

### Alternatives considered

- **A constant margin.** Simplest, but it overpays at low Apex.
- **The full headroom model in code**, a fixed point per release. It diverges
  where the model has no solution (above 157.7 rev/s), which is exactly the
  high-Apex case.
- **Keep the seed clamp at 3500 while the STOP goes to 4500.** The model then
  has solutions up to 161 rev/s (K 3.6e-5), but the QP finishes the stop
  above a clamped seed anyway (3590-3804 at Apex 1.42-1.45 under 3500), so
  that model was optimistic in the same place.
- **Fixing the cause.** The 1.16 rev launch lag is the root and may be
  specific to a launch off a deep Catch. Still open on BUG-57 (higher-rate
  capture, a firmware overspeed guard, the IDEA-17 bumper). The margin is
  defence in depth, not the fix for the lag.

### Accepted: the brake peak rises, and the box loses its top cells

The brake now plans to a much lower top, so it is the whole-plan peak from
Apex 0.95 up. Apex 1.50 plans at 4479 rev/s²; **Apex 1.52 and up are refused
HAND_LIMIT_ACC**. The U2b torque feedforward scales with the brake: by
proportion to C-HAND-5's -0.21 N·m at brake 3083, the 4075 brake at Apex 1.40
asks about -0.28 N·m, over the 0.234 N·m clamp; the feedback carries the rest.
This is an estimate, not a measurement.

## Control-cycle walk-through (one release at Apex 1.05, 139 rev/s)

1. The THROW window plans the launch exactly as before (launch 2366).
2. `_meta_for` records the brake seed on the release mark, sized to the box top
   less 0.92 rev (29.9 mm), so `B = 1.10 · v² / 2d` rises from about 2017 to
   about 2810 rev/s² (vertical), under the 4500 clamp.
3. The next window (SETTLE tail, folded STEADY, a splice on the release knot)
   is seeded with the take-off velocity, rising, so `post_release_cup_config`
   lowers its QP box top by the same 0.92 rev.
4. The QP continues from the seed with continuous acceleration and stops at
   9.221 rev instead of 10.143. Brake peak 2269 → 2809 (kappa 1.10).
5. In the stream the Hand's acceleration still steps once, at the release
   knot, to the larger brake seed. A stall mid-brake hands the Hand to the
   4500 STOP, harder than this brake. Nothing else in the cycle moves: the
   knot grid, the gate's HAND_STROKE ceiling and the Platform (the step is
   along the cup axis) are unchanged.

## Probe: brake cost (`python tools/probes/release_step_shape.py 0.95 1.05 1.2 1.35 1.40 1.45 1.5`, 2026-11-02)

Release 860, hand 4500, step on. "Before" is origin/main 630e8aaf (60 A, STOP
3500, no margin); "after" is this integration (70 A, STOP 4500, K 4.75e-5).

| Apex | v (rev/s) | margin (rev) | brake top (rev) | THROW brake before → after | STEADY brake before → after | whole-plan peak after |
|---|---|---|---|---|---|---|
| 0.95 | 132.5 | 0.834 | 10.143 → 9.309 | 2046 → 2484 | 2115 → 2507 | 2507 |
| 1.05 | 139.3 | 0.922 | 10.143 → 9.221 | 2261 → 2809 | 2269 → 2809 | 2809 |
| 1.20 | 149.0 | 1.054 | 10.143 → 9.089 | 2584 → 3326 | 2611 → 3326 | 3326 |
| 1.35 | 158.0 | 1.186 | 10.143 → 8.957 | 2907 → 3881 | 2940 → 3881 | 3881 |
| 1.40 | 160.9 | 1.230 | 10.143 → 8.913 | 3014 → 4075 | 3043 → 4075 | 4075 |
| 1.45 | 163.7 | 1.274 | 10.143 → 8.869 | 3122 → 4275 | 3143 → 4275 | 4275 |
| 1.50 | 166.5 | 1.318 | 10.143 → 8.825 | 3230 → 4479 | 3230 → 4479 | 4479 |

All seven pass the gate. Launch peaks are unchanged. Apex 1.52, 1.55, 1.58,
1.60 and 1.65 are refused HAND_LIMIT_ACC with the margin on.

## 70 A ripple (owner decision)

- `throw_envelope` derives everything from the config: DECEL_FF_HEADROOM
  5495.7, DECEL_AUTHORITY 5849.8, ACCEL_AUTHORITY 4569.3 rev/s².
- `hand_acc_limit_rps2` and the ceiling stay at 4500.
- With REGEN on bus power the envelope's speed ceiling is DECEL_FF_HEADROOM
  6.768 m/s, so a re-sweep is not cut at Apex 1.2 by the envelope.

## Verification

- Admissible box re-sweep, 2026-11-02 20:01-22:34. Command:
  `WT=<worktree> OUT=temp/sweeps/h70 bash sweep_h70.sh`, which is the h4500
  runner plus the pose Columns families at Apex 1.00-1.50; logs are in
  `temp/sweeps/h70/`. Results:
  - Rounds a and b were byte-identical. The box went from 100 to 104 cells,
    gate_hash 242c56b683b5.
  - Unchanged: every pose oval, turned, high-low and plain Columns
    (0.85-0.95) cell.
  - Added: the 1.45 and 1.50 Columns cells, both admissible.
  - Moved:
    - Hop cells in both modes;
    - level self_toss, Columns and oval cells (level oval Dwell 0.50 at 0.95
      is now empty; level oval 0.45 at 1.10 is now admissible);
    - six high pose Columns cells. The Apex 1.25 cells (both Sites) and P1 at
      1.40 were selected with a 0 x 0 mm landing rectangle, and P2 1.30 / 1.40
      cap the commanded Apex lower. That is the band-for-landing trade BUG-54
      replaces, so do not fly 1.25 or 1.40 Columns until BUG-54's sweep lands.
- Full tier (2026-11-02, `./run_tests.sh --full`, after rebasing onto
  f81984b2): 8014 passed, 56 skipped (parallel 396 s) + 8 passed (serial),
  with 2 failures:
  - `test_item_bodies_are_readable` fails on FEAT-38, another session's Item,
    not this change.
  - f5's new `test_s2_l1489_the_bumped_successor_plans_inside_the_hand_limit`:
    its fail-before half (a stale seed must refuse) no longer refuses with the
    margin on. The test now runs margin OFF, the shape it was measured on.
    `pytest tests/motion/test_skills_bump.py` then gave 22 passed.

## Open questions for the owner

1. K: the 200 rev/s fit has no solution, so K is the largest satisfiable fit
   (157.7 rev/s). It lowers the top 0.92 rev at Apex 1.05, where the model
   asks 0.33. Keep it, or take a smaller K and accept a model stop past the
   cushion at high Apex?
2. Apex 1.52 and up drop out of the box (HAND_LIMIT_ACC). Accept?
3. The model: is a 0.426·v velocity error at the step a plausible design case,
   and is the 0.10 rev cushion under the clip right?
4. Torque-feedforward clamp: by proportion, the Apex 1.40 brake asks about
   -0.28 N·m against the 0.234 N·m clamp. Measure with `leg_interp` / the
   probe before the high-Apex cells fly?
