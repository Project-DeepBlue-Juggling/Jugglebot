---
title: Phase 3.1 Oscillation — Root Cause Analysis
created: 2026-03-30
status: completed
completed: 2026-03-30
archived: 2026-03-30
---

# Phase 3.1 Oscillation — Root Cause Analysis

**Date:** 2026-03-30
**Test:** `python3 main.py --hardware --mpc --pose 0,0,175,0,0,0 --duration 30`
**Expected:** Platform rises 5mm smoothly from Active (z=170mm) to z=175mm
**Actual:** Violent oscillation (z: 147-194mm) for ~2.1s, then motor guard E-stop

---

## Context

This was the first commanded motion during MPC hardware bringup. Phase 2.4 (hold at Active position) passed perfectly — zero movement, tracking error <0.04mm. The hold test did not exercise the velocity feedforward path because zero motion means zero velocity.

The MPC pipeline was developed in simulation where `MuJoCoPlant` applies extension commands directly to MuJoCo actuators and ignores the `cmd_vel` return value. The motor guard's velocity-based interpolation only exists in the hardware path.

### Command pipeline (hardware mode)
```
MPC (50 Hz) → HardwarePlant (:5557) → MotorGuard (500 Hz) → MotionBridge → CAN → ODrives (8 kHz)
```

---

## Timeline

Source: `sim/logs/mpc_20260330_171932.csv` (500 rows)

### Phase A — Stale Feedback (t=0.34-0.45s)

| Row | time  | cmd_ext_0 | actual_ext_0 | pose_z | vel_0 |
|-----|-------|-----------|--------------|--------|-------|
| 1   | 0.341 | 157.97    | 154.50       | 170.00 | 0.0   |
| 2   | 0.395 | 159.76    | 154.49       | 170.00 | 0.0   |
| 3   | 0.428 | 160.47    | 154.49       | 170.00 | 0.0   |
| 4   | 0.452 | 160.72    | 154.49       | 170.00 | 0.0   |

MPC solves against frozen telemetry for 110ms (startup pipeline latency). Each solve commands increasingly extended positions while seeing the platform as stationary. The motor guard interpolates these commands at 500 Hz with aggressive velocity feedforward.

### Phase B — Massive Overshoot (t=0.48-0.59s)

| Row | time  | cmd_ext_0 | actual_ext_0 | pose_z | vel_0   |
|-----|-------|-----------|--------------|--------|---------|
| 5   | 0.477 | 160.14    | 157.98       | 173.93 | +196.9  |
| 6   | 0.515 | 157.75    | 166.18       | 182.67 | +150.6  |
| 7   | 0.554 | 155.76    | 170.64       | 187.37 | +179.7  |
| 8   | 0.591 | 155.63    | 169.01       | 185.84 | -201.2  |

When feedback arrives, the platform is already at z=174mm (nearly at target) with 197 mm/s of velocity. It overshoots to z=187mm — 12mm past target. The MPC reverses commands.

### Phase C — Growing Oscillation (t=0.59-2.4s)

Oscillation at ~2.5-3 Hz, amplitude growing from +/-8mm to +/-30mm. Tracking error peaks at 29.8mm. Lateral drift develops (x=-6mm, y=-9mm) from asymmetric CAN rejections.

### Phase D — E-Stop (t=2.45s)

Motor guard fires MAX_DEVIATION E-stop (leg 1 = 0.557 rev, limit 0.5). Platform freezes at z=147mm, tilted and displaced.

---

## Root Cause: Velocity Feedforward Semantic Mismatch

### The bug

`controller/mpc.py` line 777 computed:
```python
cmd_vel = (cmd - q_cur) / dt0    # state-to-command velocity
```

This is the average velocity to travel FROM current state TO command in one timestep. But the motor guard interprets it as velocity AT the command point and extrapolates forward:
```python
pos(t) = cmd + cmd_vel * dt + ...    # motor_guard.py:653-657
```

### The math

At `dt = dt0 = 20ms` (one MPC period), the motor guard extrapolates:
```
pos(20ms) = cmd + (cmd - q_cur)/dt0 * dt0
          = cmd + (cmd - q_cur)
          = 2*cmd - q_cur
```

**This doubles the intended step.** For the first command (`q_cur=154.5`, `cmd=158.0`):
- Motor guard extrapolates to: 2*158.0 - 154.5 = **161.5mm**
- MPC's internal prediction (tau model, alpha=0.487): **156.2mm**
- Overshoot: **5.3mm** past the MPC's expected actuator state on step 1 alone

### The positive-feedback loop

1. Motor guard extrapolates past the command; ODrive vel_ff (2.48 rps vs 0.56 rps from position error) physically drives motors there
2. MPC sees overshoot → reverses command direction
3. Motor guard extrapolates past the reversed command (same doubling)
4. Oscillation amplitude grows each half-cycle

### Why the hold test passed

During hold, cmd = actual (zero motion). `cmd_vel = (cmd - q_cur) / dt = 0`. No extrapolation, no overshoot, no oscillation.

### Why simulation was unaffected

`MuJoCoPlant` ignores `cmd_vel` — it passes extensions directly to MuJoCo actuators. The velocity feedforward path was never exercised in simulation.

### Contributing factors

- **80-140ms startup feedback latency**: MPC ran 4 blind solves, accumulating velocity against stale feedback. With correct velocity semantics this would produce ~2-3mm of manageable overshoot; with the wrong velocity it produced ~8-12mm of catastrophic overshoot.
- **CAN step rejection cascade**: Once oscillation exceeded 21mm (~0.3 rev), CAN node rejected all 6 legs. Motor guard has no awareness of rejections (no NACK path). This amplified late-stage oscillation but did not cause onset.

---

## Fix Applied

### Fix 1: Corrected velocity formula (`controller/mpc.py`)

Changed `cmd_vel` to use the rate of change of the command sequence:
```python
# Before (wrong — state-to-command velocity):
cmd_vel = (cmd - q_cur) / dt0

# After (correct — command trajectory derivative):
if self._prev_u is not None:
    cmd_vel = (cmd - self._prev_u) / dt0
else:
    cmd_vel = np.zeros(6)  # first solve: safe zero-vel startup
```

Applied in both the success path (line 779) and failure fallback path (line 954).

**Expected behavior:**
- Step 1: `cmd_vel = 0`. Motor guard holds at cmd, ODrive servos smoothly via PID.
- Step 2+: `cmd_vel = (cmd - prev_cmd) / dt ≈ 89 mm/s`. Extrapolation overshoot ~1.8mm, converging.
- vel_ff to ODrive: 1.27 rps (vs 2.48 rps before) — much less aggressive.

### Fix 2: Motor guard tracking clamp (`motor_guard.py`)

Added a position tracking clamp after interpolation (line 701):
```python
MAX_LEAD_REV = 0.15  # ~10.6mm — half the CAN step limit
deviation = commanded_pos - motor_fb_pos
clip(deviation, -MAX_LEAD, MAX_LEAD)
commanded_pos = motor_fb_pos + deviation
```

Defense-in-depth: regardless of any future feedforward bugs, the motor guard never sends positions more than 10.6mm ahead of actual encoder position. This eliminates CAN rejections (0.15 < 0.3 rev limit) and bounds any oscillation amplitude.

Safe for ball-catching: 700 mm/s / 500 Hz = 1.4 mm/step = 0.02 rev/step, well within the 0.15 rev margin.

---

## Verification

- **345 sim tests pass** (`pytest tests/sim/ -v`)
- **61 motion tests pass** (`pytest tests/motion/ -v`)
- Hardware Phase 3.1 re-test pending deployment to Jetson
