---
title: Velocity feedforward semantic mismatch causing violent oscillation
type: investigation
date: 2026-03-30
status: resolved
phase: "3.1"
related_plan: hardware-bringup.md
related_issues:
  - VEL_FF_BUG
sessions:
  - mpc_20260330_171932.csv
files_changed:
  - controller/mpc.py
  - ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py
  - tests/motion/test_motor_guard.py
commits:
  - a618751
subsystem:
  - mpc
  - motion
tags:
  - safety
  - dynamics
---

# Velocity feedforward semantic mismatch causing violent oscillation

## Summary

During the first Phase 3.1 motion test (5mm z-axis move), the platform entered
violent oscillation (z: 147-194mm) for ~2.1s before the motor guard E-stopped.
Root cause was `cmd_vel` being computed as `(cmd - q_cur) / dt` instead of
`(cmd - prev_cmd) / dt`, which doubled the intended position step on every cycle,
creating a positive-feedback oscillation loop.

## Symptoms

- Commanded a gentle 5mm z-axis rise: `--pose 0,0,175,0,0,0` from Active at z=170mm
- Platform immediately entered violent oscillation (z range: 147-194mm)
- Oscillation amplitude grew from +/-8mm to +/-30mm over ~2.1s
- Tracking error peaked at 29.8mm
- Motor guard E-stop triggered: leg 1 MAX_DEVIATION = 0.557 rev (limit: 0.5 rev)

## Diagnosis

**Verdict: FAIL**

Key flags:
- **[error]** Oscillation detected with growing amplitude
- **[error]** MAX_DEVIATION E-stop triggered on leg 1
- **[warning]** Tracking error 29.8mm (limit: typically <3mm for Phase 3)

The oscillation was immediate and violent — not a gradual drift. This pointed to a
sign/scaling error in the control loop rather than tuning or stability margin issues.

### Flagged Issues

- **VEL_FF_BUG** (critical) — velocity feedforward semantic mismatch

## Discussion

The motor guard computed velocity feedforward as:
```python
cmd_vel = (cmd - q_cur) / dt0
```

This represents the average velocity to travel from current position to the command
in one timestep. However, the motor guard then extrapolated:
```python
pos(t) = cmd + cmd_vel * dt
```

Substituting: `pos(dt0) = cmd + (cmd - q_cur)/dt0 * dt0 = 2*cmd - q_cur`

This **doubled the intended position step** every cycle. With the 500 Hz guard
running ahead of the 40 Hz MPC, this created a positive-feedback loop where each
guard cycle amplified the overshoot.

**Rejected alternatives:**
- PID gain tuning — wrong because the oscillation was a control architecture bug,
  not a tuning issue
- Reducing MPC rate — would slow the loop but not fix the fundamental sign error

## Fix

Two changes:

1. **`controller/mpc.py`** — Changed velocity formula to use command-sequence derivative:
   `cmd_vel = (cmd - prev_cmd) / dt` instead of `(cmd - q_cur) / dt`. First solve
   uses zero velocity (safe startup).

2. **`ros_ws/.../motion/motor_guard.py`** — Added MAX_LEAD_REV tracking clamp (0.15 rev
   = ~10.6mm). Commanded position is clamped to within MAX_LEAD_REV of motor feedback,
   preventing runaway even if velocity computation has errors.

**Commit:** `a618751` `Fix MPC velocity feedforward causing hardware oscillation`
**Branch:** `refactor`

## Outcome

After the fix, Phase 3.1 re-test passed cleanly:
- Platform rose smoothly from z=170mm to z=175mm
- Tracking error <0.5mm steady-state
- No oscillation detected
- Motor guard tracking clamp provides defense-in-depth against future velocity bugs

Full root cause analysis preserved in `plans/archived/2026-03-30 mpc-oscillation-analysis.md`.

## Open Questions

None — issue fully resolved. The tracking clamp remains as a permanent safety layer.
