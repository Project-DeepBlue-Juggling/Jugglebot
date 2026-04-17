---
title: MPC fallback-command sawtooth produces visible stutter on off-Active multi-axis moves
type: investigation
date: 2026-04-17
status: in-progress
# --- Context (use what's relevant) ---
phase: "STANDBY-mode multi-axis MPC (post-2.4, pre-4.x combined-axis work)"
related_plan: "hardware-bringup.md"
related_issues:
  - MPC_STALENESS
  - LEG2_TRACKING
sessions:
  - mpc_20260417_184446.csv
  - mpc_20260417_184516.csv
  - mpc_20260417_184532.csv
  - mpc_20260417_184547.csv
  - mpc_20260417_184601.csv
# --- Traceability ---
files_changed: []
commits: []
# --- Classification ---
subsystem:
  - controller
  - mpc
tags:
  - performance
  - safety
  - IPC
---

# MPC fallback-command sawtooth produces visible stutter on off-Active multi-axis moves

## Summary

On off-Active multi-axis MPC moves, the solver's fallback and hold paths (triggered when IPOPT exceeds the 24 ms CPU budget) emit commands that go backwards or freeze relative to the evolving reference. The motor guard faithfully tracks these stale/reversed commands, producing a visible 1–2 s cmd_ext sawtooth and a platform overshoot-recovery cycle — this is the stutter the operator observed. Root mechanism is new (not documented in known_issues.yaml yet); it is triggered on combined X/Y/Z moves where p50 solve time sits already at the 24 ms budget edge.

## Symptoms

- Operator-visible platform stutter during a STANDBY-mode multi-axis move.
- Session 184601 (the stutter session): 74.2% solver timeouts, max 440 consecutive `hold(Maximum_CpuTime_Exceeded)`, p50 25.9 ms > 24 ms budget (p95 31.3 ms, max 65.4 ms), steady-state pos RMS 33 mm, peak 42 mm.
- Two flagged cmd_ext discontinuities at t=1.477 s: leg 2 jumped −10.5 mm and leg 3 jumped −9.4 mm in a single 25 ms sample. 29 additional actual-position jumps (2–8 mm) in the window t=1.27–2.22 s.
- HardwarePlant telemetry-stale E-STOP fired repeatedly (108 times) late in the run (t≈11 s), downstream of the stall — it is a consequence, not the cause.
- Preceding commands 1–3 in the same ROS2 session (pure single-axis Z or Y moves up to 50 mm) were clean: 97.5–100% solver success, < 1 mm final error.
- Command 4 (the combined 174 mm X+Y diagonal in 2.3 s) had only 90% solve success and left the platform 29.6 mm short of target — that off-Active end state became the starting pose for command 5, which is where the stutter appeared.

## Diagnosis

### Stutter mechanism (direct CSV evidence, leg 2 — leg 3 shape is identical)

Steps 15–26 (t=0.75–1.07 s): healthy ramp, all `Solve_Succeeded`, solve time 16–27 ms.
Step 27 (t=1.10 s): first `fallback(Maximum_CpuTime_Exceeded)` — cmd2 drops 151.62 → 149.37 mm (−2.25 mm) on one sample while actual is still climbing. First stutter kick.
Steps 28–36: alternating `Solve_Succeeded` / `fallback`/`Solved_To_Acceptable_Level`, cmd zigzagging forward-and-back ≈ 2–3 mm per sample.
Step 38 (t=1.48 s): `fallback` — cmd 160.58 → 158.74 → 148.27 mm, a **−10.5 mm backwards cmd on a single sample**. Leg 3 same shape (−9.4 mm). This is the largest single-sample stutter and matches the diagnosis flags.
Steps 41–60: `hold(Maximum_CpuTime_Exceeded)` — cmd frozen at 145.32 mm while actual has momentum: act2 coasts 143.7 → 146.2 → 152.3 → 158.8 → 160.0 → 161.9 → **163.3 mm peak** (18 mm past cmd) before settling back to ≈ 145.1 mm. This is the dominant second-long visible stutter.
Steps 60–500: `hold(Maximum_CpuTime_Exceeded)` the whole time, platform parked at ~(60, ?, 184) while ref holds at (33, ?, 197), 37.7 mm unresolved position error.

### Why session 184601 stutters but 184446/184516/184532 don't

Commands 1–3 vary only one axis at a time starting from home. Stewart-platform Jacobian is near-symmetric there, IPOPT converges in a few iterations, solves land well under 24 ms, fallback path almost never fires. Command 5 starts off-Active in all three axes simultaneously (platform at (71.5, −92.4, 170) from unfinished command 4), which is where solve time hits the budget edge consistently — fallback/hold paths fire on nearly every other sample, which is exactly where the sawtooth lives.

### Root cause (new hypothesis)

Both fallback paths in controller/mpc.py emit commands that fail to track the evolving reference:

1. `fallback(Maximum_CpuTime_Exceeded)` salvages a partial IPOPT iterate from the previous warm-start. Against a still-evolving reference that iterate can be behind the current cmd, so the controller outputs a command that moves backward.
2. `hold(Maximum_CpuTime_Exceeded)` pins cmd at the last successful value. The reference keeps moving and the platform has momentum, so actual overshoots past cmd and has to return — a physical overshoot-recovery cycle.

The motor guard is not at fault: it is faithfully interpolating to whatever cmd the MPC emits. The cmd itself is the stutter source.

Secondary contributor: p50 solve time 25.9 ms is already > the 24 ms budget for this operating point. The fallback/hold paths were designed to fire rarely as a safety net. On off-Active multi-axis moves they fire constantly, which is why a mild design compromise becomes a dominant stutter.

### Flagged Issues

- [error] MPC solver saturated: 74.2% timeouts, max 440 consecutive → known issue MPC_STALENESS
- [error] MPC solve budget exceeded: 379 consecutive > 24 ms, max 65.4 ms → same (MPC_STALENESS)
- [error] Command discontinuity leg 2: 10.5 mm at t=1.477 s → unknown pattern (new: MPC_FALLBACK_SAWTOOTH candidate for known_issues.yaml)
- [error] Command discontinuity leg 3: 9.4 mm at t=1.477 s → unknown pattern (same)
- [warning] Leg 2 RMS 2.5× median → known issue LEG2_TRACKING (persistent, mechanical)
- [warning] Steady-state RMS 33 mm → symptom of the above, not independent
- [info] First-sample cold solve 28.4 ms → TIMING_FIRST_SAMPLE, expected

### Known-issue matches

- MPC_STALENESS (active) — direct, via solve-time cascade
- LEG2_TRACKING (persistent, mechanical) — direct
- ZMQ_TELEMETRY_STALE (2026-04-15) — referenced as prior art, but NOT the cause here; telemetry went stale as a downstream consequence of the MPC loop starving its own SUB thread once the sawtooth degenerated into 440 consecutive holds.

### New pattern worth adding to known_issues.yaml

MPC_FALLBACK_SAWTOOTH: cmd_ext jumps backwards > 2 mm between consecutive samples while solver alternates success/fallback/hold, typically on off-Active multi-axis ramps where p50 solve time ≥ budget. Detection signature: count of negative cmd_ext deltas > 2 mm during solver success/fail alternation windows.

## Discussion

<!-- To be filled in during /investigate discussion step. -->

## Fix

<!-- To be filled in during /investigate fix step. -->

## Outcome

<!-- To be filled in during /investigate outcome step. -->
