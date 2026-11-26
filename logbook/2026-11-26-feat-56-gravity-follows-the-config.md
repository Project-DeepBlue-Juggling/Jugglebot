---
title: "FEAT-56: the sim skills gate and human_catch take the generated gravity (9806) instead of a typed 9810"
type: refactor
date: 2026-11-26
status: resolved
phase: ""
related_plan: ""
related_issues: []
files_changed:
  - sim/skills_gate.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/human_catch.py
  - tests/firmware/test_hand_copied_constants.py
  - tests/sim/test_skills_gate.py
  - logbook/2026-11-26-feat-56-gravity-follows-the-config.md
subsystem: [sim, skills, tooling]
---

## Summary

Owner decision 2026-11-26: the two typed gravities of 9810 mm/s^2 follow the config's 9806 (`hw.GRAVITY_MMPS2`, generated).

- `sim/skills_gate.py`: `_GRAVITY_MMS2 = float(hw.GRAVITY_MMPS2)` (`hw` was already imported there). Its only reader is `_plant_landing_bias_release`, the FEAT-7 plant-bias hook: it adds lateral speed `bias * g / (2 vz)` to a release, so only attempts run with a nonzero `plant_landing_bias_mm` move (by 0.04 % of that lateral speed, about 0.01 mm/s of 33 mm/s). Unbiased gate runs are unaffected.
- `human_catch.py`: `nominal_arrival_vel_mm_s` is the vertical arrival speed `sqrt(2 g rise)` for the Human-catch nomination (`nominate.py` plans the Catch with it; rise = nominal apex minus the catch plane). It now reads `hw.GRAVITY_MMPS2` through `import jugglebot.hardware_config as hw`, the same import `segments.py`, `executor.py`, `nominate.py`, `bump.py` and `magazine_throw.py` already use in this package. At the 1.9 m default apex the speed moves 0.02 %, about 0.9 mm/s of about 4580 (the policy test's tolerance is 10 mm/s). The planned arrival is a hair slower and matches the Ball's true ballistic arrival under 9806 slightly better. No timing, feedforward or setpoint path changes shape; one planning input moves by 0.02 %.
- The `test_hand_copied_constants.py` "as found" pins for both became equality with the config (source must read `hw.GRAVITY_MMPS2`, evaluated value equals the config, no 9806/9810 literal in human_catch). `test_plant_landing_bias_release_...` in `tests/sim/test_skills_gate.py` had 9810.0 typed in its expectation; it now uses `sg._GRAVITY_MMS2` (its 33.1 +/- 0.1 absolute check still holds).
- Probes under `tools/probes/` that type 9810.0 or 9806.65 are untouched (none import either changed symbol; still pinned as found by `test_probe_gravity_literals_are_pinned_as_they_are_today`).

## Verification

- 2026-11-26 before the change, `pytest -q` on the biased online gate test, the bias-release unit test, `test_hand_copied_constants.py`, `test_skills_human_catch_policy.py`: 99 passed in 52.38 s.
- 2026-11-26 mutation: restoring `_GRAVITY_MMS2 = 9810.0` fails `test_sim_skills_gate_gravity_follows_the_config`; restoring `9810.0 * rise` fails `test_human_catch_gravity_follows_the_config` (both restored after).
- 2026-11-26 after the change: scoped set, result in the commit message / orchestrator report.
