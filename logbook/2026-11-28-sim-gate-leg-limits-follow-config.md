---
title: "The sim skills gate's Sitting Leg limits now come from the generated launch working point (600 / 8000 / 200000) instead of the typed 350 / 5000 / 200000"
type: refactor
date: 2026-11-28
status: complete
phase: "offline work 2026-11-28"
related_plan: ""
related_issues: []
files_changed:
  - sim/skills_gate.py
  - tests/firmware/test_hand_copied_constants.py
  - logbook/2026-11-28-sim-gate-leg-limits-follow-config.md
subsystem: [sim, skills]
tags:
  - sim-gate
---

# Sim gate Leg limits follow the config

**What.** `sim/skills_gate.py` `_SESSION_LEG_VEL_MMPS`, `_SESSION_LEG_ACC_MMPS2`, `_SESSION_LEG_JERK_MMPS3` were typed 350 / 5000 / 200000, the R5 Sitting limits. The robot has launched at 600 / 8000 / 200000 since `db435e54` (2026-11-03) and `nominate.default_limits()` already used those. They now read `hw.JB_TRAJ_LEG_VEL_LIMIT_MMPS`, `_ACC_LIMIT_MMPS2`, `_JERK_LIMIT_MMPS3`, as `_SESSION_HAND_ACC_RPS2` does. Owner decision 2026-11-28.

**Why.** Plain Columns at 74 Throws ended LIMIT_VEL on 4 of 5 seeds at 350 and passed at the live limits (`out/jp11-limvel`): the sim judged a stricter plant than the robot flies. No config value, tolerance or robot-side code changed; only which limits the sim gate plans against.

**Control-cycle walk.** None of the planner, setpoint stream or timing code changed. The gate hands `Limits(...)` to the same planner; a looser leg bound only widens the admissible box, the knot grid (40 Hz, 25 ms) is untouched.

**Test.** `test_sim_skills_gate_leg_limits_follow_the_config` (tests/firmware/test_hand_copied_constants.py) reads the source: it fails with the typed triple restored (run 2026-11-28, `git stash -- sim/skills_gate.py`: 1 failed, 20 passed) and passes with the change (21 passed).

**Finding (one test flips, NOT adjusted).** `tests/sim/test_skills_gate_online.py::test_an_unbiased_self_toss_run_passes_every_check_with_no_stepped_command` (`self_toss` seed 0, policy A, 5 Throws) passed at 350 and now ends one attempt `LIMIT_JERK`: skill 3 (REST) `REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK: peak leg jerk 211669 mm/s^3 > 200000, leg 1 at t=0.075 s)`, end_codes `['LIMIT_JERK', '', '']`. The jerk limit did not move; the looser velocity/acceleration lets that REST move plan 5.8 % over it, and the executor's own check refuses it (safe-failing). The assertion is left as is for the owner to adjudicate.

**Verification (2026-11-28).**
- `python -m pytest tests/firmware/test_hand_copied_constants.py -q`: 21 passed.
- Readers: `pytest tests/sim/test_skills_gate.py tests/sim/test_skills_gate_online.py tests/sim/test_skills_gate_beats.py tests/sim/test_skills_gate_nominate.py tests/motion/test_skills_schedule.py tests/motion/test_skills_executor.py -q -n 2 --dist loadfile`: 612 passed, 1 failed (the test above, passes at 350 in a baseline run).
- `python sim/skills_gate.py --seeds 0 1 2`: PASS. Columns `--separation-mm 125 --apex-m 0.95 --n-throws 74 --seeds 0 1 2 3 4`: PASS 5/5 (74/74 makes). Rotating `--step-deg 10 --separation-mm 125 --apex-m 0.95 --n-throws 38 --seeds 0 1 2 3 4`: PASS 5/5 (38/38).

## Integration note (2026-11-28)

At the live limits one reader flips: `tests/sim/test_skills_gate_online.py::test_an_unbiased_self_toss_run_passes_every_check_with_no_stepped_command` ends Attempt 1 on a REST refused LIMIT_JERK (211,669 mm/s^3 against 200,000). The diagnosis is BUG-159: the Banking tilt schedule guards jerk on Knot differences while the plan gate reads the interpolated curve; the robot has had the same exposure since 2026-11-03. The owner chose to merge this change now with that test marked a strict expected failure citing BUG-159, and to fix BUG-159 in the planner.
