---
title: "R2 hardware-gate runsheet — a no-motion timing gate on the loaded Jetson; install_segment's missing A5 marker; two skill-path preconditions R3 must port"
type: feature
date: 2026-09-13
status: in-progress
phase: "two-ball-skill-stack — R2 (hardware gate)"
related_plan: two-ball-skill-stack.md
---

# R2 hardware-gate runsheet

**What.** `tests/hardware/session_skills_r2_plan_gate.md` and its driver
`tests/hardware/skills_plan_bench.py` (pure core in
`tests/ros/test_skills_plan_bench.py`) close the one R2 gate left open: per-skill
`install_segment` plan time in the launched `trajectory_node` under sitting load.
The driver plays the real columns schedule with synthetic landings; arm A is the
schedule, arm B adds ±3 mm landing jitter to force catch re-sends, and a
non-gating row C adds two busy cores. Five pre-registered gates: G1 plan < 50 ms
(the plan's), G2 zero `SPLICE_TOO_LATE` and round trip under the 125 / 75 ms
splice budgets, G3 emitter gap < 40 ms, G4 completion, G5 no motion. The
decision for "G1 misses, G2 holds" is written down before the sitting.

**Why no motion.** The planner solves from the commanded state, so a disarmed
wire (`auto_arm:=false`, ACTIVATE, never arm) runs the same solves as an armed
one, and motion would add the never-flown 200k jerk ceiling — R3's ramp — to a
timing test. The driver refuses unless `mpc_active = 0` and holds + stops if it
ever reads armed.

**Found writing it.**
- *A5 gap (fixed).* `trajectory_node._accept_segment` did not append
  `_wire_state_suffix()`, so an install accepted on a disarmed wire was silent —
  the one accept path added at R2 that broke ARMING_CONTRACT A5. Fixed, pinned
  by two tests in `tests/ros/test_install_segment.py`; the gate uses the marker
  as its per-install no-motion proof.
- *Two FSM preconditions the skill path lacks (carried to R3, plan § 4 R3).* An
  offline real-time rehearsal from the ACTIVATE park (hand 0 rev, cup 679.6 mm,
  10 mm under the 689.6 mm floor) at 300/5000/200k refused the first THROW
  `HAND_STROKE`; with one REST pre-position first, every install of a
  20-throw ±3 mm attempt was accepted, zero `SPLICE_TOO_LATE`, solves p50 10.7 /
  p95 34.3 / max 49.3 ms over 62 installs (scratchpad probe, idle Jetson,
  2026-09-13). The FSM's `_unified_floor_lift` and `_unified_prelevel` have no
  skill-path twin, so `skills/start_columns` from a fresh activation would refuse
  its first throw. The gate pre-positions and skips `level`.
- *The idle max is already at 49.3 ms*, which is why G1's miss case is decided
  in advance rather than at the robot.

## Verification

- `pytest tests/ros/test_install_segment.py -q` (2026-09-13): 14 passed.
- `pytest tests/ros/test_skills_plan_bench.py tests/ros/test_install_segment.py -q`
  (2026-09-13): 53 passed.
- `python3 tests/hardware/skills_plan_bench.py --rehearse --arm B --attempts 1`
  (2026-09-13, idle Jetson, 20 throws, ±3 mm): exit 0; G1 PASS worst 48.31 ms
  over 57 solves (CATCH+throw p50 29.9 / p95 43.9 / max 48.3; re-sends p50 10.5
  / max 27.0, 34 refused); G2 PASS, 0 `SPLICE_TOO_LATE`, handoff max 48.3 of
  125 ms, unpinned max 27.0 of 75 ms; G4 PASS; G3/G5 SKIP offline. The driver
  times a refused rehearsal solve by its own wall clock, because
  `install_segment` reports 0.0 for a refusal while the node's
  `_reject_segment` times it from callback entry.
- `./run_tests.sh` (2026-09-13, default gate: the Python change is `trajectory_node.py` plus tests, nothing under `controller/` or `sim/`): **6503 passed, 9 skipped** in 265.39 s (parallel) + **3 passed** in 8.03 s (serial); `RESULT: PASS`.
