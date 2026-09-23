---
title: "The FSM toss/reload choreography deleted under fsm-final — 13 k lines of coordinator, the ring half of trajectory_node and unified_cycle, PlanCycle, the old sim gates, 14 FSM-only config keys; every surviving claim ported to the skill path"
type: refactor
date: 2026-09-24
status: done
phase: "two-ball-skill-stack — R4"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/toss_session.py
  - ros_ws/src/jugglebot/jugglebot/reload_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/catch_reach.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot_interfaces/srv/PlanCycle.srv
  - ros_ws/src/jugglebot_interfaces/action/Toss.action
  - ros_ws/src/jugglebot_interfaces/action/TossContinuous.action
  - ros_ws/src/jugglebot_interfaces/action/Reload.action
  - sim/toss_gate.py
  - sim/reload_gate.py
  - sim/juggle_online.py
  - sim/viz/recording.py
  - config/hardware_config.yaml
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/docs/choreography.md
  - tests/ros/test_install_segment.py
  - tests/ros/test_levelling_frame.py
  - logbook/INDEX.md
subsystem:
  - ros
  - motion
  - sim
tags:
  - skill-stack
  - deletion
---

# The FSM stack goes under `fsm-final` (2026-09-24)

## What / why

Plan `two-ball-skill-stack.md` § 4 R4 "Delete", owner decision D5 (2026-09-23): the FSM is
deleted BEFORE the first R4 sitting, so the sitting rehearses the tree that ships. The FSM had
been refused at goal-accept since R1 (its hand prime rode the deleted stroke engine); nothing
powered used it. Tag **`fsm-final` = 1e7f2d9**, the last commit with every FSM path wired
including the GUI reload relay, pushed; recovery is `git checkout fsm-final`, the way the MPC
chain went under `mpc-final`.

A read-only census (`census_fsm_deletion.md`, session scratchpad) traced IMPORTERS, not names,
before anything was removed — the R0 census had "missed live importers", and this one found
the same class again: `sim/juggle_online.py`, which R0 had recorded as importer-free, is
imported by three sim demos for its video recorder and camera helpers. Those moved to
`sim/viz/recording.py` (no behaviour change) before the file went.

Three clusters, three commits: **A** — `toss_sequencer`, `toss_session`, `reload_sequencer`,
`catch_coordinator` (+node), `catch_reach`, `reload_coordinator_node` (10 555 lines; its every
publisher/subscriber/service/action inventoried in the handoff — dead with the FSM, or already
served by `skill_node`), the three actions, 14 FSM test files, the `catch/*` topics, the launch
and setup entries, the conftest mocks. **B** — `trajectory_node`'s ring half (`_svc_plan_cycle`
and its three modes, `_replan_cycle_from_target`, `arm_catch`, `dynamic_target`, `reach_center`,
the supersede deadline; 18 methods), `PlanCycle.srv`, `unified_cycle.replan_tail` /
`latest_supersede_time_s` / `REPLANNED` (the ring PRIMITIVES `extend`, `_concat_plans`,
`_gate_joined`, `_seam_check`, `splice_at`, `plan_cycle` stay — `splice_at` is built on them),
the old sim gates `sim/toss_gate.py` / `sim/reload_gate.py`, the hardware benches and probes
that drove the ring. **C** — `unified_cycle_enabled` and 13 `toss_*` keys out of
`hardware_config.yaml` (each generated constant grepped for a live reader first; none),
`INVARIANTS.md` closed on every RETIRE@R4 / PORT@R4 row (C-REACH-1 / I-CATCH-1..3 retired: the
reach envelope's job is the offline admissible box plus the lateral authority clamp; I-POSS-1
ported to `skill_node._on_hand_telemetry`; a new § 9 records `planner.build_catch` as retained
only as the C-LEVEL-1 levelling-frame contract's test vehicle, an R6 review item), the
choreography map regenerated from its generator, two contract docs given dated retirement
headers naming what survives.

## Ported, not deleted

* The T-I1 wire-seam test (a real `TrajectoryNode` + real `TeensyBridgeNode` + `SetpointPump` +
  loopback UDP, `HAS_HAND`/`HAS_V1` up while a seven-channel plan streams and clear on the falling
  edge) — re-driven through `trajectory/install_segment` in `test_install_segment.py` BEFORE the
  old integration file went; the hold-preempts-solve node-wiring tests likewise.
* `test_levelling_frame.py` (C-LEVEL-1): 8 behavioural tests now call `planner.build_catch`
  directly (the node pipeline that fed it is gone); `test_unified_cycle_levelling.py` and
  `test_trajectory_tilt_map.py` re-driven through `_svc_install_segment`.
* `tests/motion/test_unified_cycle.py`: 23 `replan_tail` tests deleted, 2 ported onto `splice_at`,
  2 left `skip`ped with measured reasons — their numeric claims were about `replan_tail`'s
  seam-scoped gate range, and `splice_at`'s `report_range_knots` stays `None` after every splice
  (`extend`'s "None unless prior" logic) — a planner-owner call whether that is a deliberate
  simplification or an observability gap.
* Possession side: `test_possession_replay.py` keeps its six bag-replay verdict tests (C-POSSESS-1),
  loses ~230 lines of `TossSessionSequencer` cadence tests; `_CAUGHT_MAX_XY_ERROR_MM` had no live
  reader and lives on only inside the replay probe that self-checks against it;
  `compute_catch_point_mm` moved to `toss_release.py`; an AST importer trace of all 69 top-level
  definitions in `ball_possession.py` found zero dead code.
* `tools/probes/possession_replay.py`, `displaced_reach_frontier.py`, `catch_reach_replay.py` and
  `possession_verdict_bag_check.py` are logbook-cited replay harnesses: each got the one constant
  or body it imported from a dead module, with provenance, rather than being deleted.

## Verification

* Grep-to-zero for every deleted module and symbol outside `logbook/`, `plans/` and dated
  retirement notes in KEEP files (each cluster's handoff lists the commands and counts).
* Box determinism pair (plan § 2.6 "regenerated whenever limits change", `tools/admissible_sweep.py`'s
  own "run twice and diff"): run 1 2026-09-23 `python tools/admissible_sweep.py --site-pairs all
  --single-apex 0.5 0.6 0.7 0.8 0.9 --out config/generated/admissible_box.yaml` → 2159.9 s,
  `gate_hash 7966cb6fadc9`; run 2 2026-09-24, same command after this deletion (unified_cycle.py is
  hashed) → 2027.9 s, `gate_hash c2736ce2e2f7`; **all nine boxes IDENTICAL and all 16 471 row verdicts
  identical** (md5 of the row logs with `wall_s` stripped equal) — the deletion moved no admitted set.
* (the gate triple is filled at commit time)
