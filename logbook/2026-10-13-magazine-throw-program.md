---
title: Magazine throw program — JB throws its ball back to Ball Butler's magazine from a formulaic (non-QP) segment, target given in BB's base-marker frame; hand-only throws fly at today's limits, the real magazine needs heave and refuses WORKSPACE
type: feature
date: 2026-10-13
status: open
phase: "two-ball-skill-stack — R5 (Ball Butler feed tuning)"
related_plan: two-ball-skill-stack.md
files_changed:
  - config/bb_magazine.yaml
  - ros_ws/src/jugglebot/jugglebot/motion/skills/magazine_throw.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/magazine_target.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot_interfaces/srv/InstallSegment.srv
  - ros_ws/docs/choreography.md
  - ros_ws/gui/js/juggle-panel.js
  - ros_ws/gui/test_juggle_panel.html
  - tests/ros/test_gui_juggle_defaults.py
  - tests/motion/test_magazine_throw.py
  - tests/motion/test_magazine_target.py
  - tests/ros/test_skill_node_magazine.py
  - tests/ros/test_install_segment.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skills_plan_bench.py
  - tests/ros/test_mocap_node_base_frame.py
  - tests/ros/conftest.py
subsystem:
  - motion
  - ros
  - gui
tags:
  - feature
---

# Magazine throw program

## Problem

The owner wants a magazine → BB → JB → magazine loop, so that BB's feed throw can be tuned over many automatic cycles. This needs JB to throw its ball back into BB's magazine. The owner measured the magazine at **(550, 300, 750) mm in BB's base-marker ("elbow") frame**, i.e. `bb_base_frame.canonical_base_frame`: origin at corner marker C, x from C to E (the long edge), y toward S (+115 mm in the as-built template), z up. Through the base pose from the 2026-10-12 calibration this is world ≈ (−1256, −620, 2413) mm: 1.40 m sideways from the cup and 1.55 m above its release, beyond BB.

## Discussion

- **The QP planner cannot reach the magazine.** An offline probe (scratchpad copies of `tools/probes/skills_single_site_sweep.py`'s harness, 2026-10-13) gave:
  - vertical throws: accepted up to apex 1.0 m (4.40 m/s release), `HAND_LIMIT_ACC` from 1.1 m, QP-infeasible at 1.6 m;
  - the magazine: QP-infeasible at every flight time from 0.6 to 1.1 s, with legs at 350/5000/200k and at 1000/30k/1M.
  The magazine needs ≥ 6.12 m/s (0.8 s flight, 16.6° off vertical).
- **A formulaic hand profile alone cannot reach it either.** The owner proposed bypassing the planner. The repo's numbers rule out a hand-only 6.3 m/s:
  - nothing has flown above 4.436 m/s since the torque-clamp fix (`ros_ws/docs/hand_throw_envelope.md:519`);
  - braking authority is 4178 rev/s² and the decel FF headroom 3925 rev/s²;
  - accel up plus braking needs ~10.3 rev of stroke at 6.3 m/s, against ~10.2 rev usable.
  On the 25 ms knot grid the hand also needs a full span to switch from +A to −D (`HAND_LIMIT_C2`), which caps the hand-relative release at **5.23 m/s**. The deleted Teensy `calcThrow` switched instantly.
- **Owner decision: platform heave assist.** The legs supply the speed the hand cannot. The Phase-A design point (0.80 s flight, 12° tilt, ~1.0 m/s platform, hand 5.22 m/s) passes `validate_cycle` only with:
  - leg acceleration 6369 mm/s², over the 5000 ceiling;
  - a per-knot leg step of 0.317 rev, over the 0.24 safety bound;
  - leg extension within 4 mm of the hard top, and every hand bound within 0–5 %.
  The owner chose **option 1**: build the program at today's limits, fly reachable targets first, and climb toward the magazine. No limit, ceiling or safety constant was raised. The owner says the landing tolerance is generous.
- **The aim uses rigid-body kinematics, not the repo cup map.** `uc.cup_state_from_platform` inverts `cup_realize.decompose` with the tilt centre fixed at world z 748.2 mm. That is the IK pose origin at active z (`shaping.py:86` = `GEOM_INITIAL_HEIGHT_MM + JB_OP_DEFAULT_ACTIVE_Z_MM`; `ik_solver.compute_leg_vectors` rotates about `pos + init_height_vec`). So on the real rigid platform the centre rides with the heave. At 12° tilt the fixed-centre map reads ~0.2 m/s of lateral cup velocity per 1 m/s of heave that the machine does not produce: (−167, −82, −20) mm/s at the magazine design point, ≈ 168 mm at the target. Hand-only throws agree to 1.8 mm. The pose and hand knots are what stream, so the machine does the rigid thing either way. The plan's release meta carries the rigid values, and the repo-map gap is kept as a diagnostic (`repo_minus_rigid_*`). `validate_cycle` is untouched.
- **The target lives in `config/bb_magazine.yaml`, not `hardware_config.yaml`.** The admissible-box gate hashes the generated `hardware_config.py` text (`admissible.py:242`). A coordinate edit there would freeze every juggle pattern until a re-sweep. No gated file was edited: `gate_hash()` reads `5072c89d4ddc` before and after. The new kind therefore lives in `magazine_throw.py` and not in `segments.py`, which is gated.
- **No SkillExecutor.** The program is one rest-terminal segment with no catch, learner row or sequencing. skill_node calls the same `_installer` the executor would, and a timer collects the miss report. `_refuse_if_running` counts `_magazine_ctx` as running. `executor.install_segment` refuses `MAGAZINE_NEEDS_REST` for any splice onto a MAGAZINE record, because its meta carries no `cup_plan`.

## Change

- `magazine_throw.py` (pure):
  - `MAGAZINE`, `MagazineTerminal`, `design_magazine`, and `plan_magazine` → `segments.Segment` with a real `CycleMeta` (`releases[-1]` = rigid site, velocity and time).
  - The build goes rest → pre-phase (tilt, dip, hand to the stroke bottom) → piecewise-linear-acceleration throw (exactly C2 on 25 ms knots) → level rest with an empty cup.
  - Least aggressive: the hand takes the speed along the axis up to its cap, and heave covers only the remainder (0 for a hand-only throw).
  - The tilt is capped at 12°; beyond that the platform leans out.
  - Refusals: `MAGAZINE_NOT_AT_REST`, `MAGAZINE_UNREACHABLE`, `MAGAZINE_TILT`, `MAGAZINE_WINDOW` (release earlier than ~1.11 s after install), `MAGAZINE_NEEDS_REST`, and every `validate_cycle` code at the LIVE Sitting limits.
- `magazine_target.py` (pure): the yaml loader (strict), the base ↔ world ↔ plan transforms, `quat_to_rotation`, and `fit_descending_crossing` (z quadratic, xy linear, ≥ 6 samples).
- `mocap_node`: a latched `bb/base_frame_pose` (`PoseStamped`: C in world **mm**, quaternion of R_base→world, stamp = newest frame's ROS time), published at 1 Hz from the existing `BaseFrameMonitor`.
- `InstallSegment.srv`: `KIND_MAGAZINE=3`, which reuses `site_mm`, `target_mm`, `flight_s`, `t_event_s` and `rest_site_mm`. trajectory_node decodes it to a `MagazineTerminal`, and skill_node's `_installer` encodes it.
- skill_node, `magazine` pattern, dispatched after the common refusal ladder:
  - `MAGAZINE_BASE_FRAME_STALE` (age > `magazine_base_frame_max_age_s`, 60 s), then `MAGAZINE_CONFIG`, then the launch preconditions (ball seated, levelled, mocap and hand fresh), then the frame check;
  - then prelevel and one install with the release `magazine_release_lead_s` (1.3 s) ahead;
  - after the throw, a `MAGAZINE miss: dx dy (elbow frame)` line plus a `MAGAZINE_MISS` attempt event (`NO_TRACK` when the fit fails);
  - result `THROWN`, success, 1 throw, 0 caught.
- GUI: a hold-to-confirm "Throw to magazine" button in the juggle panel (request `magazine`), showing the latest miss.

## How to fly it (first sitting)

1. **Deploy.** `colcon build --packages-select jugglebot_interfaces jugglebot` was already run from this worktree. **Both packages must be rebuilt together after pulling**, or the launch fails on an InstallSegment type mismatch. Restart `jugglebot-gui.service` for the button.
2. **First rung: a reachable hand-only target.** Edit `config/bb_magazine.yaml`. For example, aim near JB rather than BB: any elbow-frame point whose plan-frame target needs ≤ ~5 m/s and ≤ 12° tilt. The accept log prints the target in base, world and plan frames.
3. ACTIVE, ball seated, then `ros2 service call /jugglebot/juggle_request jugglebot_interfaces/srv/SetString "{data: 'magazine'}"` (or the button).
4. Read the `MAGAZINE miss` line, nudge the yaml, re-request. The file is re-read on every request.
5. **Toward the magazine:** heave rungs refuse `WORKSPACE` today (see Outcome). They need the generator's pre-phase and release-z geometry tuned before any limit question.

## Verification

- Scoped (`python -m pytest -q -n 4 --dist loadfile tests/ros/test_skill_node*.py tests/motion/test_magazine_target.py tests/motion/test_magazine_throw.py tests/ros/test_install_segment.py tests/motion/test_skills_executor.py tests/ros/test_skills_plan_bench.py tests/ros/test_mocap_node_base_frame.py`, run 2026-10-13): **618 passed**.
- `colcon build --packages-select jugglebot_interfaces jugglebot` (2026-10-13): both packages OK.
- Full gate: see the commit message.
- Not verified on hardware: the whole program. In particular:
  - the hand at a 5 m/s, 3900 rev/s² braking profile from this generator (above the measured coast ladder);
  - the tracker following an unannounced ball toward BB;
  - the base-frame pose arriving live.

## Outcome

**Superseded the same day** by `2026-10-13-magazine-throw-hand-only-and-reload-button.md`: the owner withdrew the platform heave ("too risky, asking too much of the legs, for little benefit"), so the generator is hand-only from that entry on and the heave rows below are history — the stroke-geometry "next unit" never starts.

Gate verdicts by design at 350/5000/200k (heave rows identical at 1000/6500/140k):

| design | gate |
|---|---|
| hand-only (300, 100, 1100) plan mm, 0.75 s | OK |
| heave (200, 0, 2400), 0.9 s | `WORKSPACE`, leg 0 after release |
| the magazine, 0.9 s | `WORKSPACE`, leg 5 at the pre-phase dip |

Both heave refusals are stroke geometry (dip depth, release z, post-release rise), not limits. That is the next unit.
