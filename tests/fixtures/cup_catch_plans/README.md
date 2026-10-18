# cup_catch_plans - realised-plan fixtures for the catch / seating checks

Consumed by `tests/motion/test_validate_cycle.py` (CATCH_CLOSING_SPEED,
CATCH_SKID, CUP_SEATING_CONE, CUP_SEATING_FLOOR; plan `two-ball-oval` 2.3/2.4).
Each `.npz` is a knot-sampled `CyclePlan`: `pose`, `pose_vel`, `hand_rev`,
`hand_vel_rps`, `dt`, `catch_k`, `contact_knots` (inclusive ranges), plus the
catch evidence the plan carries: `t_td` (interpolated touch-down, plan clock),
`arrival_vel_mm_s` (ball velocity at the cup), `receive_tilt` (rad, the receive
axis is `tilt_geometry.cup_axis(*receive_tilt)`), `program` (`held_axis` | `level_pin` | `match`, the catch program; `held_axis` is exempt from CATCH_CLOSING_SPEED).

| file | what it is | provenance | expected |
|---|---|---|---|
| `fx_proto_capture.npz` | 237-knot chain, last-but-one STEADY catch (`catch_k` 161), realised axial cup speed -345 mm/s | wayfinder PROTOTYPE pose-level QP with the decision-28 capture span: `p13_run.run((0.95,150.,'L0t',0.50,'capture',30,1,2,1))`, w=10, arm `capture`, 2026-10-17 | REFUSES `CATCH_CLOSING_SPEED` (ratio 0.114 vs 0.25) |
| `fx_proto_pose.npz` | same cell, arm `pose` (pose-level QP, no capture span), -1742 mm/s | same run, arm `pose` | PASSES (ratio 0.575) |
| `fx_sep175.npz` | production level planner, 175 mm separation, apex 0.95, dwell 0.50 (workspace_xy_m patched to 0.25 so it solves), 350/5000/200000, hand 3900 | `tools/probes` chain (`plan_launch` + 3x `plan_steady` + `extend`) run BEFORE the check existed; third STEADY window | residual 21.44 deg: PASSES the re-based 30 deg cone (owner 2026-10-18); kept as a P3 shaper-target pin (> 20 deg) |
| `fx_dwell040.npz` | production level planner, 150 mm, apex 0.95, dwell 0.40, same limits | same chain | residual 24.41 deg: PASSES (30 deg cone); P3 target pin, min 0.772 g |

The two `fx_proto_*` files exist ONLY as data: the prototype pose-level QP is a
monkeypatch on `cup_cycle` in the wayfinder worktree (`.scratch/two-ball-oval/`)
and is NOT in committed code, so they cannot be regenerated here. They were
dumped (pose, pose_vel, hand_rev, hand_vel_rps, dt, catch_k, t_td, arrival
velocity, receive tilt) from the realised plan; `contact_knots` was filled in
afterwards from the chain's windows ((0,15),(46,70),(101,125),(156,180),(211,236)).
The `fx_sep175` / `fx_dwell040` cells cannot be regenerated either, now that
the check refuses them at planning time (that is its job).

The cone-REFUSING case (rung 1 through the production segment path, 32.1 deg) is generated in-test (`_oval_carry_segment`), not stored.

The rung-1 plan (150 mm, apex 0.95, dwell 0.50) is NOT stored: the test
regenerates it from production code (`_rung1_window` in the test file).
