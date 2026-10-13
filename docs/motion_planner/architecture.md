# Architecture

How the skill stack is wired on the Jetson and below it. Overview: [index](index.md). Layer design: `plans/archived/two-ball-skill-stack.md` section 2.1.

## Processes on the Jetson

| Process | Role |
|---|---|
| `skill_node` | Compiles the pattern into a wall-clock schedule, dispatches THROW / CATCH / REST skills, runs the learner, records outcomes. One thread; no solve on the emitter. |
| `trajectory_node` | Sole holder of the live plan state. Plans each segment from that state and splices it in. A 40 Hz emitter thread streams knots; the segment solve runs in a **worker process** (since 2026-10-13) so the emitter is never starved. |
| `teensy_bridge_node` | Takes the :5557 command stream, validates it in `SetpointPump`, sends UDP to the can-bridge Teensy. |
| `mocap_node` | QTM mocap, about 300 frames/s, with clock sync. Feeds `ball_tracker_node` (Kalman, landing prediction). |
| `ball_butler_node` | Ball Butler launcher client: throw announcements, calibration. |
| GUI | Browser dashboard (rosbridge); observes, does not command the leg path. |

`teensy_link/` is a root-level package, deliberately not installed into the ROS package: both launch files put the repo root on `PYTHONPATH`, so the bridge runs the live tree. Business logic lives in pure-Python modules; `*_node.py` files are thin wrappers (no ROS imports in `motion/`, `controller/`, `teensy_link/`).

## IPC

- `skill_node` to `trajectory_node`: ROS service `trajectory/install_segment` (kinds THROW, CATCH, REST).
- Tracker output: the `/balls` topic; hardware state: `/robot_state`.
- `trajectory_node` to `teensy_bridge_node`: ZeroMQ PUB/SUB, msgpack, commands on tcp://localhost:5557 (telemetry on :5556).
- `teensy_bridge_node` to the Teensy: UDP setpoint frames; the protocol version is the generated constant in `config/generated/udp_protocol.py` and must match on both ends or the link is dark.

## CAN side

The can-bridge Teensy owns the CAN buses, runs a 2 ms tick and does 500 Hz Hermite interpolation of seven axes (six legs and the hand). It hosts the fault machine and the `MAX_DEVIATION` guard, which is the leg-path safety authority. The hand has one master, the streamed lane; the Platform Teensy stroke engine retired at R1. CAN encoding must match the firmware (`Teensy_code_canbridge/`) and the generated `protocol_config` constants (negate, scale, int16, clamp); the firmware is the encoding authority. See [control flow](../can_bridge/control.md) and [safety](../can_bridge/safety.md).

## Timing

- Skills dispatch at `t_abs - lead` (about 0.10 s: plan plus a two-knot install margin).
- The emitter runs at 40 Hz; the Teensy interpolates to 500 Hz. Knots are installed at least two knots ahead of the emit cursor for the firmware's lookahead.
- `max_emit_gap_ms` (published by `trajectory_node`) is the observability for emitter starvation; a long gap means the emitter was blocked.
- Every segment is rest-terminal, so a late or refused skill leaves the previous segment's rest tail streaming.

## Invariants

Every enforced physical invariant, with its single enforcement point and test, is in `ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md`. Sections: transport and firmware guards; wire frame and emitter; planner and reference feasibility; levelling frame; possession and arming; hand; catch, arrival and reach; session-level refusals; retained past R4. Rule: a deletion must first check every row naming the deleted path.

## Configuration

Physical parameters come from `config/hardware_config.yaml`; run `python config/generate_config.py` after editing and never hand-edit generated files.
