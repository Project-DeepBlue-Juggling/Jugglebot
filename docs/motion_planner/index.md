# Motion planning — the skill stack (2026-10)

> **Status.** The CasADi/IPOPT MPC chain was removed 2026-09-01 (final implementation at git tag `mpc-final`).
> The unified 7-DoF ring and the FSM toss/reload choreography were deleted 2026-09-24 (tag `fsm-final`).
> The live planner is a schedule-driven skill stack: skills dispatched on the shared CAN wall clock, rest-terminal segments, one hand master.

## Scope

This section covers the Stewart platform's motion path on the Jetson and the math under it (kinematics, dynamics, workspace). The CAN-bridge firmware has its own section: [CAN Bridge](../can_bridge/index.md). Ball Butler (the launcher) has an independent control path.

## Live data path

```
skill_node  (schedule -> THROW / CATCH / REST skills, learner)
   | ROS service trajectory/install_segment
trajectory_node  (segment solve in a WORKER PROCESS; 40 Hz emitter thread)
   | ZMQ :5557 (msgpack)
teensy_bridge_node  (SetpointPump -> UDP)
   |
can-bridge Teensy  (500 Hz Hermite interpolation, MAX_DEVIATION guard)
   |
leg ODrives 0-5  +  hand ODrive 6 (streamed lane, one master)
```

Details, processes and timing: [Architecture](architecture.md).

## Segments and feasibility

Every segment is rest-terminal and is planned from the live state at the splice knot, so the joined trajectory is C2 there. A segment is installed only if it passes the robot's segment gates (`ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py`; invariants in `ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md`) and lies inside the admissible box (`config/generated/admissible_box.yaml`, produced by `tools/admissible_sweep.py`). An infeasible segment is refused with the gate's code and the attempt ends on the previous segment's rest tail.

## Learner

The throw command is the commanded landing plus apex, chosen by a memory-based learner (identity prior with no data; local-linear fit over remembered throws). The catch is aimed from the tracker's ballistic fit (`tracking/flight_fit.py`). Source: `plans/archived/two-ball-skill-stack.md` sections 2.4-2.6.

## Perception and outcome

Perception is QTM mocap via `mocap_node` (about 300 frames/s) feeding the ball tracker, plus the hand's cup ball sensor for possession. The observed landing of each throw becomes one learner row; no observation means no row and the attempt ends.

## Safety

The leg-path safety authority is the can-bridge Teensy's `MAX_DEVIATION` guard and fault machine; nothing on the Jetson is in that loop. The Jetson-side gates (feasibility, the admissible box) keep plans sane but are not safety. See [CAN Bridge safety](../can_bridge/safety.md) and [control flow](../can_bridge/control.md).

## What is still on this site

- [Kinematics](kinematics.md), [Dynamics](dynamics.md), [Workspace](workspace.md): current math.
- [Validation Results](results.md): historical record of earlier hardware tests.
- [Motor Guard (parked)](control_loop.md): the parked `motor_guard.py` and its Hermite twin role.

## History

- Tag `mpc-final`: the MPC chain, see `logbook/2026-09-01-mpc-chain-removed.md`.
- Tag `fsm-final`: the FSM stack, see `logbook/2026-09-24-skill-stack-r4-fsm-deletion.md`.
- The design and its rungs: `plans/archived/two-ball-skill-stack.md`.
