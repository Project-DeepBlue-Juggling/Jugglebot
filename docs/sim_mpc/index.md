# Simulation MPC Overview

!!! info "Scope: MuJoCo Simulation Only"
    This MPC controller runs in the **offline MuJoCo simulation** (`sim/`). It replaces the production motion planner's quintic trajectory + 500 Hz IK pipeline with a CasADi nonlinear MPC that jointly optimizes platform pose and leg commands. The production motion planner (documented in [Motion Planner](../motion_planner/index.md)) remains unchanged — this is a parallel development path for exploring optimal control strategies before porting to hardware.

The simulation MPC solves for Stewart platform motion using **inverse kinematics encoded as NLP constraints** rather than as an explicit computation step. At each control cycle, the MPC receives a target pose (with optional arrival deadline and twist), solves a nonlinear program to find the optimal sequence of leg commands, and applies the first command to the MuJoCo plant.

## Why MPC?

The production motion planner uses a two-stage approach: plan a quintic trajectory offline, then track it open-loop at 500 Hz. This works well for pre-planned motions but has limitations for ball catching:

- **No receding-horizon replanning.** Once a trajectory is committed, the controller tracks it blindly. If the ball prediction updates mid-flight, the system must abort and replan from scratch.
- **Feasibility is binary.** The production feasibility checker says yes or no — it can't find the "best reachable" pose when the ideal catch pose is barely infeasible.
- **No actuator model in planning.** Quintic trajectories are planned in Cartesian space and checked against motor limits after the fact. The MPC plans directly in motor command space with actuator dynamics as constraints.

The MPC addresses all three: it replans every 20 ms, naturally finds the best achievable trajectory within constraints, and models actuator lag explicitly.

## Architecture

```
                    TargetSource
                    (adapters for each input mode)
                        |
                        | TargetCommand (pose, arrival_time, twist, hand_cmd)
                        v
                  +-----+------+
                  |  main.py   |  50 Hz control loop
                  |  MPC solve |
                  +-----+------+
                        |
            +-----------+-----------+
            |                       |
     leg commands (6)         hand command
            |                       |
            v                       v
      +-----------+          +-----------+
      | MuJoCo    |          | MuJoCo    |
      | leg       |          | hand      |
      | actuators |          | actuator  |
      +-----------+          +-----------+
            |                       |
            +-------+-------+------+
                    |
              MuJoCo physics
                    |
                    v
              PlantState
              (sensors → leg ext, platform pose, twist, hand pos)
```

The MPC controls only the 6 Stewart platform legs. The hand actuator is commanded separately by the hand coordinator, matching the production architecture where the Teensy handles hand trajectories independently.

## Module Map

```
sim/
├── controller/
│   ├── mpc.py          CasADi NLP construction and solve
│   └── params.py       MPCParams dataclass (all tuning knobs)
├── plant/
│   ├── interface.py    PlantInterface ABC + PlantState dataclass
│   └── mujoco_plant.py MuJoCo simulation plant
├── ball/
│   └── manager.py      Ball spawn, capture detection, kinematic hold
├── ball_butler/
│   └── sim.py          Ball Butler throw simulator (pure-kinematic)
├── hand/
│   ├── coordinator.py  Hand state machine + ball lifecycle
│   ├── trajectory.py   Catch/throw hand trajectories (Teensy port)
│   ├── feasibility.py  Coarse-horizon MPC reachability check
│   ├── ballistics.py   Inverse ballistics + orientation computation
│   └── planner.py      Throw-catch plan generation (14-step pipeline)
├── input/
│   ├── scripted.py     Pre-defined test sequences (T1-T6, DT1-DT8)
│   ├── interactive_catch.py  Interactive ball spawn + catch
│   ├── continuous_throw_catch.py  Self-throw-catch juggling loop (--juggle)
│   ├── toss_loop.py    Continuous-motion toss loop (--cycle-time)
│   ├── sim_control.py  Pause/step/speed viewer controls
│   ├── spacemouse.py   SpaceMouse input adapter
│   └── keyboard.py     Keyboard input adapter
├── viz/
│   ├── telemetry.py    CSV logging + StepRecord dataclass
│   ├── horizon.py      MPC predicted-horizon renderer
│   └── dashboard/      Live web dashboard (SSE)
├── analysis/
│   ├── record_baselines.py  Record sim baselines for T1-T6 (headless)
│   ├── compare.py           Sim-to-real overlay plots + gap metrics
│   └── baselines/           Saved baseline CSVs
├── main.py             Entry point + 50 Hz loop + TargetSource adapters
├── demo_mpc.py         Standalone MPC demo (Phase 2)
├── sweep_speed_ratio.py  Batch sweep of --platform-event-speed-ratio
├── test_hand_stroke.py   Visual hand stroke smoke test
└── MPC_BUGS.md         Known issues tracker
```

## Data Flow

Each control step (20 ms) follows this sequence:

```
1. Read PlantState from MuJoCo sensors
2. TargetSource.update() → TargetCommand
3. Handle ball spawning (if any)
4. Handle hand commands (prime, catch/throw sequence, position)
5. MPC solve:
   a. Build reference trajectory (all nodes = target pose)
   b. Compute urgency multipliers (timed targets use ramp;
      toss loop uses ASAP = uniform 1.0)
   c. Pack parameter vector (current state + reference + urgency)
   d. Warm-start from shifted previous solution
   e. IPOPT solve → optimal leg commands
   f. Extract first command u[0]
6. Apply leg command to plant
7. Step MuJoCo physics
8. Check ball capture
9. Log telemetry
```

## Quick Links

| Topic | Page | Key Question |
|---|---|---|
| [Usage](usage.md) | Installation, CLI modes, telemetry, tests | How do I run and analyze the simulation? |
| [NLP Formulation](nlp_formulation.md) | Decision variables, cost, constraints | How does the optimizer find leg commands? |
| [Variable Horizon](variable_horizon.md) | Fine/coarse timesteps, urgency ramp | How does the MPC see both near and far? |
| [Hand & Ball Physics](hand_and_ball.md) | Trajectories, ballistics, capture, throw-catch, toss loop | How does catching/throwing work? |
| [Control Loop](control_loop.md) | 50 Hz loop, target sources, hand coordination | What happens every 20 ms? |
| [Plant Interface](plant.md) | MuJoCo plant, coordinate conventions | How does the sim model the robot? |
| [Tuning Guide](tuning.md) | Parameters, weights, solver options | How do I tune the MPC? |

## Relationship to Production Motion Planner

| Aspect | Production (motion/) | Simulation (sim/controller/) |
|---|---|---|
| Planning approach | Quintic trajectory, open-loop | Receding-horizon NMPC |
| Control rate | 500 Hz | 50 Hz |
| Actuator model | None (IK only) | First-order lag (τ = 30 ms) |
| IK | Explicit computation | CasADi symbolic constraint |
| Dynamics | Explicit feedforward | Not modeled (MuJoCo handles physics) |
| Solver | Polynomial evaluation | IPOPT (nonlinear interior-point) |
| Runtime | Jetson Orin Nano | Desktop (Windows/Linux) |
| Dependencies | numpy only | numpy + CasADi + MuJoCo |
