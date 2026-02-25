# Motion Planner — Project Plan
**Stewart Platform + Linear Throw Axis**

---

## Scope & Assumptions

### System Overview
This plan covers model-based motion planning and control for a 6-DoF Stewart platform driven by string-based linear actuators with ODrive motor controllers. The platform hosts a linear throwing axis used for ball launch.

### Throw Axis Decoupling
The linear throwing axis is treated as kinematically decoupled from the Stewart platform throughout this plan. The throw axis contributes to the platform's total mass and inertia (accounted for in the dynamics model as a static payload), but its motion is planned and controlled independently. This decoupling is justified by the assumption that throw axis reaction forces are small relative to the platform's inertial loads during throw preparation, and that coordinating the two systems at the trajectory planning level is not required at this stage. If throw accuracy or platform stability issues emerge during integration testing, revisiting this assumption should be the first diagnostic step.

### Actuator Characteristics
The linear actuators are string-driven with approximately constant efficiency across their stroke and speed range. This simplifies the leg force → motor torque conversion to a fixed geometric ratio. The dominant unmodelled dynamics in the actuator are reflected motor inertia (rotor inertia scaled by the drive ratio squared), not friction.

### Hardware Safety Philosophy
There is only one robot. Repairs are time-consuming and costly. The bring-up strategy throughout this plan is designed around the principle of **maximum offline validation before hardware, and graduated hardware exposure**. Every phase that involves commanding the robot follows a progression: simulation/offline first → isolated leg bench test → mechanically supported platform → free platform at low speed → free platform at increasing speed. Current limits on the ODrive controllers should be set conservatively at all times and only relaxed when a phase demonstrates that higher limits are needed.

---

## Phase 1: Kinematic Foundation — DONE (2026-02-20)

**Goal:** Establish a correct, well-tested kinematic model before any dynamics or planning work builds on top of it. All subsequent phases depend on this being right.

> **Hardware exposure: None.** This phase is entirely offline computation and validation.

### Deliverables
- Position IK: platform pose → leg lengths (formalise/consolidate what you already have)
- Velocity IK: platform twist → leg velocities via Jacobian `J`
- Acceleration IK: platform acceleration → leg accelerations via `J̈` with bias term
- Inverse functions (FK) for each level, used primarily for validation

### Verification
- **Numerical Jacobian check:** compute `J` analytically, then verify against finite-difference of position IK at a range of poses. Errors should be at floating-point precision.
- **Round-trip test:** generate random platform twists, compute leg velocities via `J`, integrate leg lengths numerically using a 4th-order Runge-Kutta integrator with a step size small enough that integration error is below 1e-8, and verify the resulting platform pose matches direct integration of the twist to the same tolerance. The purpose is to validate the Jacobian, not the integrator — if discrepancies appear, halve the step size to confirm they are integration artefacts before investigating the Jacobian.
- **Bias term test:** hold platform at a fixed pose with nonzero velocity, verify `J̇q̇` matches the numerically-differentiated Jacobian-velocity product.
- **Singularity map:** sweep the workspace and log `det(JJᵀ)` — identify and document singular/ill-conditioned regions. Record poses where the condition number of `J` exceeds a defined threshold (e.g., 100). These regions will inform trajectory planning constraints in Phase 4.

---

## Phase 2: Standalone Control Process & IPC Layer — SOFTWARE DONE (2026-02-20)

**Goal:** Establish the non-ROS2 control process skeleton with IPC plumbing to ROS2, so all subsequent phases are developed and tested in their final runtime environment.

> **Hardware exposure: Isolated leg only.** All hardware tests in this phase are performed on a single leg disconnected from the platform, bench-mounted or clamped. No multi-leg or platform tests until Phase 3 Stage B.

### Deliverables
- Standalone Python control process with a fixed-rate main loop (target: 500–1000 Hz)
- Loop timing instrumentation (measure and log jitter on every cycle)
- ODrive torque passthrough mode interface (replace TRAP_TRAJ — send torque/current commands directly)
- Leg force → motor torque conversion module: fixed geometric ratio from string drive geometry, verified against known loads
- IPC layer (ZeroMQ or shared memory) with defined message schemas for: target state in, telemetry out, mode commands in
- ROS2 bridge node that translates ROS2 messages to/from IPC

### Verification

#### Software-only tests (no hardware)
- **Loop timing test:** run the loop for 60 seconds with no motion and no ODrive connection, log all cycle times. Verify mean and 99th-percentile jitter are within acceptable bounds. If 99th-percentile jitter exceeds 2× the nominal cycle period, profile and address before proceeding — this is a Phase 2 exit gate.
- **IPC latency test:** measure round-trip latency from ROS2 message publish to control process receipt. Should be well under one control cycle period.

#### Isolated leg bench tests
Before connecting any hardware, set ODrive current limits to a conservative value (e.g., 50% of the motor's rated current). These limits remain in place for all Phase 2 testing.

- **Single-leg passthrough smoke test:** command a constant small torque on the isolated leg, verify the leg moves and encoder feedback is received correctly. Verify the leg stops cleanly on mode switch to idle.
- **Emergency stop test (single leg):** verify that the control process correctly idles the leg on IPC loss, process crash, or explicit stop command. Test each failure mode individually.
- **Force conversion validation:** apply a known static load to the isolated leg (e.g., hang a calibrated weight) and command zero torque. Measure the motor current required to hold position via PD. Compare the measured current to the predicted current from the geometric force-to-torque conversion. Discrepancy should be under 10%.
- **Encoder direction and sign convention check:** command a small positive torque pulse, verify encoder position increases in the expected direction. Repeat for negative torque. This catches sign errors before they become dangerous on the assembled platform.

> **Note:** The six-leg coordinated torque test is deferred to Phase 3 Stage B, where it is performed on the mechanically supported platform. There is no value in running six disconnected legs simultaneously, and the test is only meaningful when the CAN bus is loaded with the full set of drives as they will be in operation.

---

## Phase 3: Gravity Compensation & Static Feedforward

**Goal:** Achieve stable, low-effort platform hold at arbitrary poses using dynamics-based feedforward, without any trajectory planning yet. This validates the dynamics model independently.

> **Hardware exposure: Staged bring-up.** This phase uses a three-stage hardware progression from mechanically supported platform to free-standing operation.

### Deliverables
- Platform dynamics model: rigid body mass and inertia parameters, including the static mass contribution of the throw axis assembly as a fixed payload at its nominal position
- Gravity wrench computation: `W_gravity = [0, 0, mg, 0, 0, 0]` in world frame, rotated to platform frame
- Leg force decomposition: `f_legs = Jᵀ W`
- Reflected motor inertia model: compute reflected inertia per leg (`J_motor × gear_ratio²`) for use in later phases — document the value but do not include in the Phase 3 feedforward (it only matters during acceleration)
- PD feedback on leg position/velocity to close the loop

### PD Tuning Guide
The PD loop is the inner feedback layer that corrects for modelling errors and disturbances. Tuning targets:
- **Steady-state accuracy:** ±1 mm position, ±0.1° orientation at the platform
- **Procedure:** Detailed per-stage procedure below. The general principle is: tune on the isolated leg first to establish gain order-of-magnitude, then re-tune on the supported platform, then validate on the free platform. Gains from earlier stages are starting points, not final values.
- **Gain scheduling:** if the platform feels underdamped at some poses and overdamped at others, gains may need to vary with configuration. Note this as a risk but defer gain scheduling until it is demonstrated to be necessary.

### Stage A — Isolated Leg (Bench Test)

**Setup:** Single leg disconnected from the platform, bench-mounted or clamped. ODrive current limits remain at the conservative value set in Phase 2.

**Purpose:** Validate all low-level infrastructure with zero risk to the robot. Establish baseline PD gain order-of-magnitude for the unloaded actuator.

**Tests:**
- **Preliminary PD tuning (unloaded):** Increase P gain until the leg holds a target position within ±0.5 mm with no oscillation. Add D gain to suppress any residual overshoot. Record these gains as the "unloaded baseline" — they will be too aggressive for the loaded platform.
- **Gravity feedforward unit test (single leg):** With the leg oriented vertically and a known mass attached, enable gravity feedforward for that single leg. Verify the computed feedforward torque roughly matches the load, and that the PD effort drops when feedforward is enabled.
- **Current limit adequacy check:** With the known mass attached, verify that the conservative current limit is sufficient to hold the load. If not, increase the limit incrementally with documented justification. This informs what the current limits need to be for the assembled platform.

### Stage B — Platform Supported

**Setup:** Assemble the full robot. Mechanically support the platform so the legs are not load-bearing — rest it on blocks, clamp the frame, or use a sling — such that if all legs go limp simultaneously, nothing falls or is damaged.

**Purpose:** Catch wiring errors, sign convention bugs, and CAN coordination issues in a mechanically safe configuration. These are the most likely classes of first-assembly bugs and the most dangerous if they occur on a free-standing platform.

**Tests:**
- **Six-leg CAN coordination test:** Command time-varying torque profiles to all six legs simultaneously at the full control rate. Verify CAN bus throughput is sufficient (no dropped frames), all legs receive commands within the same control cycle, and encoder feedback from all six legs is received within one cycle. Log any CAN frame timing violations. This is a Phase 3 Stage B exit gate — do not proceed to Stage C if CAN drops frames.
- **Direction and sign convention check (all legs):** Command a small positive torque pulse on each leg in sequence. Verify each leg moves in the direction that would shorten/extend it as expected by the kinematic model. A sign error on a single leg will cause the platform to fight itself when free-standing.
- **Multi-leg PD hold (supported):** Command all six legs to hold their current positions with the unloaded baseline gains from Stage A (or lower). Verify all legs respond, no leg oscillates, and the system is stable. This is a low-risk first test of the full control loop because the platform support prevents any consequence of instability.
- **Emergency stop test (six legs):** Trigger each failure mode (IPC loss, process crash, explicit stop) and verify all six legs idle simultaneously and cleanly. The platform is supported, so even a failure to idle is non-destructive.
- **Feedforward dry run (supported):** Enable gravity feedforward while the platform is still supported. Log the feedforward torques for each leg at the home pose. Verify they are in the expected direction and roughly the expected magnitude (compare to `platform_mass × g / 6` as a sanity check, adjusted for geometry). Do not remove the support yet.

### Stage C — Platform Free, Gravity Compensation Active

**Setup:** Remove mechanical support. Enable gravity feedforward. Start with PD gains at 50% of the unloaded baseline from Stage A.

**Purpose:** First unsupported operation under the new control system. Feedforward is active from the start so the PD loop is not solely responsible for holding the platform weight.

**Procedure:**
1. With feedforward active and PD at 50% of baseline, release the platform support gradually (don't just remove it — ease it out so you can re-engage if something goes wrong).
2. If the platform holds stable, log the steady-state error and PD effort.
3. Incrementally increase PD gains toward the ±1 mm / ±0.1° target. At each step, verify no oscillation before proceeding.
4. If steady-state error exceeds ±1 mm / ±0.1° with feedforward active and PD gains as high as they can go without oscillation, diagnose the feedforward model (mass, CoM location, force conversion) rather than continuing to raise PD gains.

### Verification (Phase 3 exit criteria — all performed at Stage C)
- **Static hold test:** Command the platform to hold several poses including tilted configurations. Measure steady-state position error and current draw vs. a pure PD baseline. Feedforward should significantly reduce the current/effort required to hold pose. Verify the ±1 mm / ±0.1° target is met.
- **Gravity vector rotation test:** Tilt the platform to a known angle, log the expected and commanded gravity compensation forces per leg, verify they are geometrically consistent.
- **Parameter sensitivity:** Vary mass and inertia parameters ±20% in software and observe effect on hold quality — this bounds how precisely you need to identify the physical parameters. If hold quality degrades unacceptably, consider a system identification experiment (chirp excitation + force measurement).
- **Log `Jᵀ` condition number** across the workspace to confirm force decomposition is stable at intended operating poses.

---

## Phase 4: Quintic Trajectory Generator

**Goal:** Given a start state and a target state (pose + velocity + acceleration) at a specified future time, generate a smooth trajectory that respects leg kinematic limits. This phase is tested at low speed only — full-speed validation occurs after Phase 5 adds inertia feedforward.

> **Hardware exposure: Offline first, then low-speed on free platform.** All trajectory generation logic is validated in simulation before any hardware execution. Hardware tests are at ≤25% of actuator velocity limits. Every trajectory is previewed offline before execution.

### Deliverables
- Quintic polynomial solver: 6 boundary conditions → 6 coefficients, per Cartesian DoF
- Trajectory evaluator: given time `t`, return `(x, ẋ, ẍ)` in Cartesian space
- Leg-space trajectory mapper: evaluate IK at each timestep to get `(q, q̇, q̈)`
- Feasibility checker: compute peak leg velocity and acceleration along proposed trajectory, compare against leg limits. Also evaluate the Jacobian condition number along the trajectory and reject paths that enter ill-conditioned regions identified in Phase 1.
- Speed limit interface: the trajectory manager must accept an externally-imposed speed scale factor (0.0–1.0) that uniformly scales all velocities and accelerations. This will be used by the runtime monitor in Phase 6 for singularity avoidance and protective slowdowns.
- Trajectory manager: handles active trajectory execution. At this phase, the manager executes a single trajectory to completion — mid-motion re-planning is deferred to Phase 7.

### Verification

#### Offline tests (no hardware)
- **Boundary condition test:** Evaluate polynomial at `t=0` and `t=T`, verify position/velocity/acceleration exactly match specified boundary conditions.
- **Limit checking test:** Construct trajectories that are known to violate leg velocity/acceleration limits and verify the feasibility checker correctly rejects them. Separately, construct trajectories that pass through ill-conditioned Jacobian regions and verify rejection.
- **Visualisation:** Plot Cartesian and leg-space trajectories (position, velocity, acceleration) for a set of representative moves — inspect by eye for smoothness and absence of spikes.
- **Speed limit test (offline):** Compute a trajectory, then recompute with speed scale factor at 0.5. Verify the trajectory shape is preserved and peak velocities/accelerations are halved.
- **Torque preview:** For each trajectory that will be executed on hardware, compute the expected feedforward torques offline before executing. Verify no torque exceeds the ODrive current limit. Verify no leg velocity or acceleration exceeds the limit. This is a mandatory pre-flight check — do not execute a trajectory on hardware without first previewing its torque profile.

#### Hardware tests (low speed, free platform)
All hardware tests use trajectories whose peak velocities are ≤25% of actuator velocity limits. Every trajectory is previewed offline (torque preview test above) before execution.

- **Small move from home:** Command a small move (5–10 mm translation, near home pose) to a target with zero velocity and acceleration at `t=T`. Verify the platform reaches and holds the target without oscillation.
- **Graduated move distance:** Incrementally increase move distance and orientation change. At each step, verify tracking is smooth and torques are well within limits before proceeding to a larger move.
- **Multi-pose sequence:** Execute a series of moves to different poses across the workspace (still at ≤25% speed). Verify the platform reaches each target and holds within the ±1 mm / ±0.1° steady-state target.
- **Speed limit test (hardware):** Execute a trajectory, then re-execute with speed scale factor at 0.5. Verify the physical motion matches the offline prediction.

---

## Phase 5: Full Inertia Feedforward & Dynamic Compensation

**Goal:** Extend the feedforward model to include platform inertia and reflected motor inertia during motion, enabling accurate force feedforward during fast moves. After this phase, re-run Phase 4 trajectory tests at progressively increasing speed.

> **Hardware exposure: Graduated speed ramp-up.** Do not jump from 25% speed (Phase 4) to full speed. Increase in increments (25% → 50% → 75% → 100%), validating tracking and torques at each level before proceeding.

### Deliverables
- Full Newton-Euler dynamics: `F = ma_com`, `τ = Iα + ω × Iω`
- Full 6D wrench computation from desired platform acceleration
- Reflected motor inertia compensation: add `J_reflected × q̈` per leg to the feedforward torque, using the reflected inertia values documented in Phase 3
- Combined feedforward: gravity + platform inertia + reflected motor inertia terms summed before `Jᵀ` decomposition (for platform terms) and added per-leg (for motor inertia terms)

### Verification

#### Offline tests (no hardware)
- **Torque profile preview:** For the same trajectories used in Phase 4 hardware tests, compute the full feedforward torques (gravity + inertia + reflected motor inertia) at 50%, 75%, and 100% speed. Verify no torque exceeds current limits at any speed. If a trajectory exceeds limits at a given speed, it must not be executed at that speed — lengthen the move duration until the profile is feasible.

#### Hardware tests (graduated speed ramp-up)

Execute the following tests at 25% speed first. Only proceed to the next speed increment when the current level passes.

- **Step response comparison:** Execute identical moves with gravity-only vs. full inertia feedforward at the current speed level. Measure tracking error (platform pose vs. desired trajectory). Full feedforward should reduce peak tracking error.
- **Trajectory replay at speed:** Re-run Phase 4's trajectory set at the current speed level. Verify tracking error remains within acceptable bounds:
  - At 50% speed: ≤2 mm peak position error, ≤0.3° peak orientation error
  - At 75% speed: ≤2.5 mm peak position error, ≤0.4° peak orientation error
  - At 100% speed: ≤3 mm peak position error, ≤0.5° peak orientation error
- **Continuity check:** Verify position, velocity, and acceleration profiles are smooth with no spikes or discontinuities in the commanded torques at the current speed level.
- **Torque prediction validation:** Log commanded feedforward torques and actual motor currents during a known trajectory. If discrepancies exceed 15%, investigate: the most likely sources are inaccurate mass/inertia parameters, CoM offset errors, or unmodelled dynamics. Document findings for Phase 6. Do not increase speed until discrepancies are understood.

#### Full-speed validation (after 100% speed level passes)
- **High-speed move test:** Execute moves at the upper end of the intended speed envelope, verify torque commands are smooth and tracking remains acceptable.
- **Energy consistency check:** Verify that the feedforward torques are physically consistent with the trajectory (compute net work done and compare to kinetic energy change).

---

## Phase 6: Hardening & Operational Readiness

**Goal:** Make the system robust enough for sustained operation and aggressive commanding before connecting the ball predictor. The ball prediction integration (Phase 7) has the highest likelihood of pushing the platform into extreme operating conditions, so all protective systems must be in place first.

> **Hardware exposure: Progressive stress.** Build up the stress test library gradually — start with moderate trajectories and add more aggressive ones as protective systems are validated. Fault injection tests are performed at low speed first.

### Deliverables
- Workspace limit enforcement: hard and soft limits in both Cartesian and leg space, active at all times
- Singularity avoidance: runtime condition number monitoring that feeds back through the Phase 4 speed limit interface — degrade speed gracefully as condition number rises, abort trajectory if a hard threshold is exceeded
- Watchdog and fault recovery: ODrive fault detection, IPC loss handling, leg overextension recovery
- Telemetry and logging: full state logging via ROS2 rosbag for post-run analysis
- Performance dashboard: real-time visualisation of loop timing, tracking error, leg states, feedforward vs. feedback torque split
- Stress test trajectories: a library of aggressive test trajectories that exercise the full speed/acceleration envelope, workspace boundaries, and near-singular configurations, used to validate all protective systems before Phase 7

### Verification

#### Protective system validation (at low speed first)
Implement and validate all protective systems before executing any aggressive trajectories. These tests use slow moves (≤25% speed) that intentionally approach or cross limits.

- **Workspace boundary test (low speed):** Command slow trajectories that approach workspace limits. Verify soft limits decelerate and hard limits abort cleanly. Test all six faces of the Cartesian workspace envelope and all six leg length limits individually.
- **Singularity avoidance test (low speed):** Command a slow trajectory that would pass through an ill-conditioned region. Verify the runtime monitor slows the trajectory via the speed limit interface and aborts if the condition number exceeds the hard threshold.
- **Fault injection test (static):** With the platform holding a static pose, deliberately trigger an ODrive fault (e.g., overcurrent on a single drive) and verify the platform idles all legs safely. Repeat for IPC loss and process crash. The platform may drop when legs idle — ensure nothing below is at risk and be prepared to catch or support the platform.
- **Fault injection test (low-speed motion):** Repeat fault injection during a slow trajectory. Verify the platform stops safely mid-motion.

#### Endurance and stress testing (graduated)
Only after all protective systems pass low-speed validation.

- **Moderate endurance test:** Run the system for 30 minutes executing moderate-speed trajectories (50% speed). Monitor loop jitter, tracking error, and temperature. No faults or degradation.
- **Aggressive trajectory ramp-up:** Build the stress test library incrementally. Start with trajectories at 75% of speed/acceleration limits. Verify all protective systems function correctly. Then add trajectories at 90% and finally 100% of limits. At each level, confirm workspace limits, singularity avoidance, and fault recovery all work before proceeding.
- **Full endurance test:** Run the full stress test library in a loop for 60 minutes at full speed. Monitor loop jitter, tracking error, and temperature. No faults or degradation.
- **Fault injection test (full speed):** Repeat fault injection during an aggressive trajectory. Verify the platform stops safely.
- **Telemetry completeness check:** Play back a rosbag from the endurance test and verify all required signals are present and correctly timestamped for post-analysis.

---

## Phase 7: Integration with Ball Prediction

**Goal:** Connect the motion planner to the ball predictor so that throw-preparation moves are commanded automatically to the right pose/velocity/acceleration at the right time. All protective systems from Phase 6 are active.

> **Hardware exposure: Simulated ball feed first, then live.** All re-planning logic is validated with synthetic ball trajectories before connecting the live ball predictor. The Phase 6 protective systems are the safety net — verify they remain effective under ball-prediction-driven commanding.

### Deliverables
- Interface between ball predictor output and trajectory manager input (target state + deadline time)
- Mid-motion re-planning: extend the Phase 4 trajectory manager to accept new targets during execution, re-planning from the current state with guaranteed continuity in position, velocity, and acceleration at the re-plan instant
- Feasibility-gated acceptance: only commit to a target if the trajectory planner confirms it is reachable within limits
- Graceful handling of target updates (ball prediction improves as ball approaches) — re-plan from current state when a better estimate arrives
- Timeout handling: if no target is received within a configurable deadline after entering TRACKING mode, or if a target arrives too late for a feasible trajectory, the system returns to IDLE rather than attempting an infeasible move
- Mode sequencing: IDLE → TRACKING → THROW_PREP → THROW, with the motion planner active in THROW_PREP

### Verification

#### Offline / simulation tests (no hardware)
- **Re-planning continuity test (offline):** Simulate a trajectory in progress, inject a new target, and verify the re-planned trajectory has continuous position, velocity, and acceleration at the join. Inspect commanded torque profiles for discontinuities.
- **Timeout logic test (offline):** Simulate TRACKING mode entry with no ball prediction target. Verify the system times out and returns to IDLE after the configured deadline. Separately, simulate a target that arrives too late for feasible trajectory planning — verify rejection and return to IDLE.
- **Unreachable target test (offline):** Inject a target that requires exceeding leg limits. Verify it is correctly rejected.

#### Hardware tests with synthetic ball feed
Use synthetic ball trajectories (known intercept point and time) injected in place of the real ball predictor. Start with targets near the centre of the workspace and gradually move toward the edges.

- **Simulated ball feed test (conservative targets):** Inject synthetic ball trajectories with intercept points near the workspace centre and generous time horizons. Verify the platform reaches the correct pose/velocity/acceleration at the right moment.
- **Re-planning continuity test (hardware):** During a live trajectory driven by a synthetic ball feed, inject an updated target. Verify smooth re-planning with no discontinuities in platform motion or commanded torques.
- **Late-update test:** Send an initial target, then update it 50–100 ms later with a slightly different intercept point. Verify smooth re-planning.
- **Unreachable target test (hardware):** Inject a target that requires exceeding leg limits. Verify it is correctly rejected and the platform remains in a safe state with Phase 6 protections active.
- **No-target / late-target test (hardware):** Enter TRACKING mode but send no ball prediction target. Verify the system times out and returns to IDLE. Separately, send a target that arrives too late — verify rejection and return to IDLE.
- **Timing accuracy test:** Measure the error between the planned arrival time and the actual time the platform reaches the target state. This is the end-to-end timing budget for the throw.
- **Graduated stress with synthetic feed:** Progressively inject targets closer to workspace edges, with shorter time horizons, and with more frequent re-planning updates. Verify Phase 6 protective systems handle all cases correctly.

#### Live ball predictor integration
Only after all synthetic feed tests pass.

- **Full-loop stress test:** Run continuous simulated ball feeds for 30+ minutes with varying intercept points, including edge cases (near workspace limits, near-singular configurations, rapid re-planning). Verify all Phase 6 protective systems remain effective under ball-prediction-driven commanding.
- **Live ball predictor test:** Connect the real ball predictor. Start with slow, predictable ball trajectories. Gradually increase difficulty. Monitor all telemetry for anomalies.

---

## Dependencies & Risk Notes

| Risk | Mitigation |
|---|---|
| Platform mass/inertia parameters poorly known | Phase 3 sensitivity test bounds the impact; consider a system ID experiment (chirp excitation + force measurement) if hold quality is insufficient |
| Throw axis coupling is non-negligible | Monitor platform stability during throw axis motion after Phase 7 integration. If systematic pose errors correlate with throw axis acceleration, revisit the decoupling assumption — the minimum intervention is adding throw axis reaction force as a feedforward disturbance term |
| Loop jitter exceeds acceptable level in Python | Profile Phase 2 thoroughly; the 99th-percentile jitter gate is a hard exit criterion. Migrate hot loop to C++ extension if needed before Phase 4 |
| CAN bus bandwidth insufficient for 6 legs at full rate | Phase 3 Stage B six-leg coordinated test is the gate. If frame drops occur, reduce control rate or investigate CAN FD. This must be resolved before Phase 3 Stage C |
| Jacobian singularities in operating workspace | Phase 1 singularity map feeds into Phase 4 feasibility checker and Phase 6 runtime monitor. If singularities fall within the intended workspace, trajectories must actively route around them |
| Reflected motor inertia dominates actuator dynamics | Phase 3 documents the reflected inertia magnitude. If it exceeds 20% of the platform-induced leg force during typical accelerations, it must be included in Phase 5 feedforward (it is included by default in this plan) |
| Ball predictor latency eats into throw-prep window | Measure end-to-end latency early in Phase 7; may require moving the predictor closer to the control process if ROS2 bridge adds too much delay |
| Ball predictor fails to produce a target | Phase 7 timeout handling ensures the system returns to IDLE rather than waiting indefinitely or attempting a last-known stale target |
| Hardware damage during bring-up | Three-stage bring-up (isolated leg → supported platform → free platform) catches wiring, sign, and coordination bugs before they can cause damage. Conservative current limits throughout. Torque profile preview before every new hardware trajectory. |

---

## Phase Gate Summary

Each phase has explicit exit criteria. Do not begin the next phase until the current phase's verification tests pass.

| Phase | Key Exit Gate |
|---|---|
| 1 — Kinematics | Numerical Jacobian error < floating-point precision; singularity map complete |
| 2 — Control Process | 99th-percentile loop jitter < 2× nominal period; isolated leg passthrough verified; e-stop functional on single leg |
| 3A — Isolated Leg Tuning | Unloaded PD baseline gains established; feedforward unit test passes; current limit adequacy confirmed |
| 3B — Supported Platform | Six-leg CAN throughput verified; all leg directions correct; multi-leg PD hold stable; e-stop functional on all legs |
| 3C — Free Platform | Static hold accuracy ≤ ±1 mm / ±0.1° with feedforward active across multiple poses |
| 4 — Trajectory Generator | Boundary conditions exact; feasibility checker rejects known-bad trajectories; low-speed (≤25%) tracking verified on hardware; torque preview workflow established |
| 5 — Inertia Feedforward | Tracking error within bounds at each speed level (50% → 75% → 100%); torque prediction within 15% of measured |
| 6 — Hardening | All protective systems validated at low speed; fault injection passes at all speeds; 60-min full-speed endurance pass |
| 7 — Ball Prediction | All tests pass with synthetic ball feed first; timing accuracy within budget; re-planning continuous; timeout handling verified; 30-min stress test pass; live predictor integration stable |

---

## Appendix: Phase 1 & 2 Implementation Notes (2026-02-20)

### Files created

| File | Lines | Purpose |
|------|-------|---------|
| `motion/geometry.py` | ~65 | `StewartGeometry` class — loads all platform constants from `hardware_config.py` |
| `motion/ik_solver.py` | ~270 | Position IK, analytical Jacobian, velocity/acceleration IK, numerical FK (Newton-Raphson), rotation utilities |
| `motion/workspace.py` | ~130 | Leg extension checking, condition number, reachability, singularity mapping |
| `motion/conversions.py` | ~70 | Leg force ↔ motor torque, extension ↔ rev, velocity conversions |
| `motion/ipc.py` | ~190 | ZeroMQ PUB/SUB IPC with msgpack; `ControlProcessIPC` + `BridgeIPC` classes |
| `motion/control_loop.py` | ~230 | Fixed-rate standalone process with timing instrumentation, heartbeat watchdog |
| `motion_bridge_node.py` | ~170 | ROS2 bridge: subscriptions → IPC → publishers |
| `motion/tests/test_kinematics.py` | ~260 | 6 Phase 1 verification tests |
| `motion/tests/test_control_loop.py` | ~170 | 3 Phase 2 verification tests |
| `motion/__init__.py` | ~40 | Public API exports |

### Phase 1 verification results (all PASS)

| Test | Result | Notes |
|------|--------|-------|
| Regression vs sp_ik.py | PASS (0.00e+00 mm) | Exact match with legacy implementation |
| Numerical Jacobian | PASS (1.11e-06) | Analytical vs finite-difference, 30 poses |
| Round-trip (twist integration) | PASS (2.19e-05 mm/s) | Twist → leg velocities → integrate → verify |
| Bias term (Jdot * twist) | PASS (1.71e-06 mm/s^2) | Analytical vs numerical J-dot |
| FK round-trip | PASS (0.00e+00 mm, 0.00e+00 rad) | IK → FK → compare, 20 poses |
| Singularity map | INFO | 929/1944 poses reachable; cond range 449-644 |

### Phase 2 verification results

| Test | Result | Notes |
|------|--------|-------|
| Loop timing | SKIP | Requires pyzmq/msgpack (Jetson only) |
| IPC latency | SKIP | Requires pyzmq/msgpack (Jetson only) |
| Force conversion | PASS (<1e-14) | Round-trip force/torque; spool radii ~11 mm |

### Findings & items for future investigation

1. **Singularity map condition numbers (449-644)**: These raw condition numbers reflect mixed units (mm for translation, rad for rotation) in the Jacobian. This does not indicate the platform is near-singular — it means the raw `cond(J)` is not directly comparable to a "well-conditioned = near 1" interpretation. For Phase 6 runtime monitoring, consider normalizing the Jacobian (e.g., characteristic length scaling) so condition numbers are more interpretable.

2. **New dependencies**: `pyzmq` and `msgpack` are required on the Jetson for the IPC layer. These are not installed on the Windows dev machine, so IPC-dependent tests gracefully skip with `[SKIP]` on Windows.

3. **Rotation perturbation for numerical Jacobian**: Jacobian columns 3-5 correspond to world-frame angular velocity, not rotation vector components. Numerical validation must perturb rotation as `exp(skew(delta * e_i)) @ R`, not as `rotvec + delta * e_i`. This was caught and fixed during Phase 1 testing.

4. **Windows Unicode**: Test output uses ASCII-only characters to avoid cp1252 encoding errors on Windows terminals. The code itself uses standard Python unicode strings internally.

5. **Phase 2 hardware test harness built** (2026-02-25): `tools/single_leg_test.py` is a standalone script that bypasses ROS2 and talks directly to a single ODrive via python-can. It implements all four isolated-leg bench tests: torque passthrough smoke test, emergency stop, encoder sign check, and force conversion validation. See `tools/README.md` for full documentation. The harness uses conservative current limits (50% of rated) and sends IDLE on all exit paths. Hardware execution is pending — run on the Jetson when the robot is available.
