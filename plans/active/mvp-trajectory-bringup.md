---
title: MVP Trajectory Bringup — Simple Streaming Control to Two-Ball Juggling
created: 2026-07-07
status: active
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/ipc.py::make_mpc_command
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py::_MpcCommandSetpointSource
  - controller/teensy_link/setpoint_pump.py::SetpointPump
  - ros_ws/src/jugglebot/jugglebot/motion/ik_solver.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
---

# MVP Trajectory Bringup

## Context

### Why this exists

The MPC-based control stack (`run_mpc.py` + `controller/mpc.py`, 40 Hz CasADi solve)
has consumed weeks of effort: the Jetson is compute-marginal at 40 Hz, and the
simulated behaviour is not smooth. For the MVP the MPC is **removed from the hot
path** — kept dormant and untouched for future, more complex behaviours — and
replaced with a deliberately simple Jetson-side trajectory generator that streams
waypoints to the can-hub Teensy over the already-validated setpoint path.

MVP goals, in bringup order:

1. **Direct waypoint commands** — profiled point-to-point platform moves via a ROS2
   service, starting at very low actuator limits, ramped up across sessions.
2. **SpaceMouse streaming** — continuous target following through the same layer.
3. **Timed target states** — ~arbitrary ((position), (orientation), arrival_time)
   requests; infeasible states (too-tight timing or unreachable) **loudly rejected**,
   never silently dropped.
4. **BB→Jugglebot "reload"** — Ball Butler aims at Jugglebot's ACTIVE position and
   throws; Jugglebot tilts to align with the incoming ball's velocity and catches it.
   Timing-critical. Exposed as a ROS2 action.
5. *(stretch)* **Single-ball self-toss** loop.
6. *(extra stretch)* **BB-initiated two-ball juggling** — BB throws a second ball to
   an already-loaded Jugglebot, which begins a two-ball pattern.

The trajectory generation is shallowly kinetics-aware (leg jerk is the binding
actuator constraint; a lean-into-translation heuristic is included but opt-in) —
explicitly **not** an optimal-control rework.

### Decisions locked (2026-07-07)

1. **Topology**: a new thin ROS2 node (`trajectory_node`) wrapping a new pure-Python
   package (`jugglebot.motion.trajectory`), publishing `make_mpc_command` dicts on
   ZMQ :5557 — the exact seam `run_mpc.py` used. `teensy_bridge_node` consumes :5557
   unchanged. `mpc_bridge_node` leaves the launch file (source retained).
2. **Knot rate: fixed 40 Hz.** Firmware `SEGMENT_T_S = 0.025 s` stays compile-time.
   Configurable knot rate is deferred (see § Deferred).
3. **Sim gates apply to the ballistic phases only** (reload catch, self-toss,
   two-ball). Waypoint/spacemouse bringup goes hardware-direct at low limits.
4. **Reload is a ROS2 action** (phase feedback, outcome result, cancellation), not a
   plain service.
5. **Lean shaping ships default-OFF** with a single hardware A/B in the ramp phase;
   the feasibility gate's always-on jerk/accel/vel enforcement is the primary
   smoothness mechanism.
6. Work branches from `ea39f89` on a new branch `mvp-trajectory-bringup` (the
   SET_LEAD_ACCEL_LIMIT lead-clamp experiment was abandoned; tree is clean).

### Ground truth that shapes the plan

- **The streaming path already exists and is validated.** `teensy_bridge_node`'s
  `_MpcCommandSetpointSource` (teensy_bridge_node.py:214) subscribes ZMQ :5557
  topic `mpccmd`, feeds `SetpointPump` (setpoint_pump.py:152 — mm→rev knots
  u0/u1/u2 + v0, per-step |Δu0| ≤ 0.3 rev gate), and streams UDP `SETPOINT` frames
  to the can-hub Teensy, whose 500 Hz cubic-Hermite interpolator
  (leg_interp.cpp:202–386, bit-exact vs the Python reference, residual 5.5e-7 rev)
  drives CAN3. The path is gated off by `enable_setpoint_output:=false` and the
  heartbeat `mpc_active` bit. Anything that publishes valid `make_mpc_command`
  dicts at ≥4 Hz effective (250 ms staleness E-STOP) with 40 Hz knot spacing gets
  the whole validated chain for free.
- **Validated quintic math already exists**: `controller/hermite.py`
  (`quintic_interp`:15, `quintic_interp_with_accel`:88, `quintic_jerk_integral`:161)
  and `controller/feasibility.py` (`quintic_peak_vel_per_axis`:155,
  `quintic_peak_acc_per_axis`:196). The full pose→leg chain lives in
  `motion/ik_solver.py` (IK :165, Jacobian :190, twist→leg-vel :225, J̇ :269,
  accel→leg-accel :303, FK :335) plus `motion/workspace.py`.
- **BB's ROS2 stack lives in this repo** (same Jetson, same graph): `bb/throw`
  action (yaw/pitch/speed/absolute time, loud rejections with binding axis +
  shortfall), `bb/throw_at_target` service (inverse ballistics in
  `can/throw_ballistics.py`), `bb/reload` Trigger, `bb/heartbeat` 10 Hz
  (`ball_in_hand`, state), `throw_announcements` (`ThrowAnnouncement`: release
  pos/vel, ToF, predicted landing pos/vel/time). Temporal accuracy: arrival error
  ≤14 ms worst / <10 ms mean (the 44 ms release latency is already compensated —
  `BB_OP_THROW_RELEASE_LATENCY_MS`, hardware_config.py:369). Throws need ≥~2.5 s
  lead or come back `CANT_MAKE_LEAD`. Limits: speed ≤5.0 m/s, height ≤0.5 m,
  pitch 12–85°, yaw 0–185°.
- **The catch machinery exists and has caught balls smoothly on hardware.**
  mocap → `ball_tracker_node` (`/balls`) → `catch_coordinator_node` emits
  `catch/dynamic_target` (`DynamicTargetCommand`; `arrival_time` in the
  perf_counter clock domain) and arms the hand: prime via `smooth_move_hand` to
  `JB_OP_HAND_CATCH_PRIME_REV`, soft catch gains, then
  `set_hand_traj_cmd(traj_type=1)` once per ball (catch_coordinator_node.py:221–271).
  Hand profiles execute **on the Platform Teensy** (arm-and-forget; firmware
  traj_type enumeration 0=throw, 1=catch, 2=full — Teensy_code.ino:515–517), carried
  as an opaque payload with an absolute wall-clock deadline over CAN 0x6D0.
  **The current hand firmware + config has produced smooth real catches; it is
  close to well-tuned.** Catch-smoothness work therefore targets sim fidelity,
  not the hardware hand path (see Phase 6).
- **Prior art in `~/Desktop/Jugglebot-bb`** (branch `demo/bb-led-two-ball-juggle`):
  sim-validated catch **geometry** — slow translate-to-reach (reliable to ~60–80 mm),
  tilt-to-receive with the cup axis collinear to the arrival velocity (≤12° usable,
  lever arm 1.66 mm/deg of cup travel per degree of tilt), tilt **ramped through the seat** (a parked tilted rim
  deflects the ball), constant-deceleration hand seat (a min-jerk seat ejects the
  ball via its mid-stroke acceleration peak) — plus tilt-aimed throws and a
  self-catch loop. **Caveat**: those sims' claimed smoothness is not real — the
  simulated hand fails to "meet" the ball and wanders during the hold phase, and
  the sims used continuous sub-tick velocity-matched hand commands that do not
  exist on hardware. Prior sim results are treated as geometry-validated only.
- The n-ball juggling planner (Ploeger et al.,
  `~/Desktop/kinematic_planning_for_nball_toss_juggling`) contributes boundary-
  condition formulations, not machinery: catch position at touchdown + catch
  velocity collinear with the ball; throw position + exact ballistic launch
  velocity `v = (p_tgt − p_to)/T − ½·g·T` + cup acceleration = gravity at release.
  These lift into numpy quintics directly; the CasADi NLP is not ported.

---

## Architecture

### The command seam

```
trajectory_node (NEW, 40 Hz emitter thread)
   │  make_mpc_command dicts — ZMQ PUB bound on tcp://127.0.0.1:5557, topic 'mpccmd'
   ▼
teensy_bridge_node::_MpcCommandSetpointSource  (UNCHANGED, teensy_bridge_node.py:214)
   ▼  SetpointPump.build()  (UNCHANGED — mm→rev knots, per-step 0.3 rev gate)
UDP SETPOINT :5005 → can-hub Teensy 500 Hz Hermite → CAN3 → 6 leg ODrives
```

Envelope requirements (verified against `HardwarePlant` and the pump):

- The PUB socket **binds** :5557 (`HardwarePlant` binds today — hardware_plant.py:272);
  the bridge SUB connects. The new publisher must be the **sole binder**; a bind
  failure is fatal at startup with an explicit "is run_mpc.py running?" message.
  This doubles as the MPC/trajectory mutual-exclusion interlock.
- Wire format re-uses `motion/ipc.py::_pack` + `_ndarray_default` (ipc.py:405) so
  frames are byte-compatible with what `HardwarePlant` sends.
- `vel_mm_s` is **mandatory and finite** (the pump SAFETY-rejects frames without
  it). `motor_rev = ext_mm × GEOM_MM_TO_REV` (no extra offsets —
  hardware_plant.py:413). `cmd_next_mm`/`cmd_next2_mm` populate the u1/u2 knots
  (firmware Mode-1 Hermite with C1 endpoint velocity).
- Firmware interlocks (verified): `MPC_STALE` arms only after a first command AND
  `mpc_active` (fault_machine.cpp:338); `MAX_DEVIATION` (0.5 rev) requires
  `mpc_active` + a latched frame; ACTIVATE/DEACTIVATE RPCs are **rejected while
  `mpc_active=1`** (leg_activate.cpp:112) — the firmware itself enforces
  disarm-before-deactivate, loudly.

### New pure-Python package: `ros_ws/src/jugglebot/jugglebot/motion/trajectory/`

Python 3.8, `from __future__ import annotations`, numpy-only, no ROS imports,
no repo-root imports.

| Module | Contents |
|---|---|
| `quintic.py` | **Copies** of `controller/hermite.py` interp functions and `controller/feasibility.py` peak-bound functions. Copied, not imported: keeps `motion/` free of repo-root path dependence and leaves the dormant `controller/` untouched. |
| `limits.py` | `TrajectoryLimits` dataclass: leg vel/acc/jerk limits, pose box (xy extent, z range, max tilt), `knot_dt_s = 0.025` (fixed). Constructed from the new `JB_TRAJ_*` config constants. |
| `segment.py` | `QuinticSegment` — one quintic per pose axis `[x, y, z, rx, ry, rz]` (mm; rotvec rad; z STOW-relative, 170.0 = ACTIVE) with pos/vel/acc boundary conditions at both ends. `eval(t) → (pose, twist, accel)`, clamped past both ends. |
| `plan.py` | `TrajectoryPlan` — immutable ordered segments + implicit terminal hold (past the final segment: final pose, zero twist/accel). `state_at(t)` for replan seeding. `HoldPlan(pose)` degenerate case. |
| `planner.py` | Plan constructors: `build_move(state0, target_pose, duration_s\|None, limits, geom)`, `build_timed(state0, target_pose, target_twist, t_arrival, hold_after, …)`, `build_catch(…)` (Phase 6). **Every constructor validates before returning**; failures raise `TrajectoryInfeasible(code, reasons, min_duration_s)`. |
| `feasibility.py` | **The single canonical gate** (below). |
| `shaping.py` | Lean-into-translation heuristic (below). |
| `emitter.py` | `KnotEmitter` — samples the active plan at `(t, t+0.025, t+0.050)`, runs the IK chain, and assembles the `make_mpc_command` field set. |
| `follower.py` | Streaming-target follower for spacemouse/GUI (Phase 3). |
| `tilt_geometry.py` | Port of `Jugglebot-bb/sim/juggle_tilt.py` core math: `tilt_to_receive()` (cup axis collinear with arrival velocity), the tilt lever arm (`LEVER_ARM_MM_PER_DEG = 1.66`, positive, juggle_tilt.py:73 — **port the derivation from the cup height above the tilt centre (`CUP_TILT_CENTER_Z_MM`), not the bare constant**, and define the compensation sign convention explicitly at port time), `MAX_TILT_DEG = 12.0` (Phase 6). |
| `ballistics_bc.py` | numpy ports of the touchdown quadratic and launch-velocity boundary conditions (Phase 6+; check overlap with `controller/ballistics.py` first and reuse where identical). |

**Why quintic-per-axis in pose space**: (a) the basis and closed-form peak bounds
are already validated in this codebase on hardware; (b) matching pos/vel/acc at
both ends gives C2 continuity at every segment join and every replan by
construction, which is what bounds leg jerk (the binding constraint); (c) rotvec
axes match the `pose_6dof` convention everywhere. Known approximation:
rotvec-rate ≠ angular velocity at large tilt — acceptable for ≤12° MVP tilts, and
the gate measures true sampled leg velocities anyway. Trapezoids were rejected
(acceleration steps ⇒ unbounded jerk); septic polynomials rejected (more knobs,
no validated code, unnecessary once the gate stretches duration).

### The feasibility gate (`feasibility.py::validate`)

One canonical enforcement point; all entry points (go-to-pose, timed target,
spacemouse, `catch/dynamic_target`, reload, juggle coordinators) construct plans
exclusively through `planner.py`, so nothing bypasses it.

1. **Reachability/workspace/stroke** along the sampled path (200 samples/segment):
   `workspace.check_workspace_limits` / `check_reachability`, leg extensions within
   stroke margins.
2. **Leg kinematic peaks**: leg pos/vel/acc via the `ik_solver` Jacobian chain,
   jerk via third difference — against `TrajectoryLimits`.
3. **Timing**: `T ≥ min_duration`. Rest-to-rest quintic pre-size:
   `peak_vel = 1.875·|Δp|/T`, `peak_acc ≈ 5.77·|Δp|/T²`, `peak_jerk = 60·|Δp|/T³`
   per axis; on leg-space violation with worst ratio r,
   `T ← T · max(r_vel, √r_acc, ∛r_jerk) · 1.05` and re-verify (converges in 1–2
   iterations). A timed target with less lead than the converged T is rejected
   **with `min_duration_s` in the response**.
4. **Knot-step bound**: max per-knot |Δu0| < `JB_OP_MAX_POSITION_STEP_REV` (0.3 rev)
   with 20% margin — rejection happens before motion, not mid-stream at the pump.
5. Output: `FeasibilityReport` — `ok`, machine `code` (`OK / UNREACHABLE /
   WORKSPACE / LIMIT_VEL / LIMIT_ACC / LIMIT_JERK / TOO_FAST / STEP_BOUND /
   STALE_STATE / WRONG_MODE`), human `reasons[]`, `min_duration_s`, measured peaks.

**Loud surfacing per entry point**: services return `accepted=false` + code +
message + `min_duration_s` AND log at ERROR; spacemouse rejections log throttled
(1 Hz) and keep the last valid plan; `catch/dynamic_target` rejections publish on
`trajectory/target_feedback`, which `catch_coordinator_node` consumes in place of
its ZMQ :5559 `TargetFeedbackSub` (blacklist semantics preserved,
catch_coordinator.py:162–190). The emitter carries one defence-in-depth assertion
(step bound); if it ever fires the node freezes to hold and logs ERROR.

### Kinetics-aware shaping (one mechanism, opt-in)

For a translation segment with planned lateral acceleration `a_xy(t)` (analytic
from the quintic), superpose tilt `rx(t) = −k·a_y(t)/g`, `ry(t) = +k·a_x(t)/g`,
gain `k = JB_TRAJ_LEAN_GAIN ∈ [0,1]` (default **0.0**), tilt capped at 5°.
Quintic boundary accelerations are zero, so the added tilt vanishes smoothly at
segment ends — continuity is preserved by construction. The cup's lateral offset
under tilt is compensated by subtracting the tilt-induced cup travel
(lever arm 1.66 mm/deg per `juggle_tilt.py:73`; cup-height-dependent — see the
`tilt_geometry.py` port note) from the xy translation. Shaping runs **before**
`validate()` so the gate always sees the shaped plan. The always-on smoothness
mechanism is the gate's duration stretch; lean is an A/B'd refinement (Phase 4),
not a dependency.

### `trajectory_node.py` (thin ROS wrapper)

- **Emitter thread** (not an rclpy timer; mirrors `_setpoint_loop`'s
  dedicated-thread pattern): absolute-deadline 40 Hz loop on `perf_counter`; the
  ZMQ PUB is created inside the thread. Always has a plan — `HoldPlan` when idle —
  so stream gaps never approach the 250 ms staleness E-STOP (10× margin).
- **Plan installation** is an atomic reference swap; every new plan is built from
  `old_plan.state_at(t_install + ε)`, so pose/twist/accel are continuous (C2)
  across swaps, including operator aborts.
- Subscriptions:
  - `control_mode_topic` — stream gating. On stream start, seed the hold pose from
    **measured** `robot_state` telemetry (pos_estimate rev → ext mm → FK via
    `leg_lengths_to_pose`), never from nominal — this is what keeps the first u0
    inside both the pump's 0.3 rev step gate and the firmware's 0.5 rev
    MAX_DEVIATION. Entering STANDBY from another sub-mode triggers a profiled
    return to neutral `(0, 0, 170, 0, 0, 0)` then hold.
  - `robot_state` — seeding + staleness monitor (plan requests rejected
    `STALE_STATE` if telemetry older than 0.5 s).
  - `platform_pose_topic` — SPACEMOUSE/GUI/SHELL gating by `msg.publisher`
    (ported from mpc_bridge_node.py:171–200).
  - `catch/dynamic_target` — CATCH mode; the perf_counter time-domain contract is
    kept, with one conversion point in this node.
  - `gravity_offset` — levelling correction composed into outgoing orientations
    (ported verbatim from mpc_bridge_node.py:143–165).
- Services (new files in `ros_ws/src/jugglebot_interfaces/srv/` + CMakeLists):
  - `trajectory/go_to_pose` — **GoToPose.srv**: `geometry_msgs/Pose pose`
    (mm, z STOW-relative), `float64 duration_s` (0 ⇒ minimal feasible) →
    `bool accepted, string code, string message, float64 planned_duration_s,
    float64 min_duration_s` (`code` is a **string**, the feasibility/service code
    verbatim — changed from `int32`; see the Phase 2 Outcome). TRAJECTORY mode
    only (else `WRONG_MODE`, loud).
  - `trajectory/timed_target` — **TimedTarget.srv**: pose +
    `geometry_msgs/Vector3 velocity_mm_s` + `builtin_interfaces/Time arrival_time`
    + `bool hold_after` → same response shape (`string code` included).
  - `trajectory/hold`, `trajectory/go_home` — `std_srvs/Trigger`.
  - `trajectory/set_limits` — **SetTrajectoryLimits.srv**: leg vel/acc/jerk
    (0 ⇒ keep), clamped to the YAML hard ceilings → success/message. The
    in-session ramp-up knob.
- Publications: `trajectory/status` (**TrajectoryStatus.msg**: streaming, mode,
  plan_kind, plan_time_remaining_s, seq, last_rejection; 5 Hz — *Phase 1 shipped
  this as `diagnostic_msgs/DiagnosticStatus`; the typed msg + migration land in
  Phase 2, see the Phase 1 Outcome*),
  `trajectory/target_feedback` (accepted/code/reason/arrival_time/source),
  `trajectory/diagnostics` (active-plan leg peaks + measured emitter jitter).
  All three join the rosbag record list in `jugglebot_launch.py`.

### Orchestrator integration

Add `ActiveMode.TRAJECTORY = 'TRAJECTORY'` (state_machine.py:34) and accept a
`'trajectory'` command in ACTIVE (state_machine.py:416). STANDBY's docstring
updates: no longer "silence everything for a manually-launched run_mpc.py" but
"hold at neutral via trajectory_node".

### Runtime arming — `set_setpoint_output` (fixes the arm-before-stream trap)

New `std_srvs/SetBool` service on `teensy_bridge_node`. On `true`, require:
(a) Teensy link up; (b) a fresh `mpccmd` frame observed on :5557 within 0.5 s;
(c) that frame's u0 within **0.25 rev** of every leg's live `pos_estimate` (half
the firmware backstop). Then `_start_setpoint_output()` →
`_set_mpc_active(True)` — stream-then-arm, the pattern validated in
`tests/hardware/teensy_guard_validation.py`. On `false`: stop the setpoint
thread, `_set_mpc_active(False)`. The response message carries the reject
reason. The `enable_setpoint_output` launch parameter remains for bench use;
the documented production flow is the service.

Operator sequences (also documented in the ops doc when Phase 1 lands):

- **Arm**: orchestrator `activate` (TRAP_TRAJ park at ~ext 154.5 mm) → mode
  STANDBY (trajectory_node seeds from telemetry and streams hold) →
  `ros2 service call /set_setpoint_output std_srvs/srv/SetBool "{data: true}"`
  → verify `mpc_active=1` and zero motion.
- **Disarm**: `trajectory/go_home` → `set_setpoint_output false` → orchestrator
  `deactivate` (the firmware rejects DEACTIVATE while armed, enforcing order).

### Configuration (`config/hardware_config.yaml` → `generate_config.py` → `JB_TRAJ_*`)

```yaml
trajectory_op:
  leg_vel_limit_mmps: 100.0        # session-ramped; hard ceiling 280 (= 4.0 rev/s)
  leg_acc_limit_mmps2: 400.0       # deliberately very low for first sessions
  leg_jerk_limit_mmps3: 8000.0     # the binding constraint; ramped by feel
  leg_vel_ceiling_mmps: 280.0
  leg_acc_ceiling_mmps2: 4000.0
  leg_jerk_ceiling_mmps3: 200000.0
  min_move_duration_s: 0.20
  min_timed_lead_s: 0.25
  spacemouse_horizon_s: 0.35
  lean_gain: 0.0
  catch_reach_freeze_s: 0.30
  catch_settle_hold_s: 0.50
```

### Reload sequence (goal 4)

New **`action/Reload.action`** in `jugglebot_interfaces`:

```
float64 throw_delay_s          # 0 => default 3.0 s (>= BB's ~2.5 s lead floor)
---
bool success
string outcome                 # CAUGHT | REJECTED_<code> | ABORTED_<code> | MISSED | MISSED_<code>
float64 catch_error_mm         # mocap miss distance (NaN if unknown)
---
string phase                   # CHECKING | AIMING | THROW_PENDING | BALL_IN_FLIGHT | CATCHING | SETTLING
```

Served by a new thin `reload_coordinator_node.py` wrapping a pure-Python
`reload_sequencer.py` FSM (unit-testable). The coordinator **orchestrates only**:
`trajectory_node` plans all platform motion; `catch_coordinator_node` arms the
hand (existing behaviour reused unchanged).

1. **Preconditions (loud rejects)**: orchestrator ACTIVE with control mode CATCH
   (set by the operator; the action does not switch modes); `bb/heartbeat`
   connected ∧ IDLE; if `ball_in_hand == false`, call `bb/reload` and await the
   heartbeat `RELOADING → IDLE` + `ball_in_hand == true` (10 s timeout →
   `REJECTED_NO_BALL`); mocap fresh (`/rigid_body_poses` < 0.5 s); trajectory
   streaming on (from `trajectory/status`).
2. **Aim + throw**: `bb/throw_at_target` extended with optional
   `geometry_msgs/Point target_point_global_mm` + `bool use_target_point`
   (skips the QTM rigid-body lookup; default-zero fields keep existing callers
   working). Catch point = `(0, 0, GEOM_INITIAL_HEIGHT_MM + landing_z_offset)`
   in the world frame. **Phase 7a verifies the QTM-world vs jugglebot-base frame
   convention with an aim-only (speed = 0) command before any ball flies.**
   `throw_delay_s ≥ 2.5`; BB rejections surface as `REJECTED_BB(<message>)`.
3. **Announcement → catch**: BB publishes `ThrowAnnouncement`;
   `catch_correlation_node` tags the tracked ball; `catch_coordinator_node`
   (CATCH mode) emits `catch/dynamic_target` and arms the hand — the existing
   path, unchanged. No announcement within `throw_delay + 0.5 s` ⇒
   `ABORTED_NO_ANNOUNCEMENT` (platform holds; hand never armed).
4. **Platform catch trajectory** (`planner.build_catch`, sim-gated in Phase 6):
   reach translation (target ≤ 80 mm — the sim-established reliable envelope)
   starts immediately and **freezes** at `t_arrival − catch_reach_freeze_s`
   (later target updates inside the freeze window are ignored except aborts);
   tilt-to-receive (≤ 12°, cup axis collinear with arrival velocity) **ramps
   through the seat** with a small residual tilt rate at `t_arrival` decayed to
   zero over the following 0.15 s; then a **literal quiescent hold segment**
   (zero twist, `catch_settle_hold_s`); then a slow return to neutral. Platform
   translation velocity at contact is zero — velocity matching is the hand's job.
5. **Confirmation**: ball status `CAUGHT` from the tracker (tracking/ball.py)
   within 0.7 s of predicted landing, cross-checked against hand telemetry;
   `catch_error_mm` from the mocap rest position. Otherwise `MISSED`.
6. **Aborts**: BB reject → clean reject; announcement timeout → hold;
   post-release infeasible catch target → platform holds last valid pose, hand
   fires per its armed schedule, outcome `MISSED_INFEASIBLE` (with the gate
   code); operator abort = mode switch away from CATCH (C2 hold replan, no snap).

### Hand-catch smoothness: a sim-fidelity work item (Phase 6)

**Premise (ground truth from hardware)**: real catches with the current Platform
Teensy generator and config have been smooth; the hardware arm-and-forget path is
the proven reference. The Teensy catch profile is velocity-matched by design
(accelerate to −0.9·v_ball, ~10%-stroke constant-velocity hold centred on
arrival, constant deceleration — mirrored in `sim/hand/trajectory.py`).
The Jugglebot-bb sims are the janky side: their hand model didn't meet the ball
and wandered during hold, and their control technique (continuous sub-tick
velocity-matched commands) doesn't exist on hardware.

Work item, reframed accordingly:

- Build the Phase-6 sim harness around the **arm-and-forget Python mirror** of
  the Teensy generator with realistic arm latency — make the sim reproduce the
  hardware-proven behaviour, rather than tuning hardware to match a janky sim.
- Robustness sweeps: arm-time error ±30 ms, `event_vel` error ±10% — establish
  margins, not new configs.
- **Acceptance criteria** (measured in sim, re-measured on hardware in Phase 7):
  |v_hand − v_ball| at first contact ≤ 15% of |v_ball|; no ball–cup separation
  > 10 ms after first contact; hold-phase platform centroid travel < 5 mm and
  tilt change < 1° from `t_arrival + 0.15 s` until the return move; ≥ 18/20
  catches under the noise model.
- Hardware-side changes (CATCH_VEL_RATIO / hold-pct config, re-arm policy) are
  **last resort** and require hardware evidence, not sim evidence alone. The slim
  `makeCatch()` firmware parameterisation fallback is retained as a pre-scoped
  option but is expected to be unnecessary; invoking it is a stop-and-decide
  point, not a default.

### Sim strategy and the Jugglebot-bb port

Port into main from `demo/bb-led-two-ball-juggle`:

- `sim/juggle_tilt.py`, `juggle_noise.py`, `juggle_catch.py`, `juggle_bb_catch.py`
  (interactive BB→Jugglebot single-catch demo), `juggle_throw.py`,
  `juggle_selfcatch.py`, `juggle_online.py`, plus a **diff audit** of the six
  diverged support files the juggle sims depend on (`sim/plant/mujoco_plant.py`,
  `sim/model/jugglebot.xml`, `sim/ball/manager.py` + `__init__.py`,
  `sim/ball_butler/sim.py`, `sim/hand/trajectory.py`, `sim/main.py`) — carry over
  only juggle-required changes, reconciled with main-side changes since the
  2026-05-22 branch point.
- `controller/demo/` is **not** ported wholesale: `player.py`'s lookahead/IK
  logic is absorbed into `motion/trajectory/emitter.py`; `timeline.py`'s
  absolute-wall-time event scheduling into `reload_sequencer.py`;
  `juggle_planner.py`'s boundary-condition math into `ballistics_bc.py`;
  `juggle_optimizer.py` (offline CasADi) is not ported.
- **Production-in-the-loop rule**: sim gates drive the **actual**
  `motion.trajectory` planner + emitter (knots → sim plant), with the
  arm-and-forget hand mirror; every emitted knot must pass a real `SetpointPump`
  instance inside the harness. This is what makes a sim pass transfer to
  hardware.

Sim-gate definitions (a juggle behaviour may not go to hardware before its gate
passes; results recorded in the logbook with seeds and configs):

| Gate | Criteria |
|---|---|
| **Reload** (Phase 6) | ≥ 18/20 catches under noise (2% BB speed, 0.5 mm tracking) across 5 landing offsets within ±60 mm xy and arrival speeds 2.5–4.0 m/s; hand-contact + hold-quiescence criteria above; zero feasibility violations in accepted runs; all knots pump-accepted. |
| **Self-toss** (Phase 8) | ≥ 9/10 throw (~2–3 m/s) + catch cycles with arm-and-forget traj_type=0 throw and traj_type=1 catch; same contact/hold criteria. |
| **Two-ball** (Phase 9) | ≥ 5 consecutive cycles under noise. |

---

## Implementation Phase Summary

| Phase | Title | Sim gate | Hardware | Status |
|---|---|---|---|---|
| 1 | Streaming foundation (hold via new path) | — | arm + 120 s hold | CODE COMPLETE (hardware deferred) |
| 2 | Waypoint moves at low limits | — | move battery + loud rejection | CODE COMPLETE (hardware deferred) |
| 3 | SpaceMouse streaming | — | manual flight | CODE COMPLETE (hardware deferred) |
| 4 | Limit ramp-up + lean A/B | — | multiple short sessions | CODE COMPLETE (hardware deferred) |
| 5 | Timed target states | — | timed moves ±25 ms | CODE COMPLETE (hardware deferred) |
| 6 | Sim port + catch trajectory + hand-model fidelity | Reload gate | none | NOT STARTED |
| 7 | Reload on hardware (action) | — | 7a aim-only / 7b static catch / 7c full | NOT STARTED |
| 8 | *(stretch)* Single-ball self-toss | Self-toss gate | staged | NOT STARTED |
| 9 | *(extra stretch)* Two-ball juggling | Two-ball gate | staged | NOT STARTED |

Every phase ends with: full `pytest tests/ -q` green, a logbook entry
(`/log feature mvp-phaseN-<slug>`), commit(s) with `Logbook-Entry:` trailers,
push. Hardware sessions follow the operator protocol: Harrison runs all
robot-actuating commands; the implementing session preps exact commands with
PASS/ABORT criteria and verifies read-only.

---

## Implementation Phases

### Phase 1 — Streaming foundation *(deliberately small)*

**Goal**: `trajectory_node` streams hold frames on :5557; the bridge arms at
runtime; the platform holds the ACTIVE pose through the new path; disarm and
deactivate are clean.

**Code**:
- `motion/trajectory/{__init__,quintic,limits,segment,plan,emitter}.py` with a
  minimal gate (stroke + workspace + vel/acc caps — trivial for hold; the full
  gate lands in Phase 2). `motion/ipc.py::MpcCommandPub`.
- `jugglebot/trajectory_node.py`: telemetry seeding, mode-gated streaming,
  `trajectory/status`, `trajectory/hold`, `trajectory/go_home`. No move services.
- `teensy_bridge_node.py`: `set_setpoint_output` (SetBool) with the arming
  preconditions.
- `jugglebot_launch.py`: add trajectory_node, remove mpc_bridge_node, extend the
  rosbag list; `setup.py` entry point; `hardware_config.yaml` `trajectory_op:`
  section + a new `HW_SECTIONS` row in `config/generate_config.py` (e.g.
  `("trajectory_op", "JB_TRAJ_", …)` — without it a regenerate silently emits no
  `JB_TRAJ_*` constants) + regenerate.

**Tests** (`tests/motion/`, `tests/ros/`): emitter golden-parity vs
`make_mpc_command` (byte-compatible via `_pack`); a real
`SetpointPump(GEOM_MM_TO_REV)` accepts every emitted frame; seed FK roundtrip
(`JB_OP_ACTIVATE_POSITION_REVS` → pose ≈ (0,0,170)); arming service vs FakeTeensy
(reject on no-stream; reject on u0-vs-encoder > 0.25 rev; arm and disarm paths) —
extends `tests/ros/test_teensy_bridge_node_setpoint.py`; emitter cadence
(40 ± 2 Hz, max gap < 100 ms over 10 s).

**Hardware session** (protocol file `tests/hardware/session_phase1_hold.md` +
read-only probe `tools/probes/traj_stream_probe.py` — SUB on :5557 printing
rate/u0/steps):
1. Launch with `enable_setpoint_output:=false`; home; activate; mode STANDBY.
   Probe shows 40 Hz hold frames with u0 ≈ activate revs.
2. `set_setpoint_output true` → PASS: success response, `mpc_active=1`, **zero
   motion at arm**, steady tracking. ABORT: any E-STOP (MPC_STALE /
   MAX_DEVIATION), any visible motion, pump-reject spam in the bridge log.
3. Hold 120 s → PASS: no rejects, no faults, leg drift < 0.02 rev.
4. `trajectory/go_home` (no-op from hold) → disarm → deactivate → shutdown.
   PASS: firmware accepts DEACTIVATE (proves `mpc_active` cleared).

**Exit**: platform holds via the new path; clean disarm/deactivate.
**Logbook**: `mvp-phase1-streaming-foundation`.

**Outcome (2026-07-07 — CODE COMPLETE, hardware deferred)**: All Phase 1 software
landed on branch `mvp-trajectory-bringup` (commits `63031c3` config, `aab9811`
package+ipc, `337eb41` node+launch, `33da615` bridge arming, `d09846e` probe+docs). New pure package
`jugglebot.motion.trajectory` (`quintic`/`limits`/`segment`/`plan`/`feasibility`/
`planner`/`emitter`), `motion/ipc.MpcCommandPub` (sole-binder :5557), the thin
`trajectory_node` (40 Hz emitter thread, telemetry-seeded hold, `trajectory/hold`
+ `trajectory/go_home`, `trajectory/status` as a `DiagnosticStatus`), the bridge
`set_setpoint_output` (`SetBool`) runtime-arming service (link + fresh-stream +
0.25 rev u0-vs-encoder preconditions), `JB_TRAJ_*` config (+ `HW_SECTIONS` row),
and the launch cutover (drop `mpc_bridge_node`, add `trajectory_node`). The
load-bearing invariant is tested directly — every emitter frame is accepted by a
real `SetpointPump`. Verification: `pytest tests/ -q` (2026-07-07) = **1993 passed,
1 xfailed in 625.44 s** (baseline 1956/1; net +37 = the new tests only, no
regressions). Codegen deterministic (regenerate → clean tree). **Deferred to the
operator bench session**: `tests/hardware/session_phase1_hold.md` (arm at ACTIVE +
120 s hold + clean disarm→deactivate) with the read-only probe
`tools/probes/traj_stream_probe.py`. Full narrative in
`logbook/2026-07-07-mvp-phase1-streaming-foundation.md`.

### Phase 2 — Waypoint moves at very low limits

**Goal**: `trajectory/go_to_pose` executes profiled moves; infeasible requests
are loudly rejected.

**Code**: `planner.build_move` + the full `feasibility.py` gate (all checks +
duration-stretch loop); `GoToPose.srv`, `SetTrajectoryLimits.srv`,
`TrajectoryStatus.msg`; `ActiveMode.TRAJECTORY` + orchestrator `'trajectory'`
command; node handlers + `trajectory/set_limits` + `trajectory/diagnostics`.

**Tests**: boundary-condition exactness (pose/twist/accel at both ends); C2
continuity across install-time replans; closed-form peak bounds vs dense
sampling (property test); duration-stretch convergence; a rejection matrix
(every gate code reachable, messages populated); WRONG_MODE; step-bound margin
at ceiling limits.

**Hardware session**: limits at defaults (100 mm/s, 400 mm/s², 8000 mm/s³).
Battery: z 170→190→170; x ±20; y ±20; tilt rx ±3°; then one deliberately
infeasible request (`duration_s: 0.05`) → PASS: rejected `TOO_FAST` with
`min_duration_s`, zero motion. PASS per move: subjectively smooth (no audible
snap), `/diagnose --latest` shows leg jerk within limits, no pump rejects, no
E-STOP. ABORT: oscillation, gate violation, tracking error > 0.1 rev.

**Exit**: 10/10 scripted moves clean + one demonstrated loud rejection.

**Outcome (2026-07-07 — CODE COMPLETE, hardware deferred)**: All Phase 2 software
landed on branch `mvp-trajectory-bringup` (commits `614820c` motion gate +
`build_move`; `1dc9571` interfaces + node + orchestrator). The full
`feasibility.validate` gate (geometry/workspace/condition → leg vel/acc/jerk peaks →
per-knot step bound, all peaks measured in one pass) + `planner.build_move` with the
duration-stretch loop (minimal-feasible, or a loud `TOO_FAST` with `min_duration_s`
for a too-tight request; spatial `WORKSPACE`/`UNREACHABLE` re-raised immediately).
New interfaces `GoToPose.srv` (response `code` is the feasibility **string** enum, not
the plan's `int32` — see the logbook Discussion), `SetTrajectoryLimits.srv` (clamped
ramp), and the typed `TrajectoryStatus.msg` (migrated off the Phase-1
`DiagnosticStatus` stand-in); `trajectory/go_to_pose` (TRAJECTORY-mode-only, else
`WRONG_MODE`) + `trajectory/set_limits` + `trajectory/diagnostics`;
`ActiveMode.TRAJECTORY` + the `'trajectory'` orchestrator command. Verification:
`pytest tests/ -q` (2026-07-07) = **2041 passed, 1 xfailed in 500.17 s** (baseline
1996/1; net +45 = the new tests only, no regressions); `colcon build
--packages-select jugglebot_interfaces jugglebot` (2026-07-07) = 2 packages finished,
0 errors; codegen unchanged (no YAML edit this phase). **Deferred to the operator
bench session**: the waypoint move battery (z 170→190→170, x/y ±20, tilt rx ±3°) +
one `duration_s: 0.05` loud-rejection demo at the default low limits (100 mm/s,
400 mm/s², 8000 mm/s³), using the Phase-1 `tools/probes/traj_stream_probe.py` for
read-only knot inspection. Full narrative in
`logbook/2026-07-07-mvp-phase2-waypoint-moves.md`.

**Outcome addendum (2026-07-07 — audit-fix round)**: a `/audit` of the two Phase-2
commits found one BLOCKING issue — the mid-move `go_to_pose` install step. Because
the seed is sampled at service entry but the plan installs ~1.5 s later (4–5
`validate` passes at ~377 ms each) while the emitter streams the OLD plan, an
in-flight move meant the install jumped `u0` back to the stale seed (measured
~575 mm/s transient that passed both step gates). Fixes: (a) a **permanent
install-continuity guard** — re-sample the commanded state immediately before
install and reject `STALE_STATE` if the plan's t=0 leg positions drifted past
`0.25·STEP_BOUND_MARGIN·JB_OP_MAX_POSITION_STEP_REV` (≈0.06 rev); (b) a **temporary,
documented Phase-2 `BUSY` restriction** — moves are accepted only from a hold
(interrupting an in-flight move needs the follower's C2 supersede, which lands in
**Phase 3/5**, at which point this restriction is lifted); (c) **validate-perf quick
wins** — `build_move` returns its accepting report (node drops a redundant
re-validate), validates an explicitly-requested duration FIRST (accept immediately
if the gate passes — this also removes a TOO_FAST false-reject for durations below
the 1.05-margin-padded `t_min`), and hoists the once-per-geometry `WorkspaceLimits`
construction out of `validate()`. Also fixed: `go_to_pose` now rejects on stale
telemetry and on a non-finite `duration_s`; leaving TRAJECTORY mid-move for another
streaming mode installs a profiled decel-stop. Verification: `pytest tests/ -q`
(2026-07-07) = **2047 passed, 1 xfailed** (net +6 audit-fix tests). Narrative +
per-finding trace in the logbook entry's "Audit fixes (2026-07-07)" section.

### Phase 3 — SpaceMouse streaming

**Goal**: continuous target following through the same layer.

**Prerequisite (from the Phase-2 audit)**: the SpaceMouse follower replans every
40 Hz tick, so `feasibility.validate()` must drop from ~377 ms (measured 2026-07-07
on the Jetson at the 200-sample gate) to low-single-digit ms for the follower's
scoped checks. Vectorise the per-sample IK/Jacobian sampling chain, decimate or skip
the per-sample condition-number SVD, and/or run a follower-specific reduced gate —
the exact design decision belongs to this phase. (The Phase-2 hoist of
`WorkspaceLimits` out of `validate()` is a first step but nowhere near sufficient.)

**Code**: `follower.py` (drain-to-latest per 40 Hz tick; replan from the current
commanded state toward the newest target with horizon
`max(min_feasible, JB_TRAJ_SPACEMOUSE_HORIZON_S)`; 0.5 mm / 0.1° deadband);
trajectory_node: `platform_pose_topic` publisher-field gating + gravity-offset
composition (verbatim ports from mpc_bridge_node); saturation policy — a target
outside the workspace clamps to nearest-valid with a throttled WARN.

**Tests**: property test — random 100 Hz target streams produce knot sequences
with bounded discrete vel/acc/jerk and C2 at every replan boundary; step
response settles within the horizon; deadband behaviour; mode-gating both ways.

**Hardware session**: mode `spacemouse`; gentle flight at low limits; a
hard-shove saturation test; a mid-flight spacemouse unplug (the existing handler
publishes an ACTIVE-pose hold on disconnect — verify a smooth return). PASS:
subjectively smooth throughout, no rejects, clean disconnect. ABORT: any jerk
event, E-STOP, runaway.

**Outcome (2026-07-08 — CODE COMPLETE, hardware deferred)**: All Phase 3 software
landed on branch `mvp-trajectory-bringup` (commits `5cac69b` motion layer +
`4bdc688` ROS surface; the docs commit that follows). The two orchestration prerequisites from the
Phase-2 audit are met. (1) **Fast follower gate** `feasibility.validate_follow` —
profiling showed the analytic gate's ~377 ms is dominated by the per-sample analytic
Jacobian chain (~90 %), NOT the condition SVD (~9 %), so the fast gate sidesteps the
Jacobian entirely: vectorised pose→leg-extension sampling (`QuinticSegment.eval_pose_batch`
+ batched IK, verified equal to the scalar chain to 1e-9) with leg vel/acc/jerk by
finite difference and a decimated condition SVD. **~1.5–4 ms**; vel/acc/step match the
analytic gate to < 0.1 %, jerk is held **strictly conservative** (≤ 1.5 % under-measure
at 300 samples, then ×1.05); the knot-step-bound is **bit-identical** so pump-acceptance
is preserved. (2) **Duration-stretched graceful stop** `planner.build_graceful_stop` —
a decel-in-place that converges for gate-limited seeds by lengthening the horizon,
*except* a near-boundary seed whose margin is under the decel overshoot (~0.2·v·T),
which fails the stroke check spatially (audit-corrected — the node retries from the
decaying live state via a pending-stop flag rather than latching failure); replaces the
Phase-2 STANDBY-exit catch-and-complete fallback and backs the follower's input-loss
handling. New pure `TargetFollower` (clamp → deadband → replan / keep-last-plan)
+ `planner.build_follow`; `trajectory_node` gains `platform_pose_topic` publisher-field
mode gating + `gravity_offset` composition (verbatim ports from `mpc_bridge_node`), the
follower replan run inside the 40 Hz emitter tick (drain-to-latest), input-loss stop,
and a ray-clamp saturation policy (clamp to the nearest reachable pose along the
current→target ray — uses the EXISTING stroke envelope, invents no new limit; throttled
WARN). No new interfaces/config/launch → no colcon or codegen gate this phase.
Verification: `pytest tests/ -q` (2026-07-08) = **2079 passed, 1 xfailed in 502.33 s**
(baseline 2047/1 at `f38153f`; net +32 = the new tests only: 23 follower-motion + 9
node, no regressions). **Deferred to the operator bench session**: the manual-flight
battery (gentle fly / saturation shove / mid-flight unplug) at the default low limits,
using the Phase-1 `tools/probes/traj_stream_probe.py` for read-only knot inspection.
Full narrative in `logbook/2026-07-08-mvp-phase3-spacemouse-streaming.md`.

**Outcome addendum (2026-07-08 — audit-fix round)**: a `/audit` of the Phase-3 range
raised 4 WARNINGs + 6 NOTEs, all applied in a follow-up commit. Highlights: (a)
**TOCTOU** — the follower's plan install is now conditional under `_plan_lock` on the
mode still being a follower mode, so a concurrent `STANDBY`-exit graceful stop is no
longer clobbered by a stale follow plan; (b) **`validate_follow` now gates zero-segment
`HoldPlan`s** (degenerate branch) and `build_graceful_stop`'s at-rest path gates its
`HoldPlan` — an ungated out-of-stroke hold can no longer reach the emitter; (c) the
graceful stop is **not** unconditionally always-valid (a near-boundary outward seed
overshoots the stroke edge), so both failure fallbacks are replaced by a **pending-stop
retry** that retries from the decaying live state instead of latching failure; (d)
input-loss no longer fires instantly on follower-mode entry (grace window); (e)
sustained-reject re-plan spam is skipped via a rejected-target deadband; (f) per-replan
`t0` matches the seed sample. Full detail + the deferred *retargeted*-stop structural
fix in the logbook entry's "Audit fixes (2026-07-08)" section. Verification: `pytest
tests/ -q` (2026-07-08) = **2093 passed / 1 xfailed / 1 failed in 507.16 s**, where the
sole failure is the load-flaky `test_hot_loop_allocation_contract` (passes isolated, 1
passed in 7.17 s) → effectively **2094 passed / 1 xfailed, 0 real failures**; net +15 =
the new audit-fix tests only.

### Phase 4 — Limit ramp-up *(multiple short sessions)*

**Goal**: raise vel/acc/jerk toward the levels Phase 6 publishes as required for
catching; one lean-shaping A/B.

**Code** (small): `shaping.py` (lean heuristic + lever-arm compensation; config
gain + per-call override in GoToPose for the A/B); peak tracking in
`trajectory/diagnostics`; a `/diagnose` extension summarising per-move peaks
from rosbags; scripted battery `tests/hardware/traj_ramp_battery.py`
(service-calls only; operator-run).

**Per-step session protocol**: raise ONE limit ~1.5× via `trajectory/set_limits`
→ run the battery → `/diagnose` review (tracking error, jerk headroom,
audible/visual assessment) → operator PASS ⇒ persist to `hardware_config.yaml`
(edit → regenerate → commit between sessions). ABORT at any step: revert to the
last-good YAML values. Lean A/B: identical xy move at `lean_gain` 0.0 vs 0.3 —
kept only if measured leg jerk drops and motion looks calmer.

**Exit**: limits at or above the Phase-6-published catch requirements (expected
order: vel ~150–250 mm/s, acc ~1500–3000 mm/s²; the sim provides the real
numbers — Phase 6 feeds back into this phase's target).

**Outcome (2026-07-08 — CODE COMPLETE, hardware deferred)**: All Phase 4 *software*
landed on branch `mvp-trajectory-bringup` (commits `2d3afa0` shaping + planner
wiring; `6eb2c74` `GoToPose.lean_gain` + node observability; `7588fe6`
`/diagnose` extension + ramp harness; docs `4b2c02e`). New `motion/trajectory/
shaping.py`: the lean-into-translation heuristic (`rx += −k·a_y/g`, `ry += +k·a_x/g`,
gain `JB_TRAJ_LEAN_GAIN` default **0.0**, 5° cap) + a cup-height-derived lever-arm
xy compensation ported from `Jugglebot-bb/sim/juggle_tilt.py::realize_tilted`
(`centroid = target − shift`, arm **+95.1 mm ⇒ 1.66 mm/deg**, sign pinned to the
physics). Shaping runs **before** `validate` — `planner._build_rest_move` wraps the
plan and the node passes the shaper *into* `build_move`, so the single-gate invariant
holds and the duration-stretch loop sizes the shaped motion. `GoToPose.srv` gains a
per-call `float64 lean_gain` (< 0 ⇒ config default, ≥ 0 ⇒ clamp [0,1], default 0.0 ⇒
OFF) for the A/B. `trajectory_node` tracks **realized** leg peaks off the emitted
knots (reset per move) alongside the gate's **predicted** peaks and publishes them +
the session limits + active `lean_gain` on `trajectory/diagnostics`; `sim/analysis/
diagnose.py::summarise_trajectory_moves` turns a ramp rosbag into a per-move
`peaks + headroom` table (the `/diagnose` "Trajectory Moves" block), segmented by a
new `move_seq` diagnostics key — the salvage corrected a bug where the summariser
collapsed the whole holdless battery into one row (`plan.kind` stays `'move'`
across a completed move's terminal hold) and used the coarse realized jerk for
headroom (now also surfaces `used_pct_predicted`, gate-authoritative). Operator harness
`tests/hardware/traj_ramp_battery.py` (service-calls-only) + protocol
`session_phase4_ramp.md`. **No limit values landed** (Phase 4 lands none — the ramp
is operator bench work), so **no `hardware_config.yaml` / codegen change**.
Verification: `pytest tests/ -q` (2026-07-08) = **2120 passed, 1 xfailed in 518.69 s**
(baseline 2094/1 at `e039cc0`; net +26 = the new tests only: 14 shaping + 5 diagnose
+ 7 node); `colcon build --packages-select jugglebot_interfaces jugglebot` (2026-07-08) =
2 packages finished, 0 errors (`GoToPose.lean_gain`). Salvage: the phase was
implemented by a prior agent that died mid-run leaving the diff uncommitted and
unverified; this session independently verified every invariant against the
reference geometry, ran the gates, and applied the audit — full narrative in
`logbook/2026-07-08-mvp-phase4-shaping-ramp-tooling.md`. **Deferred to operator bench
sessions**: the per-step limit ramp (raise ONE limit ~1.5× → battery → `/diagnose`
review → persist to YAML between sessions) and the lean A/B (gain 0.0 vs 0.3 —
kept only if measured leg jerk drops and motion looks calmer).

**Outcome addendum (2026-07-08 — audit-fix round)**: a `/audit e039cc0..HEAD` raised
1 BLOCKING + 4 WARNING + 2 NOTE, all applied surgically — the BLOCKING was a shaped
`state_at(T)` seam bug (the terminal-hold branch fired at the gate's final grid
sample) that fabricated a 721,215 mm/s³ jerk spike and inflated shaped lateral moves
~5–8×; fixing the boundary (+ refining the stretch-loop overshoot by bisection) brings
gain-0.3 lateral moves to an honest **1.45×** the unshaped minimum, plus the ramp
battery's BUSY-cascade settle, a `move_seq` bump so go_home/stops are their own
`/diagnose` rows, a realized-peaks install-straddle guard, and a loud `validate_follow`
shaped-plan rejection (full detail in the logbook's "Audit fixes" section).

### Phase 5 — Timed target states (goal 3)

**Goal**: ((position), (orientation), arrival_time) with loud rejection of
too-tight timing.

**Code**: `TimedTarget.srv`; `planner.build_timed` (arrival-twist boundary
conditions, `hold_after`); one ROS-time→perf_counter conversion point;
`catch/dynamic_target` consumption in CATCH mode through the same `build_timed`
(+ the reach-freeze window); `trajectory/target_feedback` publisher +
`catch_coordinator_node` swap from the ZMQ :5559 `TargetFeedbackSub` to the
topic (blacklist semantics preserved).

**Tests**: arrival-time accuracy in pure-Python playback (pose error at
`t_arrival` zero by construction; knot quantisation ≤ one 25 ms knot);
TOO_FAST / min-lead rejections; nonzero arrival-velocity BCs; freeze-window
behaviour; catch_coordinator feedback-topic integration.

**Hardware session**: timed moves with generous timing (PASS: mocap-measured
arrival within ±25 ms, pose within 3 mm / 0.5°); a same-pose request at 60% of
`min_duration_s` (PASS: rejected, response carries the achievable duration); a
mid-plan superseding timed target (PASS: C2 replan; both deadlines honoured or
the second loudly rejected).

**Outcome (2026-07-08 — CODE COMPLETE, hardware deferred)**: All Phase 5 software
landed on branch `mvp-trajectory-bringup` (commits `62e9ea7` motion +
interfaces + node + catch swap; `d40da26` docs). New pure `planner.build_timed`:
a **fixed-lead reach** to `(target_pose, target_twist)` — the arrival lead is a hard
constraint, so a too-tight lead is loudly rejected `TOO_FAST` with the minimal feasible
lead (never silently slowed), a spatial failure re-raised as-is; the plan is ALWAYS
rest-terminating (a nonzero arrival velocity gets a decel-to-rest continuation — a
final nonzero-velocity segment would be an unbounded-jerk terminal-hold snap);
`hold_after` = hold-at-target vs return-to-neutral. Gated by the **fast**
`validate_follow`, which is what enables the mid-plan **supersede** (a C2 replan
installs within ~2 ms of its seed — the TOCTOU class the Phase-2 guard closed on the
~377 ms path), so the Phase-2 `BUSY` restriction is **lifted for the timed path** (
`go_to_pose` keeps `BUSY` — it uses the analytic gate for shaped plans; the asymmetry
is deliberate + tested). New interfaces `TimedTarget.srv` (response `code` = the
feasibility **string** enum, matching GoToPose) and `TargetFeedback.msg`;
`trajectory/timed_target` (TRAJECTORY mode) + `catch/dynamic_target` (CATCH mode)
BOTH route through `build_timed` (+ a reach-freeze window that holds the committed
catch reach through the last `JB_TRAJ_CATCH_REACH_FREEZE_S`); `trajectory/target_feedback`
carries accept/reject to `catch_coordinator_node`, which **swaps** its dormant ZMQ :5559
`TargetFeedbackSub` for the topic (feasibility blacklist semantics preserved). One
ROS-clock→perf_counter conversion point (`_ros_time_to_perf`); the catch path is already
perf-domain (system-wide `CLOCK_MONOTONIC`). The catch z is lifted by the active-z (170)
from the MPC-offset convention (0 = active) to the trajectory STOW-relative convention
(170 = active) — flagged for Phase-7 hardware verification (the gate rejects an
out-of-stroke z loudly meanwhile). Every emitted timed knot is pump-accepted (invariant
re-asserted). Verification: `pytest tests/ -q` (2026-07-08) = **2164 passed,
1 xfailed in 535.82 s** (baseline 2128/1 at `1c0f9c1`; net **+36** = new tests only: 12
planner-timed + 17 node + 7 catch-coordinator); ci-deep (`pytest tests/ -q
--hypothesis-profile=ci-deep`, 2026-07-08) = **<PENDING>**; `colcon build
--packages-select jugglebot_interfaces jugglebot` (2026-07-08) = 2 packages finished,
0 errors. No `hardware_config.yaml`/codegen change (the Phase-5 constants landed in
Phase 1). **Deferred to the operator bench session**: the ±25 ms mocap timed-move
battery + too-tight loud-rejection + mid-plan supersede demo
(`tests/hardware/session_phase5_timed.md`). Full narrative in
`logbook/2026-07-08-mvp-phase5-timed-targets.md`.

### Phase 6 — Sim port + catch trajectory + hand-model fidelity *(sim only)*

**Goal**: the reload behaviour passes the Reload gate using the production
planner/emitter and the arm-and-forget hand model.

**Code**: the Jugglebot-bb port + six-file diff audit (§ Sim strategy);
`tilt_geometry.py`, `ballistics_bc.py`, `planner.build_catch` (reach / freeze /
tilt-through-seat / quiescent hold); a new headless seeded harness
`sim/reload_gate.py` (drives `motion.trajectory` planner + emitter → sim plant;
arm-and-forget `HandCatchSequence` with realistic arm latency; noise from
`juggle_noise.py`; JSON gate report); `sim/juggle_bb_catch.py` retained for
interactive visual verification. Hand-model fidelity work per
§ Hand-catch smoothness — the sim reproduces the hardware-proven catch;
hardware-side changes are out of scope for this phase.

**Tests**: `tests/sim/test_reload_gate.py` (small-N smoke in CI; the full
20-run gate runs manually); tilt/lever-arm geometry regression vs the
Jugglebot-bb reference values (1.66 mm/deg, ≤12°); knots→pump acceptance
inside the harness.

**Exit**: Reload gate PASS including the contact and hold-quiescence criteria;
the required leg limits published back to Phase 4. **Logbook** entry includes
the contact relative-velocity and hold-travel plots (the evidence that resolves
the "prior sim smoothness was not real" caveat).

### Phase 7 — Reload on hardware (goal 4)

**Goal**: BB throws; Jugglebot catches; exposed as `Reload.action`.

**Code**: `Reload.action`; `reload_sequencer.py` + `reload_coordinator_node.py`
(launch + setup.py); the `BallButlerThrow.srv` point-target extension +
`ball_butler_node` handler branch; the frame-convention verification task;
an integration test of the announcement→correlation→coordinator path against
recorded bags.

**Hardware sessions (staged)**:
- **7a — aim-only**: `bb/throw_at_target` speed-0 fast-path at the computed
  catch point; verify yaw/pitch geometry against mocap. No Jugglebot motion, no
  ball. PASS: aim converges on the catch point within BB's spatial calibration.
- **7b — throw + static catch**: Jugglebot in CATCH mode holding the neutral
  catch pose; hand armed by the existing coordinator; BB throws dead-centre.
  PASS: ball seated; hand telemetry matches the profile. ABORT: two consecutive
  bounce-outs → back to Phase 6 with the hardware traces. (Hardware has caught
  smoothly before — priors are good.)
- **7c — full reload action**: translate + tilt catch; ≥ 3/5 catches with
  `catch_error_mm` logged; every abort path exercised once deliberately
  (no-ball reject; announcement-timeout abort with BB disabled mid-sequence).

**Exit**: `ros2 action send_goal /jugglebot/reload …` reliably catches. MVP
goal 4 complete.

### Phase 8 *(stretch)* — Single-ball self-toss loop

Sim gate first (Self-toss gate). Code: throw scheduling in the coordinator —
hand `traj_type=0` throw with `event_delay`/`event_vel`, platform quiescent
through release (cup acceleration = gravity at detach is the hand's job; the
platform stays level and static for the MVP); a self-`ThrowAnnouncement`
(`thrower_name='jugglebot'`) so the existing correlation → catch path closes the
loop. Hardware: single toss-and-catch, then N-loop. Starts only after Phase 7
exits.

### Phase 9 *(extra stretch)* — Two-ball juggling

Sim gate first (Two-ball gate; `juggle_planner.plan_cup_cycle` timing math as
prior art). BB second-ball throw into a loaded Jugglebot; alternating
catch–throw cycles. Scoped as a sketch; a detailed re-plan happens when the
phase is reached, informed by Phases 6–8 evidence.

---

## Testing Plan

- **Unit/property tests** land with each phase as listed; all pure-math tests
  live in `tests/motion/` (no ROS), node behaviour in `tests/ros/` (mock-ROS
  conftest), sim harnesses in `tests/sim/`.
- **The cross-cutting invariant test**: every frame the emitter can produce is
  accepted by a real `SetpointPump` — enforced in Phase 1 and re-asserted inside
  every sim gate harness (production-in-the-loop rule).
- **Pre-commit gate**: full `pytest tests/ -q` before every commit (known
  order-flaky allocation-budget tests re-run isolated per the standing note).
- **Hardware verification** per phase protocols above; `/diagnose --latest`
  after every motion session; rosbags retained for the ramp table.
- **`ci-deep` run** at the end of Phases 2, 5, and 7 (the phases that add the
  most property-tested math), cited with the (date, command, result) triple.

## Risk register

| Risk | Impact | Mitigation |
|---|---|---|
| 40 Hz emitter jitter on the non-RT Jetson | velocity ripple at knots; worst-case MPC_STALE | dedicated thread + absolute deadlines (the discipline the validated 40 Hz MPC path used); jitter published in diagnostics; 250 ms staleness ⇒ 10× margin |
| Arm-time u0/encoder mismatch | MAX_DEVIATION E-STOP at arm | measured-telemetry seeding + 0.25 rev service pre-check; firmware 0.5 rev backstop |
| RPC/stream interlock ordering mistakes | rejected ACTIVATE/DEACTIVATE mid-session | firmware already rejects loudly; ops sequences documented; disarm-before-deactivate enforced by firmware |
| rotvec-rate ≈ angular-velocity approximation | leg-vel FF error at large tilt | MVP tilts ≤ 12°; the gate checks true sampled leg velocities |
| Hand arm-time sensitivity on hardware | bounce-outs | hardware priors are good (smooth catches already achieved); Phase 6 sweeps establish margins; config changes only on hardware evidence |
| QTM-world vs jugglebot-base frame error in BB aiming | throws miss the platform | Phase 7a aim-only gate before any ball flies |
| perf_counter clock-domain mixups | mistimed catch | keep the existing perf-domain contract; single conversion point; unit-tested offset math |
| Diverged bb-branch sim support files | port breaks main sims | explicit six-file diff-audit task in Phase 6 |
| Marginal CAN3 (known, tier-2) | transient bus faults during sessions | out of scope; occurrences noted; unexplained bus faults are session-ABORT criteria |

## Deferred (explicitly out of MVP)

- **Configurable knot rate.** `SEGMENT_T_S` stays compile-time 25 ms. When
  needed: prefer a per-frame segment-duration field in `SetpointPayload`
  (protocol bump via `generate_udp_protocol.py`; staleness/extrapolation windows
  scale from the live value) over a stateful RPC.
- **MPC return path**: dormant and untouched. The :5557 envelope stays
  byte-identical, so `run_mpc.py` can be relaunched (with trajectory_node
  stopped; the single-binder interlock makes conflicts loud).
  `mpc_bridge_node.py` source retained.
- **Full jerk-limited hand-generator overhaul**
  (`Jugglebot-bb/plans/active/hand-trajectory-generator-overhaul.md`); only the
  slim `makeCatch()` parameterisation remains pre-scoped, and only on hardware
  evidence.
- **motor_guard + motion_bridge_node launch retirement** — both are off the leg
  path; verify GUI topic consumption before removal (cleanup, not MVP).
- **Orchestrator-automated arming** (auto `set_setpoint_output` on ACTIVE entry).
- **Lean shaping beyond the single gain**; `juggle_optimizer.py` port; GUI
  integration for TRAJECTORY mode.

## Notes for Collaborators

- **Phases are sized for one fresh agent session each** via `/next-phase-prompt
  mvp-trajectory-bringup <N>`; Phase 4 is several short operator sessions with
  small code deltas.
- The single most load-bearing convention: **everything that moves the platform
  under command flows through `planner.py` and therefore through
  `feasibility.py::validate`**. Resist adding side-channel motion paths; that is
  how the loud-rejection guarantee dies.
- The emitter envelope is byte-compatible with `HardwarePlant` **by
  construction** (shared `_pack`); any change to `make_mpc_command` fields must
  keep `SetpointPump` acceptance and the golden-parity test green.
- Hardware sessions: the operator runs all robot-actuating commands; implementing
  sessions prepare exact commands + PASS/ABORT criteria and verify read-only.
  At session start, invite physical-intuition pushback explicitly (per
  CLAUDE.md).
- Jugglebot-bb remains the archaeological reference (plans, logbook chapter
  2026-06-24 → 07-04, sim evolution); code ports come from there, but its ROS
  side pre-dates the Teensy cutover — never port nodes from it.
