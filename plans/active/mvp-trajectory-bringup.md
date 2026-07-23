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

1. **Reachability/workspace/stroke** along the sampled path (200 samples/segment
   as designed; *2026-07-16: default now 80 for unshaped plans with a ×1.05 jerk
   margin; 2026-07-17: shaped plans floored at 1600 on a numpy-vectorised gate —
   the accuracy fix, see logbook 2026-07-17-shaped-planning-efficiency-implemented
   (Phase 1a/1b) and 2026-07-16-lean-planning-latency-and-boundary-step*):
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
  - `catch/dynamic_target` — gated on the catch-armed latch (`trajectory/arm_catch`,
    raised by the `jugglebot/reload` action — no CATCH mode since 2026-07-20); the
    perf_counter time-domain contract is kept, with one conversion point in this node.
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
    *(2026-07-16: `arrival_time` replaced by `float64 lead_time_s`, relative to
    service receipt — see logbook 2026-07-16-timed-target-lead-time.)*
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
reason. *(Superseded 2026-07-15, ARMING_CONTRACT: the `enable_setpoint_output`
launch parameter is now INERT — the service is the only arming path, invoked
automatically on ACTIVE entry by the orchestrator under the default
`auto_arm:=true`.)*

Operator sequences (also documented in the ops doc when Phase 1 lands):

- **Arm**: orchestrator `activate` (TRAP_TRAJ park at ~ext 154.5 mm) → mode
  STANDBY (trajectory_node seeds from telemetry and streams hold) →
  `ros2 service call /set_setpoint_output std_srvs/srv/SetBool "{data: true}"`
  → verify `mpc_active=1` and zero motion.
- **Disarm**: `trajectory/go_home` → `set_setpoint_output false` → orchestrator
  `deactivate` (the firmware rejects DEACTIVATE while armed, enforcing order).

### Configuration (`config/hardware_config.yaml` → `generate_config.py` → `JB_TRAJ_*`)

*(Initial 2026-07-08 values shown. The ceilings were opened to administrative
5000/5000/200000 on 2026-07-16 — see
`logbook/2026-07-16-max-deviation-guard-tracking-lag.md` Addendum. On 2026-07-17
the session DEFAULTS were persisted to the S4 working point (1000, 5000, 30000)
with `lean_gain 0.6` — see `logbook/2026-07-17-s4-closed-working-point-persisted.md`.)*

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

> **⚡ Action-driven reframe, 2026-07-20 (no CATCH mode; 809.08 cup-plane aim)**: the
> reload no longer relies on a persistent **CATCH** control mode the operator holds — that
> mode was retired (`plans/active/reload-action-catch-latch.md`;
> `logbook/2026-07-20-reload-action-catch-latch.md`). The `jugglebot/reload` action now
> **owns the platform + hand for its duration**, running from **ACTIVE + streaming a hold
> in TRAJECTORY** (armed). It raises a **catch-armed latch** on `trajectory_node`
> (`trajectory/arm_catch`, mirrored on the `catch/armed` topic that gates the hand) only
> for the ball's flight window — that latch is what lets `catch/dynamic_target` reach
> `planner.build_catch` and actuate the platform. It **primes the hand to top the moment
> the goal's preconditions pass** and **raises the latch BEFORE the throw is committed**
> (2026-07-23 reorder — BB's firmware countdown has no abort opcode, so every
> Jugglebot-side arming step precedes `bb/throw_at_target`); on CAUGHT it lowers the
> latch and **re-centers** (`go_home`); on any abort it **retracts the hand + re-centers**. The catch *mechanics* below are unchanged;
> only the *trigger* moved from a mode to the action-owned latch. The aim point is now the
> **809.08 mm cup plane** (the Q1 fix, `bdbd186`), not the 744.3 mm centroid. The numbered
> steps below are updated in place to the action-driven model.

New **`action/Reload.action`** in `jugglebot_interfaces`:

```
float64 throw_delay_s          # 0 => default 3.0 s (>= BB's ~2.5 s lead floor)
---
bool success
string outcome                 # CAUGHT | REJECTED_<code> | ABORTED_<code> | MISSED | MISSED_<code>
float64 catch_error_mm         # mocap miss distance (NaN if unknown)
---
string phase                   # CHECKING | PREPARING | AIMING | THROW_PENDING | BALL_IN_FLIGHT | CATCHING | SETTLING
```

Served by a new thin `reload_coordinator_node.py` wrapping a pure-Python
`reload_sequencer.py` FSM (unit-testable). The coordinator **orchestrates only**:
`trajectory_node` plans all platform motion; `catch_coordinator_node` arms the
hand (existing behaviour reused unchanged).

1. **Preconditions (loud rejects)**: orchestrator ACTIVE + **streaming a hold in
   TRAJECTORY** (armed) — the action never switches control mode; a non-streaming or
   non-TRAJECTORY mode → `REJECTED_WRONG_MODE`, a stopped emitter → `NOT_STREAMING`.
   `bb/heartbeat` connected ∧ IDLE; if `ball_in_hand == false`, call `bb/reload` and await
   the heartbeat `RELOADING → IDLE` + `ball_in_hand == true` (10 s timeout →
   `REJECTED_NO_BALL`); mocap fresh (`/rigid_body_poses` < 0.5 s); trajectory
   streaming on (from `trajectory/status`).
2. **Aim + throw**: `bb/throw_at_target` extended with optional
   `geometry_msgs/Point target_point_global_mm` + `bool use_target_point`
   (skips the QTM rigid-body lookup; default-zero fields keep existing callers
   working). Catch point = `(0, 0, GEOM_INITIAL_HEIGHT_MM + JB_OP_DEFAULT_ACTIVE_Z_MM +
   HAND_CATCH_OFFSET_MM)` in the world frame — the STOW height plus the STOW→ACTIVE lift
   plus the centroid→cup-plane offset (= 574.3 + 170.0 + 64.78 = **809.08 mm**, the cup
   plane where the hand intercepts the ball, per
   `reload_sequencer.compute_catch_point_mm`; the Q1 fix `bdbd186` — aiming at the 744.3 mm
   centroid ate tilt/reach margin on every catch). **The hand prime already ran at
   CHECKING (the moment preconditions passed) and the catch-armed latch was raised +
   node-confirmed in PREPARING — both BEFORE this throw call (2026-07-23 ordering: the
   throw is the LAST commitment, because it cannot be aborted).** **Phase 7a verifies the
   QTM-world vs jugglebot-base frame convention with an aim-only (speed = 0) command
   before any ball flies.**
   `throw_delay_s ≥ 2.5`; BB rejections surface as `REJECTED_BB(<message>)`.
3. **Announcement → catch**: BB publishes `ThrowAnnouncement`;
   `catch_correlation_node` tags the tracked ball; with the **catch-armed latch raised**,
   `catch_coordinator_node` primes/arms the hand (gated on `catch/armed`) and the reactive
   tilt drives `catch/dynamic_target` → `planner.build_catch` — the existing catch path,
   now latch-triggered instead of mode-gated. No announcement within `throw_delay + 0.5 s`
   ⇒ `ABORTED_NO_ANNOUNCEMENT` (SAFE_ABORT: retract hand + re-center, since the hand was
   primed and the latch raised before the throw).
4. **Platform catch trajectory** (`planner.build_catch`, sim-gated in Phase 6):
   reach translation (target ≤ 80 mm — the sim-established reliable envelope)
   starts immediately and **freezes** at `t_arrival − catch_reach_freeze_s`
   (later target updates inside the freeze window are ignored except aborts);
   tilt-to-receive (≤ 12°, cup axis collinear with arrival velocity) **ramps
   through the seat** with a small residual tilt rate at `t_arrival` decayed to
   zero over the following 0.15 s; then a **literal quiescent hold segment**
   (zero twist, `catch_settle_hold_s`); then a slow return to neutral. Platform
   translation velocity at contact is zero — velocity matching is the hand's job.
5. **Confirmation** (MVP evidence): a **tracker-id-correlated** `CAUGHT` — the reload
   coordinator latches the tracker-assigned id of our announced ball (the one it puts
   IN_FLIGHT for this robot) and confirms only THAT id's `CAUGHT` status, within the
   confirm window past the announced landing (release + ToF, not release). A stray caught
   ball with a different id never confirms. `catch_error_mm` is the tracker's last-KF
   horizontal miss from the catch point (an in-flight estimate, **not** a settled rest
   position); the hand-telemetry cross-check is documented-deferred (implement if 7c shows
   false `CAUGHT`s). Otherwise `MISSED`.
6. **Aborts**: BB reject → `REJECTED_BB(<message>)` via SAFE_ABORT (arming precedes
   the throw, so the reject always lands armed — nothing is in the air, but the robot
   must be un-armed; a prime/latch FAILURE aborts earlier still, with the throw unsent:
   `ABORTED_PRIME_FAILED` / `ABORTED_PREPARE_FAILED`);
   announcement timeout → SAFE_ABORT (retract hand + lower latch + re-center);
   post-release infeasible catch target → platform holds last valid pose, hand
   fires per its armed schedule, and the outcome resolves AT SETTLE (a
   tracker-confirmed CAUGHT wins; else `MISSED_INFEASIBLE` with the gate code —
   never a mid-flight teardown);
   operator abort = **cancel the action** (or an early node exit) → SAFE_ABORT once
   PREPARE has run: retract the hand, lower the latch, and re-center via `go_home` — the
   Phase-1 disarm-edge graceful stop makes the latch-lower seam a C2 replan (no snap).

### Hand-catch smoothness: a sim-fidelity work item (Phase 6)

**Premise (ground truth from hardware)**: real catches with the current Platform
Teensy generator and config have been smooth; the hardware arm-and-forget path is
the proven reference. The Teensy catch profile is velocity-matched by design
(accelerate to −0.9·v_ball, ~10%-stroke constant-velocity hold centred on
arrival, constant deceleration — mirrored in `sim/hand/trajectory.py`).
[**Correction (Phase-6 Outcome, 2026-07-08):** the −0.9 above is stale. The
config/firmware ground truth is `catch_vel_ratio` **0.6**
(`config/hardware_config.yaml:403` → `Teensy_code/hardware_config.h:199`); the
`sim/hand/trajectory.py` mirror's hardcoded `CATCH_VEL_RATIO = 0.9` is the stale
value (a HIGH-priority deferred fix — see the Phase 6 Outcome / logbook Open
Question 2). The physical implication is material: at 0.6 the catch is a *designed*
~40 % first-contact mismatch (the hand absorbs the ball over the stroke, not a
velocity-match at contact), which is why the ≤15 %-at-first-contact acceptance
criterion below is itself under review.]
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
| 1 | Streaming foundation (hold via new path) | — | arm + 120 s hold | **HARDWARE PASS (S1, 2026-07-09)** |
| 2 | Waypoint moves at low limits | — | move battery + loud rejection | **HARDWARE PASS (S2, 2026-07-09)** |
| 3 | SpaceMouse streaming | — | manual flight | **HARDWARE PASS (S3, 2026-07-10 — after the chase-clamp rework; first attempt 2026-07-09 failed, see below)** |
| 4 | Limit ramp-up + lean A/B | — | multiple short sessions | CODE COMPLETE (hardware deferred) |
| 5 | Timed target states | — | timed moves ±25 ms | CODE COMPLETE (hardware deferred) |
| 6 | Sim port + catch trajectory + hand-model fidelity | Reload gate | none | SIM GATE CORE PASS (vel-match criterion deferred — see Phase 6/7) |
| 7 | Reload on hardware (action) | — | 7a aim-only / 7b static catch / 7c full | CODE COMPLETE (hardware deferred) |
| 8 | *(stretch)* Single-ball self-toss | Self-toss gate | staged | NOT STARTED (stretch) |
| 9 | *(extra stretch)* Two-ball juggling | Two-ball gate | staged | NOT STARTED (stretch) |

## Build-run status (2026-07-08)

Phases 1–7 are **software-complete** on branch `mvp-trajectory-bringup` (range
`63031c3^..94d6336`, commits `63031c3` through `94d6336` inclusive): seven
autonomous phase agents (Opus 4.8) under Fable 5
orchestration, each followed by an independent `/audit` fix-and-land round. Full suite
`pytest tests/ -q` (2026-07-08) = **2274 passed, 1 xfailed in 553.60 s**; ci-deep
(`pytest tests/ -q --hypothesis-profile=ci-deep`, 2026-07-08) = **2274 passed,
1 xfailed, 198 warnings in 3024.70 s** — green. Suite grew **1956 → 2274 (+318 tests)**,
all new tests only.

**Every hardware session is deferred to the operator.** The single consolidated bench
sequencing + checklists layer is `tests/hardware/mvp_bench_runbook.md` (sessions S1→S8
in strict order, pointing at each phase's detailed `session_*.md` / plan section). The
closing meta-entry — cross-phase decision digest, audit-arc retrospective, and the
consolidated bench-must-answer list — is
`logbook/2026-07-08-mvp-autonomous-build-run.md`. Phases 8–9 remain stretch scope, not
started.

**Bench progress addendum (2026-07-10)**: S1 and S2 PASSED 2026-07-09 (see the Phase
1/2 Outcomes). The first S3 attempt (2026-07-09) FAILED — z-stutter ending in a latched
MAX_DEVIATION E-STOP and a permanent follower lockup. The root causes (reject-and-keep
follower design, replan cost blocking the knot cadence, workspace clamp parking
commanded state exactly on the stroke bound) were fixed by the **chase-clamp follower
rework** (commit `73dba2b`: `motion/trajectory/chase.py` + follower/planner/emitter
changes; suite 2304 passed / 5 skipped / 1 xfailed, `pytest tests/ -q`, 2026-07-10),
validated by replaying the recorded S3 stick stream (old code reproduces the incident;
new code: 0 rejects, 0 boundary parking, p99 14.9 ms). **S3 re-flown 2026-07-10:
PASS.** Full post-mortem + proposal dispositions:
`plans/active/follower-cadence-and-divergence.md` § RESOLUTION. This supersedes Phase
3's original keep-last-plan rejection policy and the plan-section text describing it;
Phase 4 (S4) is next on the bench.

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
1. Launch disarmed (`auto_arm:=false` for the probe-first variant); home;
   activate; mode STANDBY. Probe shows 40 Hz hold frames with u0 ≈ activate
   revs. *(Superseded 2026-07-15: under the default `auto_arm:=true` the arm
   happens automatically on ACTIVE entry — see ARMING_CONTRACT.md.)*
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

**Outcome (2026-07-09 — HARDWARE VALIDATED, S1 PASS)**: the deferred bench session
ran and passed every criterion. 40.03 Hz mean stream pre-arm, 40.02 Hz across the
120 s hold; `u0_mean` 2.19680 rev with zero spread; **120 s leg drift 0.0005 rev**
(limit 0.02); largest single-sample leg step 0.00172 rev over the entire 293 s
armed window ⇒ zero motion at the arm edge; zero pump rejects and an empty
`last_rejection` for the whole session; DEACTIVATE accepted with a clean stow,
proving `mpc_active` cleared. Emitter session-max gap 42.27 ms against the 250 ms
staleness window. Artefacts: `temp/probes/traj_stream_probe_20260709_{125835,130008}.csv`,
rosbag `~/Desktop/rosbags/2026-07-09_12-51-08`; full table in
`tests/hardware/session_phase1_hold.md` § Session result.

The session also surfaced a **documentation defect, not a code defect**: the
protocol said "home → activate" without naming the mechanism, and the operator
reasonably used the `/activate` **service**. That service belongs to
`teensy_bridge_node`, not the orchestrator — it leaves the state machine in `IDLE`
(so `control_mode = ''`, not a streaming mode ⇒ the emitter never publishes) and
skips the `_run_configure` the orchestrator's `/activate_or_deactivate` path folds
in (so the legs stay in TRAP_TRAJ rather than POSITION/PASSTHROUGH). Fixed by
spelling out the `/orchestrator_command` publishes in
`tests/hardware/session_phase1_hold.md` Step 1 and adding Sharp Edge #4 to
`tests/hardware/mvp_bench_runbook.md`. No production code changed.

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
Entry state is armed + holding in TRAJECTORY: `activate` then `trajectory` over
`/orchestrator_command` (**not** the `/activate` service — runbook Sharp Edge #4),
then `set_setpoint_output true`.
Battery: z 170→190→170; x ±20; y ±20; tilt rx ±3°; then one deliberately
infeasible request (`duration_s: 0.05`) → PASS: rejected `TOO_FAST` with
`min_duration_s`, zero motion. Run it with the scripted
`tests/hardware/traj_ramp_battery.py --lean-gain 0.0` (its `_BATTERY` list *is*
this battery; with no `--set-*` it changes no limits, and it settles
`max(settle_s, planned_duration_s + 0.5)` between moves so `go_to_pose`'s
one-move-in-flight `BUSY` restriction never cascades). PASS per move: subjectively
smooth (no audible snap), `/diagnose --latest` shows leg jerk within limits, no
pump rejects, no E-STOP. ABORT: oscillation, gate violation, tracking error
> 0.1 rev.

**Exit**: 11/11 scripted moves clean (the `_BATTERY` list holds 11 feasible moves —
2 in z, 3 in x, 3 in y, 3 in rx) + one demonstrated loud rejection.

**Outcome (2026-07-09 — HARDWARE VALIDATED, S2 PASS)**: the bench session ran the
scripted battery 13:34:07–13:34:43 at the Phase-1 default limits (100 mm/s,
400 mm/s², 8000 mm/s³) with `lean_gain = 0.0`: **11/11 feasible moves accepted and
executed cleanly**, and the deliberately-infeasible request returned
`TOO_FAST: requested duration 0.050s < minimal feasible 0.629s` while installing no
plan (`move_seq` held at 11 across it) — a loud rejection with zero motion, exactly
the designed behaviour. Realized leg peaks tracked the gate prediction to within
~1 % on vel/acc across all 11 moves (worst-case predicted 68.4 / 362.8 / 6911 vs
realized 68.3 / 362.8 / 5583 mm·s⁻¹˒⁻²˒⁻³); realized jerk ran ~20 % under prediction,
consistent with the gate's conservative jerk bound. Headroom at the defaults: vel
68 %, acc **91 %**, jerk 70 % — acceleration binds first here, which is worth
remembering when Phase 4 ramps (the ramp targets jerk first because jerk binds at the
*Phase-6* operating point, not at these defaults). Session-max emitter gap 56.60 ms.
Rosbag `~/Desktop/rosbags/2026-07-09_13-17-56`.

Two process findings, neither blocking. (1) A lost `ros2 topic pub --once` mode change
left the platform in STANDBY; the operator armed without re-checking, so all 11 moves
came back `WRONG_MODE` (no motion — the gate did its job). The cleanup then sent
`deactivate` **while still armed**, which transitions the state machine to IDLE
instantly (so `control_mode=''`, the emitter stops, and the guard latches `MPC_STALE`
within 250 ms) while the firmware *rejects* the DEACTIVATE because `mpc_active=1` — so
the legs never stowed. Recovered with `/clear_errors` + re-activate; the retry passed
clean. Both hazards are now runbook Sharp Edges #5 and #6, and the protocol gained an
explicit "verify the mode took effect before arming" gate. (2) `/link_status` is absent
from the launch's rosbag record list, so the E-STOP itself never reached the bag —
recommend adding it before S4.

**Open NOTE for `/diagnose`**: the teardown `go_home` installed as `move_seq=12` with
realized peaks 0.0 (correct — a no-op from neutral) but reported **predicted** peaks
identical to move 11's rather than zero, i.e. `peak_leg_*` appears stale for a
zero-distance plan. S4's per-step review reads predicted-vs-realized headroom directly,
so this should be confirmed or fixed before the ramp begins.

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
`validate` passes at ~377 ms each, as measured at the time; *2026-07-16: now
~0.1 s unshaped / ~1.2–1.3 s shaped after the planning speedup; 2026-07-17: shaped
now ~0.09 s after the batched 1600-sample gate + retiming-model search — see
logbook 2026-07-17-shaped-planning-efficiency-implemented*) while the
emitter streams the OLD plan, an
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

**Outcome (2026-07-17 — ✅ HARDWARE CLOSED)**: the ladder ran far past the
Phase-6 targets (zero latches post-guard-raise; 0.359 rev peak deviation at
(1500,5000,40000)); the lean A/B resolved KEEP; the working point
(1000, 5000, 30000) + `lean_gain 0.6` is persisted to YAML — see
`logbook/2026-07-17-s4-closed-working-point-persisted.md` and the runbook S4
Result block.

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

**Phase-6-published catch requirements (2026-07-08, from the reload gate — the
ramp targets)**: at the sim gate's operating point (0.7 s arrival lead, ≤ 80 mm
reach, ≤ 12° tilt, arrival speeds 2.5–4.0 m/s), the measured leg peaks over all
accepted catches, with 1.15× headroom, are **leg vel ≈ 156 mm/s, acc ≈ 660 mm/s²,
jerk ≈ 10 331 mm/s³**. All three targets exceed the Phase-1 defaults
(100 / 400 / 8000) and require ramp steps — **jerk** is the binding constraint and
the largest relative step; all three stay well inside the original YAML hard ceilings
(280 / 4000 / 200 000; administrative 5000/5000/200000 since 2026-07-16). A faster/tighter catch (shorter
lead) would raise these; the operator ramp should reach at least these before Phase 7.

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
per-call `float64 lean_gain` (< 0 ⇒ config default, ≥ 0 ⇒ clamp [0,1]; the srv field
default was 0.0 ⇒ OFF as delivered here, changed to −1.0 ⇒ defer-to-config in
b6391c1) for the A/B. `trajectory_node` tracks **realized** leg peaks off the emitted
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
installs single-digit ms typical after its seed, guard-bounded — install-continuity
rejects `STALE_STATE` on drift >0.06 rev; worst case tens of ms with the appended
stop-stretch — the TOCTOU class the Phase-2 guard closed on the ~377 ms path), so
the Phase-2 `BUSY` restriction is **lifted for the timed path** (
`go_to_pose` keeps `BUSY` — it uses the analytic gate for shaped plans; the asymmetry
is deliberate + tested). New interfaces `TimedTarget.srv` (response `code` = the
feasibility **string** enum, matching GoToPose) and `TargetFeedback.msg`;
`trajectory/timed_target` (TRAJECTORY mode) + `catch/dynamic_target` (CATCH mode)
BOTH route through `build_timed` (+ a reach-freeze window that holds the committed
catch reach through the last `JB_TRAJ_CATCH_REACH_FREEZE_S`); `trajectory/target_feedback`
carries accept/reject to `catch_coordinator_node`, which **swaps** its dormant ZMQ :5559
`TargetFeedbackSub` for the topic (feasibility blacklist semantics preserved). One
ROS-clock→perf_counter conversion point (`_ros_time_to_perf`); the catch path is already
perf-domain (system-wide `CLOCK_MONOTONIC`). The catch z was lifted by the active-z (170)
on a false premise that the wire was MPC-offset (0 = active) — the wire is in fact
STOW-relative (the coordinator subtracts GEOM_INITIAL_HEIGHT; the MPC frame is itself
stow-relative, run_mpc.py:80-82), so the lift double-counted the active height. The
Phase-7 hardware verification this was flagged for (2026-07-23 session) caught exactly
that: every catch reach commanded z≈341 mm and was gate-rejected out-of-stroke; the
lift was removed and the wire declared STOW-relative in DynamicTargetCommand.msg. Every emitted timed knot is pump-accepted (invariant
re-asserted). Verification: `pytest tests/ -q` (2026-07-08) = **2164 passed,
1 xfailed in 535.82 s** (baseline 2128/1 at `1c0f9c1`; net **+36** = new tests only: 12
planner-timed + 17 node + 7 catch-coordinator); ci-deep (`pytest tests/ -q
--hypothesis-profile=ci-deep`, run 2026-07-08) = **2164 passed, 1 xfailed, 198
warnings in 3001.75 s (0:50:01)**; `colcon build
--packages-select jugglebot_interfaces jugglebot` (2026-07-08) = 2 packages finished,
0 errors. A post-landing audit round (2026-07-08) closed 3 WARNING + 6 NOTE findings —
headline: a `max_timed_lead_s` (60 s) clock-domain guard in `build_timed` (an
epoch-magnitude lead previously drove a ~7e10-element `np.arange` → MemoryError that
killed the node), CATCH added to `_MOTION_MODES` (graceful stop on both
leaving-CATCH-mid-reach and entering-CATCH-mid-move), and the reach-freeze now
releases after `arrival + JB_TRAJ_CATCH_SETTLE_HOLD_S` (was: latched forever, silently
dropping every later catch target) with a `FROZEN` service-level feedback code the
coordinator excludes from blacklist counting (alongside a `source=='catch'` filter) —
see the logbook entry's "Audit fixes (2026-07-08)" section. No
`hardware_config.yaml`/codegen change in the original phase; the audit round added
`trajectory_op.max_timed_lead_s` (regenerated). **Deferred to the operator bench
session**: the ±25 ms mocap timed-move
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
`juggle_noise.py`; JSON gate report). Hand-model fidelity work per
§ Hand-catch smoothness — the sim reproduces the hardware-proven catch;
hardware-side changes are out of scope for this phase.
*(Amended 2026-07-08 — Phase 6 Outcome: interactive visual verification is
**superseded by the headless `sim/reload_gate.py`**; the `sim/juggle_bb_catch.py`
port is **deferred to the Phase-8/9 bb-file port** and NOT done in Phase 6. It
remains runnable in the `Jugglebot-bb` checkout meanwhile.)*

**Tests**: `tests/sim/test_reload_gate.py` (small-N smoke in CI; the full
20-run gate runs manually); tilt/lever-arm geometry regression vs the
Jugglebot-bb reference values (1.66 mm/deg, ≤12°); knots→pump acceptance
inside the harness.

**Exit**: Reload gate PASS including the contact and hold-quiescence criteria;
the required leg limits published back to Phase 4. **Logbook** entry includes
the contact relative-velocity and hold-travel plots (the evidence that resolves
the "prior sim smoothness was not real" caveat).

**Outcome (2026-07-08 — LANDED LIGHT, CORE PASS, vel-match deferred)**: All Phase-6
*software* landed on branch `mvp-trajectory-bringup` (commit `12c7ad1`). New pure
`motion/trajectory/tilt_geometry.py` (`tilt_to_receive` collinear-catch tilt, ≤12°,
+ a re-export of the Phase-4 `shaping.py` cup lever arm — single source of truth) and
`ballistics_bc.py` (touchdown quadratic + launch/arrival BCs, copied-not-imported from
`controller/ballistics.py`, overlap regression-pinned); `planner.build_catch` (reach
with zero translational arrival velocity → tilt-through-seat decay → literal quiescent
hold → optional return; fixed-lead loud `TOO_FAST`; gated by the fast `validate_follow`
— the fork owned + documented); a ported `sim/juggle_noise.py`; and a headless seeded
**production-in-the-loop** harness `sim/reload_gate.py` (drives the real `build_catch`
+ `KnotEmitter` + a real `SetpointPump` into the MuJoCo plant, ballistic ball under §3
noise, arm-and-forget `HandCatchSequence` sampled at the physics rate; JSON gate
report). Six-file diff-audit: the bb divergences are two-ball features (Phases 8/9),
NOT catch-critical — nothing ported but `juggle_noise` (one HIGH finding surfaced +
deferred: `sim/hand/trajectory.py` `CATCH_VEL_RATIO = 0.9` is stale vs config `0.6`).
Verification: `pytest tests/ -q` (2026-07-08) = **2222 passed, 1 xfailed in 526.04 s**
(baseline 2179/1 at `22ed9cf`; net **+43** = new tests only: 15 tilt + 7 ballistics +
12 catch + 9 harness). Reload gate (`python sim/reload_gate.py --trials 20`, 2026-07-08)
= **CORE PASS**: caught **20/20**, held **20/20**, core_clean **18/20**, hold travel
**≤ 0.01 mm**, tilt **≤ 0.01°**, separation **0 ms**, feasibility violations **0**,
pump rejects **0**; all four robustness sweeps (arm ±30 ms, event_vel ±10 %) CORE PASS
**20/20 each** (nominal core_clean 18/20 — the two off-core trials are still CAUGHT,
just over the ≤80 mm reach flag). **Contact-quality scope:** under MuJoCo's
contact→instant-hold model the core gate has **no operative contact criterion**
(separation is vacuously 0; "caught" is geometric) — it validates platform-side
behaviour (reach-under + quiescent hold), and **Phase 7b's two-consecutive-bounce-out
abort is the operative hardware guard for contact quality**. **Deferred (light-scope,
operator-approved):** the hand-contact velocity-match criterion (|v_hand − v_ball| ≤ 15 %
at first contact) floors at ~0.26 — root-caused as a sim contact→instant-hold artifact
(the ~14 ms Teensy velocity-hold is narrower than the achievable ±20 ms capture-timing
alignment; the cup axis IS correctly aligned) and inconsistent with the
hardware-validated 0.6 catch ratio (a designed ~40 % first-contact mismatch — the hand
absorbs over the stroke). See the logbook Open Questions. Full narrative in
`logbook/2026-07-08-mvp-phase6-catch-trajectory-sim-gate.md`.

**Deferred operator handoff (Phase 7 entry gate)**: before any hardware reload, (a)
reconcile `sim/hand/trajectory.py` `CATCH_VEL_RATIO` 0.9→0.6 with the existing hand
tests, and (b) revisit the vel-match criterion definition (measure over the seat
stroke, or against the 0.6 design) — the current ≤15%-at-first-contact metric is
inconsistent with the hardware-proven hand.

### Phase 7 — Reload on hardware (goal 4)

**Goal**: BB throws; Jugglebot catches; exposed as `Reload.action`.

> **⚡ Action-driven reload, 2026-07-20 (no CATCH mode; 809.08 cup-plane aim)**: the
> reactive catch is driven by the `jugglebot/reload` action via a **catch-armed latch**,
> not a persistent CATCH mode — see the banner in § *Reload sequence (goal 4)* above and
> `logbook/2026-07-20-reload-action-catch-latch.md`. The hardware sessions and the
> 2026-07-08 Outcome below are kept for the record; where they say "CATCH mode" read
> "TRAJECTORY, armed, with the action owning the latch", and where they say the 744.3 mm
> catch-z read the **809.08 mm cup plane** (the Q1 fix `bdbd186`). Live protocol:
> `tests/hardware/session_phase7_reload.md`.

**Code**: `Reload.action`; `reload_sequencer.py` + `reload_coordinator_node.py`
(launch + setup.py); the `BallButlerThrow.srv` point-target extension +
`ball_butler_node` handler branch; the frame-convention verification task;
an integration test of the announcement→correlation→coordinator path against
recorded bags.

**Hardware sessions (staged)**:
- **7a — aim-only**: `bb/throw_at_target` speed-0 fast-path at the computed
  catch point; verify yaw/pitch geometry against mocap. No Jugglebot motion, no
  ball. PASS: aim converges on the catch point within BB's spatial calibration.
- **7b — throw + static catch**: Jugglebot holding the neutral catch pose in
  **TRAJECTORY** (streaming); the hand armed via the `catch/armed` latch while the reach
  latch stays down so the platform holds (bench manual split — see
  `session_phase7_reload.md` § 7b); BB throws dead-centre at the 809.08 cup plane.
  PASS: ball seated; hand telemetry matches the profile; zero platform motion. ABORT: two
  consecutive bounce-outs → back to Phase 6 with the hardware traces. (Hardware has caught
  smoothly before — priors are good.)
- **7c — full reload action**: translate + tilt catch; ≥ 3/5 catches with
  `catch_error_mm` logged; every abort path exercised once deliberately
  (no-ball reject; announcement-timeout abort with BB disabled mid-sequence).

**Exit**: `ros2 action send_goal /jugglebot/reload …` reliably catches. MVP
goal 4 complete.

**Outcome (2026-07-08 — CODE COMPLETE, hardware deferred)**: All Phase-7 *software*
landed on branch `mvp-trajectory-bringup` (commits `6107d06` integration gap;
`3889444` `Reload.action` + `BallButlerThrow` point-target/aim-only; `e2c5afe` reload
sequencer FSM + coordinator node + integration test; `f3cca4c` `CATCH_VEL_RATIO`
0.9→0.6; docs `3a4fe69`). (1) **Integration gap closed**: `trajectory_node`'s
`catch/dynamic_target` path now routes through `planner.build_catch` (reach with zero
translational arrival velocity → tilt-through-seat decay → literal quiescent hold)
instead of the Phase-5 reach-only `build_timed` that parked the tilted rim at contact —
without this a hardware reload would catch WITHOUT the tilt-through-seat. The receive
tilt is already collinear with the arrival velocity (the hardware-validated
`catch_coordinator.compute_catch_orientation` ships it in `target_quat`); `build_catch`
reads it to aim the through-seat rate. The settle-bounded reach-freeze + `FROZEN`
feedback semantics are preserved. (2) New **`Reload.action`** + thin
**`reload_coordinator_node`** wrapping the pure-Python **`reload_sequencer`** FSM
(CHECKING → AIMING → THROW_PENDING → BALL_IN_FLIGHT → CATCHING → SETTLING, every loud
reject + abort); the coordinator **orchestrates only** — `trajectory_node` plans all
motion, `catch_coordinator` arms the hand, `ball_butler` throws; it never bypasses the
feasibility gate. (3) **`BallButlerThrow` point-target extension** (`use_target_point`
skips the QTM lookup; `aim_only` commands speed 0 — the 7a fast-path; default-zero
fields preserve existing callers). (4) The separable Phase-6 follow-up:
`sim/hand/trajectory.py` `CATCH_VEL_RATIO` 0.9→0.6 (config source of truth) with the
five reload-gate JSONs refreshed (all CORE PASS unchanged; the *deferred* vel-match rose
as expected — nominal worst 0.343→0.440). The
announcement→correlation→coordinator integration test drives the **real** BallTracker +
CatchCoordinator engines with synthesized messages (the recorded bags predate this
pipeline; no rosbag2 reader in the mocked CI — stated in the test). Verification:
`pytest tests/ -q` (2026-07-08) = **2261 passed, 1 xfailed in 541.51 s** (baseline
2223/1 at `bf5b46e`; net +38 = new tests only: 3 trajectory-node catch + 3 BB-node + 19
sequencer + 10 coordinator + 3 integration); `colcon build --packages-select
jugglebot_interfaces jugglebot` (2026-07-08) = 2 packages finished, 0 errors. **Deferred
to the staged operator bench sessions** (`tests/hardware/session_phase7_reload.md`):
**7a** aim-only frame + z-convention verification (verifies the QTM-world vs
jugglebot-base frame AND the 809.08 mm cup-plane catch-z before any ball flies), **7b** throw +
static catch (two-consecutive-bounce-out abort is the operative contact guard), **7c**
full `jugglebot/reload` action (≥ 3/5 catches + every abort path exercised). Full
narrative in `logbook/2026-07-08-mvp-phase7-reload-action.md`.

**Outcome addendum (2026-07-08 — audit-fix round, last before hardware)**: a pre-hardware
`/audit` found **five BLOCKING choreography bugs** on the nominal reload path — all
cross-process ordering the mocked-ROS per-node tests could not see: (1) `_send_throw`
omitted `target_name` → BB announced `target_id='point'` → the whole catch pipeline dropped
the ball; (2) the announcement is published *inside* the throw handler (during AIMING) so
the phase-gated `note_announcement` discarded it — regated on the throw being *commanded*;
(3) the settle deadline treated RELEASE as landing, omitting the 0.61–0.73 s ToF, so a
nominal catch read MISSED — now anchored on the announced landing (ROS→perf converted);
(4) `_goal_callback` accepted concurrent goals (double-throw) — now REJECTs while a reload is
active; (5) `FROZEN`/`STALE_STATE` catch feedback latched `MISSED_INFEASIBLE` — now ignored,
and a later `accepted=True` clears an earlier reject. Plus WARNINGs: the receive tilt is now
**clamped at `MAX_TILT_DEG=12°`** (single source with `tilt_geometry`), not rejected above 30°
(root cause: 12° is the sim-validated + lead-time-feasible envelope, and a clamped tilt seats
better than level — clamp-don't-reject strictly improves seating and never blacklists); the
non-finite-point guard; tracker-id-correlated CAUGHT evidence; and the 7b protocol rewritten
to hold the pose in TRAJECTORY mode (the genuine static-catch config). Verification:
`pytest tests/ -q` (2026-07-08) = **2274 passed, 1 xfailed in 553.60 s** (baseline 2261/1 at
`23e5476`; net +13 = audit-fix tests only); ci-deep (`pytest tests/ -q
--hypothesis-profile=ci-deep`, run 2026-07-08) = **2274 passed, 1 xfailed, 198 warnings in
3024.70 s (0:50:24)**. Full narrative + finding-by-finding in the logbook's **Audit fixes
(2026-07-08)** section.

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
