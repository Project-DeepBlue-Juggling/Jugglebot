---
title: MVP Phase 1 — Streaming foundation (hold via the new trajectory path)
type: feature
date: 2026-07-07
status: resolved
phase: "1"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - config/hardware_config.yaml
  - config/generate_config.py
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/ (new package)
  - ros_ws/src/jugglebot/jugglebot/motion/ipc.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot/setup.py
  - tests/motion/test_trajectory_quintic.py
  - tests/motion/test_trajectory_plan.py
  - tests/motion/test_trajectory_emitter.py
  - tests/ros/test_trajectory_node.py
  - tests/ros/test_teensy_bridge_node_setpoint.py
  - tests/ros/conftest.py
  - tools/probes/traj_stream_probe.py
  - tools/probes/README.md
  - tests/hardware/session_phase1_hold.md
commits:
  - 63031c3
  - aab9811
  - 337eb41
  - 33da615
  - d09846e
  - c0b31a9
subsystem:
  - motion
  - ros
  - can
tags:
  - feature
  - trajectory
  - safety
  - codegen
---

# MVP Phase 1 — Streaming foundation (hold via the new trajectory path)

## Summary

The inaugural phase of `mvp-trajectory-bringup.md`: the MPC 40 Hz CasADi hot loop
is removed from the leg path (kept dormant, source retained) and replaced by a
deliberately simple Jetson-side trajectory generator. Phase 1 lands the whole
streaming substrate but streams only a **hold**: a new `trajectory_node` seeds the
platform's ACTIVE pose from measured telemetry and streams 40 Hz `make_mpc_command`
knot frames on ZMQ :5557 — the exact seam `run_mpc.py`'s `HardwarePlant` used — so
`teensy_bridge_node`'s `_MpcCommandSetpointSource` → `SetpointPump` → the can-hub
Teensy 500 Hz Hermite chain consumes them unchanged. A new `set_setpoint_output`
(`SetBool`) service on the bridge arms the setpoint downlink **at runtime** with
stream-then-arm preconditions, fixing the arm-before-stream trap.

Everything that moves the platform flows through `planner.py` →
`feasibility.validate` (one canonical gate), and the emitter envelope is
byte-compatible with `HardwarePlant` **by construction** (shared `make_mpc_command`
+ `ipc._pack`). The load-bearing cross-cutting invariant is tested directly: every
frame the emitter can produce is accepted by a real `SetpointPump` instance.

Software is complete and the full suite is green (count triple in Verification).
**Hardware validation is DEFERRED** to an operator bench session — the protocol
file `tests/hardware/session_phase1_hold.md` and the read-only probe
`tools/probes/traj_stream_probe.py` are the deliverables the operator will use.

## Motivation

The MPC stack consumed weeks: the Jetson is compute-marginal at 40 Hz and the
simulated behaviour is not smooth (see `logbook/2026-05-22-mpc-compute-bound-
jetson-profiling.md` and the warm-start-deadlock arc). For the MVP the MPC is
removed from the hot path. The streaming path itself is already validated on
hardware — the migration's headline benefit (`SetpointPump` + Teensy-side Hermite,
bit-exact vs the Python reference at 5.5e-7 rev; `logbook/2026-06-25-phase11-u4-
production-cutover.md`). So the task is to put a *simple, gate-guarded* waypoint
source in front of that validated chain, starting from the smallest possible
increment: hold the ACTIVE pose. If the platform holds cleanly through the new
path and disarm/deactivate are clean, the substrate is proven and Phase 2 can add
profiled moves.

## Design

### New pure package `jugglebot.motion.trajectory` (numpy-only, no ROS/repo-root imports)

| Module | Phase-1 contents |
|---|---|
| `quintic.py` | **Copies** of `controller/hermite.py` interp + `controller/feasibility.py` closed-form peak bounds. Copied, not imported: keeps `motion/` free of repo-root/`controller` path dependence (`controller/` is dormant). The copy is pinned to the source by a bit-for-bit xref test. |
| `limits.py` | `TrajectoryLimits` — leg vel/acc/jerk session limits + YAML hard ceilings + 25 ms knot spacing + step bound, built from the new `JB_TRAJ_*` constants. `from_config` clamps runtime overrides to the ceilings (the Phase 4 `set_limits` ramp). |
| `segment.py` | `QuinticSegment` — one quintic per pose axis with pos/vel/accel BCs at both ends; `eval(t) → (pose, twist, accel)`. C2 at every join by construction. |
| `plan.py` | `TrajectoryPlan` (ordered segments + implicit terminal hold) + `state_at(t)` (replan seeding) + `HoldPlan(pose)`. |
| `feasibility.py` | The single canonical gate. Phase 1: **minimal** (stroke/workspace + leg vel/acc caps, dense-sampled). `FeasibilityReport` code enum is the full set; Phase 2 fills in checks. `TrajectoryInfeasible(code, reasons, min_duration_s)`. |
| `planner.py` | `build_hold` / `build_return_to_neutral` — both validate before returning (loud-rejection contract). Arbitrary-target `build_move`/`build_timed` land later. |
| `emitter.py` | `KnotEmitter` — samples the plan at `(τ, τ+25ms, τ+50ms)`, runs the IK chain, assembles the exact `make_mpc_command` field set. `motor_rev = ext × mm_to_rev` (no offset), byte-identical to `HardwarePlant` (hardware_plant.py:413). |

`motion/ipc.py` gains `MpcCommandPub` — a PUB that **binds** :5557 (sole-binder
interlock: a bind failure means `run_mpc.py` is running → loud fatal).

### `trajectory_node.py` (thin ROS wrapper)

Dedicated 40 Hz emitter thread (not an rclpy timer) with absolute deadlines on
`perf_counter`; the PUB is bound *inside* the thread. Always has a plan (a
`HoldPlan` once seeded) so stream gaps never approach the 250 ms staleness E-STOP
(10× margin at 25 ms knots). Plan install is an atomic reference swap seeded from
`state_at(now)` (C2 across swaps). Seeds the hold pose from **measured**
`robot_state` telemetry (`pos_estimate` rev → ext mm → FK), never nominal — this
is what keeps the first `u0` inside the pump's 0.3 rev step gate and the firmware's
0.5 rev MAX_DEVIATION. `trajectory/hold` + `trajectory/go_home` are `Trigger`
services (no move services in Phase 1). Status is a `diagnostic_msgs/DiagnosticStatus`
(see Discussion for why not a typed msg yet). A defence-in-depth per-knot step
bound in the emitter freezes to a hold at the last good pose if it ever fires.

### `set_setpoint_output` (SetBool) — runtime arming

New bridge service. On `true`, require (a) Teensy link up + fresh heartbeat; (b) a
fresh `mpccmd` frame on :5557 within 0.5 s; (c) that frame's `u0` within **0.25 rev**
(half the firmware 0.5 rev backstop) of every leg's live `pos_estimate` — derived
via a throwaway `SetpointPump.build`, so the frame we arm on is itself
pump-acceptable. Then stream-then-arm (`_start_setpoint_output` →
`_set_mpc_active(True)`) using the same source the check observed. On `false`: stop
the thread + `mpc_active=0`. Fixes the arm-before-stream trap
(`project_canhub_tier2_validated` memory: arm-before-stream self-E-STOPs).

### Config + launch

New `trajectory_op:` YAML section + a `HW_SECTIONS` row (`JB_TRAJ_`) — without the
row a regenerate silently emits no constants. `jugglebot_launch.py` drops
`mpc_bridge_node` (source retained), adds `trajectory_node`, records
`/trajectory/status`. `setup.py` adds the `motion.trajectory` sub-package + the
`trajectory_node` entry point.

## Implementation

The full pose→leg chain is reused verbatim from `motion/ik_solver.py`
(IK/Jacobian/J̇/FK) and `motion/workspace.py` — the trajectory package adds only the
quintic basis (copied) and the plan/gate/emitter orchestration. Empirical
head-start (throwaway probe): the emitter over a hold plan produced 400/400
pump-accepted frames with `u0` == the activate revs (2.19 rev), a profiled
10 mm/5 mm move produced 0 rejects, and a 0.2 s / 80 mm request raised
`TrajectoryInfeasible(LIMIT_ACC)` — confirming the gate rejects loudly. Per-frame
emitter cost measured ~2.5 ms (3 IK + J̇), ~10 % of the 25 ms budget.

## Verification

(date, command, result triples — re-runnable from the artefact alone)

- **New motion tests** (`pytest tests/motion/test_trajectory_quintic.py
  tests/motion/test_trajectory_plan.py tests/motion/test_trajectory_emitter.py -q`,
  run 2026-07-07) = **22 passed in 5.29 s**.
- **New/extended ROS node + arming tests** (`pytest
  tests/ros/test_trajectory_node.py tests/ros/test_teensy_bridge_node_setpoint.py
  -q`, run 2026-07-07) = **28 passed in 5.57 s**.
- **Affected groups** (`pytest tests/ros tests/motion tests/teensy_link -q`, run
  2026-07-07) = **956 passed, 51 warnings in 61.87 s** (warnings pre-existing —
  `test_motor_guard` return-vs-assert).
- **Full suite** (`pytest tests/ -q`, run 2026-07-07) = **1993 passed, 1 xfailed
  in 625.44 s** — 0 failed. Baseline before Phase 1 (`pytest tests/ -q`,
  2026-07-07): 1956 passed, 1 xfailed in 476.59 s. Net **+37 passed**, fully
  accounted for by the new tests and nothing else: 22 motion
  (`test_trajectory_{quintic,plan,emitter}.py` = 6+10+6) + 11 node
  (`test_trajectory_node.py`) + 4 arming (`test_teensy_bridge_node_setpoint.py`,
  the `set_setpoint_output` cases). No pre-existing test changed count; the 1
  xfailed is unchanged.
- **Codegen determinism** (2026-07-07): re-running `python config/generate_config.py`
  produced no NEW working-tree changes beyond the intended `JB_TRAJ_*` emission
  (byte-identical output on a second run).

### Hardware bench session — 2026-07-09, S1: **PASS**

Protocol `tests/hardware/session_phase1_hold.md`, run by the operator on the real
robot. Read-only artefacts, all re-derivable:
`temp/probes/traj_stream_probe_20260709_125835.csv` (30 s pre-arm),
`temp/probes/traj_stream_probe_20260709_130008.csv` (120 s hold), rosbag
`~/Desktop/rosbags/2026-07-09_12-51-08` (738.3 s, 259 788 msgs, mcap).

- **Stream rate** — pre-arm 40.03 Hz mean (39.9–41.0) over 29.37 s; 120 s hold
  40.02 Hz mean (39.9–41.7) over 119.73 s. PASS (34–46 acceptable).
- **Hold pose** — `u0_mean` 2.19680 rev, **zero spread** across both probes;
  `max_step` 0.00000 rev; `pump_rejects` 0. PASS.
- **Zero motion at the arm edge** — the largest single-sample leg step anywhere in
  the 293.2 s armed streaming window is **0.00172 rev** (10 ms sample period), so
  no step motion occurred at the arm transition. PASS.
- **120 s hold drift** — max per-leg spread (max−min) **0.0005 rev**, max endpoint
  drift 0.0001 rev, and max |cross-leg mean − commanded `u0_mean`| = 0.00011 rev.
  PASS (< 0.02 rev, 40× margin). Over the *whole* armed window (which includes
  `go_home` and the disarm) the max per-leg spread is 0.0110 rev — still PASS.
- **Faults / rejections** — `TrajectoryStatus.last_rejection` empty for every one
  of the 3 690 status messages; no E-STOP; no pump rejects. PASS.
- **Clean disarm → deactivate** — `orchestrator_command 'deactivate'` @ 13:03:25.270
  was accepted and the legs stowed from 2.192 rev to ≈ 0.0 rev within ~3 s. Since
  the firmware rejects DEACTIVATE while `mpc_active=1`, acceptance proves the flag
  had cleared. PASS.
- **Mode/streaming choreography** (single absolute clock, from the bag):
  `'activate'` @ 12:58:29.614 → `ACTIVE:STANDBY` @ 12:58:29.73 → `control_mode
  'STANDBY'` @ 12:58:31.22 → `streaming=True` @ 12:58:31.295. **Activate → streaming
  latency 1.68 s**, consistent with `_run_activate`'s TRAP_TRAJ move plus the folded
  `_run_configure`. Exactly two `/orchestrator_command` messages all session
  (`activate`, `deactivate`) — empirical confirmation that STANDBY is automatic on
  ACTIVE entry (`state_machine.py:403`) and needs no separate publish.
- **Emitter cadence under load** — `seq` reached 11 763 over the 293.2 s armed
  window (≈ 40.1 Hz sustained); session-max `max_emit_gap_ms` = **42.27 ms**, far
  inside the 250 ms staleness window. This is the max, not the p95 — open question
  #6 (emitter jitter p95) still wants the DEBUG install-latency logs across S2–S8.

**A documentation defect surfaced (no production code changed).** The protocol's
Step 1 said "home → activate" without naming the mechanism, so the operator used
`ros2 service call /activate std_srvs/srv/Trigger`. `/activate` is a
`teensy_bridge_node` service — `orchestrator_node` serves no such service; it only
subscribes to `/orchestrator_command`. The consequences were exactly two, and both
were observed: (1) the state machine never left `IDLE`, so `IdleHandler.on_enter`
held `control_mode = ''`, which is absent from `trajectory_node._DEFAULT_STREAM_MODES`
⇒ `_streaming` stayed `False` and the 40 Hz emitter never published, giving a
30 s probe run of `rate_hz 0` / `u0_mean nan` while :5557 was bound and healthy;
and (2) `_svc_activate` calls `_run_activate` only, whereas the orchestrator's
`/activate_or_deactivate` path calls `_run_activate` *then* `_run_configure` — so
the legs were left in TRAP_TRAJ rather than POSITION/PASSTHROUGH, i.e. not
interp-ready even had the mode been right. Driving `activate` over
`/orchestrator_command` fixed both at once and the session then passed end-to-end.
Fixes landed in `tests/hardware/session_phase1_hold.md` (Step 1 now spells out the
publishes and the trap) and `tests/hardware/mvp_bench_runbook.md` (new Sharp Edge
#4). Worth noting the near-miss, flagged as **hypothesis, not observation**: had the
operator forced `control_mode` to STANDBY without noticing the missing configure,
the arming preconditions would still have passed — they check link freshness, a
fresh `:5557` frame, and `u0` within 0.25 rev of `pos_estimate`, none of which
depend on the ODrive input mode. The failure would then have presented not as an
obvious no-stream but as whatever TRAP_TRAJ does to a 40 Hz setpoint chain that
expects PASSTHROUGH. We did not exercise that path, so its signature is unmeasured;
if a future session ever sees the legs armed and streaming yet tracking poorly,
**check the input mode before anything else**. The `_svc_activate` /
`_svc_activate_or_deactivate` asymmetry — one folds `_run_configure`, the other does
not — is the latent hazard, and it is reachable from any operator terminal.

## Discussion

CLAUDE.md makes the Discussion non-negotiable here: several reversible design forks
were decided under the autonomous decide+document policy, and one deviation from
the plan's stated Phase-1 surface was made deliberately.

### Why `trajectory/status` is a `DiagnosticStatus`, not the typed `TrajectoryStatus.msg`

Fork — publish status as `diagnostic_msgs/DiagnosticStatus` (key/values) rather
than add `TrajectoryStatus.msg` to `jugglebot_interfaces` now. Concrete failure
modes this prevents: (a) a new `.msg` forces a `colcon build --packages-select
jugglebot_interfaces` that the operator must run *before the node will even
import* — doing that in the same bench session as the first arming bring-up adds a
build step to a safety-sensitive session for zero Phase-1 benefit; (b) the plan
lands `GoToPose.srv` + `SetTrajectoryLimits.srv` + `TrajectoryStatus.msg` together
in Phase 2 (the services can't work without an interface build regardless), so
deferring the typed status keeps the interface change to **one** atomic colcon
rebuild instead of two partial ones. `DiagnosticStatus` is the real package
(already used by `link_status`), so Phase 1 needs no interface rebuild at all. The
cost is a small Phase-2 migration of the status publisher — one file, well-scoped.

### Why no auto-return-to-neutral on STANDBY entry (deviation from the plan)

The plan says "entering STANDBY from another sub-mode triggers a profiled return to
neutral". Phase 1 deliberately does NOT: it seeds a hold at the **measured** pose
on stream start and leaves the return-to-neutral to the explicit `trajectory/go_home`
service. Root cause this prevents: an un-commanded platform move at a mode
transition during the *first ever* arming bring-up is exactly the kind of surprise
motion the hardware protocol's ABORT criteria exist to catch. After `activate` the
measured pose already *is* neutral, so seeding-a-hold-there and requiring an
explicit `go_home` is both safe and functionally equivalent for Phase 1. The
auto-return belongs with the orchestrator `TRAJECTORY`-mode wiring (Phase 2), where
the state-machine semantics that justify an automatic move are actually present.

### Why derive the arming `u0` through a throwaway `SetpointPump.build`

Fork — reconstruct the frame's `u0` for the 0.25 rev encoder check by running a
throwaway `SetpointPump.build(frame)` rather than re-deriving `motor_rev × 1`
inline. Root cause: the pump owns the *authoritative* u0 convention (motor_rev if
present, else ext×mm_to_rev, with finite/length validation). Re-deriving inline
would be a second copy of that convention that could silently drift from the pump
the frame actually flows through at arm time; using `build` guarantees the frame we
arm on is itself pump-acceptable (same rejection path), so a malformed frame is
caught at the arming gate, not one tick after `mpc_active=1`.

### Why copy the quintic math instead of importing `controller/`

The plan mandates it and the root cause is real: `controller/` is dormant and must
not become a live import dependency of the ROS leg path (a `controller` refactor
could then break the robot). Copying ~250 lines of closed-form, already-hardware-
validated math is cheap; the risk it introduces — silent drift between copy and
original — is closed by `test_trajectory_quintic.py`, which asserts **bit-for-bit**
equality with the `controller` originals across randomised BCs. Contract, not
patch: the copy can never drift without a red test.

### The step-bound freeze is a backstop, not the primary mechanism

The emitter's per-knot `|Δu0|` freeze exists as defence-in-depth. The *primary*
smoothness/step mechanism is the gate (Phase 2's duration-stretch keeps steps under
the 0.3 rev bound with margin) plus the firmware's own MAX_DEVIATION. In Phase 1
(holds + slow go_home) the freeze can't fire; it's wired now so the invariant holds
the moment moves arrive. On a (hypothetical) violation it installs a hold at the
last good pose and resumes within one 25 ms knot — far inside the 250 ms staleness
window, so it degrades to a safe hold, never a stream gap.

## Open questions / next steps

- **Hardware session is the gate** (deferred): run `tests/hardware/
  session_phase1_hold.md` with the operator — arm at ACTIVE, 120 s hold, clean
  disarm→deactivate. The `traj_stream_probe.py` PASS/ABORT criteria are pre-written.
- Phase 2 adds `planner.build_move` + the full gate (reachability, closed-form leg
  jerk, duration-stretch, per-knot step bound), `GoToPose.srv` /
  `SetTrajectoryLimits.srv` / `TrajectoryStatus.msg` (migrate the status publisher
  off `DiagnosticStatus` then), and the orchestrator `ActiveMode.TRAJECTORY` wiring.
- The emitter's ~2.5 ms/frame IK cost has ~90 % headroom at 40 Hz; if Phase 2 moves
  (with J̇ every tick along a real trajectory) tighten it, cache the Jacobian across
  the 3 knot samples (only the first needs the velocity/accel chain).

## Audit fixes (2026-07-07)

A post-merge audit of the Phase 1 diff surfaced two WARNINGs and seven NOTEs; all
nine are fixed in one follow-up commit (SHA backfilled to `commits:` above).

**Warnings**

1. **Arming u0-vs-encoder precondition failed open on NaN `pos_rev`**
   (`teensy_bridge_node._arm_setpoint_output`). `if d > tol` let a NaN encoder
   through (`NaN > 0.25` is False), arming onto an unknown pose. Inverted to
   `if not (d <= tol)` (fails CLOSED) with a "non-finite or mismatched encoder"
   message. New test `test_arm_rejected_on_nan_pos_rev`.
2. **`robot_state` staleness guard declared but never enforced**
   (`trajectory_node`). The node recorded `_robot_state_mono` and declared
   `robot_state_stale_s` but seeded a hold from `_latest_pos_rev` regardless of
   age. Added `_robot_state_fresh()`; the mode-entry seed path and (defensively)
   `_seed_hold_from` now require fresh telemetry, else log ERROR and defer to the
   next (inherently fresh) `_on_robot_state` seed. New test
   `test_stale_telemetry_defers_seed_until_fresh_robot_state`.

**Notes**

3. **Ungated HoldPlan install + `validate()` passed NaN.** The telemetry seed now
   routes through `planner.build_hold((pose, 0, 0), …)` (gated) instead of a bare
   `HoldPlan`; `feasibility.validate` rejects a non-finite pose up front (reusing
   `UNREACHABLE`). The emitter's step-bound freeze stays a direct `HoldPlan`
   install (commented exempt — that backstop must never raise). New test
   `test_nonfinite_pose_hold_raises`.
4. **Mode-transition state written outside `_plan_lock`.** `_on_control_mode`'s
   `_streaming`/`_seeded`/`_last_motor_rev`/`_current_mode` writes are now under
   `_plan_lock` (the lock `_emit_once` snapshots them under); `_seed_hold_from` is
   still called unlocked to avoid a re-entrant double-acquire.
5. **Armed-mode-exit ⇒ latched MPC_STALE sharp edge (doc-only).** Documented in
   the `trajectory_node` module docstring and `session_phase1_hold.md`: leaving a
   streaming mode while armed stops the stream and latches an `MPC_STALE` E-STOP
   within 250 ms — always disarm before mode changes. **Deferred structural fix
   (Phase 2)**: couple mode-exit to an auto-disarm in the orchestrator wiring.
6. **`_EMIT_HZ` hardcoded.** Dropped the constant; the emitter period is derived
   from `hw.JB_TRAJ_KNOT_DT_S` (25 ms), with a comment that the firmware pins its
   segment time to match the knot spacing.
7. **quintic.py "copied verbatim" header overclaimed.** Reworded the peak-bounds
   header: copied from `controller/feasibility.py`, N-generalised (hardcoded 6 →
   N via `np.outer`), output pinned bit-for-bit by `test_trajectory_quintic.py`.
8. **Plan publications line lacked a deviation marker.** Annotated the
   `TrajectoryStatus.msg` line: Phase 1 shipped it as
   `diagnostic_msgs/DiagnosticStatus`; the typed msg + migration land in Phase 2.
9. **Disarm closed a non-owned injected source.** `_stop_setpoint_output` skips
   `close()` when `_sp_source is _injected_setpoint_source` (ownership stays with
   the test fixture). Asserted in `test_arm_and_disarm_paths`.

**Verification** (net +3 tests — the finding-9 change extends an existing test):

- Scoped (`pytest tests/ros/test_teensy_bridge_node_setpoint.py
  tests/ros/test_trajectory_node.py tests/motion/ -q`, run 2026-07-07) =
  **161 passed, 51 warnings in 30.92 s** (warnings pre-existing — `test_motor_guard`
  return-vs-assert).
- Full suite (`pytest tests/ -q`, run 2026-07-07) = **1996 passed, 1 xfailed in
  480.04 s** — 0 failed (baseline 1993/1; net +3 = the new tests only).

## Related

- Plan: [`plans/active/mvp-trajectory-bringup.md`](../plans/active/mvp-trajectory-bringup.md) — Phase 1 detail + the command-seam architecture.
- [2026-06-25-phase11-u4-production-cutover.md](2026-06-25-phase11-u4-production-cutover.md) — the validated `SetpointPump` + Teensy-Hermite chain this streams into.
- [2026-06-29-canbridge-phase0-native-harness.md](2026-06-29-canbridge-phase0-native-harness.md) — format precedent for this entry.
- Hardware protocol: [`tests/hardware/session_phase1_hold.md`](../tests/hardware/session_phase1_hold.md); probe: [`tools/probes/traj_stream_probe.py`](../tools/probes/traj_stream_probe.py).
