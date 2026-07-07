# Hardware Session — MVP Phase 1: Streaming Foundation (arm + 120 s hold)

**Plan**: `plans/active/mvp-trajectory-bringup.md` § Phase 1
**Logbook**: `logbook/2026-07-07-mvp-phase1-streaming-foundation.md`
**Goal**: `trajectory_node` streams 40 Hz hold frames on :5557; the bridge arms at
runtime via `set_setpoint_output`; the platform holds the ACTIVE pose through the
new path; disarm + deactivate are clean.

This session validates the **software already merged** on branch
`mvp-trajectory-bringup`. No code changes should be needed to run it.

---

## Roles & safety framing

- **The operator (Harrison) runs every robot-actuating command below.** The
  implementing session prepared these exact commands + PASS/ABORT criteria and
  verifies read-only (the probe, `ros2 topic echo`, the bridge log).
- **If your physical intuition disagrees with any framing here, that is
  load-bearing signal — say so before proceeding.** This is the first time the
  legs move under the new trajectory path; a surprise is a stop-and-discuss, not
  a push-through.
- E-STOP is always available. Any ABORT criterion below ⇒ cut power / trigger the
  guard, then debrief before re-trying.
- **Disarm before any control-mode change away from streaming.** Leaving the
  stream-mode set while the bridge is ARMED stops the emitter publishing, so the
  bridge stops receiving frames and latches an `MPC_STALE` E-STOP within 250 ms.
  Always run the Step 4 disarm (`set_setpoint_output false`) before changing the
  control mode out of a streaming mode. (A structural auto-disarm on mode-exit is
  a deferred Phase 2 item.)

## Preconditions

- Jugglebot powered, ODrives up, CAN3 healthy (green `link_status`).
- Repo on branch `mvp-trajectory-bringup`, `colcon build --packages-select
  jugglebot jugglebot_interfaces` succeeded, `install/setup.bash` sourced.
- `run_mpc.py` is **NOT** running (it binds :5557 — the trajectory emitter is the
  sole binder; a conflict aborts the emitter loudly with an "is run_mpc.py
  running?" error in the `trajectory_node` log).

---

## Step 0 — Launch with setpoint output OFF (default)

```bash
ros2 launch jugglebot jugglebot_launch.py enable_setpoint_output:=false
```

- The bridge sends `mpc_active=0` (no setpoint downlink) until explicitly armed.
- `trajectory_node` starts its 40 Hz emitter thread and binds :5557. It does NOT
  stream until the control mode enters a streaming mode (STANDBY) AND telemetry
  has seeded a hold pose.

## Step 1 — Home, activate, mode STANDBY; confirm the hold stream

Operator drives the orchestrator through the normal cold-start (home → activate).
Activate parks the legs at the active pose (~ext 154.5 mm ≈ 2.19 rev/leg). Set the
control mode to **STANDBY**.

Then, in a separate terminal, run the READ-ONLY probe (never commands anything):

```bash
python tools/probes/traj_stream_probe.py --duration 30
```

**PASS**:
- `rate_hz` ≈ 40 (34–46 acceptable on the non-RT Jetson).
- `u0_mean` ≈ **2.19 rev** and each `u0[0..5]` ≈ the activate revs
  (`JB_OP_ACTIVATE_POSITION_REVS`) — i.e. a bumpless hold at the active pose,
  seeded from measured telemetry.
- `max_step` ≈ 0.000 (a hold has no per-knot motion), `pump_rej` = 0.

**ABORT** (do not proceed to Step 2):
- No frames (rate 0) → the emitter never seeded (check `trajectory_node` log for
  the seed line, and that mode == STANDBY) or :5557 bind failed (is `run_mpc.py`
  running?).
- `u0` far from 2.19 rev, or any `pump_rej > 0`.

## Step 2 — Arm at runtime (`set_setpoint_output true`)

```bash
ros2 service call /set_setpoint_output std_srvs/srv/SetBool "{data: true}"
```

The bridge runs the arming preconditions before enabling: (a) Teensy link up +
fresh heartbeat; (b) a fresh `mpccmd` frame on :5557 within 0.5 s; (c) that
frame's `u0` within **0.25 rev** (half the firmware 0.5 rev MAX_DEVIATION
backstop) of every leg's live `pos_estimate`. Only then does it stream-then-arm.

**PASS**:
- Service returns `success: true`, message "setpoint output ENABLED (armed)…".
- `link_status` shows `mpc_active=1`.
- **Zero visible platform motion at the arm edge.** Steady tracking after.
- No pump-reject spam in the `teensy_bridge_node` log.

**ABORT** (immediately disarm — Step 4 first line — then debrief):
- Any E-STOP (MPC_STALE / MAX_DEVIATION) in the bridge/firmware log.
- Any visible platform motion at arm.
- `success: false` — read the message; it names the failing precondition
  (link / no-stream / u0-vs-encoder). Fix and retry; do NOT force-arm.

## Step 3 — Hold for 120 s

Leave it armed and holding. Watch the probe and `link_status`.

**PASS**:
- No setpoint rejects, no firmware faults over the full 120 s.
- Leg drift < 0.02 rev (compare `u0` and the telemetry `pos_estimate`).

**ABORT**: any fault, any reject, or drift > 0.02 rev.

## Step 4 — Clean disarm + deactivate + shutdown

```bash
ros2 service call /trajectory/go_home std_srvs/srv/Trigger        # no-op from the held active pose
ros2 service call /set_setpoint_output std_srvs/srv/SetBool "{data: false}"   # disarm (mpc_active=0)
# operator: orchestrator deactivate
```

**PASS**:
- `go_home` returns `success: true` (a genuine no-op: hold pose ≈ neutral).
- Disarm returns `success: true`; `link_status` shows `mpc_active=0`.
- **The firmware ACCEPTS DEACTIVATE** (it rejects DEACTIVATE while `mpc_active=1`,
  so acceptance proves the flag cleared) — legs profile-stow cleanly.
- Ctrl-C the launch; shutdown stow completes.

---

## Exit criteria

Platform holds the ACTIVE pose via the new trajectory path for 120 s with zero
motion at arm, zero rejects/faults, and a clean disarm → deactivate. Record the
probe CSV (`temp/probes/traj_stream_probe_*.csv`) and the rosbag with the session
outcome in the logbook Verification section.
