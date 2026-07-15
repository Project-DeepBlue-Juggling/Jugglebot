# Hardware Session — MVP Phase 5: Timed Target States (±25 ms arrival + supersede)

**Plan**: `plans/active/mvp-trajectory-bringup.md` § Phase 5
**Logbook**: `logbook/2026-07-08-mvp-phase5-timed-targets.md`
**Goal**: `trajectory/timed_target` reaches a pose at an ABSOLUTE arrival time within
±25 ms (mocap-measured); a too-tight lead is loudly rejected with the achievable
duration; a mid-plan superseding timed target replans C2 (no snap).

This session validates the **software already merged** on branch
`mvp-trajectory-bringup`. No code changes should be needed to run it.

---

> **⚡ Superseded in part, 2026-07-15 (ARMING CONTRACT)**: arming is now
> **automatic on ACTIVE entry** (`ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md`;
> see the banner in `mvp_bench_runbook.md`). Where this protocol says to arm per
> the Phase-1 sequence, the arm happens automatically on `activate` — the manual
> `set_setpoint_output true` call runs only under `auto_arm:=false`.

## Roles & safety framing

- **The operator (Harrison) runs every robot-actuating command below.** The
  implementing session prepared these exact commands + PASS/ABORT criteria and
  verifies read-only.
- **If your physical intuition disagrees with any framing here, that is
  load-bearing signal — say so before proceeding.**
- E-STOP is always available. Any ABORT criterion ⇒ cut power / trigger the guard,
  then debrief before re-trying.
- **Disarm before any control-mode change away from streaming** (leaving a
  streaming mode while ARMED latches an `MPC_STALE` E-STOP within 250 ms).

## Preconditions

- Jugglebot powered, ODrives up, CAN3 healthy.
- Branch `mvp-trajectory-bringup`; `colcon build --packages-select jugglebot
  jugglebot_interfaces` succeeded; `install/setup.bash` sourced.
- `run_mpc.py` is **NOT** running (sole-binder :5557 interlock).
- Arm per the Phase-1 sequence: launch (`enable_setpoint_output:=false`) → home →
  activate → mode **TRAJECTORY** → confirm the 40 Hz hold stream (probe
  `tools/probes/traj_stream_probe.py`) → `set_setpoint_output true` → verify
  `mpc_active=1`, **zero motion at arm**.
- Mocap recording the platform rigid body for arrival-time measurement.

---

## Step 1 — Timed move, generous lead (repeat 5×)

Compute an absolute arrival ~3 s ahead of `now` in the ROS clock, then:

```bash
# Example: arrive at (x=15, z=185) 3 s from now (fill arrival_time.sec/nanosec).
ros2 service call /trajectory/timed_target jugglebot_interfaces/srv/TimedTarget \
  "{pose: {position: {x: 15.0, y: 0.0, z: 185.0}, orientation: {w: 1.0}}, \
    velocity_mm_s: {x: 0.0, y: 0.0, z: 0.0}, \
    arrival_time: {sec: <NOW+3>, nanosec: 0}, hold_after: true}"
```

Battery (all rest-to-rest, `hold_after: true`): z 170→185; x +15; y −15; a small
tilt rx ~2°. Vary the lead 2.5–4 s.

- **PASS** per move: mocap-measured arrival within **±25 ms** of the commanded
  `arrival_time`, pose within **3 mm / 0.5°**; subjectively smooth; no pump
  rejects; no E-STOP. `/diagnose --latest` shows leg jerk within limits.
- **ABORT**: audible snap, oscillation, gate violation, tracking error > 0.1 rev.

## Step 2 — Too-tight lead → loud rejection

```bash
# Same target, arrival only 0.05 s ahead → infeasible timing.
ros2 service call /trajectory/timed_target jugglebot_interfaces/srv/TimedTarget \
  "{pose: {position: {x: 20.0, y: 20.0, z: 185.0}, orientation: {w: 1.0}}, \
    velocity_mm_s: {x: 0.0, y: 0.0, z: 0.0}, \
    arrival_time: {sec: <NOW>, nanosec: 50000000}, hold_after: true}"
```

- **PASS**: `accepted: false`, `code: TOO_FAST`, `min_duration_s` populated with the
  achievable lead, **zero motion**.
- **ABORT**: any motion on a rejected request.

## Step 3 — Mid-plan superseding timed target (C2 replan)

Issue a generous timed move (Step 1), then — **while it is in flight** (before it
arrives) — issue a *second* timed move to a different target with its own generous
lead.

- **PASS**: the platform smoothly bends onto the second target (no visible/audible
  snap at the supersede); the second arrival lands within ±25 ms; OR the second
  request is loudly rejected (`accepted: false` with a code) and the first continues
  cleanly. No E-STOP, no pump rejects across the supersede.
- **ABORT**: any snap/jerk at the moment of supersede, tracking error > 0.1 rev.

*(Note: `go_to_pose` still returns `BUSY` mid-move by design — only the fast-gated
`timed_target` path supersedes. Use `timed_target` for the supersede demo.)*

## Step 4 — Disarm + deactivate

`trajectory/go_home` → `set_setpoint_output false` → orchestrator `deactivate`
(firmware accepts DEACTIVATE, proving `mpc_active` cleared) → shutdown.

---

## After the session

- `/diagnose --latest` on the recorded rosbag; note per-move leg peaks + headroom.
- Record measured arrival errors (mocap) per move against the ±25 ms budget.
- Log outcomes in the Phase-5 logbook Verification section; if any ABORT fired,
  open an `/investigate` before re-attempting.
