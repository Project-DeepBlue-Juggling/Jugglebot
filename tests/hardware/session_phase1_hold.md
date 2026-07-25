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

> **⚡ Superseded in part, 2026-07-15 (ARMING CONTRACT)**: arming is now
> **automatic on ACTIVE entry** (`ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md`;
> see the banner in `mvp_bench_runbook.md`). The manual
> `set_setpoint_output true` step below runs only under `auto_arm:=false` —
> which is what you want if you intend the pre-arm probe verification this
> protocol scripts, **so Step 0's launch now needs `auto_arm:=false`** (without
> it, `activate` arms the wire before the probe step, defeating the probe-first
> premise). PASS/ABORT criteria are unchanged.

## Step 0 — Launch disarmed, with auto-arm off (probe-first protocol)

```bash
ros2 launch jugglebot jugglebot_launch.py auto_arm:=false
```

- The bridge sends `mpc_active=0` (no setpoint downlink) until explicitly armed.
- `trajectory_node` starts its 40 Hz emitter thread and binds :5557. It does NOT
  stream until the control mode enters a streaming mode (STANDBY) AND telemetry
  has seeded a hold pose.

## Step 1 — Home, activate, mode STANDBY; confirm the hold stream

Operator drives the orchestrator through the normal cold-start (home → activate).
Activate parks the legs at the active pose (~ext 154.5 mm ≈ 2.19 rev/leg).

**Drive the orchestrator over the `/orchestrator_command` topic — NOT the
same-named bridge services:**

```bash
ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'home'"      # skip if is_homed
ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'activate'"
```

> ⚠ **Do not use `--once`.** It publishes and exits before FastRTPS has matched the
> orchestrator's subscription, so the command is frequently lost with no error (this
> Foxy build has no `-w/--wait-matching-subscriptions`). Repeat-publish with
> `-t 3 -r 2` and confirm `Command received: <cmd>` appears in the launch window —
> `orchestrator_node._on_command` logs every command it accepts. Repeats are safe
> (mode commands are idempotent; unrecognised ones are discarded). See runbook Sharp
> Edge #5.

> ⚠ **Do NOT call `ros2 service call /activate std_srvs/srv/Trigger`.** `/activate`
> and `/home` are **low-level `teensy_bridge_node` services**; `orchestrator_node`
> serves neither (it only *subscribes* to `/orchestrator_command`). Calling the
> service directly fails twice over:
>
> 1. The state machine never leaves `IDLE`, so `control_mode` stays `''`
>    (IdleHandler blanks it once any pending operation resolves — A4). That
>    empty string is not in `trajectory_node`'s
>    `_DEFAULT_STREAM_MODES`, so `_streaming` stays `False` and **the 40 Hz emitter
>    never publishes** — the probe below reports `rate_hz 0`, `u0_mean nan`,
>    `pump_rej 0` even though :5557 is bound and healthy.
> 2. `_svc_activate` runs `_run_activate` **only**, while the orchestrator's
>    `/activate_or_deactivate` path runs `_run_activate` *then* `_run_configure`.
>    Skipping the configure leaves the legs in **TRAP_TRAJ** instead of
>    POSITION/PASSTHROUGH, so the setpoint chain is not interp-ready.
>
> Both faults clear by driving `activate` over `/orchestrator_command`.

You do **not** need a separate `'standby'` publish: `ActiveHandler.on_enter`
(`state_machine.py:403`) always resets `active_mode` to `STANDBY` on entering
ACTIVE, so re-activation never inherits a prior sub-mode. Publish `'standby'` only
to *return* to STANDBY from TRAJECTORY / SPACEMOUSE / CATCH.

Confirm before proceeding: `/orchestrator_state` → `ACTIVE:STANDBY`, and
`/control_mode_topic` → `STANDBY`.

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
- No frames (rate 0) → in order of likelihood: (a) `control_mode` is not a
  streaming mode — check `/control_mode_topic` and `/orchestrator_state`; the usual
  cause is having activated via the `/activate` **service** instead of the
  `/orchestrator_command` topic (see the warning above); (b) the emitter never
  seeded (check `trajectory_node` log for the seed line and that `robot_state` is
  arriving); (c) :5557 bind failed (is `run_mpc.py` running?).
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
ros2 service call /trajectory/go_home std_srvs/srv/Trigger        # see the go_home note below
ros2 service call /set_setpoint_output std_srvs/srv/SetBool "{data: false}"   # disarm (mpc_active=0)
# operator: orchestrator deactivate
```

**PASS**:
- `go_home` returns `success: true`. **Whether it moves depends on whether a
  levelling correction is loaded** (`levelling-frame-contract` Phases 1–2,
  2026-07-26; contract `ros_ws/docs/levelling_frame.md`): with **no** correction
  it is a genuine no-op (hold pose ≈ neutral, realized peaks 0.0), and with one
  loaded `go_home` targets the *corrected* neutral, so the worst leg walks up to
  **2.7736 mm = 0.03908 rev** over the 2.0 s profile (peak leg velocity
  ~2.60 mm/s, per-knot |Δu0| 9.2e-4 rev, for the 2026-07-25 offset). Both are a
  PASS. Every sitting begins with a manual `level`, so **expect the small move**
  — it is not a fault.
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

---

## Session result — 2026-07-09: **PASS**

Artefacts: `temp/probes/traj_stream_probe_20260709_125835.csv` (30 s pre-arm),
`temp/probes/traj_stream_probe_20260709_130008.csv` (120 s hold),
rosbag `~/Desktop/rosbags/2026-07-09_12-51-08` (738.3 s, 259 788 msgs).

| Criterion | Measured | Verdict |
|---|---|---|
| Step 1 stream rate | 40.03 Hz mean (39.9–41.0) over 30 s | PASS (34–46) |
| Step 1 `u0_mean` | 2.19680 rev, zero spread | PASS (≈ 2.19) |
| Step 1 `max_step` / `pump_rej` | 0.00000 rev / 0 | PASS |
| Step 2 motion at arm edge | largest single-sample step 0.00172 rev over the whole 293 s armed window | PASS (no snap) |
| Step 3 hold rate | 40.02 Hz mean (39.9–41.7) over 119.73 s | PASS |
| Step 3 leg drift (120 s) | max per-leg spread **0.0005 rev**; max endpoint drift 0.0001 rev | PASS (< 0.02) |
| Step 3 tracking | max abs(cross-leg mean − commanded `u0_mean`) = 0.00011 rev | PASS |
| Rejects / faults | `pump_rej` 0; `last_rejection` empty for the whole session | PASS |
| Step 4 DEACTIVATE | accepted — legs stowed 2.192 → ≈ 0.0 rev within ~3 s of the command | PASS (`mpc_active` cleared) |

Sequencing observed in the bag (single absolute clock):

- `orchestrator_command 'activate'` @ 12:58:29.614 → `ACTIVE:STANDBY` @ 12:58:29.73
  → `control_mode 'STANDBY'` @ 12:58:31.22 → `streaming=True` @ 12:58:31.295.
  **Activate → streaming latency = 1.68 s** (the `_run_activate` move + the folded
  `_run_configure`).
- `orchestrator_command 'deactivate'` @ 13:03:25.270 → `streaming=False` @ 13:03:25.495.
- Exactly **two** `/orchestrator_command` messages all session (`activate`,
  `deactivate`) — confirming STANDBY is automatic on ACTIVE entry.
- Emitter: `seq` reached 11 763 over the 293.2 s armed window (≈ 40.1 Hz);
  session-max `max_emit_gap_ms` = **42.27 ms**, comfortably inside the 250 ms
  staleness window (partial evidence for runbook open item #6 — this is the max,
  not the p95, which still wants the DEBUG install-latency logs).

**Cost of the doc gap**: the session's first attempt reported `rate_hz 0` for 30 s
because activation went through the `/activate` service rather than
`/orchestrator_command`. That is why Step 1 above now spells out the commands.
