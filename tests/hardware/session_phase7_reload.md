# Hardware Sessions — MVP Phase 7: BB→Jugglebot Reload (staged 7a / 7b / 7c)

**Plan**: `plans/active/mvp-trajectory-bringup.md` § Phase 7
**Logbook**: `logbook/2026-07-08-mvp-phase7-reload-action.md`
**Goal**: Ball Butler aims at Jugglebot's ACTIVE catch point and throws; Jugglebot
(CATCH mode, hand armed by the existing coordinator) tilts to receive and catches.
Exposed as `jugglebot/reload` (Reload.action).

This validates **software already merged** on branch `mvp-trajectory-bringup`. No code
changes should be needed to run it. The stages are gated: **do not run 7b before 7a
passes, or 7c before 7b passes.**

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
- **If your physical intuition disagrees with any framing here, that is load-bearing
  signal — say so before proceeding.**
- E-STOP is always available. Any ABORT criterion ⇒ cut power / trigger the guard,
  then debrief before re-trying.
- **Disarm before any control-mode change away from streaming** (leaving a streaming
  mode while ARMED latches an `MPC_STALE` E-STOP within 250 ms).
- **Contact-quality guard (7b/7c):** two consecutive bounce-outs ⇒ ABORT the stage and
  return to Phase 6 with the hardware traces (the sim gate has no contact-quality
  criterion; this is the operative hardware guard — plan § Phase 6 / § Phase 7b).

## Preconditions (all stages)

- Jugglebot powered, ODrives up, CAN3 healthy. BB powered, calibrated from the GUI
  (`bb/calibration_result` seen), heartbeat IDLE.
- Branch `mvp-trajectory-bringup`; `colcon build --packages-select jugglebot
  jugglebot_interfaces` succeeded; `install/setup.bash` sourced.
- `run_mpc.py` is **NOT** running (sole-binder :5557 interlock).
- Mocap (QTM) up, recording the platform rigid body AND the ball; `/rigid_body_poses`
  and `/balls` flowing.
- Arm per the Phase-1 sequence: launch (`enable_setpoint_output:=false`) → home →
  activate → confirm the 40 Hz hold stream → `set_setpoint_output true` → verify
  `mpc_active=1`, **zero motion at arm**.

---

## Stage 7a — aim-only (frame-convention verification, NO ball, NO platform motion)

**Purpose**: verify the QTM-world vs jugglebot-base frame convention AND the catch-point
z-convention (`GEOM_INITIAL_HEIGHT_MM + JB_OP_DEFAULT_ACTIVE_Z_MM` = 574.3 + 170.0 =
744.3 mm world) **before any ball flies**. The reload coordinator aims BB at exactly
this point; here we command it directly, speed 0.

Catch point (world frame, mm): **x = 0, y = 0, z = 744.3**.

```bash
ros2 service call /bb/throw_at_target jugglebot_interfaces/srv/BallButlerThrow \
  "{use_target_point: true, aim_only: true, \
    target_point_global_mm: {x: 0.0, y: 0.0, z: 744.3}, throw_delay_s: 0.0}"
```

- **PASS**: `success: true`; the returned `yaw_rad` / `pitch_rad` point BB at the space
  directly above Jugglebot's base at ~744 mm; measured against mocap, the aim ray passes
  within BB's spatial calibration tolerance of (0, 0, 744.3). **No ball leaves** (speed
  0), **no platform motion** (no ball ⇒ no announcement ⇒ hand never armed).
- **ABORT**: the aim points somewhere other than above the base — the frame convention
  or z-convention is wrong. Do NOT proceed to 7b; debrief the frame math first.
- If the aim is consistently offset by a fixed vector, record it: the catch-point
  computation (`reload_sequencer.compute_catch_point_mm`) or a frame transform needs a
  correction before any ball flies.

---

## Stage 7b — throw + STATIC catch (Jugglebot holds the catch pose in TRAJECTORY mode)

**Purpose**: prove the ball seats in the cup with the hand armed by the existing
coordinator, WITHOUT platform tilt/translation.

**Why TRAJECTORY, not CATCH, mode**: in CATCH mode the SAME `catch_coordinator_node` path
that arms the hand also publishes `catch/dynamic_target`, which `trajectory_node` turns
into a *moving* catch plan — so "static platform + coordinator-armed hand" is impossible
in CATCH. In TRAJECTORY mode, `trajectory_node` IGNORES `catch/dynamic_target` (it is
CATCH-gated), so the platform holds its commanded pose, while `catch_coordinator_node`'s
`/balls` handler is NOT mode-gated and still primes + arms the hand on the first catchable
ball. That is the genuine static-catch configuration.

1. Operator holds the neutral ACTIVE catch pose in **TRAJECTORY** mode (streaming; the
   platform does not move for the catch).
2. Confirm the platform holds and the hand is primed once `/balls` shows the ball
   (coordinator arms the hand on the first catchable ball, mode-independent).
3. Fire a single dead-centre throw. `target_name` names the robot so the announcement's
   `target_id` is `jugglebot` (not the default `point`) — otherwise the tracker tags the
   ball destination `point` and the whole catch pipeline drops it:

```bash
ros2 service call /bb/throw_at_target jugglebot_interfaces/srv/BallButlerThrow \
  "{use_target_point: true, aim_only: false, target_name: 'jugglebot', \
    target_point_global_mm: {x: 0.0, y: 0.0, z: 744.3}, throw_delay_s: 3.0}"
```

- **PASS**: ball seated in the cup; hand telemetry matches the armed catch profile;
  `/balls` reports the ball `CAUGHT`. A dead-centre throw arrives near-vertical, so the
  hand does the seating with little/no tilt. (Hardware has caught smoothly before —
  priors are good.)
- **ABORT**: two consecutive bounce-outs ⇒ stop, capture the hand traces + rosbag, and
  return to Phase 6. Also ABORT on any E-STOP or visible platform jerk.
- After each throw: `/diagnose --latest` for the hand/tracking traces.

---

## Stage 7c — full reload action (translate + tilt catch via `jugglebot/reload`)

**Purpose**: the whole sequence through the action — preconditions, aim+throw at the
catch point, announcement→catch (tilt-through-seat `build_catch`), confirmation.

Operator keeps control mode **CATCH**. Then:

```bash
ros2 action send_goal /jugglebot/reload jugglebot_interfaces/action/Reload \
  "{throw_delay_s: 3.0}" --feedback
```

Watch the feedback phases: `CHECKING (dwells up to 10 s while BB reloads if the
hand is empty — RELOADING is BB's heartbeat state, not a feedback phase) → AIMING →
THROW_PENDING → BALL_IN_FLIGHT → CATCHING → SETTLING`, then a result.

- **PASS**: ≥ 3/5 reloads return `outcome: CAUGHT` with a logged `catch_error_mm`;
  motion subjectively smooth (no snap); no pump rejects; no E-STOP.
- **Receive-tilt expectation** (7b/7c): the receive tilt is clamped to the 12° usable
  ceiling (`tilt_geometry.MAX_TILT_DEG`). Arrivals more off-vertical than ~12° are still
  CAUGHT, but with only *partial* collinear seating (the cup axis is not fully aligned to
  the arrival) — the residual is the hand's to absorb. A steep off-centre arrival that
  bounces out is a seating-margin signal, not a gate reject.
- **Cancel does not recall the ball**: cancelling the action after AIMING does NOT recall
  the ball — BB has already committed the throw, so expect a throw regardless. Stay clear
  of the flight path.
- Exercise each abort path once, deliberately:
  - **no-ball reject**: run with the hand empty and `bb/reload` disabled / ball removed
    → expect `REJECTED_NO_BALL` after the 10 s reload wait, **zero platform motion**.
  - **announcement-timeout abort**: disable BB (or block the throw) after the goal is
    accepted → expect `ABORTED_NO_ANNOUNCEMENT` within `throw_delay + 0.5 s`, platform
    holds, hand never armed.
  - **wrong-mode reject**: send the goal while NOT in CATCH mode → expect
    `REJECTED_WRONG_MODE` immediately, zero motion.
- **ABORT**: two consecutive bounce-outs (back to Phase 6); any E-STOP; a catch target
  the gate rejects mid-sequence surfacing as `MISSED_INFEASIBLE_<code>` on more than one
  in five (revisit the reach envelope / limits).

**Exit**: `ros2 action send_goal /jugglebot/reload …` reliably catches. MVP goal 4
complete.

---

## Deferred / open (carry into the debrief)

- **Catch z-convention** (`744.3 mm`) is verified in 7a — if 7a required a correction,
  update `reload_sequencer.compute_catch_point_mm` and re-run the software gate.
- **Vel-match / CATCH_VEL_RATIO** (Phase-6 open question): if 7b/7c show poor seating,
  the `catch_vel_ratio` 0.6 hand design (not the sim's ≤15%-first-contact metric) is the
  reference; hardware evidence — not sim — gates any hand-config change.
