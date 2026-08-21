# MVP Trajectory Bringup — Consolidated Bench Runbook

**Plan**: `plans/active/mvp-trajectory-bringup.md`
**Closing logbook**: `logbook/2026-07-08-mvp-autonomous-build-run.md`
**Audience**: Harrison, returning to the bench to validate the software built while away.

This is the **single sequencing + checklists layer** for the whole MVP bringup. It
orders the hardware sessions, states entry/exit criteria and PASS/ABORT for each, and
**points at the detailed per-session protocol** (a `session_*.md` file or a plan
section) rather than duplicating it. Run the sessions **in the order S1 → S8**; each
gates the next.

---

## What was built while you were away

Seven phases of `mvp-trajectory-bringup` were implemented autonomously (a fresh
Opus 4.8 agent per phase under Fable 5 orchestration), each landed software-complete
with the full suite green and then followed by an independent `/audit` round that
fixed and re-landed. The MPC 40 Hz CasADi hot loop is out of the leg path (dormant,
source retained); in its place a simple, gate-guarded Jetson-side trajectory generator
(`trajectory_node`) streams 40 Hz waypoints on ZMQ :5557 into the already-validated
`SetpointPump` → can-hub-Teensy Hermite chain. Goals 1–4 (waypoint moves, SpaceMouse
streaming, timed targets, BB→Jugglebot reload) are all coded and tested; **nothing has
been on hardware yet** — that is this runbook.

## Branch & suite state

- Branch: **`mvp-trajectory-bringup`**, 7 phases code-complete + 7 audit rounds.
- **2026-07-10 — SpaceMouse follower reworked** after the S3 incident (commit
  `73dba2b`): chase-clamp tracking (cap-and-chase, never rejects in steady state),
  publish-first emitter (a knot never waits on planning), boundary-margin clamp (the
  S3 deadlock fix), escalation latch + hold backstop. Resolution + analysis corrections:
  `plans/archived/follower-cadence-and-divergence.md` § RESOLUTION.
- Full suite (`pytest tests/ -q`, 2026-07-10, post-rework) = **2304 passed, 5 skipped,
  1 xfailed in 582.32 s**. (Pre-rework 2026-07-08 baseline: 2274 passed, 1 xfailed;
  ci-deep 2026-07-08 green.)
- Before running: `colcon build --packages-select jugglebot jugglebot_interfaces`,
  `source install/setup.bash`. No code changes should be needed for any session below.

## ⚡ 2026-07-15 — the ARMING CONTRACT landed (read before using S1/S2 commands)

The first S1-style evening (2026-07-15) hit three arming-lifecycle failures in one
night: a stale prior-session guard latch wedged HOMING twice; a battery ran
"accepted" with **zero motion and zero warnings** because the manual arm step was
skipped; and `enable_setpoint_output:=true` self-E-STOPd at boot (the
arm-before-stream trap). All three were the same root cause — "armed" was a
cross-process invariant with no owner — and are now closed structurally by
`ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md` (A1–A5). What changes for the
operator:

- **Arming is now AUTOMATIC on ACTIVE entry** (orchestrator auto-arm; the bridge's
  stream-then-arm pre-check is unchanged and still the gate). The manual
  `set_setpoint_output true` steps in S1/S2 below are **superseded** — kept for the
  historical record. To get the old probe-first manual flow, launch with
  `auto_arm:=false`.
- **Disarm-before-deactivate is now enforced in-process by the bridge** (A3) — the
  operator sequence in Sharp Edge #6 is no longer load-bearing.
- **`enable_setpoint_output:=true` is INERT** (loud ERROR, no boot-arm). Never use it.
- **A stale guard latch at BOOT is auto-cleared** (disarmed, one-shot, loud); a latch
  that returns after the clear goes to FAULT as a live fault.
- **A move accepted while the wire is disarmed is now loud**: the service response
  carries `[wire DISARMED — setpoints not reaching the legs]` and trajectory_node
  WARNs. If you see it under auto-arm, something real is wrong — stop.

## ⚡ 2026-07-16 — MAX_DEVIATION guard raised to 1.0 rev + ODrive vel_limit raised 6.0 → 12.0 rev/s + ceilings opened (read before resuming S4)

Today's S4 limit-ramp session hit three `MAX_DEVIATION` guard E-STOPs at session
vel_limit=200 mm/s (always leg 1 first, 0.52–0.56 rev at trip). Forensics
(`logbook/2026-07-16-max-deviation-guard-tracking-lag.md`) found this was
**legitimate tracking lag, not a runaway**: the guard compares the raw streamed
40 Hz knot against the encoder *before* the lead clamp, while reflected platform
inertia (~5–20× the bare rotor J_eff) makes the velocity loop lag a fast
coordinated ramp — the deficit integrates into position deviation
superlinearly with commanded speed (0.17–0.22 rev @100 mm/s → ~0.52–0.56 rev
@190–200 mm/s), crossing the old 0.5 rev threshold. Drives were never
current-railed (peak iq 8.7 of 10 A) or velocity-railed (peak 2.2 of 4.0 rev/s).
The operator has confirmed two independent changes as a result:

1. **`MAX_DEVIATION_REV` 0.5 → 1.0 rev** (firmware guard,
   `ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h`; `FW_VERSION`
   bumped 1→2 as a human-facing identity marker only — it has no runtime/handshake
   effect). Raising the guard adds **zero** physical excursion for command-side
   faults — the lead clamp, stroke clamp, and ODrive clip independently bound the
   *executed* command regardless of the guard threshold; the accepted cost is a
   longer encoder-side-runaway detect distance (35 → 70 mm).
2. **ODrive leg `vel_limit` 4.0 → 6.0 rev/s** (config,
   `config/hardware_config.yaml` `leg_vel_limit_rps` → generated
   `ODRIVE_LEG_VEL_LIMIT_RPS`), for lead-clamp catch-up headroom. Legs are
   bench-proven to 3.4 m/s (48 rev/s) so 6.0 rev/s stays far inside the
   envelope, and the firmware's independent overspeed guard
   `MAX_MOTOR_VEL_RPS = 16.5 rev/s` still exceeds any 6.0 rev/s catch-up sprint.
   **`MAX_LEAD` stays 0.10 rev — unchanged.** The v2-bug constraint
   `Kp·MAX_LEAD ≤ vel_limit` is now `40×0.10 = 4.0 ≤ 6.0` (a healthy
   inequality — 2.0 rev/s of headroom for `vel_ff` catch-up, vs the old
   4.0 = 4.0 exact saturation that clipped `vel_ff` to zero added authority).

**Deployment is two independent halves — both must land before resuming S4:**

- **(A) The 1.0 rev guard lives in firmware** (compiled in) and takes effect
  **only after the can-bridge Teensy is reflashed**. Until reflashed the guard
  stays at 0.5 rev and will keep latching at ~190 mm/s.
- **(B) The 6.0 rev/s vel_limit lives in config** and is **pushed to each leg
  ODrive over CAN at runtime** — no reflash, no ODrive-NVM edit. It is pushed
  from `teensy_bridge_node._run_configure()`, which fires after every
  successful `/home`, on the `/configure` service, and after every
  `/activate`. The earliest effective point is **the first homing of a
  session that has rebuilt the ROS2 install** — a bare relaunch of a stale
  install keeps pushing 4.0, because the launch runs the **installed** copy of
  `hardware_config.py`, not the source tree (known project gotcha).

**Session prerequisites for the next sitting (operator runs these):**

1. **Flash firmware**: `cd ros_ws/src/jugglebot/Teensy_code_canbridge && pio run
   -e teensy41 -t upload` (USB to the can-bridge Teensy). This also arms the
   previously-dormant `torque_ff` ingest clamp from `10de03c` — reviewed and
   intended, not a side effect to chase.
2. **`colcon build --packages-select jugglebot`, then relaunch** — picks up the
   vel_limit push (reaches the drives at the first homing) and the new
   per-leg latch messaging (below).
3. **GUI** needs only a browser refresh — `state-minimap.js`'s guard-latch
   tooltip now shows `(MAX_DEVIATION, leg N)`.
4. **Optional 10-s check**: confirm the drives' live `input_mode` is
   passthrough while powered (read back via odrivetool or SDO read) —
   flagged during today's analysis; the saved JSON says `input_mode=5` but the
   live path requires passthrough semantics.

**⚡ 2026-07-16 evening update (after the raised-limit run validated: bag
`2026-07-16_17-38-15` ramped vel 100→200→280 mm/s, 22 moves, zero latches —
inventoried in `plans/parked/accel-ff-inertia.md`):**
the ODrive leg `vel_limit` is now **12.0 rev/s ≈ 846 mm/s** and the
`set_limits` ceilings are OPENED to **5000 / 5000 / 200000** (vel mm/s /
acc mm/s² / jerk mm/s³) — the ceilings are now *administrative*, deliberately
above the trackable envelope; the guard + deliberate session ramping are the
backstops. Deployment: **colcon build + relaunch only** (the 12.0 push reaches
the drives at the first homing; NO reflash — the guard/attribution flash from
the morning covers everything firmware-side). Know the real envelope while
ramping: **while the lead clamp is engaged** (which happens on every fast move
onset) catch-up velocity is capped by the firmware `VELFF` cap at `4.0 + 3.5 =
7.5 rev/s ≈ 529 mm/s`, so session vel limits much beyond **~450–500 mm/s**
will latch the guard on any sustained clamp engagement until the VELFF-cap
raise (a flash item) and/or the accel FF land. Between 200 and ~450 mm/s the
deviation scaling is **unmeasured** — extend the ladder in modest steps and
watch `live_deviation` against the 1.0 rev guard.

**New guard-fault attribution** (firmware-ground-truth only — Python never
re-thresholds `MAX_DEVIATION_REV`; it reads the firmware's frozen
`max_dev_leg`/`max_dev_value` snapshot off `HeartbeatT2J`, so the attribution
auto-tracks whatever threshold the firmware trips at):

- Edge: `Teensy guard FAULT LATCHED: fault_state=MAX_DEVIATION (leg 1,
  dev=-0.552 rev at trip) live_dev=[-0.55,-0.31,+0.34,+0.33,-0.12,+0.41] rev —
  leg output is now SUPPRESSED and every leg command (incl. DEACTIVATE, which
  returns ERR_BUS_DOWN) will be refused. Recover with: ros2 service call
  /clear_errors std_srvs/srv/Trigger`
- Persistent reminder (every 5.0 s): `TEENSY GUARD LATCHED (MAX_DEVIATION)
  (leg 1, dev=-0.552 rev at trip) — leg output suppressed; CLEAR_ERRORS
  required` (now carries the leg hint for **all** fault types, not just
  MAX_DEVIATION).
- New `/link_status` field `guard_fault_leg` — the culprit leg number as a
  string while a MAX_DEVIATION latch is **active**, else `''`. Resets to `''`
  after `/clear_errors` even though the raw `max_dev_leg` persists as "last
  latch since boot".

## Standing rules (every session)

- **You (Harrison) run every robot-actuating command.** Claude prepares the exact
  commands + PASS/ABORT criteria and verifies read-only (the probe, `/diagnose`, the
  bridge log). Claude does not run motion, arming, or the ramp/battery scripts.
- **Physical-intuition pushback is load-bearing** (per CLAUDE.md): *if your physical
  intuition disagrees with a framing here, say so before proceeding* — at session start
  and at every framing pivot. User intuition has repeatedly caught framing errors in
  this project's hardware investigations. A surprise is a stop-and-discuss, not a
  push-through.

## Global ABORT criteria & recovery

**ABORT any session immediately on:**

- any **E-STOP** (MPC_STALE / MAX_DEVIATION latch in the bridge/firmware log);
- any **oscillation** or audible snap, at a hold or in transit;
- tracking error > 0.1 rev **at a hold**. During a move the criterion is
  different (operator-confirmed 2026-07-16 — see the S4 ABORT recalibration
  note): peak `|live_deviation|` must stay under ~0.6 rev (60 % of the
  1.0 rev guard) **and** must collapse back under 0.1 rev once the platform
  settles at arrival. Any **MAX_DEVIATION latch** is itself an ABORT
  regardless of the peak value — stop the battery and review with the new
  per-leg fault log line (see the ⚡ 2026-07-16 banner above);
- any **unexplained bus fault** (marginal CAN3 is a known tier-2 quirk, but an
  *unexplained* bus fault during a session is an ABORT).

**Recovery basics:**

- A **latched fault** clears with `CLEAR_ERRORS` (the guard latches E-STOP until then).
  The latch **survives ROS relaunches** — the can-hub Teensy is powered from the
  Jetson 5V rail, so only `CLEAR_ERRORS` or a Teensy power-cycle (= a Jetson reboot)
  clears it. If a session ends with a latched guard, assume it is STILL latched at the
  next session until cleared (this trapped the 2026-07-09 S3 recovery attempt).
- **Disarm before deactivate** is now **bridge-enforced in-process** (ARMING_CONTRACT
  A3, 2026-07-15): `_run_deactivate` disarms first, on every entry point (orchestrator
  deactivate, direct `/deactivate`, shutdown stow). The firmware's
  reject-DEACTIVATE-while-armed gate remains as the backstop. A manual
  `set_setpoint_output false` before deactivate is harmless but no longer required.
- On ABORT: cut power / trigger the guard, then debrief before re-trying. For a limit
  ramp (S4), **revert the in-session `set_limits` to the last-good values** before
  retrying.

---

## ⚠ Sharp edges — read before any session

1. **Leaving a streaming mode while armed latches MPC_STALE within 250 ms.** The
   emitter stops publishing when you leave a streaming mode, so the bridge stops
   receiving frames and self-E-STOPs. **Structurally closed 2026-07-15**
   (ARMING_CONTRACT A3/A4): in the production flow the mode now stays published
   until the deactivate resolves, and the bridge disarms in-process before the
   stow — the deferred auto-disarm landed. The edge remains live only for
   `auto_arm:=false` manual sessions: there, always disarm before any control-mode
   change away from a streaming mode.

2. **Reload cancel after AIMING cannot recall the ball.** Once the reload action
   reaches AIMING, BB has committed the throw — cancelling the action does **not**
   recall the ball; a throw happens regardless. **Stay clear of the flight path.**

3. **The catch z-convention and QTM-frame mapping are hardware-UNVERIFIED.** The catch
   point `(0, 0, 809.08)` mm world (= STOW height 574.3 + STOW→ACTIVE lift 170.0 +
   centroid→cup-plane offset 64.78; the 809.08 cup plane, Q1 fix `bdbd186` — not the
   744.3 mm centroid) and the QTM-world vs jugglebot-base frame mapping are **unverified
   until S6 (7a) passes**. Do not throw a ball (7b/7c) before 7a confirms the aim
   geometry.

4. **Drive the orchestrator over `/orchestrator_command`, never the same-named bridge
   services.** `/home`, `/activate`, `/deactivate` are low-level `teensy_bridge_node`
   services; `orchestrator_node` serves none of them (it only *subscribes* to
   `/orchestrator_command`). Calling `/activate` directly (a) leaves the state machine
   in `IDLE` with `control_mode = ''`, which is not a streaming mode — so
   `trajectory_node`'s 40 Hz emitter **never publishes** and the probe reads `rate_hz 0`
   with :5557 bound and healthy; and (b) skips the `_run_configure` that the
   orchestrator's `/activate_or_deactivate` path folds in, leaving the legs in
   **TRAP_TRAJ** rather than POSITION/PASSTHROUGH. Mode changes (`standby` /
   `trajectory` / `spacemouse`) use the same topic. STANDBY is automatic on
   ACTIVE entry — no separate publish needed. (Cost the 2026-07-09 S1 session one
   false-negative probe run.)

5. **`ros2 topic pub --once` silently loses the message to the DDS discovery race.**
   `--once` creates a publisher, publishes immediately, and exits; FastRTPS needs
   ~100–500 ms to match the orchestrator's subscription, so the command frequently
   never arrives. This Foxy build has **no** `-w/--wait-matching-subscriptions` flag.
   **Always repeat-publish and always verify the mode took effect before arming:**
   ```bash
   ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'trajectory'"
   ```
   Confirm `Command received: trajectory` in the launch window (`orchestrator_node`
   logs every accepted command) **and** that `/control_mode_topic` reads `TRAJECTORY`,
   *before* `set_setpoint_output true`. Repeat publishes are safe: every mode command is
   idempotent and the handlers discard commands they don't recognise. The GUI's mode
   buttons (:8081) are immune — rosbridge holds a long-lived publisher. (On 2026-07-09
   a lost `trajectory` publish left the platform in STANDBY; the operator armed anyway,
   the whole battery came back `WRONG_MODE`, and the cleanup triggered Sharp Edge #6.)

6. **`deactivate` while ARMED — CLOSED 2026-07-15** (ARMING_CONTRACT A3/A4; kept for
   the historical record — observed 2026-07-09 at 13:29:47 during the S2 session).
   The old failure: ACTIVE→IDLE blanked `control_mode` instantly, the emitter
   stopped, MPC_STALE latched within 250 ms, and the firmware rejected the DEACTIVATE
   while `mpc_active=1` — orchestrator in IDLE, platform still standing, latched
   fault. Now: the bridge disarms in-process at the head of `_run_deactivate` (A3)
   and the streaming mode stays published until the deactivate resolves (A4 —
   IdleHandler blanks it afterwards), so neither half can happen in the production
   flow. If you ever see the old signature again, that is a contract violation —
   stop and investigate, don't work around it.

---

## Sessions (run in strict order)

### S1 — Phase-1 hold (arm + 120 s hold + clean disarm) — ✅ **PASS 2026-07-09**

- **Purpose**: prove the platform holds the ACTIVE pose through the new trajectory path;
  clean runtime arm and disarm→deactivate.
- **Entry**: powered, ODrives up, CAN3 healthy; `run_mpc.py` NOT running (sole :5557
  binder). Fresh `colcon build` + `source`.
- **Commands** (summary — full protocol in the pointer):
  1. `ros2 launch jugglebot jugglebot_launch.py enable_setpoint_output:=false`
  2. `ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'activate'"`
     (home first if `is_homed` is false). This lands in **STANDBY** automatically — see
     Sharp Edge #4; do **not** use the `/activate` service, and repeat-publish per Sharp
     Edge #5. Confirm the 40 Hz hold stream
     with the read-only probe `python tools/probes/traj_stream_probe.py --duration 30`
     (`rate_hz ≈ 40`, `u0_mean ≈ 2.19 rev`, `max_step ≈ 0`, `pump_rej = 0`).
  3. `ros2 service call /set_setpoint_output std_srvs/srv/SetBool "{data: true}"` (arm).
     *(Superseded 2026-07-15: automatic on ACTIVE entry under auto-arm — see the
     ARMING CONTRACT banner above. Needed only with `auto_arm:=false`.)*
  4. Hold 120 s.
  5. `trajectory/go_home` → `set_setpoint_output false` (disarm) → orchestrator
     `deactivate` → shutdown.
- **PASS**: 40 Hz hold before arm; **zero motion at the arm edge**; no rejects/faults
  over 120 s; leg drift < 0.02 rev; firmware **accepts DEACTIVATE** (proves `mpc_active`
  cleared).
- **ABORT**: any E-STOP, any visible motion at arm, pump-reject spam, drift > 0.02 rev.
- **Exit**: platform holds via the new path; clean disarm→deactivate.
- **Result (2026-07-09)**: **PASS on every criterion.** 40.03 Hz mean pre-arm / 40.02 Hz
  over the 120 s hold; `u0_mean` 2.19680 rev with zero spread; 120 s leg drift **0.0005 rev**
  (40× margin on the 0.02 limit); largest single-sample leg step 0.00172 rev across the
  whole 293 s armed window (no snap at the arm edge); zero pump rejects and an empty
  `last_rejection` all session; DEACTIVATE accepted and legs stowed cleanly. Emitter
  session-max gap 42.27 ms (vs the 250 ms staleness window). Artefacts + full table in
  `tests/hardware/session_phase1_hold.md` § Session result; rosbag
  `~/Desktop/rosbags/2026-07-09_12-51-08`.
- **Detailed protocol**: `tests/hardware/session_phase1_hold.md`.

### S2 — Phase-2 waypoint battery + loud-rejection demo — ✅ **PASS 2026-07-09**

- **Purpose**: profiled point-to-point moves execute smoothly; an infeasible request is
  loudly rejected with zero motion.
- **Entry**: S1 passed; armed and holding in **TRAJECTORY** mode; limits at the Phase-1
  defaults (100 mm/s, 400 mm/s², 8000 mm/s³ — pre-2026-07-17 defaults; a relaunch now
  gives the S4 working point, so `set_limits` down first for a faithful re-run).
- **Getting to the entry state** (mode change goes over the topic — Sharp Edges #4, #5):
  1. `ros2 launch jugglebot jugglebot_launch.py enable_setpoint_output:=false`
  2. `ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'activate'"`
     → ACTIVE:STANDBY, emitter streaming.
  3. `ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'trajectory'"`
     → TRAJECTORY. STANDBY→TRAJECTORY is streaming→streaming, so it is safe armed or
     unarmed; arming last keeps the irreversible step last.
  4. **VERIFY `/control_mode_topic` reads `TRAJECTORY` before arming.** A lost mode
     publish (Sharp Edge #5) is silent, and arming into STANDBY means every battery move
     returns `WRONG_MODE` — harmless in itself, but the armed cleanup that follows is how
     the 2026-07-09 session tripped an `MPC_STALE` E-STOP (Sharp Edge #6).
  5. `ros2 service call /set_setpoint_output std_srvs/srv/SetBool "{data: true}"` (arm).
     *(Superseded 2026-07-15: automatic on ACTIVE entry under auto-arm — see the
     ARMING CONTRACT banner above. Needed only with `auto_arm:=false`.)*
- **Battery** (`trajectory/go_to_pose`): z 170→220→170; x ±150; y ±150; tilt rx ±10°
  (widened 2026-07-16); then
  one deliberately-infeasible `duration_s: 0.05` request. **Do not hand-roll these** —
  `go_to_pose` takes one pose per call and returns `BUSY` if a move is already in flight
  (a deliberate Phase-2 restriction, lifted by the Phase 3/5 supersede work). The
  scripted battery `tests/hardware/traj_ramp_battery.py` fires exactly this list and
  sleeps `max(settle_s, planned_duration_s + 0.5)` between moves to avoid cascading
  `BUSY` rejections. It is named for Phase 4 but with no `--set-*` flag it changes no
  limits. ⚡ Since 2026-07-17 `--lean-gain` defaults to `-1.0` = defer to the shipped
  config (lean 0.6) — for S2's original lean-off conditions pass an EXPLICIT
  `--lean-gain 0.0` (the ±150 traverses are the guard-latch canary when unshaped —
  see `logbook/2026-07-17-wobble-latch-unshaped-traverse.md`):
  ```bash
  python3 tests/hardware/traj_ramp_battery.py --dry-run     # print the plan, no ROS calls
  python3 tests/hardware/traj_ramp_battery.py --lean-gain 0.0
  ```
- **Teardown**: `trajectory/go_home` → `set_setpoint_output false` (**disarm before
  leaving TRAJECTORY** — Sharp Edge #1) → orchestrator `deactivate`.
- **PASS**: each move subjectively smooth (no audible snap); `/diagnose --latest` shows
  leg jerk within limits; no pump rejects; no E-STOP. The infeasible request comes back
  `accepted=false code=TOO_FAST` with a populated `min_duration_s` and **moves nothing**.
  Target: **11/11** scripted moves clean (the `_BATTERY` list holds 11 feasible moves:
  2 in z, 3 in x, 3 in y, 3 in rx) + one demonstrated loud rejection.
- **ABORT**: oscillation, gate violation, tracking error > 0.1 rev at holds;
  during a move the recalibrated criterion applies (peak `|live_deviation|`
  under ~0.6 rev, collapsing back under 0.1 rev at arrival) and any
  MAX_DEVIATION latch is an ABORT regardless of the peak value — see the S4
  ABORT recalibration note.
- **Result (2026-07-09)**: **PASS — 11/11 moves clean + the loud rejection.** Battery ran
  13:34:07–13:34:43 at the Phase-1 default limits (100 mm/s, 400 mm/s², 8000 mm/s³),
  `lean_gain = 0.0`. Per-move realized leg peaks tracked the gate prediction closely
  (worst case across the 11: predicted vel 68.4 / acc 362.8 / jerk 6911 vs realized
  68.3 / 362.8 / 5583 mm·s⁻¹˒⁻²˒⁻³). Headroom against the session limits: **vel 68 %,
  acc 91 %, jerk 70 % (realized)** — acceleration is the tightest of the three at these
  defaults. The infeasible request returned
  `TOO_FAST: requested duration 0.050s < minimal feasible 0.629s` and **installed no
  plan** (`move_seq` held at 11 across it) — zero motion, as designed. Session-max
  emitter gap 56.60 ms (vs the 250 ms staleness window). Rosbag
  `~/Desktop/rosbags/2026-07-09_13-17-56`.
  - *Two NOTES from the bag, neither a PASS blocker.* (a) The teardown `go_home`
    installed as `move_seq=12` with realized peaks 0.0 (a genuine no-op from neutral —
    **that session ran with no levelling correction loaded; since
    `levelling-frame-contract` Phases 1–2 landed on 2026-07-26, `go_home` targets the
    *corrected* neutral, so with a correction loaded it is a real ~2.77 mm worst-leg
    move and realized peaks are NOT 0.0**. See `ros_ws/docs/levelling_frame.md`),
    but its **predicted** peaks were reported identical to move 11's rather than zero —
    i.e. `peak_leg_*` looks stale for a zero-distance plan. **DIAGNOSED 2026-07-26**
    (`logbook/2026-07-25-catch-reach-overshoot-repro.md`): `_svc_go_home` is one of six
    install paths that call `_install` — which bumps `move_seq` and resets the REALIZED
    peaks — without writing `_last_peak_*`, so the PREDICTED column describes the
    superseded plan. Enumerated and annotated at `trajectory_node.__init__` and at the
    `_publish_status` KeyValue block; **FIXED 2026-07-26** — `_install` now clears
    `_last_peak_*` and `_svc_go_to_pose`'s write moved to after its `_install`, so a
    report-less install publishes `0.0` rather than the superseded plan's peaks. On a
    post-2026-07-26 capture, read `peak_leg_* == 0.0` as "this install carried no
    prediction"; on an older one, read the predicted column only for a `move_seq` whose
    install carried a `FeasibilityReport`. (b) **`/link_status` is not
    in the launch's rosbag record list**, so the E-STOP that occurred at 13:29:47 is
    absent from the bag — the fault channel is invisible to post-hoc analysis. Adding it
    is a one-line launch change and would have made this session self-documenting.
- **Detailed protocol**: `plans/active/mvp-trajectory-bringup.md` § Phase 2 "Hardware
  session" (Phase 2 has no separate session file — its protocol lives in the plan). Use
  `tools/probes/traj_stream_probe.py` for read-only knot inspection.

### S3 — Phase-3 SpaceMouse flight (gentle / saturation / [unplug]) — ✅ **PASS 2026-07-10**

- **Purpose**: continuous target following is smooth; saturation and input-loss handled
  cleanly.
- **Entry**: S2 passed; mode `spacemouse` (over `/orchestrator_command`, Sharp Edges
  #4/#5); default low limits.
- **Sub-tests**: (a) gentle flight; (b) a hard-shove **saturation** test (expect: tracks
  to the workspace edge along the approach ray — stopping 0.5 mm inside the stroke
  bound by design — then a throttled "clamped to nearest reachable" WARN, no runaway);
  (c) a mid-flight **SpaceMouse disconnect** (expect: a smooth graceful stop / the
  SpaceMouse node's ACTIVE-pose hold on disconnect).
- **PASS**: subjectively smooth throughout, no rejects, clean disconnect.
- **ABORT**: any jerk event, E-STOP, runaway.
- **History**: the **2026-07-09 first attempt FAILED** — z-stutter (accept/reject limit
  cycling + firmware decay→sprint bursts to 2.5× the vel limit) ending in a latched
  MAX_DEVIATION E-STOP and a permanent follower lockup (commanded state parked exactly
  on the stroke bound). Root-caused and fixed by the chase-clamp rework (`73dba2b`);
  full post-mortem + fix disposition in
  `plans/archived/follower-cadence-and-divergence.md` § RESOLUTION.
- **Result (2026-07-10, post-rework)**: **PASS** — (a) and (b) smooth throughout
  ("worked perfectly"), no rejections, no E-STOP, both ascent and descent flown.
  Sub-test (c) **not performed**: the SpaceMouse connects over Bluetooth (the dongle
  turned out to be unnecessary), so a clean physical unplug isn't easily produced.
  Accepted without it: the input-loss path is unit-tested and exercised by the S3
  replay harness's end-of-stream stop, and SpaceMouse control is a test/fun mode, not
  a production dependency. (If a disconnect test is ever wanted: power the SpaceMouse
  off mid-flight, or kill the `spacemouse_handler` node — both drive the same
  input-loss → graceful-stop path.)
- **Detailed protocol**: `plans/active/mvp-trajectory-bringup.md` § Phase 3 "Hardware
  session" (no separate session file — protocol lives in the plan).

### S4 — Phase-4 limit ramp (multiple short sessions + one lean A/B)

- **Result (2026-07-16/17): ✅ PASSED — CLOSED.** The ladder was run far beyond the
  Phase-6 targets across the 2026-07-16 sessions: bag `18-45-29` ran the ladder to
  (1500,5000,40000) with **zero latches post-guard-raise and 0.359 rev peak
  deviation (36 % of the guard)** — see
  `logbook/2026-07-16-lean-planning-latency-and-boundary-step.md` Diagnosis §3.
  Caveat for the record: the limits A/B's **gain-0 arm** at (2000,5000,30000)
  (bag `21-58-59`) peaked at **0.94 rev = 94 % of the guard**; the lean-ON
  working point holds **≤ ~0.45 rev on the same traverse** (re-extraction of
  bag `22-06-30`; the earlier "~0.25 rev" figure was a narrower slice, not
  the traverse — see the wobble entry's reconciliation) — lean is part of WHY
  the working point has margin. ⚡ **2026-07-17 confirmed the caveat is a real
  cliff**: two bare battery runs (bag `17-35-14`) latched MAX_DEVIATION on the
  ±150 x-traverse at **1.02–1.08 rev** — same plans as the 0.94 run to the
  decimal (planner output unchanged); the +9–16 % on this move was ordinary
  day-to-day plant variation (matched moves elsewhere in the session ran
  +10–30 %; first-moves-cold, leg 1 deepest both trips). Both runs were unshaped
  because the battery script's old `--lean-gain` default (0.0 = explicit OFF)
  overrode the shipped lean — the default is now `-1.0` (defer to config).
  **Unshaped ±150 traverses at jerk 30000 are OUT of the trackable envelope,
  not marginal** — treat any unshaped hot-lateral latch as expected until the
  accel-FF chapter lands (`plans/parked/accel-ff-inertia.md`); see
  `logbook/2026-07-17-wobble-latch-unshaped-traverse.md`. The lean A/B
  resolved KEEP (`lean_gain 0.6` default; jerk criterion met at 0.3–0.6).
  **Working point (1000, 5000, 30000) + lean 0.6 persisted to YAML
  2026-07-17** (step-5 closure; see
  `logbook/2026-07-17-s4-closed-working-point-persisted.md`). The ladder
  protocol below is retained for reference / future re-ramps.
- **Purpose**: raise the session leg vel/acc/jerk limits from the Phase-1 defaults
  (100 mm/s, 400 mm/s², 8000 mm/s³) to the Phase-6 catch requirements, one small
  validated step at a time; resolve the lean A/B.
- **Ramp TARGETS** (from the Phase-6 reload gate, at 0.7 s lead / ≤80 mm reach / ≤12°
  tilt, 1.15× headroom): **leg vel ≈ 156 mm/s, acc ≈ 660 mm/s², jerk ≈ 10 331 mm/s³**.
  All three stay far inside the pre-2026-07-16 ceilings (280 / 4000 / 200 000;
  the ceilings are now administrative 5000/5000/200000 — see the ⚡ evening update).
- **What S4 is actually testing (post-rework framing)**: the software stack has already
  been validated through these limits and beyond — the chase-clamp sweep passed the
  production regime at the defaults, the S4 targets, AND the then-YAML-ceilings tier
  (280/4000/200000; 0 reject streaks, follow p99 6–11 ms — the sweep does NOT cover
  the opened administrative ceilings, so sessions above the old-ceilings tier are new
  territory for the chase gate too), and every plan is still individually gated. **S4 is a
  physical/mechanical validation**: vibration, resonance, audible harshness, ODrive
  tracking error, and how the platform *feels* at each step. Your senses are the
  instrument; the ABORT criteria are the guardrail.
- **Entry**: S2/S3 passed; armed + holding in **TRAJECTORY** mode — since the
  2026-07-15 arming contract that is just: launch → `activate` → `trajectory`
  (arming is automatic on ACTIVE entry; verify the "setpoint output ENABLED"
  banner); rosbag recording on; **know the last-good YAML session limits** so an
  ABORT reverts cleanly. Note: session limits are runtime state — **a relaunch always
  reverts to the YAML values**. ⚠ Since 2026-07-17 the YAML values ARE the hot S4
  working point (1000, 5000, 30000) with `lean_gain 0.6` — a relaunch is no longer a
  "revert to gentle" escape hatch. For a cautious probe session, `set_limits` DOWN
  explicitly (e.g. 100/400/8000) after launch.
- **New since 2026-07-16 — the ramp runs with the gravity FF live** (ships
  enabled; `logbook/2026-07-16-gravity-ff-armed.md`). Two watch items from the
  arming A/B: (1) **the first commanded move after any armed hold is the
  harshest** — in the FF-on session it briefly engaged the lead clamp on all
  six legs (`lead_clamp_mask` 63 for ~0.2 s) and peaked 7.35 A on one leg
  (vs ~3.5 A for the same move in the baseline). Isolated and self-recovering,
  but at RAISED limits watch the first move of every battery: a leg peaking
  above ~8 A ⇒ stop and keep the bag — **scoped to vel steps ≤ 130 mm/s**. At
  vel ≥ 160 mm/s lawful catch-up current alone reaches ~8.7 A (measured at
  vel=200 on 2026-07-16), so this rule would false-trip there; at those steps
  compare the first move's peak against the same move's peak in the previous
  battery instead of an absolute bar. A gentle wake-up move after arming
  (before the battery) is a legitimate mitigation. (2) Move-time
  `live_deviation` **rides the MAX_LEAD 0.10 rev ceiling by design** at these
  speeds — both A/B sessions brushed 0.09–0.11 rev at move onsets while
  looking and sounding completely clean, so a brief 0.1 rev excursion at onset
  is NOT by itself the ABORT signal (see the ABORT note below). The lead-clamp
  mask itself is release-based, not duration-based — see the ABORT note.
- **New since 2026-07-16 (later the same day) — the guard is now 1.0 rev and
  the ODrive leg vel_limit is now 12.0 rev/s** (6.0 at midday, 12.0 in the
  evening update; the `set_limits` ceilings are opened to 5000/5000/200000 —
  see the ⚡ 2026-07-16 banner + evening update near the top of this file for
  deployment status and the real trackable envelope, ~529 mm/s clamp-engaged
  until the VELFF-cap flash item lands). Expected peak transit
  `live_deviation` per the measured scaling law, for comparing against the
  live number (unmeasured above 200 mm/s — extend by modest steps):

  | session vel limit | expected peak transit `live_deviation` |
  |---|---|
  | 100 mm/s | ~0.17–0.22 rev |
  | 130 mm/s | ~0.3 rev |
  | 160 mm/s | ~0.41 rev |
  | 190–200 mm/s | ~0.52–0.56 rev |

  With `vel_limit` raised (6.0, then 12.0) the velocity loop has more catch-up
  headroom (`vel_ff` is no longer clipped to zero added authority once the
  clamp engages — see `canbridge_config.h`), which should mean *less* lag at
  a given speed, not more — but `vel_integrator_limit=Infinity`, so also
  watch the other direction: a harder catch-up sprint or arrival
  overshoot/oscillation that wasn't there before. If that appears where it
  didn't at the old vel_limit, note it and consider stepping back before the
  next ladder step.
- **New since 2026-07-16 (evening) — the lean A/B pause growth is EXPLAINED and
  two software fixes landed; re-run the lean A/B**
  (`logbook/2026-07-16-lean-planning-latency-and-boundary-step.md`). The
  operator's earlier lean run showed `--lean-gain 0.3` pausing much longer than
  `0.0` with a sharp tilt at each move onset. Both were confirmed root causes,
  not motion: **(1)** the pause is **planning COMPUTE** — a shaped plan runs 6
  feasibility passes vs 2 unshaped, and the Jacobian dominated them. Fixed
  (component-form Jacobian + 80-sample unshaped gate): offline per-move planning
  drops **unshaped ~0.73 → ~0.10 s** and **shaped ~2.6 → ~1.2 s**, so the
  matched-battery pause should fall from ~4.3 s toward **~2 s, now dominated by
  the platform settle**, not compute. **(2)** the sharp tilt was a boundary
  `vel_ff` STEP (~70–182 mm/s, 1.0–2.6 rev/s) emitted at every shaped-move
  boundary — it caused the +43 % peak-iq rise (5.93→8.48 A) seen at matched
  limits. Fixed with a C2 plateau window: the boundary step is now **0.00 mm/s**
  at both ends for all tiers, so the onset `iq` spikes should be gone. **Re-run
  the identical lean A/B after `colcon build --packages-select jugglebot` +
  relaunch** (a stale install keeps the old planner). **The open question for
  the re-run** is whether lean *improves* realized smoothness at matched limits:
  pre-fix, at matched limits, lean 0.3 made peak `iq` **worse** (that was the
  vel_ff step), so this is genuinely undecided — keep lean only if measured leg
  jerk drops AND the motion looks/sounds calmer.
- **⚡ 2026-07-17 update — shaped planning is now ~0.23 s, not ~1.2 s.** The
  "~0.10 s unshaped / ~1.2 s shaped" figures above were the 2026-07-16 state
  (component-form Jacobian + 80-sample unshaped gate only). The shaped-planning
  efficiency arc has since landed (`logbook/2026-07-17-shaped-planning-efficiency-implemented.md`):
  a batched 1600-sample shaped gate + a retiming-model duration search. ⚡ The
  retiming model shipped OFF the same evening after its first hardware A/B —
  its honest durations on lean traverses exceed the tracking envelope
  (dev 0.45 → 0.73 rev; `logbook/2026-07-17-retime-model-tracking-envelope.md`)
  — so **shaped `build_move` is ~0.23 s on the legacy loop + batched gate
  (~5× vs the ~1.1 s baseline; ~91 ms when the model is re-enabled), unshaped
  ~100 ms** —
  so between-move pauses are **settle-dominated for real now**, not
  planning-dominated (the earlier re-run's "pauses are planning-dominated after
  all" caveat is superseded). The gate also now measures shaped leg jerk ~7× more
  honestly (jerk-bound lateral moves plan ~5 % longer — the intended fix for the
  "sharp" lean, gentler not snappier). Deployment for this: same as everything
  else — `colcon build --packages-select jugglebot` + relaunch (Python-only). The
  `JB_TRAJ_RETIME_MODEL` YAML flag now ships **OFF**; to re-enable the model for
  an A/B: flip `retime_model` in `config/hardware_config.yaml`, run
  `python config/generate_config.py`, `colcon build`, relaunch.

#### The ladder (recommended order + step sizes)

One limit per step, ≤ ~1.3× per step, battery + review between steps. Rationale for
the order: **jerk first** (it is the binding *physical* constraint on this hardware —
the legs are bench-proven to 3.4 m/s velocity, jerk is what shakes the structure — and
its target is the smallest relative step, ×1.29, so it is also the gentlest opener);
**acceleration second, in two steps** (it was the tightest limit at S2 — realized peaks
used 91 % of the acc limit vs 68 %/70 % for vel/jerk — so acc steps produce the largest
visible change in move aggressiveness: watch for overshoot/ringing here); **velocity
last** (a vel raise alone changes little until acc/jerk allow faster transients; it
mainly shortens the longer strokes).

| step | limit | from → to | ratio | watch for |
|---|---|---|---|---|
| 1 | jerk | 8000 → **10 500** | ×1.31 | harshness / buzz at move start & end (jerk lives in the transitions). Durations barely shrink — moves are acc-bound at these limits; that is expected, not a null result. |
| 2 | acc | 400 → **520** | ×1.30 | overshoot / ringing at direction changes; ODrive tracking error. Planned durations visibly shrink. |
| 3 | acc | 520 → **660** | ×1.27 | same, harder. This is the step most likely to feel "snappy" — linger here. |
| 4 | vel | 100 → **130** | ×1.30 | long-stroke moves (the z moves) get faster mid-stroke; listen for anything speed-proportional (bearing noise, frame hum). |
| 5 | vel | 130 → **156** | ×1.20 | as step 4. After this step all three targets are met (10 500 ≥ 10 331). |

One step per short session is the conservative default; two steps in one session is
fine if the first felt completely clean — but never skip the battery+review between.

#### Per-step protocol (exact commands)

1. **Raise the one limit** — either let the battery do it (recommended; it calls
   `trajectory/set_limits` first and prints the applied values), e.g. step 1:

   ```bash
   python3 tests/hardware/traj_ramp_battery.py --set-jerk 10500 --lean-gain 0.0
   ```

   or set it explicitly and verify the echo before any motion:

   ```bash
   ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits \
     "{leg_vel_limit_mmps: 0.0, leg_acc_limit_mmps2: 0.0, leg_jerk_limit_mmps3: 10500.0}"
   ```

   `0` means "keep current"; every request is clamped to its YAML ceiling and the
   response echoes the **applied** values (`applied_*`) — read them back, don't assume.
   Subsequent steps: `--set-acc 520`, `--set-acc 660`, `--set-vel 130`, `--set-vel 156`.

2. **Run the battery** (11 profiled moves: z 170→220→170, x ±150, y ±150, rx ±10°
   since the 2026-07-16 widening, plus one
   deliberate `TOO_FAST` rejection; it sleeps between moves to avoid `BUSY`). Use
   `--dry-run` first if you want to see the plan without ROS calls. **Expected output
   per move**: `accepted=true`, a `planned_duration_s` that shrinks as the ramp
   progresses, and no pump-reject lines in the bridge log. The infeasible request must
   return `accepted=false code=TOO_FAST` with `min_duration_s > 0` and move nothing —
   its `min_duration_s` should also shrink step by step (the same move is achievable
   faster at higher limits).

3. **SpaceMouse sortie (OPTIONAL, recommended since the chase-clamp rework)** —
   60–90 s per step: since the arming contract, TRAJECTORY↔SPACEMOUSE is a
   streaming↔streaming mode change and is **safe while armed** — just publish
   mode `spacemouse` (Sharp Edge #5 repeat-publish + verify), fly gently, then
   a few full-deflection shoves, then back to `trajectory`. (The old
   disarm→mode→re-arm dance is no longer needed; it was over-conservative even
   then — Sharp Edge #1 concerned leaving the streaming SET, which SPACEMOUSE
   is inside.) This exercises the moving-seed regime the battery cannot.
   **Skipping the sorties is a legitimate trim** if SpaceMouse time is better
   spent — S3 already validated the follower at default limits; what a skipped
   sortie loses is only the moving-seed exercise at each RAISED limit step.
   **Expected**: `last_rejection` on `/trajectory/status` stays empty; no 1 Hz
   ERROR mentioning "escalation" in the trajectory_node log; the shove
   saturates smoothly at the workspace edge.

4. **Review** — `/diagnose --latest`: in the **Trajectory Moves** block the raised
   limit's realized peak should climb toward its new value while the other two keep
   comfortable headroom; cross-check tracking error and hold quiescence. From the
   rework's diagnostics, also glance at `follow_block_max_ms` (should stay ≲ 20 ms)
   and `chase_alpha`/`consecutive_rejects` (rejects should read 0).

5. **Operator PASS ⇒ persist** the bump: edit `config/hardware_config.yaml`
   `trajectory_op:` → `python config/generate_config.py` → stage the regenerated
   artifacts → `pytest tests/ -q` → commit **between sessions** (one commit per
   validated bump, `/diagnose` numbers in the message). **ABORT ⇒ revert** in-session
   (`set_limits` back to last-good, or relaunch to fall back to YAML); leave YAML
   unchanged.

- **Lean A/B** (once, not every step — do it at step 3's limits or later so lean has
  authority): run the identical battery at `--lean-gain 0.0` then `--lean-gain 0.3`,
  `/diagnose --compare`. **Keep lean only if** measured leg jerk drops AND the motion
  looks/sounds calmer; else leave `lean_gain: 0.0` and log the null result. **Expect the
  gain-0.3 arm's moves to run ~1.45× LONGER** — shaped lateral moves are legitimately
  slower because the gate sizes the added tilt; the battery prints each
  `planned_duration_s`, so the unequal durations are expected, not a regression. Watch
  for a tilt-rate tick at move start/end (the boundary transient) — **this should now be
  GONE** after the 2026-07-16 C2 plateau-window fix (boundary `vel_ff` step driven to
  0.00 mm/s; see the "New since 2026-07-16 (evening)" note above and
  `logbook/2026-07-16-lean-planning-latency-and-boundary-step.md`); if a tick is still
  present, report it rather than pushing through.
- **PASS/ABORT** per move: as S2 (smooth, jerk within limits, TOO_FAST rejects nothing;
  ABORT on oscillation / snap / E-STOP). **The "tracking error > 0.1 rev" ABORT line is
  recalibrated for moves (operator-confirmed 2026-07-16 — see
  `logbook/2026-07-16-max-deviation-guard-tracking-lag.md`)**: move-onset `live_deviation`
  legitimately rides the MAX_LEAD 0.10 rev clamp ceiling by design, and clean vel=100
  passes showed 0.17–0.22 rev transit deviation that the old rule would have flagged as
  an ABORT — it was never a runaway. **The confirmed rule**: 0.1 rev remains the ABORT
  threshold **at holds**, where it retains its original meaning; **during a move**, peak
  `|live_deviation|` must stay under ~0.6 rev (60 % of the new 1.0 rev guard) **and** must
  collapse back under 0.1 rev once the platform settles at arrival; any **MAX_DEVIATION**
  latch is itself an ABORT regardless of the peak value — stop the battery, keep the bag,
  and review with the firmware's per-leg fault log line (`fault_state=MAX_DEVIATION
  (leg N, dev=... rev at trip) live_dev=[...]`). On ABORT, revert and debrief before
  re-attempting — the failing step's `/diagnose` block + rosbag are the evidence. This
  criterion also appears in the Global ABORT list and at S2/S5 — the recalibration
  ripples to all three. **Lead-clamp mask**: mask stretches lengthen legitimately with
  velocity (the clamp engages whenever deviation > 0.10 rev, i.e. most of every fast
  move), so duration alone is not a signal — the confirmed rule is **release-based**: the
  mask must release by arrival; a mask still engaged **at the following hold** is an
  ABORT. The ~0.2 s onset engagements observed 2026-07-16 are normal. The
  first-move-after-armed-hold iq watch item (leg peaking above ~8 A ⇒ stop and keep the
  bag, above) is unchanged.
- **Detailed protocol**: `tests/hardware/session_phase4_ramp.md` (per-step mechanics;
  this section's ladder supersedes its "~1.5×, jerk 12000" example values).

### S4b — v3-firmware regression replay (was: "leg-gain retune") — **one run, at production gains**

> **⚠️ REWRITTEN 2026-07-13 — S4b is no longer a gain retune, and it no longer gates S4 on a
> retune converging.** The "under-damped position loop" framing below was **wrong**: the leg
> already tracks to **0.054 mm median / 0.192 mm worst at the catch instant** (operator spec:
> ±1 mm), and the measured cascade at production gains is healthy (`ω_v` ≈ 20 Hz / `ω_p` = 6.4 Hz,
> ratio ≈ 3.2). The "accuracy knee" that motivated a retune was a **~14× units bug**. **Production
> `40 / 0.20 / 0.32` stands; there is no winner to transfer and nothing to persist to YAML.**
> See `logbook/2026-07-13-leg-plant-id-and-the-units-bug.md`.

The 2026-07-10 battery latched `MAX_DEVIATION` on a fast vertical stroke (z 170→250 at
156/660/10500) with an audible ~6 Hz stutter. **Forensics traced it to a STRUCTURAL defect in the
interp path, not to the gains:** `MAX_LEAD = 0.15` gave `pos_gain × lead = 6.0 rev/s` — *above*
the 4.0 rev/s `vel_limit` — and `vel_ff` was discontinuously **zeroed at clamp engage**, a
bang-bang excitation whose frequency is set by the loop's own bandwidth (which is why it *looked*
like a gain problem: `≈ pos_gain/2π` = 6.4 Hz). **Both were fixed in v3 firmware** (`MAX_LEAD` →
0.10, `vel_ff` kept through the clamp), and the unloaded bench leg under v3 **cannot reproduce the
ring in any regime**.

**What S4b is now: one replay, at production gains, to confirm v3 fixed it on the LOADED robot.**
Re-run the S4 excitation that produced the limit cycle, record a bag, and re-run the 2026-07-10 S4
analysis pipeline. Start at low stroke amplitude.

- **PASS** — no 5.9–6.1 Hz / ~12.3 Hz spectral peak on any leg, no guard latch, hold-current
  ripple bounded ⇒ **the S4 chapter closes; resume the S4 ramp.** No gain work.
- **FAIL** — the ring survives v3 on the loaded robot ⇒ this is a **structural / regulation**
  finding, not a tracking one. Investigate the interp/clamp path and inter-leg coupling first. A
  gain change is a *last* resort, and the honest knob is **`vel_gain` up** (raises the inner loop
  and the cascade ratio), **not `pos_gain` up** (which *lowers* the ratio and marches the outer
  loop into the 15–19 Hz resonance).

Sessions **S5+ are unaffected** (they run at generous leads — the feasibility stretch keeps
realized motion slow — and, since 2026-07-17, at the validated S4 working point rather than
gentle limits; `set_limits` down for a deliberately gentle session). Commands + safety mechanics: `tests/hardware/session_gain_retune.md` (**read its
superseded banner first** — use its arming/abort/`/recover` procedure, ignore its gain sweep).
Background: `plans/active/leg-gain-tuning-methodology.md` § "Fast-motion tier (Level-2f)".

### S5 — Phase-5 timed targets (±25 ms arrival + supersede)

- **Purpose**: reach a pose a relative `lead_time_s` seconds after service receipt,
  within ±25 ms; too-tight timing loudly rejected; a mid-plan supersede replans C2
  (no snap). *(Interface change 2026-07-16: `TimedTarget.srv` takes `lead_time_s`
  — a typeable relative lead — not the old absolute `arrival_time`; rebuild
  `jugglebot_interfaces` + `jugglebot` and relaunch before running.)*
- **Entry**: armed + holding in **TRAJECTORY** mode; mocap recording the platform rigid
  body for arrival-time measurement.
- **Steps**: (1) generous-lead timed moves (z 170→185; x +15; y −15; rx ~2°;
  `lead_time_s` 2.5–4 s), repeat 5×; (2) a too-tight lead (`lead_time_s: 0.05`) →
  loud rejection; (3) a mid-plan **superseding** timed target while the first is
  in flight.
- **PASS**: (1) mocap-measured arrival within **±25 ms**, pose within 3 mm / 0.5°, smooth,
  no rejects/E-STOP; (2) `accepted=false code=TOO_FAST` with `min_duration_s`, **zero
  motion**; (3) the platform smoothly bends onto the second target (no snap at the
  supersede) and lands within ±25 ms, OR the second is loudly rejected and the first
  continues cleanly. (`go_to_pose` still returns `BUSY` mid-move by design — use
  `timed_target` for the supersede demo.)
- **ABORT**: any snap/jerk at supersede, motion on a rejected request, tracking error
  > 0.1 rev at holds; during a move the recalibrated criterion applies (peak
  `|live_deviation|` under ~0.6 rev, collapsing back under 0.1 rev at arrival)
  and any MAX_DEVIATION latch is an ABORT regardless of the peak value — see
  the S4 ABORT recalibration note.
- **Detailed protocol**: `tests/hardware/session_phase5_timed.md`.

### S6 = 7a — aim-only (frame + z-convention verification, NO ball, NO JB motion)

- **Purpose**: verify the QTM-world vs jugglebot-base frame convention AND the 809.08 mm
  cup-plane catch-z **before any ball flies**. This session answers Sharp Edge #3.
- **Entry**: S1–S5 clean; BB powered + calibrated (`bb/calibration_result` seen),
  heartbeat IDLE; mocap up on the platform rigid body AND the ball.
- **Command**: `bb/throw_at_target` with `use_target_point: true, aim_only: true,
  target_point_global_mm: {x: 0, y: 0, z: 809.08}, throw_delay_s: 0.0`.
- **PASS**: `success: true`; the returned yaw/pitch aim ray passes within BB's spatial
  calibration tolerance of (0, 0, 809.08) directly above the base; **zero balls, zero JB
  motion** (speed 0 ⇒ no reload action ⇒ latch never raised ⇒ hand never armed).
- **ABORT**: the aim points anywhere other than above the base → the frame or
  z-convention is wrong; **do not proceed to 7b**, debrief the frame math. Record any
  fixed offset — the catch-point computation (`reload_sequencer.compute_catch_point_mm`)
  or a frame transform needs correction before any ball flies.
- **Detailed protocol**: `tests/hardware/session_phase7_reload.md` § Stage 7a.

### S7 = 7b — static catch in TRAJECTORY-hold (hand armed via the catch/armed latch)

- **Purpose**: prove a ball seats in the cup with the hand armed by the existing
  coordinator, WITHOUT platform tilt/translation.
- **How static catch works without CATCH mode**: the hand's prime/arm is gated on the
  `catch/armed` latch and the platform tilt is gated on `trajectory/arm_catch` — **two
  separate gates**. For a static catch, publish `catch/armed = true` (arms + primes the
  hand via `catch_coordinator_node`) while leaving `trajectory/arm_catch` **down**, so
  `trajectory_node` ignores `catch/dynamic_target` and the platform holds. This is the
  exact isolation the old TRAJECTORY-mode 7b relied on, expressed through the new latch
  (bench-only manual override — see `session_phase7_reload.md` § 7b).
- **Entry**: 7a passed; hold the neutral ACTIVE catch pose in **TRAJECTORY** mode
  (streaming); publish `/catch/armed` true; confirm the hand primes once `/balls` shows a
  ball and the platform stays still.
- **Command**: a single dead-centre throw — `bb/throw_at_target` with
  `use_target_point: true, aim_only: false, target_name: 'jugglebot',
  target_point_global_mm: {x: 0, y: 0, z: 809.08}, throw_delay_s: 3.0`. (`target_name:
  'jugglebot'` is required — without it the announcement's `target_id` defaults to
  `point` and the whole catch pipeline drops the ball.) Lower `/catch/armed` after.
- **PASS**: ball seated; hand telemetry matches the armed catch profile; `/balls` reports
  `CAUGHT`. `/diagnose --latest` after each throw.
- **ABORT**: **two consecutive bounce-outs ⇒ ABORT the stage, capture the hand traces +
  rosbag, and return to Phase 6 (sim) with the hardware traces** — the sim gate has no
  contact-quality criterion; this is the operative hardware guard. Also ABORT on any
  E-STOP or visible platform jerk. (Hardware has caught smoothly before — priors are
  good.)
- **Detailed protocol**: `tests/hardware/session_phase7_reload.md` § Stage 7b.

### S8 = 7c — full reload action (translate + tilt catch)

- **Purpose**: the whole sequence through `jugglebot/reload` — preconditions, proactive
  prime + latch raise, aim+throw at the 809.08 cup plane, announcement→tilt-through-seat
  catch, confirmation, recenter; all abort paths exercised.
- **Entry**: 7b passed; **TRAJECTORY** mode, armed and streaming a hold (the action owns
  the latch — no CATCH mode); session limits ramped to the S4 targets (≥ 156 / 660 /
  10 331).
- **Command**: `ros2 action send_goal /jugglebot/reload jugglebot_interfaces/action/Reload
  "{throw_delay_s: 3.0}" --feedback`; watch `CHECKING (dwells up to 10 s while BB
  reloads if the hand is empty — RELOADING is BB's heartbeat state, not a feedback
  phase) → AIMING → THROW_PENDING → BALL_IN_FLIGHT → CATCHING → SETTLING → result`.
- **PASS**: **≥ 3/5 reloads return `outcome: CAUGHT`** with a logged `catch_error_mm`;
  motion subjectively smooth; no pump rejects; no E-STOP. Receive tilt is clamped to 12°
  — arrivals more off-vertical than ~12° are still CAUGHT with only partial collinear
  seating (the hand absorbs the residual).
- **Abort paths — exercise each once, deliberately**: no-ball reject
  (`REJECTED_NO_BALL`, zero motion, nothing armed — lands before PREPARE);
  announcement-timeout abort with BB disabled mid-sequence
  (`ABORTED_NO_ANNOUNCEMENT` within `throw_delay + 0.5 s`; PREPARE has run, so SAFE_ABORT
  retracts the hand + re-centers); wrong-mode reject — send the goal while NOT in a
  streaming TRAJECTORY hold (e.g. STANDBY) → (`REJECTED_WRONG_MODE`, zero motion).
  **Remember
  Sharp Edge #2: cancel after AIMING does not recall the ball — stay clear of the flight
  path.**
- **ABORT**: two consecutive bounce-outs (back to Phase 6); any E-STOP; a gate-rejected
  catch target surfacing as `MISSED_INFEASIBLE_<code>` on more than one in five (revisit
  the reach envelope / limits).
- **Exit**: `jugglebot/reload` reliably catches — MVP goal 4 complete.
- **Detailed protocol**: `tests/hardware/session_phase7_reload.md` § Stage 7c.

---

## Data capture (every motion session)

- **Rosbags auto-record** the trajectory topics — `/trajectory/status`,
  `/trajectory/diagnostics`, `/trajectory/target_feedback` are in the launch record list.
  Launch with recording on for S2–S8. Since the chase-clamp rework, the diagnostics also
  carry `chase_alpha` (last per-tick feasible-progress fraction), `consecutive_rejects`
  (should read 0), `escalation_stop` (should read false), and `follow_block_max_ms`
  (post-publish planning cost — should stay ≲ 20 ms).
- **`/diagnose --latest` after every motion session** — read the Trajectory Moves block
  (realized + `used_pct_predicted` peaks/headroom) and tracking/hold-quiescence plots.
  For S4, `/diagnose --compare` the two lean-A/B sessions.
- **Note session results against each checklist** here and in the corresponding phase
  logbook's Verification section. If any ABORT fired, open an `/investigate` before
  re-attempting.

## Open items the bench must answer

Carried from the phase open-questions (identical to the closing logbook entry's list):

1. **Catch z-convention (809.08 mm cup plane) + QTM frame** — verified at **S6 (7a)**; if
   7a needs a correction, fix `reload_sequencer.compute_catch_point_mm` (or the
   `HAND_CATCH_OFFSET_MM` term) and re-run the software gate.
2. **Vel-match criterion redefinition** — the ≤15 %-at-first-contact metric is
   inconsistent with the 0.6 hand design (a designed ~40 % first-contact mismatch that
   absorbs over the stroke); redefine it with 7b/7c evidence, not sim. On poor seating
   the 0.6 hand design — not the sim metric — is the reference; hardware evidence gates
   any hand-config change.
3. **Reach envelope vs offsets** — the sim flagged 89/92 mm reach on two nominal trials
   (> the ≤80 mm reliable envelope); tighten the offset spec or widen the envelope with
   S8 (7c) evidence.
4. **Tilt clamp behaviour beyond 12° collinearity** — real arrivals are 18–40° off
   vertical, now clamped at 12°; watch partial-tilt catches in 7c (a steep off-centre
   bounce-out is a seating-margin signal, not a gate reject).
5. **Hand-telemetry CAUGHT cross-check** — MVP `CAUGHT` is a tracker-id-correlated
   in-flight estimate, not a settled rest position; implement the hand-telemetry rest
   cross-check **only if 7c shows false CAUGHTs**.
6. **Emitter jitter p95** — read it from the DEBUG install-latency logs across the motion
   sessions (S2–S8) to confirm the 40 Hz emitter stays well inside the 250 ms staleness
   window under load. Since the chase-clamp rework the binding budget is better observed
   directly: `max_emit_gap_ms` (should stay near 25 ms — the S3 incident showed the true
   contract is the 25 ms knot cadence, not the 250 ms staleness window) and
   `follow_block_max_ms` on `/trajectory/diagnostics`.
7. **Diagnostics leftovers from the S3 post-mortem** (`follower-cadence-and-divergence.md`
   § 4.5): realized peaks on the SPACEMOUSE / reactive-catch path are still per-install
   (≈ per-tick) rather than rolling-window, and `peak_leg_*` looks stale for zero-distance
   plans. The realized-peaks half is still open. The `peak_leg_*` half is **diagnosed**
   (2026-07-26, `logbook/2026-07-25-catch-reach-overshoot-repro.md`) and **not yet
   fixed**: six install paths bump `move_seq` without writing `_last_peak_*`, so the
   PREDICTED column can describe a superseded plan; annotated at
   `trajectory_node.__init__`, fix deferred as its own commit because it reorders a
   safety-adjacent install path. Matters if S4's `/diagnose` review is ever run on a
   spacemouse sortie rather than the battery.
   **The `peak_leg_*` half is now FIXED (2026-07-26).** `_install` clears
   `_last_peak_*` alongside the realized peaks and every report-carrying install writes
   after it (`_svc_go_to_pose`'s write moved), so a report-less install publishes `0.0`
   — "no prediction for this plan" — instead of the superseded plan's numbers. Bench
   check: `tests/hardware/session_anomaly_fixes.md` § CHECK CCATCH-5. The
   realized-peaks-are-per-install half remains open.
8. **Jolt-fix regression check (deferred until bench-leg testing completes — operator is
   currently rigged for the bench leg)**: on the next powered Jugglebot sitting,
   deliberately latch the guard, then armed `/clear_errors` — the pre-fix ~2 rev/s /
   9 A kick must now be a bounded ≤ 1 rev/s slew (firmware re-enable slew, flashed
   2026-07-11, commit f218acf).
9. **GUI websocket drop 2026-07-11 13:40 — recorded as an unexplained one-off.**
   Operator context kills both leading hypotheses (browser foregrounded, wired-Ethernet
   Win10 desktop); the second identical guard fault did not reproduce it. The landed
   keepalive-ping + staleness-reconnect fixes (f218acf) make any recurrence self-heal in
   ≤ ~40 s without a refresh; if one is observed, rosbridge client connect/disconnect
   lines now land in the launch log for diagnosis. No further action planned.
