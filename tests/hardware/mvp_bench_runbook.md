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
  `plans/active/follower-cadence-and-divergence.md` § RESOLUTION.
- Full suite (`pytest tests/ -q`, 2026-07-10, post-rework) = **2304 passed, 5 skipped,
  1 xfailed in 582.32 s**. (Pre-rework 2026-07-08 baseline: 2274 passed, 1 xfailed;
  ci-deep 2026-07-08 green.)
- Before running: `colcon build --packages-select jugglebot jugglebot_interfaces`,
  `source install/setup.bash`. No code changes should be needed for any session below.

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
- any **oscillation**, audible snap, or tracking error > 0.1 rev;
- any **unexplained bus fault** (marginal CAN3 is a known tier-2 quirk, but an
  *unexplained* bus fault during a session is an ABORT).

**Recovery basics:**

- A **latched fault** clears with `CLEAR_ERRORS` (the guard latches E-STOP until then).
  The latch **survives ROS relaunches** — the can-hub Teensy is powered from the
  Jetson 5V rail, so only `CLEAR_ERRORS` or a Teensy power-cycle (= a Jetson reboot)
  clears it. If a session ends with a latched guard, assume it is STILL latched at the
  next session until cleared (this trapped the 2026-07-09 S3 recovery attempt).
- **Disarm before deactivate** is **firmware-enforced** — the firmware rejects
  DEACTIVATE while `mpc_active=1`, so the clean order is always
  `set_setpoint_output false` (disarm) → orchestrator `deactivate`.
- On ABORT: cut power / trigger the guard, then debrief before re-trying. For a limit
  ramp (S4), **revert the in-session `set_limits` to the last-good values** before
  retrying.

---

## ⚠ Sharp edges — read before any session

1. **Leaving a streaming mode while armed latches MPC_STALE within 250 ms.** The
   emitter stops publishing when you leave a streaming mode, so the bridge stops
   receiving frames and self-E-STOPs. **Always disarm (`set_setpoint_output false`)
   before any control-mode change away from a streaming mode.** (A structural
   auto-disarm on mode-exit is a Deferred item — the operator sequence is the guard.)

2. **Reload cancel after AIMING cannot recall the ball.** Once the reload action
   reaches AIMING, BB has committed the throw — cancelling the action does **not**
   recall the ball; a throw happens regardless. **Stay clear of the flight path.**

3. **The catch z-convention and QTM-frame mapping are hardware-UNVERIFIED.** The catch
   point `(0, 0, 744.3)` mm world (= STOW height 574.3 + STOW→ACTIVE lift 170.0) and the
   QTM-world vs jugglebot-base frame mapping are **unverified until S6 (7a) passes**.
   Do not throw a ball (7b/7c) before 7a confirms the aim geometry.

4. **Drive the orchestrator over `/orchestrator_command`, never the same-named bridge
   services.** `/home`, `/activate`, `/deactivate` are low-level `teensy_bridge_node`
   services; `orchestrator_node` serves none of them (it only *subscribes* to
   `/orchestrator_command`). Calling `/activate` directly (a) leaves the state machine
   in `IDLE` with `control_mode = ''`, which is not a streaming mode — so
   `trajectory_node`'s 40 Hz emitter **never publishes** and the probe reads `rate_hz 0`
   with :5557 bound and healthy; and (b) skips the `_run_configure` that the
   orchestrator's `/activate_or_deactivate` path folds in, leaving the legs in
   **TRAP_TRAJ** rather than POSITION/PASSTHROUGH. Mode changes (`standby` /
   `trajectory` / `spacemouse` / `catch`) use the same topic. STANDBY is automatic on
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

6. **`deactivate` while ARMED latches `MPC_STALE` *and* leaves the legs un-stowed.**
   The state-machine transition ACTIVE→IDLE is pure software and happens instantly, so
   `control_mode` becomes `''` and the emitter stops — the guard latches `MPC_STALE`
   within 250 ms. But the firmware **rejects** the DEACTIVATE while `mpc_active=1`, so
   the legs never profile-stow: you end with the orchestrator in IDLE, the platform
   still at the ACTIVE pose, and a latched fault. Recover with
   `ros2 service call /clear_errors std_srvs/srv/Trigger` (a bridge service — the
   orchestrator only routes `'clear_errors'` from its FAULT state, `state_machine.py`
   `FaultHandler`, so a topic publish from IDLE is silently discarded), then re-activate.
   **Always `set_setpoint_output false` before `deactivate`.** (Observed 2026-07-09 at
   13:29:47 during the S2 session.)

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
  defaults (100 mm/s, 400 mm/s², 8000 mm/s³).
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
- **Battery** (`trajectory/go_to_pose`): z 170→190→170; x ±20; y ±20; tilt rx ±3°; then
  one deliberately-infeasible `duration_s: 0.05` request. **Do not hand-roll these** —
  `go_to_pose` takes one pose per call and returns `BUSY` if a move is already in flight
  (a deliberate Phase-2 restriction, lifted by the Phase 3/5 supersede work). The
  scripted battery `tests/hardware/traj_ramp_battery.py` fires exactly this list and
  sleeps `max(settle_s, planned_duration_s + 0.5)` between moves to avoid cascading
  `BUSY` rejections. It is named for Phase 4 but with no `--set-*` flag it changes no
  limits, and `--lean-gain` defaults to `0.0` (lean off) — precisely S2's conditions:
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
- **ABORT**: oscillation, gate violation, tracking error > 0.1 rev.
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
    installed as `move_seq=12` with realized peaks 0.0 (a genuine no-op from neutral),
    but its **predicted** peaks were reported identical to move 11's rather than zero —
    i.e. `peak_leg_*` looks stale for a zero-distance plan. Worth a look before S4 leans
    on `/diagnose`'s predicted-vs-realized headroom numbers. (b) **`/link_status` is not
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
  `plans/active/follower-cadence-and-divergence.md` § RESOLUTION.
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

- **Purpose**: raise the session leg vel/acc/jerk limits from the Phase-1 defaults
  (100 mm/s, 400 mm/s², 8000 mm/s³) to the Phase-6 catch requirements, one small
  validated step at a time; resolve the lean A/B.
- **Ramp TARGETS** (from the Phase-6 reload gate, at 0.7 s lead / ≤80 mm reach / ≤12°
  tilt, 1.15× headroom): **leg vel ≈ 156 mm/s, acc ≈ 660 mm/s², jerk ≈ 10 331 mm/s³**.
  All three stay far inside the YAML ceilings (280 / 4000 / 200 000).
- **What S4 is actually testing (post-rework framing)**: the software stack has already
  been validated through these limits and beyond — the chase-clamp sweep passed the
  production regime at the defaults, the S4 targets, AND the YAML ceilings (0 reject
  streaks, follow p99 6–11 ms), and every plan is still individually gated. **S4 is a
  physical/mechanical validation**: vibration, resonance, audible harshness, ODrive
  tracking error, and how the platform *feels* at each step. Your senses are the
  instrument; the ABORT criteria are the guardrail.
- **Entry**: S2/S3 passed; armed + holding in **TRAJECTORY** mode (same arm sequence as
  S2 steps 1–5); rosbag recording on; **know the last-good YAML session limits** so an
  ABORT reverts cleanly. Note: session limits are runtime state — **a relaunch always
  reverts to the YAML values**, so a relaunch is also a valid "revert everything".

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

2. **Run the battery** (11 profiled moves: z 170→190→170, x ±20, y ±20, rx ±3°, plus one
   deliberate `TOO_FAST` rejection; it sleeps between moves to avoid `BUSY`). Use
   `--dry-run` first if you want to see the plan without ROS calls. **Expected output
   per move**: `accepted=true`, a `planned_duration_s` that shrinks as the ramp
   progresses, and no pump-reject lines in the bridge log. The infeasible request must
   return `accepted=false code=TOO_FAST` with `min_duration_s > 0` and move nothing —
   its `min_duration_s` should also shrink step by step (the same move is achievable
   faster at higher limits).

3. **SpaceMouse sortie (new, recommended since the chase-clamp rework)** — 60–90 s per
   step: disarm-free mode change is NOT allowed (Sharp Edge #1 — disarm first if leaving
   TRAJECTORY), so: `set_setpoint_output false` → mode `spacemouse` (Sharp Edge #5
   repeat-publish + verify) → re-arm → fly gently, then a few full-deflection shoves.
   This exercises the moving-seed regime the battery cannot. **Expected**:
   `last_rejection` on `/trajectory/status` stays empty; no 1 Hz ERROR mentioning
   "escalation" in the trajectory_node log; the shove saturates smoothly at the
   workspace edge. Then disarm → back to `trajectory` → re-arm for the next step.

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
  for a tilt-rate tick at move start/end (the boundary transient) — report it rather
  than pushing through.
- **PASS/ABORT** per move: as S2 (smooth, jerk within limits, TOO_FAST rejects nothing;
  ABORT on oscillation / snap / tracking error > 0.1 rev / E-STOP). On ABORT, revert
  and debrief before re-attempting — the failing step's `/diagnose` block + rosbag are
  the evidence.
- **Detailed protocol**: `tests/hardware/session_phase4_ramp.md` (per-step mechanics;
  this section's ladder supersedes its "~1.5×, jerk 12000" example values).

### S4b — Leg-gain retune (root fix for the 2026-07-10 ~6 Hz stutter) — **gates further S4**

The 2026-07-10 battery latched `MAX_DEVIATION` on a fast vertical stroke (z 170→250 at
156/660/10500) with an audible ~6 Hz stutter. Forensics traced it to the leg velocity loop
being tuned only for quiet HOLD (`40 / 0.20 / 0.32`, the methodology's Level-1 tier), not for
fast-motion tracking: the under-damped position loop rings at its own bandwidth, the ODrive
current surges/brakes, and the accumulated command-vs-encoder lead trips the 0.5-rev guard.
The stutter is worst at the raised limits, so **the S4 ramp cannot proceed above the levels
that provoke it until the leg-gain fast-motion tier converges** — running S4 harder first just
re-triggers the latch. Do S4b now (a `pos_gain` sweep to separate control-loop from structural
resonance, then a `vel_gain` damping A/B), persist the winning gains to YAML, then resume S4
from the last clean step. Sessions **S5+ are unaffected** by this gate (they run at generous
leads / gentle limits, below the stutter regime). Detailed protocol + exact commands:
`tests/hardware/session_gain_retune.md`; methodology tier:
`plans/active/leg-gain-tuning-methodology.md` § "Fast-motion tier (Level-2f)".

### S5 — Phase-5 timed targets (±25 ms arrival + supersede)

- **Purpose**: reach a pose at an absolute arrival time within ±25 ms; too-tight timing
  loudly rejected; a mid-plan supersede replans C2 (no snap).
- **Entry**: armed + holding in **TRAJECTORY** mode; mocap recording the platform rigid
  body for arrival-time measurement.
- **Steps**: (1) generous-lead timed moves (z 170→185; x +15; y −15; rx ~2°; lead
  2.5–4 s), repeat 5×; (2) a too-tight lead (0.05 s ahead) → loud rejection; (3) a
  mid-plan **superseding** timed target while the first is in flight.
- **PASS**: (1) mocap-measured arrival within **±25 ms**, pose within 3 mm / 0.5°, smooth,
  no rejects/E-STOP; (2) `accepted=false code=TOO_FAST` with `min_duration_s`, **zero
  motion**; (3) the platform smoothly bends onto the second target (no snap at the
  supersede) and lands within ±25 ms, OR the second is loudly rejected and the first
  continues cleanly. (`go_to_pose` still returns `BUSY` mid-move by design — use
  `timed_target` for the supersede demo.)
- **ABORT**: any snap/jerk at supersede, motion on a rejected request, tracking error
  > 0.1 rev.
- **Detailed protocol**: `tests/hardware/session_phase5_timed.md`.

### S6 = 7a — aim-only (frame + z-convention verification, NO ball, NO JB motion)

- **Purpose**: verify the QTM-world vs jugglebot-base frame convention AND the 744.3 mm
  catch-z **before any ball flies**. This session answers Sharp Edge #3.
- **Entry**: S1–S5 clean; BB powered + calibrated (`bb/calibration_result` seen),
  heartbeat IDLE; mocap up on the platform rigid body AND the ball.
- **Command**: `bb/throw_at_target` with `use_target_point: true, aim_only: true,
  target_point_global_mm: {x: 0, y: 0, z: 744.3}, throw_delay_s: 0.0`.
- **PASS**: `success: true`; the returned yaw/pitch aim ray passes within BB's spatial
  calibration tolerance of (0, 0, 744.3) directly above the base; **zero balls, zero JB
  motion** (speed 0 ⇒ no announcement ⇒ hand never armed).
- **ABORT**: the aim points anywhere other than above the base → the frame or
  z-convention is wrong; **do not proceed to 7b**, debrief the frame math. Record any
  fixed offset — the catch-point computation (`reload_sequencer.compute_catch_point_mm`)
  or a frame transform needs correction before any ball flies.
- **Detailed protocol**: `tests/hardware/session_phase7_reload.md` § Stage 7a.

### S7 = 7b — static catch in TRAJECTORY-hold mode

- **Purpose**: prove a ball seats in the cup with the hand armed by the existing
  coordinator, WITHOUT platform tilt/translation.
- **Why TRAJECTORY, not CATCH**: in CATCH mode the same coordinator path that arms the
  hand also publishes `catch/dynamic_target`, which `trajectory_node` turns into a
  *moving* catch — so a static platform + coordinator-armed hand is impossible in CATCH.
  In TRAJECTORY mode `trajectory_node` ignores `catch/dynamic_target` (CATCH-gated) so
  the platform holds, while the coordinator's `/balls` handler (not mode-gated) still
  primes + arms the hand on the first catchable ball. That is the genuine static-catch
  config.
- **Entry**: 7a passed; hold the neutral ACTIVE catch pose in **TRAJECTORY** mode
  (streaming); confirm the hand primes once `/balls` shows a ball.
- **Command**: a single dead-centre throw — `bb/throw_at_target` with
  `use_target_point: true, aim_only: false, target_name: 'jugglebot',
  target_point_global_mm: {x: 0, y: 0, z: 744.3}, throw_delay_s: 3.0`. (`target_name:
  'jugglebot'` is required — without it the announcement's `target_id` defaults to
  `point` and the whole catch pipeline drops the ball.)
- **PASS**: ball seated; hand telemetry matches the armed catch profile; `/balls` reports
  `CAUGHT`. `/diagnose --latest` after each throw.
- **ABORT**: **two consecutive bounce-outs ⇒ ABORT the stage, capture the hand traces +
  rosbag, and return to Phase 6 (sim) with the hardware traces** — the sim gate has no
  contact-quality criterion; this is the operative hardware guard. Also ABORT on any
  E-STOP or visible platform jerk. (Hardware has caught smoothly before — priors are
  good.)
- **Detailed protocol**: `tests/hardware/session_phase7_reload.md` § Stage 7b.

### S8 = 7c — full reload action (translate + tilt catch)

- **Purpose**: the whole sequence through `jugglebot/reload` — preconditions, aim+throw,
  announcement→tilt-through-seat catch, confirmation; all abort paths exercised.
- **Entry**: 7b passed; control mode **CATCH**; session limits ramped to the S4 targets
  (≥ 156 / 660 / 10 331).
- **Command**: `ros2 action send_goal /jugglebot/reload jugglebot_interfaces/action/Reload
  "{throw_delay_s: 3.0}" --feedback`; watch `CHECKING (dwells up to 10 s while BB
  reloads if the hand is empty — RELOADING is BB's heartbeat state, not a feedback
  phase) → AIMING → THROW_PENDING → BALL_IN_FLIGHT → CATCHING → SETTLING → result`.
- **PASS**: **≥ 3/5 reloads return `outcome: CAUGHT`** with a logged `catch_error_mm`;
  motion subjectively smooth; no pump rejects; no E-STOP. Receive tilt is clamped to 12°
  — arrivals more off-vertical than ~12° are still CAUGHT with only partial collinear
  seating (the hand absorbs the residual).
- **Abort paths — exercise each once, deliberately**: no-ball reject
  (`REJECTED_NO_BALL`, zero motion); announcement-timeout abort with BB disabled
  mid-sequence (`ABORTED_NO_ANNOUNCEMENT` within `throw_delay + 0.5 s`, platform holds,
  hand never armed); wrong-mode reject (`REJECTED_WRONG_MODE`, zero motion). **Remember
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

1. **Catch z-convention (744.3 mm) + QTM frame** — verified at **S6 (7a)**; if 7a needs a
   correction, fix `reload_sequencer.compute_catch_point_mm` and re-run the software gate.
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
   § 4.5): realized peaks in SPACEMOUSE/CATCH are still per-install (≈ per-tick) rather
   than rolling-window, and `peak_leg_*` looks stale for zero-distance plans — both still
   open; matters if S4's `/diagnose` review is ever run on a spacemouse sortie rather
   than the battery.
