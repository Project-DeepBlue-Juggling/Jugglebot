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
- Full suite (`pytest tests/ -q`, 2026-07-08) = **2274 passed, 1 xfailed in 553.60 s**.
- ci-deep (`pytest tests/ -q --hypothesis-profile=ci-deep`, 2026-07-08) = **2274
  passed, 1 xfailed, 198 warnings in 3024.70 s** — green.
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

---

## Sessions (run in strict order)

### S1 — Phase-1 hold (arm + 120 s hold + clean disarm)

- **Purpose**: prove the platform holds the ACTIVE pose through the new trajectory path;
  clean runtime arm and disarm→deactivate.
- **Entry**: powered, ODrives up, CAN3 healthy; `run_mpc.py` NOT running (sole :5557
  binder). Fresh `colcon build` + `source`.
- **Commands** (summary — full protocol in the pointer):
  1. `ros2 launch jugglebot jugglebot_launch.py enable_setpoint_output:=false`
  2. home → activate → control mode **STANDBY**; confirm the 40 Hz hold stream with the
     read-only probe `python tools/probes/traj_stream_probe.py --duration 30`
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
- **Detailed protocol**: `tests/hardware/session_phase1_hold.md`.

### S2 — Phase-2 waypoint battery + loud-rejection demo

- **Purpose**: profiled point-to-point moves execute smoothly; an infeasible request is
  loudly rejected with zero motion.
- **Entry**: S1 passed; armed and holding in **TRAJECTORY** mode; limits at the Phase-1
  defaults (100 mm/s, 400 mm/s², 8000 mm/s³).
- **Battery** (`trajectory/go_to_pose`): z 170→190→170; x ±20; y ±20; tilt rx ±3°; then
  one deliberately-infeasible `duration_s: 0.05` request.
- **PASS**: each move subjectively smooth (no audible snap); `/diagnose --latest` shows
  leg jerk within limits; no pump rejects; no E-STOP. The infeasible request comes back
  `accepted=false code=TOO_FAST` with a populated `min_duration_s` and **moves nothing**.
  Target: 10/10 scripted moves clean + one demonstrated loud rejection.
- **ABORT**: oscillation, gate violation, tracking error > 0.1 rev.
- **Detailed protocol**: `plans/active/mvp-trajectory-bringup.md` § Phase 2 "Hardware
  session" (Phase 2 has no separate session file — its protocol lives in the plan). Use
  `tools/probes/traj_stream_probe.py` for read-only knot inspection.

### S3 — Phase-3 SpaceMouse flight (gentle / saturation / unplug)

- **Purpose**: continuous target following is smooth; saturation and input-loss handled
  cleanly.
- **Entry**: S2 passed; mode `spacemouse`; default low limits.
- **Sub-tests**: (a) gentle flight; (b) a hard-shove **saturation** test (expect: tracks
  to the workspace edge along the approach ray, then a throttled "clamped to nearest
  reachable" WARN, no runaway); (c) a mid-flight **SpaceMouse unplug** (expect: a smooth
  graceful stop / the SpaceMouse node's ACTIVE-pose hold on disconnect).
- **PASS**: subjectively smooth throughout, no rejects, clean disconnect.
- **ABORT**: any jerk event, E-STOP, runaway.
- **Detailed protocol**: `plans/active/mvp-trajectory-bringup.md` § Phase 3 "Hardware
  session" (no separate session file — protocol lives in the plan).

### S4 — Phase-4 limit ramp (multiple short sessions + one lean A/B)

- **Purpose**: raise the session leg vel/acc/jerk limits toward the Phase-6 catch
  requirements, one small validated step per session; resolve the lean A/B.
- **Entry**: S2/S3 passed; armed + holding in **TRAJECTORY** mode; rosbag recording on
  (`/trajectory/diagnostics` + `/trajectory/status` are in the record list); **know the
  last-good YAML session limits** so an ABORT reverts cleanly.
- **Per-step protocol** (repeat once per limit bump):
  1. Raise **ONE** limit ~1.5× at runtime via `trajectory/set_limits` (do NOT edit YAML
     yet — a bad value is one service call to undo; jerk is the binding constraint, raise
     it first).
  2. Run the operator battery `python3 tests/hardware/traj_ramp_battery.py --lean-gain 0.0`.
  3. `/diagnose --latest` review — read the **Trajectory Moves** block (realized peaks +
     `used_pct_predicted` headroom; the raised limit's realized peak should climb toward
     it, keep comfortable headroom on the other two).
  4. **Operator PASS ⇒ persist** the bump: edit `config/hardware_config.yaml`
     `trajectory_op:` → `python config/generate_config.py` → stage → `pytest tests/ -q`
     → commit **between sessions** (one commit per validated bump, `/diagnose` numbers in
     the message). **ABORT ⇒ revert** the in-session `set_limits` to last-good; leave YAML
     unchanged.
- **Lean A/B** (once, not every step): run the identical battery at `--lean-gain 0.0`
  then `--lean-gain 0.3`, `/diagnose --compare`. **Keep lean only if** measured leg jerk
  drops AND the motion looks/sounds calmer; else leave `lean_gain: 0.0` and log the null
  result. **Expect the gain-0.3 arm's moves to run ~1.45× LONGER** — shaped lateral moves
  are legitimately slower because the gate sizes the added tilt; the battery prints each
  `planned_duration_s`, so the unequal durations are expected, not a regression. Watch for
  a tilt-rate tick at move start/end (the boundary transient) — report it rather than
  pushing through.
- **Ramp TARGETS** (from the Phase-6 reload gate, at 0.7 s lead / ≤80 mm reach / ≤12°
  tilt, 1.15× headroom): **leg vel ≈ 156 mm/s, acc ≈ 660 mm/s², jerk ≈ 10 331 mm/s³**.
  **All three targets exceed the Phase-1 defaults (100 / 400 / 8000) and must be ramped
  past them before S8**; jerk is the binding constraint and the largest relative step —
  raise it first. All three stay well inside the YAML ceilings (280 / 4000 / 200 000).
- **PASS/ABORT** per move: as S2 (smooth, jerk within limits, TOO_FAST rejects nothing;
  ABORT on oscillation / snap / tracking error > 0.1 rev / E-STOP).
- **Detailed protocol**: `tests/hardware/session_phase4_ramp.md`.

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
  Launch with recording on for S2–S8.
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
   window under load.
