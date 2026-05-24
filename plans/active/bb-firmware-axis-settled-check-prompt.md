# Prompt for fresh Claude session — BB firmware: yaw + pitch settled check before throw

Copy the content below into a fresh `claude` session.  It's self-
contained: the receiving Claude has no memory of the prior conversation
that surfaced this task.  Goal is one focused PR on the BB firmware repo
that adds an axis-settled check to `executeThrow_`.

---

## Prompt

I need you to fix a Ball-Butler firmware bug.  Background, the bug, the
constraints, and the acceptance criteria are all below.  Work on the
**BallButler** repo (the standalone repo at `/home/jetson/Desktop/BallButler`),
not the Jugglebot repo.  After flashing, hardware validation requires
running the trimmed cone-test launch in the Jugglebot repo
(`ros2 launch jugglebot catching_cone_test.launch.py`) and observing throws.

### Background

Ball Butler is a Teensy 4.x microcontroller-driven robot with three
axes — yaw (PWM-controlled by a custom PID loop in
`ball_butler_main/YawAxis.cpp`), pitch (ODrive trap-traj, configured by
`ball_butler_main/PitchAxis.cpp`), and a hand (ODrive, runs a planned
trajectory for the actual throw).  Host computes (yaw, pitch, speed,
absolute throw_wall_us) and sends them via CAN; firmware queues the
throw via `StateMachine::requestThrow` and eventually executes it via
`StateMachine::executeThrow_`.

### The bug

[`StateMachine.cpp:826-894`](ball_butler_main/StateMachine.cpp#L826)
(`executeThrow_`) does this:

1. Commands yaw + pitch to their targets (motors start moving).
2. Plans the hand trajectory.
3. Checks lead time against the **hand wind-up time only**:
   ```cpp
   const float required_lead_us = (-min_ts + OpCfg::SCHEDULE_MARGIN_S) * 1e6f;
   const int64_t actual_lead_us = (int64_t)pending_throw_wall_us_ - (int64_t)can_.wallTimeUs();
   if ((float)actual_lead_us < required_lead_us) { return false; }
   ```
4. Arms the streamer.

There is no check that yaw + pitch will be **at their target by the time
the hand fires the ball**.  The pitch trap-traj is configured at
`traj_vel_rps: 1.0, traj_accel_rps2: 0.5` (`hardware_config.yaml:533-538`
in the Jugglebot repo).  For a 60° pitch change:
- accel phase covers `(0.5 * accel * t²)` for `t = sqrt(2 * (1/6) / 0.5) ≈ 0.82 s`
- decel phase symmetric → **~1.6 s total** for a 60° move
- ~1.15 s for 30°, ~0.67 s for 10°

If the hand wind-up only needs 600 ms and the host commands a throw with
1.0 s lead, the firmware accepts the throw, the hand fires at +600 ms,
but pitch is still mid-traverse → ball releases at a wrong angle.  Every
throw with a different aim is non-deterministic in landing direction.

Operator confirmed the symptom on hardware: "BB tends to throw very
early; the pitch and yaw axes are still finding their positions by the
time the hand throws."

### What I need you to add

Inside `executeThrow_`, after step 4 (planning the hand trajectory) and
before step 6 (arming the streamer), add an **axis-settled lead check**:

```cpp
// 4b) Verify yaw + pitch will be settled by throw_wall_us
const float pitch_settle_us = estimatePitchTraverseTimeUs(
    PRO.getPitchDeg(), pitch_deg);
const float yaw_settle_us = estimateYawTraverseTimeUs(
    PRO.getYawDeg(), yaw_deg);
const float axis_required_lead_us =
    std::max(pitch_settle_us, yaw_settle_us)
    + OpCfg::SCHEDULE_MARGIN_S * 1e6f;
const int64_t axis_actual_lead = (int64_t)pending_throw_wall_us_ - (int64_t)can_.wallTimeUs();
if ((float)axis_actual_lead < axis_required_lead_us) {
    debugf_("[SM] Throw rejected: yaw/pitch settle needs %.0f ms, have %.0f ms\n",
            (double)(axis_required_lead_us / 1000.0f),
            (double)(axis_actual_lead / 1000.0f));
    return false;
}
```

The two helper functions are the substance of the work:

**`estimatePitchTraverseTimeUs(current_deg, target_deg)` — pitch is the
clean case.**  ODrive trap-traj with known `vel_max, accel_max`.
Standard kinematics:
- `|Δθ|` in revolutions
- If triangular (never reaches v_max): `t = 2 * sqrt(|Δθ| / accel)`
- If trapezoidal: `t = 2 * v_max / accel + (|Δθ| - v_max² / accel) / v_max`
- Threshold: triangular if `|Δθ| ≤ v_max² / accel`

The values are available in code — look at `PitchAxis.h`/`PitchAxis.cpp`
for whatever struct/field holds `traj_vel_rps` and `traj_accel_rps2`
(these are passed from the Teensy's config, originally sourced from
`hardware_config.yaml`).

**`estimateYawTraverseTimeUs(current_deg, target_deg)` — yaw is harder.**
Yaw is PWM-controlled with a custom PID loop (`YawAxis.cpp`), not an
ODrive trap-traj.  No clean v_max + accel.  Options, easiest first:

1. **Conservative constant rate.**  Pick a worst-case "deg per second"
   that the operator can experimentally validate (e.g. 60°/s →
   `t_us = |Δyaw_deg| / 60.0 * 1e6 + margin`).  Crude but works.
2. **Empirical map.**  Bench-measure yaw traversal times for several
   step sizes (10°, 30°, 60°, 90°), fit a curve.  Hardcoded in C++ as
   a small lookup or polynomial.
3. **Read from yaw's own state.**  If `YawAxis` has a `bool isMoving()`
   or similar, we could use that — but `executeThrow_` is called once
   *at queue time*, so it'd need a predictive estimate, not an observed
   "is currently moving."  Option 1 or 2 is right.

Default to option 1 (conservative rate, configurable, lean towards
safe-rejection).  Make the rate a value in `BallButlerConfig.h` so the
operator can tune.

### Constraints

* **Single function modified**: `executeThrow_`.  Two helpers added
  (one each for pitch + yaw traversal estimation).  Don't touch the
  state machine, don't touch `requestThrow`, don't restructure the
  lead-time check that already exists for the hand.
* **Behaviour when rejection happens**: same as the existing hand
  lead-time rejection.  The throw rejection logs to debug + returns
  `false` → state stays IDLE/TRACKING, no error state.
* **No new public API** unless absolutely necessary.  Helpers can be
  private members or anonymous-namespace free functions in
  `StateMachine.cpp`.
* **Margin**: reuse `OpCfg::SCHEDULE_MARGIN_S` (the same margin used
  for the hand check).  Don't introduce a second margin constant
  unless there's a concrete reason to make them different.
* **No firmware-side mocap or QTM dependency.**  Estimation must use
  only data the Teensy has access to: current axis position (from
  proprioception), commanded target, and configured limits.

### Files you'll touch (BallButler repo)

* `ball_butler_main/StateMachine.cpp` — add the check in
  `executeThrow_`, define the two estimators.
* `ball_butler_main/StateMachine.h` — declare the estimators if you
  make them private members.
* `ball_butler_main/BallButlerConfig.h` — add yaw conservative-rate
  constant if you go with option 1 (recommended).

Probably nothing else.  Definitely **not** the
`HandPathPlanner`, `HandTrajectoryStreamer`, or `Trajectory` files —
the hand trajectory and its wind-up timing are out of scope.

### Acceptance criteria

After your change, on hardware (operator will run this — describe the
checks clearly in your PR / commit body so they can validate):

1. **No regression**: a throw with current pitch already at target
   should accept unchanged (lead-time check is dominated by hand
   wind-up, as before).
2. **Settled-gate works**: a throw requesting a +60° pitch change
   with only 1.0 s lead time should be **rejected** with the new
   "yaw/pitch settle needs N ms, have M ms" log.
3. **Same throw with longer lead accepts**: same +60° pitch with 3.0 s
   lead should accept.
4. **Yaw constant-rate gate**: a throw requesting a +90° yaw change
   with too-short lead is rejected.
5. **Combined**: pitch + yaw both changing — the gate uses
   `max(pitch_settle, yaw_settle)`.  Verify by commanding a small
   pitch change + large yaw change and a large pitch change + small
   yaw change; both should be rejected when lead is too short.

### Repo conventions worth knowing

* The Teensy build uses PlatformIO (`platformio.ini` in
  `ball_butler_main/`).  The receiving operator can flash.
* This codebase doesn't have unit tests for firmware — validation is
  via hardware behaviour.  That's fine; don't add testing
  infrastructure as part of this PR.
* Match the existing debug-log style in `StateMachine.cpp` —
  `debugf_("[SM] ...")` with the same level of detail as the hand
  rejection.
* Don't write or modify markdown docs.  This is a code-only PR.

### Output I need from you

A single PR on the BallButler repo with:
1. The code change (one function modified, two helpers added,
   possibly one config constant).
2. A commit message that includes:
   - the symptom (throws fire before axes settle),
   - the kinematics summary for pitch (trap-traj, ~1.6 s for 60°),
   - the yaw rate-of-thumb you chose + why,
   - the per-acceptance-criteria validation plan the operator should
     run on hardware.
3. **No** logbook entry or other documentation files — that's the
   Jugglebot-repo's responsibility, not the BallButler firmware repo.

Start by reading
`/home/jetson/Desktop/BallButler/ball_butler_main/StateMachine.cpp`
(particularly `executeThrow_` and `requestThrow`),
`PitchAxis.cpp`/`.h`, `YawAxis.cpp`/`.h`, and `BallButlerConfig.h` —
then propose your design before writing code.

---

## Notes for the Jugglebot-side operator

After this firmware change lands and is flashed:

* The host-side `_DEFAULT_THROW_DELAY_S = 2.5` bump from commit
  `35bc12c` can probably be lowered again (the firmware will now
  reject premature throws, so the host can be optimistic) — but only
  lower it after a full hardware-validation pass.  Leave it at 2.5 s
  until you've confirmed the firmware gate works in practice.
* Any throw rejected by the firmware will show up in the debug log as
  "[SM] Throw rejected: yaw/pitch settle needs N ms, have M ms" — the
  Jugglebot GUI doesn't currently surface that (BB heartbeat doesn't
  echo throw-rejection reasons).  Worth a follow-up: have can_node
  log BB throw-rejection events when it sees the heartbeat field that
  indicates the last command was rejected, if such a field exists.
