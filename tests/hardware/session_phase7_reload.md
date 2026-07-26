# Hardware Sessions — MVP Phase 7: BB→Jugglebot Reload (staged 7a / 7b / 7c)

**Plan**: `plans/active/mvp-trajectory-bringup.md` § Phase 7
**Logbook**: `logbook/2026-07-08-mvp-phase7-reload-action.md`,
`logbook/2026-07-20-reload-action-catch-latch.md` (the action-driven reframe),
`logbook/2026-07-23-phase7-reload-first-hardware-session.md` (first session's findings + fixes)
**Goal**: Ball Butler aims at Jugglebot's ACTIVE **cup-plane** catch point and throws;
the **`jugglebot/reload` action** drives the whole catch from ACTIVE — it proactively
primes the hand, raises the **catch-armed latch** for the flight window (which lets the
reactive tilt actuate the platform), the hand catches, then it re-centers. Exposed as
`jugglebot/reload` (Reload.action).

This validates **software already merged** on branch `mvp-trajectory-bringup`. No code
changes should be needed to run it. The stages are gated: **do not run 7b before 7a
passes, or 7c before 7b passes.**

**Stage arc at a glance:**
| Stage | What it proves | Ball? | Platform moves? |
|-------|----------------|-------|-----------------|
| **7a** aim-only | the 809.08 cup-plane frame/z convention (BB aims where the hand catches) | no | no |
| **7b** static catch | the hand seats a dead-centre ball with the platform held still | yes | no |
| **7c** full reload | the `jugglebot/reload` action: prime → throw → reactive tilt-catch → recenter, + each abort path | yes | yes |

---

> **⚡ FIFTH SITTING — after the 2026-07-24 open-loop pivot**
> (`logbook/2026-07-24-phase7-fourth-sitting-openloop-telemetry-ladders.md`. The
> fourth sitting: 19 real throws, ~15 caught; 2 BB-side refusals — PV_STALE + another
> yaw NOT_SETTLED — and one FALSE prime abort.) What changed for THIS run:
> 1. **The reload platform is OPEN-LOOP now** (your call; config
>    `reload_platform_open_loop: true`): once the throw is announced, the platform
>    takes the pre-tilt pose and HOLDS it — live ball tracking can no longer move it
>    (the Ball-30 "platform moved at the last second" miss is dead). The hand-stroke
>    TIMING stays tracker-driven (announced landing runs ~0.05–0.16 s early, plus a
>    ~+0.31 s outlier mode ≈ once per sitting — timing the stroke from it would
>    bounce balls). Manual 7b bench arming is unaffected.
> 2. **No more up-down prime jerk / false PRIME aborts**: the hand ladders now read
>    hand telemetry after a failed ack — a "failed" dispatch that is actually moving
>    the hand is accepted instead of re-dispatched (the ack channel lies ~59 % of
>    calls). If you still see a mid-ascent yank, that is NEW evidence.
> 3. **The launch shell now tells the truth**: ONE outcome line per reload
>    ("Reload CAUGHT/MISSED/..."); the arm-failed and IMPLAUSIBLE spam is demoted
>    (DEBUG/INFO). WARNs you still see are meaningful — note them.
> 4. **Expectations for the miss rate**: this round removes only the self-inflicted
>    platform-move class. Late arrivals (one ball arrived 0.31 s late — 0.18 s after
>    the hand had already retracted) and BB scatter (one landed 105 mm off-centre)
>    remain — those are the ball-held-sensor era's work (arrival-triggered stroke)
>    and/or BB aim tightening.
> 5. **Verdicts still read MISSED for real catches** (tracker corruption stands;
>    0/21 CAUGHT verdicts last sitting despite ~15 catches). Judge by eye; the
>    ball-held sensor is the fix.
> 6. **BB-side watch items**: two throws refused in one sitting (PV_STALE, yaw
>    NOT_SETTLED — the second yaw abort in two sittings). If a goal ends with no
>    ball flying, check the bridge log for these before suspecting Jugglebot.
> **Before this session**: `colcon build --packages-select jugglebot` +
> `source install/setup.bash` + relaunch (no interface changes this round), and
> power-cycle the can-bridge Teensy as usual.

> **⚡ FOURTH SITTING — after the 2026-07-23 evening fixes**
> (`logbook/2026-07-23-phase7-third-sitting-verdicts-stutter-pretilt.md`. The third
> sitting WORKED: 10 real throws at scale 0.8, **8 caught**; 2 "throws" were aborted
> by BB itself — yaw `THROW_ABORTED_NOT_SETTLED` in the bridge log — and never flew.)
> What changed for THIS run:
> 1. **`catch_vel_scale` defaults to 0.8 now** (config `JB_OP_CATCH_VEL_SCALE_DEFAULT`).
>    `ros2 action send_goal /jugglebot/reload jugglebot_interfaces/action/Reload
>    "{throw_delay_s: 3.0}" --feedback` gets 0.8; pass the field only to override.
> 2. **GUI RELOAD button** (STATE MACHINE panel, purple; **hard-refresh the browser**
>    Ctrl-Shift-R to pick it up). Enabled when connected + ACTIVE; one click = one
>    reload goal at the defaults. Outcome appears in the orchestrator log; the button
>    cools down ~4 s.
> 3. **The platform now arrives at the receive tilt ~1.5 s BEFORE landing** (the third
>    sitting's pre-tilt was scheduled to arrive exactly AT landing — tilt was still
>    >1° off until ~0.3 s before contact). Visible check: the platform should be
>    seated and STILL while the ball is in the air. Tilt-only-settling-at-contact =
>    stale build.
> 4. **The prime ascent should be one smooth continuous move** — the third sitting's
>    stutter (5/12 ascents) was our own 0.5 s retry tick restarting the ascent after
>    a failed dispatch ack. If you still see a mid-ascent stall, say so — that would
>    be NEW evidence, not the known mechanism.
> 5. **Verdicts**: `MISSED_INFEASIBLE_*` now genuinely means "the platform never had
>    a reachable catch pose" (all 12 third-sitting goals falsely read
>    MISSED_INFEASIBLE_WORKSPACE off corrupt-track rejects). Expect caught balls to
>    read `MISSED` while the tracker corruption stands — **keep judging seating by
>    eye/hand telemetry**, and keep counting "IMPLAUSIBLE" warnings for the tracker
>    investigation.
> 6. **Watch for `hand prime dispatch failed (attempt N/4)` warnings**: the hand
>    dispatch path failed ~40–60 % per call last session even on a fresh reboot (a
>    bridge-Teensy CAN TX issue, investigation queued — NOT the uptime-lag regime).
>    The new 4-attempt ladders (prime + safe-abort retract) should ride it out; note
>    the warning frequency — it is the epidemic's cheapest gauge. An
>    `ABORTED_PRIME_FAILED` despite the ladder escalates that investigation.
> 7. BB aborting a throw (`THROW_ABORTED_NOT_SETTLED`, bridge log) currently rides to
>    a `MISSED` verdict — known cosmetic; the yaw-settle issue itself is BB-side.
> **Before this session**: `colcon build --packages-select jugglebot_interfaces
> jugglebot` + `source install/setup.bash` + relaunch (four nodes + the orchestrator
> changed), hard-refresh the GUI, and power-cycle the can-bridge Teensy as usual (the
> uptime-lag reboot isolation experiment is still outstanding).

> **⚡ THIRD SITTING — after the 2026-07-23 afternoon fixes**
> (`logbook/2026-07-23-phase7-retest-stroke-race-tracker-corruption.md`; the morning
> fixes are in `…-phase7-reload-first-hardware-session.md` and validated — the
> platform reached + tilted on every throw of the re-test). What changed for THIS run:
> 1. **The catch stroke should now fire on EVERY attempt.** The re-test lost 3/6
>    strokes to a same-tick re-prime racing the catch arm on the Teensy's single
>    trajectory queue; no smooth-move is ever sent while a catch sequence is live
>    now.
> 2. **The platform pre-tilts during the countdown** (new): right after the throw is
>    accepted you should see the platform settle into the receive tilt ~3 s BEFORE
>    the ball flies (predicted from the announcement), then refine slightly
>    mid-flight. A platform that only starts tilting at release = stale build.
> 3. **`catch_vel_scale` knob** (new, your tuning request):
>    `ros2 action send_goal /jugglebot/reload jugglebot_interfaces/action/Reload
>    "{throw_delay_s: 3.0, catch_vel_scale: 0.8}" --feedback` — scales the hand's
>    catch speed (effective hand/ball ratio = 0.6 × scale; clamped [0.3, 1.5];
>    resets to 1.0 each attempt). For manual 7b throws publish `catch/vel_scale`
>    (std_msgs/Float64) after arming. Suggested sweep: 1.0 → 0.8 → 0.6 → 1.2,
>    one throw each, judge seating.
> 4. **CAUGHT verdicts are honest now**: the re-test's 5 spurious `CAUGHT` results
>    came from corrupt tracker tracks coasting below the floor; those are rejected
>    (watch for "tracker CAUGHT … is IMPLAUSIBLE" warnings — each one is more
>    evidence for the OPEN tracker investigation). Expect `MISSED` on a bounce-out —
>    **and note that while the corruption stands, a REAL catch may ALSO read
>    `MISSED`** (the latched announced track is the corrupt one, so its CAUGHT is
>    rightly rejected). During the vel-scale sweep, judge seating **by eye and hand
>    telemetry, not by the action verdict**.
> 5. A transient first-attempt prime failure now retries once instead of aborting.
> **Before this session**: `colcon build --packages-select jugglebot_interfaces
> jugglebot` (the action gained a goal field) + `source install/setup.bash` +
> relaunch; and **POWER-CYCLE THE CAN-BRIDGE TEENSY** — it was at 67.6 h uptime
> during the re-test, deep in the known lag regime; the reboot isolation experiment
> is STILL outstanding and any catch-timing tuning on a degraded bridge may not
> transfer.

> **⚡ Action-driven reload, 2026-07-20 (no CATCH mode)**: the catch is no longer
> gated by a persistent **CATCH** control mode the operator holds. That mode was
> retired (`logbook/2026-07-20-reload-action-catch-latch.md`); the whole reactive
> catch is now driven by the **`jugglebot/reload` action** for its duration via a
> **catch-armed latch** on `trajectory_node` (`trajectory/arm_catch`, mirrored on the
> `catch/armed` topic that gates the hand). The reload runs from **ACTIVE + streaming a
> hold in TRAJECTORY** (armed) — it never switches control mode. Anywhere an older
> protocol said "set / keep CATCH mode", the answer is now: stay in **TRAJECTORY**,
> armed, and let the action own the latch.

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
- **Two platform-command sources during a reload** — the action's `go_home` recenter and
  the reactive catch (`catch/dynamic_target` → `build_catch`). They are **temporally
  disjoint by construction**: the catch-armed latch is raised only for the flight window
  (PREPARE → terminal), and `go_home` runs only at the terminal *after* the latch is
  lowered. The Phase-1 arm/disarm-edge graceful stop smooths both seams (seeded C2 off
  the live commanded state — no command discontinuity). Watch for any overlap; a platform
  move while the latch is still raised is a stop-and-debrief.
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
  activate → confirm the 40 Hz hold stream → `trajectory` mode → verify
  `mpc_active=1`, **zero motion at arm**. The reload action's precondition is
  **ACTIVE + streaming a hold in TRAJECTORY** (armed) — it rejects `WRONG_MODE` if the
  control mode is anything but a streaming TRAJECTORY hold, and `NOT_STREAMING` if the
  emitter is not running.

### Pre-flight — confirm the freshly-built code is live

These are **src-only** commits, so the running graph must be the one you just rebuilt
(`colcon build --packages-select jugglebot jugglebot_interfaces` + `source install/setup.bash`
— done). Launch runs the *installed* copy, so confirm the new interfaces are actually up
**before any ball flies**:

```bash
ros2 action list  | grep jugglebot/reload       # the reload action is served
ros2 service list | grep trajectory/arm_catch    # the catch-armed latch service exists
ros2 topic info   /catch/armed                    # the hand-gate topic exists
```

- **Expected**: all three present. If `trajectory/arm_catch` or `/catch/armed` is missing, the
  launch is still running a **stale install** — rebuild + relaunch before proceeding (this is the
  #1 way the session silently tests the old code).
- The retired **CATCH** / **SHELL** modes are gone: the GUI state-minimap no longer offers them and
  an `orchestrator_command` of `catch`/`shell` is silently ignored. There is nothing to "set" — the
  reload action owns the catch from TRAJECTORY.

---

## Stage 7a — aim-only (frame-convention verification, NO ball, NO platform motion)

**Purpose**: verify the QTM-world vs jugglebot-base frame convention AND the **cup-plane**
catch-point z-convention
(`GEOM_INITIAL_HEIGHT_MM + JB_OP_DEFAULT_ACTIVE_Z_MM + HAND_CATCH_OFFSET_MM` =
574.3 + 170.0 + 64.78 = **809.08 mm world**) **before any ball flies**. The reload
coordinator aims BB at exactly this point (the cup plane where the hand intercepts the
ball, **not** the platform centroid at 744.3 mm — that was the Q1 aim-low bug, fixed in
`bdbd186`); here we command it directly, speed 0.

Catch point (world frame, mm): **x = 0, y = 0, z = 809.08**.

```bash
ros2 service call /bb/throw_at_target jugglebot_interfaces/srv/BallButlerThrow \
  "{use_target_point: true, aim_only: true, \
    target_point_global_mm: {x: 0.0, y: 0.0, z: 809.08}, throw_delay_s: 0.0}"
```

- **PASS**: `success: true`; the returned `yaw_rad` / `pitch_rad` point BB at the space
  directly above Jugglebot's base at ~809 mm (the cup plane); measured against mocap, the
  aim ray passes within BB's spatial calibration tolerance of (0, 0, 809.08). **No ball
  leaves** (speed 0), **no platform motion** (no reload action running ⇒ latch never
  raised ⇒ hand never armed).
- **ABORT**: the aim points somewhere other than above the base — the frame convention
  or z-convention is wrong. Do NOT proceed to 7b; debrief the frame math first.
- If the aim is consistently offset by a fixed vector, record it: the catch-point
  computation (`reload_sequencer.compute_catch_point_mm`) or a frame transform needs a
  correction before any ball flies.

---

## Stage 7b — throw + STATIC catch (platform holds; hand armed via the catch-armed latch)

**Purpose**: prove the ball seats in the cup with the hand armed by the existing
coordinator, WITHOUT platform tilt/translation.

**How static catch works without CATCH mode**: the hand's prime/arm is now gated on the
`catch/armed` topic (the reload action's catch-armed latch), and the platform tilt is
gated on the `trajectory/arm_catch` latch inside `trajectory_node`. These are **two
separate gates** driven together by the action. For a genuine *static* catch we split
them by hand: publish `catch/armed = true` (which arms + primes the hand via
`catch_coordinator_node`) while leaving `trajectory/arm_catch` **down**, so
`trajectory_node` **ignores** `catch/dynamic_target` and the platform holds its commanded
pose. This is the exact static isolation the old TRAJECTORY-mode 7b relied on, expressed
through the new latch. **Bench-only manual override** — outside a reload the latch stays
down for a reason (a stray tracked ball must not arm the hand); here you raise the hand
gate deliberately for a single throw.

1. Operator holds the neutral ACTIVE catch pose in **TRAJECTORY** mode (streaming; the
   platform does not move for the catch).
2. Manually arm the hand only (platform stays static — `trajectory/arm_catch` stays
   down):

```bash
ros2 topic pub -t 3 -r 2 /catch/armed std_msgs/msg/Bool "{data: true}"
```

   **The hand primes to top IMMEDIATELY on this publish** (the 2026-07-23 fix:
   `catch_coordinator_node` primes on the `catch/armed` rising edge — it no longer
   waits for a ball on `/balls`, which made the first session's primes a coin flip
   against the 0.878 s flight). If the hand does not start rising within ~1 s, the
   build is stale — stop and rebuild. The platform holds because the reach latch is
   down.
3. Fire a single dead-centre throw. `target_name` names the robot so the announcement's
   `target_id` is `jugglebot` (not the default `point`) — otherwise the tracker tags the
   ball destination `point` and the whole catch pipeline drops it:

```bash
ros2 service call /bb/throw_at_target jugglebot_interfaces/srv/BallButlerThrow \
  "{use_target_point: true, aim_only: false, target_name: 'jugglebot', \
    target_point_global_mm: {x: 0.0, y: 0.0, z: 809.08}, throw_delay_s: 3.0}"
```

4. Lower the hand gate after the catch: `ros2 topic pub -t 3 -r 2 /catch/armed
   std_msgs/msg/Bool "{data: false}"` (resets the coordinator's prime/arm one-shots).

- **PASS**: ball seated in the cup; hand telemetry matches the armed catch profile;
  `/balls` reports the ball `CAUGHT`; **zero platform motion** throughout (the reach latch
  never rose). A dead-centre throw arrives near-vertical, so the hand does the seating
  with little/no tilt. (Hardware has caught smoothly before — priors are good.)
- **ABORT**: two consecutive bounce-outs ⇒ stop, capture the hand traces + rosbag, and
  return to Phase 6. Also ABORT on any E-STOP or visible platform jerk (a platform move
  here means `trajectory/arm_catch` rose when it should not have — investigate).
- After each throw: `/diagnose --latest` for the hand/tracking traces.

---

## Stage 7c — full reload action (translate + tilt catch via `jugglebot/reload`)

**Purpose**: the whole sequence through the action — preconditions, proactive prime, aim
+ throw at the cup-plane catch point, catch-armed latch raise, announcement→reactive
catch (tilt-through-seat `build_catch`), confirmation, and recenter.

Operator stays in **TRAJECTORY** mode, armed and streaming a hold — the action drives
everything from there (it never switches modes). Then:

```bash
ros2 action send_goal /jugglebot/reload jugglebot_interfaces/action/Reload \
  "{throw_delay_s: 3.0}" --feedback
```

Watch the feedback phases: `CHECKING (primes the hand immediately; dwells up to 10 s
while BB reloads if the hand is empty — RELOADING is BB's heartbeat state, not a
feedback phase) → PREPARING (raises the catch-armed latch, BEFORE any throw) →
AIMING → THROW_PENDING → BALL_IN_FLIGHT → CATCHING → SETTLING`, then a result.
Under the hood (2026-07-23 ordering: **every arming step precedes the throw**,
because BB's firmware countdown has no abort opcode): CHECKING dispatches the hand
prime to top (`JB_OP_HAND_CATCH_PRIME_REV` = **9.9594** since 2026-07-26, when it
became the derived stroke top; 9.858 before that, so a pre-2026-07-26 capture
reads the old value) the moment preconditions pass;
PREPARING raises the latch and confirms it; only then does AIMING send
`bb/throw_at_target`. On **CAUGHT** it lowers the latch and **re-centers**
(`go_home`, hand keeps the ball); on **any abort** it **retracts the hand to
bottom** (`JB_OP_HAND_RETRACT_REV = 0.0`), lowers the latch, and re-centers. A
standing catch-target rejection no longer ends the action mid-flight — the platform
holds, the primed cup gets its chance (it caught twice that way in the first
session), and the outcome resolves at settle (`CAUGHT` beats
`MISSED_INFEASIBLE_<code>`).

- **PASS**: ≥ 3/5 reloads return `outcome: CAUGHT` with a logged `catch_error_mm`;
  motion subjectively smooth (no snap at the prime, the latch-raise seam, or the
  recenter); no pump rejects; no E-STOP.
- **Receive-tilt expectation** (7b/7c): the receive tilt is clamped to the 12° usable
  ceiling (`tilt_geometry.MAX_TILT_DEG`). Arrivals more off-vertical than ~12° are still
  CAUGHT, but with only *partial* collinear seating (the cup axis is not fully aligned to
  the arrival) — the residual is the hand's to absorb. A steep off-centre arrival that
  bounces out is a seating-margin signal, not a gate reject.
- **Cancel does not recall the ball**: cancelling the action after AIMING does NOT recall
  the ball — BB has already committed the throw, so expect a throw regardless. Stay clear
  of the flight path. (A cancel after PREPARE still safes the robot: the node retracts the
  hand, lowers the latch, and re-centers on early exit.)
- Exercise each abort path once, deliberately:
  - **no-ball reject**: run with the hand empty and `bb/reload` disabled / ball removed
    → expect `REJECTED_NO_BALL` after the 10 s reload wait. The hand was already primed
    at command, so the reject now **retracts it** (SAFE_ABORT) — the hand ends at the
    bottom; the platform never moved.
  - **announcement-timeout abort**: disable BB (or block the throw) after the goal is
    accepted → expect `ABORTED_NO_ANNOUNCEMENT` within `throw_delay + 0.5 s`; the action
    **retracts the hand + re-centers** (SAFE_ABORT), platform ends level, hand at bottom
    (the first session's aborts left the hand parked at TOP — that was the −0.1 rev
    silent-reject bug, now fixed; verify the retract actually happens).
  - **wrong-mode reject**: send the goal while NOT in a streaming TRAJECTORY hold (e.g.
    in STANDBY) → expect `REJECTED_WRONG_MODE` immediately, zero motion, no prime (the
    reject lands before any arming).
- **ABORT**: two consecutive bounce-outs (back to Phase 6); any E-STOP; a catch target
  the gate rejects mid-sequence surfacing as `MISSED_INFEASIBLE_<code>` on more than one
  in five (revisit the reach envelope / limits).

**Exit**: `ros2 action send_goal /jugglebot/reload …` reliably catches. MVP goal 4
complete.

---

## Deferred / open (carry into the debrief)

- **Catch z-convention** (`809.08 mm`, the cup plane) is verified in 7a — if 7a required
  a correction, update `reload_sequencer.compute_catch_point_mm` (or the
  `HAND_CATCH_OFFSET_MM` term) and re-run the software gate.
- **Vel-match / CATCH_VEL_RATIO** (Phase-6 open question): if 7b/7c show poor seating,
  the `catch_vel_ratio` 0.6 hand design (not the sim's ≤15%-first-contact metric) is the
  reference; hardware evidence — not sim — gates any hand-config change.
