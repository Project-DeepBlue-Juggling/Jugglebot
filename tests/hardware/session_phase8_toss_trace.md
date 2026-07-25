# Hardware Session — MVP Phase 8 (single-ball-toss § Phase 3): real-ordering dry trace (powered bench, NO ball)

**Plan**: `plans/active/single-ball-toss.md` § Phase 3
**Logbook**: `logbook/2026-07-25-toss-phase1-action-sequencer-coordinator.md`
(the Phase-1 baseline, commit `5447f03`) + the Phase-3 prep entry (see
`logbook/INDEX.md`)
**Harness**: `tests/hardware/toss_trace_recorder.py` (`record` + `check`)
**Goal**: record the real cross-process ordering of the Toss choreography on the
live launch (**powered bench, NO ball**) and review it against the sequencer's
assumed ordering, **before any ball flies** — the Phase-7 audit found five
BLOCKING ordering bugs exactly in the mocked-ROS blind spot
(memory: mocked-ROS unit tests are blind to cross-process message choreography;
multi-node features need a real-ordering trace before hardware).

> **⚡ READ THIS FIRST — this is a POWERED sitting, and the dry trace fires ONE
> real empty-cup throw stroke: the first kind-0 (throw) trajectory ever
> dispatched on Jugglebot's hand** (the reload arc only ever dispatched
> kind-1/kind-3). "Dry" means *no ball*, not *no motion*. Full powered-session
> discipline applies throughout, and the stroke is treated as its own
> observation with PASS/ABORT criteria — a de-facto precursor to Phase 5's T0.
>
> Bench ground truth (2026-07-25 design review): a strictly-unpowered-ODrive
> bench **cannot** run this trace — activation FAULTs against undervoltage
> ODrives, so `control_mode` never reads TRAJECTORY and every goal dies
> `REJECTED_WRONG_MODE` before the waiver even matters. The trace therefore
> runs on a **powered no-ball bench**: every precondition passes naturally
> except ball possession, which is exactly what the trace waiver waives — and
> everything the trace observes (null positioning move, latch raise,
> self-announcement, pre-tilt, empty-cup stroke, SAFE_ABORT retract) is the
> real production choreography.

**Scheduling recommendation**: fold this trace capture into the **same sitting
as Phase 5's T0** (first real toss). Both need the identical powered-bench
discipline (can-bridge Teensy reboot, QTM up, armed TRAJECTORY hold, operator
on E-STOP), and the empty-cup dry stroke here is strictly milder than T0's
event_vel ladder — running it as T0's opening act costs ~15 minutes and saves a
separate power-up. The trace review (checker PASS) is still a **hard gate
before any ball flies**: if the ordering FAILs, T0 does not run that day.
Sequencing is the operator's call — a standalone trace sitting is equally
valid, just costlier.

## Roles & safety framing

- **The operator (Harrison) runs every robot-actuating command below.** The
  implementing session prepared these exact commands + PASS/ABORT criteria and
  verifies read-only.
- **If your physical intuition disagrees with any framing here, that is
  load-bearing signal — say so before proceeding.**
- E-STOP is always available. Any ABORT criterion ⇒ cut power / trigger the
  guard, then debrief before re-trying.
- **Stay clear of the hand's stroke path during capture D** — the empty cup
  fires a full-speed throw stroke (release ≈ 3.93 m/s; anything resting on the
  cup would be launched ≈ 0.79 m above the release plane).
- **The recorder is observe-only**: it subscribes only — it never publishes on
  any production topic and never calls any service. The goal trigger is your
  `ros2 action send_goal`.
- **No ball anywhere in the workspace, ever, this session** — a stray tracked
  marker/ball produces a REAL (tracking=1) detection that fails DT-14. (The
  self-announced toss's OWN predicted track — tracking=0, destination=jugglebot,
  synthesised by `ball_prediction_node` from the announcement — is intrinsic and
  expected; DT-14 allows it.) Clear the volume before starting.

## Preconditions

- Jugglebot powered, ODrives up, CAN3 healthy. **BB is NOT commanded this
  session** (idle heartbeat is fine).
- **POWER-CYCLE THE CAN-BRIDGE TEENSY** before the sitting (the uptime-lag
  discipline). The recorder's meta captures `uptime_ms` first/last — quote
  both in the debrief.
- `run_mpc.py` is **NOT** running (sole-binder :5557 interlock).
- QTM up, streaming the Base body at minimum; `/rigid_body_poses` flowing.
- Branch `mvp-trajectory-bringup` at ≥ `5447f03`.
- **Build gate (Phase 1 changed ros_ws AND added an interface — the Toss
  action)**:
  ```bash
  cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot
  source install/setup.bash
  ```
  then **relaunch** `jugglebot_launch.py` (launch runs the *installed* copy).
- Arm per the Phase-1 sequence: launch → home → activate → confirm the 40 Hz
  hold stream → TRAJECTORY → **zero motion at arm**.

### Pre-flight — confirm the freshly-built code is live

The #1 silent-stale hazard (per Phase 7): the launch quietly runs the old
install. Confirm the new graph **before anything else**:

```bash
ros2 action list  | grep jugglebot/toss          # the toss action is served
ros2 topic list   | grep catch/prime_hold        # the suppression gate exists
ros2 param get /reload_coordinator_node toss_ball_evidence_waiver_trace_only   # -> False
ros2 node list    | grep ball_tracker            # tracker LIVENESS (see below)
```

- **All four present/False, or STOP and rebuild + relaunch.**
- The tracker-liveness line is load-bearing: a **dead tracker passes every
  CHECKING gate** (Phase-1 known limitation — `balls` silence is expected with
  no ball, so process liveness is the only bench check there is).

### Start the recorder (own terminal, system python3 with the ROS env — NOT the venv)

```bash
python3 tests/hardware/toss_trace_recorder.py record
```

Confirm its 1 Hz live line **before sending any goal**:
- `hand ~100 Hz` and `pos` within ±0.5 rev of 0 (the park band CHECKING will
  demand),
- `mocap` flowing,
- `stream=True mode=TRAJECTORY plan=hold`,
- `balls 0`.

If `hand 0 Hz`: the Teensy telemetry stream is down — CHECKING would reject
`HAND_STALE`; fix before proceeding.

---

## Capture R — un-waived REJECTED_NO_BALL (waiver OFF, ZERO motion) — FIRST

Run this **before** the dry capture: it is simultaneously the refusal-path
capture the plan requires and the cheapest possible proof that every *other*
precondition wire is healthy before the waiver ever goes up —
`REJECTED_NO_BALL` is CHECKING's second-to-last gate, so getting exactly that
code proves mode/streaming/mocap/hand-fresh/hand-parked all PASSED (checker
RJ-1). Zero motion.

> **⚠️ Superseded default (2026-07-25, single-ball-toss Phase 5 change B):** the
> ball-evidence gate now defaults OFF (`toss_require_ball_evidence: false` — the
> operator guarantees the ball; there is no ball-in-cup sensor), so an un-waived
> toss no longer `REJECTED_NO_BALL` by default. This historical Capture R needs
> the gate ENABLED: set `toss_require_ball_evidence: true` (config → colcon build
> → relaunch) for this capture only. Phase 3 is CLOSED; this note is for re-runs.

```bash
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.78}" --feedback
```

(`throw_height_m` 0.78 ≈ the 0.8 s production default, mid-band — the trace is
about ordering, not ballistics, and 0.78 m stays away from the hardware-marginal
< 0.7 s flight band flagged for T0. `throw_delay_s` omitted ⇒ default 5.0 s;
`catch_vel_scale` omitted ⇒ default 0.8 — both trace the production values.)

- **PASS**: result `outcome: REJECTED_NO_BALL`; **no feedback phases**; zero
  motion; zero choreography traffic (the recorder's
  `hold/disp/armed/ann/dyn` counters do not advance).
- **ABORT/debug**: any OTHER `REJECTED_*` code ⇒ that precondition wire is
  unhealthy (the checker's RJ-1 output maps code → wire); fix before
  proceeding. **Any motion ⇒ stop and debrief** — nothing may move on a
  reject.

**Stop the recorder** (Ctrl-C) and note the trace path — one goal per trace
file keeps the review clean. Restart it for capture D.

---

## Capture D — waived dry choreography (ONE goal)

1. Raise the waiver. **Bench affordance — this runbook is the ONE document
   sanctioned to set it; ball-flying session runbooks (Phase-5 T0–T4, reload
   sittings) must NEVER set it.** It waives only the ball-possession
   precondition; hand-parked/hand-fresh stay hard:

   ```bash
   ros2 param set /reload_coordinator_node toss_ball_evidence_waiver_trace_only true
   ros2 param get /reload_coordinator_node toss_ball_evidence_waiver_trace_only   # -> True
   ```

2. **Clear the hand's stroke path.** Recorder running (fresh file), live line
   healthy. Send the same goal (`--feedback`):

   ```bash
   ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
     "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.78}" --feedback
   ```

3. Watch: feedback `POSITIONING → PREPARING → THROWING` (~5 s countdown from
   accept) → the hand fires **ONE smooth full stroke bottom→top** (kind-0,
   release ≈ 3.93 m/s; telemetry peak ≥ 40 rev/s) → `BALL_IN_FLIGHT` /
   `CATCHING` → result **`outcome: MISSED`** (the honest dry verdict: the
   stroke was real, the tracker never confirms a ball) → hand retracts to
   bottom → **one** `Toss MISSED` WARN in the launch shell. The platform stays
   still throughout — both the positioning move and the pre-tilt are null at
   (0, 0, 170).

- **PASS**: `MISSED` result; exactly one stroke; hand ends parked at bottom;
  platform never moved; no E-STOP. Retract-ladder WARNs (if any) are noted as
  ERR_TIMEOUT-epidemic gauge data, not failures.
- **Expected-variant**: `ABORTED_NO_RELEASE` = the single throw dispatch was
  eaten (ERR_TIMEOUT class — `_svc_set_hand_traj` logs nothing, so the eaten
  dispatch is silent). Record it as epidemic evidence and re-run once for the
  MISSED capture; **two consecutive `ABORTED_NO_RELEASE` ⇒ stop** — that is
  the epidemic investigation's evidence, not a re-run situation. The checker
  detects this variant and says so.
- **ABORT**: any platform motion (latch/pose hazard — stop and debrief); a
  second stroke or mid-ascent yank (re-dispatch class — **NEW evidence**, the
  ladders should never blind-re-dispatch); E-STOP.

4. **Lower the waiver immediately** and verify:

   ```bash
   ros2 param set /reload_coordinator_node toss_ball_evidence_waiver_trace_only false
   ros2 param get /reload_coordinator_node toss_ball_evidence_waiver_trace_only   # -> False
   ```

   (A relaunch also resets it — the parameter is declared `False` at node init
   and read fresh at every goal accept — but verify anyway; the debrief
   checklist re-verifies at end of session.)

**Stop the recorder** (Ctrl-C); note both trace file paths and the meta files.

---

## Review step (can run off-robot — the `check` path is pure stdlib, venv OK)

```bash
python3 tests/hardware/toss_trace_recorder.py check temp/logs/toss_trace_<R-stamp>.jsonl --reject
python3 tests/hardware/toss_trace_recorder.py check temp/logs/toss_trace_<D-stamp>.jsonl --dry --timeline
```

Invariants the checker evaluates (IDs cross-reference the Phase-3 design spec):

| ID | Checks |
|----|--------|
| DT-1 | prime_hold True precedes armed True by ≥ 1 tick (the Phase-1 logbook's named trace item) |
| DT-2 | armed True precedes the self-announcement by ≥ 1 tick |
| DT-3 / DT-4 | PREPARE bundle order: vel_scale, then ONE prime_dispatched belt stamp, before armed (same-tick AMBIGUOUS tolerated) |
| DT-5 | announcement precedes the THROWING transition (dispatch-time proxy, ~1 tick — the true `set_hand_traj_cmd` instant is unobservable); no stroke-command evidence before the announcement |
| DT-6 | no auto-prime while prime_hold is raised (no CCN prime logs, no ascent before the stroke) |
| DT-7 / DT-8 | DT-7: pre-tilt log + the announcement's predicted-track dynamic_target STREAM, all within 30 mm of (0,0,170), catch-accepted (a target that wanders off = a real ball pulled the reach). DT-8: none before arming |
| DT-9 | exactly one stroke, at the announced release time, before BALL_IN_FLIGHT; no re-dispatch |
| DT-10 | feedback phase order, no regressions, no CHECKING |
| DT-11 | teardown order: armed False → latch-disarmed → … → prime_hold False LAST; hand retracted |
| DT-12 | one truthful `Toss MISSED` WARN + one waiver WARN |
| DT-13 | streaming=True + mode=TRAJECTORY throughout; positioning 'move' present (the catch pre-position keeps the platform in an active plan — the 'hold' recenter falls after the trace window) |
| DT-14 | pollution guard: no REAL-detection (tracking=1) or foreign-destination balls entries; the toss's own announcement-seeded predicted track (tracking=0, destination=jugglebot) is intrinsic and expected |
| RJ-1..4 | reject: right code (ABORTED status), total choreography silence, waiver provably unset, zero feedback |

- **PASS**: all RJ-* and DT-* invariants PASS (AMBIGUOUS is acceptable ONLY on
  DT-3/DT-4's same-tick bundle orderings). **Any FAIL = the Phase 3 gate is
  not met** — file the finding, do NOT proceed to Phase 5 hardware; an
  ordering FAIL here is exactly the bug class this phase exists to catch.
- Paste the checker output + a `--timeline` excerpt into the session logbook
  entry; quote `uptime_ms` first/last from each meta file.

---

## Bench task T — resolve the platform QTM body for `toss_mocap_body`

(Phase-1 BLOCKING-finding follow-up; the POSITIONING mocap cross-check ships
**disabled** because no platform rigid body exists in QTM yet.)

1. From the trace meta's `observed_rigid_bodies` list, confirm today's bodies
   (known: `Base`, `Ball_Butler`, `Catching_Cone` — no platform body).
2. In QTM, define/verify a rigid body on the platform.
3. With the platform holding ACTIVE, re-run `record` briefly and read that
   body's position rows: it must publish in the **platform_start frame —
   z ≈ 170 at ACTIVE, near-zero x/y** (non-base bodies are z-shifted by
   mocap_interface; comparing global-converted values would re-import the
   z double-add the coordinator's frame note warns about).
4. Record the body's exact name + observed (x, y, z) in the debrief.
   **Do NOT set the `toss_mocap_body` parameter this session** — enabling the
   cross-check is a separate reviewed decision (its false-ABORT mode killed
   every toss in the Phase-1 review; default-off is the shipped safe state).

---

## Out of scope this session

- **Unarmed-announcement injection (the optional "capture X") is SKIPPED.**
  Publishing a synthetic `ThrowAnnouncement` would positively demonstrate the
  CCN unarmed-drop, but it **seeds a phantom tracker expectation** (a
  destination-`jugglebot` track with no ball) that makes every subsequent toss
  goal `REJECTED_TRACK_ACTIVE` until it expires. DT-8's negative form on the
  main trace already covers the invariant. Do not publish announcements by
  hand.
- Enabling `toss_mocap_body` (bench task T resolves the name only).
- Any ball, any BB command, any catch — that is Phase 5.

## Deferred / open (carry into the debrief)

- Waiver verified `False` **twice**: post-capture-D and end-of-session.
- ERR_TIMEOUT epidemic gauge: dispatch-failure WARN counts from both hand
  ladders (prime + retract), plus any `ABORTED_NO_RELEASE`.
- Stroke observations feed Phase-5 T0 (first kind-0 data point: commanded
  event_vel ≈ 3.93 m/s vs telemetry peak vel; the 40 rev/s stroke-watch
  threshold and the 1.0 s min event-delay are derived values — T0 re-tunes).
- Tracker-liveness structural fix (heartbeat) remains open.
- The powered-era Reload → Toss chain trace (real ball) belongs to the first
  Phase-5 sitting; the recorder already captures the Reload action wires
  for it.
