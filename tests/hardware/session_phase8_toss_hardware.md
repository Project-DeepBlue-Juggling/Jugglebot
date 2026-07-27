# Hardware Session — MVP Phase 8 (single-ball-toss § Phase 5): staged throw testing (T0–T4)

**Plan**: `plans/active/single-ball-toss.md` § Phase 5
**Action**: `jugglebot/toss` (`Toss.action`) — goal nominates a **throw height**
**Depends on**: Phase-3 trace VALIDATED (`logbook/2026-07-25-toss-phase3-trace-validated.md`);
Toss.action height + operator-guaranteed-ball changes
(`logbook/2026-07-25-toss-action-height-and-operator-guaranteed-ball.md`).
**Goal**: bring single-ball tossing to hardware in staged rungs — characterise the
throw (T0), then vertical toss-and-catch (T1), a height ladder (T2), toss-at-position
(T3), and finally the Tier-8b displaced throw (T4). Each rung has its own PASS/ABORT.

> **⚠ RUN `session_anomaly_fixes.md` FIRST (added 2026-07-27).** The 2026-07-25
> self-toss anomaly-fix run landed eleven phases that change what this file
> measures — the levelling frame, the catch arrival twist, the hand-catch arm
> gating, the catch prime, and a **firmware change requiring a flash**. Its runbook
> `tests/hardware/session_anomaly_fixes.md` answers *"did those fixes land?"*; this
> file answers *"how well does the toss work?"*, and a T-rung failure cannot
> distinguish the two. Run the anomaly runbook first, then come back here.
>
> Two things from that run bind this file directly: the build gate is
> `colcon build --packages-select jugglebot_interfaces jugglebot` (the interfaces
> package is **not** optional — `TrajectoryStatus.msg` gained a field and a partial
> rebuild makes `trajectory_node` exit shortly after launch), and the Platform
> Teensy needs a **flash** it cannot report, since it carries no `FW_VERSION`.

> **⚡ READ THIS FIRST — this session FIRES REAL THROWS.** Jugglebot throws a real
> ball into the air and attempts to catch it. On any miss (expected early), the ball
> **lands on the floor** — clear the area. The hand fires a full kind-0 stroke
> (release up to ~5.4 m/s at the 1.48 m band ceiling); **stay clear of the hand's
> stroke path and the landing zone**. E-STOP is always in reach.
>
> **The operator guarantees a ball is in the hand before every toss.** There is no
> ball-in-cup sensor — the ball-evidence CHECKING gate is OFF by default
> (`toss_require_ball_evidence: false`), so the toss will NOT refuse an empty cup; a
> forgotten ball fires an empty stroke (benign) but wastes the rung. Load a ball
> first (see § Loading).

## Roles & safety framing

- **The operator (Harrison) runs every robot-actuating command below.** The
  implementing session prepared these exact commands + PASS/ABORT criteria and
  verifies read-only.
- **If your physical intuition disagrees with any framing here, that is load-bearing
  signal — say so before proceeding.**
- E-STOP always available. Any ABORT criterion ⇒ cut power / trigger the guard, then
  debrief before retrying.
- One toss at a time — the coordinator runs ONE ball-op at a time (a concurrent goal
  is `REJECTED_BUSY`).

## Preconditions

- Jugglebot powered, ODrives up, CAN3 healthy; QTM up streaming **Base + Platform**
  rigid bodies; `/rigid_body_poses` flowing.
- **POWER-CYCLE THE CAN-BRIDGE TEENSY** before the sitting (the uptime-lag
  discipline). Log `uptime_ms` alongside every measurement.
- `run_mpc.py` is **NOT** running (sole-binder :5557 interlock).
- **Build gate — BOTH the interface AND config changed this cycle (throw-height goal
  field + `toss_require_ball_evidence`):**
  ```bash
  cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot
  source install/setup.bash
  ```
  then **relaunch** `jugglebot_launch.py` (launch runs the *installed* copy).
- Arm per the Phase-1 sequence: launch → home → **`level`** → activate → confirm the
  40 Hz hold stream → TRAJECTORY → **zero motion at arm**.
- **The `level` step is not optional as of 2026-07-26.** CHECKING now refuses with
  `REJECTED_NOT_LEVELLED` unless `trajectory_node` reports a loaded gravity
  correction, and that correction is **per-process**: `/gravity_offset` is VOLATILE
  with one latched push per orchestrator boot, so every launch/relaunch (including
  the build gate's, and this file's mandated Teensy power-cycle) empties it while the
  Teensy-persisted `RobotState.levelling_complete` still reads `true` and proves
  nothing. Without it **every toss below refuses before anything moves** — and a
  Reload still works, so it is easy to load a ball and then discover the refusal.
  `level` runs from **IDLE** and returns to IDLE, so it goes before `activate`.
  Contract C-LEVEL-1.O (`ros_ws/docs/levelling_frame.md`); the gate's own checks are
  `tests/hardware/session_anomaly_fixes.md` § Section LVLGATE.

### Pre-flight — confirm the freshly-built code is live

```bash
ros2 action list | grep jugglebot/toss                                   # toss action served
ros2 interface show jugglebot_interfaces/action/Toss | grep throw_height_m # NEW field present
ros2 param get /reload_coordinator_node toss_ball_evidence_waiver_trace_only  # -> False (unused here)
ros2 node list | grep ball_tracker                                        # tracker LIVENESS
ros2 topic echo /trajectory/status --once | grep gravity_correction_loaded # -> true (after `level`)
```
- `throw_height_m` present, or STOP and rebuild+relaunch (a stale install still has
  `flight_time_s` and will silently use the default height).
- `gravity_correction_loaded: true`, or the `level` step was skipped / a relaunch
  dropped it and every toss returns `REJECTED_NOT_LEVELLED`. If the key is missing
  entirely, `jugglebot_interfaces` was not rebuilt.

## Loading a ball (before every toss)

A toss needs a ball seated in the cup. Load it with a Reload (BB throws, JB catches):
```bash
ros2 action send_goal /jugglebot/reload jugglebot_interfaces/action/Reload \
  "{throw_delay_s: 0.0, catch_vel_scale: 0.0}" --feedback
```
- **PASS**: `outcome: CAUGHT` (or judge by eye — tracker verdicts can read MISSED on
  a real catch; § Open items). Confirm a ball is physically in the cup before tossing.
- If reload misses, retry until a ball is seated. (The toss will not check possession
  — YOU confirm the ball is there.)

## The toss command

```bash
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.6}" --feedback
```
- `catch_position` — nominated catch point, **mm, STOW-relative** (z 170 = ACTIVE
  plane); Tier 8a throws and catches at the same (x, y).
- `throw_height_m` — apex height of the ball above release, **metres**. `0` ⇒ config
  default (~0.78 m). `throw_delay_s`/`catch_vel_scale` omitted ⇒ defaults (5.0 s / 0.8).
- Feedback phases: `CHECKING → POSITIONING → PREPARING → THROWING → BALL_IN_FLIGHT →
  CATCHING → SETTLING`. Result `outcome`: `CAUGHT | MISSED | REJECTED_<code> | ABORTED_<code>`.

### Height ↔ flight ↔ release-speed reference (`h ∝ T²`, NOT linear)

| `throw_height_m` | flight T (s) | release speed (m/s) |
|---|---|---|
| 0.20 | 0.40 | 1.98 |
| 0.40 | 0.57 | 2.80 |
| 0.60 | 0.70 | 3.43 |
| 0.78 (default) | 0.80 | 3.91 |
| 1.00 | 0.90 | 4.43 |
| 1.48 (band ceiling) | 1.10 | 5.39 |

(`T = sqrt(8h/g)`, release `v = sqrt(2gh)`, `g = 9.806 m/s²`. Firmware band:
`T ∈ [0.55, 1.10] s` inclusive — **`throw_height_m > 1.48` (T > 1.10 s) ⇒
`REJECTED_FLIGHT_TIME`**, and `T < 0.7 s` (h < ~0.6 m) is stroke-marginal — start
at 0.6 m / 0.7 s.)

---

## T0 — throw characterisation (release, not catch)

Load a ball, then fire tosses at a fixed low height and **measure the outgoing
ball's release velocity from QTM** — the point is the throw, not the catch (the ball
goes to the floor; the catch is not scored). This is the empirical release model that
replaces the sim gate's placeholder noise.

```bash
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.4}" --feedback
```
- Repeat ~5× at 0.4 m, then ~5× at 0.6 m. Between throws: reload / re-seat a ball,
  clear the floor.
- **Measure** (from the QTM ball track / `/diagnose`): peak release speed vs the
  table's expected (2.80 m/s @ 0.4 m, 3.43 m/s @ 0.6 m); shot-to-shot **scatter**
  (feeds the gate noise model); release-position repeatability; hand telemetry peak
  (≥ 40 rev/s = a real stroke).
- **PASS**: a clean single stroke each time; release speed tracks the commanded
  height within a consistent bias; scatter recorded. **ABORT**: any platform motion
  beyond the null pre-position; a second stroke / mid-ascent yank (re-dispatch class —
  NEW evidence); E-STOP.

## T1 — single vertical toss-and-catch (centre, low)

```bash
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.6}" --feedback
```
- Load a ball before each attempt; 5 attempts.
- **PASS**: ≥ **3/5** caught (judge by eye + tracker-id evidence — verdicts may read
  MISSED on a real catch). **ABORT**: uncontrolled platform motion; hand re-dispatch;
  E-STOP.

## T2 — height ladder (centre)

Repeat T1 at rising heights: **0.6 → 1.0 → 1.48 m** (T ≈ 0.70 → 0.90 → 1.10 s —
1.48 m is the band ceiling; `throw_height_m > 1.48` is `REJECTED_FLIGHT_TIME`).
- **PASS** per step: ≥ **3/5** caught. Expect degradation as height rises — the
  Phase-2 sim gate flagged the long-flight tail (T ≥ 0.95 s, i.e. h ≥ ~1.1 m) as where
  drift ∝ v·T grows against the catch envelope. T2 finds the real hardware ceiling;
  record the highest height that still makes ≥ 3/5.

## T3 — toss-at-position (workspace corners)

Fixed height 0.6 m, catch at the four ±60 mm corners (throw = catch site, Tier 8a):
```bash
# e.g. +x corner; repeat for (-60,0), (0,60), (0,-60)
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 60.0, y: 0.0, z: 170.0}, throw_height_m: 0.6}" --feedback
```
- **PASS**: ≥ **3/5** at each corner. This exercises the platform pre-positioning
  (the POSITIONING move to the nominated x, y before a level vertical toss).

## T4 — Tier 8b displaced throw→catch (LAST; gated)

**Only after** Phase 4's gate AND the 8b dry-trace addendum (plan § Phase 4 note (d):
confirm `pretilt_hold` reaches `catch_coordinator` before the announcement). Enable
8b in config (`toss_tier: "8b"`, + colcon build + relaunch). Displaced A→B: the shipped
gate caps displacement at **70 mm**; per the Phase-4 asymmetry map, aim into the
**−y hemisphere or −x** at **T ≥ 0.80 s** (height ≥ ~0.78 m):
```bash
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: -70.0, y: 0.0, z: 170.0}, throw_height_m: 0.8}" --feedback
```
- **PASS**: ≥ **3/5** at 70 mm into a sim-robust direction. Do NOT push to 100 mm until
  the map's +y-hemisphere weakness is retired (plan § Phase 4).

---

## After every rung

- `/diagnose --latest` on the session telemetry; retain rosbags.
- **One truthful outcome line per toss** in the debrief; quote `uptime_ms` first/last.
- Record for each toss: commanded height, `outcome`, `catch_error_mm`,
  `achieved_flight_s` (the measured release→catch time — note the result reports TIME,
  not height; convert via the reference table if wanted).

## Open items / watch (carried from the reload arc)

- **Tracker verdict corruption**: CAUGHT can read MISSED — judge PASS counts by eye +
  the tracker-id-correlated evidence, as in the Phase-7 fourth sitting.
- **ERR_TIMEOUT epidemic**: more hand dispatches = more exposure; the telemetry-verified
  ladder is the mitigation; log dispatch-failure WARNs / any `ABORTED_NO_RELEASE` as
  epidemic gauge data (two consecutive `ABORTED_NO_RELEASE` ⇒ stop).
- **Teensy-uptime lag**: all timing-sensitive numbers (achieved flight, catch error)
  are only meaningful on a fresh can-bridge boot — `uptime_ms` logged with each.
