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
> package is **not** optional — `TrajectoryStatus.msg` gained a field, `RobotState.msg`
> gained two, and a partial rebuild makes `trajectory_node` exit shortly after
> launch), and the Platform Teensy needs a **flash**. As of 2026-07-27 that flash
> *is* reportable — the board declares a `FW_VERSION` and
> `link_status/platform_fw_version` reads **`2`** when flashed current,
> `1` on a board carrying only the Phase-4 prelude (**stale — re-flash before any
> toss above 0.78 m; a v1 board has no post-release decel feedforward and is the
> board that touched the end stop on 2026-07-27**), and `0 (PRE-VERSIONING)` when
> never flashed. It warns, it does not refuse, so read it:
> `ros2 topic echo /link_status --once | grep -A1 platform_fw_version`
> (`ros_ws/docs/platform_fw_version.md`).

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
- **CHECK the correction; `level` only if it is missing.** *(Corrected 2026-07-27 —
  an earlier version of this bullet said `level` is mandatory after every relaunch.
  That was wrong, and it would have cost you a needless levelling routine on every
  build gate.)* CHECKING refuses with `REJECTED_NOT_LEVELLED` unless
  `trajectory_node` holds a gravity correction, and that correction is
  **per-process**. But it is normally **restored automatically on ROS2 boot**:
  `RobotState` carries both `levelling_complete` **and** `pose_offset_rad` from the
  **Platform** Teensy (`teensy_bridge_node.py:1430`), the orchestrator stores them
  (`orchestrator_node.py:165-167`), and on the first IDLE entry after boot it pushes
  the persisted offset to `/gravity_offset` (`:329-335`). The standing session
  power-cycle is the **can-bridge** Teensy, which does **not** clear the Platform
  Teensy's cache — so a mid-session relaunch, including this file's build gate,
  should come back already levelled.
  **The one caveat, and it is why you check rather than assume:** `/gravity_offset`
  is VOLATILE, so a `trajectory_node` that finishes subscribing after the push
  misses it. Whether discovery wins that race is unmeasured. So read the flag:
  ```bash
  ros2 topic echo /trajectory/status --once | grep gravity_correction_loaded
  ```
  `true` ⇒ nothing to do. `false` ⇒ run `level` (from **IDLE**, so before
  `activate`). A fresh **Platform Teensy** power-cycle clears the cache and always
  requires a manual `level`. Without a correction **every toss refuses before
  anything moves** — and a Reload still works, so it is easy to load a ball and then
  discover the refusal. Contract C-LEVEL-1.O (`ros_ws/docs/levelling_frame.md`); the
  gate's own checks are `tests/hardware/session_anomaly_fixes.md` § Section LVLGATE.

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
- **PASS**: `outcome: CAUGHT` (or judge by eye — a RELOAD verdict still reads MISSED
  on a real catch, correctly; § Open items). Confirm a ball is physically in the cup
  before tossing.
- If reload misses, retry until a ball is seated. (The toss will not check possession
  — YOU confirm the ball is there.)

## The toss command

```bash
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.6}" --feedback
```
- `catch_position` — nominated catch point, **mm, STOW-relative** (z 170 = ACTIVE
  plane); Tier 8a throws and catches at the same (x, y).

> ### ⚠️ TIER: THE SHIPPED DEFAULT IS `8b` SINCE 2026-07-28 — READ BEFORE T0
>
> `jugglebot_operational.toss_tier` was flipped `"8a" → "8b"` on 2026-07-28 (operator
> decision on the T4 evidence below; see
> [`logbook/2026-07-28-toss-tier-8b-default.md`](../../logbook/2026-07-28-toss-tier-8b-default.md)).
> **T0–T3 below were written for Tier 8a and their PASS criteria describe 8a
> behaviour.** They do not self-select a tier — the tier comes from the build — so on
> a default build they run the **8b** choreography while the prose claims 8a:
>
> - the platform pre-positions **tilted at the config throw site `A = (0, 0)`**, not
>   level at the nominated catch `(x, y)`;
> - the throw is **tilt-aimed** at the displaced catch;
> - the A→B platform reach is **deferred to `t_release`**, i.e. it happens in flight.
>
> `T0`, `T1` and `T2` all nominate `(0, 0)`, so `|B − A| = 0` and the *release* is
> bitwise identical to 8a — but `catch/pretilt_hold` is still raised and a 0 mm
> deferred reach is still published, and **that co-located 8b path has never run on
> hardware** (every one of the 11 validated T4 throws was displaced). Row `TIER-D` of
> [`session_anomaly_fixes.md`](session_anomaly_fixes.md) § SECTION TIER scores it, and
> that file runs first. **`T3` is the
> one that changes materially**: its ±60 mm corners are inside the 70 mm cap, so they
> are **ACCEPTED as 8b displaced throws** — its stated purpose ("the POSITIONING move
> to the nominated x, y") does not happen at all.
>
> **Choose one before starting, and record which:**
>
> | | how | consequence |
> |---|---|---|
> | **(A) Run T0–T3 as written** | set `toss_tier: "8a"` in `config/hardware_config.yaml`, `python config/generate_config.py`, `colcon build --packages-select jugglebot`, relaunch. Flip back to `8b` before T4 (same four steps) | The rungs mean what they say. Costs two rebuilds. **This is the default choice** — T0–T3 are a Tier-8a capability ladder and scoring them against 8b behaviour measures nothing |
> | **(B) Run everything on the 8b default** | change nothing | T0–T2 are still valid (`\|B − A\| = 0`). **Do NOT run T3's `(0, +60)` corner**: it is an un-gated 8b displaced throw into the `+y` hemisphere that the Phase-4 asymmetry map flags weak, at `T ≈ 0.70 s`, below the `T ≥ 0.80 s` that T4 itself stipulates for displaced throws. Re-score the other three corners against T4's criteria, not T3's |
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
- **PASS**: ≥ **3/5** caught (judge by eye + tracker-id evidence). *(Updated
  2026-07-28: a SELF-TOSS verdict should now be trustworthy — the possession gate
  read 17/17 real catches on the 2026-07-27 capture where it previously read 0/17.
  A self-toss reading `MISSED` on a catch you watched land is a **finding**. See
  § Open items.)* **ABORT**: uncontrolled platform motion; hand re-dispatch;
  E-STOP.

## T2 — height ladder (centre)

Repeat T1 at rising heights: **0.6 → 1.0 → 1.48 m** (T ≈ 0.70 → 0.90 → 1.10 s —
1.48 m is the band ceiling; `throw_height_m > 1.48` is `REJECTED_FLIGHT_TIME`).
- **PASS** per step: ≥ **3/5** caught. Expect degradation as height rises — the
  Phase-2 sim gate flagged the long-flight tail (T ≥ 0.95 s, i.e. h ≥ ~1.1 m) as where
  drift ∝ v·T grows against the catch envelope. T2 finds the real hardware ceiling;
  record the highest height that still makes ≥ 3/5.

## T3 — toss-at-position (workspace corners)

> **TIER-SENSITIVE — this rung means nothing on an 8b build.** Under the shipped `8b`
> default a ±60 mm corner is a *displaced* throw (60 mm is inside the 70 mm cap), so
> the platform pre-tilts at `(0, 0)` and never makes the POSITIONING move this rung
> exists to exercise. Run it under option **(A)** (an 8a build) or skip it — and under
> option (B) do not run the `(0, +60)` corner at all. See § TIER above.

Fixed height 0.6 m, catch at the four ±60 mm corners (throw = catch site, Tier 8a
**build required**):
```bash
# e.g. +x corner; repeat for (-60,0), (0,60), (0,-60)
ros2 action send_goal /jugglebot/toss jugglebot_interfaces/action/Toss \
  "{catch_position: {x: 60.0, y: 0.0, z: 170.0}, throw_height_m: 0.6}" --feedback
```
- **PASS**: ≥ **3/5** at each corner. This exercises the platform pre-positioning
  (the POSITIONING move to the nominated x, y before a level vertical toss).

## T4 — Tier 8b displaced throw→catch (LAST; gated)

**GATE SATISFIED 2026-07-27** — Phase 4's gate and the 8b dry-trace addendum (plan
§ Phase 4 note (d): `pretilt_hold` reaches `catch_coordinator` before the
announcement) both cleared at that sitting, where **11/11 displaced throws were
accepted out to the 70 mm cap** with the deferred reach firing correctly on every
one. That is why 8b is now the shipped default; the "LAST; gated" ordering below is
kept because the rungs still build on each other, not because the tier is withheld.
**`toss_tier: "8b"` is already the default — the old "enable 8b in config" step is a
no-op** unless you took option (A) above, in which case flip it back (`toss_tier:
"8b"`, `python config/generate_config.py`, colcon build, relaunch) **before** this
rung. Displaced A→B: the shipped
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

- **Tracker verdict corruption**: *(rewritten 2026-07-28 — half of what this said is
  no longer true, and the wrong half would make an operator ignore a working verdict.)*
  The possession gate was fixed (contract **C-POSSESS-1**,
  `ros_ws/docs/ball_possession_contract.md`): it ANDed a `|z − catch_z| ≤ 150 mm`
  bound onto a quantity that is a dead-reckoned free-fall extrapolation, so it scored
  **0 of 17** real self-toss catches. **Self-toss** verdicts are now expected to be
  right (17/17 offline on the 2026-07-27 capture) — a `MISSED` on a catch you watched
  land is a finding. **Reload** verdicts still read `MISSED` on a real catch and that
  is correct: every Ball-Butler track in that capture is a split track carrying no
  measurements, so the gate refuses to mint a verdict from it. Keep judging by eye +
  the tracker-id-correlated evidence until a sitting has scored
  `tests/hardware/session_anomaly_fixes.md` § SECTION POSS row `POSS-1`.
- **ERR_TIMEOUT epidemic**: more hand dispatches = more exposure; the telemetry-verified
  ladder is the mitigation; log dispatch-failure WARNs / any `ABORTED_NO_RELEASE` as
  epidemic gauge data (two consecutive `ABORTED_NO_RELEASE` ⇒ stop).
- **Teensy-uptime lag**: all timing-sensitive numbers (achieved flight, catch error)
  are only meaningful on a fresh can-bridge boot — `uptime_ms` logged with each.
