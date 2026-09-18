**CLOSED 2026-09-16 — K=0.7 ADOPTED.** The ladder was flown in full, both
arms, all five rungs (2026-09-16 16:22, bag
`~/Desktop/rosbags/2026-09-16_16-22-22`, log
`temp/logs/launch_r2gate_20260916_1622.log`, K switch at
`t=1789540208.552`). The pre-registered **"K=0.7 → ratio ≤ 1.00" criterion
was NOT met literally** — mean hand meas/cmd at K=0.7 sits at 1.00–1.03
(peak up to 1.05) across every rung, not ≤1.00 — but the owner's decision is
to **adopt K=0.7 as the launch default anyway**: it closes the gap from
1.05–1.22× (K=0, mean 1.05–1.13×) to 1.00–1.05× (K=0.7) at every rung with
peak current well under the 48 A live cutoff (max 39.9 A), and Verdict B
(missing torque FF was most of the story) is strongly supported — more
strongly than the offline model itself predicted. The residual ~1–5 % hand
overspeed and the larger ~7–10 % raw-mocap ball-apex residual are launch
dynamics for the learner to correct, not a hand-tuning problem; see
`logbook/2026-09-16-apex-ladder-k07-ab-result.md` for the full Discussion
(why not K=1.0 now, the current-vs-height mechanism, the catch-early
pattern). The `platform wobble` / low-throw / `LIMIT_JERK` /
`ABORTED_NO_RELEASE` events observed on this sitting are diagnosed and fixed
separately in `logbook/2026-09-16-outcome-landing-frozen-at-the-crossing.md`
(commit `cd8cd82`) — not re-narrated here. This runsheet stays flyable for a
future re-ladder (e.g. at a new K, or after a further plant fix); do not
delete its steps.

---

# R3 apex ladder — hand C2 + torque feedforward A/B (FW 23 / protocol 9)

**Flash target updated 2026-09-15, same day as the plumbing below.** The
can-bridge FW 22 / PROTOCOL_VERSION 8 work this sheet was written against
(hand C2 scheduling + torque feedforward) has since been folded into
**can-bridge FW 23 / PROTOCOL_VERSION 9**, which additionally replaces the
fatal `CAN_BUS_DOWN` predicate with an axis-silence watchdog (see
`logbook/2026-09-15-fw23-axis-silence-watchdog.md`). There is no standalone
FW 22 build left in the tree — `pio run` against the current source produces
FW 23 — so **flash FW 23 for this sitting, not FW 22**; every "FW 22" /
"protocol 8" reference below describes the hand-lane feature this ladder
measures and stays accurate, but the board and host you actually run are
FW 23 / protocol 9. **A board left on FW 22 is DARK against this host**
(PROTOCOL_VERSION 8 ≠ 9 — `decode_frame` rejects every frame both ways; see
row 6).

A measurement sitting, not a gate. R3's first powered sitting (2026-09-13,
`logbook/2026-09-14-skill-stack-r3-first-powered-sitting.md`) threw about
25 % fast at a 0.9 m commanded apex: announced 4.17 m/s, ball apex 1.38 m,
hand encoder peak 161 rev/s against 128 commanded, hand current saturated at
47.8 A. The learner's box could not reach it. The follow-up analysis
(`logbook/2026-09-14-skill-stack-r3-apex-ladder-prep.md`) found why the
Platform-Teensy engine never showed this: it sent the hand ODrive an
acceleration **torque feedforward** with every frame, and the streamed hand
lane sent zero (`Teensy_code_canbridge/leg_interp.cpp:1021`, pre-FW-22).
Without it the velocity loop built the acceleration torque out of tracking
error and paid it back as overspeed after the ramp.

Since that prep, the hand C2 + torque-FF plumbing has landed (uncommitted,
not yet flashed): can-bridge FW 22 / PROTOCOL_VERSION 8 replaces off-knot
arrival-time playback with phase-locked, knot-aligned frames stamped with
their own play time (`t_origin_us`, flags bit5 `HAS_SCHED`), and the firmware
now computes an acceleration torque feedforward **in firmware**, from the
curve it is actually playing: `τ = fade · sat(Ks · J_HAND · 2π · a_cmd) +
bias`. The gain `K` rides the wire per frame (`hand_ff_gain`, ROS param
`hand_torque_ff_gain` on `teensy_bridge_node`, default 0 — goes straight to
the wire; the readback gate that used to force it to 0 was removed
2026-09-15, see `logbook/2026-09-15-hand-torque-ff-gate-removed.md`, so
confirm the hand ODrive's `input_torque_scale` by hand before raising K).
This sitting flashes FW 22 and flies the apex ladder as an **A/B**: arm A at `K=0` (FW 22's
scheduling alone, still zero torque FF — the direct comparison against the
2026-09-13 baseline) against arm B at `K=0.7` (the offline model's mid-gain
case; see the predictions table below).

This ladder answers three questions:

1. **Does FW 22's scheduling alone (arm A, K=0) change the overspeed?** The
   offline model says no — knot-aligned sampling barely moves the ratio
   versus the old off-knot stream (1.145 vs 1.143 at 0.9 m, K=0). A large
   change here would mean the off-knot-sampling story was wrong and the
   analysis needs reopening before arm B means anything.
2. **Does torque feedforward at K=0.7 close the gap?** The pre-registered
   criterion from the prep sitting was "K=0.7 → ratio ≤ 1.00". The offline
   model (`temp/probes/hand_cascade_ff/hand_cascade_ff_20260914T122549Z.md`)
   says this is **NOT SUPPORTED**: predicted K=0.7 ratios are 1.05× (0.9 m)
   and 1.04× (0.5 m), not ≤1.00 — the ball's own +23 % reflected inertia and
   the 50 A drive ceiling account for the gap, not a modelling error. The
   sitting's job is to confirm or refute this on hardware, not assume it.
3. **Operating point.** The highest apex at which the plant (whichever arm
   wins) is close enough to commanded for the learner to reach.

**If your physical intuition disagrees with this framing, that is
load-bearing signal — say so before step 1.**

### Pre-registered predictions (decided before the sitting)

Commanded peak hand acceleration for a single throw, from the planner
(`tools/probes/skills_single_site_sweep.py --study grid`, 2026-09-14) —
unaffected by K, planner output only:

| Apex (m) | 0.5 | 0.6 | 0.7 | 0.8 | 0.9 |
|---|---|---|---|---|---|
| Commanded peak hand acc (rev/s²) | 1706 | 1871 | 2309 | 2715 | 3097 |
| Commanded release speed (rev/s) | ≈ 96 | ≈ 105 | ≈ 114 | ≈ 121 | ≈ 128 |

**Offline torque-FF model** (`temp/probes/hand_cascade_ff/hand_cascade_ff_20260914T122549Z.md`,
knot-aligned rows — the FW 22 case; ladder apexes 0.6/0.7/0.8 were not
modelled, only the 0.5/0.9 m endpoints). These are **model predictions, not
measurements** — read the sitting's own numbers against them, not the other
way round:

| Apex (m) | K | Peak meas/cmd (model) | Peak iq (A, model) |
|---|---|---|---|
| 0.9 | 0 | 1.145 | 50.0 |
| 0.9 | 0.7 | 1.054 | 49.5 |
| 0.9 | 1.0 | 1.021 | 49.4 |
| 0.5 | 0 | 1.109 | 28.8 |
| 0.5 | 0.7 | 1.041 | 28.3 |
| 0.5 | 1.0 | 1.016 | 28.2 |

- **The pre-registered "K=0.7 → ratio ≤ 1.00" criterion is NOT SUPPORTED** by
  this model at either endpoint (1.054 at 0.9 m, 1.041 at 0.5 m) — expect arm
  B to still read fast, not corrected.
- **Peak iq at K≥0.7 sits within 1 A of the 50 A drive limit at 0.9 m** —
  this is why the A/B stop criteria below include a live current cutoff at
  48 A, five rungs before the ladder reaches 0.9 m in either arm.
- **Verdict B (missing torque feedforward is most of the story), the prep
  analysis's call:** measured hand peak / commanded falls substantially from
  arm A to arm B and continues falling as K rises toward 1.0.
- **Verdict A (K makes little difference):** the ratio at K=0.7 reads within
  0.02 of the K=0 arm on every rung — the missing-FF story is wrong or the
  firmware path isn't delivering the modelled torque; stop and re-open the
  analysis rather than flying K=1.0 the same sitting.
- The ball follows the hand encoder at 32.567 mm/rev (0.95–1.05 on
  2026-09-13), so either channel reads the verdict; the table uses both.

**Measure only timing-robust quantities.** `/hand_telemetry` is stamped with
the Jetson's 100 Hz poll clock, not the bridge's sample time, and the
command echo is decimated 5:1 at the bridge (`telemetry.cpp:230-280`,
`teensy_bridge_node.py:2157-2172`, `:3486`). Peak speeds, peak current and the
ball's apex are robust; fitted delays and 10 ms accelerations are not.

---

## 1. Before the robot is powered

Every terminal: `source /opt/ros/foxy/setup.bash && source
~/Desktop/Jugglebot-skills/ros_ws/install/setup.bash`. Venv
(`source ~/Desktop/PDJ_venv/venv/bin/activate`) for anything marked "venv".

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git status -sb` | Clean, at or after the commit that landed this sheet (the hand C2 + torque-FF unit). |
| 2 | (ROS) `cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && source install/setup.bash && cd ..` | Builds. |
| 3 | (venv) `./run_tests.sh --full` | Green. Record the pass count in § 6. This is the gate the plan's Rigor rule requires before any powered sitting, and it is the ONLY place the new firmware-twin (`test_sched_c2_twin.py`, `test_hand_torque_ff_twin.py`) and native (`test_leg_interp.cpp`) suites run together with everything else. |
| 4 | **Re-apply the hand ODrive's CAN torque scale.** Over USB/odrivetool on the hand Pro (node 6): `odrv0.axis0.config.can.input_torque_scale = 1000`, then `odrv0.save_configuration()`. Or re-apply `config/ODrive config Files/odrive_pro_hand_config.json`. Leave `input_vel_scale` at 100. | Read back before continuing: `odrv0.axis0.config.can.input_torque_scale` reports `1000`. The host no longer gates K on this readback (removed 2026-09-15) — it goes straight to the wire, so getting this wrong before arm B means a wrong-current command, not a degraded one; confirm it here. |
| 5 | **With the launch DOWN**, flash can-bridge FW 23 (folds in the FW 22 hand C2 + torque-FF plumbing this sheet was written against, plus the FW 23 axis-silence watchdog — there is no separate FW 22 build in the tree) in lockstep with the v9 host build already in step 2: `cd ros_ws/src/jugglebot/Teensy_code_canbridge && pio run -e teensy41 -t upload`. | **The boot banner is the receipt** — `jugglebot-canbridge v23` on the console (open `pio device monitor -e teensy41 \| tee temp/logs/console_ff_ladder_$(date +%Y%m%d_%H%M).log` in its own terminal right after). A bare `pio run` (no `-t upload`) BUILDS ONLY and is NOT a flash — a matching hex md5 is not a flash receipt either; only the boot banner is. A board still reporting `v22` (or older) is UNFLASHED for this sitting — re-run the upload. |
| 6 | **Protocol 9 note.** If the host (v9, from step 2) and board (FW 23, from step 5) do not land in lockstep — e.g. the board is still on FW 22/protocol 8 — the symptom is **link darkness**, not a cable fault: `link=NO_HEARTBEAT` on `/link_status` with `decode_errors == rx_frames` — `decode_frame` rejects every frame both ways on a version mismatch. If you see this, check the boot banner version against `teensy_link/rpc_args.py::EXPECTED_BRIDGE_FW_VERSION` (23) before touching any cable. | No action unless it happens. |
| 7 | (venv, repo root) `PYTHONPATH=ros_ws/src/jugglebot python -c "from jugglebot.motion.skills import admissible as a; [print(b.site_pair, b.apex_band_m, b.flight_s) for b in a.load('config/generated/admissible_box.yaml')]"` | Two columns boxes plus five `('P1', 'P1')` boxes, apex bands 0.45–0.55, 0.55–0.65, 0.65–0.75, 0.75–0.85, 0.85–0.95 m, each with the flight range in § 4's table. |
| 8 | (venv, quiet machine — nothing else running) `for A in 0.5 0.6 0.7 0.8 0.9; do python3 tests/hardware/skills_plan_bench.py --rehearse --pattern self-toss --arm A --attempts 3 --n-throws 1 --apex-m $A; done 2>&1 \| tee temp/logs/apex_ladder_rehearse_$(date +%Y%m%d).log` | Every rung: `blas threads: 1`, **G1, G2 and G4 PASS** (three attempts, `ended_early=False (4/4 skills)`). This is an offline solve rehearsal only — it exercises no wire, so it reads the same whether FW 23 is flashed or not; run it after step 2's build so it's checked against the Jetson-side code this sitting actually carries. Known, pre-existing refusal: the closing REST is refused `LIMIT_JERK` when it is dispatched early in its 40 Hz tick (a deterministic sweep refuses 24 % of tick phases at 0.9 m, 52 % at 0.6 m; the bench's own timing hits it rarely). It ends the attempt AFTER the throw and catch were accepted, so it does not fail the rung. A THROW or CATCH refusal, or any other early end, fails the rung. |
| 9 | QTM: disable the `Catching Cone` rigid body; mask the Ball Butler reflectors | Hard precondition, unchanged from `session_skills_r3.md` row 10. |

## 2. Bring-up

Rows 11–16 and 18 of `session_skills_r3.md`, with the log names below. Hand
limits stay at the launch defaults (200 rev/s, 3500 rev/s²) for the whole
ladder in both arms — changing them would change what is being measured.

| # | Step | Expect |
|---|---|---|
| 10 | Load capture: `( while true; do echo "$(date +%H:%M:%S) $(cat /proc/loadavg)"; sleep 1; done ) \| tee temp/logs/loadavg_apex_ladder_$(date +%Y%m%d).txt` | One line a second. |
| 11 | `ros2 launch jugglebot jugglebot_launch.py record:=true auto_arm:=true 2>&1 \| tee temp/logs/launch_apex_ladder_$(date +%Y%m%d_%H%M).log` | Note the bag folder it prints. |
| 12 | `grep 'blas threads' temp/logs/launch_apex_ladder_*.log` | `blas threads: 1` for `trajectory_node` AND `skill_node`. |
| 13 | GUI (http://localhost:8081): start QTM streaming; **Home**, then **Activate**. | Hand parked at 0 rev. `/hand_telemetry` `pos_cmd` also reads 0.0 from here (the host writes the echo on a completed ACTIVATE since 2026-09-16 — the firmware's own echo uplink is event-driven off the streamed lane and the park does not touch it). A non-zero `pos_cmd` against a zero `pos_meas` is now a real disagreement worth reading, not the known artifact it was on 2026-09-16. |
| 14 | `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 300.0, leg_acc_limit_mmps2: 5000.0, leg_jerk_limit_mmps3: 150000.0}"` | `applied_*` echoes 300 / 5000 / 150000. |
| 15 | `ros2 param set /skill_node site_x_mm -50.0`, `... site_y_mm 0.0`, `... dwell_s 0.30`, `... n_throws 1` | Set once for the whole ladder. `dwell_s` is raised again at step 17 for the no-motion check only, and restored to this value at step 23. |
| 16 | `ros2 param set /teensy_bridge_node hand_torque_ff_gain 0.0` | Explicit arm A value — do this even though 0.0 is the launch default, so § 6's log has a positive record of when arm A started. |

## 3. No-motion / low-motion stream check (before any throw fires)

This checks that the scheduled hand lane is healthy — clock-synced, playing,
and C2-continuous at the firmware's own promotion boundaries — while the only
hand motion on the wire is the opening REST's own lift, before the
ladder's first THROW is allowed to fire. (That lift is USUALLY static — the
hand is at the ACTIVATE park and the REST settles 0.31 rev above it — but it
is not guaranteed to be: since 2026-09-16 a fresh-origin window seeds its
hand from the MEASURED encoder rather than from the commanded value, so if
the hand starts anywhere else the REST carries it home over the 1.5 s floor
lift. `trajectory_node` prints ONE `HAND SEED RECONCILED for SETTLE:
commanded X rev vs MEASURED Y rev` line when that happens; it is
informational, not a fault, and it REPLACES the retired
`REJECTED_HAND_NOT_PARKED` refusal — a hand off the park no longer stops the
attempt, and DEACTIVATE/ACTIVATE is no longer the recovery for one.) It rides the FIRST real attempt of
the ladder (rung 0.5 m): the dwell
is temporarily lengthened so there is time to read the diagnostics during the
REST, and the attempt is stopped from here if anything below is wrong —
**the THROW has not happened yet at this point**, so stopping is free.

| # | Step | Expect |
|---|---|---|
| 16b | **Quarantine the 2026-09-16 memories BEFORE the first rung** (once per box): read `temp/learn/README_QUARANTINE_20260916.md` (in the `~/Desktop/Jugglebot-skills` worktree — `temp/` is per-checkout) and run the `mv` it names, from that directory. | The ten `temp/learn/arm[AB]-0*0-20260916` directories move under `temp/learn/_quarantine_20260916/`. 21 of their 43 rows are the pre-fix contamination (observed flights 2.2–3.0x commanded); `Memory._load` now drops out-of-band rows, but the survivors were learned against contaminated neighbours. Re-using one of those `plant_id`s would otherwise reload them. A rung whose `plant_id` directory is absent starts COLD (identity prior), which is what we want after the fix. |
| 17 | `ros2 param set /skill_node apex_m 0.5`, `... dwell_s 2.0` (temporary — long enough to read diagnostics twice during the REST), `... plant_id ffcheck-$(date +%Y%m%d)` (a throwaway id, not a ladder rung) | Set for this check only. |
| 18 | `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK` and `box OK` naming a `('P1', 'P1')` band containing 0.5 m. |
| 19 | Seat a ball; `ros2 service call skills/start_self_toss std_srvs/srv/Trigger` | Accepted — the opening REST installs and starts streaming immediately. |
| 20 | **Within the REST's dwell, before the THROW fires**, read `ros2 topic echo /link_status --once` | `time_synced: 1` (the bridge's wall anchor is set — without it every scheduled frame demotes to legacy and none of this check means anything); `hand_torque_ff_gain_requested: 0.0000` and `hand_torque_ff_gain_effective: 0.0000` (arm A). **FW 23 axis-silence watchdog fields** (fresh boot, healthy bus): `can_fault_leg: 255` (no CAN_BUS_DOWN trip since boot), `can_fault_count: 0`, `hb_stale_mask: 0` (no axis heartbeat older than 500 ms). A nonzero `can_fault_count` here means a trip already happened earlier in this session (check `can_fault_age_ms` and the log for the ERROR line); a nonzero `hb_stale_mask` is diagnostic-only (see `hb_stale_axes`) and does not by itself stop the check. |
| 21 | In the same window, read the console `[hand7]` line (from the `pio device monitor` opened at step 5) | `sched=play` (not `off` — confirms the REST is riding the scheduled lane, not a legacy fallback); `promo_dp=`, `promo_dv=`, `promo_da=` all ≈ 0 (a static REST has no knot-to-knot motion to promote through, so these should read at or near the printed precision's zero); `promo_over=0`; `stops=0 refused=0 expired=0 demoted=0` — any of these counting up during a clean, on-time REST stream means a frame is arriving late, out of order, or unstamped, and needs diagnosis before flying the ladder for real. |
| 22 | `ros2 topic echo /link_status --once \| grep interp_max_jitter_us` | Record the value. No pass bound exists yet (U2a residual: "measure at the first sitting") — note it here as the reference for later sittings; only a growing trend tick-over-tick, not a single reading, would indicate a real ISR-timing problem. |
| 23 | **Tracker sees the ball.** Let the check throw fire, then `ros2 topic echo /balls` | The announced ball appears with `tracking: 1` (CONFIRMED) within a few frames of release, and `time_at_land` is populated. `tracking: 0` (ANNOUNCED) all the way to the catch means the tracker has no candidate — the 2026-09-15 failure, where QTM labelled the ball `Ball Butler - 1` and the node forwarded only unlabelled markers. Check the `ball_tracker_node` startup line for `excluded_labels=('Platform', 'Base')` and `announced_gate=200mm`; if a NEW rigid body has appeared in the QTM model and claimed the ball, the ball is still eligible by design — but if the ball is being labelled as a `Platform`/`Base` marker, it is excluded and `ball_tracking.excluded_label_prefixes` needs the fix before flying. Not a stop condition for the ladder itself (the catch is aimed open-loop from the schedule), but the learner's outcome capture is tracker-sourced, so a blind tracker means no memory rows. |

**Stop here (before the THROW) if:**
- `time_synced: 0` — the stream cannot be scheduled at all; nothing below is meaningful.
- `[hand7]` reads `sched=off` during the REST — the frame isn't reaching the scheduled path; check `HAS_SCHED`/`HAS_V2` upstream before continuing.
- any of `promo_over`, `stops`, `refused`, `expired`, `demoted` is nonzero and still counting on a second read.

If the check is clean, let the attempt continue (it will throw at 0.5 m) —
this throw stands as arm A's first ladder rung; there is no need to abort and
redispatch. Restore the real ladder dwell before the next attempt:

| # | Step | Expect |
|---|---|---|
| 23 | `ros2 param set /skill_node dwell_s 0.30` | Back to the ladder's real value for every rung from here on. |

## 4. The ladder — arm A (K = 0), then arm B (K = 0.7), same sitting

Fly each arm's rungs in order **0.5, 0.6, 0.7, 0.8, 0.9 m** — lowest demand
first, so a knee is met from below. Each rung gets its own fresh `plant_id`,
so each rung's first two throws use the identity command. The learner may
adjust throw 3; the analysis compares each throw with the command it was
actually sent, so that does not spoil the measurement. **Rung order and
predictions are pre-registered above — do not reorder rungs or skip ahead
based on how a rung looks mid-flight.**

Arm A's 0.5 m rung is already flown (§ 3, row 19) — count it as rung 1 of arm
A rather than redispatching it.

For each apex `A` (write it as `050`, `060`, … in the id), each arm:

| # | Step | Expect |
|---|---|---|
| 24 | `ros2 param set /skill_node apex_m A` and `... plant_id <arm>-A-$(date +%Y%m%d)` (e.g. `armA-090-20260915`) | Fresh id per rung per arm. |
| 25 | `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK` and `box OK` naming a `('P1', 'P1')` band that contains `A`. A `box REFUSED ... at apex` line means step 7 was not satisfied — stop. |
| 26 | Seat a ball; `ros2 service call skills/start_self_toss std_srvs/srv/Trigger` | Accepted. One `skill announced ball 0` line, then a `CATCH-AIM skill 2: source=schedule landing=(…) mm t_land=…` line — the catch is aimed **open loop** now (see below), so `END NO_LANDING` must not appear at all. An `OUTCOME` line appears only when mocap happened to see the ball; its ABSENCE is expected at this sitting and is no longer a failure. **The `OUTCOME` line now lands LATER than it used to** (2026-09-16): it finalises `CAUGHT_WINDOW_S` = 0.35 s after the *observed* landing rather than 0.15 s after the *scheduled* one, and up to `CAUGHT_LAND_DEFER_CAP_S` = 0.35 s later still if the tracker's landing runs late — so expect it up to ~0.7 s after the scheduled touch-down, and do not read a missing row until a beat has passed. `caught=True` now means **the possession sensor read SEATED at some tick between 0.10 s before the landing and the finalise instant**, not "the cup was seated at one sampled instant" — a ball that seats and is re-thrown before the row closes still reads True, and a seat more than 0.35 s after the landing reads False (the window was widened from 0.25 s by owner ruling: a catch that SETTLES LATE is still a catch — the +282 ms arrival on armA-050 was re-thrown, not dropped). |
| 27 | Note in § 6: caught Y/N, and the `memory row appended ... y=[x, y, flight]` values. | Flight longer than `sc.flight_s(A)` means the throw was fast. |
| 28 | Repeat 24–27 until three throws are recorded for this rung. | |

**The catch is open loop from the throw state (owner decision 2026-09-15).**
At the 2026-09-15 sitting every one of 13 self-tosses ended
`END NO_LANDING: the tracker has no landing for ball 0 by the deadline` —
mocap never produced a marker for the flying ball, so the catch was never
aimed and the hand just returned to rest. The catch is now aimed at the
landing the schedule's own THROW was **commanded** to achieve, dispatched at
its scheduled instant, with no tracker call at all. What you should see per
catch is exactly one line:

```
CATCH-AIM skill 2: source=schedule landing=(-50.0, 0.0, 830.0) mm t_land=…
```

The `catch_aim_source` parameter selects where the aim comes from — set it
BEFORE `start_self_toss` (it is read when the schedule is compiled):

| Value | What it does | When to use it |
|---|---|---|
| `schedule` | The commanded landing, dispatched at the scheduled instant. No tracker, no re-aim. The default WHEN THIS LADDER FLEW (2026-09-16) and what its rows record. | Reproducing a row of this ladder. |
| `schedule_hand` | The same, corrected ONCE by the MEASURED hand launch-speed ratio `r = v_meas/v_cmd` from `/hand_telemetry` (never QTM). Logs `source=schedule_hand (r=1.086)` and, when the correction arrives after the dispatch, a second `CATCH-AIM … r=… Δt=+0.074 s` line. | The optional A/B once a rung's throw lands cleanly: the plant threw ~8–9 % fast at this sitting, which is ~74 ms of late arrival at 0.9 m. |
| `tracker` (default since 2026-09-18) | The tracker's CONVERGED fit aims the catch, with the commanded landing as the prior when no fit has converged, then later fits re-aim the committed catch (≤ 2, ≥ 10 mm / 10 ms apart). Logs `source=tracker`, `source=schedule` for the prior, and `RESEND …` / `RESEND-SKIPPED <reason> …` for the refine. | The live default; use `schedule` to re-fly this ladder's own rows. |

`ros2 param set /skill_node catch_aim_source schedule_hand`. A value that is
not one of the three logs an error and falls back to the live default
(`tracker` since 2026-09-18). Two lines
that are **not** failures: `CATCH-AIM-LATE …the theoretical aim stands` (the
hand ratio did not arrive in time to splice — the catch still flies on the
commanded landing) and `CATCH-AIM-HAND-REFUSED …` (the re-aim's solve was
refused; the committed catch stands).

**Expected, not a stop:** an attempt that ends `LIMIT_JERK` on the closing
REST after the catch (the pre-existing refusal noted at step 8 — up to about
half the attempts at some apexes). The throw and catch are already recorded,
and the catch's own rest tail brings the machine to rest. Note it in § 6 and
carry on.

**Switching from arm A to arm B**, after arm A's five rungs are complete:

| # | Step | Expect |
|---|---|---|
| 29 | Confirm `input_torque_scale=1000` in the ODrive GUI (persists across power cycles) | `1000`. The host no longer gates K on a readback (removed 2026-09-15) — a wrong scale here reaches the wire as a wrong-current command, not a degraded one, so confirm it before raising K rather than after. |
| 30 | `ros2 param set /teensy_bridge_node hand_torque_ff_gain 0.7` | `ros2 param get` echoes `0.7`. Record the wall-clock time — the close-out probe needs it to separate arm A from arm B in the one bag. |
| 31 | `ros2 topic echo /link_status --once` | `hand_torque_ff_gain_requested: 0.7000` and `hand_torque_ff_gain_effective: 0.7000` — the wire gain follows the param directly; if `effective` stays `0.0000` here, something else is wrong (e.g. no hand-bearing frame is being built) — stop and diagnose rather than flying arm B. |
| 32 | Repeat rows 24–28 for apexes 0.5, 0.6, 0.7, 0.8, 0.9 m, `plant_id` prefix `armB-` | Same procedure, arm B. |

### A/B stop criteria (either arm, every rung)

**Stop the ladder — do not climb further — if any of these happens:**
- a `MAX_DEVIATION` latch on any axis (recover with CLEAR_ERRORS, then
  DEACTIVATE and ACTIVATE before anything else — the latch line now names the
  axis);
- **peak `iq_meas` on `/hand_telemetry` reaches or exceeds 48 A** (2 A under
  the 50 A drive limit — the offline model already predicts ≈49–50 A at
  K ≥ 0.7 near 0.9 m, so this is expected to bind before the ladder's top
  rung, not a surprise);
- **the `[hand7]` `tclamp=` counter is climbing on repeated reads** (heartbeat
  bit 14, `HAND_TORQUE_CLAMP` — the torque command is saturating against
  `HAND_TORQUE_FF_CLAMP_NM` repeatedly, not as a single transient at a knot
  seam);
- **`[hand7]` reads `sched=HOLD-LATCHED`** (heartbeat bit 15,
  `SCHED_HOLD_LATCHED` — the scheduled lane refused a resume and is holding;
  recover per the latch note above, and treat the refused frame as a NEW
  finding, not routine);
- a ball leaves the capture volume or clears the cup by a margin you judge
  unsafe;
- `skills/check` shows any refusal other than a transient `REJECTED_NOT_LEVELLED`
  before the first pre-level;
- **a `CAN_BUS_DOWN` flash with the new ERROR line** (`Teensy guard FAULT:
  CAN_BUS_DOWN — leg <L> silent on the Jugglebot bus for <age> ms (trip #<n>
  since boot)…`): this is the FW 23 axis-silence watchdog, it self-clears and
  the firmware stows on its own — record leg/age/count from `/link_status`
  (`can_fault_leg`/`can_fault_age_ms`/`can_fault_count`), re-ACTIVATE after the
  stow, and continue the ladder. **A second `CAN_BUS_DOWN` flash in the same
  sitting → stop and read the bag** rather than continuing — one is the
  load-gated dropout the watchdog was built to tolerate, two is a pattern.

A stopped ladder is still a result: the rungs flown (in whichever arm) are
the curve.

## 5. Close-out

| # | Step | Expect |
|---|---|---|
| 33 | `ros2 param set /teensy_bridge_node hand_torque_ff_gain 0.0` | Back to the fail-safe default before deactivating. |
| 34 | `ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'deactivate'"` | Robot stows. |
| 35 | Stop the launch and the load capture. | |
| 36 | (venv) `python tools/probes/hand_overspeed_bag_probe.py --bag ~/Desktop/rosbags/<bag id> --until <row 30's wall-clock time>` | Arm A's rows. One row per stroke; the CSV lands under `temp/probes/`. |
| 37 | (venv) `python tools/probes/hand_overspeed_bag_probe.py --bag ~/Desktop/rosbags/<bag id> --since <row 30's wall-clock time>` | Arm B's rows, same bag, split at the gain-change timestamp recorded in row 30. |
| 38 | Send the bag id and the paths: `temp/logs/launch_apex_ladder_*.log`, `temp/logs/loadavg_apex_ladder_*.txt`, `temp/logs/console_ff_ladder_*.log`, both probe CSVs, `temp/learn/armA-*/memory.csv`, `temp/learn/armB-*/memory.csv`. | Paths, not pasted logs. |

### Swept admissible boxes for this ladder

`config/generated/admissible_box.yaml`, `tools/admissible_sweep.py --site-pairs
both --single-apex 0.5 0.6 0.7 0.8 0.9 --dwell-s 0.30 --leg-vel 300 --leg-acc
5000 --leg-jerk 150000 --hand-acc 3500 --single-site-xy=-50,0` (2026-09-14).
The **reach** column is how fast a plant the learner can still centre: a
ball that flies `r` times too fast needs a commanded flight of
`nominal / r`, so reach = nominal flight / lowest admitted flight.

| Apex band (m) | Nominal flight (s) | Admitted flight (s) | Reach | Landing xy (mm) |
|---|---|---|---|---|
| 0.45–0.55 | 0.639 | 0.511–0.703 | 1.25 | x 0, y 0 |
| 0.55–0.65 | 0.700 | 0.560–0.770 | 1.25 | x 0…+10, y 0 |
| 0.65–0.75 | 0.756 | 0.605–0.831 | 1.25 | x 0, y 0 |
| 0.75–0.85 | 0.808 | 0.646–0.848 | 1.25 | x 0, y 0 |
| 0.85–0.95 | 0.857 | 0.686–0.857 | 1.25 | x 0…+40, y 0 |

Sweep wall time 253 s. **The learner has flight authority on every rung but
almost no xy authority**: every landing rectangle contains the identity
offset, and at most apexes it is only that point. A ring of small landing
offsets (±10–20 mm) fails the chained catch's margin at flights near 0.64 s
while zero and larger offsets pass, so the largest rectangle that also
contains zero collapses. That is filed as a planner artefact; it does not
affect this ladder, which aims every throw at the nominal landing and flies
single throws rather than the chained catch the sweep certifies.

### Decision rule for K (pre-registered)

- **Operating apex for R3's gate sitting** = the highest rung, in the better
  of the two arms, whose measured ball/commanded speed ratio is at most
  0.9 × that rung's reach (table below) AND whose peak hand current stayed
  under 48 A.
- **If arm B's ratio at K=0.7 does not improve on arm A's by at least half
  the model's predicted gap** (model: 1.145→1.054 at 0.9 m, 1.109→1.041 at
  0.5 m — i.e. arm B should close at least ~45 % of arm A's overshoot): the
  torque-feedforward mechanism is not delivering what the model predicts —
  stop and re-open the analysis before flying K=1.0 or any higher gain.
- **If arm B still exceeds the reach bound on every rung** (the model's own
  prediction, since K=0.7 was never expected to reach ≤1.00): that is the
  EXPECTED outcome per the pre-registered model, not a new finding by
  itself. Report the measured ratios and defer the K=1.0 / flash-worthiness
  decision to the owner rather than concluding anything further this
  sitting — K=1.0 was not flown here (arm A and arm B only).
- The owner makes the final call; this rule is what the numbers are read
  against.

### A note on the disable edge (recorded, not diagnosed here)

`vel_ff` persists on the hand ODrive after hand TX stops (pre-existing since
FW 17; the FW 22 drain deliberately zeros only `input_torque`, leaving
`pos`/`vel_ff` bit-identical to the last frame — `leg_interp.cpp`'s drain
block, around :1523). If a drain or a `vel_ff`-driven creep past the held setpoint is
observed on any disable edge this sitting (DEACTIVATE, an E-STOP, a
`skills/stop`), **record it in § 6** — whether the drain should also zero
`vel_ff` is an owner decision still open, not something to fix or work around
live.

## 6. Results

| Item | Result |
|---|---|
| Date, commit, bag, `--full` count | 2026-09-16 16:22; bag `~/Desktop/rosbags/2026-09-16_16-22-22`; log `temp/logs/launch_r2gate_20260916_1622.log`; `--full` count carried from the pre-sitting build/test gate (see the sitting's own launch log for the exact count — not re-quoted here to avoid a stale number drifting from the source). |
| FW 23 boot banner confirmed (step 5) | Confirmed — the sitting ran FW 23 / protocol 9 throughout (link stayed synced for the whole bag, no `NO_HEARTBEAT`/decode-error darkness observed). |
| § 3 no-motion check (row 20–22): time_synced / sched= / promo_dp,dv,da / promo_over,stops,refused,expired,demoted / interp_max_jitter_us | Clean — the ladder proceeded through all ten rung/arm cells with `time_synced: 1` and no `sched=off`/`HOLD-LATCHED` reported for the whole sitting; no stop/refused/expired/demoted counters climbing were flagged. |
| Rungs flown, arm A (stopped early? why) | All five rungs (0.5, 0.6, 0.7, 0.8, 0.9 m) flown, 4 attempts each (chained self-tosses per attempt). Not stopped early — completed the full arm. |
| Rungs flown, arm B (stopped early? why) | All five rungs (0.5, 0.6, 0.7, 0.8, 0.9 m) flown, 4 attempts each. Not stopped early — completed the full arm. |
| Per-rung, per-arm measured hand peak / commanded (3 throws) | From `hand_overspeed_bag_probe` (peak meas/cmd per stroke; max / mean over EVERY stroke flown in the rung's time window, learner-shortened attempts included — `temp/logs/ladder2_probe_arm{A,B}_20260916.log`, windows = the rung's `schedule compiled` instants in the launch log): armA-050 1.096/1.077 (n=4); armA-060 1.051/1.050 (n=4); armA-070 1.220/1.127 (n=4); armA-080 1.147/1.094 (n=8); armA-090 1.113/1.087 (n=10); armB-050 1.048/1.026 (n=4); armB-060 1.014/1.013 (n=4); armB-070 1.020/1.015 (n=4); armB-080 1.013/1.011 (n=4); armB-090 1.013/1.004 (n=8). Arm B collapses to 1.00–1.05× at every rung (vs arm A's 1.05–1.22×), closer to the offline model's K=1.0 prediction than its own K=0.7 prediction — the pre-registered "K=0.7 → ratio ≤ 1.00" criterion is still not met (every rung's mean sits at 1.00–1.03, peak up to 1.048), but Verdict B (missing FF is most of the story) is strongly supported and Verdict A (K makes little difference) is refuted. Source: `temp/logs/ladder2_probe_armA_20260916.log` / `..._armB_...log`, CSV `temp/probes/hand_overspeed_2026-09-16_16-22-22.csv`. |
| Per-rung, per-arm ball / commanded and ball apex (m) | Raw-mocap re-measurement (ground truth, independent of the tracker's chained-id `/balls` stream — see the metrics report's Follow-up section), apex ratio (meas/cmd) and mean measured apex: armA-050 1.227 (0.540 m), armA-060 1.285 (0.592 m), armA-070 1.268 (0.889 m), armA-080 1.250 (0.834 m), armA-090 1.244 (0.861 m); armB-050 1.097 (0.550 m), armB-060 1.070 (0.643 m), armB-070 1.089 (0.763 m), armB-080 1.070 (0.856 m), armB-090 1.067 (0.869 m). Arm A apex ratio 1.23–1.29×; arm B 1.07–1.10× — a real, substantial improvement of similar size to the hand meas/cmd gap closing. tof ratio improves less (arm A ~1.17–1.26×, arm B ~1.11–1.15×) — the torque FF closes most, not all, of the apex gap and a smaller fraction of the flight-time gap. |
| Per-rung, per-arm peak hand current (A) | Peak `\|iq\|` (max) per rung: armA-050 11.7, armA-060 17.2, armA-070 24.2, armA-080 31.6, armA-090 30.1; armB-050 19.8, armB-060 20.2, armB-070 20.0, armB-080 23.8, armB-090 39.9. Current roughly doubled at low apex (armA-050 mean 10.0 A → armB-050 mean 19.2 A, since the FF adds commanded current directly) but never approached the 48 A live cutoff / 50 A drive ceiling — max observed 39.9 A (armB-090). |
| `tclamp=` / `sched=HOLD-LATCHED` observed on any rung? | Not observed / not flagged during the sitting. |
| Caught per rung, per arm | Operator notes (verbatim) per rung: armA-050 "worked, hand started moving for catches slightly early"; armA-060 "very clean catches, almost perfect"; armA-070 "messy, first two throws a little spatially off, catches early, all caught"; armA-080 "first catch decent, second had the hand retract immediately with SPLICE_TOO_LATE, further attempts didn't improve"; armA-090 "decent, a little spatially off, third attempt very short throws"; armB-050 "clean, catches a little early"; armB-060 "very clean, catches slightly early"; armB-070 "clean, catches a little early"; armB-080 "fairly clean, one ball contact with the top of the hand stroke, recovered"; armB-090 "fairly clean; second and third attempts platform wobble, throw missed, throw very low" (the wobble/low-throw events are diagnosed and fixed in `logbook/2026-09-16-outcome-landing-frozen-at-the-crossing.md`, commit `cd8cd82`, not re-narrated here). Quantitative catch timing: the hand's scheduled aim ran EARLY relative to the ball's measured landing at essentially every throw in both arms (mean `early_s` per rung -0.04 s to -0.20 s, raw-mocap re-measurement), matching the operator's "catches slightly early" note; the pattern does not scale cleanly with apex and is present in both arms (the torque FF does not remove it) — it tracks the flight-time residual (Table 2), not the hand-overspeed residual (Table 1) that the torque FF fixes. |
| Drain / vel_ff note (see above) | No drain / `vel_ff`-driven creep past the held setpoint was flagged on any disable edge this sitting. |
| Verdict (A / B / other) and the chosen operating apex and K | **Verdict B** (missing torque feedforward was most of the story) — strongly supported, more strongly than the offline model predicted; Verdict A (K makes little difference) is refuted. The pre-registered "K=0.7 → ratio ≤ 1.00" criterion is NOT met literally (measured 1.00–1.05× at K=0.7, not ≤1.00), but **the owner's decision is to adopt K=0.7 as the operating value and close the ladder as a measurement** — the residual ~1–10 % is launch dynamics for the memory-based learner to correct, not a hand-tuning problem. `hand_torque_ff_gain` = 0.7 is now the `jugglebot_launch.py` default (`ros_ws/src/jugglebot/launch/jugglebot_launch.py`); K=1.0 was deliberately not flown this sitting (see `logbook/2026-09-16-apex-ladder-k07-ab-result.md` for why not now). Full detail, Discussion and verification triples in that entry. |
