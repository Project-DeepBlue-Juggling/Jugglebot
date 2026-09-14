# R3 apex ladder — how fast does the streamed hand throw, apex by apex?

A measurement sitting, not a gate. R3's first powered sitting (2026-09-13,
`logbook/2026-09-14-skill-stack-r3-first-powered-sitting.md`) threw about
25 % fast at a 0.9 m commanded apex: announced 4.17 m/s, ball apex 1.38 m,
hand encoder peak 161 rev/s against 128 commanded, hand current saturated at
47.8 A. The learner's box could not reach it. The follow-up analysis
(`logbook/2026-09-14-skill-stack-r3-apex-ladder-prep.md`) found why the
Platform-Teensy engine never showed this: it sent the hand ODrive an
acceleration **torque feedforward** with every frame, and the streamed hand
lane sends zero (`Teensy_code_canbridge/leg_interp.cpp:1021`). Without it the
velocity loop builds the acceleration torque out of tracking error and pays
it back as overspeed after the ramp.

This ladder flies single self-tosses at five apexes with the hand limits
unchanged, so the overspeed can be read against commanded acceleration. It
answers two questions:

1. **Operating point.** The highest apex at which the plant is close enough
   to commanded for the learner to reach — the apex R3's gate sitting flies.
2. **Baseline.** The before-curve for a later hand torque-feedforward flash
   (firmware), which would be flown on the same ladder.

**If your physical intuition disagrees with this framing, that is
load-bearing signal — say so before step 1.**

### Pre-registered predictions (decided before the sitting)

Commanded peak hand acceleration for a single throw, from the planner
(`tools/probes/skills_single_site_sweep.py --study grid`, 2026-09-14):

| Apex (m) | 0.5 | 0.6 | 0.7 | 0.8 | 0.9 |
|---|---|---|---|---|---|
| Commanded peak hand acc (rev/s²) | 1706 | 1871 | 2309 | 2715 | 3097 |
| Commanded release speed (rev/s) | ≈ 96 | ≈ 105 | ≈ 114 | ≈ 121 | ≈ 128 |

- **Verdict B (missing torque feedforward), the analysis's call:** measured
  hand peak / commanded rises with apex — about 1.03–1.06 at 0.5 m, a knee as
  peak current nears the 50 A limit around 0.8 m, 1.13–1.27 at 0.9 m.
- **Verdict A (same physics as the old engine, just more demand):** the ratio
  stays flat at 1.00 ± 0.05 on every rung.
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
| 1 | `cd ~/Desktop/Jugglebot-skills && git status -sb` | Clean, at or after the commit that landed this sheet. |
| 2 | (venv) `./run_tests.sh --full` | Green. Record the pass count in § 5. |
| 3 | (ROS) `cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && source install/setup.bash && cd ..` | Builds. Needed: skill_node's apex-scoped box lookup, the sitting-1 fixes. |
| 4 | (venv, repo root) `PYTHONPATH=ros_ws/src/jugglebot python -c "from jugglebot.motion.skills import admissible as a; [print(b.site_pair, b.apex_band_m, b.flight_s) for b in a.load('config/generated/admissible_box.yaml')]"` | Two columns boxes plus five `('P1', 'P1')` boxes, apex bands 0.45–0.55, 0.55–0.65, 0.65–0.75, 0.75–0.85, 0.85–0.95 m, each with the flight range in § 4's table. |
| 4a | (venv, quiet machine — nothing else running) `for A in 0.5 0.6 0.7 0.8 0.9; do python3 tests/hardware/skills_plan_bench.py --rehearse --pattern self-toss --arm A --attempts 3 --n-throws 1 --apex-m $A; done 2>&1 \| tee temp/logs/apex_ladder_rehearse_$(date +%Y%m%d).log` | Every rung: `blas threads: 1`, **G1, G2 and G4 PASS** (three attempts, `ended_early=False (4/4 skills)`). Known, pre-existing refusal: the closing REST is refused `LIMIT_JERK` when it is dispatched early in its 40 Hz tick (a deterministic sweep refuses 24 % of tick phases at 0.9 m, 52 % at 0.6 m; the bench's own timing hits it rarely). It ends the attempt AFTER the throw and catch were accepted, so it does not fail the rung. A THROW or CATCH refusal, or any other early end, fails the rung. |
| 5 | QTM: disable the `Catching Cone` rigid body; mask the Ball Butler reflectors | Hard precondition, unchanged from `session_skills_r3.md` row 10. |

## 2. Bring-up

Rows 11–16 and 18 of `session_skills_r3.md`, with the log names below. Hand
limits stay at the launch defaults (200 rev/s, 3500 rev/s²) for the whole
ladder — changing them would change what is being measured.

| # | Step | Expect |
|---|---|---|
| 6 | Load capture: `( while true; do echo "$(date +%H:%M:%S) $(cat /proc/loadavg)"; sleep 1; done ) \| tee temp/logs/loadavg_apex_ladder_$(date +%Y%m%d).txt` | One line a second. |
| 7 | `ros2 launch jugglebot jugglebot_launch.py record:=true auto_arm:=true 2>&1 \| tee temp/logs/launch_apex_ladder_$(date +%Y%m%d_%H%M).log` | Note the bag folder it prints. |
| 8 | `grep 'blas threads' temp/logs/launch_apex_ladder_*.log` | `blas threads: 1` for `trajectory_node` AND `skill_node`. |
| 9 | GUI (http://localhost:8081): start QTM streaming; **Home**, then **Activate**. | Hand parked at 0 rev. |
| 10 | `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 300.0, leg_acc_limit_mmps2: 5000.0, leg_jerk_limit_mmps3: 150000.0}"` | `applied_*` echoes 300 / 5000 / 150000. |
| 11 | `ros2 param set /skill_node site_x_mm -50.0`, `... site_y_mm 0.0`, `... dwell_s 0.30`, `... n_throws 1` | Set once for the whole ladder. |

## 3. The ladder (ascending apex, three single throws per rung)

Fly the rungs in order **0.5, 0.6, 0.7, 0.8, 0.9 m** — lowest demand first,
so a knee is met from below. Each rung gets its own fresh `plant_id`, so each
rung's first two throws use the identity command. The learner may adjust
throw 3; the analysis compares each throw with the command it was actually
sent, so that does not spoil the measurement.

For each apex `A` (write it as `050`, `060`, … in the id):

| # | Step | Expect |
|---|---|---|
| 12 | `ros2 param set /skill_node apex_m A` and `ros2 param set /skill_node plant_id ladder-A-$(date +%Y%m%d)` | Fresh id per rung. |
| 13 | `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK` and `box OK` naming a `('P1', 'P1')` band that contains `A`. A `box REFUSED ... at apex` line means step 4 was not satisfied — stop. |
| 14 | Seat a ball; `ros2 service call skills/start_self_toss std_srvs/srv/Trigger` | Accepted. One `skill announced ball 0` line, then an `OUTCOME` line with a landing (not `NO_LANDING` — that was fixed at sitting 1). |
| 15 | Note in § 5: caught Y/N, and the `memory row appended ... y=[x, y, flight]` values. | Flight longer than `sc.flight_s(A)` means the throw was fast. |
| 16 | Repeat 14–15 until three throws are recorded for this rung. | |

**Expected, not a stop:** an attempt that ends `LIMIT_JERK` on the closing
REST after the catch (the pre-existing refusal in step 4a — up to about half
the attempts at some apexes). The throw and catch are already recorded, and
the catch's own rest tail brings the machine to rest. Note it in § 5 and
carry on.

**Stop the ladder — do not climb further — if any of these happens:**
- a `MAX_DEVIATION` latch on any axis (recover with CLEAR_ERRORS, then
  DEACTIVATE and ACTIVATE before anything else — the latch line now names the
  axis);
- a ball leaves the capture volume or clears the cup by a margin you judge
  unsafe;
- `skills/check` shows any refusal other than a transient `REJECTED_NOT_LEVELLED`
  before the first pre-level.

A stopped ladder is still a result: the rungs flown are the curve.

## 4. Close-out

| # | Step | Expect |
|---|---|---|
| 17 | `ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'deactivate'"` | Robot stows. |
| 18 | Stop the launch and the load capture. | |
| 19 | (venv) `python tools/probes/hand_overspeed_bag_probe.py --bag ~/Desktop/rosbags/<bag id>` | One row per stroke; the CSV lands under `temp/probes/`. |
| 20 | Send the bag id and the paths: `temp/logs/launch_apex_ladder_*.log`, `temp/logs/loadavg_apex_ladder_*.txt`, the probe CSV, `temp/learn/ladder-*/memory.csv`. | Paths, not pasted logs. |

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

### Decision rule (pre-registered)

- **Operating apex for R3's gate sitting** = the highest rung whose measured
  ball/commanded speed ratio is at most 0.9 × that rung's reach AND whose
  peak hand current stayed under 45 A (5 A under the 50 A limit).
- **If every rung reads 1.00 ± 0.05** (verdict A): the torque-feedforward
  story is wrong — stop and re-open the analysis before any flash.
- **If even 0.5 m fails the rule**: fly the torque-feedforward flash before
  R3's gate sitting, not a lower apex.
- The owner makes the final call; this rule is what the numbers are read
  against.

## 5. Results

| Item | Result |
|---|---|
| Date, commit, bag, `--full` count | |
| Rungs flown (stopped early? why) | |
| Per-rung measured hand peak / commanded (3 throws) | |
| Per-rung ball / commanded and ball apex (m) | |
| Per-rung peak hand current (A) | |
| Caught per rung | |
| Verdict (A / B / other) and the chosen operating apex | |
