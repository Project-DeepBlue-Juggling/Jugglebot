# R2 hardware gate — per-skill plan time on the loaded Jetson

> ⬜ **NOT YET FLOWN.** One short sitting, before R3. The robot is powered and
> activated but **never moves**: the wire stays disarmed the whole time.

Skill-stack R2's one outstanding gate (`plans/active/two-ball-skill-stack.md` § 4
R2: *"per-skill plan < 50 ms measured on the Jetson with the launch up and a bag
recording"*). Entry: `logbook/2026-09-12-skill-stack-r2-skills-schedule-stream.md`.
Everything here is run by the operator. Driver:
`tests/hardware/skills_plan_bench.py` (pure core tested in
`tests/ros/test_skills_plan_bench.py`).

**What this sitting is, in five lines.**
1. The driver plays the real columns schedule (0.9 m, 100 mm, dwell 0.30 s, 20
   throws) through the real `trajectory/install_segment` in the launched
   `trajectory_node`, with made-up ball landings, and times every solve.
2. **Nothing moves.** The launch runs with `auto_arm:=false` and you never arm,
   so every setpoint is dropped at the bridge. The planner solves from the
   *commanded* state, never the measured one, so these are the same solves an
   armed session would run.
3. The load is the sitting's own: launch up, bag recording, GUI open, QTM
   streaming. That is the point — an idle box measured below the knee has misled
   this project before (UH-3, 2026-09-06).
4. No ball, no `level`, no `skills/start_columns`, no `/set_setpoint_output`.
5. About 30 minutes including bring-up. Send back three files and the bag id.

---

## 0. Why the robot never moves (and what you can push back on)

The gate is plan time, and the plan is a function of the commanded state, the
goal and the limits — not of where the legs physically are. Moving the robot
would add the never-flown 200 000 mm/s³ jerk ceiling to a timing measurement,
and that ramp belongs to R3's first sitting, not to this one. Two safety belts:
the driver **refuses to start** unless `/link_status` reads `mpc_active = 0`,
and it **stops and holds** if the wire ever reads armed or an accepted install
comes back without the `[wire DISARMED …]` marker.

If your physical intuition says the disarmed box is not the loaded box — for
example that the armed bridge's UDP traffic loads the Jetson materially more —
that is load-bearing, say so, and we add an armed row.

## 1. Before the robot is powered (no ROS, any time)

Every terminal in this sheet: `source /opt/ros/foxy/setup.bash && source
~/Desktop/Jugglebot-skills/ros_ws/install/setup.bash`. **Your `.zshrc` sources
the MAIN checkout's install**, which has no `install_segment`; the worktree's
must be sourced on top. Use `python3`, not the project venv.

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git status -sb` | Clean, at or after the commit that landed this sheet. |
| 2 | `cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && source install/setup.bash && cd ..` (the new srv needs both packages) | Builds. |
| 3 | `python3 tests/hardware/skills_plan_bench.py --dry-run` | Prints one attempt's 22 skills (1 THROW, 19 CATCH+throw, 1 CATCH, 1 REST; every attempt also runs one REST pre-position before them), the two splice budgets **125 ms** (handoff) and **75 ms** (unpinned), and the five gates. |
| 4 | `python3 tests/hardware/skills_plan_bench.py --rehearse --arm B --attempts 1` | The real planner in real time with no ROS; G3 and G5 read SKIP (no emitter, no wire). **G1, G2, G4 PASS: every scheduled install accepted, zero `SPLICE_TOO_LATE`.** Reference, this command on the idle Jetson 2026-09-13: G1 worst **48.31 ms** over 57 solves, carried by the CATCH+throw handoffs (p50 29.9 / p95 43.9 ms); re-sends p50 10.5 / max 27.0 ms, all 34 refused (expected); G2 handoff max 48.3 of 125 ms, unpinned max 27.0 of 75 ms. Note your max — it is the idle baseline the loaded rows are read against, and it already sits 2 ms under G1's bar (§ 4). |

## 2. Bring-up (launch UP, robot powered, wire DISARMED)

| # | Step | Expect |
|---|---|---|
| 5 | Load capture, in its own terminal, left running: `( while true; do echo "$(date +%H:%M:%S) $(cat /proc/loadavg)"; sleep 1; done ) \| tee temp/logs/loadavg_r2gate_$(date +%Y%m%d).txt` | One line a second. |
| 6 | `ros2 launch jugglebot jugglebot_launch.py record:=true auto_arm:=false 2>&1 \| tee temp/logs/launch_r2gate_$(date +%Y%m%d_%H%M).log` | Note the bag folder it prints. |
| 7 | `grep 'blas threads' temp/logs/launch_r2gate_*.log` | `blas threads: 1` for `trajectory_node` (and the other planner nodes). **Anything else: stop** — the E-STOP class of 2026-09-06 is back. |
| 8 | `ros2 service list \| grep install_segment` | `/trajectory/install_segment`. Missing = the launch terminal sourced the main install (step 1's note). |
| 9 | Open the GUI (http://localhost:8081) and leave it open. Start QTM streaming. | Both are part of the load being measured. |
| 10 | GUI: **Home**, then **Activate**. **Do not press `level`.** | Robot at the active pose, hand parked at 0 rev. With `auto_arm:=false` the orchestrator skips arming. |
| 11 | `ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'trajectory'"` | `/trajectory/status` mode `TRAJECTORY`. |
| 12 | `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 300.0, leg_acc_limit_mmps2: 5000.0, leg_jerk_limit_mmps3: 200000.0}"` | `applied_*` echoes **300 / 5000 / 200000** (all at or under the YAML ceilings). |
| 13 | `timeout 5 ros2 topic echo /link_status \| grep -A1 'key: mpc_active' \| head -2` | `value: '0'`. **If it reads `'1'`, stop**: something armed the wire. |
| 14 | `python3 tests/hardware/skills_plan_bench.py --check` | **P1–P7 all PASS.** A refusal prints every failing check at once with the command that fixes it (§ 5). |

Why step 10 skips `level` and why the driver pre-positions (both are R3 items,
not defects of this sheet): the skill path has no pre-level and no floor lift
yet. Measured 2026-09-13 offline, a THROW straight from the hand's 0 rev park
(cup 679.6 mm, 10 mm under the 689.6 mm planner floor) refuses `HAND_STROKE`,
so every attempt starts with one REST install to the site's rest height; and a
loaded levelling correction brings back R1's knot-0 tilt snap, which changes
jerk verdicts but not solve cost. P5 refuses a session with a correction loaded.

## 3. The gate

Launch up, robot activated, wire disarmed, GUI open, QTM streaming, bag
recording. No ball. The robot must not move at any point; if it does, E-STOP
and send me everything.

| # | Row | Command | Pass |
|---|---|---|---|
| 15 | **A — the schedule** (perfect landings: the launch, the handoffs, the last catch, the rest) | `python3 tests/hardware/skills_plan_bench.py --arm A` | Verdict **G1–G5 PASS**, exit 0. |
| 16 | **B — with re-sends** (±3 mm landing jitter forces catch re-aims at the general lead; most of them refuse `LIMIT_ACC`/`LIMIT_JERK`, which is expected and still a timed solve) | `python3 tests/hardware/skills_plan_bench.py --arm B` | Verdict **G1–G5 PASS**, exit 0. |
| 17 | **C — margin, NOT gating** (row 16 with two extra busy cores) | `for i in 1 2; do ( timeout 150 python3 -c 'while True: pass' ) & done; python3 tests/hardware/skills_plan_bench.py --arm B --label stress2` | Record the verdict and the G1/G2 maxima whatever they are. |

Each row runs 3 attempts of 20 throws (about 15 s each) and prints a per-kind
table and the verdict. The gates:

| Gate | Criterion | Source |
|---|---|---|
| G1 | Every solve's server `plan_wall_ms` **< 50 ms** (max, over all solves incl. refused re-sends and the pre-position) | Plan § 4 R2 gate |
| G2 | Zero `SPLICE_TOO_LATE`; client round trip max **< 125 ms** on handoff installs and **< 75 ms** on unpinned splices | `motion/skills/schedule.py` leads: (8 − 3) and (6 − 3) knots × 25 ms |
| G3 | `/trajectory/status.max_emit_gap_ms` **< 40 ms** throughout | Plan § 5 (the UH-6 numbers) |
| G4 | No attempt ends early; every scheduled install accepted (re-send refusals allowed, counted) | The schedule |
| G5 | Every accept carries `[wire DISARMED …]`; `mpc_active` never 1 | ARMING_CONTRACT A5 |

## 4. What the numbers mean — decided before the sitting

- **All PASS on rows 15 and 16** → R2's gate is MET. R3 starts.
- **G1 fails, G2 holds** (some solve between 50 and 75 ms, no `SPLICE_TOO_LATE`)
  → the plan's 50 ms target is missed but the robot path is inside its measured
  splice budgets. Not a stop: record the maxima and the kinds that carry them,
  and the owner decides between keeping 50 ms (then R3 opens with an
  optimisation unit) and restating the gate as the splice budgets.
- **G2 fails** → at this lead, on this box, under this load, a real session
  would refuse installs. **Stop.** The options are a longer lead (costs tracker
  freshness on every re-aim) or less load on the box; not a sitting-time decision.
- **G3 fails** → a solve starved the 40 Hz emitter; on an armed wire that is the
  250 ms `SETPOINT_STALE` class. **Stop**, and check step 7 first.
- **G4 fails with G1–G3 passing** → a planning refusal, not a timing one. Send
  the CSV; the `code` and `message` columns name it.
- **G5 fails** → the driver has already called `trajectory/hold`. E-STOP if
  anything moved, then stop the sitting.

## 5. If something refuses

| You see | Meaning | Do |
|---|---|---|
| P1 `mode is 'STANDBY'` | Activated, but not in TRAJECTORY | Step 11 |
| P2 `is_homed false` | Not homed this power cycle | GUI **Home**, then step 10 |
| P3 `mpc_active '1'` | The wire is armed | Deactivate, relaunch with `auto_arm:=false` (step 6) |
| P4 limits not 300 / 5000 / 200000 | Step 12 skipped or clamped | Step 12 |
| P5 correction loaded | `level` ran this launch | Deactivate, relaunch, skip `level` |
| P6 cycle active | A plan from an earlier run is still installed | Wait for it to end, or `ros2 service call /trajectory/hold std_srvs/srv/Trigger` |
| P7 service missing | The launch sourced the main install | Step 1's note, relaunch |
| `SERVICE_TIMEOUT` rows | A solve over 2 s — the box is badly loaded | Send `loadavg` and the launch log |

## 6. Close-out

| # | Step | Expect |
|---|---|---|
| 18 | `ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'deactivate'"` | Robot stows. The commanded state wandered through the schedule while the legs held still; deactivating discards it, and the next activation reseeds from the encoders. **Do not arm this launch for anything else without deactivating first.** |
| 19 | Stop the launch and the load capture. | |
| 20 | Send the paths — not pasted contents: the three `temp/logs/skills_plan_bench_*.csv` with their `_meta.json`, `temp/logs/loadavg_r2gate_*.txt`, `temp/logs/launch_r2gate_*.log`, and the bag folder name. | |

## 7. Results (fill in)

| Item | Result |
|---|---|
| Date, commit, bag | |
| Rehearsal (row 4) max ms | |
| `blas threads` (row 7) | |
| Row 15 (A) — G1 max / G2 handoff max / unpinned max / G3 max / verdict | |
| Row 16 (B) — same, plus re-sends accepted / refused | |
| Row 17 (C, not gating) — same | |
| `load1` range during rows 15–17 | |
