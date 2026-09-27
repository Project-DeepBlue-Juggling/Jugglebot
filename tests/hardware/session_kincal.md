# Runsheet — kinematic-calibration sweep (plan `kinematic-calibration.md` § 6 step 3)

About 35 min at the robot: 19 min of motion plus one re-home. No ball, no throwing.
The tool is request-only; you own homing, activating and E-STOP.

## 0. Before powering up

| # | Step | Expect |
|---|---|---|
| 1 | **Decide now whether the base will be re-shimmed or QTM re-aligned before the acceptance sitting.** If yes, do it **before** this capture. | The fit is expressed in today's QTM Base frame. A re-align between capture and acceptance puts a new frame error on top of the fitted geometry. |
| 2 | QTM running with the `Platform` and `Base` bodies enabled. No ball in the hand. | — |

## 1. Bring-up (system python3, ROS sourced)

| # | Step | Expect |
|---|---|---|
| 3 | `ros2 launch jugglebot jugglebot_launch.py auto_arm:=true record:=true 2>&1 \| tee temp/logs/launch_kincal_$(date +%Y%m%d_%H%M).log`, with the GUI open | `mocap_node` logs "Mocap base aligned" |
| 4 | GUI: **Home**, then **Activate** | Hand parks at 0 rev; wire reads ARMED |
| 5 | `ros2 topic pub --once /orchestrator_command std_msgs/String "data: trajectory"` | Mode TRAJECTORY. Do **not** call `set_limits`: the launch defaults are what the dry-run checked. |

## 2. Rehearsal (no motion)

| # | Step | Expect |
|---|---|---|
| 6 | `python3 tests/hardware/kincal_capture.py --rehearse 2>&1 \| tee temp/logs/kincal_rehearse_$(date +%H%M).log` | `Rehearsal: 0 sequence refusal(s), 0 preflight problem(s).` Anything else: fix what it names, then re-run. |

## 3. Capture

| # | Step | Expect |
|---|---|---|
| 7 | `python3 tests/hardware/kincal_capture.py 2>&1 \| tee temp/logs/kincal_capture_console_$(date +%H%M).log` | It goes to the centre pose (0, 0, 170, level), then pose 1. |
| 8 | **Watch the first recorded line.** | A `frame sanity check failed` ABORT means a frame, quaternion or sign error. **Stop and send me the log**; do not work around it. |
| 9 | Watch the running lines: `[k/185] id role … err N mm spread S mm` | `err` about 5–15 mm (the known geometry error; it varies with height). `spread` under 0.5. An occasional `SKIPPED` is fine, including a `go_to_pose refused … WORKSPACE` one. A run of stillness skips means the Platform body is occluded at that height: note which poses. An ABORT saying `the platform did not follow the command` or naming `MAX_DEVIATION` means the legs stalled. **Stop, note the time, and send me the log.** Do not re-run until we have looked at the bag. |
| 10 | **At the RE-HOME banner:** GUI **Deactivate** → **Home** (wait for IDLE and homed) → **Activate** → repeat row 5's `trajectory` command → press Enter | It re-runs the preflight and continues with the post-home poses. If it prints "Not ready yet", fix what it names and press Enter again. |
| 11 | Abort anytime: **Ctrl-C** | Returns to centre and keeps every row taken. If it prints `RETURN TO CENTRE REFUSED/FAILED`, bring the platform home yourself. E-STOP is always yours. |
| 12 | End | `Wrote N rows: temp/logs/kincal_sweep_<ts>.csv`, plus `_meta.json` (skipped poses, any abort reason) |

## 4. Fit (any python, robot can be off)

| # | Step | Expect |
|---|---|---|
| 13 | `python tools/kincal_fit.py temp/logs/kincal_sweep_<ts>.csv` | `temp/reports/kincal/kincal_sweep_<ts>/report.md`. Send me the path. Nothing in `config/` changes. |

What we read together:
- the § 3 path-dependence verdict: STATIC, DIRECTIONAL, or RANDOM (RANDOM reopens the pose-servo question)
- the § 4 re-home verdict
- the § 8 hold-out criteria: ≤ 1 mm RMS, ≤ 2 mm max, ≤ 0.1°

Applying the geometry is a separate, deliberate commit (§ 6 step 5: config, codegen, the two-run box re-sweep, then `colcon build`), then a tilt-map recapture and the acceptance sitting.
