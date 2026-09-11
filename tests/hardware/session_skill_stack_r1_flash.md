# R1 sitting — flash the pair, run the ladder, catch one streamed self-toss

Skill-stack R1 (`plans/active/two-ball-skill-stack.md` § 4 R1, entry
`logbook/2026-09-11-skill-stack-r1-one-hand-master.md`). Everything here is run
by the operator. Every number carries its source in the linked entry.

**What changed for you, in five lines.**
1. There is no hand latch. `/set_hand_source`, `--source-only` and the
   `[hand7] src=` field are gone; the hand lane runs whenever a Setpoint frame
   carries `HAS_HAND`.
2. `set_hand_traj_cmd` and `smooth_move_hand` are gone. A plain `Toss` goal, or
   `TossContinuous` without `unified_cycle: true`, is refused at accept with
   `REJECTED_STROKE_ENGINE_RETIRED`.
3. `/activate` energises the hand and parks it at 0 rev; `/deactivate` idles it.
   With the launch up you never energise the hand by hand. (Bench driver rows with
   the launch down still use `--close-loop`, the driver's own energise.)
4. The hand deviation guard boots ARMED. `hand7 observe` on the bridge console
   lasts one armed session; the disarm edge re-arms it. Nothing to restore.
5. Platform FW 7 never transmits to CAN node 6. The bridge is the only writer.

---

## 1. Flash (launch DOWN, nothing else on the UDP link)

A host on protocol 7 against a board on FW ≤ 20 is DARK by design
(`link=NO_HEARTBEAT`, `decode_errors == rx_frames`). That is not a cable fault.
Rollback is the pair: this commit's parent for the host, FW 20, Platform FW 6.

| # | Step | Receipt |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git status -sb` clean, at or after `1e2c0c9`. | |
| 2 | Bridge: `cd ros_ws/src/jugglebot/Teensy_code_canbridge && pio run -e teensy41 -t upload` (builds, then flashes; the only USB Teensy is the bridge). | Boot banner `jugglebot-canbridge v21` on the console; the 1 Hz `[hand7]` line reads `guard=ARMED` and has no `src=` field. |
| 3 | Platform, over CAN through the freshly flashed bridge: `cd ../Teensy_code_platform && pio run -e teensy40 -t upload` (image already built: FW 7; ~55 s; a first-attempt failure was host windowing last time, just re-run). | The tool prints `STATE_READ 6 → 7`. |
| 4 | Host: `cd ~/Desktop/Jugglebot-skills/ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && source install/setup.bash` (the srv deletion needs both packages). | |
| 5 | `ros2 launch jugglebot jugglebot_launch.py record:=true`, then `timeout 5 ros2 topic echo /link_status | head -80` (Foxy has no `--once`). | `bridge_fw_version: 21 (proto 7)`, `BRIDGE_FW_CHECK: OK`, Platform `7`, NO `hand_source` row. Note the bag id. |

Start a console capture before row 2 and leave it running to the end:
`pio device monitor -b 115200 | tee ~/Desktop/Jugglebot-skills/temp/logs/r1_console_$(date +%Y%m%d_%H%M).txt`
(close it before each `-t upload`, reopen after). Every counter below is read as a
delta across a row from this capture.

## 2. Before power: the rehearsal that exists, and the one that does not

- `python3 tests/hardware/unified_cycle_bench.py --rung throw --dry-run` and
  `--rung carry --dry-run`: proves the tree imports and the generated numbers
  (the new `hand_mm_per_rev` chain) resolve. Arithmetic only, no ROS.
- `python3 tests/hardware/hand_stream_bench.py --help`: proves the driver's flags
  resolve. **There is no dry-run for the ladder stages or for `plan_cycle`; their
  first execution is live.** The owner accepted this for R1 on 2026-09-11; R2 adds
  the validate-only path.

Preconditions: hand mechanically clear, no ball, cords clear of the slider,
`git status -sb` clean, the `Jugglebot-geometry` worktree untouched.

## 3. The ladder (launch DOWN for every driver row; console captured)

Driver: `python tests/hardware/hand_stream_bench.py …`. Read the `[hand7]` line
before and after each row; every refusal in a row is reported, not stopped at.

| # | Row | Command | Pass |
|---|---|---|---|
| 12 | No latch exists | `--stage hold --duration 10` (no `--close-loop`), then `--stage hold --duration 10 --close-loop` | Half one: `unseen` counts, the IDLE hand does not move, `[hand7]` has no `src=` field. Half two: the same stream holds the energised hand. |
| 12b | ACTIVATE parks the hand (launch UP for this row only) | `ros2 service call /activate std_srvs/srv/Trigger`, read `/robot_state`, then `ros2 service call /deactivate std_srvs/srv/Trigger` | Hand `axis_state 8`, `controller_mode 3` / `input_mode 1`, encoder `0.00 ± 0.02` rev, `dev_max ≈ 0` (the FW 17 row-13 clip residual is gone by construction). After deactivate: axis 6 IDLE, hand on the stop. |
| 12c | Hard-stop span (replaces the archived geometry plan's G3) | Hand de-energised: slide the carriage by hand to each stop and read the encoder (`/robot_state` or the console `hand7` line), as on 2026-09-06 and 2026-09-11 | Span **10.81 ± 0.01 rev**, ends near **−0.11 / 10.69**. **Outside that band, STOP the sitting**: `hand_mm_per_rev 32.567` is wrong. |
| 13 | Streamed hold, 600 s | `--stage hold --duration 600 --close-loop` | No motion, no trip; deltas `lead=0 dev_over=0 unseen=0 stale=0`; `dev_max` logged (the post-flash zero). |
| 14 | Slow triangle | `--stage triangle --tri-span 2.0 --tri-speed 0.5 --duration 60 --close-loop` | Smooth 0.5 rev/s over 2 rev, `lead` delta 0, `dev_max` logged. |
| 16 | Host step refusal | `--stage step --step-rev 6.0 --close-loop` | `pump refused=True`, nothing on the wire, hold unbroken, firmware counters unmoved. |
| 17 | Gap re-entry | `--stage gap --gap-pre 3 --gap-s 1.0 --gap-delta 1.0 --close-loop` | Lane decays to rest in the gap, one bounded ~32 mm re-entry, `sent` climbs, the driver's echo-moved proof passes. No trip at 1.0 rev against the 2.5 rev band. |
| 17b | Moving gap (the decay rule) | `--stage moving_gap --duration 10 --close-loop` | Driver criteria G1–G5 pass (coast `+0.0525 rev` at the defaults, same rule as FW 17); deltas `lead=0 dev_over=0`. |
| 18 | ARMED guard, trip observed COLD | `--stage gap --gap-delta 3.0 --gap-pre 3 --gap-s 1.0 --close-loop`, recover with `--clear-errors`, then re-run row 14 | The 3.0 rev re-entry passes the 5 rev pump gate and exceeds the 2.5 rev band on the first tick: E-STOP latches with the hand free, the driver's abort line names leg 6. After `--clear-errors`: `fault_state NONE`, recovery slew ≤ 1 rev/s. Row 14 re-run ARMED: zero trips (2026-09-05 worst residual was 10 % of band). |
| 19 | Sole writer | Read the Platform banner / `/link_status` Platform version | Platform reads **7** (the only code that could write node 6 is deleted). No dedicated second-master counter exists yet; that is an open R2 item. |
| 20 | Close-out sweep | `/link_status` + console | `can3_errors` 0, `leak_* ≡ 0`, `interp_deadline_misses` 0, jitter in envelope, `latency_monitor` OK, `tx_deferred` 0, `bridge_fw_version 21 (proto 7)`. |
| 21 | Close-out state | `/deactivate` | Hand IDLE on the stop. Nothing to restore. |

Row 15 (legacy stroke replay) no longer exists: the stroke it replayed is deleted.

## 4. The R1 gate: one streamed self-toss, caught, no latch step

Launch UP, ball in the cup, session limits and preconditions as UH-6
(`tests/hardware/session_unified7_cycle_ladder.md` § "UH-6", rows 10–11, minus
`hand7 arm`, which is now the boot state). ONE throw:

```bash
ros2 action send_goal --feedback jugglebot/toss_continuous \
  jugglebot_interfaces/action/TossContinuous \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0}, throw_height_m: 0.5,
    num_throws: 1, dwell_time_s: 6.0, throw_delay_s: 5.0, catch_vel_scale: 0.0,
    stop_on_miss: true, on_empty_cup: 'STOP', max_reloads: 0, unified_cycle: true}"
```

Pass = caught, no guard trip, `[hand7]` deltas `lead=0 dev_over=0` across the
throw, `dev_max` logged, bag id recorded. There is no `/set_hand_source` call
anywhere in this sheet, which is the point.

## 5. Results (fill in)

| Item | Result |
|---|---|
| Boot banners | bridge v__ (console line), Platform STATE_READ __ → __ |
| Bags | |
| 12 / 12b / 12c | |
| 13 / 14 / 16 / 17 / 17b | |
| 18 (cold trip) | |
| 19 / 20 / 21 | |
| Self-toss | caught: __ / trip: __ / `dev_max`: __ |
