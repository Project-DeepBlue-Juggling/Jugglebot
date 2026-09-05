# Unified cycle ladder — Phase 5 (UH-3 … UH-7)

## Before you start

- First time the robot moves under the unified 7-DoF planner: platform and hand
  are now one plan on one clock.
- UH-1/UH-2 are Phase 3's T-H1/T-H2, flown 2026-09-04. That is why this ladder
  starts at UH-3.
- Safety, in order: **your E-stop, in your hand**, then the can-bridge hand
  guard, **ARMED** for this sitting (owner, 2026-09-05).
- A guard trip is **data, not a disaster** — it E-stops safely, one service call
  releases it (it's in the table at the end), and I want the log.
- Ball in the cup only from UH-3 on. UH-5 throws it away on purpose — stand clear.
- If your physical intuition disagrees with any framing here, say so.
- Every robot-actuating command is yours. I read only.

---

## Preconditions

Do these in order. Do not skip one because it "was fine last time".

**1. Pull, then run the full gate.** Expect a pass count and no failures.
```bash
cd ~/Desktop/Jugglebot && git pull && ./run_tests.sh --full
```

**2. Rebuild the ROS packages.** The interfaces changed; the launch will not
start without this. Expect two packages, no errors.
```bash
cd ~/Desktop/Jugglebot/ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && source install/setup.bash
```

**2b. Check the INSTALLED unified-cycle switch.** Must print `True`. **`False`
is a STOP** — go back to step 2 and rebuild, then run this again.
```bash
cd ~/Desktop/Jugglebot/ros_ws && source install/setup.bash && \
  python3 -c "import jugglebot.hardware_config as h; print(h.JB_OP_UNIFIED_CYCLE_ENABLED)"
```
This reads the copy the node actually runs, not the source tree, and the two can
disagree — on 2026-09-05 the source read `True` and the installed tree read
`False`. Nothing refuses on it: with the installed copy `False`, a goal asking
for `unified_cycle: true` silently runs the legacy path instead.

**3. Reboot the can-bridge Teensy.** Press its reset button.

**4. With the launch still DOWN, park the hand by hand.** Push the slider down
to its bottom rest — between **−0.20 and +0.10 rev** — and let it stop moving,
or step 5 is refused. If the slider will not move it is still energised — idle
the hand axis in the ODrive GUI first.

**5. Switch the hand latch to STREAMED.** Launch still down. Expect
`hand_source → STREAMED: OK`; if refused, the hand is not down far enough.
```bash
cd ~/Desktop/Jugglebot
python3 tests/hardware/hand_stream_bench.py --source-only streamed
```

**6. Start the console capture and leave it running all sitting.**
```bash
mkdir -p ~/Desktop/Jugglebot/temp/logs
script -f ~/Desktop/Jugglebot/temp/logs/cycle_ladder_$(date +%Y%m%d_%H%M%S).log \
  -c "pio device monitor -d ~/Desktop/Jugglebot/ros_ws/src/jugglebot/Teensy_code_canbridge -e teensy41 -f time"
```
`-f time` stamps every console line with a time, so a rung can be lined up
against its CSV afterwards without guesswork.

Expect the `[hand7]` line once a second. **Press Enter between every step
below** — the counters are cumulative, so a step's value is the difference
across it.

**7. Bring the launch up, with recording on.** Note the bag folder it prints
(`~/Desktop/rosbags/<stamp>`).
```bash
ros2 launch jugglebot jugglebot_launch.py record:=true
```

**8. Home, level, then ACTIVATE.** GUI at http://localhost:8081 — `Home`, then
`level`, then the **Activate** button.

**9. Check the hand woke up.** This prints one line — **the seventh line is the
hand; it must read `8`** (closed loop). If it reads `1` the hand is idle —
deactivate, recheck step 5, activate again.
```bash
timeout 4 ros2 topic echo /robot_state | grep -m7 'current_state:' | tail -1
```

**10. Arm the hand guard.** Type `hand7 arm` into the console window from step
6. It **echoes immediately** — a `[hand7]` line out of the usual once-a-second
cadence — and it must read `guard=ARMED`. It stays armed for the rest of the
sitting; only a can-bridge reboot puts it back to `observe`.

**11. Raise the session limits.** Expect 250 / 3000 / 150000 echoed back as the
`applied_*` values.
```bash
ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits \
  "{leg_vel_limit_mmps: 250.0, leg_acc_limit_mmps2: 3000.0, leg_jerk_limit_mmps3: 150000.0}"
```

**Rehearsal (any time, no robot):** `python3
tests/hardware/unified_cycle_bench.py --rung carry --dry-run` prints the request
it would send and makes no ROS calls. The cup it prints is a **placeholder at
`[0, 0, 689.6]` mm**, not a live reading — on the robot the cup site comes from
the live pose and hand.

**Which python:** every driver command in this runbook runs under `python3` with
ROS sourced, which is the drivers' own rule. Do **not** activate the project
venv for these.

---

## UH-3 — carry a seated ball

**Put one ball in the cup.** The cup moves 60 mm sideways over 1.4 s, then back.
```bash
python3 tests/hardware/unified_cycle_bench.py --rung carry --dx 60
```
**Watch the ball, not the screen.** That is the whole test.

- **Pass:** the ball stays seated and does not visibly shift, roll or hop; the
  driver prints every check `PASS`; the move takes no longer than the legacy
  duration it prints.
- **Stop if:** the ball unseats, the guard trips, or V3/V4/V5/V6 `FAIL`. A V2 or
  V7 `FAIL` is a number to send me, not a reason to stop.

Repeat once along the other horizontal axis if the first is clean. The `--dx 0`
is not optional — `--dx` defaults to 60, so `--dy 60` on its own would command
an 85 mm diagonal instead of a single-axis move.
```bash
python3 tests/hardware/unified_cycle_bench.py --rung carry --dx 0 --dy 60
```

## UH-4 — planned catch

**Not a separate step.** The only planned-catch path that exists is the full
session, so this is folded into UH-6's first cycles. Score it there.

## UH-5 — planned throw, no catch

**Empty cup for the first run.** The hand strokes up, releases at 860 mm, then
the cup settles back down.
```bash
python3 tests/hardware/unified_cycle_bench.py --rung throw --apex 0.5
```
- **Pass:** smooth stroke, nothing trips, all checks `PASS`. The encoder must
  stay clear of the 10.8 rev hard stop — the driver prints the margin.
- Then **put a ball in** and repeat: ~0.5 m straight up, lands on the floor.
- I read throw accuracy from the bag afterwards; nothing for you to do here.
- **Bracket the `[hand7]` line** either side of this rung. A non-zero `lead` or
  `dev_over` **delta** across the rung is a hard abort — the absolutes never
  reset, so only the difference means anything.
- **Stop if:** the ball unseats, the guard trips, the driver prints a `!!` line
  saying the plan ends at the throw, or V3/V4/V5/V6 `FAIL`. A V2 or V7 `FAIL` is
  a number to send me, not a reason to stop.

## UH-6 — full planned cycles

The GUI cannot send this goal, so use the command line. Ball in the cup.
```bash
ros2 action send_goal --feedback jugglebot/toss_continuous \
  jugglebot_interfaces/action/TossContinuous \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0},
    throw_height_m: 0.5,
    num_throws: 3,
    dwell_time_s: 6.0,
    throw_delay_s: 5.0,
    catch_vel_scale: 0.0,
    stop_on_miss: true,
    on_empty_cup: 'STOP',
    max_reloads: 0,
    unified_cycle: true}"
```
`unified_cycle: true` is the whole point. Without it you get a legacy session.
`max_reloads: 0` means "use the config default (3)", and `catch_vel_scale: 0.0`
likewise means "use the default" — neither is an off switch.

- **Expect:** feedback counting `cycle_index` 1, 2, 3, and one line at the end
  saying the hand latch **remains STREAMED**. In a second terminal:
  ```bash
  ros2 topic echo /trajectory/status | grep -E 'cycle_active|cycle_plan_wall_ms'
  ```
  `cycle_active: true` while a cycle runs. **`cycle_plan_wall_ms` around 400 ms
  applies to cycle 1 only** — cycles 2 and 3 are extensions of the first plan and
  read about 200 ms, which is a healthy number, not a fault.
- **Pass:** 3 throws, 3 catches, no guard trips, catching as well as a legacy
  session at the same height.
- **Bracket the `[hand7]` line** either side of this rung. A non-zero `lead` or
  `dev_over` **delta** across the rung is a hard abort — the absolutes never
  reset, so only the difference means anything.
- **A miss stops the session** here — what you want first time. (With
  `stop_on_miss: false` a survived miss holds the pose, waits ~2 s, carries on;
  it does **not** go home.)
- **To stop:** Ctrl-C the send-goal terminal. It may finish the cycle already in
  the air. **The cancel is not the E-stop** — to stop NOW, use the E-stop.

## UH-7 — two-pose ring at a constant beat

**Not in this sitting.** It needs the steady-chain plumbing, which does not exist
yet.

---

## After the sitting

The hand latch stays STREAMED. Put it back, in this order:

1. **Deactivate** in the GUI, then bring the launch down. The switch is refused
   while the output is armed.
2. **Idle the hand motor** in the ODrive GUI.
3. **Push the hand by hand** into the retract band — between −0.20 and +0.10
   rev, and let it stop moving. A unified session leaves it at about +0.32 rev,
   which is **outside** the band, and the switch will be refused there (it was,
   at sitting three's close-out, at +1.06 rev).
4. **Switch it back.**
   ```bash
   cd ~/Desktop/Jugglebot
   python3 tests/hardware/hand_stream_bench.py --source-only legacy
   ```
   You should see `hand_source → LEGACY: OK`.

If that will not work, **reboot the can-bridge** — it boots LEGACY.

**Send me:** the console log from step 6, the bag folder from step 7, and the
CSV + meta files the driver printed at the end of each run
(`temp/logs/unified_cycle_bench_*`).

---

## If something refuses

`P1`–`P5` are the driver's five preconditions. It checks all five before every
run and refuses without commanding anything, so a `P… REFUSED` line means the
robot did not move.

| What you see | What it means | What to do |
|---|---|---|
| `P4 REFUSED … hand_source=LEGACY_STROKE` | The hand latch is still on the old (legacy) path, so the firmware throws away every hand command | Launch down, redo preconditions 4 and 5, launch and activate again |
| `P3 REFUSED … current_state=1` | The hand motor is idle — it will look like a perfect hold and prove nothing | Deactivate, confirm the latch, activate again |
| `P5 REFUSED … want 250 / 3000 / 150000` | Session limits are still the shipped defaults | Run precondition 11 |
| `REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK: …)` | The plan needs more jerk than the limits allow | Almost always precondition 11 was skipped or a relaunch reset it. Re-run it |
| `REJECTED_HAND_NOT_PARKED` | The hand is outside the park band before a cycle | Let the previous cycle settle; if it will not, stop and tell me |
| `STALE_STATE` | Telemetry went quiet, or the machine moved during the solve | Wait two seconds and re-run. If it repeats, check the link: `ros2 topic echo /link_status --once` |
| `CHAIN_SKEW` | The session and the plan disagree about when the throw is | Stop the session. This is a finding — send the log |
| `ERR_HAND_SOURCE` on a legacy goal | You sent a legacy goal while the latch is STREAMED | Expected. Either add `unified_cycle: true`, or do the "After the sitting" recovery first |
| Guard trip / E-STOP latched | The hand deviated past 2.5 rev | `ros2 service call /clear_errors std_srvs/srv/Trigger` then re-activate. **Log it — this is the data we want** |
