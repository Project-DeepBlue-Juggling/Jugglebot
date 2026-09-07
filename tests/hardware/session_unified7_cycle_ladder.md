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

**2c. START QTM, and set the project up for a FLYING BALL.** Two separate
things, and both bit the 2026-09-06 sitting.

- **QTM must be up before the launch.** On 2026-09-06 the very first goal (G0)
  was refused `REJECTED_MOCAP_STALE` for no reason other than that QTM had not
  been started yet. Start it, confirm it is streaming, *then* bring the launch up
  at step 7.
- **DISABLE the Catching Cone rigid body in the QTM project, and mask the Ball
  Butler's reflectors.** With the Cone body enabled, QTM binds the flying ball to
  it about **70 ms after release** — the ball stops existing as a ball and the
  tracker is blind for the whole flight, which is exactly the window the catch
  needs. The BB's reflectors are the other half: unmasked, they minted **seven
  phantom balls** in one sitting. Neither is a tracker bug and neither can be
  fixed from the Jetson side.

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

### Before UH-3 can be retried — read this first

**The first attempt (2026-09-06) refused three times and then E-STOPped the
robot.** The three refusals are fixed. The E-STOP is understood but not closed,
so there is one thing to check before you put a ball in the cup.

**What `MPC_STALE` means.** The Teensy expects a setpoint frame every 25 ms. If
none arrives for **250 ms** it E-STOPs and latches — that is the whole rule. On
2026-09-06 the planner solved for **2.1 seconds** on the same CPU as the thing
sending those frames, the stream went quiet for **804 ms**, and the guard fired.
**It fired correctly.** Nothing was wrong with the hand, the legs, the CAN bus or
the link; the Jetson simply stopped talking for long enough that the Teensy was
right to stop trusting it.

**The cause is now known and fixed** (2026-09-06 evening). The planner was using
**all six CPU cores** for arithmetic that needs one, and the six workers *spin*
— they sit burning a core waiting for the next piece of work. On an idle box
that costs nothing. On a box with anything else running, those spinners shove
the thing sending setpoints off the CPU, and the solve itself slows down about
tenfold. Measured, with three of the six cores busy: **1350–2314 ms** and the
stream gapping **225–942 ms** — versus **214–217 ms** with the planner held to
one core, at any load. That is the whole story; the 2.1 s was never a slow
planner, it was six threads fighting over a box that also had to talk to the
robot.

The fix pins the three planner nodes to one thread in the launch file. **There
is one thing to check, and it takes ten seconds.**

**After the `colcon build`, start the launch and look in the launch terminal for
this line:**

```bash
# In the launch terminal (or on a saved log):
grep 'blas threads' <the launch output>
```

**You want `blas threads: 1` for trajectory_node.** The line looks like:

```
[trajectory_node-N] [INFO] [...] [trajectory_node]: blas threads: 1 (threadpoolctl:openblas)
```

**If it says anything other than 1**, a loud WARNING sits right underneath it
saying so — do not fly the rung. It means the launch file's cap did not reach
the node, and the E-STOP class is back. Send me the two lines.

**If the guard latches during a plan anyway**, note the solve time the driver
prints — a solve over about 1 second is still the signature — and it is also on
`/trajectory/status.cycle_plan_wall_ms`. The accept line now also prints where
the time went and how busy the box was, e.g. `[qp=11.2 tilt=4.2 dec=3.7 val=163.9
cont=0.7 ms] load1=0.42`; send that whole line. Recovery is `/recover` (or
`/clear_errors`) as usual.

**Please also capture the box's own load this sitting** — the rosbag has no
channel for it at all, so without this a repeat is unattributable after the
fact. Leave this running in its own terminal for the sitting:

```bash
( while true; do echo "$(date +%H:%M:%S) $(cat /proc/loadavg)"; sleep 1; done ) \
  | tee temp/logs/loadavg_$(date +%Y%m%d).txt &
vmstat 1 | tee temp/logs/vmstat_$(date +%Y%m%d).txt
```

**The probe stays as the diagnostic, not as a gate — you do not need to run it
to fly.** If a latch does recur, this is the pair that shows the problem (and
the pair the morning of 2026-09-06 got wrong by running it on an idle box, where
the two arms are identical and prove nothing):

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate

# The causal pair: same load, different thread count. Expect the first to be
# several times slower than the second if the cap is not working.
python tools/probes/emitter_gap_under_solve.py --reps 2 --load 3
python tools/probes/emitter_gap_under_solve.py --reps 2 --load 3 --threads 1
```

Measured on the shipped code (2026-09-06, `--reps 2 --load 3`): **capped 209 and
217 ms; uncapped 2920 and 641 ms** on the identical load.

⚠ **`--load 3` spawns CPU burners.** A clean exit tidies them up, but a crashed
or `Ctrl-C`-ed probe leaves them spinning and they will slow down everything you
do next. Kill them and check the box is quiet again before you fly anything:

```bash
pkill -f 'while True: a@a'
vmstat 1 3          # the `id` column should be back near 99
```

---

**Put one ball in the cup.** The cup moves 60 mm sideways over 1.4 s, then back.
```bash
python3 tests/hardware/unified_cycle_bench.py --rung carry --dx 60
```
**Watch the ball, not the screen.** That is the whole test.

**Expect the driver to say the carry ends ~11 mm higher than it starts.** That
line is normal, not a fault. The planner's cup box has a floor at 689.6 mm, and
a hand parked at retract puts the cup opening at ~678.7 mm — below it — so a
carry taken at the live height cannot be planned at all. The driver raises the
site to the floor and says so. The carry is still purely sideways; only its
height is set once, at the start. `--z-mm` overrides it if you want a specific
height.

**Expect the `carry check:` line to read within about 1 mm of the settle
height.** A carry is flat, so the hand the planner intends to command should be
the settle height and nothing more; the driver prints the two side by side and
**refuses to let the plan run past 16 mm of lift**, holding the machine before
it stops. If you ever see that refusal, send me the line — the planner is
solving a different problem from the one the rung is asking about.

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
- **Pass:** smooth stroke, nothing trips, all checks `PASS`.
- **What you should see on the encoder.** A 0.5 m throw peaks at **9.76–9.87 rev**
  — about **0.8 rev clear of the metal**. The metal is the stop *you measured* on
  2026-09-06: stroke 352 mm, bottom **−0.107 rev**, top **10.701 rev**. The
  driver now scores V3 against that 10.701, not against the firmware's clip.
- ⚠ **Watch this one, because no firmware guard can.** The firmware clip
  `HAND_MOTOR_MAX_POSITION` is a zero-margin alias of
  `Geometry::HAND_MOTOR_HARD_STOP_REVS` = **10.8**, i.e. **0.099 rev (3.1 mm)
  PAST the metal you measured**. Nothing in the firmware can see a stall in that
  gap: the deviation guard compares the encoder to the *command* (and at the clip
  they agree once the slider is jammed), and the lead clamp *anchors* the
  setpoint to the encoder rather than refusing it. So between 10.701 and 10.8 the
  slider can be pressed into the stop with every guard reading clean. Pulling the
  clip back below the metal is FW 18 work. Until then: if the encoder gets within
  ~0.2 rev of 10.701, stop the rung — the driver fails V3 there and prints the
  margin in mm.
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

> ⚠ **Aimed throws are NOT supported on the unified path yet — keep
> `catch_position` at the pre-throw xy.** The unified launch throws *vertically*
> by construction (the plan pins its ballistic target onto the release site) and
> the plan owns the whole traverse, so a displaced `catch_position` is not an aim:
> it is a site the machine would silently throw from somewhere else. Since
> 2026-09-07 a unified goal whose `catch_position` is more than **5 mm** from the
> platform's live commanded xy is refused at acceptance,
> `REJECTED_UNIFIED_AIM_UNSUPPORTED`, **before** anything is armed or tilted. On
> 2026-09-06 it was not: goal 4 asked for 2.384°, the *legacy* preamble physically
> tilted the platform (mocap held +2.27°), the plan's own first knots tilted it
> straight back, and the ball would have gone vertically up with nothing saying
> so. If you want an aimed throw tonight, run the goal with `unified_cycle: false`.

- **NEW 2026-09-07 — the session lifts the hand to the planner floor before the
  first throw. You will see one ~1 s hand move UP first, before any cycle
  starts, and a console line naming it.** The hand rests below its homed zero by
  design and the planner's usable cup floor sits at **689.6 mm** (0.3162 rev), so
  an ordinary parked machine starts a session *below* the floor. A LAUNCH seeded
  there must slam the cup into the box inside one 25 ms knot — that is the whole
  of what refused the 2026-09-06 23:55 sitting four times
  (`HAND_LIMIT_ACC` 4063–4077 rev/s² against the 3500 cap), and worse, a seed
  only ~11 mm low is **accepted** and flies a 0.35 rev step with the cup arcing
  past its release site. The lift is a **pure z move** (the settle xy is wherever
  the platform already is), takes 1.0 s, and is a **no-op on cycles 2 and 3** —
  each cycle's chained LANDING settles the cup exactly on the floor, so a healthy
  chain never lifts again. If you see a lift before every cycle, say so: something
  is moving the hand between cycles.
- **Keep `dwell_time_s`/`throw_delay_s` at the defaults FOR THIS RUNG.** UH-6 is
  the one-throw-per-cycle shape: each cycle pays the 1.0 s floor lift, the 1.80 s
  launch lead and a ~0.5 s joined solve before the release instant exists, so a
  shorter beat lets the scheduled release pass before a plan is installed and the
  cycle aborts `ABORTED_NO_RELEASE`. Confirmed 2026-09-07. **A short beat is what
  UH-7a is for** — it chains the cycles instead, and it refuses a too-short beat
  at acceptance instead of aborting mid-session. Do not shorten the beat here;
  go to the UH-7a rung.
- **Expect:** feedback counting `cycle_index` 1, 2, 3, and one line at the end
  saying the hand latch **remains STREAMED**. In a second terminal:
  ```bash
  ros2 topic echo /trajectory/status | grep -E 'cycle_active|cycle_plan_wall_ms'
  ```
  `cycle_active: true` while a cycle runs — and it **stays true after the window
  ends**, because it is a type test on the plan that is installed, and a cycle
  that has run out is still the plan holding the pose. Watch
  `plan_time_remaining_s` reach 0, not `cycle_active` go false. (A `cycle_active:
  false` mid-session means something *superseded* the cycle.)

  **`cycle_plan_wall_ms` is 500–600 ms on EVERY cycle, not 400 then 200.**
  Corrected 2026-09-06: the older note here said cycles 2 and 3 were "extensions
  of the first plan" reading about 200 ms. At this cadence they are not. The
  dwell is long enough that each cycle comes to rest, so every cycle is a fresh
  **joined** LAUNCH+LANDING solve from rest — two solves plus the join's own
  re-validate. 500–600 ms is the healthy number; the bench driver now fails a
  chained install only past the coordinator's own 1200 ms release budget, and
  says "ADVISORY" between 700 ms and that.

  On the accept line's stage split (`[qp=… tilt=… dec=… val=… cont=… ms]`):
  **`cont` is not overhead.** On a joined install it is the join's *third* full
  `validate_cycle` pass, over the concatenated plan — measured 247 ms, 99.7 % of
  the join — so expect `cont ≈ val` there. The driver now prints a note saying
  so under the accept line.
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

## UH-7a — one pose, constant beat

**This is the first rung where the machine never stops between throws.** Up to
now every cycle came to rest: the plan settled the cup at the floor, the session
planned a fresh launch from a stopped machine, and the beat was whatever that
took (~2.5–2.7 s). From here the plan carries straight through — release, catch,
and the *next* release are one trajectory on one clock — and the next window is
planned *inside* the one that is playing. The beat is now the number you ask for.

**Software landed 2026-09-07. NOT FLOWN. You are the first evidence.**

### Before you start

1. **Full gate green.** Expect a pass count and no failures.
   ```bash
   cd ~/Desktop/Jugglebot && git pull && ./run_tests.sh --full
   ```
2. **Rebuild.** The interfaces did **not** change this time, so it is one
   package. Expect one package, no errors.
   ```bash
   cd ~/Desktop/Jugglebot/ros_ws && colcon build --packages-select jugglebot && source install/setup.bash
   ```
   If a console line ever says *"STALE INSTALL … rebuild the package"*, this is
   the step that was skipped.
3. **Everything in § Preconditions, in order** — including **2b** (the installed
   switch prints `True`), **2c** (QTM up, the `Catching Cone` rigid body
   DISABLED, the Ball Butler's reflectors masked), **10** (`hand7 arm` →
   `guard=ARMED`) and **11** (limits 250 / 3000 / 150000).
4. **Confirm `blas threads: 1`** for `trajectory_node` in the launch terminal
   (§ "Before UH-3 can be retried"). Anything else and do not fly.

### The knobs, and what they mean on a chain

Same goal as UH-6, three knobs read differently.

- **`unified_cycle: true`** — the whole point, as before.
- **`catch_position` must be where the platform already is.** One pose, this
  rung: for a centred machine that is `{x: 0.0, y: 0.0, z: 170.0}`. More than
  **5 mm** away is refused at acceptance (`REJECTED_UNIFIED_AIM_UNSUPPORTED`).
  The two-pose ring is UH-7b and is not in this sitting.
- **`num_throws`: 3 first, then 5. Never more on the first sitting.** A ring
  never lets the hand rest — at a 2 s beat the slider strokes continuously, where
  the one-throw-per-cycle shape parked the cup at the floor for most of every
  dwell. **Nothing in this change models hand-motor thermal load.** Watch hand
  `iq_rms`, and let the hand cool between rungs.
- **`dwell_time_s` IS the beat knob.** beat = flight + dwell, and the plan's
  STEADY window *is* that number. At `throw_height_m: 0.5` the flight is about
  **0.64 s**, so `dwell 1.5` is a beat of ~2.14 s.
- **`throw_delay_s` no longer sets when the ball leaves on cycles 2..N** — the
  plan owns that, and it is only how far before the planned release the next
  cycle's FSM spins up. **On cycle 1 it still does**, which is why it has a floor
  of its own below.

**Three accept-time floors. They are different floors, and each names a
different knob.** All are checked before anything is armed, lifted or commanded,
and the refusal message quotes the exact numbers, so treat the values here as
the arithmetic rather than the authority.

- **The chain dwell floor: `dwell_time_s ≥ 0.800 s`**, refused
  `REJECTED_BEAT_TOO_SHORT`. The chain has two requirements and the larger holds;
  the one that binds is **the next cycle's lead** — cycle k+1 cannot start before
  `catch + max(verdict 0.560, extend 0.600)` and then needs `0.160 s` of preamble
  plus one `0.040 s` tick to announce and dispatch. (The other requirement, that
  the carry `dwell − 0.025` hold one 0.600 s extend, is only 0.625 s and does not
  bind.) It says the chain has time to *ask* for the next window; it does **not**
  say the beat is flyable — that is answered per cycle by the planner.
- **The cycle-1 delay floor: `throw_delay_s ≥ 0.866 s`**, also refused
  `REJECTED_BEAT_TOO_SHORT` (the message names `throw_delay_s`). Cycle 1 is the
  only cycle whose release is *derived* (`now + throw_delay_s`) while the LAUNCH
  that throws lands its release at `install + 0.600`, so the ball leaves
  `preamble 0.160 + solve 0.606 + window 0.600 − throw_delay` **late** against
  the `0.500 s` release grace. Below this floor cycle 1 aborts
  `ABORTED_NO_RELEASE` with the ball unthrown. The 0.606 s is the **measured
  worst joined install on hardware** (500–606 ms at load1 3.7–7.4); a loaded box
  has reached **1.06 s**, so **raise `throw_delay_s` further if the box is
  busy** — being early costs nothing.
- **The legacy dwell floor (unchanged, still live): `dwell ≥ throw_delay_s +
  handoff_margin_s`.** The handoff margin depends on the flight: **0.177 s at
  this rung's flight** (`throw_height_m: 0.5` ⇒ flight 0.639 s) and 0.141 s at a
  0.80 s flight. So at this rung's flight it binds at **`dwell ≥ 1.177 s` when
  `throw_delay 1.0`**, and at **`dwell ≥ 1.047 s` when `throw_delay 0.87`**.
  Below it the goal is refused `REJECTED_DWELL`. **To lower the dwell you must
  lower `throw_delay_s` in step — but not below 0.866.**

⚠ **The cycle-1 refusal message does not quote the dwell it needs — read that
from `REJECTED_DWELL`.** The handoff margin in the session's dwell floor is
flight-dependent, so the message only says the floor applies on top. At
`throw_height_m: 0.5` the real margin is **0.177 s**, so the true minimum dwell
at `throw_delay_s` 0.866 is **1.043 s**, and a dwell of 0.99 will be refused by
`REJECTED_DWELL` one layer down, which names the exact number for the goal.

**So the shortest beat this rung can ask for is about 1.69 s**
(`throw_delay 0.87` + `dwell 1.05` + flight 0.639), against the ~2.5–2.7 s the
one-throw-per-cycle shape needed. A shorter beat than that needs a longer
flight, not a smaller knob.

### The rungs

Ball in the cup. Run them in this order, and **let the hand cool between each
one** — a minute with the session stopped is enough; if `iq_rms` has not come
back down, wait longer. Every dwell below is admissible at accept; that is
deliberate, so a refusal means something happened rather than that the goal was
mis-typed.

**(a) 3 throws at a ~2.14 s beat.** The safe first one.
```bash
ros2 action send_goal --feedback jugglebot/toss_continuous \
  jugglebot_interfaces/action/TossContinuous \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0},
    throw_height_m: 0.5,
    num_throws: 3,
    dwell_time_s: 1.5,
    throw_delay_s: 1.0,
    catch_vel_scale: 0.0,
    stop_on_miss: true,
    on_empty_cup: 'STOP',
    max_reloads: 0,
    unified_cycle: true}"
```

**(b) 3 throws at `dwell_time_s: 1.2`, `throw_delay_s: 1.0`** (beat ~1.84 s).
Only if (a) was clean. This is 23 ms above the legacy floor at this flight — if
it is refused `REJECTED_DWELL`, read the number in the message and use it.

**(c) 5 throws at `dwell_time_s: 1.2`, `throw_delay_s: 1.0`.** Only if (b) was
clean. **This is the largest `num_throws` for the first sitting.**

**(d) 3 throws at `dwell_time_s: 1.05`, `throw_delay_s: 0.87`** (beat ~1.69 s).
The shortest beat this rung can ask for. `throw_delay_s` **must** come down with
the dwell here, and 0.87 is the bottom of its own floor. This clears the legacy
dwell floor (1.047 s at this flight) by only 3 ms — if it is refused
`REJECTED_DWELL`, the message names the exact number; add 10 ms and retry rather
than lowering `throw_delay_s` any further. Expect the planner to
start refusing windows somewhere around here — that is a finding, not a fault,
and the session stops by name (`STOPPED_CHAIN_REFUSED`) rather than quietly
lengthening the beat.

**(e) The refusal you should see, once, on purpose.** Ask for
`dwell_time_s: 0.7` and expect **`REJECTED_BEAT_TOO_SHORT`** naming *"the next
cycle's lead"* — nothing armed, nothing lifted, nothing commanded, the machine
still where it was. Confirming that the floor refuses before it flies is worth
one goal; it is also the cheapest way to know the build you are running has the
UH-7a gate in it at all.

### What to watch in the console

- **`unified STEADY chained (depth N): duration … s, solve … ms, installed X s
  before the superseded deadline (budget 0.60 s)`** — **X is the number to
  watch.** Healthy X is roughly `dwell − solve`. A ring whose X is drifting
  toward zero is one slow solve away from the cliff.
  It prints on cycles **1 … N−2**, not on every cycle: the extend fired in cycle
  k installs the window cycle k+1 lives in, so the last two cycles have nothing
  left to chain. **A 3-throw ring prints exactly ONE**, at depth 2. A 5-throw
  ring prints three, at depths 2, 3, 4.
- **`unified LANDING chained (depth 0)`** — expected **once**, on the
  second-to-last cycle. That is the window that brings the machine to rest, and
  depth 0 is correct (the plan is rest-terminal again).
- **`POSITIONING skipped — CHAINED cycle: …`** — expected on every cycle from 2
  on. The plan owns the platform; there is no pre-positioning move to make.
- **`blas threads: 1`** in the launch terminal, as always.

**Not expected — say so if you see any of these:**

- a second ~1 s hand lift, or any *"the hand is below the planner floor"* line,
  after cycle 1 — a healthy chain never returns to the floor mid-session;
- `POSITIONING skipped — platform is already at …` on a chained cycle (that is
  the *measurement* line, and a chained cycle should print the structural one);
- a second fresh LAUNCH accept line after cycle 1.

### What each new outcome means

| What you see | What it means | What to do |
|---|---|---|
| `REJECTED_BEAT_TOO_SHORT(dwell … < unified chain floor (the next cycle's lead) 0.800 s …)` | The beat leaves the next cycle no time to reach its announcement. Refused at acceptance — **nothing was armed, lifted or commanded** | Raise `dwell_time_s` to at least **0.800** (and mind the legacy floor above it), or raise `throw_height_m` for a longer flight. The message quotes both requirements and says which one bound |
| `REJECTED_BEAT_TOO_SHORT(throw_delay … < unified cycle-1 floor 0.866 s …)` | Different knob, different failure: **cycle 1's** release is derived from `throw_delay_s`, and below this floor the ball would leave later than the 0.5 s release grace allows and cycle 1 would abort `ABORTED_NO_RELEASE` | Raise `throw_delay_s` to at least **0.866**, and higher if the box is busy — early costs nothing. ⚠ Its "needs dwell >= 0.986 s" tail uses a representative handoff; at this rung's flight the real dwell floor is **1.047 s** |
| `REJECTED_CYCLE_PLAN(CHAIN_PAST: the chained release was … s ago …)` | **The beat is not wrong; the cycle was late.** This cycle spun up after its ball had already left, so there was nothing left to announce | Look at the *previous* cycle's verdict and settle times, not at `dwell_time_s`. Send me the line and the two cycles around it |
| `STOPPED_CHAIN_REFUSED(<the planner's own refusal>)` | **Expected, not a fault.** The planner refused the next STEADY window, the fall-back LANDING installed instead, the ball was caught and the machine settled — and the session then STOPPED rather than quietly carrying on at a longer beat. You will see `unified ring TRUNCATED` first, then exactly one more throw | **Raise `dwell_time_s` and re-run** (or the session limits if the refusal names `LIMIT_*`). The throws before the stop are good data — the point of stopping is that they are all at the SAME beat |
| `STOPPED_CHAIN_LOST(NO_WINDOW: STEADY: … \| LANDING: … — held …)` | Both windows were refused (or the planner never answered) and **the machine was HELD**. The cycle's own catch still counts | **Read the hold verdict at the end of the string first.** Anything other than a clean hold means a stroke may still be coming — treat the machine as moving. Send me the whole line |
| `toss_session has NO note_chain_refused: this is a STALE INSTALL …` | The built `jugglebot` package is older than the running node. The chain was held, but the session cannot stop by name and may report `COMPLETED` after losing its ring (you would also see `ABORTED_CYCLE_REJECTED_CHAIN_LOST` in the record instead of a session terminal) | **Stop.** `colcon build --packages-select jugglebot`, re-source, relaunch. Nothing in this ladder is trustworthy on a stale install |
| `SUPERSEDED_BY_HOLD` in `trajectory_node`'s log | A `trajectory/hold` landed while a plan was being solved, and the solve was refused rather than allowed to install over the stop. **This is the guard working** | Nothing, on its own. It should appear only next to a cancel, a MISS or a chain loss. If it appears with none of those, send the log |
| `CHAIN_SKEW` | The session and the plan disagree about when the throw is, by more than one 40 ms tick. On this software they are the same number by construction | Stop the session. This is a finding — send the log |

### ⚠ A supersede alarm is a plumbing defect, not a tuning finding

If you see **`unified … chained X s AFTER the deadline it was racing`**, or
`trajectory_node`'s own `CLIFF` error line, that beat carried **10.90 mm** of
slider error inside every firmware guard (a true 93 rev/s of terminal hand
velocity emitted as 0.0 — no guard can see it, and the only symptom is a throw
that went somewhere else). **Stop the sitting and raise `dwell_time_s`.** Do not
retune around it. This alarm has never fired in production; UH-7a is the first
shape that can arm it.

### Two behaviour changes that are not about the ring

- **A cancel now issues a `trajectory/hold`, even at `num_throws: 1`.** If you
  Ctrl-C before the ball has left, the machine stops instead of throwing. That is
  stricter than before and it is correct: an operator who cancels before the
  throw gets no throw. **The cancel is still not the E-stop** — to stop NOW, use
  the E-stop.
- **On a MISS mid-ring you may see TWO hold lines.** The first stops the chain on
  the tick the miss is seen; the second answers a window that installed after it.
  One hold plus an *"ACCEPTED after the hold was issued"* error with **no** second
  hold would be a defect — send that.

### Record these

- the **achieved extend lead** — the `installed X s before the superseded
  deadline` number from every cycle. Worst and typical are both wanted;
- the **solve times** from the same lines;
- the **catch verdict per cycle**, and the achieved release-to-release beat;
- **hand `iq_rms`** across each rung, and how long you let the hand cool;
- the console log, the bag folder, and `/proc/loadavg` as for every other rung.

### Stop if

- a supersede alarm or a `CLIFF` line appears (above);
- the guard trips, or any E-STOP latches;
- `STOPPED_CHAIN_LOST` reports a hold that did not land;
- two consecutive cycles abort for any reason;
- the hand gets hot, or `iq_rms` is climbing rung over rung.

## UH-7b — two-pose ring at a constant beat

**Not in this sitting, and not one decision away.** It needs three separate
things: the aim-authority re-derivation against the 12° mechanical tilt ceiling,
a change to the goal itself (`TossContinuous` carries exactly ONE catch site —
there is nowhere on the wire to say "two poses"), and the FSM's legacy
displacement bound removed, since it measures a platform reach the unified plan
never performs.

---

## Results

**Sitting one (2026-09-06 evening) — the first unified cycles.** Canonical
record, and the place to read before the re-fly:
`logbook/2026-09-06-unified-cycle-first-hardware-cycles.md`. Run on `ce14e2f`
plus a `colcon build`. Artefacts: console
`temp/logs/cycle_ladder_20260906_193029.log`; bags
`~/Desktop/rosbags/2026-09-06_19-*` (two — a relaunch at ~19:32); driver
CSV/meta `temp/logs/unified_cycle_bench_{carry_193247,carry_193313,throw_193345,throw_193400,throw_193407}*`.

Before the rungs: P5 refused once at the shipped limits (precondition 11 fixed
it), one relaunch, and the very first `toss_continuous` goal was refused
`REJECTED_MOCAP_STALE` because QTM was not up — which is why precondition 2c now
exists.

| Rung | Verdict | Note |
|---|---|---|
| UH-3 | **PASS** | Both carries — `--dx 60` and `--dx 0 --dy 60` — **ACCEPTED and executed**, ball never disturbed. Plan **213–222 ms** (`qp` 7–11, `val` 191–202, `cont` 1–2), `load1` 3.7–4.7. The hand was **flat**: peak = the settle at **0.4949 rev**, driver printing *"0.0 mm above"*. V4 read 0.486 rev at t = 0.25 s on the first carry (the telemetry-stale ceiling) and 0.0049 rev on the second. **V6 FAILed on every rung and V7 SKIPped — both DRIVER defects, now fixed**, not machine findings. |
| UH-4 | **folded into UH-6** | As designed — the session is the only planned-catch path. |
| UH-5 | **PASS — with a release-velocity caveat that only the bag could see** | Three `--apex 0.5` throws. Joined LAUNCH+SETTLE, 2.0 s / 81 knots; plan **508 / 547 / 571 ms** (`val` 240–274, `cont` 237–268). Commanded hand peak **9.6432 rev @ 99.01 rev/s**; encoder worst **9.7591–9.8032 rev**, ~0.9 rev clear of the measured 10.701 metal. Guard ARMED, never tripped. **V2 FAILed against the driver's bare 500 ms bar — a driver defect** (a joined install is two solves plus the join's re-validate; the bar is now 1200 ms with an advisory at 700). ⚠ **The caveat: the throw left +5.2…+16.7 % fast** (mocap parabola, mean +10.9 %; flight time +10…+20 %; peak rev/s × empirical gain +12…+19 %) — flight **0.766–0.837 s** against 0.6387 planned, apex **556–682 mm** against the 500 asked. Nothing on the day showed it; it took the bag. |
| UH-6 | **FLOWN — 6 of 7 caught, and the CATCH QUALITY FAILS** | Four goals: G0 `REJECTED_MOCAP_STALE`; **G1 3/3 CAUGHT** (plan 505/593/500 ms, `load1` 4.6–5.4); **G2 3/3 CAUGHT** (606/585/530, `load1` 5.4–6.0); **G3 one MISSED → `STOPPED_ON_MISS`** (500 ms, `load1` 7.4). Guard ARMED throughout, never tripped. **Every cycle was a fresh joined solve from rest — no `EXTEND` fired all sitting** (see the UH-6 note above; the older "extensions ~200 ms" claim is corrected). ⚠ **The catch verdict: every catch was a FEEDFORWARD catch into a parked cup.** The ball arrives at **3961–4161 mm/s** onto a hand doing **−232…+4 mm/s** — a velocity ratio of **0.001–0.059** against the **0.7** the planner asks for — a **3730–4157 mm/s** mismatch (~4× design speed, ~17× design energy) absorbed by **2–7.4 mm** of the planned 120 mm runway. The plan itself executed correctly (cup at −1831…−2158 mm/s vs the −2199 target at the *planned* instant); the receive stroke simply completed **172–185 ms** before the ball arrived, and the ball was **146–195 ms** late. **Re-fly 2026-09-07 with the fixes: FLOWN — clean (operator).** Dwell/delay below the defaults produced `ABORTED_NO_RELEASE`, expected with the serial per-cycle choreography (lift + 1.80 s lead + solve); UH-7's steady chaining removes it. |
| UH-7a | **NOT FLOWN** | The steady-chain plumbing landed 2026-09-07 and has never moved a machine. The session now installs `LAUNCH + STEADY` and plans each cycle's window inside the previous one, so the beat is the number the operator asks for rather than the ~2.5–2.7 s the settle-plus-relaunch choreography needed. New at acceptance, both minting `REJECTED_BEAT_TOO_SHORT`: a chain **dwell floor of 0.800 s** (binding term "the next cycle's lead" — `max(verdict 0.560, extend 0.600) + preamble 0.160 + tick 0.040`) and a **cycle-1 `throw_delay_s` floor of 0.866 s** (`preamble 0.160 + measured joined solve 0.606 + window 0.600 − grace 0.500`); the legacy `dwell ≥ throw_delay + handoff` floor sits on top, so the shortest beat this rung can ask for is **~1.69 s** at `throw_height_m 0.5`. Also new: `REJECTED_CYCLE_PLAN(CHAIN_PAST: …)` for a cycle that spun up after its own release had passed, and a chained cycle's runtime release-window guard is one tick (0.040 s) instead of the legacy kind-0 dispatch budget (0.281 s) — charging the latter had aborted every chained cycle `ABORTED_CANT_MAKE_RELEASE` at a dwell the accept gate had admitted. New terminals: `STOPPED_CHAIN_REFUSED` (a window was refused, the fall-back LANDING caught the ball, the session stopped rather than quietly lengthening its beat) and `STOPPED_CHAIN_LOST` (nothing chained, machine held). ⚠ The release-terminal **supersede cliff is armed for the first time** — a `chained … AFTER the deadline it was racing` line, or `trajectory_node`'s `CLIFF` error, is a plumbing defect: stop and raise the dwell. First sitting capped at `num_throws` **3 then 5** (hand thermal — a ring never lets the hand rest, and nothing models that load). See the UH-7a rung above and `logbook/2026-09-07-unified-7dof-uh7-steady-chain-plumbing.md`. |
| UH-7b | **not run** | Two-pose ring. Blocked on three things, not one: the aim-authority re-derivation against `MAX_TILT_DEG` (12°), a goal-surface change (`TossContinuous` has exactly one catch site), and the FSM's legacy `REJECTED_DISPLACEMENT` bound. |

**The operator's three observations, all of which turned out to be the evidence.**
*"The catches are rough — the ball lands before or after the receive motion"* (the
feedforward catch, above). *"There is a small tilt step just before every throw"*
(**a levelling-frame drop**: the unified cycle never passed
`levelling.correct_pose`, and knot 0's tilt pin was in the plan frame while the
release, catch and banking pins were gravity frame — predicted step 0.671°,
measured **+0.669°**). *"The balls are thrown slightly backwards, hitting the top
of the hand axis"* (the same drop: the platform was physically at **+9.9…+10.3
mrad** at a release commanded to mechanical zero, and the ball left **−9 mrad
into −y on 7/7 throws** — the *opposite sign* to the legacy path's +8.5 mrad +y
bias — for **9–38 mm** of lateral drift and rim strikes on arrival).

**What went well and should not be re-litigated.** The host and the wire were
clean end to end: `blas threads: 1` on all three planner nodes, `max_emit_gap_ms`
p50 **26.0** / max **35.4** ms with **zero** samples over 40, zero
`interp_deadline_misses`, ≤ 3 µs of interp jitter, zero TX deferrals,
`link=1 fault=0`. **The morning's `MPC_STALE` E-STOP class did not recur.** The
tightest hardware number of the evening was the encoder: worst peak **9.8699
rev**, **0.089 rev = 2.8 mm** below the 9.9594 band ceiling.

### What changed since, and what must be true before the re-fly

**Fixed in the commit that follows this sitting** (all software, none of it
hardware-verified):

- **The levelling frame** — contract row **E8**. The correction is built once at
  the seed and carried on the cycle state and meta; the tilt schedule is built
  single-frame and re-expressed into the plan frame exactly once, in
  `unified_cycle._realize`. Measured on the probe: the release goes from
  **+11.663 mrad physical** to **0.000000**, the pre-throw step from **0.6682°**
  to **0.0000°**, and the flight's lateral drift at a 0.5 m apex from **23.325 mm**
  to **0.000**.
- **The catch replan gate opens under unified mode** — it had published zero
  messages all sitting — with a 5 mm movement gate on the published target and the
  cup/centroid lever applied at the consumer.
- **The session speed trim is honoured on the unified launch.** The live channel
  is the **ILC artifact**, not a knob.
- **The plan-service timeout hazard is closed**: an UNACKED call now holds the
  machine first and says so, instead of logging *"nothing was commanded"* while a
  plan installs anyway.
- **The driver stops crying wolf**: V6, V2 and V7 corrected, a new V2b on
  `max_emit_gap_ms`, and V3 scored against the **measured** 10.701 metal.

**Before the re-fly, all four:**

1. **QTM must be fixed first — this is the blocker, and it is operator-side.**
   Disable the `Catching Cone` rigid body and mask the Ball Butler's reflectors
   (precondition **2c**). On 2026-09-06 QTM bound the flying ball to the stale
   Cone body **60–80 ms after release on 5 of 7 throws**, and static BB reflectors
   minted **seven phantom balls**. Until both are done the tracker is blind for
   most of every flight and **the catch replan cannot be evaluated at all**.
2. **`colcon build`** the two packages, and confirm the installed switch
   (precondition 2b). The E8 fix is in `unified_cycle.py` and `trajectory_node.py`
   — a stale install flies the frame drop again, silently.
3. **Confirm `blas threads: 1`** in the launch terminal for `trajectory_node`
   (§ "Before UH-3 can be retried"). Unchanged from last sitting; it held.
4. **Drop the fitted ILC artifact** if you want the speed trim in the loop. The
   trim is **not** a parameter — the node reads `ilc_vel_trim` from the artifact at
   start-up behind a provenance match, bounded ±0.15. Last sitting every learned
   correction was inactive (`ilc_vel_trim 0.0 no_artifact`) while the ILC's own fit
   for this plant was **−0.1076**, which is most of the throw excess.

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
| `IN_MOTION` | The machine was still moving (a legacy move or a hold's decel) when you asked for a new plan, and there is no unified cycle running to read the motion from | Wait for it to stop (the status shows the plan idle), then retry |
| `REJECTED_CYCLE_INFEASIBLE(SETTLE_SITE: settle site z … outside the cup box …)` | The settle height is under the planner's 689.6 mm floor. The driver lifts to that floor on its own, so seeing this means you passed a `--z-mm` below it | Drop the `--z-mm` and let the driver pick the height, or pass one ≥ 689.6 |
| `REJECTED_CYCLE_INFEASIBLE(HAND_STROKE: hand position −0.0… rev … BELOW the homed zero)` | Pre-2026-09-06 only. The hand rests slightly below its homed zero after a retract (the homing reference is −0.1 rev by design) and the gate had no tolerance for it | Should no longer happen: the gate now takes the plan's own parked first knot as the floor. If it reappears, the hand is parked below −0.20 rev — outside the firmware's retract band — so re-home it |
| `REJECTED_CYCLE_INFEASIBLE(HAND_STROKE: … DIVES past the bottom of travel)` | The plan goes DOWN from where the hand is parked, i.e. it commands travel the machine does not have | Not a tolerance to widen. Send me the request — the site or the kind is wrong |
| `REJECTED_CYCLE_PLAN(HAND_BELOW_FLOOR: hand rests … < the planner floor 689.6 mm …; lift: …)` | **New 2026-09-07.** The hand is parked below the planner's 689.6 mm cup floor and the session's automatic lift did not land it. Nothing was solved and nothing was commanded — this is refused *before* the LAUNCH is planned. The message carries three numbers: how far below the floor the cup is, the hand rev, and the `6·Δ/dt²` acceleration the launch would have opened with (the 2026-09-06 refusals were 4063–4077 rev/s² against the 3500 cap). ⚠ A deficit of only ~11 mm is **still refused deliberately** — it is under the cap and would have been *flown*, with a 0.35 rev knot-1 step and the cup arcing past its release site | Read the `lift: …` tail — it says what went wrong with the fix. `not attempted` / `stale` ⇒ `/hand_telemetry` or `trajectory/commanded_position` is not arriving; `REFUSED` ⇒ the planner refused the lift itself, send me the message; `STILL … below the floor … did not move` ⇒ **the hand did not move when commanded** — check the hand ODrive is in CLOSED_LOOP (precondition 3/4), do not retry blind |
| `REJECTED_UNIFIED_AIM_UNSUPPORTED(\|B-A\| = … mm > 5.0 mm …)` | **New 2026-09-07.** You sent a unified goal with a `catch_position` away from where the platform is standing. The unified launch throws vertically and the plan owns the whole traverse, so that displacement is not an aim — refused at acceptance, before anything is armed or tilted | Set `catch_position` to the platform's live xy (the message quotes it) — for a centred machine that is `{x: 0.0, y: 0.0, z: 170.0}` — or run the goal with `unified_cycle: false` if you actually want the legacy aimed throw |
| `REJECTED_CYCLE_INFEASIBLE(START_BELOW_BOX: window starts at cup z … BELOW the cup box floor …)` | The cycle was asked to start from a cup height more than 20 mm under the planner's floor. A hand parked anywhere in its retract band is at most 16 mm under, so this says the START is wrong — a stale reading, or a hand outside its operating band — not the plan | Check the hand really is parked (`ros2 topic echo /hand_telemetry --once`) and re-home it if it is below −0.20 rev. Send me the number in the message either way |
| `carry excursion refused — the planner wants to lift the hand … during a flat carry` | The driver's own belt, not a planner refusal: the accepted plan would have stroked the hand far above the settle height. The plan was already installed, so the driver holds the machine as it refuses | **Stop the rung and send me the line.** Confirm the machine actually held (`held: holding at current pose` on the next line); if it says the plan IS STILL RUNNING, E-stop |
| `STALE_STATE` | Telemetry went quiet, or the machine moved during the solve | Wait two seconds and re-run. If it repeats, check the link: `ros2 topic echo /link_status --once` |
| `CHAIN_SKEW` | The session and the plan disagree about when the throw is | Stop the session. This is a finding — send the log |
| `ERR_HAND_SOURCE` on a legacy goal | You sent a legacy goal while the latch is STREAMED | Expected. Either add `unified_cycle: true`, or do the "After the sitting" recovery first |
| Guard trip / E-STOP latched, `fault_state=MAX_DEVIATION` | The hand deviated past 2.5 rev | `ros2 service call /clear_errors std_srvs/srv/Trigger` then re-activate. **Log it — this is the data we want** |
| Guard trip / E-STOP latched, `fault_state=MPC_STALE`, right after a refusal | **Not a hand fault and not a link fault.** The 40 Hz setpoint stream gapped for more than 250 ms while the planner was solving in the same process, and the Teensy's staleness watchdog latched. The name is historical — the MPC was deleted 2026-09-01; the watchdog is the setpoint stream's, and it is doing its job. **Cause found 2026-09-06: the planner was spreading over all six cores with spinning workers**, which shoved the setpoint sender off the CPU (1350–2314 ms solves, 225–942 ms gaps at three busy cores; 214–217 ms with the cap). Fixed in the launch file — so if this happens now, **check the `blas threads: 1` line first** (§ "Before UH-3 can be retried") | `ros2 service call /clear_errors std_srvs/srv/Trigger`, then re-activate. **Send me the whole accept/refuse line** — it now carries the per-stage split and `load1=` — plus `/trajectory/status.cycle_plan_wall_ms` and the `/proc/loadavg` capture |
