# Hardware Session — Hand ball-present sensor commissioning (Phase 7 commissioning — plan steps 1–5, as six runbook steps)

**Plan**: `plans/archived/2026-08-15 hand-ball-sensor.md` § Phase 7 (§ Architecture is **normative**
for the signal semantics this session commissions)
**Logbook**: `logbook/2026-07-29-hand-sensor-bridge-gpio-poller.md` (Phase 3, the poller),
`logbook/2026-07-29-hand-sensor-uplink-message.md` (Phase 4, the wire frame),
`logbook/2026-07-29-hand-sensor-ros-surface.md` (Phase 5, `/hand_telemetry` + `/link_status`)
**Goal**: prove that the switch in Jugglebot's hand — a ball seated in the cup shorts the
hand ODrive Pro's **G02** to GND — reaches ROS as a trustworthy tri-state, and size the
three provisional numbers the software shipped with (`max_missing_samples`, the 240 ms
staleness window, the bus cost of polling).

This validates **software already merged** on branch `mvp-trajectory-bringup` plus **one
bridge flash**. No code changes should be needed to run it.

> **⚠ Step 2 is a BLOCKING GATE.** Do not run steps 3–6 until it passes. The failure mode
> this whole session exists to catch is a **wrong-but-existing endpoint id**: on the Pro at
> fw 0.6.11, endpoint **700** (the value the repo carried before Phase 2, and the value
> BallButler's S1 correctly uses) resolves to `encoder_estimator1.status` — a read-only
> `uint8` that answers every request with a well-formed reply carrying a plausible,
> probably-constant byte. **There is no timeout to diagnose.** A live-looking sensor that
> never changes passes every check of the form "did we get a reply?". Only **bit 2 of the
> raw word moving on both edges** proves the endpoint id, the pin mode and the wiring at
> once. Never accept a decoded boolean as evidence.

**Step arc at a glance:**
| Step | What it proves | Ball? | Robot armed? | Motion? |
|------|----------------|-------|--------------|---------|
| **1** flash | FW_VERSION 5 (or 4 if the CAN3-gate fix has not been flashed yet) is running; the hand ODrive reports the fw the endpoint table is pinned to | no | no | none |
| **2** toggle gate | endpoint id + pin mode + wiring, end to end (**BLOCKING**) | no | **no** | none |
| **3** bring-up | home → activate → 40 Hz hold; the arming state steps 4–6 need | optional (see § Safe state) | **arms here** | **FIRST MOTION** (homing + the ACTIVE lift) |
| **4** reply cadence | the poll is healthy on the loaded bus; sizes the staleness window | no | yes | (a) hold, (b) **platform motion** |
| **5** stroke jitter A/B | polling does not degrade the hand's command stream | no | yes | **hand strokes** |
| **6** ball soak | raw-bit dropout statistics → sizes `max_missing_samples` | **yes** | yes | platform + hand |

**Plan step → runbook step**: plan 1 → **1**, plan 2 → **2**, *bring-up inserted as* **3**,
plan 3 → **4**, plan 4 → **5**, plan 5 → **6**. (The plan names five Phase 7 steps; this
runbook runs them as six, the extra one being the bring-up that makes the motion framing
honest. Cite runbook steps by *this* numbering, and say so.)

---

## Roles & safety framing

- **The operator (Harrison) runs every robot-actuating command below.** The implementing
  session prepared these exact commands + PASS/ABORT criteria and verifies read-only.
- **If your physical intuition disagrees with any framing here, that is load-bearing
  signal — say so before proceeding.** Especially on step 2: you are the one who installed
  the switch, and "that's not how that contact behaves" outranks anything written here.
- E-STOP is always in reach. Any ABORT criterion ⇒ stop, capture the bag and the node
  logs, and debrief before retrying.
- **No motion command beyond what a step names.**
  - **Steps 1 and 2 command ZERO motion**, and the robot is **not homed and not armed**
    through both. The switch in step 2 is pressed **by hand, with the hand parked and
    nothing energised** — that is the safest hand-in-cup action of the whole session.
  - **Motion begins at step 3.** The bring-up step *is* the first motion: homing drives
    every leg to its endstop, and `activate` lifts the platform to the ACTIVE pose.
    Arming also happens there (automatic on ACTIVE entry — ARMING CONTRACT).
  - **Step 4(b)** commands platform motion; **steps 5 and 6** command hand strokes.
    **Steps 4, 5 and 6 all require the robot homed and ACTIVE** — they cannot be run
    before step 3.
  - *(This corrects an earlier framing of this runbook that said "motion starts at
    step 4 and it is a single profiled `smooth_move_hand`". Homing, the ACTIVE lift and
    the step-4(b) platform battery all precede it.)*
- **Safe state — required before EVERY hand-in-cup action.** "Hand-in-cup" means any time
  your fingers enter the cup or the hand's stroke path: the step-2 switch press, seating
  the ball, a mid-soak re-seat or nudge, and removing the ball. All four:
  1. **No commanded motion in flight.** No `go_to_pose` / battery running, no reload or
     toss goal, no `smooth_move_hand` call outstanding, no retry ladder mid-escalation.
  2. **Both axes confirmed at rest — score them separately.**
     - **Platform**: visually still for ≥ 2 s **and** `/robot_state`
       `motor_states[0..5].vel_estimate` ≈ 0 (equivalently: no move in flight).
     - **Hand**: `/hand_telemetry` `pos_meas` steady inside its park band.
       **Do NOT score hand stillness on `vel_meas`.** That field is the **hand**
       axis — `teensy_bridge_node` populates `vel_meas` from the hand, not the
       platform — and a *parked* hand's |vel| p99 is **1.82 rev/s** at the bottom
       and **5.39 rev/s** at the top
       (`logbook/2026-07-24-phase7-fourth-sitting-openloop-telemetry-ladders.md:146`).
       That is measurement noise on a stationary axis, so "`vel_meas` ≈ 0" is a
       test a genuinely-parked hand routinely fails, and a small real motion
       routinely passes. It is exactly why the reload ladders qualify on
       **position** band, never on velocity.
  3. **E-STOP within arm's length** (in your other hand for a re-seat mid-soak).
  4. **The hand axis is ENERGISED whenever the robot is ACTIVE.** Axis 6 is in closed-loop
     position hold; a stray command, a dispatch-retry ladder, or a fault-recovery retract
     can move the cup while your fingers are in it. *"Nothing is moving" is not "nothing
     can move."*
  - **Prefer seating the ball BEFORE `activate`** (i.e. during step 3, or by dropping back
    out of ACTIVE first). With the robot not yet ACTIVE nothing on the platform is in
    closed loop — and the sensor still reads, because the poll needs only a **powered,
    heartbeating hand ODrive**; it needs neither homing nor arming. Step 2 is the proof
    of exactly that.
  - **Mid-step re-seat**: return to a held rest pose first —
    `ros2 service call /trajectory/go_home std_srvs/srv/Trigger` — then re-check (1)–(3)
    before reaching in.
- **The poller is firmware-internal.** `gpio_poll` sends through `can_jugglebot_send()`,
  never `send_axis_frame`, so it does not touch the operator-locked axis-6 allow table
  (`hand_axis6_permitted`) and cannot command the hand. Nothing in this session widens
  that table.
- **Never blind-re-dispatch a hand move** (step 5). The hand-command ack lies ~59 % of
  calls in **both** directions; judge every hand move by `/hand_telemetry` `pos_meas`, not
  by the service response.

## Preconditions

- Branch `mvp-trajectory-bringup`, working tree clean, `git fetch && git status -sb`
  showing no divergence (parallel sessions work this branch).
- **The can-bridge Teensy's USB cable is plugged into the Jetson.** This is
  load-bearing and was **NOT** connected when this runbook was reviewed:

  ```bash
  lsusb | grep -i 16c0        # expect: 16c0:0483 Teensyduino Serial
  ls -l /dev/ttyACM0          # expect: the device node exists
  ```

  Without it there is **no flash path and no serial console**: step 1, step 2's
  surfaces A and B, and step 5's A/B toggle are all blocked. (Surface B is blocked
  transitively — it reads the bridge's uplink, which only carries a sensor row once
  step 1's flash has landed.) The Ethernet link to the bridge is a *separate* cable
  and does not substitute.
- Jugglebot powered, ODrives up, CAN3 healthy. **The hand ODrive (axis 6) must be
  powered and heartbeating** from step 1 on — the bridge only sweeps `Get_Version` for
  axes that have heartbeated, and the poller stays silent (no fault, by design) until
  that version lands.
- `run_mpc.py` is **NOT** running (sole-binder :5557 interlock).
- **No second binder on the Teensy link.** Do not run the bench sysid harness or any
  probe that binds UDP :5005/:5006 while the launch is up — the kernel hands one socket
  the whole uplink.
- Mocap is **not** needed for any step of this session.
- Artifacts directory for this sitting:
  ```bash
  mkdir -p ~/Desktop/Jugglebot/temp/logs/hand-sensor
  ```

---

## Pre-flight

### P0 — repo state

```bash
cd ~/Desktop/Jugglebot
git fetch && git status -sb
git log --oneline -1
```
- **Expected**: on `mvp-trajectory-bringup`, no divergence from `origin`, no unfamiliar
  working-tree changes. If `origin` is ahead or the tree carries edits you did not make,
  **pause and surface it** — another session may be mid-flight.

### P1 — power-cycle the can-bridge Teensy (standing session rule)

Power-cycle the can-bridge Teensy **before the sitting**, and log `uptime_ms` next to
every timing number you record. Tracking lag grows with that board's uptime (10 ms at a
fresh boot → ~240 ms at 30 h), so a timing number without an `uptime_ms` beside it is not
interpretable. Step 1's flash reboots the board anyway — but do this first so the
pre-flight measurements are on a fresh board too.

Once the launch is up (P2), read and record the board's uptime:

```bash
PYTHONUNBUFFERED=1 timeout 4 ros2 topic echo /link_status | grep -A1 'key: uptime_ms'
```
- Expect a small number of ms.
- **`ros2 topic echo --once` does not exist on this box's Foxy** — `timeout N` is the
  idiom. Confirmed 2026-07-29: `ros2: error: unrecognized arguments: --once`. Other
  runbooks in this directory still show `--once`; they are wrong on this box.
- **An empty `ros2 topic echo` is a known CLI false negative on this box** (a Foxy
  discovery race on high-rate RELIABLE topics — `reference_ros2_topic_echo_flaky_foxy`).
  **This applies to every `ros2 topic echo` in this runbook.** On an empty result:
  **retry once** before believing it, and where a log-grep surface exists for the same
  fact (the launch log, the serial console), **prefer the log-grep** — it is not subject
  to the race. Never score an empty echo as an ABORT on the first attempt.

### P2 — deploy the Phase 5 Jetson code (BOTH packages)

`HandTelemetryMessage` gained four fields in Phase 5, so this is an **interfaces** change.
The launch runs the **installed** copy; a stale interfaces build kills consumers at
startup.

```bash
cd ~/Desktop/Jugglebot/ros_ws
colcon build --packages-select jugglebot_interfaces jugglebot
source install/setup.bash
ros2 launch jugglebot jugglebot_launch.py
```

Confirm the new surface is actually live **before** anything else. **Two checks, because
they detect different stale builds:**

**(a) The `jugglebot` package** — `/link_status` gained a `hand_ball_sensor` row:

```bash
PYTHONUNBUFFERED=1 timeout 4 ros2 topic echo /link_status | grep -A1 'key: hand_ball_sensor'
```
- **Expected now (bridge not yet flashed)**: `value: unknown (never seen)`.
- **No `hand_ball_sensor` key at all** ⇒ the launch is running a **stale `jugglebot`
  install**. Rebuild both packages, source, relaunch. Never score an absent row as a pass.
- **This row detects a stale `jugglebot` package ONLY.** It is a plain string KeyValue
  built inside `teensy_bridge_node`; it touches none of the four new
  `HandTelemetryMessage` fields, so it renders perfectly against a **stale
  `jugglebot_interfaces` build**. Building only `jugglebot` passes this check and still
  leaves the session dead — `teensy_bridge_node` writes the four new fields inside a
  `try/except` that logs one throttled error per 5 s, so the node stays in
  `ros2 node list` looking healthy while `/hand_telemetry` goes silently dead.

**(b) The `jugglebot_interfaces` package** — the check that actually exercises the
interfaces build, by requiring one of the new fields to appear on the wire:

```bash
PYTHONUNBUFFERED=1 timeout 4 ros2 topic echo /hand_telemetry | grep -m1 ball_held_valid
LOG=$(ls -td ~/.ros/log/*/ | head -1)launch.log
grep 'Hand telemetry error' "$LOG"        # must print NOTHING
```
- **Expected**: the echo prints a `ball_held_valid: false` line (false is correct — the
  bridge is not flashed yet, so the reading is UNKNOWN), and the grep is **empty**.
- **The echo prints `/hand_telemetry` messages with no `ball_held_*` fields** ⇒ stale
  interfaces build. Rebuild both, source, relaunch.
- **The grep prints `Hand telemetry error: …`** ⇒ same diagnosis, caught on the
  producer side: the node is failing to populate the new fields. This grep is the
  **preferred** surface of the two (it is not subject to the echo false negative — P1).
- **The echo prints nothing at all** ⇒ retry once (P1), then check
  `ros2 topic info /hand_telemetry` for a publisher.

**Then Ctrl-C the launch.** P3 needs `/hand_telemetry` quiet; the launch comes back up at
the end of P3.

### P3 — GUI ball-in-hand pill, synthetic check (no hardware)

The pill lives in the **System** panel, bottom-left cell, labelled *Ball in Hand*.
Three pill states — **HELD** (filled green) / **EMPTY** (hollow) / **UNKNOWN** (greyed,
dashed) — plus a **trailing `~`** marking raw-vs-debounced disagreement. The `~` is
**text only**: it does not change the pill's colour, border or shape, so the state stays
readable underneath it.

> **Why this runs against rosbridge alone, not the live launch.** With the full launch up,
> `teensy_bridge_node` publishes `/hand_telemetry` from a free-running **100 Hz** timer;
> the GUI's subscription is throttled to 100 ms, so the node's messages win almost every
> throttle window and a hand-run `ros2 topic pub` at a few Hz would barely show. Drive the
> pill with **only** the synthetic publisher on the topic.

With the launch stopped:

```bash
# Terminal A — rosbridge ONLY. No hardware nodes; the robot may be unpowered.
source /opt/ros/foxy/setup.bash
source ~/Desktop/Jugglebot/ros_ws/install/setup.bash
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090

# Terminal A2 — rosapi, same as the launch runs it. Only the GUI's topic-discovery
# panel needs it; without it the console fills with unrelated service errors and you
# cannot tell a real pill fault from the noise.
source /opt/ros/foxy/setup.bash && ros2 run rosapi rosapi_node
```

Open the GUI (`http://<jetson>:8081`) and **hard-refresh (Ctrl-Shift-R)** — the server
sends `Cache-Control: no-cache`, but the refresh costs nothing and the static JS/CSS
changed.

Terminal B drives the pill. **One command for every row** — substitute the three booleans
from the table, and Ctrl-C between rows:

```bash
# Terminal B — the synthetic publisher.
source /opt/ros/foxy/setup.bash
source ~/Desktop/Jugglebot/ros_ws/install/setup.bash
ros2 topic pub -r 2 /hand_telemetry jugglebot_interfaces/msg/HandTelemetryMessage \
  "{ball_held: <H>, ball_held_raw: <R>, ball_held_valid: <V>}"
```

| # | `ball_held` (H) | `ball_held_raw` (R) | `ball_held_valid` (V) | Pill must read |
|---|---|---|---|---|
| **P3.0** | *nothing published — just load the page* | | | **UNKNOWN**, greyed + dashed |
| **P3.1** | `true` | `true` | `true` | **HELD**, filled green |
| **P3.2** | `false` | `false` | `true` | **EMPTY**, hollow |
| **P3.3** | `true` | `true` | `false` | **UNKNOWN** — *not* HELD, and above all **not EMPTY** |
| **P3.4** | `true` | `false` | `true` | **`HELD ~`** — the trailing tilde, with the pill still filled green (no ring, no colour change) |
| **P3.5** | *Ctrl-C on P3.4 and wait ~1 s* | | | back to **UNKNOWN** (the message-stop watchdog) |

- **P3.0 is the load-bearing one.** `/hand_telemetry` does not publish at all until the
  bridge's first `Telemetry` frame lands, so the pill's **initial DOM state** must itself
  be UNKNOWN — the topic cannot be relied on to drive it there. If the pill is missing,
  blank, or shows EMPTY on a freshly loaded page with no publisher, **stop**: the System
  panel failed to build (check the browser console).
- **P3.3 is the safety property.** UNKNOWN must never render as EMPTY. `ball_held: true`
  is set deliberately in that message — the pill must ignore the bits entirely when
  `ball_held_valid` is false.
- **P3.4 holds the `~` indefinitely** because the disagreement is re-asserted at 2 Hz and
  the debounced verdict never changes. That is the *persisting* marker — the one that
  means real spotty contact. The complementary behaviour (the marker **clearing** the
  instant the debounced verdict flips) cannot be driven by hand here: switching payloads
  takes longer than the 1 s watchdog, so the pill passes through UNKNOWN and the latch
  clears for the wrong reason. **Step 2's release edge is the live surface for it.**
- **P3.5** proves the staleness idiom. Two independent covers exist and they answer
  different questions: `ball_held_valid` (published by a **live node**) covers a dead
  **bridge**; the GUI's 1 s watchdog covers a dead **publisher** (node crashed,
  subscription dropped) — without it the pill would freeze on its last HELD forever. A
  websocket drop is covered by the same watchdog (the drop stops the topic, and the
  watchdog fires one period later).
- **PASS** ⇒ all six rows as stated. **ABORT** (do not proceed to hardware) ⇒ any row
  wrong, or any browser-console error naming `panels.js` / `main.js`. The pill is step 2's
  live surface; a lying pill during the toggle gate is worse than no pill.

Ctrl-C the standalone rosbridge and rosapi. **Leave the launch down** — step 1 flashes the
bridge (which reboots it) and relaunches afterwards, so bringing the stack up now would
only cost you a cycle.

### P4 — confirm what you are about to flash

```bash
cd ~/Desktop/Jugglebot
git status --porcelain ros_ws/src/jugglebot/Teensy_code_canbridge/     # expect: empty
grep -n 'FW_VERSION' ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
grep -n -A1 'namespace odrive_pro_0_6_11' ros_ws/src/jugglebot/Teensy_code_canbridge/protocol_config.h
grep -n -A7 'namespace JBBallDetect' ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
```
- **Expected**: `FW_VERSION = 5` (or `4` if the CAN3-gate fix has not landed in the
  tree yet — verified in-tree at `4` on 2026-07-29, before that fix);
  `odrive_pro_0_6_11::get_gpio_states = 726`; `JBBallDetect` with `ENABLED = true`,
  `GPIO_PIN = 2`, `CHECK_INTERVAL_MS = 20`, `MAX_MISSING_SAMPLES = 5`,
  `CHECK_TIMEOUT_MS = 100`, `EXPECTED_FW[3] = {0, 6, 11}`.
- **`get_gpio_states = 700` under `odrive_pro_…`** ⇒ **STOP.** That is the S1 value; see
  the blocking-gate box at the top. 700 belongs only under `odrive_s1_0_6_11`.
- A dirty firmware directory means uncommitted generated headers — resolve before
  flashing, or you will not know what you flashed.

### P5 — CAN3 A/B on the CURRENT FW4, **before** you flash v5

**Run this before the flash, not after.** The 2026-07-29 sitting
(bag `2026-07-29_22-37-06`) surfaced `bus1_health` flapping OK↔WARN at a **42.4 % duty**
with FW 4's poller live. The v5 fix changes the CAN3 command gate that produces that flap,
so the poller-ON/OFF discriminator is only measurable on the firmware that is on the board
**now** — flash first and the pre-fix arm is gone. Skip this only if the board is already
running v5.

**Robot idle, ODrives powered, no motion.**

```bash
ls -l /dev/ttyACM0
source ~/Desktop/PDJ_venv/venv/bin/activate
cd ~/Desktop/Jugglebot/ros_ws/src/jugglebot/Teensy_code_canbridge
pio device monitor -e teensy41
```

Capture **60 s with the poller ON**, then type `gpio_poll off`, capture **60 s**, then
`gpio_poll on`, capture **60 s**. Record the 1 Hz `[canhealth] jugglebot …` and
`[canerrs]  jugglebot …` lines for each phase.

- **The discriminator** is whether `tecInc` / `recInc` climb with the poller ON and go flat
  with it OFF, and which of `ack= crc= form= stuff= bit0= bit1=` dominates, plus the
  `txctx=` vs `rxctx=` split.
- **`pio device monitor` holds `/dev/ttyACM0`** — close it before any flash.
- **`gpio_poll off` is NOT persistent** — the poller boots ON
  (`gpio_poll.cpp:190`, `s_enabled = true`), so it must be re-typed after every bridge
  reboot or power cycle until the fix is flashed.

---

## Step 1 — flash the bridge and confirm firmware identity

**Robot state**: ODrives powered and heartbeating; **not homed, not armed, no motion.**

```bash
# Close any `pio device monitor` first — it holds /dev/ttyACM0 and the flash will fail.
source ~/Desktop/PDJ_venv/venv/bin/activate
cd ~/Desktop/Jugglebot/ros_ws/src/jugglebot/Teensy_code_canbridge
pio run -e teensy41 -t upload -t monitor
```

> **⚠ `-e teensy41` is load-bearing — never omit it from a flash command here.**
> `platformio.ini` declares **two** environments (`teensy41` and
> `teensy41_bench_sysid`) and, historically, **no `default_envs`**. An `-e`-less
> `pio run … -t upload` therefore builds and flashes **both**, in file order — so the
> **bench system-ID binary**, the one whose own header says *"NEVER flash to the
> assembled robot"*, lands **last** and is what the board ends up running. It is a
> cadence-only variant (250 Hz telemetry, 100 Hz knots, a forced 250 Hz DIAGNOSTIC
> on axis 0) with **zero wire-format change**, which is precisely the problem:
> **every firmware-identity gate in this runbook — the boot banner, `gpio_poll`, the `HAND_SENSOR`
> row, the toggle gate — passes identically on it.** Nothing downstream would tell
> you. `default_envs = teensy41` is now set in `platformio.ini` as a second line of
> defence, but keep the explicit `-e teensy41` in every command: it is the one that
> survives someone editing that file.

`platformio.ini` calls the locally-built PJRC loader directly
(`/home/jetson/tools/teensy_loader_cli/teensy_loader_cli --mcu=TEENSY41 -w -s -v $SOURCE`)
because PlatformIO's bundled uploaders are glibc-2.34 and this box is 2.31. The **`-s`**
soft-reboots the running firmware into the bootloader over USB, so **no button press is
normally needed**. `-w` is the fallback: if the loader sits at *"Waiting for Teensy
device…"*, press the small white program button next to the USB connector.

> **Why `-t upload -t monitor` and not two commands.** The boot banner (check 1a) prints
> **once, at reset**. Flash first and open the monitor afterwards and the banner is
> already gone. Chaining the targets attaches the monitor immediately after the
> post-flash reset, so the banner lands in your scrollback.
>
> **If you miss it anyway, the only way to re-trigger it is another reset.** Verified
> 2026-07-29 against the source: the firmware's serial console has exactly **one**
> command, `gpio_poll` (`Teensy_code_canbridge.ino:290-307` dispatches to
> `gpio_poll_console()` and prints `[console] unknown command` for everything else) —
> there is **no** `reboot`, `version` or banner command. And `platformio.ini`'s `-s`
> soft-reboot drops the board into the **bootloader** as part of a flash; it cannot
> restart the running application on its own. So: **re-run
> `pio run -e teensy41 -t upload -t monitor`.** Re-flashing byte-identical firmware
> is harmless and costs ~15 s.

**Leave this shell open.** Every later `pio device monitor` in this runbook (step 2
surface A, step 5's A/B toggle) runs from **this same shell** — venv active, cwd
`~/Desktop/Jugglebot/ros_ws/src/jugglebot/Teensy_code_canbridge`.

| Check | Expected | If not |
|---|---|---|
| **1a** boot banner | `[boot] jugglebot-canbridge v5  eth link=1  ip=192.168.42.2` — **v5 (or v4 if the CAN3-gate fix has not been flashed yet)** | **Anything below the version you just flashed ⇒ the flash did not take.** Re-flash |
| **1b** LED | on-board LED blinking at 1 Hz | scheduler did not start — see `Teensy_code_canbridge/BRINGUP.md` § "If something goes wrong" |
| **1c** `gpio_poll` (type it into the monitor) | `[gpio_poll] enabled=1 state=<HELD\|EMPTY\|UNKNOWN> raw=0x……… raw_held=… miss=… stale=… synced=… mismatch=0` — those three are the console's **only** `state=` tokens, and **`UNKNOWN` is expected here** before any good reply has landed (or while `synced=0`) | `[console] unknown command` ⇒ pre-v4 firmware. `[gpio_poll] compiled out` ⇒ built with `jugglebot_ball_detect.enabled=false` |
| **1d** version park | **no** `[gpio_poll] VERSION MISMATCH … poller parked` line (it repeats once per second while latched) | see 1f |

> **1a is not the only firmware-identity surface.** It is the only surface that prints the
> FW_VERSION **number** (that number is not on the UDP wire and no ROS topic carries it),
> but two independent checks below discriminate v4-or-later from v3 without it:
> **1c** — `gpio_poll` is a v4-only console command, so *any* `[gpio_poll] …` reply proves
> v4-or-later and `[console] unknown command` proves pre-v4; and **1h** — the 1 Hz
> `HAND_SENSOR` keepalive is v4-only, so a populated sensor row proves v4-or-later reached
> the Jetson. Neither of them separates **v5 from v4** — the banner is the only surface
> that does, so if you need to know whether the CAN3-gate fix is running, score 1a. A
> missed banner is otherwise an annoyance, not a blocker.

Jetson-side, after relaunching the stack (`ros2 launch jugglebot jugglebot_launch.py`
from the P2 shell, so `install/setup.bash` is still sourced):

```bash
LOG=$(ls -td ~/.ros/log/*/ | head -1)launch.log
grep 'Jugglebot firmware check' "$LOG"
PYTHONUNBUFFERED=1 timeout 4 ros2 topic echo /link_status | grep -A1 -e 'key: odrive_fw_versions' -e 'key: hand_ball_sensor'
```

| Check | Expected | Notes |
|---|---|---|
| **1e** fw line | `Jugglebot firmware check PASSED — all axes match expected versions (fw 0:0.6.11-<u> 1:… 6:0.6.11-<u>)` | **On a partial bench rig this log line NEVER FIRES** — it is emitted only once every present Jugglebot axis has replied, and the sweep only queries axes that heartbeat. The `/link_status` `odrive_fw_versions` row is then the surface; absent axes render `?`, never a fabricated number |
| **1f** axis 6 triple | `6:0.6.11-<u>` | The **triple** is what matters: `0.6.11` is what the endpoint table (726) is pinned to. Anything else ⇒ **ABORT** — `gpio_poll` will latch the MISMATCH park and never send an RxSdo, which is correct behaviour, not a bug to work around |
| **1g** `fw_unreleased` | read `<u>` and **record it** | This is Get_Version's fourth byte, surfaced by Phase 0 specifically for this moment. The ODrive GUI reports the drive as `0.6.11-1`; the plan's expectation is that **the CAN frame does not carry the `-1`**, i.e. `<u>` reads `0`. Either value is fine for the endpoint id (0.6.11 and 0.6.11-1 share endpoint-tree CRC 55416) — **record what it actually says**, do not assume. `-?` means the byte was never surfaced (stale node) |
| **1h** sensor row | `stale miss=0 raw=0x00000000` (or `held`/`empty` if the poll is already running) | **`unknown (never seen)` ⇒ no `HAND_SENSOR` frame has ever reached the Jetson.** On a v4-or-later bridge the 1 Hz keepalive starts within ~1 s of boot regardless of whether the poll is working, so this row is a **secondary confirmation that v4-or-later is running** |

**PASS**: 1a–1h as stated.

**ABORT / next action, per check** (mirrors the PASS list — do not carry an unresolved
row into step 2):

| Failing check | What it means | Do this |
|---|---|---|
| **1a** banner reads below the version you flashed | the flash did not take (or you are watching a pre-flash scrollback) | Re-flash: `pio run -e teensy41 -t upload -t monitor`. If the loader waited, press the program button. **ABORT step 2** until a banner (or 1c/1h) confirms v4-or-later |
| **1a** banner never appears | the monitor attached after the reset | Not itself a failure — re-run `pio run -e teensy41 -t upload -t monitor`, or score v4-or-later from **1c + 1h** instead (they cannot tell you v5 vs v4). Record which surface you used |
| **1b** LED solid / not blinking at 1 Hz | FreeRTOS scheduler did not start (`task_diag` owns the blink) — the console and the poller are both dead with it | `BRINGUP.md` § "If something goes wrong"; check the fatal-path text on the monitor. **ABORT the session** — nothing downstream is trustworthy |
| **1c** `[console] unknown command` | pre-v4 firmware is running | Re-flash. **ABORT step 2** |
| **1c** `[gpio_poll] compiled out` | built with `jugglebot_ball_detect.enabled=false` | Fix the YAML, `python config/generate_config.py`, re-flash. **ABORT step 2** |
| **1c** `enabled=0` | v4-or-later is running but the poller was toggled off (a previous sitting's ABORT-park — see step 2) | Type `gpio_poll on`, confirm `enabled=1`, continue |
| **1d** `VERSION MISMATCH … poller parked` | the hand ODrive is not on the expected fw | Go to **1f**. **Do not** widen `EXPECTED_FW` to silence it — the park is protecting you from an unverified endpoint id |
| **1e** line absent | partial bench rig (expected), **or** a genuinely silent axis | Read `odrive_fw_versions` on `/link_status` instead. If axis 6 renders `?`, the hand ODrive is not heartbeating — power/CAN3 problem. **ABORT step 2**: an axis that never heartbeats never gets a version, and the poller stays parked |
| **1f** axis 6 is not `0.6.11` | the endpoint table (726) is pinned to a fw the drive is not running | **ABORT.** Do not proceed. Resolve the drive's firmware, or re-derive the endpoint id for the fw it actually runs |
| **1g** `<u>` reads `-?` | the fourth byte was never surfaced (stale node) | Rebuild + relaunch (P2), re-read. Not a step-2 blocker on its own — but **record** that you could not read it |
| **1h** `unknown (never seen)` | **no `HAND_SENSOR` frame has EVER reached the Jetson** — with a confirmed v4-or-later banner this is the **UDP link**, not the sensor | **BLOCKS step 2.** Surface B of the gate reads exactly this row, so a dead row means the gate has only one working surface. Check `/link_status` `bridge_link` and `rx_frames`; the bridge is Jetson-5V powered and outlives the ODrives, so it can be alive with the link down. Also see § Deferred — a `HandSensor` decode failure renders identically with nothing in the log |

**If the flash itself fails or you ABORT at step 1** (loader error, 1b, or a banner you
cannot get): the board is left running **whatever firmware was on it before**. Nothing in
this session has changed its behaviour, so **no park action is needed** — record the
FW_VERSION you last confirmed (`v4` if the flash never took: the operator flashed v4 on
2026-07-29 and the board has run it since) and note that subsequent sittings run on that
firmware unchanged. Do **not** leave the board half-flashed: a failed
`teensy_loader_cli` write leaves the bootloader waiting, and the board will not run
*anything* until a write completes. Re-run the flash (with the program button if needed)
before walking away.

---

## Step 2 — raw-word toggle gate (MANDATORY, BLOCKING)

**Robot state**: ODrives powered, hand axis heartbeating, CAN3 healthy. **Not homed, not
armed, no motion commanded, no ball.** The poller needs nothing more than a healthy CAN3
and a `Get_Version` match — this gate deliberately runs in the safest state the robot has,
and it is the reason § Safe state can promise that the sensor reads with nothing energised.

Press and release the switch **by hand**, with the hand parked. Required: **bit 2 of the
raw `uint32` word moves on BOTH edges, ≥ 5 complete cycles, on BOTH surfaces**, with the
GUI pill tracking.

**Polarity (active-low, from `gpio_poll.cpp`'s decode `raw_held = !((states >> 2) & 1)`):**

| Switch | Bit 2 | Canonical word | Pill |
|---|---|---|---|
| **pressed / ball seated** | **CLEAR** | `raw=0x00000000` | **HELD** |
| **released / empty** | **SET** | `raw=0x00000004` | **EMPTY** |

> `get_gpio_states` returns the **whole** GPIO bitmask, so other pins may legitimately
> read as set and the absolute word may not be exactly `0x00000000` / `0x00000004`. The
> gate is therefore: **`raw_pressed XOR raw_released == 0x00000004`**, and nothing else
> changes. Record both literal words.

**Surface A — serial console** (live, per-edge; type `gpio_poll` after each press and each
release). Runs from the **step-1 shell** (venv active, cwd `Teensy_code_canbridge`):

```bash
pio device monitor -e teensy41
# press  -> type: gpio_poll   -> expect ... state=HELD  raw=0x00000000 raw_held=1 miss=0 ...
# release-> type: gpio_poll   -> expect ... state=EMPTY raw=0x00000004 raw_held=0 miss>=5 ...
```

**Surface B — `/link_status`** (the ROS surface; lowercase hex, matching the console's
`%08lx` so the two compare textually):

```bash
PYTHONUNBUFFERED=1 timeout 180 ros2 topic echo /link_status \
  | grep -A1 'key: hand_ball_sensor' | grep 'value:' \
  | sed 's/ miss=[0-9]*//' | uniq -c \
  | tee ~/Desktop/Jugglebot/temp/logs/hand-sensor/step2_link_status.txt
```
- **The `sed` is load-bearing.** The row renders as
  `<state> miss=<n> raw=0x<word>`, and `miss` is a free-running counter that ticks on its
  own while the switch is released. Without stripping it, `uniq` sees a new value on
  nearly every 10 Hz sample and the transcript collapses to nothing — hundreds of
  count-1 lines instead of one line per edge.
- **Score edges on the `raw=` word**, not on the line count: each complete press/release
  cycle must produce a `raw=0x…4`-bit transition in both directions. The `state=` word is
  the *debounced* verdict and lags the raw bit by up to 100 ms; it is corroboration, not
  the gate.
- Output flushes in blocks through the pipe — the live surfaces are the serial console
  and the GUI pill; this file is the evidence.

**Surface C — the GUI pill**: HELD ⇄ EMPTY, tracking your hand.

**Expected on the RELEASE edge, and not a fault:** the pill shows **`HELD ~`** for **at
most one 100 ms frame**, then goes to plain **EMPTY** with no marker. That single frame is
the debounce doing its job — the raw bit flips immediately, the debounced verdict needs
`MAX_MISSING_SAMPLES = 5` **consecutive good replies reading raw-EMPTY** (100 ms at
50 Hz) to follow. Two consequences worth knowing before you score an edge: **a timeout is
not a miss** (it advances staleness only and never touches `miss_count`), so a lossy bus
*stretches* the edge rather than shortening it; and **the first reply after a stale gap
seeds straight from the raw bit**, so an edge that straddles a staleness window flips
**undebounced** — no `~` frame at all. The GUI clears the
marker the instant the debounced verdict changes value, precisely so that an ordinary edge
does not look like a fault; at the GUI's 100 ms throttle you may well not see the marker
at all on a clean edge, and that is also fine. The **press** edge has no marker (any
single HELD reading restores HELD immediately).

**A `~` that persists beyond the edge — i.e. still showing once the pill has settled on
HELD or EMPTY — is real spotty contact. Record it**: what the marker was on, how long it
lasted, and what you were doing. That is the whole point of the marker.

- **PASS**: ≥ 5 complete cycles, bit 2 moving on both edges, on surfaces A and B, with C
  tracking. **Proceed.**
- **ABORT** otherwise. Diagnosis table:

| Observation | Most likely cause | Next action |
|---|---|---|
| Replies are arriving (`state` is `held`/`empty`, `raw` is a stable non-trivial word) but **no bit moves** | **Wrong endpoint id** — the classic 700-on-a-Pro failure: a real register answering plausibly. Or the switch is not reaching the pin at all | Re-check P4's `get_gpio_states = 726` **in the flashed tree**, then the wiring continuity from switch to G02/GND |
| **A bit other than 2 moves** | Wrong pin: the switch is on a different G0x, or `gpio2_mode` is not `DIGITAL_PULL_UP` | Check the ODrive hand config `config.gpio2_mode == 1` — `tests/motion/test_hand_ball_detect_config.py::test_ball_detect_pin_is_flashed_as_a_pulled_up_input` asserts the committed JSON dump says so, but the DUMP is not the DRIVE; re-dump if in doubt. Then the physical harness. Note which bit moved — it names the actual pin |
| Bit 2 moves on **one** edge only | Intermittent contact, or you are reading the **debounced** word instead of the raw one | Judge on `raw=` / `raw_held=`, never on `state=`. If raw really is one-sided, it is a mechanical/wiring fault |
| **Console** reads `state=UNKNOWN`, **`/link_status`** reads `stale miss=0 raw=0x00000000` (two different surfaces, two different words for it — the console's `state=` token is only ever `HELD`/`EMPTY`/`UNKNOWN`, so `UNKNOWN` is where it puts *both* "no good reply has ever landed" **and** "the bridge's wall clock is un-anchored" — read `synced=` on the same line to tell them apart) | No good reply has ever landed, **or** `synced=0` | `gpio_poll` console line: `synced=0` ⇒ the clock, not the sensor (the reply may be fine); `mismatch=1` ⇒ version park (step 1f); otherwise check the hand axis is powered and heartbeating, and that CAN3 health is not WARN/BUS_OFF (the poller refuses to TX into an unhealthy bus by design) |
| `unknown (never seen)` on `/link_status` while the console looks healthy | The UDP link, not the sensor | `/link_status` `bridge_link` / `rx_frames`; the bridge is Jetson-5V powered and outlives the ODrives |
| Console prints `[gpio_poll] VERSION MISMATCH … poller parked` | The hand ODrive is not on 0.6.11 | Step 1f. **Do not** widen `EXPECTED_FW` to make the park go away — the park is protecting you from an unverified endpoint id |

**Record** (this is the plan's central claim, so write it down verbatim): the two literal
raw words, the cycle count, `uptime_ms`, and the fw triple from step 1f.

### ABORT-park — what to do with the bridge if step 2 fails

Do **not** reflash to back the poller out. Park it at runtime, from the step-1 shell:

```bash
pio device monitor -e teensy41
gpio_poll off        # -> [gpio_poll] enabled=0 …
```

Record in the results log: **"bridge left at FW_VERSION \<the version you flashed\> with
`gpio_poll` OFF"**.

**Subsequent sittings MAY run on this firmware — yes, deliberately.** With the poller off:

- **Zero added CAN3 traffic.** `gpio_poll` sends no RxSdo when `enabled=0`, so the bus load
  is byte-for-byte the v3 load. That is the only thing v4 added to the *robot's* control
  path.
- **The UDP wire is unchanged.** `HAND_SENSOR` is an **additive** `MsgType` and did **not**
  bump `PROTOCOL_VERSION` (still 4, the same value v3 shipped — `udp_protocol.h:12`), so a
  Jetson checkout that predates it simply ignores the frame. The 1 Hz keepalive keeps
  flowing with a stale/invalid cache, which renders as `stale …` or `unknown` on
  `/link_status` — informational, consumed by nothing.
- **Nothing else in 3→4 touches the control path.** `canbridge_config.h:44` records the
  whole delta: *"3→4: 2026-07-29 gpio_poll hand ball sensor — CAN3 get_gpio_states poll on
  task_homing + serial console"*. `FW_VERSION` itself is a human-facing identity marker
  with no runtime or handshake effect (same as the 1→2 bump).

The toggle is **not persistent** — a reboot or reflash comes back at `enabled=1`. So if you
park it, either re-park after every bridge power-cycle or (cleaner) fix the gate.

---

## Step 3 — bring-up: home → activate → confirm the hold

**FIRST MOTION OF THE SESSION.** Homing drives every leg to its endstop; `activate` lifts
the platform to the ACTIVE pose. Stand clear; E-STOP in reach.

**Entry**: step 2 PASSED. Stack relaunched (step 1). Nothing armed yet.

This reproduces the Phase-1 arm sequence used by every other session in this directory
(`tests/hardware/session_phase7_reload.md` § Preconditions →
`tests/hardware/mvp_bench_runbook.md` § S1/S2). Run it verbatim:

```bash
# 1 — launch. (If the stack is already up from step 1, skip this; the flag's `true`
#     value is INERT and `false` is the default, so the two forms are equivalent.)
ros2 launch jugglebot jugglebot_launch.py enable_setpoint_output:=false

# 2 — home, if `is_homed` is false. Mode/state commands go over the TOPIC, never the
#     same-named bridge services (runbook Sharp Edge #4), and are repeat-published
#     because `--once` loses the message to the DDS discovery race (Sharp Edge #5).
ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'home'"

# 3 — activate. Lands in ACTIVE:STANDBY and AUTO-ARMS (ARMING CONTRACT, 2026-07-15).
ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'activate'"

# 4 — confirm the 40 Hz hold stream (READ-ONLY probe; Claude may run this one).
python tools/probes/traj_stream_probe.py --duration 30

# 5 — TRAJECTORY mode. Needed by step 4(b)'s battery and step 6's carry moves.
#     STANDBY->TRAJECTORY is streaming->streaming, so it is safe armed.
ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'trajectory'"
```

| Check | Expected | If not |
|---|---|---|
| **3a** homing | every leg finds its endstop; `is_homed` true; no guard latch | A stale guard latch at BOOT is auto-cleared once (loud, disarmed). A latch that **returns** after the clear is a live fault ⇒ **ABORT** |
| **3b** activate | `Command received: activate` in the launch window; platform lifts to the ACTIVE pose; **zero motion at the arm edge** | Any visible jerk at arm ⇒ **ABORT** |
| **3c** hold stream | probe reads `rate_hz ≈ 40`, `u0_mean ≈ 2.19 rev`, `max_step ≈ 0`, `pump_rej = 0` | `rate_hz 0` with :5557 bound usually means you used the `/activate` **service** instead of the topic (Sharp Edge #4) |
| **3d** mode | `/control_mode_topic` reads `TRAJECTORY` **before** any move is commanded | A lost mode publish is silent and every step-4(b) move comes back `WRONG_MODE`. Re-publish and re-verify |
| **3e** sensor still alive | `/link_status` `hand_ball_sensor` still reads `held`/`empty` (not `stale`) with the robot armed and holding | A row that goes `stale` **only** once armed is real news for step 4 — record it and say so before proceeding |
| **3f** arm took | `/link_status` `mpc_active=1` — the reference arm sequence's last item, and the proof the **firmware** sees a live streaming arm (not just that the Jetson thinks it armed) | `mpc_active=0` with the hold stream apparently running ⇒ the arm did not reach the bridge. Steps 4(b), 5 and 6 all command motion that will simply not happen. Re-check the topic publishes (Sharp Edge #4/#5), then **ABORT** rather than commanding into it |

- **PASS**: 3a–3e as stated. The robot is homed, ACTIVE, armed, holding at 40 Hz in
  TRAJECTORY.
- **ABORT**: any E-STOP, any MAX_DEVIATION / MPC_STALE latch, visible motion at the arm
  edge, or drift you can see at the hold. Recovery: `ros2 service call /clear_errors
  std_srvs/srv/Trigger` (the latch survives ROS relaunches — only CLEAR_ERRORS or a Teensy
  power-cycle clears it).
- **Teardown, whenever you end the session** (any step): `trajectory/go_home` →
  orchestrator `deactivate`. Disarm-before-deactivate is bridge-enforced in-process
  (ARMING CONTRACT A3); a manual `set_setpoint_output false` is harmless but not required.

---

## Step 4 — SDO reply-cadence measurement

**Robot state**: homed, ACTIVE, armed, holding in TRAJECTORY (step 3). Two conditions —
**(a) holding** and **(b) platform motion**. The third condition the plan names (during a
hand stroke) is deferred into step 5's runs.

> *(Framing note: an earlier version of this runbook ran condition (a) with the robot
> unarmed. Running both conditions ACTIVE-and-armed is the better control — the only
> variable between (a) and (b) is then the commanded motion, not the arming state, and
> arming is itself a change in CAN3 traffic.)*

> ### Tooling honesty — read before running
> **The plan asks for request→reply latency. Nothing shipped measures that.** The bridge
> stamps the reply at arrival (`t_bridge_us`, surfaced as `/hand_telemetry`
> `ball_held_stamp`) and the console prints the resulting cache — **the request send time
> is never recorded anywhere**, on the wire or on the console, and `GpioPollSnapshot` has
> no latency field. Differencing anything available gives you the **reply cadence**
> (poll interval + RTT jitter + lost round trips), not RTT.
>
> **What today's build actually measures:**
> - **inter-reply interval** — successive *distinct* `ball_held_stamp` values. Healthy
>   mode ≈ **20 ms** (`CHECK_INTERVAL_MS`). A lost round trip shows as a ≈40 ms gap; the
>   poller's own timeout is 100 ms and a timeout is deliberately **not** a miss (it
>   advances staleness only, and never touches the verdict or `miss_count`).
> - **stale / unknown fraction** and `miss` from `/link_status` at 10 Hz.
> - the tail of that gap distribution is what should size `REPLY_STALE_US`
>   (currently `2 × (20 + 100) = 240 ms`, explicitly PROVISIONAL in `gpio_poll.cpp`).
>
> **To get true RTT you must choose one — this is an operator decision, not something to
> fudge:**
> 1. **Follow-up firmware counter**: stamp `micros64()` at the RxSdo send, difference it
>    against the arrival stamp already carried through the reply mailbox, and accumulate
>    min/mean/max. It is a small, contained change to `gpio_poll.cpp` + one console field.
>    Costs a second flash.
> 2. **Bench instrument**: a CAN analyser on CAN3 timing the 0x0C4 → 0x0C5 pair directly.
>    Costs no firmware but needs the tool on the bus.
>
> Record which you chose (or that you deferred RTT entirely and shipped on cadence).

**Run (a) — holding.** 60 s armed and holding the ACTIVE pose, no commanded motion, no
ball. Note the wall-clock start/stop so you can slice the bag.

**Run (b) — platform motion: MOTION-A.** The named, repeatable leg-motion sequence for
this session is the **standard bench battery**, i.e. the same **12 calls** that
`mvp_bench_runbook.md` § S2 uses — **11 rest-to-rest moves** (z 170→220→170; x ±150→0;
y ±150→0; tilt rx ±10°→0) plus **one deliberately-infeasible `duration_s: 0.05` request**
that must come back `TOO_FAST` and move nothing:

```bash
cd ~/Desktop/Jugglebot
python3 tests/hardware/traj_ramp_battery.py --dry-run     # print the plan, no ROS calls
python3 tests/hardware/traj_ramp_battery.py               # MOTION-A, shipped config
```

- It changes **no** limits (no `--set-*` flag) and arms nothing — it is a pure
  `trajectory/go_to_pose` client, so motion only happens because step 3 armed the robot.
- Bare (no `--lean-gain`) it defers to the shipped `lean_gain = 0.6`, which is the
  production configuration. **Do not pass `--lean-gain 0.0` here** — unshaped ±150
  traverses measure 0.94–1.08 rev deviation against a 1.0 rev guard, and a guard E-STOP
  mid-measurement wastes the run.
- One pass is ~35–40 s. **Run it twice back-to-back** for a comfortable 60 s; ≥ 20 s
  (≈1000 polls at 50 Hz) is the actual floor.
- **You may substitute your usual bench motion sequence** — but if you do, **write down
  exactly what you ran** in the results log. Condition (a) vs (b) is an A/B and the
  comparison is only meaningful if a future reader knows what (b) was.

`/hand_telemetry` and `/link_status` are **already in the launch's rosbag topic list**, so
the running launch is the recorder. Note the bag directory:

```bash
ls -td ~/Desktop/rosbags/*/ | head -1
```

Analysis (project venv; strictly offline, read-only):

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
python - <<'PY'
import glob
from mcap_ros2.reader import read_ros2_messages

BAG = '/home/jetson/Desktop/rosbags/<YYYY-MM-DD_HH-MM-SS>'   # <-- set me

samples, last = [], None
for path in sorted(glob.glob(BAG + '/*.mcap')):
    for m in read_ros2_messages(path, topics=['/hand_telemetry']):
        h = m.ros_msg
        st = h.ball_held_stamp.sec * 1_000_000 + h.ball_held_stamp.nanosec // 1000
        if st == last:
            continue          # the 100 Hz publisher repeats each 50 Hz sample
        last = st
        samples.append((st, bool(h.ball_held), bool(h.ball_held_raw),
                        bool(h.ball_held_valid)))

gaps = sorted(b[0] - a[0] for a, b in zip(samples, samples[1:]) if b[0] > a[0])
n = len(gaps)
print(f'distinct replies {len(samples)}   valid {sum(s[3] for s in samples)}')
if n:
    print(f'inter-reply gap us: min {gaps[0]}  p50 {gaps[n//2]}  '
          f'p95 {gaps[min(int(n*0.95), n-1)]}  max {gaps[-1]}')
    # 30 ms = halfway between one and two poll intervals, so the threshold is not
    # sitting exactly on the 40 ms a single lost round trip produces.
    print(f'gaps > 30 ms (a skipped poll): {sum(g > 30000 for g in gaps)}')
print(f'raw-bit transitions      : {sum(1 for a,b in zip(samples,samples[1:]) if a[2]!=b[2])}')
print(f'raw != debounced samples : {sum(1 for s in samples if s[1] != s[2])}')
PY
```

- **Dedupe by `ball_held_stamp` is not optional.** `/hand_telemetry` publishes at 100 Hz
  from cached state while the sensor is polled at 50 Hz, so every reply appears roughly
  twice; counting messages would halve every interval and double every dropout.
- The `mcap_ros2` plumbing above was validated 2026-07-29 against
  `~/Desktop/rosbags/2026-07-27_16-07-30` (3 `/hand_telemetry` messages read). The
  `ball_*` field access is **not** yet validated against real data — no bag carrying those
  fields exists until this sitting. On a **pre-deploy** bag the script fails with exactly
  `AttributeError: 'HandTelemetryMessage' object has no attribute 'ball_held_stamp'`,
  which is itself the diagnosis: that bag predates P2's `colcon build`.
- **≥ 1000 polls per condition** = ≥ 20 s at 50 Hz; 60 s gives ≈3000.

- **PASS**: ≥ 1000 distinct replies per condition; p50 gap ≈ 20 ms; the lost-round-trip
  count small and — this is the interesting comparison — **not materially worse under
  MOTION-A than at the hold**.
- **ABORT / stop and debrief**: a lost-round-trip rate that climbs sharply with bus load,
  or any `valid` fraction below ~99 % at the hold. Both point at the SDO pair losing
  arbitration on a bus that ADR-0013 already puts at 64–78 % utilisation — which is the
  scenario whose recorded fallback is rewiring the switch to a Teensy GPIO. Also ABORT on
  any E-STOP or MAX_DEVIATION latch during MOTION-A.
- If p95/max sit well inside 240 ms with margin, say so explicitly: that is the evidence
  that retires `REPLY_STALE_US`'s PROVISIONAL label.

---

## Step 5 — stroke-jitter A/B (polling on vs off)

**Robot state**: homed and ACTIVE (step 3) — the hand must be in closed loop for a
position move. One profiled hand move, **no ball**.

The A/B arm is the **runtime serial toggle — no reflash between arms**. From the step-1
shell (venv active, cwd `Teensy_code_canbridge`):

```bash
pio device monitor -e teensy41
gpio_poll off      # -> [gpio_poll] enabled=0 …
gpio_poll on       # -> [gpio_poll] enabled=1 …
```
(`JBBallDetect::ENABLED` in YAML is a *build-time* kill switch and is **not** the A/B
mechanism — flipping it costs a flash.)

The stroke: a single profiled prime-and-retract. **Verify each move by telemetry, never by
the ack.** Keep clear of the cup — this step energises and moves the hand.

```bash
ros2 service call /smooth_move_hand jugglebot_interfaces/srv/SetFloat "{data: 9.9594}"   # prime to top
ros2 service call /smooth_move_hand jugglebot_interfaces/srv/SetFloat "{data: 0.0}"      # retract
```

Run ≥ 5 prime/retract pairs per arm, alternating arms, logging `uptime_ms` for each block.

> ### Tooling honesty — what "0x0CC inter-frame timing" actually costs
> **No shipped Jetson-side surface can measure it.** The bridge sniffs the hand's
> `Set_Input_Pos` frames on CAN3 and uplinks them as `HAND_CMD_ECHO`, but that path uses a
> **single-slot stash that coalesces to the newest command** (`can_buses.cpp:385-404`) and
> is popped **once per telemetry tick at 100 Hz** (`telemetry.cpp:234-242`). Against a
> 500 Hz stroke stream that is a 5:1 decimation — the per-frame intervals are gone before
> anything leaves the Teensy. The Jetson-side handler then discards `t_bridge_us`
> entirely, keeping only pos/vel/tor.
>
> **What you CAN measure today, and what each one answers:**
> | Observable | Where | Answers |
> |---|---|---|
> | `can1_util_pct`, `can1_rx`, `can1_tx` | `/profile` (1 Hz; `can1` = **CAN3**, Jugglebot core, per `profiling.h`) | the poll's actual **bus cost** — tests the plan's derived **+1.1 pp** estimate directly |
> | `interp_max_jitter_us`, `interp_deadline_misses` | `/profile` | the 500 Hz **leg-interp ISR**'s health. Not the hand stream, but the poller shares `task_homing` with the rest of the cold-start work, so a scheduling cost shows here |
> | `pos_cmd` vs `pos_meas` | `/hand_telemetry` | the **outcome** proxy: if polling degraded the hand's command stream, hand tracking degrades. Caveat: `pos_cmd` is itself the 100 Hz coalesced echo |
> | your eyes and ears | the stroke | roughness at the top of the stroke is a real signal — say so |
>
> **True inter-frame jitter needs**: a CAN analyser on CAN3, **or** a follow-up firmware
> counter (min/max/histogram of deltas between sniffed 0x0CC frames in
> `decode_into_cache`, surfaced on the console or in `PROFILE`). Operator decision; record
> which you chose.

`/profile` is **not** in the launch's bag topic list, so capture it explicitly per arm:

```bash
PYTHONUNBUFFERED=1 timeout 30 ros2 topic echo /profile \
  | grep -A1 -e 'key: can1_util_pct' -e 'key: can1_tx' -e 'key: interp_max_jitter_us' \
              -e 'key: interp_deadline_misses' \
  | tee ~/Desktop/Jugglebot/temp/logs/hand-sensor/step5_profile_<on|off>.txt
```

- **PASS**: no visible or measurable degradation with the poller ON — hand tracking error
  unchanged within noise, `interp_deadline_misses` not increasing, stroke subjectively
  identical. Record the `can1_util_pct` delta between arms and compare it against the
  plan's +1.1 pp estimate.
- **ORDER THE SUPPRESSION KNOB** (plan § Approved decisions row 2 — a stroke-window
  suppression that stops polling during the stroke) if jitter is material: visible
  roughness, a rise in `interp_deadline_misses`, or a hand tracking-error increase that
  survives repetition.
- **ABORT** on any E-STOP, any unexpected hand motion, or a `MAX_DEVIATION` guard trip.
- **End the step with `gpio_poll on`.** Step 6 needs the poller running.

---

## Step 6 — ball soak (raw-bit dropout statistics)

**Robot state**: homed and ACTIVE (step 3), **ball seated in the cup**, then carry/tilt
motion. This is the step that **sizes `max_missing_samples`** — the shipped 5 is a 100 ms
window at the 50 Hz poll, and nothing has ever measured whether that is right for this
contact on this bus.

> **⚠ Every ball handling action in this step is a hand-in-cup action.** Re-read
> § Roles & safety framing → *Safe state* and satisfy items **(1)–(3) each time**, with
> **(4) in mind** (matching the mid-step re-seat rule there) —
> seating (1), every re-seat or nudge (3), and removal (4). The hand axis is in
> closed-loop hold whenever the robot is ACTIVE.

1. **Seat the ball — preferably BEFORE the robot is ACTIVE.** The clean order is:
   `trajectory/go_home` → orchestrator `deactivate` → seat the ball by hand → confirm the
   pill reads **HELD** and `/link_status` reads `held miss=0 raw=0x…` (the poll works
   unarmed — that is what step 2 proved) → then re-run step 3's `activate` +
   `trajectory` publishes.
   If you seat it while ACTIVE instead, satisfy the safe-state checklist first: motion
   stopped, platform at rest ≥ 2 s, E-STOP in your other hand.
2. **Run ≥ 3 minutes of carry/tilt motion — MOTION-B.** The named, repeatable
   ball-carrying sequence is a **reduced-amplitude cycle of the standard battery's own
   poses**, run as literal `go_to_pose` calls. Full-amplitude MOTION-A is deliberately
   *not* used here: **there is a loose ball riding in the cup, and the ±150 traverses are
   the largest excursions the session has.** (Guard headroom is *not* the concern — under
   the shipped `lean_gain = 0.6` those traverses measure ≤ ~0.45 rev against a 1.0 rev
   guard. The reason is the ball, not the latch.)

   ```bash
   # MOTION-B — one cycle (~10 s). Each call is rest-to-rest; wait for each to finish
   # (go_to_pose returns BUSY if a move is already in flight). Repeat ≥ 18 cycles.
   S=/trajectory/go_to_pose; T=jugglebot_interfaces/srv/GoToPose
   Q0='{x: 0.0, y: 0.0, z: 0.0, w: 1.0}'                      # level
   QP='{x: 0.069756, y: 0.0, z: 0.0, w: 0.997564}'            # rx +8°
   QN='{x: -0.069756, y: 0.0, z: 0.0, w: 0.997564}'           # rx -8°

   ros2 service call $S $T "{pose: {position: {x: 0.0, y: 0.0, z: 220.0}, orientation: $Q0}, duration_s: 0.0}"
   ros2 service call $S $T "{pose: {position: {x: 0.0, y: 0.0, z: 170.0}, orientation: $Q0}, duration_s: 0.0}"
   ros2 service call $S $T "{pose: {position: {x: 75.0, y: 0.0, z: 170.0}, orientation: $Q0}, duration_s: 0.0}"
   ros2 service call $S $T "{pose: {position: {x: -75.0, y: 0.0, z: 170.0}, orientation: $Q0}, duration_s: 0.0}"
   ros2 service call $S $T "{pose: {position: {x: 0.0, y: 75.0, z: 170.0}, orientation: $Q0}, duration_s: 0.0}"
   ros2 service call $S $T "{pose: {position: {x: 0.0, y: -75.0, z: 170.0}, orientation: $Q0}, duration_s: 0.0}"
   ros2 service call $S $T "{pose: {position: {x: 0.0, y: 0.0, z: 170.0}, orientation: $QP}, duration_s: 0.0}"
   ros2 service call $S $T "{pose: {position: {x: 0.0, y: 0.0, z: 170.0}, orientation: $QN}, duration_s: 0.0}"
   ros2 service call $S $T "{pose: {position: {x: 0.0, y: 0.0, z: 170.0}, orientation: $Q0}, duration_s: 0.0}"
   ```

   The tilt quaternions follow the battery's own convention
   (`traj_ramp_battery._quat_rx`: `(sin(θ/2), 0, 0, cos(θ/2))` about world x), and ±8° sits
   inside the 12° usable tilt ceiling (`tilt_geometry.MAX_TILT_DEG`).
   **You may substitute your usual carry sequence** (or escalate to MOTION-A once MOTION-B
   is clean) — **provided you record exactly what you ran**. Comparability across this
   session's A/B arms is the point.
   Note every visible `~` marker and what the platform was doing at the time.
3. **Provoke the failure mode once**: with the safe state satisfied, nudge the ball so it
   rides high in the cup without leaving it. The raw bit should chatter; the debounced
   verdict should hold HELD.
4. **Remove the ball by hand** (safe state again) and confirm HELD → EMPTY.

Analysis: the **same script as step 4**, over the soak bag, plus one line the step-4 block
does not print — append it inside the same heredoc, after `samples` is built:

```python
run = best = 0
for _st, _held, raw_bit, valid in samples:
    run = run + 1 if (valid and not raw_bit) else 0
    best = max(best, run)
print(f'longest consecutive raw-EMPTY run: {best} samples '
      f'({best * 20} ms at the 50 Hz poll)')
```

The numbers that matter:

- `raw-bit transitions` while the ball never left the cup — every one of those is a
  dropout the debounce had to absorb.
- `raw != debounced samples` — how much work the debounce actually did.
- The **longest run of consecutive raw-EMPTY samples with the ball still seated**. That
  is the number `max_missing_samples` must exceed.

- **PASS**: HELD holds continuously through the soak (no spurious EMPTY), and the longest
  raw-EMPTY run is comfortably below `max_missing_samples` (5 samples = 100 ms).
- **RETUNE** (not an abort): longest run ≥ 5 ⇒ a seated ball would have been reported
  EMPTY. Raise `max_missing_samples` — BallButler's effective window is 250 ms, which is
  **≈ 12–13 samples at Jugglebot's 50 Hz** (12 = 240 ms, 13 = 260 ms; 250 ms is not an
  integer number of our samples), a reasonable ceiling to tune toward. Treat it as an
  **analogy, not a like-for-like debounce**: BB's count gates a *recovery action*, not the
  reported bit, so the two windows are not protecting the same thing. Then regenerate
  config, reflash, re-run step 6, and record the measured basis in the YAML comment.
- **ABORT**: the debounced verdict flips to EMPTY with the ball visibly seated, **and**
  raising the window would need a run longer than ~250 ms to cover — that is a mechanical
  contact problem, not a tuning problem, and it belongs back with the switch install.

---

## Recording the results

Results go into a **new** `logbook/YYYY-MM-DD-hand-ball-sensor-commissioning.md`, written
by the session that runs the sitting, with **(date, command, result) triples** for every
number — not bare counts. Minimum contents:

- Step 1: the boot banner line (or which alternate firmware-identity surface you scored it from), the
  axis-6 fw triple **and** `fw_unreleased`, and whether the Phase 0 log line fired or the
  rig was partial.
- Step 2: **both literal raw words**, the XOR, the cycle count, all three surfaces
  confirmed, `uptime_ms`. Any `~` that persisted past an edge, verbatim.
- Step 3: that the arm sequence ran clean (or what it took), and whether the sensor row
  changed state on arming (check 3e).
- Step 4: reply-cadence percentiles per condition, lost-round-trip counts, **what
  condition (b) actually was** (MOTION-A or your substitution), and an explicit statement
  of whether `REPLY_STALE_US = 240 ms` is retired-as-measured or still provisional — plus
  which RTT option (firmware counter / CAN analyser / deferred) was chosen.
- Step 5: the `can1_util_pct` delta ON vs OFF against the +1.1 pp estimate, the jitter
  verdict, and whether the stroke-window suppression knob is ordered.
- Step 6: dropout statistics, the longest raw-EMPTY run with a ball seated, **what the
  carry motion actually was** (MOTION-B or your substitution), and the
  `max_missing_samples` verdict (keep / retune to N, with the measurement behind it).
- `uptime_ms` beside **every** timing number (standing rule).
- If you ABORT-parked the bridge (step 2), say so: **"FW_VERSION \<n\>, `gpio_poll` OFF"**.

Then the validated `/hand_telemetry` tri-state hands to the **possession workstream** at
the C-POSSESS-1 seam (`SOURCE_HAND_BALL_SENSOR`, reserved by name in
`ros_ws/src/jugglebot/jugglebot/ball_possession.py`).

## Deferred / open (carry into the debrief)

The content lives in the Phase 5 logbook entry and the plan; these are pointers, not
restatements:

- **Nothing consumes the tri-state yet, and `toss_require_ball_evidence` is NOT flipped by
  this session** — `logbook/2026-07-29-hand-sensor-ros-surface.md` open question 3, and
  `plans/archived/2026-08-15 hand-ball-sensor.md` § Out of scope.
- **The 3.0 s Jetson RX-freshness window is derived, not measured** (3× the Phase 4
  keepalive; nothing binds the two constants) — same entry, open question 2. If this
  sitting changes the keepalive, that window is silently wrong.
- **A `HandSensor` decode failure renders as `unknown (never seen)` with nothing in the
  log** — same entry, open question 1. Relevant only if step 1h stays stubborn against a
  confirmed v4-or-later banner.
- **BallButler's fail-open reload-skip defect** is a separate shipping bug with its own
  investigation; this session's two-hop validity pattern is the shape of its fix.
