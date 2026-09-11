# Bench session — skill-stack R1 flash (one hand master) + bench runbook

R1 of `plans/active/two-ball-skill-stack.md` § 4 deletes the `hand_source`
latch: the hand lane is now ALWAYS active while a `HAS_HAND` Setpoint frame is
latched. This is a **lockstep flash** (can-bridge FW 21, Platform FW 7,
PROTOCOL_VERSION 7) — every older board or host tree goes DARK against the
new one, by design.

## What changed for the operator

1. **No latch.** `/set_hand_source` and the driver's `--source-only` /
   `--no-source-switch` verbs are GONE. The hand lane follows `HAS_HAND` on
   every frame; there is nothing to switch and nothing to refuse a switch.
2. **`set_hand_traj_cmd` / `smooth_move_hand` are gone.** `SetHandTrajCmd.srv`
   and the legacy stroke path are deleted; the unified path is the only path
   that moves the hand.
3. **ACTIVATE parks the hand at 0 rev, energised, with no operator step.**
   `leg_activate` widens to axis 6: TRAP_TRAJ move to `hand_activate_position_rev`
   (0.0, the clip floor) at `JBOp::GENTLE_MOVE_VEL_LIMIT_RPS` (2.5 rev/s ≈
   81 mm/s), then the firmware itself hands axis 6 to POSITION/PASSTHROUGH.
   **Asymmetry to expect:** the legs are still handed to PASSTHROUGH by the
   *host* (`teensy_bridge_node._run_configure`) after ACTIVATE returns, so
   immediately post-activate the hand is already in PASSTHROUGH while the legs
   are not yet — both converge before arming, but don't read the gap as a
   fault mid-sequence. There is no `--close-loop` step in the launch-up path
   any more; that bench-only energise verb survives ONLY for driving
   `hand_stream_bench.py` with the launch down.
4. **The hand deviation guard boots ARMED** (owner decision, 2026-09-11;
   was observe-first). `hand7 observe` / `hand7 arm` remain as bench console (**`hand7 observe` lasts ONE armed session** — the disarm edge that ends the session returns the guard to ARMED, so nothing needs restoring afterwards; pinned by the FW 21 native test `hand7 observe lasts ONE armed session`)
   verbs for diagnosis, not as a required session step.
5. **Platform FW 7 never transmits to node 6.** `Trajectory.h`, the 0x6D0
   decode and the 0x0C9 hand-encoder cache are deleted from the Platform
   Teensy; the can-bridge is the one hand master on the bus.

## The lockstep flash

| Pairing | Result |
|---|---|
| FW ≤ 20 can-bridge board + this (R1) host | **DARK** — `link=NO_HEARTBEAT`, `decode_errors == rx_frames`. Reads like a cable fault; it is the intended failure (message types removed, struct shrunk — PROTOCOL_VERSION 6→7). |
| FW 21 can-bridge board + pre-R1 host | Dark the other way, same symptom. |
| FW 21 + this tree | The only live pairing. |

Platform FW 7 has no wire-version gate of its own (`platform_fw_version` is
advisory, warn-never-refuse per `ros_ws/docs/platform_fw_version.md`), but it
must still be flashed for `HOME(6)` / the stroke-engine deletion to be true —
a board still on Platform FW 6 keeps transmitting the legacy 0x6D0 path
harmlessly (nothing on the can-bridge listens for it any more) but is not the
board this plan describes.

**Rollback is a PAIR, never a half**: check out the R1 commit's parent for the
host tree, rebuild can-bridge FW 20 (or 19, whichever the board was running —
read it live at the sitting) from that same parent tree, and reflash Platform
FW 6 from the same tree. A half-rollback is loud (the link stays dark until
the pair matches), not silent — do not "fix" that by mixing versions.

### Flash order

**(a) Host, before touching either board:**
```bash
git fetch && git status -sb        # clean, origin not ahead
./run_tests.sh --full              # green; record the (date, command, result) triple
cd ros_ws && colcon build --packages-select jugglebot jugglebot_interfaces && source install/setup.bash
```
`jugglebot_interfaces` is rebuilt explicitly — `SetHandTrajCmd.srv`'s deletion
needs the interfaces package regenerated, not just `jugglebot`.

**(b) can-bridge Teensy 4.1 (USB):**
```bash
cd ros_ws/src/jugglebot/Teensy_code_canbridge
rm -rf .pio/build && pio run -e teensy41           # BUILD ONLY — nothing uploaded
```
Verify this is a clean build before continuing. Then, from here to the end of
the sitting, capture the console to a file (the `[hand7]` counters and every
row below exist ONLY on this serial console — see the model runbook's row 11c
for the exact `script -f` recipe):
```bash
mkdir -p temp/logs
script -f temp/logs/r1_console_$(date +%Y%m%d_%H%M%S).txt \
  -c "pio device monitor -d ros_ws/src/jugglebot/Teensy_code_canbridge -e teensy41"
```
In a second terminal, **THE FLASH**:
```bash
cd ros_ws/src/jugglebot/Teensy_code_canbridge && pio run -e teensy41 -t upload
```
Record the boot banner **verbatim** from the capture — expect
`[boot] jugglebot-canbridge v21  eth link=... ip=...`.

**(c) Platform Teensy (over CAN — its USB port is dead):**
```bash
# launch DOWN
cd ros_ws/src/jugglebot/Teensy_code_platform && pio run -e teensy40 -t upload
```
No boot banner is visible (USB console gone). The receipt is the **STATE_READ
version the tool prints before and after (6 → 7)**, ~55 s, or the GUI Hardware
panel's `platform_fw_version` after the next authoritative read. See
`ros_ws/docs/platform_fw_version.md` § Flash route. A first-attempt failure
here previously traced to host windowing, not the board — read that section
if the first attempt fails.

**(d) Relaunch:**
```bash
ros2 launch jugglebot jugglebot_launch.py record:=true
```
Confirm `/link_status`: `bridge_fw_version` **`21 (proto 7)`**,
`BRIDGE_FW_CHECK: OK`, **no `hand_source` row at all** (it no longer exists —
its absence, not a new value, is the proof this host tree matches), Platform
`platform_fw_version` **`7`**. Record the bag id.

## Dress rehearsal on the loaded Jetson, before power (CLAUDE.md rule)

Launch up, bag recording, GUI up. Run every dry-run/validate-only path this
sitting will use against the LIVE `/robot_state`, and report every refusal at
once rather than stopping at the first.

- `python3 tests/hardware/unified_cycle_bench.py --rung throw --dry-run` and
  `--rung carry --dry-run` — confirms the CLI and the generated constants it
  reads resolve. ⚠ **This is arithmetic only** — the tool's own banner says
  "no ROS calls made, no ROS objects constructed", so it does **not** check
  against the live `/robot_state`; it only proves the tree is importable and
  the derived numbers (e.g. `hand_mm_per_rev`-derived constants) are sane
  before anything is powered.
- `python3 tests/hardware/hand_stream_bench.py --help` — confirms the CLI
  resolves. ⚠ **`hand_stream_bench.py` has no `--dry-run` flag at all**
  (checked against its argparse block, 2026-09-11) — every stage listed below
  is a live command the first time it runs. There is no rehearsal path for it;
  say so rather than inventing one.
- ⚠ **`trajectory/plan_cycle` (the unified path both the flash bring-up and
  the R1 gate ride) has no validate-only or dry-run service.** `_svc_plan_cycle`
  installs on accept unconditionally for every mode (NEW/EXTEND/REPLAN,
  `trajectory_node.py:4255`); `feasibility.validate_cycle` is an internal gate
  called *inside* that install, not a separately callable preview. **First
  execution of the streamed self-toss (§ below) is therefore live** — there is
  no way to rehearse it against `/robot_state` without actually planning and
  installing a cycle. Report this rather than treating a passing `--dry-run`
  elsewhere as cover for it.
- ⚠ **Ladder row 18 (the cold ARMED trip) is also a first-live-execution
  event, not a dry run** — the hand deviation guard has never fired on
  hardware. Rehearsing it means re-reading the row's pass criteria and the
  recovery verb (`--clear-errors`) before power, not running it dry; there is
  no non-powered form of a guard trip.
- `git status -sb` clean (no uncommitted edits riding into the flash).

## Preconditions

| # | Check | Done |
|---|---|---|
| 1 | Hand mechanically clear: no ball, cords clear of the slider. | |
| 2 | No hand-geometry worktree in play (`~/Desktop/Jugglebot-geometry` stays unmerged and untouched — its measurement is already absorbed into this tree's `hand_mm_per_rev: 32.567`). | |
| 3 | `git status -sb` clean on this tree. | |
| 4 | E-stop within reach and tested before arming. | |
| 5 | Full gate green (§ Flash order (a)) with the (date, command, result) triple recorded. | |
| 6 | Read `tests/firmware/test_fault_machine.cpp`'s staleness sub-case ("setpoint staleness E-STOPs and LATCHES until an explicit clear (Gap 5)") and `tests/firmware/test_hand_single_master.py` — the two tests backing this rung's sole-writer and staleness guarantees. | |

## The ladder — rows re-cut from the FW 17 ladder for a latch-less lane

Launch DOWN for rows 12–19; `hand_stream_bench.py` is the sole UDP owner.
Console captured continuously per § Flash order (b). **Every refusal in a row
is reported at once, not first-refusal-stops.**

| # | Test | Command | Pass criteria |
|---|---|---|---|
| 12 | No latch exists | With the hand IDLE (not yet activated/energised this session), stream a hold: `python tests/hardware/hand_stream_bench.py --stage hold --duration 10` (no `--close-loop`). | The lane counts `unseen`/no TX until the first axis-6 encoder frame, and the IDLE hand does not move — this is now the operator's first POSITIVE check that ACTIVATE, not a latch, is what energises the hand. `[hand7]` prints with no `src=` field at all (the field is deleted, not just unused). Then `--stage hold --duration 10 --close-loop` (bench-only energise) → the same stream moves/holds the hand. Pass = both halves observed. |
| 12b | ACTIVATE parks the hand (NEW) | Launch UP for this row only: `ros2 service call /activate std_srvs/srv/Trigger` (the bridge node's existing activate service, `teensy_bridge_node.py::_svc_activate` → the firmware ACTIVATE op, which since FW 21 widens to axis 6 — the GUI's Activate button calls the same service). No driver verb exists for this row; it is a launch-up row. | After activate: hand `axis_state` **8** (CLOSED_LOOP), `controller_mode` **3** (POSITION) / `input_mode` **1** (PASSTHROUGH), encoder **0.00 ± 0.02 rev**. No guard residual — the FW 17 row-13 first-frame clip residual (a park below the clip floor reading as a permanent deviation) is GONE by construction: the park target IS the clip floor (0.0), so `dev_max` reads ≈0 on the first hold, not the 0.0995 rev / 3.15 mm the old below-floor park produced. Then deactivate → axis 6 IDLE, hand settles on the stop under gravity/friction (unpowered, no controlled park). |
| 12c | Hard-stop span (replaces the archived hand-geometry plan's G3) | ⚠ **No `hand_stream_bench.py` flag reaches the metal.** The driver clamps every commanded position to `[0, HAND_MAX_POS − HAND_MARGIN]` = `[0, 10.701 − 0.2]` = `[0, 10.501]` rev (`tests/hardware/hand_stream_bench.py:167-168,954`) — 0.2 rev short of the top stop and the triangle stage never commands below its own live start position, so neither stage can be driven onto the physical hard stop. **Use the same method as both prior owner readings** (`plans/archived/hand-geometry-correction.md`: 2026-09-06 bench, 2026-09-11 manual slide): with the hand de-energised (post row 12b's deactivate, or `--clear-errors` + leave IDLE), manually slide the carriage to each hard stop by hand and read the live encoder off `/robot_state` or the console `hand7` line at each end. | Span **10.81 ± 0.01 rev**, ends near **−0.11 / +10.69 rev** (matches both prior readings to 0.01 %, per the archived plan's table). **A span outside that band STOPS THE SITTING** — the generated `hand_mm_per_rev = 32.567` is wrong if so, and every site computed from it is wrong with it. |
| 13 | T-H1 streamed hold, 600 s | `--stage hold --duration 600 --close-loop` | Zero hand motion, zero guard trips, `[hand7]` deltas across the row `lead=0 dev_over=0 unseen=0 stale=0`, `dev_max` small and logged (the true post-flash zero — take the first post-flash `[hand7]` line as baseline). Drop-episode rate at the established 2026-08-30 baseline (CacheDiag CSV). |
| 14 | T-H2a slow triangle | `--stage triangle --tri-span 2.0 --tri-speed 0.5 --duration 60 --close-loop` | Smooth 0.5 rev/s motion over 2 rev; tracking error steady; `lead` delta across the row = 0; `dev_max` logged. |
| **15** | **DELETED.** The legacy-stroke replay stage (`--stage stroke`) and its host-side `smooth_move_hand` path are deleted with `hand_stroke.py` — there is no closed-form stroke left to replay. | — | — |
| 16 | T-H3a host step refusal | `--stage step --step-rev 6.0 --close-loop` | The pump refuses the oversized knot host-side (`pump refused=True`, reject counted); nothing reaches the wire; the hold continues unbroken; firmware counters unmoved (delta, bracketed). |
| 17 | T-H3c gap re-entry | `--stage gap --gap-pre 3 --gap-s 1.0 --gap-delta 1.0 --close-loop` | During the gap the lane DECAYS to rest (the normative falling edge, `leg_interp.cpp:686-748`); re-entry is one bounded ~32 mm catch-up move; `dev_over` may increment transiently (expected, log it); no E-STOP would be expected at this delta against the 2.5 rev ARMED band. `[hand7] sent` climbs across the stage; the driver's own echo-moved proof must pass (it aborts otherwise). |
| 17b | T-H3d moving gap (falling-edge decay) | `--stage moving_gap --duration 10 --close-loop` | The driver's own five criteria (G1–G5) all PASS or a SKIP whose reason is accepted, plus `[hand7]` deltas `lead=0 dev_over=0`, `sent` climbing, `discard_legacy` unmoved. Same closed-form coast prediction as the FW 17 ladder (`+0.0525 rev` at the defaults) — the rule did not change at R1, only the latch around it. |
| 18 | **The ARMED guard, trip observed COLD** (replaces the closed arming half) | `--stage gap --gap-delta 3.0 --gap-pre 3 --gap-s 1.0 --close-loop` | The 3.0 rev re-entry passes the 5.0 rev pump gate (`step-rev` boundary, row 16) and exceeds `MAX_DEVIATION_HAND_REV` (2.5) on the first post-gap tick → the guard E-STOPs and LATCHES, hand free (no rotor holding, no thermal load — this is the cold trip the FW 17 arc never got to run). Pass = the driver's abort line prints the latch trio with leg 6 (the hand), the hand stops, and `/link_status` (launch up) later reads `guard_fault_leg: 6`. Recover with `--clear-errors`, confirm `fault_state` returns to NONE, then re-arm (`--stage hold --duration 10 --close-loop`) and observe the recovery slew (≤ 1 rev/s). **Then the nuisance half**: re-run row 14's triangle ARMED → expect zero trips (the 2026-09-05 FW 17 record's worst residual under a full triangle was 0.25 rev = 10 % of the band). |
| 19 | **Sole writer** (replaces the deleted interlock row — nothing left to interlock against) | With the launch up and the guard ARMED, run a normal streamed session (row 12b's activate, then any hold/triangle bench exercise, or the § self-toss below) for the whole window. | ⚠ **No distinct counter exists for this check.** The bridge's CAN3 sniff (`can_buses.cpp:236-251`, `hand_cmd_echo_record`) records ANY `Set_Input_Pos` to axis 6 that is not its own TX (CAN3 `SRX_DIS` makes the bridge's own transmissions invisible to its own RX, so *any* sniffed frame is by construction a second master) — but the same `HAND_CMD_ECHO` uplink also carries the bridge's own interp-lane re-source echo (`telemetry.cpp::hand_cmd_echo_uplink_step`, the `interp_hand_sent()` branch), and nothing on the wire or in `teensy_bridge_node.py` (`_on_hand_cmd_echo` just stashes `pos/vel/tor`) distinguishes a genuine sniff from a self-echo. **Read this instead**: the Platform banner reads **v7** (Platform FW 7 deletes `Trajectory.h` and the 0x6D0 decode entirely — a board that still contains that code is the only way a second master could exist, so a v7 banner is the actual proof, not a runtime counter) and the hand's commanded/echoed pos in `hand_telemetry` never steps in a way inconsistent with the bridge's own knot stream. **Cross-unit ask**: a dedicated `sniff_second_master_count` (incrementing only on the `can_buses.cpp` path, separate from the interp re-source) would make this row a real counter check instead of an inference from the FW banner — flagging for whichever unit owns telemetry.cpp/BridgeTxDiag next. |
| 20 | Close-out validity sweep | `/link_status` + console | `can3_errors` all-zero, `leak_* ≡ 0`, `interp_deadline_misses` 0, `interp_max_jitter_us` within the established envelope, `latency_monitor` OK, `tx_deferred` 0, `bridge_fw_version` **21 (proto 7)**, Platform **7**. |
| 21 | Close-out state | Deactivate. | Hand deactivated, IDLE on the stop. No latch to restore — there is nothing left to leave in a particular state. |

## The streamed self-toss (the R1 gate)

Launch UP, through the existing unified path exactly as UH-6 flew it
(`tests/hardware/session_unified7_cycle_ladder.md` § "UH-6 — full planned
cycles", Preconditions rows 10–11 for `hand7 arm` and session limits). Ball in
the cup. **ONE** self-toss:
```bash
ros2 action send_goal --feedback jugglebot/toss_continuous \
  jugglebot_interfaces/action/TossContinuous \
  "{catch_position: {x: 0.0, y: 0.0, z: 170.0},
    throw_height_m: 0.5,
    num_throws: 1,
    dwell_time_s: 6.0,
    throw_delay_s: 5.0,
    catch_vel_scale: 0.0,
    stop_on_miss: true,
    on_empty_cup: 'STOP',
    max_reloads: 0,
    unified_cycle: true}"
```
**With no operator latch step anywhere in the runsheet** — there is no
`/set_hand_source` call before or during this goal; the hand lane is active
purely because the plan's Setpoint frames carry `HAS_HAND`. Record the bag id,
the `[hand7]` deltas across the throw (`lead`, `dev_over` must be 0), and
`dev_max`. Pass = caught, no guard trip, hand latch never mentioned because it
no longer exists.

## Results

| Row | Test | Verdict | Note |
|---|---|---|---|
| 12 | No latch exists | | |
| 12b | ACTIVATE parks the hand | | |
| 12c | Hard-stop span | | |
| 13 | T-H1 streamed hold, 600 s | | |
| 14 | T-H2a slow triangle | | |
| 15 | DELETED | — | — |
| 16 | T-H3a host step refusal | | |
| 17 | T-H3c gap re-entry | | |
| 17b | T-H3d moving gap | | |
| 18 | ARMED guard, cold trip | | |
| 19 | Sole writer | | |
| 20 | Close-out validity sweep | | |
| 21 | Close-out state | | |
| — | Streamed self-toss (R1 gate) | | |

**Boot banners:** bridge v21 text — ; Platform v7 STATE_READ — .
**Bags:** flash bring-up — ; self-toss — .
