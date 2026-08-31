# Bench session — unified-7DOF bus headroom + leg-bus frame-drop A/B

Phase 0, probe 3 of `plans/active/unified-7dof-planner.md`, run jointly with
workstream B of `plans/active/leg-bus-frame-drops.md` § 4.1.

## Purpose

The unified 7-DOF planner will stream the hand as a **seventh interpolated axis**
out of the bridge's 2 ms interp burst, taking jugglebot-bus TX from ~3000 to
~3500 frames/s (~56.5 % → ~65 % streaming utilisation). Two questions must be
answered on hardware **before** any planner work, and neither can be answered at a
desk:

1. **Does the bus have the headroom?** Utilisation, TX deferral, TX queue
   high-water and the per-class `[cantx]` census at 6 vs 7 frames per tick.
2. **Does the `leg-bus-frame-drops` per-axis encoder-frame drop rate scale with
   the bridge's TX rate?** That plan's leading hypothesis is ODrive-side TX
   suppression — a drive discarding its own pending cyclic sample when its TX
   mailbox is still occupied at the next 100 Hz tick, under the arbitration
   backlog our stream creates. § 4.1's A/B discriminates it, and the 7-frame arm
   pushes the same knob in the opposite direction on the same boot.

Established offline, **do not re-derive**: the design bound on simultaneously
pending jugglebot-bus TX frames is 8 (the 6-frame leg burst + the 100 Hz 0x7DD
time-sync broadcast + the 50 Hz hand SDO poll) against **16 TX mailboxes** since
FW 10's `setMaxMB(24)`; the 7th frame takes that bound to 9. FlexCAN's
`write()` returns +1 (mailbox) or −1 (**deferred** into the 64-slot software
txBuffer, transmitted in order) and never 0 — a deferral is a late send, not a
drop. `tx_q_hwm` is boot-monotone with no reset RPC.

## Predictions this session tests

| Hypothesis | Prediction |
|---|---|
| The bus has headroom for a 7th frame | Arm B `can1_util_pct` ~65 %, `defer jb` increment **0**, `txq jb` unmoved, `[cantx] legs=` unmoved |
| ODrive-side TX suppression (leg-bus-frame-drops § 2.5) | Per-axis encoder-frame deficit **scales with bridge TX rate**: lowest in arm D (250 Hz), baseline in arm A, highest in arm B (7 frames) |
| The drives' own TX pressure is the driver | Arm C (cyclic messages off) drops the deficit **without** the bridge's rate changing |
| Neither — the hypothesis is dead | Deficit unmoved across A / B / C / D; the search moves to the drives' broadcast scheduling and the 7-frame arm carries no sequencing consequence |

## Safety (operator-owned)

- **The whole sitting is BENCH. No ball, no catch sequence, no reload, no toss.**
  E-stop within reach and tested before arming.
- **Reboot-before-session is RETIRED** (2026-08-15, FW 14 closed the RX-ring
  leak). Flashing reboots the board anyway, so every arm here runs on a fresh
  plant by construction. **Still log `uptime_ms` with every measurement block.**
- **The BENCH image energises nothing new.** The 7th frame is a `set_input_pos`
  to the hand axis carrying the hand's **own measured position**, with zero
  `vel_ff` and zero `torque_ff` — see § "Hand-frame safety note" below for the
  full argument and the bus-topology check behind it. Treat the hand as live
  regardless; it is on the same bus and the same 48 V.
- **No hand command of any kind while `bench7` is on** — the 7th frame holds the
  hand at its current position at 500 Hz and would fight any dispatched hand move
  (co-driving). `bench7 off` before any CLEAR_ERRORS / re-arm / homing sequence.
- Platform motion during the battery arms is ordinary `traj_ramp_battery.py`
  motion at whatever session limits the operator is comfortable with. Nothing
  here requires a new or larger move.
- The CAN partners must be **alive** in every arm. A dark bus measures `tx_gated`,
  not `tx_deferred` — that is a different experiment and invalidates the arm.
- **ABORT** on: any guard E-STOP, any nonzero `can3_errors` wire-error field, any
  hand motion at all (the 7th frame is a position hold and must move nothing), or
  a lost bridge link.

## Preconditions (once, before the sitting)

| # | Step | Done |
|---|---|---|
| 1 | `./run_tests.sh --full` **green** — standing rule before any hardware sitting. Record the (date, command, result) triple in the Results section. | |
| 2 | `git status -sb` clean and `origin` not ahead — the image you are about to build must be the image in the tree. | |
| 3 | **Read § "Firmware skew — this sitting is also FW 16's first flash" below before continuing.** It is not optional; the board today is not the tree. | |
| 4 | Build BOTH images clean (the `extra_script.py` stale-object hazard is why `rm -rf .pio/build` is not optional): `cd ros_ws/src/jugglebot/Teensy_code_canbridge && rm -rf .pio/build && pio run -e teensy41_unified7_bench && rm -rf .pio/build && pio run -e teensy41` | |
| 5 | **Operator flashes the BENCH image**: `rm -rf .pio/build && pio run -e teensy41_unified7_bench -t upload`. Close any `pio device monitor` first — it holds `/dev/ttyACM0` and a flash with it open fails. | |
| 6 | Open the monitor **after** the flash: `pio device monitor -e teensy41`. Confirm the boot banner shows the six-line `** UNIFIED7 BENCH IMAGE **` block. **This banner and the repeating 1 Hz `[bench7]` line are the ONLY evidence of which image is aboard** (see § Firmware skew). | |
| 7 | Confirm the toggle booted OFF: the `[bench7]` line reads `frames=6 toggle=off`. If it does not, stop — something re-armed it and arm A is not a baseline. | |
| 8 | Launch with recording: `ros2 launch jugglebot jugglebot_launch.py record:=true`. Note the printed bag directory (`~/Desktop/rosbags/<stamp>`). The launch list is the union list and already carries `/link_status`, `/profile`, `/cache_diag`, `/ring_diag`, `/clock_diag` — do not hand-roll a topic list. | |
| 9 | Snapshot `/link_status`: `bridge_fw_version`, `bridge_tx_diag`, `uptime_ms`, `latency_monitor`, and the install-skew row. `ros2 topic echo --once /link_status` is flaky on this box — retry, or read the GUI. | |
| 10 | No `colcon build` is required for this change (it touches firmware and docs only). Confirm the install-skew row reads clean anyway; a stale install would mis-render the rows every arm depends on. | |

### Firmware skew — this sitting is also FW 16's first flash

**The board has self-reported `bridge_fw_version` 15 since ~2026-08-20** (the
hand end-stop image). `FW_VERSION` in the tree is **16** (poller cadence +
tri-state TX accounting), **committed unflashed** — which is why every launch
currently prints the advisory `15 (SKEW — expected v16)`.

The BENCH image is built from the current tree, so **it is an FW 16 image**, and
the stock reflash at the end of the sitting puts FW 16 on the board for the first
time. Three consequences, all owner-acknowledged before step 5:

- There is no way to fly this probe on an FW 15 base without back-porting. The
  sitting therefore also carries **FW 16's own first-flash acceptance**: the hand
  ball-sensor poller should now reach ~50 Hz (was ~42), and the per-class
  `[cantx]` deferral counters should exist at all. Read both in the same sitting.
- **`BRIDGE_FW_CHECK` will read OK during this sitting, and that is NOT evidence
  that stock is aboard.** `FW_VERSION` deliberately carries no bench marker (it is
  pinned to `teensy_link.rpc_args.EXPECTED_BRIDGE_FW_VERSION` by
  `tests/firmware/test_bridge_fw_version_xref.py`), and the bench image adds no
  UDP wire change and no cadence tell. With the toggle off it is behaviourally
  stock. The console is the only authority.
- **The BENCH image never leaves the bench.** Reflash stock at the end (row 24)
  and confirm the banner no longer shows the bench block.

## Procedure — the arms

Four arms, three of which need no reflash. Interleave A and B on **one boot** so
the 6-vs-7 comparison carries no uptime, temperature or seating confound. Run the
**identical** 11-move battery in every arm — `tests/hardware/traj_ramp_battery.py`
(its `_BATTERY` list is 11 accepted moves plus one deliberately-rejected
`TOO_FAST` demo). Referenced read-only, operator-run, never edited.

| # | Arm | Manipulation | Bridge TX | Drive TX | Battery |
|---|---|---|---|---|---|
| 11 | **A — baseline** | `bench7 off` (the boot state) | 6 frames/tick, ~3150 fps | nominal | 1× battery |
| 12 | **B — 7 frames** | `bench7 on` at the console | 7 frames/tick, ~3650 fps | nominal | 1× battery |
| 13 | **A′ — repeat baseline** | `bench7 off` | 6 frames/tick | nominal | 1× battery |
| 14 | **B′ — repeat 7 frames** | `bench7 on` | 7 frames/tick | nominal | 1× battery |
| 15 | **C — drive cyclic messages off** (§ 4.1 arm 1) | leg ODrive `iq_msg_rate_ms`, `error_msg_rate_ms`, `temperature_msg_rate_ms`, `bus_voltage_msg_rate_ms` → **0**; `encoder_msg_rate_ms` **stays 10** and `heartbeat_msg_rate_ms` **stays 100** | 6 frames/tick (unchanged) | −~162 fps per drive | 1× battery |
| 16 | **D — halved command rate** (§ 4.1 arm 2) | `INTERP_RATE_HZ` 500 → 250 — **needs its own build; see the note below** | 3 frames/ms, ~1650 fps | nominal | 1× battery |

**A / A′ / B / B′ ordering is deliberate.** `tx_q_hwm` is boot-monotone: whichever
arm drives it highest pins it for the rest of the boot, so run the light arm
first, read the **increment**, and never read a pinned value from an earlier arm
as evidence about a later one. The A → B → A′ → B′ interleave is also the
within-session drift control (the `session_err_timeout_bench.md` A/A′ pattern).

### Row 15 — how to disable the drives' cyclic messages, and how not to

The rates live in `config/ODrive config Files/odrive_pro_leg_config.json`
(`axis0.config.can.*_msg_rate_ms`). Today: `encoder` 10, `iq` 10, `error` 20,
`temperature` 100, `bus_voltage` 500, `heartbeat` 100, `torques`/`powers`/`version`
already 0. Zeroing `iq` + `error` + `temperature` + `bus_voltage` removes
~162 frames/s of each drive's own TX pressure — ~970 fps across six legs —
**without touching the encoder broadcast whose drops are the measurement**.

- **Nothing in the bridge's fault machine reads `temp_fet`, `temp_motor`,
  `bus_voltage` or `iq`** (verified by grep against `fault_machine.cpp`), so this
  arm disarms **no guard**. What it does blind is Jetson-side telemetry — the
  `DIAGNOSTIC` uplink's `iq_measured`, temperatures and bus voltage go quiet for
  the arm. Restore before any other work.
- **Apply and restore with `odrivetool` over USB, per drive.** Back up first
  (`odrivetool backup-config`); `tools/odrive_fleet_reflash.py` is the existing
  whole-config vehicle and its backup/restore discipline is the model.
- ⚠️ **DO NOT use the `SDO_WRITE` RPC for this.** `ArgSdoWrite.value` is a
  `float`, and `odrive_protocol.h::encode_sdo` `memcpy`s those four float32 bytes
  into the frame. `*_msg_rate_ms` is an **integer** property. Writing `0.0f` works
  *by accident* (float 0.0 and uint32 0 share the all-zero bit pattern), but the
  restore does **not**: `10.0f` is `0x41200000`, not `10`. The disable would
  succeed and the restore would silently corrupt the property. This asymmetry is
  the trap — use `odrivetool`.

### Row 16 — the 250 Hz arm needs a decision, and a third build

`leg-bus-frame-drops` § 4.1 calls this arm "no firmware", but on the current
architecture it is not: the bridge emits a leg frame **every interp tick** while
output is enabled, so the only knob is `canbridge_config.h`'s
`INTERP_RATE_HZ = 500`. **This is not built and is not in the BENCH image**, on
purpose — halving the interp rate halves the cadence of the lead clamp, the
stroke clamp and the whole 500 Hz safety ladder, which is a control-path timing
change and an owner decision, not something to fold into a bus-load probe.

Two honest options, for the owner:

- **Fly rows 11–15 only.** The A-vs-B pair is already a bridge-TX-rate A/B in the
  same direction the § 4.1 arm probes, just weaker: +17 % (3150 → 3650 fps)
  rather than −50 %. If the deficit moves between A and B, the hypothesis is
  convicted without row 16; if it does not, row 16's larger lever is worth its
  cost and can be decided on that evidence.
- **Authorise a `INTERP_RATE_HZ` 250 companion under the same BENCH guard** and
  fly row 16 in a follow-up sitting. One constant, the same default-off
  discipline; note that `interp_begin_stow`'s accel ramp granularity changes with
  it and the ladder's `s` is wall-clock-derived (so positions stay correct, only
  the update rate halves).

## Per-bag measurements, and the reading discipline for each

Capture **every** row below at the start and end of each arm. All counters named
here are **boot-cumulative**: read the **increment across an arm**, never an
absolute.

| # | Observable | Where | Reading discipline |
|---|---|---|---|
| 17 | `can1_util_pct`, `can1_rx`, `can1_tx` | `/profile` (wire slot 1 = jugglebot role). Standalone alternative: `tools/probes/teensy_link_profiling/jetson/profile_monitor.py` → `can1_util_pct` from `PROFILE.can1_util_x100/100`. | **The probe BINDS the STREAM port and cannot run while the launch is up** (single-owner UDP link). During a live sitting read `/profile`; use `profile_monitor.py` only for a launch-less capture. `can1_rx`/`can1_tx` are per-1 s-window rates already differenced on-chip. |
| 18 | `defer jb`, `txq jb` (and bb / cone) | `/link_status` → `bridge_tx_diag`, rendered `defer jb=N bb=N cone=N txq jb=N …` | `defer` counts `write() == -1`, i.e. **deferred into the 64-slot software queue, not dropped**. `txq` is a **boot-monotone high-water mark**, not a count — it never falls, so a value carried from an earlier arm says nothing about this one. Read `defer` as an increment; read `txq` as "did this arm push it higher". |
| 19 | `[cantx] defer_by_class poller= legs= hand= rpc= safety= timesync= coldstart=` | 1 Hz USB console (`Teensy_code_canbridge.ino`, the `[cantx]` block) | Console-only — it is not on the wire. Nominal on this bridge is **every column 0**. The 7th frame is `TxCls::LEGS`, so `legs=` is the column that would move; `legs=` at 0 in arm A and nonzero in arm B attributes the deferral to the 7th frame **by construction** (that A/B is the attribution — nothing else needed). |
| 20 | `[bench7] frames= toggle= sent= unseen_skip= stale_hold= last_pos=` | 1 Hz USB console | `sent` should climb at ~500/s throughout arm B and be **flat in arm A**. `unseen_skip` > 0 means the bridge never saw an encoder frame from axis 6 — **the 7th frame was never transmitted and arm B is invalid**; find the hand ODrive. `stale_hold` > 0 means the frame went out holding a last-known position: **expected 0 in every arm**; nonzero in B/B′ is a real hand-telemetry gap worth recording — the frame still transmits (hold-on-stale), so the 7-frame arm stays valid. |
| 21 | Per-axis encoder-frame deficit | `/cache_diag` keys `enc_frames_0` … `enc_frames_6`, plus `window_us` and `samples` | Difference `enc_frames_i` across consecutive 1 s windows; **deficit = Δenc_frames_i − (100 × window_us / 1e6)** (the ODrives broadcast `get_encoder_estimate` at 100 Hz) (negative = frames short, matching the 2026-08-15 reference numbers: clean −3.6 ± 10.3, episode past −20). Reduce per window, then per arm. Reference from 2026-08-15: clean windows sit at **−3.6 ± 10.3**; a window past **−20** is an episode; drop rate per streaming second has historically run **1.8–5.0 with no uptime trend**. Cross-check each deficit window against the `can1_rx` deficit — they correlated **r = 0.62, slope 0.82**, which is what localises the loss to *before* the bridge's CAN peripheral. |
| 22 | Validity counters that must stay clean | `/link_status` `can3_errors` (all 15 fields), `/ring_diag` `leak_*`, `fifo_overflows`, `fifo_warns`, `/cache_diag` `rx_cap_hits_*`, `decode_short`, `interp_deadline_misses`, `interp_max_jitter_us` | Any nonzero wire-error tick **invalidates the arm** and reopens the bus-health question — same rule as `session_err_timeout_bench.md`. `leak` must be **identically 0** at any uptime (FW 14's acceptance criterion); a nonzero leak means the RX ring is a delay line again and every deficit number is confounded. `decode_bad_axis` at a constant ~2/s is normal and appears in clean and drop windows alike. |
| 23 | `uptime_ms`, `latency_monitor` | `/link_status` | Log with **every** block. `latency_monitor` is the alarmed row from the FW 14 closure; it is the live plant-health verdict this sitting's numbers are conditional on. |

## Close-out

| # | Step | Done |
|---|---|---|
| 24 | **Restore the ODrive cyclic-message rates** if row 15 was run (`odrivetool`, from the backup — not `SDO_WRITE`). Verify against `config/ODrive config Files/odrive_pro_leg_config.json`. | |
| 25 | **Reflash the stock image**: close the monitor, `rm -rf .pio/build && pio run -e teensy41 -t upload`. | |
| 26 | Reopen the monitor and confirm the boot banner **no longer** shows the `** UNIFIED7 BENCH IMAGE **` block, and that `bench7` is now reported as an unknown command. The BENCH image never leaves the bench. | |
| 27 | Confirm `/link_status` `bridge_fw_version` reads `16 (proto 5)` with `BRIDGE_FW_CHECK: OK` — the FW 15 → 16 skew advisory should now be gone for good. | |
| 28 | Fill the Results section: the arm table, the deficit reduction, and the decision-rule verdict. | |

## Pinned predictions — decide before looking

| Observable | Arm A (6 frames) | Arm B (7 frames) | Expected Δ | What a miss means |
|---|---|---|---|---|
| Leg-burst TX | 3000 fps | **3500 fps** | +500 fps | Anything else and the toggle is not doing what it claims — check `[bench7] sent=`. |
| `can1_tx` (total, incl. 0x7DD + poll) | ~3150 fps | ~3650 fps | +500 fps | A **drop** rather than a rise re-opens the FW 10 MAXMB arbitration-scan hazard (the one RM-unverified inference in that fix): FlexCAN scans all enabled mailboxes per arbitration round, and the scan can cost a back-to-back transmission slot. It degrades bus THROUGHPUT, not ISR timing, so `interp_max_jitter_us` cannot see it. |
| `can1_util_pct` | 56.5 % | **~65 %** | +8.5 pp | Materially above ~70 % and the headroom answer is "not comfortably" — record it; the planner's frame budget is then a real constraint, not a formality. |
| `defer jb` increment | 0 | **0** | **0** | Nonzero re-opens the TX-deferral path that FW 10 parked and FW 14 P4 disarmed — see the desk verdict in the plan/logbook. Read `[cantx] legs=` to attribute it. |
| `txq jb` | 0 | **0** | 0 | Any rise means the software txBuffer was entered at all; approaching 64 is genuine loss (overflow overwrites the oldest). |
| `[cantx]` every column | 0 | **0** | 0 | `legs=` moving is the 7th frame; any other column moving is an unrelated producer and should be chased separately. |
| Design-bound pending frames | 8 | **9** | +1 | Both ≤ 16 mailboxes. This bound is arithmetic, not measured — the measurement is `defer`/`txq` staying 0. |
| `interp_max_jitter_us` / `interp_deadline_misses` | pre-arm budget | unchanged | 0 | The 7th frame adds one `can_jugglebot_tx` inside the 500 Hz ISR's existing PRIMASK region. A rise here is the ISR-cost read; the throughput read is `can1_tx` above and this row cannot see it. |
| Per-axis encoder deficit | baseline | **the question** | — | See the decision rule. |

## The decision rule (verbatim, from `plans/active/unified-7dof-planner.md` § Phase 0.3)

> **If the drop rate scales with TX rate, the `leg-bus-frame-drops` source fix
> sequences before Phase 3.**

Concretely: if the per-axis encoder-frame deficit is materially larger in arms B /
B′ (7 frames, ~3650 fps) than in arms A / A′ (6 frames, ~3150 fps) — and, if row
16 is flown, smaller again in arm D (250 Hz, ~1650 fps) — then the unified
planner's own 7th frame **makes the drop problem worse**, and
`leg-bus-frame-drops` workstream B's source fix must land **before**
unified-7dof-planner Phase 3 (hardware streaming), not after it.

If arm C (drive cyclic messages off) moves the deficit while the bridge's rate is
unchanged, the drives' own TX pressure is implicated independently — that is
§ 4.3's cheapest remedy and it sequences on its own merits.

If the deficit is unmoved by every arm, the ODrive-TX-suppression hypothesis is
**dead**, the search moves to the drives' broadcast scheduling (§ 4.1's own
falsification branch), and the 7-frame arm carries **no** sequencing consequence
for the unified planner.

## Hand-frame safety note

**Content: current-position hold.** The 7th frame is
`ODrive::encode_leg_setpoint(HAND_AXIS, axes[6].pos_rev, 0.0f, 0.0f)` — a
`set_input_pos` (0x0CC) to axis 6 carrying the hand's **own most recently decoded
encoder position**, with `vel_ff` and `torque_ff` hard zero. It runs through the
same `clip_position()` the hand path always runs, so the value cannot leave
`[0, HAND_MOTOR_MAX_POSITION]` = `[0, 10.8]` rev (the operator-measured metal
contact) even if the cache were corrupt; `leg_sign()` is identity for axis 6, so
the Jugglebot-convention cache value round-trips correctly. It never transmits
before the first encoder frame from axis 6 (`pos_timestamp_us == 0` skips and
counts on `unseen_skip`, because 0.0 rev is a real, reachable, **wrong**
position). Whatever state the hand ODrive is in, the command is a no-op: in
CLOSED_LOOP it asks the hand to stay where it already is; in IDLE `set_input_pos`
merely latches `input_pos` and moves nothing.

**No ACK-retry storm — and here is the topology check behind that claim, not just
the assertion.** CAN acknowledgement is a **bus-level, per-frame** mechanism, not
a per-recipient one: the dominant ACK slot is driven by **every** node that
received the frame without error, regardless of whether the arbitration id
addresses it. The jugglebot core bus carries the six leg ODrives, the hand
ODrive, the Platform Teensy 4.0 and the can-bridge itself (`canbridge_config.h`
bus map; the loom currently sits on the CAN2 controller in the jugglebot role
after the 2026-07-31 swap, which changes the controller, not the membership). So
with the six leg drives alive and heartbeating — a precondition of every arm
above — a 0x0CC frame addressed to axis 6 is **ACKed by the legs even if the hand
ODrive is absent, idle or unpowered**, the transmitter sees a successful
transmission, and there is no retransmission and no TEC rise.

Two corollaries worth stating, because they are what would actually go wrong:

- **The absent-hand failure mode is silent, not noisy.** An unpowered hand ODrive
  broadcasts no encoder frames, so `pos_timestamp_us` stays 0, the frame is never
  transmitted, and `unseen_skip` climbs instead of `sent`. Arm B then measures a
  6-frame burst while claiming 7. **Row 20 is the check that catches this**, and
  it is the single most likely way this sitting produces a confidently wrong
  number.
- **The genuinely noisy case is a bus with nothing else alive on it** — a
  single-node bus has no ACKer, `write()` succeeds, the frame retransmits
  forever, and TEC climbs to error-passive. That is exactly the "dark bus" the
  Safety section already forbids, and it is why "the CAN partners must be alive"
  is a validity condition and not merely good practice. `can3_errors` `ack=` is
  the observable; it must stay 0.

## Artifacts

- Bag: `~/Desktop/rosbags/<stamp>` from `record:=true` (the launch's union topic
  list — do not hand-roll one).
- Console capture: `pio device monitor` scrollback for the whole sitting; the
  `[bench7]`, `[cantx]` and `[canhealth]` lines are console-only and exist
  **nowhere else**.
- Battery: `tests/hardware/traj_ramp_battery.py` (read-only, operator-run).
- ⚠️ **Gap, stated so the next session does not discover it mid-analysis:** no
  committed tool computes the § 2.4 per-window deficit. The 2026-08-15 numbers
  (`logbook/2026-08-15-fw14-validated-arc-closed.md`) came from an ad-hoc
  reduction over `/cache_diag` + `/profile`. Row 21 states the recipe in full;
  promoting it to `tools/probes/` is the obvious follow-up and would make this
  sitting's answer reproducible rather than re-derived.

## Results — 2026-08-30

Flown by the operator 2026-08-30 15:59–16:05, one boot, rows 11–14 only.
Bag: `~/Desktop/rosbags/2026-08-30_15-59-52/` (46 MB, 337 s, 169 514 msgs).
Analysis: `python /tmp/probe_bag_unified7_headroom.py ~/Desktop/rosbags/2026-08-30_15-59-52`
(run 2026-08-30 — the ad-hoc § 2.4 reduction the Artifacts § flags as missing;
see Residuals).

Row 1 gate — `./run_tests.sh --full`, run 2026-08-30: **6676 passed, 4 skipped,
3 xfailed (+ 9 serial) in 580 s — PASS.**

Bridge self-report through the whole bag: `bridge_fw_version` **`16 (proto 5)`**,
`install_skew` 0, `BRIDGE_FW_CHECK` clean — **FW 16 is aboard**, and the
`15 (SKEW — expected v16)` advisory is gone (rows 3 / 27). Per the § "Firmware
skew" note this is *not* by itself evidence of which image ran; the console
banner is. The wire evidence that the BENCH toggle worked is the `can1_tx` step
below.

| arm | frames/tick | uptime_ms | can1_tx | can1_util_pct | defer jb Δ | txq jb | [cantx] legs Δ | deficit / streaming s |
|---|---|---|---|---|---|---|---|---|
| A | 6 | 133 → 236 s | **3150** flat (n=77) | 56.55 mean / **56.7 peak** | **0** | **0** | not on the wire | −0.78 ± 5.66 /axis/window → **4.65 f/s** over 6 legs; 9 eps / 103 w; **no battery ran** |
| B | 7 | 238 → 334 s | **3650** flat (n=77) | 62.13 mean / **62.2 peak** | **0** | **0** | not on the wire | −0.36 ± 4.24 → **2.14 f/s**; 2 eps / 96 w. Battery-only: −0.03 ± 0.36, **0 eps / 20 w** |
| A′ | 6 | 335 → 367 s | **3150** flat (n=25) | 56.58 mean / **56.7 peak** | **0** | **0** | not on the wire | −0.70 ± 5.74 → **4.19 f/s**; 3 eps / 32 w. Battery-only: −1.06 ± 7.06, **3 eps / 21 w** |
| B′ | 7 | 368 → 396 s | **3650** flat (n=18) | 62.20 mean / **62.2 peak** | **0** | **0** | not on the wire | **0.00 ± 0.00** → 0.00 f/s; 0 eps / 28 w. Battery-only identical |
| C | 6 (drives quiet) | — | — | — | — | — | — | **NOT FLOWN** — deferred by the operator; needs the per-drive `odrivetool` backup/apply/restore of row 15, which is a separate bench block, not a console toggle |
| D | 6 @ 250 Hz | — | — | — | — | — | — | **NOT FLOWN** — row 16's `INTERP_RATE_HZ` 250 companion build was never authorised, so no such image exists |

Row 19 / 20 (`[cantx]`, `[bench7]`) are **USB-console only** and are not in the
bag; the console scrollback for this sitting was not captured. Their wire proxy
is `bridge_tx_diag` (row 18, identically 0) and the `can1_tx` step (below).

**The 7th frame flowed — the B arms are valid.** `can1_tx` steps
**3150 → 3650 fps, exactly +500**, twice (t 175.9 → 176.9 and t 305.9 → 306.9),
and steps back to 3150 in between. That is the row-20 `unseen_skip` hazard
closed from the wire side: had the hand ODrive never been seen, the burst would
have stayed at 6 frames and `can1_tx` flat. `can1_util_pct` moves
**56.7 → 62.2 %, +5.5 pp** — *below* the pinned ~65 % prediction and well under
the ~70 % "not comfortably" line. Peak utilisation over the whole sitting is
**62.2 %**, matching the operator's GUI reading of ~62.5 %.

Row 22 validity, whole bag, every arm: `leak_jb`/`leak_bb`/`leak_cone` ≡ **0**,
`fifo_overflows_*` and `fifo_warns_*` ≡ 0, all fifteen `can3_errors` fields ≡ 0
(`ack=0` — the dark-bus failure mode did not occur), `rx_cap_hits_*` ≡ 0,
`decode_short` ≡ 0, `seen_mask` ≡ 127 (all seven axes present in every window),
`interp_deadline_misses` ≡ 0 and `interp_max_jitter_us` peaking at **2 µs in
every one of the four arms**, 6- and 7-frame alike (the whole bag's single 3 µs
sample is at t = 56.5 s, before streaming began) — the 7th frame inside the
existing PRIMASK region costs nothing measurable. `latency_monitor` **OK** on all 3358 samples;
`bus1_health`/`bus2_health` OK; `fault_state` NONE; `seq_gaps`/`crc_errors`/
`decode_errors` 0. `decode_bad_axis` climbs at 2.00/s, the documented normal.
`uptime_ms` 61.5 → 397.2 s: one boot, fresh plant, no arm above 6.6 min.

Row 21 cross-check: the 6-leg per-window deficit against the `can1_rx` deficit
gives **r = 0.453, slope 0.670** (2026-08-15 reference r = 0.62, slope 0.82) —
same relationship, so the loss still localises *before* the bridge's CAN
peripheral. Total **net** leg-frames short across the 259 one-second arm windows
(259 s of streaming, all four arms; A −479, B −205, A′ −134, B′ +0): **818**.
The 75 out-of-arm windows contribute −0.0, so 818 is also the whole-bag total.
(This line read "751 across 285 streaming seconds" until the same probe was
re-run against the same bag on 2026-09-01: **neither figure is reproducible** —
the arm windows span 259 s, not 285, and no denominator in the reduction yields
751. The 818 above supersedes it.)

**Decision-rule verdict: the drop rate does NOT scale with the bridge's TX rate.
The `leg-bus-frame-drops` source fix does NOT sequence before Phase 3.**

Pooled over all streaming windows, the per-axis deficit is **−0.76 ± 5.68 at 6
frames/tick (12 episodes / 135 windows)** against **−0.28 ± 3.73 at 7 frames/tick
(2 episodes / 124 windows)** — smaller at the higher TX rate, not larger.
Restricted to battery-moving windows, where the drops are load-gated and the
comparison is fairest, it is **−1.06 ± 7.06 (3 episodes / 21 w) at 6 frames**
against **−0.02 ± 0.25 (0 episodes / 41 w) at 7 frames**. Judged against the
A-vs-A′ within-boot spread — the runbook's designed drift control, here used as
the yardstick for "materially" — that spread
**exceeds the 6-vs-7 difference**: A's quiet windows sit at −0.78 while A′'s
quiet windows sit at exactly 0.00, a same-arm-type swing of 0.78 against a
pooled 6-vs-7 difference of 0.48 that points the *wrong way* for the hypothesis.
The +16 % (6→7 frames/tick; 3150→3650 fps) TX arm therefore returns
**"unmoved, if anything cleaner"** — § 4.1's
falsification branch, not its conviction branch.

Three qualifications, so this is not over-read:

1. **Only three of the four 11-move batteries are in the bag, and arm A carries
   none.** Platform motion (from `/leg_setpoint_echo`, corroborated by
   `/trajectory/commanded_position` and the single `TOO_FAST` rejection at
   t = 262.9 s) occurs in exactly three ~20 s blocks: t 242–262 (inside arm B),
   280–299 (inside A′), 313–332 (inside B′). Arm A's 103 s window is entirely
   quiescent streaming. So the moving-window comparison is **one 6-frame battery
   against two 7-frame batteries**, not the designed 2-vs-2, and the whole 6f
   moving signal comes from a single 20 s block.
2. **This plant is roughly 5× cleaner than the 2026-08-15 reference** (−0.76
   here vs −3.6 ± 10.3 there). The probe had correspondingly little dynamic
   range in which a +16 % (6→7 frames/tick; 3150→3650 fps) change could show up
   — which is exactly the argument
   row 16's −50 % lever was reserved for.
3. **The only hand-axis (ax 6) episodes in the entire bag fall in arm B** — four
   windows at t 202.5/226.5/227.5/232.5, worst −49 — giving arm B a hand mean of
   −1.22 against 0.00 everywhere else. The 7th frame addresses axis 6, so this is
   worth a second look; but **B′ (also 7 frames, 28 s) had none**, and the same
   one-axis-at-a-time, 1–2-window episode shape appears on axes 0–5 in arm A with
   no 7th frame at all. On this evidence it is the ordinary episodic pattern
   landing on axis 6, not a cost of the hand frame.

**Headroom answer: comfortable.** ~62 % streaming utilisation, zero deferral,
zero software-txBuffer entry, unchanged ISR jitter, every validity counter
clean. The planner's frame budget is a formality at 7 frames, not a constraint.

**Residuals / follow-ups:**

- **Arm C (row 15) not flown** — the drives' own TX pressure is untested, so
  § 4.1's independent second manipulation and § 4.3's cheapest remedy are
  undecided. It needs no firmware; it needs an `odrivetool` block per drive.
- **Arm D (row 16) not flown** — the `INTERP_RATE_HZ` 250 companion build was
  not authorised. Under the § "Row 16" decision, arm A/B returning *unmoved* is
  precisely the branch where "row 16's larger lever is worth its cost"; that
  choice is now live and informed, and qualification 2 above is the argument for
  it.
- **The § 2.4 deficit tool is still uncommitted.** This sitting was reduced by
  `/tmp/probe_bag_unified7_headroom.py` (arm segmentation from the `can1_tx`
  plateau, per-window deficit split by arm *and* by battery motion, row-22
  validity sweep). That file is the **seed** for the `tools/probes/` promotion
  the Artifacts § asks for — `/tmp` references rot at the next reboot
  (`tools/probes/README.md`), so promote it before the next sitting.
- **Console scrollback was not captured**, so rows 19 and 20 have no record at
  all for this sitting. `defer`/`txq` at 0 make the `[cantx]` census moot here,
  but `[bench7] sent=` / `unseen_skip` / `stale_hold` would have been the direct
  read; the `can1_tx` step is a sound but indirect substitute. Capture the
  console next time.
- **FW 16 first-flash acceptance is only half read.** The counters exist and the
  board reports 16; the hand ball-sensor poller's ~42 → ~50 Hz claim is
  console-side and unverified (`/link_status` `hand_ball_sensor` was static
  `held miss=0` all session, the bench having no ball).
- **Close-out rows 24–27 are unrecorded here** — row 24 is moot (arm C not
  flown), but whether the stock image was reflashed (rows 25–26) is not
  determinable from the bag. **The BENCH image never leaves the bench**;
  confirm before any non-bench session.
- Minor, not a validity trip: `/ring_diag` `lag_now_us` drifts ~225 µs/s and
  reseeds once at t = 308.5 s (`lag_reseeds` 0 → 1), peaking at 199 ms. Per
  `teensy_bridge_node.py`'s RING_DIAG normaliser note this integral is a
  *clock-divergence* quantity, not delivery latency, and `lag_corr_state` reads
  `uncalibrated`. `leak ≡ 0` and `latency_monitor = OK` are the health verdicts,
  and both are clean.
