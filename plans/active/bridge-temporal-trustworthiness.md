---
title: Bridge Temporal Trustworthiness — closing the uptime command-latency drift and the clock-precision half in one arc
status: active
owner: harrison
created: 2026-08-11
last_updated: 2026-08-11
related_logbook:
  - 2026-07-18-teensy-uptime-tracking-degradation.md   # the LATENCY half — open; owns the four-arm experiment and the closure contract
  - 2026-07-28-anomaly-fixes-validation-sitting.md     # the drift reaching the hand/throw dispatch path
  - 2026-08-10-sensor-truth-possession.md              # the drift sizing the ARRIVAL window (release lag vs uptime)
  - 2026-08-02-err-timeout-attribution-instrumentation.md  # the 0x8D/0x8E additive-MsgType precedent this arc's FW 11 follows
related_config:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h → FW_VERSION (the changelog constant, line 44 — FW 11 and FW 12 append here)
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h → TIMEOFDAY_RESYNC_MS
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_base.h → TIME_OFFSET_IIR_SHIFT / TIME_STEP_THRESHOLD_US / TIME_ANCHOR_STALE_US
  - config/generate_udp_protocol.py → ENUMS["MsgType"] (the wire-format authoring point; CLOCK_DIAG is added here, not hand-edited into the generated headers)
related_code:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py         # RX subscribe table, /link_status + /profile publishers, TimeOfDayServer host
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py              # THE ONE rosbag record list
  - teensy_link/tod_server.py::TimeOfDayServer._handle           # the Jetson-side TOD stamp
  - teensy_link/client.py::TeensyLinkClient._drain_socket        # the RX path that must yield a kernel RX timestamp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp::send_leg_cmd
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_base.cpp::set_wall_anchor / now_wall_us
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_sync_master.cpp::on_tod_response
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp::interp_on_setpoint / interp_isr
  - ros_ws/src/jugglebot/Teensy_code_canbridge/profiling.cpp     # udp_rtt_us / udp_jitter_us into the 1 Hz PROFILE frame
---

# Bridge Temporal Trustworthiness

**Branch:** `mvp-trajectory-bringup`
**Absorbs:** `plans/active/refactor-2026-07.md` § Phase 7's remaining bullet
*"Bridge-uptime tracking-lag reboot-isolation experiment (pre-registered;
calendar cost is real — the degraded cell needs a multi-hour soak)"*. That
bullet is satisfied by this arc's S1; `refactor-2026-07.md` is not edited by
this plan.
**Sequences:** `plans/active/bridge-clock-frequency-discipline.md`, which stays
the authoritative design document for the clock half. Its Phase 1 → this arc's
P1, its Phase 2 → this arc's P2 (with one superseding correction, § P2), its
Phases 3–5 → this arc's P4. No design content is duplicated here.

## Context

Two failure classes sit on the same Jetson↔can-bridge boundary, and until now
they were owned by two documents that explicitly declared each other
out of scope.

### Half 1 — command-latency drift with bridge uptime (the open half)

`logbook/2026-07-18-teensy-uptime-tracking-degradation.md` established that leg
setpoint→encoder tracking lag grows monotonically with the can-bridge Teensy's
continuous power-on time and resets on a Teensy reboot. Its seven-bag table
(lag = per-leg command↔encoder cross-correlation over every move on a 100 Hz
grid; `uptime_ms` from `/link_status`):

| session (bag) | bridge uptime at start | tracking lag (median, xcorr) |
|---|---|---|
| 2026-07-16 17-38-15 | 1.8 min (post-flash boot) | ~10 ms |
| 2026-07-16 18-45-29 | 1.15 h | ~40 ms |
| 2026-07-16 21-58-59 | 4.38 h | ~160 ms |
| 2026-07-16 22-06-30 ("silky") | 4.50 h | ~130 ms |
| 2026-07-17 17-35-14 | 24.0 h | ~250 ms |
| 2026-07-17 19-32-03 | 25.9 h | ~240 ms |
| 2026-07-17 23-20-20 | 29.7 h | ~230 ms |

The drift is not confined to the leg path. It reaches the hand/throw dispatch:
`logbook/2026-07-28-anomaly-fixes-validation-sitting.md` measured the
dispatch shift (fit − announcement) tracking uptime at 0.24 h → +14.5 ms,
0.54 h → +23.4 ms, 1.57–1.74 h → +54…+63 ms, and named it the 2026-07-18
finding "reaching the arm path". `logbook/2026-08-10-sensor-truth-possession.md`
then sized the possession ARRIVAL window *around* the drift rather than around
a fresh-boot plant, citing +54–63 ms against +118–133 ms at ~16 h uptime — the
window is deliberately 1.9× above the observed band precisely because a
fresh-boot-fitted window "starts refusing real catches as a sitting wears on".
Every one of those numbers is a symptom the fix should retire.

The 2026-07-18 entry's Addendum records a **full-file audit of the can-bridge
firmware** (18 .cpp + 20 .h + .ino, plus the pinned FlexCAN_T4 and QNEthernet
internals) that found **no mechanism whose cost or error grows with uptime** —
every hunted class located and cleared with a concrete reason. Two facts from
that audit shape this arc:

- **The Teensy-and-below contribution is bounded.** The Teensy's own
  `live_deviation` accounts for only ~30 ms of the added lag, while the
  Jetson-echo-referenced lag grew far more — most of the delay accrues
  **upstream of the Teensy's u0 latch** (`leg_interp.cpp::interp_on_setpoint`
  stamps arrival with `micros64()` and the ISR executes faithfully relative to
  that stamp).
- **"Resets on a Teensy reboot" never localized the fault to the Teensy.** A
  Teensy reboot bounces the PHY and therefore resets the Jetson-side link state
  too. The ranked candidate list is headed by **Jetson-side transport below the
  ROS emit point (USB-Ethernet driver/queue)**, followed by an ODrive-side
  accumulator, a stale-encoder-cache lead-clamp mechanism, and interp-tick
  overruns.

**The localization is UNCONFIRMED.** Candidate 1 is a ranking, not a verdict;
Arm A (ODrive) and Arm C (Teensy-internal) outcomes stay live until S1 runs.

The entry's **2026-07-24 Addendum** is the closure contract and is quoted here
because it is non-negotiable: closing that entry requires **two** deliverables,
not one — *(1) the fix itself, and (2) a continuously-measured, alarmed
end-to-end command-latency monitor (logged with `uptime_ms`)*. The class the
investigation exposed is "command-latency drift is invisible until a session is
already degraded"; a one-off fix without the monitor leaves the class open. The
same Addendum names the concrete telemetry gaps the monitor should close:
LEG_CMD not published, PROFILE not bagged, recover-slew and extrapolation-mode
occupancy not uplinked.

### Half 2 — clock precision (the designed-but-unstarted half)

`plans/active/bridge-clock-frequency-discipline.md` makes the bridge a
disciplined oscillator: the servo in `time_base.cpp::set_wall_anchor` is
offset-only (type-1), so a constant crystal rate error leaves a steady-state
phase bias of order `D·τ` (≈4.8 ms at 10 ppm with τ ≈ 480 s), and the *thermal
wander* of that bias is the residual few-ms session scatter. Its remedy is a
frequency term plus min-RTT anchor gating plus Jetson kernel timestamping, with
acceptance at **< 10 µs RMS over ≥ 1 h at stable temperature**.

### Why one arc

Because the two halves share one wire, one discriminator, and one ordering
constraint — see the coupling insight below.

## The coupling insight (2026-08-11 — new, and load-bearing)

The 2026-07-24 scope-boundary note ("clock sync can be perfect while command
latency drifts 10→240 ms") holds **only as long as the latency drift stays off
the UDP path, or stays symmetric on it.**

The firmware anchors the wall clock as
`set_wall_anchor(r.jetson_wall_us + rtt/2)`
(`time_sync_master.cpp::on_tod_response`, line 78). That `rtt/2` is a
**symmetry assumption**. If the drift lives on the UDP transport — the ranked
candidate 1 — then an asymmetric one-way delay of ~240 ms biases the **clock**
by up to about half of it. Three consequences follow, and all three are
structural rather than stylistic:

1. **`udp_rtt_us` is a single discriminator serving both halves.** RTT growing
   with uptime implicates the UDP path for the latency half *and* invalidates
   the anchor for the clock half. RTT flat under a growing command lag clears
   the clock half and narrows the latency hunt to below the transport.
2. **The latency fix is a prerequisite for the clock servo.** A frequency
   estimator trained through a drifting, possibly asymmetric transport bakes
   the asymmetry into `freq_ppb` as a real rate error. Hence P4-after-P3, with
   no exception.
3. **Min-RTT anchor gating is the clock's standing defence** against any
   *future* transport misbehaviour, not merely a precision optimisation. It is
   why that item stays in scope even if S1 exonerates the transport.

## Owner decisions (2026-08-10/11, harrison) — recorded verbatim

1. **Route: B then A.** Land telemetry/instrumentation first, then run the
   pre-registered four-arm isolation experiment, bundled into an upcoming
   sitting on a deliberately aged (16–24 h uptime) bridge.
2. **Both halves are tackled in ONE arc** (this plan), with the clock plan kept
   intact as the design reference.
3. **S1 runs BEFORE the P1 firmware flash.** Reason (operator's own): flashing
   firmware reboots the Teensy, which resets the aged state S1 needs. Corollary:
   P1's clock-baseline diagnostics are better captured *after* the latency fix
   anyway — measuring crystal ppm through a possibly-asymmetric drifting
   transport would contaminate the baseline. FW 11 is therefore written and
   committed now but flashed only after S1 (the post-experiment reboot is free).
4. **The reboot-the-Teensy-before-every-session standing rule is deliberately
   SUSPENDED for the S1 sitting only** — the aged state is the experiment's
   subject.

## Architecture — what is observable, and where

The command path, Jetson to leg:

```
trajectory_node ──:5557──► teensy_bridge_node ──UDP SETPOINT──► can-bridge Teensy
   (plan knots)                (accepted u0 echoed on            interp_on_setpoint()
                                /leg_setpoint_echo)              stamps arrival (micros64)
                                                                      │
                                                        interp_isr() 500 Hz ladder
                                                        Mode 1 Hermite / Mode 2 cubic
                                                        Taylor extrapolation / Mode 3 decay
                                                                      │
                                                        axes[i].target_pos_rev ──► leg ODrives
                                                                      │
                                              telemetry.cpp::send_leg_cmd() 100 Hz
                                              ──UDP LEG_CMD (0x88)──► Jetson  ← NOT SUBSCRIBED TODAY
```

The anchor path, Teensy to Jetson and back:

```
time_sync_master::send_tod_query()  t1 = micros64()
        │  RPC_REQUEST (forward delay df)
        ▼
client.py::_drain_socket  →  rpc.py::RpcServer._on_request  →  tod_server.py::_handle
   kernel RX at t2                                  userspace stamp at ≈ t3 = time.time()
        │  RPC_RESPONSE (return delay dr)
        ▼
on_tod_response():  rtt = micros64() − t1 ;  set_wall_anchor(jetson_wall_us + rtt/2)
```

Verified state of the telemetry surface (2026-08-11):

- `MsgType.LEG_CMD = 136 (0x88)` exists on the wire and is emitted at the
  telemetry rate (`telemetry.cpp::send_leg_cmd`, called from `task_telem`);
  payload is `t_teensy_us` (wall by contract) + `cmd_pos_rev[6]` +
  `cmd_vel_rps[6]`, 56 bytes. The Jetson-side decoder **already exists**:
  `LegCmd` and `LEG_CMD_SIZE` are re-exported by `teensy_link/protocol.py` and
  `teensy_link/__init__.py`. `teensy_bridge_node.py` does **not** subscribe it —
  its subscribe table covers HEARTBEAT_T2J, TELEMETRY, DIAGNOSTIC, PROFILE,
  CONE_FRAME, BB_AXIS_ESTIMATES, CMD_RESULT, PLATFORM_FRAME, HAND_CMD_ECHO,
  HAND_SENSOR, CAN_ERRORS, BRIDGE_TX_DIAG, BRIDGE_IDENTITY, and nothing else.
- `/profile` **is published** (a `DiagnosticStatus` carrying `udp_rtt_us`,
  `udp_jitter_us`, `interp_deadline_misses`, `interp_max_jitter_us`, CAN
  counters, heap) and **is not in the rosbag record list** — confirmed against
  the launch file's list, which does carry `/link_status`. This is exactly the
  gap the 2026-07-18 entry flagged when it told the operator to "watch it live".
- Neither `s_recover_slewing` occupancy nor Mode-2 extrapolation occupancy is
  uplinked at all (`leg_interp.cpp`).
- The next free STREAM `MsgType` is **143 (0x8F)**; 144 is `RPC_RESPONSE`.

## Implementation phase summary

| Phase | Content | Touches firmware? | Flashed? | Gate |
|---|---|---|---|---|
| **P0** | Jetson-side telemetry: subscribe LEG_CMD → ROS topic; bag that topic + `/profile` | no | n/a (`colcon build` + relaunch only) | full suite; does **not** reset bridge uptime |
| **P1** | FW 11: additive `CLOCK_DIAG` uplink + recover-slew/extrapolation occupancy counters; Jetson decode in the same commit | yes | **NO — held until after S1** | full suite; wire-invisible to an unaware Jetson |
| **P2** | Midpoint-stamped TOD responder (kernel RX + pre-send userspace, `(t2+t3)/2`) | no | n/a | full suite; Py 3.8-safe with graceful fallback |
| **S1** | Aged-bridge sitting: passive capture, then the four-arm isolation experiment | no | no | operator; `--full` gate before the sitting |
| **P3** | Latency root-cause fix + the alarmed end-to-end command-latency monitor | scope known only after S1 | per S1 outcome | lag ≤ 20 ms sustained at high uptime |
| **P4** | FW 12 clock servo (= clock plan Phases 3–5) | yes | yes | clock plan acceptance; **must follow P3** |

## Implementation phases

### P0 — Jetson-side telemetry (no flash, no reboot, no aging reset)

The whole point of P0 is that it can land on a *degraded* bridge without
destroying the degradation: a `colcon build` plus a ROS relaunch does not
touch the Teensy's power-on state.

- **P0.1 Subscribe LEG_CMD and publish it.** Add
  `self._client.subscribe(int(MsgType.LEG_CMD), self._on_leg_cmd)` to the
  subscribe table in `teensy_bridge_node.py` and decode with the existing
  `LegCmd.unpack`. Publish on the executor thread, not the RX thread, following
  the `bb/axis_estimates` precedent already in that file (queue in the RX
  callback, drain in a timer). The natural message is `JointState` — `name` the
  six legs, `position` = `cmd_pos_rev`, `velocity` = `cmd_vel_rps`, stamped from
  `t_teensy_us` — again matching `bb/axis_estimates`, which solved the identical
  problem for the BB axes.
- **P0.2 Bag it.** Add the new topic **and `/profile`** to the rosbag record
  list in `jugglebot_launch.py`. That list is THE ONE LIST (toss-selftuning
  D18) — adding rather than trimming is its standing rule, on the grounds that
  a missing topic is unrecoverable after the fact while a recorded quiet topic
  costs nothing. `/profile` is 1 Hz and the LEG_CMD echo is 100 Hz of twelve
  floats; both are within the list's existing cost envelope.
- **P0.3 Keep P0 minimal.** No duplication of `/profile` fields into
  `/link_status`. Two publishers carrying the same field is precisely the drift
  hazard the record has paid for before; `/profile` already carries
  `udp_rtt_us` and `udp_jitter_us` and only needed to be *recorded*.

Why this is the first phase and not a by-product of the sitting: without it,
S1 produces a bag that once again cannot distinguish transport delay from
interp behaviour from ODrive behaviour, and the sitting's multi-hour soak cost
would have to be paid twice.

### P1 — FW 11: additive instrumentation, written and committed, **NOT flashed until after S1**

FW 11 follows the `BridgeTxDiag` (0x8D) / `BridgeIdentity` (0x8E) precedent
exactly: a **new** `MsgType` rather than appended fields on an existing frame,
because exact-size prefix unpacks make an appended field a per-frame decode
error on an unaware Jetson — the frame goes dark instead of degrading. A new
message type is ignored cleanly. `PROTOCOL_VERSION` stays **5**, so an FW 10
board and an FW 11 board remain wire-compatible and a healthy link is not
evidence the new firmware is aboard (the FW 10 lesson, recorded in
`canbridge_config.h`'s FW_VERSION changelog at line 44).

- **P1.1 `CLOCK_DIAG` (new `MsgType`, next free = 143 / 0x8F).** Per-anchor
  clock diagnostics: `rtt`, the offset error *before* the slew is applied, and
  the implied instantaneous frequency in ppm. This is the raw material the
  clock plan's Phase 1 calls for — the actual crystal ppm, its thermal
  coefficient, and the RTT-jitter floor — which is what sets `Kp`/`Ki` and
  proves the frequency term is worth having before the servo is touched.
- **P1.2 Occupancy counters.** Recover-slew occupancy (`s_recover_slewing`
  duty) and extrapolation-mode occupancy (the Mode-2 cubic-Taylor branch of
  `interp_isr`), carried on the same frame. These are the 2026-07-18 entry's
  other two named telemetry gaps, and they are the discriminator that says
  whether "the response shape also slows" is the interp ladder engaging its
  fallback modes more often.
- **P1.3 Console-only where a wire field is not warranted.** The `[handphase]`
  diagnostic shipped with FW 10 is the precedent: a bench-only question gets a
  serial-console line rather than a wire field.
- **P1.4 Jetson-side decode lands in the same commit** as the firmware, again
  by precedent: 0x8D/0x8E shipped their host decode with FW 9 software while
  the firmware itself stayed unflashed, so the host was ready the moment the
  board was. Authoring point for the frame is
  `config/generate_udp_protocol.py`; the generated headers are never
  hand-edited.
- **P1.5 The flash is held.** Per owner decision 3, FW 11 is *not* flashed
  until after S1, because a flash reboots the Teensy and destroys the aged
  state the experiment exists to interrogate. The reboot after S1 (Arm C) is
  free, and that is the flash window.

### P2 — Jetson kernel RX timestamping in the TOD responder

**This section supersedes the kernel-RX-stamping sketch in
`bridge-clock-frequency-discipline.md` (§ "The PREEMPT-RT question", restated
in § Proposed design 2 / Phase 2).** That plan proposes stamping the
TOD reply "from the *kernel's* packet-receive time instead of a userspace
`clock_gettime` after scheduling", on the reasoning that this "deletes the
userspace scheduling jitter from the Jetson-side stamp". The derivation below
shows the jitter term does not vanish under that change — it changes sign — and
gives the fix that actually cancels it.

Let the query leave the Teensy at `t1`, arrive at the Jetson kernel at `t2`, be
stamped in userspace at `ts`, be sent back at `t3`, and arrive at the Teensy at
`t4`. Write `df = t2 − t1` (forward), `p = t3 − t2` (server processing), and
`dr = t4 − t3` (return). The firmware anchors to `stamp + rtt/2`, so with
`midpoint = (t2 + t3)/2`:

```
anchor error = (df − dr)/2 + (ts − midpoint)
```

- **Stamping at userspace processing time** (`ts ≈ t3`, today's behaviour in
  `tod_server.py::_handle` via `time.time()`): error = `(df − dr)/2 + p/2`.
- **Stamping at kernel RX alone** (`ts = t2`, the clock plan's sketch):
  error = `(df − dr)/2 − p/2`.

The scheduling-jitter term `p` does **not** disappear; it flips sign. On a
non-RT kernel `p` is exactly the quantity that varies run to run, so either
choice leaves a jittering anchor.

**The correct Jetson-only fix:** capture **both** the kernel RX timestamp `t2`
(via `SO_TIMESTAMPNS` ancillary data on the RPC socket) **and** a
just-before-send userspace stamp `t3`, and return `(t2 + t3)/2` in the
**existing** `jetson_wall_us` field. Then `ts = midpoint` exactly, `p` cancels,
and the residual error is pure path asymmetry `(df − dr)/2`. This is
**wire-compatible** — same field, same `ResultTimeOfDay` layout, no firmware
change, no `PROTOCOL_VERSION` bump — which is what makes it landable ahead of
S1 alongside P0. The remaining `(df − dr)/2` term is what min-RTT anchor gating
in P4 is for: the least-queued round-trip is the most symmetric one.

Implementation constraints, both verified on this box:

- **`teensy_link/client.py::_drain_socket` currently uses `sock.recvfrom`**,
  which cannot return ancillary data. The RPC socket's drain must move to
  `recvmsg` with a control buffer, and the resulting `t2` must be plumbed to
  the RPC server handler (`rpc.py::RpcServer._on_request` →
  `tod_server.py::_handle`). The stream socket does not need this and should
  not pay for it.
- **Python 3.8 compatibility is required** (`ros_ws/` runs the system
  interpreter; measured 2026-08-11: `python3 --version` → 3.8.10, and
  `socket.SO_TIMESTAMPNS` / `socket.SCM_TIMESTAMPNS` are **absent** from that
  build's `socket` module, and from the 3.9 on this box — do not assume any
  interpreter names them). Use the
  numeric constant with a **runtime probe** (attempt the `setsockopt`, confirm
  a timestamp actually arrives on a real packet) and a **graceful fallback to
  today's behaviour** if unavailable. On this kernel's headers
  `SO_TIMESTAMPNS` resolves to `SO_TIMESTAMPNS_OLD = 35` for a 64-bit
  `time_t` (`/usr/include/asm-generic/socket.h`), with the ancillary payload a
  `struct timespec`; the probe is what keeps that from being an unchecked
  assumption baked into the code.
- **Fallback must be observable.** A silent fallback to userspace stamping
  reintroduces `+p/2` invisibly, which is the same "invisible until degraded"
  class this whole arc exists to close. Log the mode once at startup and expose
  it where a bag can see it.

### S1 — the aged-bridge sitting (operator)

*This arc's S1. `plans/active/mvp-trajectory-bringup.md` has its own S1–S8
sitting ladder; there is no relation between the two labels.*

**Preconditions (all before the aging window starts):** P0 and P2 landed,
`colcon build` done, ROS relaunched, so the capture instrumentation is live for
the whole soak. FW 11 **not** flashed. `./run_tests.sh --full` green before the
sitting, per the standing pre-hardware rule.

**Standing rule suspended for this sitting only:** the bridge is *not* rebooted
beforehand. It is deliberately aged to **16–24 h** of continuous power-on
uptime. The aged state is the subject of the experiment.

**During the soak:** passive baseline capture — ordinary bags, which now
include `/profile` and the LEG_CMD echo. The RTT-vs-uptime curve assembled from
`/profile` is the transport discriminator described in § coupling.

**At the end of the soak, the pre-registered experiment, run exactly as
registered in the 2026-07-18 entry.** Nothing needs rebuilding — this is
hardware/driver state. Record a bag per arm; each arm uses the SAME bare
battery (shaped, defer-lean); add one `--lean-gain 0.0` run on whichever arm
fixes it (the previously-latching configuration is the sharpest before/after).
**Note `uptime_ms` (from `/link_status`) before every arm** so the lag-vs-uptime
curve gains labelled points. Run the arms in this order — each is cheaper and
less state-destroying than the next, and **the ORDER is what makes the
attribution unambiguous**:

- **Arm 0 — degraded reference.** One battery run before touching anything
  (confirms the degradation persisted; expected lag ~240 ms class).
- **Arm A — ODrive-only reboot** (Teensy + link untouched):
  `ros2 service call /reboot_odrives std_srvs/srv/Trigger "{}"` → re-home /
  re-activate per the arming contract → battery. Fixes it ⇒ ODrive-side
  accumulator.
- **Arm B — link bounce only** (Teensy + ODrives untouched): bounce the
  Jetson↔Teensy Ethernet link driver state with
  `sudo ip link set <iface> down && sleep 2 && sudo ip link set <iface> up`
  — per the standing rule, `ip link`, **never** physically unplug that dongle
  (kernel-hang hazard) → re-activate as needed → battery. Fixes it ⇒
  **Jetson-side transport (USB-Ethernet driver/queue) confirmed**, Teensy fully
  exonerated.
- **Arm C — Teensy reboot** (re-flash same firmware or power-interrupt its 5 V;
  ODrives left powered) → re-home / re-activate per the arming contract (expect
  the known benign `is_homed` boot-read transient) → battery. **A Teensy reboot
  ALSO bounces the PHY and thus the Jetson-side link state** — that is WHY Arm B
  must run first; Arm C fixing it *after* Arm B did not ⇒ genuinely
  Teensy-internal state.
- **PASS (any arm):** lag ≤ 20 ms; unshaped x-traverse deviation ≈ 0.1–0.2 rev
  (vs 1.02–1.08 latching); shaped battery deviations ≈ 0.1 rev class.
- **FAIL (no arm fixes it):** the uptime correlation is confounded by something
  else reset at the 2026-07-16 17:36 flash; reopen with the Addendum's candidate
  list and the LEG_CMD three-way discriminator — which, after P0, is available
  offline from a single degraded bag rather than requiring another soak.

**New in this run relative to the pre-registration:** the LEG_CMD echo and
`/profile` are *bagged*, not merely watched live, so each arm's attribution
(transport vs interp vs ODrive) is reconstructable after the fact. The
2026-07-18 entry expected this to be a live-watch; P0 upgrades it.

**The FW 11 flash window opens here.** Arm C already reboots the Teensy, so
flashing FW 11 as (or immediately after) Arm C costs nothing that the
experiment has not already spent.

### P3 — the latency fix + the alarmed end-to-end latency monitor

**Scope is deliberately unspecified until S1 localizes the drift.** The fix may
be system-level (USB-Ethernet driver, queue discipline, interface
configuration) rather than firmware, and writing a firmware-shaped remedy into
this plan before the experiment would prejudge exactly the question S1 exists
to answer.

What is *not* conditional is the second deliverable. Per the 2026-07-24
Addendum, closing `2026-07-18-teensy-uptime-tracking-degradation.md` requires a
**continuously-measured, alarmed end-to-end command-latency monitor, logged
with `uptime_ms`**. Its inputs are the P0/P1 telemetry:

- **Latency estimate**: the LEG_CMD echo (`t_teensy_us` + the executed command)
  against the Jetson-side emit time of the corresponding setpoint, i.e. the
  Teensy's own report of what it is executing versus when the Jetson sent it.
- **Transport trend**: `udp_rtt_us` / `udp_jitter_us` from `/profile`, now
  bagged, now with the aged-bridge curve from S1 as its reference shape.
- **Tagging**: every sample carries `uptime_ms`, so a growth trend is
  attributable rather than merely visible.
- **Alarm**: a threshold on the measured latency that fires *during* a session,
  not in post-hoc analysis. The class being closed is "invisible until a
  session is already degraded"; a monitor that only reports after the fact does
  not close it.

The pre-fix baseline for the monitor's threshold comes from S1's Arm 0 and the
post-fix sessions, not from a guess.

### P4 — FW 12: the clock servo (= clock plan Phases 3–5)

`bridge-clock-frequency-discipline.md` remains authoritative for the design.
This arc contributes only the sequencing and the ordering constraint:

- Phase 3 → frequency term in `set_wall_anchor` / `now_wall_us` (type-1 → type-2
  disciplined oscillator; PI servo on the phase error; keep step-on-large-error;
  the atomic-read discipline for ISR access).
- Phase 4 → min-RTT anchor gating in `on_tod_response`, which is also the
  clock's standing defence against future transport asymmetry (§ coupling,
  consequence 3).
- Phase 5 → holdover policy retune and the quality flag.

**P4 MUST come after P3.** A frequency estimator trained through a drifting,
possibly asymmetric transport bakes the bias into `freq_ppb`, where it is
indistinguishable from a genuine crystal rate error and persists through
holdover. Bench-validated motors-off against that plan's acceptance criteria
(< 10 µs RMS at stable temperature; < ~50 µs across a realistic thermal swing;
holdover < a few µs/min).

## Testing plan

- **P0** — unit coverage for the LEG_CMD decode-and-publish path alongside the
  existing bridge-node RX callback tests; a launch-file assertion that both new
  topics are in the record list. Ephemeral ports and `tmp_path` only, per the
  parallel-by-default rule.
- **P1** — codec round-trip for the new `CLOCK_DIAG` frame (the
  `test_protocol_codec.py` pattern), plus the standing wire-compatibility
  property: an unaware decoder ignores an unknown `MsgType` rather than
  erroring, and `PROTOCOL_VERSION` is unchanged.
- **P2** — a test that pins the **midpoint** property directly: with an
  injected clock, the value returned in `jetson_wall_us` equals `(t2 + t3)/2`
  and not `t3`; a second test that the fallback path is taken and *reported*
  when the `setsockopt` probe fails. `TimeOfDayServer` already accepts an
  injected `clock_fn`, which is the seam these tests use. Run under the system
  interpreter's constraints (no named socket timestamp constants).
- **S1** — `./run_tests.sh --full` before the sitting (standing rule (a) for
  any hardware sitting). The sitting's own PASS/FAIL criteria are the
  pre-registered ones above; they are not re-derived.
- **P3** — regression tests for the monitor's threshold logic and its
  `uptime_ms` tagging; the offline replay of an S1 bag as the fixture, per the
  production-faithful-replay convention.
- **P4** — the clock plan's testing plan verbatim (servo unit tests, multi-hour
  integration capture motors-off, thermal sweep, end-to-end re-run). Not
  duplicated here.

## Acceptance / closure criteria for the arc

1. **The four-arm experiment localizes the drift** — one arm passes, and the
   arm order makes the attribution unambiguous.
2. **The fix lands and holds at high uptime**: lag ≤ 20 ms sustained, re-measured
   on a **deliberately aged** bridge. The target state is "the Teensy stays alive
   indefinitely with zero drift" — a fix validated only on a fresh boot has
   validated nothing, since a fresh boot was always healthy.
3. **The alarmed end-to-end command-latency monitor is live and logged with
   `uptime_ms`.** Both this and (2) are required to close
   `logbook/2026-07-18-teensy-uptime-tracking-degradation.md` — the 2026-07-24
   Addendum's contract is two deliverables, not one.
4. **Clock**: `bridge-clock-frequency-discipline.md`'s acceptance criteria are
   met; that plan is then archived via `/archive-plan`.
5. **Downstream unblocks, named**: the accel-FF premise re-derivation
   (`plans/active/accel-ff-inertia.md`), the retime-ON revisit, and
   `plans/active/learned-ff-residuals.md` gate **G-A** (which reads, in that
   plan's own words, "the 2026-07-18 uptime-lag investigation is closed —
   reboot isolation experiment run, fix + continuous latency monitor landed").
   `plans/active/refactor-2026-07.md` § Phase 7's "bridge-uptime lag
   experiment" bullet is satisfied by S1.
6. **The reboot-before-every-session workaround is retired** — the standing
   memory rule exists only because the drift does.

## Notes for collaborators

- **The aged bridge is the experiment.** Rebooting the can-bridge Teensy before
  the S1 sitting destroys the subject. This is the one sitting where the
  standing reboot rule does not apply, and it applies again immediately
  afterwards until closure criterion 6 is met.
- **Do not flash FW 11 early.** A flash is a reboot. The FW 11 code being
  committed and unflashed is a deliberate state, and the FW 10 experience is the
  precedent: wire-invisible firmware means a healthy link is *not* evidence the
  new build is aboard, so the flash must be confirmed by `bridge_fw_version` on
  the `BRIDGE_IDENTITY` frame, never inferred.
- **Every timing number quoted anywhere gets `uptime_ms` beside it** until this
  arc closes. Numbers measured on a degraded plant are not wrong, they are
  *conditional*, and the record has already been re-litigated once for want of
  that label.
- **Physical-intuition pushback is load-bearing.** If the operator's sense of
  the machine disagrees with the framing above — particularly the claim that
  most of the delay accrues upstream of the Teensy — that disagreement is
  signal, not friction.
- **The localization is a ranking, not a verdict.** Arm A and Arm C outcomes
  stay live. Nothing in this plan should be read as having already concluded
  the transport is at fault.

## References

- `logbook/2026-07-18-teensy-uptime-tracking-degradation.md` — the seven-bag lag
  table, the forensic exclusion chain, the firmware-audit Addendum (NULL result
  and the ranked candidates), the pre-registered four-arm experiment, and the
  2026-07-24 closure-contract Addendum.
- `logbook/2026-07-28-anomaly-fixes-validation-sitting.md` — the dispatch-shift
  ABORT and its uptime correlation (0.24 h → +14.5 ms … 1.57–1.74 h →
  +54…+63 ms).
- `logbook/2026-08-10-sensor-truth-possession.md` — the ARRIVAL window sized
  around an uptime-dependent release lag (+118–133 ms at ~16 h).
- `logbook/2026-08-02-err-timeout-attribution-instrumentation.md` — the
  additive-MsgType discipline (0x8D / 0x8E) this arc's FW 11 reuses, and the
  ship-the-host-decode-with-the-unflashed-firmware precedent.
- `plans/active/bridge-clock-frequency-discipline.md` — authoritative design for
  the clock half; its Phase 1 → P1, Phase 2 → P2 (superseded sketch, see § P2),
  Phases 3–5 → P4.
- `plans/active/learned-ff-residuals.md` § Prerequisite gates — gate G-A, and
  its Phase-0 extractor whose Gate 0 is the 2026-07-18 lag table.
- `plans/active/refactor-2026-07.md` § Phase 7 — the absorbed bullet.
- `ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h` line 44 — the
  `FW_VERSION` changelog constant; FW 11 and FW 12 append their entries there.
