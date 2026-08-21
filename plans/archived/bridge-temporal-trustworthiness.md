---
title: Bridge Temporal Trustworthiness — closing the uptime command-latency drift and the clock-precision half in one arc
status: completed   # latency half COMPLETE and validated; the clock half (P4) hands back to bridge-clock-frequency-discipline (see Archival note)
completed: 2026-08-15
owner: harrison
created: 2026-08-11
last_updated: 2026-08-15
related_logbook:
  - 2026-07-18-teensy-uptime-tracking-degradation.md   # the LATENCY half — open; owns the four-arm experiment and the closure contract
  - 2026-08-12-s1-aged-bridge-isolation-teensy-internal.md  # S1 — the four-arm isolation; Teensy-internal CONFIRMED
  - 2026-08-14-ring-audit-available-leak-delay-line.md  # S2 endgame + the ring audit — the named defect, and what S3 must confirm
  - 2026-08-14-s3-conviction-ring-leak-measured.md     # S3 — the leak MEASURED at 97 % of a lap; the stale-install near-miss and its self-check
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
archived: 2026-08-15
---

# Bridge Temporal Trustworthiness

**Branch:** `mvp-trajectory-bringup`
**Absorbs:** `plans/parked/refactor-2026-07.md` § Phase 7's remaining bullet
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
| **S2** | FW 12 `CACHE_DIAG` confirmation soak (brief-launch protocol) + the offline forensics rounds | yes (FW 12) | yes — the flash IS t0 | decision rule in § S2; **answered NO**, see § S2 RESULTS |
| **S3** | FW 13 `RING_DIAG` conviction soak — motionless, ~3–4 h, no battery | yes (FW 13) | yes — the flash IS t0 | `true_depth − avail_reported` growing over the soak; **CONVICTED at 247–248 of 256**, see § S3 RESULTS |
| **P3** | Latency root-cause fix (FW 14; the Jetson honesty fix landed with the FW 13 change-set) + the alarmed end-to-end command-latency monitor | yes — scope known since the 2026-08-14 audit | after S3 convicts | lag ≤ 20 ms sustained at high uptime |
| **P4** | the clock-servo firmware (= clock plan Phases 3–5; FW number assigned at implementation — FW 12/13 were claimed by the latency instrumentation) | yes | yes | clock plan acceptance; **must follow P3** |

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

### S1 RESULTS (run 2026-08-12, bridge aged 62.9–63.1 h) — Teensy-internal CONFIRMED

Full record: `logbook/2026-08-12-s1-aged-bridge-isolation-teensy-internal.md`.
Verdict: **only Arm C (Teensy reboot) restored smooth motion** — 15.8 ms
end-to-end lag at 27 s uptime vs 290–340 ms at 63 h (the historical curve does
NOT saturate at ~240 ms). The decisive control is the post-power-cycle Arm B
bag (`14-52-00`): fresh ODrives + bounced link + aged Teensy → still 290 ms.
Exonerated with data: Jetson transport (RTT flat 1–3 ms — the coupling-insight
branch where RTT flatness clears the clock half is TAKEN), Teensy→ODrive TX
(zero deferral/queue in absolute terms over 63 h in bags 1–3; bag 4 opens
with a frozen 519/64 accrued during the ODrive power-cycle's bus-down window,
in-bag delta 0), RX wire-error loss (zero
in-battery deltas), heap, interp scheduling, ODrive-internal state, and the
stream-vs-RPC socket asymmetry (one 1 kHz drain loop, stream before RPC — a
stream backlog would inflate RTT, not hide from it). Surviving mechanism
signature: **per-axis encoder-cache refresh stalls** — median refresh a
constant 10 ms in every bag; the TAIL degrades (9–18 % of intervals > 30 ms
aged vs 4.3 % fresh, p95 80–130 ms, max ~500 ms), per-leg independent, while
the uplink cadence stays perfect — the value inside on-time messages freezes.
A 130 ms stale anchor at 2.5 rev/s hides 0.325 rev = 3× the lead-clamp budget,
and clamping starts at move ONSET (94 % within 100 ms; 30/49 before motion).
Caveats carried: Arm C was an FW 10→11 *flash*, not a bare reboot (clean
re-test = bare power-cycle after the next soak); "ODrive paused broadcasting
per-axis" is not fully closed until FW 12's per-axis frame counters run; and
**P0's `/leg_cmd_executed` saturates as a transport/execution discriminator
exactly when the clamp pins** (post-clamp command ≡ fb + 0.100) — the P3
monitor must watch clamp duty + cache age, not only lag.

### S2 — FW 12 confirmation soak (brief-launch protocol)

FW 12 (`CACHE_DIAG` 0x91, instrumentation only): per-axis cache-age
min/max per 1 Hz window, per-axis encoder-frame counters (splits "ODrive went
silent" from "cache stopped updating"), the never-uplinked `depth_hwm` /
`cap_hits` RX-ring counters, and decode-discard counters. Protocol — the
accumulator lives in TEENSY power-on time, not ROS uptime (the whole
historical curve was measured with ROS down between sittings), so: flash
FW 12 (t=0), keep Teensy + ODrives powered and idle, ROS DOWN between
samples; every 2–4 h a ~3-minute `record:=true` launch (no homing, no arming,
no motion — kilobyte-scale bags); after 16–24 h one battery run on the aged
plant (lag + cache age simultaneously), then a **bare power-cycle** (no flash
— also the Arm C flash-confound re-test) and one fresh battery. Decision
rule: cache-age tail grows with uptime ⇒ mechanism confirmed, fix targets the
cache path; flat cache-age under a still-lagging aged battery ⇒ the stall is
below the cache sampling point, reopen with the per-axis frame counters and
`depth_hwm`/`cap_hits` as the discriminators.

### S2 RESULTS + ring audit (2026-08-13/14) — the decision rule answered NO, and the root cause is now a named defect

Full record: `logbook/2026-08-14-ring-audit-available-leak-delay-line.md`.

**The § S2 decision rule took its second branch.** Cache age does **not** grow
with uptime: p95 **11.05 ms at 27.96 h**, *below* the 1.05–1.30 h soak bags
(18.05–18.07 ms) and indistinguishable from a 9.4 s fresh boot — while the same
aged bag carries **282.9 ms** e2e lag against **15.9 ms** fresh (echo→exec
153.4 / 5.85 ms; clamp duty 0.298 / 0.0007). `enc_frames` reads exactly
**100.0 fps/axis everywhere**. The first aged `/clock_diag` shows recover-slew
and extrapolation occupancy ≈ 0 and `interp` at exactly **500.0 Hz** at 28 h.
**FW 12's own instrument refuted the S1 cache-staleness mechanism**, and the
identical-firmware reflash that cleared the degradation also closes S1's Arm C
flash-vs-reboot confound.

**The convicted defect (audit 2026-08-14, verified in compiled assembly with the
project toolchain, not argued from source):** `FlexCAN_T4::events()` pops the RX
ring **before** its `NVIC_DISABLE_IRQ` guard, so the consumer's non-atomic
`_available--`/`head` RMWs race the CAN ISR's `_available++`
**one-directionally** — increments are swallowed, never decrements. `_available`
monotonically **under-counts**, the drain-to-empty loop exits with true occupancy
`D > 0`, and every delivery is `D` frames late. `D` **ratchets** (~1 × 10⁻⁵
collisions/pop × ~1.6 × 10⁸ pushes/day ≈ 90 slots/h ≈ **40 ms/h**, matching the
July curve's early ramp) and **caps at one ring lap — 256 slots ≈ 114–135 ms**.
The mod-512 `head ^ 256` full test bounds the leak at one lap, which **proves**
stale-lap re-reads, duplicates and replays impossible: **the RX ring is a delay
line — frames arrive exactly once, in order, late.**

**Why every counter read healthy — blind by construction, and the mechanism
predicts it:** `depth_hwm`/`cap_hits` derive from `_available` itself (the
corrupted counter reporting on itself); per-axis cache age is stamped **at
decode**, downstream of the delay, so `CACHE_DIAG` could never have seen this;
`enc_frames` counts **deliveries**, which a pure delay conserves. Only the lead
clamp and `MAX_DEVIATION` read the delayed *content* — and only they showed the
fault. That is exactly the S2 asymmetry.

**A Jetson-side measurement artifact, and it is large.** ~**97 %** of the S2
"bit-identical per-axis freezes" are manufactured on the host: the aged uplink
arrives in ~20 ms **pairs** (31 % paired drain ticks aged vs 2.6 % fresh — 12×,
uptime-dependent) hitting `/robot_state`'s **latest-wins latch** behind a **100 Hz
ROS-clock timer with no staleness gate**, so a pair-starved tick republishes the
latch verbatim — bit-identical, fresh-stamped and inherently **multi-axis**,
which is the 193× co-freeze the freeze-structure round measured. S1's "arrival
cadence is clean" exoneration was measuring that same ROS timer, not the data
(method correction #4 of the arc; the transport exoneration itself stands on
`udp_rtt_us`). The residual ~19 long runs (95–739 ms) are a **physically stalled
plant** — clamp-commanded stop plus the S1 binding signature (leg 0 at 20.53 A) —
a consequence, not a cause.

**Secondary audit finds** (all read-only, none fixed here): the ring-full pop's
`memmove` can **tear** a frame (id from one message, payload from another, past
both decode guards); FIFO overflow/warning `IFLAG`s are cleared **uncounted**;
`writeIFLAGBit` is a W1C RMW footgun (currently harmless); the vendored library
carries **no NXP errata workarounds**; `isEventsUsed` can never flip back. The
drain comment above `CAN_RX_DRAIN_BUDGET`'s claims are **refuted** — the ±1 miscount
is a **monotone** leak, and its supporting evidence was circular (`depth_hwm`
derives from `_available`).

**Honest residual, recorded rather than smoothed:** the measured aged lag
(**283–340 ms**) **exceeds** the delay line's proven **135 ms** ceiling. The
working explanation is clamp-drag amplification on top of the raw delay — **an
argument, not a measurement**. FW 13's wrap-aware delivery lag settles it.

**Closure path:** FW 13 (instrumentation + the Jetson honesty fix, both
landed) → S3 conviction soak → FW 14 (fix) → the 2026-07-24 two-deliverable
contract.

### S3 — FW 13 conviction soak (motionless, ~3–4 h)

**FW 13 — `RING_DIAG` (`MsgType` 0x92), instrumentation only**, landing on the
**vendored** FlexCAN_T4 (commit `fef2df5`, byte-identical copy of
framework-arduinoteensy 1.159.0; policy and the two-defect justification in
`lib/FlexCAN_T4/PROVENANCE.md` — the `events()` TX-deferral missing `break` and
this leak, both confirmed, both unfixable without a local copy). Contents:

- **True ring occupancy**, walked from `head`/`tail` **independently of
  `_available`** — `true_depth − avail_reported` **IS the leak**, read directly.
- **FIFO overflow/warning counters** (secondary find 2 — a real RX overflow is
  currently unobservable).
- **Wrap-aware delivery lag** — how late a frame actually is, which is what the
  283 ms-vs-135 ms residual needs.
- **An SDO-RTT causal probe** through the same ring — an end-to-end delay figure
  no consumer-side stamp can fake.
- **A `/robot_state` staleness gate** on the Jetson (the honesty fix), so the
  next round's freeze statistics describe the robot rather than the timer.

**Protocol.** Flash FW 13 = **t0**. Keep the Teensy and ODrives **powered and
idle**, ROS down between samples; one brief `record:=true` launch at the start
and one at the end (a few in between are fine). **No motion, no arming, no
battery** — the ring fills on bus traffic alone, so this costs hours, not a
sitting.

**Conviction criterion:** `true_depth` climbing toward 256 while
`avail_reported` stays low — i.e. **`true_depth − avail_reported` growing** over
the soak. Predicted rate ≈ 90 slots/h. **Refutation:** a flat difference over
3–4 h kills the leak hypothesis outright, and the arc reopens with the wrap-aware
delivery lag and the SDO-RTT probe as the surviving discriminators.

**Then:** **FW 14** — correct the pop's bookkeeping (guard the ring pop inside
the existing `NVIC_DISABLE_IRQ` window, or make the `_available` updates atomic)
— then a **validation soak on a deliberately aged bridge** (acceptance
criterion 2: lag ≤ 20 ms sustained at high uptime), and then the alarmed
monitor of P3. (The Jetson honesty fix — the `/robot_state` staleness gate —
landed with the FW 13 change-set, not FW 14.) Only both deliverables together close
`logbook/2026-07-18-teensy-uptime-tracking-degradation.md`.

### S3 RESULTS — CONVICTED (2026-08-14)

Full record: `logbook/2026-08-14-s3-conviction-ring-leak-measured.md`.

**The § S3 conviction criterion is met, at the ceiling.** On a bridge aged
**4.01–4.04 h** (bag `2026-08-14_18-18-59`, **92 `/ring_diag` samples**):

| Quantity | Value |
|---|---|
| `leak_jb` = `true_depth_jb − avail_reported_jb` | **247–248** (`true_depth_jb` 247–248 against `avail_reported_jb` **0**) |
| `leak_hwm_jb` | **249** — **≈ 97 % of the 256-slot lap** |
| `leak_bb` / `leak_cone` | **1** / **0** — the collision-rate traffic scaling, confirmed |
| `fifo_overflows`, all buses | **0** — **no peripheral loss; pure software-ring stranding** |
| `probe_ticks` | **1000** per window |
| Delivery-lag integral | **151–183 ms** (see residual (a)) |
| `robot_state_stale_skips` | **29** — the Jetson honesty gate is live |

**Two independent cross-checks agree.** echo→exec **aged − fresh = 115.4 ms**
falls inside the predicted **114–135 ms** one-lap band. And the saturation
arithmetic — 256 slots ÷ ~90 slots/h ≈ **2.84 h** — retro-explains the historical
plateau: 3.8 h → 252 ms, ≈ 28 h → 283 ms, ≈ 63 h → 290–340 ms is a lag that
**saturated by hour three**, not one still climbing.

**Root cause of the arc is ESTABLISHED: one missing IRQ guard around the vendored
FlexCAN_T4 ring pop.**

**The bracketing sessions** (same day, FW 13): **17 s** uptime — operator
"silky", e2e **19.9 ms**, clamp duty **0.0000**; **3.80 h** — "janky", e2e
**252.2 ms**, duty **0.4588**. Neither carries `/ring_diag` samples (see the
incident below) and neither is cited for anything else.

**THE INSTALL-SKEW INCIDENT — the conviction was nearly lost.** The first pass'
bags recorded **no `/ring_diag` at all**: the ROS node ran a **stale `install/`
tree dated 2026-08-12** with zero `ring_diag` references, while
`BRIDGE_FW_CHECK` printed **OK** throughout. The check was not wrong — it is
**structurally blind to install skew**, comparing the board's `FW_VERSION` (13)
against `EXPECTED_BRIDGE_FW_VERSION` (13) read from the **live repo-root
`teensy_link/` tree**, which the launch injects on PYTHONPATH and colcon never
installs. Firmware currency and host currency are independent halves, and only
one had a detector. `jugglebot_launch._install_drift` covers the generated
**config** modules only, so a stale node source passes it green.

Two consequences, both landed:

- **Recovery cost nothing but a rebuild.** `colcon build` + relaunch **does not
  reboot the Teensy**, so the aged state survived and the soak did not have to
  be re-aged — the conviction bag was taken the same afternoon at 4.01–4.04 h.
  Record this: a host-side rebuild is always available mid-soak.
- **An install-skew self-check now ships in `teensy_bridge_node`**: at
  construction it hashes its own `__file__` against the repo-source copy
  (walk-up resolution, `JUGGLEBOT_REPO` override), renders **three verdicts**
  (`0` clean / `1` stale / `unknown` could not run), logs loudly on skew with
  both paths and mtimes, publishes `install_skew` + `install_skew_detail` on
  `/link_status` beside `bridge_fw_version` so a **bag can never again silently
  record a stale node**, and appends the host verdict to the `BRIDGE_FW_CHECK`
  line. **Advisory, never a gate** — same policy as `BRIDGE_FW_CHECK`.

**Two second-order residuals, recorded rather than reconciled here.** Both are
instrument limitations, and both are FW 14 validation-pass obligations:

- **(a) The delivery-lag integral's absolute value.** **151–183 ms**, creeping
  **~0.35 ms/s** across the bag, against a naive **129 ms** (leak ÷ 1,920 fps).
  Known contributors: a boot-time offset plus **16 reseeds** in the window. **The
  trend is the instrument, not the absolute** — and the creep is unexplained.
- **(b) The SDO RTT floor reads BELOW the ring delay** (**46.9–47.9 ms**
  against ~130 ms). Cause: **single-slot request-stamp mispairing under
  pipelining** — at a 20 ms poll period and a ~130 ms delay there are **~6
  replies in flight**, so the probe's floor argument breaks. This *only* happens
  because delay ≫ poll period, so it is weak confirmation of the mechanism, not
  a contradiction.

**FW 14 acceptance criteria** (all three, on an AGED validation soak — the leak
counter **stays aboard**, it is how the fix is proven):

1. **`leak ≡ 0`** on every bus at high uptime;
2. **end-to-end lag ≤ 20 ms sustained** on the aged plant;
3. **residuals (a) and (b) reconciled** — the lag integral's absolute value and
   creep explained, and the RTT probe either fixed (multi-slot request stamps) or
   its limitation restated with the pipelining arithmetic.

### P3 — the latency fix + the alarmed end-to-end latency monitor

**Preamble updated 2026-08-14: the root cause is localized to a named defect.**
The scope this section deliberately left unspecified is now known — the fix is
**firmware** (the FlexCAN_T4 RX pop's bookkeeping, FW 14) plus the Jetson
`/robot_state` honesty fix, pending the S3 conviction measurement. The paragraph
below is retained as the record of why the scope was withheld until S1/S2
answered.

**Scope was deliberately unspecified until S1 localized the drift.** The fix may
be system-level (USB-Ethernet driver, queue discipline, interface
configuration) rather than firmware, and writing a firmware-shaped remedy into
this plan before the experiment would prejudge exactly the question S1 exists
to answer.

**The monitor's alarm input is now calibrated rather than guessed** (S1 method
correction (c) — a lag-only monitor reads healthiest exactly when the clamp
pins): **clamp duty**, plus a **T = 100 ms / 0.67 rev/s** content-hold reporting
threshold measured at **0.28/min healthy vs 4.75/min aged (17×)**. The D3 gate
that produced it also found **no viable protective threshold** — see the
supersession banner on `plans/archived/lead-clamp-content-freshness.md`.

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

### P4 — the clock-servo firmware (= clock plan Phases 3–5)

*(Originally drafted as "FW 12"; that number was claimed by the CACHE_DIAG
instrumentation and FW 13 by the ring-conviction instrument. The clock servo
takes whatever FW number is current when P4 starts.)*

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
   (`plans/parked/accel-ff-inertia.md`), the retime-ON revisit, and
   `plans/parked/learned-ff-residuals.md` gate **G-A** (which reads, in that
   plan's own words, "the 2026-07-18 uptime-lag investigation is closed —
   reboot isolation experiment run, fix + continuous latency monitor landed").
   `plans/parked/refactor-2026-07.md` § Phase 7's "bridge-uptime lag
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
- `plans/parked/learned-ff-residuals.md` § Prerequisite gates — gate G-A, and
  its Phase-0 extractor whose Gate 0 is the 2026-07-18 lag table.
- `plans/parked/refactor-2026-07.md` § Phase 7 — the absorbed bullet.
- `ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h` line 44 — the
  `FW_VERSION` changelog constant; FW 11 and FW 12 append their entries there.

---

## Archival note (2026-08-15)

**Archived COMPLETE on the latency half — the half this arc was created to close.
The clock half hands back to `plans/active/bridge-clock-frequency-discipline.md`,
which was always its authoritative design document and stays active and
independently schedulable.**

### What shipped

| Phase | Shipped | Evidence |
|---|---|---|
| **P0** | LEG_CMD echo subscribed and published; that topic **and** `/profile` added to THE ONE rosbag record list — the three-way transport/interp/ODrive discriminator became available offline from a single bag | `logbook/2026-08-11-bridge-temporal-trustworthiness-kickoff.md` and the P0 entry |
| **P1** | **FW 11** — additive `CLOCK_DIAG` (0x8F) with per-anchor clock diagnostics plus recover-slew and extrapolation occupancy counters; written and committed **unflashed**, flashed at S1 Arm C exactly as the owner decision required | plan § P1 |
| **P2** | Midpoint-stamped TOD responder — kernel RX `t2` midpointed with a pre-send userspace `t3`, returned in the existing `jetson_wall_us` field, wire-compatible, with an observable fallback. **This superseded the clock plan's kernel-RX-only sketch**: kernel-RX stamping alone flips the sign of the server-processing term rather than deleting it | plan § P2 |
| **S1** | The pre-registered four-arm isolation on a bridge aged 62.9–63.1 h. **Only Arm C fixed it** ⇒ Teensy-internal. Jetson transport, ODrives, Teensy→ODrive TX, RX wire errors, heap and interp scheduling all exonerated **with in-bag data** | `logbook/2026-08-12-s1-aged-bridge-isolation-teensy-internal.md` |
| **S2** | **FW 12** `CACHE_DIAG` soak. Its decision rule took the **second** branch — cache age was *fresher* at 28 h than at 1.1 h — refuting S1's own cache-staleness mechanism with S1's own successor instrument | `logbook/2026-08-14-ring-audit-available-leak-delay-line.md` |
| — | The RX-path concurrency audit that named the defect, and the FlexCAN_T4 vendoring (`fef2df5`) that made fixing it possible at all | same entry; `lib/FlexCAN_T4/PROVENANCE.md` |
| **S3** | **FW 13** `RING_DIAG` (0x92) — true ring occupancy read independently of `_available` — plus the Jetson `/robot_state` staleness gate. Convicted at the ceiling: `leak_jb` **247–248** of a 256-slot lap at 4.03 h. Also produced the **install-skew self-check** after a stale `install/` tree nearly cost the measurement | `logbook/2026-08-14-s3-conviction-ring-leak-measured.md` |
| **P3** | **FW 14** — the ring pop moved inside the `NVIC_DISABLE_IRQ` window (plus the dormant TX-deferral `break`), and the **alarmed end-to-end latency monitor** with the lag-clock normaliser. Validated at **5.8 h and 15.2 h**: `leak ≡ 0`, lag 10–20 ms, clamp duty 0 | `logbook/2026-08-14-fw14-ring-leak-fix.md`, `logbook/2026-08-15-fw14-validated-arc-closed.md` |
| **P4** | **Not started — deliberately.** Its authoritative design never lived here; see § Hands off | — |

**All six acceptance criteria are met.** (1) The four-arm experiment localized the
drift unambiguously, and the arm *order* is what made it unambiguous. (2) The fix
holds at high uptime — 20 ms at 15.2 h against 20 ms fresh, on a deliberately aged
plant, which was the whole point of the criterion. (3) The alarmed monitor is live
and logged with `uptime_ms`. (4) The clock half's criteria are handed to its own
plan, unblocked. (5) The named downstream unblocks are recorded and open. (6) The
reboot-before-every-session workaround is **retired** across the runbooks.

### What was learned

- **The root cause was one missing IRQ guard** in a vendored library:
  `FlexCAN_T4::events()` popped the RX ring before `NVIC_DISABLE_IRQ`, so the
  consumer's non-atomic `_available--` raced the ISR's `_available++` **in one
  direction only**. The counter under-counted monotonically, the drain exited with
  the ring non-empty, and **the RX ring became an uptime-ratcheting delay line**
  (~40 ms/h, capped at one 256-slot lap ≈ 114–135 ms).
- **A counter derived from the corrupted quantity cannot audit it.**
  `depth_hwm`/`cap_hits` are computed from `_available`; they read healthy through
  a 97 %-stranded ring, and we shipped them believing they covered it.
- **Instrument where the consumer reads, not where the producer writes.** FW 12's
  cache age is stamped at decode, downstream of the delay, so it could never have
  seen this. A freshness stamp applied by the consumer measures the consumer.
- **The mechanism that stuck predicted every instrument's blindness** instead of
  explaining it away. That is the shape of a correct explanation, and it is the
  criterion worth carrying to the next hunt.
- **Measurement beat argument, repeatedly.** The 2026-07-18 full-file firmware
  audit — a genuine sweep of 18 .cpp + 20 .h — walked past this defect and returned
  a NULL result with a wrong top-ranked candidate. A compiled-assembly check and
  then one hardware number convicted it.
- **The operator's cutover prior was right and quantitatively so**: the leak rate
  goes as arrival × pop, i.e. ~quadratic in the load the MVP streaming cutover
  changed. A physical intuition about *when* a fault started, converted into a rate
  hypothesis, would have saved weeks.
- **Two second-order instrument artefacts were found and closed** rather than
  smoothed: the delivery-lag integral is contaminated by a **load-dependent**
  FlexCAN capture-clock rate error (≈230 ppm idle, ≈580–670 ppm streaming;
  normalised in software by continuous re-estimation, since one rate pooled across
  a load transition under-corrects by ~350 ppm — and *why* bus load moves that rate
  is still unexplained), and the SDO-RTT probe's single-slot stamp pairing is valid
  only while delay ≪ poll period. Even corrected, the lag integral is a
  **growth channel since the last reseed**, never an absolute lag — `leak_*` is the
  absolute-occupancy channel.

### What it hands off

- **P4, the clock servo → `plans/active/bridge-clock-frequency-discipline.md`
  Phases 3–5.** Its **P4-after-P3 ordering constraint is now SATISFIED**: a
  frequency estimator will no longer train through a drifting, possibly asymmetric
  transport. That plan's Phase 1 instrument already exists on the wire as
  `CLOCK_DIAG`, and its Phase 2 was delivered here as P2 (with the midpoint
  correction). Min-RTT anchor gating remains its standing defence against any
  *future* transport asymmetry, independent of this arc's outcome.
- **The frame-drop follow-on → `plans/active/leg-bus-frame-drops.md`.** The
  validation battery characterised per-axis encoder-frame drops that are gated by
  the 500 Hz setpoint stream, not by uptime, and that pre-date FW 14. They are a
  *different* input to the same lead-clamp amplifier, and — unlike the delay line —
  a genuine dropout really does age `pos_timestamp_us`, so a timestamp-age-aware
  clamp works there.
- **The superseded lead-clamp draft** (`lead-clamp-content-freshness.md`) is
  archived alongside this plan; its enforcement-point analysis, its
  `MAX_DEVIATION`/stroke-clamp interaction findings and its velocity-extrapolated
  anchor are salvaged into the frame-drop plan.
- **Downstream premises now re-derivable on a healthy plant**: the 0.5→1.0 rev
  guard raise, the 2.2–2.7 rev/s chase ceiling, the retime-model OFF decision, the
  accel-FF sizing premise (`plans/parked/accel-ff-inertia.md`), and
  `plans/parked/learned-ff-residuals.md` gate **G-A**, which is now CLEARED.
- **One residue that is code, not documentation**:
  `tests/hardware/toss_cal_grid.py`'s `R7` refusal (`UPTIME_ABORT_MS`, 30 min) and
  `tests/hardware/tilt_cal_grid.py`'s matching WARN still gate on bridge uptime.
  With the rule retired they will refuse or warn on a healthy warm bridge. Removing
  or re-keying them onto `latency_monitor` is a code change with its own test, and
  is deliberately **not** folded into the documentation closure.
