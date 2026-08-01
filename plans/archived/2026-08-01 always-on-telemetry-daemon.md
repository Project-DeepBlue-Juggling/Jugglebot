---
title: Always-On Telemetry Daemon (ROS2-independent GUI feed via firmware tap)
created: 2026-06-08
status: deferred   # design only; all six phases NOT STARTED (see Archival note)
completed: 2026-08-01
related_config:
  - config/generate_udp_protocol.py → CONSTANTS.PORT_TELEM_TAP
related_code:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_link.cpp::send_to
  - controller/teensy_link/__init__.py
  - tools/jugglebot_telemetry_daemon.py
---

# Always-On Telemetry Daemon (ROS2-independent GUI feed via firmware tap)

## Context

### Why this plan exists

The Jugglebot web GUI (`ros_ws/gui/`, served by `gui_server.py` on :8081) is a
static front end. All of its live data arrives over the rosbridge v2 protocol on
a websocket to :9090, which is fed by `rosbridge_server` → ROS2 topics →
`teensy_bridge_node` → the Teensy CAN Hub. Consequently, with ROS2 not running
the GUI loads but shows no live data.

The Teensy 4.1 CAN Hub is always connected and always reports feedback from
whichever robots are attached. Making that telemetry visible in the GUI **without
the full ROS2 stack running** is a standing quality-of-life goal: a headless
machine can show motor and Ball Butler state at a glance without launching the
control stack.

### What this plan achieves

A standalone, always-on daemon serves live Teensy telemetry to the existing GUI
with **zero GUI code changes**, by speaking the same rosbridge v2 JSON protocol
the GUI already uses. The daemon is **read-only and entirely outside the
control path**: it never transmits on the Teensy link and never commands motion.

The enabling mechanism is a **read-only firmware "telemetry tap"** — the Teensy
duplicates its outbound uplink frames to a second UDP port that the daemon owns
permanently and passively. ROS2's control link is untouched; the daemon and ROS2
never contend for the Teensy datastream.

### When to do this

After the Teensy CAN-offload firmware/bridge work on
`teensy-can-offload-firmware-three-bus-wip` has stabilised, since Phase 1 adds to
the firmware uplink path. The daemon phases (2–5) are pure-Python and ROS2-free
and can proceed in parallel once the tap port exists in the protocol config.

This plan is the deferred companion to the GUI boot service already landed (the
static GUI is served headless on :8081 with a hang-watchdog; the old :8080 node
http-server was retired). That work closed "GUI is always served"; this plan
closes "GUI always shows telemetry, even without ROS2."

## Architecture

### Verified firmware facts (2026-06-08)

These facts make the passive fan-out possible and were confirmed by reading the
firmware, not assumed:

- **Telemetry TX is unconditional.** `task_telem`
  (`Teensy_code_canbridge.ino:159-166`) calls `telemetry_step()` at
  `TELEM_RATE_HZ` (100 Hz) in an unconditional loop. `telemetry_step()`
  (`telemetry.cpp:95`) → `send_telemetry()` has **no** `link_state` / heartbeat /
  fault gate.
- **Destination is a hardcoded unicast peer.** `send_to()`
  (`udp_link.cpp:127-143`) transmits to the compile-time constant `kJetsonIP`
  (`udp_link.cpp:20`) with **no peer-learning**. The Teensy emits telemetry to
  the Jetson regardless of whether anything is listening.
- **Implication:** a passive Jetson-side listener receives the full uplink with
  **zero downlink** required. The daemon never needs to send a heartbeat, a
  time-of-day reply, or anything else to keep telemetry flowing.

The STREAM channel (`PORT_STREAM` = 5005) carries the read-only uplink frames
(`TELEMETRY`, `DIAGNOSTIC`, `HEARTBEAT_T2J`, `PROFILE`). The RPC channel
(`PORT_RPC` = 5006) and the inbound setpoint/heartbeat path are separate and are
**not** part of the tap.

### Current

```
Teensy ──unicast 5005──► teensy_bridge_node (ROS2) ──topics──► rosbridge :9090 ──ws/JSON──► GUI
       ◄─ setpoints / heartbeat-J2T / RPC ── teensy_bridge_node
```

Only one Jetson process can bind 5005 to receive. ROS2 down ⇒ no receiver ⇒ GUI
shows nothing live.

### Proposed

```
                            ┌─ unicast 5005 ─► teensy_bridge_node (ROS2, unchanged) ─► rosbridge :9090 ─┐
Teensy ── STREAM uplink ────┤                                                                            ├─► GUI :8081
  (tee, read-only)          └─ unicast 5008 ─► jugglebot_telemetry_daemon (passive) ─── rosbridge-JSON :9090 ┘
       ◄─ setpoints / heartbeat-J2T / RPC ── teensy_bridge_node   (control path: BYTE-FOR-BYTE UNCHANGED)
```

- The firmware tees every outbound STREAM frame to a second port (`PORT_TELEM_TAP`,
  e.g. 5008) in addition to the existing 5005 send.
- The daemon binds 5008 **only** (read-only), decodes frames with the existing
  pure-Python `controller.teensy_link` decoders, holds a latest-state model, and
  serves it to the GUI as rosbridge-shaped JSON on :9090 when ROS2/rosbridge is
  not holding that port.
- ROS2 keeps 5005/5006 exactly as today. The two receivers never contend (separate
  ports).
- The daemon opens **no** socket toward the Teensy. It cannot command motion.

### Why fan-out over the alternatives

Two other coexistence models were considered and rejected:

- **Fallback with handoff** (daemon active only when ROS2 is down; releases the
  Teensy link before ROS2 launches). Workable, but requires a handoff of a
  setpoint-bearing socket and gives the GUI two distinct telemetry code paths.
- **Full link-ownership proxy** (daemon permanently owns 5005/5006; ROS2 reads
  setpoints/telemetry through it). Rejected: UDP single-bind forces the daemon
  into the **downlink** (setpoint + heartbeat + RPC) path, which (a) adds a
  process hop and jitter to a real-time setpoint stream the Teensy interpolates,
  (b) promotes a convenience daemon to safety-critical (its hangs interrupt live
  control), (c) requires a fail-closed redesign of heartbeat liveness so the
  proxy cannot mask a dead control process, and (d) refactors the freshly-built
  bridge. See `## Notes for Collaborators → Architecture decisions`.

Fan-out keeps the daemon entirely out of the control path. Its cost is one small,
additive, read-only firmware change. Because telemetry TX is unconditional and
peer-fixed (verified above), the daemon is purely passive.

### Single-source-of-truth notes

- `PORT_TELEM_TAP` is added to the `CONSTANTS` list in
  `config/generate_udp_protocol.py` and regenerated by `python
  config/generate_udp_protocol.py` into `config/generated/udp_protocol.{h,py}`,
  the firmware copy `Teensy_code_canbridge/udp_protocol.h`, and the profiling copy
  `tools/probes/teensy_link_profiling/jetson/udp_protocol.py`. It is never
  hand-edited in a generated file.
- The daemon reuses the existing decoders in `controller/teensy_link` /
  `config/generated/udp_protocol.py`; it does not re-implement frame parsing.
- GUI message field names are fixed by what the GUI reads — `panels.js`
  (`pos_estimate`, `vel_estimate`, `fet_temp`, `bus_voltage`), `main.js`
  (`motor_states[]`, `active_errors`, `disarm_reason`), and `telemetry-charts.js`
  (`iq_setpoint`, `iq_measured`, `motor_temp`); the daemon's JSON mirrors those
  names exactly.

## Implementation Phase Summary

| Phase | Title | Scope | Hardware |
|------|-------|-------|----------|
| 1 | Firmware telemetry tap | Tee STREAM uplink to `PORT_TELEM_TAP`; protocol-config codegen | Reflash; bench verify |
| 2 | Daemon: link RX + state model | `tools/jugglebot_telemetry_daemon.py` passive RX, latest-state model, staleness | No |
| 3 | Daemon: rosbridge-v2 server on :9090 | Minimal read-only protocol subset; map state → `robot_state` / `bb/heartbeat` JSON | No |
| 4 | :9090 coexistence with rosbridge | Daemon yields :9090 when rosbridge present; reclaims when gone (benign, non-safety) | No |
| 5 | Autostart + watchdog | `jugglebot-telemetry.service` + hang-watchdog, mirroring the GUI service | Boot test |
| 6 | (Optional) GUI telemetry-only affordance | Banner noting ROS2-offline / dark panels | No |

## Implementation Phases

### Phase 1: Firmware telemetry tap — NOT STARTED

Add a read-only duplication of the outbound STREAM uplink to a second Jetson port.

- Add `PORT_TELEM_TAP` (e.g. 5008) to the `CONSTANTS` list in
  `config/generate_udp_protocol.py`; run `python config/generate_udp_protocol.py`;
  stage the regenerated `config/generated/udp_protocol.{h,py}`, the firmware copy
  `Teensy_code_canbridge/udp_protocol.h`, and the profiling copy
  `tools/probes/teensy_link_profiling/jetson/udp_protocol.py`.
- In `udp_link.cpp`, after the existing send to `kJetsonIP:PORT_STREAM`, tee the
  same encoded frame to `kJetsonIP:PORT_TELEM_TAP`. Scope the tee to the STREAM
  TX path (`udp_send_stream` / `send_to` on `s_stream`) only — this captures
  `TELEMETRY`, `DIAGNOSTIC`, `HEARTBEAT_T2J`, and `PROFILE` and nothing else. The
  RPC channel and all inbound (setpoint / heartbeat-J2T) handling are untouched.
- Invariant: the tap is additive and read-only. The Teensy's CAN control,
  interpolation, setpoint reception, and fault machine are byte-for-byte
  unchanged. Setpoints are *received*, never teed.
- Verify Teensy headroom via the existing PROFILE counters / profiling harness
  (`tools/probes/teensy_link_profiling/`): the tee adds one extra UDP send per
  STREAM frame (~+6.4 KB/s of TELEMETRY plus per-frame overhead). Acceptance: no
  interp deadline-miss regression, CAN/UDP utilisation within margin.
- Decision required: tap delivery mechanism. The recommended mechanism is a
  **second unicast port** (independent binds; no `SO_REUSEPORT` packet-splitting
  and no IGMP/multicast setup; daemon and ROS2 fully decoupled). Subnet broadcast
  (`192.168.42.x`) or a multicast group are alternatives that avoid a fixed second
  port but add receive-side configuration; they are not required.

### Phase 2: Daemon — link RX + latest-state model — NOT STARTED

Create `tools/jugglebot_telemetry_daemon.py`, modelled on
`tools/teensy_link_bridge.py` (which is retained as a minimal debug/log tool).

- Bind `PORT_TELEM_TAP` for receive only. Reuse the decoders
  (`Telemetry.unpack`, `Diagnostic.unpack`, `HeartbeatT2J.unpack`) from
  `controller.teensy_link` / `config.generated.udp_protocol`. Do **not** import or
  use any TX / heartbeat / RPC / time-of-day client code.
- Maintain an in-memory latest-state model:
  - per-axis position / velocity from `TELEMETRY` (100 Hz);
  - per-axis ODrive `axis_state`, `ctrl_mode`, `active_errors`, `disarm_reason`,
    `iq_setpoint`, `iq_measured`, `temp_fet`, `temp_motor`, `bus_voltage` from
    `DIAGNOSTIC` (sparse / on-change → hold last value);
  - Ball Butler state + yaw/pitch/hand + link/bus health + fault state from
    `HEARTBEAT_T2J`.
- Staleness: track the age of the last received tap frame; expose it so the GUI's
  connection-quality badge reflects reality (the GUI already drives that badge off
  `robot_state` freshness).
- Invariant (enforced + tested): the daemon binds only the tap RX port and opens
  **no** socket whose destination is the Teensy. A test asserts zero datagrams are
  sent toward `TEENSY_IP`.

### Phase 3: Daemon — rosbridge-v2 server on :9090 (read-only) — NOT STARTED

Serve the GUI over the subset of the rosbridge v2 JSON protocol it actually uses.

- Implement, over a websocket on :9090:
  - `subscribe` → ack, then stream `publish` frames for the supported topics;
  - `advertise` / `publish` (GUI → server) → accept and drop (read-only);
  - `call_service` → reply `service_response` with `{ success: false, message:
    "ROS2 not running" }` (per the read-only decision — no command path to the
    Teensy in daemon mode);
  - topic discovery, if the GUI's topic-spy uses it.
- Map the latest-state model to GUI-shaped JSON with exact field names:
  - `robot_state` ← `TELEMETRY` + `DIAGNOSTIC` (`motor_states[]` with
    `pos_estimate`, `vel_estimate`, `active_errors`, `disarm_reason`,
    `iq_setpoint`, `iq_measured`, `fet_temp`, `motor_temp`, `bus_voltage`, plus
    the boolean flags `main.js` reads);
  - `bb/heartbeat` ← `HEARTBEAT_T2J`.
- Publish at the GUI's expected throttle (`robot_state` ~20 Hz).
- Topics with no Teensy source (`mocap_data`, `cone/*`, `motion/diagnostics`,
  `leg_lengths_topic`, `orchestrator_state`, `control_mode_topic`, …) are simply
  never published; the corresponding GUI panels remain inactive. This is expected
  documented behaviour, not a fault.
- Decision required: websocket implementation. Confirm whether a websocket library
  is available in the project venv; if not, vendor a minimal RFC 6455 server (the
  required protocol subset is small). Prefer a stdlib-friendly footprint matching
  `gui_server.py`.

### Phase 4: :9090 coexistence with rosbridge — NOT STARTED

`rosbridge_server` also binds :9090; only one process may own it.

- Root cause / why this handoff is benign: :9090 carries **no** setpoints. Unlike
  the Teensy link, a fault here can at worst cause a GUI websocket reconnect, never
  motion. The strict single-owner discipline required for the control link does
  **not** apply.
- Mechanism (recommended): the daemon self-arbitrates :9090. It binds and serves
  only when :9090 is free; on detecting rosbridge (ROS2 up / port taken) it
  releases :9090 and goes quiet while continuing to receive tap frames (so its
  state model stays warm and reclaim is instant when ROS2 exits). Self-arbitration
  is acceptable here precisely because :9090 is non-safety.
- The daemon's tap RX (Phase 2) never contends with ROS2, so the daemon is always
  live regardless of who owns :9090.

### Phase 5: Autostart + watchdog — NOT STARTED

Mirror the landed GUI boot service.

- `jugglebot-telemetry.service` (system service, `User=jetson`, `Restart=always`,
  start-rate-limit disabled), source copy in `/home/jetson/System_Scripts/`,
  installed to `/etc/systemd/system/`.
- Hang-watchdog: a timer + health-check that restarts the daemon if it stops
  receiving tap frames or stops serving (the analogue of the GUI's HTTP
  health-check; here the check is "tap frame age" and/or ":9090 responsiveness").
- The telemetry daemon and the static GUI server are independent units; both start
  at boot.

### Phase 6: (Optional) GUI telemetry-only affordance — NOT STARTED

Zero GUI changes are required for function. Optionally, add a small banner shown
when served by the daemon ("Telemetry-only — ROS2 offline; commands disabled;
mocap / cone / MPC panels inactive") so the dark panels are not mistaken for a
defect. This is the only GUI-touching work in the plan and is deferred / optional.

## Testing Plan

### Unit tests (offline, no hardware)

- Replay a recorded tap capture (or a synthesised frame sequence) through the
  daemon; assert the emitted `robot_state` / `bb/heartbeat` JSON matches the field
  names and shapes the GUI consumes (cross-check against `panels.js`, `main.js`,
  and `telemetry-charts.js` field reads). Follow the offline-faithful-replay
  pattern used elsewhere in the project (recorded telemetry → production source
  layer).
- Assert the daemon emits **zero** datagrams toward `TEENSY_IP` (socket-level mock
  / assertion) — the read-only safety invariant.
- Assert staleness handling: stop feeding frames; confirm the daemon marks data
  stale within the configured window.
- rosbridge protocol conformance: a synthetic ROSLIB-style client can `subscribe`
  and receive well-formed `publish` frames; `call_service` returns the read-only
  refusal.

### Firmware / bench tests

- Confirm tap frames are byte-identical to the primary 5005 frames (capture both
  ports; diff).
- Confirm the primary 5005 stream and the inbound setpoint/heartbeat/RPC path are
  unchanged (RX behaviour and control unaffected).
- PROFILE counters: no interp deadline-miss regression; CAN/UDP utilisation within
  margin with the tap enabled.

### Integration tests (Jetson + Teensy connected, motors OFF)

- ROS2 down → open the GUI on :8081 → live motor + Ball Butler telemetry appears
  via the daemon on :9090.
- ROS2 up → the daemon yields :9090, rosbridge feeds the GUI, and the daemon keeps
  receiving tap frames with no contention on 5005.
- Cycle ROS2 up/down repeatedly → telemetry continuity, no link errors, no
  `EADDRINUSE` on 5005.

### Hardware tests (Jetson + Teensy, motors ON)

- Safety acceptance gate: confirm the tap adds no measurable control-path jitter.
  The tap is TX-only on the Teensy and read-only on the Jetson, so no setpoint
  effect is expected; verify by comparing setpoint timing / interp deadline-miss
  with the tap disabled vs enabled.

### Regression tests

- Full `pytest tests/ -q` green; existing `tests/teensy_link/` and
  `tests/firmware/` suites unaffected.
- `python config/generate_udp_protocol.py` regenerates cleanly with the new port;
  no diff drift in unrelated generated artefacts.

## Notes for Collaborators

### Safety-critical invariants (must be preserved exactly)

1. The daemon transmits **nothing** on the Teensy link. It binds only the
   read-only tap port and never sends setpoints, heartbeats, RPC, or
   time-of-day. (Enforced: no socket targets `TEENSY_IP`; a unit test asserts zero
   TX. The read-only command policy means even GUI service calls return a refusal,
   never a Teensy RPC.)
2. The firmware tap is **additive and read-only**. It duplicates outbound STREAM
   uplink to a second port; it changes nothing the Teensy receives or how it
   controls motors. The setpoint / heartbeat-J2T / RPC path is byte-for-byte
   unchanged.
3. The :9090 handoff is **non-safety**. No setpoints traverse :9090; a daemon bug
   can at worst trigger a GUI reconnect.

### Architecture decisions

- **Fan-out via a firmware tap** was chosen over fallback-with-handoff and over a
  full link-ownership proxy. The deciding fact (verified 2026-06-08): the firmware
  emits telemetry unconditionally to a fixed unicast peer
  (`Teensy_code_canbridge.ino:159-166`, `telemetry.cpp:95`, `udp_link.cpp:127-143,
  20`), so a passive listener needs zero downlink. The proxy model was rejected
  because UDP single-bind would force the daemon into the setpoint/heartbeat path,
  adding jitter to a real-time stream, making a convenience daemon safety-critical,
  requiring a fail-closed heartbeat-liveness redesign, and refactoring the WIP
  bridge — all to obtain benefits (no handoff, seamless telemetry) that fan-out
  delivers without touching the control path.
- **Unicast, not broadcast, today.** Jetson-side `SO_REUSEPORT` cannot fan out a
  unicast stream (it load-balances datagrams across sockets), which is why the tap
  must be created at the firmware source rather than purely on the Jetson.
- **Read-only daemon.** In daemon mode the robot is not under full control; exposing
  commands would require per-command safety review and a Teensy RPC path. Telemetry
  visibility is the goal; commands stay with the ROS2 stack.

### Files affected

- `config/generate_udp_protocol.py` (+ regenerated
  `config/generated/udp_protocol.{h,py}`, firmware copy
  `Teensy_code_canbridge/udp_protocol.h`, profiling copy
  `tools/probes/teensy_link_profiling/jetson/udp_protocol.py`).
- `ros_ws/src/jugglebot/Teensy_code_canbridge/udp_link.cpp` (tap tee).
- `tools/jugglebot_telemetry_daemon.py` (new); `tools/teensy_link_bridge.py`
  retained unchanged as a debug tool.
- `System_Scripts/jugglebot-telemetry.service` + watchdog units (new; outside the
  repo, alongside the existing GUI units).
- `tests/` (daemon unit tests; firmware/bench checks).

### Rollback plan

- Disable the daemon service → telemetry reverts to ROS2-only; nothing else
  changes.
- The firmware tap is independently revertable (remove the second send). Left in
  place it is harmless: an unbound tap port simply drops the duplicated frames.

---

## Archival note (2026-08-01)

**Archived as deferred — design only, nothing implemented.** All six phases are
still marked NOT STARTED in the text above, and none of the named artefacts
exist: there is no `tools/jugglebot_telemetry_daemon.py` and no `PORT_TELEM_TAP`
in the generated UDP protocol. No content edit since the plan was written
(2026-06-08, `49ff5b7`).

The GUI-availability half of the original problem was solved separately — the
front end is served headless on boot by the `jugglebot-gui.service` systemd unit
on :8081 — but live data still requires the ROS2 stack, which is exactly the gap
this plan was written to close. Revive from here if a ROS2-independent live feed
is wanted.

Moved out of `plans/active/` by the 2026-07 refactor programme
(`plans/active/refactor-2026-07.md` § Phase 1, item 5).
