---
title: UDP channel-health metrics — four independently-landable phases
created: 2026-08-25
status: proposed   # outside DOCUMENTATION_GUIDE 2.6's vocabulary on purpose, for a document nobody has approved yet (as leg-bus-frame-drops.md does); `active` on approval.
owner: harrison
last_updated: 2026-08-25
related_plan: operator-observability.md
related_logbook:
  - 2026-08-22-udp-rate-panel.md            # the F2 entry that shipped the panel and the gap_<TYPE> keys
  - 2026-08-25-udp-gap-column-artifact.md   # the read-only investigation this plan carries forward
related_code:
  - teensy_link/client.py::UdpClient._track_seq
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py::_publish_udp_diag
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py::_latency_monitor_step
  - ros_ws/gui/js/udp-traffic.js
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_link.cpp::track_downlink_seq
---

# UDP channel-health metrics

> **PROPOSAL — nothing implemented.** Diagnostic surface only: nothing here is in the leg safety loop, and § 7
> rules out the firmware option precisely because it would be. Investigated 2026-08-25 (session `f0978e1e`); the
> diagnosis is complete, so implementation needs no re-investigation. **Rescoped 2026-08-25 after owner review**:
> the goal is *gauging channel health*, not counting gaps.

## 1. Why latency outranks loss

The panel must answer *"is the link carrying what it should, when it should?"* at a glance, and this repo's incident
history sets the ranking — **latency first, rate conformance second, loss last**. The five-week uptime-lag arc
(2026-07-18 → 2026-08-15, FW 14) was a pure *latency* failure every loss counter read clean through (the FlexCAN_T4
RX ring became an uptime-ratcheting delay line — frames arrived, just late), so a loss-only panel is blind to the
worst outage this project has had; the 2026-08-25 `HAND_SENSOR` confusion (§ 2) was a *rate/units* misread. Loss is
real but rare, and its honest form is one footer number. The four phases below are ranked by that value and are
**independently landable**.

## 2. Carried-over diagnosis — the per-type "Gaps" column measures nothing

`teensy_link/client.py:595-605` (`_track_seq`) keeps `_last_rx_seq` **per message type** and counts a gap whenever
`seq != prev+1` for that type. The wire carries **one shared sequence counter per (socket, direction)** —
normative in `docs/teensy-udp-protocol.md:21` ("seq u16 LE — per-(channel,direction) counter"), and implemented
that way on the uplink: a single `s_seq_stream_tx` (`udp_link.cpp:41`), consumed once per stream frame inside
`send_to` (`udp_link.cpp:171`) for every type (`udp_link.cpp:183`).

So a "gap" is counted whenever **any other frame type interleaves**. `telemetry.cpp:619-622` (`telemetry_step`)
emits `TELEMETRY`, `BB_AXIS_ESTIMATES` and `LEG_CMD` back-to-back every 100 Hz tick, so for those three
gaps ≡ rx−1, deterministically — confirmed on the bench 2026-08-25 (the operator saw Gaps ≈ RX exactly). **The
per-type intent is unsatisfiable**: a shared counter carries no per-type information, so no Jetson-side bookkeeping
can recover per-type loss — not a tuning bug. The only test (`tests/teensy_link/test_client.py:63-72`) drives one
type through an otherwise silent link, so the interleave case was never exercised.

Predicted by the mechanism, not yet observed: **the fingerprint is free at a bench glance**, and real loss cannot produce it — `RPC_RESPONSE` shows ~zero gaps
(alone on the RPC socket, nothing interleaves), while burst-drained types (`DIAGNOSTIC`, `CONE_FRAME`, `CMD_RESULT`,
`PLATFORM_FRAME`, `HAND_CMD_ECHO`) show gaps *well below* RX — they arrive adjacent, so the shared counter often does
read `prev+1`. Loss would invert that ordering. The one unexplained observation was `HAND_SENSOR` gaps ≈ rx/2 where
the mechanism predicts ≡ rx−1; gaps < rx−1 needs *sender-side back-to-back same-type frames* — **loss cannot create
adjacency** — so it is not evidence of loss. The live candidates are the unit mixing (P3) and a degraded poller.

## 3. P1 — delete the per-type Gaps column and `seq_gaps_by_type`

Removes a number that cannot mean what it claims; nothing below depends on it. **The rendered column is already gone
in the working tree** — an uncommitted 2026-08-25 GUI edit dropped it at `udp-traffic.js:511`, relabelled the
headers `rx (msg/s)`/`tx (msg/s)`, and followed through in `tests/ros/test_gui_geometry.py:610-612`. What remains is
the **producer and the key contract**: drop `seq_gaps_by_type` from `LinkStats` and `_track_seq`
(`client.py:159, 174, 604-605`), delete the `gap_<TYPE>` publish loop (`teensy_bridge_node.py:5128-5130`), and drop
`'gap'` from `PER_TYPE_KEY_RE` (`udp-traffic.js:57`) — the panel still receives keys nothing renders until it does.

**Four test sites move together** (all assert the per-type keys today): `tests/teensy_link/test_client.py:63-72`
(retires or narrows to rx/tx) and `:157` (snapshot independence — drops the dict);
`tests/ros/test_teensy_bridge_node_udp_diag.py:88-104` (`test_udp_diag_omits_zero_gap_rows_and_emits_nonzero_ones`,
retires); and the **bidirectional pin** at `tests/ros/test_gui_geometry.py:543` (`PREFIXES = ('rx','tx','gap')` →
`('rx','tx')`), asserted producer-side at `:549-552` and consumer-side at `:568-572`, where the extracted
`PER_TYPE_KEY_RE` prefix alternation is compared to `PREFIXES` — so changing either end alone fails.

**Verification, no code needed** (closes the `HAND_SENSOR` question before the keys go — do it while `/udp_diag`
still carries them): one `ros2 topic echo /udp_diag`, compare **cumulative** `rx_HAND_SENSOR` against
`gap_HAND_SENSOR` in the same message; ≈1.0 confirms the by-construction mechanism and closes it.

## 4. P2 — latency on the panel (verified: the plumbing differs from the brief)

**What actually exists.** The alarmed `latency_monitor` from the FW 14 / ring-leak close-out publishes a
**verdict token, not milliseconds**:

- Tokens `OK | CLAMP_DUTY | CACHE_AGE | RING_LEAK` — `teensy_bridge_node.py:294-297`, causal rank `:304-308`; computed
  in `_latency_monitor_step` (`:4600-4686`), one evaluation point per `/link_status` tick (`:4704`); published
  **unconditionally** as `KeyValue(key='latency_monitor', …)` at `:4916`.
- It carries no numbers **by design** — `:4907-4912` states the leak, cache floor and `lead_clamp_mask` behind it
  are already bagged on `/ring_diag`, `/cache_diag` and `/link_status`. Those inputs: `_lm_ring_leak` (frames;
  `:1406`, fed at `:2822`), `_lm_cache_floor_us` (µs; `:1408`, fed `:2630`), clamp duty (`_clamp_duty` `:4592-4598`).
- `temporal_health_verdict` is **not in the node** — it is an offline harness helper
  (`tests/hardware/tilt_cal_grid.py:1324`, `worse_latency_monitor` at `:1307`) used by `toss_cal_grid.py:652` and
  `tilt_cal_grid.py:2528`, and it *reads* the token.
- **Nothing publishes an end-to-end telemetry latency in ms anywhere.** The healthy 10–20 ms e2e figure is from
  the FW 14 validation campaign, not a live surface.

So P2 is still "wire existing measurement to the panel" — the measurement is just a verdict:

- **P2a (render-only, ~30 min)** — show the current `latency_monitor` token plus **worst-in-session**.
  `udp-traffic.js` already receives `link_status` via `udpTrafficOnLinkStatus` (`:334`, routed from `main.js`): no new topic, no node change. Worst-in-session is
  a client-side max over the rank table, mirroring `worse_latency_monitor`; pin the four token strings and their
  rank order against the node's **source text**, as `tests/motion/test_tilt_cal_grid.py`'s xref test does.
- **P2b (one number, optional)** — the only live latency in ms on a topic today is `rtt_us` on `/clock_diag`
  (`teensy_bridge_node.py:2559`, also `msg.message` `:2547`), the time-sync anchor RTT, **1–3 ms healthy**; mirror
  it into the footer as a fifth aggregate. **Explicitly NOT in P2**: a true e2e latency in ms — that is *new*
  measurement (stamp-compare against the wire-bound `t_bridge_us` the anchor makes comparable), worth scoping only
  if P2a's verdict proves too coarse in practice.

## 5. P3 — per-flow rate conformance + the units fix

The msg/s column exists; it has no reference. Colour/annotate it against a nominal table, **STEADY flows only**:

| flow | nominal | firmware source |
|---|---|---|
| `TELEMETRY` rx | 100 Hz | `TELEM_RATE_HZ`, `canbridge_config.h:96` — a bench-only 250 Hz override sits at `:94`; read the *active* one |
| `HAND_SENSOR` rx | 50 Hz healthy, ~1 Hz floor | `JBBallDetect::CHECK_INTERVAL_MS = 20u`, `hardware_config.h:524` (the 50 ms at `:512` is `BBBallDetect`). The send is on-change with a 1 Hz keepalive (`HAND_SENSOR_KEEPALIVE_US`, `telemetry.cpp:249`; gate at `:275`), so **a drop to ~1 Hz is itself the diagnosis** — the poller stopped getting new replies. `static_assert` at `:253` pins poll ≥ telemetry tick |
| `HEARTBEAT` both dirs | 10 Hz | `JbUdp::HEARTBEAT_HZ`, `udp_protocol.h:28` / `HEARTBEAT_RATE_HZ`, `canbridge_config.h:99`; generated mirror `config/generated/udp_protocol.py:28` |
| `SETPOINT` tx | 0 **or** ~40 while streaming | bimodal-ok: neither value is a fault, so the annotation must accept both |

**Exempt — event-driven, no nominal**: `RPC_REQUEST`/`RPC_RESPONSE`, `CMD_RESULT`, the echo types (`HAND_CMD_ECHO`),
`CONE_FRAME`, `PLATFORM_FRAME`, `DIAGNOSTIC`. `BB_AXIS_ESTIMATES` and `LEG_CMD` ride the same 100 Hz tick
(`telemetry.cpp:619-622`) but **verify each is unconditional before giving it a nominal**. Nominals live in a JS
table in `udp-traffic.js`, **each entry commented with its firmware source line**, with a drift-pin test in the
house tripwire style (`tests/ros/test_gui_geometry.py`'s extract-source-and-compare pattern): extract the JS table,
extract the firmware constants, assert equality — a firmware retune then fails the suite, not the operator's read.

**Fold in the units fix here.** § 2's confusion was a units misread: rx/tx rendered as *rates* while gaps rendered
as a *cumulative total* in the same table. P1's working-tree half already fixed the columns (`rx (msg/s)`/`tx
(msg/s)`); P3 owns the rule for the rest — **every footer label states its unit too** (`frames since node start`) —
so P4's cumulative scalar cannot re-create the ambiguity.

## 6. P4 — one aggregate per-socket loss counter in the footer 

Demoted from headline to honesty-check; mechanics unchanged. Measure the quantity the wire actually guarantees.
`_drain_socket(self, sock)` (`client.py:523`) already has the socket in hand — hold a 2-entry tracker keyed on
`sock is self._rpc_sock` (the two channels have separate counters), count an aggregate `seq_gaps` scalar in
`LinkStats` (P1 removed the per-type dict), and publish it **lower case** beside `crc_errors` / `decode_errors` /
`drain_capped` (`teensy_bridge_node.py:5132-5138`). The case split is the contract: `PER_TYPE_KEY_RE`
(`udp-traffic.js:57`) routes UPPER-case names to per-type rows and lower-case to the footer, so it lands in the
aggregates with **no GUI restructuring** — add it to `AGGREGATE_KEYS` (`:58-60`). **Why this quantity:** it is what
the Teensy already measures for the downlink — `track_downlink_seq` (`udp_link.cpp:72-79`), aggregate, RPC socket
excluded (`:107`) — so nonzero means a frame really went missing, and the two link ends become comparable.

**Two caveats for the docstring:** (i) the Teensy consumes a seq (`udp_link.cpp:171`) *before*
`beginPacket`/`endPacket`, so a failed send correctly shows as a gap — a feature, not noise; (ii) the echo path
(`udp_link.cpp:122-132`) re-sends an unhandled received frame **verbatim, carrying the Jetson's seq** — inert today
(every Jetson TX type is handled) but it would poison the tracker the moment a future unhandled TX type appears.

## 7. Ruled out

**Per-type wire sequence numbers** (firmware + `PROTOCOL_VERSION` bump) — **ruled out.**
`leg_interp.cpp:184-193`'s monotonic-seq guard carries a "CRITICAL PARITY TRAP" note stating that `SETPOINT` and
`HEARTBEAT_J2T` share the host counter *by design*, and that a `seq == last+1` test would false-drop every
setpoint following a heartbeat; the Teensy's own downlink gap counter (P4) depends on the shared counter the same
way. Safety-loop code is not worth risking for a diagnostic.

## 8. Done means

Each phase ships alone, gated on `./run_tests.sh` green. **P1** — no per-type gap key on `/udp_diag`, no gaps column,
four test sites updated, the `/udp_diag` echo logged. **P2** — current and worst-in-session `latency_monitor` on the
panel, tokens and rank order pinned against the node source. **P3** — steady flows annotated from a firmware-pinned
JS table, every number stating its unit. **P4** — one lower-case `seq_gaps` aggregate, both sockets tracked apart.
