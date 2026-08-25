---
title: "The UDP panel stops counting interleaving and starts gauging channel health — P1–P4 in one session"
type: feature
date: 2026-08-25
status: resolved
phase: "operator-observability — UDP channel health P1–P4"
related_plan: udp-channel-health.md
files_changed:
  - teensy_link/client.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/gui/js/udp-traffic.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/css/panels.css
  - tests/teensy_link/test_client.py
  - tests/ros/test_teensy_bridge_node_udp_diag.py
  - tests/ros/test_gui_geometry.py
subsystem:
  - gui
  - ros
  - teensy_link
tags:
  - observability
  - udp
  - testing
---

# UDP channel-health metrics — P1–P4

## Summary

All four phases of `plans/archived/udp-channel-health.md` implemented in one
session, on the same day the plan was drafted out of the Gaps-artifact
investigation (`2026-08-25-udp-gap-column-artifact`). The panel loses the number
that measured nothing (per-type gaps) and gains three that gauge health:
the bridge's **latency verdict**, **rate conformance against firmware
nominals**, and **one honest aggregate loss counter**. A 17-finding audit
(2 BLOCKING / 8 WARNING / 7 NOTE) was then applied in full.

## Motivation

The plan's ranking is set by this repo's own incident history: **latency first,
rate conformance second, loss last**. The worst outage this project has had —
the five-week uptime-lag arc (2026-07-18 → 2026-08-15, closed by can-bridge
FW 14) — was a pure *latency* failure that every loss counter read clean
through: the frames all arrived, just late. A panel of rates and error totals is
blind to that entire failure class. Meanwhile the one loss-looking number the
panel *did* carry was an artifact of a shared wire counter, and the 2026-08-25
`HAND_SENSOR` confusion on top of it was a *units* misread. So: delete the
false number, then wire the two classes that actually fail here.

## Implementation

**P1 — the per-type gap counters are gone end-to-end.** `_track_seq`'s per-type
body and `LinkStats.seq_gaps_by_type` (`teensy_link/client.py`), the
`gap_<TYPE>` publish loop (`teensy_bridge_node.py`), and `'gap'` from the GUI's
`PER_TYPE_KEY_RE` prefix alternation (`udp-traffic.js`). Four coupled test sites
moved with them, including the **bidirectional pin** in
`tests/ros/test_gui_geometry.py` (`PREFIXES = ('rx','tx')` at `:567`, asserted
producer-side against the node and consumer-side against the extracted JS
regex) — changing either end alone fails the gate.
`test_udp_diag_omits_zero_gap_rows_and_emits_nonzero_ones` retired in favour of
`test_udp_diag_seq_gaps_follows_a_real_wire_skip`.

**P2a — the latency verdict on the panel.** `latency_monitor`
(`OK | CLAMP_DUTY | CACHE_AGE | RING_LEAK`) is already published unconditionally
on `link_status`; the footer now renders the current token **plus
worst-in-session**, page load being the reset. The rank table is **three-way
pinned** by `TestUdpPanelLatencyMonitor` — the JS copy, the node's
`_LATENCY_MONITOR_RANK`, and `tests/hardware/tilt_cal_grid.py`'s offline
`worse_latency_monitor` must agree. An **unknown token ranks 4**, above every
named cause: a token this page does not recognise comes from a node *newer* than
the GUI build, and must never be hidden behind a cause we do know.

**P2b — `rtt_us` from `/clock_diag`** in the footer as `anchor rtt`, via a new
`main.js` route (`udpTrafficOnClockDiag`). ~30 s anchors, so it is deliberately
**not** put through the 3 s staleness class the other two feeds use. Labelled
`anchor rtt`, not latency: it is the transport round trip, not an end-to-end
telemetry latency — nothing publishes one of those.

**P3 — rate conformance.** A `NOMINAL_RATES` table for the **steady flows only**,
each nominal carrying its firmware source in a comment and pinned against that
constant by `TestUdpNominalRates` (extract JS table, extract firmware header,
assert equal): `TELEMETRY` / `BB_AXIS_ESTIMATES` / `LEG_CMD` 100 Hz (one
unconditional tick, `telemetry.cpp:619-622`), `HAND_SENSOR` 50 Hz **configured**,
`HEARTBEAT` 10 Hz both directions, `SETPOINT` bimodal. Band ±25 %; a deviating
cell takes `.udp-rate-off` (amber), placed after the idle-row rule so it **wins
the opacity** — a `TELEMETRY` stream that fell to 0 is an idle row by the letter
of that rule and the most important thing on the panel by any other measure.
Event-driven types are absent from the table and are never annotated. Footer
units are now labelled throughout (`since node start: crc N frames · … · seq
gaps N events · capped N drains`).

**P4 — one honest aggregate `seq_gaps`.** A per-socket tracker in `LinkStats`
keyed on `sock is self._rpc_sock`, u16-wrap-safe, **baseline cleared on
reconnect** (a new socket pair is a new sequence epoch). Foreign-source frames
are **excluded from tracking but kept in `rx_frames` and the per-type rows**, so
a second binder on the fixed ports :5005/:5006 surfaces as a count discrepancy
rather than as fabricated wire loss. Published on `/udp_diag` **and** on
`/link_status` beside `uptime_ms` — the reboot detector must be readable from
the same tick, because a peer restart costs exactly one gap and nothing else
distinguishes it from a real drop. The docstring carries four caveats:
failed-Teensy-send-still-counts (correct — the seq is consumed before
`beginPacket`), the echo path (re-sends verbatim to *whoever* sent the frame and
bypasses the TX seq counters entirely), `rpc.cpp:403`'s `if (n)` silent
send-loss blind spot (no seq consumed, so no hole appears), and
peer-restart-costs-one-gap.

## Discussion

**1. The two halves are not equally comparable, and the docstrings said they
were.** P4's whole justification is that the Teensy measures the same quantity
for the opposite direction (`track_downlink_seq`, `udp_link.cpp:72-79`) so the
two link ends become comparable. The audit caught (BLOCKING) that the call is
gated `if (!is_rpc)` at `:107` — it is **STREAM-ONLY**. The stream half of our
aggregate has a far-end counterpart; the RPC half has none at all, so a nonzero
*total* can carry RPC gaps the far end never counted, and an end-to-end mismatch
is not by itself evidence of anything until the halves are separated. The
counter was kept as-is (splitting it would double a footer number for a case
nobody is chasing) and the docstrings now say exactly what is and is not
comparable, at both the client and the publisher.

**2. The pre-registered `/udp_diag` echo check was dropped BY DECISION, not
skipped.** § 8 of the plan pre-registered one `ros2 topic echo /udp_diag` to
compare cumulative `rx_HAND_SENSOR` against `gap_HAND_SENSOR` and close the
`≈ RX/2` open question. P1 deleted those keys before anyone ran it. It is not
being reinstated: `send_to` consumes **one** counter per socket
(`udp_link.cpp:171`) for every message type, and `telemetry_step` emits its trio
back-to-back on a single tick, so **any** per-type ratio over that counter is an
artifact by construction. The particular ratio `HAND_SENSOR` showed is a fact
about how often another type interleaved with a round-trip-paced flow and
carries no information about link health. Measuring it would have confirmed a
mechanism already proven from the firmware source, at the cost of implying the
number meant something. The question is closed by firmware read.

**3. `SETPOINT` polices only the high side.** It is genuinely bimodal — 0 while
nothing streams, ~40 while `trajectory_node` does — and a rate window whose
edges straddle a stream's start reads *somewhere in between*, legitimately.
Policing the low side would flash amber at every move onset, which is exactly
how an annotation stops being read. `rateDeviates` therefore returns early for
`bimodal` entries after the overshoot test.

## Verification

All 2026-08-25, project venv (`~/Desktop/PDJ_venv/venv`):

- Per-phase gate, run after each of P1–P4:
  `pytest tests/teensy_link/ tests/ros/test_teensy_bridge_node_udp_diag.py tests/ros/test_gui_geometry.py -q`
  → grew **352 → 370** across the four phases; final **370 passed in 13.06 s**.
- `./run_tests.sh` (default gate, **pre-audit-fixes**) → **5948 passed,
  4 skipped, RESULT: PASS**, 267 s.
- Post-audit-fixes combined re-run of the six target files → **580 passed in
  13.91 s**.
- `node --check` clean on both `ros_ws/gui/js/udp-traffic.js` and
  `ros_ws/gui/js/main.js`.
- `py_compile` clean on `teensy_link/client.py` and `teensy_bridge_node.py`
  under **both** the venv and `/usr/bin/python3` (3.8.10 — the ROS runtime).

**The `./run_tests.sh --full` triple for the committed tree goes in the commit
message, not here** — this entry lands alongside the code it describes.

## Outcome

The panel now answers *"is the link carrying what it should, when it should?"*
in the plan's own ranking: latency verdict and anchor RTT first, per-flow rate
conformance in the table, one honest loss number in the footer. The number that
started this — per-type Gaps — is gone from producer, wire and consumer, with a
bidirectional pin so it cannot creep back on one side.

**Deploy:** `colcon build` for `teensy_bridge_node` (the node runs from the
installed copy); `teensy_link/` is PYTHONPATH-live and takes effect at the next
relaunch with no build; the GUI is static JS/CSS — **browser hard-refresh**.

## Open Questions

- **First bench read of the new footer.** `latency_monitor`, `anchor rtt` and
  the amber rate annotations have never been seen against a live link. Expect
  `anchor rtt` at 1–3 ms and `latency OK`; anything else is the panel doing its
  job.
- **`HAND_SENSOR` will read ~42 msg/s against a 50 nominal until the FW 16
  flash.** That is in-band by design (±25 % of 50 is 37.5–62.5, so it does not
  colour) and is the known FW 15 poller cadence, diagnosed and fixed in FW 16
  (`2026-08-24-poller-cadence-and-tristate-tx`). An operator comparing the live
  number against the stated nominal is not looking at a fault.
- **The RPC half of `seq_gaps` has no far-end counterpart** (Discussion 1). If a
  nonzero total ever needs attributing, splitting the scalar into stream and RPC
  is the follow-up.
- **Plan archival — DONE.** The plan was archived `completed` on 2026-08-25
  (`plans/archived/udp-channel-health.md`), the same day it was drafted,
  implemented and audited. The first bench read of the new footer is the
  **watch item** recorded in that plan's archival note, not a gate on
  archival.
