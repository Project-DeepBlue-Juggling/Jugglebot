---
title: The topics panel toggles to true per-type UDP message rates
type: feature
date: 2026-08-22
status: resolved
phase: "operator-observability F2"
related_plan: operator-observability.md
files_changed:
  - teensy_link/client.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/gui/js/udp-traffic.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/index.html
  - ros_ws/gui/css/panels.css
  - ros_ws/gui/css/theme.css
  - ros_ws/docs/choreography.md
  - tests/ros/test_gui_geometry.py
  - tests/ros/test_teensy_bridge_node_udp_diag.py
  - tests/teensy_link/test_client.py
  - logbook/INDEX.md
subsystem:
  - gui
  - teensy_link
  - ros
tags:
  - observability
  - udp
---

# UDP rate panel (F2)

## Summary

The "ROS2 Topics" panel counted messages **this browser received**, and spy
subscriptions are throttled to 200 ms — so its Hz column saturates at 5 for
every spied topic and says nothing about the UDP link. The link's 21 message
types had per-type RX counters in `LinkStats` that nothing published.

- **R1** `LinkStats` gains `tx_count_by_type` (one increment in each of
  `send_stream`/`send_rpc`/`send_to`, after the `sendto` so a raising send
  counts nothing, matching `tx_frames`). All three per-type dicts are now
  **pre-seeded** with every `MsgType` id: `snapshot()` copies them unlocked
  from the ROS timer while the RX thread writes, and rebinding an existing key
  is safe where inserting one raises `RuntimeError` mid-`dict()`. It also makes
  a never-seen type an honest `0` row instead of an absent one.
  **[2026-08-25: two dicts, not three — `seq_gaps_by_type` was removed by
  `plans/archived/udp-channel-health.md` P1. The pre-seeding rationale is
  unchanged for the surviving `rx_count_by_type`/`tx_count_by_type` pair; the
  sentence above is the 2026-08-22 record.]**
- **R2** `/udp_diag`, 1 Hz, modelled on `_publish_profile`: `rx_<TYPE>` /
  `tx_<TYPE>` by enum NAME for every type, `gap_<TYPE>` only when nonzero, plus
  the lower-case aggregates `rx_frames`/`tx_frames`/`crc_errors`/
  `decode_errors`/`drain_capped`. **Counters, never rates** — the divisor of a
  rate computed here would be this timer's actual firing interval, thrown away
  before the bag saw it. Stats are snapshotted once per tick so one message is
  one instant. The level follows the CRC+decode **delta**, not the total: one
  corrupted frame at boot must not latch WARN for the session.
  **[2026-08-25: the `gap_<TYPE>` keys were removed and one lower-case
  aggregate `seq_gaps` added in their place, by
  `plans/archived/udp-channel-health.md` P1/P4; the key inventory above is the
  2026-08-22 record of what F2 shipped.]**
- **R3–R5** new ES module `ros_ws/gui/js/udp-traffic.js` (no imports; it is a
  table, not a chart), routed beside `profile`/`link_status` in `main.js`. Rate
  = Δcount/Δt over a 5/10/30/60 s window (1 Hz cadence makes a 1 s window one
  sample — the CAN panel's lesson). Column header is **`msg/s`, never Hz**, and
  the tooltips name the difference. A `<button>` segmented control in the shared
  `.panel-header` switches views (a button is inert to `initCollapsiblePanels`'
  interactive-element guard); mode persists beside `jugglebot-hidden-topics`,
  and the ROS monitor keeps running hidden. The view is gated on `bridge_link`:
  a dead uplink renders a red **LINK DOWN** banner and `--` rates, never a
  plausible table of zeros.
- **R6** `TestUdpDiagKeyValueContract` pins producer ⊇ consumer for the new
  pair (reconstructing the per-type key set from `MsgType` on both sides, with
  both f-string shapes asserted), `js/udp-traffic.js` joins `EXPECTED_FILES`,
  and `choreography.md` is regenerated.

One deliberate divergence from `can-traffic.js`: it DROPS a payload that
arrives while the uplink is down (its payload is a frozen cache). A counter is
a total, and a total that stopped advancing is the truth about a dead link — so
those samples are kept, and a window spanning a 3 s outage on a 100 msg/s
stream honestly averages ~70.

## Verification

- `./run_tests.sh` (run 2026-08-22, worktree `~/Desktop/Jugglebot-obs`):
  parallel phase **5461 passed, 6 skipped, 3 warnings in 235.05 s**; serial
  phase 5902 deselected (every `serial`-marked test is also `nightly`, so the
  gate's serial phase is empty); total 248 s, `RESULT: PASS`. Exactly +15 on
  the F1 run's 5446: 4 in `test_client.py`, 6 in the new
  `test_teensy_bridge_node_udp_diag.py`, 4 in `TestUdpDiagKeyValueContract`,
  and 1 more `EXPECTED_FILES` parametrisation.
- One-off `/tmp/probe_udp_traffic.mjs` (stubbed DOM, shipped module imported
  verbatim, `Date.now` faked): 11 samples at 1 Hz → TELEMETRY 100 rx msg/s,
  SETPOINT 40 tx msg/s, aggregate 140 rx; `bridge_link=LOST` → banner
  `LINK DOWN`, every rate `--`, container `.udp-stale`; recovery → 70 msg/s
  across the outage then 100; a counter reset drops history instead of showing
  a negative; `gap_TELEMETRY=4` surfaces, zero gaps do not.
  **[2026-08-25: the per-type gap keys and `seq_gaps_by_type` were removed by
  `plans/archived/udp-channel-health.md` P1; the triple above is historical —
  it is what the probe exercised on 2026-08-22, not a claim about the panel
  today.]**
- `node --check` on every touched JS file.
- Not yet eyeballed on hardware. Deploy: `colcon build --packages-select
  jugglebot` + relaunch for `/udp_diag`, relaunch only for `teensy_link`
  (live tree via PYTHONPATH), browser reload for the GUI.
