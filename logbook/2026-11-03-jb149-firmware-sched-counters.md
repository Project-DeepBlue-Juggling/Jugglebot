---
title: "JB-149: can-bridge FW 38 / PROTOCOL 11 uplinks the scheduled-lane counters and a per-heartbeat frame-age census, and the STOP WARN names a firmware-side cover expiry (built and tested, NOT flashed)"
type: feature
date: 2026-11-03
status: in-progress
phase: "JB-149 firmware-side counters (owner decision via JB-155)"
subsystem: can-bridge
related_entries:
  - 2026-11-03-jb145-hand-stall-stop-options.md
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - teensy_link/gap_monitor.py
  - teensy_link/rpc_args.py
  - tests/firmware/native/test_leg_interp.cpp
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/teensy_link/test_protocol_codec.py
  - tests/teensy_link/test_hand_move_to.py
  - tests/teensy_link/test_v5_wire_regression.py
  - tests/ros/test_teensy_bridge_node_bridge_diag.py
  - tests/ros/test_teensy_bridge_node_read.py
  - logbook/INDEX.md
---

# JB-149: firmware-side scheduled-lane counters on the wire

**Why.** On 2026-11-03 both LANE_REFUSED ends happened with a clean host stream (forward
gap 41.7 / 42.1 ms, emitter max 38.9 ms, 4 setpoints per 100 ms), while the one real
81.6 ms forward gap STOPped without a refusal. The firmware measures the cover from the
frame's stamp (`t_start`), so a delay between the bridge's send and the firmware's
promotion eats the cover with no gap on the host. The counters that would show it
(`s_sched_expired` and friends) were serial-only. The owner chose (JB-155) to build both a
host probe (already landed) and these firmware counters.

**Wire change (incompatible): FW 37 -> 38, PROTOCOL_VERSION 10 -> 11.** `HeartbeatT2J`
grows 126 -> 158 B, appended after `sched_demoted`:

- cumulative since boot: `sched_superseded`, `sched_stale`, `sched_hand_c1_stops` (the FW 37
  C1 stop, a subset of `sched_stops`), `sched_future_drops`, `sched_q_overflow`
  (`sched_stops/expired/refused/demoted` were already there);
- per heartbeat window (reset at every heartbeat): `rx_age_max_us` (i32, INT32_MIN when no
  stamped frame arrived) and `rx_age_le25 / le50 / le70 / gt70` (u16, saturating), with
  rx_age = `micros64()` - the frame's `t_start` at queue entry in the net task
  (`interp_on_setpoint`, inside the queue's existing IRQ-off section, O(1), off the 500 Hz
  ISR). The edges map onto the firmware's own rules: an age past 50 ms (the 2T cover) is
  dropped at promotion as `sched_expired`; past 70 ms (cover + grace, `static_assert`ed)
  the playing frame had already run out at arrival. The heartbeat task reads and resets
  the window through `interp_sched_rx_age_take`, under PRIMASK.

A 126 B heartbeat read as 158 B would misread every counter, so darkness against an
FW <= 37 board is the intended failure; `EXPECTED_BRIDGE_FW_VERSION` = 38.

**Jetson side.** teensy_link decodes the fields through the generated module. `/link_status`
(a `DiagnosticStatus`, no message change) gains `sched_expired`, `sched_demoted`,
`sched_superseded`, `sched_stale`, `sched_hand_c1_stops`, `sched_future_drops`,
`sched_q_overflow`, `rx_age_max_us` (`none` for an empty window) and the four `rx_age_*`
buckets. The bridge node keeps the last 10 heartbeat windows (~1 s) for the STOP WARN.

**STOP WARN, third verdict.** `host stream clean, cover expired firmware-side` is chosen
when the forward-gap tracker forwarded at least one frame in its 1 s window and its max,
its OPEN gap (time since the last forward, which `snapshot` cannot see until the gap
closes) and the loop-gap max are all below the 70 ms cover. Otherwise the two host
candidates print as before (now with the open gap). Every WARN appends
`fw_sched last N heartbeats: expired +a superseded +b stale +c refused +d hand_c1_stops +e
future_drops +f q_overflow +g; frame age at bridge receipt max X ms, <=25 ms n, <=50 ms n,
<=70 ms n, >70 ms n`. The WARN also fires once per disarm when the STOP rise lands within
0.5 s of `mpc_active` falling 1 -> 0 (`at the disarm edge (mpc_active 1->0)`, past the
1 s rate limit): the host flag falls a heartbeat or two before the firmware reports the
stop, so "the same sample" alone would miss most of them. One `warning` call site.

**Operator grep after a Sitting** (launch.log):

- `grep -n 'firmware scheduler STOP' launch.log` - every STOP WARN;
- `grep -n 'host stream clean, cover expired firmware-side' launch.log` - the third verdict;
- `grep -n 'at the disarm edge' launch.log` - the disarm-edge STOPs;
- `grep -n 'fw_sched last' launch.log` - the firmware counters and frame ages of each WARN.

In a bag, `/link_status` keys `rx_age_max_us`, `rx_age_gt70`, `sched_expired`.

**Reading it.** Large frame ages with a clean host = the link or the net task delays frames;
small ages with expiries or STOPs = the stamps or the ISR, not the link.

## Verification

- (2026-11-03, `OMP_NUM_THREADS=1 nice -n 10 python -m pytest -q -p no:cacheprovider tests/teensy_link/ tests/firmware/test_bridge_fw_version_xref.py tests/firmware/test_udp_protocol_xlang.py tests/ros/`, result: **1 failed, 4075 passed, 2 skipped in 1017.68 s**; the one failure, `tests/ros/test_skill_node.py::test_the_tick_dispatches_the_opening_rest_bridge_through_the_mocked_client`, passed on a rerun by itself (`... pytest -q -p no:cacheprovider <that node id>`, **1 passed in 1.24 s**). skill_node does not import the code this change touched, and the box was carrying a peer's sweep, so this reads as load.)
- (2026-11-03, `OMP_NUM_THREADS=1 nice -n 10 python -m pytest -q -p no:cacheprovider tests/firmware/test_native_firmware.py`, result: **17 passed in 2022.52 s**, including the new `FW 38 frame-age census` case in `test_leg_interp.cpp`.)
- (2026-11-03, `pio run -e teensy41` in `ros_ws/src/jugglebot/Teensy_code_canbridge/`, result: `teensy41 SUCCESS 00:00:08.746`, an incremental build after the last edit; the first full build was `SUCCESS 00:01:47.157`; no warnings in the files this change touched.) NOT flashed.
- Wire layout digest re-pinned in `test_udp_protocol_xlang.py::test_wire_layout_frozen` (`e184b76e...`).

**Owed:** flash FW 38 (`pio run -e teensy41 -t upload`, launch down) together with a
`colcon build` of this tree (PROTOCOL 11 on both ends or the link is dark), check the
BridgeIdentity receipt (`38 (proto 11)`), then one Sitting to read the counters.

**Flashed 2026-11-03 ~18:05** (session jugglebot-50, launch down, from the pushed tip 341950b5): `pio run -e teensy41 -t upload` -> SUCCESS 80.17 s; receipt: a read-only listener on PORT_STREAM saw 8 BridgeIdentity frames in 8 s, all `(fw_version=38, protocol_version=11)`, every frame decoding at PROTOCOL 11. The live checkout was rebuilt (`colcon build --packages-select jugglebot`, 2026-11-03, 1 package finished). The bench-hand Sitting row on JB-145 and the robot Sitting row on JB-149 remain the behaviour checks.
