---
title: "P1 — FW 11 CLOCK_DIAG uplink and interp-occupancy census: written, compiled, NOT flashed"
type: feature
date: 2026-08-11
status: resolved
phase: "bridge-temporal-trustworthiness P1"
related_plan: bridge-temporal-trustworthiness.md
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_base.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_base.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_sync_master.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/protocol.py
  - teensy_link/__init__.py
  - teensy_link/rpc_args.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/docs/choreography.md
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - tests/teensy_link/test_protocol_codec.py
  - tests/ros/test_teensy_bridge_node_clock_diag.py
  - tests/ros/test_launch_nodes.py
  - tests/firmware/test_udp_protocol_xlang.py
subsystem:
  - can
  - ros
  - config
tags:
  - performance
  - IPC
  - testing
---

# P1 — FW 11 `CLOCK_DIAG`: written, compiled, NOT flashed

FW 11 adds an additive **`CLOCK_DIAG`** uplink — `MsgType` **0x8F** (the last free id below `RPC_RESPONSE` = 144), 49 bytes,
`PROTOCOL_VERSION` **stays 5** (0x8D/0x8E precedent). Per anchor: `t_local_us` (pure-crystal x-axis), `jetson_wall_us` (the applied
anchor, raw — so the offset stays consumer-derived), `dt_local_us`, `rtt_us` (stored *before* `set_wall_anchor`, so happens-before
holds), `err_us` (the pre-slew diff, previously computed and discarded), `freq_ppb` (consecutive measurements, open-loop w.r.t. the
IIR), `anchor_seq` (loss-visible), `flags` (STEPPED / FIRST_ANCHOR / FREQ_VALID); plus the interp-occupancy census `interp_ticks`
(denominator), `recover_slew_ticks`, `extrap_ticks` (Mode 2) — cumulative ISR counters differenced at emit, never reader-cleared.

**Where it runs.** `task_time_sync`, once per anchor (~30 s; 500 ms fast-retry), deliberately **off** the RX drain — the net task
carries every SETPOINT, so the handler contract is fast/non-blocking. The diagnostics arithmetic (64-bit divide) sits outside the
servo critical section; `ClockAnchorSample` publish/read is whole-struct under its own PRIMASK window, so a torn sample is
impossible; `now_wall_us` is untouched; the 500 Hz ISR gains exactly three aligned-`u32` increments.

**Jetson decode ships in the same commit** (0x8D/0x8E precedent): `teensy_link` re-exports `ClockDiag`; the bridge node queues and
drains at 1 Hz onto **`/clock_diag`** (`DiagnosticStatus`), added to the launch record list — a topic, not `/link_status` KeyValues,
because these are independent per-anchor measurements for a multi-hour fit rather than cumulative counters (the `/profile` vs
`/bb/axis_estimates` split). An FW 10 board never sends 0x8F: the topic records empty, pinned by test.

**Not flashed, deliberately:** the board stays on FW 10 until after the S1 aged-bridge sitting — a flash is a reboot and would reset
the aged state. `FW_VERSION` 10→11 (with changelog) forces `EXPECTED_BRIDGE_FW_VERSION` 10→11 (mandatory, pinned by
`test_bridge_fw_version_xref`), so the operator-facing consequence is `/link_status` `bridge_fw_version` reading
`10 (SKEW — expected v11, proto 5)` for the entire pre-flash window, **S1 included** — audit-verified ADVISORY everywhere: log and
string surfaces only, consumed by no launch gate, arming precondition, RPC refusal or harness hard-fail; the toss libraries
partition by it, never refuse.

**Audit — one NOTE, fixed:** census reads were denominator-first, so an ISR tick interleaving the three reads could make a window
report `extrap_ticks > interp_ticks` (>100 % duty); numerators are now read first and the denominator last (the ratio can only round
down), mirrored in the init baseline, and the firmware was recompiled after the fix. **Deviations recorded:** no Mode-3
(velocity-decay) occupancy counter — the plan named only recover-slew and Mode 2, 4 bytes to add later if S1 analysis wants it; and
`interp_ticks` deliberately counts early-return ticks, making it a true tick census and therefore a starvation detector.

## Verification

`pio run -e teensy41` (run 2026-08-11): **SUCCESS** — build only, nothing flashed. `python -m pytest tests/teensy_link/ tests/ros/ -q`
(run 2026-08-11): **2139 passed in 214.01 s**. `python -m pytest tests/firmware/ -q` (run 2026-08-11): **399 passed in 219.79 s**.
Gate (`./run_tests.sh`, run 2026-08-11): **5015 passed in 222 s, RESULT: PASS.**
