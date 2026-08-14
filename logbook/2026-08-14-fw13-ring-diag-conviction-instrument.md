---
title: "FW 13 — the additive RING_DIAG uplink: true ring depth against the leaking `_available`, plus the /robot_state staleness gate"
type: feature
date: 2026-08-14
status: resolved
phase: "bridge-temporal-trustworthiness S3 prep"
related_plan: bridge-temporal-trustworthiness.md
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/gpio_poll.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/gpio_poll.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/lib/FlexCAN_T4/FlexCAN_T4.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/lib/FlexCAN_T4/FlexCAN_T4.tpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/lib/FlexCAN_T4/circular_buffer.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/lib/FlexCAN_T4/PROVENANCE.md
  - teensy_link/protocol.py
  - teensy_link/__init__.py
  - teensy_link/rpc_args.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/docs/choreography.md
  - ros_ws/docs/can-node-teensy-parity.md
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - tests/teensy_link/test_protocol_codec.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/ros/test_launch_nodes.py
  - tests/ros/test_teensy_bridge_node_coldstart.py
  - tests/ros/test_teensy_bridge_node_ring_diag.py
  - tests/ros/test_teensy_bridge_node_robot_state_freshness.py
subsystem:
  - can
  - ros
tags:
  - performance
  - IPC
  - testing
---

# FW 13 — the `RING_DIAG` conviction instrument

## Summary

FW 13 written + compiled (`pio run -e teensy41`, 2026-08-14, SUCCESS), **NOT flashed** — the operator's flash *starts* the S3 conviction soak (~3–4 h powered idle; protocol + decision rule in the plan § S3). Additive `RING_DIAG` uplink, `MsgType` **0x92**, 103 B, `PROTOCOL_VERSION` stays **5**.

## Implementation

Two vendored-library patches on the verbatim FlexCAN_T4 copy (vendor commit `fef2df5`; `PROVENANCE.md` § Local patches): **P1** a read-only ring probe (head/tail first, `_available` last — a concurrent push can only *under*-report the leak), exposed as `rxRingProbe`; **P2** FIFO overflow/warning counters at the ISR IFLAG bit-6/7 clear site (sole writer, exact; lower-bounded by the BUF5I-branch scope, documented). The `_available` defect itself is deliberately **not** fixed — measure first, FW 14 fixes against a number.

`RING_DIAG` emits at 1 Hz from `task_telem`: per-bus `true_depth` + `avail_reported` probed **immediately** after the drain loop (where `_available` is 0 by definition, so `true_depth` *is* the stranded leak and `true_depth − avail_reported` is the conviction number); `leak_hwm` + `true_depth_hwm` maximised per 1 kHz service tick, not per 1 Hz sample, over a `probe_ticks` denominator; FIFO overflow/warn counts; a wrap-aware delivery-lag integral (16-bit CAN capture stamps accumulated into a 64-bit arrival clock — reseed + counter + flag on decode gaps ≥50 ms against the 65.536 ms wrap, plus an audit-added physical-cap reseed at |lag| > 200 ms, since one ring lap is ~135 ms and beyond that is unwrap corruption, not physics); and SDO round-trip min/max/last/count wrapping the existing 50 Hz `gpio_poll` — the causal cross-check, because aged-minus-fresh RTT *is* the ring delay. Jetson side: `/ring_diag` `DiagnosticStatus` (queue → 1 Hz drain), WARN on leak ≥ 2, bagged; `EXPECTED_BRIDGE_FW_VERSION` 12→13.

The `/robot_state` honesty fix is Jetson-only (live with the next `colcon build` + relaunch, no flash needed): the 100 Hz timer republished a latest-wins telemetry latch with no staleness gate — the source of the ~97 % freeze artifact in the S2 bags. It now skips when the telemetry generation hasn't advanced **and** the link is up (the link-lost carve-out keeps the fatal-fault channel loud), **bounded at 50 consecutive skips ≈ 0.5 s** (audit finding: telemetry and heartbeats come from separate firmware tasks, so "link up" does not prove telemetry flows, and an unbounded gate could silence the orchestrator's fault-content channel in that double failure); skips counted on `/link_status` as `robot_state_stale_skips`. Consumer sweep found nothing cadence-sensitive: `trajectory_node`'s 500 ms arrival gate fails safe, and GUI quiescence counts `/leg_setpoint_echo`, which publishes *before* the gate (pinned by test).

Audit findings applied pre-commit: the suppression bound + its double-failure test; the WARN-threshold docstring's skew direction corrected (it claimed over-reporting — the probe *under*-reports, so a leak of 1 is already real and 2 is margin against the sole full-ring over-report edge); the refuted drain comment in `can_buses.cpp` annotated in place (the ±1-self-corrects claim marked REFUTED with a pointer to this entry, historical text kept verbatim); the lag physical-cap reseed; stale line refs in the docs replaced with anchor-text cites; the 0x92 wire-id collision note added to the lead-clamp draft's banner; and the "Jetson honesty fix" de-double-booked from FW 14's deliverables in four documents.

## Verification

`pio run -e teensy41` (run 2026-08-14, post-audit-fixes): SUCCESS. Scoped (`python -m pytest tests/teensy_link/ tests/ros/ tests/firmware/ -q`, run 2026-08-14): 2589 passed in 234.65 s (pre-audit-fix state; the audit-fix deltas re-verified scoped: 83 passed). Gate (`./run_tests.sh`, run 2026-08-14): **5125 passed in 224 s, RESULT: PASS.**

## Outcome

Instrumentation only — nothing in the bridge reads the new accumulators, so a wrong value here cannot move a leg. S3 begins at the operator's flash.
