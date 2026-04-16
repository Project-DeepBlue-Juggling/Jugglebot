---
title: "ZMQ telemetry stale on :5556 — RECONNECT_IVL fix and staleness E-STOP"
type: bugfix
date: 2026-04-15
status: resolved
phase: ""
related_plan: ""
related_issues: []
sessions:
  - mpc_20260415_144916.csv
  - mpc_20260415_192410.csv
files_changed:
  - controller/hardware_plant.py
  - ros_ws/src/jugglebot/jugglebot/motion/ipc.py
commits: []
subsystem:
  - motion
  - controller
tags:
  - IPC
  - safety
---

## Summary

Two hardware sessions on 2026-04-15 suffered frozen MPC telemetry feedback due to missing ZMQ RECONNECT_IVL on :5556 SUB sockets. The MPC saw stale position data and ramped commands upward, causing dangerous platform oscillation. Fixed by adding RECONNECT_IVL/RECONNECT_IVL_MAX to all :5556 SUB sockets and adding a defense-in-depth telemetry staleness E-STOP.

## Problem

After the RECONNECT_IVL fix was applied to motor guard SUB sockets on :5557, two SUB sockets on :5556 (telemetry channel) were missed:

1. `HardwarePlant._sub` (`controller/hardware_plant.py`) — MPC's telemetry receiver
2. `BridgeIPC._sub` (`ipc.py`) — ROS2 bridge's telemetry receiver

Without RECONNECT_IVL_MAX, ZMQ's default exponential backoff grows from 100ms to 30s max. After any transient disconnect on :5556, the subscriber waited up to 30 seconds to reconnect, leaving telemetry frozen.

The existing staleness detection (`_TELEM_STALE_HARD_S = 0.125s`) zeroed velocities but did NOT trigger an E-stop. With frozen position feedback and zeroed velocities, the MPC still saw tracking error and kept ramping commands — the motor guard faithfully executed these diverging commands, causing physical platform oscillation.

## Root Cause

Incomplete application of the RECONNECT_IVL fix across all ZMQ SUB sockets. The fix was applied to :5557 (MPC command channel) but not to :5556 (telemetry channel). Additionally, the staleness detection lacked an E-stop for the case where telemetry is completely lost.

## Fix

1. Added `RECONNECT_IVL=100ms, RECONNECT_IVL_MAX=200ms` to HardwarePlant SUB on :5556
2. Added `RECONNECT_IVL=100ms, RECONNECT_IVL_MAX=200ms` to BridgeIPC SUB on :5556
3. Added `_TELEM_STALE_ESTOP_S = 0.5s` (20 MPC cycles) threshold — triggers E-STOP when telemetry is completely lost, preventing runaway commands

Additional changes in the same diff (applied before this session):

- ZMQ 4.3.5 CONFLATE+topic-filter bug workaround: changed SUBSCRIBE to `b''` on late-connecting SUBs
- FK cold-start: skip FK E-STOP until first successful convergence (`_fk_ever_succeeded` flag)
- HardwarePlant.enable() two-phase handshake: verify both telemetry and command channels before starting MPC

## Follow-up: Stale telemetry was actually a CONFLATE bug (2026-04-16)

Hardware verification on 2026-04-16 revealed the RECONNECT_IVL fix alone was
insufficient. A new `--pose 0,0,190` run failed with `HardwarePlant: motor guard
did not acknowledge probe command within 2.0s`. The rosbag showed motor_guard
was publishing 941 `leg_lengths_topic` messages during the probe window, meaning
the telemetry (with `leg_pos` set) was flowing — HardwarePlant just wasn't
seeing it.

Root cause: `CONFLATE=1` was set **after** `socket.connect()`, which ZMQ
silently ignores. This made the SUB a FIFO queue with default HWM=1000. At
500 Hz publish rate and 40–100 Hz poll rate in HardwarePlant, messages piled
up and we read from the back of the queue — always stale.

This is also what caused the `mpc_20260415_192410.csv` oscillation: the
"frozen" `actual_ext` at 154.5mm was actually old telemetry from before the
platform started moving.  The `motion_bridge_node` polls at 500 Hz so it
happened to keep up despite the same bug.

### Follow-up fix

- Drain the SUB in a `while`-loop in `HardwarePlant.get_state()` and
  `BridgeIPC.recv_telemetry()`, keeping only the latest message — robust
  regardless of CONFLATE semantics.
- Remove `CONFLATE=1` and use `RCVHWM=2` instead for bounded memory.
- Explicit drain also avoids the libzmq issue where CONFLATE + multi-part
  messages can drop the topic frame on some versions.

## Verification

- 895 tests pass (`pytest tests/ -v`)
- Hardware verification: next hardware run should succeed past `plant.enable()`
  and `actual_ext` in the MPC CSV should track commanded extensions.

## Outcome

Defense-in-depth:
- Explicit drain eliminates CONFLATE semantics as a failure mode.
- RECONNECT_IVL cap prevents slow reconnect from freezing telemetry.
- `_TELEM_STALE_ESTOP_S` (0.5s) triggers E-STOP if telemetry is ever
  completely lost, preventing runaway commands.
