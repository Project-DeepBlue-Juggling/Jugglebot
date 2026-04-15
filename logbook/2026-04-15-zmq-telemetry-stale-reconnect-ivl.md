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

## Verification

- 895 tests pass (`pytest tests/ -v`)
- Hardware verification pending: need to run `ros2 launch` and verify `actual_ext` updates in MPC CSV

## Outcome

Defense-in-depth: even if RECONNECT_IVL fails to prevent stale telemetry for a new reason, the platform will E-stop at 0.5s rather than running away with diverging commands.
