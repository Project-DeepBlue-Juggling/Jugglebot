---
title: "UDP loopback test flake: SO_REUSEADDR let parallel tests share an ephemeral port, and the node's own heartbeat raced an exact TX count"
type: bugfix
date: 2026-09-30
status: resolved
files_changed:
  - teensy_link/client.py
  - tests/teensy_link/conftest.py
  - tests/teensy_link/test_client.py
  - tests/ros/test_teensy_bridge_node_udp_diag.py
subsystem:
  - can
  - testing
tags:
  - testing
---

# UDP loopback test flake

## Problem

On 2026-09-30, gate run 1 of `2026-09-30-operator-console-phase-3` failed two bridge tests
against the loopback FakeTeensy:
- `test_udp_diag_counts_tx_by_type`, with `3 == 0 + 2`;
- `test_recover_hand_park_is_a_noop_when_the_hand_is_already_parked`, with
  `CLEAR_ERRORS: ERR_UNKNOWN_METHOD`.

The first had failed the same way on 09-27 and 09-28, and both passed on a rerun. That entry
guessed that the two tests' fake Teensies were exchanging packets under xdist. That guess was
only half right: there were two separate causes.

## Discussion

**The RPC failure was a shared port.** The recover test registers a `CLEAR_ERRORS` handler on
its own FakeTeensy before it calls, so an `ERR_UNKNOWN_METHOD` reply can only come from a
different FakeTeensy. Every loopback socket binds port 0, which should make ports unique. But
`TeensyLinkClient` and `FakeTeensy` set `SO_REUSEADDR` on every socket. On Linux, a UDP
`bind(0)` with that option may be given a port another `SO_REUSEADDR` socket still holds, and
the newer socket then receives all of that port's datagrams.

A probe on the Jetson (2026-09-30) bound 900 sockets to `('127.0.0.1', 0)`, three trials each
way. With the option set, 8–15 ports were shared in each trial, which matches random choice
from the 28k-port range. Without it, none were. A datagram sent to a shared port always
reached the newer socket.

**The TX-count failure was not cross-talk.** A socket's TX count is only incremented when that
socket sends. The node starts its own 10 Hz heartbeat thread
(`teensy_bridge_node.py`, `start_heartbeat`). One beat landed between the test's two reads.
`before == 0` shows the thread's first beat had not gone out yet.

## Fix

- `teensy_link/client.py`: new `_reuse_fixed_port` sets `SO_REUSEADDR` only on a fixed port.
  `FakeTeensy` does the same.
  - Production binds 5005/5006 and is unchanged, as are the hardware bench scripts, which also
    bind those ports.
  - An ephemeral port is never shared, and the kernel now guarantees it is unique.
- `test_udp_diag_counts_tx_by_type` counts SETPOINT exactly. Nothing sends SETPOINT unprompted
  in that test: the setpoint thread only sends a command, over a live link. HEARTBEAT_J2T is
  checked with `>=`.
- `tests/teensy_link/test_client.py`:
  - No ephemeral client or FakeTeensy socket has the option.
  - The helper keeps it on 5005/5006.
  - Both tests fail on the old code: `assert 1 == 0`, and an ImportError for the missing
    helper.

## Verification

- Scoped, 2026-09-30: `pytest tests/teensy_link/ tests/ros/test_teensy_bridge_node_udp_diag.py
  tests/ros/test_teensy_bridge_node_recover.py -q`: **478 passed**.
- Gate, `./run_tests.sh`, 2026-09-30, ending 10:04: **PASS, 5669 passed, 9 skipped; serial 3
  passed** (`temp/logs/gate_udp_reuse_fix_20260930.log`). One clean run cannot prove a rare
  flake gone. The evidence is the mechanism: both causes are traced, and both regression
  tests are red on the old code.

## Notes

- The production socket still sets `SO_REUSEADDR` on 5005/5006. So a second process binding
  those ports, such as a probe run while the launch is up, silently takes the bridge's traffic
  instead of failing with EADDRINUSE. This is the "single-owner UDP hazard" in the bench notes.
  Dropping the option there would make that mistake fail loudly. That is a production behaviour
  change and was not made here.
