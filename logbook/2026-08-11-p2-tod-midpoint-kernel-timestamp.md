---
title: "P2 — the TOD responder returns the midpoint of the kernel RX stamp and a just-before-send stamp"
type: feature
date: 2026-08-11
status: resolved
phase: "bridge-temporal-trustworthiness P2"
related_plan: bridge-temporal-trustworthiness.md
files_changed:
  - teensy_link/client.py
  - teensy_link/rpc.py
  - teensy_link/tod_server.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/teensy_link/test_rx_timestamping.py
  - tests/ros/test_teensy_bridge_node_read.py
subsystem:
  - can
  - ros
tags:
  - performance
  - IPC
  - testing
---

# P2 — midpoint TOD stamp (kernel RX + just-before-send)

The Jetson-side TOD responder now returns **`(t2 + t3) // 2`** in the existing
`jetson_wall_us` field: `t2` is the **kernel** RX timestamp (`SO_TIMESTAMPNS`
ancillary data, numeric constant 35 with a runtime probe), `t3` a userspace stamp
taken just before send. Wire-compatible — no firmware change, no
`PROTOCOL_VERSION` bump.

**Why the midpoint, not kernel-RX alone.** The midpoint cancels the Jetson
scheduling-jitter term `p` *exactly*. Kernel-RX-only stamping — the clock plan's
original sketch — would merely have flipped its sign (`+p/2` → `−p/2`), trading
one bias for another. What is left is pure path asymmetry `(df − dr)/2`,
addressed later by min-RTT gating (arc P4).

**Transport.** `teensy_link/client.py` arms the **RPC socket only** (the stream
path is untouched); `recvmsg` with a 32-byte control buffer; a three-state
`rx_stamp_mode` `'probing'` → `'kernel'` | `'userspace'`. The mode is confirmed
by a real packet carrying a parseable stamp, **not** by `setsockopt` succeeding,
and the degrade latches behind a one-shot WARNING. Plumbing is opt-in —
`subscribe(..., with_rx_stamp=True)` / `register(..., wants_rx_stamp=True)` — so
all ~40 existing 4-arg callers are unchanged (audited).

**Observable fallback.** `tod_server.py` carries a per-query `stamp_mode`
(`'unknown'` | `'kernel-midpoint'` | `'userspace'`) that recovers on the next
good stamp — deliberately *not* the client-side latch, so a transient NTP step
doesn't condemn the session — plus a plausibility guard (`0 < t2 <= t3`,
delta ≤ 1 s) with an `implausible_rx_stamps` counter. Both reach the bagged
`/link_status` as `tod_stamp_mode` and `tod_implausible_rx_stamps`: a silent
fallback would reinstate `+p/2` invisibly, the exact "invisible until degraded"
class this arc exists to close.

**Audit findings fixed pre-commit.** (1) WARNING — `t3` was read at handler
entry while the plan says just-before-send; restructured so the guard runs on a
provisional read and the final `t3` is re-read after all bookkeeping and
logging, leaving only pack + encode + `sendto` (~tens of µs) uncancelled, with
an honest comment stating the residual. (2) The userspace→kernel *recovery*
direction was implemented but unpinned — the degrade test now drives it. (3) The
rejection counter was not bag-visible — KeyValue added.

**Empirical.** `SO_TIMESTAMPNS` probed live on this box 2026-08-11 (kernel
5.10.104-tegra, both system 3.8.10 and the venv): cmsg (level=1, type=35,
16-byte timespec) tracking `time.time()` to ~100 µs.

**Deployment.** `teensy_link/` is live at the next relaunch (repo-root
`PYTHONPATH`); the bridge-node `/link_status` keys need
`colcon build --packages-select jugglebot` + relaunch. Both must be live before
the S1 aging window.

## Verification

`tests/teensy_link/` + the bridge-node read tests
(`python -m pytest tests/teensy_link/ tests/ros/test_teensy_bridge_node_read.py -q`,
run 2026-08-11): **251 passed in 15.51 s** — including the new
`tests/teensy_link/test_rx_timestamping.py` (19 tests). Gate
(`./run_tests.sh`, run 2026-08-11): **5000 passed in 221 s, RESULT: PASS.**
