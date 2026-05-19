---
title: Mocap — QTM connection logging (state-transition + throttled), stale qtm_host (mDNS), unbounded-connect bug
type: bugfix
date: 2026-05-18
status: resolved
phase: "standalone — hardware log hygiene"
related_plan: null
related_entries:
  - 2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation
files_changed:
  - ros_ws/src/jugglebot/jugglebot/mocap_interface.py
  - config/hardware_config.yaml
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - logbook/2026-05-18-mocap-qtm-connection-logging.md
  - logbook/INDEX.md
commits:
  - fe01ab6   # A — mocap_interface state-transition logging gate
  - b344228   # B — config qtm_host → VR-Computer.local (mDNS)
  - 07a4253   # C — bound qtm_rt.connect + recurring throttled WARNING
subsystem:
  - ros
  - tracking
  - mocap
tags:
  - bugfix
  - logging
  - mocap
  - qtm
  - networking
  - mdns
  - dhcp
  - asyncio
  - probe-fidelity
---

# Mocap — QTM connection logging, stale qtm_host, unbounded-connect bug

## Summary

When the QTM motion-capture system was off, `mocap_node` emitted a
continuous stream of WARNING/ERROR lines about failed connections.
Goal: handle a missing mocap gracefully (mocap isn't always on), with
automatic resync if QTM starts mid-session, **and** a reliable, ongoing
declaration that mocap is not connected.

The investigation uncovered **three stacked causes**, fixed in order:

1. **Log spam (the reported symptom).** The `connect()` retry loop
   logged a WARNING on *every* attempt forever; and — found only by
   probe — the third-party `qtm_rt` library logged its *own* ERROR on
   its own logger per attempt, independent of `self.logger`.
2. **Stale `qtm_host` (why it was *continuous*).** `qtm_host` was
   `192.168.20.16`, a dead DHCP address; the QTM PC had drifted to
   `.21`. The node was *permanently* in the failed-connect path.
3. **Unbounded TCP connect (why it logged *nothing* in production).**
   `qtm_rt.connect()`'s `timeout=` does **not** cover socket
   establishment. Once B made `qtm_host` resolve to a *live* host whose
   QTM port was down, `connect()` wedged in kernel SYN-retry for
   minutes — the retry/except path (and hence *all* of fix A's logging)
   was never reached.

Fixes: **A** (`fe01ab6`) state-transition logging + gate the `qtm_rt`
library logger by connection state. **B** (`b344228`) point `qtm_host`
at the QTM PC's mDNS name `VR-Computer.local`. **C** (`07a4253`) wrap
the attempt in `asyncio.wait_for(_CONNECT_TIMEOUT_S=6s)` and replace the
one-shot WARNING with an rclpy-throttled recurring WARNING
(`_OUTAGE_WARN_THROTTLE_S=30s`).

## Symptom

`mocap_node` under `ros2 launch` continuously printed connection
failures when QTM was off: a burst of ~4 WARNINGs in ~17 s, then one
every ~35 s, plus a `qtm_rt - ERROR - [Errno 113] Connect call failed`
per attempt. **After fixes A+B**, the user reported the opposite
extreme — the real node logged **nothing at all** about the missing
connection. That report (not a probe) is what surfaced cause 3.

## Investigation

**1. Located the loop.** `connect()` is a `while True` with
`qtm_rt.connect(..., timeout=5.0)`; each failed attempt logged a WARNING
and slept `2→5→10→30…`s. Downstream was already graceful
(`_publish_mocap_data` returns when `not is_receiving()`), so the fix
was contained to `mocap_interface.py`.

**2. Probe overturned the first fix.** Assumption: the `except
(asyncio.TimeoutError, ConnectionError, OSError)` clause was the noise
path. `/tmp/probe_qtm_raw.py` against the unreachable host showed
`qtm_rt.connect()` does **not raise** — it catches the OSError
internally (`qrt.py:359-361`), logs `LOG.error(...)` on the named
`"qtm_rt"` logger, and **returns `None`**. So a separate noise source
existed; fix A extended to gate the `qtm_rt` logger by connection state.

**3. Network probe → stale IP.** `[Errno 113]` = `EHOSTUNREACH`; on a
local /24 that means ARP got no reply. `192.168.20.16` never resolved
while neighbour `.17` did; a subnet sweep found 18 hosts but not `.16`;
a port scan found QTM RT (22223) open on **`192.168.20.21`**. The
configured IP was a stale DHCP lease.

**4. mDNS.** `avahi-resolve -a .21` → `VR-Computer.local`, resolvable
via the system NSS path (`nsswitch` has `mdns4_minimal`).
`qtm_rt.connect("VR-Computer.local")` connected and streamed
end-to-end. A name is DHCP-proof with zero router config (fix B).

**5. Real-node test → cause 3.** The user ran the *deployed*
`mocap_node` with QTM down and saw nothing for the whole session.
Reproduced exactly: 45 s of the real node → zero QTM lines. A targeted
probe (`/tmp/probe_why_silent.py`) showed
`qtm_rt.connect("VR-Computer.local", timeout=5.0)` with the QTM port
down **hangs > 30 s** (had to be cancelled by an outer `wait_for`) —
its `timeout=` covers only the QRT protocol handshake, not
`loop.create_connection`. A second probe confirmed rclpy logs fine from
the asyncio background thread (so logging was never the issue). Fix C
bounds the attempt and makes the declaration recurring.

## Fix

**A — `mocap_interface.py` (`fe01ab6`).** State-transition logging:
one WARNING per outage transition, one INFO on (re)connect; the
`qtm_rt` library logger set to `CRITICAL` while in an outage (and
pre-emptively at `__init__`) and restored on connect. Auto-reconnect
and param-refresh-on-mismatch unchanged (mid-session QTM start still
auto-syncs).

**B — `config/hardware_config.yaml` (`b344228`).** `qtm_host`
`192.168.20.16` → `VR-Computer.local`; regenerated the four config
consumers (`git diff` showed the `QTM_HOST` line only).

**C — `mocap_interface.py` (`07a4253`).**

- `await asyncio.wait_for(qtm_rt.connect(...), timeout=_CONNECT_TIMEOUT_S=6.0)`
  — the failure path is now reached deterministically; a wedged TCP
  connect raises `asyncio.TimeoutError` (already in the `except`).
- The per-attempt WARNING is now **rclpy-throttled**
  (`throttle_duration_sec=_OUTAGE_WARN_THROTTLE_S=30.0`): logs
  immediately on the first failure, then ≤ once/30 s while down — a
  reliable ongoing declaration, not a single startup line. The
  vestigial `_conn_outage_logged` flag was removed (throttling
  subsumes its role; one fewer cross-thread state flag).
- `str(asyncio.TimeoutError)` is `""` → fall back to the type name so
  the line stays informative.

## Verification

Probes on the Jetson against the real QTM host (`2026-05-18`,
throwaway `/tmp/probe_*.py`, not committed):

| Scenario | Result |
|----------|--------|
| `qtm_rt.connect(VR-Computer.local)` with QTM port down | **hangs > 30 s** (pre-C — root cause of "logs nothing") |
| Real `mocap_node`, QTM down, post-A+B (45 s) | **0 lines** — confirmed the user's report |
| Real `mocap_node`, QTM down, **post-C** (95 s) | WARNING at ~6 s, then recurs ~every 35 s; no task pile-up, no dup disconnects |
| QTM-off → `qtm_rt` library logger | 0 records emitted (suppressed) |
| Connected-idle, no session (72 s) | 1 INFO, 0 WARNING/ERROR, `qtm_rt` restored to loud |
| Session started mid-connection | `marker_dict` 0 → 16, `is_receiving` False → True, 0 spam over 96 s |

Full suite — `pytest tests/ -q`, ci-fast default profile:
- A+B tree (run 2026-05-18): **1407 passed, 1 xfailed in 423.35 s** —
  clean, no regressions.
- C tree (run 2026-05-18): **1417 passed, 1 xfailed, 1 failed in
  432.04 s**. The single failure —
  `tests/motion/test_motor_guard.py::test_normal_interpolation_unchanged`
  — **passes standalone** (`1 passed in 0.35 s`) and is outside this
  change's scope (`mocap_interface` is not imported by any test;
  `motor_guard` ≠ QTM). Attributed to a concurrent parallel Claude
  session sharing the workspace (active `motor_guard`/`hardware_plant`
  edits + its own pytest run); not caused by this change. The
  authoritative test for ROS2 code the suite cannot exercise is the
  real-node run above.

## Discussion

**Why state-transition + throttled-liveness logging.** The user offered
"require QTM before ROS2 starts" as acceptable; rejected on root-cause
grounds — a hard startup requirement adds a failure mode (ROS2 won't
come up) and removes the mid-session auto-sync, which already existed
and only needed to be quiet. The contract evolved across the
investigation: A said "log on state transition only" (→ silent after
one line); the user's "I saw nothing" showed that a single line is
operationally invisible even when it *does* fire. C refined the
contract to **state-transition events + a throttled liveness
declaration while in the failed state** — quiet, but never silent about
a degraded state. That is the durable invariant.

**Three hypotheses were reframed mid-investigation** (the point of this
section):

1. *"The `except` clause is the noise path."* — Withdrawn. `qtm_rt`
   swallows the OSError and spams its own logger; the `except`-only fix
   would have shipped half the noise.
2. *"QTM doesn't stream until a session opens."* (user) — Correct in
   general, but not the operative blocker: that manifests as `connect()`
   *succeeding* with `is_receiving()==False`; we were failing two layers
   below at ARP/L3 (stale IP). Later confirmed directly
   (connected-idle: `is_receiving=False`, 4 bodies, 0 markers).
3. *"Fix A is validated; the user just missed the one line."* —
   **Withdrawn, and the most important one.** Probes (fake logger,
   `node=None`, explicit fast-failing host) showed exactly 1 WARNING and
   I believed A was production-correct. The user's "I saw NOTHING" was
   ground truth that overturned it: in production the line never fired
   because `qtm_rt.connect()` was wedged *before* the except path. The
   confidence of the probe was not evidence of correctness.

**Probe-fidelity lesson.** The probes failed to catch cause 3 because
their failure *topology* did not match production. Two gaps compounded:
(a) they ran with `node=None` and an explicitly-passed host, not the
real rclpy node + config; (b) more decisively, every probe host was
either a *dead IP* (fast `EHOSTUNREACH`, ~3 s) or QTM-*up* — none
exercised the case fix B newly created: a **resolvable, live host with
a dead QTM port**, where TCP connect hangs for minutes. The lesson:
when a fix changes the failure *mode* (B turned "host unreachable" into
"host reachable, service down"), prior probes are no longer
representative — re-probe the *new* topology, and treat a real-system
report as authoritative over a green probe.

**Contract-level takeaway.** Never trust a third-party library's
`timeout=` to cover the socket layer — bound external connects yourself
(`asyncio.wait_for`). `qtm_rt`'s `timeout=` looked like it covered
connect; it did not. This is the same class as "climb one level": the
fix is not "increase the timeout" but the invariant *every external
connect attempt is wall-clock bounded by us*.

**Host addressing — the non-obvious tradeoff.** (a) hardcode `.21` —
reproduces the bug on the next DHCP lease; (b) router DHCP reservation
— the user's goal, but unreliable for them (wrong NIC MAC, lease not
renewed, in/out of pool); (c) static IP on the QTM PC's NIC — robust
but forgettable manual setup; (d) **mDNS hostname** — router-
independent, zero config, already resolvable, survives DHCP drift.
(d) chosen: it *eliminates the failure class* (stale address) at least
operator burden. Residual deps: `avahi-daemon` + same L2 segment (both
confirmed); if mDNS hiccups, A+C now fail gracefully and self-heal.

## Open Questions

- **Pre-existing shutdown noise (deferred follow-up).**
  `MocapInterface.stop()` halts the asyncio loop while `connect()` may
  be mid-retry-sleep, producing one `asyncio - ERROR - Task was
  destroyed but it is pending!` line. Pre-existing, cosmetic
  (process-exit only), not runtime spam. Correct fix is cross-thread
  asyncio task cancellation in `stop()` with probe verification —
  scoped out to avoid lifecycle risk. Tracked for a separate session.

## Related

- Deferred shutdown item ties to `mocap_interface.py:stop()` /
  `_on_qtm_disconnect()`.
- Logged per `/log`; commits carry
  `Logbook-Entry: 2026-05-18-mocap-qtm-connection-logging`.
