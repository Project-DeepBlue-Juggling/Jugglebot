---
title: S3 — the ring leak MEASURED, at 97 % of a lap, and the stale-install near-miss that nearly cost the conviction
type: investigation
date: 2026-08-14
status: resolved
phase: "bridge-temporal-trustworthiness S3"
related_plan: bridge-temporal-trustworthiness.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_teensy_bridge_node_install_skew.py
subsystem:
  - can
tags:
  - performance
  - testing
---

# S3 — the leak, measured

## Summary

FW 13's `RING_DIAG` answered the S3 decision rule, and it answered it at the
ceiling. On a bridge aged **4.01–4.04 h**, the jugglebot bus reports
`true_depth_jb` **247–248** slots stranded against `avail_reported_jb` **0** —
i.e. **`leak_jb = 247–248`**, high-water **249**, or **≈ 97 % of the 256-slot
ring lap** — while the BallButler bus leaks **1** and the cone bus **0**, exactly
the ordering the collision-rate-scales-with-traffic prediction requires.
`fifo_overflows = 0` on every bus: nothing was dropped by the peripheral. The
loss is **pure software-ring stranding**, and the RX ring is a **delay line**, as
the 2026-08-14 audit predicted.

The 2026-07-18 arc's root cause is therefore **established**: *one missing IRQ
guard around the vendored FlexCAN_T4 ring pop.*

The conviction was very nearly missed. The first pass' bags contained **no
`/ring_diag` at all** — the ROS node was running a **stale `install/` tree dated
2026-08-12** with zero `ring_diag` references, while `BRIDGE_FW_CHECK` printed
**OK** the entire time. That check was not broken; it is **structurally blind to
install skew**, and closing that class is this entry's code product.

## Symptoms

Two FW 13 sessions on 2026-08-14, the operator-visible half unchanged since
2026-07-16:

| | Session 1 (fresh) | Session 2 (aged) |
|---|---|---|
| Bridge uptime | **17 s** | **3.80 h** |
| Operator verdict | "silky" | "janky" |
| End-to-end lag | **19.9 ms** | **252.2 ms** |
| Lead-clamp duty | **0.0000** | **0.4588** |

Neither session produced the conviction number: the node was stale, so the
0x92 frames it did not subscribe to never reached a topic, and both bags carry
zero `/ring_diag` samples.

## Diagnosis

### 1. The recovery — aged state is not lost to a rebuild

`colcon build` + relaunch **does not reboot the Teensy**. The bridge's uptime
clock, and therefore the accrued leak, survives a host-side rebuild untouched —
so the aged plant did not have to be re-aged for hours. Rebuilt, relaunched, and
re-recorded the same afternoon: bag **`2026-08-14_18-18-59`**, bridge uptime
**4.01–4.04 h**, **92 `/ring_diag` samples**.

### 2. The conviction (bag `2026-08-14_18-18-59`, 92 samples)

| Quantity | Value | Reading |
|---|---|---|
| `leak_jb` (`true_depth_jb − avail_reported_jb`) | **247–248** | ≈ **97 %** of the 256-slot lap |
| `true_depth_jb` / `avail_reported_jb` | 247–248 / **0** | the ring is full; the counter says empty |
| `leak_hwm_jb` | **249** | the lap ceiling, in practice |
| `leak_bb` / `leak_cone` | **1** / **0** | scales with bus traffic, as predicted |
| `fifo_overflows` (all buses) | **0** | no peripheral loss — pure software stranding |
| `probe_ticks` | **1000** per window | the probe ran every service tick |
| Delivery-lag integral | **151–183 ms** | see residual (a) |
| `robot_state_stale_skips` | **29** | the Jetson honesty gate is live |

`avail_reported_jb = 0` is the whole mechanism in one number: the drain loop
exits believing the ring is empty **while 247 frames are stranded in it**. Every
delivery after that point is 247 frames late, and the high-water 249 sits just below the
structural cap.

### 3. Cross-checks — two independent numbers agree

- **echo→exec, aged − fresh = 115.4 ms**, inside the audit's predicted
  **114–135 ms** one-lap band. The lap ceiling was derived from the ring's
  mod-512 `head ^ 256` full test; the plant reproduces it.
- **Saturation arithmetic retro-explains the historical plateau.** 256 slots ÷
  ~90 slots/h ≈ **2.84 h to saturate**. That reframes the whole July curve:
  3.8 h → 252 ms, ≈ 28 h → 283 ms, ≈ 63 h → 290–340 ms is **not** a lag still
  climbing with uptime — it is a lag that **saturated by hour three** and then
  varied for other reasons. Everything past ~3 h has been measuring the same
  full ring.

## Discussion

### The near-miss: a green detector that could not see the question being asked

`BRIDGE_FW_CHECK` compared the board's reported `FW_VERSION` (**13**) against
`rpc_args.EXPECTED_BRIDGE_FW_VERSION` (**13**) and printed **OK**. Both halves of
that comparison were true, and the conclusion the operator drew from it — "the
system is running FW 13's instrumentation" — was false, because the constant
comes from the **live repo-root `teensy_link/` tree** (injected on PYTHONPATH by
the launch, deliberately never colcon-installed) while the *node* comes from
`install/`. The check reports **firmware currency**. Nothing reported **host
currency**, and the operator had no way to tell from the log that only one of
the two questions had been answered.

This is the same class as the S4 `vel_limit` incident ("a stale install keeps
pushing the old value") and as the `24608bb` stale-object flash. The launch file
already carries the sibling detector for the generated **config** modules
(`_install_drift`) — its scope is exactly `hardware_config.py` and
`protocol_config.py`, so a stale *node source* with unchanged config passes it
green, which is precisely what happened here.

The generalisable lesson, and the reason this is a contract rather than a patch:
**a currency check that reads its expectation from a different tree than the
artifact it certifies is not a currency check.** The fix is not to make
`BRIDGE_FW_CHECK` smarter — it is right about what it measures — but to give the
missing half its own detector, put both on one line, and make the verdict
**persistent in the bag** rather than a startup log line that `/rosout`
recording can race. The failure mode being closed is not "the operator was
careless"; it is "no artifact recorded which build produced these numbers", and
that is a property of the instrumentation, not the person.

### Why the residuals are recorded rather than resolved

Two second-order numbers do not tie out. Both are documented as **limitations of
the instrument** with a named reconciliation owner (the FW 14 validation pass)
rather than smoothed over, because the arc has already been burned four times by
measurements that described the instrument rather than the plant.

**(a) The delivery-lag integral's absolute value.** It reads **151–183 ms**
against a naive expectation of **129 ms** (leak ÷ 1,920 frames/s), and it
**creeps ~0.35 ms/s** across the bag. The known contributors are a boot-time
offset and **16 reseeds** during the window. The *trend* is the instrument; the
absolute value is not, and the creep itself is **unexplained**.

**(b) The SDO RTT floor reads BELOW the ring delay.** 46.9–47.9 ms, against a
ring delay of ~130 ms. The cause is **single-slot request-stamp mispairing under
pipelining**: at a 20 ms poll period and a ~130 ms delay there are **~6 replies
in flight**, so a one-slot request stamp is overwritten before its reply lands
and the measured "round trip" pairs a late reply with a recent request. That is
a **limitation, not a contradiction** — and note it *only occurs because* the
delay greatly exceeds the poll period, so the artifact is itself weak
confirmation of the mechanism. The probe's floor argument is only valid while
one request is in flight at a time.

## Fix

**The measurement's product is the conviction, not a firmware fix** — FW 13 is
instrumentation only, and the `_available` defect is deliberately still aboard
so FW 14 fixes against a number.

**The code shipped with this entry is the install-skew self-check** in
`teensy_bridge_node`, closing the class the near-miss exposed:

- At construction, before the RX thread exists, the node hashes **its own
  `__file__`** and compares it against the repo-source copy
  (`ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py`), located by walking up
  from the running file (with a `JUGGLEBOT_REPO` override, matching
  `jugglebot_launch._repo_root`). Content hash, not mtime — colcon copies the
  `.py` verbatim, and every build restamps mtimes.
- **Three verdicts, never collapsed:** `0` clean, `1` stale install, `unknown`
  the check could not run (no source tree, unreadable file). An unrunnable check
  must not render clean; that is the failure being closed.
- **Loud on skew:** a startup ERROR naming both paths and both mtimes (which side
  is behind is the difference between "run colcon build" and "your checkout is
  stale"), the fix command, and the note that `BRIDGE_FW_CHECK` cannot see this.
- **Persistent in the bag:** `/link_status` carries `install_skew` (bare token)
  and `install_skew_detail`, adjacent to `bridge_fw_version`, published
  unconditionally at 10 Hz. A bag can no longer silently record a stale node.
- **`BRIDGE_FW_CHECK` now names both halves on one line**, on the OK line too — a
  green firmware verdict beside a silent host verdict is the original trap's
  shape.
- **Advisory, never a gate**, identical policy to `BRIDGE_FW_CHECK` and the
  launch banner: a stale install is a confusion failure, not a dangerous one
  (build-frozen is the safe direction), and a hash comparison in front of the
  leg/hand command path would convert a reporting gap into an outage.

## Outcome

The 2026-07-18 arc has its root cause **established, not inferred**: one missing
IRQ guard around the vendored FlexCAN_T4 ring pop, measured at 97 % of a lap on
an aged plant, with three independent numbers (leak, one-lap echo→exec delta,
saturation time) agreeing.

Next, in order:

1. **FW 14 — the fix.** Guard the ring pop inside the existing
   `NVIC_DISABLE_IRQ` window (or make the `_available` updates atomic). **The
   leak counter stays aboard** — it is how the fix is proven, not scaffolding.
   **Acceptance: `leak ≡ 0` and end-to-end lag ≤ 20 ms on an AGED validation
   soak**, plus a reconciliation of residuals (a) and (b).
2. **The alarmed end-to-end latency monitor** (P3), whose alarm input is already
   calibrated (clamp duty + the 100 ms / 0.67 rev/s reporting threshold).

Only both together close `logbook/2026-07-18-teensy-uptime-tracking-degradation.md`
and its 2026-07-24 contract; that entry **stays `open`**.

## Verification

- **The durable record of the conviction is the bag stamp plus the sample
  count**: `2026-08-14_18-18-59`, bridge uptime **4.01–4.04 h**, **92
  `/ring_diag` samples**. The extraction script (`verdict_ring_diag.py`) lives in
  the session scratchpad and is **session-local** — the same `/tmp` volatility
  that destroyed rounds 1–2 of the ring audit applies, so every number quoted
  above is quoted here *because* the script will not survive. Anything that must
  outlive a session belongs in an entry or under `tools/probes/`.
- Sessions 1 and 2 (17 s / 3.80 h) are the operator-verdict and lag/clamp-duty
  pair; they carry **no** `/ring_diag` samples and are cited for nothing else.
- **Install-skew self-check:** `pytest tests/ros/test_teensy_bridge_node_install_skew.py -q`
  (run 2026-08-14): **16 passed in 2.34 s**. Scoped ROS surface:
  the scoped skew tests (`pytest tests/ros/test_teensy_bridge_node_install_skew.py -q`, run 2026-08-15): **17 passed**.
- Gate (`./run_tests.sh`, run 2026-08-15): **5147 passed in 231 s, RESULT: PASS.**

## Open Questions

1. **The lag integral's absolute value and its ~0.35 ms/s creep** (residual (a)).
   Owner: the FW 14 validation pass. The trend is trustworthy; the absolute is
   not, and the creep has no mechanism yet.
2. **The SDO RTT floor's mispairing under pipelining** (residual (b)). A
   multi-slot request-stamp ring would make the probe valid at any delay; until
   then its floor argument holds only for one-in-flight polling.
3. **Does the residual 283–340 ms gap close once the leak does?** The lap
   accounts for ~115–135 ms of it. If an aged FW 14 plant still lags after
   `leak ≡ 0`, a second term exists and the arc is not finished.
4. **Does the ring-full pop tear in practice?** Unchanged from the audit — and
   now known to run **at** the full ring for hours, which is the condition the
   tear needs.

## Related

- `logbook/2026-08-14-ring-audit-available-leak-delay-line.md` — the audit that
  named the defect and specified this measurement; the arc's Discussion lives
  there.
- `logbook/2026-08-14-fw13-ring-diag-conviction-instrument.md` — the instrument
  flashed for this soak.
- `logbook/2026-07-18-teensy-uptime-tracking-degradation.md` — the parent, still
  `open`; see its 2026-08-14 (S3) addendum.
- `plans/active/bridge-temporal-trustworthiness.md` § "S3 RESULTS — CONVICTED
  (2026-08-14)" and § P3.
- `ros_ws/src/jugglebot/launch/jugglebot_launch.py::_install_drift` — the
  config-side sibling of the new install-skew check, and the reason its blind
  spot (node source) needed closing separately.
