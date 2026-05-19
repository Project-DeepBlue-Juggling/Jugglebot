---
title: SpaceMouse — resilient connection handling + home-to-ACTIVE on dropout (mocap-pattern transfer)
type: feature
date: 2026-05-19
status: resolved
phase: "standalone — hardware log hygiene"
related_plan: null
related_entries:
  - 2026-05-18-mocap-qtm-connection-logging
files_changed:
  - ros_ws/src/jugglebot/jugglebot/spacemouse_handler.py
  - logbook/2026-05-19-spacemouse-resilient-connection-handling.md
  - logbook/INDEX.md
commits:
  - e08453b
subsystem:
  - ros
  - tracking
  - spacemouse
tags:
  - feature
  - bugfix
  - logging
  - spacemouse
  - hid
  - reconnection
  - safety
  - control-system
---

# SpaceMouse — resilient connection handling + home-to-ACTIVE on dropout

## Summary

Applied the validated mocap connection-handling contract
([[2026-05-18-mocap-qtm-connection-logging]]) to `spacemouse_handler`,
adapted for USB HID. The node now stays alive whether or not the
SpaceMouse is present, retries quietly with a state-transition +
rclpy-throttled-recurring WARNING, auto-connects when the device is
plugged in, and — on a mid-session unplug *while SpaceMouse is the
active control mode* — commands the platform smoothly back to the
ACTIVE pose `(0,0,170,0,0,0)` so it never freezes at whatever pose the
stick last commanded. Commit `e08453b`.

This also fixes a **severe latent bug**: a mid-session unplug raised an
uncaught `HIDException` inside the 100 Hz timer callback → ~100
tracebacks/second.

## Motivation

`spacemouse_handler` had three problems, worse than mocap's:

1. **Self-terminates if the device is absent at startup.** `__init__`
   tried 3× then set `shutdown_flag=True`; `main()`'s loop exited and
   the node died — the exact "require the device before ROS2 starts"
   anti-pattern rejected for mocap. No recovery if plugged in later.
2. **Zero mid-session reconnection.**
3. **100 Hz exception spam on unplug.** `pyspacemouse.read()` →
   `easyhid` `device.read()` raises `HIDException` on device removal;
   nothing in `pyspacemouse` or `publish_pose()` caught it, and
   `publish_pose` runs on a 100 Hz timer.

## Design

The connection-handling *philosophy* transferred directly; the
*mechanism* was adapted by root cause, not copied:

| Mocap fix | Transferred? | Why |
|-----------|--------------|-----|
| State-transition + rclpy-throttled recurring WARNING | yes | Same need: reliably declare the no-device state, no spam |
| Keep node alive, retry, auto-reconnect | yes | Same anti-pattern to kill (the startup hard-fail) |
| Guard device calls in try/except | yes — *more critical* | Kills 100 Hz spam (mocap's was ~1/35 s) |
| `asyncio.wait_for` connect bound | **no** | No hang failure mode — `pyspacemouse.open()` enumerates HID and *raises immediately* when absent (verified in `pyspacemouse.py:790`). Copying `wait_for` would be cargo-culting. |
| mDNS hostname (mocap fix B) | **no** | Local USB device — no addressing |
| `logging.getLogger("qtm_rt")` suppression | **partial** | `pyspacemouse` noise is bare `print()` to stdout, not a named logger — different, lower-priority; flagged, not fixed |

Constants mirror the mocap contract: `_OUTAGE_WARN_THROTTLE_S = 30.0`.
Reconnect is rate-limited (`_RECONNECT_INTERVAL_S = 2.0`), explicitly
**not** the 100 Hz publish rate (`pyspacemouse.open()` also emits bare
`print()`s — hammering it would spam stdout).

**Safe-state on teleop loss — the user's design, chosen over mine.**
The initial recommendation was "hold last pose" (rely on the MPC's
existing hold-last-target). The user proposed instead: on dropout,
send the platform back to ACTIVE at a smooth pace, reasoning that
SpaceMouse mode is only ever used for qualitative "fun" testing. This
is the better design and was adopted — see Discussion for why.

## Implementation

- Removed `shutdown_flag`; `main()` loops on `rclpy.ok()` only.
- `_ensure_open()`: rate-limited, guarded `pyspacemouse.open()`; one
  INFO on (re)connect, one throttled WARNING while absent.
- `_handle_disconnect()`: closes, sets `_is_open=False`, one WARNING
  per disconnect *event* (distinct call site from the throttled
  "still absent" line — mirrors mocap's `_on_qtm_disconnect`).
- `publish_pose()`: guards both `pyspacemouse.read()` call sites. On
  read failure → `_handle_disconnect()`; if SpaceMouse is the active
  mode, publish ACTIVE. While disconnected+enabled, publish ACTIVE
  every tick (idempotent; survives MPC restart; continuous on
  reconnect since stick-at-rest == ACTIVE).
- `_publish_pose_from_axes()`: the original pose-construction,
  refactored to take raw axes. All-zero axes == ACTIVE
  `(0, 0, JB_OP_DEFAULT_ACTIVE_Z_MM=170, identity)` by construction —
  the home path reuses this exact transform, no new frame assumptions.
- Guard: home-to-ACTIVE fires **only when `spacemouse_enabled`**
  (SpaceMouse owns the platform). `mpc_bridge_node._on_platform_pose`
  independently re-checks `_current_mode == 'SPACEMOUSE'` — double-safe.

## Verification

Mocked-`pyspacemouse` probe on the Jetson (`2026-05-19`, throwaway
`/tmp/probe_spacemouse.py`, not committed — the device cannot be
physically unplugged remotely):

| Scenario | Result |
|----------|--------|
| Startup absent | node alive, `is_open=False`, 1 throttled WARNING (no hard-fail) |
| Absent + not active mode | no publish; `open()` rate-limited (not hammered) |
| Absent + SPACEMOUSE active | publishes ACTIVE `(0,0,170,identity,'SPACEMOUSE')` each tick |
| Device appears | INFO "connected"; resumes real readings (`15,-30,212`) |
| Mid-session unplug while flying | exception caught (no spam), `close()`, 1 WARNING, platform homed to ACTIVE |
| Stays disconnected | holds ACTIVE; `open()` rate-limited (`5→5` without forced retry) |
| Reconnect | resumes real readings |

Full suite — `pytest tests/ -q` (run 2026-05-19, ci-fast default
profile): **1424 passed, 1 xfailed, 1 failed in 438.77 s**. The single
failure — `tests/sim/test_hot_loop_allocation_contract.py::test_hot_loop_allocation_contract`
— **passes standalone** (`1 passed in 7.55 s`) and is outside this
change's scope: no test imports `spacemouse_handler` (verified by
`grep -rl spacemouse tests/` — all four hits are the `'SPACEMOUSE'`
mode string/enum, none import the module), and a sim hot-loop
allocation contract is unrelated to a ROS2 USB-HID node. A *different*
unrelated test (`test_motor_guard.py::test_normal_interpolation_unchanged`)
failed the prior full-suite run under the same conditions — the
signature of concurrent parallel-session pollution (another Claude
session was actively editing `can_node.py`/`test_can_node.py` and
running its own pytest). The authoritative validation for this ROS2
code the suite cannot exercise is the probe above.

## Discussion

**Why the user's "home to ACTIVE" beat the initial "hold last pose"
recommendation.** Both are safe in the common case (the MPC profiles
all target changes — no jerk either way). The decisive failure mode:
if the stick is *deflected* at the instant of the unplug,
"hold last pose" freezes the platform at that extreme deflection,
whereas "home to ACTIVE" brings it to the mechanically-safest central
mid-span configuration, away from singularities and travel limits.
Combined with the operational fact that SpaceMouse mode is only used
for qualitative testing (no precision task depends on holding the exact
last pose), a known safe home on loss-of-toy strictly dominates. This
is a case where the user's physical intuition produced a better safety
design than the AI's first recommendation — recorded so the reasoning,
not just the outcome, survives.

**Control-system tradeoff accepted.** This adds a write to the motion
command path (publish ACTIVE on dropout). Accepted because: (a) it
reuses the exact pose transform the node already uses (zero state ==
ACTIVE — no new frame assumptions); (b) it is a *target* change the MPC
profiles, not a raw step position command (consistent with the
"profiled trajectories only" rule); (c) it is gated by
`spacemouse_enabled` *and* re-checked by `mpc_bridge`'s mode filter, so
it cannot perturb the platform when another mode owns it; (d)
publishing ACTIVE repeatedly while disconnected is idempotent and makes
the behaviour robust to an MPC restart mid-outage.

**An intentional non-obvious behaviour.** If SPACEMOUSE is selected as
the control mode while no device is connected, the node publishes
ACTIVE (not nothing). This is deliberate and safe: "SpaceMouse mode
selected, stick necessarily at neutral (no device == no deflection)"
maps to the same ACTIVE pose the device would produce at rest. The
alternative (publish nothing) would leave the platform holding whatever
the *previous* mode left — less predictable. Documented here because a
future reader will otherwise read it as a bug.

**Probe-fidelity carryover.** The mocap investigation's hard lesson
(probes whose failure topology didn't match production missed the real
bug) shaped this work: the probe deliberately exercises the *transition
events* (connect→disconnect→reconnect, enabled/not-enabled) and asserts
the exact ACTIVE pose value and `open()` call-count rate-limiting —
not just "it logs something." The one gap acknowledged up front: a
real physical unplug may surface `easyhid`/hidapi behaviour the mock
doesn't (e.g. a partial read, or `open()` blocking) — to be confirmed
on the next bench session with the real device.

## Open Questions

- **Real-device unplug not yet exercised.** The mock simulates
  `read()` raising and `open()` raising/succeeding; a physical unplug
  on the bench should confirm `easyhid` raises (not hangs / partial)
  exactly as modelled. Low risk (the except is broad) but unverified
  on hardware.
- **`pyspacemouse` bare `print()` noise.** `pyspacemouse.open()` prints
  "No HID devices detected" / "{device} found" to stdout on every
  attempt. Rate-limited to ~1/2 s by `_RECONNECT_INTERVAL_S` so not
  spam, but not silenced (it is `print()`, not a named logger — would
  need stdout redirection). Flagged, deliberately not fixed in this
  change.

## Related

- Pattern source and rationale: [[2026-05-18-mocap-qtm-connection-logging]].
- Logged per `/log`; commit carries
  `Logbook-Entry: 2026-05-19-spacemouse-resilient-connection-handling`.
