---
title: Deactivate no-op when already at STOW — skip the redundant re-arm+lower
type: feature
date: 2026-07-05
status: resolved
related_entries:
  - 2026-07-04-canbridge-stow-on-shutdown
  - 2026-07-05-canhub-hardening-18a-homing-result-uplink
commits:
  - 559c6dc
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_teensy_bridge_node_deactivate.py
subsystem:
  - ros
tags:
  - deactivate
  - cold-start
  - operator-request
---

## Summary

Operator request (2026-07-05): **the legs should NOT deactivate if the platform is
already at STOW** (all target legs IDLE with position near the stow pose). Previously
a `/deactivate` (or the activate-or-deactivate service, or stow-on-shutdown) always
ran the full firmware DEACTIVATE — re-arming each leg to CLOSED_LOOP (a small jolt),
lowering to STOW (a no-op when already there), then IDLE. Now a guard at
`_run_deactivate` short-circuits: if every target leg is already IDLE and within
`_STOW_POS_MAX_REV` (0.2 rev) of STOW, it skips the redundant leg move (still idling the
hand, for parity) and reports success.

## Changes

- `teensy_bridge_node._legs_already_stowed(axes)` — reads the telemetry + per-axis
  diagnostic cache and returns True iff every leg in `axes` is IDLE and `|pos| <=
  _STOW_POS_MAX_REV` with a **finite** position. **Fail-safe**: a missing telemetry
  snapshot, a missing per-axis diagnostic, OR a non-finite (NaN) position returns False,
  so the deactivate proceeds rather than silently skipping a real lower.
- `_run_deactivate` gains the guard right after the empty-axes check (before the leg
  DEACTIVATE): if `_legs_already_stowed(axes)`, skip the redundant leg move, still
  best-effort-idle the HAND (axis 6 — de-energise-all-axes parity), and return
  `(True, "already at STOW …")`.
- `_STOW_POS_MAX_REV = 0.2` module constant.
- +3 node tests (already-stowed no-op fires no RPC; a still-extended leg proceeds; the
  predicate across IDLE/CLOSED_LOOP/extended/missing-diag cases).

## Discussion

### Single enforcement point + why host-side

`_run_deactivate` is the one place the deactivate MOVE is issued — the `/deactivate`
service, the activate-or-deactivate service, and `_shutdown_stow` (stow-on-shutdown)
all route through it — so the guard lands once and covers every path (including a
redundant stow-on-shutdown when the platform was already brought down). It is host-side
(not firmware) because "all legs at STOW" is a PLATFORM-level predicate over every leg's
position + state, which the bridge already caches from the telemetry + diagnostic
streams; the firmware `deactivate_request` is per-axis and has no natural whole-platform
view. Host-side also means **no reflash** for a behaviour tweak.

### The STOW band + the sign convention (the load-bearing bit)

The check is `|pos| <= 0.2`, not `pos <= 0.2`. STOW is `pos = 0.0` in the
`DeactivateMonitor` convention (`DEFAULT_STOW_REV = 0.0`; the arrival latch already
uses `|pos − stow| <= tol`), and the active pose is ~2.2 rev in magnitude. Using the
absolute value is safe **regardless of the telemetry sign convention** — a bare
`pos <= 0.2` would wrongly classify an active pose as stowed if the active position
read negative, leaving the platform UP when the operator asked it DOWN (the one
dangerous failure of this guard). `abs()` requires the leg to be genuinely near 0, so
an extended leg (either sign, ~2.2 magnitude) is never mistaken for stowed. 0.2 rev
sits just above the monitor's 0.15 arrival tolerance to absorb the small foam-relaxed
drift a leg takes the instant it IDLEs.

### Fail-open toward action, not toward skipping (incl. NaN — an adversarial-review catch)

Every uncertainty (no telemetry yet, a leg with no diagnostic, any leg not IDLE or not
near STOW) returns False → the deactivate proceeds. The guard only skips when it can
POSITIVELY confirm the whole platform is stowed. Skipping a needed lower would be the
harmful error; a redundant lower is merely the pre-existing behaviour.

The subtle one: `pos_rev` is **NaN** until the encoder is ready and after an encoder
fault — and a faulted ACTIVE leg drops to IDLE reporting NaN. Since `abs(NaN) > 0.2` is
False, a bare `> _STOW_POS_MAX_REV` test would misclassify a NaN leg as within the stow
band → with the others stowed, the guard would skip the lower and leave the platform UP
with a faulted leg. `math.isfinite(pos)` (mirroring the existing
`_encoder_search_axis_status` guard) makes NaN fail SAFE. NB the asymmetry vs
`DeactivateMonitor`'s non-inverted `abs(pos − stow) <= tol` arrival latch, where NaN
already fails safe (never latches); the guard's inverted test needed the explicit
finite check. (+ a NaN predicate test.)

### The no-op still de-energises the hand

DEACTIVATE de-energises ALL JUGGLEBOT_AXES (legs via the TRAP_TRAJ→IDLE, hand via
SET_AXIS_STATE(IDLE) — can_node parity, row 29). The guard skips only the redundant LEG
move, NOT the hand idle: the no-op branch still fires the best-effort hand IDLE before
returning, so a `/deactivate` (or stow-on-shutdown) with the legs already down but the
hand still armed correctly drops the hand.

## Verification

- Node + predicate tests: `pytest tests/ros/test_teensy_bridge_node_deactivate.py -q`
  → **9 passed** (2026-07-05).
- Full suite (`pytest tests/ -q`, run 2026-07-05): **2054 passed, 1 xfailed in 645.50 s**
  (+3 deactivate stow-guard tests). Order-flaky alloc tests confirmed isolated as usual.
- **Hardware VALIDATED (2026-07-05, operator)**: `/deactivate` when the platform is
  already stowed is a clean no-op (no CLOSED_LOOP re-arm jolt); `/deactivate` from the
  active pose still lowers normally. Both directions confirmed on the robot.

## Related

- `logbook/2026-07-04-canbridge-stow-on-shutdown.md` (the other deactivate caller this
  guard now also short-circuits when already stowed).
