---
title: Can-bridge foundation Phase 2 — cold-start state via the relay (self-solving persistence)
type: feature
date: 2026-06-29
status: resolved
phase: "2"
related_plan: canbridge-foundation-coldstart-parity.md
related_entries:
  - 2026-06-29-canbridge-phase1-platform-relay-seam
  - 2026-06-29-canbridge-phase0-native-harness
  - 2026-06-27-can-node-teensy-parity-audit
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/ros/test_teensy_bridge_node_coldstart.py
commits:
  - 57f8885
  - fc22e4d
subsystem:
  - can
  - ros
  - testing
tags:
  - cold-start
  - relay
  - persistence
  - parity
---

# Can-bridge foundation Phase 2 — cold-start state via the relay

## Summary

Phase 2 of `canbridge-foundation-coldstart-parity.md` sources the orchestrator's
cold-start fields (`is_homed` / `levelling_complete` / `pose_offset`) from the
Platform Teensy's `RobotState` through the Phase-1 relay, replacing the hardcoded
conservative defaults the bridge had carried since phase 10b
(`teensy_bridge_node.py` `_publish_robot_state`). This is the regression the
2026-06-27 parity audit flagged: the production launch runs `orchestrator_node`
beside the bridge, but the bridge fed it constant `False`, so the automated
cold-start path was non-functional. With locked-decision #1 (the orchestrator
stays, behaviour-parity is the goal), feeding it the real persisted state is the
fix.

The bridge now:

* **READS** `RobotState` via the relay `STATE_READ` at boot (bounded retries; the
  read completes BEFORE construction returns so the first publish already carries
  the right `is_homed`) **and on each confirmed link reconnect** (the existing
  watchdog `LinkLossLatch` LOST→RESTORED edge), **caches** it, and surfaces the
  three fields on `robot_state` **from the cache** — the 100 Hz publish path never
  does a CAN3 round-trip.
* **WRITES** the home **result** (`is_homed = ok`) after `/home` (read-modify-write
  THROUGH the cache, so the levelling + pose fields are preserved), exact
  `can_node.py:847` parity — a FAILED home persists `is_homed=False` too, so a
  failed re-home of an already-homed robot cannot leave a stale `True`.
* clears all three on `REBOOT_ODRIVES` via a shared ordered hook (`_reboot_odrives`
  step 2 — the `STATE_WRITE` clear; step 1, the Phase-6 watchdog-suppression
  latch, is out of scope), mirroring `can_node.py:1559-1565`.
* **DERIVES** `encoder_search_complete = is_homed OR within-session-search-done`
  (the in-session bit is set on a successful `/encoder_search`), exact can_node
  parity (`can_node.py:549-550`).

**Safety invariant (the one safety-relevant design point):** if the boot read
genuinely fails after all retries, the cache stays at the **conservative**
`is_homed=False` — forcing a re-home is wasteful but SAFE; the reverse (a stale
`is_homed=True`) could skip homing on an unhomed robot. This is the only
acceptable failure direction.

**No new can-bridge store, no invalidation rule** (dissolved by the power
topology, locked-decision #3): the Platform Teensy shares the ODrive supply, so
its `RobotState` is lost exactly when the references are lost. The bridge is the
sole writer and read-modify-writes through its cache.

This phase is **software-only — no firmware change** (the `STATE_WRITE` clear is
host-orchestrated via the existing Phase-1 `relay_write_robot_state`; `STATE_READ`/
`STATE_WRITE` were already in firmware) and **no codegen change** (the wire ids
were reserved in Phase 0; `encode_state_write` exists). It has **no hardware gate
of its own** — the eventual validation is the Phase-4 powered cold-start sitting.

Verified: full suite **1894 → 1908 passed, 1 xfailed, 0 failed** (`pytest tests/
-q`, run 2026-06-29) — net **+14** new Phase-2 tests, no regressions.

## Motivation

`can_node` owned a `last_known_state` dict (`is_homed` / `levelling_complete` /
`pose_offset`) that the Platform Teensy persisted over the `0x6E0` `RobotState`
protocol; `can_node._decode_teensy_state` refreshed it on every incoming `0x6E0`
frame, and `_publish_robot_state` surfaced it on `robot_state` for the
orchestrator's skip-if-saved cold-start logic (`state_machine.py:239` `is_homed`,
`:252` `encoder_search_complete`). The bridge hardcoded those fields to
`False`/zero (the phase-10b "conservative defaults" block), so the orchestrator
always saw an unhomed robot — the cold-start regression.

Phase 1 built the conduit (the typed `STATE_READ`/`STATE_WRITE` RPCs + the verbatim
`PLATFORM_FRAME` reply uplink, host-correlated by `(can_id, dlc)`) and left the
`robot_state` wiring explicitly deferred to Phase 2. This phase is that wiring.

## Design

### The field-sourcing matrix (each field sourced exactly once)

| field | source | how |
|---|---|---|
| `is_homed` | Platform Teensy `RobotState` | relay `STATE_READ`, cached (boot + reconnect) |
| `levelling_complete` | Platform Teensy `RobotState` | relay `STATE_READ`, cached |
| `pose_offset_rad` / `pose_offset_quat` | Platform Teensy `RobotState` | relay `STATE_READ`, cached; `pose_offset_quat` via the can_node-identical `_tilt_to_quat` |
| `encoder_search_complete` | bridge-**derived** | `is_homed OR _encoder_search_done_session` (no wire field) |
| `firmware_validated` | (Phase 3) | untouched here — still conservative `False` |

### The cache (`_cold_start_state` + `_cold_start_authoritative`)

A `RelayRobotState` namedtuple guarded by the existing `self._lock` (the node runs
under a `MultiThreadedExecutor`, so the publish timer and the writers — health
check, service callbacks — can race). The relay read/write themselves run WITHOUT
the lock (they block on the CAN3 round-trip); only the cache swap is under the
lock. Read-modify-write uses `namedtuple._replace` on the freshly re-read cache so
a concurrent field update is not clobbered. `_cold_start_authoritative` (set once
a real read or a bridge-issued write lands) is surfaced on `link_status` so the
Phase-4 sitting can confirm the boot read landed.

### Boot read (`_boot_read_cold_start_state`)

Called from `__init__` (gated on `boot_state_read=True`, default True) AFTER the
relay infra + the `PLATFORM_FRAME` subscription are up, so it runs before the
executor spins and before the first `robot_state` is published — the orchestrator
never sees a wrong `is_homed`. Bounded retries (`_BOOT_STATE_READ_ATTEMPTS=3` ×
`_RELAY_READ_TIMEOUT_S`, `0.2 s` between) give a momentarily-not-yet-synced CAN3 a
second chance; on total failure it sets the **conservative** cold value.

### Reconnect re-read

Hooked into `_health_check`'s existing LOST→RESTORED edge — sound here because a
reconnect only triggers a re-read of the authoritative store, never *infers*
reference state. A FAILED reconnect re-read **keeps** the last authoritative cache
value (can_node's passive "last-known-state until a fresh frame" — never downgrade
a good read on a transient hiccup). Only the boot read applies the conservative
fallback (it has no prior authoritative value to keep).

### Write paths

* `_write_is_homed(ok)` from `/home` with the home **result** (after `_run_home`,
  unconditionally, before the configure gate) — exact `can_node.py:847` parity:
  can_node writes `success`, so a FAILED home persists `is_homed=False`. (The audit
  caught an earlier `_write_is_homed(True)`-on-success-only version that would have
  left a stale `True` on a failed re-home — see the Discussion.)
* `_reboot_odrives` (the shared hook) routes both reboot entry points
  (`_svc_reboot_odrives` and `odrive_command`'s `reboot_odrives`) through one
  ordered dispatch; step 2 clears all three with a small retry
  (`_REBOOT_CLEAR_ATTEMPTS=3`) and sets the cache cleared regardless.

### encoder_search_complete is monotonic (the can_node:549-550 latch)

`encoder_search_complete = is_homed OR _encoder_search_done_session`. `is_homed`
alone is not monotonic (reboot/reconnect can read it `False`), so to match
can_node's monotonic field (set True at `can_node.py:549-550` whenever a decoded
frame has `is_homed`, plus `:1355` after a search, and **never cleared**) the
in-session bit **latches True whenever a relay read shows `is_homed`** (homing
requires a prior search). The bit is sticky for the process — not cleared on
reboot — so the derived field is monotonic-True once homed/searched, exactly as
can_node.

## Implementation

`teensy_bridge_node.py`: module-level `_tilt_to_quat` (byte-identical to
`can_node._tilt_to_quat`, importing the same `quaternion` library); the cache
state in `__init__`; `_publish_robot_state` reads the cache snapshot under the
existing top-of-method lock and replaces the hardcoded block; the four cold-start
methods (`_refresh_cold_start_state`, `_boot_read_cold_start_state`,
`_write_is_homed`, `_clear_cold_start_state_on_reboot`); the reconnect hook in
`_health_check`; the `_reboot_odrives` shared hook; the `/home` + `/encoder_search`
service edits; two `link_status` diagnostic keys; a `pose_offset_quat` memo (the
quaternion composition runs only when the cached tilt changes, not per 100 Hz
publish — can_node likewise computed it on-change). Tests: `_build_paired_node`
gained a `boot_state_read=False` default (keeps the existing suite fast — those
tests don't wire a Platform responder before `__init__`); new
`test_teensy_bridge_node_coldstart.py` (14 tests, incl. the two audit-driven ones:
the failed-home `is_homed=False` persist and the encoder-search monotonicity
across reboot).

## Verification

(date, command, result triples — re-runnable from the artefact alone)

- **Phase-2 unit file** (`pytest tests/ros/test_teensy_bridge_node_coldstart.py
  -q`, 2026-06-29) = **14 passed** (run repeatedly, all green — no order-flake
  after the reconnect-test relay-path warmup was added).
- **ros subset** (`pytest tests/ros/ -q`, 2026-06-29, after the audit fixes) =
  **656 passed** (includes the 14 coldstart tests + the restructured `_svc_home`).
- **Full suite** (`pytest tests/ -q`, run 2026-06-29, after the audit fixes) =
  **1908 passed, 1 xfailed, 0 failed in 472.87 s**. Net **+14** vs the Phase-1
  baseline (1894 passed, 1 xfailed), fully accounted by
  `test_teensy_bridge_node_coldstart.py` (14). No test was removed; no existing
  test changed semantics (the `_build_paired_node` helper edit only disables the
  new boot read for tests that don't wire a responder — those tests never asserted
  the cold-start fields).
- **Firmware:** untouched (the `STATE_WRITE` clear is host-orchestrated). No
  `pio run` / codegen change.
- **Hardware:** none — Phase 2 has no hardware gate of its own (the Phase-4
  powered cold-start sitting is the eventual validation).

## Discussion

CLAUDE.md makes the Discussion non-negotiable here: a non-obvious tradeoff was
accepted (the reconnect re-read trigger choice + its residual hole), a deliberate
parity choice was made over a more "correct" alternative (the reboot
encoder-search bit), and the chosen boot-read placement beat reasonable
alternatives for reasons not inferable from the code.

### Why the boot read lives in `__init__` (synchronous, before spin)

The safety requirement is that the FIRST `robot_state` the orchestrator could act
on already carries the authoritative `is_homed`. The publish path returns early
until the first telemetry frame arrives, and telemetry only flows once the link is
up — but tying the read to "link up" (in the watchdog) would couple correctness to
timer timing and risk a publish racing ahead of the read. Doing the read
synchronously in `__init__` (before the executor spins) makes the ordering
structural: construction does not return until the cache holds either the real
state or the conservative fallback. The cost is a bounded block during
construction (a deliberate cold-start step, exactly like `_run_home`). To keep the
~30 node-building unit tests fast, the read is gated behind a `boot_state_read`
flag the shared test helper defaults off; the boot path itself is covered
explicitly (success + conservative-fallback).

### The conservative-on-failure invariant is the load-bearing safety choice

`is_homed=False` forces a re-home — wasteful but safe. `is_homed=True` on an
unhomed robot would let the orchestrator skip homing → the legs are commanded from
an unreferenced encoder → dangerous motion. So every failure path resolves toward
`False`: a failed boot read defaults conservative, and a failed reboot clear still
sets the local cache `False`. The only place a `True` survives a failure is a
transient reconnect re-read hiccup, where keeping the *prior authoritative* `True`
is correct (a momentary UDP blip does not un-home a powered robot) and matches
can_node's passive model.

### The audit-caught HIGH: a failed `/home` must persist `is_homed=False`

The `/audit --unstaged` pass before the docs commit caught a HIGH-severity hole in
my first cut: I had `_write_is_homed(True)` gated behind `if ok:` (persist on
home-success only). can_node writes `_update_teensy_state({'is_homed': success})`
— i.e. it persists the home **result**, `False` on a failed home. With the
success-only version, an **operator-direct re-home** (GUI / `ros2 service call`,
not gated by the orchestrator's `is_homed==False` HOMING precondition) of an
already-homed robot whose home then FAILS would keep the cache `True` →
`state_machine.py:238-239` skips homing on FAULT-recovery → the legs are commanded
from an unreferenced encoder. That is the exact dangerous direction the phase is
built to prevent — so this is the conservative-on-failure invariant applied to the
write path, not just the read path. Fix: `_write_is_homed(ok)` (the home result),
unconditionally, before the configure gate — exact can_node:847 parity, the
lowest-risk possible change (it matches the proven reference). The path was
untested (the only home test stubbed `_run_home` to succeed); a new test
(`test_home_failure_persists_is_homed_false`) pins the failed-home `is_homed=False`
persist with levelling/pose preserved. Per CLAUDE.md's "fix surfaced bugs in the
same session when diagnosis is clear" — the diagnosis is unambiguous and the fix
is reference-matching, so it landed here, not deferred.

### The reconnect-trigger residual (flagged for the Phase-4 sitting)

The plan says to re-read "on each confirmed CAN3 reconnect (reusing the existing
watchdog reconnect detection)". The existing watchdog (`LinkLossLatch`) detects
the **UDP link** (Jetson↔can-bridge) reconnect, not a CAN3-bus reconnect. These
differ in exactly the canonical power-topology scenario: **Jugglebot disconnected
while the Jetson + can-bridge stay powered** (the can-bridge is on Jetson 5 V — see
`project_canbridge_power_topology`). Then the Platform Teensy (on Jugglebot's 12 V
logic) loses power and forgets (`RobotState`→`{false}`, correct), the ODrives lose
their references, CAN3 goes down — but the UDP link never drops, so there is **no
LOST→RESTORED edge and no reconnect re-read**. The bridge *cache* could hold a
stale `is_homed=True` until the next refresh.

I implemented the plan's literal trigger (UDP-watchdog reconnect) rather than
inventing a CAN3-bus-health trigger, because the instructions scoped Phase 2's one
safety-relevant point to the *boot read + conservative fallback* and said to
resolve it the plan's way, not to invent. The residual is **bounded by three safe
backstops already in place**: (a) the production launch co-starts the orchestrator
+ bridge, so every cold-start evaluation is normally preceded by a fresh **boot
read** (which returns `False` after a disconnect, since the Platform Teensy
forgot); (b) the **`REBOOT_ODRIVES` clear hook** clears `is_homed` on any
reboot-based recovery — but (audit, 2026-06-29) this backstop is **narrower than
first written**: it does NOT cover a **transient FAULT→IDLE recovery that does not
reboot the ODrives**, which is exactly the path `state_machine.py:238` reaches the
skip-if-homed on; (c) the failure direction of the hole is the one we guard
against, but it requires the bridge to outlive a disconnect AND a cold-start
re-trigger without a bridge restart or reboot — and the audit further notes the
reconnect re-read is single-attempt and keeps the cached value on read FAILURE
(can_node's passive last-known-state parity), so the hole is reachable by a
read-fail sub-case too, not only a trigger-doesn't-fire sub-case.

**Crucially, the hole is LATENT in Phase 2** and cannot cause motion here: the
bridge still hardcodes `firmware_validated=False`, and `state_machine.py:228-235`
gates the `is_homed` skip *behind* `firmware_validated` — so the orchestrator never
reaches the skip until **Phase 3** lands the real `firmware_validated`. **This
residual MUST therefore be closed before Phase 3 makes `is_homed` consumption
live** (re-targeted from "the Phase-4 sitting" to a hard **Phase-3 precondition**
per the audit). The surgical fix is the CAN3-bus-health (`bus1_health` BUS_OFF→OK)
reconnect re-trigger, with the re-read falling back **conservatively** (retry, then
`is_homed=False`) on read failure rather than keeping the stale value. Kept out of
Phase 2 deliberately (the UDP-watchdog trigger is the plan's literal instruction;
inventing the CAN3-health trigger here would be an un-reviewed plan deviation), but
it is now a tracked Phase-3 blocker, not a Phase-4 probe.

### encoder_search_complete monotonicity (the second audit fix)

`encoder_search_complete = is_homed OR _encoder_search_done_session`. can_node's
field is **monotonic**: set True at `can_node.py:549-550` (whenever a decoded
frame shows `is_homed`) and `:1355` (after a search), and **never cleared** — the
reboot at `:1559-1565` deliberately leaves it. My first cut latched
`_encoder_search_done_session` only on a successful `/encoder_search`, so a boot
read of `is_homed=True` (persisted-homed, no in-session search) followed by a
reboot (clears `is_homed`) would publish `encoder_search_complete=False` — a
non-monotonic drop the audit flagged against the "sticky / exact parity" claim in
my code + this entry. Fix: **also latch the bit when a relay read shows
`is_homed`** (homing requires a prior search — `can_node.py:549-550` exactly), so
the derived field is monotonic-True once homed/searched and never drops, even
across a reboot. On `REBOOT_ODRIVES` I clear `is_homed`/levelling/pose but **not**
the (now-monotonic) search bit — exact `can_node.py:1559-1565` parity. (Physically
an ODrive reboot loses the encoder index too, so a stricter design would
re-search; but the orchestrator is unchanged — locked-decision #1 — and the
definition of done is behaviour-identical-to-can_node, so we preserve can_node's
monotonic field rather than introduce an out-of-scope divergence.) A new test
(`test_encoder_search_complete_monotonic_across_reboot`) pins the
read-latch-then-reboot path.

### Why generate the quaternion from the same library

`pose_offset_quat` is byte-identical to `can_node._tilt_to_quat` (same
`quaternion.from_rotation_vector` composition, same sign convention) so downstream
consumers (GUI, tf) see no change across the cutover. Re-deriving it with ad-hoc
trig would risk a sign/convention drift that only shows up as a subtly-wrong
platform-tilt visualization — the kind of bug that is expensive to chase. Reusing
the production library is the cheap, faithful choice.

## Open questions / next steps

- **Deployment:** the source edit needs a `colcon build --packages-select
  jugglebot` before the Phase-4 sitting (production imports the installed copy).
- **Phase 4** (orchestrator wiring) consumes these fields: the `home_motors`
  action shim, the tilt/level relay, and the runtime drift-guard contract test. It
  also lands the powered cold-start parity sitting that validates this phase.
- **Phase 3** (Get_Version → `firmware_validated`) is independent of Phase 2 — it
  rewrites the still-conservative `firmware_validated=False`. Startable now.
- The reconnect-trigger residual above is the one thing the Phase-4 sitting should
  explicitly probe.
- **Deferred to Phase 4 (audit LOWs):** (a) the reconnect re-read + the reboot/home
  STATE_WRITE retries share the node's default `MutuallyExclusiveCallbackGroup`
  with the 100 Hz publish timers, so a blocking relay round-trip stalls publishing
  for its duration (pre-existing in kind — `_run_home` already blocks the group);
  Phase 4's design already calls for moving the blocking cold-start verbs into a
  `ReentrantCallbackGroup`, which subsumes this. (b) `_write_is_homed` persists a
  pre-RPC snapshot of levelling/pose to the wire — RMW-correct for the cache, but
  when Phase 4 lands the `set_level_state` levelling write, a concurrent levelling
  write between the snapshot and the STATE_WRITE could be clobbered on the Teensy;
  revisit the write serialization then so the "read-modify-write … and vice versa"
  guarantee holds on the wire too (unreachable in Phase 2 — no levelling-write path
  exists yet).

## Related

- Plan: [`plans/archived/canbridge-foundation-coldstart-parity.md`](../plans/archived/canbridge-foundation-coldstart-parity.md) — Phase 2 detail, the field-sourcing matrix, the REBOOT_ODRIVES shared hook, locked-decisions #2/#3.
- [2026-06-29-canbridge-phase1-platform-relay-seam.md](2026-06-29-canbridge-phase1-platform-relay-seam.md) — the relay mechanism (`relay_read_robot_state` / `relay_write_robot_state` / `RelayRobotState` / `encode_state_write`) this phase wires in; the SRX_DIS + reply-latency bench validation.
- [2026-06-27-can-node-teensy-parity-audit.md](2026-06-27-can-node-teensy-parity-audit.md) — the cold-start-broken headline this phase closes.
- `project_canbridge_power_topology` (memory) — why persistence lives on the Platform Teensy, and the reconnect-trigger residual scenario.
