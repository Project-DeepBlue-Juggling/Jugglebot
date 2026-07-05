---
title: Can-hub hardening [18A] — HomingResult uplink closes the silent-abort false-success class
type: feature
date: 2026-07-05
status: fix-landed-pending-hardware-confirm
phase: "Tier-2 / [18A]"
related_plan: canhub-hardening.md
related_entries:
  - 2026-07-02-canhub-hardening-tier2
  - 2026-07-05-canhub-hardening-item20-firmware
commits:
  - 2b749e3
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - controller/teensy_link/homing.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/teensy_link/test_homing.py
  - tests/teensy_link/test_protocol_codec.py
  - tests/firmware/test_udp_protocol_xlang.py
  - tests/ros/test_teensy_bridge_node_home.py
subsystem:
  - firmware
  - controller
  - ros
  - protocol
  - testing
tags:
  - hardening
  - firmware
  - protocol
  - homing
  - fable-5
  - wire-format
---

## Summary

Plan item [18A] (Fable-5 Tier-2): close the homing **silent-abort false-success**
class by uplinking the firmware's real per-axis `HomingResult` and having the host
observer trust it, replacing the CLOSED_LOOP→IDLE state-cycle inference. A
wire-protocol change (`PROTOCOL_VERSION` 1→2), deployed firmware+host in lockstep.

- **Wire**: the Diagnostic frame's 3-byte alignment `pad` → `homing_result` (u8) +
  2-byte `pad`. **Size-preserving** (36 B), but a layout change → `PROTOCOL_VERSION`
  1→2 + the [15] freeze `_EXPECTED` re-pinned + version-freeze pin bumped.
- **Firmware** (`telemetry.cpp`): `send_diag` populates `d.homing_result =
  homing_result(axis)`; a `homing_result` transition also forces a Diagnostic, which
  delivers the RUNNING transition promptly (decoupled from the lagging ODrive
  `axis_state`) so the host reliably arms its `saw_running` gate.
- **Host** (`homing.py`): the observer TRUSTS the firmware result (gated on having
  seen `HOMING_RUNNING` this run): `OK`→DONE, `FAILED`→FAILED. The bridge node's
  `_homing_axis_status` plumbs the new field into `HomingAxisStatus`.

## Context

The homing move runs autonomously in firmware (fire-and-monitor); the Jetson's
`HomingMonitor` observes completion. Since Phase 9b it inferred **success** from the
axis cycling CLOSED_LOOP→IDLE with no active errors (trusting the firmware's internal
Iq-spike trip, exactly as can_node trusted its own). The firmware already tracked the
real outcome in a per-axis `HomingResult` (NONE/RUNNING/OK/FAILED) — the `leg_homing.h`
comment even flagged it as a "possible future uplink" — but did not put it on the wire.
The Tier-2 review flagged the resulting false-success class; it was deferred as its own
cycle (a wire change) after the Tier-2 flash.

## Discussion

*(Written before Verification — the design decisions a future reader won't infer.)*

### The defect: a firmware abort IDLEs the leg exactly like a success

`leg_homing.cpp`'s `finish()` is called on BOTH a successful trip AND every abort
(bus-down / guard-E-STOP mid-move / SETUP send-fail / timeout), and it ALWAYS issues
`set_state(IDLE)`. On success it also runs `set_absolute_position(home_ref)`; on abort
it does NOT. So to the host's state-cycle observer, a mid-drive abort (which reaches
CLOSED_LOOP, then aborts to IDLE within a tick — faster than the host's 20 s timeout)
is **indistinguishable** from a success: both are CLOSED_LOOP→IDLE with no active
errors. The host declares the leg homed, the orchestrator proceeds, but the leg's
encoder reference was never set — its zero is wrong, and every subsequent move is
mis-calibrated. (The timeout-abort case was already caught by the host's own 20 s
budget, which sits below the firmware's 30 s; the bus-down/E-STOP mid-home case is the
real gap this closes.)

### Wire: reuse the Diagnostic pad byte, not a new message

The Diagnostic is already a per-axis on-change frame carrying `axis_state` +
`active_errors` — precisely the fields the observer reads — and it had a 3-byte
alignment `pad`. Repurposing ONE pad byte as `homing_result` is size-preserving
(36 B unchanged, `active_errors` stays 4-byte aligned) and the natural per-axis fit,
versus a whole new message type + emit path. The byte that was `pad[0]` (right after
`flags`) is now `homing_result`. It is a LAYOUT change (a named field where a pad was),
so the freeze hash flips and `PROTOCOL_VERSION` bumps 1→2 — an incompatible-wire
change, so firmware + host must ship the same version. That is fine here: they deploy
together in one flash. (BB-node Diagnostics — axis_id 7/8 — leave `homing_result` at
the zero-init `HOMING_NONE`; they are not Jugglebot legs.)

### Host: full-trust replace, gated on saw_running

The observer now trusts the firmware outcome as authoritative rather than augmenting
the (buggy) state-cycle inference — the state cycle is dropped entirely. Two guards
make it robust:

1. **`saw_running`** — a terminal `OK`/`FAILED` is trusted ONLY after `HOMING_RUNNING`
   has been observed for this run. `s_result[]` persists across homes, so without this
   a stale `OK`/`FAILED` from a PRIOR home of the same axis could terminate the new one
   the instant the first (stale) Diagnostic arrives. `HOMING_RUNNING` is the firmware
   state for the whole multi-second move and is force-emitted on transition, so the
   20 Hz host poll always observes it first.
2. **timeout backstop** — retained (below the firmware's 30 s) for a leg that never
   resolves at all (a lost RUNNING→terminal transition), plus the `active_errors` check.

A false-FAILURE (from a dropped terminal Diagnostic → timeout) is safe — a failed home
is retried / surfaced; the class we must never allow is the false-SUCCESS, which the
result closes definitively for every abort reason (current + future).

### Firmware: force a Diagnostic on a homing_result transition

`diag_changed` keys on `axis_state`/`active_errors`/etc., not `homing_result`. Adding a
`homing_result != baseline` term to the emit condition delivers the RUNNING transition
PROMPTLY — decoupled from the lagging ODrive `axis_state` (which the ODrive only reports
a tick or two after the firmware commands CLOSED_LOOP) — so a normal multi-second home
reliably lets the host arm its `saw_running` gate, on which every terminal-result trust
depends.

Note the limit (an audit catch): a genuine early SETUP-fail goes RUNNING→FAILED inside
ONE `homing_step()` call (the request-consume sets RUNNING then falls through to SETUP
with no yield), so the 100 Hz telemetry cannot sample the intervening RUNNING — the host
never arms `saw_running`, ignores the (correctly force-emitted) FAILED, and falls to the
20 s timeout backstop. That is a safe false-FAILURE, never a false-success; the
force-emit's real value is the RUNNING-transition promptness above, not this edge case.

## Verification

- **Full suite** (`pytest tests/ -q`, run 2026-07-05): **2051 passed, 1 xfailed in
  469.60 s** — the rewritten `HomingMonitor` tests (the false-success regression +
  `saw_running` gating), the four home-flow ros tests updated for the new Diagnostic
  field, and the config-in-sync + freeze guards after the regen. Order-flaky alloc
  tests confirmed isolated as usual.
- **On-target compile** (`pio run -e teensy41`, 2026-07-05): **SUCCESS** — 225600 text
  / 35520 data / 107168 bss (unchanged from item 20; `homing_result` reuses the pad
  byte, so the Diagnostic and the whole image stay the same size).
- **Wire freeze (intentional re-pin)**: `test_protocol_version_frozen` bumped 1→2 and
  `test_wire_layout_frozen` `_EXPECTED` re-pinned to
  `4fb6260671a9468ebfc124602fbe4012e853721b59ab365b4b128b510761250c` — both paired with
  the incompatible-wire bump, as the [15] freeze guard requires.
- **Flash + powered validation**: pending — consolidates with item 20 into ONE Phase-2
  flash cycle, then a powered re-validation (a real home success + an induced mid-home
  abort — e.g. a CAN3 drop during the drive — to confirm the abort now reports FAILED,
  not the pre-fix false success).

## Related

- Plan: `plans/active/canhub-hardening.md` (Tier-2 [18A]).
- Predecessor: `logbook/2026-07-02-canhub-hardening-tier2.md` ([18B] fixed the inverted
  timeout docstring; [18A] deferred here). Sibling: item 20
  (`logbook/2026-07-05-canhub-hardening-item20-firmware.md`) — the same Phase-2 flash.
- The foam-stop position-assertion history this preserves (do NOT assert post-IDLE
  position): the 2026-06-26 homing bugfix.
