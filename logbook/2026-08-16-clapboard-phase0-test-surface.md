---
title: Clapboard Phase 0 — ConeFrame round-trip coverage + cone-bus role discriminators
type: feature
date: 2026-08-16
status: resolved
phase: "clapboard-can3-integration Phase 0"
related_plan: clapboard-can3-integration.md
files_changed:
  - tests/firmware/test_udp_protocol_xlang.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - tests/firmware/native/test_platform_relay.cpp
  - tests/firmware/test_native_firmware.py
subsystem:
  - can-bridge
  - testing
tags:
  - clapboard
  - coverage
---

# Clapboard Phase 0 — ConeFrame round-trip coverage + cone-bus role discriminators

**What/why.** Phase 0 of `plans/active/clapboard-can3-integration.md` closes the two
coverage holes Phases 1–3 would otherwise land into blind. Nothing calls the new
code, so this phase changes no behaviour and implies no flash — which is exactly
what makes it independently committable.

1. **`ConeFrame` had no round-trip test anywhere.** `grep -rn "ConeFrame" tests/`
   returned exactly one hit, a fixture builder in `test_teensy_bridge_node_cone.py`.
   Every clapboard uplink frame rides that 21-byte payload verbatim, so it is
   pinned before anything depends on it: added to both spec-driven parametrize
   lists in `test_udp_protocol_xlang.py` (pack/unpack + frame encode/decode). Both
   lists derive their expectations from the generator spec, so — as the plan
   predicted — no other edit was needed; the two new params passed first try.
2. **`is_clapboard_id()` / `is_cone_id()`** land as pure header-inline predicates
   beside `is_platform_reply_id`, with their truth table pinned by a new
   `TEST_CASE` in the existing `test_platform_relay` native binary (which already
   links what it needs, so no `build.py` entry and no new binary). The cone bus is
   shared by *role*: `cone_health` is manufactured from a timestamp stamped by
   every frame on that bus, so once a clapboard heartbeats the bridge reports a
   catching cone that is not attached — an operator-facing field naming the wrong
   peripheral, which will be believed.

**Why a predicate and not a FlexCAN host shim.** Compiling `can_buses.cpp`
natively would mean hand-shimming 7356 lines of template-heavy vendored
FlexCAN_T4 against i.MXRT register maps, faking raw register dereferences, and
resolving an ODR clash with `fake_hal.o` — 1–2 days plus a permanent maintenance
tax, declined twice before (`can_buses.h:444-449`). Extracting the classifier
costs ~1 h and pins the only part with branching.

**Two judgement calls, both recorded inline.** (a) `is_clapboard_id` spans the
*whole* `0x7E8`–`0x7EF` block including the reserved `0x7EE`/`0x7EF` pair — a
frame there can only have come from a clapboard, so for a presence question the
whole block is the right answer and a future allocation needs no edit here.
(b) `is_cone_id` **enumerates** `CATCH_EVENT`/`HEARTBEAT` rather than spanning
`0x7E0`–`0x7E1`, so the unallocated `0x7E2`–`0x7E7` gap classifies as "no known
peripheral" instead of as a cone. Both predicates use the generated
`CatchingConeCanId` constants where they exist; the clapboard block is literals
until Phase 1 adds its `protocol_config.yaml` section.

Normative source for the id allocation is the Electronic-Clapboard repo's
`docs/protocol.md` §8.2, a cross-repo contract neither side changes unilaterally.

## Verification

- Native truth table: `python tests/firmware/native/build.py && temp/firmware_native/test_platform_relay`
  (run 2026-08-16): **11/11 test cases, 174/174 assertions pass** (was 10 cases).
- Firmware still builds and is unchanged: `pio run -e teensy41` in
  `ros_ws/src/jugglebot/Teensy_code_canbridge` (run 2026-08-16): SUCCESS,
  text 232768 / data 35520 / bss 107872, `firmware.hex` md5
  `ea705b4bb4026047318c0361750c87ab` — **byte-identical** to the pre-change
  build, so no reflash is implied. (Uncalled `inline` functions in a header emit
  no code; this is the direct proof, not an inference.)
- Gate: `./run_tests.sh --full` (run 2026-08-16): **RESULT: PASS** — parallel
  phase **5667 passed, 3 xfailed in 480.57 s**; serial phase **9 passed,
  5670 deselected, in 40.79 s**; total 528 s. (+2 collected items over the
  pre-change tree: the two new xlang parametrize cases. The native `TEST_CASE`
  adds no pytest item — it runs inside the existing
  `test_native_platform_relay_binary_passes`, whose binary went 10 → 11 doctest
  cases and 168 → 174 assertions.)

## Open Questions

- Six other messages still have no round-trip test — `CmdResultFrame`,
  `BbAxisEstimates`, `LegCmd`, `ClockDiag`, `CacheDiag`, `RingDiag`. Closing the
  class (a completeness guard asserting every `gen.MESSAGES` entry appears in the
  round-trip list) was deliberately left out of scope here; the `CLAP_DIAG`
  message Phase 2b adds would land in the same hole.
