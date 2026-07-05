---
title: Can-hub hardening item 20 — eight small firmware follow-ups (Fable-5 Tier-2)
type: feature
date: 2026-07-05
status: fix-landed-pending-hardware-confirm
phase: "Tier-2 / item 20"
related_plan: canhub-hardening.md
related_entries:
  - 2026-07-02-canhub-hardening-tier2
  - 2026-07-05-canhub-marginal-can3-diagnosis
commits:
  - f806ec0
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_homing.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/version_check.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/native/test_version_check.cpp
  - tests/firmware/native/test_udp_framing.cpp
subsystem:
  - firmware
  - can
  - config
  - testing
tags:
  - hardening
  - firmware
  - fable-5
  - item-20
---

## Summary

Item 20 of the can-hub hardening plan (`canhub-hardening.md`): the eight small,
independent firmware follow-ups the Fable-5 review folded into "the next firmware
phase". All landed on top of the concurrent CAN3-diagnosis session's presence-gate
work (`b8b6faf`, merged 2026-07-05) — nothing in that investigation was reverted.
Two operator-confirmed decisions drove the ambiguous items (REBOOT-during-stow →
reject; cmd-result ring → drop-oldest).

| # | Item | Change |
|---|------|--------|
| 1 | version-sweep re-query | Re-query present-but-UNRECEIVED axes (paced, 1 s) so a lost Get_Version reply no longer leaves an axis's version permanently missing (a spurious Jetson `validate_group` mismatch). |
| 2 | REBOOT-during-stow interlock | `REBOOT_ODRIVES` now returns `ERR_REJECTED` while `fault_stow_pending()` — a reboot mid-descent would disarm the raised legs (gravity drop). Mirrors the cold-start requests. |
| 3 | stow-gate fatal/E-STOP | **Documented** (not a logic change): the deferred stow deliberately completes through a guard E-STOP / ODrive-fatal because aborting it would gravity-drop the raised legs. |
| 4 | SETTLE_STOP_US from config | Sourced from a new `Homing::STOP_SETTLE_MS` (`jugglebot_homing.stop_settle_ms: 5`) — was a hardcoded `5000ull` whose comment named a `Homing::` constant that never existed. |
| 5 | HOME via axis-6 policy | **Documented**: `HOME` bypasses the `hand_axis6_permitted` allow-table; `homing_request` is the single enforcement point for the HOME path (legs 0-5 + hand axis 6), consistent with the table. |
| 6 | RESULT_BUF_CAP static_assert | `static_assert(RESULT_BUF_CAP >= sizeof(ResultAxisVersions))` — a future result-blob growth fails the build instead of silently truncating. |
| 7 | encode_frame nullptr guard | C++ `encode_frame` returned a garbage frame when `len>0 && payload==nullptr`; now returns 0. Codegen change + regen (no wire-layout / PROTOCOL_VERSION impact). |
| 8 | cmd-result ring drop policy | Drop-OLDEST (was drop-newest) — the loud CAN1 command-outcome channel keeps the freshest results under a burst, not stale ones. |

Native tests where logic warranted: item 2 (REBOOT reject, `test_rpc_dispatch`),
item 1 (re-query, `test_version_check`), item 7 (nullptr guard, `test_udp_framing`).

## Context

After the Tier-2 flash (`logbook/2026-07-02-canhub-hardening-tier2.md`) and the
marginal-CAN3 investigation closed (`2026-07-05-canhub-marginal-can3-diagnosis.md`,
CAN3 session), item 20 was the remaining bundle of small firmware hardening the
review deferred to a later flash. The CAN3 session owned `Teensy_code_canbridge/`
until 2026-07-05; this work began only after it landed (`git fetch` + fast-forward),
and re-read every current file before editing so the presence-gate changes on
`can_*_send()` were preserved.

## Discussion

*(Written before Verification. The three non-obvious items:)*

### Item 4 — the config source the comment named did not exist

`SETTLE_STOP_US` was `5000ull` with a comment "mirrors `Homing::HOMING_STOP_SETTLE_MS`
(5 ms)". Verifying against ground truth (the "empirical probe before trusting a
plan-author hedge" discipline), **there is no `Homing::HOMING_STOP_SETTLE_MS`** —
the only `HOMING_STOP_SETTLE_MS` lives in the `BBHand` namespace (the Ball Butler
hand, a different subsystem). So the item's literal premise ("source it from that
constant") rested on a false comment.

Two honest resolutions: (a) add a real leg-homing settle constant to the config, or
(b) keep it local and just fix the misleading comment (as the sibling
`SETTLE_DRIVE_US` already is — a firmware-local timing). I chose (a): the 5 ms
"delay after stopping before set_absolute_position" is the canonical ODrive-homing
stop-settle (documented identically in the BB-hand config), leg homing uses the same
recipe, and putting it in `jugglebot_homing.stop_settle_ms` gives it a single source
of truth in the leg-homing's OWN config section — cleanly honouring "from config"
without the semantically-wrong cross-namespace coupling to `BBHand::`. The cost: a
shared-config regen ripples the additive constant into every subsystem's generated
config (cone / platform-Teensy `hardware_config.h`, jugglebot `hardware_config.py`),
inert there (none read it); only the can-bridge copy is actually consumed
(`leg_homing.cpp` sources `Homing::STOP_SETTLE_MS`) — the price of a config citizen.

### Item 2 — reject REBOOT during a stow (not just document it)

A `REBOOT_ODRIVES` landing during a pending or in-progress deferred stow reboots the
leg ODrives → disarm → the raised legs gravity-drop — the exact hazard class as the
DEACTIVATE-during-stow reject the Tier-2 review already fixed. Operator-confirmed to
**interlock** (reject) rather than keep REBOOT as an always-available recovery
escape hatch: the safety default (never disarm raised legs mid-descent) wins, and a
stow completes in ~2-3 s after which REBOOT is available again. Rejects every axis
(incl. a hand-only reboot) for a simple, conservative gate that mirrors the
cold-start requests' `fault_stow_pending()` reject exactly.

### Item 8 — drop-oldest for the loud command-outcome channel

The cmd-result uplink ring was drop-newest. For a channel whose whole purpose is
surfacing the outcome of the operator's LAST command, dropping the newest under a
burst discards exactly what they are waiting on. Operator-confirmed to **drop-oldest**
(keep the freshest outcomes). Safe to advance the tail from the producer because the
push and `can_cmd_result_pop` are mutually exclusive under the shared PRIMASK
critical section (both mask IRQs on this single-core M7 — not a lock-free head/tail
split, so there is no producer/consumer tail race).

### Item 1 — a version-sweep re-query needed a test-isolation fix

The re-query keys on `s_received_mask` (not `s_query_sent_mask`) so a replied axis is
never re-queried, paced by a monotonic `s_last_query_us` so a never-replying axis is
a 1 s trickle, not a per-tick flood at the 100 Hz task rate. Its native test surfaced
a latent test-isolation gap: `version_check_init()` cleared the masks but not the raw
cache, so a `version_record()` in one case leaked stale raw bytes into a later case's
`fill_blob`. Fixed by zeroing the raw cache in `init` too (zero production impact —
boot statics are already zero; the function is explicitly the test-isolation / re-arm
seam, so full clearing aligns with its purpose).

## Verification

- **Full suite** (`pytest tests/ -q`, run 2026-07-05): **2049 passed, 1 xfailed in
  481.29 s** — includes the native harness (build + run all 13 binaries + fault-golden
  conformance; the item-3 comment-only change left the golden byte-identical) and the
  config-generation in-sync guards after the `hardware_config` + `udp_protocol` regens.
  Order-flaky alloc tests confirmed isolated as usual (memory
  `project_hot_loop_alloc_test_flaky`).
- **Native binaries** (`python tests/firmware/native/build.py`): all 13 build + pass on
  the Jetson g++. New cases: `test_rpc_dispatch` REBOOT-during-stow reject (item 2),
  `test_version_check` lost-reply re-query (item 1), `test_udp_framing` null-payload
  guard (item 7).
- **On-target compile** (`pio run -e teensy41`, 2026-07-05): **SUCCESS** — 225600 text /
  35520 data / 107168 bss (fits the Teensy 4.1; the `RESULT_BUF_CAP` static_assert
  compiled clean). Only the pre-existing benign `FlexCAN_T4.tpp` strncpy warning.
- **Flash (2026-07-05)**: flashed to the can-bridge Teensy in one Phase-2 cycle (with
  [18A], `PROTOCOL_VERSION` 2). A cold-boot HOMING interrupted by a CAN3 drop re-homed
  cleanly on reconnect (operator, 2026-07-05). The dedicated per-item powered checks
  (REBOOT-during-stow reject, cmd-result drop-oldest under a burst) remain a
  nice-to-have at a fuller sitting.

## Related

- Plan: `plans/active/canhub-hardening.md` (Tier-2 item 20).
- Predecessor: `logbook/2026-07-02-canhub-hardening-tier2.md` (the Tier-2 flash this
  extends); `logbook/2026-07-05-canhub-marginal-can3-diagnosis.md` (the concurrent
  CAN3 work this landed on top of).
- Deferred to its own cycle: **[18A]** HomingResult uplink (wire-protocol change).
