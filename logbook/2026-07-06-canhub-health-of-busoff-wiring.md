---
title: health_of() bus-off wiring — live FLTCONF joins RX staleness in the per-bus health classification
type: feature
date: 2026-07-06
status: resolved
phase: "canhub-hardening (Tier-2 residual)"
related_plan: canhub-hardening.md
related_entries:
  - 2026-07-05-canhub-marginal-can3-diagnosis   # instrumented the live fault_conf/TEC inputs this entry consumes
  - 2026-07-05-canhub-canhealth-err-wire-errors # the observability follow-up that made err= trustworthy
sessions: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/README.md
  - tests/firmware/native/test_platform_relay.cpp
commits:
  - 10beb10   # firmware(canbridge): health_of bus-off wiring - live FLTCONF joins the per-bus health classification
subsystem:
  - canbridge
tags:
  - can-bus
  - firmware
  - bus-health
  - safety-gate
---

## Summary

`health_of()` — the per-bus classifier behind the WARN/BUS_OFF command gates
(`jugglebot_commands_allowed()`, the homing/activate/deactivate gates) and the
`bus1_health`/`bus2_health` heartbeat uplink slots — classified purely on RX
staleness since the original port, with a shipped `TODO(bench): read the
FlexCAN error/bus-off registers for WARN/BUS_OFF`. `BUS_OFF` was unreachable:
the host's `has_fatal_odrive_error`-style OR-term on it, the shutdown-stow
skip, and part of the recovery-edge trigger were dead code. This entry wires
the live ESR1.FLTCONF fault-confinement state (sampled per 1 kHz service tick
from the same ESR1 read that already feeds `synced`) into the classification,
per-bus and non-sticky. Bench-validated with two operator-run powered checks
(BB CAN1 unplug/replug; CAN3 12 V supply cycle), which also surfaced and
characterised one new known-benign signature: a mid-session 12 V ramp punches
the sticky `flt=` high-water to `BUSOFF` via a real milliseconds-scale,
self-healing bus-off excursion.

## Problem

The prerequisite instrumentation (live `tec_live`/`rec_live` ECR sampling,
sticky `fault_conf`, live `synced`) landed during the 2026-07-05 marginal-CAN3
investigation, leaving the wiring itself as the last Tier-2 residual on the
canhub-hardening plan. Without it:

- a bus in error-passive or bus-off with frames still flowing classified `OK`
  and accepted motion commands;
- `BusHealth::BUS_OFF` never appeared on the wire, so three host behaviours
  keyed on it could never fire;
- the sticky `fault_conf` was the only confinement signal, and it latches
  worst-ever — unusable for a recovering classification.

## Fix

New classification, one pure header-inline function
(`classify_bus_health(last_rx_us, now_us, flt_live)` in `can_buses.h`,
natively pinned — same pattern as `bus_partner_present`):

1. never seen a frame → `UNKNOWN` (unchanged; bring-up allowed, the TX
   presence gate independently refuses sends)
2. live FLTCONF = bus-off → `BUS_OFF`
3. live FLTCONF = passive OR RX stale > 2 s → `WARN`
4. else `OK`

Plumbing: `BusRxHealth.flt_live` (clamped 0/1/2) sampled in `service_bus()`
from the existing per-tick ESR1 read (zero new register traffic);
`health_of()` reduced to a wrapper over the classifier (TODO closed);
`can_buses_stats()` feeds each bus its own `flt_live`; `snapshot_bus()` copies
the field; `[canhealth]` prints it as `fltNow=` beside the sticky `flt=`;
README serial-console reference updated (`fltNow` row, `flt` known-benign
ramp signature). Native truth-table test pins the severity ordering, the
2 s staleness boundary, the ≥2 clamp, and UNKNOWN-dominates-never-seen.

## Discussion

- **WARN keys on error-passive (128), not the CAN warning level (96).** The
  2026-07-05 power-cycle capture measured the 12 V supply-ramp RX burst at
  REC ≈ 121: a 96-threshold would flag WARN — and refuse motion — on every
  normal CAN power-on; 128 sits above the measured benign envelope. Warning-
  level detail stays visible via `tecNow`/`recNow` on `[canerrs]`. Operator
  confirmed this choice explicitly before implementation.
- **Live FLTCONF, not the sticky `fault_conf`.** The sticky high-water would
  latch WARN/BUS_OFF forever after any recovered transient (e.g. the ramp
  signature below would permanently gate CAN3 after every mid-session 12 V
  cycle). The live read recovers with the bus; bus-off itself cannot latch
  because FlexCAN_T4 never sets BOFFREC (macro defined, never referenced), so
  the controller hardware auto-recovers after 128×11 recessive bits.
- **Gate + report, not report-only.** A passive/bus-off controller has
  degraded-to-zero ability to deliver frames; commanding motion through it
  risks half-delivered command ladders. The existing WARN-refuse semantics
  already encode this — the wiring extends the *inputs*, not the policy.
- **Per-bus independence (operator-stipulated constraint).** Each bus is
  classified from its own registers/timestamps only; the gates consume only
  `jugglebot_health`, the host's fatal-CAN OR-term/stow-skip/recovery-edge
  consume only `bus1_health` (CAN3). A CAN1 fault cannot gate CAN3, and the
  BB-unplug check confirmed it live (CAN3 stayed `fltNow=active gated=0`
  throughout).
- **Why a frozen TEC=128 is safe to gate on.** TEC only decays on successful
  TX, so a closing-window pin could in principle hold WARN against a healthy
  bus. It self-clears within one 0x7DD period once time-synced +
  partner-present — the 100 Hz broadcast IS the decay pump, and it shares its
  origin (ROS2) with any command that could race it. Validation added a second
  mechanism with the same outcome: during an outage the already-queued TX
  mailbox frame retries indefinitely (FlexCAN auto-retransmit), holding
  passive via ~2/s ACK-error snapshots (`0x00042252` ring words) — TEC stays
  capped at 128 by the passive-ACK rule, classification unchanged (WARN,
  agreeing with staleness).
- **The ramp punch-through — a prediction miss, characterised and accepted.**
  The validation recipe said `fltNow` must "never show BUSOFF" during the 12 V
  cycle. The live field held (no printed sample showed it, and no gate
  misbehaved), but the sticky `flt=` recorded `BUSOFF` + `tec=254`: TEC enters
  the ramp already pinned at 128, the presence gate reopens on the FIRST valid
  frame from the fastest-booting ODrive while the bus is still electrically
  dirty for a few hundred ms, and the resumed TX eats BIT1/STUFF errors
  (`bit1` 2→59, `stuff` 1→22, `tecInc` +809 ≈ 101 errors × 8) — punching
  through 256 into a genuine, milliseconds-scale bus-off that auto-recovers
  before the next 1 Hz print (`tecNow=0` in the same sample). The gate
  contract comment had flagged ramp escalation as possible; the recipe's
  PASS wording hadn't. **Decision (operator): document as known-benign, do
  not harden.** The considered alternative — reopen the presence gate only
  after a ~200 ms clean-bus window ("partner present AND medium clean") —
  would close the class but adds reconnect-timing behaviour (delayed 0x7DD
  resumption) plus its own validation cycle, for a purely cosmetic benefit:
  the excursion is self-healing, functionally invisible (`fault=6` already
  covers the whole window, so no new `has_fatal_can_error` edge is possible),
  and fully disambiguated by `[canerrs]`. Recorded in the README `flt` row so
  a future session doesn't re-open the ghost hunt. The fresh-BOOT rule
  ("flt/tec must never exceed passive/128 on a fresh boot") is unaffected —
  sticky state resets at boot.
- **UNKNOWN outranks the register states.** Bus-off before the first RX frame
  is structurally unreachable (bus-off needs TEC ≥ 256, which needs TX, which
  the presence gate refuses until a first frame arrives), and REC-driven
  passive on a never-seen bus is harmless to allow (the presence gate refuses
  TX there anyway) — so never-seen keeps its boot bring-up semantics exactly.
- **The Phase-6 recovery carve-out gets MORE necessary.** A just-repowered bus
  can now read WARN from the frozen-TEC pin (not just staleness) until the
  first successful TX decays it; `jugglebot_bus_transmittable()` (live SYNCH)
  is exactly the path that lets CLEAR_ERRORS/REBOOT reach the bus through that
  state. Untouched by this change; its comment updated to say so.

## Verification

- Native firmware suite (`pytest tests/firmware/test_native_firmware.py -q`,
  run 2026-07-05): **16/16 pass in 170.23 s** — includes the new
  classify_bus_health truth-table doctest case.
- Full suite (`pytest tests/ -q`, run 2026-07-05): **2054 passed, 1 xfailed
  in 486.53 s** (post-implementation gate). Re-run after the audit's
  doc/comment-only fixes (`pytest tests/ -q`, run 2026-07-06): **2054 passed,
  1 xfailed in 651.63 s** (pre-commit gate).
- Firmware compiled (`pio run -e teensy41`) and flashed 2026-07-05; robot
  idle-verified (no ROS2/run_mpc processes) before flash.
- 45 s idle baseline post-flash: all three buses `err=0 flt=active
  fltNow=active gated=0`, zero `[canerrs]`/`[canesr1]` lines
  (`temp/probes/2026-07-05-marginal-can3/canhealth_busoff_wiring_baseline.txt`).
- **Check 1 (BB CAN1 unplug/replug, operator-run): PASS** — `bb` reached
  `fltNow=passive` unplugged, recovered to `fltNow=active tecNow=0` on replug
  (sticky `flt=passive tec=135` correctly retained); CAN3 untouched
  throughout (`..._check1_bb_reconnect.txt`). Mid-outage `fltNow=passive` was
  observed live on the operator's console; the cited capture is a
  post-recovery excerpt (see its header) whose sticky `flt=passive tec=135`
  corroborates the excursion.
- **Check 2 (CAN3 12 V cycle, operator-run): PASS** on all live criteria —
  `fltNow=passive` + `fault=6` through the outage, `fltNow=active`/`fault=0`/
  `fresh=7/7` after recovery, host logged the "CAN3 bus health recovered to
  OK" conservative cold-start re-read exactly once (operator-confirmed in the
  ROS2 session). Sticky `flt=BUSOFF tec=254` = the known-benign ramp
  signature analysed above (`..._check2_12v_cycle.txt`).

## Outcome

`BusHealth::BUS_OFF` is now reachable end-to-end: a live controller bus-off on
CAN3 gates motion in firmware and reaches the host's fatal-CAN OR-term,
shutdown-stow skip, and recovery-edge re-read — all previously dead code. A
bus that is actively erroring (error-passive) now refuses motion even while
frames still flow, per-bus, with the measured benign envelope (12 V ramp REC
burst) safely below the threshold. The last shipped `TODO(bench)` in the bus
health path is closed. The canhub-hardening plan was archived by a parallel
session on 2026-07-05 carrying this wiring as its open-item 2 (explicitly
owned by this session) — that item is now backfilled DONE in the archived
plan; of its open items only the deliberately-deferred Tier-3 item 21, the
firmware-comment residue, and two nice-to-have targeted powered checks
remain.
