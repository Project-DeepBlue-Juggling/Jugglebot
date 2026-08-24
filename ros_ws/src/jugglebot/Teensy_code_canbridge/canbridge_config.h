#pragma once
// =============================================================================
//  canbridge_config.h — firmware-wide configuration for the can-bridge Teensy
// =============================================================================
//  Hand-authored (NOT codegen). Pulls protocol/hardware constants from the
//  generated headers and adds firmware-only configuration: pin map, FreeRTOS
//  task table, and the control/safety constants ported 1:1 from the Jetson
//  Python (motor_guard.py / can_node.py) so behaviour is identical.
//
//  Single source of truth for any value that also lives in Python is the
//  generated config; values mirrored here from Python source files are marked
//  with their origin so a future codegen pass can hoist them.
// =============================================================================

#include <cstdint>
#include "protocol_config.h"   // CAN protocol: CanBus, ODriveCmd, NodeId, SharedCanId, InputScale...
#include "hardware_config.h"   // Geometry, ODrive limits, operational constants
#include "udp_protocol.h"      // JbUdp:: wire protocol (ports, IPs, framing)

// ── BENCH_SYSID_BUILD — one-off bench-measurement variant (NEVER production) ──
// DEFAULT OFF. A dedicated compile-time flag that lifts three measurement blind
// spots for on-wire gain-ladder / system-ID runs on the single-leg bench rig:
//   * 250 Hz telemetry (was 100), so a pos_gain-90 loop (~14 Hz closed-loop BW)
//     is not telemetry-aliased;
//   * 100 Hz knots (SEGMENT_T_S 0.010, was 0.025) — the honest chirp ceiling is
//     otherwise KNOT-bound at 8 Hz (knot_stream_top_freq = knot_hz/5); the Jetson
//     harness MUST then run --knot-hz 100 to match this segment duration;
//   * a 250 Hz forced DIAGNOSTIC on axis 0, so iq_measured (only carried by the
//     on-change-gated DIAGNOSTIC) is observable at rate for the current-rail and
//     vel_gain-buzz ceilings the on-change gate hides.
// It changes CADENCE and internal timing ONLY — ZERO wire-format changes (no
// struct/field/protocol-version edit). With the flag OFF the binary is IDENTICAL
// to production (every use sits behind #if BENCH_SYSID_BUILD). Build the variant
// with `pio run -e teensy41_bench_sysid` (platformio.ini adds -DBENCH_SYSID_BUILD=1);
// DO NOT flash it to the assembled robot.
#ifndef BENCH_SYSID_BUILD
#define BENCH_SYSID_BUILD 0
#endif

namespace CanBridge {

// ── Identity ────────────────────────────────────────────────────────────────
constexpr char     FW_NAME[]    = "jugglebot-canbridge";
constexpr uint16_t FW_VERSION   = 16;         // bump on behavioural change (1→2: 2026-07-16 MAX_DEVIATION_REV 0.5→1.0 guard raise; 2→3: 2026-07-24 Diagnostic bus_current + heartbeat_seen, UDP PROTOCOL_VERSION 4; 3→4: 2026-07-29 gpio_poll hand ball sensor — CAN3 get_gpio_states poll on task_homing + serial console; 4→5: 2026-07-29 CAN3 command gate keys on SUSTAINED error-passive confinement (classify_command_gate, CAN_PASSIVE_SUSTAIN_US) + additive CanErrors uplink 0x8C; 5→6: 2026-07-31 TEMP gpio_poll boots OFF — runtime default only — for the CAN3 wire-error isolation experiment; 6→7: 2026-07-31 experiment closed (root cause: bridge CAN3 analog drive path, load-dependent): gpio_poll boots ON again, jugglebot↔cone controller swap becomes the OPERATING CONFIG (jugglebot role on CAN2, cone on CAN3 — see can_buses.cpp) until the CAN3 path is repaired, cone BusHealth added to HeartbeatT2J.flags bits 4-5; 7→8: 2026-07-31 Profile gains wire slot 3 = cone role traffic (can3_rx/tx/util_x100, appended — UDP PROTOCOL_VERSION 4→5, Jetson must run the matching checkout); 8→9: 2026-08-02 ERR_TIMEOUT attribution — additive BridgeTxDiag 0x8D (per-bus tx_deferred/tx_q_hwm + hand_ops per-stage exit counters) and BridgeIdentity 0x8E (this constant, finally on the wire); PROTOCOL_VERSION stays 5; 9→10: 2026-08-09 ERR_TIMEOUT FIX — can_jugglebot setMaxMB 16→24, i.e. 8→16 TX mailboxes (the 6-frame 500 Hz leg burst no longer starves a hand dispatch of a free mailbox; bench-measured 15/40 → expected 0/40), plus a console-only [handphase] interp-phase-stamp diagnostic on HAND_TRAJ_CMD and the corrected NVIC-priority comment in can_buses.cpp. NO WIRE CHANGE — no MsgType, no payload, PROTOCOL_VERSION stays 5, so an FW 9 board and an FW 10 board are wire-identical and the skew is purely behavioural; 10→11: 2026-08-11 bridge-temporal-trustworthiness P1 — ADDITIVE ClockDiag uplink 0x8F, one frame per accepted time-of-day anchor (~30 s steady state, 500 ms during the pre-first-anchor fast retry) carrying (a) the per-anchor clock-discipline sample time_base.cpp::set_wall_anchor has always computed and DISCARDED — pre-slew offset error, the exchange RTT, and the implied crystal frequency error in ppb from consecutive measurements, with STEPPED/FIRST_ANCHOR/FREQ_VALID flags so a non-crystal interval can never be read as a rate — and (b) the 500 Hz interp ladder occupancy census (total ticks, recover-slew ticks, Mode-2 cubic-Taylor extrapolation ticks) differenced over the window since the previous emit, which are the last two telemetry gaps named in the Addendum to logbook/2026-07-18-teensy-uptime-tracking-degradation.md. Existing frames are BYTE-IDENTICAL and PROTOCOL_VERSION stays 5, so an FW 10 board and an FW 11 board remain wire-compatible in both directions — which means, exactly as with 9→10, a healthy link is NOT evidence this build is aboard: confirm the flash on the BRIDGE_IDENTITY frame (link_status/bridge_fw_version), never by inference. DELIBERATELY WRITTEN AND COMMITTED UNFLASHED: a flash reboots the Teensy and destroys the aged uptime state the S1 isolation experiment exists to interrogate, so the flash window opens at S1's Arm C ('plans/archived/bridge-temporal-trustworthiness.md', archived 2026-08-15, owner decision 3); 11→12: 2026-08-12 bridge-temporal-trustworthiness — ADDITIVE CacheDiag uplink 0x91, 1 Hz, the CONFIRMATION INSTRUMENT for the one question S1 left open. S1 localized the uptime command-latency drift to the Teensy (63 h uptime: 290-340 ms end-to-end leg lag, lead clamp pinning the executed command at fb+MAX_LEAD_REV on 44-74 % of ticks vs 5.8 % fresh, while udp_rtt_us stayed flat at 1-3 ms, interp_deadline_misses 0, heap flat, CAN throughput flat, ODrives and the Jetson-side link exonerated), leaving exactly two survivors: the encoder cache (axes[i].pos_rev / pos_timestamp_us, written by can_buses.cpp decode_into_cache) is going STALE with uptime, or the leg genuinely trails. /robot_state and /leg_cmd_executed both read that same cache, so no frame on the wire today can separate them. This one can: per-axis cache-age MINIMUM and MAXIMUM over each 1 s window (the age is sampled every 100 Hz telemetry tick through axis_state.h's existing snapshot_pos_vel seqlock and reduced task-side — a FLOOR, not a lucky instant), plus seen_mask, the window length and sample count, plus the rxBuffer depth_hwm / drain cap_hits per bus and the decode_short / decode_bad_axis discards, all of which have been computed on-chip since 2026-06-04 / 2026-07-05 and were NEVER uplinked (CanErrors 0x8C deliberately carries only wire-error and fault-confinement fields), plus a NEW per-axis get_encoder_estimate frame counter (enc_frames[7], cumulative, incremented in can_buses.cpp decode_into_cache AFTER write_pos_vel so that "counter advanced ⇒ that axis's cache write completed"). The counter is the one field that is not merely a promotion of existing on-chip state, and it earns its place: the S1 bag forensics found the per-axis cache VALUE stalling 30-500 ms in a fat tail (9-18 % of refresh intervals > 30 ms aged vs 4.3 % fresh) while every AGGREGATE RX counter stayed flat — which is not a contradiction, because an aggregate cannot see ONE axis of seven go quiet. Differenced across a stall: still advancing ⇒ frames arrived and the decode ran (a stale estimate came in over the wire); paused ⇒ nothing arrived (the ODrive's broadcast or a per-axis bus loss). Different faults, different owners, no other field separates them. The CacheDiag work is INSTRUMENTATION ONLY — no new ISR work, no new IRQ-off window (the seqlock does not mask interrupts), every WINDOW accumulator written and read solely by task_telem (the promoted RX counters keep their existing single-writer task_can_rx discipline), and nothing in the firmware reads a single field of it. The FW 12 image ALSO carries one control-path fix, separate commit: fault_machine.cpp's MOTOR_FB_STALE guard read pos_timestamp_us with a naked (tearable) u64 load and no now>ts ordering guard — both halves could flap the guard (false ~100 ms output suppression, or a one-tick missed real freeze); now atomic_read_u64 + ordering-guarded, matching the file's heartbeat-guard reads on the atomic half (their ordering half is a flagged follow-up). Existing frames are BYTE-IDENTICAL and PROTOCOL_VERSION stays 5, so an FW 11 board and an FW 12 board remain wire-compatible in both directions — which means, exactly as with 9→10 and 10→11, a healthy link is NOT evidence this build is aboard: confirm the flash on the BRIDGE_IDENTITY frame (link_status/bridge_fw_version), never by inference. 0x91 opens a NEW uplink id block above RpcResponse 0x90 because the 0x81-0x8F block is full; nothing was renumbered, and nothing anywhere routes on a MsgType range; 12→13: 2026-08-14 bridge-temporal — ADDITIVE RingDiag uplink 0x92, 1 Hz, the CONVICTION INSTRUMENT for the FlexCAN_T4 RX-ring `_available` leak, which is the surviving candidate mechanism after S2 (2026-08-13) killed the cache-AGE hypothesis (cache age was FRESHER at 28 h than at 1.3 h; the real signature is ~100-150 ms bit-identical per-axis telemetry freezes that the freshness-blind lead clamp amplifies). THE DEFECT, assembly-verified 2026-08-14 against this toolchain and recorded in lib/FlexCAN_T4/PROVENANCE.md: FlexCAN_T4::events() pops the RX ring BEFORE its NVIC_DISABLE_IRQ guard, and Circular_Buffer's `_available` is read-modify-written non-atomically by BOTH the CAN ISR (increment on push) and that unguarded task-side pop (decrement). The race is ONE-DIRECTIONAL — the ISR preempts the task, never the reverse — so ISR increments are swallowed and `_available` monotonically UNDER-counts. can_buses.cpp's drain loop (do { events(); } while (++n < 32 && rx_remaining)) then exits believing the ring is empty while a residue D is stranded, and from that point every frame it delivers is D frames old. D ratchets with uptime and caps at one ring depth — 256 slots, ~114-135 ms at jugglebot-bus rates — which is the right order for both the 290-340 ms lag S1 measured at 63 h and the ~100-150 ms freezes S2 measured directly. WHY A NEW FRAME WAS UNAVOIDABLE: getRXQueueCount() returns `_available`, so depth_hwm and cap_hits (uplinked since FW 12) are computed FROM the corrupted number and are blind to this failure by construction — they read healthy through a fully-leaked ring. The frame carries, per bus and per 1 s window: TRUE occupancy from the ring's head/tail probed immediately AFTER the drain loop (where `_available` is 0 by definition, so the residue IS the leak), the `_available` reading at that same probe, the leak's high-water mark over every 1 kHz service tick, and the hardware FIFO overflow/warning counts that upstream FlexCAN_T4 clears without counting; plus two independent cross-checks on the jugglebot bus — a delivery lag derived from the FlexCAN HARDWARE capture timestamp (16-bit, unwrapped into a 64-bit arrival clock, reseeded rather than silently corrupted when a gap erodes the wrap margin) and the hand ball-sensor SDO round-trip floor, which must grow by exactly the ring delay. A leak of D moves all three together; a bus or ODrive fault does not. TWO VENDORED-LIBRARY PATCHES ship with it (P1 read-only true-occupancy accessors, P2 ISR-side FIFO overflow/warning counters), both minimal, both marked `// JUGGLEBOT PATCH` at the site and listed in PROVENANCE.md. THE LEAK ITSELF IS DELIBERATELY NOT FIXED HERE: the fix waits on the occupancy number so it can be judged against a measurement rather than a theory. NO CONTROL-PATH CHANGE — nothing in the firmware reads any field or accumulator introduced for this frame, every reduction happens where the data already lives, and no new IRQ-off window is opened on any hot path. Existing frames are BYTE-IDENTICAL and PROTOCOL_VERSION stays 5, so an FW 12 board and an FW 13 board remain wire-compatible in both directions — which means, exactly as with 9→10, 10→11 and 11→12, a healthy link is NOT evidence this build is aboard: confirm the flash on the BRIDGE_IDENTITY frame (link_status/bridge_fw_version), never by inference; 13→14: 2026-08-14 bridge-temporal — THE FIX. FW 13's RING_DIAG CONVICTED the FlexCAN_T4 RX-ring `_available` leak on hardware: on a 4.03 h board /ring_diag read leak_jb = 247-248 (true_depth 247-248 against avail_reported 0; hwm 249, i.e. ~97 % of one full 256-slot lap) on the 500 Hz-loaded jugglebot bus, against 1 on bb and 0 on cone — exactly the arrival × pop rate ordering the mechanism predicts, and matching the predicted ~90 slots/h ratchet — while end-to-end leg lag on the same board read 19.9 ms fresh and 252.2 ms at 3.80 h. That is a measurement, not a theory, so the fix that FW 13 deliberately withheld now lands. TWO VENDORED-LIBRARY PATCHES, both in lib/FlexCAN_T4/FlexCAN_T4.tpp events(), both marked `// JUGGLEBOT PATCH` at the site and listed with full rationale in lib/FlexCAN_T4/PROVENANCE.md. P3 — THE LEAK, FIXED AT THE POP: the RX pop (the payload memmove out of _cabuf[head], the `head` advance and the `_available--`, all non-atomic RMWs) now runs inside a NVIC_DISABLE_IRQ(nvicIrq) / NVIC_ENABLE_IRQ(nvicIrq) window with `dsb; isb` after the mask write, so the CAN ISR's mirror-image `_available++` can no longer be swallowed by a collision with it. Masking the WHOLE pop rather than only the two bookkeeping RMWs is deliberate on two counts: a narrower guard would have to live in Circular_Buffer::read()/readBytes(), which are shared with busESR1/busECR and every other Circular_Buffer instance and have no idea which NVIC line to mask (the narrower-looking guard is the wider blast radius), and it would leave the payload memmove racing the ISR's overwrite-oldest push — the ring-full frame TEAR that was secondary find 1 of the same audit. One guard closes the leak and the tear. IRQ-OFF COST, bounded: ~24-byte memmove plus two 16-bit RMWs, under 0.1 us at 600 MHz, at most 32 such disjoint windows per 1 kHz drain tick (CAN_RX_DRAIN_BUDGET) — a few us per ms of THAT BUS's OWN CAN IRQ masked, against a 6-deep hardware RX FIFO that tolerates ~690 us of ISR latency at 1 Mbit/s before it can overflow; and the pre-existing TX window immediately below already masks the same line for longer. Dispatch order is unchanged — mbCallbacks still runs outside every mask and still before the TX section. After the patch `head` and `_available` still have two writers each but are MUTUALLY EXCLUDED (the ISR cannot preempt the masked window; task context can never preempt the ISR), which is the property that matters; the ISR-side write() needs no patch, and the full-ring `head` advance is covered by the same window. P4 — the library's OTHER confirmed defect, the events() `mb == -1` TX-deferral refill loop's missing `break` (one peeked deferred frame written into EVERY free TX mailbox while pop_front discards the next queued frames unsent): one `break;`, disarming a mine rather than changing behaviour — every bus runs tx_deferred == 0 (jugglebot since the FW 10 setMaxMB raise, bb/cone never), so the branch is currently UNREACHABLE and this patch CANNOT alter live behaviour. It is fixed now because the FW 10 mailbox raise DOUBLED its blast radius (8 → 16 mailboxes) and because the path re-opens the instant any future TX producer re-introduces deferral; can_buses.cpp's standing "must fix the vendored loop" warning is thereby discharged. RING_DIAG (0x92) IS RETAINED UNCHANGED — it is the fix's own proof, and post-fix acceptance is leak ≡ 0 on all three buses at any uptime, read by the same instrument that convicted the defect (the leak criterion only — full FW 14 acceptance, incl. aged-soak e2e lag ≤ 20 ms and the two S3 residual reconciliations, is plan § S3 RESULTS). NO WIRE CHANGE of any kind — RingDiag, CacheDiag and ClockDiag are byte-identical, no MsgType and no payload moves, PROTOCOL_VERSION stays 5 — so an FW 13 board and an FW 14 board are wire-identical and the skew is purely behavioural, which means, exactly as with 9→10, 10→11, 11→12 and 12→13, a healthy link is NOT evidence this build is aboard: confirm the flash on the BRIDGE_IDENTITY frame (link_status/bridge_fw_version), never by inference; 14→15: 2026-08-18 hand END-STOP correction — BEHAVIOURAL, safety clamp. Geometry::HAND_MOTOR_HARD_STOP_REVS (was HAND_MOTOR_MAX_POSITION_REVS) 11.1 → 10.8 rev, the operator-measured metal contact on the sensorised hand. This board consumes it as HAND_MOTOR_MAX_POSITION (canbridge_config.h) → odrive_protocol.h::clip_position, so the hand setpoint clamp tightens from [0, 11.1] to [0, 10.8] rev: an FW 14 board will pass commanded setpoints up to 0.3 rev (9.5 mm) PAST metal, an FW 15 board will not. NO WIRE CHANGE — no MsgType, no payload, PROTOCOL_VERSION stays 5 — so an FW 14 board and an FW 15 board are wire-identical and the skew is purely behavioural; a healthy link is NOT evidence this build is aboard, confirm on the BRIDGE_IDENTITY frame. See logbook/2026-08-18-hand-end-stop-corrected.md. FLASHED ~2026-08-20 — the board has self-reported bridge_fw_version 15 on the BRIDGE_IDENTITY frame ever since, so 15 IS the image aboard and 15 IS THE END-STOP CLAMP ALONE; nothing else folds into it; 15→16: 2026-08-24 hand ball-sensor POLLER CADENCE + TRI-STATE TX ACCOUNTING — BEHAVIOURAL, no wire change, WRITTEN AND COMMITTED UNFLASHED. This change-set was briefly folded into 15 (commit 2995855) on the false premise that 15 was still unflashed; it is not, so the poller/tri-state image takes its own number and bridge_fw_version can once again discriminate the two. (a) the hand ball-sensor poller finally reaches its configured 50 Hz. Two independent causes, both in gpio_poll_step: the AWAIT branch consumed a reply and then RETURNED, so closing a round trip cost a whole task_homing tick before the next request could be considered; and the interval restarted from `now` (post-wake-jitter) against a >= compare that sits exactly on the 2-tick boundary, so a negative wake jitter promoted a 20 ms cycle to 30 ms. Transfer function was C = 10*max(2, ceil(RTT/10)+1) ms; measured distinct-sample cadence on the healthy plant was 20 ms p50 / 30 ms p95, ~42 Hz against the configured 50, with ~38 % of cycles taking the 30 ms mode. The fix is consume-and-send in the same tick PLUS an absolute schedule (advance from the previous DUE time, with a catch-up guard that re-bases rather than replaying a backlog) PLUS a half-task-period early-fire band, which is what actually removes the boundary mode — the absolute grid alone leaves the due instants landing exactly ON nominal tick instants. Still at most ONE CAN3 TX per tick. NO SIGNAL SEMANTICS CHANGE: the tri-state verdict, the miss_count freeze, the debounce asymmetry, timeout-is-not-a-miss, REPLY_TIMEOUT_US/REPLY_STALE_US and the RTT instrument`s stamp-BEFORE-send ordering are all untouched, so no ball_held verdict and no arming gate moves as a result — only how often a fresh sample lands. (b) TRI-STATE TX ACCOUNTING. Every send used to end in `bus.write(m) > 0`. FlexCAN_T4::write(const CAN_message_t&) returns +1 (hardware mailbox) or -1 (QUEUED into the 64-slot software txBuffer, transmitted in order by the TX-complete ISR) and NEVER 0, so every caller in this firmware has been reading `queued, will transmit` as `failed` — the mechanism behind the 2026-08-09 lying-ack. Replaced by TxResult{FAILED, MAILBOX, DEFERRED}: FAILED is now ONLY the bus-partner presence gate refusing, i.e. the one outcome in which no frame exists. Per-caller rulings (owner-delegated): poller DEFERRED arms AWAIT and keeps the RTT armed (queue latency IS round-trip latency); leg setpoints DEFERRED = sent, counted, NEVER retried (latest-wins, a retry would put a stale setpoint behind a fresher one); hand traj + RPC-acked dispatch DEFERRED = sent and acked truthfully, with NO blind re-dispatch and the existing hand ladders / _MAX_ARM_DISPATCHES cap retained untouched; version sweep + RPC replies DEFERRED = sent; fault machine, CLEAR_ERRORS/REBOOT and platform relay ops DEFERRED = sent but counted on a DEDICATED safety-deferral counter, with retry still only on FAILED; time-sync beacons LEFT AT TODAY`S BEHAVIOUR (result discarded) and flagged OPEN — the 0x7DD payload is stamped BEFORE the write and the slaves` handleSyncFrame does a hard step then an unfiltered 1/8 slew with no outlier rejection, so a deferred beacon is genuinely absorbed; bounded and self-healing, but the mitigation is a three-firmware decision and is not taken here. Instrument: per-class deferral counters (poller/legs/hand/rpc/safety/timesync/coldstart) on the 1 Hz [cantx] console line. NO WIRE CHANGE of any kind — no MsgType, no payload, PROTOCOL_VERSION stays 5 — so an FW 15 board and an FW 16 board are wire-identical (as both are to FW 14) and the skew is purely behavioural, which means, exactly as with 9→10 through 14→15, a healthy link is NOT evidence this build is aboard: confirm the flash on the BRIDGE_IDENTITY frame, never by inference. EXPECTED_BRIDGE_FW_VERSION moves 15 → 16 in the same commit (the bump-at-commit precedent of 3760daa): until the flash, the live board reporting 15 raises the BRIDGE_FW_CHECK skew alarm on every launch, and that is the POINT — it is advisory only (logged, never enforced, no gate), and it is the standing reminder that the image on the bench is not the image in the tree. See logbook/2026-08-24-poller-cadence-and-tristate-tx.md)

// ── Network (static, point-to-point /30) ─────────────────────────────────────
// IP octets and ports come from the generated udp_protocol.h (JbUdp::).
constexpr uint8_t  TEENSY_IP[4] = { JbUdp::TEENSY_IP_0, JbUdp::TEENSY_IP_1,
                                    JbUdp::TEENSY_IP_2, JbUdp::TEENSY_IP_3 };
constexpr uint8_t  JETSON_IP[4] = { JbUdp::TEENSY_IP_0, JbUdp::TEENSY_IP_1,
                                    JbUdp::TEENSY_IP_2, JbUdp::JETSON_IP_3 };
constexpr uint8_t  NETMASK[4]   = { 255, 255, 255, 252 };   // /30
// No gateway, no DHCP, no mDNS (static point-to-point link).

// ── CAN bus wiring (Teensy 4.1) — three subsystem-isolated buses (ADR-0013) ───
//  One FlexCAN_T4 peripheral per robot subsystem (supersedes the old two-bus
//  shared-aux / private-leg split). Pin directions below are the FlexCAN_T4
//  silicon-fixed DEF mux (FlexCAN_T4.tpp setTX/setRX, invoked by begin()) — the
//  library default that can_buses.cpp actually runs; these constants are
//  documentation only (nothing reads them; the peripheral picks its own pads).
//
//    CAN1 = Ball Butler bus     (BB Teensy only)                  TX 22 / RX 23
//    CAN2 = catching cone bus   (cone Teensy, often disconnected) TX  1 / RX  0
//    CAN3 = Jugglebot core bus  (6 leg ODrives + Hand ODrive +    TX 31 / RX 30
//           platform Teensy 4.0 + can-bridge)
//
//  CAN3 is the FD-capable peripheral, run classical 1 Mbps today to match the
//  classical-only ODrive firmware; a future CAN-FD upgrade is a config change,
//  not a rewire. All three buses carry the 100 Hz 0x7DD time-sync broadcast.
//  NB: ADR-0013 lists CAN2 and CAN3 TX/RX reversed — the
//  FlexCAN_T4 silicon mux above is authoritative.
constexpr uint32_t CAN_BITRATE  = CanBus::BAUD_RATE;   // 1 Mbps, all nodes agree
constexpr uint8_t  CAN1_TX_PIN  = 22;   // Ball Butler
constexpr uint8_t  CAN1_RX_PIN  = 23;
constexpr uint8_t  CAN2_TX_PIN  = 1;    // catching cone (FlexCAN_T4 DEF: TX=1, RX=0)
constexpr uint8_t  CAN2_RX_PIN  = 0;
constexpr uint8_t  CAN3_TX_PIN  = 31;   // Jugglebot core (FlexCAN_T4 DEF: TX=31, RX=30)
constexpr uint8_t  CAN3_RX_PIN  = 30;

// ── Status LED ────────────────────────────────────────────────────────────────
constexpr uint8_t  LED_PIN      = 13;          // Teensy on-board LED

// ── Axis layout (mirrors odrive.py groupings) ────────────────────────────────
constexpr uint8_t  NUM_LEGS     = JbUdp::NUM_LEGS;   // 6, node ids 0..5
constexpr uint8_t  HAND_AXIS    = NodeId::JUGGLEBOT_HAND;  // 6
constexpr uint8_t  NUM_AXES     = JbUdp::NUM_AXES;   // 7 (legs + hand) — cache + telemetry breadth

// ── Loop rates ────────────────────────────────────────────────────────────────
constexpr uint32_t INTERP_RATE_HZ      = 500;     // motor_guard DEFAULT_RATE_HZ
constexpr uint32_t INTERP_PERIOD_US    = 1000000u / INTERP_RATE_HZ;   // 2000 us
constexpr uint32_t TIME_SYNC_RATE_HZ   = 100;     // 0x7DD broadcast (can_node ts_period 0.01)
constexpr uint32_t TIME_SYNC_PERIOD_US = 1000000u / TIME_SYNC_RATE_HZ;
#if BENCH_SYSID_BUILD
constexpr uint32_t TELEM_RATE_HZ       = 250;     // bench-only: un-alias the ~14 Hz pos_gain-90 loop (4 ms task tick; configTICK_RATE_HZ=1000)
#else
constexpr uint32_t TELEM_RATE_HZ       = 100;     // motor-state uplink
#endif
constexpr uint32_t TELEM_PERIOD_US     = 1000000u / TELEM_RATE_HZ;
constexpr uint32_t HEARTBEAT_RATE_HZ   = JbUdp::HEARTBEAT_HZ;   // 10
constexpr uint32_t DIAG_HEARTBEAT_HZ   = 1;       // 1 Hz forced diagnostic/profile refresh
constexpr uint32_t FAULT_TASK_HZ       = 10;      // can_node fault cadence
constexpr uint32_t WATCHDOG_HZ         = 1;       // can_node 1 Hz heartbeat watchdog
// Homing monitor. 100 Hz ≈ the Iq update rate so the current EMA tracks
// can_node's per-reading cadence (a slower tick over-smooths → laggier, but still
// safe: it detects the stall later, never harder than the ODrive current limit).
constexpr uint32_t HOMING_RATE_HZ      = 100;

// Time-of-day drift re-sync interval (design range 10-60 s).
constexpr uint32_t TIMEOFDAY_RESYNC_MS = 30000u;  // 30 s

// ── FreeRTOS task table (priorities + stacks) ────────────────────────────────
//  Priority 6 (interp) is the HIGHEST — nothing preempts the 500 Hz tick.
//  Stacks are in StackType_t WORDS (4 bytes on Cortex-M7). Budgets are in
//  bytes; *_STACK is the word count = bytes / 4.
constexpr uint8_t PRIO_INTERP      = 6;   // leg_interp (hard deadline)
constexpr uint8_t PRIO_CAN_TX      = 5;   // CAN TX mailbox drain
constexpr uint8_t PRIO_CAN_RX      = 5;   // can_rx task (pumps all three buses)
constexpr uint8_t PRIO_UDP_RX      = 4;   // usb_rx (UDP downlink)
constexpr uint8_t PRIO_TIME_SYNC   = 4;   // time_sync_master
constexpr uint8_t PRIO_UDP_TX      = 3;   // usb_tx (telemetry uplink)
constexpr uint8_t PRIO_FAULT       = 3;   // fault_state
constexpr uint8_t PRIO_HOMING      = 2;   // leg_homing monitor (rare bench op)
constexpr uint8_t PRIO_WATCHDOG    = 2;   // watchdog
constexpr uint8_t PRIO_DIAG        = 1;   // diag / profiling

constexpr uint16_t STACK_INTERP    = 2048 / 4;   // 512 words
constexpr uint16_t STACK_CAN_TX    = 1024 / 4;
constexpr uint16_t STACK_CAN_RX    = 2048 / 4;
constexpr uint16_t STACK_UDP_RX    = 4096 / 4;
constexpr uint16_t STACK_TIME_SYNC = 1024 / 4;
constexpr uint16_t STACK_UDP_TX    = 4096 / 4;
constexpr uint16_t STACK_FAULT     = 2048 / 4;
constexpr uint16_t STACK_HOMING    = 1024 / 4;   // state machine + a few CAN sends
constexpr uint16_t STACK_WATCHDOG  = 1024 / 4;
constexpr uint16_t STACK_DIAG      = 8192 / 4;   // bumped from 1024 (7-arg printf overflowed), again from 4096
                                                 // for the 13-arg [canerrs] diagnostic printf (2026-07-05)

// ── Interpolator constants (PORTED 1:1 from motor_guard.py) ──────────────────
//  Source: ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py constants block.
#if BENCH_SYSID_BUILD
// Bench-only: 100 Hz knots. SEG_T is the Hermite segment span and MUST equal the
// actual inter-knot interval — the interpolated velocity uses invT = 1/SEG_T and
// v1 = (u2-u1)/SEG_T, so a SEG_T that disagrees with the send cadence distorts the
// velocity profile (each segment re-latches at s = dt/SEG_T before completing). The
// Jetson harness must run --knot-hz 100 so knots actually arrive every 10 ms.
constexpr float    SEGMENT_T_S          = 0.010f;  // 100 Hz knots (matches --knot-hz 100)
#else
constexpr float    SEGMENT_T_S          = 0.025f;  // _mpc_segment_T (nominal MPC fine step)
#endif
// MAX_EXTRAP / EXTRAP_DECAY / JERK_EMA are NOT knot-cadence-derived: they are the
// missed-knot graceful-degradation windows (Taylor-extrapolate then decay velocity
// to a safe stop) and the jerk-EMA memory, all consumed ONLY when has_next is false.
// The bench harness streams Hermite (has_next true), where s is clamped to 1.0 and
// these never fire; and they are wall-clock safety ramps (a leg coasting to rest),
// not knot-count. Justify-KEEP at their absolute production values under both builds.
constexpr float    MAX_EXTRAP_DT_S      = 0.05f;   // MAX_EXTRAP_DT_S
constexpr float    EXTRAP_DECAY_DT_S    = 0.06f;   // EXTRAP_DECAY_DT_S
constexpr float    JERK_EMA_ALPHA       = 0.3f;    // JERK_EMA_ALPHA
// MAX_LEAD_REV lead-clamp: cap the commanded position at encoder ± this.
// Lowered 0.15 → 0.10 (2026-07-10 MAX_DEVIATION-runaway forensics). The ODrive
// legs run POSITION/PASSTHROUGH with LEG_POS_GAINS = 40 and the LEG_VEL_LIMIT_RPS
// pushed from hardware_config leg_vel_limit_rps (4.0 through 2026-07-15; 6.0 then
// 12.0 on 2026-07-16 — see logbook 2026-07-16-max-deviation-guard-tracking-lag.md).
// While the clamp is engaged the position error is pinned at MAX_LEAD_REV, so the
// position loop's velocity command is pos_gain × lead. At the old 0.15 that was
// 40 × 0.15 = 6.0 rev/s — ABOVE the THEN-4.0 vel_limit — so the loop saturated the
// limit, sprinted, overshot, braked (measured negative iq), and re-lagged: a ~6 Hz
// bang-bang limit cycle whose accumulated lead crossed MAX_DEVIATION_REV and latched
// the guard.
// At 0.10, 40 × 0.10 = 4.0 bounds the position-loop P-term (pos_gain × lead) to
// 4.0 rev/s — but that is the P-TERM ALONE. The ODrive's velocity setpoint is
// vel_ff + pos_gain × lead, and vel_ff is now PASSED THROUGH when the clamp engages
// (bounded to LEAD_CLAMP_VELFF_LIMIT_RPS = 3.5, no longer zeroed — see below).
// Against the OLD 4.0 vel_limit the P-term (4.0) EXACTLY equalled the limit, so when
// the clamp engaged the setpoint was already pinned at 4.0 by the P-term and vel_ff
// had ZERO room to add — every rev/s of feedforward was clipped, and the steady
// clamped sprint stayed vel_limit-bound. With the 2026-07-16 raises (4.0 → 6.0 → 12.0)
// the constraint becomes an INEQUALITY: 40 × 0.10 = 4.0 ≤ vel_limit, so the P-term sits
// well BELOW the limit and vel_ff regains genuine catch-up authority — the full 3.5
// VELFF cap now passes unclipped, making the clamp-engaged velocity setpoint top out at
// 4.0 + 3.5 = 7.5 rev/s (the VELFF cap, not vel_limit, is the binding bound). What the 0.10 +
// vel_ff-pass-through pair bought at 2026-07-10 was BOUNDARY CONTINUITY (no vel_ff
// discontinuity at clamp engage, a P-term no longer above the limit, killing the ~6 Hz
// limit cycle); the 6.0 raise converts the remaining P-term-EQUALS-limit saturation
// into usable catch-up headroom. MAX_LEAD stays 0.10 — do NOT change it; Kp·MAX_LEAD ≤
// vel_limit (the v2-bug constraint) is exactly what un-saturates the velocity setpoint.
constexpr float    MAX_LEAD_REV         = 0.10f;
// Feed-forward velocity cap applied to the interpolated vel_ff before it reaches the
// leg ODrive (2026-07-10 forensics). The lead clamp used to ZERO vel_ff whenever it
// engaged; that discontinuity (and the pos-loop sprint above) seeded the stutter.
// leg_interp now sends the TRUE interpolated vel_ff instead of zeroing it, bounded to
// this magnitude — kept below the drive vel_limit so a runaway command can never inject
// an over-vel_limit feedforward. NOTE (2026-07-16, vel_limit now 12): this 3.5 cap is now
// the BINDING clamp-engaged catch-up bound (Kp·MAX_LEAD + 3.5 = 7.5 rev/s ≈ 529 mm/s of
// leg speed); raising it toward ~10 is a flash item for the accel-FF chapter — keep it
// strictly below the drive vel_limit when raised.
constexpr float    LEAD_CLAMP_VELFF_LIMIT_RPS = 3.5f;
// MAX_DEVIATION_REV (E-stop): raw streamed u0 vs encoder guard, checked at FAULT_TASK_HZ
// (fault_machine.cpp). Raised 0.5 → 1.0 (2026-07-16 max-deviation-guard-tracking-lag
// forensics). Three guard E-STOPs latched during the S4 limit-ramp bench session at
// leg-vel 200 mm/s (trip deviations 0.52 / 0.55 / 0.56 rev, always leg 1 first, always
// at move onset). Those deviations were LEGITIMATE velocity-loop lag under coordinated-
// move load — the reflected platform inertia (~5-20× the bare rotor J_eff) makes the
// velocity loop lag the commanded ramp, and the deficit integrates into position
// deviation superlinearly with velocity (0.17-0.22 rev at 100 mm/s, ~0.41 at 160,
// 0.52-0.56 at ~190) — NOT runaway (peak iq_setpoint 8.7 A of 10 A, peak vel 2.2 of
// 4.0 rev/s: never current- nor velocity-railed). The guard watches the RAW streamed
// u0 against the encoder PRE lead-clamp, so it counts exactly the command-space lag the
// lead clamp deliberately tolerates. Raising it adds ZERO physical excursion for
// command-side faults: the executed command is bounded independently by the lead clamp
// (enc ± MAX_LEAD 0.10) + the stroke clamp + the ODrive position clip. The accepted cost
// is the encoder-side-runaway detect distance: 0.5→1.0 rev ⇒ 35→70 mm before a true
// encoder-side runaway latches (still caught; the firmware MAX_MOTOR_VEL_RPS = 16.5
// overspeed guard is unchanged). NOTE: the legacy Jetson-side MPC-path guard
// (motor_guard.py MAX_DEVIATION_REV) deliberately stays 0.5 — it has seen none of this
// data and its path is dormant on the mvp branch; re-decide when the MPC path revives.
// See logbook 2026-07-16-max-deviation-guard-tracking-lag.md.
constexpr float    MAX_DEVIATION_REV    = 1.0f;
// MAX_MOTOR_VEL_RPS = ODRIVE_TRAP_VEL_LIMIT_RPS * 1.1 (10% noise margin).
constexpr float    MAX_MOTOR_VEL_RPS    = ODriveDefaults::TRAP_VEL_LIMIT_RPS * 1.1f;  // 16.5

// ── Staleness / liveness thresholds (PORTED 1:1) ─────────────────────────────
//  motor_guard.py: MPC_CMD_STALENESS_S=0.25, MOTOR_FB_STALENESS_S=0.15.
//  can_node.py:   _HEARTBEAT_TIMEOUT_S=2.0 (CAN heartbeat watchdog).
constexpr uint32_t MPC_CMD_STALENESS_US = 250000u;   // 0.25 s → E-STOP (link-fault trigger)
constexpr uint32_t MOTOR_FB_STALENESS_US = 150000u;  // 0.15 s → suppress commands
constexpr uint32_t CAN_HEARTBEAT_TIMEOUT_US = 2000000u;  // 2.0 s → fatal_can_error
// Ball Butler heartbeat (CAN1) is tighter: BB broadcasts at 10 Hz and the
// production timeout (can_node._bb_heartbeat_timeout_s = proto.BB_HEARTBEAT_TIMEOUT_MS
// / 1000) is 0.5 s — only 5 missed heartbeats vs the ODrive's 20. Reused 1:1 here.
constexpr uint32_t BB_HEARTBEAT_TIMEOUT_US = BallButler::HEARTBEAT_TIMEOUT_MS * 1000u;
// Bus-partner presence window (generalised 2026-07-05 from CONE_PRESENT_STALENESS_US).
// EVERY can_*_send() gates its TX on "some partner frame arrived on this bus within
// this window" — never transmit into a bus with no recently-seen partner. Un-ACKed TX
// into a dead/unpowered bus retransmits forever, pins TEC at the error-passive
// threshold (128), and during supply transitions can push through to bus-off — the
// sticky BUSOFF/tec=254 pollution diagnosed in the 2026-07-05 marginal-CAN3
// investigation. Originally shipped for CAN2 only (cone-absent tolerance); the failure class is identical on
// all three buses (CAN1 pinned tec=128/passive from a Ball-Butler-absent window, CAN3
// reached BUSOFF across robot-supply cycles). A partner counts as "present" if any
// frame arrived within this window; generous so a brief silence (e.g. the ~3.5 s
// worst-case ODrive reboot gap) doesn't blip the gate — and on CAN3 the Platform
// Teensy's 2 Hz TRAFFIC_REPORT keeps the bus "present" through ODrive-only outages.
constexpr uint32_t BUS_PARTNER_STALENESS_US = 5000000u;  // 5.0 s
// How long live error-PASSIVE confinement must PERSIST before the CAN3 command
// gate refuses commands (classify_command_gate in can_buses.h; the observability
// classifier classify_bus_health is deliberately NOT debounced). BUS_OFF is exempt
// — it refuses instantly.
//
// SIZED BY THE DECAY-PUMP ARITHMETIC, NOT by the observed flap distribution —
// picking a threshold that makes a symptom disappear is tuning-to-green, and the
// 2026-07-29 wire errors are still unexplained (layer 2 of that investigation is
// open pending the bench A/B). The physics: TEC climbs +8 per errored TX and
// decays −1 per successful one, so recovery time is set by the SUCCESSFUL TX rate.
// CAN3's floor is the 100 Hz 0x7DD time-sync broadcast (the only TX that survives
// an idle robot — leg_interp is silent when the output gate is closed), so
// climbing back from TEC ≈ 160 to the 127 error-active boundary takes ~330 ms.
// 1.0 s therefore sits comfortably above every excursion a self-healing bus can
// produce at the minimum TX rate, while staying WELL under the 2.0 s
// CAN_HEARTBEAT_TIMEOUT_US staleness term — so the gate is never slower to react
// than the path sitting beside it.
constexpr uint32_t CAN_PASSIVE_SUSTAIN_US = 1000000u;    // 1.0 s
// Jetson UDP link: declare lost after LINK_LOST_MISSES missed heartbeats.
constexpr uint32_t JETSON_LINK_TIMEOUT_US =
    (1000000u / HEARTBEAT_RATE_HZ) * JbUdp::LINK_LOST_MISSES;   // 500 ms

// ── Fault state machine constants (PORTED 1:1 from can_node.py / motor_state.py) ─
//  motor_state.py: max_soft_reset_attempts = 1 (the cap exists because of a real
//  soft-reset bounce-loop incident — DO NOT raise it without that context).
constexpr uint8_t  MAX_SOFT_RESET_ATTEMPTS = 1;
//  odrive.py ERR_DC_BUS_UNDER_VOLTAGE = 512.
constexpr uint32_t ERR_DC_BUS_UNDER_VOLTAGE = 512u;

// Reboot-in-progress watchdog-suppression window (see logbook 2026-06-30-canbridge-phase6-reboot-latch). A REBOOT_ODRIVES RPC
// arms fault_notify_reboot_started(), which suppresses the CAN3 CAN-loss detector
// for this bounded window so the deliberate ODrive-reboot silence is NOT read as a
// real CAN loss (which would falsely arm the 2026-05-19 deferred stow). Sized to the
// MEASURED reboot→first-leg-heartbeat latency + margin: 2.3–3.5 s across 3 bench runs
// (motor power off, 2026-06-30, tools/probes/canbridge_reboot_latch_probe.py) + ~2.5 s
// headroom for reboot variance / ODrive cold-boot — NOT the untuned ~10 s can_node
// assumed. With the fresh-heartbeats-after-stale release, a normal reboot releases the
// latch the instant the legs return (well before this deadline), so this deadline only
// bounds the FAILURE case (legs never return); during it the ODrives autonomously hold
// their last setpoint (safe — the deferred-stow inversion), so the blind-spot cost is
// a bounded delay of the eventual safe-stow, never a collapse.
constexpr uint64_t REBOOT_WATCHDOG_SUPPRESS_US = 6000000ULL;  // 6.0 s

// Deferred-stow profiled descent (mirrors can_node _gently_move_to_setpoint):
// velocity-limited move to the off pose at the same limit on_shutdown uses.
constexpr float GENTLE_MOVE_VEL_LIMIT_RPS = JBOp::GENTLE_MOVE_VEL_LIMIT_RPS;  // 2.5
constexpr float STOW_OFF_POSE_REV = 0.0f;    // off pose = fully retracted (can_node stows to 0.0)
constexpr float STOW_DONE_EPS_REV = 0.01f;   // legs within this of the off pose ⇒ stow complete
// Accel limit for the stow descent so the commanded velocity ramps (no startup
// step). Reaches the gentle-move velocity limit in ~0.25 s.
constexpr float STOW_ACCEL_RPS2 = 10.0f;

// ── Re-enable recovery slew (2026-07-11 clear-errors jolt forensics) ──────────
// On the s_output_enabled false→true edge (every /clear_errors, /recover, arm, and
// fb-stale/stow resumption) the ISR re-baselines the transmitted command to the LIVE
// encoder and slews toward the streamed (lead-clamped) command at this bounded
// velocity + accel. Root cause it defends: while output is suppressed the ODrive
// holds its frozen setpoint (enc_freeze + lead) while the leg physically drifts onto
// the live encoder, so at the re-enable instant the streamed command can sit up to a
// full lead clamp (0.10 rev) BELOW the encoder. Transmitting it directly injects
// pos_gain × lead = 40 × 0.10 = a 4 rev/s velocity step straight to the -10 A current
// rail on every leg (measured on both clear events, 2026-07-11 forensics RESULT 3).
// The bounded slew replaces that step with a visibly-gentle ramp. VEL is kept well
// under the gentle-move limit (2.5) so the recovery reads as deliberate, not a lurch;
// ACCEL smooths the onset so the very first re-enabled frame is at the encoder with
// zero vel_ff (no velocity step either). At 5 rev/s² a worst-case full-clamp 0.10 rev
// slew peaks at exactly √(2·5·0.10) = 1.0 rev/s and completes in ~0.2 s.
constexpr float RECOVER_SLEW_VEL_RPS       = 1.0f;    // bounded recovery slew velocity (rev/s)
constexpr float RECOVER_SLEW_ACCEL_RPS2    = 5.0f;    // onset accel so there is no vel step at the edge
constexpr float RECOVER_SLEW_DONE_EPS_REV  = 0.002f;  // per-leg converged band → hand back to normal streaming

// ── Per-leg stroke clamp bounds (rev) ────────────────────────────────────────
//  Backstop matching motor_guard's _stroke_min_rev / _stroke_max_rev, which are
//  WorkspaceLimits.from_geometry(leg_hard_{min,max}_mm) * GEOM_MM_TO_REV (per-leg).
//  Captured 2026-06-01 from the live MotorGuard; the xref harness asserts these
//  equal the running guard's values so drift is caught.
//  TODO: hoist into codegen (derive from workspace limits in YAML).
constexpr float STROKE_MIN_REV[NUM_LEGS] =
    { 0.070917f, 0.070954f, 0.070448f, 0.070934f, 0.071340f, 0.071248f };
constexpr float STROKE_MAX_REV[NUM_LEGS] =
    { 3.900413f, 3.902459f, 3.874629f, 3.901381f, 3.923703f, 3.918615f };

// ── ODrive position clip bounds (odrive.clip_position) ───────────────────────
constexpr float LEG_MOTOR_MAX_POSITION  = Geometry::LEG_MOTOR_MAX_POSITION_REVS;   // 4.2
constexpr float HAND_MOTOR_MAX_POSITION = Geometry::HAND_MOTOR_HARD_STOP_REVS;  // 10.8 — the measured hard stop; see
                                       // hardware_config.yaml hand_motor_hard_stop_revs

// ── ODrive int16 input scaling (matches protocol_config InputScale) ──────────
constexpr float LEG_VEL_SCALE  = InputScale::leg_vel;   // 1000.0
constexpr float LEG_TOR_SCALE  = InputScale::leg_tor;   // 10000.0
constexpr float HAND_VEL_SCALE = InputScale::hand_vel;  // 100.0
constexpr float HAND_TOR_SCALE = InputScale::hand_tor;  // 100.0

}  // namespace CanBridge
