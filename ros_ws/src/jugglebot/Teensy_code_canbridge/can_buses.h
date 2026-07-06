#pragma once
// =============================================================================
//  can_buses.h — three subsystem-isolated FlexCAN_T4 buses (ADR-0013)
// =============================================================================
//  One FlexCAN_T4 peripheral per robot subsystem (supersedes the old two-bus
//  shared-aux / private-leg split):
//    bb        = CAN1 (Ball Butler Teensy + BB pitch/hand ODrives). BB heartbeat/
//                CMD_RESULT/ODrive telemetry decode into bb_state/bb_axes; we TX
//                the 100 Hz 0x7DD time-sync broadcast + relayed BB commands.
//    cone      = CAN2 (catching cone Teensy, often physically disconnected). RX
//                is counted AND copied into a small SPSC ring for the cone
//                uplink: task_telem drains the ring into CONE_FRAME
//                UDP messages so the Jetson sees every cone frame verbatim
//                (CATCH_EVENT 0x7E0 / CONE_HEARTBEAT 0x7E1). TX is the 0x7DD
//                broadcast.
//    jugglebot = CAN3 (Jugglebot core: 6 leg ODrives + Hand ODrive + platform
//                Teensy 4.0 + can-bridge). ALL ODrive telemetry/heartbeat/error
//                frames decode into the per-axis cache from here; leg setpoints,
//                RPC-driven ODrive commands, and fault-machine commands TX here.
//
//  TX presence gate (2026-07-05, generalised from the cone-absent tolerance
//  ): EVERY can_*_send() is gated on bus_partner_present() — a frame
//  seen on that bus within BUS_PARTNER_STALENESS_US — so the bridge never
//  transmits into a partner-less (unpowered/disconnected) bus. See the predicate
//  below and the 2026-07-05 marginal-CAN3 logbook entry for the failure class.
//
//  Peripheral identity (CAN1/CAN2/CAN3) appears ONLY as the FlexCAN_T4 template
//  parameter in can_buses.cpp; everywhere else the bus is named by subsystem so
//  a future re-pin only touches the wiring file (ADR-0013).
//
//  RX is dispatched from canX.events() (pumped by the CAN RX task) via onReceive
//  callbacks — the proven platform-Teensy idiom (Teensy_code.ino canSniff) — and
//  decoded straight into the per-axis cache (fast, bounded; the decode is
//  microseconds and the cache writes are seqlock-guarded).
//
//  NB on the data path: there IS a library-internal queue hop. The FlexCAN RX
//  interrupt pushes each frame into the peripheral's RX_SIZE_256 rxBuffer; events()
//  pops from it. (So the shorthand "straight into the cache, not by enqueuing" is
//  imprecise: events()-driven onReceive dispatch routes through that rxBuffer first.
//  There is no *application-level* queue between the callback and the cache -- that
//  is the distinction that matters.) Because events() pops only ONE frame per call, can_buses_service()
//  drains each bus in a bounded loop so the ~2,240 fps of CAN3 telemetry the can-bridge
//  receives cannot overflow the rxBuffer — see CAN_RX_DRAIN_BUDGET in can_buses.cpp.
// =============================================================================

#include <cstdint>
#include "odrive_protocol.h"
#include "udp_protocol.h"   // JbUdp::BusHealth
#include "protocol_config.h"  // PlatformCanId (relay reply ids)
#include "canbridge_config.h" // BUS_PARTNER_STALENESS_US (TX presence gate)

namespace CanBridge {

void can_buses_init();      // begin all three buses, register RX callbacks
void can_buses_service();   // pump bb/cone/jugglebot events(); call from CAN RX task

// All three sends are presence-gated (bus_partner_present(); false = refused, nothing
// queued) — the bridge never transmits into a partner-less bus (2026-07-05).
bool can_bb_send(const ODrive::CanFrame& f);         // CAN1 Ball Butler TX (time-sync + relayed BB cmds)
bool can_cone_send(const ODrive::CanFrame& f);       // CAN2 cone TX (time-sync)
bool can_jugglebot_send(const ODrive::CanFrame& f);  // CAN3 Jugglebot core TX (setpoints, RPC, fault cmds)

// "Never command a confirmed-dead bus." True iff CAN3 (the Jugglebot core bus
// carrying the leg + hand ODrives AND the Platform-Teensy relay partner) is NOT
// in a WARN/BUS_OFF state. Since 2026-07-05 those states come from
// classify_bus_health(): WARN = RX stale > 2 s OR live error-passive; BUS_OFF =
// live controller bus-off (previously staleness-only — BUS_OFF was unreachable).
// The shared gate for every CAN3-bound RPC (rpc.cpp leg frames + platform_relay
// reads/writes). OK/UNKNOWN both allow commands so the initial bring-up sequence
// works before telemetry warms. (Native harness fakes this via
// fake_set_commands_allowed; the firmware reads can_buses_stats().)
bool jugglebot_commands_allowed();

// "Is CAN3 electrically transmittable RIGHT NOW?" — the LIVE ESR1.SYNCH bus-lock bit
// (s_jugglebot_rxh.synced), NOT heartbeat-staleness. Gates ONLY the operator recovery
// one-shots CLEAR_ERRORS / REBOOT_ODRIVES: a just-repowered or motor-bus-
// cycled bus is SYNCH=1 the instant it is electrically alive — BEFORE any heartbeat is
// decoded — so the recovery clear is allowed exactly when jugglebot_commands_allowed()
// would still read WARN from the stale RX timestamp and refuse it (the 2026-06-27
// just-repowered-bus deadlock; audit rec "carve clear/reboot out of the bus-health
// gate"). SYNCH=0 (bus-off / not synced) still refuses, guarding against blind-clearing
// a truly dead bus — strictly safer than a blanket carve-out. NON-sticky (unlike
// fault_conf), so it reflects the current state and recovers when the bus returns.
// (Native harness fakes this via fake_set_bus_transmittable if a TU ever needs it.)
bool jugglebot_bus_transmittable();

struct CanStats {
  uint32_t bb_rx, bb_tx, cone_rx, cone_tx, jugglebot_rx, jugglebot_tx;
  uint8_t  bb_health;          // JbUdp::BusHealth
  uint8_t  cone_health;
  uint8_t  jugglebot_health;
};
CanStats can_buses_stats();

// ── Cone uplink ring (CAN2 → CONE_FRAME UDP relay) ─────────────────
//  Every frame received on the cone bus is copied into a small SPSC ring by the
//  RX callback (producer: task_can_rx — briefly the FlexCAN ISR during the boot
//  window, so the push is PRIMASK-guarded) and popped by the telemetry task
//  (consumer: cone_uplink_step in telemetry.cpp), which forwards each record as
//  a JbUdp CONE_FRAME. The cone's impact timestamp travels inside buf (latched
//  in the cone's piezo ISR); t_bridge_us only stamps bridge-side CAN RX.
struct ConeFrameRec {
  uint64_t t_bridge_us;   // bridge wall-clock at CAN2 RX (us)
  uint32_t can_id;        // CAN arbitration id (0x7E0 / 0x7E1)
  uint8_t  dlc;           // CAN payload length (0..8)
  uint8_t  buf[8];        // raw CAN payload (zero-padded past dlc)
};
bool can_cone_pop(ConeFrameRec& out);   // consumer side; false when ring empty
uint32_t can_cone_fwd_drops();          // frames dropped on ring overflow (cumulative)

// ── BB command-result uplink ring (CAN1 CMD_RESULT → UDP relay) ──────
//  BB firmware publishes one CMD_RESULT (0x7D5) per operator command at its
//  terminal point (throw today; reload/calibrate/home later). on_bb_rx copies the
//  frame verbatim into this SPSC ring; cmd_result_uplink_step (telemetry.cpp)
//  forwards each record as a JbUdp CMD_RESULT so the host learns the firmware
//  outcome instead of only the bridge-side RPC ack. Layout mirrors ConeFrameRec.
struct CmdResultFrameRec {
  uint64_t t_bridge_us;   // bridge wall-clock at CAN1 RX (us)
  uint32_t can_id;        // CAN arbitration id (0x7D5 CMD_RESULT)
  uint8_t  dlc;           // CAN payload length (0..8)
  uint8_t  buf[8];        // raw CAN payload (zero-padded past dlc)
};
bool can_cmd_result_pop(CmdResultFrameRec& out);  // consumer side; false when ring empty
uint32_t can_cmd_result_fwd_drops();              // frames dropped on ring overflow (cumulative)

// ── Platform-Teensy relay-reply uplink ring (CAN3 0x6E0/0x7DE → UDP) ─
//  The can-bridge relays the Platform Teensy's RobotState (0x6E0 STATE_UPDATE)
//  and inclinometer tilt (0x7DE TILT_READING) replies VERBATIM to the host, so
//  the host owns the byte-layout decode (decouples the bridge from Teensy_code's
//  RobotState/tilt packing). on_jugglebot_rx copies every CAN3 frame whose id is
//  a Platform reply id into this SPSC ring; platform_uplink_step (telemetry.cpp)
//  forwards each record as a JbUdp PLATFORM_FRAME. Layout mirrors ConeFrameRec.
//  NOTE(bench): host (id, dlc) reply-correlation is only sound with CAN3 SRX_DIS
//  set so the bridge's own 0x6E0 STATE_WRITE is not looped back here as a reply.
struct PlatformFrameRec {
  uint64_t t_bridge_us;   // bridge wall-clock at CAN3 RX (us)
  uint32_t can_id;        // CAN arbitration id (0x6E0 STATE_UPDATE / 0x7DE TILT_READING)
  uint8_t  dlc;           // CAN payload length (0..8)
  uint8_t  buf[8];        // raw CAN payload (zero-padded past dlc)
};
bool can_platform_pop(PlatformFrameRec& out);  // consumer side; false when ring empty
uint32_t can_platform_fwd_drops();             // frames dropped on ring overflow (cumulative)

// ── Hand command-echo ───────────────────────────────────────────────
// The can-bridge sniffs the Platform Teensy's Set_Input_Pos to the HAND ODrive
// (axis 6) on CAN3 and forwards the raw 8-byte payload so the host echoes the
// hand's commanded pos/vel_ff/tor_ff (can_node._handle_hand_input_pos parity — the
// hand_telemetry pos_cmd/vel_ff_cmd/tor_ff_cmd fields). SINGLE-slot latest-value
// (coalesced to the newest command, consumed at the telemetry-task rate): the hand
// setpoint is a stream, and a diagnostic echo only needs the freshest sample, so a
// ring is not warranted. SPSC: producer decode_into_cache (task_can_rx), consumer
// hand_cmd_echo_uplink_step (telemetry.cpp, task_telem).
struct HandCmdEchoRec {
  uint64_t t_bridge_us = 0;   // bridge wall-clock at CAN3 RX of the hand Set_Input_Pos (us)
  uint8_t  buf[8] = {0};      // raw ODrive Set_Input_Pos payload: <f h h> pos_rev, vel_ff, tor_ff
};
bool can_hand_cmd_echo_pop(HandCmdEchoRec& out);  // true + clears the dirty flag when a fresh cmd is pending

// True iff `id` is a Platform-Teensy relay-reply arbitration id (0x6E0 / 0x7DE).
// Single classifier shared by the CAN3 RX ring filter (can_buses.cpp) and the
// native harness. Inline (header-only) so the relay test can reach it without
// compiling can_buses.cpp (which pulls FlexCAN_T4) on the host.
inline bool is_platform_reply_id(uint32_t id) {
  return id == PlatformCanId::STATE_UPDATE || id == PlatformCanId::TILT_READING;
}

// ── Bus-partner presence predicate (the TX-gate contract, 2026-07-05) ─────────
// True iff some partner frame has been seen on the bus within the presence window.
// EVERY can_*_send() refuses to transmit when this is false: an un-ACKed frame on a
// partner-less bus retransmits forever, pinning TEC at the error-passive threshold
// (128) and polluting the sticky bus diagnostics (the 2026-07-05 marginal-CAN3
// investigation's root finding); during supply ramps it can escalate to bus-off.
// last_rx_us == 0 means "no frame ever" (boot) — absent by definition. Pure and
// header-only so the native harness can pin the window semantics without compiling
// can_buses.cpp (same pattern as is_platform_reply_id above).
inline bool bus_partner_present(uint64_t last_rx_us, uint64_t now_us) {
  return last_rx_us != 0 && (now_us - last_rx_us) <= BUS_PARTNER_STALENESS_US;
}

// ── Bus-health classification (the health_of() bus-off wiring, 2026-07-05) ────
// Per-bus classifier behind health_of(): RX staleness PLUS the live fault-
// confinement state (flt_live in BusRxHealth below — NOT the sticky fault_conf
// high-water, which would latch WARN/BUS_OFF forever after any recovered
// transient).
//   never seen a frame           → UNKNOWN  (bring-up: commands allowed; the TX
//                                            presence gate independently refuses sends)
//   FLTCONF bus-off              → BUS_OFF  (controller off the bus; TX impossible;
//                                            outranks staleness — it is the cause)
//   FLTCONF passive OR RX stale  → WARN
//   else                         → OK
// WARN keys on error-PASSIVE (TEC/REC ≥ 128), deliberately NOT the CAN warning
// level (96): the measured 12 V supply-ramp RX burst peaks at REC ≈ 121, so a
// 96-threshold would flag WARN on every normal CAN power-on (2026-07-05 power-
// cycle capture). Two liveness facts make the register terms safe to gate on:
// a frozen TEC=128 (presence-gate closing-window pin with no TX left to decay it)
// self-clears within one 0x7DD period once time-synced + partner-present — the
// 100 Hz broadcast IS the decay pump (TEC −1 per clean TX) and shares its origin
// (ROS2) with any command that could race it; and bus-off cannot latch — BOFFREC
// stays 0 (FlexCAN_T4 never sets it), so the controller auto-recovers after
// 128×11 recessive bits. Each bus is classified INDEPENDENTLY from its own
// registers/timestamps: a CAN1 fault must never gate CAN3 commands, and vice
// versa. Pure and header-only so the native harness pins the truth table without
// compiling can_buses.cpp (same pattern as bus_partner_present above).
inline uint8_t classify_bus_health(uint64_t last_rx_us, uint64_t now_us,
                                   uint8_t flt_live) {
  if (last_rx_us == 0) return JbUdp::BusHealth::UNKNOWN;
  if (flt_live >= 2) return JbUdp::BusHealth::BUS_OFF;
  if (flt_live == 1) return JbUdp::BusHealth::WARN;
  if (now_us - last_rx_us > CAN_HEARTBEAT_TIMEOUT_US) return JbUdp::BusHealth::WARN;
  return JbUdp::BusHealth::OK;
}

// ── RX-health observability (bench/debug telemetry) ──────────────────────────
//  Diagnostic counters that let a future bug be told apart by class. Surfaced over
//  the USB Serial console by task_diag (NOT on the UDP uplink — the wire format is
//  owned elsewhere; promoting these to a telemetry frame is a follow-up). Every
//  field is CUMULATIVE or STICKY-since-boot, so a single rare transient (a one-off
//  starvation, a momentary bus-off) is never lost between 1 Hz samples. Collected
//  in can_buses_service() (1 kHz); see can_buses.cpp.
//
//  What each field discriminates:
//    depth_hwm   — peak rxBuffer occupancy seen at a service tick. The direct
//                  witness that the bounded drain keeps up: stays ~single digits
//                  in health; climbing toward 256 means task_can_rx is starved.
//    cap_hits    — # ticks the per-tick CAN_RX_DRAIN_BUDGET bound with frames still
//                  queued. The overflow PRECURSOR alarm; must stay 0.
//    err_events  — # FlexCAN ESR1-change snapshots (bus saw an error/state change).
//    err_flags   — OR of the BusErrFlag bits below: WHICH wire error (ACK/CRC/form/
//                  stuff/bit) — distinguishes wiring/termination/noise from protocol.
//    rec_max     — peak RX error counter (REC): severity of RECEIVE-side errors.
//    tec_max     — peak TX error counter (TEC): the bus-off precursor when a bus partner
//                  is DISCONNECTED. Our un-ACKed TX (0x7DD broadcast on every bus, plus
//                  CAN3 setpoints) climbs TEC while REC stays 0, so this — not rec_max —
//                  is what rises ahead of a "Ball Butler / cone / Jugglebot unplugged"
//                  bus-off.
//    fault_conf  — worst fault-confinement reached (0 active / 1 passive / 2 bus-off):
//                  bus-off means that bus's nodes are momentarily uncommandable.
//    synced      — LIVE ESR1.SYNCH (NOT sticky): 1 = the controller is synchronised to the
//                  bus right now. The cleanest "is this bus electrically alive" indicator —
//                  flips 0→1 the instant a powered bus appears, before any frame decodes.
//  Plus decode_short / decode_bad_axis: CAN3 frames that arrived but could not be
//  cached (truncated DLC, or a node id outside 0..NUM_AXES) — tells "wrong/garbled
//  data arriving" apart from "no data arriving".
namespace BusErrFlag {
  constexpr uint8_t ACK   = 1u << 0;   // missing ACK (no other node / bus fault)
  constexpr uint8_t CRC   = 1u << 1;   // CRC error (noise / signal integrity)
  constexpr uint8_t FORM  = 1u << 2;   // form error (framing)
  constexpr uint8_t STUFF = 1u << 3;   // bit-stuffing error (noise / clocking)
  constexpr uint8_t BITERR0 = 1u << 4; // dominant bit not echoed back (named to dodge
  constexpr uint8_t BITERR1 = 1u << 5; // the BIT0/BIT1 macros in imxrt_flexcan.h)
}

struct BusRxHealth {
  uint16_t depth_hwm;     // peak rxBuffer occupancy at a service tick
  uint32_t cap_hits;      // ticks the drain budget bound with frames still queued
  uint32_t err_events;    // ESR1-change error snapshots captured
  uint8_t  err_flags;     // OR of BusErrFlag::*
  uint8_t  rec_max;       // peak RX error counter (REC)
  uint8_t  tec_max;       // peak TX error counter (TEC) — bus-off precursor when a bus
                          // partner is disconnected (un-ACKed TX climbs TEC, not REC)
  uint8_t  fault_conf;    // worst fault-confinement: 0 active / 1 passive / 2 bus-off
  uint8_t  synced;        // LIVE (not sticky) ESR1.SYNCH: 1 = controller locked onto the bus

  // ── marginal-CAN3 diagnostic instrumentation (2026-07-05 investigation) ────
  // Snapshots WITH at least one error-type bit set — the TRUE wire-error event
  // count, printed as [canhealth] err= (the raw err_events change counter above
  // includes benign IDLE/RX/TX phase flips and is printed as chg=). Lower-bound
  // during a sustained identical-error storm (the library's change-detect captures
  // a repeated identical ESR1 once); tec_inc_sum/rec_inc_sum below stay
  // rate-accurate in that case.
  uint32_t wire_errs;
  // Per-type ESR1-snapshot counters: WHICH error class fires and at what rate
  // (err_flags only says "seen since boot"). One snapshot can set several types.
  uint32_t ack_cnt;       // ACKERR snapshots  (TX un-ACKed)
  uint32_t crc_cnt;       // CRCERR snapshots  (RX: CRC mismatch — noise/SI)
  uint32_t form_cnt;      // FRMERR snapshots  (RX: fixed-form field violated)
  uint32_t stuff_cnt;     // STFERR snapshots  (RX: >5 equal bits — noise/clocking)
  uint32_t bit0_cnt;      // BIT0ERR snapshots (TX: sent dominant, read recessive)
  uint32_t bit1_cnt;      // BIT1ERR snapshots (TX: sent recessive, read dominant)
  // TX-vs-RX attribution: ESR1.TX (bit 6) / ESR1.RX (bit 3) captured with the
  // WIRE-ERROR snapshot — was the controller transmitting or receiving when the
  // error fired? (Benign phase-flip snapshots are excluded — they'd swamp these.)
  uint32_t err_tx_ctx;    // wire-error snapshots with ESR1.TX set
  uint32_t err_rx_ctx;    // wire-error snapshots with ESR1.RX set
  // Live ECR sample (base+0x1C): the CURRENT error counters each service tick,
  // not the event-captured high-water. TEC decays -1 per clean TX, REC likewise
  // for RX, so live values falling ⇒ recovering; pinned high ⇒ sustained fault.
  uint8_t  tec_live;      // ECR TXERRCNT at last service tick
  uint8_t  rec_live;      // ECR RXERRCNT at last service tick
  // Live fault confinement (ESR1.FLTCONF bits 5:4, clamped: 0 active / 1 passive /
  // 2 bus-off), sampled each service tick from the SAME ESR1 read as synced. The
  // NON-sticky counterpart of fault_conf above — recovers when the bus does. Feeds
  // classify_bus_health() (the health_of() bus-off wiring, 2026-07-05); printed as
  // fltNow= on [canhealth].
  uint8_t  flt_live;
  // Cumulative positive deltas of the live counters between ticks: which counter
  // is being DRIVEN. TEC +8/TX-error vs REC +1/RX-error (CAN fault confinement),
  // so compare rates, not magnitudes.
  uint32_t tec_inc_sum;   // Σ max(0, ΔTEC) across service ticks
  uint32_t rec_inc_sum;   // Σ max(0, ΔREC) across service ticks
  // TX attempts refused by the bus-partner presence gate (bus_partner_present()
  // false at send time). Non-zero while a bus is unpowered/disconnected and some
  // caller still tries to TX — the visible witness that the gate is doing its job
  // instead of letting un-ACKed TX pin TEC.
  uint32_t tx_gated;
};

struct CanRxHealth {
  BusRxHealth bb, cone, jugglebot;
  uint32_t decode_short;     // CAN3 frames dropped in decode: DLC < 8 (truncated)
  uint32_t decode_bad_axis;  // CAN3 frames dropped in decode: node id >= NUM_AXES
};
CanRxHealth can_buses_rx_health();

// Marginal-CAN3 diagnostic (2026-07-05): print one [cantiming] line per bus on the
// USB Serial console — the DECODED CTRL1 bit timing (prescaler/propseg/pseg1/pseg2/
// rjw) plus the computed bit rate and sample point, and the CCM CAN root clock.
// Ground truth for "what timing is this controller actually running", independent
// of what setBaudRate was asked for.
void can_buses_dump_timing();

// Marginal-CAN3 diagnostic (2026-07-05): print the raw ESR1 words of any error
// snapshots captured since the last call (up to the 8 most recent per bus), so
// flag-less "error" events can be attributed to the exact ESR1 bits that changed.
void can_buses_print_esr1();

}  // namespace CanBridge
