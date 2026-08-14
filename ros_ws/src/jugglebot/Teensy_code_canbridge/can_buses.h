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
//  callbacks — the proven platform-Teensy idiom (Teensy_code_platform.ino canSniff) — and
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
// in a WARN/BUS_OFF state. Since 2026-07-29 those states come from
// classify_command_gate() — NOT classify_bus_health(): WARN = RX stale > 2 s OR
// SUSTAINED live error-passive (≥ CAN_PASSIVE_SUSTAIN_US); BUS_OFF = live
// controller bus-off, refused instantly. The sustain requirement is the fix for
// the 2026-07-29 CAN3 flap — see classify_command_gate above for why an
// instantaneous error-passive reading is the wrong thing to gate on. The
// 2026-07-05 bus-off wiring (BUS_OFF previously unreachable, staleness-only) is
// preserved unchanged.
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
//  the host owns the byte-layout decode (decouples the bridge from Teensy_code_platform's
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

// ── Command gate: SUSTAINED confinement, not instantaneous ────────────────────
// NORMATIVE SPLIT (2026-07-29): bus health REPORTS instantaneous confinement;
// the command gate ACTS only on SUSTAINED confinement. classify_bus_health above
// is deliberately left untouched — it is the observability signal, and it must
// keep telling the truth on the uplink and in /link_status even while the gate
// declines to act on a transient.
//
// Root cause this split closes (2026-07-29 CAN3 flap,
// logbook/2026-07-29-can3-bus-health-flap-hand-sensor-poller.md). Error-passive
// is a TRANSIENT, self-healing controller state by construction: TEC decays −1
// per successful TX. Gating every CAN3 command on an INSTANTANEOUS reading of it
// produced two concrete failure modes, both measured:
//   1. POSITIVE FEEDBACK. Refusing TX removes the very traffic that decays TEC —
//      the 100 Hz 0x7DD broadcast IS the decay pump (see the classify_bus_health
//      note above). The gate therefore prolongs the condition it reacts to. The
//      hand ball-sensor poller (gpio_poll.cpp:247) closed exactly this loop:
//      poll → wire error → passive → WARN → poller gated → bus quiets → OK →
//      poll, cycling 213 times in 83.5 s.
//   2. AMPLIFICATION. Every CAN3 consumer shares this one predicate, so a
//      low-rate wire-error source became a 42.4 %-duty outage across
//      platform_relay (0x6D0 hand traj + STATE_READ), hand_ops, rpc.cpp leg
//      frames, leg_homing and version_check at once — from an error source whose
//      actual rate was not even measurable at the time (the per-class counters
//      were serial-console-only; the CanErrors uplink now fixes that).
//
// BUS_OFF still refuses INSTANTLY and is exempt from the sustain requirement: it
// is not a counter excursion, it is "the controller is off the bus and TX is
// physically impossible", so waiting to act on it would buy nothing. RX staleness
// (> 2 s) likewise stays instantaneous — it is already a sustained-by-definition
// measurement.
//
// Accepted tradeoff, stated plainly: for up to CAN_PASSIVE_SUSTAIN_US we may emit
// a discrete command onto a briefly error-passive bus. Bounded — error-passive
// nodes still transmit normally (only BUS_OFF stops TX), FlexCAN retransmits, each
// caller's own ack/timeout path handles a genuine loss, and the bus-partner
// presence gate inside can_*_send() independently refuses a partner-less bus. The
// 500 Hz leg-setpoint path was ALREADY ungated on this bus (leg_interp.cpp), so
// this is strictly less exposure than the status quo already accepts.
//
// Pure and header-only for the same reason as classify_bus_health: the native
// harness pins the truth table without compiling can_buses.cpp.
inline uint8_t classify_command_gate(uint64_t last_rx_us, uint64_t now_us,
                                     uint8_t flt_live, uint8_t flt_sustained) {
  if (last_rx_us == 0) return JbUdp::BusHealth::UNKNOWN;
  if (flt_live >= 2) return JbUdp::BusHealth::BUS_OFF;          // instant, never debounced
  if (flt_live == 1 && flt_sustained) return JbUdp::BusHealth::WARN;
  if (now_us - last_rx_us > CAN_HEARTBEAT_TIMEOUT_US) return JbUdp::BusHealth::WARN;
  return JbUdp::BusHealth::OK;
}

// ── RX-health observability (bench/debug telemetry) ──────────────────────────
//  Diagnostic counters that let a future bug be told apart by class. Surfaced over
//  the USB Serial console by task_diag; since 2026-07-29 CAN3's subset (the
//  per-type wire-error counts, TX/RX attribution, live TEC/REC, the inc-sums,
//  tx_gated and the confinement bytes) ALSO rides the UDP uplink as the additive
//  CanErrors frame (MsgType 0x8C, telemetry.cpp can_errors_uplink_step) and lands
//  in /link_status as the can3_errors row. That promotion IS the follow-up this
//  comment used to defer: being serial-only is precisely why the 2026-07-29 CAN3
//  flap could be seen in a rosbag but not explained from one. CAN1/CAN2 remain
//  serial-only until something needs them. Every
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
//    tx_deferred — sends whose FlexCAN_T4::write() returned -1. That overload returns
//                  1 or -1 and NEVER 0: -1 means no TX mailbox was free, so the frame
//                  was pushed into the 64-slot software txBuffer and the TX-complete
//                  ISR sends it ~0.1-1 ms later. This counts DEFERRAL, i.e. TX-path
//                  pressure — NOT loss. The two paths that genuinely lose a frame are
//                  txBuffer OVERFLOW (a 65th pending entry silently overwrites the
//                  oldest — circular_buffer.h push_back, no counter, no return code)
//                  and the vendored events() TX drain (it writes ONE peeked frame into
//                  every free mailbox while popping one queue entry per mailbox, so
//                  the extra entries are discarded unsent). tx_q_hwm below is the
//                  observable for the first; the second is un-instrumentable without
//                  patching the vendored library.
//    tx_q_hwm    — peak software txBuffer occupancy (cap 64), sampled at SEND instants
//                  rather than on the 1 kHz service tick. Deliberate: the queue drains
//                  in ~115 us/frame, so post-drain 1 kHz sampling reads ~0 through
//                  exactly the us-scale bursts that make a send defer. At/near 64 ⇒
//                  overwrite-loss has occurred or is imminent.
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
  // ── TX-path pressure (2026-08-02 ERR_TIMEOUT attribution) ─────────────────
  // Both live in the SAME writer class as tx_gated — the interp ISR and several
  // FreeRTOS tasks — so both are read-modify-written inside the send's EXISTING
  // PRIMASK region (never the task_can_rx-only, unmasked class the fields above
  // belong to). Semantics and the two real loss paths: see the doc block above.
  // KNOWN COVERAGE GAP (accepted, 2026-08-02): can_buses.cpp compiles in no
  // native test binary, so these increments have no unit test — exactly as
  // tx_gated has had none since it landed. Their coverage is the xlang struct
  // pin, the ROS render test, and the on-hardware [canhealth] serial line.
  // Building a FlexCAN_T4 host shim to close it is real work, deliberately not
  // done here.
  uint32_t tx_deferred;   // sends whose write() returned -1 (queued, NOT dropped)
  uint16_t tx_q_hwm;      // peak txBuffer occupancy at a send instant (cap 64)
  // ── Sustained-confinement tracking (2026-07-29 CAN3 flap) ─────────────────
  // flt_live above is INSTANTANEOUS. These two derive "has confinement PERSISTED"
  // from it, sampled on the same 1 kHz service tick, and feed
  // classify_command_gate() — see the normative split above that classifier.
  uint64_t flt_passive_since_us;  // micros64() when flt_live last went 0→≥1; 0 ⇒ currently active
  uint8_t  flt_sustained;         // 1 ⇒ flt_live ≥ 1 held continuously ≥ CAN_PASSIVE_SUSTAIN_US
};

struct CanRxHealth {
  BusRxHealth bb, cone, jugglebot;
  uint32_t decode_short;     // CAN3 frames dropped in decode: DLC < 8 (truncated)
  uint32_t decode_bad_axis;  // CAN3 frames dropped in decode: node id >= NUM_AXES
  // Per-axis get_encoder_estimate frames DECODED AND CACHED, cumulative since boot
  // (index 0-5 legs, 6 hand). Added 2026-08-12 for the bridge-temporal arc: the S1
  // bag forensics found the per-axis cache VALUE stalling 30-500 ms (9-18 % of
  // refresh intervals > 30 ms on an aged bridge vs 4.3 % fresh) while every
  // AGGREGATE RX counter stayed flat — an aggregate cannot see one axis of seven
  // going quiet, because the other six keep the total moving. These are the split:
  // during a stall window the counter still ADVANCING means frames arrived and the
  // decode ran, so the freeze is at or above write_pos_vel (i.e. the ODrive sent a
  // stale value); the counter PAUSED means nothing arrived, and the fault is the
  // ODrive's broadcast or a per-axis loss on the bus.
  // Incremented AFTER write_pos_vel() so the invariant is one-directional and
  // useful: enc_frames[i] advanced ⇒ that axis's cache write COMPLETED. Same
  // single-writer discipline as decode_short/decode_bad_axis above — one word,
  // task_can_rx only, cumulative, read whole by the consumer, which differences
  // two samples (the BridgeTxDiag census idiom).
  uint32_t enc_frames[NUM_AXES];
};
CanRxHealth can_buses_rx_health();

// ── RX-ring TRUE-occupancy probe (2026-08-14, can-bridge FW 13) ─────────────
// The conviction instrument for the FlexCAN_T4 `_available` leak. THE DEFECT
// (assembly-verified; recorded in lib/FlexCAN_T4/PROVENANCE.md): events() pops
// the RX ring BEFORE its NVIC_DISABLE_IRQ guard, so the ISR's `_available++` and
// the task-side `_available--` race one-directionally and increments are
// swallowed. `_available` therefore UNDER-counts monotonically, the drain loop in
// service_bus() exits believing the ring is empty while a residue D > 0 is
// stranded, and from then on every delivered frame is D frames old.
//
// WHY THESE FIELDS CANNOT LIVE IN BusRxHealth ABOVE. Its depth_hwm and cap_hits
// are computed from getRXQueueCount(), which returns `_available` — they are
// built on the corrupted number and are blind to this failure BY CONSTRUCTION.
// Keeping the true-occupancy fields in a SEPARATE struct also avoids the other
// hazard: BusRxHealth is snapshotted by a hand-written field-by-field copy
// (snapshot_bus) that feeds the CanErrors uplink and the [canhealth] console
// line, so a field added there and forgotten in that copy reads as uninitialised
// stack. This struct is copied whole and cannot have that bug.
//
// Sampled in service_bus IMMEDIATELY AFTER the drain loop finishes: at that
// instant `_available` is 0 by definition (that is why the loop exited), so
// `depth - avail` is the leak itself, measured directly. Single-writer
// (task_can_rx), one word per field, cumulative or high-water — the same
// lock-free cross-task read discipline as BusRxHealth's unmasked class.
struct BusRingProbe {
  uint16_t depth;           // post-drain TRUE occupancy from head/tail (cap 256)
  uint16_t avail;           // `_available` at the same probe — the leak is depth - avail
  uint16_t leak_hwm;        // max(depth - avail) over every service tick since boot
  uint16_t depth_hwm;       // max TRUE post-drain depth since boot (cf. BusRxHealth::depth_hwm,
                            // which is the PRE-drain depth from the corrupted `_available`)
  uint32_t fifo_overflows;  // hardware FIFO overflow events (IFLAG1 bit 7), ISR-counted, exact
  uint32_t fifo_warns;      // hardware FIFO warning events (IFLAG1 bit 6), ISR-counted, exact
};
struct CanRingProbe {
  BusRingProbe bb, cone, jugglebot;
  uint32_t probe_ticks;     // service ticks probed since boot (the extrema's denominator)
};
// Whole-struct read; the FIFO counters are read from the library instances at
// call time (single aligned words, ISR-written). DIAGNOSTICS ONLY — nothing in
// the firmware reads any field of this.
CanRingProbe can_buses_ring_probe();

// ── Jugglebot delivery-lag accumulator (2026-08-14, can-bridge FW 13) ───────
// Formulation-2 delivery lag off the FlexCAN HARDWARE capture timestamp, which
// is stamped at frame reception and is therefore immune to whatever the ring
// does afterwards — the one clock in the system that can see through the leak.
// The stamp is 16-bit (1 us/tick at 1 Mbit, wrapping every 65.536 ms), so
// on_jugglebot_rx accumulates (uint16_t)(ts - ts_prev) into a 64-bit arrival
// clock; delivery is FIFO-ordered and the jugglebot bus runs ~2240 frames/s
// (~0.45 ms apart), so the unwrap has ~100x margin. A gap that erodes that
// margin forces a RESEED rather than a silent corruption — see lag_reseeds.
struct JbLagProbe {
  int32_t  lag_now_us;   // (micros64 since seed) - (arrival clock since seed) == lag_now - lag_at_seed
  int32_t  lag_hwm_us;   // max lag_now_us since the last seed (reset by a reseed)
  uint64_t arrival_us;   // arrival clock since seed — differenced by the caller for cap_span_us
  uint32_t frames;       // frames folded since seed (cumulative; differenced per window)
  uint32_t reseeds;      // cumulative reseeds since boot
  bool     seeded;       // false ⇒ lag_now_us / lag_hwm_us are 0 because no reference exists
  bool     reseeded_since_read;  // a reseed landed since the previous take (cleared by the take)
};
// Consistent snapshot; CLEARS reseeded_since_read so the caller sees each reseed
// in exactly one window. Called at 1 Hz from task_telem.
JbLagProbe can_buses_jb_lag_take();

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
