#pragma once
// =============================================================================
//  can_buses.h — three subsystem-isolated FlexCAN_T4 buses (ADR-0013)
// =============================================================================
//  One FlexCAN_T4 peripheral per robot subsystem (supersedes the old two-bus
//  shared-aux / private-leg split):
//    bb        = CAN1 (Ball Butler Teensy only). No ODrive lives here, so RX is
//                counted but never decoded into the cache; we TX the 100 Hz
//                0x7DD time-sync broadcast.
//    cone      = CAN2 (catching cone Teensy, often physically disconnected). RX
//                counted only. TX is the 0x7DD broadcast, GATED on recent cone
//                presence so an un-ACKed send can't drive the bus to bus-off
//                (cone-absent tolerance — see can_buses.cpp + HANDOFF D2).
//    jugglebot = CAN3 (Jugglebot core: 6 leg ODrives + Hand ODrive + platform
//                Teensy 4.0 + can-bridge). ALL ODrive telemetry/heartbeat/error
//                frames decode into the per-axis cache from here; leg setpoints,
//                RPC-driven ODrive commands, and fault-machine commands TX here.
//
//  Peripheral identity (CAN1/CAN2/CAN3) appears ONLY as the FlexCAN_T4 template
//  parameter in can_buses.cpp; everywhere else the bus is named by subsystem so
//  a future re-pin only touches the wiring file (ADR-0013 / HANDOFF D3).
//
//  RX is dispatched from canX.events() (pumped by the CAN RX task) via onReceive
//  callbacks — the proven platform-Teensy idiom (Teensy_code.ino canSniff) — and
//  decoded straight into the per-axis cache (fast, bounded; the decode is
//  microseconds and the cache writes are seqlock-guarded).
//
//  NB on the data path: there IS a library-internal queue hop. The FlexCAN RX
//  interrupt pushes each frame into the peripheral's RX_SIZE_256 rxBuffer; events()
//  pops from it. (HANDOFF D7's "straight into the cache ... not by enqueuing" is
//  imprecise: events()-driven onReceive dispatch routes through that rxBuffer first.
//  There is no *application-level* queue between the callback and the cache, which is
//  what D7 meant.) Because events() pops only ONE frame per call, can_buses_service()
//  drains each bus in a bounded loop so the ~2,240 fps of CAN3 telemetry the can-bridge
//  receives cannot overflow the rxBuffer — see CAN_RX_DRAIN_BUDGET in can_buses.cpp.
// =============================================================================

#include <cstdint>
#include "odrive_protocol.h"
#include "udp_protocol.h"   // JbUdp::BusHealth

namespace CanBridge {

void can_buses_init();      // begin all three buses, register RX callbacks
void can_buses_service();   // pump bb/cone/jugglebot events(); call from CAN RX task

bool can_bb_send(const ODrive::CanFrame& f);         // CAN1 Ball Butler TX (time-sync broadcast)
bool can_cone_send(const ODrive::CanFrame& f);       // CAN2 cone TX (time-sync; GATED, cone-absent tolerant)
bool can_jugglebot_send(const ODrive::CanFrame& f);  // CAN3 Jugglebot core TX (setpoints, RPC, fault cmds)

struct CanStats {
  uint32_t bb_rx, bb_tx, cone_rx, cone_tx, jugglebot_rx, jugglebot_tx;
  uint8_t  bb_health;          // JbUdp::BusHealth
  uint8_t  cone_health;
  uint8_t  jugglebot_health;
};
CanStats can_buses_stats();

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
};

struct CanRxHealth {
  BusRxHealth bb, cone, jugglebot;
  uint32_t decode_short;     // CAN3 frames dropped in decode: DLC < 8 (truncated)
  uint32_t decode_bad_axis;  // CAN3 frames dropped in decode: node id >= NUM_AXES
};
CanRxHealth can_buses_rx_health();

}  // namespace CanBridge
