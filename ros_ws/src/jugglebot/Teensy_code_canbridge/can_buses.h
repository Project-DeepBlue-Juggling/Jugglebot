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
//  decoded straight into the per-axis cache (fast, bounded; no queue hop). This
//  deviates from the plan's "FlexCAN ISR + queue → RX task" purely for lower
//  latency and fewer moving parts; the decode is microseconds and the cache
//  writes are seqlock-guarded.
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

}  // namespace CanBridge
