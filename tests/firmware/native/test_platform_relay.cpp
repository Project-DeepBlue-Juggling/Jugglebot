// =============================================================================
//  test_platform_relay.cpp — compiled test of the REAL Platform-Teensy relay seam
// =============================================================================
//  Drives the actual compiled platform_relay.cpp (it #includes the .cpp) against
//  the recording fake HAL, asserting the safety-relevant BEHAVIOURS of the
//  Platform-Teensy relay write seam:
//
//    * tilt_read() / state_read() emit the right CAN3 TRIGGER frame on the right
//      arbitration id (0x7DE / 0x6E0) and dlc — so a Platform-Teensy reply is
//      actually solicited;
//    * state_write() byte-for-byte re-encodes the 0x6E0 RobotState frame exactly
//      as Teensy_code_platform.ino decodeStateCANMessage unpacks it (flags + int16 pose) —
//      so the can-bridge, as the SOLE writer, cannot corrupt the persisted state;
//    * every relay send is GATED on jugglebot_commands_allowed() (the never-
//      command-a-dead-bus fail-fast: ERR_BUS_DOWN, and NOTHING put on CAN3);
//    * is_platform_reply_id() classifies the two reply ids and nothing else (the
//      CAN3 RX ring filter that routes replies to the verbatim uplink);
//    * is_cone_id() / is_clapboard_id() split the cone bus by ROLE (catching cone
//      0x7E0-0x7E1 vs electronic clapboard 0x7E8-0x7EF) — the classifier behind
//      honest cone_health once the two devices share that bus;
//    * the generated hand_axis6_permitted() allow-table permits exactly the hand
//      ODrive ops the plan locks and rejects the rest (the compiled predicate the
//      rpc.cpp axis-6 gate consumes).
//
//  SCOPE: validates DECISION LOGIC, not FreeRTOS/ISR concurrency or 500 Hz
//  timing (the SPSC relay ring + the on-wire SRX_DIS reply-correlation assumption
//  remain on-hardware-replay/bench gaps). See README.md.
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>
#include <cmath>                 // NAN / INFINITY (state_write float validation)

#include "udp_protocol.h"
#include "protocol_config.h"
#include "odrive_protocol.h"
#include "can_buses.h"
#include "rpc.h"                 // method_gates_on_bus_transmittable (reboot-latch gate policy)
#include "fake_hal.h"

#include "platform_relay.cpp"   // the unit under test

using namespace CanBridge;

// Decode a recorded little-endian int16 from a frame buffer (pose field check).
static int16_t le_i16(const uint8_t* b) {
  return (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

TEST_CASE("relay reads emit the right CAN3 trigger frame") {
  fake_reset();
  fake_set_commands_allowed(true);

  CHECK(Relay::tilt_read() == JbUdp::RpcStatus::OK);
  REQUIRE(fake_sent_count() == 1);
  CHECK(fake_sent_at(0).id == PlatformCanId::TILT_READING);   // 0x7DE
  CHECK(fake_sent_at(0).len == 1);                            // dlc 1 (≠ the dlc-8 reply)

  fake_clear_sent();
  CHECK(Relay::state_read() == JbUdp::RpcStatus::OK);
  REQUIRE(fake_sent_count() == 1);
  CHECK(fake_sent_at(0).id == PlatformCanId::STATE_UPDATE);   // 0x6E0
  CHECK(fake_sent_at(0).len == 1);
  CHECK(fake_sent_at(0).buf[0] == 0x01);                      // request discriminator
}

TEST_CASE("state_write re-encodes the 0x6E0 RobotState frame (createStateCANMessage parity)") {
  fake_reset();
  fake_set_commands_allowed(true);

  JbUdp::RpcArgs::ArgRobotState s{};
  s.is_homed = 1;
  s.levelling_complete = 0;
  s.pose_offset_tiltX = 0.012f;    // *1000 → 12
  s.pose_offset_tiltY = -0.034f;   // *1000 → -34
  CHECK(Relay::state_write(s) == JbUdp::RpcStatus::OK);

  REQUIRE(fake_sent_count() == 1);
  const SentFrame& f = fake_sent_at(0);
  CHECK(f.id == PlatformCanId::STATE_UPDATE);
  CHECK(f.len == 8);
  CHECK(f.buf[0] == 0x01);          // flags: bit0 is_homed=1, bit1 levelling=0
  CHECK(le_i16(&f.buf[1]) == 12);   // pose_offset_tiltX * 1000
  CHECK(le_i16(&f.buf[3]) == -34);  // pose_offset_tiltY * 1000
  CHECK(f.buf[5] == 0);
  CHECK(f.buf[6] == 0);
  CHECK(f.buf[7] == 0);

  // The flags byte ORs both bits when both are set.
  fake_clear_sent();
  s.is_homed = 1; s.levelling_complete = 1;
  s.pose_offset_tiltX = 0.0f; s.pose_offset_tiltY = 0.0f;
  CHECK(Relay::state_write(s) == JbUdp::RpcStatus::OK);
  REQUIRE(fake_sent_count() == 1);
  CHECK(fake_sent_at(0).buf[0] == 0x03);   // bit0 | bit1
}

TEST_CASE("state_write rejects a non-finite pose offset (ERR_BAD_ARGS, nothing sent)") {
  // A NaN/Inf pose offset makes the `* 1000.0f` → int16 cast UNDEFINED, and the corrupt
  // value would be PERSISTED on the Platform Teensy across reboots. Reject before the cast.
  fake_reset();
  fake_set_commands_allowed(true);

  JbUdp::RpcArgs::ArgRobotState s{};
  s.is_homed = 1;
  s.pose_offset_tiltX = NAN;   s.pose_offset_tiltY = 0.0f;
  CHECK(Relay::state_write(s) == JbUdp::RpcStatus::ERR_BAD_ARGS);
  CHECK(fake_sent_count() == 0);          // rejected before any CAN3 frame

  s.pose_offset_tiltX = 0.0f;  s.pose_offset_tiltY = INFINITY;
  CHECK(Relay::state_write(s) == JbUdp::RpcStatus::ERR_BAD_ARGS);
  CHECK(fake_sent_count() == 0);

  s.pose_offset_tiltX = -INFINITY;  s.pose_offset_tiltY = NAN;
  CHECK(Relay::state_write(s) == JbUdp::RpcStatus::ERR_BAD_ARGS);
  CHECK(fake_sent_count() == 0);

  // A finite pose still encodes + sends (the guard is non-finite ONLY).
  s.pose_offset_tiltX = 0.012f;  s.pose_offset_tiltY = -0.034f;
  CHECK(Relay::state_write(s) == JbUdp::RpcStatus::OK);
  REQUIRE(fake_sent_count() == 1);
  CHECK(le_i16(&fake_sent_at(0).buf[1]) == 12);
  CHECK(le_i16(&fake_sent_at(0).buf[3]) == -34);
}

TEST_CASE("relay sends fail-fast when CAN3 is down (never command a dead bus)") {
  fake_reset();
  fake_set_commands_allowed(false);

  CHECK(Relay::tilt_read()  == JbUdp::RpcStatus::ERR_BUS_DOWN);
  CHECK(Relay::state_read() == JbUdp::RpcStatus::ERR_BUS_DOWN);
  JbUdp::RpcArgs::ArgRobotState s{};
  s.is_homed = 1;
  CHECK(Relay::state_write(s) == JbUdp::RpcStatus::ERR_BUS_DOWN);

  // Crucially: nothing reached CAN3 (the bus-down fail-fast is BEFORE the send).
  CHECK(fake_sent_count() == 0);
}

TEST_CASE("is_platform_reply_id classifies exactly the two relay reply ids") {
  CHECK(is_platform_reply_id(PlatformCanId::STATE_UPDATE));   // 0x6E0
  CHECK(is_platform_reply_id(PlatformCanId::TILT_READING));   // 0x7DE
  CHECK_FALSE(is_platform_reply_id(PlatformCanId::TRAJ_CMD)); // 0x6D0 (hand traj — not a reply)
  CHECK_FALSE(is_platform_reply_id(ODrive::arb_id(0, ODriveCmd::heartbeat_message)));  // a leg frame
  CHECK_FALSE(is_platform_reply_id(0x000u));
}

TEST_CASE("cone-bus role discriminators split the cone and clapboard id blocks") {
  // The cone bus is shared BY ROLE, not by device: a catching cone (0x7E0/0x7E1)
  // or an electronic clapboard (0x7E8-0x7EF) — never both, they are mutually
  // exclusive by physical connection. cone_health is manufactured from a
  // timestamp that on_cone_rx stamps for EVERY frame on the bus, so without this
  // split the bridge reports a catching cone whenever a clapboard heartbeats, and
  // that field is operator-facing: a wire field naming the wrong peripheral is
  // worse than one reporting nothing, because it will be believed.
  //
  // Normative id allocation: Electronic-Clapboard docs/protocol.md §8.2, a
  // cross-repo contract neither side may change unilaterally.
  //
  // Pinned here BEFORE any caller exists (Phase 2 of
  // plans/active/clapboard-can3-integration.md wires these into on_cone_rx),
  // because the classifier is the only branching part of that change and
  // can_buses.cpp compiles in no native binary — the FlexCAN_T4 host shim that
  // would close that gap stays deliberately unbuilt (can_buses.h, BusRxHealth
  // coverage-gap note). Header-inline extraction buys the coverage that matters
  // for ~1 % of the shim's cost.

  // ── Cone: exactly the two allocated ids, and not the clapboard's ──
  CHECK(is_cone_id(CatchingConeCanId::CATCH_EVENT));            // 0x7E0
  CHECK(is_cone_id(CatchingConeCanId::HEARTBEAT));              // 0x7E1
  CHECK_FALSE(is_clapboard_id(CatchingConeCanId::CATCH_EVENT));
  CHECK_FALSE(is_clapboard_id(CatchingConeCanId::HEARTBEAT));

  // ── Clapboard: the WHOLE 0x7E8-0x7EF block, reserved pair included ──
  for (uint32_t id = 0x7E8u; id <= 0x7EFu; ++id) {
    CHECK(is_clapboard_id(id));
    CHECK_FALSE(is_cone_id(id));
  }
  // ...and each named frame spelled out, so a renumber in either repo fails HERE
  // rather than as a silently unclassified frame on the bench.
  CHECK(is_clapboard_id(0x7E8u));   // CLAP_FIELD       J→C
  CHECK(is_clapboard_id(0x7E9u));   // CLAP_COMMIT      J→C
  CHECK(is_clapboard_id(0x7EAu));   // CLAP_LINK        Bridge→C
  CHECK(is_clapboard_id(0x7EBu));   // CLAP_ACK         C→J
  CHECK(is_clapboard_id(0x7ECu));   // CLAP_HEARTBEAT   C→J
  CHECK(is_clapboard_id(0x7EDu));   // CLAP_FIRE_EVENT  C→J
  CHECK(is_clapboard_id(0x7EEu));   // reserved — still clapboard territory
  CHECK(is_clapboard_id(0x7EFu));   // reserved

  // ── Boundaries: both blocks, both sides, explicit ──
  CHECK_FALSE(is_cone_id(0x7DFu));       // one below the cone allocation
  CHECK(is_cone_id(0x7E0u));             //   first cone id
  CHECK(is_cone_id(0x7E1u));             //   last cone id
  CHECK_FALSE(is_cone_id(0x7E2u));       // one above
  CHECK_FALSE(is_clapboard_id(0x7E7u));  // one below the clapboard block
  CHECK(is_clapboard_id(0x7E8u));        //   first clapboard id
  CHECK(is_clapboard_id(0x7EFu));        //   last clapboard id
  CHECK_FALSE(is_clapboard_id(0x7F0u));  // one above

  // ── The unallocated gap belongs to NEITHER role ──
  // 0x7E2-0x7E7 is why is_cone_id enumerates rather than spanning a range: a
  // frame here must classify as "no known peripheral", not as a cone.
  for (uint32_t id = 0x7E2u; id <= 0x7E7u; ++id) {
    CHECK_FALSE(is_cone_id(id));
    CHECK_FALSE(is_clapboard_id(id));
  }

  // ── Neighbouring allocations are neither, including the bridge's OWN TX ──
  // 0x7DD is the 100 Hz time-sync broadcast the bridge itself puts on this bus;
  // classifying it as a peripheral would let the bridge see its own traffic as
  // proof that a peripheral is attached.
  CHECK_FALSE(is_cone_id(SharedCanId::TIME_SYNC));               // 0x7DD
  CHECK_FALSE(is_clapboard_id(SharedCanId::TIME_SYNC));
  CHECK_FALSE(is_cone_id(PlatformCanId::TILT_READING));          // 0x7DE
  CHECK_FALSE(is_clapboard_id(PlatformCanId::TILT_READING));
  CHECK_FALSE(is_cone_id(PlatformCanId::TRAFFIC_REPORT));        // 0x7DF
  CHECK_FALSE(is_clapboard_id(PlatformCanId::TRAFFIC_REPORT));
  CHECK_FALSE(is_cone_id(0x7F0u));
  CHECK_FALSE(is_clapboard_id(0x7F0u));
  CHECK_FALSE(is_cone_id(0x000u));
  CHECK_FALSE(is_clapboard_id(0x000u));

  // The three id classifiers in this header must not overlap: a Platform-Teensy
  // relay reply is neither a cone nor a clapboard frame (different bus, and the
  // cone-bus RX path must never route one into the role discriminators).
  CHECK_FALSE(is_cone_id(PlatformCanId::STATE_UPDATE));         // 0x6E0
  CHECK_FALSE(is_clapboard_id(PlatformCanId::STATE_UPDATE));
  CHECK_FALSE(is_platform_reply_id(CatchingConeCanId::CATCH_EVENT));
  CHECK_FALSE(is_platform_reply_id(CatchingConeCanId::HEARTBEAT));
  CHECK_FALSE(is_platform_reply_id(0x7EAu));   // CLAP_LINK
  CHECK_FALSE(is_platform_reply_id(0x7ECu));   // CLAP_HEARTBEAT

  // ── Mutual exclusivity is TOTAL, not merely local ──
  // Sweep the whole 11-bit standard id space: no id may satisfy both predicates.
  // The health discriminator keys a presence decision on this pair, and "both
  // peripherals present" is a state the physical wiring makes impossible — so it
  // must be unrepresentable here too. Accumulated (not 2048 CHECKs) so a failure
  // names the offending id.
  constexpr uint32_t NONE = 0x1000u;   // sentinel: outside the 11-bit space
  uint32_t both = NONE;
  for (uint32_t id = 0; id <= 0x7FFu && both == NONE; ++id) {
    if (is_cone_id(id) && is_clapboard_id(id)) both = id;
  }
  CHECK(both == NONE);
}

TEST_CASE("bus_partner_present pins the TX presence-gate window semantics") {
  // The can_*_send() gate (2026-07-05 marginal-CAN3 fix): TX is allowed only when
  // some partner frame arrived within BUS_PARTNER_STALENESS_US. Pin the window
  // value and the boundary/never-seen semantics so a config drift or an off-by-one
  // in the predicate fails loudly here rather than as silent bus-off pollution.
  constexpr uint64_t W = CanBridge::BUS_PARTNER_STALENESS_US;
  CHECK(W == 5000000u);                                    // 5.0 s contract window
  CHECK_FALSE(CanBridge::bus_partner_present(0, 0));       // never seen ⇒ absent (boot)
  CHECK_FALSE(CanBridge::bus_partner_present(0, 10 * W));  // never seen stays absent
  CHECK(CanBridge::bus_partner_present(1000, 1000));       // same-instant frame ⇒ present
  CHECK(CanBridge::bus_partner_present(1000, 1000 + W));   // exactly at window ⇒ present
  CHECK_FALSE(CanBridge::bus_partner_present(1000, 1001 + W));  // one past ⇒ absent
}

TEST_CASE("classify_bus_health pins the health_of truth table (bus-off wiring 2026-07-05)") {
  // health_of() = classify_bus_health(last_rx, now, flt_live): RX staleness plus
  // the LIVE fault-confinement state (0 active / 1 passive / 2 bus-off). Pin the
  // severity ordering and boundaries — WARN/BUS_OFF refuse motion commands
  // (jugglebot_commands_allowed + the homing/activate/deactivate gates), so a
  // drift here silently changes when the robot refuses to move.
  using CanBridge::classify_bus_health;
  constexpr uint64_t T = CanBridge::CAN_HEARTBEAT_TIMEOUT_US;
  CHECK(T == 2000000u);   // 2.0 s staleness contract (can_node _HEARTBEAT_TIMEOUT_S parity)

  // Never seen a frame ⇒ UNKNOWN, whatever the registers say (bring-up allows
  // commands; the TX presence gate independently refuses sends on such a bus).
  CHECK(classify_bus_health(0, 0, 0) == JbUdp::BusHealth::UNKNOWN);
  CHECK(classify_bus_health(0, 10 * T, 2) == JbUdp::BusHealth::UNKNOWN);

  // Live confinement, RX fresh: active ⇒ OK, passive ⇒ WARN, bus-off ⇒ BUS_OFF.
  CHECK(classify_bus_health(1000, 1000, 0) == JbUdp::BusHealth::OK);
  CHECK(classify_bus_health(1000, 1000, 1) == JbUdp::BusHealth::WARN);
  CHECK(classify_bus_health(1000, 1000, 2) == JbUdp::BusHealth::BUS_OFF);
  CHECK(classify_bus_health(1000, 1000, 3) == JbUdp::BusHealth::BUS_OFF);  // clamped ≥2

  // Staleness boundary (active confinement): exactly at the window ⇒ still OK
  // (strict >, matching the pre-wiring health_of), one past ⇒ WARN.
  CHECK(classify_bus_health(1000, 1000 + T, 0) == JbUdp::BusHealth::OK);
  CHECK(classify_bus_health(1000, 1001 + T, 0) == JbUdp::BusHealth::WARN);

  // Bus-off outranks staleness (a bus-off controller stops receiving, so the two
  // co-occur; report the cause, not the symptom).
  CHECK(classify_bus_health(1000, 1001 + T, 2) == JbUdp::BusHealth::BUS_OFF);
  // Passive + stale still WARN (both terms agree).
  CHECK(classify_bus_health(1000, 1001 + T, 1) == JbUdp::BusHealth::WARN);
}

TEST_CASE("classify_command_gate keys on SUSTAINED confinement, health stays instantaneous") {
  // The 2026-07-29 CAN3 flap fix. classify_bus_health REPORTS instantaneous
  // confinement (the uplink must keep telling the truth); classify_command_gate
  // ACTS only once error-passive has PERSISTED >= CAN_PASSIVE_SUSTAIN_US. Root
  // cause: error-passive is transient by construction (TEC decays -1 per clean
  // TX), so gating every CAN3 command on an instantaneous reading created
  // positive feedback — refusing TX removes the 0x7DD traffic that IS the decay
  // pump — and amplified a low-rate wire-error source into a 42.4%-duty outage
  // across every consumer of the predicate. See can_buses.h.
  //
  // The classifier takes flt_sustained as an INPUT: the dwell timing itself is
  // maintained in service_bus() from the 1 kHz ESR1 read (can_buses.cpp) and is
  // not reachable from the native harness. What is pinned here is the truth
  // table the gate applies to it.
  using CanBridge::classify_bus_health;
  using CanBridge::classify_command_gate;
  constexpr uint64_t T = CanBridge::CAN_HEARTBEAT_TIMEOUT_US;
  CHECK(CanBridge::CAN_PASSIVE_SUSTAIN_US == 1000000u);   // 1.0 s dwell contract
  CHECK(CanBridge::CAN_PASSIVE_SUSTAIN_US < T);           // never slower than staleness

  // Never seen a frame ⇒ UNKNOWN, whatever the registers say (bring-up allows
  // commands; the TX presence gate independently refuses sends on such a bus).
  CHECK(classify_command_gate(0, 0, 0, 0) == JbUdp::BusHealth::UNKNOWN);
  CHECK(classify_command_gate(0, 10 * T, 2, 1) == JbUdp::BusHealth::UNKNOWN);

  // Active confinement, RX fresh ⇒ OK regardless of the (meaningless) sustain bit.
  CHECK(classify_command_gate(1000, 1000, 0, 0) == JbUdp::BusHealth::OK);
  CHECK(classify_command_gate(1000, 1000, 0, 1) == JbUdp::BusHealth::OK);

  // THE FIX: passive but NOT yet sustained ⇒ commands still ALLOWED (OK), where
  // classify_bus_health reports WARN for the very same registers.
  CHECK(classify_command_gate(1000, 1000, 1, 0) == JbUdp::BusHealth::OK);
  CHECK(classify_bus_health(1000, 1000, 1) == JbUdp::BusHealth::WARN);

  // Passive AND sustained ⇒ refuse. A genuinely stuck bus still gates.
  CHECK(classify_command_gate(1000, 1000, 1, 1) == JbUdp::BusHealth::WARN);

  // Bus-off refuses INSTANTLY — exempt from the dwell requirement in both
  // directions (it is not a counter excursion; TX is physically impossible).
  CHECK(classify_command_gate(1000, 1000, 2, 0) == JbUdp::BusHealth::BUS_OFF);
  CHECK(classify_command_gate(1000, 1000, 2, 1) == JbUdp::BusHealth::BUS_OFF);
  CHECK(classify_command_gate(1000, 1000, 3, 0) == JbUdp::BusHealth::BUS_OFF);  // clamped >=2

  // RX staleness is unchanged and still instantaneous (it is sustained by
  // definition — 2 s of silence is already a dwell).
  CHECK(classify_command_gate(1000, 1000 + T, 0, 0) == JbUdp::BusHealth::OK);       // boundary: strict >
  CHECK(classify_command_gate(1000, 1001 + T, 0, 0) == JbUdp::BusHealth::WARN);
  // ...and it refuses even when confinement is clean and unsustained.
  CHECK(classify_command_gate(1000, 1001 + T, 1, 0) == JbUdp::BusHealth::WARN);
  // Bus-off still outranks staleness (report the cause, not the symptom).
  CHECK(classify_command_gate(1000, 1001 + T, 2, 0) == JbUdp::BusHealth::BUS_OFF);

  // classify_bus_health is BYTE-UNCHANGED by this work: it has no sustain input
  // and no gate state can alter its verdict. Pinned explicitly so a future
  // session cannot "simplify" the two classifiers back into one — that merge is
  // exactly what would hide the flap from the uplink and from /link_status,
  // which is the observability the fix depends on keeping.
  for (uint8_t flt = 0; flt <= 3; ++flt) {
    const uint8_t reported = classify_bus_health(1000, 1000, flt);
    // The gate may DIFFER from the report only in the passive-unsustained cell.
    const uint8_t gated_unsust = classify_command_gate(1000, 1000, flt, 0);
    const uint8_t gated_sust   = classify_command_gate(1000, 1000, flt, 1);
    if (flt == 1) {
      CHECK(reported == JbUdp::BusHealth::WARN);
      CHECK(gated_unsust == JbUdp::BusHealth::OK);     // the whole point
      CHECK(gated_sust == JbUdp::BusHealth::WARN);
    } else {
      CHECK(gated_unsust == reported);                 // identical everywhere else
      CHECK(gated_sust == reported);
    }
  }
}

TEST_CASE("hand axis-6 allow-table permits exactly the locked hand ODrive ops") {
  using namespace JbUdp;
  // Permit (the hand homing + command surface).
  CHECK(hand_axis6_permitted(RpcMethod::SET_AXIS_STATE));
  CHECK(hand_axis6_permitted(RpcMethod::SET_CONTROLLER_MODE));
  CHECK(hand_axis6_permitted(RpcMethod::SET_POS_GAIN));
  CHECK(hand_axis6_permitted(RpcMethod::SET_VEL_GAINS));
  CHECK(hand_axis6_permitted(RpcMethod::SET_VEL_CURR_LIMITS));
  CHECK(hand_axis6_permitted(RpcMethod::CLEAR_ERRORS));
  CHECK(hand_axis6_permitted(RpcMethod::REBOOT_ODRIVES));
  CHECK(hand_axis6_permitted(RpcMethod::HOME));
  CHECK(hand_axis6_permitted(RpcMethod::SET_ABSOLUTE_POSITION));
  // Reject (leg-specific cold-start moves + absolute-encoder search + non-axis ops).
  CHECK_FALSE(hand_axis6_permitted(RpcMethod::ENCODER_SEARCH));
  CHECK_FALSE(hand_axis6_permitted(RpcMethod::ACTIVATE));
  CHECK_FALSE(hand_axis6_permitted(RpcMethod::DEACTIVATE));
  CHECK_FALSE(hand_axis6_permitted(RpcMethod::SDO_READ));
  CHECK_FALSE(hand_axis6_permitted(RpcMethod::SDO_WRITE));
  CHECK_FALSE(hand_axis6_permitted(RpcMethod::NOP));
}

TEST_CASE("gate policy: only CLEAR_ERRORS/REBOOT gate on bus-transmittable (SYNCH)") {
  using namespace JbUdp;
  using CanBridge::Rpc::method_gates_on_bus_transmittable;
  // The two operator recovery one-shots gate on the LIVE bus-transmittable (SYNCH)
  // signal so a recovery clear reaches a just-repowered bus (the 2026-06-27 deadlock).
  CHECK(method_gates_on_bus_transmittable(RpcMethod::CLEAR_ERRORS));
  CHECK(method_gates_on_bus_transmittable(RpcMethod::REBOOT_ODRIVES));
  // Every other op keeps the heartbeat-staleness gate (jugglebot_commands_allowed):
  // a stale bus SHOULD withhold a setpoint/config. A future edit that drops a method
  // from the bus-transmittable set (re-gating clear/reboot onto staleness) fails here.
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::SET_AXIS_STATE));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::SET_CONTROLLER_MODE));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::SET_POS_GAIN));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::SET_VEL_GAINS));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::SET_VEL_CURR_LIMITS));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::SET_ABSOLUTE_POSITION));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::HOME));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::ACTIVATE));
  CHECK_FALSE(method_gates_on_bus_transmittable(RpcMethod::DEACTIVATE));
}
