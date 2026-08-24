// =============================================================================
//  test_hand_ops.cpp — compiled test of the REAL hand traj / smooth-move conduit
// =============================================================================
//  Drives the actual compiled hand_ops.cpp (it #includes the .cpp) against the
//  recording fake HAL, asserting the safety-relevant BEHAVIOURS of the hand-conduit
//  HAND_TRAJ_CMD dispatch:
//
//    * a valid traj cmd emits the CLOSED_LOOP + POSITION/PASSTHROUGH PREAMBLE to
//      the hand ODrive (axis 6) — byte-identical to ODrive::encode_set_state /
//      encode_set_controller_mode — THEN the 0x6D0 frame (the dropped precondition
//      the audit flagged, row 37);
//    * the 0x6D0 frame carries the host-built payload VERBATIM on the FIRMWARE-
//      OWNED PlatformCanId::TRAJ_CMD id (never a Jetson-supplied raw frame; the
//      firmware forwards opaque bytes so it CANNOT re-stamp the deadline);
//    * hand_traj_cmd GATES on jugglebot_commands_allowed() (a stale/dead CAN3 →
//      ERR_BUS_DOWN, and NOTHING put on CAN3 — the hand is gated like a leg);
//    * the traj TX ABORTS with NO 0x6D0 frame if a preamble send fails (running a
//      trajectory against a hand not in CLOSED_LOOP/PASSTHROUGH would fault/no-op);
//    * every exit is ATTRIBUTED to the right HandOpsCounters slot (2026-08-02).
//
//  Why the counters are tested here and not only on the wire: hand_traj_cmd has
//  five failure exits and THREE of them return an identical bare ERR_TIMEOUT, so
//  the return value cannot distinguish them and neither could the 2026-08-01
//  recount. The counters are the only thing that can, which makes "the right one
//  incremented" the actual contract — a copy-paste that bumped pre1 from the pre2
//  branch would be invisible in every other test in this file and would send the
//  next bench session after the wrong CAN send. Each case calls
//  hand_ops_counters_reset() so the assertions are absolute rather than deltas
//  and stay order-independent (the version_check_init() isolation pattern).
//
//  SCOPE: validates DECISION LOGIC, not FreeRTOS/ISR concurrency or 500 Hz timing.
//  See README.md.
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>
#include <cstring>

#include "udp_protocol.h"
#include "protocol_config.h"
#include "odrive_protocol.h"
#include "canbridge_config.h"
#include "can_buses.h"
#include "leg_interp.h"   // declares interp_last_tick_us (we fake it below)
#include "fake_hal.h"

// ── Driver-local fake for interp_last_tick_us() ──────────────────────────────
//  Deliberately NOT in fake_hal.cpp: test_fault_machine and test_leg_interp both
//  #include the REAL leg_interp.cpp *and* link fake_hal.o, so a definition there
//  would be a duplicate-symbol link error in those two binaries (the ODR rule at
//  build.py:16-24). Driver-local stubbing is the established alternative —
//  test_udp_link.cpp does the same for micros64/net_ethernet_service.
//
//  Paired with fake_hal's clock (micros64()), this makes the dispatch PHASE
//  exactly determined: phase_us == fake_mono_us() - g_fake_interp_tick_us.
static uint64_t g_fake_interp_tick_us = 0;
namespace CanBridge {
uint64_t interp_last_tick_us() { return g_fake_interp_tick_us; }
}  // namespace CanBridge
static void fake_set_interp_tick_us(uint64_t t_us) { g_fake_interp_tick_us = t_us; }

#include "hand_ops.cpp"   // the unit under test

using namespace CanBridge;

// A representative catch-trajectory payload (byte 0 = traj_type 1, vel_u16, wall
// time low32, pad) — the firmware treats it as opaque bytes.
static JbUdp::RpcArgs::ArgHandTraj make_traj_arg() {
  JbUdp::RpcArgs::ArgHandTraj a{};
  const uint8_t payload[8] = {0x01, 0x2C, 0x01, 0x44, 0x33, 0x22, 0x11, 0x00};
  memcpy(a.payload, payload, 8);
  return a;
}

// Assert the WHOLE counter block, not just the field a case cares about: the
// failure mode worth catching is an increment landing in the wrong slot, and
// checking only the expected slot cannot see that. `ok` is derived exactly as
// the /link_status row derives it, so the identity
//   calls - rej - down - pre1 - pre2 - traj == ok
// is under test too.
static void check_counters(uint32_t calls, uint32_t rej, uint32_t down,
                           uint32_t pre1, uint32_t pre2, uint32_t traj,
                           uint32_t ok) {
  const auto c = hand_ops_counters();
  CHECK(c.calls == calls);
  CHECK(c.rej_homing == rej);
  CHECK(c.bus_down == down);
  CHECK(c.pre1_fail == pre1);
  CHECK(c.pre2_fail == pre2);
  CHECK(c.traj_fail == traj);
  CHECK(c.calls - c.rej_homing - c.bus_down - c.pre1_fail - c.pre2_fail
        - c.traj_fail == ok);
}

TEST_CASE("valid hand traj emits the CLOSED_LOOP + PASSTHROUGH preamble then the 0x6D0 frame") {
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);

  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::OK);
  // The success path increments ONLY calls — OK is derived, never counted.
  check_counters(/*calls=*/1, /*rej=*/0, /*down=*/0,
                 /*pre1=*/0, /*pre2=*/0, /*traj=*/0, /*ok=*/1);

  // Exactly 3 frames: two preamble ODrive frames to axis 6, then the 0x6D0 frame.
  REQUIRE(fake_sent_count() == 3);

  // Frame 0 = set_state(HAND_AXIS, CLOSED_LOOP) — byte-identical to the encoder.
  const auto pre0 = ODrive::encode_set_state(HAND_AXIS, ODriveState::CLOSED_LOOP);
  CHECK(fake_sent_at(0).id == pre0.id);
  CHECK(memcmp(fake_sent_at(0).buf, pre0.buf, 8) == 0);

  // Frame 1 = set_controller_mode(HAND_AXIS, POSITION, PASSTHROUGH).
  const auto pre1 = ODrive::encode_set_controller_mode(
      HAND_AXIS, ODriveControlMode::POSITION, ODriveInputMode::PASSTHROUGH);
  CHECK(fake_sent_at(1).id == pre1.id);
  CHECK(memcmp(fake_sent_at(1).buf, pre1.buf, 8) == 0);

  // Frame 2 = the 0x6D0 PLATFORM_TRAJ_CMD, id firmware-owned, payload verbatim.
  CHECK(fake_sent_at(2).id == PlatformCanId::TRAJ_CMD);   // 0x6D0
  CHECK(fake_sent_at(2).len == 8);
  CHECK(memcmp(fake_sent_at(2).buf, a.payload, 8) == 0);
}

TEST_CASE("the 0x6D0 payload is forwarded VERBATIM regardless of discriminator") {
  // A smooth-move payload (byte 0 = 3) forwards identically — the firmware does NOT
  // branch on the discriminator (it is meaningful only to the Platform Teensy).
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);

  JbUdp::RpcArgs::ArgHandTraj a{};
  const uint8_t smooth[8] = {0x03, 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00};
  memcpy(a.payload, smooth, 8);

  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::OK);
  REQUIRE(fake_sent_count() == 3);
  CHECK(fake_sent_at(2).id == PlatformCanId::TRAJ_CMD);
  CHECK(memcmp(fake_sent_at(2).buf, smooth, 8) == 0);
}

TEST_CASE("a hand traj is REJECTED while homing is active (interlock), nothing sent") {
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);    // bus is fine — the rejection is the homing interlock, not the gate
  fake_set_homing(true);              // the shared state machine is mid-homing (e.g. axis 6)

  const auto a = make_traj_arg();
  // Checked FIRST — before the bus gate and before any preamble frame reaches CAN3.
  // A catch-traj mid-homing would fight the axis-6 move-to-hardstop and corrupt the
  // just-defined HAND_ABS_POS_REV reference.
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_REJECTED);
  CHECK(fake_sent_count() == 0);      // not even the preamble reached the bus
  // A refused call still counts as a call: `calls` is the denominator, so a gate
  // that rejects everything must read as traffic, not as silence.
  check_counters(/*calls=*/1, /*rej=*/1, /*down=*/0,
                 /*pre1=*/0, /*pre2=*/0, /*traj=*/0, /*ok=*/0);
}

TEST_CASE("a dead CAN3 refuses the traj (ERR_BUS_DOWN, nothing sent)") {
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(false);   // never-command-a-dead-bus

  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_BUS_DOWN);
  CHECK(fake_sent_count() == 0);      // not even the preamble reached the bus
  check_counters(/*calls=*/1, /*rej=*/0, /*down=*/1,
                 /*pre1=*/0, /*pre2=*/0, /*traj=*/0, /*ok=*/0);
}

TEST_CASE("a failed FIRST preamble send aborts the traj (no controller-mode, no 0x6D0)") {
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);
  fake_set_send_fail_index(0);        // set_state fails to enqueue

  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_TIMEOUT);
  // The failed send is not recorded, and the abort short-circuits before the
  // controller-mode preamble AND the 0x6D0 forward.
  CHECK(fake_sent_count() == 0);
  // The whole point of the split: this ERR_TIMEOUT and the next two cases' are
  // byte-identical on the wire, and ONLY the counter says which send refused.
  check_counters(/*calls=*/1, /*rej=*/0, /*down=*/0,
                 /*pre1=*/1, /*pre2=*/0, /*traj=*/0, /*ok=*/0);
}

TEST_CASE("a failed SECOND preamble send aborts the traj (no 0x6D0 frame)") {
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);
  fake_set_send_fail_index(1);        // set_controller_mode fails to enqueue

  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_TIMEOUT);
  // Only the first preamble (set_state) was recorded; the controller-mode send
  // failed and the 0x6D0 frame was NEVER attempted — the crux: a trajectory must
  // not run against a hand that is not in CLOSED_LOOP/PASSTHROUGH.
  REQUIRE(fake_sent_count() == 1);
  for (size_t i = 0; i < fake_sent_count(); ++i) {
    CHECK(fake_sent_at(i).id != PlatformCanId::TRAJ_CMD);
  }
  check_counters(/*calls=*/1, /*rej=*/0, /*down=*/0,
                 /*pre1=*/0, /*pre2=*/1, /*traj=*/0, /*ok=*/0);
}

TEST_CASE("a failed 0x6D0 send is attributed to the TRAJ stage, not to a preamble") {
  // The third send had NO case at all before 2026-08-02, which is the exit an
  // arm-ack failure most plausibly comes from: by then both preambles have
  // consumed TX mailboxes, so the traj frame is the one most likely to find the
  // queue full. Both preambles must still have gone out — a traj-stage failure
  // is NOT a preamble failure, and conflating them would point a bench session
  // at the hand's CLOSED_LOOP transition instead of at TX-queue pressure.
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);
  fake_set_send_fail_index(2);        // the 0x6D0 traj frame fails to enqueue

  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_TIMEOUT);
  REQUIRE(fake_sent_count() == 2);    // both preambles landed; the traj did not
  for (size_t i = 0; i < fake_sent_count(); ++i) {
    CHECK(fake_sent_at(i).id != PlatformCanId::TRAJ_CMD);
  }
  check_counters(/*calls=*/1, /*rej=*/0, /*down=*/0,
                 /*pre1=*/0, /*pre2=*/0, /*traj=*/1, /*ok=*/0);
}

TEST_CASE("counters accumulate across calls and stay attributed per stage") {
  // Cumulative-since-boot is the contract the wire relies on (the consumer
  // differences two captures), so a mixed sequence must sum correctly rather
  // than latch a last-writer-wins verdict — the exact weakness that ruled out
  // a single last_fail_stage byte.
  fake_reset();
  hand_ops_counters_reset();
  const auto a = make_traj_arg();

  fake_set_commands_allowed(true);
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::OK);          // ok #1

  fake_set_send_fail_index(2);
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_TIMEOUT); // traj #1

  fake_set_send_fail_index(0);
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_TIMEOUT); // pre1 #1

  fake_set_commands_allowed(false);
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_BUS_DOWN); // down #1

  fake_set_commands_allowed(true);
  fake_set_send_fail_index(-1);
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::OK);          // ok #2

  check_counters(/*calls=*/5, /*rej=*/0, /*down=*/1,
                 /*pre1=*/1, /*pre2=*/0, /*traj=*/1, /*ok=*/2);
}

// ── Interp-phase stamp (2026-08-09) ─────────────────────────────────────────
//  The [handphase] ring is the falsifiable test for the phase-locked-dispatch
//  verdict (logbook 2026-08-02 addendum § A3), so what has to be right is that the
//  stamp is taken at ENTRY — the same point for every dispatch, before the gates —
//  and that the exit stage is labelled correctly. A stamp taken per-exit instead
//  would make the OK and FAIL phase distributions incomparable by construction,
//  and the bench read (two clusters vs uniform) would be meaningless while still
//  looking like data. That is the regression these cases exist to catch.
TEST_CASE("a dispatch is phase-stamped at ENTRY and labelled with its exit stage") {
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);
  const auto a = make_traj_arg();

  // OK dispatch at phase 812 µs into the 2 ms interp cycle.
  fake_set_interp_tick_us(5'000'000);
  fake_set_clock(/*wall=*/0, /*mono=*/5'000'812);
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::OK);

  // A pre2 failure at a DIFFERENT phase. The stamp must be the entry phase, not
  // anything measured at the failing send.
  fake_set_send_fail_index(1);            // set_controller_mode refuses to enqueue
  fake_set_interp_tick_us(9'000'000);
  fake_set_clock(/*wall=*/0, /*mono=*/9'001'793);
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_TIMEOUT);

  const auto r = hand_phase_ring();
  REQUIRE(r.n == 2);
  CHECK(r.v[0].phase_us == 812);
  CHECK(r.v[0].outcome  == HAND_PHASE_OK);
  CHECK(r.v[1].phase_us == 1793);
  CHECK(r.v[1].outcome  == HAND_PHASE_PRE2);
  // The stamp is orthogonal to the counters — both instruments must agree.
  check_counters(/*calls=*/2, /*rej=*/0, /*down=*/0,
                 /*pre1=*/0, /*pre2=*/1, /*traj=*/0, /*ok=*/1);
}

TEST_CASE("the reset seam clears the phase ring too, not just the counters") {
  // The ring is folded into hand_ops_counters_reset() precisely so a case cannot
  // isolate one instrument and inherit the other's leftovers — a ring that
  // survived reset would let a stale sample from a previous case masquerade as
  // this one's phase.
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);
  fake_set_interp_tick_us(1'000'000);
  fake_set_clock(/*wall=*/0, /*mono=*/1'000'400);
  CHECK(HandOps::hand_traj_cmd(make_traj_arg()) == JbUdp::RpcStatus::OK);
  REQUIRE(hand_phase_ring().n == 1);

  hand_ops_counters_reset();
  const auto r = hand_phase_ring();
  CHECK(r.n == 0);
  CHECK(r.v[0].phase_us == 0);
  CHECK(r.v[0].outcome  == HAND_PHASE_OK);   // 0 == HAND_PHASE_OK; the count is what says "empty"
}

TEST_CASE("the reset seam zeroes every counter") {
  // Production never calls this; it exists so the cases above are absolute and
  // order-independent. If it silently missed a field, every earlier case would
  // start reading a neighbour's leftovers — a failure that presents as a
  // mis-attributed stage, i.e. as the exact bug these counters exist to catch.
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);
  const auto a = make_traj_arg();
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::OK);
  fake_set_commands_allowed(false);
  CHECK(HandOps::hand_traj_cmd(a) == JbUdp::RpcStatus::ERR_BUS_DOWN);

  hand_ops_counters_reset();
  check_counters(/*calls=*/0, /*rej=*/0, /*down=*/0,
                 /*pre1=*/0, /*pre2=*/0, /*traj=*/0, /*ok=*/0);
}

// ═══════════════════════════════════════════════════════════════════════════
//  TRI-STATE TX (2026-08-24) — the ack stops lying about a deferred dispatch
// ═══════════════════════════════════════════════════════════════════════════

TEST_CASE("a DEFERRED dispatch acks OK: the frames went out, so the ack says so") {
  // THE DEFECT THIS CLOSES, in one sentence: every send here ended in
  // `bus.write(m) > 0`, and FlexCAN_T4::write returns -1 for "no mailbox free,
  // QUEUED into the software txBuffer" — a frame that transmits, in order, ~0.1-1 ms
  // later. So a dispatch under TX pressure was acked ERR_TIMEOUT while the bench
  // watched its frames go out. That is the 2026-08-09 "lying ack", and it is what
  // drove the hand ladders and the _MAX_ARM_DISPATCHES cap.
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);
  fake_set_send_defer_all(true);          // every mailbox busy: all three sends defer

  CHECK(HandOps::hand_traj_cmd(make_traj_arg()) == JbUdp::RpcStatus::OK);

  // All three frames are on the bus — deferral is a queue hop, not a drop.
  REQUIRE(fake_sent_count() == 3);
  const auto traj_id = PlatformCanId::TRAJ_CMD;
  CHECK(fake_sent_at(2).id == traj_id);   // the 0x6D0 traj frame was NOT aborted

  // No stage is charged a failure, so the OK count derives correctly and the
  // /link_status attribution row does not manufacture a phantom timeout.
  check_counters(/*calls=*/1, /*rej=*/0, /*down=*/0,
                 /*pre1=*/0, /*pre2=*/0, /*traj=*/0, /*ok=*/1);

  // Every one of the three is charged to the HAND deferral bucket, not to a
  // neighbour's — a mis-bucketed dispatch would hide inside the leg burst total,
  // which is the one place this pressure is guaranteed to be invisible.
  CHECK(fake_sent_count_cls(TxCls::HAND) == 3u);
  CHECK(fake_sent_count_cls(TxCls::LEGS) == 0u);

  // AND NOTHING RE-DISPATCHED. The lying-ack lesson stands: a truthful ack is the
  // fix, a blind retry is not. Exactly three frames, one per logical send.
  CHECK(fake_sent_count() == 3);
}

TEST_CASE("a FAILED preamble still aborts: only FAILED means the frame never left") {
  // The abort path is unchanged for a genuine failure — the presence gate refusing
  // a partner-less bus. Running a trajectory against a hand that never received its
  // CLOSED_LOOP would fault or silently no-op the move, so this must stay.
  fake_reset();
  hand_ops_counters_reset();
  fake_set_commands_allowed(true);
  fake_set_send_defer_all(true);          // deferral is NOT what aborts...
  fake_set_send_fail_index(0);            // ...a real failure is

  CHECK(HandOps::hand_traj_cmd(make_traj_arg()) == JbUdp::RpcStatus::ERR_TIMEOUT);
  CHECK(fake_sent_count() == 0);          // nothing reached the bus at all
  check_counters(/*calls=*/1, /*rej=*/0, /*down=*/0,
                 /*pre1=*/1, /*pre2=*/0, /*traj=*/0, /*ok=*/0);
}
