// =============================================================================
//  test_can_tx_result.cpp — the TX tri-state classifier, EXECUTED
// =============================================================================
//  can_buses.cpp compiles in no native test binary (the coverage gap recorded at
//  BusRxHealth::tx_deferred), so for four years the rule that decided whether a
//  frame "went out" — `bus.write(m) > 0` — lived somewhere no test could reach.
//  That rule was WRONG, and its wrongness is the mechanism behind the 2026-08-09
//  lying-ack: FlexCAN_T4::write(const CAN_message_t&) returns -1 for "no mailbox
//  free, QUEUED into the software txBuffer", which every caller read as failure.
//
//  The fix moved the rule OUT of the .cpp and into a header-inline pure
//  classifier, classify_tx_write(), for exactly the reason this file exists: a
//  contract that cannot be executed cannot be defended. This driver pins the
//  classifier and the two predicates built on it against the vendored library's
//  documented return contract.
//
//  Pure header (can_buses.h) — no firmware .cpp is #included and no object is
//  linked, so nothing here can be perturbed by another TU's file-statics.
//
//  THE RETURN CONTRACT BEING PINNED, quoted from the vendored source
//  (lib/FlexCAN_T4/FlexCAN_T4.tpp, FCTP_OPT::write(const CAN_message_t&)):
//      writeTxMailbox(i, msg); return 1;   /* transmit entry accepted */
//      struct2queueTx(msg_copy); return -1;/* transmit entry failed, no mailboxes
//                                             available, queued */
//  — and NO path in that overload returns 0. The mailbox-indexed overload
//  write(FLEXCAN_MAILBOX, ...) DOES return 0 (FIFO / not a transmit mailbox), and
//  nothing in this tree calls it; 0 is nevertheless classified, as FAILED, because
//  it is the one return value meaning the frame reached neither hardware nor the
//  queue. The source-level half of this pin (that the .tpp still says so, and that
//  FW 14's P4 `break` still guards the deferral drain) is
//  tests/firmware/test_flexcan_tx_defer_guard.py.
// =============================================================================

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <cstdint>

#include "protocol_config.h"
#include "canbridge_config.h"
#include "odrive_protocol.h"
#include "can_buses.h"

using namespace CanBridge;

// ── 1. The classifier: +1 is a mailbox, -1 is a DEFERRAL, 0 is a loss ────────

TEST_CASE("classify_tx_write maps the FlexCAN_T4 write() return contract") {
  CHECK(classify_tx_write(1)  == TxResult::MAILBOX);
  CHECK(classify_tx_write(-1) == TxResult::DEFERRED);
  // 0 never comes back from the overload this firmware uses; classified as FAILED
  // because on the overload that DOES return it, the frame was neither written nor
  // queued. Defaulting it to DEFERRED would invent a transmission that never happened.
  CHECK(classify_tx_write(0)  == TxResult::FAILED);

  // The library returns only 1 and -1, but the classifier must not depend on the
  // magnitude — a future library that returned a mailbox index would still be
  // "accepted", and a more negative code would still be "queued".
  CHECK(classify_tx_write(7)   == TxResult::MAILBOX);
  CHECK(classify_tx_write(-42) == TxResult::DEFERRED);
}

// ── 2. The predicate that replaced the old bool ──────────────────────────────

TEST_CASE("a DEFERRED frame counts as having reached the wire") {
  // THE REGRESSION THIS GUARDS. `bus.write(m) > 0` made this false, and that is
  // what turned a ~0.1-1 ms queue hop into an ERR_TIMEOUT ack for a hand dispatch
  // the bench then observed transmitting. If a future edit re-collapses the
  // tri-state, this is the assertion that fires.
  CHECK(tx_reached_the_wire(TxResult::DEFERRED) == true);
  CHECK(tx_reached_the_wire(TxResult::MAILBOX)  == true);
  // FAILED is the bus-partner presence gate refusing — the ONE outcome in which
  // no frame exists to wait for, ack, or retry.
  CHECK(tx_reached_the_wire(TxResult::FAILED)   == false);

  CHECK(tx_was_deferred(TxResult::DEFERRED) == true);
  CHECK(tx_was_deferred(TxResult::MAILBOX)  == false);
  CHECK(tx_was_deferred(TxResult::FAILED)   == false);
}

TEST_CASE("the old bool is exactly `not FAILED`, for every write() return") {
  // The bool wrapper the cold-start ladders still use must agree with the
  // predicate on every input, or a ladder and an RPC would disagree about whether
  // the same frame went out.
  for (int rc = -3; rc <= 3; ++rc) {
    const TxResult r = classify_tx_write(rc);
    CHECK(tx_reached_the_wire(r) == (r != TxResult::FAILED));
    // And the sign rule, restated independently of the enum: only rc == 0 is a loss.
    CHECK(tx_reached_the_wire(r) == (rc != 0));
  }
}

// ── 3. The census class vocabulary ───────────────────────────────────────────

TEST_CASE("every TxCls is distinct and inside the census array") {
  const uint8_t all[] = {TxCls::POLLER, TxCls::LEGS, TxCls::HAND, TxCls::RPC,
                         TxCls::SAFETY, TxCls::TIMESYNC, TxCls::OTHER};
  const size_t n = sizeof(all) / sizeof(all[0]);
  CHECK(n == (size_t)TxCls::COUNT);          // the vocabulary is complete
  for (size_t i = 0; i < n; ++i) {
    CHECK(all[i] < TxCls::COUNT);            // every class indexes the census array
    for (size_t j = i + 1; j < n; ++j)
      CHECK(all[i] != all[j]);               // no two classes share a bucket
  }
  // SAFETY is broken out as its OWN bucket on purpose: a deferred E-stop /
  // CLEAR_ERRORS / relay op is a watch signal, and folding it into a leg-burst
  // total is exactly how it would go unnoticed.
  CHECK(TxCls::SAFETY != TxCls::LEGS);
  // The census struct must be able to hold one counter per class.
  TxDeferCensus c{};
  CHECK(sizeof(c.by_class) / sizeof(c.by_class[0]) == (size_t)TxCls::COUNT);
  for (uint8_t i = 0; i < TxCls::COUNT; ++i) CHECK(c.by_class[i] == 0u);
}
