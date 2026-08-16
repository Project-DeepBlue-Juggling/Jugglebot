// =============================================================================
//  clap_link.cpp — CLAP_LINK (0x7EA) beacon. Contract + rationale in clap_link.h
// =============================================================================
#include "clap_link.h"

#include <cstring>

#include "canbridge_config.h"
#include "protocol_config.h"   // ClapboardCanId
#include "udp_protocol.h"      // JbUdp::LinkState
#include "odrive_protocol.h"   // ODrive::CanFrame (the neutral frame POD)
#include "can_buses.h"         // can_cone_send
#include "time_base.h"         // micros64

namespace CanBridge {

// ── Wire invariants (Electronic-Clapboard docs/protocol.md §8.5) ─────────────
// DLC 8 on every frame, always — see the header for why a 1-byte frame is the
// natural mistake and why it fails silently on BOTH sides.
static constexpr uint8_t  CLAP_DLC             = 8;
// 2 Hz. Six missed frames of margin against the clapboard's 3 s bridge-dead
// timeout. Do not raise it: this bus's analog drive path is known-degraded.
static constexpr uint32_t CLAP_LINK_PERIOD_US  = 500000u;   // 2 Hz

// Cadence state. `s_started` rather than a `!= 0` test on the timestamp so the
// first call emits immediately even when micros64() legitimately reads 0 (which
// it does under the native harness's controllable clock right after fake_reset).
static bool     s_started         = false;
static uint64_t s_last_emit_us    = 0;

// ── clap_tx ring (contract + rationale in clap_link.h) ───────────────────────
// Ticks per drained frame. 1 = one frame per 100 Hz tick = 410 ms for the
// worst-case 41-frame transaction. THE PRE-REGISTERED FALLBACK IS TO SET THIS
// TO 2 (owner decision 2026-08-16): if the Phase 5 soak shows any rise in
// cone-bus bit0_cnt/bit1_cnt/ack_cnt during slate pushes over the idle baseline,
// halve the drain rate — 820 ms per transaction, still far inside the ~8 s action
// budget — rather than investigating. That is the whole reason this is a named
// constant and not a literal 1.
static constexpr uint8_t CLAP_TX_DRAIN_TICK_DIVISOR = 1;

// Ring capacity. Must hold a whole worst-case burst, since enqueue is
// all-or-nothing; 64 leaves headroom above CLAP_MAX_FRAMES without being large
// (13 B/slot ≈ 832 B of .bss).
static constexpr uint8_t CLAP_TX_RING_CAP = 64;
static_assert(CLAP_TX_RING_CAP >= JbUdp::CLAP_MAX_FRAMES,
              "clap_tx ring must hold one whole CLAP_SEND burst — enqueue is all-or-nothing");

struct ClapTxRec {
  uint32_t id;
  uint8_t  len;
  uint8_t  buf[8];
};
static ClapTxRec s_tx_ring[CLAP_TX_RING_CAP];
// SPSC across two FreeRTOS tasks of EQUAL priority (producer: rpc.cpp on
// task_net; consumer: clap_link_step on task_time_sync), so either can preempt
// the other. head/tail/count are therefore read-modify-written inside a PRIMASK
// critical section — the same idiom the cone/cmd_result/platform relay rings use
// — which sidesteps every memory-ordering subtlety for a few hundred ns per
// ~13-byte record. No ISR is involved on either side.
static volatile uint8_t s_tx_head = 0, s_tx_tail = 0, s_tx_count = 0;
static uint8_t s_drain_phase = 0;   // consumer-only

// Counters: ONE writer each, so a plain non-atomic ++ is safe outside the mask.
// queued/dropped/ring_hwm are producer-only (task_net); sent/gated are
// consumer-only (task_time_sync). Each is a single word, read whole by the 1 Hz
// CLAP_DIAG emitter, which differences nothing and tolerates a one-frame skew.
static uint32_t s_queued = 0, s_sent = 0, s_gated = 0, s_dropped = 0;
static uint8_t  s_ring_hwm = 0;

uint8_t clap_link_state_byte(uint8_t link_state) {
  return (link_state == JbUdp::LinkState::UP) ? 1u : 0u;
}

void clap_link_reset() {
  s_started = false;
  s_last_emit_us = 0;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  s_tx_head = s_tx_tail = s_tx_count = 0;
  __set_PRIMASK(pm);
  s_drain_phase = 0;
  s_queued = s_sent = s_gated = s_dropped = 0;
  s_ring_hwm = 0;
}

bool clap_tx_enqueue_burst(const uint32_t* ids, const uint8_t* lens,
                           const uint8_t* data8, uint8_t count) {
  if (count == 0 || count > JbUdp::CLAP_MAX_FRAMES) return false;
  if (!ids || !lens || !data8) return false;
  // DLC bound. This is FRAMING/memory safety, not protocol knowledge: len is
  // copied into CAN_message_t::len, and a value above 8 is not a legal classic-CAN
  // frame. The clapboard's stricter "must be exactly 8" rule is deliberately NOT
  // enforced here — that is field-model knowledge, and putting it in the bridge
  // would make a clapboard protocol change require a Teensy reflash, which is the
  // one thing the byte-relay design exists to avoid.
  for (uint8_t i = 0; i < count; ++i) {
    if (lens[i] > 8) return false;
  }

  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  // ALL-OR-NOTHING: a partial transaction reaches the clapboard as a fragment and
  // comes back as CRC_MISMATCH/INCOMPLETE, which reads as a panel fault. A refusal
  // with nothing on the wire is unambiguous.
  if ((uint16_t)s_tx_count + count > CLAP_TX_RING_CAP) {
    __set_PRIMASK(pm);
    s_dropped += count;
    return false;
  }
  for (uint8_t i = 0; i < count; ++i) {
    ClapTxRec& r = s_tx_ring[s_tx_head];
    r.id  = ids[i];
    r.len = lens[i];
    memcpy(r.buf, data8 + (size_t)i * 8u, 8);
    s_tx_head = (uint8_t)((s_tx_head + 1) % CLAP_TX_RING_CAP);
    ++s_tx_count;
  }
  const uint8_t occupancy = s_tx_count;
  __set_PRIMASK(pm);

  s_queued += count;
  if (occupancy > s_ring_hwm) s_ring_hwm = occupancy;
  return true;
}

ClapTxStats clap_tx_stats() {
  ClapTxStats s{};
  s.queued   = s_queued;
  s.sent     = s_sent;
  s.gated    = s_gated;
  s.dropped  = s_dropped;
  s.ring_hwm = s_ring_hwm;
  return s;
}

// Drain at most ONE frame. Called from clap_link_step at 100 Hz.
static void clap_tx_drain() {
  if (++s_drain_phase < CLAP_TX_DRAIN_TICK_DIVISOR) return;
  s_drain_phase = 0;

  ClapTxRec r;
  bool have = false;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  if (s_tx_count != 0) {
    r = s_tx_ring[s_tx_tail];
    s_tx_tail = (uint8_t)((s_tx_tail + 1) % CLAP_TX_RING_CAP);
    --s_tx_count;
    have = true;
  }
  __set_PRIMASK(pm);
  if (!have) return;

  // Gate CHECKED, never bypassed, and the frame is DISCARDED rather than held when
  // it is closed: holding a half-drained transaction would dribble it out later
  // interleaved with a newer one, and the clapboard's CRC would then reject BOTH.
  // The loss is visible — as `gated` here, and as a CRC_MISMATCH/INCOMPLETE ack
  // from the panel, which is exactly what that CRC exists to detect.
  if (!cone_partner_present()) { ++s_gated; return; }

  ODrive::CanFrame f;
  f.id  = r.id;
  f.len = r.len;
  memcpy(f.buf, r.buf, 8);
  // A false return HERE, with the gate open a moment ago, is a write() DEFERRAL —
  // the frame is in the 64-slot software txBuffer and the TX-complete ISR sends it
  // ~0.1-1 ms later — so it is NOT a loss and is counted as sent. Genuine TX-path
  // pressure is already reported per-bus by BridgeTxDiag (tx_deferred/tx_q_hwm).
  // can_cone_send re-checks the gate itself, so a stale pre-check cannot bypass it;
  // the residual is a frame counted `sent` that the gate refused in the ~us between
  // the two reads, which BusRxHealth::tx_gated still records.
  can_cone_send(f);
  ++s_sent;
}

void clap_link_step(uint8_t link_state) {
  // Drain FIRST so the beacon's 2 Hz cadence check can early-return without
  // stalling the downlink: the two are independent producers on one bus and must
  // not be able to starve each other.
  clap_tx_drain();

  const uint64_t now = micros64();   // interval clock: emission pacing
  if (s_started && (now - s_last_emit_us) < CLAP_LINK_PERIOD_US) return;
  s_started = true;
  s_last_emit_us = now;

  ODrive::CanFrame f;
  f.id  = ClapboardCanId::LINK;      // 0x7EA
  f.len = CLAP_DLC;                  // NEVER 1 — see clap_link.h
  memset(f.buf, 0, sizeof(f.buf));   // bytes 1-7 "reserved, must be 0" (§8.5)
  f.buf[0] = clap_link_state_byte(link_state);

  // UNGATED on clapboard presence, deliberately. Gating the beacon on the RX-side
  // role discriminator would add a second way for the panel to sit in screensaver
  // forever (a wrong discriminator), which is the named first-integration-bug
  // class this whole file is written around; and the cost of not gating is 2 extra
  // frames/s beside the 100 Hz 0x7DD this bus already carries. The presence gate
  // that DOES matter — bus-partner, i.e. "is anyone here to ACK" — is inside
  // can_cone_send() and applies unchanged.
  can_cone_send(f);   // result discarded — see clap_link.h
}

}  // namespace CanBridge
