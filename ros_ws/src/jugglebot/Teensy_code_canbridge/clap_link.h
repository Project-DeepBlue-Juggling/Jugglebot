#pragma once
// =============================================================================
//  clap_link.h — the electronic clapboard's 2 Hz CLAP_LINK (0x7EA) beacon
// =============================================================================
//  The clapboard shares the CONE-role bus (physical CAN3) with the catching cone
//  — one peripheral on the segment, ever, chosen by which connector is plugged
//  in. Normative wire contract: Electronic-Clapboard `docs/protocol.md` §8, which
//  NEITHER repo may change unilaterally.
//
//  WHAT THIS BEACON IS FOR. The clapboard shows the scene slate while ROS2 is up
//  and a screensaver otherwise, because its panel is meant to be readable as a
//  health indicator. `0x7DD` absence already implies "ROS2 is gone", but only
//  TIME_ANCHOR_STALE_US (90 s) after the fact — `broadcast_0x7dd()` opens with
//  `if (!time_synced()) return;`. CLAP_LINK closes that latency gap: the bridge
//  already knows within JETSON_LINK_TIMEOUT_US. The clapboard keeps the 0x7DD
//  staleness backstop as well, and neither may be deleted in favour of the other:
//  push covers the case where the bridge is alive to speak, staleness covers the
//  case where it is not (dead Teensy, pulled cable). A push-only design would
//  leave the last scene frame on the panel through the SEVEREST failure, which is
//  exactly backwards.
//
//  TWO RULES THAT ARE EASY TO BREAK AND SILENT WHEN BROKEN:
//
//  1. EVERY frame is DLC 8. Byte 0 is the only meaningful byte, so the natural
//     implementation is a 1-byte frame — and the clapboard SILENTLY DROPS any
//     other length (`can_frames.h:226` in that repo). The panel would sit in
//     screensaver forever with no error on either side. `protocol.md` documents
//     the layout but never states this rule; only the peer's code enforces it.
//     The same applies to 0x7DD / 0x7E8 / 0x7E9.
//  2. 2 Hz, and NOT faster. The clapboard treats a 3 s gap as bridge-dead, so
//     2 Hz gives six missed frames of margin while keeping a bus whose analog
//     drive path is known-degraded quiet (logbook 2026-07-31). The handoff brief
//     says "Do not raise it."
//
//  WHY link_state IS A PARAMETER. `link_state()` is `static` inside
//  Teensy_code_canbridge.ino, which no test ever compiles. Passing it in keeps
//  this TU pure and natively testable, needs no .ino refactor, and — the real
//  reason — avoids minting a THIRD copy of the link predicate: the two that
//  already exist (`link_state()` and fault_machine.cpp's `jetson_link_up()`)
//  disagree in one cell, since one keys on the Jetson HEARTBEAT and the other on
//  any UDP frame. A third copy would be a third opinion.
// =============================================================================

#include <cstdint>

namespace CanBridge {

// Emit the 2 Hz CLAP_LINK beacon AND drain one queued downlink frame (see the
// clap_tx section below). Call every tick from task_time_sync (100 Hz); both
// cadences are enforced here, not by the caller. `link_state` is a
// JbUdp::LinkState value — pass `link_state()` straight through.
//
// The beacon send is presence-gated inside can_cone_send() and its result is
// DISCARDED, deliberately, exactly as broadcast_0x7dd() discards its three: one
// bus being absent must never block anything else, and there is nothing useful to
// do with the refusal here (the clapboard's own 3 s staleness rule is the
// recovery path).
void clap_link_step(uint8_t link_state);

// Byte 0 of the CLAP_LINK payload for a given LinkState: 1 = ROS2 UP, 0 = DOWN.
// Exposed for the native test — and because the mapping is the interesting part:
// only LinkState::UP is a healthy bidirectional link, so INIT (Ethernet up, no
// Jetson heartbeat yet), DEGRADED and LOST all report DOWN. Screensaver is the
// safe default, so an ambiguous state must never render as UP.
uint8_t clap_link_state_byte(uint8_t link_state);

// ── clap_tx: the downlink burst queue (RPC CLAP_SEND → cone bus) ─────────────
//  One CLAP_SEND request carries a WHOLE slate transaction (up to
//  CLAP_MAX_FRAMES frames). It is enqueued here and drained ONE FRAME PER TICK,
//  never burst.
//
//  WHY PACED. The cone bus's analog drive path is known-degraded (load-dependent
//  fault, logbook 2026-07-31) and every existing producer on every bus already
//  sends one frame per tick by convention. 41 frames — the worst-case transaction
//  — is 410 ms at 100 Hz, far inside the ~8 s action budget the panel's own
//  1.5-3.5 s e-paper refresh dominates anyway. The divisor is a NAMED CONSTANT so
//  the owner's pre-registered fallback (if the Phase 5 soak shows any rise in
//  cone-bus bit0_cnt/bit1_cnt/ack_cnt during slate pushes over the idle baseline,
//  halve the rate) is a one-line change rather than an investigation.
//
//  WHY FIFO, ONE QUEUE. Chunks may arrive in any order, but CLAP_COMMIT must
//  arrive AFTER its chunks or the transaction is INCOMPLETE. One RPC carrying the
//  whole burst, drained FIFO, guarantees that by construction provided the host
//  puts the commit last — no ordering logic in the firmware at all.
//
//  WHY ALL-OR-NOTHING. A partially-enqueued transaction is worse than a refused
//  one: the clapboard would reassemble a fragment and answer CRC_MISMATCH or
//  INCOMPLETE, which reads as a panel fault. ERR_REJECTED with nothing on the
//  wire is unambiguous.

// Enqueue a whole burst or nothing. `ids`, `lens` and `data8` are parallel
// arrays of `count` entries (data8 is count*8 bytes, slot-major) — the
// structure-of-arrays layout of JbUdp::RpcArgs::ArgClapSend, passed as plain
// pointers so this TU stays free of RPC concerns. Returns false (and enqueues
// NOTHING) if count is 0, exceeds the ring, any len > 8, or the ring cannot hold
// the whole burst. The caller checks the TX gate; this function does not.
bool clap_tx_enqueue_burst(const uint32_t* ids, const uint8_t* lens,
                           const uint8_t* data8, uint8_t count);

// Cumulative-since-boot census of every frame the host handed over, uplinked as
// CLAP_DIAG (0x93) at 1 Hz. queued == sent + gated + still-in-ring; dropped
// counts frames that never got in. See the ClapDiag summary in
// config/generate_udp_protocol.py for why a mid-drain failure cannot be an RPC
// error and therefore needs its own frame.
//
// Each field has exactly ONE writer (queued/dropped/ring_hwm on the producer
// task, sent/gated on the consumer task) and is a single word, so no value is
// ever torn — but the five are read WITHOUT a mask, so that accounting identity
// can be off by at most one frame if a drain lands mid-read. It is a 1 Hz
// cumulative diagnostic read as a trend; masking five word loads against the
// producer would buy a consistency nobody needs.
struct ClapTxStats {
  uint32_t queued;    // frames accepted into the ring
  uint32_t sent;      // frames handed to can_cone_send with the gate open
  uint32_t gated;     // frames discarded at drain because the gate was closed
  uint32_t dropped;   // frames refused at enqueue (burst would not fit)
  uint8_t  ring_hwm;  // peak ring occupancy
};
ClapTxStats clap_tx_stats();

// Zero the emitter cadence, the clap_tx ring and its counters. The firmware never
// needs this — the file statics are zero at boot — but the native driver calls it
// between TEST_CASEs so file-scope state cannot leak across cases (the fake_reset
// discipline).
void clap_link_reset();

}  // namespace CanBridge
