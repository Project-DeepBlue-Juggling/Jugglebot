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

// Emit the 2 Hz CLAP_LINK beacon. Call every tick from task_time_sync (100 Hz);
// the cadence is enforced here, not by the caller. `link_state` is a
// JbUdp::LinkState value — pass `link_state()` straight through.
//
// The send is presence-gated inside can_cone_send() and its result is DISCARDED,
// deliberately, exactly as broadcast_0x7dd() discards its three: one bus being
// absent must never block anything else, and there is nothing useful to do with
// the refusal here (the clapboard's own 3 s staleness rule is the recovery path).
void clap_link_step(uint8_t link_state);

// Byte 0 of the CLAP_LINK payload for a given LinkState: 1 = ROS2 UP, 0 = DOWN.
// Exposed for the native test — and because the mapping is the interesting part:
// only LinkState::UP is a healthy bidirectional link, so INIT (Ethernet up, no
// Jetson heartbeat yet), DEGRADED and LOST all report DOWN. Screensaver is the
// safe default, so an ambiguous state must never render as UP.
uint8_t clap_link_state_byte(uint8_t link_state);

// Zero the emitter cadence. The firmware never needs this — the file statics are
// zero at boot — but the native driver calls it between TEST_CASEs so file-scope
// state cannot leak across cases (the fake_reset discipline).
void clap_link_reset();

}  // namespace CanBridge
