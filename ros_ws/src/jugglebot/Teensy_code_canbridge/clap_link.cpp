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

uint8_t clap_link_state_byte(uint8_t link_state) {
  return (link_state == JbUdp::LinkState::UP) ? 1u : 0u;
}

void clap_link_reset() {
  s_started = false;
  s_last_emit_us = 0;
}

void clap_link_step(uint8_t link_state) {
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
