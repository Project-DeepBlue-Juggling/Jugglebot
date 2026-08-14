#pragma once
// =============================================================================
//  gpio_poll.h — hand ball-present sensor poll (hand ODrive G02, CAN3 SDO)
// =============================================================================
//  A switch in Jugglebot's hand shorts the hand ODrive Pro's G02 to GND when a
//  ball is seated (gpio2_mode = DIGITAL_PULL_UP, flashed + NVM-persisted
//  2026-07-28). No ODrive firmware on any released 0.6.x pushes GPIO state on
//  CANSimple, so the bridge POLLS: an RxSdo function-invoke of get_gpio_states
//  every JBBallDetect::CHECK_INTERVAL_MS whose TxSdo reply carries the GPIO
//  bitmask as a uint32. plans/active/hand-ball-sensor.md § Architecture is
//  NORMATIVE for the signal semantics (tri-state, timeout-is-not-a-miss, the
//  miss_count freeze, the debounce asymmetry) — read them there, not here.
//
//  Concurrency: gpio_poll_step() is the ONLY writer of the published record and
//  runs on task_homing. gpio_poll_record() runs in the CAN3 RX decode context and
//  only fills a single-slot reply mailbox that the step consumes (SPSC, the
//  can_hand_cmd_echo_pop pattern). Readers take a PRIMASK-guarded whole-record
//  copy — deliberately NOT a seqlock: the uplink reader runs on task_telem
//  (PRIO_UDP_TX = 3), which OUTRANKS the writer on task_homing (PRIO_HOMING = 2),
//  so a reader that preempted a mid-write seqlock would spin its retry loop
//  against a writer that can never be scheduled. axis_state.h's seqlock is safe
//  only because its writer outranks its task readers.
//
//  Two INDEPENDENT off-switches: JBBallDetect::ENABLED == false is the
//  build-time kill switch (constexpr-gated early returns — never `#if`, see
//  gpio_poll.cpp), and `gpio_poll on|off` on the serial console is the runtime
//  one (boots ON; it is Phase 7's A/B arm, so no reflash between arms).
// =============================================================================

#include <cstdint>

namespace CanBridge {

// Published ball-sensor state. Read via gpio_poll_snapshot(); `stale` / `valid`
// are computed at read time against micros64(), the rest are the last good reply.
struct GpioPollSnapshot {
  uint32_t raw_states       = 0;      // last raw get_gpio_states word, verbatim
  uint64_t t_bridge_us      = 0;      // now_wall_us() at that reply — WALL by contract (wire field)
  uint8_t  miss_count       = 0;      // consecutive EMPTY good readings (saturating, frozen while stale)
  bool     raw_held         = false;  // undebounced bit from that reply
  bool     held             = false;  // debounced verdict
  bool     valid            = false;  // false ⇒ UNKNOWN (no reply yet / stale / version gate not passed)
  bool     stale            = false;  // a good reply is overdue
  bool     time_synced      = false;  // bridge wall anchor was set at that reply
};

// Init / re-arm: clears the state machine, the cache and the mismatch latch, and
// arms the runtime toggle ON. Called once from setup() (version_check_init pattern).
void gpio_poll_init();

// Cold-start monitor tick (task_homing, HOMING_RATE_HZ, never an ISR). Runs the
// two-phase request/await state machine: at most ONE CAN3 TX per tick, internally
// rate-limited to JBBallDetect::CHECK_INTERVAL_MS. No-op when compiled out or
// toggled off. Sole writer of the published record.
void gpio_poll_step();

// CAN3 RX decode seam: a TxSdo reply from the hand carrying the get_gpio_states
// word, STAMPED AT ARRIVAL (`wall_us` = now_wall_us(), `mono_us` = micros64() in
// the RX decode context). The stamps travel with the word through the mailbox
// because the step consumes it up to one task tick later: stamping at
// consumption would bias t_bridge_us — defined as "at the last good TxSdo
// reply" — late by that whole tick, and inflate every staleness age with it.
// Fills the single-slot reply mailbox; all decoding happens in the step.
void gpio_poll_record(uint32_t raw_states, uint64_t wall_us, uint64_t mono_us);

// Consistent copy of the published state (task_telem uplink, console).
void gpio_poll_snapshot(GpioPollSnapshot& out);

// ── SDO round-trip census (2026-08-14, can-bridge FW 13 RING_DIAG) ──────────
// The CAUSAL cross-check for the FlexCAN_T4 RX-ring `_available` leak. This
// poller already round-trips the jugglebot bus at 50 Hz, and its reply comes
// back through the same RX ring as everything else — so if the ring is holding
// D frames, this round trip's FLOOR grows by exactly D's worth of delay.
// Aged-minus-fresh on the floor is the ring delay measured end-to-end, through a
// path whose own floor (the ODrive's SDO service time) is physically fixed.
// Nothing new goes on the wire: this is two timestamps around traffic that was
// already there.
//
// WHY A FLOOR AND NOT A MEAN. The request is stamped on task_homing (PRIO_HOMING
// = 2, the lowest real-work task), so a preemption between the stamp and the
// wire can only INFLATE a sample. Inflation cannot move a minimum drawn from ~50
// round trips; it would corrupt a mean. The stamp is taken BEFORE the send for
// the same reason — stamping after would DEFLATE samples, putting the corruption
// exactly where it does damage.
struct GpioPollRtt {
  uint32_t min_us;   // window minimum — THE measurement (see above)
  uint32_t max_us;   // window maximum — the tail; catches the ~100-150 ms freezes
  uint32_t last_us;  // most recent sample — the unreduced spot reading
  uint16_t count;    // round trips COMPLETED this window. READ THIS FIRST: 0 ⇒ the
                     // other three are 0 because nothing was measured, not because
                     // the path was instant
};

// Take and CLEAR the window census (task_telem, 1 Hz). Reader-side PRIMASK: the
// folding happens in the CAN RX decode context, which outranks the caller.
void gpio_poll_rtt_take(GpioPollRtt& out);

// 1 Hz diagnostics tick (task_diag). While the version gate is parked on a
// MISMATCH it prints ONE park line per tick — repeating is what makes the park
// actually loud, and task_diag is the only task whose stack (STACK_DIAG, 8 KB)
// is sized for Serial formatting (canbridge_config.h:135's printf-overflow
// history is why task_homing must never print). No-op otherwise.
void gpio_poll_diag_step();

// Serial-console command seam. Handles "gpio_poll", "gpio_poll on", "gpio_poll
// off" (printing a status line); returns false for anything else so the caller
// can report an unknown command. A build with ENABLED == false still OWNS the
// prefix and answers "compiled out" — "unknown command" there reads as a typo
// and sends the operator hunting for a spelling instead of at the YAML.
bool gpio_poll_console(const char* line);

}  // namespace CanBridge
