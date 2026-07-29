// =============================================================================
//  gpio_poll.cpp — hand ball-present sensor poll (hand ODrive G02, CAN3 SDO)
// =============================================================================
//  See gpio_poll.h for the design + the NORMATIVE signal semantics. This TU owns
//  the two-phase request/await state machine, the debounce, and the published
//  cache.
//
//  BUILD-TIME KILL SWITCH MECHANISM: JBBallDetect::ENABLED is a constexpr bool,
//  NOT a preprocessor macro, so the compile-out is a plain early return on
//  !ENABLED that the optimiser eliminates. `#if JB_BD_ENABLED` would evaluate an
//  UNDEFINED identifier to 0 and silently compile the poller out even when the
//  YAML says enabled (logbook/2026-07-29-hand-sensor-ball-detect-config.md).
// =============================================================================
#include "gpio_poll.h"

#include <Arduino.h>           // Serial (the diag park line + the console status line)
#include <cstring>             // strncmp / strcmp (console command parsing)

#include "canbridge_config.h"  // HAND_AXIS, JBBallDetect::*
#include "can_buses.h"         // can_jugglebot_send, jugglebot_commands_allowed
#include "odrive_protocol.h"   // encode_sdo_write, decode_get_version
#include "protocol_config.h"   // EndpointId::odrive_pro_0_6_11::get_gpio_states
#include "time_base.h"         // micros64 (intervals), now_wall_us (wire stamp), time_synced
#include "version_check.h"     // version_raw_copy (the Get_Version gate's input)

namespace CanBridge {

// A pin outside the 32-bit GPIO word can never answer, and a debounce depth the
// saturating uint8 counter can never reach would leave HELD latched forever —
// both are YAML edits away, so they fail the build instead.
static_assert(JBBallDetect::GPIO_PIN < 32u, "JB_BD_GPIO_PIN outside the get_gpio_states word");
static_assert(JBBallDetect::MAX_MISSING_SAMPLES > 0u && JBBallDetect::MAX_MISSING_SAMPLES <= 255u,
              "JB_BD_MAX_MISSING_SAMPLES must fit the saturating uint8 miss counter");

static constexpr uint64_t POLL_INTERVAL_US = (uint64_t)JBBallDetect::CHECK_INTERVAL_MS * 1000ull;
static constexpr uint64_t REPLY_TIMEOUT_US = (uint64_t)JBBallDetect::CHECK_TIMEOUT_MS  * 1000ull;

// Staleness window. A healthy poller lands a good reply at least every
// (CHECK_INTERVAL_MS + CHECK_TIMEOUT_MS); two of those tolerate ONE lost round
// trip without flapping the tri-state to UNKNOWN, and still surface a dead sensor
// within ~240 ms. PROVISIONAL — Phase 7 step 3's SDO round-trip measurement is
// what actually sizes it (no measured figure exists anywhere today).
static constexpr uint64_t REPLY_STALE_US = 2ull * (POLL_INTERVAL_US + REPLY_TIMEOUT_US);

enum class PPhase : uint8_t { IDLE, AWAIT };
enum class Gate   : uint8_t { UNRECEIVED, MATCH, MISMATCH };

// ── Writer-private state (task_homing only) ──────────────────────────────────
static PPhase   s_phase        = PPhase::IDLE;
static uint64_t s_attempt_us   = 0;       // monotonic stamp of the last request attempt (pacing + timeout)
static uint8_t  s_miss_count   = 0;       // consecutive EMPTY good readings (saturating)
static bool     s_held         = false;   // debounced verdict
static uint64_t s_last_good_mono_us = 0;  // writer's mirror of the published mono stamp; 0 ⇒ none
// Runtime toggle + park latch: written by the console (task_diag), read by the
// step (task_homing). Single-byte volatile accesses are atomic on Cortex-M7.
static volatile bool s_enabled  = true;
static volatile bool s_mismatch = false;
// The fw triple actually seen when the gate latched (+ fw_unreleased), stashed by
// the step so task_diag can print the park line without re-decoding the cache.
// Published BEFORE s_mismatch (version_record's barrier idiom), so a reader that
// sees the latch always sees the matching bytes.
static volatile uint8_t s_seen_fw[4] = {0, 0, 0, 0};

// ── Reply mailbox (SPSC): written by the CAN3 RX decode, drained by the step ──
//  Carries the ARRIVAL stamps with the word (see gpio_poll.h): the step may run
//  up to a task tick later, and t_bridge_us is contractually the reply instant.
static uint32_t      s_reply_states  = 0;
static uint64_t      s_reply_wall_us = 0;
static uint64_t      s_reply_mono_us = 0;
static volatile bool s_reply_pending = false;

// ── Published record: whole-record copy under PRIMASK on BOTH sides ──────────
struct Published {
  uint32_t raw_states  = 0;
  uint64_t wall_us     = 0;   // now_wall_us() at the last good reply (wire field)
  uint64_t mono_us     = 0;   // micros64() at the same reply; 0 ⇒ no good reply yet
  uint8_t  miss_count  = 0;
  bool     raw_held    = false;
  bool     held        = false;
  bool     time_synced = false;
};
static Published s_pub;

static inline void publish(const Published& p) {
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  s_pub = p;
  __set_PRIMASK(pm);
}

// Drop any cached reply. Called BEFORE every send: without it the await phase
// re-reads the PREVIOUS reply as fresh and the timeout branch is unreachable
// (BallButler CanInterface.cpp:792-793 does the same invalidate). A plain
// single-byte volatile STORE — atomic on Cortex-M7, no read-modify-write, so
// PRIMASK would buy nothing (version_check.cpp's mask idiom); the barrier keeps
// it from sinking past the send it precedes.
static inline void reply_invalidate() {
  s_reply_pending = false;
  asm volatile("" ::: "memory");
}

static inline bool reply_take(uint32_t& out, uint64_t& wall_us, uint64_t& mono_us) {
  bool have = false;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  if (s_reply_pending) {
    out = s_reply_states; wall_us = s_reply_wall_us; mono_us = s_reply_mono_us;
    s_reply_pending = false; have = true;
  }
  __set_PRIMASK(pm);
  return have;
}

// Three-state Get_Version gate (plan Phase 2 item 4). UNRECEIVED is NORMAL for
// seconds after boot and permanent for an unpowered hand axis — quiet, no fault.
// The compare is the ONE version semantic the firmware owns (version_check.h's
// split note): the wrong-endpoint failure mode ANSWERS PLAUSIBLY, so the refusal
// has to happen before the RxSdo leaves the Teensy.
// `out` receives the decoded reply on MATCH/MISMATCH (untouched on UNRECEIVED)
// so the park latch reports the triple the gate actually judged, rather than
// re-reading a cache that could have moved between the two decodes.
static Gate version_gate(ODrive::Version& out) {
  uint8_t raw[8];
  if (!version_raw_copy(HAND_AXIS, raw)) return Gate::UNRECEIVED;
  out = ODrive::decode_get_version(raw);
  const bool match = (out.fw_major == JBBallDetect::EXPECTED_FW[0])
                  && (out.fw_minor == JBBallDetect::EXPECTED_FW[1])
                  && (out.fw_rev   == JBBallDetect::EXPECTED_FW[2]);
  return match ? Gate::MATCH : Gate::MISMATCH;
}

// Latch the park and stash what was seen. NO Serial here: this runs on
// task_homing, whose STACK_HOMING is 1 KB, and a multi-arg printf is exactly
// what overflowed task_diag's original 1 KB stack (canbridge_config.h:135). The
// loud part is gpio_poll_diag_step(), on task_diag's 8 KB stack.
static void park_latch(const ODrive::Version& v) {
  if (s_mismatch) return;
  s_seen_fw[0] = v.fw_major;
  s_seen_fw[1] = v.fw_minor;
  s_seen_fw[2] = v.fw_rev;
  s_seen_fw[3] = v.fw_unreleased;
  asm volatile("" ::: "memory");   // publish the triple before the latch
  s_mismatch = true;
}

// A good reply: decode, debounce, publish. The ONLY place the verdict and the
// miss count move (a timeout never reaches here — that is the tri-state rule).
static void apply_good_reply(uint32_t states, uint64_t reply_wall_us, uint64_t reply_mono_us) {
  // Active-low: the switch shorts G02 to GND against the ODrive's pull-up.
  const bool raw_held = ((states >> JBBallDetect::GPIO_PIN) & 1u) == 0u;
  // Did the poller still hold a VALID sample when this reply landed? Boot (no
  // good reply yet) and any not-valid gap — a stale window, a runtime-off
  // window — end the debounce history: a pre-gap HELD must not outlive a
  // multi-second UNKNOWN and keep re-asserting itself for MAX_MISSING_SAMPLES
  // replies after recovery, when nothing observed the hand in between. So the
  // first reply after a gap SEEDS from the raw bit (the same first-reading-is-
  // the-truth deviation the boot case already takes). A MISMATCH park is NOT in
  // this set — that latch never un-latches, so no reply ever follows it.
  const bool prev_valid = (s_last_good_mono_us != 0)
                       && ((reply_mono_us - s_last_good_mono_us) <= REPLY_STALE_US);
  if (!prev_valid) {
    s_held       = raw_held;
    s_miss_count = raw_held ? 0u : 1u;
  } else if (raw_held) {
    s_miss_count = 0;
    s_held = true;                 // any single HELD reading restores HELD
  } else {
    if (s_miss_count < 255u) ++s_miss_count;
    // The MAX_MISSING_SAMPLES rule guards the HELD→EMPTY transition only;
    // UNKNOWN→EMPTY is the fail-safe direction for a ball_seated gate and is
    // taken immediately by the seed branch above.
    if (s_miss_count >= JBBallDetect::MAX_MISSING_SAMPLES) s_held = false;
  }
  s_last_good_mono_us = reply_mono_us;
  Published p;
  p.raw_states  = states;
  p.wall_us     = reply_wall_us;   // wire-bound absolute timestamp — wall by contract
  p.mono_us     = reply_mono_us;
  p.miss_count  = s_miss_count;
  p.raw_held    = raw_held;
  p.held        = s_held;
  p.time_synced = time_synced();
  publish(p);
}

void gpio_poll_init() {
  s_phase             = PPhase::IDLE;
  s_attempt_us        = 0;
  s_miss_count        = 0;
  s_held              = false;
  s_last_good_mono_us = 0;
  s_enabled           = true;   // boots ON — the Phase 7 A/B arm is the runtime toggle
  s_mismatch          = false;
  for (uint8_t i = 0; i < 4; ++i) s_seen_fw[i] = 0;
  s_reply_states      = 0;
  s_reply_wall_us     = 0;
  s_reply_mono_us     = 0;
  reply_invalidate();
  publish(Published{});
}

void gpio_poll_record(uint32_t raw_states, uint64_t wall_us, uint64_t mono_us) {
  if (!JBBallDetect::ENABLED) return;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  s_reply_states  = raw_states;
  s_reply_wall_us = wall_us;
  s_reply_mono_us = mono_us;
  s_reply_pending = true;
  __set_PRIMASK(pm);
}

void gpio_poll_step() {
  if (!JBBallDetect::ENABLED) return;   // build-time kill switch (constexpr, never #if)
  if (!s_enabled) {
    // Runtime toggle OFF. DRAIN, don't just return: a reply latched in AWAIT
    // when the console toggled off would otherwise be taken as fresh on
    // re-enable and published with its (now arbitrarily old) arrival stamps.
    s_phase = PPhase::IDLE;
    reply_invalidate();
    return;
  }

  const uint64_t now = micros64();      // interval clock: pacing, timeout, staleness

  if (s_phase == PPhase::AWAIT) {
    uint32_t states = 0;
    uint64_t reply_wall_us = 0, reply_mono_us = 0;
    if (reply_take(states, reply_wall_us, reply_mono_us)) {
      apply_good_reply(states, reply_wall_us, reply_mono_us);
      s_phase = PPhase::IDLE;
    } else if (now - s_attempt_us >= REPLY_TIMEOUT_US) {
      s_phase = PPhase::IDLE;           // timeout ⇒ staleness only: verdict + miss_count untouched
    }
    return;                             // at most one CAN3 TX per tick (bus pacing)
  }

  if (now - s_attempt_us < POLL_INTERVAL_US) return;
  s_attempt_us = now;   // paces EVERY outcome below, refusals included, off the 100 Hz tick

  ODrive::Version v{};
  const Gate g = version_gate(v);
  if (g == Gate::UNRECEIVED) return;              // quiet, zero RxSdo, retry forever, no fault
  if (g == Gate::MISMATCH) { park_latch(v); return; }

  // Never TX into a dead bus: the health gate here, the CAN3 partner-presence
  // gate inside can_jugglebot_send() (an un-ACKed TX climbs the FlexCAN TEC).
  // can_jugglebot_send is version_check_step's TX path — the poller is
  // firmware-internal and never touches send_axis_frame's allow-table.
  if (!jugglebot_commands_allowed()) return;
  reply_invalidate();
  // OPCODE_WRITE with a zero payload is the ODrive FUNCTION-INVOKE idiom, not a
  // parameter write: get_gpio_states is a function endpoint whose return value
  // arrives as the TxSdo reply (BallButler requestArbitraryParameter's frame,
  // byte-for-byte). encode_sdo_read would send OPCODE_READ, which does not
  // invoke it.
  if (can_jugglebot_send(ODrive::encode_sdo_write(
          HAND_AXIS, EndpointId::odrive_pro_0_6_11::get_gpio_states, 0.0f)))
    s_phase = PPhase::AWAIT;
}

void gpio_poll_snapshot(GpioPollSnapshot& out) {
  Published p;
  const uint32_t pm = __get_PRIMASK(); __disable_irq();
  p = s_pub;
  __set_PRIMASK(pm);
  const bool have  = (p.mono_us != 0);            // 0 ⇒ no good reply this boot
  const bool stale = !have || (micros64() - p.mono_us) > REPLY_STALE_US;
  out.raw_states       = p.raw_states;
  out.t_bridge_us      = p.wall_us;
  out.miss_count       = p.miss_count;
  out.raw_held         = p.raw_held;
  out.held             = p.held;
  out.stale            = stale;
  out.valid            = have && !stale && !s_mismatch;
  out.time_synced      = p.time_synced;
}

void gpio_poll_diag_step() {
  if (!JBBallDetect::ENABLED) return;
  if (!s_mismatch) return;
  // Once per diag tick for as long as the park holds — a one-shot line scrolls
  // off the console long before an operator attaches a monitor, and a silently
  // parked poller looks exactly like a poller that is simply never seeing HELD.
  Serial.printf("[gpio_poll] VERSION MISMATCH seen %u.%u.%u-%u expected %u.%u.%u"
                " — poller parked\n",
                (unsigned)s_seen_fw[0], (unsigned)s_seen_fw[1],
                (unsigned)s_seen_fw[2], (unsigned)s_seen_fw[3],
                (unsigned)JBBallDetect::EXPECTED_FW[0],
                (unsigned)JBBallDetect::EXPECTED_FW[1],
                (unsigned)JBBallDetect::EXPECTED_FW[2]);
}

bool gpio_poll_console(const char* line) {
  if (line == nullptr || strncmp(line, "gpio_poll", 9) != 0) return false;
  if (!JBBallDetect::ENABLED) {
    // Own the command even when compiled out: "unknown command" on a
    // kill-switched build reads as a typo and sends the operator hunting for
    // the right spelling instead of at the YAML.
    Serial.println("[gpio_poll] compiled out"
                   " (jugglebot_ball_detect.enabled=false) — reflash to enable");
    return true;
  }
  const char* arg = line + 9;
  while (*arg == ' ') ++arg;
  if      (strcmp(arg, "on")  == 0) s_enabled = true;
  else if (strcmp(arg, "off") == 0) s_enabled = false;
  else if (*arg != '\0')            return false;   // not this command after all
  GpioPollSnapshot s;
  gpio_poll_snapshot(s);
  // The rendered tri-state also requires time_synced: an un-anchored bridge
  // clock is one of the plan's UNKNOWN cases (§ Architecture), because the
  // sample's t_bridge_us cannot be trusted against the Jetson epoch. The wire
  // flags stay ORTHOGONAL — `valid` does not fold in time_synced — so the
  // uplink can still distinguish "no reply" from "reply, unanchored clock".
  Serial.printf("[gpio_poll] enabled=%u state=%s raw=0x%08lx raw_held=%u miss=%u"
                " stale=%u synced=%u mismatch=%u\n",
                (unsigned)s_enabled,
                (s.valid && s.time_synced) ? (s.held ? "HELD" : "EMPTY") : "UNKNOWN",
                (unsigned long)s.raw_states, (unsigned)s.raw_held,
                (unsigned)s.miss_count, (unsigned)s.stale,
                (unsigned)s.time_synced, (unsigned)s_mismatch);
  return true;
}

}  // namespace CanBridge
